/**
 * @file logger.cpp
 * @brief Non-blocking USB RAM-buffered logger.
 */

#include "logger.h"

#include <Arduino_USBHostMbed5.h>
#include <FATFileSystem.h>
#include <USB/PluggableUSBSerial.h>
#include <string.h>

#define SERIAL_HW _UART_USB_

static USBHostMSD msd;
static mbed::FATFileSystem usb_fs("usb");

USBLogger::USBLogger()
    : buffer_head(0),
      buffer_overflow(false),
      session_file_num(-1),
      logger_state(LOGGER_IDLE),
      state_started_ms(0),
      next_attempt_ms(0),
      connect_attempts(0),
      mount_attempts(0),
      retry_cycles(0),
      flush_target_length(0),
      flush_offset(0),
      active_file(nullptr),
      active_file_start_size(0),
      filesystem_mounted(false),
      terminal_disconnect_protection(false),
      terminal_tx_head(0),
      terminal_tx_tail(0),
      terminal_tx_count(0)
{
    log_buffer[0] = '\0';
    session_filepath[0] = '\0';
    pinMode(PA_15, OUTPUT);
    digitalWrite(PA_15, LOW);
}

void USBLogger::begin(unsigned long baud)
{
    SERIAL_HW.begin(baud);
}

int USBLogger::available() { return SERIAL_HW.available(); }
int USBLogger::read()
{
    return SERIAL_HW.read();
}
int USBLogger::peek() { return SERIAL_HW.peek(); }
void USBLogger::flush()
{
    if (!terminal_disconnect_protection)
        SERIAL_HW.flush();
}
USBLogger::operator bool() { return (bool)SERIAL_HW; }

void USBLogger::protect_from_terminal_disconnect()
{
    terminal_disconnect_protection = true;
}

void USBLogger::allow_blocking_terminal_output()
{
    terminal_disconnect_protection = false;
    terminal_tx_head = 0;
    terminal_tx_tail = 0;
    terminal_tx_count = 0;
}

void USBLogger::buffer_terminal(const uint8_t *data, size_t size)
{
    for (size_t i = 0; i < size; ++i) {
        if (terminal_tx_count >= TERMINAL_TX_BUFFER_SIZE)
            return;
        terminal_tx_buffer[terminal_tx_head] = data[i];
        terminal_tx_head = (terminal_tx_head + 1) % TERMINAL_TX_BUFFER_SIZE;
        ++terminal_tx_count;
    }
}

void USBLogger::update_terminal()
{
    if (!terminal_disconnect_protection || terminal_tx_count == 0 ||
        !_SerialUSB.connected())
        return;

    size_t chunk = TERMINAL_TX_BUFFER_SIZE - terminal_tx_tail;
    if (chunk > terminal_tx_count)
        chunk = terminal_tx_count;

    uint32_t sent = 0;
    _SerialUSB.send_nb(&terminal_tx_buffer[terminal_tx_tail], chunk, &sent);
    terminal_tx_tail = (terminal_tx_tail + sent) % TERMINAL_TX_BUFFER_SIZE;
    terminal_tx_count -= sent;
}

size_t USBLogger::write(uint8_t c)
{
    buffer_char((char)c);
    if (terminal_disconnect_protection) {
        buffer_terminal(&c, 1);
    } else if (SERIAL_HW) {
        SERIAL_HW.write(c);
    }
    return 1;
}

size_t USBLogger::write(const uint8_t *buffer, size_t size)
{
    for (size_t i = 0; i < size; ++i)
        buffer_char((char)buffer[i]);
    if (terminal_disconnect_protection) {
        buffer_terminal(buffer, size);
    } else if (SERIAL_HW) {
        SERIAL_HW.write(buffer, size);
    }
    return size;
}

void USBLogger::buffer_char(char c)
{
    if (buffer_head < LOG_BUFFER_SIZE - 1) {
        log_buffer[buffer_head++] = c;
        log_buffer[buffer_head] = '\0';
    } else {
        buffer_overflow = true;
    }
}

void USBLogger::clear()
{
    if (is_busy())
        return;

    buffer_head = 0;
    buffer_overflow = false;
    log_buffer[0] = '\0';
    session_file_num = -1;
    session_filepath[0] = '\0';
    digitalWrite(PA_15, LOW);
}

bool USBLogger::is_busy() const
{
    return logger_state != LOGGER_IDLE;
}

void USBLogger::write_to_usb()
{
    if (buffer_head == 0 || is_busy())
        return;

    flush_target_length = buffer_head;
    flush_offset = 0;
    retry_cycles = 0;
    begin_attempt();
}

void USBLogger::begin_attempt()
{
    logger_state = LOGGER_POWER_SETTLE;
    state_started_ms = millis();
    next_attempt_ms = state_started_ms + 300;
    connect_attempts = 0;
    mount_attempts = 0;
    active_file = nullptr;
    filesystem_mounted = false;

    pinMode(LEDR, OUTPUT);
    pinMode(LEDG, OUTPUT);
    digitalWrite(LEDR, LOW);
    digitalWrite(LEDG, HIGH);
    pinMode(PA_15, OUTPUT);
    digitalWrite(PA_15, HIGH);

    if (!terminal_disconnect_protection && SERIAL_HW)
        SERIAL_HW.println("[LOGGER] Asynchronous USB save started.");
}

void USBLogger::remove_written_prefix(size_t count)
{
    if (count == 0 || count > buffer_head)
        return;

    memmove(log_buffer, log_buffer + count, buffer_head - count);
    buffer_head -= count;
    log_buffer[buffer_head] = '\0';
}

void USBLogger::retry_or_fail(const char *reason)
{
    if (!terminal_disconnect_protection && SERIAL_HW) {
        SERIAL_HW.print("[LOGGER] ");
        SERIAL_HW.println(reason);
    }

    if (active_file != nullptr) {
        fclose(active_file);
        active_file = nullptr;
    }
    if (filesystem_mounted) {
        usb_fs.unmount();
        filesystem_mounted = false;
    }
    digitalWrite(PA_15, LOW);

    if (++retry_cycles < 3 && flush_offset < flush_target_length) {
        begin_attempt();
        return;
    }

    finish_attempt(false);
}

void USBLogger::restart_full_flush(const char *reason)
{
    // A failed stdio write/flush may have reached the medium partially. Never
    // discard the RAM copy and never append an uncertain suffix to that file.
    // The retry creates a new sequential file containing the complete snapshot.
    if (active_file != nullptr) {
        fclose(active_file);
        active_file = nullptr;
    }
    session_file_num = -1;
    session_filepath[0] = '\0';
    flush_offset = 0;
    retry_or_fail(reason);
}

void USBLogger::finish_attempt(bool success)
{
    if (active_file != nullptr) {
        fflush(active_file);
        fclose(active_file);
        active_file = nullptr;
    }
    if (filesystem_mounted) {
        usb_fs.unmount();
        filesystem_mounted = false;
    }
    digitalWrite(PA_15, LOW);

    state_started_ms = millis();
    if (success) {
        remove_written_prefix(flush_target_length);
        flush_target_length = 0;
        flush_offset = 0;
        buffer_overflow = false;
        digitalWrite(LEDR, HIGH);
        digitalWrite(LEDG, LOW);
        logger_state = LOGGER_SUCCESS_FEEDBACK;
        if (!terminal_disconnect_protection && SERIAL_HW)
            SERIAL_HW.println("[LOGGER] Requested log data saved.");
    } else {
        digitalWrite(LEDR, LOW);
        digitalWrite(LEDG, LOW);
        logger_state = LOGGER_ERROR_FEEDBACK;
        if (!terminal_disconnect_protection && SERIAL_HW)
            SERIAL_HW.println("[LOGGER] Save incomplete; unwritten bytes remain buffered.");
    }
}

void USBLogger::update()
{
    update_terminal();
    const unsigned long now = millis();

    switch (logger_state) {
    case LOGGER_IDLE:
        return;

    case LOGGER_POWER_SETTLE:
        if ((long)(now - next_attempt_ms) >= 0) {
            logger_state = LOGGER_CONNECT_QUICK;
            next_attempt_ms = now;
        }
        return;

    case LOGGER_CONNECT_QUICK:
        if ((long)(now - next_attempt_ms) < 0)
            return;
        if (msd.connect()) {
            logger_state = LOGGER_MOUNT;
            next_attempt_ms = now;
            return;
        }
        ++connect_attempts;
        next_attempt_ms = now + 100;
        if (connect_attempts >= 5) {
            logger_state = LOGGER_CONNECT_EXTENDED_WAIT;
            next_attempt_ms = now + 1200;
        }
        return;

    case LOGGER_CONNECT_EXTENDED_WAIT:
        if ((long)(now - next_attempt_ms) >= 0) {
            logger_state = LOGGER_CONNECT_EXTENDED;
            connect_attempts = 0;
            next_attempt_ms = now;
        }
        return;

    case LOGGER_CONNECT_EXTENDED:
        if ((long)(now - next_attempt_ms) < 0)
            return;
        if (msd.connect()) {
            logger_state = LOGGER_MOUNT;
            next_attempt_ms = now;
            return;
        }
        ++connect_attempts;
        next_attempt_ms = now + 100;
        if (connect_attempts >= 30)
            retry_or_fail("USB drive not detected.");
        return;

    case LOGGER_MOUNT: {
        if ((long)(now - next_attempt_ms) < 0)
            return;
        const int error = usb_fs.mount(&msd);
        if (error == 0) {
            filesystem_mounted = true;
            logger_state = LOGGER_OPEN_FILE;
            return;
        }
        ++mount_attempts;
        next_attempt_ms = now + 100;
        if (mount_attempts >= 3)
            retry_or_fail("USB filesystem mount failed.");
        return;
    }

    case LOGGER_OPEN_FILE:
        if (session_file_num < 1) {
            int log_number = 1;
            do {
                snprintf(
                    session_filepath,
                    sizeof(session_filepath),
                    "/usb/log_%d.txt",
                    log_number++);
                FILE *probe = fopen(session_filepath, "r");
                if (probe == nullptr)
                    break;
                fclose(probe);
            } while (true);
            session_file_num = log_number - 1;
        }

        active_file = fopen(session_filepath, "a");
        if (active_file == nullptr) {
            retry_or_fail("Could not open log file.");
            return;
        }
        if (fseek(active_file, 0, SEEK_END) != 0 ||
            (active_file_start_size = ftell(active_file)) < 0) {
            restart_full_flush("Could not determine USB log size; retrying full log.");
            return;
        }
        logger_state = LOGGER_WRITE;
        return;

    case LOGGER_WRITE: {
        if (flush_offset >= flush_target_length) {
            bool finalized = true;
            if (buffer_overflow &&
                fprintf(active_file,
                        "\n*** WARNING: LOG BUFFER OVERFLOW ***\n") < 0)
                finalized = false;
            if (finalized && fflush(active_file) != 0)
                finalized = false;
            if (ferror(active_file))
                finalized = false;

            FILE *completed_file = active_file;
            active_file = nullptr;
            if (fclose(completed_file) != 0)
                finalized = false;

            // Reopen the file after close so a USB stack that acknowledged
            // writes but committed only a prefix cannot report false success.
            if (finalized) {
                FILE *verification_file = fopen(session_filepath, "rb");
                if (verification_file == nullptr ||
                    fseek(verification_file, 0, SEEK_END) != 0) {
                    finalized = false;
                } else {
                    const long saved_size = ftell(verification_file);
                    const long expected_minimum =
                        active_file_start_size +
                        static_cast<long>(flush_target_length);
                    if (saved_size < expected_minimum)
                        finalized = false;
                }
                if (verification_file != nullptr &&
                    fclose(verification_file) != 0)
                    finalized = false;
            }

            if (!finalized) {
                restart_full_flush("USB final verification failed; retrying full log.");
                return;
            }
            finish_attempt(true);
            return;
        }

        static const size_t CHUNK_SIZE = 512;
        const size_t remaining = flush_target_length - flush_offset;
        const size_t requested =
            remaining < CHUNK_SIZE
                ? remaining
                : CHUNK_SIZE;

        const size_t written =
            fwrite(log_buffer + flush_offset, 1, requested, active_file);
        const bool durable_chunk =
            written == requested &&
            fflush(active_file) == 0 &&
            !ferror(active_file);
        if (durable_chunk) {
            flush_offset += requested;
        } else {
            clearerr(active_file);
            // Do not remove any bytes from RAM. Start a new complete recovery
            // file immediately because the old file's last chunk is uncertain.
            restart_full_flush("USB write verification failed; retrying full log.");
        }
        return;
    }

    case LOGGER_SUCCESS_FEEDBACK:
        if (now - state_started_ms >= 1000) {
            digitalWrite(LEDG, HIGH);
            logger_state = LOGGER_IDLE;
        }
        return;

    case LOGGER_ERROR_FEEDBACK:
        if (now - state_started_ms >= 2000) {
            digitalWrite(LEDR, HIGH);
            digitalWrite(LEDG, HIGH);
            logger_state = LOGGER_IDLE;
        }
        return;
    }
}

USBLogger robot_logger;
