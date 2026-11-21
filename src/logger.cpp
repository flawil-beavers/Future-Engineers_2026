/**
 * @file logger.cpp
 * @brief Non-blocking USB RAM-buffered logger.
 */

#include "logger.h"
#include "motor_control.h"

#include <Arduino_USBHostMbed5.h>
#include <FATFileSystem.h>
#include <USB/PluggableUSBSerial.h>
#include <string.h>
#include <errno.h>
#include "drivers/DigitalOut.h"
#include "drivers/Timeout.h"
#include "platform/mbed_stats.h"

#define SERIAL_HW _UART_USB_

static USBHostMSD msd;
static mbed::FATFileSystem usb_fs("usb");
static mbed::DigitalOut usb_vbus(PA_15, 0);
static mbed::DigitalOut save_red(digitalPinToPinName(LEDR), 1);
static mbed::DigitalOut save_green(digitalPinToPinName(LEDG), 1);
static mbed::Timeout save_deadline;

static void setUsbPower(bool on) { usb_vbus.write(on); }
static void setSaveLights(bool red, bool green)
{
    save_red.write(!red);
    save_green.write(!green);
}

void USBLogger::expire_save()
{
    // This callback can run while FAT or USBHost is blocked. Only touch the
    // power switch, status pins and flag. The caller owns all open handles.
    save_timed_out = true;
    setUsbPower(false);
    setSaveLights(true,true);
}

void USBLogger::refresh_save_deadline()
{
    constexpr unsigned long maxSaveMs = 60000;
    constexpr unsigned long maxWithoutProgressMs = 8000;
    mbed::CriticalSectionLock lock;
    if (save_timed_out) return;
    const unsigned long now = millis();
    if (!save_timer_running) {
        save_timer_running = true;
        save_started_ms = now;
    }
    const unsigned long elapsed = now - save_started_ms;
    if (elapsed >= maxSaveMs) {
        expire_save();
        return;
    }
    const unsigned long remaining = maxSaveMs - elapsed;
    save_deadline.attach(mbed::callback(this, &USBLogger::expire_save),
        std::chrono::milliseconds(remaining < maxWithoutProgressMs ? remaining : maxWithoutProgressMs));
}

void USBLogger::stop_save_deadline()
{
    save_deadline.detach();
    save_timer_running = false;
}

// Terminal-only diagnostics while stopped: identify the last USB call reached
// before a fatal error. No log-buffer allocation or new driving output.
void USBLogger::log_save_stage(const char *stage)
{
    if(diagnostic_stage && strcmp(diagnostic_stage,stage)==0)return;
    diagnostic_stage=stage;
    refresh_save_deadline();
    if(dc_state==DC_ENABLED || !_SerialUSB.connected())return;
    mbed_stats_heap_t heap={};
    mbed_stats_heap_get(&heap);
    char line[144];
    const int length = snprintf(line, sizeof(line),
        "[LOGGER USB] stage=%s heap_used/peak/reserved/fails=%lu/%lu/%lu/%lu\r\n", stage,
        static_cast<unsigned long>(heap.current_size), static_cast<unsigned long>(heap.max_size),
        static_cast<unsigned long>(heap.reserved_size), static_cast<unsigned long>(heap.alloc_fail_cnt));
    if (length > 0 && static_cast<size_t>(length) < sizeof(line)) {
        buffer_terminal(reinterpret_cast<const uint8_t *>(line), static_cast<size_t>(length));
        update_terminal();
    }
}

void USBLogger::report_save_status(const char *message)
{
    if (!_SerialUSB.connected()) return;
    char line[144];
    const int length = snprintf(line, sizeof(line), "[LOGGER] %s\r\n", message);
    if (length > 0 && static_cast<size_t>(length) < sizeof(line))
        buffer_terminal(reinterpret_cast<const uint8_t *>(line), static_cast<size_t>(length));
}

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
    if (terminal_tx_count == 0 || !_SerialUSB.connected())
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
    if (buffer_head < LOG_BUFFER_SIZE - 1 - tail_reserve) {
        log_buffer[buffer_head++] = c;
        log_buffer[buffer_head] = '\0';
    } else {
        buffer_overflow = true;
    }
}

void USBLogger::clear()
{
    // A failed USB save leaves its snapshot in RAM. Starting another run must
    // not discard that evidence just because the retry state machine is idle.
    if (is_busy() || flush_target_length != 0)
        return;

    logger_state = LOGGER_IDLE;
    setSaveLights(false,false);

    buffer_head = 0;
    tail_reserve = 0;
    buffer_overflow = false;
    log_buffer[0] = '\0';
    session_file_num = -1;
    next_probe_number = 1;
    session_filepath[0] = '\0';
    setUsbPower(false);
}

bool USBLogger::is_busy() const
{
    // The file is already closed/unmounted during LED feedback. A cosmetic
    // green/red indication must not postpone an otherwise valid new start.
    return logger_state != LOGGER_IDLE &&
        logger_state != LOGGER_SUCCESS_FEEDBACK &&
        logger_state != LOGGER_ERROR_FEEDBACK;
}

void USBLogger::write_to_usb()
{
    if (buffer_head == 0)
        return;
    if (is_busy()) {
        // The OFF switch can arrive while the automatic abort export is active.
        // Honour that request if the export fails or leaves a new suffix.
        queued_flush_length = buffer_head;
        return;
    }

    flush_target_length = buffer_head;
    flush_offset = 0;
    retry_cycles = 0;
    save_timed_out = false;
    begin_attempt();
}

void USBLogger::begin_attempt()
{
    diagnostic_stage=nullptr;
    state_started_ms = millis();
    // USBHostMSD keeps its connection until the host processes removal.
    // A LOW/HIGH pulse within the same call can leave the old device mounted.
    logger_state = LOGGER_POWER_OFF_WAIT;
    next_attempt_ms = usb_has_powered_off ? usb_power_off_ms + 300 : state_started_ms;
    connect_attempts = 0;
    mount_attempts = 0;
    active_file = nullptr;
    filesystem_mounted = false;

    setSaveLights(true,false);

    report_save_status("USB save started.");
}

void USBLogger::power_off()
{
    setUsbPower(false);
    usb_power_off_ms = millis();
    usb_has_powered_off = true;
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
    if (directory_scan != nullptr) {
        closedir(directory_scan);
        directory_scan = nullptr;
    }
    report_save_status(reason);

    if (active_file != nullptr) {
        fclose(active_file);
        active_file = nullptr;
    }
    if (filesystem_mounted) {
        log_save_stage("abort_unmount");
        usb_fs.unmount();
        filesystem_mounted = false;
    }
    if (!save_timed_out && ++retry_cycles < 3 && flush_offset < flush_target_length) {
        power_off();
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
    if (session_file_num > 0) next_probe_number = session_file_num + 1;
    session_file_num = -1;
    session_filepath[0] = '\0';
    flush_offset = 0;
    retry_or_fail(reason);
}

void USBLogger::finish_attempt(bool success)
{
    if (active_file != nullptr) {
        if (success) fflush(active_file);
        fclose(active_file);
        active_file = nullptr;
    }
    if (filesystem_mounted) {
        log_save_stage(success ? "unmount" : "abort_unmount");
        if (usb_fs.unmount() != 0) success = false;
        filesystem_mounted = false;
    }
    stop_save_deadline();
    success = success && !save_timed_out;
    power_off();

    state_started_ms = millis();
    const size_t requested = queued_flush_length;
    queued_flush_length = 0;
    if (success) {
        const size_t saved = flush_target_length;
        remove_written_prefix(flush_target_length);
        flush_target_length = 0;
        flush_offset = 0;
        buffer_overflow = false;
        setSaveLights(false,true);
        logger_state = LOGGER_SUCCESS_FEEDBACK;
        report_save_status("Requested log data saved.");
        if (requested > saved) {
            flush_target_length = requested - saved;
            retry_cycles = 0;
            begin_attempt();
        }
    } else {
        // A file may contain an incomplete prefix. Retry to a new name.
        session_file_num = -1;
        session_filepath[0] = '\0';
        setSaveLights(true,true);
        logger_state = LOGGER_ERROR_FEEDBACK;
        report_save_status("Save incomplete; log retained in RAM.");
        if (requested != 0 && !save_timed_out) write_to_usb();
    }
}

void USBLogger::update()
{
    update_terminal();
    // File I/O may block in the USB stack even though our state machine yields.
    // Defer it during any speed-controlled drive; a settled position hold can
    // still save the completed parking run. Never interrupt an existing drive.
    if (dc_state == DC_ENABLED && is_busy()) return;
    if (save_timed_out && is_busy()) {
        retry_or_fail("USB save timed out; snapshot retained.");
        return;
    }
    const unsigned long now = millis();

    switch (logger_state) {
    case LOGGER_IDLE:
        return;

    case LOGGER_POWER_OFF_WAIT:
        if ((long)(now - next_attempt_ms) >= 0) {
            log_save_stage("power_settle");
            {
                mbed::CriticalSectionLock lock;
                if (!save_timed_out) setUsbPower(true);
            }
            logger_state = LOGGER_POWER_SETTLE;
            next_attempt_ms = now + 300;
        }
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
        log_save_stage("connect");
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
        log_save_stage("connect");
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
        log_save_stage("mount");
        const int error = usb_fs.mount(&msd);
        if (error == 0) {
            filesystem_mounted = true;
            next_probe_number = 1;
            log_save_stage("directory_open");
            directory_scan = opendir("/usb");
            if (directory_scan == nullptr) {
                retry_or_fail("Could not read USB log directory.");
                return;
            }
            logger_state = LOGGER_SCAN_DIRECTORY;
            return;
        }
        ++mount_attempts;
        next_attempt_ms = now + 100;
        if (mount_attempts >= 3)
            retry_or_fail("USB filesystem mount failed.");
        return;
    }

    case LOGGER_SCAN_DIRECTORY:
        log_save_stage("directory_scan");
        // Walk the directory once instead of reopening log_1..log_N on every
        // save. Each fopen caused another FAT name search, growing quadratically.
        for (uint8_t entries = 0; entries < 16; ++entries) {
            errno = 0;
            struct dirent *entry = readdir(directory_scan);
            if (entry == nullptr) {
                if (errno != 0) {
                    retry_or_fail("USB directory read failed.");
                    return;
                }
                closedir(directory_scan);
                directory_scan = nullptr;
                logger_state = LOGGER_OPEN_FILE;
                return;
            }
            unsigned long number = 0;
            int consumed = 0;
            if (sscanf(entry->d_name, "log_%lu.txt%n", &number, &consumed) == 1 &&
                consumed > 0 && entry->d_name[consumed] == '\0' &&
                number > 0 && number < 2147483647UL &&
                number >= static_cast<unsigned long>(next_probe_number))
                next_probe_number = static_cast<int>(number) + 1;
        }
        refresh_save_deadline();
        return;

    case LOGGER_OPEN_FILE:
        log_save_stage("file_open");
        if (session_file_num < 1) {
            for (uint8_t probes = 0; probes < 4; ++probes) {
                snprintf(
                    session_filepath,
                    sizeof(session_filepath),
                    "/usb/log_%d.txt",
                    next_probe_number);
                FILE *probe = fopen(session_filepath, "r");
                if (probe == nullptr) {
                    session_file_num = next_probe_number;
                    break;
                }
                fclose(probe);
                ++next_probe_number;
            }
            if (session_file_num < 1) return;
        }

        active_file = fopen(session_filepath, "ab");
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
            log_save_stage("finalize_close");
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

            if (!finalized) {
                restart_full_flush("Could not close USB log; retrying full log.");
                return;
            }
            logger_state = LOGGER_VERIFY_OPEN;
            return;
        }

        static const size_t CHUNK_SIZE = 4096;
        log_save_stage("write_flush");
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
            refresh_save_deadline();
        } else {
            clearerr(active_file);
            // Do not remove any bytes from RAM. Start a new complete recovery
            // file immediately because the old file's last chunk is uncertain.
            restart_full_flush("USB write verification failed; retrying full log.");
        }
        return;
    }

    case LOGGER_VERIFY_OPEN:
        log_save_stage("verify_open");
        active_file = fopen(session_filepath, "rb");
        if (active_file == nullptr || fseek(active_file, active_file_start_size, SEEK_SET) != 0) {
            restart_full_flush("Could not reopen saved USB log; retrying full log.");
            return;
        }
        verify_offset = 0;
        logger_state = LOGGER_VERIFY_READ;
        return;

    case LOGGER_VERIFY_READ: {
        if (verify_offset == flush_target_length) {
            log_save_stage("verify_close");
            FILE *verified = active_file;
            active_file = nullptr;
            if (fclose(verified) != 0) {
                restart_full_flush("Could not close verified USB log; retrying full log.");
                return;
            }
            finish_attempt(true);
            return;
        }
        log_save_stage("verify_read");
        uint8_t check[2048];
        const size_t remaining = flush_target_length - verify_offset;
        const size_t count = remaining < sizeof(check) ? remaining : sizeof(check);
        if (fread(check, 1, count, active_file) != count ||
            memcmp(check, log_buffer + verify_offset, count) != 0) {
            restart_full_flush("Saved USB bytes differ from RAM; retrying full log.");
            return;
        }
        verify_offset += count;
        refresh_save_deadline();
        return;
    }

    case LOGGER_SUCCESS_FEEDBACK:
        if (now - state_started_ms >= 1000) {
            setSaveLights(false,false);
            logger_state = LOGGER_IDLE;
        }
        return;

    case LOGGER_ERROR_FEEDBACK:
        if (now - state_started_ms >= 2000) {
            setSaveLights(false,false);
            logger_state = LOGGER_IDLE;
        }
        return;
    }
}

USBLogger robot_logger;
