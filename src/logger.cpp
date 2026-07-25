/**
 * @file logger.cpp
 * @brief USB RAM-buffered logger implementation.
 * 
 * This file must NOT include config.h (which would cause circular macro issues).
 * It uses _UART_USB_ directly, which is the underlying Arduino GIGA USB CDC serial.
 */

#include "logger.h"

#include <Arduino_USBHostMbed5.h>
#include <FATFileSystem.h>

// Use the raw Arduino GIGA USB CDC serial object directly.
// On the GIGA R1, 'Serial' is a macro for '_UART_USB_', which is the USB CDC.
// We reference the underlying object to avoid the macro clash with our own logger.
#define SERIAL_HW _UART_USB_

// USB Host MSD and FileSystem instances (static to this file)
static USBHostMSD msd;
static mbed::FATFileSystem usb_fs("usb");

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
USBLogger::USBLogger() {
    buffer_head = 0;
    buffer_overflow = false;
    log_buffer[0] = '\0';
    session_file_num = -1;
    session_filepath[0] = '\0';
}
 
// ---------------------------------------------------------------------------
// Stream interface — delegates to hardware UART
// ---------------------------------------------------------------------------
void USBLogger::begin(unsigned long baud) {
    SERIAL_HW.begin(baud);
}

int USBLogger::available() {
    return SERIAL_HW.available();
}

int USBLogger::read() {
    return SERIAL_HW.read();
}

int USBLogger::peek() {
    return SERIAL_HW.peek();
}

void USBLogger::flush() {
    SERIAL_HW.flush();
}

USBLogger::operator bool() {
    return (bool)SERIAL_HW;
}

// ---------------------------------------------------------------------------
// Print interface — buffer the char AND forward to hardware serial if connected
// ---------------------------------------------------------------------------
size_t USBLogger::write(uint8_t c) {
    buffer_char((char)c);
    if (SERIAL_HW) {
        SERIAL_HW.write(c);
    }
    return 1;
}

size_t USBLogger::write(const uint8_t *buffer, size_t size) {
    for (size_t i = 0; i < size; i++) {
        buffer_char((char)buffer[i]);
    }
    if (SERIAL_HW) {
        SERIAL_HW.write(buffer, size);
    }
    return size;
}

// ---------------------------------------------------------------------------
// Private helper — append one char to RAM buffer
// ---------------------------------------------------------------------------
void USBLogger::buffer_char(char c) {
    if (buffer_head < LOG_BUFFER_SIZE - 1) {
        log_buffer[buffer_head++] = c;
        log_buffer[buffer_head] = '\0';
    } else {
        buffer_overflow = true;
    }
}

// ---------------------------------------------------------------------------
// clear() — reset buffer
// ---------------------------------------------------------------------------
void USBLogger::clear() {
    buffer_head = 0;
    buffer_overflow = false;
    log_buffer[0] = '\0';
    session_file_num = -1;
    session_filepath[0] = '\0';

    // Ensure LED pins are configured and all LEDs are OFF (GIGA LEDs are active low)
    pinMode(LEDR, OUTPUT);
    pinMode(LEDG, OUTPUT);
    pinMode(LEDB, OUTPUT);
    digitalWrite(LEDR, HIGH);
    digitalWrite(LEDG, HIGH);
    digitalWrite(LEDB, HIGH);
}

// ---------------------------------------------------------------------------
// write_to_usb() — mount drive, write buffered log, unmount, LED feedback
// ---------------------------------------------------------------------------
void USBLogger::write_to_usb() {
    // Nothing to save
    if (buffer_head == 0) {
        return;
    }

    // 1. RED LED ON — signals "USB write in progress, do not power off"
    pinMode(LEDR, OUTPUT);
    pinMode(LEDG, OUTPUT);
    digitalWrite(LEDR, LOW);   // Active LOW = ON
    digitalWrite(LEDG, HIGH);  // OFF

    if (SERIAL_HW) {
        SERIAL_HW.println("\n[LOGGER] Saving log to USB drive...");
    }

    // 2. Power up USB-A port via PA_15 power enable pin
    pinMode(PA_15, OUTPUT);
    digitalWrite(PA_15, HIGH);
    delay(300); // Let device settle

    // 3. Connect with up to 10 attempts (1 second total)
    bool connected = false;
    for (int i = 0; i < 10; i++) {
        if (msd.connect()) {
            connected = true;
            break;
        }
        delay(100);
    }

    if (!connected) {
        if (SERIAL_HW) SERIAL_HW.println("[LOGGER] No USB drive detected. Log not saved.");
        digitalWrite(LEDR, HIGH); // LED OFF
        digitalWrite(PA_15, LOW); // Cut USB power
        return;
    }

    // 4. Mount FAT filesystem with retry
    int err = 0;
    for (int mount_attempt = 0; mount_attempt < 3; mount_attempt++) {
        err = usb_fs.mount(&msd);
        if (err == 0) {
            break;
        }
        if (SERIAL_HW) {
            SERIAL_HW.print("[LOGGER] USB mount attempt ");
            SERIAL_HW.print(mount_attempt + 1);
            SERIAL_HW.print(" failed. Error: ");
            SERIAL_HW.println(err);
        }
        delay(100);
    }
    if (err) {
        if (SERIAL_HW) {
            SERIAL_HW.print("[LOGGER] USB mount failed after retries. Error: ");
            SERIAL_HW.println(err);
        }
        digitalWrite(LEDR, HIGH);
        digitalWrite(PA_15, LOW);
        return;
    }

    // 5. Find next available sequential filename if not already determined for this boot/reset session
    if (session_file_num < 1) {
        int log_num = 1;
        while (true) {
            snprintf(session_filepath, sizeof(session_filepath), "/usb/log_%d.txt", log_num);
            FILE *probe = fopen(session_filepath, "r");
            if (probe == NULL) {
                session_file_num = log_num;
                break; // This filename is free
            }
            fclose(probe);
            log_num++;
        }
    }

    // 6. Write log buffer to file in small chunks with error checking
    // Use append mode so repeated writes in the same run extend the same log file.
    FILE *f = fopen(session_filepath, "a");
    if (f == NULL) {
        if (SERIAL_HW) {
            SERIAL_HW.print("[LOGGER] Could not open file for append: ");
            SERIAL_HW.println(session_filepath);
        }
    } else {
        // Write in small chunks to avoid filesystem buffer overflows
        static const size_t CHUNK_SIZE = 512;
        size_t total_written = 0;
        size_t remaining = buffer_head;
        const char *src = log_buffer;

        while (remaining > 0) {
            size_t to_write = (remaining < CHUNK_SIZE) ? remaining : CHUNK_SIZE;
            size_t written = fwrite(src, 1, to_write, f);
            total_written += written;
            
            if (written < to_write) {
                // Write failed or truncated — report and stop
                if (SERIAL_HW) {
                    SERIAL_HW.print("[LOGGER] WARNING: fwrite truncated at byte ");
                    SERIAL_HW.print(total_written);
                    SERIAL_HW.print(" / ");
                    SERIAL_HW.println(buffer_head);
                }
                break;
            }
            
            src += written;
            remaining -= written;
        }
        
        if (buffer_overflow) {
            fprintf(f, "\n*** WARNING: LOG BUFFER OVERFLOW - SOME EARLY LOGS WERE LOST ***\n");
        }
        
        // Flush to ensure data is written before unmount
        fflush(f);
        fclose(f);
        
        if (SERIAL_HW) {
            SERIAL_HW.print("[LOGGER] Log saved to ");
            SERIAL_HW.print(session_filepath);
            SERIAL_HW.print(" (");
            SERIAL_HW.print(total_written);
            SERIAL_HW.print(" / ");
            SERIAL_HW.print(buffer_head);
            SERIAL_HW.println(" bytes written)");
        }
        
        // Clear buffer after successful write to avoid re-writing stale data
        buffer_head = 0;
        buffer_overflow = false;
        log_buffer[0] = '\0';
    }

    // 7. Safely unmount and power down USB port
    usb_fs.unmount();
    digitalWrite(PA_15, LOW);

    // 8. GREEN LED for 1s → signals "Safe to power off / remove USB"
    digitalWrite(LEDR, HIGH); // RED OFF
    digitalWrite(LEDG, LOW);  // GREEN ON
    delay(1000);
    digitalWrite(LEDG, HIGH); // GREEN OFF
}

// ---------------------------------------------------------------------------
// Global singleton
// ---------------------------------------------------------------------------
USBLogger robot_logger;
