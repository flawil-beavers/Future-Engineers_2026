#pragma once

#include <Arduino.h>
#include <Stream.h>

#define LOG_BUFFER_SIZE (128 * 1024) // 128 KB RAM buffer

/**
 * @brief USB Logger class that intercepts Serial output, buffers it in RAM,
 * and writes it to a USB flash drive when requested.
 * 
 * Extends arduino::Stream so it can be used as a drop-in replacement for Serial
 * in all robot source files via a local #define Serial robot_logger.
 */
class USBLogger : public arduino::Stream {
public:
    USBLogger();

    // --- Stream / Print interface ---
    size_t write(uint8_t c) override;
    size_t write(const uint8_t *buffer, size_t size) override;
    int available() override;
    int read() override;
    int peek() override;
    void flush() override;
    operator bool();
    
    // Needed for Serial.begin() call in serial_setup()
    void begin(unsigned long baud);

    // --- Logger control ---
    /**
     * @brief Flush the RAM log buffer to a sequentially-numbered file on the USB drive.
     * Turns the onboard RGB LED RED while writing, then GREEN briefly when done.
     * Safe to call when motors are stopped.
     */
    void write_to_usb();

    /**
     * @brief Clear the RAM log buffer (call on each new run via system_enable()).
     */
    void clear();

private:
    char log_buffer[LOG_BUFFER_SIZE];
    size_t buffer_head;
    bool buffer_overflow;
    int session_file_num;
    char session_filepath[64];

    void buffer_char(char c);
};

extern USBLogger robot_logger;
