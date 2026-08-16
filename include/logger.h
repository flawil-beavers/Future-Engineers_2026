#pragma once

#include <Arduino.h>
#include <Stream.h>
#include <stdio.h>

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

    /** Keep terminal I/O active, but make writes non-blocking while unplugging. */
    void protect_from_terminal_disconnect();
    void allow_blocking_terminal_output();

    // --- Logger control ---
    /**
     * @brief Flush the RAM log buffer to a sequentially-numbered file on the USB drive.
     * Turns the onboard RGB LED RED while writing, then GREEN briefly when done.
     * Safe to call when motors are stopped.
     */
    void write_to_usb();
    void update();
    bool is_busy() const;

    /**
     * @brief Clear the RAM log buffer (call on each new run via system_enable()).
     */
    void clear();

private:
    static constexpr size_t TERMINAL_TX_BUFFER_SIZE = 4096;

    enum LoggerState {
        LOGGER_IDLE,
        LOGGER_POWER_SETTLE,
        LOGGER_CONNECT_QUICK,
        LOGGER_CONNECT_EXTENDED_WAIT,
        LOGGER_CONNECT_EXTENDED,
        LOGGER_MOUNT,
        LOGGER_OPEN_FILE,
        LOGGER_WRITE,
        LOGGER_SUCCESS_FEEDBACK,
        LOGGER_ERROR_FEEDBACK
    };

    char log_buffer[LOG_BUFFER_SIZE];
    size_t buffer_head;
    bool buffer_overflow;
    int session_file_num;
    char session_filepath[64];
    LoggerState logger_state;
    unsigned long state_started_ms;
    unsigned long next_attempt_ms;
    uint8_t connect_attempts;
    uint8_t mount_attempts;
    uint8_t retry_cycles;
    // A save operates on a stable prefix of log_buffer. Bytes are retained in
    // RAM until the complete prefix has been flushed and the file closed.
    size_t flush_target_length;
    size_t flush_offset;
    FILE *active_file;
    long active_file_start_size;
    bool filesystem_mounted;
    bool terminal_disconnect_protection;
    uint8_t terminal_tx_buffer[TERMINAL_TX_BUFFER_SIZE];
    size_t terminal_tx_head;
    size_t terminal_tx_tail;
    size_t terminal_tx_count;

    void buffer_char(char c);
    void buffer_terminal(const uint8_t *data, size_t size);
    void update_terminal();
    void begin_attempt();
    void retry_or_fail(const char *reason);
    void restart_full_flush(const char *reason);
    void finish_attempt(bool success);
    void remove_written_prefix(size_t count);
};

extern USBLogger robot_logger;
