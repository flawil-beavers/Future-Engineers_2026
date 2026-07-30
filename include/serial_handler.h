#pragma once

/**
 * @file serial_handler.h
 * @brief Serial communication and command parsing subsystem
 * 
 * Handles incoming serial commands and responses.
 * Commands:
 *   d<speed>  : Select manual mode and set drive speed (mm/s)
 *   s<angle>  : Select manual mode and set steering angle (degrees)
 *   n         : Print current distance
 *   p         : Pause/stop
 *   h         : Select HOLD mode
 *   q<value>  : Set Kp tuning parameter (value/10)
 *   w<value>  : Set Ki tuning parameter (value/100)
 *   e<value>  : Set Kd tuning parameter (value/10)
 *   g         : Print current gyro angle
 *   v         : Print ToF distances
 *   a<value>  : Set acceleration (mm/s^2)
 *   r         : Resume the pending mode
 *   x         : Print steering timing difference
 *   m         : Select manual mode
 *   l         : Start Open Challenge mode
 *   O         : Start Obstacle Challenge mode
 *   z         : Stop the active mode
 *   b1/b0     : Start/stop Obstacle Bench mode
 *   c         : Start Camera Calibration mode
 *   j         : Print learned obstacle course map
 *   u<dist>   : Set wall target distance (mm)
 *   i         : Print all serial commands
 *   f         : Enable navigation debug output
 *   o         : Disable navigation debug output
 *   C         : Start turn-radius calibration sequence
 *   B         : Start straight servo-center calibration
 *   y         : Start PID autotune
 *   t         : Print current position (x, y, heading, confidence)
 *   k         : Print calibration data summary
 */

#include <Arduino.h>

// ==========================================
// SERIAL BUFFER MANAGEMENT
// ==========================================

/**
 * @brief Check for available serial data and buffer it
 * Should be called regularly in the main loop
 */
void check_serial_available();

/**
 * @brief Process any complete messages in the buffer
 * Automatically called by check_serial_available()
 */
void processMessage();

/**
 * @brief Parse a command string and execute the corresponding action
 * @param msg The command message string
 */
void parseMessage(char *msg);

// ==========================================
// DEBUG OUTPUT
// ==========================================

/**
 * @brief Print PID configuration and debug information
 * Throttled to print every 200ms to avoid serial flooding
 */
void pid_config_print();

// ==========================================
// INITIALIZATION
// ==========================================

/**
 * @brief Initialize serial communication
 * Sets up serial port and ring buffer
 * Must be called during setup()
 */
void serial_setup();
