#pragma once

/**
 * @file serial_handler.h
 * @brief Serial communication and command parsing subsystem
 * 
 * Handles incoming serial commands and responses.
 * Commands:
 *   d<speed>  : Set drive speed (mm/s)
 *   s<angle>  : Set steering angle (degrees)
 *   n         : Print current distance
 *   p         : Pause/stop
 *   h         : Hold position
 *   q<value>  : Set Kp tuning parameter (value/10)
 *   w<value>  : Set Ki tuning parameter (value/100)
 *   e<value>  : Set Kd tuning parameter (value/10)
 *   g         : Print current gyro angle
 *   v         : Print ToF distances
 *   a<value>  : Set acceleration (mm/s^2)
 *   r         : Resume with last speed
 *   x         : Print steering timing difference
 *   m         : Master enable (motors + steering)
 *   w         : Wall follower START
 *   z         : Wall follower STOP
 *   b1/b0     : Stationary obstacle bench test ON/OFF
 *   c         : Print camera-center HSV for colour calibration
 *   j         : Print learned obstacle course map
 *   u<dist>   : Set wall target distance (mm)
 *   i         : Enable wall follower debug output
 *   o         : Disable wall follower debug output
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
