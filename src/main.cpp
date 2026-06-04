/**
 * @file main.cpp
 * @brief Main application entry point
 * 
 * Organizes robot control subsystems:
 *   - Motor control (DC motor, steering servo, PID)
 *   - Sensor management (Gyro, ToF distance sensors)
 *   - Serial communication and command parsing
 * 
 * The main loop coordinates all subsystems in a clean, modular architecture.
 */

#include <Arduino.h>
#include "config.h"
#include "motor_control.h"
#include "sensors.h"
#include "serial_handler.h"

// ==========================================
// SETUP - Initialize all subsystems
// ==========================================

void setup()
{
  Serial.begin(SERIAL_BAUD);
  while (!Serial)
  {
    delay(10);
  }

  Serial.println("\n===== ROBOT INITIALIZATION START =====\n");

  // Initialize each subsystem in order
  serial_setup();
  motor_control_setup();
  sensors_setup();

  Serial.println("\n===== INITIALIZATION COMPLETE =====\n");
}

// ==========================================
// MAIN LOOP - Coordinate all subsystems
// ==========================================

void loop()
{
  // Update timing and distances
  loop_updater();

  // Check for and process serial commands
  check_serial_available();

  // Monitor motor health
  check_stalling();

  // Execute motor control logic
  drive_loop();

  // Update distance sensors
  update_lasers();

  // Update heading (gyro)
  update_gyro();

  // Optional: Print debug info (uncomment to enable)
  // pid_config_print();
}