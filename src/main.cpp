/**
 * @file main.cpp
 * @brief Main application entry point
 * 
 * Organizes robot control subsystems:
 *   - Motor control (DC motor, steering servo, PID)
 *   - Sensor management (Gyro, ToF distance sensors)
 *   - Serial communication and command parsing
 *   - Autonomous wall-following behavior
 * 
 * The main loop coordinates all subsystems in a clean, modular architecture.
 */

#include <Arduino.h>
#include "config.h"
#include "motor_control.h"
#include "sensors.h"
#include "serial_handler.h"
#include "wall_follower.h"

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
  wall_follower_setup();

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

  // Update distance sensors (needed for all subsystems)
  // unsigned long time_1 = micros();
  update_lasers();
  // unsigned long time_2 = micros();
  update_gyro();
  // unsigned long time_3 = micros();
  // Serial.print("update_lasers: ");
  // Serial.print((float)(time_2 - time_1)/1000);
  // Serial.print("ms | update_gyro: ");
  // Serial.print((float)(time_3 - time_2)/1000);
  // Serial.println("ms");

  // Execute autonomous wall-following or manual control
  wall_follower_update();

  // Execute motor control logic (may be overridden by wall_follower)
  drive_loop();

  // Optional: Print debug info (uncomment to enable)
  // pid_config_print();
}