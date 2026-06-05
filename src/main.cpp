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
  
  // Configure enable switch
  pinMode(ENABLE_SWITCH_PIN, INPUT);

  // Check initial enable switch state
  system_enabled = digitalRead(ENABLE_SWITCH_PIN); // Start enabled only if switch is already HIGH

  Serial.begin(SERIAL_BAUD);
  while (!Serial && !system_enabled)
  {
    system_enabled = digitalRead(ENABLE_SWITCH_PIN);
    delay(10);
  }

  // Determine if we are starting in an idle or active state
  if (!system_enabled)
  {
    Serial.println("Enable switch is LOW - System starting in IDLE mode");
  }
  else {
    Serial.println("Enable switch is HIGH - System ENABLED");
  }

  Serial.println("\n===== ROBOT INITIALIZATION START =====\n");

  // Initialize each subsystem in order
  serial_setup();
  sensors_setup();
  motor_control_setup();
  wall_follower_setup();
  
  Serial.println("\n===== INITIALIZATION COMPLETE =====\n");

  // Start wall following immediately if switch is enabled at startup
  if (system_enabled)
  {
    wall_follower_enable();
  }

}

// ==========================================
// MAIN LOOP - Coordinate all subsystems
// ==========================================

void loop()
{
  // Update timing and distances
  loop_updater();

  // Handle enable switch state
  handle_enable_switch();



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

  // Only proceed if system is enabled via the enable switch
  if (!system_enabled)
  {
    return; // Skip all other operations when disabled
  }
  
  // Execute autonomous wall-following or manual control
  wall_follower_update();

  // Execute motor control logic (may be overridden by wall_follower)
  drive_loop();

  // Optional: Print debug info (uncomment to enable)
  // pid_config_print();
}