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
#include "camera.h"

// ==========================================
// SETUP - Initialize all subsystems
// ==========================================

void setup()
{
  Serial.println("\n===== ROBOT INITIALIZATION START =====\n");

  // Initialize each subsystem in order
  system_interface_setup(); // Sets up Enable Switch and Serial
  sensors_setup();
  motor_control_setup();
  camera_setup();
  gyro_follower_setup();
  
  Serial.println("\n===== INITIALIZATION COMPLETE =====\n");
  // Fully enable system if switch is already HIGH at startup
  if (system_enabled)
  {
    system_enable();
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
  update_lasers();
  update_gyro();

  // Process camera frames
  camera_update();
  CameraResults res = get_camera_results();
  if (res.red_block.found) {
    // Logic for red block detection (e.g. steer away or stop)
  }
  if (res.green_block.found) {
    // Logic for green block detection
  }

  // Update wall follower (internal logic handles suppression but allows telemetry)
  gyro_follower_update(system_enabled);
  
  // Execute motor control logic (may be overridden by wall_follower)
  drive_loop();

  // Optional: Print debug info (uncomment to enable)
  // pid_config_print();
}