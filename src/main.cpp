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
#include "obstacle.h"

CameraSystem camera;
Vision vision;
#include "logger.h"
// Redirect all Serial output in this file through the USB logger
#define Serial robot_logger

// ==========================================
// SETUP - Initialize all subsystems
// ==========================================

/**
 * @brief System entry point. Initializes hardware and starts the
 * control loop interface.
 */
void setup()
{
  Serial.println("\n===== ROBOT INITIALIZATION START =====\n");

  // Initialize each subsystem in order
  system_interface_setup(); // Sets up Enable Switch and Serial
  sensors_setup();
  motor_control_setup();
  gyro_follower_setup();

  Serial.println("\n===== INITIALIZATION COMPLETE =====\n");
  // Fully enable system if switch is already HIGH at startup
  if (system_enabled)
  {
    system_enable();
  }
  

  if (!camera.begin())
  {
    Serial.println("Camera failed!");

    while (true)
      ;
  }
  Serial.println("Camera initialized.");

  vision.begin();
  Serial.println("Vision initialized.");
}


// ==========================================
// MAIN LOOP - Coordinate all subsystems
// ==========================================

/**
 * @brief Continuous execution loop. Manages sensor updates,
 * state machines, and low-level motor control.
 */
void loop()
{
  // Update timing and distances
  loop_updater();

  // Handle enable switch state
  handle_enable_switch();

  // Check for and process serial commands
  check_serial_available();

  // Camera + Image processing
  updateCameraVision();

  // Evaluate results
  handleObstacleDetection();

  // Debug only during development
  printVisionDebug();

  // Monitor motor health
  check_stalling();

  // Update distance sensors (needed for all subsystems)
  update_lasers();
  update_gyro();

  // Update wall follower (internal logic handles suppression but allows telemetry)
  gyro_follower_update(system_enabled);

  // Execute motor control logic (may be overridden by wall_follower)
  drive_loop();

  // Optional: Print debug info (uncomment to enable)
  // pid_config_print();
}




