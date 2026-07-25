/**
 * @file main.cpp
 * @brief Main application entry point
 * 
 * Organizes robot control subsystems:
 *   - Motor control (DC motor, steering servo, PID)
 *   - Sensor management (Gyro, ToF distance sensors)
 *   - Serial communication and command parsing
 *   - Autonomous wall-following behavior
 *   - Calibration mode (optional, via #define CALIBRATION_MODE)
 *   - Position estimation (dead reckoning odometry)
 *   - Mode manager (centralized mode switching)
 * 
 * The main loop runs the currently active mode from the mode manager.
 * Only ONE mode runs at a time. The enable switch pauses/resumes modes.
 * 
 * To add a new mode:
 *   1. Add to RobotMode enum in mode_manager.h
 *   2. Add a case in the switch below
 *   3. Add serial command in serial_handler.cpp
 */

#include <Arduino.h>
#include "config.h"
#include "motor_control.h"
#include "sensors.h"
#include "serial_handler.h"
#include "wall_follower.h"
#include "calibration.h"
#include "position_estimator.h"
#include "mode_manager.h"
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

#ifdef CALIBRATION_MODE
  // In calibration mode: wait 5 seconds, then auto-start calibration.
  // During the delay, you can unplug the USB cable so the robot runs
  // fully standalone on battery power.
  Serial.print("Calibration mode active. Waiting ");
  Serial.print(CAL_STARTUP_DELAY_MS / 1000);
  Serial.println(" seconds before starting...");
  Serial.println("Unplug USB now if running on battery.");
  delay(CAL_STARTUP_DELAY_MS);
  
  // Start calibration automatically via mode manager
  mode_switch(MODE_CALIBRATION);
  Serial.println("Auto-calibration started.");
#else
  // Normal mode: if the enable switch is already HIGH at startup,
  // we don't start any mode automatically. The user must start one
  // via serial command (l for gyro follow, etc.) or toggle the switch.
  // This allows starting with switch disabled, then enabling later.
  if (system_enabled)
  {
    system_enable();
    Serial.println("System enabled. Send 'l' to start gyro follow, 'c' for calibration.");
  }
  else
  {
    Serial.println("System disabled (switch OFF). Toggle switch or send serial command.");
  }
#endif
}

// ==========================================
// MAIN LOOP - Coordinate all subsystems
// ==========================================

/**
 * @brief Continuous execution loop. Manages sensor updates, 
 * state machines, and low-level motor control.
 * 
 * Runs the currently active mode from the mode manager.
 * The enable switch (via handle_enable_switch) calls mode_pause/mode_resume.
 */
void loop()
{
  // Update timing and distances
  loop_updater();

  // Handle enable switch state (calls mode_pause/mode_resume)
  handle_enable_switch();

  // Check for and process serial commands (non-blocking)
  check_serial_available();

  // Update distance sensors (needed for all subsystems)
  update_lasers();
  update_gyro();

  // Update position estimate (dead reckoning from encoder + gyro)
  update_position();

  // ==========================================
  // RUN THE ACTIVE MODE
  // ==========================================
  switch (current_mode)
  {
    case MODE_GYRO_FOLLOW:
      // Gyro-stabilized wall following + motor control
      check_stalling();
      gyro_follower_update(true);
      drive_loop();
      break;

    case MODE_CALIBRATION:
      // Turn radius calibration (controls steering/speed directly)
      calibration_update();
      break;

    case MODE_NONE:
    default:
      // No active mode - nothing to do
      break;
  }
}