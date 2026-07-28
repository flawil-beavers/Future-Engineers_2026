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
#include "turn_radius_calibration.h"
#include "servo_center_calibration.h"
#include "calibration.h"
#include "position_estimator.h"
#include "mode_manager.h"
#include "logger.h"
// Redirect all Serial output in this file through the USB logger
#define Serial robot_logger

// ==========================================
// CALIBRATION AUTO-START FLAG
// ==========================================

#ifdef CALIBRATION_MODE
static bool cal_auto_start_pending = true;
static unsigned long cal_auto_start_time = 0;
#endif

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
  
  // Load calibration polynomial coefficients from config.h into runtime structs.
  // This enables get_calibrated_radius() to work in position_estimator.cpp
  // for calibration-enhanced slip detection during normal operation.
  {
    float left_coeffs[4]  = {CAL_LEFT_A0,  CAL_LEFT_A1,  CAL_LEFT_A2,  CAL_LEFT_A3};
    float right_coeffs[4] = {CAL_RIGHT_A0, CAL_RIGHT_A1, CAL_RIGHT_A2, CAL_RIGHT_A3};
    calibration_set_coefficients(left_coeffs, right_coeffs);
  }

  Serial.println("\n===== INITIALIZATION COMPLETE =====\n");

#ifdef CALIBRATION_MODE
  // In calibration mode: wait 5 seconds, then auto-start calibration.
  // During the delay, you can unplug the USB cable so the robot runs
  // fully standalone on battery power.
  // NOTE: We use a non-blocking approach so the enable switch polling
  // continues to work throughout. Calibration starts after the delay
  // only if the system is enabled.
  Serial.print("Calibration mode active. Waiting ");
  Serial.print(CAL_STARTUP_DELAY_MS / 1000);
  Serial.println(" seconds before starting...");
  Serial.println("Unplug USB now if running on battery.");
  cal_auto_start_pending = true;
  cal_auto_start_time = millis();
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

#ifdef CALIBRATION_MODE
  // Non-blocking calibration auto-start:
  // Wait CAL_STARTUP_DELAY_MS, then start calibration if system is enabled.
  // This allows the enable switch to pause/resume even during startup.
  if (cal_auto_start_pending && (millis() - cal_auto_start_time >= CAL_STARTUP_DELAY_MS)) {
    cal_auto_start_pending = false;
    if (system_enabled) {
      mode_switch(MODE_TURN_RADIUS_CAL);
      Serial.println("Auto-calibration started.");
    } else {
      pending_mode = MODE_TURN_RADIUS_CAL;
      Serial.println("Auto-calibration skipped (system disabled). Toggle switch to enable.");
    }
  }
#endif

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

    case MODE_TURN_RADIUS_CAL:
      // Turn radius calibration (controls steering/speed directly)
      check_stalling();
      turn_radius_cal_update();
      drive_loop();   // Execute the steering and speed set by calibration
      break;

    case MODE_SERVO_CENTER_CAL:
      // Straight-line servo-center calibration
      check_stalling();
      servo_center_cal_update();
      drive_loop();
      break;

    case MODE_NONE:
    default:
      // No active mode - nothing to do
      break;
  }
}
