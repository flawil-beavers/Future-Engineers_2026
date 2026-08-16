#pragma once

/**
 * @file mode_manager.h
 * @brief Centralized mode management for the robot
 * 
 * Manages which autonomous mode is active (gyro follow, calibration, etc.)
 * and handles pause/resume via the enable switch.
 * 
 * Only ONE mode can be active at a time. Switching modes automatically
 * stops the previous mode and starts the new one.
 * 
 * To add a new mode in the future:
 *   1. Add to the RobotMode enum
 *   2. Add a case in the main loop switch
 *   3. Add the serial command in serial_handler.cpp
 */

#include <Arduino.h>

// ==========================================
// ROBOT MODE ENUM
// ==========================================

/**
 * @brief Available robot operation modes
 * 
 * Only one mode can be active at a time.
 * Add new modes here for future expansion.
 */
enum RobotMode {
    MODE_NONE,                  ///< No active mode, motors stopped
    MODE_MANUAL,                ///< Direct serial speed and steering control
    MODE_HOLD,                  ///< Actively hold the current encoder position
    MODE_OPEN_CHALLENGE,        ///< Gyro-stabilized Open Challenge
    MODE_OBSTACLE_CHALLENGE,    ///< Camera-based Obstacle Challenge
    MODE_OBSTACLE_PATH_TEST,    ///< One-lap empty-track Pure Pursuit test
    MODE_OBSTACLE_SEAT_TEST,    ///< Stationary Pure Pursuit seat-snap test
    MODE_OBSTACLE_BENCH,        ///< Stationary camera/steering obstacle test
    MODE_CAMERA_CALIBRATION,     ///< Stationary live camera HSV calibration
    MODE_TURN_RADIUS_CAL,       ///< Turn-radius calibration
    MODE_SERVO_CENTER_CAL,      ///< Straight servo-center calibration
    MODE_PID_AUTOTUNE,          ///< PID speed controller auto-tuning
    MODE_MOTOR_MIN_CAL,         ///< Motor minimum DC calibration
};

enum ModeResult {
    MODE_RESULT_RUNNING,
    MODE_RESULT_COMPLETED,
    MODE_RESULT_FAILED
};

// ==========================================
// EXTERNAL STATE
// ==========================================

extern RobotMode current_mode;      ///< The currently active mode
extern RobotMode pending_mode;      ///< Mode to resume when switch toggles ON

// ==========================================
// PUBLIC FUNCTIONS
// ==========================================

/**
 * @brief Switch to a new mode. Stops the old mode, starts the new one.
 * 
 * If the new mode is the same as the current mode, this is a no-op.
 * If the system is disabled, the mode is remembered as pending and
 * will start when the enable switch is toggled ON.
 * 
 * @param new_mode The mode to switch to
 */
void mode_switch(RobotMode new_mode);

/**
 * @brief Run one update of the active mode.
 */
void mode_update();

/**
 * @brief Print general debug information across all modes.
 */
void general_debug_print();

/**
 * @brief Pause the current mode. Stops motors but remembers the mode.
 * Called automatically when the enable switch is toggled OFF.
 */
void mode_pause();

/**
 * @brief Resume the pending mode. Called when enable switch toggles ON.
 * If no mode is pending, does nothing.
 */
void mode_resume();

/**
 * @brief Stop all modes. Sets current mode to MODE_NONE.
 * Motors are stopped and no mode will resume on switch toggle.
 */
void mode_stop_all();

/**
 * @brief Get a human-readable name for a mode
 * @param mode The mode to name
 * @return String like "GYRO_FOLLOW", "CALIBRATION", "NONE"
 */
const char* mode_name(RobotMode mode);
