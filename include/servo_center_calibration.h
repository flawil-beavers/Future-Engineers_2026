#pragma once

/**
 * @file servo_center_calibration.h
 * @brief Straight-line servo-center calibration subsystem
 * 
 * Drives the robot straight while using gyro-follow steering to estimate
 * the neutral servo position (the servo value that makes the robot drive
 * straight). Uses a weighted running mean to combine observations over
 * a straight run, producing both an estimate and an uncertainty value.
 */

#include <Arduino.h>

// ==========================================
// SERVO CENTER CALIBRATION STATE MACHINE
// ==========================================

enum ServoCenterState {
    SC_IDLE,           ///< Not running
    SC_DRIVING,        ///< Driving straight, measuring steering corrections
    SC_DONE            ///< Calibration complete, estimate available
};

// ==========================================
// EXTERNAL STATE
// ==========================================

extern ServoCenterState servo_center_state;

// ==========================================
// PUBLIC FUNCTIONS
// ==========================================

/**
 * @brief Start the straight-line servo-center calibration sequence.
 * Resets all state and begins driving straight while measuring
 * the steering corrections needed to maintain heading.
 */
void servo_center_cal_start();

/**
 * @brief Main servo-center calibration update function. Call every loop.
 * 
 * Drives straight, uses gyro-follow to correct heading, and builds
 * a weighted estimate of the neutral servo position.
 */
void servo_center_cal_update();

/**
 * @brief Stop servo-center calibration immediately. Resets state to SC_IDLE.
 * Safe to call even if calibration is not running.
 */
void servo_center_cal_stop();

/**
 * @brief Check if servo-center calibration is currently running
 * @return true if state machine is active
 */
bool servo_center_cal_is_active();