#pragma once

/**
 * @file turn_radius_calibration.h
 * @brief Automatic turn radius calibration subsystem
 * 
 * Drives the robot in circles at various steering angles, measures the
 * actual turn radius using encoder distance and gyro heading, and outputs
 * a polynomial fit: R(servo_angle) = a0 + a1*|δ| + a2*|δ|² + a3*|δ|³
 * 
 * The servo "angle" here refers to the offset from SERVO_CENTER (81),
 * i.e. the value passed to set_steering(). Range: ±50.
 * 
 * Uses shared data types (TRCalPoint, TRCalResult) and utility functions
 * from calibration.h.
 */

#include <Arduino.h>
#include "calibration.h"

// ==========================================
// TURN RADIUS CALIBRATION STATE MACHINE
// ==========================================

enum TurnRadiusState {
    TR_IDLE,           ///< Not running
    TR_SETTLING,       ///< Waiting for the steering servo before measuring
    TR_DRIVING,        ///< Driving in a circle, waiting for 360°
    TR_STOPPING,       ///< Reached 360°, stopping motors
    TR_NEXT_ANGLE,     ///< Transitioning to next angle
    TR_DONE,           ///< All angles measured
    TR_FAILED          ///< Aborted after repeated invalid measurements
};

// ==========================================
// EXTERNAL STATE
// ==========================================

extern TurnRadiusState turn_radius_state;
extern int tr_cal_current_angle_index;
extern int tr_cal_current_angle;

// ==========================================
// PUBLIC FUNCTIONS
// ==========================================

/**
 * @brief Start the turn radius calibration sequence
 * Resets all state and begins measuring at the first steering angle.
 */
void turn_radius_cal_start();

/**
 * @brief Main turn radius calibration update function. Call every loop.
 * 
 * Drives the state machine: drives in circles, measures radius,
 * advances through angles, and finally outputs results.
 */
void turn_radius_cal_update();

/**
 * @brief Stop turn radius calibration immediately. Resets state to TR_IDLE.
 * Safe to call even if calibration is not running.
 */
void turn_radius_cal_stop();

/**
 * @brief Check if turn radius calibration is currently running
 * @return true if state machine is active
 */
bool turn_radius_cal_is_active();

/** Return true after the complete left phase while the right phase is pending. */
bool turn_radius_cal_waiting_for_right();
