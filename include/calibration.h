#pragma once

/**
 * @file calibration.h
 * @brief Automatic turn radius calibration subsystem
 * 
 * Drives the robot in circles at various steering angles, measures the
 * actual turn radius using encoder distance and gyro heading, and outputs
 * a polynomial fit: R(servo_angle) = a0 + a1*|δ| + a2*|δ|² + a3*|δ|³
 * 
 * The servo "angle" here refers to the offset from SERVO_CENTER (81),
 * i.e. the value passed to set_steering(). Range: ±50.
 */

#include <Arduino.h>

// ==========================================
// CALIBRATION STATE MACHINE
// ==========================================

enum CalibrationState {
    CAL_IDLE,           ///< Not running
    CAL_DRIVING,        ///< Driving in a circle, waiting for 360°
    CAL_STOPPING,       ///< Reached 360°, stopping motors
    CAL_NEXT_ANGLE,     ///< Transitioning to next angle
    CAL_DONE            ///< All angles measured
};

// ==========================================
// CALIBRATION DATA STRUCTURE
// ==========================================

/**
 * @brief A single calibration measurement point
 */
struct CalPoint {
    int servo_angle;        ///< Servo offset from center (e.g. 25, -30)
    float radius_mm;        ///< Measured turn radius
    float distance_mm;      ///< Total distance traveled for 360°
};

/**
 * @brief Complete calibration results for one direction
 */
struct CalResult {
    int num_points;                 ///< Number of valid measurements
    CalPoint points[12];            ///< Measurement data (max 12 angles)
    float coeffs[4];                ///< Polynomial coefficients a0..a3
    float rmse_mm;                  ///< RMSE of polynomial fit
    float correction_factor_K;      ///< Best-fit K for R = K * L/tan(model_angle)
    float ackermann_rmse_mm;        ///< RMSE of Ackermann model fit
};

// ==========================================
// EXTERNAL STATE
// ==========================================

extern CalibrationState cal_state;
extern CalResult cal_left;
extern CalResult cal_right;
extern int cal_current_angle_index;
extern int cal_current_angle;

// ==========================================
// PUBLIC FUNCTIONS
// ==========================================

/**
 * @brief Start the calibration sequence
 * Resets all state and begins measuring at the first steering angle.
 */
void calibration_start();

/**
 * @brief Main calibration update function. Call every loop.
 * 
 * Drives the state machine: drives in circles, measures radius,
 * advances through angles, and finally outputs results.
 */
void calibration_update();

/**
 * @brief Stop calibration immediately. Resets state to CAL_IDLE.
 * Safe to call even if calibration is not running.
 */
void calibration_stop();

/**
 * @brief Check if calibration is currently running
 * @return true if calibration state machine is active
 */
bool calibration_is_active();

/**
 * @brief Get the calibrated turn radius for a given servo angle
 * Uses the polynomial fit coefficients (if calibration has been run).
 * 
 * @param servo_angle Servo offset from center (e.g. 25, -30)
 * @return Turn radius in mm, or -1 if no calibration data available
 */
float get_calibrated_radius(int servo_angle);

/**
 * @brief Print calibration results to serial
 */
void calibration_print_results();

/**
 * @brief Print calibration data in CSV format (for USB log)
 */
void calibration_print_csv();

/**
 * @brief Set calibration coefficients manually (from config.h)
 * @param left_coeffs Array of 4 coefficients for left turns
 * @param right_coeffs Array of 4 coefficients for right turns
 */
void calibration_set_coefficients(const float left_coeffs[4], const float right_coeffs[4]);

/**
 * @brief Check if calibration coefficients have been set
 * @return true if coefficients are valid (non-zero)
 */
bool calibration_has_data();