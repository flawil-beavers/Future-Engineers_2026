#pragma once

/**
 * @file calibration.h
 * @brief Shared calibration data types and utility functions
 * 
 * Provides the data structures and utility functions used by both
 * turn radius calibration and servo-center calibration, including:
 *   - Measurement point and result data structures
 *   - Polynomial fitting (3rd-degree least squares)
 *   - Ackermann model fitting
 *   - Calibrated radius lookup from polynomial coefficients
 *   - Result printing and CSV export
 * 
 * The global coefficient storage (tr_cal_left, tr_cal_right) is
 * populated either by running turn radius calibration at runtime
 * or by loading pre-computed coefficients from config.h.
 */

#include <Arduino.h>

// ==========================================
// CALIBRATION DATA STRUCTURES
// ==========================================

/**
 * @brief A single turn radius calibration measurement point
 */
struct TRCalPoint {
    int servo_angle;        ///< Servo offset from center (e.g. 25, -30)
    float radius_mm;        ///< Measured turn radius
    float distance_mm;      ///< Total distance traveled for 360°
};

/**
 * @brief Complete turn radius calibration results for one direction
 */
struct TRCalResult {
    int num_points;                 ///< Number of valid measurements
    TRCalPoint points[12];          ///< Measurement data (max 12 angles)
    float coeffs[4];                ///< Polynomial coefficients a0..a3
    float rmse_mm;                  ///< RMSE of polynomial fit
    float correction_factor_K;      ///< Best-fit K for R = K * L/tan(model_angle)
    float ackermann_rmse_mm;        ///< RMSE of Ackermann model fit
};

// ==========================================
// EXTERNAL STATE (global coefficient storage)
// ==========================================

extern TRCalResult tr_cal_left;
extern TRCalResult tr_cal_right;

// ==========================================
// SHARED UTILITY FUNCTIONS
// ==========================================

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
 * Uses the global tr_cal_left / tr_cal_right coefficient storage.
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

// ==========================================
// POLYNOMIAL FITTING (used by turn radius calibration)
// ==========================================

/**
 * @brief Fit a 3rd-degree polynomial y = a0 + a1*x + a2*x² + a3*x³
 * using the normal equations (least squares).
 */
void fit_polynomial_3rd(const float* x, const float* y, int n, float* coeffs);

/**
 * @brief Compute the RMSE of a polynomial fit
 */
float compute_rmse(const float* x, const float* y, int n, const float* coeffs);

/**
 * @brief Fit the Ackermann model: R = K * L / tan(δ_model)
 * Returns the best-fit correction factor K.
 */
float fit_ackermann_k(const float* x, const float* y, int n, float L);

/**
 * @brief Compute RMSE for the Ackermann model with given K
 */
float compute_ackermann_rmse(const float* x, const float* y, int n, float L, float K);

/**
 * @brief Compute both polynomial and Ackermann fits for a TRCalResult
 */
void compute_fits(TRCalResult* result, bool is_left);