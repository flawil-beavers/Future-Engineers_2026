/**
 * @file calibration.cpp
 * @brief Shared calibration utility functions
 * 
 * Provides polynomial fitting (3rd-degree least squares), Ackermann model
 * fitting, calibrated radius lookup, result printing, and coefficient
 * storage that are shared between the turn radius calibration and
 * servo-center calibration subsystems.
 * 
 * The global TRCalResult variables (tr_cal_left, tr_cal_right) store
 * the polynomial coefficients either from runtime calibration or from
 * pre-computed values loaded from config.h.
 */

#include "calibration.h"
#include "config.h"
#include "motor_control.h"
#include "logger.h"
#define Serial robot_logger

// ==========================================
// EXTERNAL DEPENDENCIES
// ==========================================

extern float current_distance;
extern unsigned long current_time;
extern float last_loop_time;
extern int set_degree;
extern bool servo_disabled;

// ==========================================
// GLOBAL COEFFICIENT STORAGE
// ==========================================

// Note: The actual values for tr_cal_left and tr_cal_right are defined
// in turn_radius_calibration.cpp, but extern'd in calibration.h so that
// shared functions (calibration_print_results, get_calibrated_radius, etc.)
// can access them without depending on the turn radius calibration module.
// This is intentional to allow coefficient lookup without needing the
// calibration state machine to be active.

// ==========================================
// POLYNOMIAL FITTING (Least Squares)
// ==========================================

/**
 * @brief Fit a 3rd-degree polynomial y = a0 + a1*x + a2*x² + a3*x³
 * using the normal equations (least squares).
 * 
 * @param x Array of x values (independent variable)
 * @param y Array of y values (dependent variable)
 * @param n Number of data points
 * @param coeffs Output array of 4 coefficients [a0, a1, a2, a3]
 */
void fit_polynomial_3rd(const float* x, const float* y, int n, float* coeffs)
{
    // Build the Vandermonde-like matrix: X = [1, x, x², x³]
    // Normal equations: (X^T * X) * coeffs = X^T * y
    // We solve a 4x4 system using Gaussian elimination.
    
    float A[4][4] = {0};
    float b[4] = {0};
    
    for (int i = 0; i < n; i++) {
        float xi = x[i];
        float yi = y[i];
        float xp[7]; // powers: 0..6
        xp[0] = 1.0f;
        for (int p = 1; p < 7; p++) {
            xp[p] = xp[p-1] * xi;
        }
        
        // Accumulate A = X^T * X
        for (int r = 0; r < 4; r++) {
            for (int c = 0; c < 4; c++) {
                A[r][c] += xp[r] * xp[c];
            }
            b[r] += xp[r] * yi;
        }
    }
    
    // Gaussian elimination with partial pivoting
    for (int col = 0; col < 4; col++) {
        // Find pivot
        int pivot = col;
        float max_val = fabs(A[col][col]);
        for (int row = col + 1; row < 4; row++) {
            if (fabs(A[row][col]) > max_val) {
                max_val = fabs(A[row][col]);
                pivot = row;
            }
        }
        
        // Swap rows
        if (pivot != col) {
            for (int c = 0; c < 4; c++) {
                float temp = A[col][c];
                A[col][c] = A[pivot][c];
                A[pivot][c] = temp;
            }
            float temp = b[col];
            b[col] = b[pivot];
            b[pivot] = temp;
        }
        
        // Eliminate below
        for (int row = col + 1; row < 4; row++) {
            float factor = A[row][col] / A[col][col];
            for (int c = col; c < 4; c++) {
                A[row][c] -= factor * A[col][c];
            }
            b[row] -= factor * b[col];
        }
    }
    
    // Back substitution
    for (int row = 3; row >= 0; row--) {
        float sum = b[row];
        for (int c = row + 1; c < 4; c++) {
            sum -= A[row][c] * coeffs[c];
        }
        if (fabs(A[row][row]) > 1e-10f) {
            coeffs[row] = sum / A[row][row];
        } else {
            coeffs[row] = 0;
        }
    }
}

/**
 * @brief Compute the RMSE of a polynomial fit
 */
float compute_rmse(const float* x, const float* y, int n, const float* coeffs)
{
    float sum_sq = 0;
    for (int i = 0; i < n; i++) {
        float xi = x[i];
        float predicted = coeffs[0] + coeffs[1]*xi + coeffs[2]*xi*xi + coeffs[3]*xi*xi*xi;
        float error = predicted - y[i];
        sum_sq += error * error;
    }
    return sqrtf(sum_sq / n);
}

/**
 * @brief Fit the Ackermann model: R = K * L / tan(δ_model)
 * where δ_model is derived from servo angle.
 * 
 * We find the best K such that sum of squared errors is minimized.
 * Since the model is linear in K, K = Σ(R_i * L/tan(δ_i)) / Σ(L/tan(δ_i))²
 * 
 * Note: The servo angle does NOT equal the wheel angle. The "model_angle"
 * is unknown, so we treat K*L as a lumped empirical parameter.
 * Actually, for this robot we simply compute the best-fit K as:
 *   K = mean(R_i * tan(δ_i) / L)
 * where δ_i is the servo angle converted to radians (treating servo angle
 * as proportional to wheel angle).
 */
float fit_ackermann_k(const float* x, const float* y, int n, float L)
{
    // x = servo angles (degrees), y = measured radii (mm)
    // Model: R = K * L / tan(α * x) where α is some unknown scale factor.
    // Since we don't know the actual wheel angle, we treat servo angle
    // as proportional: wheel_angle = α * servo_angle
    // We iteratively find both K and α, or simply report the best linear fit
    // of 1/R vs something.
    // 
    // Simplified approach: For each point, compute what K would be if
    // wheel_angle = servo_angle (even though it's not).
    // K_i = R_i * tan(servo_angle_rad) / L
    // Then average K to get the best fit.
    
    float sum_k = 0;
    int valid_points = 0;
    for (int i = 0; i < n; i++) {
        float servo_deg = fabs(x[i]);
        if (servo_deg < 1.0f) continue; // Skip near-zero
        float servo_rad = servo_deg * PI / 180.0f;
        float tan_val = tanf(servo_rad);
        if (tan_val < 0.01f) continue;
        sum_k += y[i] * tan_val / L;
        valid_points++;
    }
    
    if (valid_points == 0) return 0;
    return sum_k / valid_points;
}

/**
 * @brief Compute RMSE for the Ackermann model with given K
 */
float compute_ackermann_rmse(const float* x, const float* y, int n, float L, float K)
{
    float sum_sq = 0;
    for (int i = 0; i < n; i++) {
        float servo_deg = fabs(x[i]);
        if (servo_deg < 1.0f) continue;
        float servo_rad = servo_deg * PI / 180.0f;
        float tan_val = tanf(servo_rad);
        if (tan_val < 0.01f) continue;
        float predicted = K * L / tan_val;
        float error = predicted - y[i];
        sum_sq += error * error;
    }
    return sqrtf(sum_sq / n);
}

/**
 * @brief Compute fits for a TRCalResult
 */
void compute_fits(TRCalResult* result, bool is_left)
{
    if (result->num_points < 4) {
        Serial.print("WARNING: Too few points (");
        Serial.print(result->num_points);
        Serial.println(") for reliable fit");
        return;
    }
    
    float x[12], y[12];
    for (int i = 0; i < result->num_points; i++) {
        x[i] = (float)result->points[i].servo_angle;
        if (!is_left) x[i] = fabs(x[i]); // Use absolute value for right turns
        y[i] = result->points[i].radius_mm;
    }
    
    // Polynomial fit
    fit_polynomial_3rd(x, y, result->num_points, result->coeffs);
    result->rmse_mm = compute_rmse(x, y, result->num_points, result->coeffs);
    
    // Ackermann fit
    float L = 100.0f; // Wheelbase = 100 mm
    result->correction_factor_K = fit_ackermann_k(x, y, result->num_points, L);
    result->ackermann_rmse_mm = compute_ackermann_rmse(x, y, result->num_points, L, result->correction_factor_K);
}

// ==========================================
// PUBLIC FUNCTIONS
// ==========================================

float get_calibrated_radius(int servo_angle)
{
    if (servo_angle == 0) return -1;
    
    bool is_right = (servo_angle < 0);
    float abs_angle = fabs((float)servo_angle);
    const float* coeffs = is_right ? tr_cal_right.coeffs : tr_cal_left.coeffs;
    
    // Check if coefficients are valid (non-zero)
    bool has_data = false;
    for (int i = 0; i < 4; i++) {
        if (fabs(coeffs[i]) > 1e-6f) {
            has_data = true;
            break;
        }
    }
    
    if (!has_data) return -1;
    
    float radius = coeffs[0] + coeffs[1]*abs_angle + coeffs[2]*abs_angle*abs_angle + coeffs[3]*abs_angle*abs_angle*abs_angle;
    return max(radius, 50.0f); // Clamp to prevent unrealistic values
}

void calibration_print_results()
{
    Serial.println("\n\n========================================");
    Serial.println("       CALIBRATION COMPLETE");
    Serial.println("========================================\n");
    
    // Print polynomial coefficients and RMSE for both directions
    for (int dir = 0; dir < 2; dir++) {
        TRCalResult* result = (dir == 0) ? &tr_cal_left : &tr_cal_right;
        const char* dir_name = (dir == 0) ? "LEFT" : "RIGHT";
        
        Serial.print("--- ");
        Serial.print(dir_name);
        Serial.println(" TURNS ---");
        Serial.println("ServoCmd, Radius_mm, Distance_mm");
        for (int i = 0; i < result->num_points; i++) {
            Serial.print(result->points[i].servo_angle);
            Serial.print(", ");
            Serial.print(result->points[i].radius_mm, 1);
            Serial.print(", ");
            Serial.println(result->points[i].distance_mm, 1);
        }
        Serial.println();
        
        // Compute fits
        compute_fits(result, (dir == 0));
        
        Serial.print("Polynomial Fit: R(δ) = ");
        Serial.print(result->coeffs[0], 4);
        Serial.print(" + ");
        Serial.print(result->coeffs[1], 4);
        Serial.print("*δ + ");
        Serial.print(result->coeffs[2], 6);
        Serial.print("*δ² + ");
        Serial.print(result->coeffs[3], 8);
        Serial.println("*δ³");
        Serial.print("  RMSE: ");
        Serial.print(result->rmse_mm, 2);
        Serial.println(" mm");
        Serial.println();
        
        Serial.print("Ackermann Correction Factor K: ");
        Serial.println(result->correction_factor_K, 4);
        Serial.print("  Ackermann RMSE: ");
        Serial.print(result->ackermann_rmse_mm, 2);
        Serial.println(" mm");
        Serial.print("  Theoretical: R = K * L / tan(δ)  where L=100mm, δ=servo_angle");
        Serial.println();
        Serial.println();
    }
    
    // Print comparison of both methods
    Serial.println("--- METHOD COMPARISON ---");
    Serial.print("Method | Left RMSE | Right RMSE\n");
    Serial.print("Polynomial (3rd deg) | ");
    Serial.print(tr_cal_left.rmse_mm, 2);
    Serial.print(" mm | ");
    Serial.print(tr_cal_right.rmse_mm, 2);
    Serial.println(" mm");
    Serial.print("Ackermann (K*L/tan(δ)) | ");
    Serial.print(tr_cal_left.ackermann_rmse_mm, 2);
    Serial.print(" mm | ");
    Serial.print(tr_cal_right.ackermann_rmse_mm, 2);
    Serial.println(" mm\n");

    // Evaluate error quality
    float max_poly_rmse = max(tr_cal_left.rmse_mm, tr_cal_right.rmse_mm);
    Serial.println("--- CALIBRATION ACCURACY EVALUATION ---");
    Serial.print("Overall Status: ");
    if (max_poly_rmse < 15.0f) {
        Serial.println("EXCELLENT (RMSE < 15 mm)");
        Serial.println("Description: High precision fit across all steering angles (5° to 50°). Vehicle dynamics match modeled trajectory closely.");
    } else if (max_poly_rmse < 35.0f) {
        Serial.println("GOOD / ACCEPTABLE (RMSE 15 - 35 mm)");
        Serial.println("Description: Minor mechanical slop or surface slippage detected, but well within tolerances for accurate navigation.");
    } else if (max_poly_rmse < 60.0f) {
        Serial.println("MODERATE ERROR (RMSE 35 - 60 mm)");
        Serial.println("Description: Noticeable deviation. Check tire traction, battery voltage, or steering linkage friction.");
    } else {
        Serial.println("POOR / CALIBRATION BAD (RMSE > 60 mm)");
        Serial.println("Description: Significant error detected. Verify SERVO_CENTER zeroing, wheel alignment, or recalibrate on a non-slip surface.");
    }
    Serial.println();
    
    // Print config.h-ready constants
    Serial.println("--- COPY AND PASTE DIRECTLY INTO config.h ---");
    Serial.print("constexpr auto CAL_LEFT_A0 = "); Serial.print(tr_cal_left.coeffs[0], 4); Serial.println("f;");
    Serial.print("constexpr auto CAL_LEFT_A1 = "); Serial.println(tr_cal_left.coeffs[1], 6); Serial.println("f;");
    Serial.print("constexpr auto CAL_LEFT_A2 = "); Serial.print(tr_cal_left.coeffs[2], 8); Serial.println("f;");
    Serial.print("constexpr auto CAL_LEFT_A3 = "); Serial.print(tr_cal_left.coeffs[3], 8); Serial.println("f;");
    Serial.print("constexpr auto CAL_RIGHT_A0 = "); Serial.print(tr_cal_right.coeffs[0], 4); Serial.println("f;");
    Serial.print("constexpr auto CAL_RIGHT_A1 = "); Serial.print(tr_cal_right.coeffs[1], 6); Serial.println("f;");
    Serial.print("constexpr auto CAL_RIGHT_A2 = "); Serial.print(tr_cal_right.coeffs[2], 8); Serial.println("f;");
    Serial.print("constexpr auto CAL_RIGHT_A3 = "); Serial.print(tr_cal_right.coeffs[3], 8); Serial.println("f;");
    Serial.print("constexpr auto CAL_LEFT_K = "); Serial.print(tr_cal_left.correction_factor_K, 4); Serial.println("f;");
    Serial.print("constexpr auto CAL_RIGHT_K = "); Serial.print(tr_cal_right.correction_factor_K, 4); Serial.println("f;");
    Serial.println();
    Serial.println("========================================\n");
    
    // Flush to USB log
    robot_logger.write_to_usb();
}

void calibration_print_csv()
{
    Serial.println("\n--- CALIBRATION CSV ---");
    Serial.println("Direction,ServoCmd,Radius_mm,Distance_mm");
    
    for (int i = 0; i < tr_cal_left.num_points; i++) {
        Serial.print("LEFT,");
        Serial.print(tr_cal_left.points[i].servo_angle);
        Serial.print(",");
        Serial.print(tr_cal_left.points[i].radius_mm, 1);
        Serial.print(",");
        Serial.println(tr_cal_left.points[i].distance_mm, 1);
    }
    for (int i = 0; i < tr_cal_right.num_points; i++) {
        Serial.print("RIGHT,");
        Serial.print(tr_cal_right.points[i].servo_angle);
        Serial.print(",");
        Serial.print(tr_cal_right.points[i].radius_mm, 1);
        Serial.print(",");
        Serial.println(tr_cal_right.points[i].distance_mm, 1);
    }
    
    Serial.println("--- END CSV ---");
}

void calibration_set_coefficients(const float left_coeffs[4], const float right_coeffs[4])
{
    for (int i = 0; i < 4; i++) {
        tr_cal_left.coeffs[i] = left_coeffs[i];
        tr_cal_right.coeffs[i] = right_coeffs[i];
    }
    Serial.println("Calibration coefficients loaded from config");
}

bool calibration_has_data()
{
    for (int i = 0; i < 4; i++) {
        if (fabs(tr_cal_left.coeffs[i]) > 1e-6f && fabs(tr_cal_right.coeffs[i]) > 1e-6f) {
            return true;
        }
    }
    return false;
}