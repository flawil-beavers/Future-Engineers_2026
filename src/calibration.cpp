/**
 * @file calibration.cpp
 * @brief Automatic turn radius calibration implementation
 * 
 * Calibration state machine:
 *   CAL_IDLE → CAL_DRIVING → CAL_STOPPING → (loop back) → CAL_DONE
 * 
 * For each steering angle, the robot drives in a circle until the gyro
 * reports 360° of rotation. The encoder distance is then used to compute
 * the actual turn radius: R = distance / (2*π).
 * 
 * After all angles are measured, the results are fitted to both a
 * 3rd-degree polynomial and the theoretical Ackermann model.
 */

#include "calibration.h"
#include "config.h"
#include "motor_control.h"
#include "sensors.h"
#include "logger.h"
#define Serial robot_logger

// ==========================================
// EXTERNAL DEPENDENCIES
// ==========================================

extern float current_distance;
extern unsigned long current_time;
extern float last_loop_time;
extern int set_degree;

// ==========================================
// CALIBRATION STATE
// ==========================================

CalibrationState cal_state = CAL_IDLE;
CalResult cal_left;
CalResult cal_right;
int cal_current_angle_index = 0;
int cal_current_angle = 0;

// ==========================================
// INTERNAL STATE
// ==========================================

// The sequence of servo angles to test (absolute values, both directions)
static const int cal_angle_sequence[] = {10, 15, 20, 25, 30, 35, 40};
static const int cal_num_angles = sizeof(cal_angle_sequence) / sizeof(cal_angle_sequence[0]);

// Per-measurement tracking
static float cal_start_distance = 0;      // Encoder distance at start of current circle
static float cal_start_angle = 0;         // Gyro angle at start of current circle
static bool cal_is_right_turn = false;    // Currently measuring right (negative) angles?
static int cal_phase = 0;                 // 0=left turns, 1=right turns
static float cal_drive_start_time = 0;    // Time when we started driving (ms)
static bool cal_printed_angle_header = false;

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
static void fit_polynomial_3rd(const float* x, const float* y, int n, float* coeffs)
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
static float compute_rmse(const float* x, const float* y, int n, const float* coeffs)
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
static float fit_ackermann_k(const float* x, const float* y, int n, float L)
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
static float compute_ackermann_rmse(const float* x, const float* y, int n, float L, float K)
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

// ==========================================
// HELPER: Process a complete measurement
// ==========================================

/**
 * @brief Finalize the current measurement and store it
 */
static void finalize_measurement()
{
    float distance_delta = current_distance - cal_start_distance;
    float angle_delta = fabs(get_angle() - cal_start_angle);
    
    if (angle_delta < 1.0f) {
        Serial.println("ERROR: No significant rotation detected, skipping");
        return;
    }
    
    // Radius = arc_length / angle (in radians)
    float angle_rad = angle_delta * PI / 180.0f;
    float radius_mm = distance_delta / angle_rad;
    
    CalResult* result = cal_is_right_turn ? &cal_right : &cal_left;
    int idx = result->num_points;
    
    if (idx < 12) {
        result->points[idx].servo_angle = cal_current_angle;
        result->points[idx].radius_mm = radius_mm;
        result->points[idx].distance_mm = distance_delta;
        result->num_points++;
        
        Serial.print("MEASURED: angle=");
        Serial.print(cal_current_angle);
        Serial.print(", distance=");
        Serial.print(distance_delta, 1);
        Serial.print(" mm, rotation=");
        Serial.print(angle_delta, 1);
        Serial.print("°, radius=");
        Serial.print(radius_mm, 1);
        Serial.println(" mm");
    }
}

/**
 * @brief Compute fits for a CalResult
 */
static void compute_fits(CalResult* result, bool is_left)
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

void calibration_start()
{
    // Reset state
    cal_state = CAL_DRIVING;
    cal_current_angle_index = 0;
    cal_phase = 0;
    cal_left.num_points = 0;
    cal_right.num_points = 0;
    cal_is_right_turn = false;
    cal_printed_angle_header = false;
    
    // Clear calibration results
    for (int i = 0; i < 4; i++) {
        cal_left.coeffs[i] = 0;
        cal_right.coeffs[i] = 0;
    }
    cal_left.rmse_mm = 0;
    cal_right.rmse_mm = 0;
    cal_left.correction_factor_K = 0;
    cal_right.correction_factor_K = 0;
    cal_left.ackermann_rmse_mm = 0;
    cal_right.ackermann_rmse_mm = 0;
    
    // Get first angle
    cal_current_angle = cal_angle_sequence[0];
    
    Serial.println("\n\n========================================");
    Serial.println("CALIBRATION STARTED");
    Serial.println("========================================");
    Serial.print("Wheelbase: 100 mm\n");
    Serial.print("Calibration speed: ");
    Serial.print(CAL_SPEED_MMS);
    Serial.println(" mm/s");
    Serial.println("Testing angles: 10, 15, 20, 25, 30, 35, 40 (both directions)");
    Serial.println();
    
    // Set steering and speed for first angle
    // Note: system_enable() is called by the mode manager before this
    set_steering(cal_current_angle);
    set_speed(CAL_SPEED_MMS);
    
    // Record initial state
    cal_start_distance = current_distance;
    cal_start_angle = get_angle();
    cal_drive_start_time = millis();
    
    Serial.print("Starting turn: angle=");
    Serial.print(cal_current_angle);
    Serial.println(" (left)");
}

void calibration_update()
{
    if (cal_state == CAL_IDLE || cal_state == CAL_DONE) {
        return;
    }
    
    if (cal_state == CAL_DRIVING) {
        // Check if we've completed 360° of rotation
        float angle_delta = fabs(get_angle() - cal_start_angle);
        
        // Need at least 355° to account for gyro noise
        if (angle_delta >= 355.0f) {
            // Reached 360°! Record measurement
            finalize_measurement();
            
            // Stop motors
            stop(false);
            cal_state = CAL_STOPPING;
            cal_drive_start_time = millis();
            
            Serial.println("Circle complete, stopping...");
        } else {
            // Safety timeout: if we've been driving too long without reaching 360°
            float elapsed = (millis() - cal_drive_start_time) / 1000.0f;
            if (elapsed > 30.0f) {
                Serial.println("TIMEOUT: No 360° rotation detected, aborting this angle");
                stop(false);
                cal_state = CAL_STOPPING;
                cal_drive_start_time = millis();
            }
        }
    }
    else if (cal_state == CAL_STOPPING) {
        // Wait 1 second for robot to settle
        if ((millis() - cal_drive_start_time) > 1000) {
            cal_state = CAL_NEXT_ANGLE;
            cal_drive_start_time = millis();
        }
    }
    else if (cal_state == CAL_NEXT_ANGLE) {
        // Advance to next angle
        cal_current_angle_index++;
        
        // Check if we finished all angles in current phase
        if (cal_current_angle_index >= cal_num_angles) {
            if (cal_phase == 0) {
                // Finished left turns, switch to right turns
                cal_phase = 1;
                cal_current_angle_index = 0;
                cal_is_right_turn = true;
                Serial.println("\n--- Switching to RIGHT turns ---\n");
            } else {
                // All done
                cal_state = CAL_DONE;
                Serial.println("\nAll angles measured!");
                calibration_print_results();
                return;
            }
        }
        
        // Set next angle
        int abs_angle = cal_angle_sequence[cal_current_angle_index];
        cal_current_angle = cal_is_right_turn ? -abs_angle : abs_angle;
        
        Serial.print("\nNext angle: ");
        Serial.print(cal_current_angle);
        Serial.println("°");
        
        // Start driving
        set_steering(cal_current_angle);
        set_speed(CAL_SPEED_MMS);
        
        cal_start_distance = current_distance;
        cal_start_angle = get_angle();
        cal_drive_start_time = millis();
        cal_state = CAL_DRIVING;
    }
}

void calibration_stop()
{
    cal_state = CAL_IDLE;
    stop(false);
    Serial.println("Calibration stopped.");
}

bool calibration_is_active()
{
    return cal_state != CAL_IDLE && cal_state != CAL_DONE;
}

float get_calibrated_radius(int servo_angle)
{
    if (servo_angle == 0) return -1;
    
    bool is_right = (servo_angle < 0);
    float abs_angle = fabs((float)servo_angle);
    const float* coeffs = is_right ? cal_right.coeffs : cal_left.coeffs;
    
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
        CalResult* result = (dir == 0) ? &cal_left : &cal_right;
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
    Serial.print(cal_left.rmse_mm, 2);
    Serial.print(" mm | ");
    Serial.print(cal_right.rmse_mm, 2);
    Serial.println(" mm");
    Serial.print("Ackermann (K*L/tan(δ)) | ");
    Serial.print(cal_left.ackermann_rmse_mm, 2);
    Serial.print(" mm | ");
    Serial.print(cal_right.ackermann_rmse_mm, 2);
    Serial.println(" mm");
    Serial.println();
    
    // Print config.h-ready constants
    Serial.println("--- COPY THESE INTO config.h ---");
    Serial.print("#define CAL_LEFT_A0 "); Serial.println(cal_left.coeffs[0], 4);
    Serial.print("#define CAL_LEFT_A1 "); Serial.println(cal_left.coeffs[1], 6);
    Serial.print("#define CAL_LEFT_A2 "); Serial.println(cal_left.coeffs[2], 8);
    Serial.print("#define CAL_LEFT_A3 "); Serial.println(cal_left.coeffs[3], 8);
    Serial.print("#define CAL_RIGHT_A0 "); Serial.println(cal_right.coeffs[0], 4);
    Serial.print("#define CAL_RIGHT_A1 "); Serial.println(cal_right.coeffs[1], 6);
    Serial.print("#define CAL_RIGHT_A2 "); Serial.println(cal_right.coeffs[2], 8);
    Serial.print("#define CAL_RIGHT_A3 "); Serial.println(cal_right.coeffs[3], 8);
    Serial.print("#define CAL_LEFT_K "); Serial.println(cal_left.correction_factor_K, 4);
    Serial.print("#define CAL_RIGHT_K "); Serial.println(cal_right.correction_factor_K, 4);
    Serial.println();
    Serial.println("========================================\n");
    
    // Flush to USB log
    robot_logger.write_to_usb();
}

void calibration_print_csv()
{
    Serial.println("\n--- CALIBRATION CSV ---");
    Serial.println("Direction,ServoCmd,Radius_mm,Distance_mm");
    
    for (int i = 0; i < cal_left.num_points; i++) {
        Serial.print("LEFT,");
        Serial.print(cal_left.points[i].servo_angle);
        Serial.print(",");
        Serial.print(cal_left.points[i].radius_mm, 1);
        Serial.print(",");
        Serial.println(cal_left.points[i].distance_mm, 1);
    }
    for (int i = 0; i < cal_right.num_points; i++) {
        Serial.print("RIGHT,");
        Serial.print(cal_right.points[i].servo_angle);
        Serial.print(",");
        Serial.print(cal_right.points[i].radius_mm, 1);
        Serial.print(",");
        Serial.println(cal_right.points[i].distance_mm, 1);
    }
    
    Serial.println("--- END CSV ---");
}

void calibration_set_coefficients(const float left_coeffs[4], const float right_coeffs[4])
{
    for (int i = 0; i < 4; i++) {
        cal_left.coeffs[i] = left_coeffs[i];
        cal_right.coeffs[i] = right_coeffs[i];
    }
    Serial.println("Calibration coefficients loaded from config");
}

bool calibration_has_data()
{
    for (int i = 0; i < 4; i++) {
        if (fabs(cal_left.coeffs[i]) > 1e-6f && fabs(cal_right.coeffs[i]) > 1e-6f) {
            return true;
        }
    }
    return false;
}