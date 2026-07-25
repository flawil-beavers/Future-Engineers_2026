/**
 * @file position_estimator.cpp
 * @brief Position Estimator (Dead Reckoning Odometry) Implementation
 * 
 * Algorithm:
 *   Every loop, the change in encoder distance (Δs) is combined with the
 *   change in gyro heading (Δθ) to update the (x, y) position:
 * 
 *     avg_heading = prev_heading + Δθ / 2
 *     x += Δs * cos(avg_heading)
 *     y += Δs * sin(avg_heading)
 *     heading += Δθ
 * 
 *   This is standard differential odometry (bicycle model), using the
 *   gyro for heading instead of steering angle, which is more accurate.
 * 
 *   During turns, if calibration data is available, the expected arc
 *   radius is used to detect slip (when encoder distance deviates
 *   significantly from the expected arc length for the measured rotation).
 */

#include "position_estimator.h"
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
extern float last_loop_time;
extern float current_speed;
extern int set_degree;

extern unsigned long current_time;

// ==========================================
// POSITION STATE
// ==========================================

static PositionEstimate pos;
static float prev_distance = 0;         // Previous encoder distance (mm)
static float prev_angle = 0;            // Previous gyro angle (degrees)
static float prev_heading_rad = 0;      // Previous heading in radians
static float total_distance_traveled = 0;
static bool pos_initialized = false;

// Slip detection
static float slip_log_timer = 0;        // Timer for throttle

// ==========================================
// INTERNAL HELPERS
// ==========================================

/**
 * @brief Normalize an angle to [-180, 180) degrees
 */
static float wrap_to_180(float angle)
{
    while (angle > 180.0f) angle -= 360.0f;
    while (angle <= -180.0f) angle += 360.0f;
    return angle;
}

/**
 * @brief Normalize an angle to [0, 360) degrees
 */
static float wrap_to_360(float angle)
{
    while (angle < 0) angle += 360.0f;
    while (angle >= 360.0f) angle -= 360.0f;
    return angle;
}

// ==========================================
// PUBLIC FUNCTIONS
// ==========================================

void position_init(float x, float y, float heading)
{
    pos.x_mm = x;
    pos.y_mm = y;
    pos.heading_deg = heading;
    pos.confidence_mm = 0;
    
    prev_distance = current_distance;
    prev_angle = get_angle();
    prev_heading_rad = heading * PI / 180.0f;
    total_distance_traveled = 0;
    pos_initialized = true;
    
    Serial.print("Position initialized: (");
    Serial.print(x);
    Serial.print(", ");
    Serial.print(y);
    Serial.print("), heading=");
    Serial.print(heading);
    Serial.println("°");
}

void update_position()
{
    if (!pos_initialized) {
        // Auto-initialize on first call
        position_init(0, 0, 0);
        return;
    }
    
    // Get current sensor readings
    float current_angle = get_angle();
    float delta_dist = current_distance - prev_distance;
    float delta_angle = current_angle - prev_angle;
    
    // Handle angle wrap-around: normalize to [-180, 180]
    delta_angle = wrap_to_180(delta_angle);
    
    if (fabs(delta_dist) < 0.01f && fabs(delta_angle) < 0.01f) {
        // No significant movement, skip to save CPU
        prev_distance = current_distance;
        prev_angle = current_angle;
        return;
    }
    
    // Convert to radians for trig
    float delta_angle_rad = delta_angle * PI / 180.0f;
    float current_heading_rad = prev_heading_rad + delta_angle_rad;
    
    // Average heading over this short arc
    float avg_heading_rad = prev_heading_rad + delta_angle_rad / 2.0f;
    
    // Integrate position (dead reckoning)
    pos.x_mm += delta_dist * cosf(avg_heading_rad);
    pos.y_mm += delta_dist * sinf(avg_heading_rad);
    
    // Update heading (wrap to 0-360 for display)
    pos.heading_deg = wrap_to_360(current_heading_rad * 180.0f / PI);
    prev_heading_rad = current_heading_rad;
    
    // Update totals
    total_distance_traveled += fabs(delta_dist);
    
    // Confidence grows with distance traveled (~1%)
    pos.confidence_mm += fabs(delta_dist) * 0.01f;
    
    // ==========================================
    // Calibration-Enhanced Slip Detection
    // ==========================================
    if (calibration_has_data() && fabs(set_degree) > 5 && fabs(delta_angle) > 0.5f) {
        float actual_radius = get_calibrated_radius(set_degree);
        if (actual_radius > 0 && last_loop_time > 1e-6f) {
            // Expected arc length for this rotation: s = R * Δθ
            float expected_arc = actual_radius * fabs(delta_angle_rad);
            float actual_arc = fabs(delta_dist);
            
            // If actual distance deviates > 20% from expected, log it
            if (expected_arc > 1.0f) {
                float deviation = (actual_arc - expected_arc) / expected_arc;
                if (fabs(deviation) > 0.20f) {
                    // Throttle slip logging to once per second
                    if (current_time - slip_log_timer > 1000000) {
                        slip_log_timer = current_time;
                        Serial.print("[SLIP] deviation=");
                        Serial.print(deviation * 100, 1);
                        Serial.print("%, angle=");
                        Serial.print(set_degree);
                        Serial.print(", expected_arc=");
                        Serial.print(expected_arc, 1);
                        Serial.print(", actual_arc=");
                        Serial.print(actual_arc, 1);
                        Serial.print(", radius=");
                        Serial.print(actual_radius, 1);
                        Serial.println(" mm");
                        
                        // Increase confidence on slip event
                        pos.confidence_mm += fabs(delta_dist) * 0.05f;
                    }
                }
            }
        }
    }
    
    // Store for next iteration
    prev_distance = current_distance;
    prev_angle = current_angle;
}

void get_position(float &x, float &y, float &heading)
{
    x = pos.x_mm;
    y = pos.y_mm;
    heading = pos.heading_deg;
}

PositionEstimate get_position_struct()
{
    return pos;
}

float get_position_confidence()
{
    return pos.confidence_mm;
}

float get_total_distance_traveled()
{
    return total_distance_traveled;
}

void position_reset(float x, float y, float heading)
{
    // When resetting, we keep the stored prev_distance/prev_angle
    // to prevent a position jump, but reset the absolute position
    position_init(x, y, heading);
}

void position_print()
{
    Serial.print("POS: x=");
    Serial.print(pos.x_mm, 1);
    Serial.print(" y=");
    Serial.print(pos.y_mm, 1);
    Serial.print(" h=");
    Serial.print(pos.heading_deg, 1);
    Serial.print(" c=");
    Serial.print(pos.confidence_mm, 1);
    Serial.print(" dist=");
    Serial.print(total_distance_traveled, 1);
    Serial.println();
}