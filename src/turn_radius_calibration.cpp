/**
 * @file turn_radius_calibration.cpp
 * @brief Automatic turn radius calibration implementation
 * 
 * Calibration state machine:
 *   TR_IDLE → TR_DRIVING → TR_STOPPING → (loop back) → TR_DONE
 * 
 * For each steering angle, the robot drives in a circle until the gyro
 * reports 360° of rotation. The encoder distance is then used to compute
 * the actual turn radius: R = distance / (2*π).
 * 
 * After all angles are measured, the results are fitted to both a
 * 3rd-degree polynomial and the theoretical Ackermann model.
 * 
 * The calibration runs in two phases: left turns first, then right turns.
 * After left turns complete, the robot pauses and waits for the user to
 * toggle the enable switch (OFF then ON) or send 'c' via serial to start
 * the right turns. This allows the user to reposition the robot between
 * sides if needed.
 */

#include "turn_radius_calibration.h"
#include "calibration.h"
#include "config.h"
#include "motor_control.h"
#include "sensors.h"
#include "navigation_controller.h"
#include "logger.h"
#define Serial robot_logger

// ==========================================
// EXTERNAL DEPENDENCIES
// ==========================================

extern float current_distance;
extern float last_loop_time;
extern bool servo_disabled;

// ==========================================
// TURN RADIUS CALIBRATION STATE
// ==========================================

TurnRadiusState turn_radius_state = TR_IDLE;
TRCalResult tr_cal_left;
TRCalResult tr_cal_right;
int tr_cal_current_angle_index = 0;
int tr_cal_current_angle = 0;

// ==========================================
// INTERNAL STATE
// ==========================================

// The sequence of servo angles to test (5° to MAX_STEERING=50°)
static const int tr_cal_angle_sequence[] = {5, 10, 15, 20, 25, 30, 35, 40, 45, 50};
static const int tr_cal_num_angles = sizeof(tr_cal_angle_sequence) / sizeof(tr_cal_angle_sequence[0]);

// Per-measurement tracking
static float tr_cal_start_distance = 0;      // Encoder distance at start of current circle
static float tr_cal_start_angle = 0;         // Gyro angle at start of current circle
static bool tr_cal_is_right_turn = false;    // Currently measuring right (positive) angles?
static int tr_cal_phase = 0;                 // 0=left turns, 1=right turns
static float tr_cal_drive_start_time = 0;    // Time when we started driving (ms)
static const unsigned long tr_cal_circle_timeout_ms = 30000UL; // Max time allowed for one measurement
static bool tr_cal_printed_angle_header = false;

// Return the required rotation angle (deg) for a given servo angle to fit within a 2x2m box.
// Diameter = 2 * R = 2 * (L / sin(steering_deg)).
// To stay within 2000mm box, for small angles we measure a partial arc (e.g. 90°-180°),
// while for larger angles we can do up to 360°.
static float get_required_turn_angle_deg(int abs_servo_angle)
{
    if (abs_servo_angle <= 5) {
        return 90.0f;  // 90° turn for 5° angle (~1.8m arc) to stay comfortably inside 2x2m box
    } else if (abs_servo_angle <= 10) {
        return 120.0f; // 120° turn for 10° angle
    } else if (abs_servo_angle <= 15) {
        return 180.0f; // 180° turn for 15° angle
    }
    return 360.0f;     // Full 360° turn for 20° and above
}

// ==========================================
// HELPER: Check if left turns are complete
// ==========================================

/**
 * @brief Check if left-turn data is complete and right-turn data is empty,
 * indicating we should resume with right turns.
 * 
 * Only returns true if all left-turn angles were measured (num_points == num_angles),
 * to avoid accidentally skipping to right turns if calibration was interrupted
 * mid-way through left turns.
 */
static bool tr_cal_should_resume_right()
{
    return (tr_cal_left.num_points == tr_cal_num_angles && tr_cal_right.num_points == 0);
}

// ==========================================
// HELPER: Process a complete measurement
// ==========================================

/**
 * @brief Finalize the current measurement and store it
 */
static void finalize_measurement()
{
    float distance_delta = current_distance - tr_cal_start_distance;
    float angle_delta = fabs(get_angle() - tr_cal_start_angle);
    
    if (angle_delta < 1.0f) {
        Serial.println("ERROR: No significant rotation detected, skipping");
        return;
    }
    
    // Radius = arc_length / angle (in radians)
    float angle_rad = angle_delta * PI / 180.0f;
    float radius_mm = distance_delta / angle_rad;
    
    TRCalResult* result = tr_cal_is_right_turn ? &tr_cal_right : &tr_cal_left;
    int idx = result->num_points;
    
    if (idx < 12) {
        result->points[idx].servo_angle = tr_cal_current_angle;
        result->points[idx].radius_mm = radius_mm;
        result->points[idx].distance_mm = distance_delta;
        result->num_points++;
        
        Serial.print("MEASURED: angle=");
        Serial.print(tr_cal_current_angle);
        Serial.print(", distance=");
        Serial.print(distance_delta, 1);
        Serial.print(" mm, rotation=");
        Serial.print(angle_delta, 1);
        Serial.print("°, radius=");
        Serial.print(radius_mm, 1);
        Serial.println(" mm");
    }
}

// ==========================================
// PUBLIC FUNCTIONS
// ==========================================

void turn_radius_cal_start()
{
    // If left-turn data exists but right-turn data is empty, resume with right turns.
    // This handles the case where the user toggles the enable switch OFF then ON
    // after left turns complete, or sends 'c' via serial.
    if (tr_cal_should_resume_right()) {
        turn_radius_state = TR_DRIVING;
        tr_cal_current_angle_index = 0;
        tr_cal_phase = 1;
        tr_cal_is_right_turn = true;
        tr_cal_printed_angle_header = false;
        
        // Clear only right-side calibration results (preserve left data)
        tr_cal_right.num_points = 0;
        for (int i = 0; i < 4; i++) {
            tr_cal_right.coeffs[i] = 0;
        }
        tr_cal_right.rmse_mm = 0;
        tr_cal_right.correction_factor_K = 0;
        tr_cal_right.ackermann_rmse_mm = 0;
        
        int abs_angle = tr_cal_angle_sequence[0];
        tr_cal_current_angle = abs_angle;
        
        Serial.println("\n========================================");
        Serial.println("RIGHT TURN CALIBRATION RESUMED");
        Serial.println("========================================\n");
        
        set_steering(tr_cal_current_angle);
        set_speed(CAL_SPEED_MMS);
        
        tr_cal_start_distance = current_distance;
        tr_cal_start_angle = get_angle();
        tr_cal_drive_start_time = millis();
        
        Serial.print("Starting turn: angle=");
        Serial.print(tr_cal_current_angle);
        Serial.println(" (right / positive = right turn)");
        return;
    }
    
    // Reset state for a fresh full calibration (both sides)
    turn_radius_state = TR_DRIVING;
    tr_cal_current_angle_index = 0;
    tr_cal_phase = 0;
    tr_cal_left.num_points = 0;
    tr_cal_right.num_points = 0;
    tr_cal_is_right_turn = false;
    tr_cal_printed_angle_header = false;
    
    // Clear calibration results
    for (int i = 0; i < 4; i++) {
        tr_cal_left.coeffs[i] = 0;
        tr_cal_right.coeffs[i] = 0;
    }
    tr_cal_left.rmse_mm = 0;
    tr_cal_right.rmse_mm = 0;
    tr_cal_left.correction_factor_K = 0;
    tr_cal_right.correction_factor_K = 0;
    tr_cal_left.ackermann_rmse_mm = 0;
    tr_cal_right.ackermann_rmse_mm = 0;
    
    // Get first angle — negative for left turn (contradicts phase label, but 
    // this is the correct sign: negative steering = left, positive = right)
    tr_cal_current_angle = -tr_cal_angle_sequence[0];
    
    Serial.println("\n\n========================================");
    Serial.println("TURN RADIUS CALIBRATION STARTED");
    Serial.println("========================================");
    Serial.print("Wheelbase: 100 mm\n");
    Serial.print("Calibration speed: ");
    Serial.print(CAL_SPEED_MMS);
    Serial.println(" mm/s");
    Serial.println("Testing angles: 10, 15, 20, 25, 30, 35, 40 (both directions)");
    Serial.println();
    
    // Set steering and speed for first angle
    // Note: system_enable() is called by the mode manager before this
    set_steering(tr_cal_current_angle);
    set_speed(CAL_SPEED_MMS);
    
    // Record initial state
    tr_cal_start_distance = current_distance;
    tr_cal_start_angle = get_angle();
    tr_cal_drive_start_time = millis();
    
    Serial.print("Starting turn: angle=");
    Serial.print(tr_cal_current_angle);
    Serial.println(" (left / negative = left turn)");
}

void turn_radius_cal_update()
{
    if (turn_radius_state == TR_IDLE || turn_radius_state == TR_DONE) {
        return;
    }
    
    if (turn_radius_state == TR_DRIVING) {
        // Check if we've completed the required rotation angle
        float angle_delta = fabs(get_angle() - tr_cal_start_angle);
        float req_angle = get_required_turn_angle_deg(abs(tr_cal_current_angle));
        
        // Need at least the required target angle to finalize measurement
        if (angle_delta >= req_angle) {
            // Reached 360°! Record measurement
            finalize_measurement();
            
            // Stop motors
            stop(false);
            turn_radius_state = TR_STOPPING;
            tr_cal_drive_start_time = millis();
            
            Serial.println("Circle complete, stopping...");
        } else {
            // Safety timeout: if we've been driving too long without reaching the required angle
            if ((millis() - tr_cal_drive_start_time) > tr_cal_circle_timeout_ms) {
                Serial.println("TIMEOUT: Required turn angle not reached, aborting this angle");
                stop(false);
                turn_radius_state = TR_STOPPING;
                tr_cal_drive_start_time = millis();
            }
        }
    }
    else if (turn_radius_state == TR_STOPPING) {
        // Wait 1 second for robot to settle
        if ((millis() - tr_cal_drive_start_time) > 1000) {
            // Re-enable servo before advancing to next angle
            // (stop() in finalize_measurement disables it)
            servo_disabled = false;
            turn_radius_state = TR_NEXT_ANGLE;
            tr_cal_drive_start_time = millis();
        }
    }
    else if (turn_radius_state == TR_NEXT_ANGLE) {
        // Advance to next angle
        tr_cal_current_angle_index++;
        
        // Check if we finished all angles in current phase
        if (tr_cal_current_angle_index >= tr_cal_num_angles) {
            if (tr_cal_phase == 0) {
                // Finished left turns — pause and wait for user to toggle switch
                // or send 'C' via serial to continue with right turns.
                // The left data is preserved in tr_cal_left so that
                // turn_radius_cal_start() can detect it and resume.
                turn_radius_state = TR_DONE;
                stop(false);
                Serial.println("\n========================================");
                Serial.println("LEFT TURNS COMPLETE!");
                Serial.println("========================================");
                Serial.println("Toggle the enable switch OFF then ON to start RIGHT turns.");
                Serial.println("(Or send 'C' via serial to continue.)");
                Serial.println("========================================\n");
                return;
            } else {
                // All done — both left and right turns complete
                turn_radius_state = TR_DONE;
                Serial.println("\nAll angles measured!");
                calibration_print_results();
                return;
            }
        }
        
        // Set next angle
        // Negative steering = left turn (phase 0 = left)
        // Positive steering = right turn (phase 1 = right)
        int abs_angle = tr_cal_angle_sequence[tr_cal_current_angle_index];
        tr_cal_current_angle = tr_cal_is_right_turn ? abs_angle : -abs_angle;
        
        Serial.print("\nNext angle: ");
        Serial.print(tr_cal_current_angle);
        Serial.println("°");
        
        // Start driving
        set_steering(tr_cal_current_angle);
        set_speed(CAL_SPEED_MMS);
        
        tr_cal_start_distance = current_distance;
        tr_cal_start_angle = get_angle();
        tr_cal_drive_start_time = millis();
        turn_radius_state = TR_DRIVING;
    }
}

void turn_radius_cal_stop()
{
    turn_radius_state = TR_IDLE;
    stop(false);
    Serial.println("Turn radius calibration stopped.");
}

bool turn_radius_cal_is_active()
{
    return turn_radius_state != TR_IDLE && turn_radius_state != TR_DONE;
}
