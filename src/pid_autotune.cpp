/**
 * @file pid_autotune.cpp
 * @brief PID speed controller auto-tuning implementation
 * 
 * Implements the Åström-Hägglund relay feedback method for automatic
 * PID tuning of the motor speed controller.
 * 
 * The robot drives straight while a relay controller causes speed
 * oscillations around the target. From the limit cycle, the ultimate
 * gain (Ku) and period (Tu) are measured, and Ziegler-Nichols gains
 * are computed.
 * 
 * Safety:
 *   - Steering locked to 0° (straight)
 *   - Max 1m travel forward or backward from start position
 *   - 30-second timeout
 *   - Enable switch pause/resume works normally
 */

#include "pid_autotune.h"
#include "motor_control.h"
#include "config.h"
#include "logger.h"
#define Serial robot_logger

// ==========================================
// PID AUTOTUNE STATE
// ==========================================

PIDAtuneState pid_atune_state = AT_IDLE;
PIDAtuneResult pid_atune_result;

// ==========================================
// INTERNAL STATE
// ==========================================

static float start_distance = 0;        // Encoder distance at start (mm)
static float last_dist = 0;             // Last distance for delta tracking
static unsigned long start_time_us = 0; // Micros at start
static float baseline_dc = 0;           // Estimated baseline DC for target speed

// Zero-crossing tracking
static int last_error_sign = 0;         // Sign of last error (+1 or -1)
static unsigned long last_zero_cross_us = 0; // Time of last zero-crossing
static float current_half_cycle_peak = 0;    // Peak |error| in current half-cycle
static float sum_peaks = 0;             // Sum of peak errors for averaging
static float sum_periods = 0;           // Sum of half-periods for averaging
static int peak_count = 0;              // Number of peaks measured
static int zero_cross_count = 0;        // Number of zero-crossings

// ==========================================
// INTERNAL HELPERS
// ==========================================

/**
 * @brief Get the actual robot speed from encoder distance change
 * Uses the last_loop_time and distance delta from loop_updater.
 */
static float get_actual_speed()
{
    extern float last_loop_time;
    extern float current_distance;
    extern float last_distance;
    
    if (last_loop_time <= 0) return 0;
    return (current_distance - last_distance) / last_loop_time;
}

/**
 * @brief Record a zero-crossing event and update measurements
 */
static void record_zero_crossing()
{
    unsigned long now = micros();
    zero_cross_count++;
    
    // Record the peak of the completed half-cycle
    if (peak_count > 0 || zero_cross_count > 1) {
        sum_peaks += current_half_cycle_peak;
        peak_count++;
    }
    
    // Record the half-period (time since last zero-crossing)
    if (last_zero_cross_us > 0) {
        float half_period = (now - last_zero_cross_us) / 1000000.0f;
        sum_periods += half_period;
    }
    
    last_zero_cross_us = now;
    current_half_cycle_peak = 0; // Reset for next half-cycle
}

/**
 * @brief Compute PID gains from measured Ku and Tu using Ziegler-Nichols
 */
static void compute_ziegler_nichols()
{
    // Average the peak amplitudes to get 'a' (process output amplitude)
    float a = (peak_count > 0) ? (sum_peaks / peak_count) : 0;
    
    // Average the half-periods to get Tu/2, then Tu = 2 * avg_half_period
    int period_count = zero_cross_count - 1; // One fewer period than crossings
    float avg_half_period = (period_count > 0) ? (sum_periods / period_count) : 0;
    float Tu = avg_half_period * 2.0f;
    
    // Relay amplitude 'd' (the DC step change)
    float d = (float)PID_AT_RELAY_AMPLITUDE;
    
    // Ultimate gain: Ku = 4d / (π * a)
    if (a > 0.001f && Tu > 0.001f) {
        pid_atune_result.Ku = 4.0f * d / (PI * a);
        pid_atune_result.Tu = Tu;
        
        // Ziegler-Nichols PID tuning
        pid_atune_result.Kp = 0.6f * pid_atune_result.Ku;
        pid_atune_result.Ki = 2.0f * pid_atune_result.Kp / Tu;
        pid_atune_result.Kd = pid_atune_result.Kp * Tu / 8.0f;
    } else {
        // Fallback: use current gains if measurement failed
        pid_atune_result.Ku = 0;
        pid_atune_result.Tu = 0;
        pid_atune_result.Kp = Kp;
        pid_atune_result.Ki = Ki;
        pid_atune_result.Kd = Kd;
    }
}

/**
 * @brief Print autotune results to serial
 */
static void print_results()
{
    Serial.println("\n=== PID AUTOTUNE ===");
    
    if (pid_atune_result.aborted) {
        Serial.println("STATUS: ABORTED (distance or time limit reached)");
    } else {
        Serial.println("STATUS: COMPLETE");
    }
    
    Serial.print("Zero-crossings: ");
    Serial.println(zero_cross_count);
    Serial.print("Peaks measured: ");
    Serial.println(peak_count);
    Serial.print("Distance forward: ");
    Serial.print(pid_atune_result.distance_forward);
    Serial.println(" mm");
    Serial.print("Distance backward: ");
    Serial.print(pid_atune_result.distance_backward);
    Serial.println(" mm");
    
    if (!pid_atune_result.aborted && pid_atune_result.Ku > 0) {
        Serial.print("Ultimate Gain (Ku): ");
        Serial.println(pid_atune_result.Ku, 3);
        Serial.print("Ultimate Period (Tu): ");
        Serial.print(pid_atune_result.Tu, 3);
        Serial.println(" s");
        Serial.print("Overshoot ratio: ");
        Serial.println(pid_atune_result.overshoot, 3);
        
        Serial.println("\nRecommended Ziegler-Nichols gains:");
        Serial.print("  Kp = ");
        Serial.println(pid_atune_result.Kp, 3);
        Serial.print("  Ki = ");
        Serial.println(pid_atune_result.Ki, 3);
        Serial.print("  Kd = ");
        Serial.println(pid_atune_result.Kd, 3);
        
        Serial.println("\nApply with:");
        Serial.print("  q");
        Serial.print((int)(pid_atune_result.Kp * 10));
        Serial.print(" w");
        Serial.print((int)(pid_atune_result.Ki * 100));
        Serial.print(" e");
        Serial.println((int)(pid_atune_result.Kd * 10));
    }
    
    Serial.println("=====================\n");
}

// ==========================================
// PUBLIC FUNCTIONS
// ==========================================

void pid_autotune_start()
{
    if (pid_atune_state != AT_IDLE) {
        pid_autotune_stop();
    }
    
    // Reset all state
    pid_atune_state = AT_ACCELERATING;
    
    pid_atune_result.Ku = 0;
    pid_atune_result.Tu = 0;
    pid_atune_result.Kp = 0;
    pid_atune_result.Ki = 0;
    pid_atune_result.Kd = 0;
    pid_atune_result.overshoot = 0;
    pid_atune_result.zero_crossings = 0;
    pid_atune_result.distance_forward = 0;
    pid_atune_result.distance_backward = 0;
    pid_atune_result.aborted = false;
    
    // Internal state
    start_distance = get_distance(encoder_pos);
    last_dist = start_distance;
    start_time_us = micros();
    
    // Estimate baseline DC for the target speed
    // Use a fraction of max DC as a rough starting point
    baseline_dc = (float)PID_AT_TARGET_SPEED / 500.0f * (float)MOTOR_MAX_DC;
    if (baseline_dc > MOTOR_MAX_DC) baseline_dc = MOTOR_MAX_DC;
    if (baseline_dc < MOTOR_MIN_DC) baseline_dc = MOTOR_MIN_DC;
    
    // Zero-crossing tracking
    last_error_sign = 0;
    last_zero_cross_us = 0;
    current_half_cycle_peak = 0;
    sum_peaks = 0;
    sum_periods = 0;
    peak_count = 0;
    zero_cross_count = 0;
    
    // Enable motor and lock steering straight
    dc_state = DC_ENABLED;
    set_steering(0);
    
    Serial.println("\n=== PID AUTOTUNE START ===");
    Serial.print("Target speed: ");
    Serial.print(PID_AT_TARGET_SPEED);
    Serial.print(" mm/s, Relay amplitude: ±");
    Serial.print(PID_AT_RELAY_AMPLITUDE);
    Serial.print(" DC, Baseline DC: ");
    Serial.println(baseline_dc, 0);
    Serial.print("Max travel: ±");
    Serial.print(PID_AT_MAX_DISTANCE_MM);
    Serial.println(" mm from start");
    Serial.println("Accelerating to target speed...\n");
}

void pid_autotune_update()
{
    if (pid_atune_state == AT_IDLE || pid_atune_state == AT_DONE) {
        return;
    }
    
    // Always lock steering to straight
    set_steering(0);
    
    // Get actual speed from encoder
    float actual_speed = get_actual_speed();
    float error = (float)PID_AT_TARGET_SPEED - actual_speed;
    
    // Track forward/backward distance from start
    float current_dist = get_distance(encoder_pos);
    float dist_from_start = current_dist - start_distance;
    
    // Track cumulative forward/backward travel
    float delta = current_dist - last_dist;
    if (delta > 0) {
        pid_atune_result.distance_forward += delta;
    } else {
        pid_atune_result.distance_backward += fabs(delta);
    }
    last_dist = current_dist;
    
    // ==========================================
    // SAFETY CHECKS
    // ==========================================
    
    // Check distance limit (±1m from start)
    if (dist_from_start > PID_AT_MAX_DISTANCE_MM || 
        dist_from_start < -PID_AT_MAX_DISTANCE_MM) {
        Serial.print("ABORT: Distance limit exceeded (");
        Serial.print(dist_from_start, 0);
        Serial.println(" mm from start)");
        pid_atune_result.aborted = true;
        pid_atune_state = AT_DONE;
        stop(false);
        print_results();
        return;
    }
    
    // Check timeout
    if (micros() - start_time_us > PID_AT_MAX_TIME_US) {
        Serial.println("ABORT: Timeout reached");
        pid_atune_result.aborted = true;
        pid_atune_state = AT_DONE;
        stop(false);
        print_results();
        return;
    }
    
    // ==========================================
    // STATE MACHINE
    // ==========================================
    
    // Determine current error sign (with small deadband to avoid noise)
    int current_sign = 0;
    if (error > 1.0f) current_sign = 1;
    else if (error < -1.0f) current_sign = -1;
    else current_sign = last_error_sign; // In deadband, keep previous sign
    
    switch (pid_atune_state) {
        
        case AT_ACCELERATING:
            // Apply baseline DC and wait for speed to approach target
            set_dc(baseline_dc);
            
            // Transition to relay control when speed is within 15% of target
            if (fabs(error) < (float)PID_AT_TARGET_SPEED * 0.15f) {
                pid_atune_state = AT_RELAY_POS;
                last_error_sign = current_sign;
                last_zero_cross_us = micros();
                current_half_cycle_peak = 0;
                Serial.println("Target speed reached, starting relay control...");
            }
            break;
            
        case AT_RELAY_POS:
            // Apply positive relay DC (speed too slow → more power)
            set_dc(baseline_dc + PID_AT_RELAY_AMPLITUDE);
            
            // Track peak error in this half-cycle
            if (fabs(error) > current_half_cycle_peak) {
                current_half_cycle_peak = fabs(error);
            }
            
            // Detect zero-crossing: error was positive, now negative or zero
            if (current_sign < 0 && last_error_sign >= 0) {
                record_zero_crossing();
                pid_atune_state = AT_RELAY_NEG;
            }
            last_error_sign = current_sign;
            break;
            
        case AT_RELAY_NEG:
            // Apply negative relay DC (speed too fast → less power)
            set_dc(baseline_dc - PID_AT_RELAY_AMPLITUDE);
            
            // Track peak error in this half-cycle
            if (fabs(error) > current_half_cycle_peak) {
                current_half_cycle_peak = fabs(error);
            }
            
            // Detect zero-crossing: error was negative, now positive or zero
            if (current_sign > 0 && last_error_sign <= 0) {
                record_zero_crossing();
                pid_atune_state = AT_RELAY_POS;
            }
            last_error_sign = current_sign;
            break;
            
        default:
            break;
    }
    
    // Check if we have enough data to compute gains
    // We need at least PID_AT_MIN_CYCLES complete cycles (2 * min_cycles zero-crossings)
    int required_crossings = PID_AT_MIN_CYCLES * 2;
    if (zero_cross_count >= required_crossings && peak_count >= required_crossings) {
        pid_atune_state = AT_DONE;
        
        // Compute overshoot as ratio of average peak to target speed
        float avg_peak = (peak_count > 0) ? (sum_peaks / peak_count) : 0;
        pid_atune_result.overshoot = avg_peak / (float)PID_AT_TARGET_SPEED;
        pid_atune_result.zero_crossings = zero_cross_count;
        
        // Compute Ziegler-Nichols gains
        compute_ziegler_nichols();
        
        // Stop motors
        stop(false);
        
        // Print results
        print_results();
    }
}

void pid_autotune_stop()
{
    if (pid_atune_state != AT_IDLE) {
        pid_atune_state = AT_IDLE;
        stop(false);
        Serial.println("PID autotune stopped.");
    }
}

bool pid_autotune_is_active()
{
    return pid_atune_state != AT_IDLE && pid_atune_state != AT_DONE;
}

const PIDAtuneResult& pid_autotune_get_result()
{
    return pid_atune_result;
}

void pid_autotune_apply_gains()
{
    if (pid_atune_state != AT_DONE || pid_atune_result.Ku <= 0) {
        Serial.println("No valid autotune results to apply.");
        return;
    }
    
    Kp = pid_atune_result.Kp;
    Ki = pid_atune_result.Ki;
    Kd = pid_atune_result.Kd;
    
    // Reset PID state for clean transition
    pid_integral = 0;
    last_error = 0;
    
    Serial.println("Applied autotune PID gains:");
    Serial.print("  Kp = ");
    Serial.println(Kp, 3);
    Serial.print("  Ki = ");
    Serial.println(Ki, 3);
    Serial.print("  Kd = ");
    Serial.println(Kd, 3);
}