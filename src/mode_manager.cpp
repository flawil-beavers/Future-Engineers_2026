/**
 * @file mode_manager.cpp
 * @brief Mode manager implementation
 * 
 * Centralizes mode switching logic so that all autonomous behaviors
 * (gyro follow, calibration, future modes) are controlled from one place.
 * 
 * Key behaviors:
 *   - Only ONE mode active at a time
 *   - Switching modes stops the old mode before starting the new one
 *   - Pause remembers the mode, resume restarts it
 *   - stop_all() clears the pending mode so nothing resumes
 */

#include "mode_manager.h"
#include "motor_control.h"
#include "turn_radius_calibration.h"
#include "servo_center_calibration.h"
#include "wall_follower.h"
#include "logger.h"
#define Serial robot_logger

// ==========================================
// MODE STATE
// ==========================================

RobotMode current_mode = MODE_NONE;
RobotMode pending_mode = MODE_NONE;

// ==========================================
// INTERNAL: Stop a specific mode
// ==========================================

/**
 * @brief Stop a mode without affecting the state variables.
 * Called by mode_switch() and mode_stop_all().
 */
static void stop_mode(RobotMode mode)
{
    switch (mode) {
        case MODE_GYRO_FOLLOW:
            gyro_follower_disable();
            break;
        case MODE_TURN_RADIUS_CAL:
            // Always stop calibration regardless of state (IDLE, DRIVING, DONE, etc.)
            // turn_radius_cal_stop() handles all states internally.
            stop(false);
            turn_radius_cal_stop();
            break;
        case MODE_SERVO_CENTER_CAL:
            stop(false);
            servo_center_cal_stop();
            break;
        case MODE_NONE:
            // Nothing to stop
            break;
    }
}

// ==========================================
// INTERNAL: Start a specific mode
// ==========================================

/**
 * @brief Start a mode. Assumes the previous mode has already been stopped.
 */
static void start_mode(RobotMode mode)
{
    switch (mode) {
        case MODE_GYRO_FOLLOW:
            gyro_follower_enable();
            break;
        case MODE_TURN_RADIUS_CAL:
            turn_radius_cal_start();
            break;
        case MODE_SERVO_CENTER_CAL:
            servo_center_cal_start();
            break;
        case MODE_NONE:
            // Nothing to start
            break;
    }
}

// ==========================================
// PUBLIC FUNCTIONS
// ==========================================

void mode_switch(RobotMode new_mode)
{
    if (new_mode == current_mode) {
        Serial.print("Already in mode: ");
        Serial.println(mode_name(new_mode));
        return;
    }
    
    // Stop the old mode
    stop_mode(current_mode);
    
    // If system is disabled, just remember the mode as pending
    if (!system_enabled && new_mode != MODE_NONE) {
        pending_mode = new_mode;
        current_mode = MODE_NONE; // No active mode, but pending
        Serial.print("System disabled. Mode ");
        Serial.print(mode_name(new_mode));
        Serial.println(" will start when enabled.");
        return;
    }
    
    // Start the new mode
    current_mode = new_mode;
    pending_mode = MODE_NONE; // Clear pending since we're starting now
    
    if (new_mode != MODE_NONE) {
        system_enable();
        start_mode(new_mode);
    }
    
    Serial.print("Mode switched to: ");
    Serial.println(mode_name(new_mode));
}

void mode_pause()
{
    if (current_mode != MODE_NONE) {
        // Remember what was running so we can resume
        pending_mode = current_mode;
        stop_mode(current_mode);
        current_mode = MODE_NONE;
        
        Serial.print("Paused. Pending mode: ");
        Serial.println(mode_name(pending_mode));
    }
    
    // Always fully disable the system: stop motors, set enabled=false, flush USB log
    system_disable();
}

void mode_resume()
{
    // Always enable the system (motors, steering)
    system_enable();
    
    if (pending_mode != MODE_NONE) {
        RobotMode mode_to_start = pending_mode;
        pending_mode = MODE_NONE; // Clear before starting to avoid re-entry
        
        current_mode = mode_to_start;
        start_mode(mode_to_start);
        
        Serial.print("Resumed mode: ");
        Serial.println(mode_name(mode_to_start));
    } else {
        // No pending mode, but motors are now enabled for manual control
        Serial.println("System enabled (no pending mode).");
    }
}

void mode_stop_all()
{
    stop_mode(current_mode);
    current_mode = MODE_NONE;
    pending_mode = MODE_NONE;
    stop(false);
    set_steering(0);
    
    Serial.println("All modes stopped.");
}

const char* mode_name(RobotMode mode)
{
    switch (mode) {
        case MODE_NONE:                 return "NONE";
        case MODE_GYRO_FOLLOW:          return "GYRO_FOLLOW";
        case MODE_TURN_RADIUS_CAL:      return "TURN_RADIUS_CAL";
        case MODE_SERVO_CENTER_CAL:     return "SERVO_CENTER_CAL";
        default:                        return "UNKNOWN";
    }
}