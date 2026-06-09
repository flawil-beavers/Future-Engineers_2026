#pragma once

/**
 * @file wall_follower.h
 * @brief Autonomous wall-following control subsystem
 * 
 * Implements robot wall-following behavior:
 * 1. Waits for enable signal (enable switch toggle or serial command)
 * 2. Follows wall at target distance using PD controller
 * 3. Detects wall gaps (distance > 1m) and turns 90° into them
 * 4. Counts complete perimeter rounds (4 turns = 1 round)
 * 5. Stops after 3 complete rounds in the middle of a straight section
 */

#include <Arduino.h>

// ==========================================
// STATE DEFINITIONS
// ==========================================

enum WallFollowerState
{
  WF_IDLE = 0,        // Waiting for enable signal
  WF_GYRO_FOLLOW = 1, // Maintaining a specific gyro heading (PD control)
  WF_FOLLOWING = 2,   // Following wall at target distance (PD control)
  WF_TURNING = 3,     // Executing 90-degree turn
  WF_STOPPED = 4      // Finished (3 rounds completed)
};

enum WallSide
{
  SIDE_LEFT = 0,
  SIDE_RIGHT = 1,
  SIDE_UNKNOWN = 2
};

// ==========================================
// STATE VARIABLES
// ==========================================

extern WallFollowerState wf_state;
extern WallFollowerState wf_last_state;

// Wall following parameters
extern float wf_target_distance;      // Target distance from wall (mm)
extern float wf_wall_margin;          // Distance threshold to detect gap (m)
extern int wf_turn_angle;             // Current turn direction (+90 or -90)
extern WallSide wf_following_wall;    // Which wall side we are following

// Gyro following parameters
extern float wf_gyro_target;          // The absolute gyro angle to maintain
extern float wf_gyro_kp;              // Proportional gain for gyro error
extern float wf_gyro_kd;              // Derivative gain for gyro error
extern float wf_last_gyro_error;      // Previous error for derivative term

// Timing and control
extern float wf_turn_start_angle;        // Angle when turn started
extern float wf_turn_target_angle;       // Target angle to complete turn (current_degree + turn_angle)

// Round counting
extern int wf_turn_count;             // Total turns executed
extern float wf_start_angle;        // Heading when wall-following started
extern int wf_completed_rounds;       // Number of complete rounds (0-3)

// PD Controller for distance control
extern float wf_pd_kp;                // Proportional gain for distance error
extern float wf_pd_kd;                // Derivative gain for distance error
extern float wf_last_distance_error;  // Previous error for derivative term

// Debug variables
extern bool wf_debug_enabled;         // Print debug info
extern unsigned long wf_last_debug_time;

// ==========================================
// CONTROL FUNCTIONS
// ==========================================

/**
 * @brief Initialize wall-following subsystem
 * Should be called during setup()
 */
void wall_follower_setup();

/**
 * @brief Main wall-following update function
 * @param enabled Whether the system is active. Logic is suppressed if false, but debug printing remains.
 */
void wall_follower_update(bool enabled);

/**
 * @brief Start wall-following behavior
 * Saves initial heading and switches to FOLLOWING state
 * Should be called when enable signal detected
 */
void wall_follower_enable();

/**
 * @brief Stop wall-following immediately
 * Returns to IDLE state
 */
void wall_follower_disable();

/**
 * @brief Get current wall-follower state
 * @return Current WallFollowerState
 */
WallFollowerState wall_follower_get_state();

/**
 * @brief Get current state as string for debugging
 * @param wf_state Optional state parameter (defaults to current state)
 * @return State name ("IDLE", "FOLLOWING", "TURNING", "STOPPED")
 */
const char* wall_follower_state_string(WallFollowerState wf_state = wf_state);

/**
 * @brief Enable/disable debug output
 * @param enable True to enable debug output
 */
void wall_follower_set_debug(bool enable);

/**
 * @brief Print debug telemetry
 * Called automatically if debug is enabled
 */
void wall_follower_print_debug();

// ==========================================
// CONFIGURATION
// ==========================================

/**
 * @brief Set target distance from wall
 * @param distance_mm Distance in millimeters
 */
void wall_follower_set_target_distance(float distance_mm);

/**
 * @brief Set wall margin threshold (distance > this triggers turn)
 * @param distance_m Distance in meters (default 1.0m)
 */
void wall_follower_set_wall_margin(float distance_m);

/**
 * @brief Set PD controller gains
 * @param kp Proportional gain
 * @param kd Derivative gain
 */
void wall_follower_set_pd_gains(float kp, float kd);
