/**
 * @file wall_follower.h
 * @brief Header for the Gyro-Stabilized Wall Follower subsystem.
 * 
 * This module handles autonomous navigation by combining inertial 
 * (gyroscope) and distance (ToF) sensors. It maintains a grid-based 
 * heading while performing lateral corrections based on wall distance.
 */

#ifndef WALL_FOLLOWER_H
#define WALL_FOLLOWER_H

#include <Arduino.h>

// ==========================================
// TYPE DEFINITIONS
// ==========================================

/**
 * @enum GyroFollowerState
 * @brief Operational states for the navigation state machine.
 */
enum GyroFollowerState {
    GF_IDLE,        ///< System is inactive, waiting for enable command
    GF_FOLLOWING,   ///< Straight-line navigation using Gyro and ToF correction
    GF_TURNING,     ///< Executing a 90-degree pivot turn
    GF_STOPPED      ///< Mission complete, final halt state
};

/**
 * @enum WallSide
 * @brief Identifiers for the wall being tracked for distance correction.
 */
enum WallSide {
    SIDE_LEFT,      ///< Tracking the left-hand wall
    SIDE_RIGHT,     ///< Tracking the right-hand wall
    SIDE_UNKNOWN    ///< Searching for a wall to track
};

// ==========================================
// EXTERNAL STATE VARIABLES
// ==========================================

extern GyroFollowerState gf_state;
extern float gf_target_distance;
extern float gf_normal_speed;

// ==========================================
// PUBLIC INTERFACE
// ==========================================

/**
 * @brief Initializes the gyro follower subsystem variables.
 */
void gyro_follower_setup();

/**
 * @brief Main update loop for the navigation logic.
 * @param enabled Whether the autonomous system should be active.
 */
void gyro_follower_update(bool enabled);

/**
 * @brief Activates the autonomous navigation mode.
 * 
 * Sets the initial gyro target to the current heading and transitions 
 * to the following state.
 */
void gyro_follower_enable();

/**
 * @brief Deactivates navigation and halts the robot.
 */
void gyro_follower_disable();

// Configuration and Telemetry
void gyro_follower_set_target_distance(float distance_mm);
void gyro_follower_set_wall_margin(float distance_m);
void gyro_follower_set_pd_gains(float kp, float kd);
void gyro_follower_set_debug(bool enable);
void gyro_follower_print_debug();
void gyro_follower_rearm_after_obstacle();
const char* gyro_follower_state_string(GyroFollowerState _state);
GyroFollowerState gyro_follower_get_state();

float gyro_follower_get_target_heading();
int gyro_follower_get_turn_count();
int gyro_follower_get_turn_angle();
#endif // WALL_FOLLOWER_H


