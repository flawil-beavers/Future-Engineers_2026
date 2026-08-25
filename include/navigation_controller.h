/**
 * @file navigation_controller.h
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
 * @enum NavigationState
 * @brief Operational states for the navigation state machine.
 */
enum NavigationState {
    NAV_IDLE,        ///< System is inactive, waiting for enable command
    NAV_FOLLOWING,   ///< Straight-line navigation using Gyro and ToF correction
    NAV_TURNING,     ///< Executing a 90-degree pivot turn
    NAV_CORNER_BRAKING_FOR_REVERSE, ///< Stop forward motion before reversing
    NAV_CORNER_REVERSING, ///< First-lap reverse heading correction
    NAV_CORNER_BRAKING_FOR_ALIGN, ///< Stop reverse motion before driving forward
    NAV_CORNER_ALIGNING,  ///< First-lap forward alignment
    NAV_CORNER_BRAKING_FOR_SECTION_BACKUP,
    NAV_CORNER_SECTION_BACKING,
    NAV_CORNER_BRAKING_AFTER_SECTION_BACKUP,
    NAV_STOPPED      ///< Mission complete, final halt state
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

extern NavigationState nav_state;
extern float nav_target_distance;
extern float nav_normal_speed;

// ==========================================
// PUBLIC INTERFACE
// ==========================================

/**
 * @brief Initializes the gyro follower subsystem variables.
 */
void navigation_setup();

/**
 * @brief Main update loop for the navigation logic.
 * @param enabled Whether the autonomous system should be active.
 */
void navigation_update(bool enabled);
bool navigation_is_complete();

/**
 * @brief Activates the autonomous navigation mode.
 * 
 * Sets the initial gyro target to the current heading and transitions 
 * to the following state.
 */
void navigation_enable();

/**
 * @brief Deactivates navigation and halts the robot.
 */
void navigation_disable();

// Configuration and Telemetry
void navigation_set_target_distance(float distance_mm);
void navigation_set_wall_margin(float distance_m);
void navigation_set_pd_gains(float kp, float kd);
void general_debug_set(bool enable);
void navigation_set_speed(float speed_mm_s);
void navigation_set_obstacle_mode(bool enable);
void navigation_select_wall(
    WallSide side,
    float target_distance_mm);
void navigation_print_debug();
void navigation_rearm_after_obstacle();
void navigation_reset_filter();
float navigation_compute_steering(float heading_error_deg, float last_error_deg, float dt_s);
float navigation_get_gyro_kp();
float navigation_get_gyro_kd();
const char* navigation_state_string(NavigationState _state);
NavigationState navigation_get_state();

float navigation_get_target_heading();
float navigation_get_section_origin_distance();
int navigation_get_turn_count();
int navigation_get_turn_angle();
WallSide navigation_get_following_wall();
WallSide navigation_get_course_wall();
float navigation_get_learned_straight_mm(uint8_t section);
#endif // WALL_FOLLOWER_H


