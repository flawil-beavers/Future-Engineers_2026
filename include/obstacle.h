#pragma once

/**
 * @file obstacle.h
 * @brief Camera based obstacle detection and avoidance for the
 *        WRO Future Engineers Obstacle Challenge.
 */

#include <Arduino.h>

#include "camera.h"
#include "vision.h"


// ==========================================
// GLOBAL OBJECTS
// ==========================================

extern CameraSystem camera;
extern Vision vision;


// ==========================================
// OBSTACLE AVOIDANCE STATES
// ==========================================

enum ObstacleAvoidanceState
{
    OA_IDLE,
    OA_TRACKING,
    OA_PASSING,
    OA_RECOVERING
};


// ==========================================
// CAMERA
// ==========================================

bool updateCameraVision();

void printVisionDebug();

void printCameraCalibration();


// ==========================================
// OBSTACLE DETECTION
// ==========================================

const Blob* getLargestObstacle();

void handleObstacleDetection();


// ==========================================
// OBSTACLE AVOIDANCE
// ==========================================

bool obstacle_avoidance_update(
    bool enabled,
    bool newCameraFrame
);

bool obstacle_avoidance_active();

void obstacle_avoidance_reset();

ObstacleAvoidanceState obstacle_avoidance_get_state();

const char* obstacle_avoidance_state_string(
    ObstacleAvoidanceState state
);


// ==========================================
// COMPLETE OBSTACLE CHALLENGE
// ==========================================

void obstacle_challenge_setup();

void obstacle_challenge_update(
    bool enabled,
    bool newCameraFrame
);

/**
 * @brief Returns true while an obstacle challenge run is active.
 */
bool obstacle_challenge_active();
bool obstacle_parking_exit_active();

/**
 * @brief Enable/disable a stationary camera/steering test.
 *        The drive motor is forced off while this mode is active.
 */
void obstacle_bench_test_set(bool enable);
bool obstacle_bench_test_active();
