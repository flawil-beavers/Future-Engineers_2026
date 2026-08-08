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

/**
 * @brief Set the measured camera-to-pillar distance used by calibration.
 *        A value of zero keeps blob diagnostics active without calculating
 *        focal-length samples.
 */
void camera_calibration_set_reference_distance(float distance_mm);


// ==========================================
// OBSTACLE DETECTION
// ==========================================

const Blob* getLargestObstacle();

/**
 * @brief Apply the production acquisition filters used before a new pillar
 *        can affect steering or the known-geometry path.
 */
bool obstacle_blob_valid_for_acquisition(const Blob *obstacle);

/**
 * @brief Return the best red/green blob after production validation.
 *        Invalid edge lines and background regions cannot mask a valid pillar.
 */
const Blob* getLargestValidObstacle();

/**
 * @brief Estimate horizontal camera-to-block-foot range in millimetres.
 */
float obstacle_estimate_camera_range_mm(const Blob *obstacle);

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
bool obstacle_challenge_complete();
bool obstacle_parking_exit_active();

/**
 * @brief Enable/disable a stationary camera/steering test.
 *        The drive motor is forced off while this mode is active.
 */
void obstacle_bench_test_set(bool enable);
bool obstacle_bench_test_active();
