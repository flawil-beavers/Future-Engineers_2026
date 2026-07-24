#pragma once

/**
 * @file obstacle.h
 * @brief Obstacle detection subsystem using camera and vision processing.
 *
 * Provides functions for camera capture, vision processing, and obstacle
 * detection (red/green blobs) from the camera feed.
 */

#include <Arduino.h>
#include "camera.h"
#include "vision.h"

// ==========================================
// GLOBAL OBJECTS (defined in main.cpp)
// ==========================================

extern CameraSystem camera;
extern Vision vision;

// ==========================================
// CAMERA FUNCTIONS
// ==========================================

/**
 * @brief Captures a frame and runs vision processing at a fixed interval.
 *
 * Uses a 50 ms throttle to avoid overloading the CPU.
 */
void updateCameraVision();

/**
 * @brief Prints vision debug information (blob positions, sizes, etc.)
 * to the serial console at 500 ms intervals.
 */
void printVisionDebug();

/**
 * @brief Prints the HSV value at the center of the camera frame
 * to the serial console at 500 ms intervals.
 *
 * Useful for calibrating colour thresholds.
 */
void printCameraCalibration();

// ==========================================
// OBSTACLE DETECTION
// ==========================================

/**
 * @brief Returns a pointer to the largest visible obstacle blob
 * (red or green), or nullptr if none are found.
 *
 * When both colours are visible, the larger blob is returned.
 */
const Blob* getLargestObstacle();

/**
 * @brief Prints obstacle detection results (position, area, colour)
 * to the serial console.
 */
void handleObstacleDetection();