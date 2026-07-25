#pragma once

/**
 * @file position_estimator.h
 * @brief Position Estimator (Dead Reckoning Odometry)
 * 
 * Integrates encoder distance and gyro heading to estimate the robot's
 * (x, y) position and heading relative to its starting origin.
 * 
 * Optionally uses calibrated turn radius data to cross-validate the
 * odometry during turns and detect slip events.
 */

#include <Arduino.h>

// ==========================================
// POSITION ESTIMATE STRUCTURE
// ==========================================

/**
 * @brief Current position estimate with confidence
 */
struct PositionEstimate {
    float x_mm;             ///< X coordinate from origin (mm)
    float y_mm;             ///< Y coordinate from origin (mm)
    float heading_deg;      ///< Heading in degrees (0 = +X, 90 = +Y)
    float confidence_mm;    ///< Uncertainty radius (0 = perfect, grows with travel)
};

// ==========================================
// PUBLIC INTERFACE
// ==========================================

/**
 * @brief Initialize the position estimator
 * Sets the origin to (0, 0) with the given heading.
 * Should be called during setup() or whenever resetting to a known pose.
 * 
 * @param x Initial X position (mm), default 0
 * @param y Initial Y position (mm), default 0
 * @param heading Initial heading in degrees, default 0
 */
void position_init(float x = 0, float y = 0, float heading = 0);

/**
 * @brief Update the position estimate
 * Must be called every main loop iteration, AFTER update_gyro().
 * Integrates encoder distance and gyro heading change using dead reckoning.
 */
void update_position();

/**
 * @brief Get the current position estimate
 * @param x Output X position (mm)
 * @param y Output Y position (mm)
 * @param heading Output heading in degrees (0 = +X, 90 = +Y)
 */
void get_position(float &x, float &y, float &heading);

/**
 * @brief Get the current position estimate as a struct
 * @return Current PositionEstimate
 */
PositionEstimate get_position_struct();

/**
 * @brief Get the position uncertainty radius
 * @return Confidence radius in mm (0 = perfect, grows ~1% of distance traveled)
 */
float get_position_confidence();

/**
 * @brief Get the total distance traveled since initialization
 * @return Total distance in mm
 */
float get_total_distance_traveled();

/**
 * @brief Reset the position estimator to a new origin
 * @param x New X position (mm)
 * @param y New Y position (mm)
 * @param heading New heading in degrees
 */
void position_reset(float x, float y, float heading);

/**
 * @brief Print the current position to serial
 * Format: "POS: x=1234.5 y=567.8 h=45.3 c=12.3 dist=1234.5"
 */
void position_print();