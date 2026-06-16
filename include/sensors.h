#pragma once

/**
 * @file sensors.h
 * @brief Sensor subsystem: Gyro (BNO085) and ToF distance sensors (VL53L4CX)
 */

#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include <vl53l4cx_class.h>

// ==========================================
// SENSOR TYPES
// ==========================================

enum TofSensor {
  TOF_LEFT = 0,
  TOF_RIGHT = 1,
  TOF_COUNT
};

// ==========================================
// SENSOR FUNCTIONS
// ==========================================

/**
 * @brief Polls the BNO085 IMU for rotation data and updates the global heading.
 * 
 * Uses the Game Rotation Vector to provide drift-free yaw measurements.
 * Handles sensor resets automatically and calculates cumulative degrees to 
 * account for multi-turn rotations.
 */
void update_gyro();

/**
 * @brief Update ToF distance sensor readings
 * Reads from both left and right sensors, selects most reliable measurements
 * Should be called regularly in the main loop
 */
void update_lasers();

/**
 * @brief Reset VL53L4CX sensor via I2C protocol
 * Helper function to reset sensors without extra wires
 * @param wire Reference to TwoWire object (Wire or Wire2)
 */
void reset_VL53L4CX_via_I2C(TwoWire &wire);

// ==========================================
// INITIALIZATION
// ==========================================

/**
 * @brief Get current accumulated angle in degrees
 * @return Current angle in degrees (can exceed 360 for multiple rotations)
 */
float get_angle();

/**
 * @brief Get current heading in degrees (normalized to -180 to +180)
 * @return Current heading in degrees
 */
float get_heading();

/**
 * @brief Get the latest distance from a specific ToF sensor
 * @param sensor The sensor to query (TOF_LEFT or TOF_RIGHT)
 * @return Distance in millimeters, or -1.0 if invalid
 */
float get_tof_distance(TofSensor sensor);

/**
 * @brief Get the latest raw distance (before clamping) from a specific ToF sensor
 * @param sensor The sensor to query (TOF_LEFT or TOF_RIGHT)
 * @return Raw distance in millimeters, or -1.0 if invalid
 */
float get_tof_raw_distance(TofSensor sensor);

/**
 * @brief Get the latest signal rate from a specific ToF sensor.
 * @param sensor The sensor to query (TOF_LEFT or TOF_RIGHT)
 * @return Signal rate in Mcps, or -1.0 if invalid
 */
float get_tof_signal_rate(TofSensor sensor);

/**
 * @brief Get the latest sigma (measurement uncertainty) from a specific ToF sensor.
 * @param sensor The sensor to query (TOF_LEFT or TOF_RIGHT)
 * @return Sigma in mm, or -1.0 if invalid
 */
float get_tof_sigma(TofSensor sensor);

/**
 * @brief Initialize all sensors
 * Sets up I2C buses, gyro (SPI), and ToF sensors
 * Must be called during setup() before main loop
 */
void sensors_setup();
