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

constexpr uint8_t TOF_DIAGNOSTIC_MAX_OBJECTS = 4;

struct TofObjectDiagnostic {
  int16_t distance_mm;
  float signal_mcps;
  float sigma_mm;
  uint8_t range_status;
  bool hardware_valid;
  bool filter_accepted;
};

struct TofDiagnosticSnapshot {
  uint32_t sequence;
  float filtered_distance_mm;
  float selected_raw_distance_mm;
  float selected_signal_mcps;
  float selected_sigma_mm;
  uint32_t timing_budget_us;
  VL53L4CX_DistanceModes distance_mode;
  uint8_t reported_object_count;
  uint8_t stored_object_count;
  int8_t selected_object_index;
  TofObjectDiagnostic objects[TOF_DIAGNOSTIC_MAX_OBJECTS];
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
 * @brief Whether fresh game-rotation-vector data is available for control.
 * @return False during startup and genuine BNO085 reset recovery
 */
bool gyro_is_healthy();

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

/** Copy the most recent complete ranging frame for stationary diagnostics. */
bool get_tof_diagnostic_snapshot(TofSensor sensor,
                                 TofDiagnosticSnapshot &snapshot);

/** Change both sensors' timing budget at runtime. */
void sensors_set_tof_timing_budget(uint32_t budget_us);

/** Stop, reconfigure, and restart both sensors for a stationary test. */
bool sensors_configure_tof_for_test(VL53L4CX_DistanceModes distance_mode,
                                    uint32_t budget_us);

/**
 * @brief Initialize all sensors
 * Sets up I2C buses, gyro (SPI), and ToF sensors
 * Must be called during setup() before main loop
 */
void sensors_setup();
