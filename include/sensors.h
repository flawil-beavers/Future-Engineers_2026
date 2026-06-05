#pragma once

/**
 * @file sensors.h
 * @brief Sensor subsystem: Gyro (BNO085) and ToF distance sensors (VL53L4CX)
 */

#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include <vl53l4cx_class.h>

// ==========================================
// SENSOR STATE VARIABLES
// ==========================================

// Gyro
extern Adafruit_BNO08x bno;
extern sh2_SensorValue_t sensor_value;
extern float current_degree;

// ToF sensors
extern VL53L4CX sensor_left;
extern VL53L4CX sensor_right;

// Distance readings in millimeters
extern float current_distance_left;
extern float current_distance_right;

// ==========================================
// SENSOR FUNCTIONS
// ==========================================

/**
 * @brief Update gyro heading angle
 * Reads latest rotation vector and calculates yaw angle in degrees
 * Should be called regularly (every ~20ms)
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
 * @brief Initialize all sensors
 * Sets up I2C buses, gyro (SPI), and ToF sensors
 * Must be called during setup() before main loop
 */
void sensors_setup();
