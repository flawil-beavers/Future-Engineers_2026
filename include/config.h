#pragma once

/**
 * @file config.h
 * @brief Centralized configuration for all pin definitions, constants, and tuning parameters
 */

// ==========================================
// SERIAL CONFIGURATION
// ==========================================
#define SERIAL_BAUD 115200
#define BUFFER_SIZE 64

// ==========================================
// MOTOR & STEERING PINS
// ==========================================
#define SERVO_PIN 4
#define MOTOR_IN1_PIN 5        // Direction control 1
#define MOTOR_IN2_PIN 6        // Direction control 2
#define MOTOR_ENA_PIN 7        // PWM speed control

// ==========================================
// ENCODER PINS
// ==========================================
#define ENCODER_PIN_A 3        // Phase A
#define ENCODER_PIN_B 2        // Phase B

// ==========================================
// BNO085 IMU (GYRO) - SPI CONFIGURATION
// ==========================================
#define BNO085_CS 10
#define BNO085_INT A0
#define BNO085_RST A1

// ==========================================
// ENABLE SWITCH
// ==========================================
#define ENABLE_SWITCH_PIN A2   // Enable switch - HIGH to enable, LOW to disable

// ==========================================
// MOTOR CONTROL CONSTANTS
// ==========================================
#define GEAR_RATIO 100                                      // Motor gear ratio
#define ENCODER_COUNTS_PER_REV (GEAR_RATIO * 7 * 2)         // CPR of the motor
#define ENCODER_COUNTS_PER_WHEEL_REV (28.0 / 20.0 * ENCODER_COUNTS_PER_REV) // CPR of the wheel
#define COUNTER_TO_MM (PI * 43.2 / ENCODER_COUNTS_PER_WHEEL_REV)  // mm per encoder count

#define MOTOR_MAX_DC 200                                    // Max duty cycle (0-255)
#define MOTOR_MIN_DC (0.34 * 255)                           // Min duty cycle to overcome static friction
#define MOTOR_MAX_ACC_DC 255                                // Max acceleration duty cycle (DC/s)

// ==========================================
// STEERING CONFIGURATION
// ==========================================
#define SERVO_CENTER 81        // Center neutral position
#define MAX_STEERING 50
#define SERVO_MAX_ANGLE (SERVO_CENTER + MAX_STEERING)  // Max right turn
#define SERVO_MIN_ANGLE (SERVO_CENTER - MAX_STEERING)  // Max left turn

// ==========================================
// PID CONTROLLER TUNING
// ==========================================
#define PID_KP 0.8             // Proportional gain
#define PID_KI 0.2             // Integral gain
#define PID_KD 0.1             // Derivative gain
#define PID_I_MAX 150.0        // Max integral term clamping

// ==========================================
// ACCELERATION SETTINGS
// ==========================================
#define DEFAULT_ACCELERATION 700    // mm/s^2

// ==========================================
// SENSOR UPDATE RATES
// ==========================================
#define GYRO_UPDATE_INTERVAL_MS 20  // Update gyro every 20ms
#define STATUS_PRINT_INTERVAL_US 200000  // Print status every 200ms

// ==========================================
// STALL DETECTION
// ==========================================
#define STALL_THRESHOLD_COUNTS 50         // Encoder counts
#define STALL_DC_THRESHOLD 0.9           // Trigger at 90% of max DC

// ==========================================
// ENABLE INTERRUPT DEBOUNCE
// ==========================================
#define ENABLE_SWITCH_POLL_INTERVAL_US 50000     // 50ms polling interval
#define ENABLE_DEBOUNCE_TIME_US 100000   // 100ms debounce

// ==========================================
// SENSOR READING MODES
// ==========================================
#define TOF_DISTANCE_MODE VL53L4CX_DISTANCEMODE_SHORT
#define TOF_I2C_CLOCK 400000             // 400kHz I2C clock (standard for VL53L4CX)
#define TOF_TIMING_BUDGET_US 30000      // 200ms budget to capture weak signals from black targets at 4m
#define TOF_MAX_RELIABLE_DISTANCE_MM 600.0f // Max distance for reliable wall detection (mm)
#define TOF_MAX_LONG_DISTANCE_MM 4000.0f // Max distance for long-range discovery (mm)
#define TOF_OUT_OF_RANGE_MM 9999.0f        // Value returned when no object is detected or beyond reliable range (mm)
#define TOF_MAX_DELTA_MM 100.0f            // Max change allowed between consecutive readings (mm)

// ==========================================
// ENABLE/DISABLE FLAGS MESSAGES
// ==========================================
#define EN_STATE_TRUE_MSG "enable start"
#define EN_STATE_FALSE_MSG "enable stop"
