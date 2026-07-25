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
#define MOTOR_PWM_PIN 7        // PWM speed control

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
#define GEAR_RATIO 50                                       // Motor gear ratio
#define ENCODER_COUNTS_PER_REV (GEAR_RATIO * 7 * 4)         // CPR of the motor
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
#define STATUS_PRINT_INTERVAL_US 200000  // Print status every 200ms

// ==========================================
// STALL DETECTION
// ==========================================
#define STALL_SPEED_THRESHOLD_MMS 1.0f    // Trigger stall if speed < 1.0 mm/s while demanding high torque
#define STALL_DC_THRESHOLD 0.99           // Trigger at 99% of max DC

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

// ==========================================
// OBSTACLE AVOIDANCE
// ==========================================

// Camera processing
#define OBSTACLE_CAMERA_INTERVAL_MS 50

// Detection validation
#define OBSTACLE_RED_MIN_AREA 300
#define OBSTACLE_RED_MIN_HEIGHT 21
#define OBSTACLE_GREEN_MIN_AREA 400
#define OBSTACLE_GREEN_MIN_HEIGHT 21
#define OBSTACLE_CONFIRM_FRAMES 2
#define OBSTACLE_LOST_FRAMES 3

// Desired obstacle positions inside the 320 px image
// Red must stay on the LEFT -> robot passes on the right
#define OBSTACLE_RED_TARGET_X 95

// Green must stay on the RIGHT -> robot passes on the left
#define OBSTACLE_GREEN_TARGET_X 225

// Camera steering
#define OBSTACLE_CAMERA_KP 0.18f
#define OBSTACLE_HEADING_KP 0.35f
#define OBSTACLE_MAX_STEERING 32.0f

// Start slowly while tuning
#define OBSTACLE_AVOID_SPEED 180
#define OBSTACLE_CRUISE_SPEED 220

// Continue around obstacle after camera loses it
#define OBSTACLE_PASS_STEERING 14.0f
#define OBSTACLE_PASS_DISTANCE_MM 90.0f

// Return to original course heading
#define OBSTACLE_RECOVER_KP 1.5f
#define OBSTACLE_RECOVER_MAX_STEERING 25.0f
#define OBSTACLE_RECOVER_SPEED 180
#define OBSTACLE_RECOVER_TOLERANCE_DEG 3.0f

// Prevent detecting the same obstacle immediately again
#define OBSTACLE_REARM_DISTANCE_MM 150.0f

// Side-barrier protection while camera avoidance owns the steering.
#define OBSTACLE_WALL_GUARD_DISTANCE_MM 170.0f
#define OBSTACLE_WALL_GUARD_KP 0.12f
#define OBSTACLE_WALL_GUARD_MAX_STEERING 18.0f

// A block must extend this far down in the logical 320x240 image.
#define OBSTACLE_MIN_BOTTOM_Y 100

// Upright WRO obstacle blocks are taller than they are wide. This rejects
// broad greenish background regions and blobs merged with the horizon.
#define OBSTACLE_MAX_WIDTH_HEIGHT_RATIO 1.25f

// A new manoeuvre may only start from a reasonably complete pillar. Once a
// pillar is confirmed, the separate tracking rules remain tolerant at edges.
#define OBSTACLE_START_MIN_X 30
#define OBSTACLE_START_MAX_X 290
#define OBSTACLE_MAX_START_WIDTH 80
#define OBSTACLE_MAX_START_HEIGHT 120

// After every 90 degree corner, let the normal gyro controller remove turn
// overshoot before camera avoidance can take steering priority.
#define OBSTACLE_CORNER_SETTLE_MIN_DISTANCE_MM 100.0f
#define OBSTACLE_CORNER_SETTLE_MAX_DISTANCE_MM 300.0f
#define OBSTACLE_CORNER_SETTLE_HEADING_DEG 4.0f



// ==========================================
// CHALLENGE MODE
// ==========================================

enum ChallengeMode : uint8_t
{
    CHALLENGE_OPEN,
    CHALLENGE_OBSTACLE
};

// Change only this line before a run:
constexpr ChallengeMode CHALLENGE_MODE =
    CHALLENGE_OBSTACLE;
