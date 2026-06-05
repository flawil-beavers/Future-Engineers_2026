#pragma once

/**
 * @file motor_control.h
 * @brief Motor control subsystem: DC motor, servo steering, and PID speed controller
 */

#include <Arduino.h>
#include <Servo.h>

// ==========================================
// MOTOR CONTROL STATE VARIABLES
// ==========================================

extern Servo servo;

// Encoder position tracking
extern long encoder_pos;
extern int encoder_dir;

// Motor state
extern bool disable_dc;
extern bool hold_dc;
extern float current_dc;
extern int target_speed;
extern float current_speed;

// Steering state
extern int set_degree;
extern bool disable_servo;
extern unsigned long steering_diff;

// PID state
extern float target_distance;
extern float current_distance;
extern float last_distance;
extern float measured_speed;

// PID tuning parameters
extern float Kp;
extern float Ki;
extern float Kd;
extern float i_max;
extern float pid_integral;
extern float last_error;

// Timing variables
extern float last_loop_time; // Last loop time in s
extern float acc;
extern float last_speed;

// Debug variables
extern int dc_out;
extern float pid_before_checking;

// ==========================================
// MOTOR CONTROL FUNCTIONS
// ==========================================

/**
 * @brief Set the steering angle of the servo
 * @param angle Angle in degrees (-60 to +60 degrees from center)
 */
void steer(int angle);

/**
 * @brief Queue a steering command
 * @param angle Angle in degrees
 */
void set_steering(int angle);

/**
 * @brief Set the duty cycle of the DC motor
 * @param dc Duty cycle value (-255 to +255)
 */
void set_dc(float dc);

/**
 * @brief Calculate distance from encoder position
 * @param encoder_pos Encoder position in counts
 * @return Distance in millimeters
 */
float get_distance(long encoder_pos);

/**
 * @brief Estimate DC value needed for a given speed
 * @param speed Desired speed in mm/s
 * @return Estimated duty cycle
 */
int estimate_dc(float speed);

/**
 * @brief PID-controlled speed loop
 * Calculates DC value based on error between target and current distance
 */
void pid_speed();

/**
 * @brief Main drive loop - handles acceleration and steering
 * Should be called every main loop iteration
 */
void drive_loop();

/**
 * @brief Set the acceleration rate
 * @param acceleration Acceleration in mm/s^2
 */
void set_acceleration(int acceleration);

/**
 * @brief Stop motors and optionally hold position
 * @param hold If true, hold position; if false, disable motors
 */
void stop(bool hold = false);

/**
 * @brief Set motor speed
 * @param speed Speed in mm/s (or use last speed if not provided)
 */
void set_speed(int speed = -1);

/**
 * @brief Update loop timing and distance tracking
 * Call at the beginning of each main loop
 */
void loop_updater();

/**
 * @brief Check for motor stalling
 * Stops the robot if stalling is detected
 */
void check_stalling();

/**
 * @brief Handle enable/disable interrupt
 * Toggles motor and servo enable/disable state
 */
void enable_interrupt();

// ==========================================
// INITIALIZATION
// ==========================================

/**
 * @brief Initialize motor control subsystem
 * Sets up pins, servo, encoders, and interrupt handlers
 */
void motor_control_setup();
