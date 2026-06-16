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

/**
 * @brief Operational states for the DC motor
 */
enum DCState {
  DC_DISABLED,  /**< Motor is powered off */
  DC_ENABLED,   /**< Motor is active and following target speed */
  DC_HOLDING    /**< Motor is active and holding current position */
};

extern Servo servo;

// Encoder position tracking
extern long encoder_pos;
extern int encoder_dir;

// Motor state
extern DCState dc_state;
extern float dc_current_dc;
extern int target_speed;
extern float current_speed;

// Steering state
extern int set_degree;
extern bool servo_disabled;
extern unsigned long steering_diff;

// PID state
extern float target_distance;
extern float current_distance;
extern float last_distance;

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

// Enable switch state management
extern bool system_enabled;           // Whether system is currently running

// ==========================================
// MOTOR CONTROL FUNCTIONS
// ==========================================

/**
 * @brief Set the steering angle of the servo
 * @param angle Target steering angle in degrees (-60 to +60 relative to SERVO_CENTER)
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
 * @param encoder_pos Encoder position in counts (defaults to global encoder_pos)
 * @return Distance in millimeters
 */
float get_distance(long encoder_pos = encoder_pos);

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
 * @param hold If true, transition to DC_HOLDING; if false, transition to DC_DISABLED
 */
void stop(bool hold = false);

/**
 * @brief Set motor speed
 * @param speed Target speed in mm/s. Use -1 to resume using the last recorded speed.
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
 * @brief Handle the enable switch on port A2
 * - HIGH (3.3V): Enable/resume the program
 * - LOW (GND): Disable/stop the motors but preserve state
 */
void handle_enable_switch();

/**
 * @brief Initialize the system interface (Enable switch and Serial)
 * Configures the enable switch pin, initial state, and waits for serial if necessary.
 */
void system_interface_setup();

/**
 * @brief Fully enable the robot system
 * Synchronizes states, enables motors/servos, and starts autonomous behavior
 */
void system_enable();

/**
 * @brief Fully disable the robot system
 * Stops motors, centers steering, and disables autonomous behavior
 */
void system_disable();

// ==========================================
// INITIALIZATION
// ==========================================

/**
 * @brief Initialize motor control subsystem
 * Sets up pins, servo, encoders, and interrupt handlers
 */
void motor_control_setup();
