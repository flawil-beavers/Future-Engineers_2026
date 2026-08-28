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

enum DriveControlPhase {
  DRIVE_ACCELERATING,
  DRIVE_CRUISING,
  DRIVE_DECELERATING,
  DRIVE_POSITION_HOLD
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
extern float measured_speed;
extern unsigned long speed_measurement_count;
extern float current_acceleration;
extern float commanded_acceleration;
extern float measured_acceleration;
extern float active_acceleration_limit;
extern DriveControlPhase drive_control_phase;

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
extern float accel_Kp;
extern float accel_Ki;
extern float motor_static_ff;
extern float motor_speed_ff;
extern float motor_accel_ff;
extern float active_cruise_kp;
extern float active_cruise_ki;
extern float low_speed_cruise_kp;
extern float low_speed_cruise_ki;
extern float mid_speed_cruise_kp;
extern float mid_speed_cruise_ki;
extern float low_speed_gain_end;
extern float mid_speed_gain_end;
extern float high_speed_gain_start;
extern float i_max;
extern float pid_integral;
extern float accel_pid_integral;
extern float last_error;

// Timing variables
extern float last_loop_time; // Last loop time in s
extern float acc;
extern float last_speed;

// Debug variables
extern int dc_out;
extern float pid_before_checking;
extern float low_speed_load_compensation_dc;

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
 * @brief Queue a steering command by turning radius.
 *
 * Converts the desired radius to a servo angle via the CAD Ackermann model
 * (include/ackermann_kinematics.h) and calls set_steering() internally.
 *
 * Sign convention:
 *   radius_mm > 0  → right turn   (same as positive servo angle)
 *   radius_mm < 0  → left  turn
 *   |radius_mm| > 2000 → straight (set_steering(0))
 *
 * @param radius_mm Desired turning radius in mm.
 */
void set_steering_radius(float radius_mm);

/**
 * @brief Set the duty cycle of the DC motor
 * @param dc Duty cycle value (-255 to +255)
 */
void set_dc(float dc, bool rate_limit = true);

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
 * @brief Two-phase PI speed loop with velocity/acceleration feedforward
 * Uses acceleration gains while following the motion profile and cruise
 * gains once the requested speed is stable.
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
