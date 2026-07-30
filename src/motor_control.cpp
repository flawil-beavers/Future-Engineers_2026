/**
 * @file motor_control.cpp
 * @brief Motor control subsystem implementation
 */

#include "motor_control.h"
#include "config.h"
#include "navigation_controller.h"
#include "mode_manager.h"
#include "logger.h"
#define Serial robot_logger

extern void serial_setup();

// ==========================================
// MOTOR CONTROL STATE VARIABLES
// ==========================================

Servo servo;

// Encoder position tracking
long encoder_pos = 0;
int encoder_dir = 1; // 1 -> CCW, -1 -> CW

// Motor state
DCState dc_state = DC_DISABLED;
float dc_current_dc = 0;
int target_speed = 0;
float current_speed = 0;
float last_speed = 0; // Used to store speed before stopping, for resuming

// Steering state
int set_degree = 0;
bool servo_disabled = false;

// PID state
float target_distance = 0;
float current_distance = 0;
float last_distance = 0;

// PID tuning parameters
float Kp = PID_KP;
float Ki = PID_KI;
float Kd = PID_KD;
float i_max = PID_I_MAX;
float pid_integral = 0.0;
float last_error = 0.0;

// Debug variables
int dc_out = 0;
float pid_before_checking = 0;

// Timing variables
float last_loop_time = 0; // in s
float acc = DEFAULT_ACCELERATION;

// Time tracking
unsigned long current_time = 0;
static unsigned long last_time = 0;
unsigned long last_pid_status_time = 0;
unsigned long steering_diff = 0;

// Enable switch state management
bool system_enabled = false;           // Whether system is currently running, otherwise no movement is done
static bool last_physical_switch_state = false; // Tracks physical switch to detect transitions

// ==========================================
// MOTOR CONTROL FUNCTIONS
// ==========================================

void steer(int angle)
{
  static int servo_last_command = 1000;
  if (angle == servo_last_command)
  {
    return; // Skip unnecessary writes
  }
  if (servo_disabled)
  {
    return;
  }

  int commanded_angle = angle;
  angle = commanded_angle + SERVO_CENTER;

  if (angle > SERVO_MAX_ANGLE)
  {
    angle = SERVO_MAX_ANGLE;
  }
  else if (angle < SERVO_MIN_ANGLE)
  {
    angle = SERVO_MIN_ANGLE;
  }

  servo.write(angle);
  servo_last_command = commanded_angle;
}

void set_steering(int angle)
{
  set_degree = angle;
}

void set_dc(float dc)
{
  if (dc_state == DC_DISABLED || fabs(dc) < MOTOR_MIN_DC)
  {
    digitalWrite(MOTOR_IN1_PIN, LOW);
    digitalWrite(MOTOR_IN2_PIN, LOW);
    dc_current_dc = dc;
    return;
  }

  // Limit maximum duty cycle
  if (dc != 0 && fabs(dc) > MOTOR_MAX_DC)
  {
    dc = MOTOR_MAX_DC * (dc / fabs(dc));
  }

  // Rate-limit acceleration
  if (dc > dc_current_dc + MOTOR_MAX_ACC_DC * last_loop_time)
  {
    dc = dc_current_dc + MOTOR_MAX_ACC_DC * last_loop_time;
  }
  else if (dc < dc_current_dc - MOTOR_MAX_ACC_DC * last_loop_time)
  {
    dc = dc_current_dc - MOTOR_MAX_ACC_DC * last_loop_time;
  }

  dc_out = fabs(dc);
  analogWrite(MOTOR_PWM_PIN, dc_out);

  if (dc > 0)
  {
    digitalWrite(MOTOR_IN1_PIN, LOW);
    digitalWrite(MOTOR_IN2_PIN, HIGH);
  }
  else if (dc < 0)
  {
    digitalWrite(MOTOR_IN1_PIN, HIGH);
    digitalWrite(MOTOR_IN2_PIN, LOW);
  }

  dc_current_dc = dc;
}

float get_distance(long encoder_pos)
{
  return encoder_pos * COUNTER_TO_MM;
}

int estimate_dc(float speed)
{
  float distance = get_distance(encoder_pos);
  float dc = speed / distance * MOTOR_MAX_DC;
  if (dc > MOTOR_MAX_DC)
  {
    dc = MOTOR_MAX_DC;
  }
  else if (dc < MOTOR_MIN_DC)
  {
    dc = MOTOR_MIN_DC;
  }
  return dc;
}

void pid_speed()
{
  if (last_loop_time == 0)
  {
    return;
  }

  float error = target_distance - current_distance;
  pid_integral += error * last_loop_time;
  pid_before_checking = pid_integral;

  // Clamp integral term
  if (pid_integral != 0 && fabs(pid_integral) > i_max)
  {
    pid_integral = i_max * (pid_integral / fabs(pid_integral));
  }

  float speed = Kp * error + Ki * pid_integral + Kd * (error - last_error) / last_loop_time;
  if (dc_state == DC_HOLDING)
  {
    speed = constrain(speed, -HOLD_MAX_DC, HOLD_MAX_DC);
  }
  set_dc(speed);
  last_error = error;
}

void drive_loop()
{
  // Steering is also serviced while the drive motor is disabled. This is
  // required for the stationary obstacle bench test.
  steer(set_degree);

  if (last_loop_time == 0 || dc_state == DC_DISABLED)
  {
    return; // Don't run until timing is initialized
  }

  if (dc_state == DC_ENABLED)
  {
    // Smooth acceleration
    if (fabs(target_speed - current_speed) > 1)
    {
      current_speed += (target_speed - current_speed) / fabs(target_speed - current_speed) * acc * last_loop_time;
    }
    else
    {
      current_speed = target_speed;
    }

    target_distance += current_speed * last_loop_time;
  }

  pid_speed();
}

void set_acceleration(int acceleration)
{
  if (acceleration < 0)
  {
    acceleration = 0;
  }
  acc = acceleration;
}

void stop(bool hold)
{
  last_speed = current_speed;
  if (!hold)
  {
    dc_state = DC_DISABLED;
    pid_integral = 0;
    last_error = 0;
    target_distance = current_distance;
    set_dc(0);
  }
  else
  {
    dc_state = DC_HOLDING;
    // Lock to current distance to hold position
    target_distance = current_distance;
    pid_integral = 0;
    last_error = 0;
  }
  servo_disabled = true;
}

void set_speed(int speed)
{
  if (speed == -1)
  {
    speed = last_speed;
  }
  dc_state = DC_ENABLED;
  target_speed = speed;
  last_speed = speed;
}

void loop_updater()
{
  static unsigned long last_loop_time_us = 0;

  last_time = current_time;
  last_distance = current_distance;

  current_time = micros();
  last_loop_time_us = current_time - last_time;
  last_loop_time = last_loop_time_us / 1000000.0; // Convert to seconds

  current_distance = get_distance(encoder_pos);
}

void check_stalling()
{
  static unsigned long window_start_us = 0;
  static float window_start_distance = 0;
  static unsigned long hold_overload_start_us = 0;

  if (dc_state == DC_HOLDING)
  {
    const bool holding_at_limit =
        fabsf(dc_current_dc) >=
        HOLD_MAX_DC * HOLD_OVERLOAD_THRESHOLD;

    if (!holding_at_limit)
    {
      hold_overload_start_us = 0;
      return;
    }

    if (hold_overload_start_us == 0)
    {
      hold_overload_start_us = current_time;
      return;
    }

    if (current_time - hold_overload_start_us >=
        HOLD_OVERLOAD_WINDOW_US)
    {
      Serial.println(
          "Holding overload: maximum holding effort exceeded for 2 seconds.");
      hold_overload_start_us = 0;
      mode_pause();
    }
    return;
  }

  hold_overload_start_us = 0;

  // DC_HOLDING intentionally produces torque at almost zero speed. It needs a
  // separate overload policy and must not be interpreted as a driving stall.
  const bool high_drive_load =
      dc_state == DC_ENABLED &&
      fabs(dc_current_dc) >
          MOTOR_MAX_DC * STALL_DC_THRESHOLD;

  if (!high_drive_load)
  {
    window_start_us = current_time;
    window_start_distance = current_distance;
    return;
  }

  if (window_start_us == 0)
  {
    window_start_us = current_time;
    window_start_distance = current_distance;
    return;
  }

  const unsigned long elapsed_us =
      current_time - window_start_us;
  if (elapsed_us < STALL_DETECTION_WINDOW_US)
    return;

  const float elapsed_s = elapsed_us / 1000000.0f;
  const float speed_mms =
      fabsf(current_distance - window_start_distance) /
      elapsed_s;

  window_start_us = current_time;
  window_start_distance = current_distance;

  if (speed_mms >= STALL_SPEED_THRESHOLD_MMS)
    return;

  Serial.print("Stall detected over ");
  Serial.print(elapsed_us / 1000);
  Serial.print(" ms. Speed: ");
  Serial.print(speed_mms, 2);
  Serial.print(" mm/s, DC: ");
  Serial.println(dc_current_dc);

  mode_pause();
}

void system_enable()
{
  system_enabled = true;
  dc_state = DC_ENABLED;
  servo_disabled = false;

  Serial.println("SYSTEM ENABLED");
}

void system_disable()
{
  system_enabled = false;

  stop(false); 
  // navigation_disable();

  Serial.println("SYSTEM DISABLED");
  robot_logger.write_to_usb();
}


void system_interface_setup()
{
  // Configure enable switch
  pinMode(ENABLE_SWITCH_PIN, INPUT);

  // Check initial state
  system_enabled = digitalRead(ENABLE_SWITCH_PIN);
  last_physical_switch_state = system_enabled;

  // Setup serial and wait if not enabled (defined in serial_handler.cpp)
  serial_setup();

  if (!system_enabled)
  {
    Serial.println("Enable switch is LOW - System starting in IDLE mode");
  }
  else
  {
    Serial.println("Enable switch is HIGH - System starting in ENABLED mode");
  }
}

void handle_enable_switch()
{
  // Throttling: Only poll the physical pin every 50ms (20Hz).
  // Human-operated switches don't need MHz-rate polling, and this saves CPU cycles.
  static unsigned long last_poll_time = 0;
  if (current_time - last_poll_time < ENABLE_SWITCH_POLL_INTERVAL_US) return;
  last_poll_time = current_time;

  bool current_switch_state = digitalRead(ENABLE_SWITCH_PIN);
  
  // If the switch state matches our internal state, no action is needed
  if (current_switch_state == last_physical_switch_state) return;
  last_physical_switch_state = current_switch_state;

  // Check if enough time has passed since the last transition to debounce the signal
  static unsigned long last_enable_interrupt_time = 0;
  if (current_time - last_enable_interrupt_time > ENABLE_DEBOUNCE_TIME_US)
  {
    last_enable_interrupt_time = current_time;
    if (current_switch_state)
      mode_resume();
    else
      mode_pause();
  }
}

// ==========================================
// ENCODER INTERRUPT HANDLERS
// ==========================================

/**
 * @brief Internal helper to update encoder position and direction.
 * 
 * Compares the state of both encoder phases to determine the rotation direction.
 * 
 * @param encoderPin The pin that triggered the interrupt
 */
void update_encoder(int encoderPin)
{
  int a = digitalRead(ENCODER_PIN_A);
  int b = digitalRead(ENCODER_PIN_B);

  if ((a == b && encoderPin == ENCODER_PIN_A) || (a != b && encoderPin == ENCODER_PIN_B))
  {
    encoder_dir = 1;
  }
  else
  {
    encoder_dir = -1;
  }

  encoder_pos += encoder_dir;
}

/**
 * @brief ISR for Encoder Phase A transitions.
 */
void update_encoder_a()
{
  update_encoder(ENCODER_PIN_A);
}

/**
 * @brief ISR for Encoder Phase B transitions.
 */
void update_encoder_b()
{
  update_encoder(ENCODER_PIN_B);
}

// ==========================================
// INITIALIZATION
// ==========================================

void motor_control_setup()
{
  // Configure motor control pins
  pinMode(MOTOR_IN1_PIN, OUTPUT);
  pinMode(MOTOR_IN2_PIN, OUTPUT);
  pinMode(MOTOR_PWM_PIN, OUTPUT);

  // Configure servo
  servo.attach(SERVO_PIN);

  // Configure encoder pins
  pinMode(ENCODER_PIN_A, INPUT);
  pinMode(ENCODER_PIN_B, INPUT);

  // Initialize motor outputs to off
  digitalWrite(MOTOR_IN1_PIN, LOW);
  digitalWrite(MOTOR_IN2_PIN, LOW);
  analogWrite(MOTOR_PWM_PIN, 0);

  // Attach encoder interrupts
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), update_encoder_a, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), update_encoder_b, CHANGE);

  // Initialize motors
  set_speed(0);
  set_steering(0);
  
  // Initialize timing for the loop_updater and stall protection
  current_time = micros();
  last_time = current_time;
}
