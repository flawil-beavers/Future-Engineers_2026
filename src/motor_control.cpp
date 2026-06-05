/**
 * @file motor_control.cpp
 * @brief Motor control subsystem implementation
 */

#include "motor_control.h"
#include "config.h"
#include "wall_follower.h"

// ==========================================
// MOTOR CONTROL STATE VARIABLES
// ==========================================

Servo servo;

// Encoder position tracking
long encoder_pos = 0;
int encoder_dir = 1; // 1 -> CCW, -1 -> CW

// Motor state
bool disable_dc = false;
bool hold_dc = false;
float current_dc = 0;
int target_speed = 0;
float current_speed = 0;

// Steering state
int set_degree = 0;
bool disable_servo = false;
int last_angle = 0;

// PID state
float target_distance = 0;
float current_distance = 0;
float last_distance = 0;
float measured_speed = 0;

// PID tuning parameters
float Kp = PID_KP;
float Ki = PID_KI;
float Kd = PID_KD;
float i_max = PID_I_MAX;
float pid_integral = 0.0;
float last_error = 0.0;

// Timing variables
float last_loop_time = 0; // in s
float acc = DEFAULT_ACCELERATION;
float last_speed = 0;

// Debug variables
int dc_out = 0;
float pid_before_checking = 0;

// Time tracking
unsigned long last_time = 0;
unsigned long current_time = 0;
unsigned long last_status_time = 0;
unsigned long last_loop_time_us = 0;
unsigned long last_enable_interrupt_time = 0;
unsigned long stall_encoder_pos = 0;
unsigned long last_steering_command = 0;
unsigned long steering_diff = 0;

// Enable switch state management
bool system_enabled = false;           // Whether system is currently running
static bool last_physical_switch_state = false; // Tracks physical switch to detect transitions

// ==========================================
// MOTOR CONTROL FUNCTIONS
// ==========================================

void steer(int angle)
{
  if (angle == last_angle)
  {
    return; // Skip unnecessary writes
  }
  if (disable_servo)
  {
    return;
  }

  angle = angle + SERVO_CENTER;

  if (angle > SERVO_MAX_ANGLE)
  {
    angle = SERVO_MAX_ANGLE;
  }
  else if (angle < SERVO_MIN_ANGLE)
  {
    angle = SERVO_MIN_ANGLE;
  }

  servo.write(angle);
  last_angle = angle;
}

void set_steering(int angle)
{
  set_degree = angle;
}

void set_dc(float dc)
{
  if (disable_dc || fabs(dc) < MOTOR_MIN_DC)
  {
    digitalWrite(MOTOR_IN1_PIN, LOW);
    digitalWrite(MOTOR_IN2_PIN, LOW);
    current_dc = dc;
    return;
  }

  // Limit maximum duty cycle
  if (dc != 0 && fabs(dc) > MOTOR_MAX_DC)
  {
    dc = MOTOR_MAX_DC * (dc / fabs(dc));
  }

  // Rate-limit acceleration
  if (dc > current_dc + MOTOR_MAX_ACC_DC * last_loop_time)
  {
    dc = current_dc + MOTOR_MAX_ACC_DC * last_loop_time;
  }
  else if (dc < current_dc - MOTOR_MAX_ACC_DC * last_loop_time)
  {
    dc = current_dc - MOTOR_MAX_ACC_DC * last_loop_time;
  }

  dc_out = fabs(dc);
  analogWrite(MOTOR_ENA_PIN, dc_out);

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

  current_dc = dc;
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
  set_dc(speed);
  last_error = error;
}

void drive_loop()
{
  if (last_loop_time == 0)
  {
    return; // Don't run until timing is initialized
  }

  steer(set_degree);

  if (!hold_dc)
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

    if (!disable_dc)
    {
      target_distance += current_speed * last_loop_time;
    }
  }
  else if (disable_dc)
  {
    return;
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
    disable_dc = true;
    pid_integral = 0;
    last_error = 0;
    hold_dc = false;
  }
  else
  {
    disable_dc = false;
    hold_dc = true;
  }
  disable_servo = true;
}

void set_speed(int speed)
{
  if (speed == -1)
  {
    speed = last_speed;
  }
  hold_dc = false;
  target_speed = speed;
  last_speed = speed;
}

void loop_updater()
{
  last_time = current_time;
  last_distance = current_distance;

  current_time = micros();
  last_loop_time_us = current_time - last_time;
  last_loop_time = last_loop_time_us / 1000000.0; // Convert to seconds

  current_distance = get_distance(encoder_pos);
}

void check_stalling()
{
  // Prevent division by zero and only check if enough time has passed
  if (last_loop_time > 0.00001 && (float)fabs(stall_encoder_pos - encoder_pos)/last_loop_time < STALL_THRESHOLD_COUNTS &&
      fabs(current_dc) > MOTOR_MAX_DC * STALL_DC_THRESHOLD &&
      !disable_dc)
  {
    Serial.print("Stall detected, stopping robot: diff_distance:");
    Serial.print(fabs(stall_encoder_pos - encoder_pos));
    Serial.print(", current_dc: ");
    Serial.println(current_dc);

    stop();
    current_speed = 0;
    target_distance = current_distance;
  }

  stall_encoder_pos = encoder_pos;
}

void enable_interrupt()
{
  if (current_time - last_enable_interrupt_time < ENABLE_DEBOUNCE_TIME_US)
  {
    return;
  }

  last_enable_interrupt_time = current_time;

  // Toggle enable state
  bool en_state = !(disable_dc || disable_servo);
  en_state = !en_state;

  if (en_state)
  {
    disable_dc = false;
    disable_servo = false;
    set_speed();
    Serial.println(EN_STATE_TRUE_MSG);
  }
  else
  {
    stop();
    Serial.println(EN_STATE_FALSE_MSG);
  }
}

/**
 * @brief Handle the enable switch on port A2
 * - HIGH (3.3V): Enable/resume the program
 * - LOW (GND): Disable/stop the motors but preserve state
 */
void handle_enable_switch()
{
  bool current_switch_state = digitalRead(ENABLE_SWITCH_PIN);

  if (current_switch_state && !last_physical_switch_state)
  {
    // Switch physically toggled ON
    last_physical_switch_state = true;
    system_enabled = true;
    disable_dc = false;
    disable_servo = false;
    
    // Sync state to prevent jumps and immediate stalls
    current_time = micros();
    last_time = current_time;
    current_distance = get_distance(encoder_pos);
    target_distance = current_distance;
    pid_integral = 0;
    stall_encoder_pos = encoder_pos;
    
    Serial.println("ENABLE SWITCH: ON - Robot enabled");
    wall_follower_enable();
  }
  else if (!current_switch_state && last_physical_switch_state)
  {
    // Switch physically toggled OFF
    last_physical_switch_state = false;
    system_enabled = false;
    stop(false); // Disable flags
    set_dc(0);   // Kill power to motors immediately
    steer(0);    // Center steering
    Serial.println("ENABLE SWITCH: OFF - Robot paused");
  }
}


// ==========================================
// ENCODER INTERRUPT HANDLERS
// ==========================================

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

void update_encoder_a()
{
  update_encoder(ENCODER_PIN_A);
}

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
  pinMode(MOTOR_ENA_PIN, OUTPUT);

  // Configure servo
  servo.attach(SERVO_PIN);

  // Configure encoder pins
  pinMode(ENCODER_PIN_A, INPUT);
  pinMode(ENCODER_PIN_B, INPUT);

  // Initialize motor outputs to off
  digitalWrite(MOTOR_IN1_PIN, LOW);
  digitalWrite(MOTOR_IN2_PIN, LOW);
  analogWrite(MOTOR_ENA_PIN, 0);

  // Attach encoder interrupts
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), update_encoder_a, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), update_encoder_b, CHANGE);

  // Initialize motor state
  disable_dc = !system_enabled;
  disable_servo = !system_enabled;
  last_physical_switch_state = system_enabled;
  set_speed(0);
  
  // Initialize timing and stall protection to current state
  current_time = micros();
  last_time = current_time;
  current_distance = get_distance(encoder_pos);
  target_distance = current_distance;
  stall_encoder_pos = encoder_pos;
}
