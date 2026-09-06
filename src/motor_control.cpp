/**
 * @file motor_control.cpp
 * @brief Motor control subsystem implementation
 */

#include "motor_control.h"
#include "ackermann_kinematics.h"
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
float measured_speed = 0;
unsigned long speed_measurement_count = 0;
float current_acceleration = 0;
float commanded_acceleration = 0;
float measured_acceleration = 0;
float active_acceleration_limit = DEFAULT_ACCELERATION;
DriveControlPhase drive_control_phase = DRIVE_CRUISING;
static bool speed_measurement_ready = false;
static float speed_measurement_dt = 0.05f;
float last_speed = 0; // Used to store speed before stopping, for resuming

// Steering state
int set_degree = 0;
bool servo_disabled = false;

// PID state
float target_distance = 0;
float current_distance = 0;
float last_distance = 0;

// PID tuning parameters
float Kp = CRUISE_KP;
float Ki = CRUISE_KI;
float Kd = 0.0f;
float accel_Kp = ACCEL_KP;
float accel_Ki = ACCEL_KI;
float motor_static_ff = MOTOR_STATIC_FF_DC;
float motor_speed_ff = MOTOR_SPEED_FF_DC_PER_MMS;
float motor_accel_ff = MOTOR_ACCEL_FF_DC_PER_MMSS;
float active_cruise_kp = LOW_SPEED_CRUISE_KP;
float active_cruise_ki = LOW_SPEED_CRUISE_KI;
float low_speed_cruise_kp = LOW_SPEED_CRUISE_KP;
float low_speed_cruise_ki = LOW_SPEED_CRUISE_KI;
float mid_speed_cruise_kp = MID_SPEED_CRUISE_KP;
float mid_speed_cruise_ki = MID_SPEED_CRUISE_KI;
float low_speed_gain_end = LOW_SPEED_GAIN_END_MMS;
float mid_speed_gain_end = MID_SPEED_GAIN_END_MMS;
float high_speed_gain_start = HIGH_SPEED_GAIN_START_MMS;
float i_max = SPEED_INTEGRAL_PWM_MAX;
float pid_integral = 0.0;
float accel_pid_integral = 0.0f;
float last_error = 0.0;
static float hold_pid_integral = 0.0f;
static DriveControlPhase last_applied_drive_phase = DRIVE_CRUISING;
static unsigned long cruise_candidate_start_us = 0;

namespace
{
struct NoProgressWatchdogState
{
  bool armed = false;
  uint32_t window_start_us = 0;
  float window_start_distance_mm = 0.0f;
  int8_t command_direction = 0;
};

struct NoProgressWatchdogEvidence
{
  uint32_t elapsed_us = 0;
  float directional_progress_mm = 0.0f;
};

NoProgressWatchdogState no_progress_watchdog;
bool no_progress_watchdog_preflight_passed = false;

void reset_no_progress_watchdog(
    NoProgressWatchdogState &state,
    uint32_t now_us,
    float distance_mm)
{
  state.armed = false;
  state.window_start_us = now_us;
  state.window_start_distance_mm = distance_mm;
  state.command_direction = 0;
}

bool update_no_progress_watchdog(
    NoProgressWatchdogState &state,
    bool drive_enabled,
    int commanded_speed_mms,
    float profile_speed_mms,
    uint32_t now_us,
    float distance_mm,
    NoProgressWatchdogEvidence &evidence)
{
  evidence.elapsed_us = 0;
  evidence.directional_progress_mm = 0.0f;

  const bool profile_matches_command =
      commanded_speed_mms * profile_speed_mms > 0.0f;
  const bool motion_command_active =
      drive_enabled &&
      abs(commanded_speed_mms) >= STALL_COMMAND_MIN_SPEED_MMS &&
      fabsf(profile_speed_mms) >= STALL_COMMAND_MIN_SPEED_MMS &&
      profile_matches_command;
  if (!motion_command_active)
  {
    reset_no_progress_watchdog(state, now_us, distance_mm);
    return false;
  }

  const int8_t direction = commanded_speed_mms > 0 ? 1 : -1;
  if (!state.armed || state.command_direction != direction)
  {
    state.armed = true;
    state.window_start_us = now_us;
    state.window_start_distance_mm = distance_mm;
    state.command_direction = direction;
    return false;
  }

  evidence.elapsed_us = now_us - state.window_start_us;
  evidence.directional_progress_mm =
      direction * (distance_mm - state.window_start_distance_mm);

  if (evidence.directional_progress_mm >=
      STALL_NO_PROGRESS_MIN_DISTANCE_MM)
  {
    state.window_start_us = now_us;
    state.window_start_distance_mm = distance_mm;
    return false;
  }

  return evidence.elapsed_us >= STALL_NO_PROGRESS_WINDOW_US;
}

bool no_progress_watchdog_preflight()
{
  NoProgressWatchdogEvidence evidence;

  // A sustained motion command without progress must expire at the deadline.
  NoProgressWatchdogState stalled;
  if (update_no_progress_watchdog(
          stalled, true, 175, 175.0f, 100U, 0.0f, evidence) ||
      update_no_progress_watchdog(
          stalled,
          true,
          175,
          175.0f,
          100U + STALL_NO_PROGRESS_WINDOW_US - 1U,
          0.0f,
          evidence) ||
      !update_no_progress_watchdog(
          stalled,
          true,
          175,
          175.0f,
          100U + STALL_NO_PROGRESS_WINDOW_US,
          0.0f,
          evidence))
    return false;

  // Meaningful forward progress resets the deadline.
  NoProgressWatchdogState progressing;
  if (update_no_progress_watchdog(
          progressing, true, 175, 175.0f, 200U, 0.0f, evidence) ||
      update_no_progress_watchdog(
          progressing,
          true,
          175,
          175.0f,
          200U + STALL_NO_PROGRESS_WINDOW_US / 2U,
          STALL_NO_PROGRESS_MIN_DISTANCE_MM,
          evidence) ||
      update_no_progress_watchdog(
          progressing,
          true,
          175,
          175.0f,
          200U + STALL_NO_PROGRESS_WINDOW_US,
          STALL_NO_PROGRESS_MIN_DISTANCE_MM,
          evidence))
    return false;

  // A planned zero-speed hold disarms the timer rather than expiring it.
  NoProgressWatchdogState stopped;
  if (update_no_progress_watchdog(
          stopped, true, 175, 175.0f, 300U, 0.0f, evidence) ||
      update_no_progress_watchdog(
          stopped,
          true,
          0,
          0.0f,
          300U + STALL_NO_PROGRESS_WINDOW_US * 2U,
          0.0f,
          evidence) ||
      stopped.armed)
    return false;

  // Reverse encoder motion cannot satisfy a forward progress requirement.
  NoProgressWatchdogState rebounding;
  if (update_no_progress_watchdog(
          rebounding, true, 175, 175.0f, 400U, 0.0f, evidence) ||
      !update_no_progress_watchdog(
          rebounding,
          true,
          175,
          175.0f,
          400U + STALL_NO_PROGRESS_WINDOW_US,
          -STALL_NO_PROGRESS_MIN_DISTANCE_MM,
          evidence))
    return false;

  return true;
}
} // namespace

// Debug variables
int dc_out = 0;
float pid_before_checking = 0;
float low_speed_load_compensation_dc = 0.0f;
bool low_speed_pulse_density_active = false;
uint32_t low_speed_pulse_density_slots = 0;
uint32_t low_speed_pulse_density_powered_slots = 0;
static bool low_speed_load_compensation_logged = false;
static float low_speed_pulse_density_accumulator = 0.0f;
static int8_t low_speed_pulse_density_direction = 0;
static uint32_t low_speed_pulse_density_last_slot_us = 0;

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

void set_steering_radius(float radius_mm)
{
  float angle = Ackermann::getServoAngleForRadius(radius_mm);
  set_steering((int)roundf(angle));
}

static void apply_motor_output(float applied_dc)
{
  dc_out = (int)roundf(fabsf(applied_dc));
  analogWrite(MOTOR_PWM_PIN, dc_out);

  if (applied_dc > 0.0f)
  {
    digitalWrite(MOTOR_IN1_PIN, LOW);
    digitalWrite(MOTOR_IN2_PIN, HIGH);
  }
  else if (applied_dc < 0.0f)
  {
    digitalWrite(MOTOR_IN1_PIN, HIGH);
    digitalWrite(MOTOR_IN2_PIN, LOW);
  }
  else
  {
    digitalWrite(MOTOR_IN1_PIN, LOW);
    digitalWrite(MOTOR_IN2_PIN, LOW);
  }
}

static void reset_low_speed_pulse_density()
{
  low_speed_pulse_density_active = false;
  low_speed_pulse_density_accumulator = 0.0f;
  low_speed_pulse_density_direction = 0;
  low_speed_pulse_density_last_slot_us = current_time;
}

static void service_motor_output()
{
  if (dc_state == DC_DISABLED || fabsf(dc_current_dc) < 0.5f)
  {
    reset_low_speed_pulse_density();
    apply_motor_output(0.0f);
    return;
  }

  const int8_t requested_direction = dc_current_dc > 0.0f ? 1 : -1;
  const bool pulse_density_eligible =
      dc_state == DC_ENABLED && target_speed != 0 &&
      dc_current_dc * target_speed > 0.0f &&
      fabsf((float)target_speed) <=
          LOW_SPEED_PULSE_DENSITY_MAX_TARGET_MMS &&
      fabsf(dc_current_dc) < LOW_SPEED_PULSE_DENSITY_CARRIER_DC;
  if (pulse_density_eligible)
  {
    if (!low_speed_pulse_density_active ||
        requested_direction != low_speed_pulse_density_direction)
    {
      low_speed_pulse_density_accumulator = 0.0f;
      low_speed_pulse_density_direction = requested_direction;
      low_speed_pulse_density_last_slot_us = current_time -
          LOW_SPEED_PULSE_DENSITY_SLOT_US;
    }
    low_speed_pulse_density_active = true;

    if (current_time - low_speed_pulse_density_last_slot_us <
        LOW_SPEED_PULSE_DENSITY_SLOT_US)
      return;

    low_speed_pulse_density_last_slot_us = current_time;
    ++low_speed_pulse_density_slots;
    low_speed_pulse_density_accumulator += fabsf(dc_current_dc);
    if (low_speed_pulse_density_accumulator >=
        LOW_SPEED_PULSE_DENSITY_CARRIER_DC)
    {
      low_speed_pulse_density_accumulator -=
          LOW_SPEED_PULSE_DENSITY_CARRIER_DC;
      ++low_speed_pulse_density_powered_slots;
      apply_motor_output(
          requested_direction * LOW_SPEED_PULSE_DENSITY_CARRIER_DC);
    }
    else
    {
      apply_motor_output(0.0f);
    }
    return;
  }

  reset_low_speed_pulse_density();
  if (fabsf(dc_current_dc) < MOTOR_MIN_DC)
    apply_motor_output(0.0f);
  else
    apply_motor_output(dc_current_dc);
}

void set_dc(float dc, bool rate_limit)
{
  if (dc_state == DC_DISABLED || fabsf(dc) < 0.5f)
  {
    dc_current_dc = 0.0f;
    reset_low_speed_pulse_density();
    apply_motor_output(0.0f);
    return;
  }

  if (fabsf(dc) > MOTOR_MAX_DC)
    dc = copysignf(MOTOR_MAX_DC, dc);

  if (rate_limit &&
      dc > dc_current_dc + MOTOR_MAX_ACC_DC * last_loop_time)
    dc = dc_current_dc + MOTOR_MAX_ACC_DC * last_loop_time;
  else if (rate_limit &&
           dc < dc_current_dc - MOTOR_MAX_ACC_DC * last_loop_time)
    dc = dc_current_dc - MOTOR_MAX_ACC_DC * last_loop_time;

  // Preserve the controller's average request independently of the current
  // carrier slot so cruise entry and stall telemetry do not see artificial
  // zero-output transitions.
  dc_current_dc = dc;
  service_motor_output();
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

  if (dc_state == DC_HOLDING)
  {
    const float error = target_distance - current_distance;
    hold_pid_integral = constrain(
        hold_pid_integral + error * last_loop_time,
        -SPEED_INTEGRAL_PWM_MAX,
        SPEED_INTEGRAL_PWM_MAX);
    const float output = constrain(
        HOLD_POSITION_KP * error +
        HOLD_POSITION_KI * hold_pid_integral +
        HOLD_POSITION_KD * (error - last_error) / last_loop_time,
        -HOLD_MAX_DC,
        HOLD_MAX_DC);
    drive_control_phase = DRIVE_POSITION_HOLD;
    set_dc(output);
    last_error = error;
    return;
  }

  if (!speed_measurement_ready)
    return;
  speed_measurement_ready = false;

  if (target_speed == 0 && fabsf(current_speed) < 1.0f &&
      fabsf(measured_speed) < 3.0f)
  {
    pid_integral = 0.0f;
    last_error = 0.0f;
    drive_control_phase = DRIVE_CRUISING;
    set_dc(0, false);
    return;
  }

  const float speed_error = current_speed - measured_speed;
  const bool profile_settled =
      fabsf(current_acceleration) <= CRUISE_ACCEL_THRESHOLD_MMSS &&
      fabsf(target_speed - current_speed) < 1.0f;
  // Cruise is latched, but entry requires a sustained settled interval so one
  // noisy encoder sample cannot switch away from acceleration control early.
  const bool cruise_candidate =
      profile_settled && fabsf(speed_error) <= CRUISE_SPEED_ERROR_MMS;
  if (drive_control_phase != DRIVE_CRUISING)
  {
    if (!cruise_candidate)
      cruise_candidate_start_us = 0;
    else if (cruise_candidate_start_us == 0)
      cruise_candidate_start_us = current_time;
  }
  const bool cruise = drive_control_phase == DRIVE_CRUISING ||
      (cruise_candidate_start_us != 0 &&
       current_time - cruise_candidate_start_us >= CRUISE_ENTRY_DWELL_US);
  const DriveControlPhase new_phase = cruise
      ? DRIVE_CRUISING
      : (current_acceleration * current_speed < 0.0f
            ? DRIVE_DECELERATING
            : DRIVE_ACCELERATING);

  drive_control_phase = new_phase;

  commanded_acceleration = cruise
      ? 0.0f
      : constrain(
            current_acceleration +
                ACCEL_SPEED_TRACKING_KP * speed_error,
            -active_acceleration_limit,
            active_acceleration_limit);

  const float direction = current_speed > 0.5f
      ? 1.0f
      : (current_speed < -0.5f ? -1.0f : 0.0f);
  const bool low_speed_pulse_profile =
      direction != 0.0f &&
      fabsf(current_speed) <= LOW_SPEED_PULSE_DENSITY_MAX_TARGET_MMS;
  const float active_static_ff = low_speed_pulse_profile
      ? LOW_SPEED_PULSE_STATIC_FF_DC
      : motor_static_ff;
  const float steering_load_ff = low_speed_pulse_profile
      ? direction * LOW_SPEED_STEERING_FF_MAX_DC * constrain(
            fabsf((float)set_degree) / MAX_STEERING,
            0.0f,
            1.0f)
      : 0.0f;
  const float feedforward = direction * active_static_ff +
      steering_load_ff +
      motor_speed_ff * current_speed +
      motor_accel_ff * commanded_acceleration;
  float output = 0.0f;

  if (cruise)
  {
    const float abs_profile_speed = fabsf(current_speed);
    if (abs_profile_speed <= mid_speed_gain_end)
    {
      const float mid_blend = constrain(
          (abs_profile_speed - low_speed_gain_end) /
              (mid_speed_gain_end - low_speed_gain_end),
          0.0f,
          1.0f);
      active_cruise_kp = low_speed_cruise_kp +
          mid_blend * (mid_speed_cruise_kp - low_speed_cruise_kp);
      active_cruise_ki = low_speed_cruise_ki +
          mid_blend * (mid_speed_cruise_ki - low_speed_cruise_ki);
    }
    else
    {
      const float high_blend = constrain(
          (abs_profile_speed - mid_speed_gain_end) /
              (high_speed_gain_start - mid_speed_gain_end),
          0.0f,
          1.0f);
      active_cruise_kp = mid_speed_cruise_kp +
          high_blend * (Kp - mid_speed_cruise_kp);
      active_cruise_ki = mid_speed_cruise_ki +
          high_blend * (Ki - mid_speed_cruise_ki);
    }
    if (last_applied_drive_phase != DRIVE_CRUISING)
    {
      // Initialize the cruise integrator so changing controllers does not
      // create a PWM step.
      pid_integral = constrain(
          dc_current_dc - feedforward - active_cruise_kp * speed_error,
          CRUISE_ENTRY_INTEGRAL_MIN,
          CRUISE_ENTRY_INTEGRAL_MAX);
    }
    const float integral_candidate = constrain(
        pid_integral +
            active_cruise_ki * speed_error * speed_measurement_dt,
        -i_max,
        i_max);
    const float candidate_output =
        feedforward + active_cruise_kp * speed_error + integral_candidate;
    if (!((candidate_output > MOTOR_MAX_DC && speed_error > 0.0f) ||
          (candidate_output < -MOTOR_MAX_DC && speed_error < 0.0f)))
      pid_integral = integral_candidate;
    output = feedforward + active_cruise_kp * speed_error + pid_integral;
    pid_before_checking = pid_integral;
    last_error = speed_error;
  }
  else
  {
    const float acceleration_error =
        commanded_acceleration - measured_acceleration;
    const float integral_candidate = constrain(
        accel_pid_integral +
            accel_Ki * acceleration_error * speed_measurement_dt,
        -ACCEL_INTEGRAL_PWM_MAX,
        ACCEL_INTEGRAL_PWM_MAX);
    const float candidate_output =
        feedforward + accel_Kp * acceleration_error + integral_candidate;
    if (!((candidate_output > MOTOR_MAX_DC && acceleration_error > 0.0f) ||
          (candidate_output < -MOTOR_MAX_DC && acceleration_error < 0.0f)))
      accel_pid_integral = integral_candidate;
    output = feedforward +
        accel_Kp * acceleration_error + accel_pid_integral;
    pid_before_checking = accel_pid_integral;
    last_error = acceleration_error;
  }

  // Once a low-speed motion profile has settled, acceleration feedback alone
  // reacts too slowly to a large persistent speed deficit. Add bounded direct
  // speed feedback for loaded operation. It is symmetric in reverse and fades
  // out before normal low-error tracking, leaving the existing PI loops intact.
  low_speed_load_compensation_dc = 0.0f;
  const float abs_profile_speed = fabsf(current_speed);
  const bool settled_low_speed_profile =
      direction != 0.0f &&
      abs_profile_speed <= LOW_SPEED_LOAD_COMP_MAX_PROFILE_MMS &&
      fabsf(current_acceleration) <= CRUISE_ACCEL_THRESHOLD_MMSS;
  if (settled_low_speed_profile)
  {
    const float directional_speed_deficit =
        direction * (current_speed - measured_speed);
    if (directional_speed_deficit >
        LOW_SPEED_LOAD_COMP_ACTIVATION_ERROR_MMS)
    {
      const float compensation_magnitude = constrain(
          (directional_speed_deficit -
           LOW_SPEED_LOAD_COMP_ACTIVATION_ERROR_MMS) *
              LOW_SPEED_LOAD_COMP_KP_DC_PER_MMS,
          0.0f,
          LOW_SPEED_LOAD_COMP_MAX_DC);
      low_speed_load_compensation_dc = direction * compensation_magnitude;
      output += low_speed_load_compensation_dc;
      if (!low_speed_load_compensation_logged)
      {
        low_speed_load_compensation_logged = true;
        Serial.print("[MOTOR LOAD COMP] target/profile/measured_mm_s=");
        Serial.print(target_speed);
        Serial.print("/");
        Serial.print(current_speed, 1);
        Serial.print("/");
        Serial.print(measured_speed, 1);
        Serial.print(" deficit_mm_s=");
        Serial.print(directional_speed_deficit, 1);
        Serial.print(" compensation_dc=");
        Serial.print(low_speed_load_compensation_dc, 1);
        Serial.print(" output_dc=");
        Serial.println(output, 1);
      }
    }
  }

  // A planned stop may reduce forward drive down to coasting, but must never
  // apply reverse torque against a still-rolling wheel. Emergency stop paths
  // bypass this controller and de-energize the motor immediately.
  if (target_speed == 0)
  {
    if (measured_speed > SOFT_STOP_SPEED_THRESHOLD_MMS && output < 0.0f)
      output = 0.0f;
    else if (measured_speed < -SOFT_STOP_SPEED_THRESHOLD_MMS && output > 0.0f)
      output = 0.0f;
  }

  set_dc(output, false);
  last_applied_drive_phase = new_phase;
}

void drive_loop()
{
  // Steering is also serviced while the drive motor is disabled. This is
  // required for the stationary obstacle bench test.
  steer(set_degree);
  // Low-speed carrier timing is independent of the 50 ms encoder/PID update.
  // Service it on every main-loop pass so powered and unpowered slots remain
  // short even while the requested average effort is unchanged.
  service_motor_output();

  if (last_loop_time == 0 || dc_state == DC_DISABLED)
  {
    return; // Don't run until timing is initialized
  }

  if (dc_state == DC_ENABLED)
  {
    // A resumed forward command must never inherit enough negative ramp
    // acceleration to make the speed profile briefly reverse (and vice versa).
    if (target_speed > 0 && current_speed <= 0.0f &&
        current_acceleration < 0.0f)
        current_acceleration = 0.0f;
    else if (target_speed < 0 && current_speed >= 0.0f &&
             current_acceleration > 0.0f)
        current_acceleration = 0.0f;

    active_acceleration_limit = target_speed == 0
        ? fminf(SOFT_STOP_DECELERATION_MMSS, acc)
        : constrain(
              fabsf((float)target_speed) * PROFILE_ACCEL_PER_TARGET_SPEED,
              MIN_PROFILE_ACCELERATION_MMSS,
              acc);
    const float speed_error = target_speed - current_speed;
    // Reduce acceleration early enough that it can reach zero at the target
    // under the gentler release jerk: delta_v = a^2 / (2 * jerk).
    const float allowed_acceleration = sqrtf(
        2.0f * DRIVE_ACCEL_RELEASE_JERK_MMSSS * fabsf(speed_error));
    const float desired_acceleration = fabsf(speed_error) > 0.5f
        ? copysignf(
              fminf(active_acceleration_limit, allowed_acceleration),
              speed_error)
        : 0.0f;
    const bool releasing_acceleration =
        current_acceleration * desired_acceleration < 0.0f ||
        fabsf(desired_acceleration) < fabsf(current_acceleration);
    const float active_jerk_limit = releasing_acceleration
        ? DRIVE_ACCEL_RELEASE_JERK_MMSSS
        : DRIVE_JERK_LIMIT_MMSSS;
    const float max_acceleration_change = active_jerk_limit * last_loop_time;
    current_acceleration += constrain(
        desired_acceleration - current_acceleration,
        -max_acceleration_change,
        max_acceleration_change);

    float next_speed =
        current_speed + current_acceleration * last_loop_time;
    const bool crossesForwardTarget =
        target_speed > 0 && next_speed < 0.0f;
    const bool crossesReverseTarget =
        target_speed < 0 && next_speed > 0.0f;
    const bool crossesZeroTarget = target_speed == 0 &&
        ((current_speed > 0.0f && next_speed < 0.0f) ||
         (current_speed < 0.0f && next_speed > 0.0f));
    if (crossesForwardTarget || crossesReverseTarget || crossesZeroTarget)
        next_speed = 0.0f;
    if ((speed_error > 0.0f && next_speed >= target_speed) ||
        (speed_error < 0.0f && next_speed <= target_speed) ||
        fabsf(speed_error) <= 0.5f)
    {
      current_speed = target_speed;
      current_acceleration = 0.0f;
    }
    else
    {
      current_speed = next_speed;
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
    accel_pid_integral = 0;
    hold_pid_integral = 0;
    last_error = 0;
    current_acceleration = 0;
    commanded_acceleration = 0;
    current_speed = 0;
    target_speed = 0;
    drive_control_phase = DRIVE_CRUISING;
    last_applied_drive_phase = DRIVE_CRUISING;
    cruise_candidate_start_us = 0;
    target_distance = current_distance;
    set_dc(0);
  }
  else
  {
    dc_state = DC_HOLDING;
    // Lock to current distance to hold position
    target_distance = current_distance;
    pid_integral = 0;
    accel_pid_integral = 0;
    hold_pid_integral = 0;
    last_error = 0;
    current_acceleration = 0;
    commanded_acceleration = 0;
  }
  servo_disabled = true;
}

void set_speed(int speed)
{
  if (speed == -1)
  {
    speed = last_speed;
  }
  if (speed != target_speed)
  {
    drive_control_phase = speed < current_speed
        ? DRIVE_DECELERATING
        : DRIVE_ACCELERATING;
    accel_pid_integral = 0.0f;
    cruise_candidate_start_us = 0;
    low_speed_load_compensation_logged = false;
  }
  dc_state = DC_ENABLED;
  target_speed = speed;
  last_speed = speed;
}

void loop_updater()
{
  static unsigned long last_loop_time_us = 0;
  static unsigned long speed_sample_start_us = 0;
  static float speed_sample_start_distance = 0;
  static float previous_measured_speed = 0;

  last_time = current_time;
  last_distance = current_distance;

  current_time = micros();
  last_loop_time_us = current_time - last_time;
  last_loop_time = last_loop_time_us / 1000000.0; // Convert to seconds

  current_distance = get_distance(encoder_pos);

  if (speed_sample_start_us == 0)
  {
    speed_sample_start_us = current_time;
    speed_sample_start_distance = current_distance;
  }
  else
  {
    const unsigned long speed_elapsed_us =
        current_time - speed_sample_start_us;
    const bool low_speed_cruise_filter =
        drive_control_phase == DRIVE_CRUISING &&
        fabsf((float)target_speed) <= low_speed_gain_end;
    if (speed_elapsed_us >= SPEED_MEASUREMENT_WINDOW_US)
    {
      const float raw_speed =
          (current_distance - speed_sample_start_distance) /
          (speed_elapsed_us / 1000000.0f);
      const float speed_filter_alpha = low_speed_cruise_filter
          ? LOW_SPEED_FILTER_ALPHA
          : SPEED_FILTER_ALPHA;
      measured_speed +=
          speed_filter_alpha * (raw_speed - measured_speed);
      speed_measurement_dt = speed_elapsed_us / 1000000.0f;
      const float raw_acceleration =
          (measured_speed - previous_measured_speed) /
          speed_measurement_dt;
      measured_acceleration += ACCELERATION_FILTER_ALPHA *
          (raw_acceleration - measured_acceleration);
      previous_measured_speed = measured_speed;
      speed_measurement_ready = true;
      ++speed_measurement_count;
      speed_sample_start_us = current_time;
      speed_sample_start_distance = current_distance;
    }
  }
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

  if (!no_progress_watchdog_preflight_passed)
  {
    reset_no_progress_watchdog(
        no_progress_watchdog,
        current_time,
        current_distance);
    if (dc_state == DC_ENABLED && target_speed != 0)
    {
      Serial.println(
          "Stall watchdog unavailable: startup preflight failed.");
      mode_pause();
    }
    return;
  }

  NoProgressWatchdogEvidence no_progress_evidence;
  if (update_no_progress_watchdog(
          no_progress_watchdog,
          dc_state == DC_ENABLED,
          target_speed,
          current_speed,
          current_time,
          current_distance,
          no_progress_evidence))
  {
    Serial.print("Stall detected: commanded motion made only ");
    Serial.print(no_progress_evidence.directional_progress_mm, 1);
    Serial.print(" mm progress in ");
    Serial.print(no_progress_evidence.elapsed_us / 1000U);
    Serial.print(" ms. Target: ");
    Serial.print(target_speed);
    Serial.print(" mm/s, profile: ");
    Serial.print(current_speed, 1);
    Serial.print(" mm/s, measured: ");
    Serial.print(measured_speed, 1);
    Serial.print(" mm/s, requested DC: ");
    Serial.print(dc_current_dc, 1);
    Serial.print(", applied DC: ");
    Serial.print(dc_out);
    Serial.print(", pulse density: ");
    Serial.print(low_speed_pulse_density_active ? "active" : "off");
    Serial.print(", low-speed load comp: ");
    Serial.println(low_speed_load_compensation_dc, 1);
    reset_no_progress_watchdog(
        no_progress_watchdog,
        current_time,
        current_distance);
    mode_pause();
    return;
  }

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

  no_progress_watchdog_preflight_passed =
      no_progress_watchdog_preflight();
  reset_no_progress_watchdog(
      no_progress_watchdog,
      current_time,
      current_distance);
  Serial.print("[STALL] commanded-motion watchdog preflight: ");
  Serial.println(no_progress_watchdog_preflight_passed ? "PASS" : "FAIL");
}
