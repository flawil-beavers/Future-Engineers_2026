/**
 * @file navigation_controller.cpp
 * @brief Gyro-Stabilized Wall Following Control Implementation
 * 
 * This module implements a hybrid navigation strategy:
 * 1. Primary Control: Gyroscope-based PD loop maintains a global grid heading (0, 90, 180, 270).
 * 2. Secondary Control: ToF-based PD loop maintains a lateral offset from the wall.
 * 3. Geometric Correction: Wall distance is corrected using the gyro angle to ensure
 *    accuracy even when the robot is not perfectly parallel to the wall.
 * 
 * State Machine:
 * - NAV_IDLE: Waiting for activation.
 * - NAV_FOLLOWING: Moving straight using Gyro + ToF.
 * - NAV_TURNING: Executing a 90-degree pivot turn based on gyro feedback.
 * - NAV_STOPPED: Mission complete (3 rounds finished).
 */

#include "navigation_controller.h"
#include "mode_manager.h"
#include "position_estimator.h"
#include "config.h"
#include "motor_control.h"
#include "sensors.h"
#include "logger.h"
#define Serial robot_logger


extern long encoder_pos;
extern float current_distance;
extern void sensors_set_tof_timing_budget(uint32_t budget_us);
extern uint32_t sensors_initial_tof_timing_budget;
extern float get_tof_raw_distance(TofSensor sensor);
extern float get_tof_signal_rate(TofSensor sensor);
extern float get_tof_sigma(TofSensor sensor);

// ==========================================
// STATE VARIABLES
// ==========================================

NavigationState nav_state = NAV_IDLE;
NavigationState nav_last_state = NAV_IDLE;

// Wall following parameters
float nav_target_distance = OPEN_WALL_TARGET_DISTANCE_MM;
float nav_wall_margin = TOF_MAX_RELIABLE_DISTANCE_MM; // Threshold to detect gap/open space (mm)
int nav_turn_angle = 0;                // +90 or -90 degrees
WallSide nav_following_wall = SIDE_UNKNOWN; // Which wall are we following

// Gyro following parameters
float nav_gyro_target = 0;             // Target gyro angle
float nav_gyro_kp = 2.5;               // Proportional gain
float nav_gyro_kd = 0.05;              // Derivative gain
float nav_last_gyro_error = 0;
float nav_steering_filter = 0.0f;

// Timing and control
float nav_turn_start_angle = 0;
float nav_corner_phase_start_distance = 0;
uint32_t nav_corner_brake_start_ms = 0;

// Round counting
int nav_turn_count = 0;
float nav_start_angle = 0;
float nav_start_distance = 0;
int nav_completed_rounds = 0;

// Speed parameters
float nav_normal_speed = OPEN_CHALLENGE_STRAIGHT_SPEED_MMS;

// Internal logic flags
bool nav_searching_for_wall = false;   // True when waiting to "re-acquire" a wall after a turn
bool nav_long_range_active = false;    // Tracks if ToF is in discovery (slow) mode
bool nav_obstacle_mode = false;
bool nav_soft_stop_started = false;
bool nav_soft_stop_complete = false;
uint8_t nav_corner_gap_samples = 0;
float nav_wall_correction_resume_distance = 0;
uint32_t nav_last_corner_tof_sample = 0;
float nav_learned_straight_mm[4] = {0, 0, 0, 0};
bool nav_final_slowing = false;

// PD Controller
float nav_pd_kp = 0.5;                 // Proportional gain
float nav_pd_kd = 0.01;                // Derivative gain
float nav_last_distance_error = 0;
float nav_distance_pd = 0;
uint32_t nav_last_distance_tof_sample = 0;
unsigned long nav_last_distance_update_us = 0;

// Telemetry and Debug
bool nav_debug_enabled = false;

// Time tracking
extern unsigned long current_time;
extern float last_loop_time;

static bool all_open_straights_learned()
{
  for (int i = 0; i < 4; ++i)
    if (nav_learned_straight_mm[i] <= 0.0f)
      return false;
  return true;
}

static float controlled_stop_distance(float speed, float deceleration,
                                      float jerk)
{
  speed = fmaxf(0.0f, speed);
  const float ramp_time = deceleration / jerk;
  const float ramp_speed_loss = 0.5f * jerk * ramp_time * ramp_time;
  if (ramp_speed_loss >= speed)
  {
    const float stop_time = sqrtf(2.0f * speed / jerk);
    return speed * stop_time - jerk * stop_time * stop_time * stop_time / 6.0f;
  }
  const float ramp_distance = speed * ramp_time -
      jerk * ramp_time * ramp_time * ramp_time / 6.0f;
  const float remaining_speed = speed - ramp_speed_loss;
  return ramp_distance +
      remaining_speed * remaining_speed / (2.0f * deceleration);
}

// ==========================================
// HELPER FUNCTIONS
// ==========================================

/**
 * @brief Calculates the perpendicular distance to a wall.
 * 
 * Uses the current gyro heading relative to the target to calculate the 
 * incidence angle. Applies cosine correction to the raw ToF reading to 
 * find the true lateral distance, preventing "phantom" distance increases 
 * when the robot is tilted.
 */
float get_followed_wall_distance(WallSide side)
{
  TofSensor sensor = (side == SIDE_LEFT) ? TOF_LEFT : TOF_RIGHT;
  float raw_dist = get_tof_distance(sensor);

  if (nav_obstacle_mode &&
      (raw_dist <= 0 || raw_dist >= TOF_OUT_OF_RANGE_MM))
  {
    float fallback = get_tof_raw_distance(sensor);
    float signal = get_tof_signal_rate(sensor);
    float sigma = get_tof_sigma(sensor);
    if (fallback > 0 && fallback <= TOF_MAX_RELIABLE_DISTANCE_MM &&
        signal >= 0.15f && sigma <= 35.0f)
      raw_dist = fallback;
  }

  if (raw_dist <= 0 || raw_dist >= TOF_OUT_OF_RANGE_MM) {
    return raw_dist;
  }

  float angle_error_deg = get_angle() - nav_gyro_target;
  float incidence_angle_rad = angle_error_deg * PI / 180.0f;
  return raw_dist * cos(incidence_angle_rad);
}

float get_followed_wall_distance()
{
  if (nav_following_wall == SIDE_UNKNOWN)
  {
    float dist_l = get_tof_distance(TOF_LEFT);
    float dist_r = get_tof_distance(TOF_RIGHT);
    return (dist_l > dist_r) ? dist_l : dist_r;
  }
  return get_followed_wall_distance(nav_following_wall);
}

/**
 * @brief Logs detailed ToF sensor data and robot state to Serial.
 * 
 * @param reason A string describing the trigger for the log (e.g., state change)
 */
void log_tof_diagnostics(const char* reason)
{
  float dist_l = get_tof_distance(TOF_LEFT);
  float dist_r = get_tof_distance(TOF_RIGHT);
  float raw_l = get_tof_raw_distance(TOF_LEFT);
  float raw_r = get_tof_raw_distance(TOF_RIGHT);
  float sig_l = get_tof_signal_rate(TOF_LEFT);
  float sig_r = get_tof_signal_rate(TOF_RIGHT);
  float sigma_l = get_tof_sigma(TOF_LEFT);
  float sigma_r = get_tof_sigma(TOF_RIGHT);
  float current_deg = get_angle()-nav_start_angle;
  float distance_since_last_state = current_distance - nav_start_distance;

  Serial.print("[STATE CHANGE] "); Serial.println(reason);
  Serial.print("  LEFT:  Dist: "); Serial.print(dist_l, 0); Serial.print(" mm (Raw: "); Serial.print(raw_l, 0); 
  Serial.print(") | Sig: "); Serial.print(sig_l, 2); Serial.print(" | Sigma: "); Serial.println(sigma_l, 2);
  Serial.print("  RIGHT: Dist: "); Serial.print(dist_r, 0); Serial.print(" mm (Raw: "); Serial.print(raw_r, 0); 
  Serial.print(") | Sig: "); Serial.print(sig_r, 2); Serial.print(" | Sigma: "); Serial.println(sigma_r, 2);
  Serial.print("  Current angle: "); Serial.print(current_deg, 1); Serial.println("°");
  Serial.print("  Distance since last state: "); Serial.print(distance_since_last_state, 0); Serial.println(" mm");
}

// ==========================================
// STATE FUNCTIONS
// ==========================================

/**
 * @brief Idle state. Robot is stationary and waiting for activation.
 */
void state_idle()
{
  // Do nothing
}

/**
 * @brief Straight-line navigation logic.
 * 
 * Combined Control Law:
 * Steering = (Gyro_Error * Kp_g) + (Wall_Distance_Error * Kp_d)
 * 
 * This ensures the robot stays straight relative to the room while 
 * gently nudging away or toward the wall to maintain the gap.
 */
void state_following()
{
  // 0. Dynamic sensor configuration: increase range/sensitivity when side is unknown
  if (nav_following_wall == SIDE_UNKNOWN && !nav_long_range_active) {
    sensors_set_tof_timing_budget(300000); // 300ms for long-range discovery
    nav_long_range_active = true;
  } else if (nav_following_wall != SIDE_UNKNOWN && nav_long_range_active) {
    sensors_set_tof_timing_budget(sensors_initial_tof_timing_budget); // Restore initial value
    nav_long_range_active = false;
  }

  // 1. Primary: Gyro Heading Control
  float gyro_error = get_angle() - nav_gyro_target;
  float safe_loop_time = (last_loop_time > 0.001f) ? last_loop_time : 0.001f;
  float gyro_derivative = (gyro_error - nav_last_gyro_error) / safe_loop_time;
  float gyro_pd = navigation_compute_steering(
      gyro_error, nav_last_gyro_error, safe_loop_time);
  if (nav_obstacle_mode) {
    gyro_pd = 0.85f * gyro_error + 0.012f * gyro_derivative;
    if (gyro_pd > 24.0f) gyro_pd = 24.0f;
    if (gyro_pd < -24.0f) gyro_pd = -24.0f;
  }
  nav_last_gyro_error = gyro_error;

  // After the twelfth corner, stop near the middle of the learned final
  // straight. Start braking before the midpoint by the controlled stopping
  // distance, instead of using one fixed distance for every field size.
  if (nav_completed_rounds >= 3)
  {
    const int final_section = nav_turn_count % 4;
    const float final_straight_length =
        nav_learned_straight_mm[final_section];
    if (final_straight_length > 0.0f)
    {
      const float final_target_distance = fmaxf(
          0.0f,
          final_straight_length * OPEN_FINAL_TARGET_FRACTION -
              OPEN_FINAL_TARGET_BEFORE_CENTER_MM);
      const float final_braking_distance =
          controlled_stop_distance(
              OPEN_CHALLENGE_PRE_CORNER_SPEED_MMS,
              OPEN_FINAL_DECELERATION_MMSS,
              OPEN_FINAL_JERK_MMSSS) +
              OPEN_FINAL_BRAKE_MARGIN_MM;
      const float final_high_to_low_distance =
          (OPEN_FINAL_APPROACH_SPEED_MMS * OPEN_FINAL_APPROACH_SPEED_MMS -
              OPEN_CHALLENGE_PRE_CORNER_SPEED_MMS *
                  OPEN_CHALLENGE_PRE_CORNER_SPEED_MMS) /
              (2.0f * OPEN_FINAL_HIGH_TO_LOW_DECEL_MMSS) +
          OPEN_FINAL_HIGH_TO_LOW_MARGIN_MM;
      const float final_stop_trigger =
          fmaxf(0.0f, final_target_distance - final_braking_distance);
      const float final_distance =
          fabsf(get_distance() - nav_start_distance);
      if (final_distance >=
          fmaxf(0.0f, final_target_distance - final_high_to_low_distance))
        nav_final_slowing = true;
      if (final_distance >= final_stop_trigger)
      {
        Serial.print("[NAV] Final braking point: ");
        Serial.print(final_distance, 0);
        Serial.print(" mm, target 100 mm before midpoint: ");
        Serial.print(final_target_distance, 0);
        Serial.println(" mm -> controlled stop");
        nav_state = NAV_STOPPED;
        set_soft_stop_profile(
            OPEN_FINAL_DECELERATION_MMSS,
            OPEN_FINAL_JERK_MMSSS);
        set_speed(0);
        return;
      }
    }
  }

  // 2. Secondary: Wall Distance Correction
  float current_wall_distance = get_followed_wall_distance();
  float dist_pd = nav_distance_pd;

  /**
   * DETECTION LOGIC: Trigger Turn
   * 
   * If the wall disappears (reading exceeds margin), it indicates a corner.
   * The robot will increment its turn count and transition to NAV_TURNING.
   */
  // The initial position can be part-way along the start section, so its
  // first corner needs only a short lockout. After a known corner a new
  // straight is about one metre long; accepting another corner after only
  // 300 mm caused false turns after obstacle manoeuvres.
  float blind_dist =
      (nav_obstacle_mode && nav_turn_count > 0) ? 650.0f :
      ((nav_turn_count == 1) ? 600.0f : 300.0f);
  float wall_margin = nav_long_range_active ? 1500 : nav_wall_margin;
  // Encoder distance can decrease during the first-lap reverse alignment.
  // Always compare travelled displacement from the corner origin; a signed
  // comparison could permanently suppress every corner after the first one.
  bool beyond_blind_distance =
      fabsf(get_distance() - nav_start_distance) > blind_dist;
  bool heading_plausible = fabs(gyro_error) < (nav_obstacle_mode ? 12.0f : 180.0f);
  bool gap_seen = current_wall_distance > wall_margin && current_wall_distance > 0;
  const bool corner_warning =
      beyond_blind_distance && heading_plausible && gap_seen;
  const TofSensor corner_sensor = nav_following_wall == SIDE_RIGHT
      ? TOF_RIGHT
      : TOF_LEFT;
  const uint32_t corner_tof_sample =
      get_tof_measurement_count(corner_sensor);
  if (corner_tof_sample != nav_last_corner_tof_sample)
  {
    nav_last_corner_tof_sample = corner_tof_sample;
    if (corner_warning) {
      if (nav_corner_gap_samples < 255) nav_corner_gap_samples++;
    } else {
      nav_corner_gap_samples = 0;
    }
  }

  const int detection_section = nav_turn_count % 4;
  const float detection_straight_distance =
      fabsf(get_distance() - nav_start_distance);
  const bool predicted_detection_zone =
      !nav_obstacle_mode && all_open_straights_learned() &&
      nav_learned_straight_mm[detection_section] > 0.0f &&
      detection_straight_distance >= fmaxf(
          0.0f,
          nav_learned_straight_mm[detection_section] -
              OPEN_CORNER_PREDICT_MARGIN_MM);
  const uint8_t required_gap_samples = nav_obstacle_mode
      ? 4
      : (predicted_detection_zone
            ? OPEN_PREDICTED_CORNER_CONFIRM_SAMPLES
            : OPEN_CORNER_CONFIRM_SAMPLES);
  if (nav_corner_gap_samples >= required_gap_samples)
  {
    nav_corner_gap_samples = 0;
    if (!nav_obstacle_mode && nav_turn_count > 0)
    {
      const float learned_length =
          fabsf(get_distance() - nav_start_distance);
      if (learned_length >= OPEN_CORNER_MIN_LEARNED_LENGTH_MM &&
          learned_length <= OPEN_CORNER_MAX_LEARNED_LENGTH_MM)
      {
        const int section = nav_turn_count % 4;
        nav_learned_straight_mm[section] = learned_length;
        const int opposite_section = (section + 2) % 4;
        nav_learned_straight_mm[opposite_section] = learned_length;
        Serial.print("[NAV] Learned straight S");
        Serial.print(section);
        Serial.print(" and opposite S");
        Serial.print(opposite_section);
        Serial.print(": ");
        Serial.print(learned_length, 0);
        Serial.println(" mm");
      }
    }
    if (nav_following_wall == SIDE_UNKNOWN)
    {
      // Searching for initial direction
      float dist_left = get_tof_distance(TOF_LEFT);
      float dist_right = get_tof_distance(TOF_RIGHT);
      if (dist_left > wall_margin && dist_left > 0)
      {
        nav_following_wall = SIDE_LEFT;
        nav_turn_angle = 90;
        Serial.print("Dist Left: ");
        Serial.println(dist_left);
      }
      else if (dist_right > wall_margin && dist_right > 0)
      {
        nav_following_wall = SIDE_RIGHT;
        nav_turn_angle = -90;
        Serial.print("Dist Right: ");
        Serial.println(dist_right);
      }
      else
        return;

      // Side determined. Restore budget immediately.
      sensors_set_tof_timing_budget(sensors_initial_tof_timing_budget);
      nav_long_range_active = false;
    }
    else
    {
      nav_turn_angle = (nav_following_wall == SIDE_LEFT) ? 90 : -90;
    }

    nav_turn_count++;
    nav_completed_rounds = (int)(nav_turn_count / 4);

    if (nav_obstacle_mode)
    {
      log_tof_diagnostics("Corner detected -> TURNING");
      nav_state = NAV_TURNING;
      nav_turn_start_angle = get_angle();
    }
    else
    {
      // The disappearing side wall is the inner boundary. Start turning at
      // its confirmed end so the car does not make a wide path to the outside.
      log_tof_diagnostics("Inner corner confirmed -> TURNING");
      nav_state = NAV_TURNING;
      nav_turn_start_angle = get_angle();
      set_speed(OPEN_CHALLENGE_CORNER_SPEED_MMS);
    }
    return;
  }

  /**
   * DISTANCE PD CONTROL
   * 
   * If a wall is within range, calculate the error from the active target.
   * This term is added to the gyro steering to gently nudge the robot 
   * away from or toward the wall while maintaining heading.
   */
  if (current_wall_distance > 0 && current_wall_distance < nav_wall_margin)
  {
    if (nav_searching_for_wall) {
      if (current_wall_distance < (nav_target_distance + 100.0)) nav_searching_for_wall = false;
    }

    const bool post_turn_gyro_only =
        nav_obstacle_mode &&
        get_distance() < nav_wall_correction_resume_distance;

    const TofSensor distance_sensor = nav_following_wall == SIDE_RIGHT
        ? TOF_RIGHT
        : TOF_LEFT;
    const uint32_t distance_sample =
        get_tof_measurement_count(distance_sensor);
    const bool new_distance_sample =
        distance_sample != nav_last_distance_tof_sample;

    if (!nav_searching_for_wall && !post_turn_gyro_only &&
        new_distance_sample) {
      nav_last_distance_tof_sample = distance_sample;
      float dist_error = current_wall_distance - nav_target_distance;
      const float distance_dt = nav_last_distance_update_us == 0
          ? TOF_TIMING_BUDGET_US / 1000000.0f
          : fmaxf(
                (current_time - nav_last_distance_update_us) / 1000000.0f,
                0.001f);
      nav_last_distance_update_us = current_time;
      float dist_derivative =
          (dist_error - nav_last_distance_error) / distance_dt;
      dist_pd = nav_pd_kp * dist_error + nav_pd_kd * dist_derivative;
      if (nav_obstacle_mode) {
        dist_pd = 0.18f * dist_error + 0.003f * dist_derivative;
        if (dist_pd > 14.0f) dist_pd = 14.0f;
        if (dist_pd < -14.0f) dist_pd = -14.0f;
      }
      else
      {
        // At high speed, large lateral steering corrections cause a violent
        // lane change. Scale them down smoothly and keep gyro heading primary.
        const float gain_scale = constrain(
            OPEN_WALL_PD_FULL_GAIN_SPEED_MMS /
                fmaxf(fabsf(measured_speed),
                      OPEN_WALL_PD_FULL_GAIN_SPEED_MMS),
            OPEN_WALL_PD_MIN_GAIN_SCALE,
            1.0f);
        dist_pd *= gain_scale;
        dist_pd = constrain(
            dist_pd,
            -OPEN_WALL_CORRECTION_MAX_DEG,
            OPEN_WALL_CORRECTION_MAX_DEG);
      }
      nav_last_distance_error = dist_error;
      nav_distance_pd = dist_pd;
    }
  }
  else if (nav_following_wall != SIDE_UNKNOWN)
  {
    const TofSensor distance_sensor = nav_following_wall == SIDE_RIGHT
        ? TOF_RIGHT
        : TOF_LEFT;
    const uint32_t distance_sample =
        get_tof_measurement_count(distance_sensor);
    if (distance_sample != nav_last_distance_tof_sample)
    {
      nav_last_distance_tof_sample = distance_sample;
      nav_distance_pd = 0.0f;
      dist_pd = 0.0f;
    }
  }

  // Combine Steering: Gyro + Distance Correction
  float total_steering = gyro_pd;
  if (nav_following_wall == SIDE_LEFT) total_steering -= dist_pd;
  else if (nav_following_wall == SIDE_RIGHT) total_steering += dist_pd;

  // Clamp to physical servo limits
  if (total_steering > 60) total_steering = 60;
  if (total_steering < -60) total_steering = -60;

  set_steering(total_steering);

  // An Ackermann-steered car needs a calm corner exit.  Keep the first part
  // of the new straight slow while gyro-only steering makes it parallel;
  // using the close return from the old wall here pulled the car across the
  // corridor in clockwise runs.
  const bool post_turn_gyro_only =
      nav_obstacle_mode &&
      get_distance() < nav_wall_correction_resume_distance;
  const int current_section = nav_turn_count % 4;
  const bool actual_map_ready = all_open_straights_learned();
  const float predicted_length = actual_map_ready
      ? nav_learned_straight_mm[current_section]
      : 0.0f;
  const float straight_distance =
      fabsf(get_distance() - nav_start_distance);
  const float speed_for_braking = fmaxf(
      fabsf(measured_speed),
      OPEN_CHALLENGE_PRE_CORNER_SPEED_MMS);
  const float prediction_braking_distance =
      (speed_for_braking * speed_for_braking -
          OPEN_CHALLENGE_PRE_CORNER_SPEED_MMS *
              OPEN_CHALLENGE_PRE_CORNER_SPEED_MMS) /
          (2.0f * OPEN_CORNER_PREDICT_DECEL_MMSS) +
      OPEN_CORNER_PREDICT_MARGIN_MM;
  const bool predicted_corner_near =
      !nav_obstacle_mode && predicted_length > 0.0f &&
      straight_distance >=
          fmaxf(0.0f, predicted_length - prediction_braking_distance);
  set_speed(
      post_turn_gyro_only
          ? OBSTACLE_POST_TURN_SPEED
          : (nav_completed_rounds >= 3
                ? (nav_final_slowing
                      ? OPEN_CHALLENGE_PRE_CORNER_SPEED_MMS
                      : OPEN_FINAL_APPROACH_SPEED_MMS)
                : (nav_following_wall == SIDE_UNKNOWN
                ? OPEN_CHALLENGE_DISCOVERY_SPEED_MMS
                : (actual_map_ready && predicted_corner_near
                      ? OPEN_CHALLENGE_PRE_CORNER_SPEED_MMS
                      : (actual_map_ready
                            ? nav_normal_speed
                            : (straight_distance < fmaxf(
                                      0.0f,
                                      OPEN_FIRST_LAP_ASSUMED_LENGTH_MM -
                                          OPEN_CORNER_PREDICT_MARGIN_MM)
                                  ? nav_normal_speed
                                  : OPEN_CHALLENGE_PRE_CORNER_SPEED_MMS))))));
}

/**
 * @brief 90-degree Pivot Turn Logic.
 * 
 * Executes a high-angle turn while maintaining forward speed. 
 * The turn is concluded only once the gyroscope confirms the 
 * delta heading is >= 90 degrees.
 */
void state_turning()
{
  const float cornerSteering =
      nav_obstacle_mode ? OBSTACLE_CORNER_STEERING : 25.0f;
  set_steering(
      nav_turn_angle > 0
          ? -cornerSteering
          : cornerSteering);
  if (nav_obstacle_mode)
  {
    set_speed(
        nav_turn_count <= 4
            ? OBSTACLE_FIRST_LAP_CORNER_SPEED
            : OBSTACLE_LATER_LAP_CORNER_SPEED);
  }
  else
  {
    set_speed(OPEN_CHALLENGE_CORNER_SPEED_MMS);
  }

  if ((get_angle() - nav_turn_start_angle - nav_turn_angle) * nav_turn_angle/fabs(nav_turn_angle) > 0)
  {
    nav_gyro_target += nav_turn_angle;
    nav_last_gyro_error = 0;
    nav_searching_for_wall = true;
    nav_start_distance = get_distance();
    nav_wall_correction_resume_distance =
        nav_start_distance + (nav_obstacle_mode ? 300.0f : 0.0f);
    if (nav_obstacle_mode && nav_turn_count <= 4)
    {
      nav_corner_brake_start_ms = millis();
      set_speed(0);
      log_tof_diagnostics("Turn arc finished -> BRAKING FOR REVERSE");
      nav_state = NAV_CORNER_BRAKING_FOR_REVERSE;
    }
    else
    {
      log_tof_diagnostics("Turn finished -> FOLLOWING");
      nav_state = NAV_FOLLOWING;
    }
    set_steering(0);
  }
}

static bool corner_direction_change_ready()
{
  const uint32_t elapsed = millis() - nav_corner_brake_start_ms;
  return (elapsed >= OBSTACLE_CORNER_DIRECTION_CHANGE_MIN_MS &&
          fabsf(measured_speed) <= OBSTACLE_CORNER_STOPPED_SPEED_MM_S) ||
         elapsed >= OBSTACLE_CORNER_DIRECTION_CHANGE_MAX_MS;
}

void state_corner_braking_for_reverse()
{
  set_steering(0);
  set_speed(0);

  if (!corner_direction_change_ready())
    return;

  nav_corner_phase_start_distance = get_distance();
  nav_state = NAV_CORNER_REVERSING;
  Serial.println("[NAV] Standstill -> FIRST-LAP REVERSING");
}

void state_corner_reversing()
{
  const float reverse_distance =
      fabsf(get_distance() - nav_corner_phase_start_distance);

  set_steering(
      nav_turn_angle > 0
          ? -OBSTACLE_FIRST_LAP_REVERSE_STEERING
          : OBSTACLE_FIRST_LAP_REVERSE_STEERING);
  set_speed(-OBSTACLE_FIRST_LAP_REVERSE_SPEED);

  if (reverse_distance >= OBSTACLE_FIRST_LAP_REVERSE_TARGET_MM)
  {
    set_speed(0);
    set_steering(0);
    nav_corner_brake_start_ms = millis();
    nav_state = NAV_CORNER_BRAKING_FOR_ALIGN;
    Serial.print("[NAV] Reverse arc complete distance=");
    Serial.print(reverse_distance, 0);
    Serial.print(" heading_error=");
    Serial.println(get_angle() - nav_gyro_target, 1);
  }
}

void state_corner_braking_for_align()
{
  set_steering(0);
  set_speed(0);

  if (!corner_direction_change_ready())
    return;

  nav_corner_phase_start_distance = get_distance();
  nav_last_gyro_error = 0;
  nav_state = NAV_CORNER_ALIGNING;
  Serial.println("[NAV] Standstill -> FIRST-LAP FORWARD ALIGNING");
}

void state_corner_aligning()
{
  const float heading_error = get_angle() - nav_gyro_target;
  const float align_distance =
      fabsf(get_distance() - nav_corner_phase_start_distance);

  float steering = 0.85f * heading_error;
  if (steering > 18.0f) steering = 18.0f;
  if (steering < -18.0f) steering = -18.0f;

  set_steering(static_cast<int>(steering));
  set_speed(OBSTACLE_FIRST_LAP_ALIGN_SPEED);

  const bool aligned =
      align_distance >= OBSTACLE_FIRST_LAP_ALIGN_MIN_MM &&
      fabsf(heading_error) <=
          OBSTACLE_FIRST_LAP_ALIGN_TOLERANCE_DEG;
  const bool distance_limit =
      align_distance >= OBSTACLE_FIRST_LAP_ALIGN_MAX_MM;

  if (aligned || distance_limit)
  {
    // Keep nav_start_distance at the geometrical corner origin established
    // when the 90-degree arc finished. Resetting it here discarded the whole
    // reverse/forward alignment distance and moved the next corner detection
    // window up to 260 mm too far into the following section.
    nav_last_gyro_error = 0;
    nav_last_distance_error = 0;
    set_speed(0);
    set_steering(0);
    nav_corner_brake_start_ms = millis();
    nav_state = NAV_CORNER_BRAKING_FOR_SECTION_BACKUP;
    log_tof_diagnostics(
        aligned
            ? "First-lap corner aligned -> BRAKING FOR 400 MM BACKUP"
            : "First-lap alignment limit -> BRAKING FOR 400 MM BACKUP");
  }
}

void state_corner_braking_for_section_backup()
{
  set_steering(0);
  set_speed(0);
  if (!corner_direction_change_ready()) return;

  nav_corner_phase_start_distance = get_distance();
  nav_state = NAV_CORNER_SECTION_BACKING;
  Serial.println("[NAV] Standstill -> BACKING 400 MM FOR VISIBILITY");
}

void state_corner_section_backing()
{
  const float heading_error = get_angle() - nav_gyro_target;
  const float backup_distance =
      fabsf(get_distance() - nav_corner_phase_start_distance);

  // Steering effect reverses when the car drives backwards.
  float steering = -0.85f * heading_error;
  steering = constrain(
      steering,
      -OBSTACLE_FIRST_LAP_SECTION_BACKUP_MAX_STEERING,
      OBSTACLE_FIRST_LAP_SECTION_BACKUP_MAX_STEERING);
  set_steering(static_cast<int>(steering));
  set_speed(-OBSTACLE_FIRST_LAP_SECTION_BACKUP_SPEED);

  if (backup_distance >= OBSTACLE_FIRST_LAP_SECTION_BACKUP_MM)
  {
    set_speed(0);
    set_steering(0);
    nav_corner_brake_start_ms = millis();
    nav_state = NAV_CORNER_BRAKING_AFTER_SECTION_BACKUP;
    Serial.print("[NAV] Section visibility backup complete distance=");
    Serial.println(backup_distance, 0);
  }
}

void state_corner_braking_after_section_backup()
{
  set_steering(0);
  set_speed(0);
  if (!corner_direction_change_ready()) return;

  nav_wall_correction_resume_distance = get_distance() + 200.0f;
  nav_searching_for_wall = true;
  nav_last_gyro_error = 0;
  nav_last_distance_error = 0;
  nav_state = NAV_FOLLOWING;
  Serial.println("[NAV] Visibility backup stopped -> FOLLOWING");
}

/**
 * @brief Shutdown Logic.
 * 
 * Reached after 12 corners (3 laps). Stops motors and centers steering.
 */
void state_stopped()
{
  set_steering(0);
  if (!nav_soft_stop_started)
  {
    nav_soft_stop_started = true;
    set_speed(0);
    Serial.println("Mission complete: controlled deceleration started.");
  }

  if (fabsf(current_speed) > SOFT_STOP_SPEED_THRESHOLD_MMS ||
      fabsf(measured_speed) > SOFT_STOP_SPEED_THRESHOLD_MMS)
    return;

  if (!nav_soft_stop_complete)
  {
    stop(false);
    nav_soft_stop_complete = true;
    Serial.println("===== GYRO TASK COMPLETE =====");
    Serial.print("Total turns: "); Serial.print(nav_turn_count);
    Serial.print(" | Complete rounds: "); Serial.println(nav_completed_rounds);
    log_tof_diagnostics("Mission complete");
    robot_logger.write_to_usb();
  }
}


// ==========================================
// PUBLIC INTERFACE
// ==========================================

void navigation_setup()
{
  nav_state = NAV_IDLE;
  Serial.println("===== GYRO FOLLOWER INITIALIZED =====");
}

void navigation_update(bool enabled)
{
  if (enabled)
  {
    if (nav_state != nav_last_state) {
      nav_last_state = nav_state;
      Serial.print("State: "); Serial.println(navigation_state_string(nav_state));
    }

    switch (nav_state)
    {
    case NAV_IDLE: state_idle(); break;
    case NAV_FOLLOWING: state_following(); break;
    case NAV_TURNING: state_turning(); break;
    case NAV_CORNER_BRAKING_FOR_REVERSE: state_corner_braking_for_reverse(); break;
    case NAV_CORNER_REVERSING: state_corner_reversing(); break;
    case NAV_CORNER_BRAKING_FOR_ALIGN: state_corner_braking_for_align(); break;
    case NAV_CORNER_ALIGNING: state_corner_aligning(); break;
    case NAV_CORNER_BRAKING_FOR_SECTION_BACKUP: state_corner_braking_for_section_backup(); break;
    case NAV_CORNER_SECTION_BACKING: state_corner_section_backing(); break;
    case NAV_CORNER_BRAKING_AFTER_SECTION_BACKUP: state_corner_braking_after_section_backup(); break;
    case NAV_STOPPED: state_stopped(); break;
    }
  }


}

void navigation_enable()
{
  static bool grid_captured = false;
  if (!grid_captured) {
    grid_captured = true;
  }

  nav_start_angle = get_angle();
  nav_start_distance = current_distance;
  nav_gyro_target = nav_start_angle;
  nav_wall_correction_resume_distance = nav_start_distance;
  
  log_tof_diagnostics("Manual enable -> FOLLOWING");
  nav_state = NAV_FOLLOWING;
  nav_soft_stop_started = false;
  nav_soft_stop_complete = false;
  nav_following_wall = SIDE_UNKNOWN;
  nav_turn_count = 0;
  nav_completed_rounds = 0;
  nav_last_distance_error = 0;
  nav_last_gyro_error = 0;
  navigation_reset_filter();
  nav_searching_for_wall = false;
  nav_long_range_active = false;
  nav_final_slowing = false;
  for (int i = 0; i < 4; ++i)
    nav_learned_straight_mm[i] = 0.0f;

  dc_state = DC_ENABLED;
  servo_disabled = false;
  set_speed(OPEN_CHALLENGE_DISCOVERY_SPEED_MMS);
  
  Serial.print("Initial Task Grid Locked: "); Serial.println(nav_start_angle);
  Serial.println("Following GYRO heading primary, looking for walls...");
  Serial.println("\n===== GYRO FOLLOWING STARTED =====");
}

void navigation_disable()
{
  log_tof_diagnostics("Manual disable -> IDLE");
  nav_state = NAV_IDLE;
  navigation_reset_filter();
  stop();
  set_steering(0);
  Serial.println("Navigation controller disabled");
}

NavigationState navigation_get_state()
{
  return nav_state;
}

const char* navigation_state_string(NavigationState _state)
{
  switch (_state)
  {
    case NAV_IDLE: return "IDLE";
    case NAV_FOLLOWING: return "FOLLOWING";
    case NAV_TURNING: return "TURNING";
    case NAV_CORNER_BRAKING_FOR_REVERSE: return "BRAKING_FOR_REVERSE";
    case NAV_CORNER_REVERSING: return "CORNER_REVERSING";
    case NAV_CORNER_BRAKING_FOR_ALIGN: return "BRAKING_FOR_ALIGN";
    case NAV_CORNER_ALIGNING: return "CORNER_ALIGNING";
    case NAV_CORNER_BRAKING_FOR_SECTION_BACKUP: return "BRAKING_FOR_SECTION_BACKUP";
    case NAV_CORNER_SECTION_BACKING: return "SECTION_BACKING";
    case NAV_CORNER_BRAKING_AFTER_SECTION_BACKUP: return "BRAKING_AFTER_SECTION_BACKUP";
    case NAV_STOPPED: return "STOPPED";
    default: return "UNKNOWN";
  }
}



bool navigation_is_complete()
{
  return nav_state == NAV_STOPPED && nav_soft_stop_complete;
}

void navigation_reset_filter()
{
  nav_steering_filter = 0.0f;
  nav_last_gyro_error = 0.0f;
  nav_distance_pd = 0.0f;
  nav_last_distance_error = 0.0f;
  nav_last_distance_tof_sample = 0;
  nav_last_distance_update_us = 0;
}

float navigation_compute_steering(
    float heading_error_deg,
    float last_error_deg,
    float dt_s)
{
  if (dt_s < 1e-4f)
    dt_s = 1e-4f;

  float gyro_derivative =
      (heading_error_deg - last_error_deg) / dt_s;
  float raw_output =
      nav_gyro_kp * heading_error_deg +
      nav_gyro_kd * gyro_derivative;

  if (fabsf(raw_output) < 0.25f)
    raw_output = 0.0f;

  float alpha = constrain(dt_s / 0.06f, 0.1f, 0.6f);
  float filtered_output =
      nav_steering_filter +
      alpha * (raw_output - nav_steering_filter);

  float max_step = 90.0f * dt_s;
  if (filtered_output - nav_steering_filter > max_step)
    filtered_output = nav_steering_filter + max_step;
  else if (filtered_output - nav_steering_filter < -max_step)
    filtered_output = nav_steering_filter - max_step;

  nav_steering_filter = filtered_output;
  return nav_steering_filter;
}

float navigation_get_gyro_kp()
{
  return nav_gyro_kp;
}

float navigation_get_gyro_kd()
{
  return nav_gyro_kd;
}

void general_debug_set(bool enable)
{
  nav_debug_enabled = enable;
  Serial.println(enable ? "General debug ON" : "General debug OFF");
}

void navigation_print_debug()
{
  Serial.print(" | NavState: ");
  Serial.print(navigation_state_string(nav_state));
  Serial.print(" | Wall: ");
  Serial.print(nav_following_wall == SIDE_LEFT ? "LEFT" : (nav_following_wall == SIDE_RIGHT ? "RIGHT" : "SEARCH"));
  Serial.print(" | Target: ");
  Serial.print(nav_gyro_target, 1);
  if (nav_following_wall != SIDE_UNKNOWN)
  {
    Serial.print(" | WallDist: ");
    Serial.print(get_followed_wall_distance(), 0);
  }
  Serial.print(" | Round: ");
  Serial.print(nav_completed_rounds);
}



void navigation_set_target_distance(float distance_mm)
{
  nav_target_distance = distance_mm;
  Serial.print("Wall target distance set to: "); Serial.print(distance_mm); Serial.println(" mm");
}

void navigation_set_wall_margin(float distance_m)
{
  nav_wall_margin = distance_m * 1000.0;
  Serial.print("Wall margin set to: "); Serial.print(distance_m); Serial.println(" m");
}

void navigation_set_pd_gains(float kp, float kd)
{
  nav_pd_kp = kp;
  nav_pd_kd = kd;
  Serial.print("Distance PD gains: Kp="); Serial.print(kp); Serial.print(", Kd="); Serial.println(kd);
}

void navigation_rearm_after_obstacle()
{
    // Wait until the normal wall has been found again
    // before applying wall-distance correction.
    nav_searching_for_wall = true;

    // Prevent old controller errors from affecting
    // the first control cycle after avoidance.
    nav_last_distance_error = 0;
    nav_last_gyro_error = 0;
}

float navigation_get_target_heading()
{
    return nav_gyro_target;
}

float navigation_get_section_origin_distance()
{
    return nav_start_distance;
}

int navigation_get_turn_count()
{
    return nav_turn_count;
}

int navigation_get_turn_angle()
{
    return nav_turn_angle;
}

WallSide navigation_get_following_wall()
{
  return nav_following_wall;
}

void navigation_set_speed(float speed_mm_s)
{
  if (speed_mm_s < 0)
    speed_mm_s = 0;

  nav_normal_speed = speed_mm_s;
  Serial.print("Navigation controller speed set to: ");
  Serial.print(nav_normal_speed, 0);
  Serial.println(" mm/s");
}

void navigation_set_obstacle_mode(bool enable)
{
  nav_obstacle_mode = enable;
  if (!enable)
    nav_target_distance = OPEN_WALL_TARGET_DISTANCE_MM;
  nav_corner_gap_samples = 0;
  nav_last_corner_tof_sample = get_tof_measurement_count(TOF_LEFT);
}

void navigation_select_wall(
    WallSide side,
    float target_distance_mm)
{
  if (side == SIDE_UNKNOWN)
    return;

  if (target_distance_mm < 120.0f)
    target_distance_mm = 120.0f;
  if (target_distance_mm > 350.0f)
    target_distance_mm = 350.0f;

  if (nav_following_wall != side ||
      fabsf(nav_target_distance - target_distance_mm) > 1.0f)
  {
    nav_following_wall = side;
    nav_target_distance = target_distance_mm;
    nav_searching_for_wall = true;
    nav_last_distance_error = 0;
    nav_distance_pd = 0;
    nav_last_distance_tof_sample = 0;
    nav_last_distance_update_us = 0;

    Serial.print("[NAV] Planned lane ");
    Serial.print(side == SIDE_LEFT ? "LEFT" : "RIGHT");
    Serial.print(" wall_mm=");
    Serial.println(target_distance_mm, 0);
  }
}
