/**
 * @file wall_follower.cpp
 * @brief Gyro-Stabilized Wall Following Control Implementation
 * 
 * This module implements a hybrid navigation strategy:
 * 1. Primary Control: Gyroscope-based PD loop maintains a global grid heading (0, 90, 180, 270).
 * 2. Secondary Control: ToF-based PD loop maintains a lateral offset from the wall.
 * 3. Geometric Correction: Wall distance is corrected using the gyro angle to ensure
 *    accuracy even when the robot is not perfectly parallel to the wall.
 * 
 * State Machine:
 * - GF_IDLE: Waiting for activation.
 * - GF_FOLLOWING: Moving straight using Gyro + ToF.
 * - GF_TURNING: Executing a 90-degree pivot turn based on gyro feedback.
 * - GF_STOPPED: Mission complete (3 rounds finished).
 */

#include "wall_follower.h"
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

GyroFollowerState gf_state = GF_IDLE;
GyroFollowerState gf_last_state = GF_IDLE;

// Wall following parameters
float gf_target_distance = 300.0;     // 300mm target distance from wall
float gf_wall_margin = TOF_MAX_RELIABLE_DISTANCE_MM; // Threshold to detect gap/open space (mm)
int gf_turn_angle = 0;                // +90 or -90 degrees
WallSide gf_following_wall = SIDE_UNKNOWN; // Which wall are we following

// Gyro following parameters
float gf_gyro_target = 0;             // Target gyro angle
float gf_gyro_kp = 2.5;               // Proportional gain
float gf_gyro_kd = 0.05;              // Derivative gain
float gf_last_gyro_error = 0;

// Timing and control
float gf_turn_start_angle = 0;

// Round counting
int gf_turn_count = 0;
float gf_start_angle = 0;
float gf_start_distance = 0;
int gf_completed_rounds = 0;

// Speed parameters
float gf_normal_speed = 300.0;        // Default normal speed (mm/s) 400 is the maximal speed without stalling on the 50:1 motor

// Internal logic flags
bool gf_searching_for_wall = false;   // True when waiting to "re-acquire" a wall after a turn
bool gf_long_range_active = false;    // Tracks if ToF is in discovery (slow) mode

// PD Controller
float gf_pd_kp = 0.5;                 // Proportional gain
float gf_pd_kd = 0.01;                // Derivative gain
float gf_last_distance_error = 0;

// Telemetry and Debug
bool gf_debug_enabled = false;
unsigned long gf_last_debug_time = 0;

// Time tracking
extern unsigned long current_time;
extern float last_loop_time;

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

  if (raw_dist <= 0 || raw_dist >= TOF_OUT_OF_RANGE_MM) {
    return raw_dist;
  }

  float angle_error_deg = get_angle() - gf_gyro_target;
  float incidence_angle_rad = angle_error_deg * PI / 180.0f;
  return raw_dist * cos(incidence_angle_rad);
}

float get_followed_wall_distance()
{
  if (gf_following_wall == SIDE_UNKNOWN)
  {
    float dist_l = get_tof_distance(TOF_LEFT);
    float dist_r = get_tof_distance(TOF_RIGHT);
    return (dist_l > dist_r) ? dist_l : dist_r;
  }
  return get_followed_wall_distance(gf_following_wall);
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
  float current_deg = get_angle()-gf_start_angle;
  float distance_since_last_state = current_distance - gf_start_distance;

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
  if (gf_following_wall == SIDE_UNKNOWN && !gf_long_range_active) {
    sensors_set_tof_timing_budget(300000); // 300ms for long-range discovery
    gf_long_range_active = true;
  } else if (gf_following_wall != SIDE_UNKNOWN && gf_long_range_active) {
    sensors_set_tof_timing_budget(sensors_initial_tof_timing_budget); // Restore initial value
    gf_long_range_active = false;
  }

  // 1. Primary: Gyro Heading Control
  float gyro_error = get_angle() - gf_gyro_target;
  float gyro_derivative = (gyro_error - gf_last_gyro_error) / last_loop_time;
  float gyro_pd = gf_gyro_kp * gyro_error + gf_gyro_kd * gyro_derivative;
  gf_last_gyro_error = gyro_error;

  // 2. Secondary: Wall Distance Correction
  float current_wall_distance = get_followed_wall_distance();
  float dist_pd = 0;

  /**
   * DETECTION LOGIC: Trigger Turn
   * 
   * If the wall disappears (reading exceeds margin), it indicates a corner.
   * The robot will increment its turn count and transition to GF_TURNING.
   */
  // Increase blind distance specifically after the first turn to ignore 
  // potential false gaps or handling specific track geometry.
  float blind_dist = (gf_turn_count == 1) ? 600.0f : 300.0f;
  float wall_margin = gf_long_range_active ? 1500 : gf_wall_margin;
  if (gf_start_distance + blind_dist < get_distance() && current_wall_distance > wall_margin && current_wall_distance > 0)
  {
    if (gf_following_wall == SIDE_UNKNOWN)
    {
      // Searching for initial direction
      float dist_left = get_tof_distance(TOF_LEFT);
      float dist_right = get_tof_distance(TOF_RIGHT);
      if (dist_left > wall_margin && dist_left > 0)
      {
        gf_following_wall = SIDE_LEFT;
        gf_turn_angle = 90;
        Serial.print("Dist Left: ");
        Serial.println(dist_left);
      }
      else if (dist_right > wall_margin && dist_right > 0)
      {
        gf_following_wall = SIDE_RIGHT;
        gf_turn_angle = -90;
        Serial.print("Dist Right: ");
        Serial.println(dist_right);
      }
      else
        return;

      // Side determined. Restore budget immediately.
      sensors_set_tof_timing_budget(sensors_initial_tof_timing_budget);
      gf_long_range_active = false;
    }
    else
    {
      gf_turn_angle = (gf_following_wall == SIDE_LEFT) ? 90 : -90;
    }

    gf_turn_count++;
    gf_completed_rounds = (int)(gf_turn_count / 4);

    log_tof_diagnostics("Corner detected -> TURNING");
    gf_state = GF_TURNING;
    gf_turn_start_angle = get_angle();
    return;
  }

  /**
   * DISTANCE PD CONTROL
   * 
   * If a wall is within range, calculate the error from target (300mm).
   * This term is added to the gyro steering to gently nudge the robot 
   * away from or toward the wall while maintaining heading.
   */
  if (current_wall_distance > 0 && current_wall_distance < gf_wall_margin)
  {
    if (gf_searching_for_wall) {
      if (current_wall_distance < (gf_target_distance + 100.0)) gf_searching_for_wall = false;
    }

    if (!gf_searching_for_wall) {
      float dist_error = current_wall_distance - gf_target_distance;
      float dist_derivative = (dist_error - gf_last_distance_error) / last_loop_time;
      dist_pd = gf_pd_kp * dist_error + gf_pd_kd * dist_derivative;
      gf_last_distance_error = dist_error;
    }
    if (gf_completed_rounds >= 3 && gf_start_distance + 500 < get_distance())
    {
      log_tof_diagnostics("Rounds finished -> STOPPED");
      gf_state = GF_STOPPED;
    }

  }

  // Combine Steering: Gyro + Distance Correction
  float total_steering = gyro_pd;
  if (gf_following_wall == SIDE_LEFT) total_steering -= dist_pd;
  else if (gf_following_wall == SIDE_RIGHT) total_steering += dist_pd;

  // Clamp to physical servo limits
  if (total_steering > 60) total_steering = 60;
  if (total_steering < -60) total_steering = -60;

  set_steering(total_steering);
  set_speed(gf_normal_speed);
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
  set_steering(gf_turn_angle > 0 ? -25 : 25);
  set_speed(gf_normal_speed);

  if ((get_angle() - gf_turn_start_angle - gf_turn_angle) * gf_turn_angle/fabs(gf_turn_angle) > 0)
  {
    gf_gyro_target += gf_turn_angle;
    gf_last_gyro_error = 0;
    gf_searching_for_wall = true;
    gf_start_distance = get_distance();
    log_tof_diagnostics("Turn finished -> FOLLOWING");
    gf_state = GF_FOLLOWING;
    set_steering(0);
  }
}

/**
 * @brief Shutdown Logic.
 * 
 * Reached after 12 corners (3 laps). Stops motors and centers steering.
 */
void state_stopped()
{
  set_speed(0);
  set_steering(0);
  stop();
  
  Serial.println("===== GYRO TASK COMPLETE =====");
  Serial.print("Total turns: "); Serial.print(gf_turn_count);
  Serial.print(" | Complete rounds: "); Serial.println(gf_completed_rounds);
  
  log_tof_diagnostics("Mission complete -> IDLE");
  gf_state = GF_IDLE;
  robot_logger.write_to_usb();
}


// ==========================================
// PUBLIC INTERFACE
// ==========================================

void gyro_follower_setup()
{
  gf_state = GF_IDLE;
  Serial.println("===== GYRO FOLLOWER INITIALIZED =====");
}

void gyro_follower_update(bool enabled)
{
  if (enabled)
  {
    if (gf_state != gf_last_state) {
      gf_last_state = gf_state;
      Serial.print("State: "); Serial.println(gyro_follower_state_string(gf_state));
    }

    switch (gf_state)
    {
    case GF_IDLE: state_idle(); break;
    case GF_FOLLOWING: state_following(); break;
    case GF_TURNING: state_turning(); break;
    case GF_STOPPED: state_stopped(); break;
    }
  }

  if (gf_debug_enabled) gyro_follower_print_debug();
}

void gyro_follower_enable()
{
  static bool grid_captured = false;
  if (!grid_captured) {
    grid_captured = true;
  }

  gf_start_angle = get_angle();
  gf_start_distance = current_distance;
  gf_gyro_target = gf_start_angle;
  
  log_tof_diagnostics("Manual enable -> FOLLOWING");
  gf_state = GF_FOLLOWING;
  gf_following_wall = SIDE_UNKNOWN;
  gf_turn_count = 0;
  gf_completed_rounds = 0;
  gf_last_distance_error = 0;
  gf_last_gyro_error = 0;
  gf_searching_for_wall = false;
  gf_long_range_active = false;

  dc_state = DC_ENABLED;
  servo_disabled = false;
  set_speed(gf_normal_speed);
  
  Serial.print("Initial Task Grid Locked: "); Serial.println(gf_start_angle);
  Serial.println("Following GYRO heading primary, looking for walls...");
  Serial.println("\n===== GYRO FOLLOWING STARTED =====");
}

void gyro_follower_disable()
{
  log_tof_diagnostics("Manual disable -> IDLE");
  gf_state = GF_IDLE;
  stop();
  set_steering(0);
  Serial.println("Gyro follower disabled");
}

GyroFollowerState gyro_follower_get_state()
{
  return gf_state;
}

const char* gyro_follower_state_string(GyroFollowerState _state)
{
  switch (_state)
  {
    case GF_IDLE: return "IDLE";
    case GF_FOLLOWING: return "FOLLOWING";
    case GF_TURNING: return "TURNING";
    case GF_STOPPED: return "STOPPED";
    default: return "UNKNOWN";
  }
}

void gyro_follower_set_debug(bool enable)
{
  gf_debug_enabled = enable;
  Serial.println(enable ? "Gyro follower debug ON" : "Gyro follower debug OFF");
}

void gyro_follower_print_debug()
{
  if (current_time - gf_last_debug_time < 200000)
    return;
  gf_last_debug_time = current_time;

  Serial.print("[GF] State: ");
  Serial.print(gyro_follower_state_string(gf_state));
  Serial.print(" | Wall: ");
  Serial.print(gf_following_wall == SIDE_LEFT ? "LEFT" : (gf_following_wall == SIDE_RIGHT ? "RIGHT" : "SEARCH"));
  Serial.print(" | Target: ");
  Serial.print(gf_gyro_target, 1);
  Serial.print(" | Angle: ");
  Serial.print(get_angle(), 1);
  if (gf_following_wall != SIDE_UNKNOWN)
  {
    Serial.print(" | Dist: ");
    Serial.print(get_followed_wall_distance(), 0);
  }
  Serial.print(" | Round: ");
  Serial.print(gf_completed_rounds);
  Serial.print(" | Tof Left: ");
  Serial.print(get_tof_distance(TOF_LEFT), 0);
  Serial.print(" | Tof Right: ");
  Serial.print(get_tof_distance(TOF_RIGHT), 0);
  Serial.print(" | long range active: ");
  Serial.print(gf_long_range_active);
  Serial.print(" | Distance: ");
  Serial.print(get_distance(), 0);
  Serial.println();
}

void gyro_follower_set_target_distance(float distance_mm)
{
  gf_target_distance = distance_mm;
  Serial.print("Wall target distance set to: "); Serial.print(distance_mm); Serial.println(" mm");
}

void gyro_follower_set_wall_margin(float distance_m)
{
  gf_wall_margin = distance_m * 1000.0;
  Serial.print("Wall margin set to: "); Serial.print(distance_m); Serial.println(" m");
}

void gyro_follower_set_pd_gains(float kp, float kd)
{
  gf_pd_kp = kp;
  gf_pd_kd = kd;
  Serial.print("Distance PD gains: Kp="); Serial.print(kp); Serial.print(", Kd="); Serial.println(kd);
}

void gyro_follower_rearm_after_obstacle()
{
    // Restart corner detection distance from the
    // current position after an avoidance maneuver.
    gf_start_distance = get_distance();

    // Wait until the normal wall has been found again
    // before applying wall-distance correction.
    gf_searching_for_wall = true;

    // Prevent old controller errors from affecting
    // the first control cycle after avoidance.
    gf_last_distance_error = 0;
    gf_last_gyro_error = 0;
}

float gyro_follower_get_target_heading()
{
    return gf_gyro_target;
}

int gyro_follower_get_turn_count()
{
    return gf_turn_count;
}

int gyro_follower_get_turn_angle()
{
    return gf_turn_angle;
}