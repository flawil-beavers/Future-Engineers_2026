/**
 * @file wall_follower.cpp
 * @brief Wall-following autonomous control implementation
 */

#include "wall_follower.h"
#include "config.h"
#include "motor_control.h"
#include "sensors.h"

// ==========================================
// STATE VARIABLES
// ==========================================

WallFollowerState wf_state = WF_IDLE;
WallFollowerState wf_last_state = WF_IDLE;

// Wall following parameters
float wf_target_distance = 300.0;     // 300mm target distance from wall
float wf_wall_margin = 1.0;           // 1.0m threshold to detect gap/open space
int wf_turn_angle = 0;                // +90 or -90 degrees
bool wf_following_left_wall = false;  // Which wall are we following

// Timing and control
unsigned long wf_enable_pressed_time = 0;
unsigned long wf_turn_start_time = 0;
unsigned long wf_turn_duration_ms = 0; // Will be calculated based on target speed

// Round counting
int wf_turn_count = 0;
float wf_start_heading = 0;
int wf_completed_rounds = 0;

// PD Controller
float wf_pd_kp = 0.5;                 // Proportional gain
float wf_pd_kd = 0.1;                 // Derivative gain
float wf_last_distance_error = 0;

// Debug
bool wf_debug_enabled = false;
unsigned long wf_last_debug_time = 0;

// Time tracking
extern unsigned long current_time;

// ==========================================
// HELPER FUNCTIONS
// ==========================================

/**
 * @brief Calculate how long a 90-degree turn should take
 * Based on target speed and servo angle
 * @return Duration in milliseconds
 */
unsigned long calculate_turn_duration()
{
  // Assuming 90° turn at moderate turning speed
  // This is a rough estimate - may need tuning
  // For a robot with target_speed = 200 mm/s doing a tight turn
  // A 90 degree turn in roughly 1-2 seconds is reasonable
  return 1500; // 1.5 seconds for 90 degree turn
}

/**
 * @brief Get the sensor distance for the followed wall
 * @return Distance in meters
 */
float get_followed_wall_distance()
{
  if (wf_following_left_wall)
  {
    return current_distance_left_m;
  }
  else
  {
    return current_distance_right_m;
  }
}

/**
 * @brief Get the sensor distance for the opposite wall
 * @return Distance in meters
 */
float get_opposite_wall_distance()
{
  if (wf_following_left_wall)
  {
    return current_distance_right_m;
  }
  else
  {
    return current_distance_left_m;
  }
}

/**
 * @brief Check if heading has completed a full rotation back to start
 * Used to count complete rounds
 * @return True if heading is within ~30° of start heading
 */
bool heading_completed_rotation()
{
  float heading_diff = current_degree - wf_start_heading;

  // Normalize to -180 to +180 range
  if (heading_diff > 180)
    heading_diff -= 360;
  if (heading_diff < -180)
    heading_diff += 360;

  // Check if we're back at starting heading (within 30° tolerance)
  return fabs(heading_diff) < 30;
}

/**
 * @brief Normalize angle to -180 to +180 range
 */
float normalize_angle(float angle)
{
  while (angle > 180)
    angle -= 360;
  while (angle < -180)
    angle += 360;
  return angle;
}

// ==========================================
// STATE FUNCTIONS
// ==========================================

/**
 * @brief Handle IDLE state
 * Waits for enable signal
 */
void state_idle()
{
  // Robot is stopped and waiting
  // enable signal comes from serial command or physical switch
  // (handled in wall_follower_enable())
}

/**
 * @brief Handle FOLLOWING state
 * Follow wall at target distance using PD controller
 */
void state_following_wall()
{
  float current_wall_distance = get_followed_wall_distance();
  float opposite_distance = get_opposite_wall_distance();

  // ==========================================
  // Check for wall gaps (distance > wall_margin)
  // ==========================================
  if (opposite_distance > wf_wall_margin && opposite_distance > 0)
  {
    // Opposite wall is too far - gap detected, time to turn
    if (wf_following_left_wall)
    {
      // Following left wall, gap on right -> turn right (+90°)
      wf_turn_angle = 90;
      wf_turn_count++;
    }
    else
    {
      // Following right wall, gap on left -> turn left (-90°)
      wf_turn_angle = -90;
      wf_turn_count++;
    }

    // Check if we completed a full round
    // After 4 turns (90° each), we've gone around 360°
    if (wf_turn_count % 4 == 0)
    {
      wf_completed_rounds = wf_turn_count / 4;
    }

    // If completed 3 rounds (12 turns), stop after this turn
    if (wf_completed_rounds >= 3)
    {
      wf_state = WF_STOPPED;
      return;
    }

    // Transition to TURNING state
    wf_state = WF_TURNING;
    wf_turn_start_time = millis();
    wf_turn_duration_ms = calculate_turn_duration();

    // Toggle which wall we're following
    wf_following_left_wall = !wf_following_left_wall;

    Serial.print("TURN ");
    Serial.print(wf_turn_count);
    Serial.print(": ");
    Serial.print(wf_turn_angle > 0 ? "RIGHT" : "LEFT");
    Serial.print(" | Round: ");
    Serial.println(wf_completed_rounds);

    return;
  }

  // ==========================================
  // Distance control using PD controller
  // ==========================================
  if (current_wall_distance > 0) // Valid sensor reading
  {
    // Convert to mm for consistency
    float distance_mm = current_wall_distance * 1000.0;

    // Calculate error (positive = too far, negative = too close)
    float error = distance_mm - wf_target_distance;

    // Calculate derivative term
    float error_derivative = error - wf_last_distance_error;

    // PD output: steering angle based on distance error
    float pd_output = wf_pd_kp * error + wf_pd_kd * error_derivative;

    // Limit steering angle (servo can't go beyond limits)
    if (pd_output > 60)
      pd_output = 60;
    if (pd_output < -60)
      pd_output = -60;

    // Apply steering
    if (wf_following_left_wall)
    {
      // Following left wall: positive error = too far, need to turn left (negative angle)
      set_steering(-pd_output);
    }
    else
    {
      // Following right wall: positive error = too far, need to turn right (positive angle)
      set_steering(pd_output);
    }

    wf_last_distance_error = error;
  }

  // Maintain speed
  if (current_speed == 0)
  {
    set_speed(200); // 200 mm/s
  }
}

/**
 * @brief Handle TURNING state
 * Execute 90-degree turn
 */
void state_turning()
{
  unsigned long turn_elapsed = millis() - wf_turn_start_time;

  // Execute turn at full steering angle
  if (wf_turn_angle > 0)
  {
    set_steering(60); // Full right
  }
  else
  {
    set_steering(-60); // Full left
  }

  // Maintain speed during turn
  if (current_speed == 0)
  {
    set_speed(150); // Slightly slower during turn
  }

  // Check if turn is complete
  if (turn_elapsed >= wf_turn_duration_ms)
  {
    // Turn complete, resume wall following
    wf_state = WF_FOLLOWING;
    set_steering(0); // Center steering
  }
}

/**
 * @brief Handle STOPPED state
 * Robot has completed 3 rounds, stop in middle of straight section
 */
void state_stopped()
{
  // Stop the robot
  set_speed(0);
  set_steering(0);
  stop();

  Serial.println("===== WALL FOLLOWING COMPLETE =====");
  Serial.print("Total turns: ");
  Serial.print(wf_turn_count);
  Serial.print(" | Complete rounds: ");
  Serial.println(wf_completed_rounds);
}

// ==========================================
// PUBLIC INTERFACE
// ==========================================

void wall_follower_setup()
{
  wf_state = WF_IDLE;
  wf_turn_count = 0;
  wf_completed_rounds = 0;
  wf_last_distance_error = 0;
  wf_turn_duration_ms = calculate_turn_duration();

  Serial.println("===== WALL FOLLOWER INITIALIZED =====");
  Serial.println("Waiting for enable signal...");
  Serial.println("Commands: 'w' = start, 's' = stop");
}

void wall_follower_update()
{
  // Check for state change
  if (wf_state != wf_last_state)
  {
    Serial.print("State change: ");
    Serial.print(wall_follower_state_string());
    Serial.print(" -> ");
    wf_last_state = wf_state;
    Serial.println(wall_follower_state_string());
  }

  // Execute state logic
  switch (wf_state)
  {
  case WF_IDLE:
    state_idle();
    break;

  case WF_FOLLOWING:
    state_following_wall();
    break;

  case WF_TURNING:
    state_turning();
    break;

  case WF_STOPPED:
    state_stopped();
    break;
  }

  // Print debug info if enabled
  if (wf_debug_enabled)
  {
    wall_follower_print_debug();
  }
}

void wall_follower_enable()
{
  if (wf_state == WF_IDLE)
  {
    wf_start_heading = current_degree;
    wf_state = WF_FOLLOWING;
    wf_following_left_wall = false; // Start by following right wall
    wf_turn_count = 0;
    wf_completed_rounds = 0;
    wf_last_distance_error = 0;

    // Enable motors and servo
    disable_dc = false;
    disable_servo = false;
    set_speed(200); // Start moving at 200 mm/s

    Serial.println("\n===== WALL FOLLOWING STARTED =====");
    Serial.print("Start heading: ");
    Serial.print(wf_start_heading);
    Serial.println("°");
    Serial.println("Following RIGHT wall initially");
  }
}

void wall_follower_disable()
{
  wf_state = WF_IDLE;
  stop();
  set_steering(0);

  Serial.println("Wall follower disabled");
  Serial.print("Completed: ");
  Serial.print(wf_turn_count);
  Serial.print(" turns, ");
  Serial.print(wf_completed_rounds);
  Serial.println(" rounds");
}

WallFollowerState wall_follower_get_state()
{
  return wf_state;
}

const char* wall_follower_state_string()
{
  switch (wf_state)
  {
  case WF_IDLE:
    return "IDLE";
  case WF_FOLLOWING:
    return "FOLLOWING";
  case WF_TURNING:
    return "TURNING";
  case WF_STOPPED:
    return "STOPPED";
  default:
    return "UNKNOWN";
  }
}

void wall_follower_set_debug(bool enable)
{
  wf_debug_enabled = enable;
  if (enable)
  {
    Serial.println("Wall follower debug ON");
  }
  else
  {
    Serial.println("Wall follower debug OFF");
  }
}

void wall_follower_print_debug()
{
  // Throttle debug output to every 500ms
  if (current_time - wf_last_debug_time < 500000)
  {
    return;
  }
  wf_last_debug_time = current_time;

  Serial.print("[WF] State: ");
  Serial.print(wall_follower_state_string());
  Serial.print(" | Wall: ");
  Serial.print(wf_following_left_wall ? "LEFT" : "RIGHT");
  Serial.print(" | Dist: ");
  Serial.print(get_followed_wall_distance(), 2);
  Serial.print("m | Opp: ");
  Serial.print(get_opposite_wall_distance(), 2);
  Serial.print("m | Turns: ");
  Serial.print(wf_turn_count);
  Serial.print(" | Rounds: ");
  Serial.print(wf_completed_rounds);
  Serial.print(" | Heading: ");
  Serial.print(current_degree, 1);
  Serial.print("° | Speed: ");
  Serial.print(current_speed);
  Serial.println(" mm/s");
}

void wall_follower_set_target_distance(float distance_mm)
{
  wf_target_distance = distance_mm;
  Serial.print("Wall target distance set to: ");
  Serial.print(distance_mm);
  Serial.println(" mm");
}

void wall_follower_set_wall_margin(float distance_m)
{
  wf_wall_margin = distance_m;
  Serial.print("Wall margin set to: ");
  Serial.print(distance_m);
  Serial.println(" m");
}

void wall_follower_set_pd_gains(float kp, float kd)
{
  wf_pd_kp = kp;
  wf_pd_kd = kd;
  Serial.print("Wall follower PD gains: Kp=");
  Serial.print(kp);
  Serial.print(", Kd=");
  Serial.println(kd);
}
