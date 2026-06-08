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
float wf_wall_margin = 800.0;         // 800.0mm threshold to detect gap/open space
int wf_turn_angle = 0;                // +90 or -90 degrees
bool wf_following_left_wall = false;  // Which wall are we following

// Timing and control
unsigned long wf_enable_pressed_time = 0;
unsigned long wf_turn_start_angle = 0;
unsigned long wf_turn_target_angle = 0;

// Round counting
int wf_turn_count = 0;
float wf_start_angle = 0;
int wf_completed_rounds = 0;

// PD Controller
float wf_pd_kp = 0.5;                 // Proportional gain
float wf_pd_kd = 0.001;                 // Derivative gain
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
 * @brief Calculate the target angle for a specific turn
 * @param turn_angle Desired turn angle in degrees (+90 for right, -90 for left)
 * @param current_angle Current heading angle in degrees
 * @return Target angle in degrees
 */
float calculate_target_angle(float turn_angle, float current_angle=get_angle())
{
  return current_angle + turn_angle;
}

/**
 * @brief Get the sensor distance for the followed wall
 * @param following_left_wall True if following left wall, false for right wall
 * @return Distance in millimeters
 */
float get_followed_wall_distance(bool following_left_wall=wf_following_left_wall)
{
  if (following_left_wall)
  {
    return current_distance_left;
  }
  else
  {
    return current_distance_right;
  }
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
  float opposite_distance = get_followed_wall_distance(!wf_following_left_wall);

  // ==========================================
  // Check for wall gaps (distance > wall_margin)
  // ==========================================
  if (current_wall_distance > wf_wall_margin && current_wall_distance > 0)
  {
    // current wall is too far - gap detected, time to turn
    if (wf_following_left_wall)
    {
      // Following left wall, gap on left -> turn left (+90°)
      wf_turn_angle = 90;
      wf_turn_count++;
    }
    else
    {
      // Following right wall, gap on right -> turn right (-90°)
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
    wf_turn_start_angle = get_angle();
    // wf_turn_target_angle = calculate_target_angle(wf_turn_angle, wf_turn_start_angle);

    Serial.print("TURN ");
    Serial.print(wf_turn_count);
    Serial.print(": ");
    Serial.print(wf_turn_angle > 0 ? "LEFT" : "RIGHT");
    Serial.print(" | Round: ");
    Serial.println(wf_completed_rounds);

    return;
  }

  // ==========================================
  // Distance control using PD controller
  // ==========================================
  if (current_wall_distance > 0) // Valid sensor reading
  {
    // Calculate error (positive = too far, negative = too close)
    float error = current_wall_distance - wf_target_distance;

    // Calculate derivative term
    float error_derivative = (error - wf_last_distance_error) / last_loop_time;

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
  if (wf_turn_angle > 0)
  {
    set_steering(-40); // left turn
  }
  else
  {
    set_steering(40); // right turn
  }
  set_speed(150); // Slightly slower during turn

  // Check if turn is complete
  if ((get_angle() - wf_turn_start_angle - wf_turn_angle) * wf_turn_angle/fabs(wf_turn_angle) > 0)
  {
    Serial.print("Turn complete. Current heading: ");
    Serial.print(get_angle());
    Serial.print("° | Start angle: ");
    Serial.print(wf_turn_start_angle);
    Serial.print("° | Turn angle: ");
    Serial.print(wf_turn_angle);
    Serial.println("°");
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
  wf_state = WF_IDLE;
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

  Serial.println("===== WALL FOLLOWER INITIALIZED =====");
  Serial.println("Waiting for enable signal...");
  Serial.println("Commands: 'w' = start, 's' = stop");
}

void wall_follower_update(bool enabled)
{
  if (enabled)
  {
    // Check for state change
    if (wf_state != wf_last_state)
    {
      Serial.print("State change: ");
      Serial.print(wall_follower_state_string(wf_last_state));
      Serial.print(" -> ");
      wf_last_state = wf_state;
      Serial.println(wall_follower_state_string(wf_state));
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
    wf_start_angle = get_angle();
    wf_state = WF_FOLLOWING;
    wf_following_left_wall = false; // Start by following right wall
    wf_turn_count = 0;
    wf_completed_rounds = 0;
    wf_last_distance_error = 0;

    // Enable motors and servo
    dc_state = DC_ENABLED;
    servo_disabled = false;
    set_speed(200); // Start moving at 200 mm/s

    Serial.println("\n===== WALL FOLLOWING STARTED =====");
    Serial.print("Start heading: ");
    Serial.print(wf_start_angle);
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

const char* wall_follower_state_string(WallFollowerState _wf_state)
{
  switch (_wf_state)
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
  Serial.print(wf_following_left_wall ? "LEFT " : "RIGHT");
  Serial.print(" | Dist: ");
  Serial.print(get_followed_wall_distance(), 2);
  Serial.print("m | Opp: ");
  Serial.print(get_followed_wall_distance(!wf_following_left_wall), 2);
  Serial.print("m | Turns: ");
  Serial.print(wf_turn_count, 2);
  Serial.print(" | Rounds: ");
  Serial.print(wf_completed_rounds, 2);
  Serial.print(" | Heading: ");
  Serial.print(get_angle(), 1);
  Serial.print("° | Speed: ");
  Serial.print(current_speed, 0);
  Serial.print(" mm/s | Last loop: ");
  Serial.print(last_loop_time, 4);
  Serial.println("s");
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
