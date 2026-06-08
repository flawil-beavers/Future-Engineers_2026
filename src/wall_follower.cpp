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
WallSide wf_following_wall = SIDE_RIGHT; // Which wall are we following

// Gyro following parameters
float wf_gyro_target = 0;             // Target gyro angle
float wf_gyro_kp = 2.5;               // Proportional gain
float wf_gyro_kd = 0.05;              // Derivative gain
float wf_last_gyro_error = 0;

// Timing and control
float wf_turn_start_angle = 0;
float wf_turn_target_angle = 0;

// Round counting
int wf_turn_count = 0;
float wf_start_angle = 0;
int wf_completed_rounds = 0;

// Internal logic flags
bool wf_searching_for_wall = false;   // True when waiting to "re-acquire" a wall after a turn

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
 * @param side Which wall side to check
 * @return Distance in millimeters
 */
float get_followed_wall_distance(WallSide side=wf_following_wall)
{
  return get_tof_distance(side == SIDE_LEFT ? TOF_LEFT : TOF_RIGHT);
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
 * @brief Handle GYRO_FOLLOW state
 * Uses gyro PD control to stay straight. 
 * 1. If turn_count is 0, it looks for the first gap to decide direction.
 * 2. Otherwise, it waits until a wall is detected to switch to PD.
 */
void state_gyro_follow()
{
  // PD controller for heading
  // Error: positive = drifted left (angle > target), negative = drifted right
  float error = get_angle() - wf_gyro_target;
  float derivative = (error - wf_last_gyro_error) / last_loop_time;
  float pd_output = wf_gyro_kp * error + wf_gyro_kd * derivative;
  wf_last_gyro_error = error;

  // Limit steering angle
  if (pd_output > 50) pd_output = 50;
  if (pd_output < -50) pd_output = -50;

  // Apply steering (positive pd_output results in right turn to correct left drift)
  set_steering(pd_output);
  set_speed(200);
  float dist_left = get_tof_distance(TOF_LEFT);
  float dist_right = get_tof_distance(TOF_RIGHT);

  // LOGIC A: Initial search for first gap
  if (wf_turn_count == 0)
  {
    if (dist_left > wf_wall_margin && dist_left > 0)
    {
      wf_following_wall = SIDE_LEFT;
      wf_turn_angle = 90;
      wf_turn_count++;
      wf_state = WF_TURNING;
      wf_turn_start_angle = get_angle();
      Serial.println("Initial gap detected: LEFT");
      return;
    }
    else if (dist_right > wf_wall_margin && dist_right > 0)
    {
      wf_following_wall = SIDE_RIGHT;
      wf_turn_angle = -90;
      wf_turn_count++;
      wf_state = WF_TURNING;
      wf_turn_start_angle = get_angle();
      Serial.println("Initial gap detected: RIGHT");
      return;
    }
  }
  // LOGIC B: Searching for wall after a turn
  else if (wf_searching_for_wall)
  {
    float current_side_dist = get_followed_wall_distance();
    // If we see a wall closer than a reasonable threshold (e.g., target + 300mm)
    if (current_side_dist > 0 && current_side_dist < (wf_target_distance + 300.0))
    {
      Serial.println("Wall re-acquired. Switching to PD control.");
      wf_searching_for_wall = false;
      wf_state = WF_FOLLOWING;
      wf_last_distance_error = 0; // Reset D term
    }
  }
}

/**
 * @brief Handle FOLLOWING state
 * Follow wall at target distance using PD controller
 */
void state_following_wall()
{
  float current_wall_distance = get_followed_wall_distance();

  // ==========================================
  // Check for wall gaps (distance > wall_margin)
  // ==========================================
  if (current_wall_distance > wf_wall_margin && current_wall_distance > 0)
  {
    // Gap detected on the wall we are following
    if (wf_following_wall == SIDE_LEFT)
    {
      wf_turn_angle = 90;
    }
    else
    {
      wf_turn_angle = -90;
    }
    
    wf_turn_count++;

    // Check if we completed a full round
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
    if (wf_following_wall == SIDE_LEFT)
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
    wf_gyro_target += wf_turn_angle; // Calculate new straight angle
    wf_last_gyro_error = 0;          // Reset D term
    wf_searching_for_wall = true;
    wf_state = WF_GYRO_FOLLOW;
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
    case WF_GYRO_FOLLOW:
      state_gyro_follow();
      break;

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
    wf_gyro_target = wf_start_angle;
    wf_state = WF_GYRO_FOLLOW;
    wf_following_wall = SIDE_RIGHT; // Start by following right wall
    wf_turn_count = 0;
    wf_completed_rounds = 0;
    wf_last_distance_error = 0;
    wf_last_gyro_error = 0;
    wf_searching_for_wall = false;

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
  case WF_GYRO_FOLLOW:
    return "GYRO_FOLLOW";
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
  Serial.print(wf_following_wall == SIDE_LEFT ? "LEFT " : "RIGHT");
  Serial.print(" | Dist: ");
  Serial.print(get_followed_wall_distance(), 2);
  Serial.print("m | Opp: ");
  Serial.print(get_followed_wall_distance(wf_following_wall == SIDE_LEFT ? SIDE_RIGHT : SIDE_LEFT), 2);
  Serial.print("m | Turns: ");
  Serial.print(wf_turn_count);
  Serial.print(" | Rounds: ");
  Serial.print(wf_completed_rounds);
  Serial.print(" | Heading: ");
  Serial.print(get_angle(), 1);
  Serial.print("° | Speed: ");
  Serial.print(current_speed, 0);
  Serial.print(" mm/s | Last loop: ");
  Serial.print(last_loop_time * 1000, 4);
  Serial.println("ms");
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
