/**
 * @file serial_handler.cpp
 * @brief Serial communication and command parsing implementation
 * 
 * Supported Commands:
 *   d<val> : Set drive speed (mm/s)
 *   s<val> : Set steering angle (degrees)
 *   p      : Emergency system disable
 *   h      : Hold position (stop)
 *   r      : Resume with previous speed
 *   m      : Master system enable
 *   n      : Print encoder distance
 *   g      : Print gyro heading
 *   v      : Print ToF sensor readings
 *   q<val> : Set motor Kp (val/10)
 *   w<val> : Set motor Ki (val/100)
 *   e<val> : Set motor Kd (val/10)
 *   a<val> : Set default acceleration
 *   x      : Debug steering timing
 *   m      : MANUAL mode
 *   l      : OPEN CHALLENGE mode
 *   O      : OBSTACLE CHALLENGE mode
 *   b1/b0  : OBSTACLE BENCH mode on/off
 *   c      : CAMERA CALIBRATION mode
 *   C      : TURN RADIUS CALIBRATION mode
 *   B      : SERVO CENTER CALIBRATION mode
 *   y      : PID AUTOTUNE mode
 *   z      : STOP active mode
 *   u<val> : Set wall distance (mm)
 *   i/o    : Wall Follower Debug ON/OFF
 */

#include "serial_handler.h"
#include "config.h"
#include "motor_control.h"
#include "sensors.h"
#include "wall_follower.h"
#include "obstacle.h"
#include "course_map.h"
#include "calibration.h"
#include "position_estimator.h"
#include "pid_autotune.h"
#include "mode_manager.h"
#include "logger.h"
#define Serial robot_logger

// ==========================================
// SERIAL BUFFER
// ==========================================

static char ringBuffer[BUFFER_SIZE];
static int head = 0;
static int tail = 0;

// Time tracking
extern unsigned long current_time;
extern unsigned long last_pid_status_time;

// ==========================================
// SERIAL COMMUNICATION
// ==========================================

void check_serial_available()
{
  while (Serial.available() > 0)
  {
    char c = Serial.read();
    ringBuffer[head] = c;
    head = (head + 1) % BUFFER_SIZE;

    // Handle buffer overflow
    if (head == tail)
    {
      tail = (tail + 1) % BUFFER_SIZE;
    }

    // Process message when newline is received
    if (c == '\n')
    {
      processMessage();
    }
  }
}

void processMessage()
{
  // Extract message from ring buffer
  char message[BUFFER_SIZE];
  int index = 0;

  while (tail != head)
  {
    char currentChar = ringBuffer[tail];
    tail = (tail + 1) % BUFFER_SIZE;

    if (currentChar == '\n')
    {
      break; // End of message
    }

    message[index++] = currentChar;
  }

  message[index] = '\0'; // Null-terminate

  // Parse the extracted message
  parseMessage(message);
}

void parseMessage(char *msg)
{
  char cmd[3]; // Command character
  int value = 0;

  // Extract command (first character)
  sscanf(msg, "%1s", cmd);

  // Skip whitespace and extract numeric value
  char *beg = ++msg;
  while (*beg == ' ')
  {
    beg++;
  }

  // Parse integer value if present
  value = atoi(beg);

  // Execute command
  switch (cmd[0])
  {
  case 'd':
    // Direct drive commands consistently select manual mode.
    mode_switch(MODE_MANUAL);
    set_speed(value);
    break;

  case 's':
    mode_switch(MODE_MANUAL);
    set_steering(value);
    break;

  case 'n':
    // Print distance
    Serial.println(get_distance(encoder_pos));
    break;

  case 'p':
    // Pause active mode and remember it for resume.
    mode_pause();
    break;

  case 'h':
    // Hold position
    stop(true);
    break;

  case 'q':
    // Tune Kp
    Kp = value / 10.0;
    Serial.print("Kp: ");
    Serial.println(Kp);
    break;

  case 'w':
    // Tune Ki
    Ki = value / 100.0;
    Serial.print("Ki: ");
    Serial.println(Ki);
    break;

  case 'e':
    // Tune Kd
    Kd = value / 10.0;
    Serial.print("Kd: ");
    Serial.println(Kd);
    break;

  case 'g':
    // Print gyro heading
    Serial.println(get_angle());
    break;

  case 'v':
    // Print ToF distances
    Serial.print("LEFT: ");
    Serial.print(get_tof_distance(TOF_LEFT), 1);
    Serial.print(" RIGHT: ");
    Serial.println(get_tof_distance(TOF_RIGHT), 1);
    break;

  case 'a':
    // Set acceleration
    set_acceleration(value);
    break;

  case 'r':
    mode_resume();
    break;

  case 'x':
    // Print steering timing
    Serial.print("Steering diff: ");
    Serial.print(steering_diff);
    Serial.println(" us");
    break;

  case 'm':
    // Enter direct manual control.
    mode_switch(MODE_MANUAL);
    break;

  case 'l':
    // Start Open Challenge.
    mode_switch(MODE_OPEN_CHALLENGE);
    break;

  case 'z':
    mode_stop_all();
    break;

  case 'u':
    // Set wall target distance
    gyro_follower_set_target_distance(value);
    break;

  case 'i':
    // Toggle wall follower debug output
    gyro_follower_set_debug(true);
    break;

  case 'o':
    // Disable wall follower debug output
    gyro_follower_set_debug(false);
    break;

  case 'b':
    // Stationary obstacle test: b1 = mode on, b0 = stop.
    if (value != 0)
      mode_switch(MODE_OBSTACLE_BENCH);
    else if (current_mode == MODE_OBSTACLE_BENCH)
      mode_stop_all();
    break;

  case 'c':
    // Start stationary live camera-colour calibration mode.
    mode_switch(MODE_CAMERA_CALIBRATION);
    break;

  case 'O':
    // Start Obstacle Challenge.
    mode_switch(MODE_OBSTACLE_CHALLENGE);
    break;

  case 'j':
    // Print the currently learned four-section obstacle map.
    course_map_print();
    break;

  case 'C':
    // Start turn-radius calibration (uppercase avoids the camera command).
    mode_switch(MODE_TURN_RADIUS_CAL);
    break;

  case 'B':
    // Start straight servo-center calibration (uppercase avoids bench test).
    mode_switch(MODE_SERVO_CENTER_CAL);
    break;

  case 'y':
    // Start PID autotune.
    mode_switch(MODE_PID_AUTOTUNE);
    break;

  case 't':
    // Print current dead-reckoning position.
    position_print();
    break;

  case 'k':
    // Print turn-radius calibration data.
    if (calibration_has_data()) {
      calibration_print_results();
    } else {
      Serial.println("No calibration data available. Run 'C' first.");
    }
    break;

  default:
    Serial.println("Unknown command");
    break;
  }
}

void pid_config_print()
{
  if (current_time - last_pid_status_time > STATUS_PRINT_INTERVAL_US)
  {
    last_pid_status_time = current_time;

    Serial.print("target_speed: ");
    Serial.print(target_speed);
    Serial.print(" current_speed: ");
    Serial.print(current_speed);
    Serial.print(" dc_current_dc: ");
    Serial.print(dc_current_dc);
    Serial.print(" Kp: ");
    Serial.print(Kp);
    Serial.print(" Ki: ");
    Serial.print(Ki);
    Serial.print(" Kd: ");
    Serial.print(Kd);
    Serial.print(" error: ");
    Serial.print(target_distance - current_distance);
    Serial.print(" pid_integral: ");
    Serial.print(pid_integral);
    Serial.print(" dc_out: ");
    Serial.print(dc_out);
    Serial.print(" pid_before_checking: ");
    Serial.print(pid_before_checking);
    Serial.print("\r\n");
  }
}

// ==========================================
// INITIALIZATION
// ==========================================

void serial_setup()
{
  Serial.begin(SERIAL_BAUD);
  
  // Wait for serial only if the robot isn't already enabled via the physical switch
  // This allows the robot to run without a PC if the switch is ON, 
  // but blocks for debugging if the switch is OFF.
  while (!Serial && !system_enabled)
  {
    // Re-check switch in case user toggles it to skip waiting
    system_enabled = digitalRead(ENABLE_SWITCH_PIN);
    delay(10);
  }

  Serial.println("===== SERIAL INITIALIZED =====");
}
