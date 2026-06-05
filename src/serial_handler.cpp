/**
 * @file serial_handler.cpp
 * @brief Serial communication and command parsing implementation
 */

#include "serial_handler.h"
#include "config.h"
#include "motor_control.h"
#include "sensors.h"
#include "wall_follower.h"

// ==========================================
// SERIAL BUFFER
// ==========================================

char ringBuffer[BUFFER_SIZE];
int head = 0;
int tail = 0;

// Time tracking
extern unsigned long current_time;
extern unsigned long last_status_time;

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
    // Set drive speed
    set_speed(value);
    break;

  case 's':
    // Set steering angle
    set_steering(value);
    break;

  case 'n':
    // Print distance
    Serial.println(get_distance(encoder_pos));
    break;

  case 'p':
    // Pause
    stop();
    current_speed = 0;
    target_distance = current_distance;
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
    Serial.print(current_distance_left, 1);
    Serial.print(" RIGHT: ");
    Serial.println(current_distance_right, 1);
    break;

  case 'a':
    // Set acceleration
    set_acceleration(value);
    break;

  case 'r':
    // Resume with last speed
    set_speed();
    break;

  case 'x':
    // Print steering timing
    Serial.print("Steering diff: ");
    Serial.print(steering_diff);
    Serial.println(" us");
    break;

  case 'm':
    // Master enable
    disable_dc = false;
    disable_servo = false;
    set_speed();
    break;

  case 'l':
    // Wall follower START
    wall_follower_enable();
    break;

  case 'z':
    // Wall follower STOP
    wall_follower_disable();
    break;

  case 'u':
    // Set wall target distance
    wall_follower_set_target_distance(value);
    break;

  case 'i':
    // Toggle wall follower debug output
    wall_follower_set_debug(true);
    break;

  case 'o':
    // Disable wall follower debug output
    wall_follower_set_debug(false);
    break;

  default:
    Serial.println("Unknown command");
    break;
  }
}

void pid_config_print()
{
  if (current_time - last_status_time > STATUS_PRINT_INTERVAL_US)
  {
    last_status_time = current_time;

    Serial.print("target_speed: ");
    Serial.print(target_speed);
    Serial.print(" current_speed: ");
    Serial.print(current_speed);
    Serial.print(" current_dc: ");
    Serial.print(current_dc);
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

  // Only wait for serial if the robot isn't already enabled via the physical switch
  while (!Serial && !system_enabled)
  {
    delay(10);
  }

  Serial.println("===== SERIAL INITIALIZED =====");
}
