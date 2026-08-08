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
 *   Q/W    : Set acceleration-phase Kp / Ki
 *   E      : Set acceleration feedforward Ka
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
 *   M      : MOTOR MIN DC CALIBRATION mode
 *   z      : STOP active mode
 *   u<val> : Set wall distance (mm)
 *   i      : Print serial command information
 *   f/o    : Navigation debug ON/OFF
 */

#include "serial_handler.h"
#include "config.h"
#include "motor_control.h"
#include "sensors.h"
#include "navigation_controller.h"
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

/**
 * Select a mode for the current power cycle only. A reset always returns to
 * STARTUP_ROBOT_MODE from config.h; nothing is written to persistent memory.
 */
static void select_temporary_mode(RobotMode mode)
{
  mode_switch(mode);
  if (current_mode == mode || pending_mode == mode) {
    Serial.print("Temporary mode until restart: ");
    Serial.println(mode_name(mode));
  } else {
    Serial.print("Temporary mode selection failed: ");
    Serial.println(mode_name(mode));
  }
}

static void print_pid_help()
{
  Serial.println("\n===== PID SETUP =====");
  Serial.println("pid show                         : show every active motor value");
  Serial.println("pid test <mm/s>                  : drive at a test speed; 'z' stops");
  Serial.println("pid tune <mm/s> <PWM> <step>     : relay autotune for LOW/MID/HIGH band");
  Serial.println("pid set low|mid|high kp|ki <v>   : set cruise PI for one speed band");
  Serial.println("pid set bound low|mid|high <mm/s>: set interpolation boundaries");
  Serial.println("pid set accel kp|ki <v>          : acceleration PI");
  Serial.println("pid set ff static|kv|ka <v>      : static/speed/acceleration feedforward");
  Serial.println("pid export                       : print config.h lines to save values");
  Serial.println("LOW is used through the low boundary. Values blend smoothly LOW->MID");
  Serial.println("and MID->HIGH between the three boundaries.");
  Serial.println("Kp reacts immediately to speed error; too high causes oscillation.");
  Serial.println("Ki removes lasting load error; too high causes slow pulsing/windup.");
  Serial.println("FF static overcomes motor friction. Kv supplies PWM per mm/s.");
  Serial.println("Ka supplies extra PWM while accelerating. Accel Kp/Ki correct Ka errors.");
  Serial.println("Autotune: mm/s chooses the band, PWM is steady motor power, step is +/-PWM.");
  Serial.println("Tune one band at a time with enough straight driving space.");
  Serial.println("Settings are RAM-only until copied from 'pid export' into config.h.");
  Serial.println("=====================\n");
}

static void print_pid_settings(bool export_format)
{
  if (export_format) {
    Serial.println("// Copy these tested values into include/config.h:");
    Serial.print("constexpr float LOW_SPEED_CRUISE_KP = "); Serial.print(low_speed_cruise_kp, 5); Serial.println("f;");
    Serial.print("constexpr float LOW_SPEED_CRUISE_KI = "); Serial.print(low_speed_cruise_ki, 5); Serial.println("f;");
    Serial.print("constexpr float MID_SPEED_CRUISE_KP = "); Serial.print(mid_speed_cruise_kp, 5); Serial.println("f;");
    Serial.print("constexpr float MID_SPEED_CRUISE_KI = "); Serial.print(mid_speed_cruise_ki, 5); Serial.println("f;");
    Serial.print("constexpr float CRUISE_KP = "); Serial.print(Kp, 5); Serial.println("f;");
    Serial.print("constexpr float CRUISE_KI = "); Serial.print(Ki, 5); Serial.println("f;");
    Serial.print("constexpr float LOW_SPEED_GAIN_END_MMS = "); Serial.print(low_speed_gain_end, 1); Serial.println("f;");
    Serial.print("constexpr float MID_SPEED_GAIN_END_MMS = "); Serial.print(mid_speed_gain_end, 1); Serial.println("f;");
    Serial.print("constexpr float HIGH_SPEED_GAIN_START_MMS = "); Serial.print(high_speed_gain_start, 1); Serial.println("f;");
    Serial.print("constexpr float ACCEL_KP = "); Serial.print(accel_Kp, 5); Serial.println("f;");
    Serial.print("constexpr float ACCEL_KI = "); Serial.print(accel_Ki, 5); Serial.println("f;");
    Serial.print("constexpr float MOTOR_STATIC_FF_DC = "); Serial.print(motor_static_ff, 3); Serial.println("f;");
    Serial.print("constexpr float MOTOR_SPEED_FF_DC_PER_MMS = "); Serial.print(motor_speed_ff, 6); Serial.println("f;");
    Serial.print("constexpr float MOTOR_ACCEL_FF_DC_PER_MMSS = "); Serial.print(motor_accel_ff, 6); Serial.println("f;");
    return;
  }
  Serial.println("\n=== ACTIVE PID SETTINGS (RAM) ===");
  Serial.print("LOW  Kp/Ki: "); Serial.print(low_speed_cruise_kp, 5); Serial.print(" / "); Serial.println(low_speed_cruise_ki, 5);
  Serial.print("MID  Kp/Ki: "); Serial.print(mid_speed_cruise_kp, 5); Serial.print(" / "); Serial.println(mid_speed_cruise_ki, 5);
  Serial.print("HIGH Kp/Ki: "); Serial.print(Kp, 5); Serial.print(" / "); Serial.println(Ki, 5);
  Serial.print("Boundaries LOW/MID/HIGH: "); Serial.print(low_speed_gain_end, 1); Serial.print(" / "); Serial.print(mid_speed_gain_end, 1); Serial.print(" / "); Serial.println(high_speed_gain_start, 1);
  Serial.print("Acceleration Kp/Ki: "); Serial.print(accel_Kp, 5); Serial.print(" / "); Serial.println(accel_Ki, 5);
  Serial.print("Feedforward static/Kv/Ka: "); Serial.print(motor_static_ff, 3); Serial.print(" / "); Serial.print(motor_speed_ff, 6); Serial.print(" / "); Serial.println(motor_accel_ff, 6);
  pid_autotune_print_config();
  Serial.println("=================================\n");
}

static bool handle_pid_command(const char *message)
{
  if (strncmp(message, "pid", 3) != 0 ||
      (message[3] != '\0' && message[3] != ' '))
    return false;

  char action[12] = {};
  char group[12] = {};
  char parameter[12] = {};
  float value = 0.0f;
  const int fields = sscanf(message, "pid %11s %11s %11s %f", action, group, parameter, &value);
  if (fields < 1 || strcmp(action, "help") == 0) {
    print_pid_help();
    return true;
  }
  if (strcmp(action, "show") == 0) {
    print_pid_settings(false);
    return true;
  }
  if (strcmp(action, "export") == 0) {
    print_pid_settings(true);
    return true;
  }
  if (strcmp(action, "test") == 0) {
    float speed = 0.0f;
    if (sscanf(message, "pid test %f", &speed) != 1 || fabsf(speed) > 1000.0f) {
      Serial.println("Usage: pid test <speed_mm_s>");
      return true;
    }
    mode_switch(MODE_MANUAL);
    set_speed((int)roundf(speed));
    Serial.print("PID test running at "); Serial.print(speed, 1);
    Serial.println(" mm/s. Send 'z' to stop.");
    return true;
  }
  if (strcmp(action, "tune") == 0) {
    float speed = 0.0f, baseline = 0.0f, relay = 0.0f;
    if (sscanf(message, "pid tune %f %f %f", &speed, &baseline, &relay) != 3 ||
        !pid_autotune_configure(speed, baseline, relay)) {
      Serial.println("Invalid tune values. Use: pid tune <speed> <baseline_PWM> <relay_step>");
      Serial.println("Requirements: speed >= 40, PWM within motor range, step >= 2.");
      return true;
    }
    pid_autotune_print_config();
    mode_switch(MODE_PID_AUTOTUNE);
    return true;
  }
  if (strcmp(action, "set") != 0 || fields != 4 || !isfinite(value)) {
    Serial.println("Invalid PID command. Send 'pid help'.");
    return true;
  }

  float *destination = nullptr;
  if (strcmp(group, "low") == 0)
    destination = strcmp(parameter, "kp") == 0 ? &low_speed_cruise_kp : strcmp(parameter, "ki") == 0 ? &low_speed_cruise_ki : nullptr;
  else if (strcmp(group, "mid") == 0)
    destination = strcmp(parameter, "kp") == 0 ? &mid_speed_cruise_kp : strcmp(parameter, "ki") == 0 ? &mid_speed_cruise_ki : nullptr;
  else if (strcmp(group, "high") == 0)
    destination = strcmp(parameter, "kp") == 0 ? &Kp : strcmp(parameter, "ki") == 0 ? &Ki : nullptr;
  else if (strcmp(group, "accel") == 0)
    destination = strcmp(parameter, "kp") == 0 ? &accel_Kp : strcmp(parameter, "ki") == 0 ? &accel_Ki : nullptr;
  else if (strcmp(group, "ff") == 0)
    destination = strcmp(parameter, "static") == 0 ? &motor_static_ff : strcmp(parameter, "kv") == 0 ? &motor_speed_ff : strcmp(parameter, "ka") == 0 ? &motor_accel_ff : nullptr;
  else if (strcmp(group, "bound") == 0)
    destination = strcmp(parameter, "low") == 0 ? &low_speed_gain_end : strcmp(parameter, "mid") == 0 ? &mid_speed_gain_end : strcmp(parameter, "high") == 0 ? &high_speed_gain_start : nullptr;

  if (!destination || value < 0.0f || value > 1000.0f) {
    Serial.println("Unknown setting or unsafe value. Send 'pid help'.");
    return true;
  }
  const float old_value = *destination;
  *destination = value;
  if (!(low_speed_gain_end < mid_speed_gain_end &&
        mid_speed_gain_end < high_speed_gain_start)) {
    *destination = old_value;
    Serial.println("Rejected: boundaries must satisfy LOW < MID < HIGH.");
    return true;
  }
  pid_integral = 0.0f;
  accel_pid_integral = 0.0f;
  Serial.print("Set "); Serial.print(group); Serial.print(" ");
  Serial.print(parameter); Serial.print(" = "); Serial.println(value, 6);
  return true;
}

static void print_serial_command_info()
{
  Serial.println("\n===== SERIAL COMMANDS =====");
  Serial.println("Mode letters are temporary until reset; reset defaults to OBSTACLE.");
  Serial.println("i          : Show this command list");
  Serial.println("m          : Select MANUAL mode");
  Serial.println("l          : Start OPEN CHALLENGE mode");
  Serial.println("O          : Start OBSTACLE CHALLENGE mode");
  Serial.println("b1 / b0    : Start / stop OBSTACLE BENCH mode");
  Serial.println("c          : Start CAMERA CALIBRATION mode");
  Serial.println("C          : Start TURN RADIUS CALIBRATION mode");
  Serial.println("B          : Start SERVO CENTER CALIBRATION mode");
  Serial.println("y          : Start PID AUTOTUNE mode");
  Serial.println("M          : Start MOTOR MIN DC CALIBRATION mode");
  Serial.println("p / r      : Pause / resume current mode");
  Serial.println("z          : Stop active and pending mode");
  Serial.println("d<speed>   : Manual drive speed in mm/s");
  Serial.println("s<angle>   : Manual steering angle in degrees");
  Serial.println("a<value>   : Set acceleration in mm/s^2");
  Serial.println("u<distance>: Set navigation wall distance in mm");
  Serial.println("f / o      : General debug ON / OFF");
  Serial.println("n / g / v  : Print encoder / gyro / ToF data");
  Serial.println("t          : Print estimated position");
  Serial.println("j          : Print learned obstacle course map");
  Serial.println("k          : Print calibration data");
  Serial.println("q/w<val>   : Set cruise Kp / Ki");
  Serial.println("Q/W<val>   : Set acceleration Kp / Ki");
  Serial.println("E<val>     : Set acceleration feedforward Ka (val/1000)");
  Serial.println("P          : Print motor-controller status");
  Serial.println("pid help   : Guided PID setup, test and autotune commands");
  Serial.println("x          : Print steering timing");
  Serial.println("h          : Hold position");
  Serial.println("===========================\n");
}

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
  if (handle_pid_command(msg))
    return;

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
    mode_switch(MODE_HOLD);
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
    Serial.println("Motor D term is disabled; use q/w for cruise PI.");
    break;

  case 'Q':
    accel_Kp = value / 10.0f;
    Serial.print("Acceleration Kp: ");
    Serial.println(accel_Kp);
    break;

  case 'W':
    accel_Ki = value / 100.0f;
    Serial.print("Acceleration Ki: ");
    Serial.println(accel_Ki);
    break;

  case 'E':
    motor_accel_ff = value / 1000.0f;
    Serial.print("Acceleration feedforward Ka: ");
    Serial.println(motor_accel_ff, 4);
    break;

  case 'P':
    pid_config_print();
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
    select_temporary_mode(MODE_MANUAL);
    break;

  case 'l':
    // Start Open Challenge.
    select_temporary_mode(MODE_OPEN_CHALLENGE);
    break;

  case 'z':
    mode_stop_all();
    break;

  case 'u':
    // Set wall target distance
    navigation_set_target_distance(value);
    break;

  case 'i':
    print_serial_command_info();
    break;

  case 'f':
    // Enable general debug output
    general_debug_set(true);
    break;

  case 'o':
    // Disable general debug output
    general_debug_set(false);
    break;

  case 'b':
    // Stationary obstacle test: b1 = mode on, b0 = stop.
    if (value != 0)
      select_temporary_mode(MODE_OBSTACLE_BENCH);
    else if (current_mode == MODE_OBSTACLE_BENCH)
      mode_stop_all();
    break;

  case 'c':
    // Start stationary live camera-colour calibration mode.
    select_temporary_mode(MODE_CAMERA_CALIBRATION);
    break;

  case 'O':
    // Start Obstacle Challenge.
    select_temporary_mode(MODE_OBSTACLE_CHALLENGE);
    break;

  case 'j':
    // Print the currently learned four-section obstacle map.
    course_map_print();
    break;

  case 'C':
    // Start turn-radius calibration (uppercase avoids the camera command).
    select_temporary_mode(MODE_TURN_RADIUS_CAL);
    break;

  case 'B':
    // Start straight servo-center calibration (uppercase avoids bench test).
    select_temporary_mode(MODE_SERVO_CENTER_CAL);
    break;

  case 'y':
    // Start PID autotune.
    select_temporary_mode(MODE_PID_AUTOTUNE);
    break;

  case 'M':
    // Start motor minimum DC calibration (uppercase avoids manual mode).
    select_temporary_mode(MODE_MOTOR_MIN_CAL);
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
    Serial.print(" measured_speed: ");
    Serial.print(measured_speed);
    Serial.print(" dc_current_dc: ");
    Serial.print(dc_current_dc);
    Serial.print(" phase: ");
    Serial.print((int)drive_control_phase);
    Serial.print(" accel: ");
    Serial.print(current_acceleration);
    Serial.print(" measured_accel: ");
    Serial.print(measured_acceleration);
    Serial.print(" accel_command: ");
    Serial.print(commanded_acceleration);
    Serial.print(" accel_limit: ");
    Serial.print(active_acceleration_limit);
    Serial.print(" cruise_Kp: ");
    Serial.print(Kp);
    Serial.print(" cruise_Ki: ");
    Serial.print(Ki);
    Serial.print(" active_cruise_Kp: ");
    Serial.print(active_cruise_kp);
    Serial.print(" active_cruise_Ki: ");
    Serial.print(active_cruise_ki);
    Serial.print(" accel_Kp: ");
    Serial.print(accel_Kp);
    Serial.print(" accel_Ki: ");
    Serial.print(accel_Ki);
    Serial.print(" ff_Kv: ");
    Serial.print(motor_speed_ff);
    Serial.print(" speed_error: ");
    Serial.print(current_speed - measured_speed);
    Serial.print(" pid_integral: ");
    Serial.print(pid_integral);
    Serial.print(" accel_integral: ");
    Serial.print(accel_pid_integral);
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
