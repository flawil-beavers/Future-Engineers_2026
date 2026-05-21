#include <Arduino.h>
#include <Wire.h>
#include <vl53lx_class.h>
#include <Servo.h>
#include <Adafruit_BNO08x.h>

// Gyro settings
#define BNO080_I2C_ADDR 0x4A

Adafruit_BNO08x bno = Adafruit_BNO08x();
sh2_SensorValue_t sensor_value;

float current_degree = 0;

#define SERIAL_BAUD 115200

#define LEFT_XSDN  A1
#define RIGHT_XSDN A2

// ST Library nutzt 8-bit Adressen
#define LEFT_ADDR  0x52
#define RIGHT_ADDR 0x54

// VL53LX sensor_left(&Wire, LEFT_XSDN);
// VL53LX sensor_right(&Wire, RIGHT_XSDN);

Servo servo;

#define servoPin 9
#define in1Pin 7
#define in2Pin 6
#define enaPin 8

#define encoderPinA 3
#define encoderPinB 2

// #define enTogglePin 10

// ------ drive settings ------
// encoder settings
const int gear_ratio = 100;                                     // gear ratio of the motor
const int countperrev = gear_ratio * 7;                           // counts per revolution of the motor
const float counter_to_mm = 20.0 / 28.0 * PI * 43.2 / countperrev; // mm per encoder count

long encoder_pos = 0;
int encoder_dir = 1; // 1 -> CCW, -1 -> CW

bool en_state = false; // enable state
const char en_state_true[] = "enable start";
const char en_state_false[] = "enable stop";

// dc motor settings
const int max_dc = 200;        // max duty cycle for motor driver
const int min_dc = 0.25 * 255; // min duty cycle for motor driver
const float max_acc_dc = 255;  // max acceleration duty cycle for motor driver (dc/s)
float current_dc = 0;          // current duty cycle for motor driver
float acc = 700;               // acceleration speed (mm/s^2)
bool disable_dc = false;       // enable dc motor
bool hold_dc = false;

// current sensor settings
const float dc_to_current = 5.0 / 1024 * 0.525; // conversion factor from duty cycle to current (A)
const float max_current = 0.5;   // max current (A)

// speed settings
float current_speed = 0;
int target_speed = 0; // target speed for the motor in mm/s
unsigned long acc_time = 20;
unsigned long last_acc_time = 0;

float last_speed = 0;

// steering settings
const int middle = 81; // +55 -55
const int degree_max = middle + 60;
const int degree_min = middle - 60;
int set_degree = 0;
bool disable_servo = false;
int last_angle = 0;

// manual stalling detection
long stall_encoder_pos = 0; // current encoder position

// PID
float target_distance = 0; // target encoder position in mm

float measured_speed = 0;   // measured speed in mm/s
float current_distance = 0; // current distance in mm
float last_distance = 0;    // last distance in mm
float Kp = 4.0;             // proportional gain for PID controller
float Ki = 3.0;             // integral gain for PID controller
float Kd = 1.0;             // derivative gain for PID controller
float i_max = 150.0;        // max integral value for PID controller
float pid_integral = 0.0;   // integral term for PID controller
float last_error = 0.0;     // last error for PID controller

// time variables
unsigned long current_time = 0;
unsigned long last_time = 0;
unsigned long last_status_time = 0;           // when the last status was printed
unsigned long last_loop_time_us = 0;          // last loop time in microseconds
float last_loop_time = 0;                     // last loop time in seconds
unsigned long last_enable_interrupt_time = 0; // last time the enable interrupt was called
unsigned long last_steering_command = 0;
unsigned long steering_diff = 0;


#define BUFFER_SIZE 64

char ringBuffer[BUFFER_SIZE];
int head = 0;
int tail = 0;

// debug variables
int dc_out = 0;
float pid_before_checking = 0;

/*
Set the steering angle of the servo
*/
void steer(int angle)
{
  if (angle == last_angle) // to remove unnecessary writes
  {
    return;
  }
  if (disable_servo)
  {
    return;
  }
  angle = angle + middle;
  if (angle > degree_max)
  {
    angle = degree_max;
  }
  else if (angle < degree_min)
  {
    angle = degree_min;
  }
  servo.write(angle);
  last_angle = angle;
}

/*
Function to set the steering angle of the servo
*/
void set_steering(int angle)
{
  // disable_servo = false;
  set_degree = angle;
}

/*
Set the duty cycle of the motor driver.
The duty cycle is limited by max_dc, min_dc and max_acc_dc.
dc can be a positive or negative value.
*/
void set_dc(float dc)
{
  if (disable_dc || fabs(dc) < min_dc)
  {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
    current_dc = dc;
    return;
  }
  if (dc != 0 && fabs(dc) > max_dc)
  {
    dc = max_dc * (dc / fabs(dc));
  }
  if (dc > current_dc + max_acc_dc * last_loop_time)
  {
    dc = current_dc + max_acc_dc * last_loop_time;
  }
  else if (dc < current_dc - max_acc_dc * last_loop_time)
  {
    dc = current_dc - max_acc_dc * last_loop_time;
  }
  dc_out = fabs(dc);
  analogWrite(enaPin, dc_out);
  if (dc > 0)
  {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
  }
  else if (dc < 0)
  {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
  }
  current_dc = dc;
}

/*
Calculate distance from encoder position
*/
float get_distance(long encoder_pos)
{
  return encoder_pos * counter_to_mm;
}

/*
Estimates dc for a given value in mm/s
50 dc = 300 mm/s
Not very accurate yet as just one measurement is used from the past loop
*/
int estimate_dc(float speed)
{
  float distance = get_distance(encoder_pos);
  float dc = speed / distance * max_dc; // todo: check if this is correct
  if (dc > max_dc)
  {
    dc = max_dc;
  }
  else if (dc < min_dc)
  {
    dc = min_dc;
  }
  return dc;
}

/*
PID controlled speed function
*/
void pid_speed()
{
  if (last_loop_time == 0)
  {
    return;
  }
  float error = target_distance - current_distance;
  pid_integral += error * last_loop_time;
  pid_before_checking = pid_integral;
  if (pid_integral != 0 && fabs(pid_integral) > i_max)
  {
    pid_integral = i_max * (pid_integral / fabs(pid_integral));
  }
  float speed = Kp * error + Ki * pid_integral + Kd * (error - last_error) / last_loop_time;
  set_dc(speed);
  last_error = error;
}

/*
Loop function that runs each time the loop is called
This function takes care of acceleration
*/
void drive_loop()
{
  if (last_loop_time == 0) // don't do loop if not yet initialised; so we don't divide by 0
  {
    return;
  }
  steer(set_degree);
  if (!hold_dc)
  {
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
  // measured_speed = (current_distance - last_distance) / last_loop_time; // approximate speed in mm/s todo: average over multiple loops
}

/*
Function to set the acceleration speed
*/
void set_acceleration(int acceleration)
{
  if (acceleration < 0)
  {
    acceleration = 0;
  }
  acc = acceleration;
}

/*
Stops both motors
*/
void stop(bool hold = false) // todo rework emergency stop
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

/*
Function to set the speed

If no speed is given, the last speed is used before the robot was paused
*/
void set_speed(int speed = last_speed) // todo when setting speed to zero no emergency stop should be called
{
  // disable_dc = false;
  // disable_servo = false;
  // digitalWrite(ledPin, LOW);
  hold_dc = false;
  target_speed = speed;
  last_speed = speed;
}

void pid_config_print()
{
  if (current_time - last_status_time > 200000)
  {
    last_status_time = current_time;
    // Serial.print("time passed (ms): ");
    // Serial.print(last_loop_time * 1000);
    // Serial.print(" encoder_pos: ");
    // Serial.print(get_distance(encoder_pos));
    Serial.print(" target_speed: ");
    Serial.print(target_speed);
    Serial.print(" current_speed: ");
    Serial.print(current_speed);
    // Serial.print(" target_distance: ");
    // Serial.print(target_distance);
    // Serial.print(" measured_speed: ");
    // Serial.print(measured_speed);
    Serial.print(" current_dc: ");
    Serial.print(current_dc);
    Serial.print(" kp: ");
    Serial.print(Kp);
    Serial.print(" ki: ");
    Serial.print(Ki);
    Serial.print(" kd: ");
    Serial.print(Kd);
    Serial.print(" dc: ");
    Serial.print(current_dc);
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

void parseMessage(char *msg)
{
  char cmd[3]; // To store the 2-char command
  int value = 0;

  sscanf(msg, "%1s", cmd);

  // skip whitespace
  char *beg = ++msg;

  while (*beg == ' ')
  {
    beg++;
  }

  char *second_int = beg;

  while ((*second_int >= '0' && *second_int <= '9') || *second_int == '-')
  {
    second_int++;
  }
  value = atoi(beg);
  int value_2 = atoi(++second_int);
  // Serial.println("ok");
  switch (cmd[0])
  {
  case 'd':
    set_speed(value);
    break;
  case 's':
    set_steering(value);
    break;
  case 'n':
    Serial.println(get_distance(encoder_pos));
    break;
  case 'p':
    stop();
    current_speed = 0;
    target_distance = current_distance;
    break;
  case 'h':
    stop(true);
    break;
  case 'q':
    Kp = value / 10.;
    break;
  case 'w':
    Ki = value / 100.;
    break;
  case 'e':
    Kd = value / 10.;
    break;
  case 'g':
    Serial.println(current_degree);
    break;
  // case 't':
  //   Serial.println(get_temperature());
  //   break;
  case 'a':
    set_acceleration(value);
    break;
  case 'r':
    set_speed();
    break;
  // case 'o':
  //   digitalWrite(ledPin, HIGH);
  //   break;
  // case 'f':
  //   digitalWrite(ledPin, LOW);
  //   break;
  // case 'z':
  //   Serial.print(get_distance(encoder_pos));
  //   Serial.print(",");
  //   Serial.println(degree_calibrated * 180 / PI);
    // break;
  // case 'y':
  //   set_speed(value);
  //   set_steering(value_2);
  //   Serial.print(get_distance(encoder_pos));
  //   Serial.print(",");
  //   Serial.println(degree_calibrated * 180 / PI);
    break;
  case 'x':
    Serial.print("Steering diff: ");
    Serial.print(steering_diff);
    Serial.println(" us");
    break;
  case 'm':
    disable_dc = false;
    disable_servo = false;
    set_speed();
    break;
  }
}

void processMessage()
{
  // Message extraction from ring buffer
  char message[BUFFER_SIZE];
  int index = 0;
  while (tail != head)
  {
    char currentChar = ringBuffer[tail];
    tail = (tail + 1) % BUFFER_SIZE;

    if (currentChar == '\n')
    { // End of message
      break;
    }

    message[index++] = currentChar;
  }

  message[index] = '\0'; // Null-terminate the message string

  // Parse the extracted message
  parseMessage(message);
}

void check_serial_available()
{
  while (Serial.available() > 0)
  {
    char c = Serial.read();
    ringBuffer[head] = c;
    head = (head + 1) % BUFFER_SIZE;

    if (head == tail)
    {
      // Buffer overflow, discard the oldest character
      tail = (tail + 1) % BUFFER_SIZE;
    }
    if (c == '\n')
    {
      processMessage();
    }
  }
}

void check_stalling()
{
  if (fabs(stall_encoder_pos - encoder_pos) < 4 && fabs(current_dc) > max_dc * 0.9 && !disable_dc)
  {
    Serial.print("Stall detected, stopping robot: diff_distance:");
    Serial.print(fabs(stall_encoder_pos - encoder_pos));
    Serial.print(", current_dc");
    Serial.println(current_dc);
    // disable motor
    stop();
    current_speed = 0;
    target_distance = current_distance;
  }
  stall_encoder_pos = encoder_pos; // update stall position
}

/*
Change enable state based on interrupt
This is the start stop and pause button

Switch on:
Starts the robot
When the robot is paused this makes the robot resume

Switch off:
Pauses the robot
*/
void enable_interrupt()
{
  if (current_time - last_enable_interrupt_time < 100000)
  {
    return;
  }
  last_enable_interrupt_time = current_time;
  en_state = !en_state;
  if (en_state)
  {
    disable_dc = false;
    disable_servo = false;
    set_speed();
    // digitalWrite(ledPin, LOW);
    Serial.println(en_state_true);
  }
  else
  {
    stop();
    // digitalWrite(ledPin, HIGH);
    Serial.println(en_state_false);
  }
}

void loop_updater()
{
  last_time = current_time;
  last_distance = current_distance;

  current_time = micros();
  last_loop_time_us = current_time - last_time;
  last_loop_time = last_loop_time_us / 1000000.0; // in seconds

  current_distance = get_distance(encoder_pos);

  // update_gyro();
}

void update_encoder(int encoderPin)
{
  int a = digitalRead(encoderPinA);
  int b = digitalRead(encoderPinB);
  if ((a == b && encoderPin == encoderPinA) || (a != b && encoderPin == encoderPinB))
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
  update_encoder(encoderPinA);
}

void update_encoder_b()
{
  update_encoder(encoderPinB);
}

void setup()
{

  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);

  // pinMode(enTogglePin, INPUT);

  servo.attach(servoPin);

  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  // pinMode(currentPin, INPUT);
  // pinMode(ledPin, OUTPUT);
  // digitalWrite(ledPin, LOW);

  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, LOW);
  analogWrite(enaPin, 0);


  Serial.begin(SERIAL_BAUD);
  delay(3000);

  attachInterrupt(digitalPinToInterrupt(encoderPinA), update_encoder_a, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), update_encoder_b, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(enTogglePin), enable_interrupt, CHANGE);

  // en_state = digitalRead(enTogglePin) == HIGH; // initial state of the enable button

  disable_dc = false;
  disable_servo = false;
  set_speed();
  // digitalWrite(ledPin, LOW);
  Serial.println(en_state_true);
  

  // Gyro setup
  if (!bno.begin_I2C(BNO080_I2C_ADDR))
  {
    Serial.println("Failed to find BNO080 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("BNO080 Found!");
  if (!bno.enableReport(SH2_ROTATION_VECTOR, 10000))
  {
    Serial.println("Failed to enable rotation vector");
    while (1) {
      delay(10);
    }
  }

  Serial.println("===== START =====");
 /*
  Wire.begin();
  Wire.setClock(400000);

  pinMode(LEFT_XSDN, OUTPUT);
  pinMode(RIGHT_XSDN, OUTPUT);

  // Beide Sensoren aus
  digitalWrite(LEFT_XSDN, LOW);
  digitalWrite(RIGHT_XSDN, LOW);
  delay(500);

  // -----------------------
  // LINKER SENSOR
  // -----------------------
  Serial.println("Starting LEFT sensor");

  digitalWrite(LEFT_XSDN, HIGH);
  delay(500);

  sensor_left.begin();

  if (sensor_left.InitSensor(LEFT_ADDR) == VL53LX_ERROR_NONE)
  {
    Serial.println("LEFT sensor OK");
    sensor_left.VL53LX_StartMeasurement();
  }
  else
  {
    Serial.println("LEFT sensor FAILED");
  }

  delay(500);

  // -----------------------
  // RECHTER SENSOR
  // -----------------------
  Serial.println("Starting RIGHT sensor");

  digitalWrite(RIGHT_XSDN, HIGH);
  delay(500);

  sensor_right.begin();

  if (sensor_right.InitSensor(RIGHT_ADDR) == VL53LX_ERROR_NONE)
  {
    Serial.println("RIGHT sensor OK");
    sensor_right.VL53LX_StartMeasurement();
  }
  else
  {
    Serial.println("RIGHT sensor FAILED");
  }

  Serial.println("===== SETUP DONE =====");
  */
}

void readSensor(VL53LX &sensor, const char *name)
{
  uint8_t ready = 0;

  if (sensor.VL53LX_GetMeasurementDataReady(&ready) != VL53LX_ERROR_NONE)
    return;

  if (!ready)
    return;

  VL53LX_MultiRangingData_t data;

  if (sensor.VL53LX_GetMultiRangingData(&data) != VL53LX_ERROR_NONE)
    return;

  Serial.print(name);
  Serial.print(": ");

  if (data.NumberOfObjectsFound > 0)
  {
    Serial.print(data.RangeData[0].RangeMilliMeter);
    Serial.println(" mm");
  }
  else
  {
    Serial.println("no object");
  }

  sensor.VL53LX_ClearInterruptAndStartMeasurement();
}

void update_gyro()
{
  if (bno.wasReset())
  {
    Serial.println("BNO080 was reset! reinitializing...");
    bno.enableReport(SH2_ROTATION_VECTOR, 10000);
  }
  if (!bno.getSensorEvent(&sensor_value))
  {
    Serial.println("Failed to read BNO080 sensor event");
  }
  else if (sensor_value.sensorId == SH2_ROTATION_VECTOR)
  {
    // Extract the rotation vector data from the sensor value
    sh2_RotationVectorWAcc_t rotationVector = sensor_value.un.rotationVector;
    float r = rotationVector.real;
    float i = rotationVector.i;
    float j = rotationVector.j;
    float k = rotationVector.k;

    // Convert the rotation vector to Yaw (Euler heading)
    float yaw = atan2(2.0 * (i * j + r * k), r * r + i * i - j * j - k * k);
    current_degree = yaw * 180.0 / PI;
  }
}

void loop()
{
  loop_updater();
  check_serial_available();
  // check_current();
  check_stalling();
  drive_loop();
  update_gyro();
  // pid_config_print();
  // gyro_config_print();
  // gyro_config();
  
  // readSensor(sensor_left, "LEFT ");
  // readSensor(sensor_right, "RIGHT");

  delay(50);
}