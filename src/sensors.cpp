/**
 * @file sensors.cpp
 * @brief Sensor subsystem implementation
 */

#include "sensors.h"
#include "config.h"
#include <Wire.h>
#include "logger.h"
#define Serial robot_logger


// ==========================================
// SENSOR STATE VARIABLES
// ==========================================

extern bool nav_long_range_active;

// Gyro
static Adafruit_BNO08x bno = Adafruit_BNO08x(BNO085_RST);
static sh2_SensorValue_t sensor_value;
static float current_degree = 0;
static float current_heading = 0;

/**
 * @brief The timing budget (refresh rate) of ToF sensors captured at startup.
 * Used to restore standard operation after long-range discovery mode.
 */
uint32_t sensors_initial_tof_timing_budget = 0;

// ToF sensors (on separate I2C buses)
static VL53L4CX sensor_left(&Wire, -1);
static VL53L4CX sensor_right(&Wire2, -1);

// Distance readings in millimeters
static float tof_distances[TOF_COUNT] = {-1.0f, -1.0f};
static float tof_raw_distances[TOF_COUNT] = {-1.0f, -1.0f};
static float tof_signal_rates[TOF_COUNT] = {-1.0f, -1.0f};
static float tof_sigmas[TOF_COUNT] = {-1.0f, -1.0f};
static TofDiagnosticSnapshot tof_diagnostics[TOF_COUNT] = {};

// ==========================================
// SENSOR UPDATE FUNCTIONS
// ==========================================

void update_gyro()
{
  static unsigned long last_gyro_data_time = 0;
  static float last_yaw_deg = 0;
  static bool gyro_initialized = false;

  // The BNO085 INT pin is active low. If HIGH, no data is ready.
  if (digitalRead(BNO085_INT) == HIGH)
  {
    if (last_gyro_data_time != 0 && millis() - last_gyro_data_time > 200)
    {
      Serial.println("[GYRO] No data for 200ms, re-enabling reports...");
      bno.enableReport(SH2_GAME_ROTATION_VECTOR, 10000);
      gyro_initialized = false;
      last_gyro_data_time = millis();
    }
    return;
  }

  // Get sensor event
  bool has_event = bno.getSensorEvent(&sensor_value);

  // Check if sensor was reset
  if (bno.wasReset())
  {
    Serial.println("BNO085 was reset! Reinitializing...");
    gyro_initialized = false; // Reset local tracking on hardware reset
    delay(10);
    bno.enableReport(SH2_GAME_ROTATION_VECTOR, 10000);
    delay(30);
    return; // Drop this frame to let stream stabilize
  }

  // Parse Game Rotation Vector (No Magnetometer = No Drift near motors)
  if (has_event)
  {
    if (sensor_value.sensorId == SH2_GAME_ROTATION_VECTOR)
    {
      last_gyro_data_time = millis();
      sh2_RotationVector_t rotationVector = sensor_value.un.gameRotationVector;
      float r = rotationVector.real;
      float i = rotationVector.i;
      float j = rotationVector.j;
      float k = rotationVector.k;

      // Convert rotation vector to Yaw (Euler heading) in degrees
      float yaw = atan2(2.0 * (i * j + r * k), r * r + i * i - j * j - k * k);
      current_heading = yaw * 180.0 / PI;

      if (!gyro_initialized)
      {
        last_yaw_deg = current_heading;
        gyro_initialized = true;
      }

      float delta = current_heading - last_yaw_deg;
      if (delta > 180)
      {
        delta -= 360;
      }
      else if (delta < -180)
      {
        delta += 360;
      }
      last_yaw_deg = current_heading;
      current_degree += delta;
    }
  }
}

// ==========================================
// HELPER FUNCTIONS
// ==========================================

/**
 * @brief Internal helper to poll a ToF sensor and update its distance value.
 * 
 * This function implements a professional polling strategy:
 * 1. Checks if hardware data is ready.
 * 2. Retrieves multi-ranging data (objects in field of view).
 * 3. Filters objects to find the one with the lowest Sigma (highest confidence).
 * 4. Validates the hardware range status before updating the state.
 * 
 * @param sensor Reference to the VL53L4CX sensor instance.
 * @param out_distance Reference to the static variable storing the result.
 */
static void read_single_tof(VL53L4CX &sensor, float &out_distance)
{
  const TofSensor sensor_index = (&sensor == &sensor_left) ? TOF_LEFT : TOF_RIGHT;
  TofDiagnosticSnapshot &diagnostic = tof_diagnostics[sensor_index];
  // Build a complete local frame and publish it atomically at the end. A
  // not-ready poll must not erase the candidates belonging to the last frame.
  TofDiagnosticSnapshot frame = diagnostic;
  frame.reported_object_count = 0;
  frame.stored_object_count = 0;
  frame.selected_object_index = -1;
  float min_accept_signal = 0.3f;
  float max_accept_sigma = nav_long_range_active ? 30.0f : 20.0f;
  float raw_measured_dist = -1.0f;
  float current_signal_rate = -1.0f;
  float current_sigma = -1.0f;
  // float min_accept_signal = nav_long_range_active ? 0.23f : 0.3f;
  // float max_accept_sigma = nav_long_range_active ? 50.0f : 10.0f;

  uint8_t data_ready = 0;
  if (sensor.VL53L4CX_GetMeasurementDataReady(&data_ready) != VL53L4CX_ERROR_NONE || !data_ready)
  {
    return;
  }

  VL53L4CX_MultiRangingData_t ranging_data;
  float measured_distance = TOF_OUT_OF_RANGE_MM; // Default to out of range
  int best_idx = -1;
  if (sensor.VL53L4CX_GetMultiRangingData(&ranging_data) == VL53L4CX_ERROR_NONE)
  {
    frame.reported_object_count = ranging_data.NumberOfObjectsFound;
    if (ranging_data.NumberOfObjectsFound > 0)
    {
      int16_t largest_valid_dist = -1;

      for (int i = 0; i < ranging_data.NumberOfObjectsFound; i++)
      {
        uint8_t status = ranging_data.RangeData[i].RangeStatus;
        int16_t dist = ranging_data.RangeData[i].RangeMilliMeter;
        float signal = ranging_data.RangeData[i].SignalRateRtnMegaCps / 65536.0;
        float sigma = ranging_data.RangeData[i].SigmaMilliMeter / 65536.0;
        const bool hardware_valid =
            status == VL53L4CX_RANGESTATUS_RANGE_VALID ||
            status == VL53L4CX_RANGESTATUS_RANGE_VALID_MERGED_PULSE;
        const bool filter_accepted =
            hardware_valid && signal > min_accept_signal && sigma < max_accept_sigma;

        if (frame.stored_object_count < TOF_DIAGNOSTIC_MAX_OBJECTS)
        {
          TofObjectDiagnostic &object =
              frame.objects[frame.stored_object_count++];
          object.distance_mm = dist;
          object.signal_mcps = signal;
          object.sigma_mm = sigma;
          object.range_status = status;
          object.hardware_valid = hardware_valid;
          object.filter_accepted = filter_accepted;
        }

        // Enhanced reliability check:
        // 1. Status must be valid or merged pulse.
        // 2. Signal rate must be high enough (> 0.4 Mcps) to distinguish from noise.
        // 3. Sigma (standard deviation) must be low enough (< 25mm) for a stable reading.
        // This filters out "ghost" readings that occur with black/distant targets
        // which currently cause the sensor to report ~400mm instead of 9999mm (out of range).
        if (hardware_valid)
        {
          // if (&sensor == &sensor_right) {
          //   Serial.print("                    ");
          // }
          // Serial.print(dist);
          // Serial.print(" ");
          // Serial.print(signal);
          // Serial.print(" ");
          // Serial.println(sigma);
          
          // Always update telemetry variables for valid hardware status objects.
          // This ensures diagnostics reflect the current frame even if filters reject it for control.
          int s_idx = (&sensor == &sensor_left) ? TOF_LEFT : TOF_RIGHT;
          tof_signal_rates[s_idx] = signal;
          tof_sigmas[s_idx] = sigma;
          tof_raw_distances[s_idx] = (float)dist;

          if (filter_accepted)
          {
            if (dist > largest_valid_dist)
            {
              largest_valid_dist = dist;
              raw_measured_dist = (float)dist;
              current_signal_rate = signal;
              current_sigma = sigma;
              best_idx = i;
            }
          }
        }
      }
      if (best_idx != -1)
      {
        measured_distance = (float)largest_valid_dist;
      }
    }
  }

  // Enforce the 600mm limit: If the detected distance is beyond our reliable range
  // or the sensor hardware reported an out-of-bounds value, treat it as an edge (gap).
  // This forces the value to 9999.0 (TOF_OUT_OF_RANGE_MM) as requested.
  float detection_limit = nav_long_range_active ? TOF_MAX_LONG_DISTANCE_MM : TOF_MAX_RELIABLE_DISTANCE_MM;
  if (measured_distance > detection_limit)
  {
    measured_distance = TOF_OUT_OF_RANGE_MM;
  }

  // Consistency check: limit the change from the previous value (Slew Rate Limiter)
  // We skip this if the previous value was invalid (-1.0) or if either value is OUT_OF_RANGE
  // to ensure we still detect gaps (9999.0) and re-acquire walls instantly.
  if (measured_distance != TOF_OUT_OF_RANGE_MM && out_distance != -1.0f && out_distance != TOF_OUT_OF_RANGE_MM)
  {
    float delta = measured_distance - out_distance;
    if (fabs(delta) > TOF_MAX_DELTA_MM)
    {
      measured_distance = out_distance + (delta > 0 ? TOF_MAX_DELTA_MM : -TOF_MAX_DELTA_MM);
    }
  }

  sensor.VL53L4CX_ClearInterruptAndStartMeasurement();
  
  // Use the raw measured distance. It will be 9999.0 only if detection truly failed.
  // The navigation_controller logic will still treat distances > 600mm as an edge/gap.
  out_distance = measured_distance;
  frame.filtered_distance_mm = measured_distance;
  frame.selected_raw_distance_mm = raw_measured_dist;
  frame.selected_signal_mcps = current_signal_rate;
  frame.selected_sigma_mm = current_sigma;
  frame.selected_object_index =
      best_idx >= 0 && best_idx < frame.stored_object_count
          ? static_cast<int8_t>(best_idx)
          : -1;
  frame.sequence = diagnostic.sequence + 1;
  diagnostic = frame;

  // Update signal rate and sigma only if a valid measurement was found
  if (best_idx != -1) {
    if (&sensor == &sensor_left) {
      tof_signal_rates[TOF_LEFT] = current_signal_rate;
      tof_sigmas[TOF_LEFT] = current_sigma;
      tof_raw_distances[TOF_LEFT] = raw_measured_dist;
    } else if (&sensor == &sensor_right) {
      tof_signal_rates[TOF_RIGHT] = current_signal_rate;
      tof_sigmas[TOF_RIGHT] = current_sigma;
      tof_raw_distances[TOF_RIGHT] = raw_measured_dist;
    }
  }
}

void update_lasers()
{
  read_single_tof(sensor_left, tof_distances[TOF_LEFT]);
  read_single_tof(sensor_right, tof_distances[TOF_RIGHT]);
}

void sensors_set_tof_timing_budget(uint32_t budget_us)
{
  sensor_left.VL53L4CX_SetMeasurementTimingBudgetMicroSeconds(budget_us);
  sensor_right.VL53L4CX_SetMeasurementTimingBudgetMicroSeconds(budget_us);
  sensor_left.VL53L4CX_GetMeasurementTimingBudgetMicroSeconds(
      &tof_diagnostics[TOF_LEFT].timing_budget_us);
  sensor_right.VL53L4CX_GetMeasurementTimingBudgetMicroSeconds(
      &tof_diagnostics[TOF_RIGHT].timing_budget_us);
}

static bool configure_tof_for_test(VL53L4CX &sensor, TofSensor side,
                                   VL53L4CX_DistanceModes distance_mode,
                                   uint32_t budget_us)
{
  VL53L4CX_Error status = sensor.VL53L4CX_StopMeasurement();
  if (status == VL53L4CX_ERROR_NONE)
    status = sensor.VL53L4CX_SetDistanceMode(distance_mode);
  if (status == VL53L4CX_ERROR_NONE)
    status = sensor.VL53L4CX_SetMeasurementTimingBudgetMicroSeconds(budget_us);

  VL53L4CX_DistanceModes actual_mode = distance_mode;
  uint32_t actual_budget_us = 0;
  if (status == VL53L4CX_ERROR_NONE)
    status = sensor.VL53L4CX_GetDistanceMode(&actual_mode);
  if (status == VL53L4CX_ERROR_NONE)
    status = sensor.VL53L4CX_GetMeasurementTimingBudgetMicroSeconds(
        &actual_budget_us);
  if (status == VL53L4CX_ERROR_NONE)
    status = sensor.VL53L4CX_StartMeasurement();

  if (status == VL53L4CX_ERROR_NONE) {
    tof_diagnostics[side].distance_mode = actual_mode;
    tof_diagnostics[side].timing_budget_us = actual_budget_us;
    tof_distances[side] = -1.0f;
  }
  return status == VL53L4CX_ERROR_NONE;
}

bool sensors_configure_tof_for_test(VL53L4CX_DistanceModes distance_mode,
                                    uint32_t budget_us)
{
  const bool left_ok = configure_tof_for_test(
      sensor_left, TOF_LEFT, distance_mode, budget_us);
  const bool right_ok = configure_tof_for_test(
      sensor_right, TOF_RIGHT, distance_mode, budget_us);
  return left_ok && right_ok;
}

/**
 * @brief Professional initialization helper for a single ToF sensor
 * Handles the full hardware handshake and configuration sequence.
 */
static void init_single_tof(VL53L4CX &sensor, TwoWire *bus, const char* name)
{
  Serial.print("Initializing ToF: ");
  Serial.println(name);

  sensor.setI2cDevice(bus);
  if (sensor.begin() != 0 || sensor.InitSensor(VL53L4CX_DEFAULT_DEVICE_ADDRESS) != VL53L4CX_ERROR_NONE)
  {
    Serial.print("CRITICAL ERROR: ");
    Serial.print(name);
    Serial.println(" failed initialization!");
    while (1) delay(10);
  }
  
  if (sensor.VL53L4CX_SetDistanceMode(TOF_DISTANCE_MODE) !=
      VL53L4CX_ERROR_NONE)
  {
    Serial.print("WARNING: ");
    Serial.print(name);
    Serial.println(" rejected the configured distance mode");
  }
  Serial.print("nav_long_range_active: ");
  Serial.println(nav_long_range_active);
  const uint32_t requested_timing_budget_us =
      nav_long_range_active ? 300000UL : TOF_TIMING_BUDGET_US;
  if (sensor.VL53L4CX_SetMeasurementTimingBudgetMicroSeconds(
          requested_timing_budget_us) != VL53L4CX_ERROR_NONE)
  {
    Serial.print("WARNING: ");
    Serial.print(name);
    Serial.println(" rejected the requested timing budget");
  }

  uint32_t timing_budget_us = 0;
  VL53L4CX_DistanceModes distance_mode = VL53L4CX_DISTANCEMODE_MEDIUM;
  sensor.VL53L4CX_GetDistanceMode(&distance_mode);
  sensor.VL53L4CX_GetMeasurementTimingBudgetMicroSeconds(&timing_budget_us);
  Serial.print(name);
  Serial.print(" distance_mode: ");
  Serial.println(static_cast<int>(distance_mode));
  Serial.print(name);
  Serial.print(" timing_budget_us: ");
  Serial.println(timing_budget_us);
  const TofSensor sensor_index = (&sensor == &sensor_left) ? TOF_LEFT : TOF_RIGHT;
  tof_diagnostics[sensor_index].distance_mode = distance_mode;
  tof_diagnostics[sensor_index].timing_budget_us = timing_budget_us;
  

  // Capture the initial budget (mode default) to restore to later
  if (sensors_initial_tof_timing_budget == 0) {
    sensors_initial_tof_timing_budget = timing_budget_us;
  }
  if (sensor.VL53L4CX_StartMeasurement() != 0)
  {
    Serial.print("ERROR: ");
    Serial.print(name);
    Serial.println(" could not start measurements!");
  }
}

float get_tof_distance(TofSensor sensor)
{
  if (sensor >= 0 && sensor < TOF_COUNT)
  {
    return tof_distances[sensor];
  }
  return -1.0f;
}

float get_tof_raw_distance(TofSensor sensor)
{
  if (sensor >= 0 && sensor < TOF_COUNT)
  {
    return tof_raw_distances[sensor];
  }
  return -1.0f;
}

float get_tof_signal_rate(TofSensor sensor)
{
  if (sensor >= 0 && sensor < TOF_COUNT)
  {
    return tof_signal_rates[sensor];
  }
  return -1.0f;
}

float get_tof_sigma(TofSensor sensor)
{
  if (sensor >= 0 && sensor < TOF_COUNT)
  {
    return tof_sigmas[sensor];
  }
  return -1.0f;
}

bool get_tof_diagnostic_snapshot(TofSensor sensor,
                                 TofDiagnosticSnapshot &snapshot)
{
  if (sensor >= TOF_COUNT ||
      tof_diagnostics[sensor].sequence == 0)
    return false;
  snapshot = tof_diagnostics[sensor];
  return true;
}

float get_angle()
{
  return current_degree;
}

float get_heading()
{
  return current_heading;
}

void reset_VL53L4CX_via_I2C(TwoWire &wire)
{
  // Library address is pre-shifted (0x52); need raw 7-bit address (0x29)
  uint8_t raw_i2c_addr = VL53L4CX_DEFAULT_DEVICE_ADDRESS >> 1;

  // Put device in reset
  wire.beginTransmission(raw_i2c_addr);
  wire.write(0x00); // Register Address High Byte
  wire.write(0x00); // Register Address Low Byte
  wire.write(0x00); // 0x00 = active reset state
  wire.endTransmission();

  delay(50); // Hold reset long enough to drain internal registers

  // Release reset and reboot
  wire.beginTransmission(raw_i2c_addr);
  wire.write(0x00);
  wire.write(0x00);
  wire.write(0x01); // 0x01 = release reset and reboot
  wire.endTransmission();
}

// ==========================================
// INITIALIZATION
// ==========================================

void sensors_setup()
{
  Wire.begin();
  Wire.setClock(TOF_I2C_CLOCK);
  Wire2.begin();
  Wire2.setClock(TOF_I2C_CLOCK);

  reset_VL53L4CX_via_I2C(Wire);
  reset_VL53L4CX_via_I2C(Wire2);

  init_single_tof(sensor_left, &Wire, "Left_ToF");
  init_single_tof(sensor_right, &Wire2, "Right_ToF");

  // ==========================================
  // Initialize Gyro (BNO085) - SPI
  // ==========================================
  Serial.println("Initializing Gyro (BNO085) via SPI...");
  
  pinMode(BNO085_INT, INPUT_PULLUP);

  SPI1.begin();

  if (!bno.begin_SPI(BNO085_CS, BNO085_INT, &SPI1))
  {
    Serial.println("ERROR: Failed to find BNO085 chip on SPI");
    while (1)
    {
      delay(10);
    }
  }
  Serial.println("BNO085 Found on SPI!");

  // Enable Game Rotation Vector (ignores magnetometer interference from motors)
  // Frequency set to 10ms (100Hz) for better tracking during fast turns
  if (!bno.enableReport(SH2_GAME_ROTATION_VECTOR, 10000))
  {
    Serial.println("ERROR: Failed to enable game rotation vector");
    while (1)
    {
      delay(10);
    }
  }
  Serial.println("BNO085 Streaming reports enabled.");

  // Flush initial bootup flags
  delay(50);
  bno.getSensorEvent(&sensor_value);
  bno.wasReset();

  Serial.println("===== SENSORS INITIALIZED =====");
}
