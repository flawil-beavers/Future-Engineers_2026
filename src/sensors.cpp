/**
 * @file sensors.cpp
 * @brief Sensor subsystem implementation
 */

#include "sensors.h"
#include "config.h"
#include <Wire.h>

// ==========================================
// SENSOR STATE VARIABLES
// ==========================================

// Gyro
static Adafruit_BNO08x bno = Adafruit_BNO08x(BNO085_RST);
static sh2_SensorValue_t sensor_value;
static float current_degree = 0;
static float current_heading = 0;

// ToF sensors (on separate I2C buses)
static VL53L4CX sensor_left(&Wire, -1);
static VL53L4CX sensor_right(&Wire2, -1);

// Distance readings in millimeters
static float tof_distances[TOF_COUNT] = {-1.0f, -1.0f};

// ==========================================
// SENSOR UPDATE FUNCTIONS
// ==========================================

void update_gyro()
{
  static unsigned long last_gyro_read = 0;
  static float last_yaw_deg = 0;
  static bool gyro_initialized = false;

  if (millis() - last_gyro_read < GYRO_UPDATE_INTERVAL_MS)
  {
    return; // Skip if not enough time has passed
  }

  if (digitalRead(BNO085_INT) == HIGH)
  {
    return;
  }

  last_gyro_read = millis();

  // Get sensor event
  bool has_event = bno.getSensorEvent(&sensor_value);

  // Check if sensor was reset
  if (bno.wasReset())
  {
    Serial.println("BNO085 was reset! Reinitializing...");
    delay(10);
    bno.enableReport(SH2_ROTATION_VECTOR, 50000);
    delay(30);
    return; // Drop this frame to let stream stabilize
  }

  // Parse rotation vector if available
  if (has_event)
  {
    if (sensor_value.sensorId == SH2_ROTATION_VECTOR)
    {
      sh2_RotationVectorWAcc_t rotationVector = sensor_value.un.rotationVector;
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
  uint8_t data_ready = 0;
  if (sensor.VL53L4CX_GetMeasurementDataReady(&data_ready) != VL53L4CX_ERROR_NONE || !data_ready)
  {
    return;
  }

  VL53L4CX_MultiRangingData_t ranging_data;
  if (sensor.VL53L4CX_GetMultiRangingData(&ranging_data) == VL53L4CX_ERROR_NONE)
  {
    if (ranging_data.NumberOfObjectsFound > 0)
    {
      // Find measurement with lowest sigma (mathematically most reliable)
      int best_idx = 0;
      uint32_t min_sigma = ranging_data.RangeData[0].SigmaMilliMeter;

      for (int i = 1; i < ranging_data.NumberOfObjectsFound; i++)
      {
        if (ranging_data.RangeData[i].SigmaMilliMeter < min_sigma)
        {
          min_sigma = ranging_data.RangeData[i].SigmaMilliMeter;
          best_idx = i;
        }
      }

      // Verify the hardware reports a valid lock on the object
      uint8_t status = ranging_data.RangeData[best_idx].RangeStatus;
      if (status == VL53L4CX_RANGESTATUS_RANGE_VALID ||
          status == VL53L4CX_RANGESTATUS_RANGE_VALID_MERGED_PULSE)
      {
        out_distance = (float)ranging_data.RangeData[best_idx].RangeMilliMeter;
      }
    }
  }
  
  sensor.VL53L4CX_ClearInterruptAndStartMeasurement();
}

void update_lasers()
{
  read_single_tof(sensor_left, tof_distances[TOF_LEFT]);
  read_single_tof(sensor_right, tof_distances[TOF_RIGHT]);
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

  sensor.VL53L4CX_SetDistanceMode(TOF_DISTANCE_MODE);
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

  // Enable rotation vector reports
  if (!bno.enableReport(SH2_ROTATION_VECTOR, 50000))
  {
    Serial.println("ERROR: Failed to enable rotation vector");
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
