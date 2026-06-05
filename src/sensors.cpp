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
Adafruit_BNO08x bno = Adafruit_BNO08x(BNO085_RST);
sh2_SensorValue_t sensor_value;
float current_degree = 0;
float current_heading = 0;
float last_yaw_deg = 0;
bool gyro_initialized = false;

// Time tracking for gyro updates
unsigned long last_gyro_read = 0;

// ToF sensors (on separate I2C buses)
VL53L4CX sensor_left(&Wire, -1);
VL53L4CX sensor_right(&Wire2, -1);

// Distance readings in millimeters
float current_distance_left = -1.0;
float current_distance_right = -1.0;

// ==========================================
// SENSOR UPDATE FUNCTIONS
// ==========================================

void update_gyro()
{
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

void update_lasers()
{
  VL53L4CX_MultiRangingData_t MultiRangingData;
  uint8_t NewDataReady = 0;

  // ==========================================
  // LEFT SENSOR (Wire)
  // ==========================================
  if (sensor_left.VL53L4CX_GetMeasurementDataReady(&NewDataReady) == VL53L4CX_ERROR_NONE && NewDataReady)
  {
    if (sensor_left.VL53L4CX_GetMultiRangingData(&MultiRangingData) == VL53L4CX_ERROR_NONE)
    {
      if (MultiRangingData.NumberOfObjectsFound > 0)
      {
        // Find measurement with lowest sigma (most reliable)
        int best_index = 0;
        uint32_t best_sigma = MultiRangingData.RangeData[0].SigmaMilliMeter;

        for (int i = 1; i < MultiRangingData.NumberOfObjectsFound; i++)
        {
          if (MultiRangingData.RangeData[i].SigmaMilliMeter < best_sigma)
          {
            best_sigma = MultiRangingData.RangeData[i].SigmaMilliMeter;
            best_index = i;
          }
        }

        uint8_t status = MultiRangingData.RangeData[best_index].RangeStatus;
        if (status == VL53L4CX_RANGESTATUS_RANGE_VALID ||
            status == VL53L4CX_RANGESTATUS_RANGE_VALID_MERGED_PULSE)
        {
          int32_t mm = MultiRangingData.RangeData[best_index].RangeMilliMeter;
          current_distance_left = mm;
        }
      }
    }
    else
    {
      Serial.println("Failed to read left sensor data");
    }

    sensor_left.VL53L4CX_ClearInterruptAndStartMeasurement();
  }

  // ==========================================
  // RIGHT SENSOR (Wire2)
  // ==========================================
  NewDataReady = 0;
  if (sensor_right.VL53L4CX_GetMeasurementDataReady(&NewDataReady) == VL53L4CX_ERROR_NONE && NewDataReady)
  {
    if (sensor_right.VL53L4CX_GetMultiRangingData(&MultiRangingData) == VL53L4CX_ERROR_NONE)
    {
      if (MultiRangingData.NumberOfObjectsFound > 0)
      {
        // Find measurement with lowest sigma (most reliable)
        int best_index = 0;
        uint32_t best_sigma = MultiRangingData.RangeData[0].SigmaMilliMeter;

        for (int i = 1; i < MultiRangingData.NumberOfObjectsFound; i++)
        {
          if (MultiRangingData.RangeData[i].SigmaMilliMeter < best_sigma)
          {
            best_sigma = MultiRangingData.RangeData[i].SigmaMilliMeter;
            best_index = i;
          }
        }

        uint8_t status = MultiRangingData.RangeData[best_index].RangeStatus;
        if (status == VL53L4CX_RANGESTATUS_RANGE_VALID ||
            status == VL53L4CX_RANGESTATUS_RANGE_VALID_MERGED_PULSE)
        {
          int32_t mm = MultiRangingData.RangeData[best_index].RangeMilliMeter;
          current_distance_right = mm;
        }
      }
    }
    else
    {
      Serial.println("Failed to read right sensor data");
    }

    sensor_right.VL53L4CX_ClearInterruptAndStartMeasurement();
  }
}

// ==========================================
// HELPER FUNCTIONS
// ==========================================

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
  // ==========================================
  // Initialize I2C buses
  // ==========================================
  Wire.begin();
  Wire.setClock(TOF_I2C_CLOCK); // 400kHz

  Wire2.begin();
  Wire2.setClock(TOF_I2C_CLOCK); // 400kHz

  // ==========================================
  // Initialize ToF Sensors
  // ==========================================
  Serial.println("Initializing VL53L4CX Left Sensor (Wire)...");

  reset_VL53L4CX_via_I2C(Wire);  // Reset left sensor
  reset_VL53L4CX_via_I2C(Wire2); // Reset right sensor

  // Bind left sensor to Wire
  sensor_left.setI2cDevice(&Wire);

  if (sensor_left.begin() != 0)
  {
    Serial.println("ERROR: VL53L4CX left communication failed!");
    while (1)
    {
      delay(10);
    }
  }
  Serial.println("VL53L4CX Left Base Communication established.");

  // Load calibration
  if (sensor_left.InitSensor(VL53L4CX_DEFAULT_DEVICE_ADDRESS) != VL53L4CX_ERROR_NONE)
  {
    Serial.println("ERROR: InitSensor failed!");
    while (1)
    {
      delay(10);
    }
  }
  Serial.println("VL53L4CX Left Calibration & Firmware Loaded.");

  // Configure distance mode
  sensor_left.VL53L4CX_SetDistanceMode(TOF_DISTANCE_MODE);

  // Start measurements
  if (sensor_left.VL53L4CX_StartMeasurement() != 0)
  {
    Serial.println("ERROR: Could not start VL53L4CX left measurements!");
    while (1)
    {
      delay(10);
    }
  }
  Serial.println("VL53L4CX left initialized.");

  // ==========================================
  // Initialize Right ToF Sensor
  // ==========================================
  Serial.println("Initializing VL53L4CX Right Sensor (Wire2)...");

  sensor_right.setI2cDevice(&Wire2);

  if (sensor_right.begin() != 0)
  {
    Serial.println("ERROR: VL53L4CX right communication failed!");
    while (1)
    {
      delay(10);
    }
  }
  Serial.println("VL53L4CX Right Base Communication established.");

  if (sensor_right.InitSensor(VL53L4CX_DEFAULT_DEVICE_ADDRESS) != VL53L4CX_ERROR_NONE)
  {
    Serial.println("ERROR: InitSensor failed for right sensor!");
    while (1)
    {
      delay(10);
    }
  }
  Serial.println("VL53L4CX Right Calibration & Firmware Loaded.");

  sensor_right.VL53L4CX_SetDistanceMode(TOF_DISTANCE_MODE);

  if (sensor_right.VL53L4CX_StartMeasurement() != 0)
  {
    Serial.println("ERROR: Could not start VL53L4CX right measurements!");
    while (1)
    {
      delay(10);
    }
  }
  Serial.println("VL53L4CX right initialized.");

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
