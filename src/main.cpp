#include <Arduino.h>
#include <Wire.h>
#include <vl53lx_class.h>

#define SERIAL_BAUD 115200

#define LEFT_XSDN  A1
#define RIGHT_XSDN A2

// ST Library nutzt 8-bit Adressen
#define LEFT_ADDR  0x52
#define RIGHT_ADDR 0x54

VL53LX sensor_left(&Wire, LEFT_XSDN);
VL53LX sensor_right(&Wire, RIGHT_XSDN);

void setup()
{
  Serial.begin(SERIAL_BAUD);
  delay(3000);

  Serial.println("===== START =====");

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

void loop()
{
  readSensor(sensor_left, "LEFT ");
  readSensor(sensor_right, "RIGHT");

  delay(50);
}