#include <Arduino.h>
#include <RPC.h>
#include <vl53l4cx_class.h>

#include "config.h"
#include "rear_tof_rpc_protocol.h"
#include "software_i2c.h"

namespace {
constexpr pin_size_t REAR_TOF_SDA_PIN = A3;
constexpr pin_size_t REAR_TOF_SCL_PIN = A4;
constexpr uint32_t REAR_TOF_SOFTWARE_I2C_CLOCK = 100000;
constexpr uint32_t STATUS_HEARTBEAT_MS = 250;
constexpr uint32_t MEASUREMENT_TIMEOUT_MS = 250;

SoftwareI2C rearBus(REAR_TOF_SDA_PIN, REAR_TOF_SCL_PIN);
VL53L4CX rearSensor(&rearBus, -1);
RearTofRpcFrame frame = {
    REAR_TOF_RPC_MAGIC,
    REAR_TOF_RPC_VERSION,
    REAR_TOF_RPC_STARTING,
    0,
    -1.0f,
    -1.0f,
    -1.0f,
    -1.0f,
};
float previousDistanceMm = -1.0f;
uint32_t lastReadyPollUs = 0;
uint32_t lastPublishMs = 0;
uint32_t lastMeasurementMs = 0;
bool rpcReady = false;

void publishFrame()
{
  RPC.write(reinterpret_cast<const uint8_t *>(&frame), sizeof(frame));
  lastPublishMs = millis();
}

void resetSensor()
{
  constexpr uint8_t address = VL53L4CX_DEFAULT_DEVICE_ADDRESS >> 1;
  rearBus.beginTransmission(address);
  rearBus.write(0x00);
  rearBus.write(0x00);
  rearBus.write(0x00);
  rearBus.endTransmission();
  delay(50);
  rearBus.beginTransmission(address);
  rearBus.write(0x00);
  rearBus.write(0x00);
  rearBus.write(0x01);
  rearBus.endTransmission();
  delay(10);
}

bool initializeSensor()
{
  rearBus.begin();
  rearBus.setClock(REAR_TOF_SOFTWARE_I2C_CLOCK);
  resetSensor();
  if (rearSensor.begin() != 0 ||
      rearSensor.InitSensor(VL53L4CX_DEFAULT_DEVICE_ADDRESS) !=
          VL53L4CX_ERROR_NONE)
    return false;
  if (rearSensor.VL53L4CX_SetDistanceMode(TOF_DISTANCE_MODE) !=
      VL53L4CX_ERROR_NONE)
    return false;
  if (rearSensor.VL53L4CX_SetMeasurementTimingBudgetMicroSeconds(
          TOF_TIMING_BUDGET_US) != VL53L4CX_ERROR_NONE)
    return false;
  return rearSensor.VL53L4CX_StartMeasurement() == VL53L4CX_ERROR_NONE;
}

void pollSensor()
{
  const uint32_t nowUs = micros();
  if (lastReadyPollUs != 0 &&
      nowUs - lastReadyPollUs < TOF_READY_POLL_INTERVAL_US)
    return;
  lastReadyPollUs = nowUs;

  uint8_t ready = 0;
  if (rearSensor.VL53L4CX_GetMeasurementDataReady(&ready) !=
      VL53L4CX_ERROR_NONE) {
    frame.status = REAR_TOF_RPC_BUS_FAILED;
    return;
  }
  if (!ready)
    return;

  VL53L4CX_MultiRangingData_t rangingData;
  float selectedDistance = TOF_OUT_OF_RANGE_MM;
  float selectedRaw = -1.0f;
  float selectedSignal = -1.0f;
  float selectedSigma = -1.0f;
  int16_t largestAcceptedDistance = -1;

  if (rearSensor.VL53L4CX_GetMultiRangingData(&rangingData) ==
      VL53L4CX_ERROR_NONE) {
    for (uint8_t i = 0; i < rangingData.NumberOfObjectsFound; ++i) {
      const auto &candidate = rangingData.RangeData[i];
      const float signal = candidate.SignalRateRtnMegaCps / 65536.0f;
      const float sigma = candidate.SigmaMilliMeter / 65536.0f;
      const bool hardwareValid =
          candidate.RangeStatus == VL53L4CX_RANGESTATUS_RANGE_VALID ||
          candidate.RangeStatus ==
              VL53L4CX_RANGESTATUS_RANGE_VALID_MERGED_PULSE;
      if (hardwareValid && signal > 0.3f && sigma < 20.0f &&
          candidate.RangeMilliMeter > largestAcceptedDistance) {
        largestAcceptedDistance = candidate.RangeMilliMeter;
        selectedRaw = candidate.RangeMilliMeter;
        selectedSignal = signal;
        selectedSigma = sigma;
      }
    }
    if (largestAcceptedDistance >= 0)
      selectedDistance = largestAcceptedDistance;
  } else {
    frame.status = REAR_TOF_RPC_BUS_FAILED;
  }

  if (selectedDistance > REAR_TOF_MAX_RELIABLE_DISTANCE_MM)
    selectedDistance = TOF_OUT_OF_RANGE_MM;
  if (selectedDistance != TOF_OUT_OF_RANGE_MM &&
      previousDistanceMm >= 0.0f &&
      previousDistanceMm != TOF_OUT_OF_RANGE_MM) {
    const float delta = selectedDistance - previousDistanceMm;
    if (fabsf(delta) > TOF_MAX_DELTA_MM)
      selectedDistance = previousDistanceMm +
                         (delta > 0.0f ? TOF_MAX_DELTA_MM
                                       : -TOF_MAX_DELTA_MM);
  }

  if (rearSensor.VL53L4CX_ClearInterruptAndStartMeasurement() !=
      VL53L4CX_ERROR_NONE) {
    frame.status = REAR_TOF_RPC_BUS_FAILED;
  } else {
    frame.status = REAR_TOF_RPC_RUNNING;
  }
  previousDistanceMm = selectedDistance;
  frame.filtered_distance_mm = selectedDistance;
  frame.raw_distance_mm = selectedRaw;
  frame.signal_mcps = selectedSignal;
  frame.sigma_mm = selectedSigma;
  ++frame.sequence;
  lastMeasurementMs = millis();
  publishFrame();
}
} // namespace

void setup()
{
  rpcReady = RPC.begin() != 0;
  if (!rpcReady)
    return;
  publishFrame();
  frame.status = initializeSensor() ? REAR_TOF_RPC_RUNNING
                                    : REAR_TOF_RPC_INIT_FAILED;
  if (frame.status == REAR_TOF_RPC_RUNNING)
    lastMeasurementMs = millis();
  publishFrame();
}

void loop()
{
  if (!rpcReady) {
    delay(1000);
    return;
  }
  if (frame.status == REAR_TOF_RPC_RUNNING ||
      frame.status == REAR_TOF_RPC_BUS_FAILED)
    pollSensor();
  if (frame.status == REAR_TOF_RPC_RUNNING &&
      millis() - lastMeasurementMs > MEASUREMENT_TIMEOUT_MS)
    frame.status = REAR_TOF_RPC_BUS_FAILED;
  if (millis() - lastPublishMs >= STATUS_HEARTBEAT_MS)
    publishFrame();
  delay(1);
}
