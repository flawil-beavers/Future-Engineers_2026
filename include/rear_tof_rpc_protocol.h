#pragma once

#include <Arduino.h>

constexpr uint32_t REAR_TOF_RPC_MAGIC = 0x5246544FUL; // "RFTO"
constexpr uint16_t REAR_TOF_RPC_VERSION = 1;
constexpr uint32_t REAR_TOF_RPC_STALE_MS = 250;

enum RearTofRpcStatus : uint16_t {
  REAR_TOF_RPC_STARTING = 0,
  REAR_TOF_RPC_RUNNING = 1,
  REAR_TOF_RPC_INIT_FAILED = 2,
  REAR_TOF_RPC_BUS_FAILED = 3,
};

struct RearTofRpcFrame {
  uint32_t magic;
  uint16_t version;
  uint16_t status;
  uint32_t sequence;
  float filtered_distance_mm;
  float raw_distance_mm;
  float signal_mcps;
  float sigma_mm;
};

static_assert(sizeof(RearTofRpcFrame) == 28,
              "Rear ToF RPC protocol layout changed");
