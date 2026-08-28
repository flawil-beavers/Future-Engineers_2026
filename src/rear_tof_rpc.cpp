#include "rear_tof_rpc.h"

#include <RPC.h>
#include <cstring>
#include <mbed.h>

namespace {
rtos::Mutex frameMutex;
RearTofRpcFrame latestFrame = {};
uint32_t latestReceiveMs = 0;
bool rpcConnected = false;
bool frameReceived = false;

void receiveRearTofFrame(const uint8_t *data, size_t length)
{
  if (length != sizeof(RearTofRpcFrame))
    return;

  RearTofRpcFrame candidate;
  memcpy(&candidate, data, sizeof(candidate));
  if (candidate.magic != REAR_TOF_RPC_MAGIC ||
      candidate.version != REAR_TOF_RPC_VERSION)
    return;

  frameMutex.lock();
  latestFrame = candidate;
  latestReceiveMs = millis();
  frameReceived = true;
  frameMutex.unlock();
}
} // namespace

bool rear_tof_rpc_setup()
{
  RPC.attach(receiveRearTofFrame);
  rpcConnected = RPC.begin() != 0;
  return rpcConnected;
}

bool rear_tof_rpc_read(RearTofRpcFrame &frame)
{
  frameMutex.lock();
  const bool available = frameReceived;
  const uint32_t receivedMs = latestReceiveMs;
  if (available)
    frame = latestFrame;
  frameMutex.unlock();

  return available &&
         millis() - receivedMs <= REAR_TOF_RPC_STALE_MS &&
         frame.status == REAR_TOF_RPC_RUNNING;
}

bool rear_tof_rpc_connected()
{
  return rpcConnected;
}
