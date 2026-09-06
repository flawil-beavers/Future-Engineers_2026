#pragma once

#include "rear_tof_rpc_protocol.h"

/** Start OpenAMP/RPC, which also boots the M4 image. */
bool rear_tof_rpc_setup();

/** Copy the newest non-stale M4 frame. Returns false when unavailable. */
bool rear_tof_rpc_read(RearTofRpcFrame &frame);

/** Whether the M4 RPC endpoint completed startup. */
bool rear_tof_rpc_connected();
