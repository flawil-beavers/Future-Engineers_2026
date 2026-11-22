#pragma once

#include <Arduino.h>

/** Reset the final-parking controller for a new obstacle run. */
void final_parking_reset();

/** Start an isolated autonomous approach/scan/entry test before the bay. */
void final_parking_start_practice(int8_t turn_sign);

/** Entry only: surveyed capture pose, nominal247.5mm bay, no exit/laps/scan. */
void final_parking_start_entry_practice(int8_t turn_sign);

/** CW corner approach and marker registration followed by the fixed test sequence. */
void final_parking_start_cw_sequence_practice();

/**
 * Own the drive after the three-lap path is complete.
 *
 * @param turn_sign +1 for the canonical CCW/east orientation, -1 for CW/west.
 * @return true while the controller owns the drive.
 */
bool final_parking_update(int8_t turn_sign);

bool final_parking_complete();
bool final_parking_aborted();
bool final_parking_active();
// Record the main-loop gyro hold without changing its existing stop/recovery.
void final_parking_sensor_hold();

// Resume the same parking state, excluding gyro-pause time from its timeout.
void final_parking_sensor_resume();
const char *final_parking_trace_phase();

struct PositionEstimate;
// Bounded read-only planning during laps2/3; never issues drive commands.
void final_parking_prepare_approach(int8_t turn_sign,const PositionEstimate &arrival);
bool final_parking_rolling_ready(const PositionEstimate &pose);
bool final_parking_start_rolling(int8_t turn_sign);
bool final_parking_try_corner_approach(int8_t turn_sign,bool wallArc);
void final_parking_notify_lap_finish(bool complete);
