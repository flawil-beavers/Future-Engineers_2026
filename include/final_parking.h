#pragma once

#include <Arduino.h>

/** Reset the final-parking controller for a new obstacle run. */
void final_parking_reset();

/** Start an isolated autonomous approach/scan/entry test before the bay. */
void final_parking_start_practice(int8_t turn_sign);

/**
 * Own the drive after the three-lap path is complete.
 *
 * @param turn_sign +1 for the canonical CCW/east orientation, -1 for CW/west.
 * @return true while the controller owns the drive.
 */
bool final_parking_update(int8_t turn_sign);

bool final_parking_complete();
bool final_parking_aborted();
