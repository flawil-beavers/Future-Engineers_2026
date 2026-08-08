#pragma once

#include <Arduino.h>

/** Known-field waypoint planner and Pure Pursuit controller for the
 * WRO Future Engineers Obstacle Challenge. */
void obstacle_path_reset();
void obstacle_path_start(int8_t turn_sign);
void obstacle_path_update(bool new_camera_frame);
bool obstacle_path_started();
bool obstacle_path_complete();
uint8_t obstacle_path_lap();
uint16_t obstacle_path_progress_index();
