#pragma once

#include <Arduino.h>

void obstacle_live_test_set_turn_sign(int8_t turn_sign);
void obstacle_live_test_start();
void obstacle_live_test_update(bool new_camera_frame);
void obstacle_live_test_stop();
bool obstacle_live_test_finished();
bool obstacle_live_test_passed();
