#pragma once

#include <Arduino.h>

void obstacle_path_test_set_turn_sign(int8_t turn_sign);
void obstacle_path_test_start();
void obstacle_path_test_update();
void obstacle_path_test_stop();
bool obstacle_path_test_finished();
bool obstacle_path_test_passed();
