#pragma once

#include <Arduino.h>

void obstacle_seat_test_set_turn_sign(int8_t turn_sign);
void obstacle_seat_test_start();
void obstacle_seat_test_stop();
void obstacle_seat_test_update(bool new_camera_frame);
bool obstacle_seat_test_expect(
    uint8_t section,
    uint8_t station,
    char side,
    float range_mm);
void obstacle_seat_test_clear();
void obstacle_seat_test_show();
bool obstacle_seat_test_preflight_passed();
