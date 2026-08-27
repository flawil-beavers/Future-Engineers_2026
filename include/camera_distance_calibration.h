#pragma once

#include <Arduino.h>

enum CameraDistanceCalState {
    CAMERA_DISTANCE_CAL_IDLE,
    CAMERA_DISTANCE_CAL_RUNNING,
    CAMERA_DISTANCE_CAL_DONE,
    CAMERA_DISTANCE_CAL_FAILED
};

extern CameraDistanceCalState camera_distance_cal_state;

/** Configure optional reverse travel; physical geometry comes from config.h. */
bool camera_distance_cal_configure(float reverse_travel_mm);

void camera_distance_cal_start();
void camera_distance_cal_update(bool new_camera_frame);
void camera_distance_cal_stop();
