#ifndef CAMERA_H
#define CAMERA_H

#include <Arduino.h>

struct Detection {
    bool found;
    int x;
    int y;
    int size;
};

struct CameraResults {
    Detection red_block;
    Detection green_block;
};

void camera_setup();
void camera_update();
CameraResults get_camera_results();

#endif // CAMERA_H