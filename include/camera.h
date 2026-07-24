#pragma once

#include <Arduino.h>
#include <arducam_dvp.h>
#include "GC2145/gc2145.h"

class CameraSystem
{
public:
    CameraSystem();

    bool begin();
    bool capture();

    uint8_t* getBuffer();
    uint32_t getBufferSize();

    uint16_t getWidth() const;
    uint16_t getHeight() const;

private:
    GC2145 sensor;
    Camera camera;
    FrameBuffer frame;

    static constexpr uint16_t WIDTH = 320;
    static constexpr uint16_t HEIGHT = 240;
};