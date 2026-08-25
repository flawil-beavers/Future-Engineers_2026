#pragma once

#include <Arduino.h>
#include <arducam_dvp.h>
#include "GC2145/gc2145.h"

class FullFovGC2145 : public GC2145
{
public:
    explicit FullFovGC2145(arduino::MbedI2C &i2c = CameraWire);
    uint32_t getClockFrequency() override { return 12000000; }
    int setResolution(int32_t resolution) override;

private:
    arduino::MbedI2C *i2c;

    int writeRegister(uint8_t reg, uint8_t value);
    int setWindow(
        uint8_t firstRegister,
        uint16_t x,
        uint16_t y,
        uint16_t width,
        uint16_t height);
};

class CameraSystem
{
public:
    CameraSystem();

    bool begin();
    bool capture();

    uint8_t* getBuffer();
    uint32_t getBufferSize();
    uint32_t getLastCaptureTimeUs() const;

    uint16_t getWidth() const;
    uint16_t getHeight() const;

private:
    FullFovGC2145 sensor;
    Camera camera;
    FrameBuffer frame;
    uint32_t lastCaptureTimeUs = 0;

    static constexpr uint16_t WIDTH = 320;
    static constexpr uint16_t HEIGHT = 240;
};
