#pragma once

#include <Arduino.h>
#include <arducam_dvp.h>
#include "GC2145/gc2145.h"

class FullFovGC2145 : public GC2145
{
public:
    explicit FullFovGC2145(arduino::MbedI2C &i2c = CameraWire);
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
    uint32_t getLastServiceTimeUs() const;
    uint32_t getCompletedFrameCount() const;

    uint16_t getWidth() const;
    uint16_t getHeight() const;

private:
    FullFovGC2145 sensor;
    Camera camera;
    FrameBuffer frameA;
    FrameBuffer frameB;

    uint8_t activeFrame = 0;
    uint8_t readyFrame = 0;
    uint32_t captureStartedUs = 0;
    uint32_t lastCaptureTimeUs = 0;
    uint32_t lastServiceTimeUs = 0;
    uint32_t completedFrameCount = 0;

    FrameBuffer& frameForIndex(uint8_t index);

    static constexpr uint16_t WIDTH = 320;
    static constexpr uint16_t HEIGHT = 240;
    static constexpr uint32_t FRAME_BYTES = WIDTH * HEIGHT * 2UL;
    static constexpr uint32_t FRAME_A_ADDRESS = 0x60000000UL;
    static constexpr uint32_t FRAME_B_ADDRESS =
        FRAME_A_ADDRESS + FRAME_BYTES;
};
