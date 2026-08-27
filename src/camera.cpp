#include "camera.h"
#include "config.h"
#include <SDRAM.h>

FullFovGC2145::FullFovGC2145(arduino::MbedI2C &cameraI2c)
    : GC2145(cameraI2c),
      i2c(&cameraI2c)
{
}

int FullFovGC2145::writeRegister(uint8_t reg, uint8_t value)
{
    i2c->beginTransmission(GC2145_I2C_ADDR);
    i2c->write(reg);
    i2c->write(value);
    return i2c->endTransmission();
}

int FullFovGC2145::setWindow(
    uint8_t firstRegister,
    uint16_t x,
    uint16_t y,
    uint16_t width,
    uint16_t height)
{
    int result = writeRegister(0xFE, 0x00);
    result |= writeRegister(firstRegister++, y >> 8);
    result |= writeRegister(firstRegister++, y & 0xFF);
    result |= writeRegister(firstRegister++, x >> 8);
    result |= writeRegister(firstRegister++, x & 0xFF);
    result |= writeRegister(firstRegister++, height >> 8);
    result |= writeRegister(firstRegister++, height & 0xFF);
    result |= writeRegister(firstRegister++, width >> 8);
    result |= writeRegister(firstRegister, width & 0xFF);
    return result;
}

int FullFovGC2145::setResolution(int32_t resolution)
{
    if (resolution != CAMERA_R320x240)
        return -1;

    // The stock QVGA mode reads a centred 960 x 720 crop and subsamples 3:1.
    // Read the complete 1600 x 1200 sensor instead and subsample 5:1 directly
    // in the GC2145. The DCMI still receives an ordinary 320 x 240 frame.
    int result = setWindow(0x09, 0, 0, 1616, 1208);
    result |= setWindow(0x91, 0, 0, 320, 240);
    result |= writeRegister(0x90, 0x01); // enable crop/subsample output
    result |= writeRegister(0x99, 0x55); // row ratio 5, column ratio 5
    result |= writeRegister(0x9A, 0x0E); // enable subsample mode
    // The stock profile uses 12 MHz XCLK with the PLL enabled. With a 24 MHz
    // input, enable its documented input /2 stage so the first test preserves
    // exactly the known-good internal/PCLK timing. PLL_DIVX4 is varied only by
    // an explicit compile-time test profile.
    result |= writeRegister(0xF7, CAMERA_GC2145_PLL_MODE1);
    result |= writeRegister(0xF8, 0x80 | CAMERA_GC2145_PLL_DIVX4);
    result |= writeRegister(0x05, CAMERA_GC2145_HBLANK >> 8);
    result |= writeRegister(0x06, CAMERA_GC2145_HBLANK & 0xFF);
    result |= writeRegister(0x07, CAMERA_GC2145_VBLANK >> 8);
    result |= writeRegister(0x08, CAMERA_GC2145_VBLANK & 0xFF);
    // AEC anti-flicker is expressed in line clocks. Scale the documented
    // 0x0168 default with the PLL ratio (DIVX4 / 5) so 50 Hz lighting does not
    // produce alternating dark horizontal bands at faster profiles.
    constexpr uint16_t antiFlickerStep =
        (0x0168U * CAMERA_GC2145_PLL_DIVX4 + 2U) / 5U;
    result |= writeRegister(0xFE, 0x01);
    result |= writeRegister(0x25, antiFlickerStep >> 8);
    result |= writeRegister(0x26, antiFlickerStep & 0xFF);
    result |= writeRegister(0xFE, 0x00);
    return result == 0 ? 0 : -1;
}

uint8_t FullFovGC2145::readRegister(uint8_t reg)
{
    i2c->beginTransmission(GC2145_I2C_ADDR);
    i2c->write(reg);
    if (i2c->endTransmission(false) != 0)
        return 0;
    i2c->requestFrom(GC2145_I2C_ADDR, 1);
    return i2c->available() ? i2c->read() : 0;
}

uint16_t FullFovGC2145::getExposureLines()
{
    // The clock/timing configuration leaves the sensor on page zero.
    return (static_cast<uint16_t>(readRegister(0x03) & 0x1F) << 8) |
        readRegister(0x04);
}

CameraSystem::CameraSystem()
    : sensor(),
      camera(sensor),
      frameA(FRAME_A_ADDRESS),
      frameB(FRAME_B_ADDRESS)
{
}

bool CameraSystem::begin()
{
    // Do not add SDRAM to the general heap: these two fixed regions are owned
    // exclusively by camera DMA and remain 32-byte aligned.
    if (!SDRAM.begin(0))
        return false;

    if (!camera.begin(CAMERA_R320x240, CAMERA_RGB565, 30))
        return false;

    activeFrame = 0;
    readyFrame = 0;
    completedFrameCount = 0;
    previousFrameCompletedUs = 0;
    minFrameIntervalUs = UINT32_MAX;
    maxFrameIntervalUs = 0;
    longFrameIntervalCount = 0;
#if CAMERA_ASYNC_CAPTURE_ENABLED
    captureStartedUs = micros();
#if CAMERA_CONTINUOUS_CAPTURE_ENABLED
    return camera.startContinuous(frameA, frameB) == 0;
#else
    return camera.startFrame(frameForIndex(activeFrame)) == 0;
#endif
#else
    return true;
#endif
}

bool CameraSystem::capture()
{
    const uint32_t serviceStartedUs = micros();
#if CAMERA_ASYNC_CAPTURE_ENABLED
#if CAMERA_CONTINUOUS_CAPTURE_ENABLED
    FrameBuffer *completedFrame = nullptr;
    uint32_t sequence = 0;
    uint32_t frameCompletedUs = 0;
    if (!camera.acquireContinuousFrame(
            acquiredFrameSequence,
            completedFrame,
            sequence,
            frameCompletedUs))
    {
        return false;
    }

    if (acquiredFrameSequence != 0 && sequence > acquiredFrameSequence + 1U)
        discardedFrameCount += sequence - acquiredFrameSequence - 1U;
    acquiredFrameSequence = sequence;
    readyFrame = completedFrame == &frameB ? 1U : 0U;

    lastCaptureTimeUs = previousFrameCompletedUs == 0
        ? frameCompletedUs - captureStartedUs
        : frameCompletedUs - previousFrameCompletedUs;
    const int32_t readyWaitUs =
        static_cast<int32_t>(serviceStartedUs - frameCompletedUs);
    lastReadyWaitTimeUs = readyWaitUs > 0
        ? static_cast<uint32_t>(readyWaitUs)
        : 0U;
    lastFrameIntervalUs = previousFrameCompletedUs == 0
        ? 0
        : frameCompletedUs - previousFrameCompletedUs;
    if (lastFrameIntervalUs != 0)
    {
        if (lastFrameIntervalUs < minFrameIntervalUs)
            minFrameIntervalUs = lastFrameIntervalUs;
        if (lastFrameIntervalUs > maxFrameIntervalUs)
            maxFrameIntervalUs = lastFrameIntervalUs;
        if (lastFrameIntervalUs > 120000UL)
            ++longFrameIntervalCount;
    }
    previousFrameCompletedUs = frameCompletedUs;
    lastServiceTimeUs = micros() - serviceStartedUs;
    completedFrameCount = sequence;
    return true;
#else
    if (!camera.frameReady())
        return false;

    uint32_t frameCompletedUs = camera.frameCompletedTimeUs();
    if (frameCompletedUs == 0)
        frameCompletedUs = serviceStartedUs;
    lastCaptureTimeUs = frameCompletedUs - captureStartedUs;
    const int32_t readyWaitUs =
        static_cast<int32_t>(serviceStartedUs - frameCompletedUs);
    lastReadyWaitTimeUs = readyWaitUs > 0
        ? static_cast<uint32_t>(readyWaitUs)
        : 0U;
    lastFrameIntervalUs = previousFrameCompletedUs == 0
        ? 0
        : frameCompletedUs - previousFrameCompletedUs;
    if (lastFrameIntervalUs != 0)
    {
        if (lastFrameIntervalUs < minFrameIntervalUs)
            minFrameIntervalUs = lastFrameIntervalUs;
        if (lastFrameIntervalUs > maxFrameIntervalUs)
            maxFrameIntervalUs = lastFrameIntervalUs;
        // Normal completion spacing is about 79.62 ms. More than 1.5 frame
        // periods means snapshot capture missed at least one sensor frame.
        if (lastFrameIntervalUs > 120000UL)
            ++longFrameIntervalCount;
    }
    previousFrameCompletedUs = frameCompletedUs;
    if (camera.finishFrame() != 0)
        return false;

    readyFrame = activeFrame;
    activeFrame ^= 1U;

    captureStartedUs = micros();
    if (camera.startFrame(frameForIndex(activeFrame)) != 0)
        return false;

    lastServiceTimeUs = micros() - serviceStartedUs;
    ++completedFrameCount;
    return true;
#endif
#else
    captureStartedUs = serviceStartedUs;
    if (camera.grabFrame(frameA, 3000) != 0)
        return false;
    readyFrame = 0;
    lastCaptureTimeUs = micros() - captureStartedUs;
    lastServiceTimeUs = lastCaptureTimeUs;
    ++completedFrameCount;
    return true;
#endif
}

uint8_t* CameraSystem::getBuffer()
{
    return frameForIndex(readyFrame).getBuffer();
}

uint32_t CameraSystem::getBufferSize()
{
    return FRAME_BYTES;
}

uint32_t CameraSystem::getLastCaptureTimeUs() const
{
    return lastCaptureTimeUs;
}

uint32_t CameraSystem::getLastReadyWaitTimeUs() const
{
    return lastReadyWaitTimeUs;
}

uint32_t CameraSystem::getLastFrameIntervalUs() const
{
    return lastFrameIntervalUs;
}

uint32_t CameraSystem::getMinFrameIntervalUs() const
{
    return minFrameIntervalUs == UINT32_MAX ? 0 : minFrameIntervalUs;
}

uint32_t CameraSystem::getMaxFrameIntervalUs() const
{
    return maxFrameIntervalUs;
}

uint32_t CameraSystem::getLongFrameIntervalCount() const
{
    return longFrameIntervalCount;
}

uint32_t CameraSystem::getDiscardedFrameCount() const
{
    return discardedFrameCount;
}

uint32_t CameraSystem::getCaptureErrorCount() const
{
#if CAMERA_CONTINUOUS_CAPTURE_ENABLED
    return camera.continuousErrorCount();
#else
    return 0;
#endif
}

uint16_t CameraSystem::getExposureLines()
{
    return sensor.getExposureLines();
}

uint32_t CameraSystem::getLastServiceTimeUs() const
{
    return lastServiceTimeUs;
}

uint32_t CameraSystem::getCompletedFrameCount() const
{
    return completedFrameCount;
}

FrameBuffer& CameraSystem::frameForIndex(uint8_t index)
{
    return index == 0 ? frameA : frameB;
}

uint16_t CameraSystem::getWidth() const
{
    return WIDTH;
}

uint16_t CameraSystem::getHeight() const
{
    return HEIGHT;
}
