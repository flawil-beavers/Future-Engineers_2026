#include "camera.h"

CameraSystem::CameraSystem()
    : sensor(),
      camera(sensor),
      frame()
{
}

bool CameraSystem::begin()
{
    // 320 x 240
    // RGB565
    // 30 FPS
    if (!camera.begin(CAMERA_R320x240, CAMERA_RGB565, 30))
    {
        return false;
    }

    // Camera is mounted upside down
    
    return true;
}

bool CameraSystem::capture()
{
    return camera.grabFrame(frame, 3000) == 0;
}

uint8_t* CameraSystem::getBuffer()
{
    return frame.getBuffer();
}

uint32_t CameraSystem::getBufferSize()
{
    return frame.getBufferSize();
}

uint16_t CameraSystem::getWidth() const
{
    return WIDTH;
}

uint16_t CameraSystem::getHeight() const
{
    return HEIGHT;
}