#include "obstacle.h"

#include "config.h"
#include "logger.h"

#define Serial robot_logger

bool updateCameraVision()
{
    static uint32_t last_camera_update = 0;
    if (millis() - last_camera_update < OBSTACLE_CAMERA_INTERVAL_MS)
        return false;

    last_camera_update = millis();
    if (!camera.capture()) {
        Serial.println("Camera capture failed.");
        return false;
    }

    return vision.update(
        camera.getBuffer(),
        camera.getWidth(),
        camera.getHeight());
}

void printCameraCalibration()
{
    static uint32_t last_print = 0;
    if (millis() - last_print < 500)
        return;

    last_print = millis();
    const uint16_t x = camera.getWidth() / 2;
    const uint16_t y = camera.getHeight() / 2;
    const HSV hsv = vision.getHSVAt(
        camera.getBuffer(),
        camera.getWidth(),
        camera.getHeight(),
        x,
        y);

    Serial.print("CENTER HSV -> H: ");
    Serial.print(hsv.h);
    Serial.print(" S: ");
    Serial.print(hsv.s);
    Serial.print(" V: ");
    Serial.println(hsv.v);
}

const Blob *getLargestObstacle()
{
    const VisionResult &result = vision.getResult();

    if (!result.red.found && !result.green.found)
        return nullptr;
    if (result.red.found && !result.green.found)
        return &result.red;
    if (result.green.found && !result.red.found)
        return &result.green;

    if (result.red.maxY != result.green.maxY) {
        return result.red.maxY > result.green.maxY
            ? &result.red
            : &result.green;
    }

    return result.red.area >= result.green.area
        ? &result.red
        : &result.green;
}
