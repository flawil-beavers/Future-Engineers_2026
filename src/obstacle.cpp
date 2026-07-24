#include "obstacle.h"

// ==========================================
// Camera functions
// ==========================================

void updateCameraVision()
{
    static uint32_t lastCameraUpdate = 0;

    constexpr uint32_t CAMERA_INTERVAL_MS = 50;

    if (millis() - lastCameraUpdate < CAMERA_INTERVAL_MS)
    {
        return;
    }

    lastCameraUpdate = millis();

    if (!camera.capture())
    {
        Serial.println("Camera capture failed.");
        return;
    }

    vision.update(
        camera.getBuffer(),
        camera.getWidth(),
        camera.getHeight());
}

void printVisionDebug()
{
    static uint32_t lastPrint = 0;

    if (millis() - lastPrint < 500)
        return;

    lastPrint = millis();

    const VisionResult &v =
        vision.getResult();

    Serial.println();
    Serial.println("======================");
    Serial.println("VISION");
    Serial.println("======================");

    Serial.print("Processing: ");
    Serial.print(
        v.processingTimeUs / 1000.0f);
    Serial.println(" ms");

    if (v.red.found)
    {
        Serial.println();
        Serial.println("RED:");

        Serial.print("X: ");
        Serial.println(v.red.centerX);

        Serial.print("Y: ");
        Serial.println(v.red.centerY);

        Serial.print("Width: ");
        Serial.println(v.red.width());

        Serial.print("Height: ");
        Serial.println(v.red.height());

        Serial.print("Area: ");
        Serial.println(v.red.area);

        Serial.print("Error X: ");
        Serial.println(v.red.centerError());
    }

    if (v.green.found)
    {
        Serial.println();
        Serial.println("GREEN:");

        Serial.print("X: ");
        Serial.println(v.green.centerX);

        Serial.print("Y: ");
        Serial.println(v.green.centerY);

        Serial.print("Width: ");
        Serial.println(v.green.width());

        Serial.print("Height: ");
        Serial.println(v.green.height());

        Serial.print("Area: ");
        Serial.println(v.green.area);

        Serial.print("Error X: ");
        Serial.println(v.green.centerError());
    }

    if (v.orange.found)
    {
        Serial.println();
        Serial.println("ORANGE LINE:");

        Serial.print("X: ");
        Serial.println(v.orange.centerX);

        Serial.print("Y: ");
        Serial.println(v.orange.centerY);

        Serial.print("Area: ");
        Serial.println(v.orange.area);
    }

    if (v.blue.found)
    {
        Serial.println();
        Serial.println("BLUE LINE:");

        Serial.print("X: ");
        Serial.println(v.blue.centerX);

        Serial.print("Y: ");
        Serial.println(v.blue.centerY);

        Serial.print("Area: ");
        Serial.println(v.blue.area);
    }
}

void printCameraCalibration()
{
    static uint32_t lastPrint = 0;

    if (millis() - lastPrint < 500)
        return;

    lastPrint = millis();

    const uint16_t x = camera.getWidth() / 2;
    const uint16_t y = camera.getHeight() / 2;

    HSV hsv = vision.getHSVAt(
        camera.getBuffer(),
        camera.getWidth(),
        camera.getHeight(),
        x,
        y);

    Serial.print("CENTER HSV -> H: ");
    Serial.print(hsv.h);

    Serial.print("  S: ");
    Serial.print(hsv.s);

    Serial.print("  V: ");
    Serial.println(hsv.v);
}

const Blob *getLargestObstacle()
{
    const VisionResult &v =
        vision.getResult();

    if (!v.red.found && !v.green.found)
    {
        return nullptr;
    }

    if (v.red.found && !v.green.found)
    {
        return &v.red;
    }

    if (v.green.found && !v.red.found)
    {
        return &v.green;
    }

    // Both are visible:
    // use the larger blob for now.

    if (v.red.area >= v.green.area)
    {
        return &v.red;
    }

    return &v.green;
}

void handleObstacleDetection()
{
    const Blob *obstacle = getLargestObstacle();

    if (obstacle == nullptr)
    {
        return;
    }

    Serial.print("Obstacle X: ");
    Serial.print(obstacle->centerX);

    Serial.print(" Area: ");
    Serial.print(obstacle->area);

    Serial.print(" Color: ");

    if (obstacle->color == ColorType::RED)
    {
        Serial.println("RED");
    }
    else if (obstacle->color == ColorType::GREEN)
    {
        Serial.println("GREEN");
    }
}