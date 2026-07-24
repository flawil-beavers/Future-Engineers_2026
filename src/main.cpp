/**
 * @file main.cpp
 * @brief Main application entry point
 *
 * Organizes robot control subsystems:
 *   - Motor control (DC motor, steering servo, PID)
 *   - Sensor management (Gyro, ToF distance sensors)
 *   - Serial communication and command parsing
 *   - Autonomous wall-following behavior
 *
 * The main loop coordinates all subsystems in a clean, modular architecture.
 */

#include <Arduino.h>
#include "config.h"
#include "motor_control.h"
#include "sensors.h"
#include "serial_handler.h"
#include "wall_follower.h"
#include "camera/camera.h"
#include "vision/vision.h"

CameraSystem camera;
Vision vision;

// ==========================================
// SETUP - Initialize all subsystems
// ==========================================

/**
 * @brief System entry point. Initializes hardware and starts the
 * control loop interface.
 */
void setup()
{
  Serial.println("\n===== ROBOT INITIALIZATION START =====\n");

  // Initialize each subsystem in order
  system_interface_setup(); // Sets up Enable Switch and Serial
  sensors_setup();
  motor_control_setup();
  gyro_follower_setup();

  Serial.println("\n===== INITIALIZATION COMPLETE =====\n");
  // Fully enable system if switch is already HIGH at startup
  if (system_enabled)
  {
    system_enable();
  }
  

  if (!camera.begin())
  {
    Serial.println("Camera failed!");

    while (true)
      ;
  }
  Serial.println("Camera initialized.");

  vision.begin();
  Serial.println("Vision initialized.");
}


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
    const Blob* obstacle = getLargestObstacle();

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




// ==========================================
// MAIN LOOP - Coordinate all subsystems
// ==========================================

/**
 * @brief Continuous execution loop. Manages sensor updates,
 * state machines, and low-level motor control.
 */
void loop()
{
  // Update timing and distances
  loop_updater();

  // Handle enable switch state
  handle_enable_switch();

  // Check for and process serial commands
  check_serial_available();

  // Camera + Image processing
  updateCameraVision();

  // Evaluate results
  handleObstacleDetection();

  // Debug only during development
  printVisionDebug();

  // Monitor motor health
  check_stalling();

  // Update distance sensors (needed for all subsystems)
  update_lasers();
  update_gyro();

  // Update wall follower (internal logic handles suppression but allows telemetry)
  gyro_follower_update(system_enabled);

  // Execute motor control logic (may be overridden by wall_follower)
  drive_loop();

  // Optional: Print debug info (uncomment to enable)
  // pid_config_print();
}




