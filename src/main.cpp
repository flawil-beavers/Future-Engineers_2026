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
#include "obstacle.h"

CameraSystem camera;
Vision vision;
#include "logger.h"
// Redirect all Serial output in this file through the USB logger
#define Serial robot_logger

// ==========================================
// SETUP - Initialize all subsystems
// ==========================================

/**
 * @brief System entry point. Initializes hardware and starts the
 * control loop interface.
 */

void setup()
{
    Serial.println(
        "\n===== ROBOT INITIALIZATION START =====\n"
    );


    // Common hardware
    system_interface_setup();

    sensors_setup();

    motor_control_setup();

    gyro_follower_setup();


    // ======================================
    // OBSTACLE CHALLENGE ONLY
    // ======================================

    if (
        CHALLENGE_MODE ==
        CHALLENGE_OBSTACLE
    )
    {
        if (!camera.begin())
        {
            Serial.println(
                "Camera failed!"
            );

            while (true)
                ;
        }


        Serial.println(
            "Camera initialized."
        );


        vision.begin();


        Serial.println(
            "Vision initialized."
        );


        obstacle_challenge_setup();
    }


    Serial.println(
        "\n===== INITIALIZATION COMPLETE =====\n"
    );


    if (system_enabled)
    {
        system_enable();
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
    // ======================================
    // COMMON
    // ======================================

    loop_updater();


    handle_enable_switch();


    check_serial_available();


    check_stalling();


    update_lasers();


    update_gyro();


    // ======================================
    // OPEN CHALLENGE
    // ======================================

    if (
        CHALLENGE_MODE ==
        CHALLENGE_OPEN
    )
    {
        // Exactly the existing Open controller.

        gyro_follower_update(
            system_enabled
        );
    }


    // ======================================
    // OBSTACLE CHALLENGE
    // ======================================

    else
    {
        const bool newCameraFrame =
            updateCameraVision();


        obstacle_challenge_update(
            system_enabled,
            newCameraFrame
        );


        // Only during development.
        printVisionDebug();
    }


    // ======================================
    // MOTOR OUTPUT
    // ======================================

    drive_loop();
}



