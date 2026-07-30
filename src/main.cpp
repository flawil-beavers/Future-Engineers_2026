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
#include "calibration.h"
#include "position_estimator.h"
#include "turn_radius_calibration.h"
#include "servo_center_calibration.h"
#include "pid_autotune.h"
#include "mode_manager.h"

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

    // Make the measured turn-radius model available to position estimation.
    {
        float left_coeffs[4] = {
            CAL_LEFT_A0, CAL_LEFT_A1, CAL_LEFT_A2, CAL_LEFT_A3
        };
        float right_coeffs[4] = {
            CAL_RIGHT_A0, CAL_RIGHT_A1, CAL_RIGHT_A2, CAL_RIGHT_A3
        };
        calibration_set_coefficients(left_coeffs, right_coeffs);
    }

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

    // Dead-reckoning estimate is maintained in both challenge modes.
    update_position();


    // ======================================
    // OPEN CHALLENGE
    // ======================================

    if (
        CHALLENGE_MODE ==
        CHALLENGE_OPEN
    )
    {
        // Open-challenge driving and all calibration tools share the
        // position-estimate branch's centralized mode manager.
        switch (current_mode)
        {
        case MODE_GYRO_FOLLOW:
            gyro_follower_update(system_enabled);
            drive_loop();
            break;

        case MODE_TURN_RADIUS_CAL:
            turn_radius_cal_update();
            drive_loop();
            break;

        case MODE_SERVO_CENTER_CAL:
            servo_center_cal_update();
            drive_loop();
            break;

        case MODE_PID_AUTOTUNE:
            // Autotune directly controls the motor output.
            pid_autotune_update();
            break;

        case MODE_NONE:
        default:
            // Keep manual serial steering/speed commands responsive.
            drive_loop();
            break;
        }
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


        // Large camera log writes can delay the gyro-controlled parking exit.
        if (!obstacle_parking_exit_active())
        {
            // Only during development.
            printVisionDebug();
        }

        drive_loop();
    }
}



