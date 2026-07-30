/**
 * @file main.cpp
 * @brief Hardware initialization and mode-independent application loop.
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
#include "mode_manager.h"
#include "logger.h"
#define Serial robot_logger

// Obstacle resources are initialized lazily when an obstacle mode starts.
CameraSystem camera;
Vision vision;

void setup()
{
    Serial.println("\n===== ROBOT INITIALIZATION START =====\n");

    system_interface_setup();
    sensors_setup();
    motor_control_setup();
    gyro_follower_setup();

    // Load the measured turn-radius model used by dead reckoning.
    float left_coeffs[4] = {
        CAL_LEFT_A0, CAL_LEFT_A1, CAL_LEFT_A2, CAL_LEFT_A3
    };
    float right_coeffs[4] = {
        CAL_RIGHT_A0, CAL_RIGHT_A1, CAL_RIGHT_A2, CAL_RIGHT_A3
    };
    calibration_set_coefficients(left_coeffs, right_coeffs);

    Serial.println("\n===== INITIALIZATION COMPLETE =====\n");

    // This starts immediately when enabled, otherwise mode_switch() stores it
    // as the pending mode for the physical enable switch.
    mode_switch(STARTUP_ROBOT_MODE);
}

void loop()
{
    loop_updater();
    handle_enable_switch();
    check_serial_available();

    update_lasers();
    update_gyro();
    update_position();
    check_stalling();

    mode_update();
}
