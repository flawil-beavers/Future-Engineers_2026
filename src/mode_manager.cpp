/**
 * @file mode_manager.cpp
 * @brief Central dispatcher for every robot operating mode.
 */

#include "mode_manager.h"
#include "motor_control.h"
#include "turn_radius_calibration.h"
#include "servo_center_calibration.h"
#include "pid_autotune.h"
#include "navigation_controller.h"
#include "obstacle.h"
#include "logger.h"
#define Serial robot_logger

RobotMode current_mode = MODE_NONE;
RobotMode pending_mode = MODE_NONE;
static bool resume_after_logging = false;

static bool obstacle_camera_setup()
{
    static bool initialized = false;
    if (initialized)
        return true;

    if (!camera.begin()) {
        Serial.println("Camera initialization failed; mode not started.");
        return false;
    }

    vision.begin();
    initialized = true;
    Serial.println("Camera and vision initialized.");
    return true;
}

static void stop_mode(RobotMode mode)
{
    switch (mode) {
    case MODE_MANUAL:
        stop(false);
        set_steering(0);
        break;

    case MODE_HOLD:
        stop(false);
        break;

    case MODE_OPEN_CHALLENGE:
        navigation_disable();
        break;

    case MODE_OBSTACLE_CHALLENGE:
        obstacle_challenge_update(false, false);
        navigation_disable();
        navigation_set_obstacle_mode(false);
        break;

    case MODE_OBSTACLE_BENCH:
        obstacle_bench_test_set(false);
        navigation_set_obstacle_mode(false);
        break;

    case MODE_CAMERA_CALIBRATION:
        stop(false);
        set_steering(0);
        break;

    case MODE_TURN_RADIUS_CAL:
        stop(false);
        turn_radius_cal_stop();
        break;

    case MODE_SERVO_CENTER_CAL:
        stop(false);
        servo_center_cal_stop();
        break;

    case MODE_PID_AUTOTUNE:
        stop(false);
        pid_autotune_stop();
        break;

    case MODE_NONE:
        break;
    }
}

static bool start_mode(RobotMode mode)
{
    switch (mode) {
    case MODE_MANUAL:
        stop(false);
        set_steering(0);
        break;

    case MODE_HOLD:
        stop(true);
        servo_disabled = false;
        break;

    case MODE_OPEN_CHALLENGE:
        navigation_set_obstacle_mode(false);
        navigation_enable();
        break;

    case MODE_OBSTACLE_CHALLENGE:
        if (!obstacle_camera_setup())
            return false;
        obstacle_bench_test_set(false);
        obstacle_challenge_setup();
        navigation_enable();
        break;

    case MODE_OBSTACLE_BENCH:
        if (!obstacle_camera_setup())
            return false;
        navigation_set_obstacle_mode(true);
        obstacle_bench_test_set(true);
        break;

    case MODE_CAMERA_CALIBRATION:
        if (!obstacle_camera_setup())
            return false;
        stop(false);
        set_steering(0);
        break;

    case MODE_TURN_RADIUS_CAL:
        turn_radius_cal_start();
        break;

    case MODE_SERVO_CENTER_CAL:
        servo_center_cal_start();
        break;

    case MODE_PID_AUTOTUNE:
        pid_autotune_start();
        break;

    case MODE_NONE:
        break;
    }

    return true;
}

void mode_switch(RobotMode new_mode)
{
    if (new_mode == current_mode) {
        Serial.print("Already in mode: ");
        Serial.println(mode_name(new_mode));
        return;
    }

    stop_mode(current_mode);
    current_mode = MODE_NONE;

    if (new_mode == MODE_NONE) {
        pending_mode = MODE_NONE;
        Serial.println("Mode switched to: NONE");
        return;
    }

    if (!system_enabled) {
        pending_mode = new_mode;
        Serial.print("System disabled. Pending mode: ");
        Serial.println(mode_name(new_mode));
        return;
    }

    pending_mode = MODE_NONE;
    // Hardware enable is mode-agnostic; the selected mode owns controller setup.
    system_enable();
    if (!start_mode(new_mode)) {
        stop(false);
        Serial.print("Could not start mode: ");
        Serial.println(mode_name(new_mode));
        return;
    }

    current_mode = new_mode;
    Serial.print("Mode switched to: ");
    Serial.println(mode_name(new_mode));
}

static ModeResult update_active_mode()
{
    switch (current_mode) {
    case MODE_MANUAL:
        drive_loop();
        return MODE_RESULT_RUNNING;

    case MODE_HOLD:
        drive_loop();
        return MODE_RESULT_RUNNING;

    case MODE_OPEN_CHALLENGE:
        navigation_update(system_enabled);
        drive_loop();
        return navigation_is_complete()
            ? MODE_RESULT_COMPLETED
            : MODE_RESULT_RUNNING;

    case MODE_OBSTACLE_CHALLENGE: {
        const bool new_camera_frame = updateCameraVision();
        obstacle_challenge_update(system_enabled, new_camera_frame);
        if (!obstacle_parking_exit_active())
            printVisionDebug();
        drive_loop();
        return navigation_is_complete()
            ? MODE_RESULT_COMPLETED
            : MODE_RESULT_RUNNING;
    }

    case MODE_OBSTACLE_BENCH:
        obstacle_challenge_update(
            system_enabled,
            updateCameraVision());
        drive_loop();
        return MODE_RESULT_RUNNING;

    case MODE_CAMERA_CALIBRATION:
        if (updateCameraVision())
            printCameraCalibration();
        drive_loop();
        return MODE_RESULT_RUNNING;

    case MODE_TURN_RADIUS_CAL:
        turn_radius_cal_update();
        drive_loop();
        return turn_radius_state == TR_DONE
            ? MODE_RESULT_COMPLETED
            : MODE_RESULT_RUNNING;

    case MODE_SERVO_CENTER_CAL:
        servo_center_cal_update();
        drive_loop();
        return servo_center_state == SC_DONE
            ? MODE_RESULT_COMPLETED
            : MODE_RESULT_RUNNING;

    case MODE_PID_AUTOTUNE:
        pid_autotune_update();
        if (pid_atune_state != AT_DONE)
            return MODE_RESULT_RUNNING;
        return pid_atune_result.aborted
            ? MODE_RESULT_FAILED
            : MODE_RESULT_COMPLETED;

    case MODE_NONE:
        // Service steering output while stopped without energizing the motor.
        drive_loop();
        return MODE_RESULT_RUNNING;
    }

    return MODE_RESULT_FAILED;
}

void mode_update()
{
    if (resume_after_logging && !robot_logger.is_busy()) {
        resume_after_logging = false;
        mode_resume();
    }

    const RobotMode updated_mode = current_mode;
    const ModeResult result = update_active_mode();
    if (updated_mode == MODE_NONE || result == MODE_RESULT_RUNNING)
        return;

    Serial.print("Mode ");
    Serial.print(mode_name(updated_mode));
    Serial.println(
        result == MODE_RESULT_COMPLETED
            ? " completed."
            : " failed.");

    stop_mode(updated_mode);
    current_mode = MODE_NONE;
    pending_mode = MODE_NONE;
}

void mode_pause()
{
    resume_after_logging = false;
    if (current_mode != MODE_NONE) {
        pending_mode = current_mode;
        stop_mode(current_mode);
        current_mode = MODE_NONE;

        Serial.print("Paused. Pending mode: ");
        Serial.println(mode_name(pending_mode));
    }

    system_disable();
}

void mode_resume()
{
    if (robot_logger.is_busy()) {
        resume_after_logging = true;
        Serial.println("Resume deferred until USB logging is complete.");
        return;
    }

    system_enable();

    if (pending_mode == MODE_NONE) {
        Serial.println("System enabled (no pending mode).");
        return;
    }

    const RobotMode mode_to_start = pending_mode;
    pending_mode = MODE_NONE;

    if (!start_mode(mode_to_start)) {
        stop(false);
        Serial.print("Could not resume mode: ");
        Serial.println(mode_name(mode_to_start));
        return;
    }

    current_mode = mode_to_start;
    Serial.print("Resumed mode: ");
    Serial.println(mode_name(current_mode));
}

void mode_stop_all()
{
    resume_after_logging = false;
    stop_mode(current_mode);
    current_mode = MODE_NONE;
    pending_mode = MODE_NONE;
    stop(false);
    set_steering(0);
    Serial.println("All modes stopped.");
}

const char* mode_name(RobotMode mode)
{
    switch (mode) {
    case MODE_NONE:               return "NONE";
    case MODE_MANUAL:             return "MANUAL";
    case MODE_HOLD:               return "HOLD";
    case MODE_OPEN_CHALLENGE:     return "OPEN_CHALLENGE";
    case MODE_OBSTACLE_CHALLENGE: return "OBSTACLE_CHALLENGE";
    case MODE_OBSTACLE_BENCH:     return "OBSTACLE_BENCH";
    case MODE_CAMERA_CALIBRATION:  return "CAMERA_CALIBRATION";
    case MODE_TURN_RADIUS_CAL:    return "TURN_RADIUS_CAL";
    case MODE_SERVO_CENTER_CAL:   return "SERVO_CENTER_CAL";
    case MODE_PID_AUTOTUNE:       return "PID_AUTOTUNE";
    default:                      return "UNKNOWN";
    }
}
