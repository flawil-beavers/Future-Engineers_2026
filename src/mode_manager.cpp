/**
 * @file mode_manager.cpp
 * @brief Central dispatcher for every robot operating mode.
 */

#include "mode_manager.h"
#include "motor_control.h"
#include "turn_radius_calibration.h"
#include "servo_center_calibration.h"
#include "pid_autotune.h"
#include "motor_min_calibration.h"
#include "navigation_controller.h"
#include "obstacle.h"
#include "obstacle_path_test.h"
#include "obstacle_live_test.h"
#include "obstacle_seat_test.h"
#include "sensors.h"
#include "position_estimator.h"
#include "camera_distance_calibration.h"
#include "logger.h"
#define Serial robot_logger

extern bool nav_debug_enabled;
extern float last_loop_time;
extern unsigned long current_time;
static unsigned long debug_loop_count = 0;
static float debug_loop_time_sum_us = 0.0f;
static float debug_loop_time_max_us = 0.0f;
static unsigned long debug_last_print_time = 0;

RobotMode current_mode = MODE_NONE;
RobotMode pending_mode = MODE_NONE;

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

    case MODE_OBSTACLE_PATH_TEST:
        obstacle_path_test_stop();
        break;

    case MODE_OBSTACLE_LIVE_TEST:
        obstacle_live_test_stop();
        break;

    case MODE_OBSTACLE_SEAT_TEST:
        obstacle_seat_test_stop();
        break;

    case MODE_OBSTACLE_BENCH:
        obstacle_bench_test_set(false);
        navigation_set_obstacle_mode(false);
        break;

    case MODE_CAMERA_CALIBRATION:
        stop(false);
        set_steering(0);
        break;

    case MODE_CAMERA_DISTANCE_CAL:
        camera_distance_cal_stop();
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

    case MODE_MOTOR_MIN_CAL:
        stop(false);
        motor_min_cal_stop();
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

    case MODE_OBSTACLE_PATH_TEST:
        obstacle_path_test_start();
        break;

    case MODE_OBSTACLE_LIVE_TEST:
        if (!obstacle_camera_setup())
            return false;
        obstacle_live_test_start();
        break;

    case MODE_OBSTACLE_SEAT_TEST:
        if (!obstacle_camera_setup())
            return false;
        obstacle_seat_test_start();
        if (!obstacle_seat_test_preflight_passed())
            return false;
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

    case MODE_CAMERA_DISTANCE_CAL:
        if (!obstacle_camera_setup())
            return false;
        camera_distance_cal_start();
        if (camera_distance_cal_state == CAMERA_DISTANCE_CAL_FAILED)
            return false;
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

    case MODE_MOTOR_MIN_CAL:
        motor_min_cal_start();
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
        return obstacle_challenge_complete()
            ? MODE_RESULT_COMPLETED
            : MODE_RESULT_RUNNING;
    }

    case MODE_OBSTACLE_PATH_TEST:
        obstacle_path_test_update();
        drive_loop();
        if (!obstacle_path_test_finished())
            return MODE_RESULT_RUNNING;
        return obstacle_path_test_passed()
            ? MODE_RESULT_COMPLETED
            : MODE_RESULT_FAILED;

    case MODE_OBSTACLE_LIVE_TEST:
        obstacle_live_test_update(updateCameraVision());
        drive_loop();
        if (!obstacle_live_test_finished())
            return MODE_RESULT_RUNNING;
        return obstacle_live_test_passed()
            ? MODE_RESULT_COMPLETED
            : MODE_RESULT_FAILED;

    case MODE_OBSTACLE_SEAT_TEST:
        obstacle_seat_test_update(updateCameraVision());
        drive_loop();
        return MODE_RESULT_RUNNING;

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

    case MODE_CAMERA_DISTANCE_CAL:
        camera_distance_cal_update(updateCameraVision());
        drive_loop();
        if (camera_distance_cal_state == CAMERA_DISTANCE_CAL_DONE)
            return MODE_RESULT_COMPLETED;
        if (camera_distance_cal_state == CAMERA_DISTANCE_CAL_FAILED)
            return MODE_RESULT_FAILED;
        return MODE_RESULT_RUNNING;

    case MODE_TURN_RADIUS_CAL:
        turn_radius_cal_update();
        drive_loop();
        if (turn_radius_state == TR_DONE)
            return MODE_RESULT_COMPLETED;
        if (turn_radius_state == TR_FAILED)
            return MODE_RESULT_FAILED;
        return MODE_RESULT_RUNNING;

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
        return (pid_atune_result.aborted ||
                !pid_atune_result.valid ||
                !pid_atune_result.applied)
            ? MODE_RESULT_FAILED
            : MODE_RESULT_COMPLETED;

    case MODE_MOTOR_MIN_CAL:
        motor_min_cal_update();
        drive_loop();
        return motor_min_cal_state == MC_DONE
            ? MODE_RESULT_COMPLETED
            : MODE_RESULT_RUNNING;

    case MODE_NONE:
        // Service steering output while stopped without energizing the motor.
        drive_loop();
        return MODE_RESULT_RUNNING;
    }

    return MODE_RESULT_FAILED;
}

void mode_update()
{
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

    const bool wait_for_right_turn_cal =
        updated_mode == MODE_TURN_RADIUS_CAL &&
        turn_radius_cal_waiting_for_right();
    stop_mode(updated_mode);
    current_mode = MODE_NONE;
    pending_mode = wait_for_right_turn_cal ? MODE_TURN_RADIUS_CAL : MODE_NONE;
}

void mode_pause()
{
    if (current_mode != MODE_NONE) {
        const RobotMode paused_mode = current_mode;
        // A camera-distance calibration can only start while the pillar is
        // touching the configured robot-front plane. After any movement, a
        // generic resume would establish a false distance origin.
        pending_mode = paused_mode == MODE_CAMERA_DISTANCE_CAL
            ? MODE_NONE
            : paused_mode;
        stop_mode(current_mode);
        current_mode = MODE_NONE;

        if (paused_mode == MODE_CAMERA_DISTANCE_CAL)
            Serial.println("Camera calibration cancelled; return to pillar contact and send camdrive again.");
        else {
            Serial.print("Paused. Pending mode: ");
            Serial.println(mode_name(pending_mode));
        }
    }

    system_disable();
}

void mode_resume()
{
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
    case MODE_OBSTACLE_PATH_TEST: return "OBSTACLE_PATH_TEST";
    case MODE_OBSTACLE_LIVE_TEST: return "OBSTACLE_LIVE_TEST";
    case MODE_OBSTACLE_SEAT_TEST: return "OBSTACLE_SEAT_TEST";
    case MODE_OBSTACLE_BENCH:     return "OBSTACLE_BENCH";
    case MODE_CAMERA_CALIBRATION:  return "CAMERA_CALIBRATION";
    case MODE_CAMERA_DISTANCE_CAL: return "CAMERA_DISTANCE_CAL";
    case MODE_TURN_RADIUS_CAL:    return "TURN_RADIUS_CAL";
    case MODE_SERVO_CENTER_CAL:   return "SERVO_CENTER_CAL";
    case MODE_PID_AUTOTUNE:       return "PID_AUTOTUNE";
    case MODE_MOTOR_MIN_CAL:      return "MOTOR_MIN_CAL";
    default:                      return "UNKNOWN";
    }
}

void general_debug_print()
{
    if (!nav_debug_enabled)
        return;

    const float current_loop_us = last_loop_time * 1000000.0f;
    debug_loop_count++;
    debug_loop_time_sum_us += current_loop_us;
    if (current_loop_us > debug_loop_time_max_us) {
        debug_loop_time_max_us = current_loop_us;
    }

    if (current_time - debug_last_print_time < 200000)
        return;

    const float avg_loop_ms = (debug_loop_count > 0)
        ? (debug_loop_time_sum_us / debug_loop_count) / 1000.0f
        : 0.0f;
    const float max_loop_ms = debug_loop_time_max_us / 1000.0f;
    const float last_loop_ms = last_loop_time * 1000.0f;

    debug_last_print_time = current_time;

    Serial.print("[DEBUG] Mode: ");
    Serial.print(mode_name(current_mode));

    // Base telemetry (Speed, Steer, Heading, Position, ToF, Distance)
    Serial.print(" | Speed: ");
    Serial.print(current_speed, 1);
    Serial.print(" | Steer: ");
    Serial.print(set_degree);
    Serial.print(" | Angle: ");
    Serial.print(get_angle(), 1);

    float px = 0, py = 0, pheading = 0;
    get_position(px, py, pheading);
    Serial.print(" | Pos: (");
    Serial.print(px, 0);
    Serial.print(", ");
    Serial.print(py, 0);
    Serial.print(")");

    Serial.print(" | Tof L/R: ");
    Serial.print(get_tof_distance(TOF_LEFT), 0);
    Serial.print("/");
    Serial.print(get_tof_distance(TOF_RIGHT), 0);
    Serial.print(" | Dist: ");
    Serial.print(get_distance(), 0);

    // Call mode-specific debug printing routines if available
    if (current_mode == MODE_OPEN_CHALLENGE || current_mode == MODE_OBSTACLE_CHALLENGE || current_mode == MODE_OBSTACLE_BENCH) {
        navigation_print_debug();
    }

    // Loop timing metrics
    Serial.print(" | Loop Last: ");
    Serial.print(last_loop_ms, 3);
    Serial.print("ms | Avg: ");
    Serial.print(avg_loop_ms, 3);
    Serial.print("ms | Max: ");
    Serial.print(max_loop_ms, 3);
    Serial.println("ms");

    debug_loop_count = 0;
    debug_loop_time_sum_us = 0.0f;
    debug_loop_time_max_us = 0.0f;
}
