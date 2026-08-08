#include "obstacle_path_test.h"

#include <math.h>

#include "config.h"
#include "motor_control.h"
#include "obstacle_path.h"
#include "position_estimator.h"
#include "sensors.h"
#include "logger.h"

#undef Serial
#define Serial robot_logger

namespace
{
enum TestState : uint8_t
{
    TEST_IDLE,
    TEST_RUNNING,
    TEST_BRAKING,
    TEST_FINISHED
};

TestState state = TEST_IDLE;
int8_t requestedTurnSign = 1;
bool resultPassed = false;
bool aborted = false;
const char *failureReason = nullptr;
PositionEstimate startPose;
float startEncoderDistance = 0.0f;
float maximumCrossTrackError = 0.0f;
float maximumHeadingError = 0.0f;
uint32_t startTimeMs = 0;
uint32_t lastTelemetryMs = 0;

float wrap180(float angle)
{
    while (angle > 180.0f)
        angle -= 360.0f;
    while (angle <= -180.0f)
        angle += 360.0f;
    return angle;
}

bool unsafeWallReading(float reading)
{
    return reading > 0.0f &&
           reading < OBSTACLE_PATH_TEST_WALL_STOP_MM;
}

void beginBraking(const char *reason)
{
    if (state != TEST_RUNNING)
        return;

    failureReason = reason;
    aborted = reason != nullptr;
    state = TEST_BRAKING;
    set_steering(0);
    set_speed(0);

    if (aborted)
    {
        Serial.print("[PATH TEST] ABORT: ");
        Serial.println(failureReason);
    }
    else
    {
        Serial.println("[PATH TEST] Lap complete; braking");
    }
}

void printTelemetry()
{
    const uint32_t now = millis();
    if (now - lastTelemetryMs < OBSTACLE_PATH_TEST_TELEMETRY_MS)
        return;
    lastTelemetryMs = now;

    const PositionEstimate pose = get_position_struct();
    Serial.print("[PATH TEST] t_ms=");
    Serial.print(now - startTimeMs);
    Serial.print(" idx=");
    Serial.print(obstacle_path_progress_index());
    Serial.print("/");
    Serial.print(obstacle_path_waypoint_count());
    Serial.print(" cte_mm=");
    Serial.print(obstacle_path_cross_track_error_mm(), 1);
    Serial.print(" heading_err_deg=");
    Serial.print(obstacle_path_heading_error_deg(), 1);
    Serial.print(" pos=");
    Serial.print(pose.x_mm, 0);
    Serial.print(",");
    Serial.print(pose.y_mm, 0);
    Serial.print(" speed=");
    Serial.print(current_speed, 0);
    Serial.print(" tof=");
    Serial.print(get_tof_distance(TOF_LEFT), 0);
    Serial.print("/");
    Serial.println(get_tof_distance(TOF_RIGHT), 0);
}

void finishAndReport()
{
    stop(false);
    set_steering(0);

    const PositionEstimate finalPose = get_position_struct();
    const float positionError = hypotf(
        finalPose.x_mm - startPose.x_mm,
        finalPose.y_mm - startPose.y_mm);
    const float headingError = fabsf(wrap180(
        finalPose.heading_deg - startPose.heading_deg));
    const float encoderTravel = fabsf(
        get_distance() - startEncoderDistance);
    const uint32_t elapsedMs = millis() - startTimeMs;

    resultPassed =
        !aborted &&
        obstacle_path_lap() == 1 &&
        positionError <= OBSTACLE_PATH_TEST_PASS_POSITION_MM &&
        headingError <= OBSTACLE_PATH_TEST_PASS_HEADING_DEG &&
        maximumCrossTrackError <=
            OBSTACLE_PATH_TEST_PASS_CROSS_TRACK_MM;

    Serial.println("\n===== EMPTY-TRACK PATH TEST RESULT =====");
    Serial.print("Result: ");
    Serial.println(resultPassed ? "PASS" : "FAIL");
    Serial.print("Direction: ");
    Serial.println(requestedTurnSign > 0 ? "LEFT/CCW" : "RIGHT/CW");
    Serial.print("Completed laps: ");
    Serial.println(obstacle_path_lap());
    Serial.print("Elapsed ms: ");
    Serial.println(elapsedMs);
    Serial.print("Encoder travel mm: ");
    Serial.println(encoderTravel, 1);
    Serial.print("Final position error mm: ");
    Serial.println(positionError, 1);
    Serial.print("Final heading error deg: ");
    Serial.println(headingError, 1);
    Serial.print("Maximum cross-track error mm: ");
    Serial.println(maximumCrossTrackError, 1);
    Serial.print("Maximum heading error deg: ");
    Serial.println(maximumHeadingError, 1);
    if (failureReason != nullptr)
    {
        Serial.print("Failure reason: ");
        Serial.println(failureReason);
    }
    Serial.println("Limits:");
    Serial.print("  position <= ");
    Serial.print(OBSTACLE_PATH_TEST_PASS_POSITION_MM, 0);
    Serial.println(" mm");
    Serial.print("  heading <= ");
    Serial.print(OBSTACLE_PATH_TEST_PASS_HEADING_DEG, 0);
    Serial.println(" deg");
    Serial.print("  cross-track <= ");
    Serial.print(OBSTACLE_PATH_TEST_PASS_CROSS_TRACK_MM, 0);
    Serial.println(" mm");
    Serial.println("========================================\n");

    state = TEST_FINISHED;
}
} // namespace

void obstacle_path_test_set_turn_sign(int8_t turn_sign)
{
    requestedTurnSign = turn_sign < 0 ? -1 : 1;
}

void obstacle_path_test_start()
{
    state = TEST_IDLE;
    resultPassed = false;
    aborted = false;
    failureReason = nullptr;
    maximumCrossTrackError = 0.0f;
    maximumHeadingError = 0.0f;
    startPose = get_position_struct();
    startEncoderDistance = get_distance();
    startTimeMs = millis();
    lastTelemetryMs = startTimeMs;

    obstacle_path_start(requestedTurnSign, true);

    Serial.println("\n===== EMPTY-TRACK PURE PURSUIT TEST =====");
    Serial.println("Camera steering: DISABLED");
    Serial.println("ToF pose correction: DISABLED");
    Serial.print("Direction: ");
    Serial.println(requestedTurnSign > 0 ? "LEFT/CCW" : "RIGHT/CW");
    Serial.print("Waypoints: ");
    Serial.println(obstacle_path_waypoint_count());
    Serial.print("Loop length mm: ");
    Serial.println(obstacle_path_loop_length_mm(), 1);
    Serial.print("Speed cap mm/s: ");
    Serial.println(OBSTACLE_PATH_TEST_MAX_SPEED_MM_S, 0);

    if (!obstacle_path_geometry_valid())
    {
        failureReason = "geometry preflight failed";
        aborted = true;
        set_speed(0);
        stop(false);
        state = TEST_FINISHED;
        Serial.println("[PATH TEST] FAIL: geometry preflight failed");
        return;
    }

    Serial.println("[PATH TEST] Geometry preflight PASS");
    Serial.println("[PATH TEST] Send 'z' or disable the switch to stop");
    Serial.println("=========================================\n");
    state = TEST_RUNNING;
}

void obstacle_path_test_update()
{
    if (state == TEST_IDLE || state == TEST_FINISHED)
        return;

    if (state == TEST_BRAKING)
    {
        set_steering(0);
        set_speed(0);
        if (fabsf(current_speed) <= SOFT_STOP_SPEED_THRESHOLD_MMS &&
            fabsf(measured_speed) <= SOFT_STOP_SPEED_THRESHOLD_MMS)
        {
            finishAndReport();
        }
        return;
    }

    obstacle_path_update(false);

    const float crossTrack = obstacle_path_cross_track_error_mm();
    const float headingError = fabsf(obstacle_path_heading_error_deg());
    if (crossTrack > maximumCrossTrackError)
        maximumCrossTrackError = crossTrack;
    if (headingError > maximumHeadingError)
        maximumHeadingError = headingError;

    printTelemetry();

    if (unsafeWallReading(get_tof_distance(TOF_LEFT)) ||
        unsafeWallReading(get_tof_distance(TOF_RIGHT)))
    {
        beginBraking("wall closer than safety limit");
    }
    else if (crossTrack > OBSTACLE_PATH_TEST_ABORT_CROSS_TRACK_MM)
    {
        beginBraking("cross-track error exceeded abort limit");
    }
    else if (millis() - startTimeMs > OBSTACLE_PATH_TEST_TIMEOUT_MS)
    {
        beginBraking("test timeout");
    }
    else if (obstacle_path_complete())
    {
        beginBraking(nullptr);
    }
}

void obstacle_path_test_stop()
{
    obstacle_path_reset();
    set_speed(0);
    set_steering(0);
    stop(false);
    state = TEST_IDLE;
}

bool obstacle_path_test_finished()
{
    return state == TEST_FINISHED;
}

bool obstacle_path_test_passed()
{
    return resultPassed;
}
