#include "obstacle_live_test.h"

#include <math.h>

#include "config.h"
#include "course_map.h"
#include "logger.h"
#include "motor_control.h"
#include "obstacle_path.h"
#include "position_estimator.h"
#include "sensors.h"

#define Serial robot_logger

namespace
{
enum LiveTestState : uint8_t
{
    LIVE_IDLE,
    LIVE_RUNNING,
    LIVE_BRAKING,
    LIVE_FINISHED
};

LiveTestState state = LIVE_IDLE;
int8_t requestedTurnSign = 1;
bool resultPassed = false;
const char *failureReason = nullptr;
float maximumCrossTrackError = 0.0f;
float maximumHeadingError = 0.0f;
uint32_t startTimeMs = 0;
uint32_t lastTelemetryMs = 0;
float windowMinimumMeasuredSpeed = 1.0e9f;
float windowMaximumMeasuredSpeed = -1.0e9f;
float windowMinimumDuty = 1.0e9f;
float windowMaximumDuty = -1.0e9f;
int windowMinimumTargetSpeed = 32767;
int windowMaximumTargetSpeed = -32768;

void resetDriveTelemetryWindow()
{
    windowMinimumMeasuredSpeed = 1.0e9f;
    windowMaximumMeasuredSpeed = -1.0e9f;
    windowMinimumDuty = 1.0e9f;
    windowMaximumDuty = -1.0e9f;
    windowMinimumTargetSpeed = 32767;
    windowMaximumTargetSpeed = -32768;
}

void sampleDriveTelemetry()
{
    windowMinimumMeasuredSpeed =
        fminf(windowMinimumMeasuredSpeed, measured_speed);
    windowMaximumMeasuredSpeed =
        fmaxf(windowMaximumMeasuredSpeed, measured_speed);
    windowMinimumDuty = fminf(windowMinimumDuty, dc_current_dc);
    windowMaximumDuty = fmaxf(windowMaximumDuty, dc_current_dc);
    windowMinimumTargetSpeed = min(windowMinimumTargetSpeed, target_speed);
    windowMaximumTargetSpeed = max(windowMaximumTargetSpeed, target_speed);
}

const char *observationStatusCode(ObstacleObservationStatus status)
{
    switch (status)
    {
    case OBSTACLE_OBSERVATION_NO_BLOB: return "NONE";
    case OBSTACLE_OBSERVATION_REJECTED_BLOB: return "REJECT";
    case OBSTACLE_OBSERVATION_INVALID_RANGE: return "RANGE";
    case OBSTACLE_OBSERVATION_NO_SEAT: return "NOSEAT";
    case OBSTACLE_OBSERVATION_VOTE: return "VOTE";
    case OBSTACLE_OBSERVATION_CONFIRMED: return "CONF";
    case OBSTACLE_OBSERVATION_ALREADY_CONFIRMED: return "KNOWN";
    }
    return "?";
}

void beginBraking(const char *reason)
{
    if (state != LIVE_RUNNING)
        return;
    failureReason = reason;
    state = LIVE_BRAKING;
    set_steering(0);
    set_speed(0);
    Serial.print("[LIVE PATH] ");
    Serial.println(reason == nullptr ? "Lap complete; braking" : reason);
}

void printTelemetry()
{
    const uint32_t now = millis();
    sampleDriveTelemetry();
    if (now - lastTelemetryMs < OBSTACLE_LIVE_TEST_TELEMETRY_MS)
        return;
    lastTelemetryMs = now;
    const PositionEstimate pose = get_position_struct();
    Serial.print("[LIVE PATH] t_ms="); Serial.print(now - startTimeMs);
    Serial.print(" idx="); Serial.print(obstacle_path_progress_index());
    Serial.print("/"); Serial.print(obstacle_path_waypoint_count());
    Serial.print(" lap="); Serial.print(obstacle_path_lap());
    Serial.print(" injections="); Serial.print(obstacle_path_injection_count());
    Serial.print(" cte_mm="); Serial.print(obstacle_path_cross_track_error_mm(), 1);
    Serial.print(" heading_err_deg="); Serial.print(obstacle_path_heading_error_deg(), 1);
    Serial.print(" pos="); Serial.print(pose.x_mm, 0);
    Serial.print(","); Serial.print(pose.y_mm, 0);
    Serial.print(" target="); Serial.print(windowMinimumTargetSpeed);
    Serial.print(".."); Serial.print(windowMaximumTargetSpeed);
    Serial.print(" profile="); Serial.print(current_speed, 0);
    Serial.print(" measured="); Serial.print(windowMinimumMeasuredSpeed, 0);
    Serial.print(".."); Serial.print(windowMaximumMeasuredSpeed, 0);
    Serial.print(" duty="); Serial.print(windowMinimumDuty, 0);
    Serial.print(".."); Serial.print(windowMaximumDuty, 0);
    Serial.print(" phase="); Serial.print(static_cast<int>(drive_control_phase));
    Serial.print(" nudge_deg=");
    Serial.print(obstacle_path_discovery_target_nudge_deg(), 1);
    Serial.print(" scan_seat=");
    Serial.print(static_cast<int>(obstacle_path_discovery_scan_seat()));
    ObstacleDiscoveryTelemetry discovery;
    if (obstacle_path_get_discovery_telemetry(discovery))
    {
        Serial.print(" disc=S");
        Serial.print(discovery.station / COURSE_STATIONS_PER_SECTION);
        Serial.print(".");
        Serial.print(discovery.station % COURSE_STATIONS_PER_SECTION);
        Serial.print(" vis=");
        Serial.print((discovery.visibleMask & 0x01) != 0 ? "R" : "-");
        Serial.print((discovery.visibleMask & 0x02) != 0 ? "L" : "-");
        Serial.print(" clear=");
        Serial.print(discovery.clearFrames[0]);
        Serial.print("/");
        Serial.print(discovery.clearFrames[1]);
        Serial.print(" obs=");
        Serial.print(observationStatusCode(discovery.observationStatus));
        Serial.print(":");
        Serial.print(discovery.observationSeat);
        if (discovery.observationStatus != OBSTACLE_OBSERVATION_NO_BLOB)
        {
            Serial.print(" blob=");
            Serial.print(discovery.left);
            Serial.print(",");
            Serial.print(discovery.top);
            Serial.print("-");
            Serial.print(discovery.right);
            Serial.print(",");
            Serial.print(discovery.bottom);
            Serial.print("@");
            Serial.print(discovery.bearingDeg, 1);
            Serial.print("/");
            Serial.print(discovery.rangeMm, 0);
        }
    }
    Serial.print(" tof="); Serial.print(get_tof_distance(TOF_LEFT), 0);
    Serial.print("/"); Serial.println(get_tof_distance(TOF_RIGHT), 0);
    resetDriveTelemetryWindow();
}

void finishAndReport()
{
    stop(false);
    set_steering(0);
    resultPassed = failureReason == nullptr &&
        obstacle_path_lap() == 1 &&
        obstacle_path_injection_count() == 1 &&
        maximumCrossTrackError <= OBSTACLE_PATH_TEST_PASS_CROSS_TRACK_MM;

    Serial.println("\n===== LIVE OBSTACLE PATH TEST RESULT =====");
    Serial.print("Result: "); Serial.println(resultPassed ? "PASS" : "FAIL");
    Serial.print("Direction: ");
    Serial.println(requestedTurnSign > 0 ? "LEFT/CCW" : "RIGHT/CW");
    Serial.print("Completed laps: "); Serial.println(obstacle_path_lap());
    Serial.print("Obstacle injections: "); Serial.println(obstacle_path_injection_count());
    Serial.print("Maximum cross-track error mm: "); Serial.println(maximumCrossTrackError, 1);
    Serial.print("Maximum heading error deg: "); Serial.println(maximumHeadingError, 1);
    if (failureReason != nullptr)
    {
        Serial.print("Failure reason: "); Serial.println(failureReason);
    }
    Serial.println("==========================================\n");
    state = LIVE_FINISHED;
}
} // namespace

void obstacle_live_test_set_turn_sign(int8_t turn_sign)
{
    requestedTurnSign = turn_sign < 0 ? -1 : 1;
}

void obstacle_live_test_start()
{
    state = LIVE_IDLE;
    resultPassed = false;
    failureReason = nullptr;
    maximumCrossTrackError = 0.0f;
    maximumHeadingError = 0.0f;
    startTimeMs = millis();
    lastTelemetryMs = startTimeMs;
    resetDriveTelemetryWindow();

    const float firstCorner = requestedTurnSign > 0
        ? OBSTACLE_PARKING_TO_FIRST_CORNER_CCW_MM
        : OBSTACLE_PARKING_TO_FIRST_CORNER_CW_MM;
    course_map_reset();
    obstacle_path_start(
        requestedTurnSign,
        false,
        firstCorner,
        1,
        OBSTACLE_PATH_TEST_MAX_SPEED_MM_S);

    Serial.println("\n===== LIVE ONE-LAP PURE PURSUIT TEST =====");
    Serial.println("Camera discovery/path injection: ENABLED");
    Serial.println("ToF pose correction: ENABLED (fresh frames only)");
    Serial.println("Parking exit: BYPASSED");
    Serial.print("Direction: ");
    Serial.println(requestedTurnSign > 0 ? "LEFT/CCW" : "RIGHT/CW");
    Serial.print("Speed cap mm/s: ");
    Serial.println(OBSTACLE_PATH_TEST_MAX_SPEED_MM_S, 0);
    Serial.println(
        "Raw ToF stop: DISABLED (side ToF cannot distinguish pillar from wall)");

    if (!obstacle_path_geometry_valid())
    {
        failureReason = "geometry preflight failed";
        stop(false);
        state = LIVE_FINISHED;
        Serial.println("[LIVE PATH] FAIL: geometry preflight failed");
        return;
    }

    Serial.println("[LIVE PATH] Ready; 'z' or switch LOW stops immediately");
    Serial.println("===========================================\n");
    state = LIVE_RUNNING;
}

void obstacle_live_test_update(bool new_camera_frame)
{
    if (state == LIVE_IDLE || state == LIVE_FINISHED)
        return;
    if (state == LIVE_BRAKING)
    {
        set_steering(0);
        set_speed(0);
        if (fabsf(current_speed) <= SOFT_STOP_SPEED_THRESHOLD_MMS &&
            fabsf(measured_speed) <= SOFT_STOP_SPEED_THRESHOLD_MMS)
            finishAndReport();
        return;
    }

    obstacle_path_update(new_camera_frame);
    const float crossTrack = obstacle_path_cross_track_error_mm();
    const float headingError = fabsf(obstacle_path_heading_error_deg());
    maximumCrossTrackError = fmaxf(maximumCrossTrackError, crossTrack);
    maximumHeadingError = fmaxf(maximumHeadingError, headingError);
    printTelemetry();

    if (obstacle_path_perception_blocked())
        beginBraking("ABORT: unresolved station reached perception limit");
    else if (crossTrack > OBSTACLE_PATH_TEST_ABORT_CROSS_TRACK_MM)
        beginBraking("ABORT: cross-track error exceeded limit");
    else if (millis() - startTimeMs > OBSTACLE_LIVE_TEST_TIMEOUT_MS)
        beginBraking("ABORT: test timeout");
    else if (obstacle_path_complete())
        beginBraking(nullptr);
}

void obstacle_live_test_stop()
{
    obstacle_path_reset();
    set_speed(0);
    set_steering(0);
    stop(false);
    state = LIVE_IDLE;
}

bool obstacle_live_test_finished()
{
    return state == LIVE_FINISHED;
}

bool obstacle_live_test_passed()
{
    return resultPassed;
}
