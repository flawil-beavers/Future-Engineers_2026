#include "obstacle_live_test.h"

#include <math.h>
#include <string.h>

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
uint8_t requestedLapTarget = 1;
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

struct PassageTofSamples
{
    uint32_t lastSequence = 0;
    uint16_t freshSamples = 0;
    uint16_t validRawSamples = 0;
    uint16_t validFilteredSamples = 0;
    float minimumRawMm = 1.0e9f;
    float minimumFilteredMm = 1.0e9f;
};

struct PillarTofPassage
{
    bool initialized = false;
    bool reported = false;
    uint8_t lapIndex = 0;
    float seatPathDistanceMm = 0.0f;
    PassageTofSamples samples[TOF_SIDE_COUNT];
    uint16_t odometrySamples = 0;
    ObstacleClearanceSample odometryMinimum;
};

PillarTofPassage pillarPassages[OBSTACLE_SEAT_COUNT];

float tofWheelEnvelopeInset(TofSensor sensor)
{
    return sensor == TOF_LEFT
        ? OBSTACLE_TOF_LEFT_TO_WHEEL_ENVELOPE_MM
        : OBSTACLE_TOF_RIGHT_TO_WHEEL_ENVELOPE_MM;
}

bool validPassageRange(float rangeMm)
{
    return isfinite(rangeMm) && rangeMm > 0.0f &&
        rangeMm <= TOF_MAX_RELIABLE_DISTANCE_MM;
}

bool addFreshPassageSample(
    PassageTofSamples &samples,
    const TofDiagnosticSnapshot &snapshot)
{
    if (snapshot.sequence == 0 || snapshot.sequence == samples.lastSequence)
        return false;
    samples.lastSequence = snapshot.sequence;
    if (samples.freshSamples < 65535)
        ++samples.freshSamples;

    if (validPassageRange(snapshot.selected_raw_distance_mm))
    {
        samples.minimumRawMm = fminf(
            samples.minimumRawMm,
            snapshot.selected_raw_distance_mm);
        if (samples.validRawSamples < 65535)
            ++samples.validRawSamples;
    }
    if (validPassageRange(snapshot.filtered_distance_mm))
    {
        samples.minimumFilteredMm = fminf(
            samples.minimumFilteredMm,
            snapshot.filtered_distance_mm);
        if (samples.validFilteredSamples < 65535)
            ++samples.validFilteredSamples;
    }
    return true;
}

void resetPassageMeasurements(PillarTofPassage &passage)
{
    passage.reported = false;
    passage.odometrySamples = 0;
    passage.odometryMinimum = ObstacleClearanceSample{};
    passage.odometryMinimum.pillarMm = 1.0e9f;
    passage.odometryMinimum.wallMm = 1.0e9f;
    for (uint8_t corner = 0; corner < 4; ++corner)
        passage.odometryMinimum.innerCornerMm[corner] = 1.0e9f;
    for (uint8_t side = 0; side < TOF_SIDE_COUNT; ++side)
    {
        passage.samples[side] = PassageTofSamples{};
        TofDiagnosticSnapshot snapshot;
        if (get_tof_diagnostic_snapshot(
                static_cast<TofSensor>(side), snapshot))
            passage.samples[side].lastSequence = snapshot.sequence;
        passage.samples[side].minimumRawMm = 1.0e9f;
        passage.samples[side].minimumFilteredMm = 1.0e9f;
    }
}

void resetPillarTofPassages()
{
    memset(pillarPassages, 0, sizeof(pillarPassages));
    for (uint8_t seatId = 0; seatId < obstacle_path_seat_count(); ++seatId)
    {
        ObstacleSeatInfo seat;
        if (!obstacle_path_get_seat(seatId, seat))
            continue;
        PillarTofPassage &passage = pillarPassages[seatId];
        passage.initialized = true;
        passage.lapIndex = 0;
        passage.seatPathDistanceMm = seat.pathDistanceMm;
        resetPassageMeasurements(passage);
    }
}

void addOdometryClearanceSample(
    PillarTofPassage &passage,
    const ObstacleClearanceSample &instant)
{
    if (!instant.valid)
        return;
    ObstacleClearanceSample &minimum = passage.odometryMinimum;
    if (!minimum.valid || instant.pillarMm < minimum.pillarMm)
    {
        minimum.pillarMm = instant.pillarMm;
        minimum.robotXmm = instant.robotXmm;
        minimum.robotYmm = instant.robotYmm;
        minimum.robotHeadingDeg = instant.robotHeadingDeg;
    }
    if (!minimum.valid || instant.wallMm < minimum.wallMm)
    {
        minimum.wallMm = instant.wallMm;
        minimum.wallFeature = instant.wallFeature;
        minimum.wallXmm = instant.wallXmm;
        minimum.wallYmm = instant.wallYmm;
        minimum.wallRobotXmm = instant.wallRobotXmm;
        minimum.wallRobotYmm = instant.wallRobotYmm;
        minimum.wallRobotHeadingDeg = instant.wallRobotHeadingDeg;
    }
    for (uint8_t corner = 0; corner < 4; ++corner)
        minimum.innerCornerMm[corner] = fminf(
            minimum.innerCornerMm[corner], instant.innerCornerMm[corner]);
    minimum.valid = true;
    if (passage.odometrySamples < 65535)
        ++passage.odometrySamples;
}

void printGeometryClearance(
    const char *source,
    uint8_t seatId,
    const ObstacleClearanceSample &sample,
    uint16_t samples = 0)
{
    Serial.print("[CLEARANCE "); Serial.print(source); Serial.print("] seat=");
    Serial.print(seatId);
    if (!sample.valid)
    {
        Serial.println(" unavailable");
        return;
    }
    if (samples > 0)
    {
        Serial.print(" samples=");
        Serial.print(samples);
    }
    if (strcmp(source, "PLAN") == 0)
        Serial.print(" snapshot=route-activation");
    Serial.print(" pillar_min_mm="); Serial.print(sample.pillarMm, 1);
    Serial.print(" pillar_pose=");
    Serial.print(sample.robotXmm, 0); Serial.print(",");
    Serial.print(sample.robotYmm, 0); Serial.print("@");
    Serial.print(sample.robotHeadingDeg, 1);
    Serial.print(" wall_min_mm="); Serial.print(sample.wallMm, 1);
    Serial.print(" wall_feature=");
    Serial.print(obstacle_path_wall_feature_name(sample.wallFeature));
    Serial.print(" wall_point=");
    Serial.print(sample.wallXmm, 0); Serial.print(",");
    Serial.print(sample.wallYmm, 0);
    Serial.print(" wall_pose=");
    Serial.print(sample.wallRobotXmm, 0); Serial.print(",");
    Serial.print(sample.wallRobotYmm, 0); Serial.print("@");
    Serial.print(sample.wallRobotHeadingDeg, 1);
    Serial.print(" inner_corners_SW_SE_NE_NW_mm=");
    for (uint8_t corner = 0; corner < 4; ++corner)
    {
        if (corner > 0) Serial.print("/");
        Serial.print(sample.innerCornerMm[corner], 1);
    }
    Serial.println(" envelope=capsule_r70_rear-axle_to_front-plane");
}

void printPassageRange(
    const char *label,
    const char *clearanceLabel,
    float minimumMm,
    uint16_t validSamples,
    float insetMm)
{
    Serial.print(" "); Serial.print(label); Serial.print("=");
    if (validSamples == 0)
    {
        Serial.print("NA");
        return;
    }
    Serial.print(minimumMm, 1);
    Serial.print(" "); Serial.print(clearanceLabel); Serial.print("=");
    Serial.print(minimumMm - insetMm, 1);
}

bool reportPillarTofPassage(
    uint8_t seatId,
    PillarTofPassage &passage,
    bool completeWindow)
{
    ObstacleSeatInfo seat;
    if (passage.reported ||
        !obstacle_path_get_seat(seatId, seat) ||
        !seat.confirmed)
        return false;
    passage.reported = true;
    // Red is passed on its right, leaving the pillar on the robot's left.
    // Green is passed on its left, leaving the pillar on the robot's right.
    const TofSensor pillarSensor = seat.red ? TOF_LEFT : TOF_RIGHT;
    const TofSensor wallSensor =
        pillarSensor == TOF_LEFT ? TOF_RIGHT : TOF_LEFT;
    const PassageTofSamples &pillar = passage.samples[pillarSensor];
    const PassageTofSamples &wall = passage.samples[wallSensor];
    const float pillarInsetMm = tofWheelEnvelopeInset(pillarSensor);
    const float wallInsetMm = tofWheelEnvelopeInset(wallSensor);
    ObstacleClearanceSample planned;
    obstacle_path_get_planned_clearance(seatId, planned);
    Serial.print("[PILLAR PASS] seat="); Serial.print(seatId);
    Serial.print(" lap="); Serial.println(passage.lapIndex + 1);
    printGeometryClearance("PLAN", seatId, planned);
    printGeometryClearance(
        "ODOM", seatId, passage.odometryMinimum,
        passage.odometrySamples);
    Serial.print("[PILLAR TOF] seat="); Serial.print(seatId);
    Serial.print(" color="); Serial.print(seat.red ? "RED" : "GREEN");
    Serial.print(" sensor=");
    Serial.print(pillarSensor == TOF_LEFT ? "L" : "R");
    Serial.print(" window_mm=-");
    Serial.print(OBSTACLE_TOF_PASSAGE_BEFORE_MM, 0);
    Serial.print("..+");
    Serial.print(OBSTACLE_TOF_PASSAGE_AFTER_MM, 0);
    Serial.print(" complete="); Serial.print(completeWindow ? "yes" : "no");
    Serial.print(" fresh="); Serial.print(pillar.freshSamples);
    Serial.print(" valid_raw="); Serial.print(pillar.validRawSamples);
    Serial.print(" valid_filtered=");
    Serial.print(pillar.validFilteredSamples);
    Serial.print(" inset_mm="); Serial.print(pillarInsetMm, 1);
    printPassageRange(
        "raw_min_mm", "raw_clearance_est_mm", pillar.minimumRawMm,
        pillar.validRawSamples, pillarInsetMm);
    printPassageRange(
        "filtered_min_mm", "filtered_clearance_est_mm",
        pillar.minimumFilteredMm,
        pillar.validFilteredSamples, pillarInsetMm);
    Serial.print(" wall_sensor=");
    Serial.print(wallSensor == TOF_LEFT ? "L" : "R");
    Serial.print(" wall_fresh="); Serial.print(wall.freshSamples);
    printPassageRange(
        "wall_raw_min_mm", "wall_raw_clearance_est_mm", wall.minimumRawMm,
        wall.validRawSamples, wallInsetMm);
    printPassageRange(
        "wall_filtered_min_mm", "wall_filtered_clearance_est_mm",
        wall.minimumFilteredMm, wall.validFilteredSamples, wallInsetMm);
    Serial.println(" estimate=range-minus-max-steered-wheel-inset");
    return true;
}

void advancePillarPassage(PillarTofPassage &passage)
{
    if (passage.lapIndex + 1 >= requestedLapTarget)
        return;
    ++passage.lapIndex;
    passage.seatPathDistanceMm += obstacle_path_loop_length_mm();
    resetPassageMeasurements(passage);
}

void updatePillarTofPassages(bool flushIncomplete = false)
{
    const float travelledMm = obstacle_path_travel_distance_mm();
    const PositionEstimate pose = get_position_struct();
    for (uint8_t seatId = 0; seatId < obstacle_path_seat_count(); ++seatId)
    {
        PillarTofPassage &passage = pillarPassages[seatId];
        if (!passage.initialized || passage.reported)
            continue;
        const float relativeMm = travelledMm - passage.seatPathDistanceMm;
        if (relativeMm >= -OBSTACLE_TOF_PASSAGE_BEFORE_MM &&
            relativeMm <= OBSTACLE_TOF_PASSAGE_AFTER_MM)
        {
            ObstacleClearanceSample odometry;
            if (obstacle_path_sample_pose_clearance(
                    seatId, pose.x_mm, pose.y_mm, pose.heading_deg,
                    odometry))
                addOdometryClearanceSample(passage, odometry);
            for (uint8_t side = 0; side < TOF_SIDE_COUNT; ++side)
            {
                TofDiagnosticSnapshot snapshot;
                if (get_tof_diagnostic_snapshot(
                        static_cast<TofSensor>(side), snapshot))
                    addFreshPassageSample(passage.samples[side], snapshot);
            }
        }
        if (relativeMm > OBSTACLE_TOF_PASSAGE_AFTER_MM)
        {
            if (reportPillarTofPassage(seatId, passage, true))
                advancePillarPassage(passage);
        }
        else if (flushIncomplete)
            reportPillarTofPassage(seatId, passage, false);
    }
}

bool pillarTofPassagePreflight()
{
    if (fabsf(DRIVE_WHEEL_DIAMETER_MM * 0.5f - 21.6f) > 0.01f ||
        fabsf(OBSTACLE_MAX_WHEEL_HALF_WIDTH_MM - 70.0f) > 0.01f ||
        fabsf(OBSTACLE_TOF_LEFT_TO_WHEEL_ENVELOPE_MM - 35.0f) > 0.01f ||
        fabsf(OBSTACLE_TOF_RIGHT_TO_WHEEL_ENVELOPE_MM - 35.0f) > 0.01f ||
        OBSTACLE_TOF_PASSAGE_BEFORE_MM <= 0.0f ||
        OBSTACLE_TOF_PASSAGE_AFTER_MM <= 0.0f)
        return false;

    PassageTofSamples first;
    TofDiagnosticSnapshot sample = {};
    sample.sequence = 10;
    sample.selected_raw_distance_mm = 140.0f;
    sample.filtered_distance_mm = 160.0f;
    if (!addFreshPassageSample(first, sample) ||
        addFreshPassageSample(first, sample))
        return false;
    sample.sequence = 11;
    sample.selected_raw_distance_mm = 90.0f;
    sample.filtered_distance_mm = 110.0f;
    if (!addFreshPassageSample(first, sample) ||
        first.freshSamples != 2 || first.validRawSamples != 2 ||
        first.validFilteredSamples != 2 ||
        fabsf(first.minimumRawMm - 90.0f) > 0.01f ||
        fabsf(first.minimumFilteredMm - 110.0f) > 0.01f)
        return false;

    PassageTofSamples overlapping;
    if (!addFreshPassageSample(overlapping, sample) ||
        overlapping.freshSamples != 1 ||
        fabsf(overlapping.minimumRawMm - 90.0f) > 0.01f)
        return false;
    sample.sequence = 12;
    sample.selected_raw_distance_mm = -1.0f;
    sample.filtered_distance_mm = TOF_OUT_OF_RANGE_MM;
    if (!addFreshPassageSample(first, sample) ||
        first.freshSamples != 3 || first.validRawSamples != 2 ||
        first.validFilteredSamples != 2)
        return false;
    return true;
}

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
    const uint32_t telemetryIntervalMs = requestedLapTarget == 3
        ? OBSTACLE_LIVE_TEST_THREE_LAP_TELEMETRY_MS
        : OBSTACLE_LIVE_TEST_TELEMETRY_MS;
    if (now - lastTelemetryMs < telemetryIntervalMs)
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
        Serial.print(" evidence=");
        Serial.print((discovery.clearEvidenceMask & 0x01) != 0 ? "R" : "-");
        Serial.print((discovery.clearEvidenceMask & 0x02) != 0 ? "L" : "-");
        Serial.print(" seat_geom=R");
        Serial.print(discovery.seatBearingDeg[0], 1);
        Serial.print("/");
        Serial.print(discovery.seatRangeMm[0], 0);
        Serial.print(",L");
        Serial.print(discovery.seatBearingDeg[1], 1);
        Serial.print("/");
        Serial.print(discovery.seatRangeMm[1], 0);
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
    Serial.print("/"); Serial.print(get_tof_distance(TOF_RIGHT), 0);
    const ObstacleTofCorrectionResult tofCorrection =
        obstacle_path_last_tof_correction();
    if (tofCorrection.leftResidualGated ||
        tofCorrection.rightResidualGated)
    {
        Serial.print(" tof_residual_gate=");
        Serial.print(tofCorrection.leftResidualGated ? "L" : "-");
        Serial.print(tofCorrection.rightResidualGated ? "R" : "-");
        Serial.print(" residual=");
        Serial.print(tofCorrection.leftResidualMm, 0);
        Serial.print("/");
        Serial.print(tofCorrection.rightResidualMm, 0);
    }
    Serial.println();
    resetDriveTelemetryWindow();
}

void finishAndReport()
{
    updatePillarTofPassages(true);
    stop(false);
    set_steering(0);
    resultPassed = failureReason == nullptr &&
        obstacle_path_lap() == requestedLapTarget &&
        maximumCrossTrackError <= OBSTACLE_PATH_TEST_PASS_CROSS_TRACK_MM;

    Serial.println("\n===== LIVE OBSTACLE PATH TEST RESULT =====");
    Serial.print("Result: "); Serial.println(resultPassed ? "PASS" : "FAIL");
    Serial.print("Direction: ");
    Serial.println(requestedTurnSign > 0 ? "LEFT/CCW" : "RIGHT/CW");
    Serial.print("Completed laps: "); Serial.println(obstacle_path_lap());
    Serial.print("Obstacle injections: "); Serial.println(obstacle_path_injection_count());
    Serial.println("Injection acceptance: layout-specific (informational here)");
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

void obstacle_live_test_set_lap_target(uint8_t lap_target)
{
    requestedLapTarget = lap_target == 3 ? 3 : 1;
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
        requestedLapTarget,
        OBSTACLE_PATH_TEST_MAX_SPEED_MM_S);
    resetPillarTofPassages();

    Serial.println("\n===== LIVE PURE PURSUIT TEST =====");
    Serial.println("Camera discovery/path injection: ENABLED");
    Serial.println("ToF pose correction: ENABLED (fresh frames only)");
    Serial.println("Parking exit: BYPASSED");
    Serial.print("Direction: ");
    Serial.println(requestedTurnSign > 0 ? "LEFT/CCW" : "RIGHT/CW");
    Serial.print("Lap target: ");
    Serial.println(requestedLapTarget);
    Serial.print("Telemetry interval (ms): ");
    Serial.println(requestedLapTarget == 3
        ? OBSTACLE_LIVE_TEST_THREE_LAP_TELEMETRY_MS
        : OBSTACLE_LIVE_TEST_TELEMETRY_MS);
    Serial.print("Speed cap mm/s: ");
    Serial.println(OBSTACLE_PATH_TEST_MAX_SPEED_MM_S, 0);
    Serial.println(
        "Raw ToF stop: DISABLED (side ToF cannot distinguish pillar from wall)");

    if (!obstacle_path_geometry_valid() || !pillarTofPassagePreflight())
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
    updatePillarTofPassages();
    const float crossTrack = obstacle_path_cross_track_error_mm();
    const float headingError = fabsf(obstacle_path_heading_error_deg());
    maximumCrossTrackError = fmaxf(maximumCrossTrackError, crossTrack);
    maximumHeadingError = fmaxf(maximumHeadingError, headingError);
    printTelemetry();

    if (obstacle_path_perception_blocked())
        beginBraking("ABORT: unresolved station reached perception limit");
    else if (crossTrack > OBSTACLE_PATH_TEST_ABORT_CROSS_TRACK_MM)
        beginBraking("ABORT: cross-track error exceeded limit");
    else if (millis() - startTimeMs >
             OBSTACLE_LIVE_TEST_TIMEOUT_MS * requestedLapTarget)
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
