#include "final_parking.h"

#include <math.h>

#include "config.h"
#include "logger.h"
#include "motor_control.h"
#include "obstacle_path.h"
#include "position_estimator.h"
#include "sensors.h"

#define Serial robot_logger

namespace
{
enum FinalParkingState : uint8_t
{
    FP_IDLE,
    FP_APPROACH_LANE,
    FP_APPROACH_OUTER,
    FP_APPROACH_BRAKE,
    FP_SCAN_SETTLE,
    FP_SCAN_DRIVE,
    FP_SCAN_BRAKE,
    FP_CAPTURE_SETTLE,
    FP_CAPTURE_DRIVE,
    FP_CAPTURE_BRAKE,
    FP_SEGMENT_SETTLE,
    FP_SEGMENT_DRIVE,
    FP_SEGMENT_BRAKE,
    FP_VERIFY,
    FP_TEST_HOLD,
    FP_HOLD,
    FP_ABORT
};

struct ParkingSegment
{
    int8_t direction;
    int8_t steering;
    float distanceMm;
};

// Search coordinates use +X in the parked heading and +Y away from the wall.
// CW is a geometric reflection, so its physical steering sign is inverted.
constexpr ParkingSegment PARKING_SEGMENTS[
    OBSTACLE_FINAL_PARKING_SEGMENT_COUNT] = {
    {-1, 0, 20.0f},
    {-1, +50, 120.0f},
    {-1, 0, 80.0f},
    {-1, -50, 65.0f},
    {+1, +50, 20.0f},
    {-1, -50, 35.0f},
    {+1, 0, 25.0f}};

constexpr float SEGMENT_LOCAL_X_MM[
    OBSTACLE_FINAL_PARKING_SEGMENT_COUNT] = {
    250.26f, 153.07f, 116.85f, 72.36f, 90.65f, 56.25f, 81.25f};
constexpr float SEGMENT_LOCAL_Y_MM[
    OBSTACLE_FINAL_PARKING_SEGMENT_COUNT] = {
    274.60f, 214.95f, 143.62f, 97.56f, 105.57f, 100.00f, 100.00f};
constexpr float SEGMENT_LOCAL_HEADING_DEG[
    OBSTACLE_FINAL_PARKING_SEGMENT_COUNT] = {
    0.00f, 63.08f, 63.08f, 28.91f, 18.40f, 0.00f, 0.00f};

FinalParkingState state = FP_IDLE;
int8_t routeTurnSign = 1;
TofSensor outerSensor = TOF_RIGHT;
float stateStartDistance = 0.0f;
float runStartDistance = 0.0f;
uint32_t stateStartMs = 0;
uint8_t segmentIndex = 0;
uint32_t tofSequence = 0;
uint8_t scanPhase = 0;
uint8_t classFrames = 0;
bool scanArmed = false;

PositionEstimate lastFixedMarkerPose{};
float lastFixedMarkerRange = 0.0f;
PositionEstimate firstMovingMarkerPose{};
float firstMovingMarkerRange = 0.0f;
bool movingCandidateStored = false;
PositionEstimate latestWallPose{};
float latestWallRange = 0.0f;
float measuredFixedInsideX = 0.0f;
float measuredMovingInsideX = 0.0f;
float measuredGapMm = 0.0f;
float captureTargetX = 0.0f;
int8_t captureFieldMotionSign = 1;
bool completed = false;
bool aborted = false;
bool practiceMode = false;

float clampFloat(float value, float minimum, float maximum)
{
    return value < minimum ? minimum : (value > maximum ? maximum : value);
}

float wrap180(float angle)
{
    while (angle > 180.0f)
        angle -= 360.0f;
    while (angle <= -180.0f)
        angle += 360.0f;
    return angle;
}

float distanceSince(float start)
{
    return fabsf(get_distance() - start);
}

float baseHeadingDeg()
{
    return routeTurnSign > 0 ? 0.0f : 180.0f;
}

int motorDirectionForFieldMotion(int8_t fieldMotionSign)
{
    return fieldMotionSign * routeTurnSign;
}

float movingInsideFaceX()
{
    return OBSTACLE_PARKING_FIXED_INNER_FACE_X_MM -
           OBSTACLE_FINAL_PARKING_GAP_MM;
}

void localToField(
    float localX,
    float localY,
    float localHeadingDeg,
    float &fieldX,
    float &fieldY,
    float &fieldHeadingDeg)
{
    fieldX = routeTurnSign > 0
        ? movingInsideFaceX() + localX
        : OBSTACLE_PARKING_FIXED_INNER_FACE_X_MM - localX;
    fieldY = OBSTACLE_SOUTH_OUTER_WALL_Y_MM + localY;
    fieldHeadingDeg = routeTurnSign > 0
        ? localHeadingDeg
        : 180.0f - localHeadingDeg;
}

struct BeamFootprint
{
    float minimumX;
    float maximumX;
};

BeamFootprint beamFootprint(
    TofSensor sensor,
    float rangeMm,
    const PositionEstimate &pose)
{
    const float localX = sensor == TOF_LEFT
        ? OBSTACLE_TOF_LEFT_LOCAL_X_MM
        : OBSTACLE_TOF_RIGHT_LOCAL_X_MM;
    const float localY = sensor == TOF_LEFT
        ? OBSTACLE_TOF_LEFT_LOCAL_Y_MM
        : OBSTACLE_TOF_RIGHT_LOCAL_Y_MM;
    const float heading = pose.heading_deg * PI / 180.0f;
    const float sensorX = pose.x_mm + cosf(heading) * localX -
                          sinf(heading) * localY;
    const float rayHeading = heading +
        (sensor == TOF_LEFT ? 0.5f * PI : -0.5f * PI);
    const float halfFov = 0.5f *
        OBSTACLE_PARKING_EXIT_TOF_DETECTION_FOV_DEG * PI / 180.0f;
    const float edge1 = sensorX + rangeMm * cosf(rayHeading - halfFov);
    const float edge2 = sensorX + rangeMm * cosf(rayHeading + halfFov);
    return {fminf(edge1, edge2), fmaxf(edge1, edge2)};
}

float expectedOuterWallRange(
    TofSensor sensor,
    const PositionEstimate &pose)
{
    const float localX = sensor == TOF_LEFT
        ? OBSTACLE_TOF_LEFT_LOCAL_X_MM
        : OBSTACLE_TOF_RIGHT_LOCAL_X_MM;
    const float localY = sensor == TOF_LEFT
        ? OBSTACLE_TOF_LEFT_LOCAL_Y_MM
        : OBSTACLE_TOF_RIGHT_LOCAL_Y_MM;
    const float heading = pose.heading_deg * PI / 180.0f;
    const float sensorY = pose.y_mm + sinf(heading) * localX +
                          cosf(heading) * localY;
    const float rayHeading = heading +
        (sensor == TOF_LEFT ? 0.5f * PI : -0.5f * PI);
    const float rayY = sinf(rayHeading);
    if (fabsf(rayY) < 0.1f)
        return -1.0f;
    return (OBSTACLE_SOUTH_OUTER_WALL_Y_MM - sensorY) / rayY;
}

bool connectorClearanceSafe(const PositionEstimate &pose)
{
    float minimumPillar = 1.0e9f;
    float minimumWall = 1.0e9f;
    bool sampled = false;
    for (uint8_t seatId = 0; seatId < obstacle_path_seat_count(); ++seatId)
    {
        ObstacleSeatInfo seat;
        if (!obstacle_path_get_seat(seatId, seat) || !seat.confirmed)
            continue;
        ObstacleClearanceSample sample;
        if (!obstacle_path_sample_pose_clearance(
                seatId,
                pose.x_mm,
                pose.y_mm,
                pose.heading_deg,
                sample) || !sample.valid)
            continue;
        sampled = true;
        minimumPillar = fminf(minimumPillar, sample.pillarMm);
        minimumWall = fminf(minimumWall, sample.wallMm);
    }
    // The three-lap path cannot complete until every station is resolved.
    return sampled && minimumPillar >= 10.0f && minimumWall >= 5.0f;
}

void abortParking(const char *reason)
{
    stop(false);
    set_steering(0);
    state = FP_ABORT;
    aborted = true;
    Serial.print("[FINAL PARK ABORT] ");
    Serial.println(reason);
    robot_logger.write_to_usb();
}

bool checkMotionHealth(const PositionEstimate &pose)
{
    if (!gyro_is_healthy())
    {
        abortParking("gyro_unhealthy");
        return false;
    }
    if (!isfinite(pose.x_mm) || !isfinite(pose.y_mm) ||
        !isfinite(pose.heading_deg))
    {
        abortParking("invalid_pose");
        return false;
    }
    if (!practiceMode && !connectorClearanceSafe(pose))
    {
        abortParking("connector_clearance_gate");
        return false;
    }
    return true;
}

void driveFieldLine(
    int8_t fieldMotionSign,
    float targetY,
    int speed)
{
    const PositionEstimate pose = get_position_struct();
    const float lateralError = targetY - pose.y_mm;
    const float lateralAngle = fieldMotionSign * atan2f(
        lateralError,
        OBSTACLE_FINAL_PARKING_LINE_LOOKAHEAD_MM) * 180.0f / PI;
    const float desiredHeading = baseHeadingDeg() + clampFloat(
        lateralAngle,
        -OBSTACLE_FINAL_PARKING_LINE_MAX_ANGLE_DEG,
        OBSTACLE_FINAL_PARKING_LINE_MAX_ANGLE_DEG);
    const float headingError = wrap180(desiredHeading - pose.heading_deg);
    const int motorDirection = motorDirectionForFieldMotion(fieldMotionSign);
    const float steering = clampFloat(
        -motorDirection * headingError *
            OBSTACLE_FINAL_PARKING_LINE_HEADING_KP,
        -OBSTACLE_FINAL_PARKING_LINE_MAX_STEERING,
        OBSTACLE_FINAL_PARKING_LINE_MAX_STEERING);
    set_speed(motorDirection * speed);
    set_steering(static_cast<int>(steering));
}

bool linePoseReady(const PositionEstimate &pose, float targetY)
{
    return fabsf(pose.y_mm - targetY) <=
               OBSTACLE_FINAL_PARKING_LINE_Y_TOLERANCE_MM &&
           fabsf(wrap180(baseHeadingDeg() - pose.heading_deg)) <=
               OBSTACLE_FINAL_PARKING_CAPTURE_HEADING_TOLERANCE_DEG;
}

void resetScanClassFrames()
{
    classFrames = 0;
    movingCandidateStored = false;
}

void processScanFrame(const TofDiagnosticSnapshot &snapshot)
{
    const PositionEstimate pose = get_position_struct();
    const float raw = snapshot.selected_raw_distance_mm;
    const float expectedWall = expectedOuterWallRange(outerSensor, pose);
    const bool marker = raw > 0.0f &&
        raw <= OBSTACLE_FINAL_PARKING_MARKER_MAX_MM;
    const bool wall = raw >= OBSTACLE_FINAL_PARKING_WALL_MIN_MM &&
        raw <= OBSTACLE_FINAL_PARKING_WALL_MAX_MM &&
        expectedWall > 0.0f &&
        fabsf(raw - expectedWall) <=
            OBSTACLE_FINAL_PARKING_WALL_RESIDUAL_MM;

    if (scanPhase == 0)
    {
        classFrames = wall ? classFrames + 1 : 0;
        if (classFrames >= OBSTACLE_FINAL_PARKING_CLASS_CONFIRM_FRAMES)
        {
            scanPhase = 1;
            classFrames = 0;
        }
        return;
    }

    if (scanPhase == 1)
    {
        classFrames = marker ? classFrames + 1 : 0;
        if (classFrames >= OBSTACLE_FINAL_PARKING_CLASS_CONFIRM_FRAMES)
        {
            lastFixedMarkerPose = pose;
            lastFixedMarkerRange = raw;
            scanPhase = 2;
            classFrames = 0;
        }
        return;
    }

    if (scanPhase == 2)
    {
        if (marker)
        {
            lastFixedMarkerPose = pose;
            lastFixedMarkerRange = raw;
            classFrames = 0;
        }
        else if (wall)
        {
            latestWallPose = pose;
            latestWallRange = raw;
            ++classFrames;
            if (classFrames >= OBSTACLE_FINAL_PARKING_CLASS_CONFIRM_FRAMES)
            {
                measuredFixedInsideX = beamFootprint(
                    outerSensor,
                    lastFixedMarkerRange,
                    lastFixedMarkerPose).maximumX;
                scanPhase = 3;
                resetScanClassFrames();
            }
        }
        else
            classFrames = 0;
        return;
    }

    if (scanPhase == 3)
    {
        if (marker)
        {
            if (!movingCandidateStored)
            {
                firstMovingMarkerPose = pose;
                firstMovingMarkerRange = raw;
                movingCandidateStored = true;
            }
            ++classFrames;
            if (classFrames >= OBSTACLE_FINAL_PARKING_CLASS_CONFIRM_FRAMES)
            {
                measuredMovingInsideX = beamFootprint(
                    outerSensor,
                    firstMovingMarkerRange,
                    firstMovingMarkerPose).minimumX;
                scanPhase = 4;
                classFrames = 0;
            }
        }
        else
            resetScanClassFrames();
        return;
    }

    if (scanPhase == 4)
    {
        if (marker)
            classFrames = 0;
        else if (wall)
        {
            latestWallPose = pose;
            latestWallRange = raw;
            ++classFrames;
            if (classFrames >= OBSTACLE_FINAL_PARKING_CLASS_CONFIRM_FRAMES)
                scanPhase = 5;
        }
        else
            classFrames = 0;
    }
}

bool applyScanLocalization()
{
    measuredGapMm = measuredFixedInsideX - measuredMovingInsideX;
    const float xCorrection =
        OBSTACLE_PARKING_FIXED_INNER_FACE_X_MM - measuredFixedInsideX;
    const float expectedWall = expectedOuterWallRange(
        outerSensor, latestWallPose);
    const float yCorrection = latestWallRange - expectedWall;
    const bool valid = scanPhase == 5 &&
        fabsf(measuredGapMm - OBSTACLE_FINAL_PARKING_GAP_MM) <=
            OBSTACLE_FINAL_PARKING_GAP_TOLERANCE_MM &&
        fabsf(xCorrection) <=
            OBSTACLE_FINAL_PARKING_MAX_POSE_CORRECTION_MM &&
        expectedWall > 0.0f &&
        fabsf(yCorrection) <=
            OBSTACLE_FINAL_PARKING_MAX_POSE_CORRECTION_MM;

    Serial.print("[FINAL PARK SCAN] fixed/moving/gap_mm=");
    Serial.print(measuredFixedInsideX, 1);
    Serial.print("/");
    Serial.print(measuredMovingInsideX, 1);
    Serial.print("/");
    Serial.print(measuredGapMm, 1);
    Serial.print(" correction_x_y_mm=");
    Serial.print(xCorrection, 1);
    Serial.print("/");
    Serial.print(yCorrection, 1);
    Serial.print(" valid=");
    Serial.println(valid ? "yes" : "no");

    if (!valid)
        return false;

    position_apply_xy_correction(xCorrection, yCorrection);
    captureTargetX = routeTurnSign > 0
        ? movingInsideFaceX() +
              OBSTACLE_FINAL_PARKING_CAPTURE_LOCAL_X_MM
        : OBSTACLE_PARKING_FIXED_INNER_FACE_X_MM -
              OBSTACLE_FINAL_PARKING_CAPTURE_LOCAL_X_MM;
    captureFieldMotionSign =
        captureTargetX >= get_position_struct().x_mm ? 1 : -1;
    return true;
}

bool endpointWithinGate(uint8_t index)
{
    float expectedX = 0.0f;
    float expectedY = 0.0f;
    float expectedHeading = 0.0f;
    localToField(
        SEGMENT_LOCAL_X_MM[index],
        SEGMENT_LOCAL_Y_MM[index],
        SEGMENT_LOCAL_HEADING_DEG[index],
        expectedX,
        expectedY,
        expectedHeading);
    const PositionEstimate pose = get_position_struct();
    const float positionError = hypotf(
        pose.x_mm - expectedX, pose.y_mm - expectedY);
    const float headingError = fabsf(wrap180(
        expectedHeading - pose.heading_deg));
    Serial.print("[FINAL PARK SEGMENT RESULT] index=");
    Serial.print(index + 1);
    Serial.print(" pose_error_mm=");
    Serial.print(positionError, 1);
    Serial.print(" heading_error_deg=");
    Serial.println(headingError, 1);
    return positionError <= 20.0f && headingError <= 6.0f;
}

bool finalFootprintContained()
{
    const PositionEstimate pose = get_position_struct();
    const float heading = pose.heading_deg * PI / 180.0f;
    const float c = cosf(heading);
    const float s = sinf(heading);
    const float localX[2] = {
        -OBSTACLE_FINAL_PARKING_REAR_OVERHANG_MM,
        OBSTACLE_FINAL_PARKING_FRONT_OVERHANG_MM};
    const float halfWidth =
        OBSTACLE_FINAL_PARKING_STRAIGHT_WIDTH_MM * 0.5f;
    const float localY[2] = {-halfWidth, halfWidth};
    const float margin = OBSTACLE_FINAL_PARKING_FINAL_BOUNDARY_MARGIN_MM;
    for (uint8_t ix = 0; ix < 2; ++ix)
    {
        for (uint8_t iy = 0; iy < 2; ++iy)
        {
            const float x = pose.x_mm + c * localX[ix] - s * localY[iy];
            const float y = pose.y_mm + s * localX[ix] + c * localY[iy];
            if (x <= movingInsideFaceX() + margin ||
                x >= OBSTACLE_PARKING_FIXED_INNER_FACE_X_MM - margin ||
                y <= OBSTACLE_SOUTH_OUTER_WALL_Y_MM + margin ||
                y >= OBSTACLE_PARKING_OPEN_END_FIELD_Y_MM - margin)
                return false;
        }
    }
    return fabsf(wrap180(baseHeadingDeg() - pose.heading_deg)) <=
           OBSTACLE_FINAL_PARKING_FINAL_HEADING_TOLERANCE_DEG;
}

void enterTestHold(const char *reason)
{
    stop(false);
    set_steering(0);
    state = FP_TEST_HOLD;
    Serial.print("[FINAL PARK TEST HOLD] ");
    Serial.println(reason);
    robot_logger.write_to_usb();
}
} // namespace

void final_parking_reset()
{
    state = FP_IDLE;
    routeTurnSign = 1;
    outerSensor = TOF_RIGHT;
    stateStartDistance = get_distance();
    runStartDistance = get_distance();
    stateStartMs = 0;
    segmentIndex = 0;
    tofSequence = 0;
    scanPhase = 0;
    classFrames = 0;
    scanArmed = false;
    movingCandidateStored = false;
    latestWallRange = 0.0f;
    measuredFixedInsideX = 0.0f;
    measuredMovingInsideX = 0.0f;
    measuredGapMm = 0.0f;
    captureTargetX = 0.0f;
    captureFieldMotionSign = 1;
    completed = false;
    aborted = false;
    practiceMode = false;
}

void final_parking_start_practice(int8_t turn_sign)
{
    final_parking_reset();
    practiceMode = true;
    routeTurnSign = turn_sign < 0 ? -1 : 1;
    outerSensor = routeTurnSign > 0 ? TOF_RIGHT : TOF_LEFT;
    const float fieldX = routeTurnSign > 0
        ? OBSTACLE_FINAL_PARKING_PRACTICE_START_X_MM
        : -OBSTACLE_FINAL_PARKING_PRACTICE_START_X_MM;
    const float fieldY = OBSTACLE_FINAL_PARKING_PRACTICE_START_Y_MM;
    const float fieldHeading = routeTurnSign > 0 ? 0.0f : 180.0f;
    position_reset(fieldX, fieldY, fieldHeading);
    runStartDistance = get_distance();
    stateStartDistance = runStartDistance;
    stateStartMs = millis();
    state = FP_APPROACH_LANE;
    stop(false);
    set_steering(0);
    Serial.print("[FINAL PARK PRACTICE] turn=");
    Serial.print(routeTurnSign > 0 ? "CCW" : "CW");
    Serial.print(" start_x_y_heading=");
    Serial.print(fieldX, 2);
    Serial.print("/");
    Serial.print(fieldY, 2);
    Serial.print("/");
    Serial.print(fieldHeading, 1);
    Serial.print(" full_approach_scan=yes segment_limit=");
    Serial.println(OBSTACLE_FINAL_PARKING_TEST_SEGMENT_LIMIT);
}

bool final_parking_update(int8_t turn_sign)
{
    if (!OBSTACLE_FINAL_PARKING_ENABLED)
        return false;

    if (state == FP_TEST_HOLD || state == FP_ABORT)
    {
        if (dc_state != DC_DISABLED)
            stop(false);
        return true;
    }
    if (state == FP_HOLD)
    {
        if (dc_state != DC_HOLDING)
            stop(true);
        return true;
    }

    PositionEstimate pose = get_position_struct();
    if (state == FP_IDLE)
    {
        routeTurnSign = turn_sign < 0 ? -1 : 1;
        outerSensor = routeTurnSign > 0 ? TOF_RIGHT : TOF_LEFT;
        runStartDistance = get_distance();
        stateStartDistance = runStartDistance;
        stateStartMs = millis();
        state = FP_APPROACH_LANE;
        Serial.print("[FINAL PARK] Start turn=");
        Serial.print(routeTurnSign > 0 ? "CCW" : "CW");
        Serial.print(" target_local_x_y=");
        Serial.print(OBSTACLE_FINAL_PARKING_TARGET_LOCAL_X_MM, 2);
        Serial.print("/");
        Serial.println(OBSTACLE_FINAL_PARKING_TARGET_LOCAL_Y_MM, 2);
    }

    pose = get_position_struct();
    if (state >= FP_APPROACH_LANE && state <= FP_CAPTURE_DRIVE &&
        !checkMotionHealth(pose))
        return true;

    if (state == FP_APPROACH_LANE)
    {
        driveFieldLine(+1, OBSTACLE_FINAL_PARKING_APPROACH_LANE_Y_MM,
                       OBSTACLE_FINAL_PARKING_APPROACH_SPEED);
        if (pose.x_mm >= OBSTACLE_FINAL_PARKING_OUTER_SHIFT_X_MM)
        {
            state = FP_APPROACH_OUTER;
            Serial.println("[FINAL PARK] Body clear of fixed marker; outer shift");
        }
        else if (distanceSince(runStartDistance) >=
                     OBSTACLE_FINAL_PARKING_APPROACH_MAX_MM ||
                 millis() - stateStartMs >=
                     OBSTACLE_FINAL_PARKING_APPROACH_TIMEOUT_MS)
            abortParking("approach_lane_distance");
        return true;
    }

    if (state == FP_APPROACH_OUTER)
    {
        driveFieldLine(+1, OBSTACLE_FINAL_PARKING_SCAN_Y_MM,
                       OBSTACLE_FINAL_PARKING_APPROACH_SPEED);
        if (pose.x_mm >= OBSTACLE_FINAL_PARKING_SCAN_START_X_MM &&
            linePoseReady(pose, OBSTACLE_FINAL_PARKING_SCAN_Y_MM))
        {
            stop(true);
            stateStartMs = millis();
            state = FP_APPROACH_BRAKE;
        }
        else if (distanceSince(runStartDistance) >=
                     OBSTACLE_FINAL_PARKING_APPROACH_MAX_MM ||
                 pose.x_mm > OBSTACLE_FINAL_PARKING_SCAN_START_X_MM + 120.0f ||
                 millis() - stateStartMs >=
                     OBSTACLE_FINAL_PARKING_APPROACH_TIMEOUT_MS)
            abortParking("approach_outer_gate");
        return true;
    }

    if (state == FP_APPROACH_BRAKE)
    {
        if (millis() - stateStartMs <
            OBSTACLE_FINAL_PARKING_HOLD_BRAKE_MS)
            return true;
        stop(false);
        servo_disabled = false;
        set_steering(0);
        steer(0);
        stateStartMs = millis();
        state = FP_SCAN_SETTLE;
        return true;
    }

    if (state == FP_SCAN_SETTLE)
    {
        if (millis() - stateStartMs <
            OBSTACLE_FINAL_PARKING_STEER_SETTLE_MS)
            return true;
        TofDiagnosticSnapshot snapshot;
        tofSequence = get_tof_diagnostic_snapshot(outerSensor, snapshot)
            ? snapshot.sequence
            : 0;
        scanPhase = 0;
        classFrames = 0;
        scanArmed = false;
        movingCandidateStored = false;
        stateStartDistance = get_distance();
        stateStartMs = millis();
        state = FP_SCAN_DRIVE;
        Serial.println("[FINAL PARK] Reverse field scan started");
        return true;
    }

    if (state == FP_SCAN_DRIVE)
    {
        driveFieldLine(-1, OBSTACLE_FINAL_PARKING_SCAN_Y_MM,
                       OBSTACLE_FINAL_PARKING_SCAN_SPEED);
        if (!scanArmed && pose.x_mm <=
            OBSTACLE_FINAL_PARKING_SCAN_ARM_X_MM)
        {
            scanArmed = true;
            scanPhase = 0;
            classFrames = 0;
            Serial.println("[FINAL PARK] Dual-marker scan armed");
        }
        TofDiagnosticSnapshot snapshot;
        if (scanArmed &&
            get_tof_diagnostic_snapshot(outerSensor, snapshot) &&
            snapshot.sequence != tofSequence)
        {
            tofSequence = snapshot.sequence;
            processScanFrame(snapshot);
        }
        if (scanPhase == 5)
        {
            stop(true);
            stateStartMs = millis();
            state = FP_SCAN_BRAKE;
        }
        else if (pose.x_mm <= OBSTACLE_FINAL_PARKING_SCAN_END_X_MM ||
                 distanceSince(stateStartDistance) >=
                     OBSTACLE_FINAL_PARKING_SCAN_MAX_MM ||
                 millis() - stateStartMs >=
                     OBSTACLE_FINAL_PARKING_SCAN_TIMEOUT_MS)
            abortParking("dual_marker_scan_incomplete");
        return true;
    }

    if (state == FP_SCAN_BRAKE)
    {
        if (millis() - stateStartMs <
            OBSTACLE_FINAL_PARKING_HOLD_BRAKE_MS)
            return true;
        stop(false);
        if (!applyScanLocalization())
        {
            abortParking("dual_marker_geometry");
            return true;
        }
        servo_disabled = false;
        set_steering(0);
        steer(0);
        stateStartMs = millis();
        state = FP_CAPTURE_SETTLE;
        return true;
    }

    if (state == FP_CAPTURE_SETTLE)
    {
        if (millis() - stateStartMs <
            OBSTACLE_FINAL_PARKING_STEER_SETTLE_MS)
            return true;
        stateStartDistance = get_distance();
        stateStartMs = millis();
        state = FP_CAPTURE_DRIVE;
        Serial.print("[FINAL PARK] Capture target x=");
        Serial.println(captureTargetX, 2);
        return true;
    }

    if (state == FP_CAPTURE_DRIVE)
    {
        driveFieldLine(captureFieldMotionSign,
                       OBSTACLE_FINAL_PARKING_SCAN_Y_MM,
                       OBSTACLE_FINAL_PARKING_CAPTURE_SPEED);
        const bool targetReached = captureFieldMotionSign > 0
            ? pose.x_mm >= captureTargetX - 1.5f
            : pose.x_mm <= captureTargetX + 1.5f;
        if (targetReached)
        {
            stop(true);
            stateStartMs = millis();
            state = FP_CAPTURE_BRAKE;
        }
        else if (distanceSince(stateStartDistance) >=
                     OBSTACLE_FINAL_PARKING_CAPTURE_MAX_MM ||
                 millis() - stateStartMs >=
                     OBSTACLE_FINAL_PARKING_CAPTURE_TIMEOUT_MS)
            abortParking("capture_distance");
        return true;
    }

    if (state == FP_CAPTURE_BRAKE)
    {
        if (millis() - stateStartMs <
            OBSTACLE_FINAL_PARKING_HOLD_BRAKE_MS)
            return true;
        stop(false);
        pose = get_position_struct();
        if (fabsf(pose.x_mm - captureTargetX) >
                OBSTACLE_FINAL_PARKING_CAPTURE_X_TOLERANCE_MM ||
            !linePoseReady(pose, OBSTACLE_FINAL_PARKING_SCAN_Y_MM))
        {
            abortParking("capture_pose_gate");
            return true;
        }
        Serial.print("[FINAL PARK CAPTURE] x_y_heading=");
        Serial.print(pose.x_mm, 1);
        Serial.print("/");
        Serial.print(pose.y_mm, 1);
        Serial.print("/");
        Serial.println(pose.heading_deg, 1);
        if (OBSTACLE_FINAL_PARKING_TEST_ONLY &&
            !OBSTACLE_FINAL_PARKING_ENTRY_ARMED && !practiceMode)
        {
            enterTestHold("capture accepted; entry segments locked");
            return true;
        }
        segmentIndex = 0;
        stateStartMs = millis();
        state = FP_SEGMENT_SETTLE;
        return true;
    }

    if (state == FP_SEGMENT_SETTLE)
    {
        if (dc_state != DC_DISABLED)
            stop(false);
        const int steering = PARKING_SEGMENTS[segmentIndex].steering *
                             routeTurnSign;
        servo_disabled = false;
        set_steering(steering);
        steer(steering);
        if (millis() - stateStartMs <
            OBSTACLE_FINAL_PARKING_STEER_SETTLE_MS)
            return true;
        stateStartDistance = get_distance();
        stateStartMs = millis();
        state = FP_SEGMENT_DRIVE;
        Serial.print("[FINAL PARK SEGMENT] index=");
        Serial.print(segmentIndex + 1);
        Serial.print(" direction=");
        Serial.print(PARKING_SEGMENTS[segmentIndex].direction);
        Serial.print(" steering=");
        Serial.print(steering);
        Serial.print(" distance_mm=");
        Serial.println(PARKING_SEGMENTS[segmentIndex].distanceMm, 1);
        return true;
    }

    if (state == FP_SEGMENT_DRIVE)
    {
        const ParkingSegment &segment = PARKING_SEGMENTS[segmentIndex];
        set_speed(segment.direction * OBSTACLE_FINAL_PARKING_ENTRY_SPEED);
        set_steering(segment.steering * routeTurnSign);
        if (distanceSince(stateStartDistance) >= segment.distanceMm)
        {
            stop(true);
            stateStartMs = millis();
            state = FP_SEGMENT_BRAKE;
        }
        else if (millis() - stateStartMs >=
                 OBSTACLE_FINAL_PARKING_SEGMENT_TIMEOUT_MS)
            abortParking("segment_timeout");
        return true;
    }

    if (state == FP_SEGMENT_BRAKE)
    {
        if (millis() - stateStartMs <
            OBSTACLE_FINAL_PARKING_HOLD_BRAKE_MS)
            return true;
        stop(false);
        if (!endpointWithinGate(segmentIndex))
        {
            abortParking("segment_pose_gate");
            return true;
        }
        ++segmentIndex;
        if (segmentIndex >= OBSTACLE_FINAL_PARKING_SEGMENT_COUNT)
        {
            state = FP_VERIFY;
            return true;
        }
        if (OBSTACLE_FINAL_PARKING_TEST_ONLY &&
            segmentIndex >= OBSTACLE_FINAL_PARKING_TEST_SEGMENT_LIMIT)
        {
            enterTestHold("configured segment limit reached");
            return true;
        }
        stateStartMs = millis();
        state = FP_SEGMENT_SETTLE;
        return true;
    }

    if (state == FP_VERIFY)
    {
        set_steering(0);
        pose = get_position_struct();
        const bool contained = finalFootprintContained();
        const bool stopped =
            fabsf(current_speed) <= SOFT_STOP_SPEED_THRESHOLD_MMS &&
            fabsf(measured_speed) <= SOFT_STOP_SPEED_THRESHOLD_MMS;
        Serial.print("[FINAL PARK RESULT] contained=");
        Serial.print(contained ? "yes" : "no");
        Serial.print(" stopped=");
        Serial.print(stopped ? "yes" : "no");
        Serial.print(" pose_x_y_heading=");
        Serial.print(pose.x_mm, 1);
        Serial.print("/");
        Serial.print(pose.y_mm, 1);
        Serial.print("/");
        Serial.print(pose.heading_deg, 1);
        Serial.print(" travel_mm=");
        Serial.println(distanceSince(runStartDistance), 1);
        if (!contained || !stopped)
        {
            abortParking(
                contained ? "final_speed_gate" : "final_containment_gate");
            return true;
        }
        stop(true);
        completed = true;
        state = FP_HOLD;
        robot_logger.write_to_usb();
        Serial.println("[FINAL PARK] Complete; motor hold active");
        return true;
    }

    return true;
}

bool final_parking_complete()
{
    return completed;
}

bool final_parking_aborted()
{
    return aborted;
}
