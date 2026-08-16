#include "obstacle_seat_test.h"

#include <ctype.h>
#include <math.h>

#include "config.h"
#include "logger.h"
#include "motor_control.h"
#include "obstacle.h"
#include "obstacle_path.h"
#include "position_estimator.h"

#undef Serial
#define Serial robot_logger

namespace
{
int8_t requestedTurnSign = 1;
int8_t expectedSeatId = -1;
bool preflightPassed = false;
unsigned long lastRepeatedPrintMs = 0;
ObstacleObservationStatus lastStatus = OBSTACLE_OBSERVATION_NO_BLOB;
int8_t lastSeatId = -2;
bool lastWrongSeat = false;

const char *statusName(ObstacleObservationStatus status)
{
    switch (status)
    {
    case OBSTACLE_OBSERVATION_NO_BLOB: return "REJECTED_NO_BLOB";
    case OBSTACLE_OBSERVATION_REJECTED_BLOB: return "REJECTED_BLOB";
    case OBSTACLE_OBSERVATION_INVALID_RANGE: return "REJECTED_RANGE";
    case OBSTACLE_OBSERVATION_NO_SEAT: return "REJECTED_NO_SEAT";
    case OBSTACLE_OBSERVATION_VOTE: return "VOTE";
    case OBSTACLE_OBSERVATION_CONFIRMED: return "CONFIRMED_INJECTED";
    case OBSTACLE_OBSERVATION_ALREADY_CONFIRMED: return "ALREADY_CONFIRMED";
    }
    return "UNKNOWN";
}

const char *colorName(ColorType color)
{
    if (color == ColorType::RED)
        return "RED";
    if (color == ColorType::GREEN)
        return "GREEN";
    return "NONE";
}

void forceMotorLock()
{
    if (dc_state != DC_DISABLED || target_speed != 0 || dc_out != 0)
        stop(false);
    set_steering(0);
    servo_disabled = true;
}

void centerAndDisableSteering()
{
    set_steering(0);
    servo_disabled = false;
    steer(0);
    servo_disabled = true;
}

void printObservation(const ObstacleObservationResult &result, bool wrongSeat)
{
    Serial.print("[SEAT] event=");
    Serial.print(wrongSeat ? "WRONG_SEAT" : statusName(result.status));
    Serial.print(" color=");
    Serial.print(colorName(result.color));
    Serial.print(" bounds=");
    Serial.print(result.left); Serial.print(",");
    Serial.print(result.top); Serial.print("-");
    Serial.print(result.right); Serial.print(",");
    Serial.print(result.bottom);
    Serial.print(" valid=");
    Serial.print(result.productionValid ? "yes" : "no");
    Serial.print(" bearing="); Serial.print(result.bearingDeg, 1);
    Serial.print(" range="); Serial.print(result.rangeMm, 1);
    Serial.print(" pose="); Serial.print(result.robotXmm, 1);
    Serial.print(","); Serial.print(result.robotYmm, 1);
    Serial.print(","); Serial.print(result.robotHeadingDeg, 1);
    Serial.print(" camera="); Serial.print(result.cameraXmm, 1);
    Serial.print(","); Serial.print(result.cameraYmm, 1);
    Serial.print(" sighting="); Serial.print(result.sightingXmm, 1);
    Serial.print(","); Serial.print(result.sightingYmm, 1);
    Serial.print(" seat="); Serial.print(result.seatId);
    Serial.print(" expected="); Serial.print(expectedSeatId);
    Serial.print(" snap_error="); Serial.print(result.snapErrorMm, 1);
    Serial.print(" votes=R"); Serial.print(result.redVotes);
    Serial.print("/G"); Serial.print(result.greenVotes);
    Serial.print(" injections="); Serial.print(result.injectionCount);
    if (result.status == OBSTACLE_OBSERVATION_CONFIRMED)
    {
        Serial.print(" pass="); Serial.print(result.passSide);
        Serial.print(" peak="); Serial.print(result.peakDisplacementMm, 1);
        Serial.print(" taper=+/-"); Serial.print(OBSTACLE_PATH_TAPER_WAYPOINTS);
        Serial.print(" circle_clearance=");
        Serial.print(result.movementCircleClearanceMm, 1);
    }
    Serial.println();
}
} // namespace

void obstacle_seat_test_set_turn_sign(int8_t turn_sign)
{
    requestedTurnSign = turn_sign < 0 ? -1 : 1;
}

void obstacle_seat_test_start()
{
    forceMotorLock();
    centerAndDisableSteering();
    expectedSeatId = -1;
    lastStatus = OBSTACLE_OBSERVATION_NO_BLOB;
    lastSeatId = -2;
    lastWrongSeat = false;

    obstacle_path_start(requestedTurnSign, true);
    const bool selectedDirectionPassed = obstacle_path_geometry_preflight();
    obstacle_path_start(-requestedTurnSign, true);
    const bool oppositeDirectionPassed = obstacle_path_geometry_preflight();
    obstacle_path_start(requestedTurnSign, true);
    preflightPassed = selectedDirectionPassed && oppositeDirectionPassed &&
                      obstacle_path_geometry_preflight();

    Serial.print("[SEAT] geometry preflight LEFT+RIGHT: ");
    Serial.println(preflightPassed ? "PASS" : "FAIL");
    Serial.print("[SEAT] motor lock: ");
    Serial.println(
        dc_state == DC_DISABLED && target_speed == 0 && servo_disabled
            ? "PASS"
            : "FAIL");
    Serial.println("[SEAT] Use: seat expect <section 0-3> <station 0-2> <L|R> <range_mm>");
}

void obstacle_seat_test_stop()
{
    forceMotorLock();
    centerAndDisableSteering();
    obstacle_path_reset();
    expectedSeatId = -1;
    Serial.println("[SEAT] stopped and cleared; drive motor remains off");
}

void obstacle_seat_test_update(bool new_camera_frame)
{
    forceMotorLock();
    if (!new_camera_frame || !preflightPassed || expectedSeatId < 0)
        return;

    const Blob *blob = getLargestValidObstacle();
    if (blob == nullptr)
        blob = getLargestObstacle();
    const ObstacleObservationResult result = obstacle_path_observe(blob);
    const bool wrongSeat = result.seatId >= 0 && result.seatId != expectedSeatId;
    const bool important =
        result.status == OBSTACLE_OBSERVATION_VOTE ||
        result.status == OBSTACLE_OBSERVATION_CONFIRMED ||
        result.status != lastStatus ||
        result.seatId != lastSeatId ||
        wrongSeat != lastWrongSeat;
    const unsigned long now = millis();
    if (important || now - lastRepeatedPrintMs >= 1000)
    {
        printObservation(result, wrongSeat);
        lastRepeatedPrintMs = now;
    }
    lastStatus = result.status;
    lastSeatId = result.seatId;
    lastWrongSeat = wrongSeat;
}

bool obstacle_seat_test_expect(
    uint8_t section,
    uint8_t station,
    char side,
    float range_mm)
{
    side = static_cast<char>(toupper(static_cast<unsigned char>(side)));
    if (!obstacle_path_started() || section >= 4 || station >= 3 ||
        (side != 'L' && side != 'R') ||
        !isfinite(range_mm) || range_mm < 150.0f || range_mm > 1000.0f)
        return false;

    const uint8_t seatId = section * 6 + station * 2 + (side == 'L' ? 1 : 0);
    ObstacleSeatInfo seat;
    if (!obstacle_path_get_seat(seatId, seat) ||
        range_mm <= fabsf(seat.lateralMm))
        return false;

    obstacle_path_clear_observations();
    const float heading = seat.headingDeg * PI / 180.0f;
    const float tangentX = cosf(heading);
    const float tangentY = sinf(heading);
    const float normalX = -sinf(heading);
    const float normalY = cosf(heading);
    const float centerX = seat.xMm - normalX * seat.lateralMm;
    const float centerY = seat.yMm - normalY * seat.lateralMm;
    const float forwardMm = sqrtf(
        range_mm * range_mm - seat.lateralMm * seat.lateralMm);
    const float cameraX = centerX - tangentX * forwardMm;
    const float cameraY = centerY - tangentY * forwardMm;
    const float robotX =
        cameraX -
        OBSTACLE_CAMERA_LOCAL_X_MM * tangentX +
        OBSTACLE_CAMERA_LOCAL_Y_MM * tangentY;
    const float robotY =
        cameraY -
        OBSTACLE_CAMERA_LOCAL_X_MM * tangentY -
        OBSTACLE_CAMERA_LOCAL_Y_MM * tangentX;
    position_reset(robotX, robotY, seat.headingDeg);
    expectedSeatId = static_cast<int8_t>(seatId);
    lastStatus = OBSTACLE_OBSERVATION_NO_BLOB;
    lastSeatId = -2;
    lastWrongSeat = false;

    const float expectedBearing = atan2f(seat.lateralMm, forwardMm) * 180.0f / PI;
    Serial.print("[SEAT] armed id="); Serial.print(seatId);
    Serial.print(" label="); Serial.print(section); Serial.print("/");
    Serial.print(station); Serial.print("/"); Serial.print(side);
    Serial.print(" camera_range="); Serial.print(range_mm, 1);
    Serial.print(" expected_bearing="); Serial.print(expectedBearing, 1);
    Serial.print(" robot_pose="); Serial.print(robotX, 1);
    Serial.print(","); Serial.print(robotY, 1);
    Serial.print(","); Serial.println(seat.headingDeg, 1);
    return true;
}

void obstacle_seat_test_clear()
{
    obstacle_path_clear_observations();
    expectedSeatId = -1;
    lastStatus = OBSTACLE_OBSERVATION_NO_BLOB;
    lastSeatId = -2;
    lastWrongSeat = false;
    Serial.println("[SEAT] votes, injection path, and expected seat cleared");
}

void obstacle_seat_test_show()
{
    Serial.println("[SEAT] id section/station/side x y heading path red green state");
    for (uint8_t i = 0; i < obstacle_path_seat_count(); ++i)
    {
        ObstacleSeatInfo seat;
        if (!obstacle_path_get_seat(i, seat))
            continue;
        Serial.print("[SEAT] "); Serial.print(seat.id);
        Serial.print(" "); Serial.print(seat.section); Serial.print("/");
        Serial.print(seat.station); Serial.print("/"); Serial.print(seat.side);
        Serial.print(" "); Serial.print(seat.xMm, 1);
        Serial.print(" "); Serial.print(seat.yMm, 1);
        Serial.print(" "); Serial.print(seat.headingDeg, 1);
        Serial.print(" "); Serial.print(seat.pathDistanceMm, 1);
        Serial.print(" R"); Serial.print(seat.redVotes);
        Serial.print(" G"); Serial.print(seat.greenVotes);
        Serial.print(" ");
        Serial.println(seat.confirmed ? (seat.red ? "RED" : "GREEN") : "open");
    }
}

bool obstacle_seat_test_preflight_passed()
{
    return preflightPassed;
}
