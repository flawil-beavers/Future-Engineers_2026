#include "obstacle_path.h"

#include <math.h>
#include <string.h>

#include "config.h"
#include "course_map.h"
#include "motor_control.h"
#include "obstacle.h"
#include "position_estimator.h"
#include "sensors.h"
#include "vision.h"
#include "logger.h"

#undef Serial
#define Serial robot_logger

namespace
{
struct PathPoint
{
    float x = 0.0f;
    float y = 0.0f;
    float headingDeg = 0.0f;
    float distanceMm = 0.0f;
    float speedMmS = OBSTACLE_PATH_MIN_SPEED;
};

struct CandidateSeat
{
    float x = 0.0f;
    float y = 0.0f;
    float pathDistanceMm = 0.0f;
    float lateralMm = 0.0f;
    float headingDeg = 0.0f;
    uint8_t redVotes = 0;
    uint8_t greenVotes = 0;
    unsigned long lastVoteMs = 0;
    bool confirmed = false;
    bool red = false;
    bool injected = false;
};

struct CornerGeometry
{
    float pathStartMm = 0.0f;
    float pathEndMm = 0.0f;
    bool recedesOnLeft = false;
    bool recedesOnRight = false;
};

struct DiscoveryStation
{
    uint8_t clearFrames[COURSE_SEATS_PER_STATION] = {};
    bool seatObservedClear[COURSE_SEATS_PER_STATION] = {};
    uint8_t lastClearEvidenceMask = 0;
    bool observedClear = false;
};

PathPoint baselinePath[OBSTACLE_MAX_PATH_WAYPOINTS];
PathPoint livePath[OBSTACLE_MAX_PATH_WAYPOINTS];
PathPoint optimizedPath[OBSTACLE_MAX_PATH_WAYPOINTS];
PathPoint smoothingBuffer[OBSTACLE_MAX_PATH_WAYPOINTS];
CandidateSeat seats[OBSTACLE_SEAT_COUNT];
DiscoveryStation discoveryStations[OBSTACLE_SEAT_COUNT / 2];

uint16_t pathLength = 0;
uint16_t progressIndex = 0;
uint8_t completedLaps = 0;
int8_t routeTurnSign = 1;
bool running = false;
bool finished = false;
bool optimizedBuilt = false;
bool runtimeTestMode = false;
uint8_t runtimeLapTarget = 3;
float runtimeSpeedCapMmS = 0.0f;
float loopLengthMm = 0.0f;
float firstCornerDistanceMm = OBSTACLE_STRAIGHT_LENGTH_MM * 0.5f;
uint16_t injectionCount = 0;
bool discoveryBlocked = false;
int8_t discoveryBlockedStation = -1;
float lastDiscoveryTargetNudgeDeg = 0.0f;
int8_t discoveryScanStation = -1;
int8_t discoveryScanSide = -1;
uint32_t lastDiscoveryNudgeUpdateMs = 0;
ObstacleObservationResult lastDiscoveryObservation;
ObstacleTofCorrectionResult lastTofCorrectionResult;
CornerGeometry corners[4];
uint32_t lastTofCorrectionSequence[TOF_COUNT] = {};

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

float distanceSquared(float x1, float y1, float x2, float y2)
{
    const float dx = x1 - x2;
    const float dy = y1 - y2;
    return dx * dx + dy * dy;
}

void transformFromRunFrame(
    float localX,
    float localY,
    const PositionEstimate &anchor,
    float &globalX,
    float &globalY)
{
    const float heading = anchor.heading_deg * PI / 180.0f;
    const float c = cosf(heading);
    const float s = sinf(heading);
    globalX = anchor.x_mm + localX * c - localY * s;
    globalY = anchor.y_mm + localX * s + localY * c;
}

void appendLocalPoint(
    float localX,
    float localY,
    float localHeadingDeg,
    float distanceMm,
    const PositionEstimate &anchor)
{
    if (pathLength >= OBSTACLE_MAX_PATH_WAYPOINTS)
        return;

    PathPoint &point = baselinePath[pathLength++];
    transformFromRunFrame(localX, localY, anchor, point.x, point.y);
    point.headingDeg = anchor.heading_deg + localHeadingDeg;
    point.distanceMm = distanceMm;
}

void appendStraight(
    float lengthMm,
    float &x,
    float &y,
    float headingRad,
    float &distanceMm,
    const PositionEstimate &anchor)
{
    float remaining = lengthMm;
    while (remaining > 0.1f)
    {
        const float step = fminf(OBSTACLE_PATH_SAMPLE_MM, remaining);
        x += step * cosf(headingRad);
        y += step * sinf(headingRad);
        distanceMm += step;
        appendLocalPoint(
            x,
            y,
            headingRad * 180.0f / PI,
            distanceMm,
            anchor);
        remaining -= step;
    }
}

void appendCorner(
    uint8_t corner,
    float &x,
    float &y,
    float &headingRad,
    float &distanceMm,
    const PositionEstimate &anchor)
{
    corners[corner].pathStartMm = distanceMm;
    corners[corner].recedesOnLeft = routeTurnSign > 0;
    corners[corner].recedesOnRight = routeTurnSign < 0;
    const float arcLength = PI * OBSTACLE_CORNER_RADIUS_MM * 0.5f;
    float remaining = arcLength;
    const float curvature = routeTurnSign / OBSTACLE_CORNER_RADIUS_MM;

    while (remaining > 0.1f)
    {
        const float step = fminf(OBSTACLE_PATH_SAMPLE_MM, remaining);
        const float nextHeading = headingRad + curvature * step;
        x += (sinf(nextHeading) - sinf(headingRad)) / curvature;
        y += (-cosf(nextHeading) + cosf(headingRad)) / curvature;
        headingRad = nextHeading;
        distanceMm += step;
        appendLocalPoint(
            x,
            y,
            headingRad * 180.0f / PI,
            distanceMm,
            anchor);
        remaining -= step;
    }
    corners[corner].pathEndMm = distanceMm;
}

PathPoint interpolateBaseline(float distanceMm)
{
    while (distanceMm < 0.0f)
        distanceMm += loopLengthMm;
    while (distanceMm >= loopLengthMm)
        distanceMm -= loopLengthMm;

    uint16_t upper = 1;
    while (upper < pathLength &&
           baselinePath[upper].distanceMm < distanceMm)
    {
        ++upper;
    }

    if (upper >= pathLength)
        return baselinePath[pathLength - 1];

    const PathPoint &a = baselinePath[upper - 1];
    const PathPoint &b = baselinePath[upper];
    const float span = b.distanceMm - a.distanceMm;
    const float t = span > 0.1f
                        ? (distanceMm - a.distanceMm) / span
                        : 0.0f;

    PathPoint result;
    result.x = a.x + (b.x - a.x) * t;
    result.y = a.y + (b.y - a.y) * t;
    result.headingDeg =
        a.headingDeg + wrap180(b.headingDeg - a.headingDeg) * t;
    result.distanceMm = distanceMm;
    return result;
}

void initializeSeats()
{
    const float cornerArc = PI * OBSTACLE_CORNER_RADIUS_MM * 0.5f;
    const float sectionStarts[4] = {
        loopLengthMm + firstCornerDistanceMm -
            OBSTACLE_STRAIGHT_LENGTH_MM,
        firstCornerDistanceMm + cornerArc,
        firstCornerDistanceMm + OBSTACLE_STRAIGHT_LENGTH_MM +
            2.0f * cornerArc,
        firstCornerDistanceMm + 2.0f * OBSTACLE_STRAIGHT_LENGTH_MM +
            3.0f * cornerArc};

    uint8_t seatIndex = 0;
    for (uint8_t section = 0; section < 4; ++section)
    {
        for (uint8_t station = 0; station < 3; ++station)
        {
            float pathDistance =
                sectionStarts[section] +
                station * (OBSTACLE_STRAIGHT_LENGTH_MM * 0.5f);
            if (pathDistance >= loopLengthMm)
                pathDistance -= loopLengthMm;

            const PathPoint center = interpolateBaseline(pathDistance);
            const float heading = center.headingDeg * PI / 180.0f;
            const float normalX = -sinf(heading);
            const float normalY = cosf(heading);

            for (uint8_t side = 0; side < 2; ++side)
            {
                CandidateSeat &seat = seats[seatIndex++];
                seat = CandidateSeat();
                seat.pathDistanceMm = pathDistance;
                seat.headingDeg = center.headingDeg;
                seat.lateralMm =
                    side == 0
                        ? -OBSTACLE_SEAT_LATERAL_MM
                        : OBSTACLE_SEAT_LATERAL_MM;
                seat.x = center.x + normalX * seat.lateralMm;
                seat.y = center.y + normalY * seat.lateralMm;
            }
        }
    }
}

PositionEstimate nominalFieldStartPose(
    int8_t turnSign,
    float distanceToFirstCornerMm)
{
    PositionEstimate pose;
    pose.y_mm =
        OBSTACLE_FIELD_ORIGIN_Y_MM -
        OBSTACLE_CENTERLINE_HALF_EXTENT_MM;
    pose.confidence_mm = 0.0f;

    if (turnSign > 0)
    {
        // CCW: travel east (+X) along the south straight before turning left.
        pose.x_mm =
            OBSTACLE_FIELD_ORIGIN_X_MM +
            OBSTACLE_STRAIGHT_LENGTH_MM * 0.5f -
            distanceToFirstCornerMm;
        pose.heading_deg = 0.0f;
    }
    else
    {
        // CW: travel west (-X) along the south straight before turning right.
        pose.x_mm =
            OBSTACLE_FIELD_ORIGIN_X_MM -
            OBSTACLE_STRAIGHT_LENGTH_MM * 0.5f +
            distanceToFirstCornerMm;
        pose.heading_deg = 180.0f;
    }
    return pose;
}

uint8_t stationIndexForSeat(uint8_t seatIndex)
{
    return seatIndex / 2;
}

bool stationResolved(uint8_t stationIndex)
{
    if (stationIndex >= OBSTACLE_SEAT_COUNT / 2)
        return true;
    const uint8_t firstSeat = stationIndex * 2;
    return discoveryStations[stationIndex].observedClear ||
           seats[firstSeat].confirmed || seats[firstSeat + 1].confirmed;
}

bool allStationsResolved()
{
    for (uint8_t station = 0;
         station < OBSTACLE_SEAT_COUNT / 2;
         ++station)
    {
        if (!stationResolved(station))
            return false;
    }
    return true;
}

int nearestSeatIndex(float x, float y, float *errorMm = nullptr)
{
    int bestSeat = -1;
    float bestDistanceSquared =
        OBSTACLE_SEAT_SNAP_RADIUS_MM * OBSTACLE_SEAT_SNAP_RADIUS_MM;
    for (uint8_t i = 0; i < OBSTACLE_SEAT_COUNT; ++i)
    {
        const float candidateDistance = distanceSquared(
            x, y, seats[i].x, seats[i].y);
        if (candidateDistance < bestDistanceSquared)
        {
            bestDistanceSquared = candidateDistance;
            bestSeat = i;
        }
    }
    if (errorMm != nullptr)
        *errorMm = bestSeat >= 0 ? sqrtf(bestDistanceSquared) : -1.0f;
    return bestSeat;
}

bool recordSeatVote(CandidateSeat &seat, ColorType color)
{
    if (seat.confirmed || (color != ColorType::RED && color != ColorType::GREEN))
        return false;

    const unsigned long now = millis();
    if (seat.lastVoteMs != 0 &&
        now - seat.lastVoteMs > OBSTACLE_SEAT_VOTE_WINDOW_MS)
    {
        seat.redVotes = 0;
        seat.greenVotes = 0;
    }

    uint8_t &votes = color == ColorType::RED
                         ? seat.redVotes
                         : seat.greenVotes;
    if (votes < 255)
        ++votes;
    seat.lastVoteMs = now;

    const uint8_t winningVotes =
        seat.redVotes > seat.greenVotes ? seat.redVotes : seat.greenVotes;
    if (winningVotes < OBSTACLE_SEAT_CONFIRM_VOTES)
        return false;

    seat.confirmed = true;
    seat.red = seat.redVotes > seat.greenVotes;
    return true;
}

void prepareConsecutiveVote(uint8_t selectedSeat, ColorType color)
{
    // Confirmation requires matching accepted observations in sequence. A
    // stale isolated blob must not remain armed while normal observations of
    // another seat continue between it and a later false blob.
    for (uint8_t i = 0; i < OBSTACLE_SEAT_COUNT; ++i)
    {
        CandidateSeat &seat = seats[i];
        if (seat.confirmed)
            continue;
        if (i != selectedSeat)
        {
            seat.redVotes = 0;
            seat.greenVotes = 0;
            seat.lastVoteMs = 0;
        }
        else if (color == ColorType::RED)
        {
            seat.greenVotes = 0;
        }
        else if (color == ColorType::GREEN)
        {
            seat.redVotes = 0;
        }
    }
}

void clearPendingVotes()
{
    for (uint8_t i = 0; i < OBSTACLE_SEAT_COUNT; ++i)
    {
        if (!seats[i].confirmed)
        {
            seats[i].redVotes = 0;
            seats[i].greenVotes = 0;
            seats[i].lastVoteMs = 0;
        }
    }
}

void expirePendingVotes()
{
    const unsigned long now = millis();
    for (uint8_t i = 0; i < OBSTACLE_SEAT_COUNT; ++i)
    {
        CandidateSeat &seat = seats[i];
        if (!seat.confirmed && seat.lastVoteMs != 0 &&
            now - seat.lastVoteMs > OBSTACLE_SEAT_VOTE_WINDOW_MS)
        {
            seat.redVotes = 0;
            seat.greenVotes = 0;
            seat.lastVoteMs = 0;
        }
    }
}

float cyclicDistanceForward(float fromMm, float toMm)
{
    float distance = toMm - fromMm;
    if (distance < 0.0f)
        distance += loopLengthMm;
    return distance;
}

bool withinCornerGate(float pathDistance, uint8_t corner)
{
    const float start =
        corners[corner].pathStartMm - OBSTACLE_CORNER_GATE_BEFORE_MM;
    const float end =
        corners[corner].pathEndMm + OBSTACLE_CORNER_GATE_AFTER_MM;

    if (start >= 0.0f && end < loopLengthMm)
        return pathDistance >= start && pathDistance <= end;

    const float wrappedStart =
        start < 0.0f ? start + loopLengthMm : start;
    const float wrappedEnd =
        end >= loopLengthMm ? end - loopLengthMm : end;
    return pathDistance >= wrappedStart || pathDistance <= wrappedEnd;
}

bool nearCorner(float pathDistance)
{
    for (uint8_t corner = 0; corner < 4; ++corner)
    {
        if (withinCornerGate(pathDistance, corner))
            return true;
    }
    return false;
}

void recomputeSpeedProfile(PathPoint *path)
{
    for (uint16_t i = 0; i < pathLength; ++i)
    {
        const PathPoint &before = path[(i + pathLength - 1) % pathLength];
        const PathPoint &at = path[i];
        const PathPoint &after = path[(i + 1) % pathLength];
        const float h1 = atan2f(at.y - before.y, at.x - before.x);
        const float h2 = atan2f(after.y - at.y, after.x - at.x);
        const float segment =
            fmaxf(1.0f, hypotf(after.x - at.x, after.y - at.y));
        const float curvature =
            fabsf(wrap180((h2 - h1) * 180.0f / PI) * PI / 180.0f) /
            segment;
        path[i].speedMmS = clampFloat(
            OBSTACLE_PATH_MAX_SPEED -
                OBSTACLE_CURVATURE_SPEED_GAIN * curvature,
            OBSTACLE_PATH_MIN_SPEED,
            OBSTACLE_PATH_MAX_SPEED);
    }
}

void smoothRange(PathPoint *path, int center, int taper)
{
    const int first = center - taper - 1;
    const int last = center + taper + 1;
    memcpy(smoothingBuffer, path, sizeof(PathPoint) * pathLength);

    for (int raw = first; raw <= last; ++raw)
    {
        int index = raw;
        while (index < 0)
            index += pathLength;
        while (index >= pathLength)
            index -= pathLength;

        float sumX = 0.0f;
        float sumY = 0.0f;
        int count = 0;
        for (int offset = -OBSTACLE_PATH_SMOOTH_RADIUS;
             offset <= OBSTACLE_PATH_SMOOTH_RADIUS;
             ++offset)
        {
            int sample = index + offset;
            while (sample < 0)
                sample += pathLength;
            while (sample >= pathLength)
                sample -= pathLength;
            sumX += smoothingBuffer[sample].x;
            sumY += smoothingBuffer[sample].y;
            ++count;
        }
        path[index].x = sumX / count;
        path[index].y = sumY / count;
    }
}

uint16_t nearestPathIndex(
    const PathPoint *path,
    float x,
    float y,
    uint16_t start,
    uint16_t count)
{
    uint16_t best = start;
    float bestDistance = 1.0e12f;
    for (uint16_t offset = 0; offset < count; ++offset)
    {
        const uint16_t index = (start + offset) % pathLength;
        const float distance =
            distanceSquared(path[index].x, path[index].y, x, y);
        if (distance < bestDistance)
        {
            bestDistance = distance;
            best = index;
        }
    }
    return best;
}

float targetLateralForSeat(const CandidateSeat &seat, float clearanceMm)
{
    // In path-local coordinates positive is left. Red is passed on its right
    // and green on its left.
    return seat.lateralMm + (seat.red ? -clearanceMm : clearanceMm);
}

void displaceForSeat(
    PathPoint *path,
    CandidateSeat &seat,
    float clearanceMm)
{
    const uint16_t center = nearestPathIndex(
        baselinePath,
        seat.x,
        seat.y,
        0,
        pathLength);
    const float targetLateral = targetLateralForSeat(seat, clearanceMm);
    const float heading = baselinePath[center].headingDeg * PI / 180.0f;
    const float normalX = -sinf(heading);
    const float normalY = cosf(heading);

    for (int offset = -OBSTACLE_PATH_TAPER_WAYPOINTS;
         offset <= OBSTACLE_PATH_TAPER_WAYPOINTS;
         ++offset)
    {
        int index = static_cast<int>(center) + offset;
        while (index < 0)
            index += pathLength;
        while (index >= pathLength)
            index -= pathLength;
        const float taper =
            1.0f - fabsf(static_cast<float>(offset)) /
                       (OBSTACLE_PATH_TAPER_WAYPOINTS + 1.0f);
        path[index].x += normalX * targetLateral * taper;
        path[index].y += normalY * targetLateral * taper;
    }

    smoothRange(path, center, OBSTACLE_PATH_TAPER_WAYPOINTS);
    recomputeSpeedProfile(path);
}

void buildOptimizedPath()
{
    memcpy(optimizedPath, baselinePath, sizeof(PathPoint) * pathLength);
    for (uint8_t i = 0; i < OBSTACLE_SEAT_COUNT; ++i)
    {
        if (seats[i].confirmed)
            displaceForSeat(
                optimizedPath,
                seats[i],
                OBSTACLE_OPTIMIZED_CLEARANCE_MM);
    }
    recomputeSpeedProfile(optimizedPath);
    optimizedBuilt = true;
    Serial.println("[PATH] Optimized laps 2-3 path built");
}

void updateProgress(const PathPoint *path, const PositionEstimate &pose)
{
    const uint16_t previous = progressIndex;
    progressIndex = nearestPathIndex(
        path,
        pose.x_mm,
        pose.y_mm,
        progressIndex,
        OBSTACLE_PATH_PROGRESS_WINDOW);

    if (previous > pathLength * 3 / 4 &&
        progressIndex < pathLength / 4)
    {
        if (!runtimeTestMode && completedLaps == 0 &&
            !allStationsResolved())
        {
            Serial.println(
                "[PATH] Lap boundary reached with unresolved stations");
            return;
        }
        if (completedLaps < 255)
            ++completedLaps;
        Serial.print("[PATH] Completed lap ");
        Serial.println(completedLaps);

        if (completedLaps >= runtimeLapTarget)
            finished = true;
        else if (completedLaps == 1)
            buildOptimizedPath();
    }
}

bool seatComfortablyVisible(
    uint8_t seatIndex,
    const PositionEstimate &pose);

void applyDiscoveryTargetNudge(
    PathPoint &target,
    const PositionEstimate &pose)
{
    float desiredNudgeDeg = 0.0f;
    const float currentDistance = baselinePath[progressIndex].distanceMm;
    float bestForward = OBSTACLE_LOOK_START_MM + 1.0f;
    int bestStation = -1;

    if (completedLaps == 0)
    {
        for (uint8_t station = 0;
             station < OBSTACLE_SEAT_COUNT / COURSE_SEATS_PER_STATION;
             ++station)
        {
            if (stationResolved(station))
                continue;
            const float forward = cyclicDistanceForward(
                currentDistance,
                seats[station * COURSE_SEATS_PER_STATION].pathDistanceMm);
            if (forward < OBSTACLE_LOOK_END_MM ||
                forward > OBSTACLE_LOOK_START_MM ||
                forward >= bestForward)
                continue;

            bestForward = forward;
            bestStation = station;
        }
    }

    if (bestStation >= 0)
    {
        const float heading = pose.heading_deg * PI / 180.0f;
        const float cameraX =
            pose.x_mm + OBSTACLE_CAMERA_LOCAL_X_MM * cosf(heading) -
            OBSTACLE_CAMERA_LOCAL_Y_MM * sinf(heading);
        const float cameraY =
            pose.y_mm + OBSTACLE_CAMERA_LOCAL_X_MM * sinf(heading) +
            OBSTACLE_CAMERA_LOCAL_Y_MM * cosf(heading);
        float bearing[COURSE_SEATS_PER_STATION] = {};
        for (uint8_t side = 0; side < COURSE_SEATS_PER_STATION; ++side)
        {
            const uint8_t seatIndex =
                bestStation * COURSE_SEATS_PER_STATION + side;
            const CandidateSeat &seat = seats[seatIndex];
            bearing[side] = wrap180(
                atan2f(seat.y - cameraY, seat.x - cameraX) *
                    180.0f / PI -
                pose.heading_deg);
        }

        DiscoveryStation &coverage = discoveryStations[bestStation];
        const bool unresolved0 = !coverage.seatObservedClear[0];
        const bool unresolved1 = !coverage.seatObservedClear[1];
        float aimBearingDeg = 0.0f;
        const float comfortableBearingDeg = fmaxf(
            0.0f,
            OBSTACLE_CAMERA_HORIZONTAL_FOV_DEG *
                    OBSTACLE_DISCOVERY_FOV_FRACTION -
                OBSTACLE_LOOK_FOV_MARGIN_DEG);
        float allowedBearingDeg = comfortableBearingDeg;
        float targetGain = OBSTACLE_LOOK_TARGET_GAIN;

        discoveryScanStation = bestStation;
        if (unresolved0 && unresolved1)
        {
            // Use the full comfortable view: nudge only when centring the pair
            // this far from the optical axis would put either seat outside it.
            // The wrapped difference avoids the long way around at +/-180.
            const float separationDeg =
                fabsf(wrap180(bearing[1] - bearing[0]));
            aimBearingDeg = wrap180(
                bearing[0] + 0.5f * wrap180(bearing[1] - bearing[0]));
            allowedBearingDeg = fmaxf(
                0.0f,
                comfortableBearingDeg - 0.5f * separationDeg);
            discoveryScanSide = -2; // telemetry: both seats simultaneously
        }
        else if (unresolved0)
        {
            aimBearingDeg = bearing[0];
            allowedBearingDeg = OBSTACLE_LOOK_SINGLE_SEAT_BEARING_DEG;
            targetGain = OBSTACLE_LOOK_SINGLE_SEAT_TARGET_GAIN;
            discoveryScanSide = 0;
        }
        else if (unresolved1)
        {
            aimBearingDeg = bearing[1];
            allowedBearingDeg = OBSTACLE_LOOK_SINGLE_SEAT_BEARING_DEG;
            targetGain = OBSTACLE_LOOK_SINGLE_SEAT_TARGET_GAIN;
            discoveryScanSide = 1;
        }
        else
        {
            discoveryScanSide = -1;
        }

        if (discoveryScanSide != -1)
        {
            const float excessBearing = fmaxf(
                0.0f,
                fabsf(aimBearingDeg) - allowedBearingDeg);
            const float taper = clampFloat(
                (OBSTACLE_LOOK_START_MM - bestForward) /
                    (OBSTACLE_LOOK_START_MM -
                     OBSTACLE_LOOK_FULL_NUDGE_MM),
                0.0f,
                1.0f);
            desiredNudgeDeg = copysignf(
                excessBearing * targetGain * taper,
                aimBearingDeg);
            desiredNudgeDeg = clampFloat(
                desiredNudgeDeg,
                -OBSTACLE_LOOK_MAX_TARGET_NUDGE_DEG,
                OBSTACLE_LOOK_MAX_TARGET_NUDGE_DEG);
        }
    }
    else
    {
        discoveryScanStation = -1;
        discoveryScanSide = -1;
    }

    const uint32_t now = millis();
    const float elapsedSeconds = lastDiscoveryNudgeUpdateMs == 0
        ? 0.02f
        : fminf(0.25f, (now - lastDiscoveryNudgeUpdateMs) / 1000.0f);
    lastDiscoveryNudgeUpdateMs = now;
    const float maximumChange =
        OBSTACLE_LOOK_NUDGE_SLEW_DEG_S * elapsedSeconds;
    lastDiscoveryTargetNudgeDeg += clampFloat(
        desiredNudgeDeg - lastDiscoveryTargetNudgeDeg,
        -maximumChange,
        maximumChange);

    const float nudge = lastDiscoveryTargetNudgeDeg * PI / 180.0f;
    const float dx = target.x - pose.x_mm;
    const float dy = target.y - pose.y_mm;
    target.x = pose.x_mm + dx * cosf(nudge) - dy * sinf(nudge);
    target.y = pose.y_mm + dx * sinf(nudge) + dy * cosf(nudge);
}

int nearestUpcomingUnresolvedStation(float &forwardMm)
{
    const float currentDistance = baselinePath[progressIndex].distanceMm;
    int bestStation = -1;
    forwardMm = loopLengthMm;
    for (uint8_t station = 0;
         station < OBSTACLE_SEAT_COUNT / 2;
         ++station)
    {
        if (stationResolved(station))
            continue;
        const float forward = cyclicDistanceForward(
            currentDistance,
            seats[station * 2].pathDistanceMm);
        // A station exactly underneath the startup pose is behind the camera;
        // it becomes observable normally when approached at the end of lap 1.
        if (forward <= 50.0f || forward >= forwardMm)
            continue;
        forwardMm = forward;
        bestStation = station;
    }
    return bestStation;
}

void seatCameraGeometry(
    uint8_t seatIndex,
    const PositionEstimate &pose,
    float &bearingDeg,
    float &rangeMm)
{
    if (seatIndex >= OBSTACLE_SEAT_COUNT)
    {
        bearingDeg = 0.0f;
        rangeMm = -1.0f;
        return;
    }

    const float heading = pose.heading_deg * PI / 180.0f;
    const float cameraX =
        pose.x_mm + OBSTACLE_CAMERA_LOCAL_X_MM * cosf(heading) -
        OBSTACLE_CAMERA_LOCAL_Y_MM * sinf(heading);
    const float cameraY =
        pose.y_mm + OBSTACLE_CAMERA_LOCAL_X_MM * sinf(heading) +
        OBSTACLE_CAMERA_LOCAL_Y_MM * cosf(heading);
    const CandidateSeat &seat = seats[seatIndex];
    const float dx = seat.x - cameraX;
    const float dy = seat.y - cameraY;
    rangeMm = hypotf(dx, dy);
    bearingDeg = wrap180(
        atan2f(dy, dx) * 180.0f / PI - pose.heading_deg);
}

float cameraBearingForImageX(float imageX)
{
    return atanf(
               (OBSTACLE_CAMERA_PRINCIPAL_X_PX - imageX) /
               OBSTACLE_CAMERA_FOCAL_X_PX) *
           180.0f / PI;
}

bool observationAllowsClearAtGeometry(
    const ObstacleObservationResult &observation,
    float seatBearingDeg,
    float seatRangeMm)
{
    if (observation.status == OBSTACLE_OBSERVATION_NO_BLOB)
        return true;

    // A rejected blob may be a partial pillar. Only production-valid geometry
    // can prove that the blob is elsewhere or safely behind this seat.
    if (!observation.productionValid ||
        !isfinite(observation.rangeMm) || observation.rangeMm <= 0.0f)
        return false;

    const float edgeBearing0 = cameraBearingForImageX(observation.left);
    const float edgeBearing1 = cameraBearingForImageX(observation.right);
    const float blobMinimumBearing =
        fminf(edgeBearing0, edgeBearing1) -
        OBSTACLE_DISCOVERY_BLOB_BEARING_MARGIN_DEG;
    const float blobMaximumBearing =
        fmaxf(edgeBearing0, edgeBearing1) +
        OBSTACLE_DISCOVERY_BLOB_BEARING_MARGIN_DEG;
    const bool overlapsSeatBearing =
        seatBearingDeg >= blobMinimumBearing &&
        seatBearingDeg <= blobMaximumBearing;
    if (!overlapsSeatBearing)
        return true;

    return observation.rangeMm >=
        seatRangeMm + OBSTACLE_DISCOVERY_BEHIND_SEAT_MARGIN_MM;
}

bool observationAllowsSeatClear(
    const ObstacleObservationResult &observation,
    uint8_t seatIndex,
    const PositionEstimate &pose)
{
    float seatBearingDeg = 0.0f;
    float seatRangeMm = -1.0f;
    seatCameraGeometry(seatIndex, pose, seatBearingDeg, seatRangeMm);
    return observationAllowsClearAtGeometry(
        observation,
        seatBearingDeg,
        seatRangeMm);
}

bool seatComfortablyVisible(
    uint8_t seatIndex,
    const PositionEstimate &pose)
{
    float bearingDeg = 0.0f;
    float rangeMm = -1.0f;
    seatCameraGeometry(seatIndex, pose, bearingDeg, rangeMm);
    const float bearingLimit =
        OBSTACLE_CAMERA_HORIZONTAL_FOV_DEG *
        OBSTACLE_DISCOVERY_FOV_FRACTION;
    return rangeMm >= OBSTACLE_DISCOVERY_VIEW_MIN_MM &&
           rangeMm <= OBSTACLE_DISCOVERY_VIEW_MAX_MM &&
           fabsf(bearingDeg) <= bearingLimit;
}

void updateDiscoveryCoverage(
    const ObstacleObservationResult &observation,
    const PositionEstimate &pose)
{
    if (completedLaps != 0)
        return;

    // A difficult corner station must not prevent the camera from collecting
    // evidence for another station that is already visible. Track every seat
    // independently so the two sides may be verified during different parts
    // of the camera sweep.
    for (uint8_t station = 0;
         station < OBSTACLE_SEAT_COUNT / COURSE_SEATS_PER_STATION;
         ++station)
    {
        if (stationResolved(station))
            continue;

        DiscoveryStation &coverage = discoveryStations[station];
        coverage.lastClearEvidenceMask = 0;
        for (uint8_t side = 0; side < COURSE_SEATS_PER_STATION; ++side)
        {
            if (coverage.seatObservedClear[side])
                continue;

            const uint8_t seatIndex =
                station * COURSE_SEATS_PER_STATION + side;
            const bool comfortablyVisible =
                seatComfortablyVisible(seatIndex, pose);
            const bool clearEvidence =
                comfortablyVisible &&
                observationAllowsSeatClear(observation, seatIndex, pose);
            if (clearEvidence)
                coverage.lastClearEvidenceMask |=
                    static_cast<uint8_t>(1U << side);
            if (!clearEvidence)
            {
                coverage.clearFrames[side] = 0;
                continue;
            }

            if (coverage.clearFrames[side] < 255)
                ++coverage.clearFrames[side];
            if (coverage.clearFrames[side] >=
                OBSTACLE_DISCOVERY_CLEAR_FRAMES)
                coverage.seatObservedClear[side] = true;
        }

        if (!coverage.seatObservedClear[0] ||
            !coverage.seatObservedClear[1])
            continue;

        coverage.observedClear = true;
        const uint8_t firstSeat =
            station * COURSE_SEATS_PER_STATION;
        course_map_record_clear_station(
            station / COURSE_STATIONS_PER_SECTION,
            station % COURSE_STATIONS_PER_SECTION,
            seats[firstSeat].x,
            seats[firstSeat].y,
            seats[firstSeat + 1].x,
            seats[firstSeat + 1].y);
    }
}

ObstacleTofCorrectionResult applyTofCorrectionAt(
    const PositionEstimate &pose,
    float pathDistance)
{
    ObstacleTofCorrectionResult result;
    if (!running || pathLength < 2 || !isfinite(pathDistance))
        return result;

    result.geometryReady = true;
    int cornerIndex = -1;
    for (uint8_t corner = 0; corner < 4; ++corner)
    {
        if (withinCornerGate(pathDistance, corner))
        {
            cornerIndex = corner;
            break;
        }
    }

    const PathPoint center = interpolateBaseline(pathDistance);
    const float pathHeading = center.headingDeg * PI / 180.0f;
    const float wallNormalX = -sinf(pathHeading);
    const float wallNormalY = cosf(pathHeading);
    const float robotHeading = pose.heading_deg * PI / 180.0f;
    const float robotCos = cosf(robotHeading);
    const float robotSin = sinf(robotHeading);

    float correctionX = 0.0f;
    float correctionY = 0.0f;
    uint8_t corrections = 0;
    for (uint8_t sensorIndex = 0; sensorIndex < 2; ++sensorIndex)
    {
        const bool left = sensorIndex == 0;
        const TofSensor sensor = left ? TOF_LEFT : TOF_RIGHT;
        TofDiagnosticSnapshot snapshot;
        if (!get_tof_diagnostic_snapshot(sensor, snapshot) ||
            snapshot.sequence == lastTofCorrectionSequence[sensor])
            continue;
        lastTofCorrectionSequence[sensor] = snapshot.sequence;

        const float reading = snapshot.filtered_distance_mm;
        if (left)
            result.leftReadingMm = reading;
        else
            result.rightReadingMm = reading;

        // The inside wall opens at a known corner: left for a left-turning
        // route, right for a right-turning route. This is the precomputed
        // corner-side geometry gate; no measurement-jump detector is used.
        const bool recedingAtCorner =
            cornerIndex >= 0 &&
            (left
                 ? corners[cornerIndex].recedesOnLeft
                 : corners[cornerIndex].recedesOnRight);
        if (left)
            result.leftCornerGated = recedingAtCorner;
        else
            result.rightCornerGated = recedingAtCorner;
        if (recedingAtCorner)
            continue;
        if (reading <= 0.0f ||
            reading >= OBSTACLE_TOF_CORRECTION_MAX_RANGE_MM)
        {
            continue;
        }

        const float localX = left
                                 ? OBSTACLE_TOF_LEFT_LOCAL_X_MM
                                 : OBSTACLE_TOF_RIGHT_LOCAL_X_MM;
        const float localY = left
                                 ? OBSTACLE_TOF_LEFT_LOCAL_Y_MM
                                 : OBSTACLE_TOF_RIGHT_LOCAL_Y_MM;
        const float sensorX =
            pose.x_mm + localX * robotCos - localY * robotSin;
        const float sensorY =
            pose.y_mm + localX * robotSin + localY * robotCos;

        const float sensorSide = left ? 1.0f : -1.0f;
        const float sensorRayHeading =
            robotHeading + sensorSide * PI * 0.5f;
        const float measuredWallX =
            sensorX + reading * cosf(sensorRayHeading);
        const float measuredWallY =
            sensorY + reading * sinf(sensorRayHeading);
        const float expectedWallX =
            center.x + sensorSide *
                           OBSTACLE_CORRIDOR_HALF_WIDTH_MM * wallNormalX;
        const float expectedWallY =
            center.y + sensorSide *
                           OBSTACLE_CORRIDOR_HALF_WIDTH_MM * wallNormalY;

        const float lateralResidual =
            (expectedWallX - measuredWallX) * wallNormalX +
            (expectedWallY - measuredWallY) * wallNormalY;
        if (left)
            result.leftResidualMm = lateralResidual;
        else
            result.rightResidualMm = lateralResidual;
        if (!isfinite(lateralResidual) ||
            fabsf(lateralResidual) >
                OBSTACLE_TOF_CORRECTION_MAX_RESIDUAL_MM)
        {
            if (left)
                result.leftResidualGated = true;
            else
                result.rightResidualGated = true;
            continue;
        }

        const float lateralError = clampFloat(
            lateralResidual * OBSTACLE_TOF_CORRECTION_GAIN,
            -OBSTACLE_TOF_CORRECTION_MAX_STEP_MM,
            OBSTACLE_TOF_CORRECTION_MAX_STEP_MM);
        correctionX += wallNormalX * lateralError;
        correctionY += wallNormalY * lateralError;
        if (left)
            result.leftUsed = true;
        else
            result.rightUsed = true;
        ++corrections;
    }

    if (corrections > 0)
    {
        result.correctionXmm = correctionX / corrections;
        result.correctionYmm = correctionY / corrections;
        position_apply_xy_correction(
            result.correctionXmm,
            result.correctionYmm);
    }
    return result;
}

PathPoint findLookahead(
    const PathPoint *path,
    const PositionEstimate &pose,
    float lookaheadMm)
{
    float accumulated = 0.0f;
    uint16_t index = progressIndex;
    while (accumulated < lookaheadMm)
    {
        const uint16_t next = (index + 1) % pathLength;
        accumulated += hypotf(
            path[next].x - path[index].x,
            path[next].y - path[index].y);
        index = next;
        if (index == progressIndex)
            break;
    }
    return path[index];
}
} // namespace

void obstacle_path_reset()
{
    pathLength = 0;
    progressIndex = 0;
    completedLaps = 0;
    routeTurnSign = 1;
    running = false;
    finished = false;
    optimizedBuilt = false;
    runtimeTestMode = false;
    runtimeLapTarget = 3;
    runtimeSpeedCapMmS = 0.0f;
    loopLengthMm = 0.0f;
    firstCornerDistanceMm = OBSTACLE_STRAIGHT_LENGTH_MM * 0.5f;
    injectionCount = 0;
    discoveryBlocked = false;
    discoveryBlockedStation = -1;
    lastDiscoveryTargetNudgeDeg = 0.0f;
    discoveryScanStation = -1;
    discoveryScanSide = -1;
    lastDiscoveryNudgeUpdateMs = 0;
    lastDiscoveryObservation = ObstacleObservationResult();
    memset(lastTofCorrectionSequence, 0, sizeof(lastTofCorrectionSequence));
    lastTofCorrectionResult = ObstacleTofCorrectionResult{};
    memset(seats, 0, sizeof(seats));
    memset(discoveryStations, 0, sizeof(discoveryStations));
}

void obstacle_path_start(
    int8_t turn_sign,
    bool test_mode,
    float first_corner_distance_mm,
    uint8_t lap_target,
    float speed_cap_mm_s)
{
    obstacle_path_reset();
    runtimeTestMode = test_mode;
    runtimeLapTarget = lap_target > 0
        ? lap_target
        : (runtimeTestMode ? 1 : 3);
    runtimeSpeedCapMmS = isfinite(speed_cap_mm_s) && speed_cap_mm_s > 0.0f
        ? speed_cap_mm_s
        : 0.0f;
    routeTurnSign = turn_sign < 0 ? -1 : 1;
    firstCornerDistanceMm = clampFloat(
        first_corner_distance_mm,
        50.0f,
        OBSTACLE_STRAIGHT_LENGTH_MM - 50.0f);
    PositionEstimate anchor;
    if (runtimeTestMode)
    {
        // Bench/empty-track tests remain portable: their path begins wherever
        // the robot was placed instead of requiring a physical field origin.
        anchor = get_position_struct();
    }
    else
    {
        // The calibrated parking exit establishes the production field pose.
        // Subsequent encoder/gyro odometry and ToF corrections now live in the
        // same fixed frame as every path point and pillar seat.
        anchor = nominalFieldStartPose(
            routeTurnSign,
            firstCornerDistanceMm);
        position_reset(
            anchor.x_mm,
            anchor.y_mm,
            anchor.heading_deg);
        anchor = get_position_struct();

        Serial.print("[FIELD] Nominal exit pose x=");
        Serial.print(anchor.x_mm, 1);
        Serial.print(" y=");
        Serial.print(anchor.y_mm, 1);
        Serial.print(" heading=");
        Serial.println(anchor.heading_deg, 1);
    }

    float x = 0.0f;
    float y = 0.0f;
    float heading = 0.0f;
    float distance = 0.0f;
    appendLocalPoint(x, y, 0.0f, distance, anchor);
    appendStraight(
        firstCornerDistanceMm,
        x, y, heading, distance, anchor);
    for (uint8_t corner = 0; corner < 4; ++corner)
    {
        appendCorner(corner, x, y, heading, distance, anchor);
        appendStraight(
            corner == 3
                ? OBSTACLE_STRAIGHT_LENGTH_MM - firstCornerDistanceMm
                : OBSTACLE_STRAIGHT_LENGTH_MM,
            x, y, heading, distance, anchor);
    }

    loopLengthMm = distance;
    memcpy(livePath, baselinePath, sizeof(PathPoint) * pathLength);
    memcpy(optimizedPath, baselinePath, sizeof(PathPoint) * pathLength);
    initializeSeats();
    recomputeSpeedProfile(baselinePath);
    recomputeSpeedProfile(livePath);
    running = pathLength > 2;

    Serial.print("[PATH] Known geometry ready points=");
    Serial.print(pathLength);
    Serial.print(" length_mm=");
    Serial.print(loopLengthMm, 0);
    Serial.print(" turns=");
    Serial.print(routeTurnSign > 0 ? "LEFT" : "RIGHT");
    Serial.print(" first_corner_mm=");
    Serial.println(firstCornerDistanceMm, 0);
}

void obstacle_path_update(bool new_camera_frame)
{
    if (!running || finished)
        return;

    const PathPoint *path =
        optimizedBuilt ? optimizedPath : livePath;
    PositionEstimate pose = get_position_struct();
    updateProgress(path, pose);
    if (finished)
        return;

    if (!runtimeTestMode && new_camera_frame)
    {
        if (completedLaps == 0)
        {
            const ObstacleObservationResult observation =
                obstacle_path_observe(getLargestValidObstacle());
            lastDiscoveryObservation = observation;
            updateDiscoveryCoverage(
                observation,
                pose);
        }
    }

    if (!runtimeTestMode)
    {
        const ObstacleTofCorrectionResult correction = applyTofCorrectionAt(
            pose,
            baselinePath[progressIndex].distanceMm);
        if (correction.leftReadingMm > 0.0f ||
            correction.rightReadingMm > 0.0f)
            lastTofCorrectionResult = correction;
    }
    pose = get_position_struct();

    const PathPoint &progress = path[progressIndex];
    const bool constrainedGeometry =
        nearCorner(progress.distanceMm);
    float lookahead =
        OBSTACLE_LOOKAHEAD_MIN_MM +
        (OBSTACLE_LOOKAHEAD_MAX_MM - OBSTACLE_LOOKAHEAD_MIN_MM) *
            clampFloat(
                (progress.speedMmS - OBSTACLE_PATH_MIN_SPEED) /
                    (OBSTACLE_PATH_MAX_SPEED - OBSTACLE_PATH_MIN_SPEED),
                0.0f,
                1.0f);
    if (constrainedGeometry)
        lookahead *= OBSTACLE_LOOKAHEAD_CORNER_SCALE;

    PathPoint target = findLookahead(path, pose, lookahead);
    if (!runtimeTestMode)
        applyDiscoveryTargetNudge(target, pose);
    const float dx = target.x - pose.x_mm;
    const float dy = target.y - pose.y_mm;
    const float heading = pose.heading_deg * PI / 180.0f;
    const float localY = -dx * sinf(heading) + dy * cosf(heading);
    const float targetDistanceSquared = fmaxf(1.0f, dx * dx + dy * dy);
    const float curvature = 2.0f * localY / targetDistanceSquared;
    // Positive geometric curvature is left; positive servo command is right.
    const float steering = clampFloat(
        -atanf(OBSTACLE_WHEELBASE_MM * curvature) * 180.0f / PI,
        -OBSTACLE_MAX_PURSUIT_STEERING_DEG,
        OBSTACLE_MAX_PURSUIT_STEERING_DEG);

    set_steering(static_cast<int>(steering));
    float commandedSpeed = runtimeTestMode
        ? fminf(progress.speedMmS, OBSTACLE_PATH_TEST_MAX_SPEED_MM_S)
        : progress.speedMmS;
    if (runtimeSpeedCapMmS > 0.0f)
        commandedSpeed = fminf(commandedSpeed, runtimeSpeedCapMmS);
    float safeSpeed = commandedSpeed;
    if (!runtimeTestMode && completedLaps == 0)
    {
        float unresolvedForward = 0.0f;
        const int unresolvedStation =
            nearestUpcomingUnresolvedStation(unresolvedForward);
        if (unresolvedStation >= 0)
        {
            if (unresolvedForward <=
                OBSTACLE_DISCOVERY_HOLD_DISTANCE_MM)
            {
                safeSpeed = 0.0f;
                if (!discoveryBlocked)
                {
                    discoveryBlocked = true;
                    discoveryBlockedStation = unresolvedStation;
                    Serial.print("[PATH] Perception blocked at S");
                    Serial.print(unresolvedStation /
                                 COURSE_STATIONS_PER_SECTION);
                    Serial.print(" station=");
                    Serial.print(unresolvedStation %
                                 COURSE_STATIONS_PER_SECTION);
                    Serial.print(" forward_mm=");
                    Serial.println(unresolvedForward, 0);
                }
            }
            else if (unresolvedForward <=
                     OBSTACLE_DISCOVERY_SLOW_DISTANCE_MM)
                safeSpeed = fminf(
                    safeSpeed,
                    OBSTACLE_DISCOVERY_SPEED_MM_S);
        }
    }
    set_speed(static_cast<int>(safeSpeed));
}

bool obstacle_path_started()
{
    return running;
}

bool obstacle_path_complete()
{
    return finished;
}

bool obstacle_path_perception_blocked()
{
    return discoveryBlocked;
}

int8_t obstacle_path_blocked_station()
{
    return discoveryBlockedStation;
}

float obstacle_path_discovery_target_nudge_deg()
{
    return lastDiscoveryTargetNudgeDeg;
}

int8_t obstacle_path_discovery_scan_seat()
{
    if (discoveryScanStation < 0 || discoveryScanSide == -1)
        return -1;
    if (discoveryScanSide == -2)
        return -2;
    return discoveryScanStation * COURSE_SEATS_PER_STATION +
           discoveryScanSide;
}

bool obstacle_path_get_discovery_telemetry(
    ObstacleDiscoveryTelemetry &telemetry)
{
    telemetry = ObstacleDiscoveryTelemetry();
    if (!running || discoveryScanStation < 0 ||
        discoveryScanStation >= OBSTACLE_SEAT_COUNT / 2)
        return false;

    telemetry.station = discoveryScanStation;
    const DiscoveryStation &coverage =
        discoveryStations[discoveryScanStation];
    telemetry.clearEvidenceMask = coverage.lastClearEvidenceMask;
    const PositionEstimate pose = get_position_struct();
    for (uint8_t side = 0; side < COURSE_SEATS_PER_STATION; ++side)
    {
        telemetry.clearFrames[side] = coverage.clearFrames[side];
        const uint8_t seatIndex =
            discoveryScanStation * COURSE_SEATS_PER_STATION + side;
        seatCameraGeometry(
            seatIndex,
            pose,
            telemetry.seatBearingDeg[side],
            telemetry.seatRangeMm[side]);
        if (seatComfortablyVisible(seatIndex, pose))
            telemetry.visibleMask |= static_cast<uint8_t>(1U << side);
    }

    telemetry.observationStatus = lastDiscoveryObservation.status;
    telemetry.observationSeat = lastDiscoveryObservation.seatId;
    telemetry.left = lastDiscoveryObservation.left;
    telemetry.top = lastDiscoveryObservation.top;
    telemetry.right = lastDiscoveryObservation.right;
    telemetry.bottom = lastDiscoveryObservation.bottom;
    telemetry.bearingDeg = lastDiscoveryObservation.bearingDeg;
    telemetry.rangeMm = lastDiscoveryObservation.rangeMm;
    return true;
}

uint8_t obstacle_path_lap()
{
    return completedLaps;
}

uint16_t obstacle_path_progress_index()
{
    return progressIndex;
}

uint16_t obstacle_path_waypoint_count()
{
    return pathLength;
}

float obstacle_path_loop_length_mm()
{
    return loopLengthMm;
}

float obstacle_path_cross_track_error_mm()
{
    if (!running || pathLength == 0)
        return -1.0f;
    const PositionEstimate pose = get_position_struct();
    const PathPoint *path = optimizedBuilt ? optimizedPath : livePath;
    return hypotf(
        pose.x_mm - path[progressIndex].x,
        pose.y_mm - path[progressIndex].y);
}

float obstacle_path_heading_error_deg()
{
    if (!running || pathLength == 0)
        return 0.0f;
    const PositionEstimate pose = get_position_struct();
    return wrap180(
        pose.heading_deg - baselinePath[progressIndex].headingDeg);
}

bool obstacle_path_geometry_valid()
{
    if (!running || pathLength < 3 ||
        pathLength > OBSTACLE_MAX_PATH_WAYPOINTS)
        return false;

    const float expectedLength =
        4.0f * OBSTACLE_STRAIGHT_LENGTH_MM +
        2.0f * PI * OBSTACLE_CORNER_RADIUS_MM;
    const float closureError = hypotf(
        baselinePath[pathLength - 1].x - baselinePath[0].x,
        baselinePath[pathLength - 1].y - baselinePath[0].y);
    return fabsf(loopLengthMm - expectedLength) <= 1.0f &&
           closureError <= 1.0f;
}

ObstacleObservationResult obstacle_path_observe(const Blob *blob)
{
    ObstacleObservationResult result;
    result.injectionCount = injectionCount;
    if (blob == nullptr || !blob->found)
    {
        expirePendingVotes();
        return result;
    }

    result.color = blob->color;
    result.left = blob->minX;
    result.top = blob->minY;
    result.right = blob->maxX;
    result.bottom = blob->maxY;
    result.productionValid = obstacle_blob_valid_for_acquisition(blob);
    if (!result.productionValid)
    {
        expirePendingVotes();
        result.status = OBSTACLE_OBSERVATION_REJECTED_BLOB;
        return result;
    }

    const PositionEstimate pose = get_position_struct();
    result.robotXmm = pose.x_mm;
    result.robotYmm = pose.y_mm;
    result.robotHeadingDeg = pose.heading_deg;
    result.bearingDeg = obstacle_camera_bearing_deg(blob);
    result.rangeMm = obstacle_estimate_camera_range_mm(blob);
    if (!isfinite(result.rangeMm) || result.rangeMm <= 0.0f)
    {
        expirePendingVotes();
        result.status = OBSTACLE_OBSERVATION_INVALID_RANGE;
        return result;
    }

    const float robotHeading = pose.heading_deg * PI / 180.0f;
    result.cameraXmm =
        pose.x_mm +
        OBSTACLE_CAMERA_LOCAL_X_MM * cosf(robotHeading) -
        OBSTACLE_CAMERA_LOCAL_Y_MM * sinf(robotHeading);
    result.cameraYmm =
        pose.y_mm +
        OBSTACLE_CAMERA_LOCAL_X_MM * sinf(robotHeading) +
        OBSTACLE_CAMERA_LOCAL_Y_MM * cosf(robotHeading);
    const float globalBearing =
        (pose.heading_deg + result.bearingDeg) * PI / 180.0f;
    result.sightingXmm =
        result.cameraXmm + result.rangeMm * cosf(globalBearing);
    result.sightingYmm =
        result.cameraYmm + result.rangeMm * sinf(globalBearing);

    result.seatId = static_cast<int8_t>(nearestSeatIndex(
        result.sightingXmm,
        result.sightingYmm,
        &result.snapErrorMm));
    if (result.seatId < 0)
    {
        expirePendingVotes();
        result.status = OBSTACLE_OBSERVATION_NO_SEAT;
        return result;
    }

    if (!runtimeTestMode)
    {
        // Permit any upcoming station within the calibrated camera horizon.
        // The seat snap still rejects a noisy estimate elsewhere on the field,
        // while a difficult corner station no longer masks another station.
        const uint8_t observedStation = stationIndexForSeat(
            static_cast<uint8_t>(result.seatId));
        const float currentDistance =
            baselinePath[progressIndex].distanceMm;
        const float observedForward = cyclicDistanceForward(
            currentDistance,
            seats[observedStation * COURSE_SEATS_PER_STATION].pathDistanceMm);
        const float maximumRelevantForward =
            OBSTACLE_CAMERA_LOCAL_X_MM +
            OBSTACLE_DISCOVERY_VIEW_MAX_MM +
            OBSTACLE_SEAT_SNAP_RADIUS_MM;
        if (observedForward <= 1.0f ||
            observedForward > maximumRelevantForward)
        {
            expirePendingVotes();
            result.status = OBSTACLE_OBSERVATION_NO_SEAT;
            result.seatId = -1;
            return result;
        }
    }

    prepareConsecutiveVote(result.seatId, blob->color);
    CandidateSeat &seat = seats[result.seatId];
    if (seat.confirmed)
    {
        result.status = OBSTACLE_OBSERVATION_ALREADY_CONFIRMED;
    }
    else if (recordSeatVote(seat, blob->color))
    {
        result.status = OBSTACLE_OBSERVATION_CONFIRMED;
        displaceForSeat(livePath, seat, OBSTACLE_LAP1_CLEARANCE_MM);
        seat.injected = true;
        if (injectionCount < 65535)
            ++injectionCount;

        const uint16_t center = nearestPathIndex(
            baselinePath, seat.x, seat.y, 0, pathLength);
        result.peakDisplacementMm = hypotf(
            livePath[center].x - baselinePath[center].x,
            livePath[center].y - baselinePath[center].y);
        result.movementCircleClearanceMm =
            hypotf(
                livePath[center].x - seat.x,
                livePath[center].y - seat.y) -
            42.5f;

        Serial.print("[PATH] Live avoidance injected seat=");
        Serial.print(result.seatId);
        Serial.print(" color=");
        Serial.println(seat.red ? "RED" : "GREEN");

        const uint8_t seatIndex = static_cast<uint8_t>(result.seatId);
        course_map_record_seat_obstacle(
            seatIndex / 6,
            (seatIndex % 6) / 2,
            seatIndex % 2,
            seat.red ? ColorType::RED : ColorType::GREEN,
            seat.x,
            seat.y);
    }
    else
    {
        result.status = OBSTACLE_OBSERVATION_VOTE;
    }

    result.redVotes = seat.redVotes;
    result.greenVotes = seat.greenVotes;
    result.confirmed = seat.confirmed;
    result.injected = seat.injected;
    result.injectionCount = injectionCount;
    if (seat.confirmed)
        result.passSide = seat.red ? 'R' : 'L';
    return result;
}

uint8_t obstacle_path_seat_count()
{
    return OBSTACLE_SEAT_COUNT;
}

bool obstacle_path_get_seat(uint8_t seat_id, ObstacleSeatInfo &info)
{
    if (!running || seat_id >= OBSTACLE_SEAT_COUNT)
        return false;

    const CandidateSeat &seat = seats[seat_id];
    info.id = seat_id;
    info.section = seat_id / 6;
    info.station = (seat_id % 6) / 2;
    info.side = (seat_id % 2) == 0 ? 'R' : 'L';
    info.xMm = seat.x;
    info.yMm = seat.y;
    info.headingDeg = seat.headingDeg;
    info.pathDistanceMm = seat.pathDistanceMm;
    info.lateralMm = seat.lateralMm;
    info.redVotes = seat.redVotes;
    info.greenVotes = seat.greenVotes;
    info.confirmed = seat.confirmed;
    info.red = seat.red;
    info.injected = seat.injected;
    return true;
}

void obstacle_path_clear_observations()
{
    if (!running)
        return;
    memcpy(livePath, baselinePath, sizeof(PathPoint) * pathLength);
    memcpy(optimizedPath, baselinePath, sizeof(PathPoint) * pathLength);
    optimizedBuilt = false;
    injectionCount = 0;
    for (uint8_t i = 0; i < OBSTACLE_SEAT_COUNT; ++i)
    {
        seats[i].redVotes = 0;
        seats[i].greenVotes = 0;
        seats[i].lastVoteMs = 0;
        seats[i].confirmed = false;
        seats[i].red = false;
        seats[i].injected = false;
    }
}

uint16_t obstacle_path_injection_count()
{
    return injectionCount;
}

bool obstacle_path_prepare_tof_diagnostic(
    int8_t turn_sign,
    bool corner,
    uint8_t index,
    float initial_lateral_mm,
    float &path_distance_mm,
    float &center_x_mm,
    float &center_y_mm,
    float &heading_deg)
{
    if ((corner && index >= 4) || (!corner && index >= 4) ||
        !isfinite(initial_lateral_mm) ||
        fabsf(initial_lateral_mm) > OBSTACLE_CORRIDOR_HALF_WIDTH_MM)
        return false;

    const int8_t direction = turn_sign < 0 ? -1 : 1;
    const float firstCorner = direction > 0
        ? OBSTACLE_PARKING_TO_FIRST_CORNER_CCW_MM
        : OBSTACLE_PARKING_TO_FIRST_CORNER_CW_MM;
    obstacle_path_start(direction, false, firstCorner);
    if (!running)
        return false;

    PathPoint center;
    if (corner)
    {
        path_distance_mm =
            0.5f * (corners[index].pathStartMm + corners[index].pathEndMm);
        center = interpolateBaseline(path_distance_mm);
    }
    else
    {
        const CandidateSeat &seat = seats[index * 6 + 2];
        path_distance_mm = seat.pathDistanceMm;
        center = interpolateBaseline(path_distance_mm);
    }

    const float headingRad = center.headingDeg * PI / 180.0f;
    const float normalX = -sinf(headingRad);
    const float normalY = cosf(headingRad);
    center_x_mm = center.x;
    center_y_mm = center.y;
    heading_deg = center.headingDeg;
    position_reset(
        center.x + normalX * initial_lateral_mm,
        center.y + normalY * initial_lateral_mm,
        center.headingDeg);
    return true;
}

ObstacleTofCorrectionResult obstacle_path_apply_tof_diagnostic(
    float path_distance_mm)
{
    return applyTofCorrectionAt(
        get_position_struct(),
        path_distance_mm);
}

ObstacleTofCorrectionResult obstacle_path_last_tof_correction()
{
    return lastTofCorrectionResult;
}

bool obstacle_path_geometry_preflight()
{
    const float normalWallResidualMm = 100.0f;
    const float pillarLikeResidualMm = -(
        OBSTACLE_CORRIDOR_HALF_WIDTH_MM -
        fabsf(OBSTACLE_TOF_RIGHT_LOCAL_Y_MM) - 108.0f);
    if (!obstacle_path_geometry_valid() ||
        OBSTACLE_SEAT_COUNT != 24 ||
        OBSTACLE_SEAT_CONFIRM_VOTES != 2 ||
        fabsf(normalWallResidualMm) >
            OBSTACLE_TOF_CORRECTION_MAX_RESIDUAL_MM ||
        fabsf(-normalWallResidualMm) >
            OBSTACLE_TOF_CORRECTION_MAX_RESIDUAL_MM ||
        fabsf(pillarLikeResidualMm) <=
            OBSTACLE_TOF_CORRECTION_MAX_RESIDUAL_MM)
        return false;

    Blob imageLeft;
    imageLeft.found = true;
    imageLeft.centerX = 100;
    Blob imageRight;
    imageRight.found = true;
    imageRight.centerX = 220;
    if (obstacle_camera_bearing_deg(&imageLeft) <= 0.0f ||
        obstacle_camera_bearing_deg(&imageRight) >= 0.0f)
        return false;

    Blob pillarShape;
    pillarShape.found = true;
    pillarShape.color = ColorType::RED;
    pillarShape.centerX = 100;
    pillarShape.minX = 70;
    pillarShape.maxX = 110;
    pillarShape.minY = 80;
    pillarShape.maxY = 166;
    pillarShape.area = 1200;
    Blob floorFragment = pillarShape;
    floorFragment.minY = 132;
    floorFragment.maxY = 174;
    if (!obstacle_blob_valid_for_acquisition(&pillarShape) ||
        obstacle_blob_valid_for_acquisition(&floorFragment))
        return false;

    ObstacleObservationResult noBlob;
    if (!observationAllowsClearAtGeometry(noBlob, 20.0f, 439.0f))
        return false;

    ObstacleObservationResult validBlob;
    validBlob.status = OBSTACLE_OBSERVATION_NO_SEAT;
    validBlob.productionValid = true;
    validBlob.left = 68;
    validBlob.right = 82;
    validBlob.rangeMm = 912.0f;
    // Reproduce log_46: a far blob on the same bearing proves the nearer seat
    // clear, while a nearby overlapping blob must still block that inference.
    if (!observationAllowsClearAtGeometry(validBlob, 20.3f, 439.0f))
        return false;
    validBlob.rangeMm = 520.0f;
    if (observationAllowsClearAtGeometry(validBlob, 20.3f, 439.0f))
        return false;
    validBlob.rangeMm = 439.0f;
    validBlob.left = 200;
    validBlob.right = 220;
    if (!observationAllowsClearAtGeometry(validBlob, 20.3f, 439.0f))
        return false;
    validBlob.productionValid = false;
    if (observationAllowsClearAtGeometry(validBlob, 20.3f, 439.0f))
        return false;

    const float inside = OBSTACLE_SEAT_SNAP_RADIUS_MM - 1.0f;
    const float outside = OBSTACLE_SEAT_SNAP_RADIUS_MM + 1.0f;
    for (uint8_t i = 0; i < OBSTACLE_SEAT_COUNT; ++i)
    {
        const CandidateSeat &seat = seats[i];
        if (!isfinite(seat.x) || !isfinite(seat.y) ||
            !isfinite(seat.headingDeg) || !isfinite(seat.pathDistanceMm) ||
            seat.pathDistanceMm < 0.0f || seat.pathDistanceMm >= loopLengthMm ||
            fabsf(fabsf(seat.lateralMm) - OBSTACLE_SEAT_LATERAL_MM) > 0.1f ||
            nearestSeatIndex(seat.x, seat.y) != i)
            return false;

        const uint8_t paired = i ^ 1;
        const float heading = seat.headingDeg * PI / 180.0f;
        const float normalX = -sinf(heading);
        const float normalY = cosf(heading);
        const float centerX = seat.x - normalX * seat.lateralMm;
        const float centerY = seat.y - normalY * seat.lateralMm;
        const CandidateSeat &opposite = seats[paired];
        if (fabsf(opposite.pathDistanceMm - seat.pathDistanceMm) > 0.1f ||
            hypotf(
                centerX - (opposite.x - normalX * opposite.lateralMm),
                centerY - (opposite.y - normalY * opposite.lateralMm)) > 0.1f)
            return false;
        const uint8_t station = (i % 6) / 2;
        if (station < 2 &&
            fabsf(cyclicDistanceForward(
                seat.pathDistanceMm,
                seats[i + 2].pathDistanceMm) -
                OBSTACLE_STRAIGHT_LENGTH_MM * 0.5f) > 0.1f)
            return false;

        for (uint8_t other = i + 1; other < OBSTACLE_SEAT_COUNT; ++other)
        {
            if (distanceSquared(
                    seat.x, seat.y, seats[other].x, seats[other].y) < 1.0f)
                return false;
        }

        const float outward = seat.lateralMm < 0.0f ? -1.0f : 1.0f;
        const float outwardX = normalX * outward;
        const float outwardY = normalY * outward;
        if (nearestSeatIndex(
                seat.x + outwardX * inside,
                seat.y + outwardY * inside) != i ||
            nearestSeatIndex(
                seat.x + outwardX * outside,
                seat.y + outwardY * outside) >= 0)
            return false;
    }

    CandidateSeat redTest = seats[0];
    CandidateSeat greenTest = seats[0];
    if (recordSeatVote(redTest, ColorType::RED) || redTest.confirmed ||
        !recordSeatVote(redTest, ColorType::RED) || !redTest.confirmed ||
        !redTest.red || recordSeatVote(redTest, ColorType::RED))
        return false;
    if (recordSeatVote(greenTest, ColorType::GREEN) || greenTest.confirmed ||
        !recordSeatVote(greenTest, ColorType::GREEN) || !greenTest.confirmed ||
        greenTest.red || recordSeatVote(greenTest, ColorType::GREEN))
        return false;

    prepareConsecutiveVote(0, ColorType::RED);
    if (recordSeatVote(seats[0], ColorType::RED))
        return false;
    prepareConsecutiveVote(1, ColorType::RED);
    if (recordSeatVote(seats[1], ColorType::RED) || seats[0].redVotes != 0)
        return false;
    prepareConsecutiveVote(0, ColorType::RED);
    if (recordSeatVote(seats[0], ColorType::RED) || seats[0].redVotes != 1)
        return false;
    clearPendingVotes();
    if (seats[0].redVotes != 0 || seats[1].redVotes != 0)
        return false;

    return fabsf(
               targetLateralForSeat(redTest, OBSTACLE_LAP1_CLEARANCE_MM) -
               redTest.lateralMm + OBSTACLE_LAP1_CLEARANCE_MM) < 0.1f &&
           fabsf(
               targetLateralForSeat(greenTest, OBSTACLE_LAP1_CLEARANCE_MM) -
               greenTest.lateralMm - OBSTACLE_LAP1_CLEARANCE_MM) < 0.1f;
}
