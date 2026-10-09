#include "obstacle_path.h"

#include <math.h>
#include <string.h>

#include "config.h"
#include "course_map.h"
#include "obstacle_section_layout.h"
#include "motor_control.h"
#include "obstacle.h"
#include "position_estimator.h"
#include "sensors.h"
#include "vision.h"
#include "logger.h"
#include "run_telemetry.h"
#include "parking_start_footprint.h"
#include "ackermann_kinematics.h"

#undef Serial
#define Serial robot_logger

void processGreenSeatCandidates(
    const PositionEstimate &pose,
    const ObstacleObservationResult &normalObservation);

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
    uint8_t greenSeatCandidateFrames = 0;
    unsigned long lastVoteMs = 0;
    unsigned long lastGreenSeatCandidateMs = 0;
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

enum ParkingEntryDrivePhase : uint8_t
{
    PARKING_ENTRY_STRAIGHT_STEER_SETTLE,
    PARKING_ENTRY_REVERSE_STRAIGHT,
    PARKING_ENTRY_ARC_STEER_SETTLE,
    PARKING_ENTRY_REVERSE_ARC
};

enum ParkingEntryScoutPhase : uint8_t
{
    PARKING_ENTRY_SCOUT_STEER_SETTLE,
    PARKING_ENTRY_SCOUT_REVERSE,
    PARKING_ENTRY_SCOUT_OBSERVE,
    PARKING_ENTRY_SCOUT_RETURN_BRAKE,
    PARKING_ENTRY_SCOUT_FORWARD_RETURN,
    PARKING_ENTRY_SCOUT_COMPLETE_BRAKE
};

PathPoint baselinePath[OBSTACLE_MAX_PATH_WAYPOINTS];
PathPoint livePath[OBSTACLE_MAX_PATH_WAYPOINTS];
PathPoint optimizedPath[OBSTACLE_MAX_PATH_WAYPOINTS];
PathPoint smoothingBuffer[OBSTACLE_MAX_PATH_WAYPOINTS];
PathPoint parkingEntryPath[OBSTACLE_PARKING_ENTRY_MAX_WAYPOINTS];
PathPoint parkingEntryConnector[
    OBSTACLE_PARKING_ENTRY_CONNECTOR_MAX_WAYPOINTS];
CandidateSeat seats[OBSTACLE_SEAT_COUNT];
DiscoveryStation discoveryStations[OBSTACLE_SEAT_COUNT / 2];
bool greenSeatCandidateThisFrame[OBSTACLE_SEAT_COUNT] = {};
bool oppositeSeatConflictReported[OBSTACLE_SEAT_COUNT] = {};
uint32_t lastGreenSeatProcessingUs = 0;
ObstacleClearanceSample plannedClearanceAtInjection[OBSTACLE_SEAT_COUNT];
bool plannedClearanceSnapshotValid[OBSTACLE_SEAT_COUNT] = {};

uint16_t pathLength = 0;
uint16_t progressIndex = 0;
uint8_t completedLaps = 0;
bool telemetryRouteDirty = true;
uint32_t laterTrackingTraceMs = 0;
uint16_t laterTrackingTraceCount = 0;
bool parkingStartRoiLogged = false;
int8_t routeTurnSign = 1;
bool running = false;
bool finished = false;
bool optimizedBuilt = false;
bool lapBoundaryPending = false;
// A start connector may join just BEFORE phase zero. That first seam crossing
// is not a lap; require progress through the middle half of the course first.
bool lapCountingArmed = false;
bool laterLapPlanRejected = false;
bool lapBoundaryHoldLogged = false;
bool lapFinishPending = false;
bool runtimeTestMode = false;
bool earlyMiddleViewActive[OBSTACLE_SEAT_COUNT] = {};
ObstacleSectionLayoutMode sectionLayoutMode = OBSTACLE_STARTUP_CHECK_ALL_STATIONS
    ? OBSTACLE_SECTION_LAYOUT_CHECK_ALL
    : OBSTACLE_SECTION_LAYOUT_OFFICIAL;
uint8_t runtimeLapTarget = 3;
float runtimeSpeedCapMmS = 0.0f;
float loopLengthMm = 0.0f;
float firstCornerDistanceMm = OBSTACLE_STRAIGHT_LENGTH_MM * 0.5f;
uint16_t injectionCount = 0;
bool discoveryBlocked = false;
int8_t discoveryBlockedStation = -1;
bool discoveryHolding = false;
int8_t discoveryHoldStation = -1;
uint32_t discoveryHoldStartMs = 0;
float lastDiscoveryTargetNudgeDeg = 0.0f;
int8_t discoveryScanStation = -1;
int8_t discoveryScanSide = -1;
int8_t lastConfirmedSeatIndex = -1;
bool extremeAdjacentReleasePending = false;
int8_t deferredInjectionSeatIndex = -1;
uint32_t lastDiscoveryNudgeUpdateMs = 0;
ObstacleObservationResult lastDiscoveryObservation;
PositionEstimate lastDiscoveryCoveragePose;
uint32_t lastDiscoveryCoverageMs = 0;
uint32_t lastDiscoveryTraceMs = 0;
uint8_t discoveryTraceCount = 0;
// One bounded straight peek per corner station during discovery lap 1.
enum CornerViewPhase : uint8_t { CORNER_VIEW_IDLE, CORNER_VIEW_SETTLE,
    CORNER_VIEW_REVERSE, CORNER_VIEW_OBSERVE, CORNER_VIEW_RETURN,
    CORNER_VIEW_RETURN_BRAKE, CORNER_VIEW_EXTRA_SETTLE, CORNER_VIEW_EXTRA_REVERSE,
    CORNER_VIEW_EXTRA_OBSERVE, CORNER_VIEW_EXTRA_RETURN_SETTLE,
    CORNER_VIEW_EXTRA_FORWARD, CORNER_VIEW_EXTRA_BRAKE, CORNER_VIEW_LOCKED };
CornerViewPhase cornerViewPhase = CORNER_VIEW_IDLE;
bool cornerViewAttempted[OBSTACLE_SEAT_COUNT / 2] = {};
uint8_t cornerViewStation = 0;
uint32_t cornerViewPhaseMs = 0;
PositionEstimate cornerViewOrigin;
float cornerViewOriginEncoder = 0.0f, cornerViewReturnEncoder = 0.0f;
float cornerViewDistanceMm = 0.0f, cornerViewMeasuredReverseMm = 0.0f;
float cornerViewRequestedMinRange = OBSTACLE_DISCOVERY_VIEW_MIN_MM;
uint8_t cornerViewStoppedFrames = 0;
uint8_t cornerViewObservationTraceCount = 0;
bool cornerViewCanResume = false;
bool cornerViewExtraUsed = false;
int cornerViewExtraSteering = 0;
PositionEstimate cornerViewExtraOrigin, cornerViewExtraReturnPose;
float cornerViewExtraStartEncoder = 0, cornerViewExtraReturnEncoder = 0;
float cornerViewExtraMeasuredMm = 0;
ObstacleTofCorrectionResult lastTofCorrectionResult;
uint32_t tofCorrectionSequence = 0;
CornerGeometry corners[4];
uint32_t lastTofCorrectionSequence[TOF_COUNT] = {};
uint8_t parkingEntryLength = 0;
bool parkingSectionInnerSeatsOnly = false;
bool parkingCwShortStart = false;
int8_t parkingCwStoredSeat = -1;
bool parkingCcwShortStart = false;
bool parkingCcwFirstEdgeStart = false;
// Separate station bits: the left/back and middle/back may both be observed
// even in a surprise diagnostic. One scalar seat would lose the first record.
uint8_t parkingCcwStoredSeatMask = 0;

bool parkingShortStart()
{
    return parkingCwShortStart || parkingCcwShortStart;
}

bool parkingStartFootprintSafe(float x, float y, float headingDeg,
                               int wheelSign, float bodyMargin = 0.0f)
{
    return parking_start_footprint::safe(x, y, headingDeg, wheelSign,
        parkingCwShortStart, bodyMargin, parkingCcwShortStart);
}

float parkingEntryScanArcMm()
{
    if (parkingCcwFirstEdgeStart)
        return OBSTACLE_PARKING_CCW_FIRST_EDGE_SCAN_ARC_MM;
    return parkingCwShortStart ? OBSTACLE_PARKING_CW_SCAN_ARC_MM
                               : OBSTACLE_PARKING_ENTRY_SCAN_ARC_MM;
}
float parkingEntryScoutArcMm()
{
    if (parkingCcwShortStart)
        return 0.0f; // Both preceding stations are behind the initial CCW body.
    return parkingCwShortStart ? OBSTACLE_PARKING_CW_SCOUT_ARC_MM
                               : OBSTACLE_PARKING_ENTRY_SCOUT_ARC_MM;
}
uint8_t parkingGreenTraceCount[COURSE_STATIONS_PER_SECTION] = {};
uint32_t parkingGreenTraceMs[COURSE_STATIONS_PER_SECTION] = {};
uint8_t parkingEntryProgress = 0;
int8_t parkingEntryTargetStation = -1;
bool parkingEntryActive = false;
bool parkingEntryObserving = false;
bool parkingEntryTestHold = false;
bool parkingEntryJoining = false;
bool parkingEntryRecovering = false;
bool parkingEntryConnectorActive = false;
bool parkingEntryConnectorReplanPending = false;
int8_t parkingEntryConnectorChangedSeat = -1;
bool parkingEntryScouting = false;
bool parkingEntryPrimaryRetryUsed = false;
ParkingEntryScoutPhase parkingEntryScoutPhase =
    PARKING_ENTRY_SCOUT_STEER_SETTLE;
int8_t parkingEntryScoutStation = -1;
float parkingEntryScoutStartEncoderDistance = 0.0f;
float parkingEntryScoutReturnTravelMm = 0.0f;
uint32_t parkingEntryScoutPhaseStartMs = 0;
PositionEstimate parkingEntryScoutReturnPose;
uint8_t parkingEntryConnectorLength = 0;
uint8_t parkingEntryConnectorProgress = 0;
uint16_t parkingEntryConnectorMergeIndex = 0;
float parkingEntryConnectorStartEncoderDistance = 0.0f;
float parkingEntryConnectorLookaheadMm = 0.0f;
bool parkingEntryConnectorRouteLookahead = false;
bool parkingEntryFarGreenFollowup = false;
float parkingEntryFarGreenFollowupStartDistance = 0.0f;
uint32_t parkingEntryFarGreenFollowupTraceMs = 0;
uint8_t parkingEntryFarGreenFollowupTraceCount = 0;
uint32_t parkingEntryConnectorTraceMs = 0;
uint8_t parkingEntryConnectorTraceCount = 0;
uint32_t parkingEntryObserveStartMs = 0;
bool parkingEntryUsbWritten = false;
float parkingEntryStartEncoderDistance = 0.0f;
float parkingEntryJoinStartEncoderDistance = 0.0f;
float parkingEntryRecoveryStartEncoderDistance = 0.0f;
bool parkingEntryPathFailed = false;
bool parkingEntryControlLogged = false;
ParkingEntryDrivePhase parkingEntryDrivePhase =
    PARKING_ENTRY_STRAIGHT_STEER_SETTLE;
uint32_t parkingEntrySteerSettleStartMs = 0;
float parkingEntryStraightHeadingDeg = 0.0f;
float parkingEntryStraightControlStartDistance = 0.0f;
float parkingEntryStraightMaxHeadingErrorDeg = 0.0f;
float parkingEntryStraightFilteredHeadingErrorDeg = 0.0f;
bool parkingEntryStraightControlLogged = false;

float clampFloat(float value, float minimum, float maximum)
{
    return value < minimum ? minimum : (value > maximum ? maximum : value);
}

float recordedLapSpeed(float pathSpeedMmS)
{
    return completedLaps > 0 && !runtimeTestMode && !lapFinishPending
        ? ceilf(pathSpeedMmS * OBSTACLE_LATER_LAP_SPEED_FACTOR) : pathSpeedMmS;
}

float cappedPathSpeed(float pathSpeedMmS)
{
    float cappedSpeed = runtimeTestMode
        ? fminf(pathSpeedMmS, OBSTACLE_PATH_TEST_MAX_SPEED_MM_S)
        : pathSpeedMmS;
    if (runtimeSpeedCapMmS > 0.0f)
        cappedSpeed = fminf(cappedSpeed, runtimeSpeedCapMmS);
    return cappedSpeed;
}

float adaptiveLookahead(float speedMmS)
{
    return OBSTACLE_LOOKAHEAD_MIN_MM +
        (OBSTACLE_LOOKAHEAD_MAX_MM - OBSTACLE_LOOKAHEAD_MIN_MM) *
            clampFloat(
                (speedMmS - OBSTACLE_PATH_MIN_SPEED) /
                    (OBSTACLE_PATH_MAX_SPEED - OBSTACLE_PATH_MIN_SPEED),
                0.0f,
                1.0f);
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
        pose.x_mm =
            OBSTACLE_FIELD_ORIGIN_X_MM +
            OBSTACLE_STRAIGHT_LENGTH_MM * 0.5f -
            distanceToFirstCornerMm;
        pose.heading_deg = 0.0f;
    }
    else
    {
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

uint8_t sectionInferredEmpty(uint8_t section)
{
    if (section >= COURSE_SECTION_COUNT)
        return 0;
    // Preserve the explicit parking-exit/scout/connector checks, including
    // arbitrary diagnostic layouts in the starting section. Layout inference
    // is used only after joining the normal lap route. The reverse preflight
    // still checks all legal seats, independently of this inference.
    if (parkingEntryActive || parkingEntryObserving || parkingEntryScouting ||
        parkingEntryJoining || parkingEntryConnectorActive || parkingEntryTestHold)
        return 0;
    uint8_t mask = 0;
    for (uint8_t local = 0; local < 6; ++local)
        if (seats[section * 6 + local].confirmed)
            mask |= static_cast<uint8_t>(1U << local);
    return obstacle_section_empty_for_mode(
        mask, sectionLayoutMode == OBSTACLE_SECTION_LAYOUT_CHECK_ALL);
}

bool stationResolved(uint8_t stationIndex)
{
    if (stationIndex >= OBSTACLE_SEAT_COUNT / 2)
        return true;
    const uint8_t firstSeat = stationIndex * 2;
    return (sectionInferredEmpty(stationIndex / COURSE_STATIONS_PER_SECTION) &
            (1U << (stationIndex % COURSE_STATIONS_PER_SECTION))) ||
           discoveryStations[stationIndex].observedClear ||
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
        // Figure 8e relocates every sign in the parking section to the inner
        // row. Reject an impossible outer-seat projection; never snap it to
        // the inner row merely because that row is the only legal one.
        if (parkingSectionInnerSeatsOnly &&
            sectionLayoutMode == OBSTACLE_SECTION_LAYOUT_OFFICIAL && i < 6 &&
            seats[i].y < seats[i ^ 1U].y)
            continue;
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
    constexpr float minimumSegmentMm = 1.0f;
    for (uint16_t i = 0; i < pathLength; ++i)
    {
        const PathPoint &at = path[i];
        uint16_t beforeIndex = i;
        uint16_t afterIndex = i;

        // A closed path intentionally ends at the same physical location as
        // waypoint zero. Skip coincident cyclic neighbours so atan2(0, 0)
        // cannot create a false curvature spike at the lap seam.
        for (uint16_t step = 0; step + 1 < pathLength; ++step)
        {
            beforeIndex =
                (beforeIndex + pathLength - 1) % pathLength;
            if (hypotf(
                    at.x - path[beforeIndex].x,
                    at.y - path[beforeIndex].y) >= minimumSegmentMm)
                break;
        }
        for (uint16_t step = 0; step + 1 < pathLength; ++step)
        {
            afterIndex = (afterIndex + 1) % pathLength;
            if (hypotf(
                    path[afterIndex].x - at.x,
                    path[afterIndex].y - at.y) >= minimumSegmentMm)
                break;
        }

        const PathPoint &before = path[beforeIndex];
        const PathPoint &after = path[afterIndex];
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

void smoothRange(
    PathPoint *path,
    int center,
    int approachWaypoints,
    int exitWaypoints)
{
    const int first = center - approachWaypoints - 1;
    const int last = center + exitWaypoints + 1;
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

void prepareParkingSectionInnerSeats()
{
    parkingSectionInnerSeatsOnly = true;
    // Rules 2026 Figure 8e moves every sign in the parking section to the
    // position closer to the inner wall. In the canonical south section that
    // is the member with the larger field-y coordinate. Mark only its paired
    // outer seat as known clear; the inner seat still needs camera evidence.
    for (uint8_t station = 0; station < COURSE_STATIONS_PER_SECTION; ++station)
    {
        const uint8_t firstSeat = station * COURSE_SEATS_PER_STATION;
        const uint8_t outerSide =
            seats[firstSeat].y < seats[firstSeat + 1].y ? 0 : 1;
        DiscoveryStation &coverage = discoveryStations[station];
        coverage.seatObservedClear[outerSide] = true;
        coverage.clearFrames[outerSide] = OBSTACLE_DISCOVERY_CLEAR_FRAMES;
    }
}

void appendParkingEntryPoint(
    float x,
    float y,
    float headingDeg,
    float distanceMm)
{
    if (parkingEntryLength >= OBSTACLE_PARKING_ENTRY_MAX_WAYPOINTS)
        return;
    PathPoint &point = parkingEntryPath[parkingEntryLength++];
    point.x = x;
    point.y = y;
    point.headingDeg = headingDeg;
    point.distanceMm = distanceMm;
    point.speedMmS = OBSTACLE_PARKING_ENTRY_SPEED_MM_S;
}

bool calculateClearanceAtPose(
    const CandidateSeat &seat,
    float x,
    float y,
    float headingDeg,
    ObstacleClearanceSample &sample);

PathPoint connectorLookaheadFrom(
    uint8_t index, float lookaheadMm, const PathPoint *route,
    uint16_t mergeIndex, bool continueIntoRoute,
    uint16_t *targetIndex = nullptr);

float connectorRouteHeading(const PathPoint *route, uint16_t index)
{
    // Displacement changes XY but deliberately leaves baseline heading metadata
    // intact. A connector must join the actual outgoing segment, not that metadata.
    for (uint16_t step = 1; step < pathLength; ++step)
    {
        const uint16_t next = (index + step) % pathLength;
        const float dx = route[next].x - route[index].x;
        const float dy = route[next].y - route[index].y;
        if (hypotf(dx, dy) >= 1.0f)
            return atan2f(dy, dx) * 180.0f / PI;
    }
    return NAN;
}

// If the forward lookahead has already carried us beyond the merge, use
// the outgoing route's tangent/cross-track gate instead of circling back to
// a waypoint behind the robot. Limit the accepted outgoing portion to250mm.
bool connectorJoinReached(float x, float y, float headingDeg,
                          const PathPoint *route, uint16_t mergeIndex,
                          bool continueIntoRoute, uint16_t *joinedIndex = nullptr)
{
    if (parkingEntryConnectorLength < 2 || !isfinite(x) || !isfinite(y) ||
        !isfinite(headingDeg)) return false;
    const PathPoint &end = parkingEntryConnector[parkingEntryConnectorLength - 1];
    if (hypotf(x-end.x,y-end.y) <= OBSTACLE_PARKING_ENTRY_JOIN_CROSS_TRACK_MM &&
        fabsf(wrap180(headingDeg-end.headingDeg)) <= OBSTACLE_PARKING_ENTRY_JOIN_HEADING_DEG)
    {
        if (joinedIndex) *joinedIndex = mergeIndex;
        return true;
    }
    if (!continueIntoRoute || route == nullptr || mergeIndex >= pathLength)
        return false;
    float traversed = 0.0f;
    for (uint16_t step=0; step<6; ++step)
    {
        const uint16_t i=(mergeIndex+step)%pathLength, next=(i+1)%pathLength;
        const float dx=route[next].x-route[i].x, dy=route[next].y-route[i].y;
        const float length=hypotf(dx,dy);
        if (length < 1.0f) continue;
        const float u=((x-route[i].x)*dx+(y-route[i].y)*dy)/(length*length);
        if (u>=0.0f && u<=1.0f && traversed+u*length<=250.0f &&
            traversed+u*length>1.0f &&
            hypotf(x-route[i].x-u*dx,y-route[i].y-u*dy)<=OBSTACLE_PARKING_ENTRY_JOIN_CROSS_TRACK_MM &&
            fabsf(wrap180(headingDeg-atan2f(dy,dx)*180.0f/PI))<=OBSTACLE_PARKING_ENTRY_JOIN_HEADING_DEG)
        {
            if (joinedIndex) *joinedIndex=i;
            return true;
        }
        traversed+=length;
        if (traversed>250.0f) break;
    }
    return false;
}

bool connectorRolloutFeasible(
    const PositionEstimate &start, float lookaheadMm,
    uint8_t referenceSeat, int8_t confirmedSeat, int8_t guardSeat,
    const PathPoint *route, uint16_t mergeIndex, bool continueIntoRoute)
{
    constexpr float stepMm = 2.0f;
    constexpr float clearanceMarginMm = 10.0f;
    float x = start.x_mm, y = start.y_mm;
    float heading = start.heading_deg * PI / 180.0f;
    uint8_t progress = 0;
    const PathPoint &end = parkingEntryConnector[parkingEntryConnectorLength - 1];
    const auto reject = [&](const char *reason, float travel, int seat,
                            const ObstacleClearanceSample &clearance,
                            float forward, float steering) -> bool {
        Serial.print("[PARK ENTRY CONNECTOR] Rollout reject reason=");
        Serial.print(reason);
        Serial.print(" lookahead/travel_mm=");
        Serial.print(lookaheadMm, 0); Serial.print("/"); Serial.print(travel, 0);
        Serial.print(" pose="); Serial.print(x, 1); Serial.print(",");
        Serial.print(y, 1); Serial.print(",");
        Serial.print(heading * 180.0f / PI, 1);
        Serial.print(" seat="); Serial.print(seat);
        Serial.print(" wall/pillar_mm=");
        Serial.print(clearance.wallMm, 1); Serial.print("/");
        Serial.print(clearance.pillarMm, 1);
        Serial.print(" forward/steer=");
        Serial.print(forward, 1); Serial.print("/");
        Serial.print(steering, 1);
        Serial.print(" end_error/heading=");
        Serial.print(hypotf(x - end.x, y - end.y), 1); Serial.print("/");
        Serial.println(wrap180(heading * 180.0f / PI - end.headingDeg), 1);
        return false;
    };
    const ObstacleClearanceSample noClearance{};
    for (float travel = 0.0f;
         travel <= (parkingCcwShortStart ? OBSTACLE_PARKING_CCW_CONNECTOR_MAX_TRAVEL_MM :
                    OBSTACLE_PARKING_ENTRY_RECOVERY_MAX_TRAVEL_MM);
         travel += stepMm)
    {
        // Check both existing front/rear capsules at every 2 mm of simulated
        // motion, with a margin for the discrete footprint movement. This is
        // a kinematic prediction, not a measured uncertainty bound.
        for (uint8_t rear = 0; rear < 2; ++rear)
        {
            const float offset = rear ? OBSTACLE_MAX_WHEEL_HALF_WIDTH_MM : 0.0f;
            const float px = x - offset * cosf(heading);
            const float py = y - offset * sinf(heading);
            ObstacleClearanceSample clearance{};
            const bool referenceValid = calculateClearanceAtPose(
                seats[referenceSeat], px, py,
                heading * 180.0f / PI, clearance);
            if (!referenceValid ||
                clearance.wallMm <= clearanceMarginMm ||
                (confirmedSeat >= 0 && clearance.pillarMm <= clearanceMarginMm))
                return reject(referenceValid ? "reference_clearance" :
                    "reference_geometry", travel, referenceSeat, clearance,
                    NAN, NAN);
            for (uint8_t seat = 0; seat < OBSTACLE_SEAT_COUNT; ++seat)
            {
                if (!seats[seat].confirmed && seat != guardSeat)
                    continue;
                const bool seatValid = calculateClearanceAtPose(
                    seats[seat], px, py,
                    heading * 180.0f / PI, clearance);
                if (!seatValid ||
                    clearance.pillarMm <= clearanceMarginMm)
                    return reject(seatValid ? "pillar_clearance" :
                        "pillar_geometry", travel, seat, clearance, NAN, NAN);
            }
        }
        if (parkingShortStart() &&
            (!parkingStartFootprintSafe(
                x, y, heading * 180.0f / PI, -1, 5.0f) ||
             !parkingStartFootprintSafe(
                x, y, heading * 180.0f / PI, 1, 5.0f)))
            return reject("parking_piece", travel, -1, noClearance, NAN, NAN);
        if (connectorJoinReached(x, y, heading * 180.0f / PI,
                                 route, mergeIndex, continueIntoRoute))
            return true;
        if (travel + stepMm > (parkingCcwShortStart ? OBSTACLE_PARKING_CCW_CONNECTOR_MAX_TRAVEL_MM :
                               OBSTACLE_PARKING_ENTRY_RECOVERY_MAX_TRAVEL_MM))
            return reject("max_travel", travel, -1, noClearance, NAN, NAN);
        while (progress + 1 < parkingEntryConnectorLength &&
               hypotf(x - parkingEntryConnector[progress + 1].x,
                      y - parkingEntryConnector[progress + 1].y) <
               hypotf(x - parkingEntryConnector[progress].x,
                      y - parkingEntryConnector[progress].y))
            ++progress;
        const PathPoint target = connectorLookaheadFrom(
            progress, lookaheadMm, route, mergeIndex, continueIntoRoute);
        const float dx = target.x - x, dy = target.y - y;
        const float forward = dx * cosf(heading) + dy * sinf(heading);
        const float lateral = -dx * sinf(heading) + dy * cosf(heading);
        const float curvature = 2.0f * lateral / fmaxf(1.0f, dx * dx + dy * dy);
        const float steering = -atanf(OBSTACLE_WHEELBASE_MM * curvature) * 180.0f / PI;
        if (!isfinite(forward) || !isfinite(steering) || forward <= 1.0f ||
            fabsf(steering) > (parkingCcwShortStart ? OBSTACLE_PARKING_CONNECTOR_PLAN_STEERING_DEG :
                                 OBSTACLE_MAX_PURSUIT_STEERING_DEG))
            return reject("tracking", travel, -1, noClearance,
                forward, steering);
        const float nextHeading = heading + curvature * stepMm;
        if (fabsf(curvature) > 1.0e-6f)
        {
            x += (sinf(nextHeading) - sinf(heading)) / curvature;
            y += (cosf(heading) - cosf(nextHeading)) / curvature;
        }
        else
        {
            x += stepMm * cosf(heading);
            y += stepMm * sinf(heading);
        }
        heading = nextHeading;
    }
    return false;
}

// Build and validate one merge candidate. Every retry uses the same swept
// footprint, steering and terminal-pose gates as the original preflight.
bool tryParkingEntryConnectorMerge(
    const PositionEstimate &start, const PathPoint *route,
    uint16_t best, uint8_t referenceSeat, int8_t confirmedSeat,
    int8_t guardSeat, float bestForward, float bestLateral,
    float bestBeforePillar, uint16_t &mergeIndex,
    float shortStartScale = 0.0f, float shortEndScale = 0.0f)
{
    const float startHeading = start.heading_deg * PI / 180.0f;
    const bool continueIntoRoute = parkingShortStart() ||
        (confirmedSeat >= 0 && !seats[confirmedSeat].red);
    PathPoint end = route[best];
    end.headingDeg = connectorRouteHeading(route, best);
    if (!isfinite(end.headingDeg))
        return false;
    const float chord = hypotf(end.x - start.x_mm, end.y - start.y_mm);
    if (!isfinite(chord) || chord < OBSTACLE_PATH_SAMPLE_MM)
        return false;
    const float endHeading = end.headingDeg * PI / 180.0f;
    const float startTangentLength = fminf(
        chord * (parkingShortStart()
            ? (shortStartScale > 0 ? shortStartScale :
                OBSTACLE_PARKING_CW_CONNECTOR_START_TANGENT_SCALE)
            : OBSTACLE_PARKING_ENTRY_CONNECTOR_START_TANGENT_SCALE),
        OBSTACLE_PARKING_ENTRY_CONNECTOR_START_TANGENT_MAX_MM);
    const float endTangentLength = fminf(
        chord * (parkingShortStart()
            ? (shortEndScale > 0 ? shortEndScale :
                OBSTACLE_PARKING_CW_CONNECTOR_END_TANGENT_SCALE)
            : OBSTACLE_PARKING_ENTRY_CONNECTOR_END_TANGENT_SCALE),
        OBSTACLE_PARKING_ENTRY_CONNECTOR_END_TANGENT_MAX_MM);
    const float startTx = cosf(startHeading) * startTangentLength;
    const float startTy = sinf(startHeading) * startTangentLength;
    const float endTx = cosf(endHeading) * endTangentLength;
    const float endTy = sinf(endHeading) * endTangentLength;
    const uint8_t count = static_cast<uint8_t>(clampFloat(
        ceilf(chord / OBSTACLE_PARKING_ENTRY_CONNECTOR_SAMPLE_MM) + 1.0f,
        3.0f,
        OBSTACLE_PARKING_ENTRY_CONNECTOR_MAX_WAYPOINTS));

    parkingEntryConnectorLength = 0;
    float previousX = start.x_mm;
    float previousY = start.y_mm;
    float distance = 0.0f;
    for (uint8_t sample = 0; sample < count; ++sample)
    {
        const float t = static_cast<float>(sample) /
            static_cast<float>(count - 1);
        const float t2 = t * t;
        const float t3 = t2 * t;
        const float h00 = 2.0f * t3 - 3.0f * t2 + 1.0f;
        const float h10 = t3 - 2.0f * t2 + t;
        const float h01 = -2.0f * t3 + 3.0f * t2;
        const float h11 = t3 - t2;
        const float x = h00 * start.x_mm + h10 * startTx +
            h01 * end.x + h11 * endTx;
        const float y = h00 * start.y_mm + h10 * startTy +
            h01 * end.y + h11 * endTy;
        if (sample > 0)
            distance += hypotf(x - previousX, y - previousY);
        previousX = x;
        previousY = y;
        PathPoint &point = parkingEntryConnector[parkingEntryConnectorLength++];
        point.x = x;
        point.y = y;
        point.distanceMm = distance;
        point.speedMmS = OBSTACLE_PARKING_ENTRY_RECOVERY_SPEED_MM_S;
        if (sample + 1 == count)
            point.headingDeg = end.headingDeg;
        else
        {
            const float dx = (end.x - start.x_mm) *
                (6.0f * t - 6.0f * t2) +
                startTx * (3.0f * t2 - 4.0f * t + 1.0f) +
                endTx * (3.0f * t2 - 2.0f * t);
            const float dy = (end.y - start.y_mm) *
                (6.0f * t - 6.0f * t2) +
                startTy * (3.0f * t2 - 4.0f * t + 1.0f) +
                endTy * (3.0f * t2 - 2.0f * t);
            point.headingDeg = atan2f(dy, dx) * 180.0f / PI;
        }
    }

    for (uint8_t sample = 0; sample < parkingEntryConnectorLength; ++sample)
    {
        ObstacleClearanceSample frontClearance{};
        ObstacleClearanceSample rearClearance{};
        const float heading = parkingEntryConnector[sample].headingDeg *
            PI / 180.0f;
        const float rearX = parkingEntryConnector[sample].x -
            OBSTACLE_MAX_WHEEL_HALF_WIDTH_MM * cosf(heading);
        const float rearY = parkingEntryConnector[sample].y -
            OBSTACLE_MAX_WHEEL_HALF_WIDTH_MM * sinf(heading);
        if (!calculateClearanceAtPose(
                seats[referenceSeat],
                parkingEntryConnector[sample].x,
                parkingEntryConnector[sample].y,
                parkingEntryConnector[sample].headingDeg,
                frontClearance) || frontClearance.wallMm <= 0.0f ||
            (confirmedSeat >= 0 && frontClearance.pillarMm <= 0.0f) ||
            !calculateClearanceAtPose(
                seats[referenceSeat],
                rearX,
                rearY,
                parkingEntryConnector[sample].headingDeg,
                rearClearance) || rearClearance.wallMm <= 0.0f ||
            (confirmedSeat >= 0 && rearClearance.pillarMm <= 0.0f))
        {
            Serial.print("[PARK ENTRY CONNECTOR] Preflight FAIL sample=");
            Serial.print(sample);
            Serial.print(" front_wall/pillar_mm=");
            Serial.print(frontClearance.wallMm, 1);
            Serial.print("/");
            Serial.print(frontClearance.pillarMm, 1);
            Serial.print(" rear_wall/pillar_mm=");
            Serial.print(rearClearance.wallMm, 1);
            Serial.print("/");
            Serial.println(rearClearance.pillarMm, 1);
            return false;
        }
        if (guardSeat >= 0)
        {
            ObstacleClearanceSample guardFrontClearance{};
            ObstacleClearanceSample guardRearClearance{};
            if (!calculateClearanceAtPose(
                    seats[guardSeat],
                    parkingEntryConnector[sample].x,
                    parkingEntryConnector[sample].y,
                    parkingEntryConnector[sample].headingDeg,
                    guardFrontClearance) ||
                guardFrontClearance.pillarMm <= 0.0f ||
                !calculateClearanceAtPose(
                    seats[guardSeat],
                    rearX,
                    rearY,
                    parkingEntryConnector[sample].headingDeg,
                    guardRearClearance) ||
                guardRearClearance.pillarMm <= 0.0f)
            {
                Serial.print(
                    "[PARK ENTRY CONNECTOR] Preflight FAIL hidden_guard sample/seat=");
                Serial.print(sample);
                Serial.print("/");
                Serial.print(static_cast<int>(guardSeat));
                Serial.print(" front/rear_pillar_mm=");
                Serial.print(guardFrontClearance.pillarMm, 1);
                Serial.print("/");
                Serial.println(guardRearClearance.pillarMm, 1);
                return false;
            }
        }
    }

    // Validate the same lookahead geometry that Pure Pursuit will use. Pure
    // Pursuit is a forward controller: a target behind the rear axle can still
    // produce a finite steering command, but driving that command forward makes
    // the robot circle instead of converging. Reject that geometry before the
    // motor is enabled, together with targets that need more than the physical
    // steering envelope.
    float desiredLookahead = adaptiveLookahead(
        OBSTACLE_PARKING_ENTRY_RECOVERY_SPEED_MM_S);
    if (!parkingShortStart() && confirmedSeat >= 0 && !seats[confirmedSeat].red)
        desiredLookahead *=
            OBSTACLE_PARKING_ENTRY_GREEN_JOIN_LOOKAHEAD_SCALE;
    parkingEntryConnectorLookaheadMm = 0.0f;
    float failedForward = 0.0f;
    float failedSteering = 0.0f;
    uint8_t failedSample = 0;
    uint16_t failedTarget = 0;
    for (float candidateLookahead = desiredLookahead;
         candidateLookahead >=
             OBSTACLE_PARKING_ENTRY_CONNECTOR_SAMPLE_MM - 0.1f;
         candidateLookahead -= OBSTACLE_PARKING_ENTRY_CONNECTOR_SAMPLE_MM)
    {
        bool feasible = true;
        for (uint8_t sample = 0;
             sample + 1 < parkingEntryConnectorLength;
             ++sample)
        {
            const PathPoint &at = parkingEntryConnector[sample];
            const PathPoint &connectorEnd = parkingEntryConnector[
                parkingEntryConnectorLength - 1];
            if (hypotf(
                    connectorEnd.x - at.x,
                    connectorEnd.y - at.y) <=
                    OBSTACLE_PARKING_ENTRY_JOIN_CROSS_TRACK_MM &&
                fabsf(wrap180(at.headingDeg - connectorEnd.headingDeg)) <=
                    OBSTACLE_PARKING_ENTRY_JOIN_HEADING_DEG)
            {
                break;
            }
            uint16_t targetSample = sample;
            const PathPoint target = connectorLookaheadFrom(
                sample, candidateLookahead, route, best,
                continueIntoRoute, &targetSample);
            const float dx = target.x - at.x;
            const float dy = target.y - at.y;
            const float heading = at.headingDeg * PI / 180.0f;
            const float localX = dx * cosf(heading) + dy * sinf(heading);
            const float localY = -dx * sinf(heading) + dy * cosf(heading);
            const float targetDistanceSquared =
                fmaxf(1.0f, dx * dx + dy * dy);
            const float curvature = 2.0f * localY / targetDistanceSquared;
            const float requiredSteering =
                -atanf(OBSTACLE_WHEELBASE_MM * curvature) * 180.0f / PI;
            if (!isfinite(localX) || !isfinite(requiredSteering) ||
                localX <= 1.0f ||
                fabsf(requiredSteering) > (parkingCcwShortStart ? OBSTACLE_PARKING_CONNECTOR_PLAN_STEERING_DEG :
                                         OBSTACLE_MAX_PURSUIT_STEERING_DEG))
            {
                feasible = false;
                failedSample = sample;
                failedTarget = targetSample;
                failedForward = localX;
                failedSteering = requiredSteering;
                break;
            }
        }
        if (feasible && !connectorRolloutFeasible(
                start, candidateLookahead, referenceSeat, confirmedSeat,
                guardSeat, route, best, continueIntoRoute))
        {
            feasible = false;
            Serial.print("[PARK ENTRY CONNECTOR] Rollout FAIL lookahead_mm=");
            Serial.println(candidateLookahead, 1);
        }
        if (feasible)
        {
            parkingEntryConnectorLookaheadMm = candidateLookahead;
            break;
        }
    }
    if (parkingEntryConnectorLookaheadMm <= 0.0f)
    {
        Serial.print(
            "[PARK ENTRY CONNECTOR] Preflight FAIL tracking_sample/target=");
        Serial.print(failedSample);
        Serial.print("/");
        Serial.print(failedTarget);
        Serial.print(" forward_mm=");
        Serial.print(failedForward, 1);
        Serial.print(" steering/min_lookahead_deg_mm=");
        Serial.print(failedSteering, 1);
        Serial.print("/");
        Serial.print(OBSTACLE_PARKING_ENTRY_CONNECTOR_SAMPLE_MM, 1);
        Serial.print(" merge_forward/lateral/before_pillar_mm=");
        Serial.print(bestForward, 0);
        Serial.print("/");
        Serial.print(bestLateral, 0);
        Serial.print("/");
        Serial.println(bestBeforePillar, 0);
        return false;
    }
    mergeIndex = best;
    Serial.print("[PARK ENTRY CONNECTOR] Preflight PASS merge_index=");
    Serial.print(mergeIndex);
    Serial.print(" merge/seat_distance_mm=");
    Serial.print(route[mergeIndex].distanceMm, 0);
    Serial.print("/");
    Serial.print(seats[referenceSeat].pathDistanceMm, 0);
    Serial.print(" pillar_seat=");
    Serial.print(static_cast<int>(confirmedSeat));
    Serial.print(" hidden_guard_seat=");
    Serial.print(static_cast<int>(guardSeat));
    Serial.print(" merge_forward/lateral/before_pillar_mm=");
    Serial.print(bestForward, 0);
    Serial.print("/");
    Serial.print(bestLateral, 0);
    Serial.print("/");
    Serial.print(bestBeforePillar, 0);
    Serial.print(" path_mm=");
    Serial.print(parkingEntryConnector[parkingEntryConnectorLength - 1].distanceMm, 0);
    Serial.print(" lookahead_mm=");
    Serial.print(parkingEntryConnectorLookaheadMm, 1);
    Serial.print(" route_lookahead=");
    Serial.print(continueIntoRoute ? "yes" : "no");
    Serial.print(" points=");
    Serial.println(parkingEntryConnectorLength);
    Serial.print("[PARK ENTRY CONNECTOR] Tangent baseline/actual_deg=");
    Serial.print(route[mergeIndex].headingDeg, 2);
    Serial.print("/"); Serial.println(end.headingDeg, 2);
    // Geometry is emitted while stopped, once per successful build. This
    // allows offline replay without inventing the missing terminal pose.
    parkingEntryConnectorTraceCount = 0;
    parkingEntryConnectorTraceMs = 0;
    Serial.print("[CONNECTOR_CONFIG] v=1 build=");
    Serial.print(__DATE__);
    Serial.print("_"); Serial.println(__TIME__);
    for (uint8_t index = 0; index < parkingEntryConnectorLength; ++index)
    {
        const PathPoint &point = parkingEntryConnector[index];
        Serial.print("[CONNECTOR_POINT] kind=connector index=");
        Serial.print(index);
        Serial.print(" x="); Serial.print(point.x, 2);
        Serial.print(" y="); Serial.print(point.y, 2);
        Serial.print(" h="); Serial.println(point.headingDeg, 2);
    }
    float routeTravel = 0.0f;
    uint16_t routeIndex = mergeIndex;
    for (uint8_t offset = 0; offset < 32; ++offset)
    {
        const PathPoint &point = route[routeIndex];
        Serial.print("[CONNECTOR_POINT] kind=route index=");
        Serial.print(offset);
        Serial.print(" x="); Serial.print(point.x, 2);
        Serial.print(" y="); Serial.print(point.y, 2);
        Serial.print(" h="); Serial.println(point.headingDeg, 2);
        if (routeTravel >= parkingEntryConnectorLookaheadMm + 50.0f)
            break;
        const uint16_t next = (routeIndex + 1) % pathLength;
        if (next == mergeIndex)
            break;
        routeTravel += hypotf(route[next].x - point.x, route[next].y - point.y);
        routeIndex = next;
    }
    return parkingEntryConnectorLength >= 3;
}

bool buildParkingEntryConnector(
    const PositionEstimate &start,
    const PathPoint *route,
    uint16_t &mergeIndex)
{
    parkingEntryConnectorRouteLookahead = false;
    if (parkingEntryTargetStation < 0)
        return false;
    const uint8_t firstSeat = static_cast<uint8_t>(
        parkingEntryTargetStation * COURSE_SEATS_PER_STATION);
    int8_t confirmedSeat = -1;
    for (uint8_t offset = 0; offset < COURSE_SEATS_PER_STATION; ++offset)
    {
        if (seats[firstSeat + offset].confirmed)
        {
            confirmedSeat = static_cast<int8_t>(firstSeat + offset);
            break;
        }
    }

    // A confirmed pillar owns the displaced-route merge phase. If the parking
    // scan resolved CLEAR, use the same inner-seat station phase as a fixed-
    // field reference, but run wall-only preflight because no pillar exists.
    const uint8_t referenceSeat = confirmedSeat >= 0
        ? static_cast<uint8_t>(confirmedSeat)
        : (seats[firstSeat].y > seats[firstSeat + 1].y
               ? firstSeat
               : firstSeat + 1);
    // The station immediately after leaving the parking lot is encountered
    // before the station resolved by the parking scan. Its inner pillar can be
    // hidden by the parking walls, so guard its legal position even while the
    // station is still unresolved. The parking-section outer seat is known
    // empty by the current rules geometry.
    int8_t guardSeat = -1;
    if (parkingEntryTargetStation > 0)
    {
        const uint8_t guardStation = static_cast<uint8_t>(
            parkingEntryTargetStation - 1);
        const uint8_t guardFirstSeat = static_cast<uint8_t>(
            guardStation * COURSE_SEATS_PER_STATION);
        if (seats[guardFirstSeat].confirmed)
            guardSeat = static_cast<int8_t>(guardFirstSeat);
        else if (seats[guardFirstSeat + 1].confirmed)
            guardSeat = static_cast<int8_t>(guardFirstSeat + 1);
        else if (!discoveryStations[guardStation].observedClear)
            guardSeat = static_cast<int8_t>(
                seats[guardFirstSeat].y > seats[guardFirstSeat + 1].y
                    ? guardFirstSeat
                    : guardFirstSeat + 1);
    }

    const float startHeading = start.heading_deg * PI / 180.0f;
    const float headingX = cosf(startHeading);
    const float headingY = sinf(startHeading);

    // Keep the established best-ray candidate first. If its complete rollout
    // fails, try up to three progressively earlier merge phases. A failed
    // candidate never enables the motor or weakens a safety threshold.
    const float minimumApproach = parkingShortStart()
        ? OBSTACLE_PARKING_CW_CONNECTOR_MIN_BEFORE_PILLAR_MM
        : OBSTACLE_PARKING_ENTRY_CONNECTOR_MIN_BEFORE_PILLAR_MM;
    float minimumBeforePillar = confirmedSeat >= 0 ? minimumApproach : 0.0f;
    uint16_t triedShortMerges[10] = {};
    for (uint8_t attempt = 0; attempt < (parkingShortStart() ? 10 : 4); ++attempt)
    {
        uint16_t best = 0;
        float bestScore = 1.0e12f;
        float bestForward = 0.0f;
        float bestLateral = 0.0f;
        float bestBeforePillar = -1.0f;
        for (uint16_t index = 0; index < pathLength; ++index)
        {
            bool tried = false;
            if (parkingShortStart())
                for (uint8_t previous = 0; previous < attempt; ++previous)
                    tried = tried || triedShortMerges[previous] == index;
            if (tried)
                continue;
            if (fabsf(wrap180(start.heading_deg - route[index].headingDeg)) > 100.0f)
                continue;
            const float dx = route[index].x - start.x_mm;
            const float dy = route[index].y - start.y_mm;
            const float forward = dx * headingX + dy * headingY;
            const float lateral = -dx * headingY + dy * headingX;
            if (forward < (parkingShortStart()
                    ? OBSTACLE_PARKING_CW_CONNECTOR_MIN_FORWARD_MM
                    : OBSTACLE_PARKING_ENTRY_CONNECTOR_MIN_FORWARD_MM) ||
                forward > OBSTACLE_PARKING_ENTRY_CONNECTOR_MAX_FORWARD_MM)
                continue;

            float beforePillar = -1.0f;
            float score = fabsf(lateral);
            if (confirmedSeat >= 0)
            {
                beforePillar = cyclicDistanceForward(
                    route[index].distanceMm,
                    seats[referenceSeat].pathDistanceMm);
                if (beforePillar <
                        minimumApproach - 0.1f ||
                    beforePillar >
                        OBSTACLE_PARKING_ENTRY_CONNECTOR_MAX_BEFORE_PILLAR_MM + 0.1f)
                    continue;
                // Prefer the route point closest to the current heading ray, then
                // the farthest forward point when sampled candidates are similar.
                score -= forward * 0.01f;
            }
            else
            {
                // With no pillar, the closest sampled point to the forward ray is
                // the route intersection requested by the measured scan heading.
                // Keep that intersection on this station's forward approach so a
                // different side of the closed lap cannot win the ray search.
                beforePillar = cyclicDistanceForward(
                    route[index].distanceMm,
                    seats[referenceSeat].pathDistanceMm);
                if (beforePillar >
                    OBSTACLE_PARKING_ENTRY_CONNECTOR_MAX_BEFORE_PILLAR_MM)
                    continue;
                score += forward * 0.001f;
            }
            if (beforePillar < minimumBeforePillar - 0.1f)
                continue;
            if (score < bestScore)
            {
                bestScore = score;
                best = index;
                bestForward = forward;
                bestLateral = lateral;
                bestBeforePillar = beforePillar;
            }
        }

        if (bestScore >= 1.0e12f)
            break;
        triedShortMerges[attempt] = best;
        Serial.print("[PARK ENTRY CONNECTOR] Candidate attempt/merge/phase_mm=");
        Serial.print(attempt + 1); Serial.print("/");
        Serial.print(best); Serial.print("/");
        Serial.println(bestBeforePillar, 0);
        bool accepted = false;
        // Bounded geometry alternatives account for the measured scan/return
        // pose. Every shape receives the identical complete swept rollout;
        // no colour-specific assumption substitutes for a clearance check.
        const float shortStartScales[] = {
            OBSTACLE_PARKING_CW_CONNECTOR_START_TANGENT_SCALE, 0.75f, 1.0f, 1.25f, 0.5f};
        const float shortEndScales[] = {
            OBSTACLE_PARKING_CW_CONNECTOR_END_TANGENT_SCALE, 1.4f, 1.25f, 1.5f, 1.5f};
        for (uint8_t shape = 0; shape < (parkingShortStart() ? 5 : 1); ++shape)
        {
            if (tryParkingEntryConnectorMerge(start, route, best,
                    referenceSeat, confirmedSeat, guardSeat, bestForward,
                    bestLateral, bestBeforePillar, mergeIndex,
                    shortStartScales[shape], shortEndScales[shape]))
            {
                accepted = true;
                break;
            }
        }
        if (accepted)
        {
            parkingEntryConnectorRouteLookahead = parkingShortStart() ||
                (confirmedSeat >= 0 && !seats[confirmedSeat].red);
            return true;
        }
        Serial.print("[PARK ENTRY CONNECTOR] Candidate rejected attempt/phase_mm=");
        Serial.print(attempt + 1); Serial.print("/");
        Serial.println(bestBeforePillar, 0);
        if (!parkingShortStart())
            minimumBeforePillar = fmaxf(
                minimumBeforePillar, bestBeforePillar + 50.0f);
        if (minimumBeforePillar >
            OBSTACLE_PARKING_ENTRY_CONNECTOR_MAX_BEFORE_PILLAR_MM)
            break;
    }
    parkingEntryConnectorLength = 0;
    parkingEntryConnectorLookaheadMm = 0.0f;
    Serial.println("[PARK ENTRY CONNECTOR] No safe merge candidate");
    return false;
}

bool parkingConnectorMergeUnchanged(const PathPoint *route)
{
    if (route == nullptr || pathLength < 2 ||
        parkingEntryConnectorMergeIndex >= pathLength ||
        parkingEntryConnectorLength < 2)
        return false;
    const PathPoint &end =
        parkingEntryConnector[parkingEntryConnectorLength - 1];
    const PathPoint &merge = route[parkingEntryConnectorMergeIndex];
    const float heading = connectorRouteHeading(
        route, parkingEntryConnectorMergeIndex);
    return isfinite(heading) &&
        hypotf(end.x - merge.x, end.y - merge.y) <= 1.0f &&
        fabsf(wrap180(heading - end.headingDeg)) <=
            OBSTACLE_PARKING_ENTRY_JOIN_HEADING_DEG;
}

bool retainParkingConnectorForFarGreen(
    const PositionEstimate &pose, const PathPoint *route)
{
    if (parkingShortStart() && parkingEntryConnectorActive &&
        parkingEntryConnectorChangedSeat >= 6 &&
        parkingEntryConnectorChangedSeat < OBSTACLE_SEAT_COUNT &&
        seats[parkingEntryConnectorChangedSeat].confirmed && parkingConnectorMergeUnchanged(route))
    {
        // A newly visible first sign in the next section may alter the corner,
        // but does not require throwing away an unchanged start merge.
        const uint8_t reference = parkingCcwShortStart ? 5 : 2;
        const int8_t frontGuard = parkingCcwShortStart && seats[5].confirmed ? 5 : -1;
        const int8_t hiddenGuard = parkingCcwShortStart
            ? (seats[3].confirmed || !discoveryStations[1].observedClear ? 3 : -1)
            : (seats[0].confirmed || !discoveryStations[0].observedClear ? 0 : -1);
        if (!connectorRolloutFeasible(pose, parkingEntryConnectorLookaheadMm,
                reference, frontGuard, hiddenGuard, route,
                parkingEntryConnectorMergeIndex, true)) return false;
        parkingEntryConnectorRouteLookahead = true;
        Serial.println("[PARK ENTRY CONNECTOR] Retained join after other-section detection; full remaining rollout checked");
        return true;
    }
    if (parkingCwShortStart && parkingEntryConnectorActive &&
        parkingEntryConnectorChangedSeat == 4 && seats[4].confirmed &&
        parkingConnectorMergeUnchanged(route))
    {
        // End-pair layouts may reveal the left pillar during the connector.
        // Keep an unchanged join only after rechecking its remaining rollout
        // against every confirmed pillar and the new outgoing route.
        if (!connectorRolloutFeasible(pose,
                parkingEntryConnectorLookaheadMm, 2, -1,
                seats[0].confirmed ? 0 : -1, route,
                parkingEntryConnectorMergeIndex, true))
            return false;
        parkingEntryConnectorRouteLookahead = true;
        parkingEntryFarGreenFollowup = !seats[4].red;
        Serial.println("[CW START] Retained join after left-place detection; all pillars rechecked");
        return true;
    }
    // CW single-pillar GREEN at the last inner start seat can become visible
    // after the empty first two stations have been scanned. Rebuilding from
    // the now advanced pose asks for a fresh 350 mm approach that no longer
    // exists. Keep the earlier connector only if its merge point and tangent
    // still match the live route and a fresh swept rollout from the actual
    // pose clears the newly confirmed pillar. The later outgoing bypass is
    // deliberately allowed to change when GREEN is injected.
    constexpr uint8_t farGreenSeat = 4;
    if (!parkingEntryConnectorActive || routeTurnSign >= 0 ||
        parkingEntryTargetStation != 1 ||
        parkingEntryConnectorChangedSeat != farGreenSeat ||
        !seats[farGreenSeat].confirmed ||
        !seats[farGreenSeat].injected || seats[farGreenSeat].red ||
        !discoveryStations[0].observedClear ||
        !discoveryStations[1].observedClear)
        return false;

    if (!parkingConnectorMergeUnchanged(route))
    {
        Serial.println("[PARK ENTRY CONNECTOR] Far GREEN merge changed");
        return false;
    }

    constexpr uint8_t firstTargetSeat = 2;
    const uint8_t referenceSeat =
        seats[firstTargetSeat].y > seats[firstTargetSeat + 1].y
            ? firstTargetSeat : firstTargetSeat + 1;
    if (!connectorRolloutFeasible(
            pose, parkingEntryConnectorLookaheadMm, referenceSeat,
            -1, -1, route, parkingEntryConnectorMergeIndex,
            true))
        return false;

    // The finite endpoint caused the actual 449/451 steering stops just
    // outside the handoff gate. Use the same outgoing-route target that was
    // checked by the fresh rollout and already worked for GREEN in station1.
    parkingEntryConnectorRouteLookahead = true;
    parkingEntryFarGreenFollowup = true;

    Serial.print("[PARK ENTRY CONNECTOR] Retained after far GREEN seat=");
    Serial.print(farGreenSeat);
    Serial.print(" pose=");
    Serial.print(pose.x_mm, 1); Serial.print(",");
    Serial.print(pose.y_mm, 1); Serial.print(",");
    Serial.print(pose.heading_deg, 1);
    Serial.print(" merge_index=");
    Serial.println(parkingEntryConnectorMergeIndex);
    return true;
}

void buildParkingEntryPath(const PositionEstimate &start)
{
    parkingEntryLength = 0;
    parkingEntryProgress = 0;
    float x = start.x_mm;
    float y = start.y_mm;
    float heading = start.heading_deg * PI / 180.0f;
    float distance = 0.0f;
    appendParkingEntryPoint(x, y, start.heading_deg, distance);

    const float arcStartX = routeTurnSign > 0
        ? OBSTACLE_PARKING_ENTRY_CCW_ARC_START_X_MM
        : OBSTACLE_PARKING_ENTRY_CW_ARC_START_X_MM;
    const float headingX = cosf(heading);
    float straight = fabsf(headingX) > 0.5f
        ? (x - arcStartX) / headingX
        : 0.0f;
    // Official short starts use the settled measured reference directly;
    // no second tiny centred reverse after localization has already stopped.
    straight = parkingShortStart() ? 0.0f : clampFloat(straight, 0.0f, 450.0f);
    float remaining = straight;
    while (remaining > 0.1f)
    {
        const float step = fminf(20.0f, remaining);
        x -= step * cosf(heading);
        y -= step * sinf(heading);
        distance += step;
        appendParkingEntryPoint(
            x, y, heading * 180.0f / PI, distance);
        remaining -= step;
    }

    // Signed path curvature is opposite the desired yaw because the vehicle
    // traverses this arc in reverse. Pure Pursuit sees targets behind the rear
    // axle and therefore requests the corresponding mirrored steering sign.
    const float curvature =
        -routeTurnSign / OBSTACLE_PARKING_ENTRY_SCAN_RADIUS_MM;
    remaining = parkingEntryScanArcMm();
    while (remaining > 0.1f)
    {
        const float step = fminf(10.0f, remaining);
        const float signedStep = -step;
        const float nextHeading = heading + curvature * signedStep;
        x += (sinf(nextHeading) - sinf(heading)) / curvature;
        y += (-cosf(nextHeading) + cosf(heading)) / curvature;
        heading = nextHeading;
        distance += step;
        appendParkingEntryPoint(
            x, y, heading * 180.0f / PI, distance);
        remaining -= step;
    }
}

struct WallSegment
{
    float ax;
    float ay;
    float bx;
    float by;
    ObstacleWallFeature feature;
};

constexpr float FIELD_INNER_HALF_MM =
    OBSTACLE_CENTERLINE_HALF_EXTENT_MM - OBSTACLE_CORRIDOR_HALF_WIDTH_MM;
constexpr float FIELD_OUTER_HALF_MM =
    OBSTACLE_CENTERLINE_HALF_EXTENT_MM + OBSTACLE_CORRIDOR_HALF_WIDTH_MM;

const WallSegment FIELD_WALLS[] = {
    {-FIELD_OUTER_HALF_MM, -FIELD_OUTER_HALF_MM,
      FIELD_OUTER_HALF_MM, -FIELD_OUTER_HALF_MM, OBSTACLE_WALL_OUTER_SOUTH},
    { FIELD_OUTER_HALF_MM, -FIELD_OUTER_HALF_MM,
      FIELD_OUTER_HALF_MM,  FIELD_OUTER_HALF_MM, OBSTACLE_WALL_OUTER_EAST},
    { FIELD_OUTER_HALF_MM,  FIELD_OUTER_HALF_MM,
     -FIELD_OUTER_HALF_MM,  FIELD_OUTER_HALF_MM, OBSTACLE_WALL_OUTER_NORTH},
    {-FIELD_OUTER_HALF_MM,  FIELD_OUTER_HALF_MM,
     -FIELD_OUTER_HALF_MM, -FIELD_OUTER_HALF_MM, OBSTACLE_WALL_OUTER_WEST},
    {-FIELD_INNER_HALF_MM, -FIELD_INNER_HALF_MM,
      FIELD_INNER_HALF_MM, -FIELD_INNER_HALF_MM, OBSTACLE_WALL_INNER_SOUTH},
    { FIELD_INNER_HALF_MM, -FIELD_INNER_HALF_MM,
      FIELD_INNER_HALF_MM,  FIELD_INNER_HALF_MM, OBSTACLE_WALL_INNER_EAST},
    { FIELD_INNER_HALF_MM,  FIELD_INNER_HALF_MM,
     -FIELD_INNER_HALF_MM,  FIELD_INNER_HALF_MM, OBSTACLE_WALL_INNER_NORTH},
    {-FIELD_INNER_HALF_MM,  FIELD_INNER_HALF_MM,
     -FIELD_INNER_HALF_MM, -FIELD_INNER_HALF_MM, OBSTACLE_WALL_INNER_WEST}
};

float pointSegmentDistance(
    float px, float py, float ax, float ay, float bx, float by,
    float *nearestX = nullptr, float *nearestY = nullptr)
{
    const float dx = bx - ax;
    const float dy = by - ay;
    const float lengthSquared = dx * dx + dy * dy;
    const float t = lengthSquared > 0.0f
        ? clampFloat(((px - ax) * dx + (py - ay) * dy) / lengthSquared,
                     0.0f, 1.0f)
        : 0.0f;
    const float qx = ax + t * dx;
    const float qy = ay + t * dy;
    if (nearestX != nullptr) *nearestX = qx;
    if (nearestY != nullptr) *nearestY = qy;
    return hypotf(px - qx, py - qy);
}

float orientation(float ax, float ay, float bx, float by, float cx, float cy)
{
    return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
}

bool segmentsIntersect(
    float ax, float ay, float bx, float by,
    float cx, float cy, float dx, float dy)
{
    const float o1 = orientation(ax, ay, bx, by, cx, cy);
    const float o2 = orientation(ax, ay, bx, by, dx, dy);
    const float o3 = orientation(cx, cy, dx, dy, ax, ay);
    const float o4 = orientation(cx, cy, dx, dy, bx, by);
    constexpr float epsilon = 0.001f;
    const bool properCrossing =
        ((o1 > epsilon && o2 < -epsilon) ||
         (o1 < -epsilon && o2 > epsilon)) &&
        ((o3 > epsilon && o4 < -epsilon) ||
         (o3 < -epsilon && o4 > epsilon));
    if (properCrossing)
        return true;
    const auto onSegment = [epsilon](
        float px, float py, float sx, float sy, float ex, float ey)
    {
        return px >= fminf(sx, ex) - epsilon &&
               px <= fmaxf(sx, ex) + epsilon &&
               py >= fminf(sy, ey) - epsilon &&
               py <= fmaxf(sy, ey) + epsilon;
    };
    return (fabsf(o1) <= epsilon && onSegment(cx, cy, ax, ay, bx, by)) ||
           (fabsf(o2) <= epsilon && onSegment(dx, dy, ax, ay, bx, by)) ||
           (fabsf(o3) <= epsilon && onSegment(ax, ay, cx, cy, dx, dy)) ||
           (fabsf(o4) <= epsilon && onSegment(bx, by, cx, cy, dx, dy));
}

float segmentDistanceToWall(
    float ax, float ay, float bx, float by, const WallSegment &wall,
    float &wallX, float &wallY)
{
    if (segmentsIntersect(ax, ay, bx, by, wall.ax, wall.ay, wall.bx, wall.by))
    {
        wallX = ax;
        wallY = ay;
        return 0.0f;
    }
    float best = 1.0e9f;
    float qx = 0.0f;
    float qy = 0.0f;
    const float endpoints[4] = {ax, ay, bx, by};
    for (uint8_t i = 0; i < 2; ++i)
    {
        float wx = 0.0f;
        float wy = 0.0f;
        const float d = pointSegmentDistance(
            endpoints[i * 2], endpoints[i * 2 + 1],
            wall.ax, wall.ay, wall.bx, wall.by, &wx, &wy);
        if (d < best) { best = d; qx = wx; qy = wy; }
    }
    const float wallEndpoints[4] = {wall.ax, wall.ay, wall.bx, wall.by};
    for (uint8_t i = 0; i < 2; ++i)
    {
        const float d = pointSegmentDistance(
            wallEndpoints[i * 2], wallEndpoints[i * 2 + 1],
            ax, ay, bx, by);
        if (d < best)
        {
            best = d;
            qx = wallEndpoints[i * 2];
            qy = wallEndpoints[i * 2 + 1];
        }
    }
    wallX = qx;
    wallY = qy;
    return best;
}

ObstacleWallFeature classifyWallFeature(
    ObstacleWallFeature face, float wallX, float wallY)
{
    if (face < OBSTACLE_WALL_INNER_SOUTH ||
        face > OBSTACLE_WALL_INNER_WEST)
        return face;
    const bool atXEnd =
        fabsf(fabsf(wallX) - FIELD_INNER_HALF_MM) < 0.1f;
    const bool atYEnd =
        fabsf(fabsf(wallY) - FIELD_INNER_HALF_MM) < 0.1f;
    if (!atXEnd || !atYEnd)
        return face;
    if (wallX < 0.0f && wallY < 0.0f) return OBSTACLE_WALL_INNER_CORNER_SW;
    if (wallX > 0.0f && wallY < 0.0f) return OBSTACLE_WALL_INNER_CORNER_SE;
    if (wallX > 0.0f && wallY > 0.0f) return OBSTACLE_WALL_INNER_CORNER_NE;
    return OBSTACLE_WALL_INNER_CORNER_NW;
}

bool calculateClearanceAtPose(
    const CandidateSeat &seat,
    float x,
    float y,
    float headingDeg,
    ObstacleClearanceSample &sample)
{
    if (!isfinite(x) || !isfinite(y) || !isfinite(headingDeg))
        return false;
    const float heading = headingDeg * PI / 180.0f;
    const float axisEndX =
        x + OBSTACLE_ROBOT_ENVELOPE_AXIS_FRONT_MM * cosf(heading);
    const float axisEndY =
        y + OBSTACLE_ROBOT_ENVELOPE_AXIS_FRONT_MM * sinf(heading);
    sample = ObstacleClearanceSample{};
    sample.valid = true;
    sample.robotXmm = x;
    sample.robotYmm = y;
    sample.robotHeadingDeg = headingDeg;
    sample.wallRobotXmm = x;
    sample.wallRobotYmm = y;
    sample.wallRobotHeadingDeg = headingDeg;
    sample.pillarMm = pointSegmentDistance(
        seat.x, seat.y, x, y, axisEndX, axisEndY) -
        OBSTACLE_MAX_WHEEL_HALF_WIDTH_MM -
        OBSTACLE_PILLAR_MOVEMENT_RADIUS_MM;

    for (const WallSegment &wall : FIELD_WALLS)
    {
        float wallX = 0.0f;
        float wallY = 0.0f;
        const float clearance = segmentDistanceToWall(
            x, y, axisEndX, axisEndY, wall, wallX, wallY) -
            OBSTACLE_MAX_WHEEL_HALF_WIDTH_MM;
        if (clearance < sample.wallMm)
        {
            sample.wallMm = clearance;
            sample.wallFeature = classifyWallFeature(
                wall.feature, wallX, wallY);
            sample.wallXmm = wallX;
            sample.wallYmm = wallY;
        }
    }

    const float cornerX[4] = {
        -FIELD_INNER_HALF_MM, FIELD_INNER_HALF_MM,
         FIELD_INNER_HALF_MM, -FIELD_INNER_HALF_MM};
    const float cornerY[4] = {
        -FIELD_INNER_HALF_MM, -FIELD_INNER_HALF_MM,
         FIELD_INNER_HALF_MM, FIELD_INNER_HALF_MM};
    for (uint8_t corner = 0; corner < 4; ++corner)
    {
        sample.innerCornerMm[corner] = pointSegmentDistance(
            cornerX[corner], cornerY[corner],
            x, y, axisEndX, axisEndY) -
            OBSTACLE_MAX_WHEEL_HALF_WIDTH_MM;
    }
    return true;
}

bool isExtremeAdjacentPair(
    uint8_t firstIndex,
    const CandidateSeat &first,
    uint8_t secondIndex,
    const CandidateSeat &second)
{
    constexpr uint8_t seatsPerSection =
        COURSE_STATIONS_PER_SECTION * COURSE_SEATS_PER_STATION;
    const int firstStation = (firstIndex % seatsPerSection) /
        COURSE_SEATS_PER_STATION;
    const int secondStation = (secondIndex % seatsPerSection) /
        COURSE_SEATS_PER_STATION;
    if (firstIndex / seatsPerSection != secondIndex / seatsPerSection ||
        abs(firstStation - secondStation) != 1)
        return false;

    const float firstTarget =
        targetLateralForSeat(first, OBSTACLE_LAP1_CLEARANCE_MM);
    const float secondTarget =
        targetLateralForSeat(second, OBSTACLE_LAP1_CLEARANCE_MM);
    return firstTarget * secondTarget < 0.0f &&
           fabsf(firstTarget) > OBSTACLE_LAP1_CLEARANCE_MM &&
           fabsf(secondTarget) > OBSTACLE_LAP1_CLEARANCE_MM;
}

bool targetsOuterExtreme(const CandidateSeat &seat)
{
    return fabsf(targetLateralForSeat(
                     seat,
                     OBSTACLE_LAP1_CLEARANCE_MM)) >
           OBSTACLE_LAP1_CLEARANCE_MM;
}

bool hasConfirmedExtremeAdjacentPair(uint8_t seatIndex)
{
    for (uint8_t otherIndex = 0;
         otherIndex < OBSTACLE_SEAT_COUNT;
         ++otherIndex)
    {
        if (otherIndex != seatIndex && seats[otherIndex].confirmed &&
            isExtremeAdjacentPair(
                seatIndex,
                seats[seatIndex],
                otherIndex,
                seats[otherIndex]))
            return true;
    }
    return false;
}

bool isSecondExtremeAdjacentSeat(uint8_t seatIndex)
{
    for (uint8_t otherIndex = 0;
         otherIndex < OBSTACLE_SEAT_COUNT;
         ++otherIndex)
    {
        if (otherIndex != seatIndex && seats[otherIndex].confirmed &&
            isExtremeAdjacentPair(
                seatIndex,
                seats[seatIndex],
                otherIndex,
                seats[otherIndex]))
            return seatIndex / COURSE_SEATS_PER_STATION >
                   otherIndex / COURSE_SEATS_PER_STATION;
    }
    return false;
}

bool upcomingAdjacentStationUnresolved(uint8_t seatIndex)
{
    const uint8_t localStation =
        (seatIndex %
         (COURSE_STATIONS_PER_SECTION * COURSE_SEATS_PER_STATION)) /
        COURSE_SEATS_PER_STATION;
    if (localStation + 1 >= COURSE_STATIONS_PER_SECTION)
        return false;
    return !stationResolved(
        seatIndex / COURSE_SEATS_PER_STATION + 1);
}

float validatedClearanceForSeat(uint8_t seatIndex)
{
    if (hasConfirmedExtremeAdjacentPair(seatIndex))
        return isSecondExtremeAdjacentSeat(seatIndex)
            ? OBSTACLE_EXTREME_ADJACENT_SECOND_CLEARANCE_MM
            : OBSTACLE_EXTREME_ADJACENT_CLEARANCE_MM;
    // Every official CW inner GREEN, including the stored return seat, sits
    // 100 mm inboard of the normal centre line. The general 260 mm route
    // drives unnecessarily far toward the outer wall (logs 455/456).
    if (routeTurnSign < 0 &&
        sectionLayoutMode == OBSTACLE_SECTION_LAYOUT_OFFICIAL &&
        seatIndex < 6 && !seats[seatIndex].red &&
        seats[seatIndex].lateralMm < 0.0f)
        return OBSTACLE_PARKING_CW_INNER_GREEN_CLEARANCE_MM;
    if (targetsOuterExtreme(seats[seatIndex]) &&
        upcomingAdjacentStationUnresolved(seatIndex))
        return OBSTACLE_OUTER_SAFE_CLEARANCE_MM;
    return OBSTACLE_LAP1_CLEARANCE_MM;
}

float optimizedClearanceForSeat(uint8_t seatIndex)
{
    if (hasConfirmedExtremeAdjacentPair(seatIndex))
        return isSecondExtremeAdjacentSeat(seatIndex)
            ? OBSTACLE_EXTREME_ADJACENT_SECOND_CLEARANCE_MM
            : OBSTACLE_EXTREME_ADJACENT_CLEARANCE_MM;
    if (routeTurnSign < 0 &&
        sectionLayoutMode == OBSTACLE_SECTION_LAYOUT_OFFICIAL &&
        seatIndex < 6 && !seats[seatIndex].red &&
        seats[seatIndex].lateralMm < 0.0f)
        return OBSTACLE_PARKING_CW_INNER_GREEN_CLEARANCE_MM;
    if (sectionLayoutMode == OBSTACLE_SECTION_LAYOUT_OFFICIAL &&
        seatIndex >= 6 && seatIndex % 6 / 2 == 1 && targetsOuterExtreme(seats[seatIndex]))
        return OBSTACLE_OPTIMIZED_MIDDLE_CLEARANCE_MM;
    if (targetsOuterExtreme(seats[seatIndex]))
        return OBSTACLE_OPTIMIZED_OUTER_CLEARANCE_MM;
    return sectionLayoutMode == OBSTACLE_SECTION_LAYOUT_OFFICIAL &&
            seatIndex >= 6 && seatIndex % 6 / 2 == 1
        ? OBSTACLE_OPTIMIZED_MODERATE_CLEARANCE_MM : OBSTACLE_LAP1_CLEARANCE_MM;
}

bool optimizedUsesOuterPlateau(uint8_t seatIndex)
{
    return targetsOuterExtreme(seats[seatIndex]) &&
           !hasConfirmedExtremeAdjacentPair(seatIndex);
}

bool completingExtremeAdjacentPair(float currentDistanceMm)
{
    if (!extremeAdjacentReleasePending || lastConfirmedSeatIndex < 0 ||
        !hasConfirmedExtremeAdjacentPair(
            static_cast<uint8_t>(lastConfirmedSeatIndex)))
    {
        extremeAdjacentReleasePending = false;
        return false;
    }

    const CandidateSeat &lastSeat = seats[lastConfirmedSeatIndex];
    const float forwardToSeat = cyclicDistanceForward(
        currentDistanceMm,
        lastSeat.pathDistanceMm);
    if (forwardToSeat < loopLengthMm * 0.5f)
        return true;
    const float distancePastSeat = loopLengthMm - forwardToSeat;
    if (distancePastSeat < OBSTACLE_EXTREME_ADJACENT_RELEASE_MM)
        return true;
    extremeAdjacentReleasePending = false;
    return false;
}

bool traversingInjectedAvoidance(float currentDistanceMm)
{
    const float approachMm = OBSTACLE_PATH_SAMPLE_MM *
        (OBSTACLE_PATH_TAPER_WAYPOINTS +
         OBSTACLE_OUTER_SAFE_APPROACH_LEAD_WAYPOINTS + 1.0f);
    for (uint8_t i = 0; i < OBSTACLE_SEAT_COUNT; ++i)
    {
        if (!seats[i].confirmed || !seats[i].injected)
            continue;
        const float forward = cyclicDistanceForward(
            currentDistanceMm, seats[i].pathDistanceMm);
        if (forward <= approachMm ||
            forward >= loopLengthMm -
                OBSTACLE_DISCOVERY_NUDGE_RESUME_PAST_PILLAR_MM)
            return true;
    }
    return false;
}

void displaceForSeat(
    PathPoint *path,
    uint8_t seatIndex,
    float clearanceMm,
    bool useOuterPlateau,
    bool earlyMiddleView = false,
    bool laterLapPlateau = false)
{
    CandidateSeat &seat = seats[seatIndex];
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
    if (earlyMiddleView)
    {
        // Check-all practice: the camera needs to inspect the station after a
        // middle green/right pillar. Start the left-side bypass earlier, then
        // flatten it before the pillar so Pure Pursuit points toward the next
        // station while the camera is still 230..600 mm away. Values are
        // lateral millimetres at 50 mm path samples, -500..+400 mm from the
        // pillar. The 160 mm offset at the pillar retains the usual 260 mm
        // centre-to-pillar clearance (right seat is at -100 mm).
        static constexpr float kEarlyViewOffsetMm[] = {
            0, 0, 20, 80, 130, 185, 205, 215, 200, 170,
            160, 160, 150, 130, 100, 70, 40, 20, 0};
        for (int offset = -10; offset <= 8; ++offset)
        {
            int index = static_cast<int>(center) + offset;
            while (index < 0) index += pathLength;
            while (index >= pathLength) index -= pathLength;
            const float lateral = kEarlyViewOffsetMm[offset + 10];
            path[index].x += normalX * lateral;
            path[index].y += normalY * lateral;
        }
        smoothRange(path, center, 10, 8);
        recomputeSpeedProfile(path);
        return;
    }
    const bool safeOuterPlateau =
        useOuterPlateau && targetsOuterExtreme(seat);
    const bool repeatedPlateau = safeOuterPlateau && laterLapPlateau;
    const int approachLeadWaypoints = safeOuterPlateau
        ? (repeatedPlateau ? OBSTACLE_LATER_LAP_OUTER_PLATEAU_WAYPOINTS
                         : OBSTACLE_OUTER_SAFE_APPROACH_LEAD_WAYPOINTS)
        : 0;
    const int exitHoldWaypoints = safeOuterPlateau
        ? (repeatedPlateau ? OBSTACLE_LATER_LAP_OUTER_PLATEAU_WAYPOINTS
                         : OBSTACLE_OUTER_SAFE_EXIT_HOLD_WAYPOINTS)
        : 0;

    for (int offset =
             -OBSTACLE_PATH_TAPER_WAYPOINTS - approachLeadWaypoints;
         offset <= OBSTACLE_PATH_TAPER_WAYPOINTS + exitHoldWaypoints;
         ++offset)
    {
        int index = static_cast<int>(center) + offset;
        while (index < 0)
            index += pathLength;
        while (index >= pathLength)
            index -= pathLength;
        // A positive offset is after the pillar in travel direction. The
        // safe outer-seat route reaches and holds its peak for one extra
        // waypoint on each side so the front and rear wheels both clear.
        const int effectiveOffset = offset <= 0
            ? (-offset > approachLeadWaypoints
                   ? -offset - approachLeadWaypoints
                   : 0)
            : (offset > exitHoldWaypoints
                   ? offset - exitHoldWaypoints
                   : 0);
        const float taper = 1.0f -
            static_cast<float>(effectiveOffset) /
                (OBSTACLE_PATH_TAPER_WAYPOINTS + 1.0f);
        path[index].x += normalX * targetLateral * taper;
        path[index].y += normalY * targetLateral * taper;
    }

    smoothRange(
        path,
        center,
        OBSTACLE_PATH_TAPER_WAYPOINTS + approachLeadWaypoints,
        OBSTACLE_PATH_TAPER_WAYPOINTS + exitHoldWaypoints);
    recomputeSpeedProfile(path);
}

bool earlyMiddleViewEligible(uint8_t seatIndex)
{
    if (sectionLayoutMode != OBSTACLE_SECTION_LAYOUT_CHECK_ALL ||
        runtimeTestMode || seatIndex >= OBSTACLE_SEAT_COUNT ||
        seatIndex % 6 != 2 || seats[seatIndex].red ||
        seats[seatIndex].lateralMm >= 0.0f)
        return false;

    const uint8_t station = seatIndex / COURSE_SEATS_PER_STATION;
    // This approach starts near the preceding station. It must be physically
    // empty, and the following station must still need an individual check.
    if (!discoveryStations[station - 1].observedClear ||
        stationResolved(station + 1))
        return false;
    const float forward = cyclicDistanceForward(
        baselinePath[progressIndex].distanceMm,
        seats[seatIndex].pathDistanceMm);
    if (earlyMiddleViewActive[seatIndex])
        return true;
    if (forward >= 400.0f && forward < loopLengthMm * 0.5f)
    {
        earlyMiddleViewActive[seatIndex] = true;
        return true;
    }
    return false;
}

bool earlyMiddleViewPathSafe(const PathPoint *path, uint8_t seatIndex)
{
    const int center = nearestPathIndex(
        baselinePath, seats[seatIndex].x, seats[seatIndex].y,
        0, pathLength);
    const uint8_t middleStation = seatIndex / COURSE_SEATS_PER_STATION;
    const uint8_t previousStation = middleStation - 1;
    for (int offset = -10; offset <= 3; ++offset)
    {
        int index = center + offset;
        while (index < 0) index += pathLength;
        while (index >= pathLength) index -= pathLength;
        const uint16_t previous = (index + pathLength - 1) % pathLength;
        const uint16_t next = (index + 1) % pathLength;
        const float dx = path[next].x - path[previous].x;
        const float dy = path[next].y - path[previous].y;
        const float heading = atan2f(dy, dx) * 180.0f / PI;
        if (!isfinite(heading))
            return false;
        if (offset > -10)
        {
            const float beforeHeading = atan2f(
                path[index].y - path[previous].y,
                path[index].x - path[previous].x);
            const float afterHeading = atan2f(
                path[next].y - path[index].y,
                path[next].x - path[index].x);
            const float segment = fmaxf(1.0f, hypotf(
                path[next].x - path[index].x,
                path[next].y - path[index].y));
            const float curvature = fabsf(
                wrap180((afterHeading - beforeHeading) * 180.0f / PI) *
                PI / 180.0f) / segment;
            if (atanf(OBSTACLE_WHEELBASE_MM * curvature) * 180.0f / PI >
                OBSTACLE_MAX_PURSUIT_STEERING_DEG)
                return false;
        }
        for (uint8_t half = 0; half < 2; ++half)
        {
            const float x = half == 0 ? path[index].x
                : 0.5f * (path[index].x + path[next].x);
            const float y = half == 0 ? path[index].y
                : 0.5f * (path[index].y + path[next].y);
            for (uint8_t other = 0; other < OBSTACLE_SEAT_COUNT; ++other)
            {
                // The preceding station has two direct camera CLEAR records;
                // one confirmed middle pillar excludes its paired seat.
                if (other / COURSE_SEATS_PER_STATION == previousStation ||
                    (other / COURSE_SEATS_PER_STATION == middleStation &&
                     other != seatIndex))
                    continue;
                ObstacleClearanceSample sample{};
                if (!calculateClearanceAtPose(
                        seats[other], x, y, heading, sample) ||
                    sample.wallMm < 40.0f || sample.pillarMm < 40.0f)
                    return false;
            }
        }
    }
    return true;
}

void joinSameColourStraightEnds(PathPoint *route, bool injectedOnly);
void roundKnownCornerPairs(PathPoint *route, bool injectedOnly = false);

void rebuildLivePath()
{
    memcpy(livePath, baselinePath, sizeof(PathPoint) * pathLength);
    int8_t earlyViewSeat = -1;
    for (uint8_t i = 0; i < OBSTACLE_SEAT_COUNT; ++i)
    {
        if (seats[i].confirmed && seats[i].injected)
        {
            const float clearance = validatedClearanceForSeat(i);
            const bool parkingEntryGreenPlateau =
                routeTurnSign > 0 && i == 5 && !seats[i].red;
            const bool earlyView = earlyMiddleViewEligible(i);
            displaceForSeat(
                livePath,
                i,
                clearance,
                parkingEntryGreenPlateau ||
                    fabsf(
                        clearance - OBSTACLE_OUTER_SAFE_CLEARANCE_MM) < 0.1f,
                earlyView);
            if (earlyView)
                earlyViewSeat = static_cast<int8_t>(i);
        }
    }
    if (earlyViewSeat >= 0)
    {
        if (earlyMiddleViewPathSafe(
                livePath, static_cast<uint8_t>(earlyViewSeat)))
        {
            Serial.print("[PATH EARLY VIEW] preflight PASS seat=");
            Serial.println(earlyViewSeat);
        }
        else
        {
            // Retain the established bypass and its unresolved-station hold
            // if a different layout, wall, or curvature invalidates the view.
            Serial.print("[PATH EARLY VIEW] preflight FAIL seat=");
            Serial.println(earlyViewSeat);
            memcpy(livePath, baselinePath, sizeof(PathPoint) * pathLength);
            for (uint8_t i = 0; i < OBSTACLE_SEAT_COUNT; ++i)
            {
                if (!seats[i].confirmed || !seats[i].injected)
                    continue;
                const float clearance = validatedClearanceForSeat(i);
                const bool parkingEntryGreenPlateau =
                    routeTurnSign > 0 && i == 5 && !seats[i].red;
                displaceForSeat(
                    livePath, i, clearance,
                    parkingEntryGreenPlateau ||
                        fabsf(clearance -
                              OBSTACLE_OUTER_SAFE_CLEARANCE_MM) < 0.1f);
            }
        }
    }
    // Once both adjoining signs have actually been injected, lap 1 also
    // needs a continuous corner rather than the sum of two straight tapers.
    // Stored behind-start observations must not alter this first departure.
    joinSameColourStraightEnds(livePath, true);
    roundKnownCornerPairs(livePath, true);
    recomputeSpeedProfile(livePath);
    telemetryRouteDirty=true;
}

int8_t earlierExtremeAdjacentSeat(uint8_t seatIndex)
{
    for (uint8_t other = 0; other < OBSTACLE_SEAT_COUNT; ++other)
    {
        if (other < seatIndex && seats[other].confirmed &&
            seats[other].injected &&
            isExtremeAdjacentPair(
                other, seats[other], seatIndex, seats[seatIndex]))
            return static_cast<int8_t>(other);
    }
    return -1;
}

void printInjectedSeat(uint8_t seatIndex, bool delayed)
{
    Serial.print("[PATH] Live avoidance injected seat=");
    Serial.print(seatIndex);
    Serial.print(" color=");
    Serial.print(seats[seatIndex].red ? "RED" : "GREEN");
    Serial.print(" clearance_mm=");
    Serial.print(validatedClearanceForSeat(seatIndex), 0);
    if (delayed)
        Serial.print(" delayed_until_first_clear=yes");
    Serial.println();
}

void injectSeat(uint8_t seatIndex, bool delayed)
{
    CandidateSeat &seat = seats[seatIndex];
    if (seat.injected)
        return;
    seat.injected = true;
    if (injectionCount < 65535)
        ++injectionCount;
    rebuildLivePath();
    if (parkingEntryConnectorActive)
    {
        parkingEntryConnectorChangedSeat =
            parkingEntryConnectorReplanPending ? -2 :
                static_cast<int8_t>(seatIndex);
        parkingEntryConnectorReplanPending = true;
    }
    ObstacleClearanceSample snapshot;
    if (obstacle_path_get_planned_clearance(seatIndex, snapshot))
    {
        plannedClearanceAtInjection[seatIndex] = snapshot;
        plannedClearanceSnapshotValid[seatIndex] = true;
    }
    printInjectedSeat(seatIndex, delayed);
    const auto injectionPose = get_position_struct();
    Serial.print("[PATH INJECT POSE] seat/x/y/heading=");
    Serial.print(seatIndex); Serial.print('/');
    Serial.print(injectionPose.x_mm, 1); Serial.print('/');
    Serial.print(injectionPose.y_mm, 1); Serial.print('/');
    Serial.println(injectionPose.heading_deg, 2);
}

void activateDeferredInjection(float currentDistanceMm)
{
    if (deferredInjectionSeatIndex < 0)
        return;
    const uint8_t deferred =
        static_cast<uint8_t>(deferredInjectionSeatIndex);
    const int8_t earlier = earlierExtremeAdjacentSeat(deferred);
    if (earlier < 0)
    {
        injectSeat(deferred, true);
        deferredInjectionSeatIndex = -1;
        return;
    }
    const float forwardToFirst = cyclicDistanceForward(
        currentDistanceMm, seats[earlier].pathDistanceMm);
    if (forwardToFirst < loopLengthMm * 0.5f)
        return;
    const float distancePastFirst = loopLengthMm - forwardToFirst;
    if (distancePastFirst < OBSTACLE_EXTREME_ADJACENT_INJECTION_DELAY_MM)
        return;
    injectSeat(deferred, true);
    deferredInjectionSeatIndex = -1;
}

void activateCwStoredSeat(float currentDistanceMm)
{
    if (parkingCwStoredSeat < 0 || parkingEntryConnectorActive)
        return;
    const uint8_t stored = static_cast<uint8_t>(parkingCwStoredSeat);
    if (cyclicDistanceForward(currentDistanceMm, seats[stored].pathDistanceMm) >
        OBSTACLE_PARKING_CW_STORED_SEAT_APPROACH_MM)
        return;
    parkingCwStoredSeat = -1;
    injectSeat(stored, true);
    Serial.print("[CW START] Stored bypass activated on later approach seat=");
    Serial.println(stored);
}

void activateCcwStoredSeats(float currentDistanceMm)
{
    if (parkingEntryConnectorActive || !parkingCcwShortStart)
        return;
    for (uint8_t seatIndex = 0; seatIndex < 4; ++seatIndex)
    {
        const uint8_t bit = static_cast<uint8_t>(1U << seatIndex);
        if (!(parkingCcwStoredSeatMask & bit) ||
            cyclicDistanceForward(currentDistanceMm,
                seats[seatIndex].pathDistanceMm) >
                    OBSTACLE_PARKING_CCW_STORED_SEAT_APPROACH_MM)
            continue;
        parkingCcwStoredSeatMask &= static_cast<uint8_t>(~bit);
        injectSeat(seatIndex, true);
        Serial.print("[CCW START] Stored bypass activated on later approach seat=");
        Serial.println(seatIndex);
    }
}

bool laterLapMapValid()
{
    if (!allStationsResolved())
        return false;
    for (uint8_t section = 0; section < COURSE_SECTION_COUNT; ++section)
    {
        uint8_t count = 0;
        bool middle = false;
        for (uint8_t station = 0; station < COURSE_STATIONS_PER_SECTION; ++station)
        {
            const uint8_t first = section * 6 + station * 2;
            if (seats[first].confirmed && seats[first + 1].confirmed)
                return false; // Two colours/places at one longitudinal station.
            const bool occupied = seats[first].confirmed || seats[first + 1].confirmed;
            if (occupied) ++count;
            middle = middle || (station == 1 && occupied);
        }
        if (sectionLayoutMode == OBSTACLE_SECTION_LAYOUT_OFFICIAL &&
            (count > 2 || (middle && count > 1)))
            return false;
    }
    return true;
}

void preserveLaterLapSeam(PathPoint *route)
{
    for (uint16_t i = 0; i < pathLength; ++i)
    {
        const float distance = fminf(baselinePath[i].distanceMm,
            loopLengthMm - baselinePath[i].distanceMm);
        float blend = clampFloat((distance - OBSTACLE_LATER_LAP_SEAM_KEEP_MM) /
            OBSTACLE_LATER_LAP_SEAM_BLEND_MM, 0.0f, 1.0f);
        blend = blend * blend * (3.0f - 2.0f * blend);
        route[i].x = livePath[i].x + blend * (route[i].x - livePath[i].x);
        route[i].y = livePath[i].y + blend * (route[i].y - livePath[i].y);
    }
}

bool laterLapRouteSafe(const PathPoint *route,
                       float *minimumWall = nullptr, float *minimumPillar = nullptr)
{
    if (minimumWall) *minimumWall=INFINITY;
    if (minimumPillar) *minimumPillar=INFINITY;
    if (pathLength < 3)
        return false;
    // Check the physical pose implied by each route tangent, including the
    // seam and every confirmed pillar. This is a geometric preflight, not a
    // substitute for motor/sensor tracking validation during the test laps.
    for (uint16_t i = 0; i < pathLength; ++i)
    {
        const uint16_t previous = (i + pathLength - 1) % pathLength;
        const uint16_t next = (i + 1) % pathLength;
        const float heading = atan2f(route[next].y - route[previous].y,
                                    route[next].x - route[previous].x) * 180.0f / PI;
        if (!isfinite(route[i].x) || !isfinite(route[i].y) || !isfinite(heading))
            return false;
        // The start bay remains on the field during repeated laps. These
        // body/wheel checks omit initial-start half-plane restrictions.
        if (!parking_start_footprint::safe(route[i].x, route[i].y,
                heading, -1, false) ||
            !parking_start_footprint::safe(route[i].x, route[i].y,
                heading, 1, false))
        {
            Serial.print("[LAPS] Parking-piece footprint rejected index=");
            Serial.println(i);
            return false;
        }
        // Seat zero also supplies the complete field-wall geometry on empty
        // tracks. No physical pillar is assumed there unless it is confirmed.
        for (uint8_t seatIndex = 0; seatIndex < OBSTACLE_SEAT_COUNT; ++seatIndex)
        {
            if (seatIndex != 0 && !seats[seatIndex].confirmed)
                continue;
            ObstacleClearanceSample sample{};
            if (!calculateClearanceAtPose(seats[seatIndex], route[i].x,
                    route[i].y, heading, sample) ||
                sample.wallMm <= OBSTACLE_LATER_LAP_ROUTE_RESERVE_MM ||
                (seats[seatIndex].confirmed &&
                 sample.pillarMm <= OBSTACLE_LATER_LAP_ROUTE_RESERVE_MM))
            {
                Serial.print("[LAPS] Route preflight rejected index/seat/wall/pillar=");
                Serial.print(i); Serial.print("/"); Serial.print(seatIndex);
                Serial.print("/"); Serial.print(sample.wallMm, 1);
                Serial.print("/"); Serial.println(sample.pillarMm, 1);
                return false;
            }
            if (minimumWall) *minimumWall=fminf(*minimumWall,sample.wallMm);
            if (minimumPillar && seats[seatIndex].confirmed)
                *minimumPillar=fminf(*minimumPillar,sample.pillarMm);
        }
    }
    return true;
}

// Two confirmed end signs with the same passing colour need one continuous
// straight bypass. Avoid returning to the centre between their tapers.
// Official cards exclude a middle sign. O3 must retain independent geometry.
void joinSameColourStraightEnds(PathPoint *route, bool injectedOnly)
{
    if (sectionLayoutMode != OBSTACLE_SECTION_LAYOUT_OFFICIAL) return;
    for (uint8_t section=0; section<4; ++section)
    {
        const uint8_t first=section*6;
        int a=-1,b=-1;
        for (uint8_t side=0; side<2; ++side)
        {
            if (seats[first+side].confirmed && (!injectedOnly || seats[first+side].injected)) a=first+side;
            if (seats[first+4+side].confirmed && (!injectedOnly || seats[first+4+side].injected)) b=first+4+side;
        }
        if (a<0 || b<0 || seats[a].red!=seats[b].red ||
            seats[first+2].confirmed || seats[first+3].confirmed) continue;
        const uint16_t ia=nearestPathIndex(baselinePath,seats[a].x,seats[a].y,0,pathLength);
        const uint16_t ib=nearestPathIndex(baselinePath,seats[b].x,seats[b].y,0,pathLength);
        const float h=seats[a].headingDeg*PI/180.0f, nx=-sinf(h), ny=cosf(h);
        const float offsetA=(route[ia].x-baselinePath[ia].x)*nx+(route[ia].y-baselinePath[ia].y)*ny;
        const float offsetB=(route[ib].x-baselinePath[ib].x)*nx+(route[ib].y-baselinePath[ib].y)*ny;
        const float span=cyclicDistanceForward(baselinePath[ia].distanceMm,baselinePath[ib].distanceMm);
        if (span<900.0f || span>1100.0f) continue;
        for(uint16_t i=0;i<pathLength;++i)
        {
            const float d=cyclicDistanceForward(baselinePath[ia].distanceMm,baselinePath[i].distanceMm);
            if (d>span) continue;
            float t=d/span; t=t*t*(3.0f-2.0f*t);
            const float offset=offsetA+t*(offsetB-offsetA);
            route[i].x=baselinePath[i].x+nx*offset;
            route[i].y=baselinePath[i].y+ny*offset;
        }
        if(injectedOnly){Serial.print("[PATH] Same-colour straight joined seats=");Serial.print(a);Serial.print('/');Serial.println(b);}
    }
}

void roundKnownCornerPairs(PathPoint *route, bool injectedOnly)
{
    // Two signs with the same passing colour at adjoining section ends need
    // one continuous corner. Summing two straight avoidance tapers can fold
    // the route back on itself. Join their actual endpoint offsets radially
    // around the existing corner centre instead; keep all straight geometry.
    for (uint8_t corner = 0; corner < 4; ++corner)
    {
        const uint8_t endFirst = corner * 6 + 4;
        const uint8_t nextFirst = ((corner + 1) % 4) * 6;
        int endSeat = -1, nextSeat = -1;
        for (uint8_t side = 0; side < 2; ++side)
        {
            if (seats[endFirst + side].confirmed &&
                (!injectedOnly || seats[endFirst + side].injected))
                endSeat = endFirst + side;
            if (seats[nextFirst + side].confirmed &&
                (!injectedOnly || seats[nextFirst + side].injected))
                nextSeat = nextFirst + side;
        }
        if (endSeat < 0 || nextSeat < 0 ||
            seats[endSeat].red != seats[nextSeat].red)
            continue;
        uint16_t first = 0, last = 0;
        for (uint16_t i = 0; i < pathLength; ++i)
        {
            if (fabsf(baselinePath[i].distanceMm - corners[corner].pathStartMm) < 1)
                first = i;
            if (fabsf(baselinePath[i].distanceMm - corners[corner].pathEndMm) < 1)
                last = i;
        }
        if (last <= first) continue;
        if (injectedOnly)
        {
            Serial.print("[PATH] Discovery corner rounded corner/end/next=");
            Serial.print(corner); Serial.print('/');
            Serial.print(endSeat); Serial.print('/');
            Serial.println(nextSeat);
        }
        const float heading = baselinePath[first].headingDeg * PI / 180.0f;
        const float centreX = baselinePath[first].x -
            routeTurnSign * OBSTACLE_CORNER_RADIUS_MM * sinf(heading);
        const float centreY = baselinePath[first].y +
            routeTurnSign * OBSTACLE_CORNER_RADIUS_MM * cosf(heading);
        const float firstRadius = hypotf(route[first].x - centreX,
                                         route[first].y - centreY);
        const float lastRadius = hypotf(route[last].x - centreX,
                                        route[last].y - centreY);
        for (uint16_t i = first; i <= last; ++i)
        {
            float t = (baselinePath[i].distanceMm - corners[corner].pathStartMm) /
                (corners[corner].pathEndMm - corners[corner].pathStartMm);
            t = t * t * (3.0f - 2.0f * t);
            const float radius = firstRadius + t * (lastRadius - firstRadius);
            const float scale = radius / OBSTACLE_CORNER_RADIUS_MM;
            route[i].x = centreX + (baselinePath[i].x - centreX) * scale;
            route[i].y = centreY + (baselinePath[i].y - centreY) * scale;
        }
    }
}

// Official repeated laps: retain an already safe inner lane through the
// corner when the next section has a same-colour solitary middle sign.
// Returning to the centre before taking the same side again wastes a turn.
// Discovery/O3 geometry and all opposite-colour transitions stay independent.
void carryKnownInnerLaneToMiddle(PathPoint *route)
{
    if (sectionLayoutMode != OBSTACLE_SECTION_LAYOUT_OFFICIAL) return;
    for (uint8_t corner = 0; corner < 4; ++corner)
    {
        const uint8_t endFirst = corner * 6 + 4;
        const uint8_t nextFirst = ((corner + 1) % 4) * 6;
        // The start-section seam is preserved separately. A carry into it
        // would overwrite that blend and narrow a previously safe start pass.
        if (nextFirst == 0) continue;
        int endSeat = -1, middleSeat = -1;
        for (uint8_t side = 0; side < 2; ++side)
        {
            if (seats[endFirst + side].confirmed) endSeat = endFirst + side;
            if (seats[nextFirst + 2 + side].confirmed) middleSeat = nextFirst + 2 + side;
        }
        if (endSeat < 0 || middleSeat < 0 ||
            seats[endSeat].red != seats[middleSeat].red ||
            seats[nextFirst].confirmed || seats[nextFirst + 1].confirmed ||
            seats[nextFirst + 4].confirmed || seats[nextFirst + 5].confirmed)
            continue;
        uint16_t first = 0, last = 0;
        for (uint16_t i = 0; i < pathLength; ++i)
        {
            if (fabsf(baselinePath[i].distanceMm-corners[corner].pathStartMm)<1) first=i;
            if (fabsf(baselinePath[i].distanceMm-corners[corner].pathEndMm)<1) last=i;
        }
        if (last <= first) continue;
        const float h = baselinePath[first].headingDeg * PI / 180.0f;
        const float cx = baselinePath[first].x-routeTurnSign*OBSTACLE_CORNER_RADIUS_MM*sinf(h);
        const float cy = baselinePath[first].y+routeTurnSign*OBSTACLE_CORNER_RADIUS_MM*cosf(h);
        const float radius = hypotf(route[first].x-cx,route[first].y-cy);
        // Never carry a tighter radius than the existing 180mm model bound,
        // nor convert an outer route into this optional inner-lane shortcut.
        if (radius < 180.0f || radius >= OBSTACLE_CORNER_RADIUS_MM-20.0f) continue;
        const float oldEndX=route[last].x, oldEndY=route[last].y;
        for (uint16_t i=first;i<=last;++i)
        {
            const float scale=radius/OBSTACLE_CORNER_RADIUS_MM;
            route[i].x=cx+(baselinePath[i].x-cx)*scale;
            route[i].y=cy+(baselinePath[i].y-cy)*scale;
        }
        const float extraX=route[last].x-oldEndX, extraY=route[last].y-oldEndY;
        // Blend to the existing middle bypass over300mm, before the middle
        // station500mm after entry. Retain its proven pillar plateau.
        for(uint16_t step=1;step<pathLength;++step)
        {
            const uint16_t i=(last+step)%pathLength;
            const float distance=cyclicDistanceForward(
                baselinePath[last].distanceMm,baselinePath[i].distanceMm);
            if(distance>=300.0f)break;
            float t=distance/300.0f;t=t*t*(3.0f-2.0f*t);
            route[i].x+=extraX*(1.0f-t);route[i].y+=extraY*(1.0f-t);
        }
        Serial.print("[LAPS] Inner lane carried corner/end/middle/radius=");
        Serial.print(corner);Serial.print("/");Serial.print(endSeat);Serial.print("/");
        Serial.print(middleSeat);Serial.print("/");Serial.println(radius,1);
    }
}

float laterLapBending(const PathPoint *route, float &length, float &peak)
{
    length=peak=0.0f;
    float bending=0.0f;
    for(uint16_t i=0;i<pathLength;++i)
    {
        const uint16_t p=(i+pathLength-1)%pathLength,n=(i+1)%pathLength;
        const float a=hypotf(route[i].x-route[p].x,route[i].y-route[p].y);
        const float b=hypotf(route[n].x-route[i].x,route[n].y-route[i].y);
        length+=b;
        if(a<1.0f || b<1.0f)continue; // coincident lap-seam point
        const float h1=atan2f(route[i].y-route[p].y,route[i].x-route[p].x);
        const float h2=atan2f(route[n].y-route[i].y,route[n].x-route[i].x);
        const float turn=wrap180((h2-h1)*180.0f/PI)*PI/180.0f;
        const float ds=0.5f*(a+b);
        bending+=turn*turn/ds;
        peak=fmaxf(peak,fabsf(turn)/ds);
    }
    return bending;
}

// Small shape changes after the map is complete. Keep pillar passing policy,
// lap seam and metadata intact; reject any shortcut losing useful reserve.
void smoothKnownLaterLapRoute(PathPoint *route, PathPoint *rollback,
                             float oldWall, float oldPillar)
{
    memcpy(rollback,route,sizeof(PathPoint)*pathLength);
    float oldLength,oldPeak;
    const float oldBending=laterLapBending(route,oldLength,oldPeak);
    constexpr float weights[]={1,4,6,4,1};
    for(uint8_t pass=0;pass<4;++pass)
    {
        memcpy(smoothingBuffer,route,sizeof(PathPoint)*pathLength);
        for(uint16_t i=0;i<pathLength;++i)
        {
            const float seam=fminf(baselinePath[i].distanceMm,
                loopLengthMm-baselinePath[i].distanceMm);
            const float keep=OBSTACLE_LATER_LAP_SEAM_KEEP_MM+OBSTACLE_LATER_LAP_SEAM_BLEND_MM;
            if(seam<=keep)continue;
            // Fade the refinement in over 150 mm; avoid a new kink at the
            // protected seam boundary. Symmetric stencil has no directional bias.
            float fade=clampFloat((seam-keep)/150.0f,0.0f,1.0f);
            fade=fade*fade*(3.0f-2.0f*fade);
            // Leave the actual pillar-passing plateau intact. Refine its
            // approach/exit only, with a gradual transition outside 150 mm.
            for(uint8_t seat=0;seat<OBSTACLE_SEAT_COUNT;++seat)
            {
                if(!seats[seat].confirmed)continue;
                const float ahead=cyclicDistanceForward(seats[seat].pathDistanceMm,
                    baselinePath[i].distanceMm);
                const float distance=fminf(ahead,loopLengthMm-ahead);
                float retain=clampFloat((distance-150.0f)/150.0f,0.0f,1.0f);
                retain=retain*retain*(3.0f-2.0f*retain);
                fade=fminf(fade,retain);
            }
            float x=0,y=0;
            for(int k=-2;k<=2;++k)
            {
                const uint16_t j=(i+pathLength+k)%pathLength;
                x+=weights[k+2]*smoothingBuffer[j].x/16.0f;
                y+=weights[k+2]*smoothingBuffer[j].y/16.0f;
            }
            x=smoothingBuffer[i].x+0.5f*fade*(x-smoothingBuffer[i].x);
            y=smoothingBuffer[i].y+0.5f*fade*(y-smoothingBuffer[i].y);
            const float dx=x-rollback[i].x,dy=y-rollback[i].y;
            const float shift=hypotf(dx,dy);
            // Smooth saturation avoids a new kink where the displacement
            // limit becomes active; shift*scale is always below that limit.
            const float relative=shift/OBSTACLE_LATER_LAP_SMOOTH_MAX_SHIFT_MM;
            const float scale=1.0f/sqrtf(1.0f+relative*relative);
            route[i].x=rollback[i].x+scale*dx;
            route[i].y=rollback[i].y+scale*dy;
        }
    }
    float length,peak,wall,pillar;
    const float bending=laterLapBending(route,length,peak);
    const bool accept=length<=oldLength+0.1f && bending<oldBending*0.98f &&
        peak<=oldPeak+1e-7f && laterLapRouteSafe(route,&wall,&pillar) &&
        wall>=OBSTACLE_LATER_LAP_SMOOTH_RESERVE_MM &&
        pillar>=OBSTACLE_LATER_LAP_SMOOTH_RESERVE_MM &&
        wall>=oldWall-OBSTACLE_LATER_LAP_SMOOTH_RESERVE_LOSS_MM &&
        pillar>=oldPillar-OBSTACLE_LATER_LAP_SMOOTH_RESERVE_LOSS_MM;
    if(!accept)
    {
        memcpy(route,rollback,sizeof(PathPoint)*pathLength);
        Serial.println("[LAPS] Smoothing rejected; checked prior shape retained");
        return;
    }
    Serial.print("[LAPS] Smooth route length_before_after_mm=");
    Serial.print(oldLength,1);Serial.print('/');Serial.print(length,1);
    Serial.print(" bending_reduction_pct=");
    Serial.println(100.0f*(oldBending-bending)/fmaxf(oldBending,1e-9f),1);
}

bool buildOptimizedPath()
{
    memcpy(optimizedPath, baselinePath, sizeof(PathPoint) * pathLength);
    for (uint8_t i = 0; i < OBSTACLE_SEAT_COUNT; ++i)
    {
        if (seats[i].confirmed)
        {
            const float clearance = optimizedClearanceForSeat(i);
            displaceForSeat(
                optimizedPath,
                i,
                clearance,
                optimizedUsesOuterPlateau(i), false,
                sectionLayoutMode == OBSTACLE_SECTION_LAYOUT_OFFICIAL);
            Serial.print("[PATH] Later-lap avoidance seat=");
            Serial.print(i);
            Serial.print(" color=");
            Serial.print(seats[i].red ? "RED" : "GREEN");
            Serial.print(" clearance_mm=");
            Serial.println(clearance, 0);
        }
    }
    preserveLaterLapSeam(optimizedPath);
    joinSameColourStraightEnds(optimizedPath, false);
    roundKnownCornerPairs(optimizedPath);
    // Optional shorter lane must pass the same full-footprint preflight.
    // Keep the prior optimized shape if this candidate is not safe.
    PathPoint beforeLaneCarry[OBSTACLE_MAX_PATH_WAYPOINTS];
    memcpy(beforeLaneCarry, optimizedPath, sizeof(PathPoint)*pathLength);
    carryKnownInnerLaneToMiddle(optimizedPath);
    float routeWall=INFINITY,routePillar=INFINITY;
    bool routeSafe = laterLapRouteSafe(optimizedPath,&routeWall,&routePillar);
    if (!routeSafe)
    {
        memcpy(optimizedPath, beforeLaneCarry, sizeof(PathPoint)*pathLength);
        Serial.println("[LAPS] Inner-lane shortcut rejected; prior optimized route retained");
        routeSafe = laterLapRouteSafe(optimizedPath,&routeWall,&routePillar);
    }
    if (!routeSafe)
    {
        // A complete map does not automatically establish a safe new shape.
        // Keep the learned route only if the same geometric gates accept it.
        memcpy(optimizedPath, livePath, sizeof(PathPoint) * pathLength);
        if (!laterLapRouteSafe(optimizedPath,&routeWall,&routePillar))
        {
            laterLapPlanRejected = true;
            Serial.println("[LAPS] Both later-lap and learned routes rejected - held");
            return false;
        }
        Serial.println("[LAPS] Later-lap shape rejected; using checked learned route");
    }
    if(sectionLayoutMode==OBSTACLE_SECTION_LAYOUT_OFFICIAL)
        smoothKnownLaterLapRoute(optimizedPath,beforeLaneCarry,routeWall,routePillar);
    recomputeSpeedProfile(optimizedPath);
    optimizedBuilt = true;
    // Lap-1 passage reports have already consumed their injection snapshots.
    // Replace them with the actual later-lap route for lap-2/3 diagnostics.
    for (uint8_t i = 0; i < OBSTACLE_SEAT_COUNT; ++i)
    {
        if (!seats[i].confirmed)
            continue;
        plannedClearanceSnapshotValid[i] = false;
        ObstacleClearanceSample snapshot;
        if (obstacle_path_get_planned_clearance(i, snapshot))
        {
            plannedClearanceAtInjection[i] = snapshot;
            plannedClearanceSnapshotValid[i] = true;
        }
    }
    Serial.println(
        "[PATH] Optimized laps 2-3 path built "
        "clearance_policy=validated-layout");
    return true;
}

bool completePendingLap()
{
    if (!lapBoundaryPending || laterLapPlanRejected)
        return false;
    if (!runtimeTestMode && completedLaps == 0 && !laterLapMapValid())
    {
        if (!lapBoundaryHoldLogged)
        {
            lapBoundaryHoldLogged = true;
            Serial.println("[LAPS] Boundary held: map incomplete or contradictory; no blind lap 2");
        }
        return false;
    }
    if (!runtimeTestMode && completedLaps == 0 && runtimeLapTarget > 1 &&
        !buildOptimizedPath())
        return false;
    lapBoundaryPending = false;
    lapBoundaryHoldLogged = false;
    ++completedLaps;
    run_telemetry_lap(completedLaps);
    telemetryRouteDirty=true;
    if (completedLaps == 1 && optimizedBuilt)
    {
        Serial.print("[LAPS] Recorded speed factor/max_mm_s=");
        Serial.print(OBSTACLE_LATER_LAP_SPEED_FACTOR, 2);
        Serial.print("/");
        Serial.println(OBSTACLE_PATH_MAX_SPEED * OBSTACLE_LATER_LAP_SPEED_FACTOR, 0);
    }
    Serial.print("[PATH] Completed lap "); Serial.println(completedLaps);
    Serial.print("[LAPS] stage=");
    Serial.println(completedLaps >= runtimeLapTarget ? "FINAL_RUNOUT" :
        (completedLaps == 1 ? "LAP_2_RECORDED_MAP" : "LAP_3_RECORDED_MAP"));
    if (completedLaps >= runtimeLapTarget)
    {
        lapFinishPending = runtimeLapTarget == 3 && !runtimeTestMode;
        finished = !lapFinishPending;
    }
    return true;
}

bool finalCornerVehicleClear(const PositionEstimate &pose)
{
    if (!isfinite(pose.x_mm) || !isfinite(pose.y_mm) || !isfinite(pose.heading_deg))
        return false;
    // Conservative rectangle inside the south straight, beyond the last
    // corner's diagonal border. Check the complete vehicle, including wheels;
    // a nearest-path index or the camera crossing is insufficient.
    using namespace parking_start_footprint;
    for (int steeringSign : {-1,1})
    {
        Quad bodies[6];
        robotQuads(pose.x_mm,pose.y_mm,pose.heading_deg,steeringSign,10.0f,bodies);
        for (const Quad &body:bodies) for (const Point &p:body.p)
            if (p.x<=-OBSTACLE_STRAIGHT_LENGTH_MM*0.5f+OBSTACLE_FINAL_CORNER_POSITION_RESERVE_MM ||
                p.x>= OBSTACLE_STRAIGHT_LENGTH_MM*0.5f-OBSTACLE_FINAL_CORNER_POSITION_RESERVE_MM ||
                p.y<= OBSTACLE_SOUTH_OUTER_WALL_Y_MM || p.y>=-500.0f)
                return false;
    }
    return true;
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

    // Laps 1/2 retain their map/seam handover. On the last lap the rules
    // allow parking as soon as the whole vehicle leaves the last corner.
    if (!runtimeTestMode && runtimeLapTarget==3 && completedLaps==2 &&
        lapCountingArmed && optimizedBuilt &&
        baselinePath[progressIndex].distanceMm>=corners[3].pathEndMm &&
        finalCornerVehicleClear(pose))
    {
        lapCountingArmed=lapBoundaryPending=lapFinishPending=false;
        completedLaps=3; finished=true;
        run_telemetry_lap(3);
        Serial.println("[PATH] Completed lap 3");
        Serial.println("[LAPS] stage=FINISH; complete vehicle clear of last corner; direct parking");
        return;
    }

    if (lapFinishPending)
    {
        const float startHeading=baselinePath[0].headingDeg*PI/180.0f;
        const float actualRunout=(pose.x_mm-baselinePath[0].x)*cosf(startHeading)+
            (pose.y_mm-baselinePath[0].y)*sinf(startHeading);
        if (baselinePath[progressIndex].distanceMm >= OBSTACLE_LAP_FINISH_RUNOUT_MM &&
            baselinePath[progressIndex].distanceMm < loopLengthMm * 0.25f &&
            actualRunout >= OBSTACLE_LAP_FINISH_RUNOUT_MM)
        {
            lapFinishPending = false;
            finished = true;
            Serial.println("[LAPS] stage=FINISH; start-section runout complete");
        }
        return; // Never count a fourth lap during the final runout.
    }

    if (progressIndex >= pathLength / 4 &&
        progressIndex <= pathLength * 3 / 4)
        lapCountingArmed = true;

    if (lapCountingArmed && previous > pathLength * 3 / 4 &&
        progressIndex < pathLength / 4)
    {
        lapCountingArmed = false;
        lapBoundaryPending = true;
        completePendingLap();
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

    if (completedLaps == 0 &&
        !completingExtremeAdjacentPair(currentDistance))
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

// A broad, distant GREEN component may be room background. It cannot
// obstruct an EMPTY near seat whose expected ground foot is well below it.
// Upright/local unknown silhouettes still veto CLEAR in the caller.
bool rejectedGreenBehindSeat(const Blob *raw, float bearing, float range)
{
    if (raw==nullptr || !raw->found || raw->color!=ColorType::GREEN ||
        raw->height()<=0 || raw->width()<2*raw->height() ||
        raw->maxY<=OBSTACLE_CAMERA_GROUND_HORIZON_Y) return false;
    const float forward=range*cosf(bearing*PI/180.0f);
    if (!isfinite(forward) || forward<=0.0f) return false;
    const float expectedFoot=OBSTACLE_CAMERA_GROUND_HORIZON_Y+
        OBSTACLE_CAMERA_GROUND_RANGE_SCALE_MM_PX/forward;
    const float rawForward=OBSTACLE_CAMERA_GROUND_RANGE_SCALE_MM_PX/
        (raw->maxY-OBSTACLE_CAMERA_GROUND_HORIZON_Y);
    return expectedFoot-raw->maxY>=OBSTACLE_PARKING_SEAT_FOOT_TOLERANCE_PX &&
        rawForward>=forward+OBSTACLE_DISCOVERY_BEHIND_SEAT_MARGIN_MM;
}

bool rejectedBlobBlocksSeatClear(
    const Blob *rawBlob,
    float seatBearingDeg)
{
    if (rawBlob == nullptr || !rawBlob->found ||
        obstacle_blob_valid_for_acquisition(rawBlob) ||
        rawBlob->maxY < OBSTACLE_MIN_BOTTOM_Y)
    {
        return false;
    }

    // A low rejected colour region may be a badly segmented pillar. It is not
    // trustworthy enough to inject a route, but it must prevent an occupied
    // seat from being certified clear. Thin horizon fragments remain above
    // OBSTACLE_MIN_BOTTOM_Y and therefore do not block empty-field evidence.
    const float edgeBearing0 = cameraBearingForImageX(rawBlob->minX);
    const float edgeBearing1 = cameraBearingForImageX(rawBlob->maxX);
    const float minimumBearing =
        fminf(edgeBearing0, edgeBearing1) -
        OBSTACLE_DISCOVERY_BLOB_BEARING_MARGIN_DEG;
    const float maximumBearing =
        fmaxf(edgeBearing0, edgeBearing1) +
        OBSTACLE_DISCOVERY_BLOB_BEARING_MARGIN_DEG;
    return seatBearingDeg >= minimumBearing &&
           seatBearingDeg <= maximumBearing;
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
            OBSTACLE_DISCOVERY_FOV_FRACTION -
        OBSTACLE_DISCOVERY_CLEAR_FOV_MARGIN_DEG;
    return rangeMm >= OBSTACLE_DISCOVERY_VIEW_MIN_MM &&
           rangeMm <= OBSTACLE_DISCOVERY_VIEW_MAX_MM &&
           fabsf(bearingDeg) <= bearingLimit;
}

// One <=8kB sampled RGB565 window on a stopped CW middle-clear decision.
// Diagnostic only: no extra camera capture, USB save, or motion. Preserve the
// colour evidence even when neither connected blobs nor the local fallback
// recognize a pillar. Coordinates are in the upright image, sampled every2px.
void logParkingStartClearImage(uint8_t seatIndex, const PositionEstimate &pose)
{
    if (parkingStartRoiLogged || !parkingCwShortStart || !parkingEntryObserving ||
        parkingEntryTargetStation != 1 || seatIndex != 2 ||
        camera.getBuffer() == nullptr || camera.getWidth()!=320 || camera.getHeight()!=240)
        return;
    parkingStartRoiLogged=true;
    float bearing=0.0f,range=-1.0f;seatCameraGeometry(seatIndex,pose,bearing,range);
    const float forward=range*cosf(bearing*PI/180.0f);
    if (!isfinite(forward) || forward<=0.0f) return;
    const int center=static_cast<int>(lroundf(OBSTACLE_CAMERA_PRINCIPAL_X_PX-
        OBSTACLE_CAMERA_FOCAL_X_PX*tanf(bearing*PI/180.0f)));
    const int foot=static_cast<int>(lroundf(OBSTACLE_CAMERA_GROUND_HORIZON_Y+
        OBSTACLE_CAMERA_GROUND_RANGE_SCALE_MM_PX/forward));
    const int x0=static_cast<int>(clampFloat(center-32,0,256));
    const int y0=static_cast<int>(clampFloat(foot-80,0,144));
    const uint8_t *buffer=camera.getBuffer();
    static const char hex[]="0123456789abcdef";
    Serial.print("[START_ROI] t=");Serial.print(millis());
    Serial.print(" seat=2 x0/y0=");Serial.print(x0);Serial.print("/");Serial.print(y0);
    Serial.print(" width/height/step=64/96/2 rgb565=msb-upright pose=");
    Serial.print(pose.x_mm,1);Serial.print(",");Serial.print(pose.y_mm,1);Serial.print(",");
    Serial.println(pose.heading_deg,1);
    for(int y=0;y<96;y+=2)
    {
        // 32 sampled pixels =>128 hex chars per row, fixed stack budget.
        char row[129];int out=0;
        for(int x=0;x<64;x+=2)
        {
            const int imageX=x0+x,imageY=y0+y;
            const int sx=Vision::rotates180()?319-imageX:imageX;
            const int sy=Vision::rotates180()?239-imageY:imageY;
            const uint32_t offset=(sy*320+sx)*2;
            const uint8_t hi=buffer[offset+(Vision::rgb565MsbFirst()?0:1)];
            const uint8_t lo=buffer[offset+(Vision::rgb565MsbFirst()?1:0)];
            row[out++]=hex[hi>>4];row[out++]=hex[hi&15];
            row[out++]=hex[lo>>4];row[out++]=hex[lo&15];
        }
        row[out]=0;Serial.print("[START_ROI_ROW] ");Serial.println(row);
    }
    Serial.println("[START_ROI_END]");
}

void updateDiscoveryCoverage(
    const ObstacleObservationResult &observation,
    const PositionEstimate &pose,
    const Blob *rawBlob)
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
            const bool deferParkingTargetClear =
                parkingEntryActive && !parkingEntryObserving &&
                parkingEntryTargetStation >= 0 &&
                station ==
                    static_cast<uint8_t>(parkingEntryTargetStation);
            const bool comfortablyVisible =
                seatComfortablyVisible(seatIndex, pose);
            float seatBearingDeg = 0.0f;
            float seatRangeMm = -1.0f;
            seatCameraGeometry(
                seatIndex,
                pose,
                seatBearingDeg,
                seatRangeMm);
            const bool distantGreenBackground =
                !observation.productionValid &&
                (observation.status == OBSTACLE_OBSERVATION_NO_BLOB ||
                 (observation.status == OBSTACLE_OBSERVATION_REJECTED_BLOB &&
                  observation.color == ColorType::GREEN)) &&
                rejectedGreenBehindSeat(rawBlob, seatBearingDeg, seatRangeMm);
            const bool clearEvidence =
                !deferParkingTargetClear && comfortablyVisible &&
                !greenSeatCandidateThisFrame[seatIndex] &&
                (!rejectedBlobBlocksSeatClear(rawBlob, seatBearingDeg) || distantGreenBackground) &&
                (observationAllowsClearAtGeometry(observation, seatBearingDeg, seatRangeMm) ||
                 distantGreenBackground);
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
            const uint8_t requiredClearFrames =
                parkingEntryObserving &&
                    parkingEntryTargetStation >= 0 &&
                    station == static_cast<uint8_t>(parkingEntryTargetStation)
                ? OBSTACLE_PARKING_ENTRY_CLEAR_FRAMES
                : OBSTACLE_DISCOVERY_CLEAR_FRAMES;
            if (coverage.clearFrames[side] >= requiredClearFrames)
            {
                logParkingStartClearImage(seatIndex,pose);
                coverage.seatObservedClear[side] = true;
            }
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

void logDiscoveryTrace(uint8_t station, const char *reason, bool forced)
{
    const uint32_t now = millis();
    if (station >= OBSTACLE_SEAT_COUNT / COURSE_SEATS_PER_STATION ||
        lastDiscoveryCoverageMs == 0 || discoveryTraceCount >= 40 ||
        (!forced && (discoveryTraceCount >= 32 || now - lastDiscoveryTraceMs < 200)))
        return;
    ++discoveryTraceCount;
    lastDiscoveryTraceMs = now;
    const PositionEstimate &pose = lastDiscoveryCoveragePose;
    const DiscoveryStation &coverage = discoveryStations[station];
    const Blob *raw = getLargestObstacle();
    Serial.print("[DISCOVERY_TRACE] v=1 t="); Serial.print(now);
    Serial.print(" frame_t="); Serial.print(lastDiscoveryCoverageMs);
    Serial.print(" reason="); Serial.print(reason);
    Serial.print(" station="); Serial.print(station);
    Serial.print(" pose="); Serial.print(pose.x_mm, 1);
    Serial.print(","); Serial.print(pose.y_mm, 1);
    Serial.print(","); Serial.print(pose.heading_deg, 2);
    Serial.print(" nudge="); Serial.print(lastDiscoveryTargetNudgeDeg, 2);
    Serial.print(" obs="); Serial.print(static_cast<uint8_t>(lastDiscoveryObservation.status));
    Serial.print(" valid="); Serial.print(lastDiscoveryObservation.productionValid ? 1 : 0);
    Serial.print(" obs_seat="); Serial.print(lastDiscoveryObservation.seatId);
    Serial.print(" obs_range="); Serial.print(lastDiscoveryObservation.rangeMm, 1);
    if (lastDiscoveryObservation.productionValid &&
        isfinite(lastDiscoveryObservation.sightingXmm) &&
        isfinite(lastDiscoveryObservation.sightingYmm))
    {
        int nearest = -1;
        float nearestSquared = INFINITY;
        for (uint8_t seat = 0; seat < OBSTACLE_SEAT_COUNT; ++seat)
        {
            const float errorSquared = distanceSquared(
                lastDiscoveryObservation.sightingXmm,
                lastDiscoveryObservation.sightingYmm,
                seats[seat].x, seats[seat].y);
            if (errorSquared < nearestSquared)
            {
                nearestSquared = errorSquared;
                nearest = seat;
            }
        }
        Serial.print(" obs_geom=");
        Serial.print((lastDiscoveryObservation.left +
                      lastDiscoveryObservation.right) / 2);
        Serial.print(","); Serial.print(lastDiscoveryObservation.bottom);
        Serial.print(","); Serial.print(lastDiscoveryObservation.bearingDeg, 1);
        Serial.print(","); Serial.print(lastDiscoveryObservation.sightingXmm, 0);
        Serial.print(","); Serial.print(lastDiscoveryObservation.sightingYmm, 0);
        Serial.print(","); Serial.print(nearest);
        Serial.print(","); Serial.print(sqrtf(nearestSquared), 0);
    }
    Serial.print(" green_roi_us="); Serial.print(lastGreenSeatProcessingUs);
    Serial.print(" evidence="); Serial.print(coverage.lastClearEvidenceMask);
    for (uint8_t side = 0; side < COURSE_SEATS_PER_STATION; ++side)
    {
        const uint8_t seat = station * COURSE_SEATS_PER_STATION + side;
        float bearing = 0.0f, range = 0.0f;
        seatCameraGeometry(seat, pose, bearing, range);
        Serial.print(" s"); Serial.print(side); Serial.print("=");
        Serial.print(bearing, 2); Serial.print(","); Serial.print(range, 1);
        Serial.print(","); Serial.print(seatComfortablyVisible(seat, pose) ? 1 : 0);
        Serial.print(","); Serial.print(rejectedBlobBlocksSeatClear(raw, bearing) ? 1 : 0);
        Serial.print(","); Serial.print(observationAllowsClearAtGeometry(lastDiscoveryObservation, bearing, range) ? 1 : 0);
        Serial.print(","); Serial.print(coverage.clearFrames[side]);
        Serial.print(","); Serial.print(coverage.seatObservedClear[side] ? 1 : 0);
    }
    if (raw != nullptr && raw->found)
    {
        Serial.print(" raw="); Serial.print(static_cast<uint8_t>(raw->color));
        Serial.print(","); Serial.print(raw->area);
        Serial.print(","); Serial.print(raw->width());
        Serial.print(","); Serial.print(raw->height());
        Serial.print(","); Serial.print(raw->maxY);
        Serial.print(","); Serial.print(obstacle_blob_valid_for_acquisition(raw) ? 1 : 0);
    }
    Serial.println();
}

// The all-seat check intentionally includes even seats previously declared
// CLEAR: a corner peek must not depend on the user's particular empty layout.
bool cornerViewSeatMayBeOccupied(uint8_t seat)
{
    if (seats[seat].confirmed) return true; // A later confirmation overrides CLEAR.
    if (discoveryStations[seat/2].seatObservedClear[seat%2]) return false;
    return !(sectionInferredEmpty(seat/6) & (1U << ((seat%6)/2)));
}

bool cornerViewSweepSafe(const PositionEstimate &start, float signedTravel)
{
    if (!isfinite(signedTravel) || fabsf(signedTravel) > 260.0f)
        return false;
    const float heading = start.heading_deg * PI / 180.0f;
    const unsigned steps = static_cast<unsigned>(ceilf(fabsf(signedTravel) / 5.0f));
    for (unsigned step = 0; step <= steps; ++step)
    {
        const float travel = steps ? signedTravel * step / steps : 0.0f;
        for (uint8_t rear = 0; rear < 2; ++rear)
        {
            const float offset = rear ? OBSTACLE_MAX_WHEEL_HALF_WIDTH_MM : 0.0f;
            const float x = start.x_mm + (travel - offset) * cosf(heading);
            const float y = start.y_mm + (travel - offset) * sinf(heading);
            for (uint8_t seat = 0; seat < OBSTACLE_SEAT_COUNT; ++seat)
            {
                ObstacleClearanceSample sample{};
                if (!calculateClearanceAtPose(seats[seat], x, y,
                        start.heading_deg, sample) ||
                    sample.wallMm <= 40.0f ||
                    (cornerViewSeatMayBeOccupied(seat) && sample.pillarMm <= 40.0f))
                    return false;
            }
        }
    }
    return true;
}

float cornerViewReverseDistance(const PositionEstimate &pose, uint8_t station,
                               float minimumViewRangeMm = OBSTACLE_DISCOVERY_VIEW_MIN_MM)
{
    const uint8_t first=station*COURSE_SEATS_PER_STATION;
    const bool needFirst=!discoveryStations[station].seatObservedClear[0];
    const bool needSecond=!discoveryStations[station].seatObservedClear[1];
    const float h=pose.heading_deg*PI/180.0f;
    for (float distance=needFirst && needSecond ? 170.0f : 60.0f;
         distance<=220.0f; distance+=10.0f)
    {
        PositionEstimate view=pose;
        view.x_mm-=distance*cosf(h); view.y_mm-=distance*sinf(h);
        float bearing0=0,range0=0,bearing1=0,range1=0;
        seatCameraGeometry(first,view,bearing0,range0);
        seatCameraGeometry(first+1,view,bearing1,range1);
        if ((!needFirst || seatComfortablyVisible(first,view)) &&
            (!needSecond || seatComfortablyVisible(first+1,view)) &&
            (!needFirst || range0 >= minimumViewRangeMm) &&
            (!needSecond || range1 >= minimumViewRangeMm) &&
            cornerViewSweepSafe(pose,-distance-20.0f)) return distance;
    }
    return 0.0f;
}

float cornerViewMinimumRange(const Blob *raw)
{
    // B502/503: after a geometrically sufficient 80-90 mm reverse, the real
    // RED was still 123-133 px high, beyond the strict 120 px acquisition cap.
    // Move to a deeper view for a close upright silhouette; never relax colour,
    // shape, seat snap, voting or empty-evidence gates for that silhouette.
    return raw && raw->found &&
        (raw->color==ColorType::RED || raw->color==ColorType::GREEN) &&
        raw->height()>=60 && raw->maxY>=180 &&
        raw->width() <= raw->height()*OBSTACLE_MAX_WIDTH_HEIGHT_RATIO
        ? 300.0f : OBSTACLE_DISCOVERY_VIEW_MIN_MM;
}

int cornerViewStraightSteering(const PositionEstimate &pose, int direction)
{
    const float h=cornerViewOrigin.heading_deg*PI/180.0f;
    const float cross=-(pose.x_mm-cornerViewOrigin.x_mm)*sinf(h)+
                       (pose.y_mm-cornerViewOrigin.y_mm)*cosf(h);
    const float desired=cornerViewOrigin.heading_deg-
        direction*atan2f(cross,80.0f)*180.0f/PI;
    return static_cast<int>(clampFloat(-direction*3.0f*
        wrap180(desired-pose.heading_deg),-8.0f,8.0f));
}

bool cornerViewSteeredCommandSafe(PositionEstimate pose, int steering, int direction)
{
    const float radius=Ackermann::getTurnRadius(static_cast<float>(steering));
    const float curvature=fabsf(radius)>100000.0f ? 0.0f : -1.0f/radius;
    for (int mm=0; mm<=15; ++mm)
    {
        if (!cornerViewSweepSafe(pose,0.0f)) return false;
        const float h=pose.heading_deg*PI/180.0f,next=h+curvature*direction;
        if (fabsf(curvature)>1e-6f)
        {
            pose.x_mm+=(sinf(next)-sinf(h))/curvature;
            pose.y_mm+=(cosf(h)-cosf(next))/curvature;
        }
        else { pose.x_mm+=direction*cosf(h); pose.y_mm+=direction*sinf(h); }
        pose.heading_deg=wrap180(next*180.0f/PI);
    }
    return true;
}

void holdCornerViewSteering(int steering)
{
    // stop() disables servo writes. Re-enable AFTER braking and physically
    // command the requested steering before settling or changing direction.
    stop(true);
    servo_disabled = false;
    set_steering(steering);
    steer(steering);
}

void holdCornerViewCentered()
{
    holdCornerViewSteering(0);
}

void lockCornerView(const char *reason)
{
    holdCornerViewCentered();
    cornerViewPhase = CORNER_VIEW_LOCKED;
    discoveryBlocked = true;
    discoveryBlockedStation = cornerViewStation;
    Serial.print("[CORNER VIEW] Locked: "); Serial.println(reason);
    const PositionEstimate pose = get_position_struct();
    const float h = cornerViewOrigin.heading_deg * PI / 180.0f;
    Serial.print("[CORNER VIEW ABORT] t="); Serial.print(millis());
    Serial.print(" heading_error=");
    Serial.print(wrap180(pose.heading_deg - cornerViewOrigin.heading_deg), 2);
    Serial.print(" cross_track=");
    Serial.print(-(pose.x_mm - cornerViewOrigin.x_mm) * sinf(h) +
        (pose.y_mm - cornerViewOrigin.y_mm) * cosf(h), 1);
    Serial.print(" origin_encoder_delta=");
    Serial.println(get_distance() - cornerViewOriginEncoder, 1);
}

void logCornerViewPose(const char *phase, const PositionEstimate &pose, float encoder)
{
    Serial.print("[CORNER VIEW POSE] t="); Serial.print(millis());
    Serial.print(" phase="); Serial.print(phase);
    Serial.print(" station="); Serial.print(cornerViewStation);
    Serial.print(" pose="); Serial.print(pose.x_mm, 1); Serial.print(",");
    Serial.print(pose.y_mm, 1); Serial.print(","); Serial.print(pose.heading_deg, 2);
    Serial.print(" encoder="); Serial.println(encoder, 1);
}

PositionEstimate cornerViewArcPose(const PositionEstimate &start, int steering, float travel)
{
    PositionEstimate result = start;
    const float curvature = -tanf(steering * PI / 180.0f) / OBSTACLE_WHEELBASE_MM;
    const float angle = start.heading_deg * PI / 180.0f;
    const float end = angle + curvature * travel;
    result.x_mm += (sinf(end) - sinf(angle)) / curvature;
    result.y_mm += (cosf(angle) - cosf(end)) / curvature;
    result.heading_deg = wrap180(end * 180.0f / PI);
    return result;
}

bool cornerViewArcSafe(const PositionEstimate &start, int steering, float travel)
{
    if (abs(steering) != 20 || !isfinite(travel) || fabsf(travel) > 160) return false;
    const unsigned steps = static_cast<unsigned>(ceilf(fabsf(travel) / 3));
    for (unsigned step = 0; step <= steps; ++step)
    {
        const PositionEstimate pose = cornerViewArcPose(start, steering,
            steps ? travel * step / steps : 0);
        const float h = pose.heading_deg * PI / 180.0f;
        for (uint8_t rear = 0; rear < 2; ++rear)
            for (uint8_t seat = 0; seat < OBSTACLE_SEAT_COUNT; ++seat)
            {
                const float offset = rear ? OBSTACLE_MAX_WHEEL_HALF_WIDTH_MM : 0;
                ObstacleClearanceSample sample{};
                if (!calculateClearanceAtPose(seats[seat], pose.x_mm-offset*cosf(h),
                    pose.y_mm-offset*sinf(h), pose.heading_deg, sample) ||
                    sample.wallMm <= 40 ||
                    (cornerViewSeatMayBeOccupied(seat) && sample.pillarMm <= 40)) return false;
            }
    }
    return true;
}

void collectCornerViewFrame(const PositionEstimate &pose, bool newFrame)
{
    if (!newFrame) return;
    lastDiscoveryObservation = obstacle_path_observe(getLargestValidObstacle());
    processGreenSeatCandidates(pose, lastDiscoveryObservation);
    updateDiscoveryCoverage(lastDiscoveryObservation, pose, getLargestObstacle());
    lastDiscoveryCoveragePose = pose;
    lastDiscoveryCoverageMs = millis();
    if (cornerViewStoppedFrames < 255) ++cornerViewStoppedFrames;
    // Preserve decisive stopped-view evidence even if the general discovery
    // trace budget was consumed on the approach. Two records per attempt.
    if (cornerViewObservationTraceCount<2 &&
        (cornerViewObservationTraceCount==0 || millis()-cornerViewPhaseMs>=1400))
    {
        ++cornerViewObservationTraceCount;
        const Blob *raw=getLargestObstacle();
        Serial.print("[CORNER VIEW OBS] station="); Serial.print(cornerViewStation);
        Serial.print(" phase="); Serial.print(static_cast<int>(cornerViewPhase));
        Serial.print(" min_range="); Serial.print(cornerViewRequestedMinRange,0);
        Serial.print(" resolved="); Serial.print(stationResolved(cornerViewStation)?1:0);
        if (raw && raw->found)
        {
            Serial.print(" color/width/height/foot/valid=");
            Serial.print(static_cast<int>(raw->color)); Serial.print("/");
            Serial.print(raw->width()); Serial.print("/"); Serial.print(raw->height());
            Serial.print("/"); Serial.print(raw->maxY); Serial.print("/");
            Serial.print(obstacle_blob_valid_for_acquisition(raw)?1:0);
        }
        Serial.println();
    }
    logDiscoveryTrace(cornerViewStation, "reverse_observe", false);
}

bool beginCornerViewExtra(const PositionEstimate &pose)
{
    if (cornerViewExtraUsed) return false;
    cornerViewExtraUsed = true;
    const DiscoveryStation &coverage = discoveryStations[cornerViewStation];
    // Only one remaining seat: preserve independently obtained clear evidence
    // for the other seat rather than require both in the new view simultaneously.
    if (coverage.seatObservedClear[0] == coverage.seatObservedClear[1]) return false;
    const uint8_t side = coverage.seatObservedClear[0] ? 1 : 0;
    const uint8_t seat = cornerViewStation * COURSE_SEATS_PER_STATION + side;
    float bearing = 0, range = 0;
    seatCameraGeometry(seat, pose, bearing, range);
    if (!seatComfortablyVisible(seat, pose) ||
        !rejectedBlobBlocksSeatClear(getLargestObstacle(), bearing)) return false;
    cornerViewExtraSteering = bearing < 0 ? -20 : 20;
    const PositionEstimate view = cornerViewArcPose(pose, cornerViewExtraSteering, -90);
    if (!seatComfortablyVisible(seat, view) ||
        !cornerViewArcSafe(pose, cornerViewExtraSteering, -110)) return false;
    holdCornerViewSteering(cornerViewExtraSteering);
    cornerViewPhase = CORNER_VIEW_EXTRA_SETTLE;
    cornerViewPhaseMs = millis();
    Serial.print("[CORNER VIEW] Extra parallax scan steering/mm=");
    Serial.print(cornerViewExtraSteering); Serial.println("/90");
    return true;
}

void updateCornerViewExtra(bool newFrame)
{
    const PositionEstimate pose = get_position_struct();
    const float encoder = get_distance();
    const uint32_t age = millis() - cornerViewPhaseMs;
    if (!isfinite(encoder) || !isfinite(pose.x_mm) || !isfinite(pose.y_mm) ||
        !isfinite(pose.heading_deg)) { lockCornerView("extra invalid pose"); return; }
    if (cornerViewPhase != CORNER_VIEW_EXTRA_SETTLE)
    {
        const bool returning = cornerViewPhase == CORNER_VIEW_EXTRA_FORWARD ||
            cornerViewPhase == CORNER_VIEW_EXTRA_BRAKE;
        const PositionEstimate expected = cornerViewArcPose(
            returning ? cornerViewExtraReturnPose : cornerViewExtraOrigin,
            cornerViewExtraSteering,
            encoder - (returning ? cornerViewExtraReturnEncoder : cornerViewExtraStartEncoder));
        if (hypotf(pose.x_mm-expected.x_mm, pose.y_mm-expected.y_mm) > 15 ||
            fabsf(wrap180(pose.heading_deg-expected.heading_deg)) > 5 ||
            fabsf(wrap180(pose.heading_deg-cornerViewOrigin.heading_deg)) > 35)
        { lockCornerView("extra arc tracking limit"); return; }
    }
    switch (cornerViewPhase)
    {
    case CORNER_VIEW_EXTRA_SETTLE:
        holdCornerViewSteering(cornerViewExtraSteering);
        if (age < 300) return;
        if (!cornerViewArcSafe(pose, cornerViewExtraSteering, -110))
        { lockCornerView("extra settled preflight"); return; }
        cornerViewExtraOrigin = pose;
        cornerViewExtraStartEncoder = encoder;
        cornerViewPhase = CORNER_VIEW_EXTRA_REVERSE;
        cornerViewPhaseMs = millis();
        logCornerViewPose("extra_origin", pose, encoder);
        return;
    case CORNER_VIEW_EXTRA_REVERSE:
        if (encoder-cornerViewExtraStartEncoder > 5 ||
            cornerViewExtraStartEncoder-encoder > 110 || age > 4500)
        { lockCornerView("extra reverse travel/time/direction"); return; }
        if (cornerViewExtraStartEncoder-encoder >= 90)
        {
            holdCornerViewSteering(cornerViewExtraSteering);
            cornerViewPhase = CORNER_VIEW_EXTRA_OBSERVE;
            cornerViewPhaseMs = millis();
            cornerViewStoppedFrames = 0;
            discoveryStations[cornerViewStation].clearFrames[0] = 0;
            discoveryStations[cornerViewStation].clearFrames[1] = 0;
            return;
        }
        set_steering(cornerViewExtraSteering);
        set_speed(-OBSTACLE_CORNER_VIEW_REVERSE_SPEED_MM_S); return;
    case CORNER_VIEW_EXTRA_OBSERVE:
        holdCornerViewSteering(cornerViewExtraSteering);
        cornerViewExtraMeasuredMm = cornerViewExtraStartEncoder-encoder;
        if (cornerViewExtraMeasuredMm < 85 || cornerViewExtraMeasuredMm > 110)
        { lockCornerView("extra reverse braking travel"); return; }
        if (age < 200) return;
        collectCornerViewFrame(pose, newFrame);
        if (age < 600 || (age < 1800 &&
            (!stationResolved(cornerViewStation) || cornerViewStoppedFrames < 2))) return;
        // Always retrace the arc, even when the new viewpoint remains unresolved.
        if (!cornerViewArcSafe(pose, cornerViewExtraSteering, cornerViewExtraMeasuredMm+20))
        { lockCornerView("extra return preflight"); return; }
        cornerViewPhase = CORNER_VIEW_EXTRA_RETURN_SETTLE;
        cornerViewPhaseMs = millis();
        logCornerViewPose("extra_scan", pose, encoder);
        return;
    case CORNER_VIEW_EXTRA_RETURN_SETTLE:
        holdCornerViewSteering(cornerViewExtraSteering);
        if (age < 300) return;
        cornerViewExtraMeasuredMm = cornerViewExtraStartEncoder-encoder;
        if (cornerViewExtraMeasuredMm < 85 || cornerViewExtraMeasuredMm > 110 ||
            !cornerViewArcSafe(pose, cornerViewExtraSteering, cornerViewExtraMeasuredMm+20))
        { lockCornerView("extra settled return preflight"); return; }
        cornerViewExtraReturnPose = pose;
        cornerViewExtraReturnEncoder = encoder;
        cornerViewPhase = CORNER_VIEW_EXTRA_FORWARD;
        cornerViewPhaseMs = millis(); return;
    case CORNER_VIEW_EXTRA_FORWARD:
        if (encoder-cornerViewExtraReturnEncoder < -5 ||
            encoder-cornerViewExtraReturnEncoder > cornerViewExtraMeasuredMm+20 || age > 4500)
        { lockCornerView("extra return travel/time/direction"); return; }
        if (encoder-cornerViewExtraReturnEncoder >= cornerViewExtraMeasuredMm)
        {
            holdCornerViewCentered();
            cornerViewPhase = CORNER_VIEW_EXTRA_BRAKE;
            cornerViewPhaseMs = millis(); return;
        }
        set_steering(cornerViewExtraSteering); set_speed(60); return;
    case CORNER_VIEW_EXTRA_BRAKE:
        holdCornerViewCentered();
        if (encoder-cornerViewExtraReturnEncoder > cornerViewExtraMeasuredMm+20)
        { lockCornerView("extra return braking travel"); return; }
        if (age < 200) return;
        logCornerViewPose("extra_returned", pose, encoder);
        if (hypotf(pose.x_mm-cornerViewExtraOrigin.x_mm, pose.y_mm-cornerViewExtraOrigin.y_mm) > 20 ||
            fabsf(wrap180(pose.heading_deg-cornerViewExtraOrigin.heading_deg)) > 3)
        { lockCornerView("extra return pose limit"); return; }
        cornerViewPhase = CORNER_VIEW_OBSERVE;
        cornerViewPhaseMs = millis()-200;
        cornerViewStoppedFrames = 0;
        return;
    default: lockCornerView("invalid extra phase"); return;
    }
}

void updateCornerView(bool newCameraFrame)
{
    if (cornerViewPhase >= CORNER_VIEW_EXTRA_SETTLE && cornerViewPhase <= CORNER_VIEW_EXTRA_BRAKE)
    {
        updateCornerViewExtra(newCameraFrame);
        return;
    }
    set_steering(0);
    if (cornerViewPhase == CORNER_VIEW_LOCKED)
    {
        holdCornerViewCentered();
        return;
    }
    const PositionEstimate pose = get_position_struct();
    const float encoder = get_distance();
    const uint32_t age = millis() - cornerViewPhaseMs;
    if (!isfinite(encoder) || !isfinite(pose.x_mm) || !isfinite(pose.y_mm) ||
        !isfinite(pose.heading_deg))
    {
        lockCornerView("invalid pose/encoder"); return;
    }
    if (cornerViewPhase != CORNER_VIEW_SETTLE)
    {
        const float h = cornerViewOrigin.heading_deg * PI / 180.0f;
        const float cross = fabsf(-(pose.x_mm - cornerViewOrigin.x_mm) * sinf(h) +
            (pose.y_mm - cornerViewOrigin.y_mm) * cosf(h));
        if (fabsf(wrap180(pose.heading_deg - cornerViewOrigin.heading_deg)) > 3.0f ||
            cross > 15.0f)
        {
            lockCornerView("heading/cross-track limit"); return;
        }
    }
    switch (cornerViewPhase)
    {
    case CORNER_VIEW_SETTLE:
        holdCornerViewCentered();
        cornerViewRequestedMinRange=fmaxf(cornerViewRequestedMinRange,
            cornerViewMinimumRange(getLargestObstacle()));
        if (age < 300) return;
        cornerViewOrigin = pose;
        cornerViewOriginEncoder = encoder;
        cornerViewDistanceMm = cornerViewReverseDistance(pose,cornerViewStation,
            cornerViewRequestedMinRange);
        if (cornerViewDistanceMm == 0)
        {
            lockCornerView("no safe view preflight"); return;
        }
        cornerViewPhase = CORNER_VIEW_REVERSE;
        cornerViewPhaseMs = millis();
        Serial.print("[CORNER VIEW] Reverse station/distance=");
        Serial.print(cornerViewStation); Serial.print("/");
        Serial.println(cornerViewDistanceMm, 0);
        logCornerViewPose("origin", pose, encoder);
        return;
    case CORNER_VIEW_REVERSE:
        if (encoder - cornerViewOriginEncoder > 5 ||
            cornerViewOriginEncoder - encoder > cornerViewDistanceMm + 20 || age > 6000)
        {
            lockCornerView("reverse travel/direction/time limit"); return;
        }
        if (cornerViewOriginEncoder - encoder >= cornerViewDistanceMm)
        {
            holdCornerViewCentered();
            cornerViewPhase = CORNER_VIEW_OBSERVE;
            cornerViewPhaseMs = millis();
            cornerViewStoppedFrames = 0;
            discoveryStations[cornerViewStation].clearFrames[0] = 0;
            discoveryStations[cornerViewStation].clearFrames[1] = 0;
            Serial.println("[CORNER VIEW] Brake then observe");
            return;
        }
        {
            const int steering=cornerViewStraightSteering(pose,-1);
            if (!cornerViewSteeredCommandSafe(pose,steering,-1))
            { lockCornerView("reverse command clearance"); return; }
            set_steering(steering);
        }
        set_speed(-OBSTACLE_CORNER_VIEW_REVERSE_SPEED_MM_S);
        return;
    case CORNER_VIEW_OBSERVE:
        holdCornerViewCentered();
        cornerViewMeasuredReverseMm = cornerViewOriginEncoder - encoder;
        if (cornerViewMeasuredReverseMm < cornerViewDistanceMm - 5 ||
            cornerViewMeasuredReverseMm > cornerViewDistanceMm + 20)
        {
            lockCornerView("reverse braking travel limit"); return;
        }
        if (age < 200) return;
        collectCornerViewFrame(pose, newCameraFrame);
        // A near pillar can drop out during the initial settling frames. If
        // the first stopped image reveals it is still too close, extend the
        // same straight scan once instead of adding a parallax manoeuvre.
        if (!stationResolved(cornerViewStation) && newCameraFrame &&
            cornerViewRequestedMinRange < 300.0f &&
            cornerViewMinimumRange(getLargestObstacle()) >= 300.0f &&
            !obstacle_blob_valid_for_acquisition(getLargestObstacle()))
        {
            cornerViewRequestedMinRange=300.0f;
            const float deeper=cornerViewReverseDistance(cornerViewOrigin,
                cornerViewStation,cornerViewRequestedMinRange);
            if (deeper>cornerViewDistanceMm &&
                cornerViewSweepSafe(pose,-(deeper-cornerViewMeasuredReverseMm+20.0f)))
            {
                cornerViewDistanceMm=deeper;
                cornerViewPhase=CORNER_VIEW_REVERSE;
                cornerViewPhaseMs=millis();
                Serial.print("[CORNER VIEW] Near silhouette; extend straight reverse total_mm=");
                Serial.println(deeper,0);
                return;
            }
        }
        if (age < 600 || (age < 1800 &&
            (!stationResolved(cornerViewStation) || cornerViewStoppedFrames < 2))) return;
        cornerViewCanResume = stationResolved(cornerViewStation) && cornerViewStoppedFrames >= 2;
        if (!cornerViewCanResume && beginCornerViewExtra(pose)) return;
        if (!cornerViewSweepSafe(pose, cornerViewMeasuredReverseMm + 20))
        {
            lockCornerView("return preflight"); return;
        }
        cornerViewReturnEncoder = encoder;
        logCornerViewPose("scan_return_start", pose, encoder);
        cornerViewPhase = CORNER_VIEW_RETURN;
        cornerViewPhaseMs = millis();
        Serial.print("[CORNER VIEW] Return measured_mm/resolved=");
        Serial.print(cornerViewMeasuredReverseMm, 1); Serial.print("/");
        Serial.println(cornerViewCanResume ? 1 : 0);
        return;
    case CORNER_VIEW_RETURN:
        if (encoder - cornerViewReturnEncoder < -5 ||
            encoder - cornerViewReturnEncoder > cornerViewMeasuredReverseMm + 20 || age > 6000)
        {
            lockCornerView("return travel/direction/time limit"); return;
        }
        if (encoder - cornerViewReturnEncoder >= cornerViewMeasuredReverseMm)
        {
            holdCornerViewCentered();
            cornerViewPhase = CORNER_VIEW_RETURN_BRAKE;
            cornerViewPhaseMs = millis();
            return;
        }
        {
            const int steering=cornerViewStraightSteering(pose,+1);
            if (!cornerViewSteeredCommandSafe(pose,steering,+1))
            { lockCornerView("return command clearance"); return; }
            set_steering(steering);
        }
        set_speed(60);
        return;
    case CORNER_VIEW_RETURN_BRAKE:
        holdCornerViewCentered();
        if (encoder - cornerViewReturnEncoder > cornerViewMeasuredReverseMm + 20)
        {
            lockCornerView("return braking travel limit"); return;
        }
        if (age < 200) return;
        logCornerViewPose("returned", pose, encoder);
        if (hypotf(pose.x_mm - cornerViewOrigin.x_mm,
                   pose.y_mm - cornerViewOrigin.y_mm) > 20)
        {
            lockCornerView("return pose limit"); return;
        }
        if (!cornerViewCanResume)
        {
            lockCornerView("returned but station unresolved"); return;
        }
        cornerViewPhase = CORNER_VIEW_IDLE;
        discoveryHolding = false;
        discoveryHoldStation = -1;
        discoveryHoldStartMs = 0;
        discoveryBlocked = false;
        discoveryBlockedStation = -1;
        lastDiscoveryTargetNudgeDeg = 0;
        lastDiscoveryNudgeUpdateMs = millis();
        Serial.println("[CORNER VIEW] Returned and resolved; resume route");
        return;
    default: return;
    }
}

bool parkingEntryPrerequisitesResolved()
{
    if (parkingEntryTargetStation < 0 ||
        !stationResolved(static_cast<uint8_t>(parkingEntryTargetStation)))
        return false;
    // CCW has no upcoming preceding station: both are behind the complete
    // body and cannot be mandatory initial observations.
    return parkingCcwShortStart || parkingEntryTargetStation == 0 ||
        stationResolved(static_cast<uint8_t>(parkingEntryTargetStation - 1));
}

void armParkingEntryConnectorFromPose(const PositionEstimate &pose)
{
    const bool primaryResolved =
        parkingEntryTargetStation >= 0 &&
        stationResolved(static_cast<uint8_t>(parkingEntryTargetStation));
    const bool scoutResolved =
        parkingCcwShortStart || parkingEntryTargetStation <= 0 ||
        stationResolved(static_cast<uint8_t>(parkingEntryTargetStation - 1));
    if (!parkingEntryPrerequisitesResolved())
    {
        parkingEntryTestHold = true;
        Serial.print(
            "[PARK ENTRY CONNECTOR] Rejected unresolved prerequisite primary/scout=");
        Serial.print(primaryResolved ? "yes" : "no");
        Serial.print("/");
        Serial.println(scoutResolved ? "yes" : "no");
        if (!parkingEntryUsbWritten)
        {
            robot_logger.write_to_usb();
            parkingEntryUsbWritten = true;
        }
        return;
    }

    const PathPoint *route = optimizedBuilt ? optimizedPath : livePath;
    uint16_t mergeIndex = 0;
    if (!buildParkingEntryConnector(pose, route, mergeIndex))
    {
        parkingEntryTestHold = true;
        Serial.println(
            "[PARK ENTRY CONNECTOR] Rejected preflight - drive motor locked off");
        if (!parkingEntryUsbWritten)
        {
            robot_logger.write_to_usb();
            parkingEntryUsbWritten = true;
        }
        return;
    }

    parkingEntryConnectorMergeIndex = mergeIndex;
    parkingEntryConnectorProgress = 0;
    parkingEntryConnectorActive = true;
    parkingEntryConnectorReplanPending = false;
    parkingEntryConnectorChangedSeat = -1;
    parkingEntryConnectorStartEncoderDistance = get_distance();
    progressIndex = mergeIndex;
    servo_disabled = false;
    Serial.print("[PARK ENTRY CONNECTOR] Armed merge_index=");
    Serial.print(mergeIndex);
    Serial.print(" points=");
    Serial.println(parkingEntryConnectorLength);
}

void restoreParkingEntryPrimaryScanTarget()
{
    discoveryScanStation = parkingEntryTargetStation;
    discoveryScanSide = -1;
    if (parkingEntryTargetStation < 0)
        return;

    const uint8_t firstSeat = static_cast<uint8_t>(
        parkingEntryTargetStation * COURSE_SEATS_PER_STATION);
    discoveryScanSide = seats[firstSeat].y > seats[firstSeat + 1].y
        ? 0
        : 1;
}

void startParkingEntryPrimaryRetry()
{
    if (parkingEntryTargetStation < 0)
    {
        parkingEntryTestHold = true;
        Serial.println(
            "[PARK ENTRY] Primary retry rejected without target station");
        return;
    }

    restoreParkingEntryPrimaryScanTarget();
    parkingEntryActive = false;
    parkingEntryObserving = true;
    parkingEntryObserveStartMs = millis();
    servo_disabled = false;
    set_steering(0);
    steer(0);
    Serial.print(
        "[PARK ENTRY] Primary stationary retry armed station=");
    Serial.println(parkingEntryTargetStation);
}

// CW uses the initial ToF seed; CCW retains the second-edge reference. Check the
// whole possible scan/scout before moving, with 5 mm footprint reserve, +/-1
// degree heading and 8 mm braking allowance. This is an explicit model budget,
// not a claim that the physical localization error has already been measured.
bool preflightCwStartArc(const PositionEstimate &start, float travelMm)
{
    if (!parkingShortStart())
        return true;
    constexpr float stepMm = 2.0f;
    const float curvature = -routeTurnSign / OBSTACLE_PARKING_ENTRY_SCAN_RADIUS_MM;
    for (int headingError = -1; headingError <= 1; ++headingError)
    {
        float x = start.x_mm, y = start.y_mm;
        float heading = (start.heading_deg + headingError) * PI / 180.0f;
        for (float travel = 0; travel <= travelMm + 8.0f; travel += stepMm)
        {
            if (!parkingStartFootprintSafe(
                    x, y, heading * 180.0f / PI, routeTurnSign, 5.0f))
                return false;
            for (uint8_t seat = routeTurnSign > 0 ? 1 : 0; seat < 6; seat += 2)
            {
                ObstacleClearanceSample clearance{};
                if (!calculateClearanceAtPose(seats[seat], x, y,
                        heading * 180.0f / PI, clearance) ||
                    clearance.wallMm <= 10 || clearance.pillarMm <= 10)
                    return false;
            }
            const float nextHeading = heading - curvature * stepMm;
            x += (sinf(nextHeading) - sinf(heading)) / curvature;
            y += (cosf(heading) - cosf(nextHeading)) / curvature;
            heading = nextHeading;
        }
    }
    return true;
}

bool preflightParkingEntryScout(
    const PositionEstimate &start,
    uint8_t guardSeat)
{
    if (!preflightCwStartArc(start, parkingEntryScoutArcMm()))
    {
        Serial.println("[CW START] Scout footprint/braking preflight rejected");
        return false;
    }
    float x = start.x_mm;
    float y = start.y_mm;
    float heading = start.heading_deg * PI / 180.0f;
    const float curvature =
        -routeTurnSign / OBSTACLE_PARKING_ENTRY_SCAN_RADIUS_MM;
    float remaining = parkingEntryScoutArcMm();
    float minimumWall = 1.0e9f;
    float minimumPillar = 1.0e9f;
    while (remaining > 0.1f)
    {
        const float step = fminf(10.0f, remaining);
        const float signedStep = -step;
        const float nextHeading = heading + curvature * signedStep;
        x += (sinf(nextHeading) - sinf(heading)) / curvature;
        y += (-cosf(nextHeading) + cosf(heading)) / curvature;
        heading = nextHeading;
        remaining -= step;

        ObstacleClearanceSample clearance{};
        if (!calculateClearanceAtPose(
                seats[guardSeat],
                x,
                y,
                heading * 180.0f / PI,
                clearance) ||
            clearance.wallMm <= 0.0f || clearance.pillarMm <= 0.0f)
        {
            Serial.print(
                "[PARK ENTRY SCOUT] Preflight FAIL seat/wall/pillar_mm=");
            Serial.print(guardSeat);
            Serial.print("/");
            Serial.print(clearance.wallMm, 1);
            Serial.print("/");
            Serial.println(clearance.pillarMm, 1);
            return false;
        }
        minimumWall = fminf(minimumWall, clearance.wallMm);
        minimumPillar = fminf(minimumPillar, clearance.pillarMm);
    }

    Serial.print("[PARK ENTRY SCOUT] Preflight PASS station/seat=");
    Serial.print(parkingEntryScoutStation);
    Serial.print("/");
    Serial.print(guardSeat);
    Serial.print(" arc/wall/pillar_mm=");
    Serial.print(parkingEntryScoutArcMm(), 1);
    Serial.print("/");
    Serial.print(minimumWall, 1);
    Serial.print("/");
    Serial.println(minimumPillar, 1);
    return true;
}

void startParkingEntryScout(const PositionEstimate &pose)
{
    if (parkingCcwShortStart)
    {
        // These two places are already behind the body. Neither an absent
        // middle observation nor a recorded behind sign blocks the front join.
        Serial.println("[CCW START] Front observation complete; both behind places deferred to normal approach");
        if (stationResolved(static_cast<uint8_t>(parkingEntryTargetStation)))
            armParkingEntryConnectorFromPose(pose);
        else if (parkingEntryPrimaryRetryUsed)
            startParkingEntryPrimaryRetry();
        else
        {
            parkingEntryTestHold = true;
            Serial.println("[CCW START] Front place unresolved - held for valid observation");
        }
        return;
    }
    if (parkingEntryTargetStation <= 0)
    {
        armParkingEntryConnectorFromPose(pose);
        return;
    }

    parkingEntryScoutStation = parkingEntryTargetStation - 1;
    if (stationResolved(static_cast<uint8_t>(parkingEntryScoutStation)))
    {
        Serial.print("[PARK ENTRY SCOUT] Station already resolved station=");
        Serial.println(parkingEntryScoutStation);
        if (stationResolved(static_cast<uint8_t>(parkingEntryTargetStation)))
            armParkingEntryConnectorFromPose(pose);
        else if (parkingEntryPrimaryRetryUsed)
            startParkingEntryPrimaryRetry();
        else
        {
            parkingEntryTestHold = true;
            Serial.println(
                "[PARK ENTRY] Primary unresolved without retry - drive motor locked off");
        }
        return;
    }

    const uint8_t firstSeat = static_cast<uint8_t>(
        parkingEntryScoutStation * COURSE_SEATS_PER_STATION);
    const uint8_t innerSeat = seats[firstSeat].y > seats[firstSeat + 1].y
        ? firstSeat
        : firstSeat + 1;
    if (!preflightParkingEntryScout(pose, innerSeat))
    {
        parkingEntryTestHold = true;
        Serial.println(
            "[PARK ENTRY SCOUT] Rejected preflight - drive motor locked off");
        return;
    }

    parkingEntryScouting = true;
    parkingEntryScoutPhase = PARKING_ENTRY_SCOUT_STEER_SETTLE;
    parkingEntryScoutStartEncoderDistance = get_distance();
    parkingEntryScoutPhaseStartMs = millis();
    parkingEntryScoutReturnPose = pose;
    discoveryScanStation = parkingEntryScoutStation;
    discoveryScanSide = static_cast<int8_t>(innerSeat - firstSeat);
    servo_disabled = false;
    const int steering = routeTurnSign * OBSTACLE_PARKING_EXIT_STEERING;
    set_steering(steering);
    steer(steering);
    Serial.print("[PARK ENTRY SCOUT] Armed station/seat steering=");
    Serial.print(parkingEntryScoutStation);
    Serial.print("/");
    Serial.print(innerSeat);
    Serial.print(" ");
    Serial.println(steering);
}

void updateParkingEntryScout(bool newCameraFrame)
{
    const PositionEstimate pose = get_position_struct();
    if (parkingShortStart() && !parkingStartFootprintSafe(
            pose.x_mm, pose.y_mm, pose.heading_deg, routeTurnSign))
    {
        stop(false);
        parkingEntryScouting = false;
        parkingEntryTestHold = true;
        Serial.println("[CW START] Scout measured footprint rejected - held");
        return;
    }
    if (newCameraFrame)
    {
        const ObstacleObservationResult observation =
            obstacle_path_observe(getLargestValidObstacle());
        lastDiscoveryObservation = observation;
        processGreenSeatCandidates(pose, observation);
        updateDiscoveryCoverage(lastDiscoveryObservation, pose,
                                getLargestObstacle());
    }

    const bool resolved = parkingEntryScoutStation >= 0 &&
        stationResolved(static_cast<uint8_t>(parkingEntryScoutStation));
    const int steering = routeTurnSign * OBSTACLE_PARKING_EXIT_STEERING;
    const float phaseTravel = fabsf(
        get_distance() - parkingEntryScoutStartEncoderDistance);
    switch (parkingEntryScoutPhase)
    {
    case PARKING_ENTRY_SCOUT_STEER_SETTLE:
        stop(true);
        servo_disabled = false;
        set_steering(steering);
        steer(steering);
        if (millis() - parkingEntryScoutPhaseStartMs <
            OBSTACLE_PARKING_EXIT_STEER_SETTLE_MS)
            return;
        parkingEntryScoutStartEncoderDistance = get_distance();
        parkingEntryScoutPhase = PARKING_ENTRY_SCOUT_REVERSE;
        Serial.println("[PARK ENTRY SCOUT] Steering settled; reverse scan");
        return;

    case PARKING_ENTRY_SCOUT_REVERSE:
        servo_disabled = false;
        set_steering(steering);
        set_speed(-static_cast<int>(OBSTACLE_PARKING_ENTRY_SCOUT_SPEED_MM_S));
        if (phaseTravel >= parkingEntryScoutArcMm())
        {
            stop(true);
            parkingEntryScoutPhase = PARKING_ENTRY_SCOUT_OBSERVE;
            parkingEntryScoutPhaseStartMs = millis();
            Serial.print("[PARK ENTRY SCOUT] Scan pose reached travel_mm=");
            Serial.println(phaseTravel, 1);
        }
        return;

    case PARKING_ENTRY_SCOUT_OBSERVE:
        stop(true);
        if (millis() - parkingEntryScoutPhaseStartMs <
            OBSTACLE_PARKING_ENTRY_SCOUT_MIN_OBSERVE_MS)
            return;
        if (!resolved &&
            millis() - parkingEntryScoutPhaseStartMs <
                OBSTACLE_PARKING_ENTRY_SCOUT_OBSERVE_MS)
            return;
        if (!resolved)
        {
            parkingEntryScouting = false;
            parkingEntryTestHold = true;
            Serial.println(
                "[PARK ENTRY SCOUT] Discovery unresolved - drive motor locked off");
            if (!parkingEntryUsbWritten)
            {
                robot_logger.write_to_usb();
                parkingEntryUsbWritten = true;
            }
            return;
        }
        Serial.print("[PARK ENTRY SCOUT] Resolved station/result=");
        Serial.print(parkingEntryScoutStation);
        {
            const uint8_t firstSeat = static_cast<uint8_t>(
                parkingEntryScoutStation * COURSE_SEATS_PER_STATION);
            if (seats[firstSeat].confirmed)
                Serial.println(seats[firstSeat].red ? "RED" : "GREEN");
            else if (seats[firstSeat + 1].confirmed)
                Serial.println(seats[firstSeat + 1].red ? "RED" : "GREEN");
            else
                Serial.println("CLEAR");
        }
        parkingEntryScoutPhase = PARKING_ENTRY_SCOUT_RETURN_BRAKE;
        parkingEntryScoutPhaseStartMs = millis();
        return;

    case PARKING_ENTRY_SCOUT_RETURN_BRAKE:
        stop(true);
        if (millis() - parkingEntryScoutPhaseStartMs <
            OBSTACLE_PARKING_ENTRY_SCOUT_BRAKE_MS)
            return;
        parkingEntryScoutReturnTravelMm = parkingCwShortStart
            ? phaseTravel : parkingEntryScoutArcMm();
        if (parkingCwShortStart && parkingEntryScoutReturnTravelMm >
                parkingEntryScoutArcMm() + 8.0f)
        {
            parkingEntryScouting = false;
            parkingEntryTestHold = true;
            Serial.println("[CW START] Scout braking allowance exceeded - held");
            return;
        }
        Serial.print("[PARK ENTRY SCOUT] Measured return travel_mm=");
        Serial.println(parkingEntryScoutReturnTravelMm, 1);
        parkingEntryScoutStartEncoderDistance = get_distance();
        parkingEntryScoutPhase = PARKING_ENTRY_SCOUT_FORWARD_RETURN;
        return;

    case PARKING_ENTRY_SCOUT_FORWARD_RETURN:
        servo_disabled = false;
        set_steering(steering);
        set_speed(static_cast<int>(OBSTACLE_PARKING_ENTRY_SCOUT_SPEED_MM_S));
        if (phaseTravel >= parkingEntryScoutReturnTravelMm)
        {
            stop(true);
            parkingEntryScoutPhase = PARKING_ENTRY_SCOUT_COMPLETE_BRAKE;
            parkingEntryScoutPhaseStartMs = millis();
            Serial.print("[PARK ENTRY SCOUT] Return complete travel_mm=");
            Serial.println(phaseTravel, 1);
        }
        return;

    case PARKING_ENTRY_SCOUT_COMPLETE_BRAKE:
        stop(true);
        if (millis() - parkingEntryScoutPhaseStartMs <
            OBSTACLE_PARKING_ENTRY_SCOUT_BRAKE_MS)
            return;
        parkingEntryScouting = false;
        restoreParkingEntryPrimaryScanTarget();
        {
            const PositionEstimate returnedPose = get_position_struct();
            Serial.print(
                "[PARK ENTRY SCOUT] Returned pose_error/heading_deg=");
            Serial.print(hypotf(
                returnedPose.x_mm - parkingEntryScoutReturnPose.x_mm,
                returnedPose.y_mm - parkingEntryScoutReturnPose.y_mm), 1);
            Serial.print("/");
            Serial.println(fabsf(wrap180(
                returnedPose.heading_deg -
                    parkingEntryScoutReturnPose.heading_deg)), 1);
            if (parkingEntryTargetStation >= 0 &&
                stationResolved(
                    static_cast<uint8_t>(parkingEntryTargetStation)))
            {
                armParkingEntryConnectorFromPose(returnedPose);
            }
            else if (parkingEntryPrimaryRetryUsed)
            {
                Serial.println(
                    "[PARK ENTRY] Primary still unresolved after scout return");
                startParkingEntryPrimaryRetry();
            }
            else
            {
                parkingEntryTestHold = true;
                Serial.println(
                    "[PARK ENTRY] Primary unresolved without retry - drive motor locked off");
            }
        }
        return;
    }
}

void updateParkingEntryDiscovery(bool newCameraFrame)
{
    if (parkingEntryTestHold)
    {
        stop(false);
        set_steering(0);
        return;
    }

    const PositionEstimate pose = get_position_struct();
    if (parkingShortStart() && !parkingStartFootprintSafe(
            pose.x_mm, pose.y_mm, pose.heading_deg, routeTurnSign))
    {
        stop(false);
        parkingEntryActive = false;
        parkingEntryTestHold = true;
        Serial.println("[PARK START] Scan measured footprint rejected - held");
        return;
    }
    if (newCameraFrame)
    {
        const Blob *rawBlob = getLargestObstacle();
        const ObstacleObservationResult observation =
            obstacle_path_observe(getLargestValidObstacle());
        lastDiscoveryObservation = observation;
        processGreenSeatCandidates(pose, observation);
        updateDiscoveryCoverage(lastDiscoveryObservation, pose, rawBlob);
    }

    if (parkingEntryObserving)
    {
        stop(false);
        set_steering(0);
        const bool resolved =
            !parkingEntryPathFailed &&
            parkingEntryTargetStation >= 0 &&
            stationResolved(static_cast<uint8_t>(parkingEntryTargetStation));
        const bool timedOut =
            millis() - parkingEntryObserveStartMs >=
            OBSTACLE_PARKING_ENTRY_OBSERVE_MS;
        if (!resolved && !timedOut)
            return;

        const uint8_t firstSeat = static_cast<uint8_t>(
            parkingEntryTargetStation * COURSE_SEATS_PER_STATION);
        const CandidateSeat &first = seats[firstSeat];
        const CandidateSeat &second = seats[firstSeat + 1];
        Serial.print("[PARK ENTRY RESULT] resolved=");
        Serial.print(resolved ? "yes" : "no_timeout");
        Serial.print(" station=");
        Serial.print(parkingEntryTargetStation);
        Serial.print(" result=");
        if (first.confirmed)
        {
            Serial.print(first.red ? "RED" : "GREEN");
            Serial.print(" seat=");
            Serial.print(firstSeat);
        }
        else if (second.confirmed)
        {
            Serial.print(second.red ? "RED" : "GREEN");
            Serial.print(" seat=");
            Serial.print(firstSeat + 1);
        }
        else
        {
            Serial.print(resolved ? "CLEAR" : "UNKNOWN");
        }
        Serial.print(" pose_x_y_heading=");
        Serial.print(pose.x_mm, 1);
        Serial.print("/");
        Serial.print(pose.y_mm, 1);
        Serial.print("/");
        Serial.print(pose.heading_deg, 1);
        float targetBearing = 0.0f;
        float targetRange = -1.0f;
        const uint8_t innerSeat =
            first.y > second.y ? firstSeat : firstSeat + 1;
        seatCameraGeometry(innerSeat, pose, targetBearing, targetRange);
        Serial.print(" target_inner_seat=");
        Serial.print(innerSeat);
        Serial.print(" bearing_range_deg_mm=");
        Serial.print(targetBearing, 1);
        Serial.print("/");
        Serial.println(targetRange, 1);
        parkingEntryActive = false;
        parkingEntryObserving = false;
        if (!resolved)
        {
            if (!parkingEntryPrimaryRetryUsed &&
                parkingEntryTargetStation > 0)
            {
                parkingEntryPrimaryRetryUsed = true;
                Serial.println(
                    "[PARK ENTRY] Primary unresolved; scouting preceding station before one retry");
                startParkingEntryScout(pose);
            }
            else
            {
                parkingEntryTestHold = true;
                Serial.println(
                    "[PARK ENTRY] Discovery unresolved after retry - drive motor locked off");
                if (!parkingEntryUsbWritten)
                {
                    robot_logger.write_to_usb();
                    parkingEntryUsbWritten = true;
                }
            }
        }
        else if (OBSTACLE_PARKING_ENTRY_DISCOVERY_TEST_ONLY)
        {
            parkingEntryTestHold = true;
            Serial.println(
                "[PARK ENTRY] Test complete - drive motor locked off");
            if (!parkingEntryUsbWritten)
            {
                robot_logger.write_to_usb();
                parkingEntryUsbWritten = true;
            }
        }
        else
        {
            startParkingEntryScout(pose);
        }
        return;
    }

    const float entryTravel = fabsf(
        get_distance() - parkingEntryStartEncoderDistance);
    while (parkingEntryProgress + 1 < parkingEntryLength &&
           parkingEntryPath[parkingEntryProgress + 1].distanceMm <=
               entryTravel)
    {
        ++parkingEntryProgress;
    }

    const PathPoint &finish = parkingEntryPath[parkingEntryLength - 1];
    const float plannedTravel = finish.distanceMm;
    const float arcStartTravel = fmaxf(
        0.0f,
        plannedTravel - parkingEntryScanArcMm());

    if (parkingEntryDrivePhase == PARKING_ENTRY_STRAIGHT_STEER_SETTLE)
    {
        stop(false);
        servo_disabled = false;
        const int preloadSteering = static_cast<int>(roundf(
            -routeTurnSign *
            OBSTACLE_PARKING_ENTRY_STRAIGHT_FEEDFORWARD_DEG));
        set_steering(preloadSteering);
        steer(preloadSteering);
        if (
            millis() - parkingEntrySteerSettleStartMs <
            OBSTACLE_PARKING_ENTRY_STRAIGHT_STEER_SETTLE_MS)
        {
            return;
        }
        parkingEntryStraightControlStartDistance = get_distance();
        parkingEntryDrivePhase = PARKING_ENTRY_REVERSE_STRAIGHT;
        Serial.print(
            "[PARK ENTRY] Straight steering settle complete preload=");
        Serial.println(preloadSteering);
        return;
    }

    if (parkingEntryDrivePhase == PARKING_ENTRY_REVERSE_STRAIGHT &&
        entryTravel >= arcStartTravel)
    {
        stop(false);
        servo_disabled = false;
        const int arcSteering =
            routeTurnSign * OBSTACLE_PARKING_EXIT_STEERING;
        set_steering(arcSteering);
        steer(arcSteering);
        parkingEntrySteerSettleStartMs = millis();
        parkingEntryDrivePhase = PARKING_ENTRY_ARC_STEER_SETTLE;
        Serial.print("[PARK ENTRY] Arc steering settle travel/planned_mm=");
        Serial.print(entryTravel, 1);
        Serial.print("/");
        Serial.print(plannedTravel, 1);
        Serial.print(" steering=");
        Serial.print(arcSteering);
        Serial.print(" straight_max_heading_error_deg=");
        Serial.println(parkingEntryStraightMaxHeadingErrorDeg, 1);
        return;
    }

    if (parkingEntryDrivePhase == PARKING_ENTRY_ARC_STEER_SETTLE)
    {
        stop(false);
        servo_disabled = false;
        const int arcSteering =
            routeTurnSign * OBSTACLE_PARKING_EXIT_STEERING;
        set_steering(arcSteering);
        steer(arcSteering);
        if (millis() - parkingEntrySteerSettleStartMs <
            OBSTACLE_PARKING_EXIT_STEER_SETTLE_MS)
            return;

        parkingEntryDrivePhase = PARKING_ENTRY_REVERSE_ARC;
        set_speed(-static_cast<int>(OBSTACLE_PARKING_ENTRY_SPEED_MM_S));
        if (!parkingEntryControlLogged)
        {
            parkingEntryControlLogged = true;
            Serial.print("[PARK ENTRY CONTROL] phase=REVERSE_ARC travel_mm=");
            Serial.print(entryTravel, 1);
            Serial.print(" steering=");
            Serial.print(arcSteering);
            Serial.print(" arc_mm/radius_mm=");
            Serial.print(parkingEntryScanArcMm(), 1);
            Serial.print("/");
            Serial.println(OBSTACLE_PARKING_ENTRY_SCAN_RADIUS_MM, 1);
        }
        return;
    }

    const float finishError = hypotf(
        pose.x_mm - finish.x, pose.y_mm - finish.y);
    const float finishHeadingError = fabsf(
        wrap180(pose.heading_deg - finish.headingDeg));
    const bool pathReached =
        entryTravel >=
            plannedTravel - (parkingShortStart() ? 3.0f :
                OBSTACLE_PARKING_ENTRY_FINISH_TOLERANCE_MM) &&
        finishError <= OBSTACLE_PARKING_ENTRY_FINISH_TOLERANCE_MM &&
        finishHeadingError <= (parkingShortStart() ? 2.0f :
            OBSTACLE_PARKING_ENTRY_FINISH_HEADING_DEG);
    const bool travelLimitReached =
        entryTravel >=
            plannedTravel + OBSTACLE_PARKING_ENTRY_MAX_OVERRUN_MM;
    if (pathReached || travelLimitReached)
    {
        stop(true);
        set_steering(0);
        parkingEntryPathFailed = !pathReached;
        parkingEntryObserving = true;
        parkingEntryObserveStartMs = millis();
        Serial.print(
            pathReached
                ? "[PARK ENTRY] Scan pose reached error_mm="
                : "[PARK ENTRY] Scan path overrun error_mm=");
        Serial.print(finishError, 1);
        Serial.print(" heading_error_deg=");
        Serial.print(finishHeadingError, 1);
        Serial.print(" travel/planned_mm=");
        Serial.print(entryTravel, 1);
        Serial.print("/");
        Serial.print(plannedTravel, 1);
        Serial.print(" target_station=");
        Serial.println(parkingEntryTargetStation);
        return;
    }

    int steering = 0;
    if (parkingEntryDrivePhase == PARKING_ENTRY_REVERSE_ARC)
    {
        steering = routeTurnSign * OBSTACLE_PARKING_EXIT_STEERING;
    }
    else
    {
        const float headingError = wrap180(
            pose.heading_deg - parkingEntryStraightHeadingDeg);
        parkingEntryStraightFilteredHeadingErrorDeg +=
            OBSTACLE_PARKING_ENTRY_STRAIGHT_HEADING_FILTER_ALPHA *
            (headingError - parkingEntryStraightFilteredHeadingErrorDeg);
        parkingEntryStraightMaxHeadingErrorDeg = fmaxf(
            parkingEntryStraightMaxHeadingErrorDeg,
            fabsf(headingError));
        if (
            fabsf(headingError) >
            OBSTACLE_PARKING_ENTRY_STRAIGHT_HEADING_ABORT_DEG)
        {
            stop(false);
            set_steering(0);
            parkingEntryActive = false;
            parkingEntryPathFailed = true;
            parkingEntryTestHold = true;
            Serial.print(
                "[PARK ENTRY] Straight heading abort error/limit_deg=");
            Serial.print(headingError, 1);
            Serial.print("/");
            Serial.print(
                OBSTACLE_PARKING_ENTRY_STRAIGHT_HEADING_ABORT_DEG,
                1);
            Serial.println(" - drive motor locked off");
            if (!parkingEntryUsbWritten)
            {
                robot_logger.write_to_usb();
                parkingEntryUsbWritten = true;
            }
            return;
        }

        const float straightControlTravel = fabsf(
            get_distance() - parkingEntryStraightControlStartDistance);
        const float feedforwardScale = clampFloat(
            1.0f - straightControlTravel /
                OBSTACLE_PARKING_ENTRY_STRAIGHT_FEEDFORWARD_FADE_MM,
            0.0f,
            1.0f);
        const float feedforward =
            -routeTurnSign *
            OBSTACLE_PARKING_ENTRY_STRAIGHT_FEEDFORWARD_DEG *
            feedforwardScale;
        float correction = feedforward;
        if (
            fabsf(parkingEntryStraightFilteredHeadingErrorDeg) >=
            OBSTACLE_PARKING_ENTRY_STRAIGHT_CORRECTION_START_DEG)
        {
            correction = clampFloat(
                feedforward -
                    OBSTACLE_PARKING_ENTRY_STRAIGHT_HEADING_KP *
                        parkingEntryStraightFilteredHeadingErrorDeg,
                -OBSTACLE_PARKING_ENTRY_STRAIGHT_MAX_STEERING_DEG,
                OBSTACLE_PARKING_ENTRY_STRAIGHT_MAX_STEERING_DEG);
        }
        steering = static_cast<int>(roundf(correction));
        if (!parkingEntryStraightControlLogged && fabsf(headingError) >= 0.5f)
        {
            parkingEntryStraightControlLogged = true;
            Serial.print(
                "[PARK ENTRY CONTROL] phase=REVERSE_STRAIGHT heading_target/error_deg=");
            Serial.print(parkingEntryStraightHeadingDeg, 1);
            Serial.print("/");
            Serial.print(headingError, 1);
            Serial.print(" steering=");
            Serial.println(steering);
        }
    }
    set_steering(steering);
    set_speed(-static_cast<int>(OBSTACLE_PARKING_ENTRY_SPEED_MM_S));
}

// Later laps use actual rectangular wall intersections. A rounded baseline
// plus a fixed corridor offset is not a physical wall in a corner.
bool laterLapWallReference(float sx, float sy, float rx, float ry,
                           float &distance, float &nx, float &ny)
{
    distance = 1.0e9f;
    nx = ny = 0.0f;
    for (const WallSegment &wall : FIELD_WALLS)
    {
        const float wx = wall.bx - wall.ax, wy = wall.by - wall.ay;
        const float determinant = rx * wy - ry * wx;
        if (fabsf(determinant) < 1.0e-5f) continue;
        const float ax = wall.ax - sx, ay = wall.ay - sy;
        const float t = (ax * wy - ay * wx) / determinant;
        const float u = (ax * ry - ay * rx) / determinant;
        if (t <= 0.0f || u < 0.0f || u > 1.0f || t >= distance) continue;
        // Near a vertex or grazing incidence the finite sensor cone may hit
        // either wall. Reject rather than fabricate a localization offset.
        const float length = hypotf(wx, wy);
        const float normalX = -wy / length, normalY = wx / length;
        const float incidence = fabsf(rx * normalX + ry * normalY);
        distance = t;
        nx = normalX; ny = normalY;
        if (incidence < 0.8f || fminf(u, 1.0f-u) * length < 80.0f)
            nx = ny = 0.0f;
    }
    if (distance > OBSTACLE_TOF_CORRECTION_MAX_RANGE_MM ||
        nx * nx + ny * ny < 0.5f) return false;
    for (const CandidateSeat &seat : seats)
    {
        if (!seat.confirmed) continue;
        const float dx = seat.x - sx, dy = seat.y - sy;
        const float along = dx * rx + dy * ry;
        const float across = fabsf(dx * ry - dy * rx);
        // Pillar half diagonal plus a conservative 15 degree half-cone.
        if (along > -40.0f && along < distance + 40.0f &&
            across < 40.0f + fmaxf(0.0f, along) * 0.268f) return false;
    }
    // Both extreme measured bay gaps, including their thickness. Treat any
    // piece intersecting the sensor cone as an ambiguous wall reference.
    const float pieceX[] = {490.0f, 227.5f, 237.5f};
    for (float px : pieceX)
    {
        for (float py = -1500.0f; py <= -1300.0f; py += 20.0f)
        {
            const float dx = px - sx, dy = py - sy;
            const float along = dx * rx + dy * ry;
            const float across = fabsf(dx * ry - dy * rx);
            if (along > -25.0f && along < distance + 25.0f &&
                across < 25.0f + fmaxf(0.0f, along) * 0.268f)
                return false;
        }
    }
    return true;
}

// Parking pieces stay on the field for all laps. This short prediction is
// a final collision guard, independent of start-only half-plane restrictions.
// The route must already provide clearance; rejection never selects a detour.
bool parkingReturnMotionSafe(const PositionEstimate &pose, int servo)
{
    if (pose.y_mm > -1000.0f || pose.x_mm < 50.0f || pose.x_mm > 800.0f)
        return true;
    float x = pose.x_mm, y = pose.y_mm;
    float heading = pose.heading_deg * PI / 180.0f;
    const float curvature = abs(servo) < 2 ? 0.0f :
        -1.0f / Ackermann::getTurnRadius(servo);
    for (uint8_t step = 0; step <= 5; ++step)
    {
        if (!parking_start_footprint::safe(x, y, heading * 180.0f / PI,
                -1, false, 5.0f) ||
            !parking_start_footprint::safe(x, y, heading * 180.0f / PI,
                1, false, 5.0f)) return false;
        const float next = heading + curvature * 10.0f;
        if (fabsf(curvature) > 1.0e-6f)
        {
            x += (sinf(next) - sinf(heading)) / curvature;
            y += (cosf(heading) - cosf(next)) / curvature;
        }
        else { x += 10.0f * cosf(heading); y += 10.0f * sinf(heading); }
        heading = next;
    }
    return true;
}

float laterLapServoForCurvature(float curvature)
{
    if (fabsf(curvature) < 1.0e-6f) return 0.0f;
    return Ackermann::getServoAngleForRadius(-1.0f / curvature);
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
        {
            // All driving laps use physical rectangular walls.
            // Filter lag near a pillar/corner must not move the map. Only use
            // fresh, settled readings whose beam can be assigned to a wall.
            float wallDistance, normalX, normalY;
            const float rayX = cosf(sensorRayHeading), rayY = sinf(sensorRayHeading);
            if (millis() - snapshot.sampled_ms > 150 ||
                !isfinite(snapshot.selected_raw_distance_mm) ||
                fabsf(snapshot.selected_raw_distance_mm - reading) > 40.0f ||
                !laterLapWallReference(sensorX, sensorY, rayX, rayY,
                                       wallDistance, normalX, normalY)) continue;
            const float residual = (wallDistance - reading) *
                (rayX * normalX + rayY * normalY);
            if (left) result.leftResidualMm = residual;
            else result.rightResidualMm = residual;
            if (!isfinite(residual) ||
                fabsf(residual) > OBSTACLE_TOF_CORRECTION_MAX_RESIDUAL_MM) continue;
            const float step = clampFloat(residual * OBSTACLE_TOF_CORRECTION_GAIN,
                -OBSTACLE_TOF_CORRECTION_MAX_STEP_MM,
                 OBSTACLE_TOF_CORRECTION_MAX_STEP_MM);
            correctionX += normalX * step; correctionY += normalY * step;
            if (left) result.leftUsed = true; else result.rightUsed = true;
            ++corrections;
            continue;
        }

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

PathPoint connectorLookaheadFrom(
    uint8_t index, float lookaheadMm, const PathPoint *route,
    uint16_t mergeIndex, bool continueIntoRoute,
    uint16_t *targetIndex)
{
    float accumulated = 0.0f;
    while (index + 1 < parkingEntryConnectorLength &&
           accumulated < lookaheadMm)
    {
        accumulated += hypotf(
            parkingEntryConnector[index + 1].x -
                parkingEntryConnector[index].x,
            parkingEntryConnector[index + 1].y -
                parkingEntryConnector[index].y);
        ++index;
    }
    PathPoint target = parkingEntryConnector[index];
    if (targetIndex != nullptr)
        *targetIndex = index;
    if (!continueIntoRoute || accumulated >= lookaheadMm ||
        route == nullptr || pathLength < 2)
        return target;

    // Green joining can still need a forward target before the handoff gate
    // is met. Continue the same lookahead along the validated outgoing route
    // instead of pinning it to the connector endpoint.
    for (uint16_t step = 1; step < pathLength; ++step)
    {
        const PathPoint &next = route[(mergeIndex + step) % pathLength];
        const float segment = hypotf(next.x - target.x, next.y - target.y);
        if (segment < 1.0f)
            continue;
        accumulated += segment;
        target = next;
        if (targetIndex != nullptr)
            *targetIndex = parkingEntryConnectorLength + step - 1;
        if (accumulated >= lookaheadMm)
            break;
    }
    return target;
}

PathPoint findConnectorLookahead(
    float lookaheadMm, const PathPoint *route)
{
    return connectorLookaheadFrom(
        parkingEntryConnectorProgress, lookaheadMm, route,
        parkingEntryConnectorMergeIndex,
        parkingEntryConnectorRouteLookahead);
}
} // namespace

void obstacle_path_set_section_layout_mode(ObstacleSectionLayoutMode mode)
{
    sectionLayoutMode = mode == OBSTACLE_SECTION_LAYOUT_CHECK_ALL
        ? OBSTACLE_SECTION_LAYOUT_CHECK_ALL
        : OBSTACLE_SECTION_LAYOUT_OFFICIAL;
}

ObstacleSectionLayoutMode obstacle_path_section_layout_mode()
{
    return sectionLayoutMode;
}

void obstacle_path_reset()
{
    telemetryRouteDirty=true;
    pathLength = 0;
    progressIndex = 0;
    completedLaps = 0;
    laterTrackingTraceMs = 0;
    laterTrackingTraceCount = 0;
    parkingStartRoiLogged = false;
    routeTurnSign = 1;
    running = false;
    finished = false;
    optimizedBuilt = false;
    lapBoundaryPending = false;
    lapCountingArmed = false;
    laterLapPlanRejected = false;
    lapBoundaryHoldLogged = false;
    lapFinishPending = false;
    runtimeTestMode = false;
    memset(earlyMiddleViewActive, 0, sizeof(earlyMiddleViewActive));
    memset(greenSeatCandidateThisFrame, 0,
           sizeof(greenSeatCandidateThisFrame));
    memset(oppositeSeatConflictReported, 0,
           sizeof(oppositeSeatConflictReported));
    lastGreenSeatProcessingUs = 0;
    runtimeLapTarget = 3;
    runtimeSpeedCapMmS = 0.0f;
    loopLengthMm = 0.0f;
    firstCornerDistanceMm = OBSTACLE_STRAIGHT_LENGTH_MM * 0.5f;
    injectionCount = 0;
    discoveryBlocked = false;
    discoveryBlockedStation = -1;
    discoveryHolding = false;
    discoveryHoldStation = -1;
    discoveryHoldStartMs = 0;
    lastDiscoveryTargetNudgeDeg = 0.0f;
    discoveryScanStation = -1;
    discoveryScanSide = -1;
    lastConfirmedSeatIndex = -1;
    extremeAdjacentReleasePending = false;
    deferredInjectionSeatIndex = -1;
    lastDiscoveryNudgeUpdateMs = 0;
    lastDiscoveryObservation = ObstacleObservationResult();
    lastDiscoveryCoveragePose = PositionEstimate();
    lastDiscoveryCoverageMs = 0;
    lastDiscoveryTraceMs = 0;
    discoveryTraceCount = 0;
    parkingEntryLength = 0;
    parkingEntryProgress = 0;
    parkingEntryTargetStation = -1;
    parkingSectionInnerSeatsOnly = false;
    parkingCwShortStart = false;
    parkingCwStoredSeat = -1;
    parkingCcwShortStart = false;
    parkingCcwFirstEdgeStart = false;
    parkingCcwStoredSeatMask = 0;
    memset(parkingGreenTraceCount, 0, sizeof(parkingGreenTraceCount));
    memset(parkingGreenTraceMs, 0, sizeof(parkingGreenTraceMs));
    parkingEntryActive = false;
    parkingEntryObserving = false;
    parkingEntryTestHold = false;
    parkingEntryJoining = false;
    parkingEntryRecovering = false;
    parkingEntryConnectorActive = false;
    parkingEntryConnectorReplanPending = false;
    parkingEntryConnectorChangedSeat = -1;
    parkingEntryScouting = false;
    parkingEntryPrimaryRetryUsed = false;
    parkingEntryScoutPhase = PARKING_ENTRY_SCOUT_STEER_SETTLE;
    parkingEntryScoutStation = -1;
    parkingEntryScoutStartEncoderDistance = 0.0f;
    parkingEntryScoutReturnTravelMm = 0.0f;
    parkingEntryScoutPhaseStartMs = 0;
    parkingEntryScoutReturnPose = PositionEstimate{};
    parkingEntryConnectorLength = 0;
    parkingEntryConnectorProgress = 0;
    parkingEntryConnectorMergeIndex = 0;
    parkingEntryConnectorStartEncoderDistance = 0.0f;
    parkingEntryConnectorLookaheadMm = 0.0f;
    parkingEntryConnectorRouteLookahead = false;
    parkingEntryFarGreenFollowup = false;
    parkingEntryFarGreenFollowupStartDistance = 0.0f;
    parkingEntryFarGreenFollowupTraceMs = 0;
    parkingEntryFarGreenFollowupTraceCount = 0;
    parkingEntryObserveStartMs = 0;
    parkingEntryUsbWritten = false;
    parkingEntryStartEncoderDistance = get_distance();
    parkingEntryJoinStartEncoderDistance = get_distance();
    parkingEntryRecoveryStartEncoderDistance = get_distance();
    parkingEntryPathFailed = false;
    parkingEntryControlLogged = false;
    parkingEntryDrivePhase = PARKING_ENTRY_STRAIGHT_STEER_SETTLE;
    parkingEntrySteerSettleStartMs = 0;
    parkingEntryStraightHeadingDeg = 0.0f;
    parkingEntryStraightControlStartDistance = 0.0f;
    parkingEntryStraightMaxHeadingErrorDeg = 0.0f;
    parkingEntryStraightFilteredHeadingErrorDeg = 0.0f;
    parkingEntryStraightControlLogged = false;
    memset(lastTofCorrectionSequence, 0, sizeof(lastTofCorrectionSequence));
    lastTofCorrectionResult = ObstacleTofCorrectionResult{};
    tofCorrectionSequence = 0;
    memset(seats, 0, sizeof(seats));
    memset(discoveryStations, 0, sizeof(discoveryStations));
    cornerViewPhase = CORNER_VIEW_IDLE;
    memset(cornerViewAttempted, 0, sizeof(cornerViewAttempted));
    cornerViewCanResume = false;
    cornerViewExtraUsed = false;
    cornerViewStoppedFrames = 0;
    memset(
        plannedClearanceSnapshotValid,
        0,
        sizeof(plannedClearanceSnapshotValid));
}

void obstacle_path_start(
    int8_t turn_sign,
    bool test_mode,
    float first_corner_distance_mm,
    uint8_t lap_target,
    float speed_cap_mm_s,
    bool parking_entry_discovery,
    bool ccw_first_edge_reference)
{
    obstacle_path_reset();
    Serial.print("[PATH LAYOUT] mode=");
    Serial.println(sectionLayoutMode == OBSTACLE_SECTION_LAYOUT_CHECK_ALL
        ? "CHECK_ALL_STATIONS" : "OFFICIAL_2026");
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
    const PositionEstimate measuredEntryPose = get_position_struct();
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
        if (!parking_entry_discovery)
        {
            position_reset(
                anchor.x_mm,
                anchor.y_mm,
                anchor.heading_deg);
        }

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
    if (parking_entry_discovery)
    {
        prepareParkingSectionInnerSeats();
        parkingCwShortStart = OBSTACLE_PARKING_CW_SHORT_START_ENABLED &&
            routeTurnSign < 0 && !runtimeTestMode &&
            sectionLayoutMode == OBSTACLE_SECTION_LAYOUT_OFFICIAL;
        parkingCcwShortStart = OBSTACLE_PARKING_CCW_SHORT_START_ENABLED &&
            routeTurnSign > 0 && !runtimeTestMode &&
            sectionLayoutMode == OBSTACLE_SECTION_LAYOUT_OFFICIAL;
        parkingCcwFirstEdgeStart = parkingCcwShortStart && ccw_first_edge_reference;
        if (parkingCcwShortStart)
            Serial.println("[CCW START] Short official start: front station=2; behind stations=0/1 stored for later approach");
        parkingEntryTargetStation = routeTurnSign > 0 ? 2 : 1;
        discoveryScanStation = parkingEntryTargetStation;
        const uint8_t targetFirstSeat = static_cast<uint8_t>(
            parkingEntryTargetStation * COURSE_SEATS_PER_STATION);
        discoveryScanSide =
            seats[targetFirstSeat].y > seats[targetFirstSeat + 1].y ? 0 : 1;
        buildParkingEntryPath(measuredEntryPose);
        if (!preflightCwStartArc(measuredEntryPose,
                parkingEntryScanArcMm() + parkingEntryScoutArcMm()))
        {
            parkingEntryLength = 0;
            parkingEntryTestHold = true;
            stop(false);
            Serial.println("[PARK START] Complete scan/scout footprint preflight rejected - held");
        }
        parkingEntryStraightHeadingDeg = measuredEntryPose.heading_deg;
        const float entryStraightMm = parkingEntryLength > 0
            ? fmaxf(
                  0.0f,
                  parkingEntryPath[parkingEntryLength - 1].distanceMm -
                      parkingEntryScanArcMm())
            : 0.0f;
        if (entryStraightMm <= OBSTACLE_PARKING_ENTRY_FINISH_TOLERANCE_MM)
        {
            parkingEntryDrivePhase = PARKING_ENTRY_ARC_STEER_SETTLE;
            servo_disabled = false;
            const int arcSteering =
                routeTurnSign * OBSTACLE_PARKING_EXIT_STEERING;
            set_steering(arcSteering);
            steer(arcSteering);
            Serial.print(
                "[PARK ENTRY] Localization reached arc start; steering preload=");
            Serial.println(arcSteering);
        }
        else
        {
            parkingEntryDrivePhase = PARKING_ENTRY_STRAIGHT_STEER_SETTLE;
        }
        parkingEntrySteerSettleStartMs = millis();
        parkingEntryActive = parkingEntryLength >= 2;
        Serial.print("[PARK ENTRY] Pure Pursuit discovery armed turn=");
        Serial.print(routeTurnSign > 0 ? "CCW" : "CW");
        Serial.print(" target=S0 station=");
        Serial.print(parkingEntryTargetStation);
        Serial.print(" points=");
        Serial.print(parkingEntryLength);
        Serial.print(" reverse_mm=");
        Serial.print(
            parkingEntryLength > 0
                ? parkingEntryPath[parkingEntryLength - 1].distanceMm
                : 0.0f,
            1);
        Serial.print(" start_x_y_heading=");
        Serial.print(measuredEntryPose.x_mm, 1);
        Serial.print("/");
        Serial.print(measuredEntryPose.y_mm, 1);
        Serial.print("/");
        Serial.println(measuredEntryPose.heading_deg, 1);
    }
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

    if (parkingEntryActive || parkingEntryObserving || parkingEntryTestHold)
    {
        updateParkingEntryDiscovery(new_camera_frame);
        return;
    }
    if (parkingEntryScouting)
    {
        updateParkingEntryScout(new_camera_frame);
        return;
    }

    if (cornerViewPhase != CORNER_VIEW_IDLE)
    {
        updateCornerView(new_camera_frame);
        return;
    }

    const PathPoint *path =
        optimizedBuilt ? optimizedPath : livePath;
    PositionEstimate pose = get_position_struct();
    if (!parkingEntryConnectorActive && !lapBoundaryPending)
        updateProgress(path, pose);
    if (finished)
    {
        stop(true);
        return;
    }

    if (!runtimeTestMode && new_camera_frame)
    {
        if (completedLaps == 0)
        {
            const ObstacleObservationResult observation =
                obstacle_path_observe(getLargestValidObstacle());
            lastDiscoveryObservation = observation;
            processGreenSeatCandidates(pose, observation);
            updateDiscoveryCoverage(
                lastDiscoveryObservation,
                pose,
                getLargestObstacle());
            lastDiscoveryCoveragePose = pose;
            lastDiscoveryCoverageMs = millis();
        }
    }

    if (lapBoundaryPending)
    {
        if (!completePendingLap())
        {
            stop(false);
            set_steering(0);
            return;
        }
        servo_disabled = false; // A previous map hold disabled servo writes.
        if (finished)
        {
            stop(true);
            return;
        }
    }
    // updateProgress may have built the later-lap route this very frame.
    // Select it now, before lookahead/ToF/control, not one update later.
    path = optimizedBuilt ? optimizedPath : livePath;

    // A second adjacent extreme route is intentionally held back until the
    // rear envelope is clear of the first pillar. Its confirmation remains
    // recorded, so this delays only geometry activation, not perception.
    if (!parkingEntryConnectorActive)
    {
        activateCwStoredSeat(baselinePath[progressIndex].distanceMm);
        activateCcwStoredSeats(baselinePath[progressIndex].distanceMm);
        activateDeferredInjection(baselinePath[progressIndex].distanceMm);
    }

    if (!runtimeTestMode && !parkingEntryActive && !parkingEntryObserving &&
        !parkingEntryJoining && !parkingEntryConnectorActive)
    {
        const ObstacleTofCorrectionResult correction = applyTofCorrectionAt(
            pose,
            baselinePath[progressIndex].distanceMm);
        if (correction.correctionXmm != 0.0f || correction.correctionYmm != 0.0f)
            ++tofCorrectionSequence;
        if (completedLaps > 0 || correction.leftReadingMm > 0.0f ||
            correction.rightReadingMm > 0.0f)
            lastTofCorrectionResult = correction;
    }
    pose = get_position_struct();

    if (parkingEntryConnectorActive)
    {
        if (parkingEntryConnectorReplanPending)
        {
            stop(false);
            if (retainParkingConnectorForFarGreen(
                    get_position_struct(), path))
            {
                parkingEntryConnectorReplanPending = false;
                parkingEntryConnectorChangedSeat = -1;
            }
            else
            {
                parkingEntryFarGreenFollowup = false;
                uint16_t mergeIndex = 0;
                if (!buildParkingEntryConnector(
                        get_position_struct(), path, mergeIndex))
                {
                    parkingEntryConnectorActive = false;
                    parkingEntryConnectorReplanPending = false;
                    parkingEntryConnectorChangedSeat = -1;
                    parkingEntryTestHold = true;
                    set_steering(0);
                    Serial.println(
                        "[PARK ENTRY CONNECTOR] Replan rejected - drive motor locked off");
                    return;
                }
                telemetryRouteDirty = true;
                parkingEntryConnectorMergeIndex = mergeIndex;
                parkingEntryConnectorProgress = 0;
                parkingEntryConnectorReplanPending = false;
                parkingEntryConnectorChangedSeat = -1;
                parkingEntryConnectorStartEncoderDistance = get_distance();
                progressIndex = mergeIndex;
                Serial.print("[PARK ENTRY CONNECTOR] Replanned merge_index=");
                Serial.println(mergeIndex);
            }
            // stop(false) above disables servo writes as well as drive output.
            // Resume steering only after retention or replan passed its swept
            // checks. Otherwise set_speed() restarts the motor with the last
            // physically written steering frozen (left drift in logs 455/456).
            servo_disabled = false;
            Serial.println(
                "[PARK ENTRY CONNECTOR] Validated update - servo writes enabled");
        }
        const PositionEstimate connectorPose = get_position_struct();
        if (parkingShortStart() &&
            (!parkingStartFootprintSafe(connectorPose.x_mm,
                connectorPose.y_mm, connectorPose.heading_deg, -1) ||
             !parkingStartFootprintSafe(connectorPose.x_mm,
                connectorPose.y_mm, connectorPose.heading_deg, 1)))
        {
            stop(false);
            parkingEntryConnectorActive = false;
            parkingEntryTestHold = true;
            Serial.println("[PARK START] Connector measured footprint rejected - held");
            return;
        }
        while (parkingEntryConnectorProgress + 1 <
                   parkingEntryConnectorLength &&
               distanceSquared(
                   connectorPose.x_mm,
                   connectorPose.y_mm,
                   parkingEntryConnector[
                       parkingEntryConnectorProgress + 1].x,
                   parkingEntryConnector[
                       parkingEntryConnectorProgress + 1].y) <
                   distanceSquared(
                       connectorPose.x_mm,
                       connectorPose.y_mm,
                       parkingEntryConnector[parkingEntryConnectorProgress].x,
                       parkingEntryConnector[parkingEntryConnectorProgress].y))
            ++parkingEntryConnectorProgress;

        const float connectorTravel = fabsf(
            get_distance() - parkingEntryConnectorStartEncoderDistance);
        const PathPoint &end = parkingEntryConnector[
            parkingEntryConnectorLength - 1];
        const float endDistance = hypotf(
            connectorPose.x_mm - end.x, connectorPose.y_mm - end.y);
        const float endHeadingError = fabsf(wrap180(
            connectorPose.heading_deg - end.headingDeg));
        // The proven join accepted the normal route once pose error was inside
        // this existing gate. Requiring the nearest sampled waypoint to be the
        // exact endpoint made a 25 mm discretization artifact outrank the pose
        // gate, and checking travel first aborted CLEAR logs 359/361 one point
        // before an otherwise valid handoff.
        uint16_t joinedIndex = parkingEntryConnectorMergeIndex;
        if (connectorJoinReached(connectorPose.x_mm, connectorPose.y_mm,
                connectorPose.heading_deg, path, parkingEntryConnectorMergeIndex,
                parkingEntryConnectorRouteLookahead, &joinedIndex))
        {
            parkingEntryConnectorActive = false;
            parkingEntryJoining = false;
            parkingEntryRecovering = false;
            if (parkingEntryFarGreenFollowup)
                parkingEntryFarGreenFollowupStartDistance = get_distance();
            progressIndex = joinedIndex;
            Serial.print("[PARK ENTRY CONNECTOR] Complete merge_index=");
            Serial.print(progressIndex);
            Serial.print(" endpoint_error/heading_deg=");
            Serial.print(endDistance, 1);
            Serial.print("/");
            Serial.println(endHeadingError, 1);
        }
        else if (
            connectorTravel > (parkingCcwShortStart ? OBSTACLE_PARKING_CCW_CONNECTOR_MAX_TRAVEL_MM :
                               OBSTACLE_PARKING_ENTRY_RECOVERY_MAX_TRAVEL_MM))
        {
            stop(false);
            parkingEntryConnectorActive = false;
            parkingEntryTestHold = true;
            set_steering(0);
            Serial.print("[PARK ENTRY CONNECTOR] Travel limit - drive motor locked off travel_mm=");
            Serial.print(connectorTravel, 1);
            Serial.print(" endpoint_error/heading_deg=");
            Serial.print(endDistance, 1);
            Serial.print("/");
            Serial.print(endHeadingError, 1);
            Serial.print(" progress=");
            Serial.print(parkingEntryConnectorProgress);
            Serial.print("/");
            Serial.println(parkingEntryConnectorLength - 1);
            return;
        }
    }

    const PathPoint &progress = parkingEntryConnectorActive
        ? parkingEntryConnector[parkingEntryConnectorProgress]
        : path[progressIndex];
    const float commandedSpeed = cappedPathSpeed(lapFinishPending
        ? fminf(progress.speedMmS, OBSTACLE_LATER_LAP_CORNER_SPEED)
        : recordedLapSpeed(progress.speedMmS));
    bool parkingEntryGreenJoin = false;
    if (parkingEntryJoining && parkingEntryTargetStation >= 0)
    {
        const uint8_t firstSeat = static_cast<uint8_t>(
            parkingEntryTargetStation * COURSE_SEATS_PER_STATION);
        for (uint8_t offset = 0; offset < COURSE_SEATS_PER_STATION; ++offset)
        {
            const CandidateSeat &seat = seats[firstSeat + offset];
            if (seat.confirmed && seat.injected && !seat.red)
            {
                parkingEntryGreenJoin = true;
                break;
            }
        }
    }
    float lookahead = parkingEntryConnectorActive
        ? parkingEntryConnectorLookaheadMm
        : (completedLaps > 0 ? OBSTACLE_LATER_LAP_LOOKAHEAD_MM
                             : adaptiveLookahead(commandedSpeed));
    // Connector distance is local to the temporary path and must not be fed to
    // cyclic corner gates. Only the normal lap route uses corner scaling.
    if (!parkingEntryConnectorActive && nearCorner(progress.distanceMm))
        lookahead *= OBSTACLE_LOOKAHEAD_CORNER_SCALE;
    if (parkingEntryGreenJoin)
        lookahead *= OBSTACLE_PARKING_ENTRY_GREEN_JOIN_LOOKAHEAD_SCALE;

    PathPoint target;
    if (parkingEntryConnectorActive)
    {
        target = findConnectorLookahead(lookahead, path);
    }
    else
    {
        target = findLookahead(path, pose, lookahead);
    }
    if (!runtimeTestMode && !parkingEntryActive && !parkingEntryObserving &&
        !parkingEntryJoining && !parkingEntryConnectorActive)
    {
        const float currentDistance =
            baselinePath[progressIndex].distanceMm;
        if (!traversingInjectedAvoidance(currentDistance))
        {
            applyDiscoveryTargetNudge(target, pose);
        }
        else
        {
            // Keep the validated pillar path authoritative for the complete
            // vehicle pass. Discovery resumes 150 mm beyond the pillar.
            lastDiscoveryTargetNudgeDeg = 0.0f;
            lastDiscoveryNudgeUpdateMs = millis();
            discoveryScanStation = -1;
            discoveryScanSide = -1;
        }
    }
    const float dx = target.x - pose.x_mm;
    const float dy = target.y - pose.y_mm;
    const float heading = pose.heading_deg * PI / 180.0f;
    const float localX = dx * cosf(heading) + dy * sinf(heading);
    const float localY = -dx * sinf(heading) + dy * cosf(heading);
    const float targetDistanceSquared = fmaxf(1.0f, dx * dx + dy * dy);
    const float curvature = 2.0f * localY / targetDistanceSquared;
    // Positive geometric curvature is left; positive servo command is right.
    const float requiredSteering = completedLaps > 0 && !parkingEntryConnectorActive
        ? laterLapServoForCurvature(curvature)
        : -atanf(OBSTACLE_WHEELBASE_MM * curvature) * 180.0f / PI;
    if (parkingEntryFarGreenFollowup && !parkingEntryConnectorActive)
    {
        constexpr float followupTravelMm = 1000.0f;
        constexpr float maximumCrossTrackMm = 100.0f;
        constexpr float maximumHeadingErrorDeg = 35.0f;
        constexpr float minimumWallMm = 80.0f;
        constexpr float minimumPillarMm = 40.0f;
        const float travel = fabsf(get_distance() -
            parkingEntryFarGreenFollowupStartDistance);
        if (travel > followupTravelMm)
        {
            parkingEntryFarGreenFollowup = false;
        }
        else
        {
            const float crossTrack = hypotf(
                pose.x_mm - path[progressIndex].x,
                pose.y_mm - path[progressIndex].y);
            // The GREEN bypass bends left before the pillar. Baseline heading
            // remains 180 degrees there and falsely classified the valid
            // bypass heading as a 35-degree route deviation in log 452.
            const float pathHeading = connectorRouteHeading(path, progressIndex);
            const float headingError = fabsf(wrap180(
                pose.heading_deg - pathHeading));
            const float poseHeading = pose.heading_deg * PI / 180.0f;
            ObstacleClearanceSample front{}, rear{};
            const bool clearancesValid = calculateClearanceAtPose(
                seats[4], pose.x_mm, pose.y_mm, pose.heading_deg, front) &&
                calculateClearanceAtPose(
                    seats[4],
                    pose.x_mm - OBSTACLE_MAX_WHEEL_HALF_WIDTH_MM *
                        cosf(poseHeading),
                    pose.y_mm - OBSTACLE_MAX_WHEEL_HALF_WIDTH_MM *
                        sinf(poseHeading), pose.heading_deg, rear);
            const float wall = fminf(front.wallMm, rear.wallMm);
            const float pillar = fminf(front.pillarMm, rear.pillarMm);
            const bool unsafe = !clearancesValid ||
                !isfinite(crossTrack) || !isfinite(headingError) ||
                !isfinite(wall) || !isfinite(pillar) ||
                crossTrack > maximumCrossTrackMm ||
                headingError > maximumHeadingErrorDeg ||
                wall <= minimumWallMm || pillar <= minimumPillarMm;
            const uint32_t now = millis();
            if (unsafe || (parkingEntryFarGreenFollowupTraceCount < 64 &&
                           now - parkingEntryFarGreenFollowupTraceMs >= 100))
            {
                parkingEntryFarGreenFollowupTraceMs = now;
                ++parkingEntryFarGreenFollowupTraceCount;
                Serial.print("[FAR GREEN FOLLOWUP] travel=");
                Serial.print(travel, 1);
                Serial.print(" pose=");
                Serial.print(pose.x_mm, 1); Serial.print(",");
                Serial.print(pose.y_mm, 1); Serial.print(",");
                Serial.print(pose.heading_deg, 1);
                Serial.print(" progress/target=");
                Serial.print(progressIndex); Serial.print("/");
                Serial.print(target.x, 1); Serial.print(",");
                Serial.print(target.y, 1);
                Serial.print(" cross/heading=");
                Serial.print(crossTrack, 1); Serial.print("/");
                Serial.print(headingError, 1);
                Serial.print(" path_heading=");
                Serial.print(pathHeading, 1);
                Serial.print(" wall/pillar=");
                Serial.print(wall, 1); Serial.print("/");
                Serial.print(pillar, 1);
                Serial.print(" steer=");
                Serial.print(requiredSteering, 1);
                Serial.print(" servo_enabled=");
                Serial.print(servo_disabled ? 0 : 1);
                Serial.print(" unsafe=");
                Serial.println(unsafe ? 1 : 0);
            }
            if (unsafe)
            {
                stop(false);
                set_steering(0);
                parkingEntryTestHold = true;
                parkingEntryFarGreenFollowup = false;
                Serial.println(
                    "[FAR GREEN FOLLOWUP] Deviation - drive motor locked off");
                return;
            }
        }
    }
    const bool connectorTrackingRejected = parkingEntryConnectorActive &&
        (!isfinite(localX) || !isfinite(requiredSteering) ||
         localX <= 1.0f ||
         fabsf(requiredSteering) > OBSTACLE_MAX_PURSUIT_STEERING_DEG);
    if (connectorTrackingRejected)
    {
        // Apply the existing safety action before formatting diagnostics.
        stop(false);
        set_steering(0);
    }
    // Bounded read-only tail telemetry. A rejection always gets a record,
    // even after the periodic budget is used. No extra sensor acquisition.
    if (parkingEntryConnectorActive)
    {
        const bool rejected = connectorTrackingRejected;
        const uint32_t now = millis();
        if (rejected || (parkingEntryConnectorProgress + 4 >= parkingEntryConnectorLength &&
                         parkingEntryConnectorTraceCount < 32 &&
                         now - parkingEntryConnectorTraceMs >= 100))
        {
            parkingEntryConnectorTraceMs = now;
            ++parkingEntryConnectorTraceCount;
            const PathPoint &end = parkingEntryConnector[parkingEntryConnectorLength - 1];
            Serial.print("[CONNECTOR_TRACK] t="); Serial.print(now);
            Serial.print(" progress="); Serial.print(parkingEntryConnectorProgress);
            Serial.print(" x="); Serial.print(pose.x_mm, 2);
            Serial.print(" y="); Serial.print(pose.y_mm, 2);
            Serial.print(" h="); Serial.print(pose.heading_deg, 2);
            Serial.print(" tx="); Serial.print(target.x, 2);
            Serial.print(" ty="); Serial.print(target.y, 2);
            Serial.print(" forward="); Serial.print(localX, 2);
            Serial.print(" lateral="); Serial.print(localY, 2);
            Serial.print(" steering="); Serial.print(requiredSteering, 3);
            Serial.print(" end_distance=");
            Serial.print(hypotf(pose.x_mm - end.x, pose.y_mm - end.y), 2);
            Serial.print(" end_heading=");
            Serial.print(fabsf(wrap180(pose.heading_deg - end.headingDeg)), 2);
            Serial.print(" lookahead="); Serial.print(lookahead, 2);
            Serial.print(" wheelbase="); Serial.print(OBSTACLE_WHEELBASE_MM, 2);
            Serial.print(" limit="); Serial.print(OBSTACLE_MAX_PURSUIT_STEERING_DEG, 2);
            Serial.print(" gate_distance="); Serial.print(OBSTACLE_PARKING_ENTRY_JOIN_CROSS_TRACK_MM, 2);
            Serial.print(" gate_heading="); Serial.print(OBSTACLE_PARKING_ENTRY_JOIN_HEADING_DEG, 2);
            Serial.print(" rejected="); Serial.println(rejected ? 1 : 0);
        }
    }
    if (connectorTrackingRejected)
    {
        parkingEntryConnectorActive = false;
        parkingEntryTestHold = true;
        Serial.print(
            "[PARK ENTRY CONNECTOR] Forward tracking rejected - drive motor locked off forward_mm=");
        Serial.print(localX, 1);
        Serial.print(" steering_deg=");
        Serial.print(requiredSteering, 1);
        Serial.print(" progress=");
        Serial.print(parkingEntryConnectorProgress);
        Serial.print("/");
        Serial.println(parkingEntryConnectorLength - 1);
        return;
    }
    const float steering = clampFloat(
        requiredSteering,
        -OBSTACLE_MAX_PURSUIT_STEERING_DEG,
        OBSTACLE_MAX_PURSUIT_STEERING_DEG);

    const bool returningPastParking = !parkingEntryActive &&
        !parkingEntryObserving && !parkingEntryJoining &&
        !parkingEntryConnectorActive &&
        (completedLaps > 0 || baselinePath[progressIndex].distanceMm > loopLengthMm - 1800.0f);
    if (returningPastParking && !parkingReturnMotionSafe(pose, static_cast<int>(steering)))
    {
        stop(false); set_steering(0); parkingEntryTestHold = true;
        Serial.print("[PARK RETURN] Footprint guard held pose=");
        Serial.print(pose.x_mm, 1); Serial.print(",");
        Serial.print(pose.y_mm, 1); Serial.print(",");
        Serial.println(pose.heading_deg, 1);
        return;
    }
    set_steering(static_cast<int>(steering));
    const char *discoveryTraceReason = nullptr;
    int discoveryTraceStation = -1;
    float safeSpeed = commandedSpeed;
    if (parkingEntryConnectorActive)
        safeSpeed = fminf(
            safeSpeed,
            OBSTACLE_PARKING_ENTRY_RECOVERY_SPEED_MM_S);
    if (parkingEntryFarGreenFollowup && !parkingEntryConnectorActive)
        safeSpeed = fminf(
            safeSpeed,
            OBSTACLE_PARKING_ENTRY_JOIN_SPEED_MM_S);
    if (!runtimeTestMode && completedLaps == 0 && !parkingEntryJoining &&
        !parkingEntryConnectorActive)
    {
        float unresolvedForward = 0.0f;
        const int unresolvedStation =
            nearestUpcomingUnresolvedStation(unresolvedForward);
        if (unresolvedStation >= 0)
        {
            if (unresolvedForward <= OBSTACLE_DISCOVERY_HOLD_DISTANCE_MM)
            {
                safeSpeed = 0.0f;
                const uint32_t now = millis();
                if (!discoveryHolding ||
                    discoveryHoldStation != unresolvedStation)
                {
                    discoveryHolding = true;
                    discoveryHoldStation = unresolvedStation;
                    discoveryHoldStartMs = now;
                    discoveryTraceReason = "hold_start";
                    discoveryTraceStation = unresolvedStation;
                    Serial.print("[PATH] Perception hold at S");
                    Serial.print(unresolvedStation /
                                 COURSE_STATIONS_PER_SECTION);
                    Serial.print(" station=");
                    Serial.print(unresolvedStation %
                                 COURSE_STATIONS_PER_SECTION);
                    Serial.print(" forward_mm=");
                    Serial.print(unresolvedForward, 0);
                    Serial.print(" grace_ms=");
                    Serial.println(OBSTACLE_DISCOVERY_HOLD_GRACE_MS);
                }
                else if (!discoveryBlocked &&
                         now - discoveryHoldStartMs >=
                             OBSTACLE_DISCOVERY_HOLD_GRACE_MS)
                {
                    discoveryBlocked = true;
                    discoveryBlockedStation = unresolvedStation;
                    discoveryTraceReason = "hold_expired";
                    discoveryTraceStation = unresolvedStation;
                    Serial.print("[PATH] Perception hold expired at S");
                    Serial.print(unresolvedStation /
                                 COURSE_STATIONS_PER_SECTION);
                    Serial.print(" station=");
                    Serial.println(unresolvedStation %
                                   COURSE_STATIONS_PER_SECTION);
                }
            }
            else if (unresolvedForward <= OBSTACLE_DISCOVERY_SLOW_DISTANCE_MM)
                safeSpeed = fminf(
                    safeSpeed,
                    OBSTACLE_DISCOVERY_SPEED_MM_S);
        }
        if (discoveryHolding &&
            (unresolvedStation < 0 ||
             unresolvedStation != discoveryHoldStation ||
             unresolvedForward > OBSTACLE_DISCOVERY_HOLD_DISTANCE_MM))
        {
            Serial.print("[PATH] Perception hold resolved at station=");
            Serial.println(discoveryHoldStation);
            discoveryHolding = false;
            discoveryHoldStation = -1;
            discoveryHoldStartMs = 0;
        }
    }
    set_speed(static_cast<int>(safeSpeed));
    // Bounded onboard diagnostics: no driving USB cable required. 200 records
    // across lap1 return and laps2/3 share the existing ~40kB budget.
    if (returningPastParking && !lapFinishPending &&
        laterTrackingTraceCount < 200 && millis() - laterTrackingTraceMs >= 250)
    {
        laterTrackingTraceMs = millis(); ++laterTrackingTraceCount;
        Serial.print("[LATER_TRACK] t="); Serial.print(laterTrackingTraceMs);
        Serial.print(" lap="); Serial.print(completedLaps + 1);
        Serial.print(" idx="); Serial.print(progressIndex);
        Serial.print(" pose="); Serial.print(pose.x_mm,1); Serial.print(",");
        Serial.print(pose.y_mm,1); Serial.print(","); Serial.print(pose.heading_deg,1);
        Serial.print(" target="); Serial.print(target.x,1); Serial.print(","); Serial.print(target.y,1);
        Serial.print(" speed/servo="); Serial.print(safeSpeed,1); Serial.print("/"); Serial.print(steering,1);
        Serial.print(" tof_used="); Serial.print(lastTofCorrectionResult.leftUsed);
        Serial.print(","); Serial.print(lastTofCorrectionResult.rightUsed);
        Serial.print(" cs="); Serial.print(tofCorrectionSequence);
        Serial.print(" correction="); Serial.print(lastTofCorrectionResult.correctionXmm,1);
        Serial.print(","); Serial.println(lastTofCorrectionResult.correctionYmm,1);
    }
    // Recovery is only for unresolved section entry after the stationary
    // observation grace. Holds at middle/end stations never request reverse.
    if (discoveryTraceReason != nullptr &&
        strcmp(discoveryTraceReason, "hold_expired") == 0 &&
        discoveryHoldStation >= 0 &&
        discoveryHoldStation % COURSE_STATIONS_PER_SECTION == 0 &&
        !cornerViewAttempted[discoveryHoldStation])
    {
        holdCornerViewCentered();
        cornerViewStation = static_cast<uint8_t>(discoveryHoldStation);
        cornerViewRequestedMinRange=cornerViewMinimumRange(getLargestObstacle());
        cornerViewObservationTraceCount=0;
        cornerViewAttempted[cornerViewStation] = true;
        cornerViewExtraUsed = false;
        // Recovery is in progress; expose a terminal block only on failure.
        discoveryBlocked = false;
        discoveryBlockedStation = -1;
        cornerViewPhase = CORNER_VIEW_SETTLE;
        cornerViewPhaseMs = millis();
        Serial.println("[CORNER VIEW] Stopped; settle steering before preflight");
        Serial.print("[CORNER VIEW] build=");
        Serial.print(__DATE__); Serial.print("_"); Serial.println(__TIME__);
    }
    // Format diagnostics after the motor command, especially at a hold. Use
    // the last coverage-frame pose, not a later corrected pose; t/frame_t
    // makes stale observations explicit. No additional camera/sensor reads.
    if (discoveryTraceReason != nullptr)
        logDiscoveryTrace(discoveryTraceStation, discoveryTraceReason, true);
    else if (!runtimeTestMode && completedLaps == 0 && new_camera_frame &&
             !parkingEntryConnectorActive)
    {
        float forward = 0.0f;
        const int station = nearestUpcomingUnresolvedStation(forward);
        if (station >= 0 && forward <= OBSTACLE_DISCOVERY_SLOW_DISTANCE_MM)
            logDiscoveryTrace(station, "approach", false);
    }
}


void obstacle_path_log_run_telemetry()
{
    if (!RUN_TELEMETRY_DETAILED) return;
    static const PathPoint *previous=nullptr;
    static const char *previousKind="none";
    const PathPoint *points=optimizedBuilt?optimizedPath:livePath;
    unsigned count=pathLength;const char *kind=optimizedBuilt?"learned":"discovery";
    const char *phase=finished?"finish_brake":discoveryHolding?"discovery_hold":"driving";
    bool closed=true;
    if(parkingEntryConnectorActive){points=parkingEntryConnector;count=parkingEntryConnectorLength;kind="connector";phase="connector";closed=false;}
    else if(parkingEntryActive){points=parkingEntryPath;count=parkingEntryLength;kind="scan";phase="scan";closed=false;}
    else if(parkingEntryScouting){phase="scout";count=0;kind="scout_control";closed=false;}
    else if(parkingEntryObserving||parkingEntryTestHold){phase="scan_hold";count=0;kind="scan_hold_control";closed=false;}
    else if(cornerViewPhase!=CORNER_VIEW_IDLE){phase="corner_view";count=0;kind="corner_control";closed=false;}
    char detailedPhase[40];
    if(parkingEntryActive){snprintf(detailedPhase,sizeof(detailedPhase),"scan_%u",static_cast<unsigned>(parkingEntryDrivePhase));phase=detailedPhase;}
    else if(parkingEntryScouting){snprintf(detailedPhase,sizeof(detailedPhase),"scout_%u",static_cast<unsigned>(parkingEntryScoutPhase));phase=detailedPhase;}
    else if(cornerViewPhase!=CORNER_VIEW_IDLE){snprintf(detailedPhase,sizeof(detailedPhase),"corner_%u",static_cast<unsigned>(cornerViewPhase));phase=detailedPhase;}
    run_telemetry_phase(phase,completedLaps<3?completedLaps+1:3);
    if(previous!=points||strcmp(previousKind,kind)!=0||telemetryRouteDirty){
        run_telemetry_route(kind,points,count,sizeof(PathPoint),closed);
        previous=points;previousKind=kind;telemetryRouteDirty=false;
    }
}

bool obstacle_path_started()
{
    return running;
}

bool obstacle_path_parking_approach_target(
    float x_mm, float y_mm, float &target_x_mm, float &target_y_mm)
{
    if (!optimizedBuilt || pathLength < 2 || !isfinite(x_mm) || !isfinite(y_mm))
        return false;
    const PathPoint *route = optimizedPath;
    uint16_t nearest = 0;
    float best = INFINITY;
    for (uint16_t i = 0; i < pathLength; ++i)
    {
        if (route[i].x < -400.0f || route[i].x > 1000.0f || route[i].y > -550.0f)
            continue;
        const float distance = hypotf(x_mm-route[i].x, y_mm-route[i].y);
        if (distance < best) { best=distance; nearest=i; }
    }
    if (best > 150.0f) return false;
    float remaining = 100.0f;
    uint16_t from = nearest;
    for (uint16_t n=0; n<pathLength; ++n)
    {
        const uint16_t to = routeTurnSign > 0 ? (from+1)%pathLength
            : (from+pathLength-1)%pathLength;
        const float distance = hypotf(route[to].x-route[from].x,
                                     route[to].y-route[from].y);
        if (distance >= remaining && distance > 0.0f)
        {
            const float fraction=remaining/distance;
            target_x_mm=route[from].x+fraction*(route[to].x-route[from].x);
            target_y_mm=route[from].y+fraction*(route[to].y-route[from].y);
            return true;
        }
        remaining-=distance; from=to;
    }
    return false;
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

float obstacle_path_travel_distance_mm()
{
    if (!running || pathLength == 0 || progressIndex >= pathLength)
        return 0.0f;
    return completedLaps * loopLengthMm +
        baselinePath[progressIndex].distanceMm;
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
           closureError <= 1.0f &&
           fabsf(
               baselinePath[0].speedMmS - OBSTACLE_PATH_MAX_SPEED) <= 0.1f &&
           fabsf(
               baselinePath[pathLength - 1].speedMmS -
               OBSTACLE_PATH_MAX_SPEED) <= 0.1f;
}

bool storeCwStartSeat(uint8_t seatIndex)
{
    if (!parkingCwShortStart || completedLaps != 0 || seatIndex >= 2 ||
        seats[seatIndex].injected)
        return false;
    parkingCwStoredSeat = static_cast<int8_t>(seatIndex);
    Serial.print("[CW START] Stored behind start seat/color=");
    Serial.print(seatIndex); Serial.print("/");
    Serial.println(seats[seatIndex].red ? "RED; bypass on later approach" :
                                        "GREEN; bypass on later approach");
    return true;
}

bool storeCcwStartSeat(uint8_t seatIndex)
{
    if (!parkingCcwShortStart || completedLaps != 0 || seatIndex >= 4 ||
        seats[seatIndex].injected)
        return false;
    // After release, a repeated observation must not re-store an injected
    // seat. Before release, re-confirmation preserves both station bits.
    parkingCcwStoredSeatMask |= static_cast<uint8_t>(1U << seatIndex);
    Serial.print("[CCW START] Stored behind start seat/color=");
    Serial.print(seatIndex); Serial.print("/");
    Serial.println(seats[seatIndex].red ? "RED; bypass on later approach" :
                                        "GREEN; bypass on later approach");
    return true;
}

void finalizeConfirmedSeat(ObstacleObservationResult &result)
{
    const uint8_t seatIndex = static_cast<uint8_t>(result.seatId);
    CandidateSeat &seat = seats[seatIndex];
    result.status = OBSTACLE_OBSERVATION_CONFIRMED;
    lastConfirmedSeatIndex = result.seatId;
    extremeAdjacentReleasePending =
        hasConfirmedExtremeAdjacentPair(seatIndex);
    const int8_t earlier = earlierExtremeAdjacentSeat(seatIndex);
    if (storeCwStartSeat(seatIndex) || storeCcwStartSeat(seatIndex))
    {
        // Confirmation/map recording below still runs; route injection waits
        // for the normal return to this station, for either colour.
    }
    else if (earlier >= 0)
    {
        deferredInjectionSeatIndex = result.seatId;
        // Rebuild without the deferred seat. This also applies the
        // established extreme-pair clearance to the first member.
        rebuildLivePath();
        Serial.print("[PATH] Avoidance confirmed seat=");
        Serial.print(result.seatId);
        Serial.print(" injection=DEFERRED until_mm_past_seat_");
        Serial.print(earlier);
        Serial.print("=");
        Serial.println(OBSTACLE_EXTREME_ADJACENT_INJECTION_DELAY_MM, 0);
    }
    else
    {
        injectSeat(seatIndex, false);
    }

    if (seat.injected)
    {
        const uint16_t center = nearestPathIndex(
            baselinePath, seat.x, seat.y, 0, pathLength);
        result.peakDisplacementMm = hypotf(
            livePath[center].x - baselinePath[center].x,
            livePath[center].y - baselinePath[center].y);
        result.movementCircleClearanceMm =
            hypotf(livePath[center].x - seat.x,
                   livePath[center].y - seat.y) -
            OBSTACLE_PILLAR_MOVEMENT_RADIUS_MM;
    }

    course_map_record_seat_obstacle(
        seatIndex / 6,
        (seatIndex % 6) / 2,
        seatIndex % 2,
        seat.red ? ColorType::RED : ColorType::GREEN,
        seat.x,
        seat.y);
    // Recomputed from confirmed seats, not persisted as observed clear.
    // A later contradicting detection automatically revokes the inference.
    Serial.print("[PATH LAYOUT] section="); Serial.print(seatIndex / 6);
    Serial.print(" mode=");
    Serial.print(sectionLayoutMode == OBSTACLE_SECTION_LAYOUT_CHECK_ALL
        ? "CHECK_ALL_STATIONS" : "OFFICIAL_2026");
    Serial.print(" inferred_empty_station_mask=");
    Serial.println(sectionInferredEmpty(seatIndex / 6));

    result.redVotes = seat.redVotes;
    result.greenVotes = seat.greenVotes;
    result.confirmed = true;
    result.injected = seat.injected;
    result.injectionCount = injectionCount;
    result.passSide = seat.red ? 'R' : 'L';
}

bool mappedParkingSeatFootMatches(uint8_t seatIndex,
                                  const PositionEstimate &pose, int footY)
{
    if (!parkingSectionInnerSeatsOnly || seatIndex >= 6) return true;
    float bearing=0.0f,range=-1.0f;
    seatCameraGeometry(seatIndex,pose,bearing,range);
    const float forward=range*cosf(bearing*PI/180.0f);
    // Keep distant discovery independent; the tight projection test addresses
    // close start views where a parking boundary can resemble a tall pillar.
    if (forward < OBSTACLE_DISCOVERY_VIEW_MIN_MM ||
        forward > OBSTACLE_DISCOVERY_VIEW_MAX_MM) return true;
    const float expectedFoot=OBSTACLE_CAMERA_GROUND_HORIZON_Y+
        OBSTACLE_CAMERA_GROUND_RANGE_SCALE_MM_PX/forward;
    return isfinite(expectedFoot) &&
        fabsf(footY-expectedFoot)<=OBSTACLE_PARKING_SEAT_FOOT_TOLERANCE_PX;
}

void logRedSeatProjection(const ObstacleObservationResult &result,
                          const char *decision, bool force = false)
{
    if (!result.productionValid || result.color != ColorType::RED)
        return;
    static unsigned long lastLoggedMs = 0;
    const unsigned long now = millis();
    if (!force && lastLoggedMs != 0 && now - lastLoggedMs < 150UL)
        return;
    lastLoggedMs = now;
    int nearest = -1;
    float nearestSquared = INFINITY;
    if (isfinite(result.sightingXmm) && isfinite(result.sightingYmm))
    {
        for (uint8_t seat = 0; seat < OBSTACLE_SEAT_COUNT; ++seat)
        {
            const float errorSquared = distanceSquared(
                result.sightingXmm, result.sightingYmm,
                seats[seat].x, seats[seat].y);
            if (errorSquared < nearestSquared)
            {
                nearestSquared = errorSquared;
                nearest = seat;
            }
        }
    }
    Serial.print("[RED SEAT] t="); Serial.print(now);
    Serial.print(" decision="); Serial.print(decision);
    Serial.print(" pose="); Serial.print(result.robotXmm, 0);
    Serial.print(","); Serial.print(result.robotYmm, 0);
    Serial.print(","); Serial.print(result.robotHeadingDeg, 1);
    Serial.print(" image_x/foot=");
    Serial.print((result.left + result.right) / 2);
    Serial.print("/"); Serial.print(result.bottom);
    Serial.print(" bearing/range=");
    Serial.print(result.bearingDeg, 1);
    Serial.print("/"); Serial.print(result.rangeMm, 0);
    Serial.print(" sighting="); Serial.print(result.sightingXmm, 0);
    Serial.print(","); Serial.print(result.sightingYmm, 0);
    Serial.print(" nearest/error="); Serial.print(nearest);
    Serial.print("/"); Serial.print(nearest >= 0 ? sqrtf(nearestSquared) : -1.0f, 0);
    Serial.print(" accepted="); Serial.println(result.seatId);
}

bool distantSeatSideAmbiguous(float x, float y, float range, uint8_t seatId)
{
    if (range <= 700.0f || seatId >= OBSTACLE_SEAT_COUNT ||
        seats[seatId].confirmed) return false;
    const CandidateSeat &a=seats[seatId], &b=seats[seatId^1U];
    const float h=a.headingDeg*PI/180.0f;
    const float lateral=-(x-0.5f*(a.x+b.x))*sinf(h)+
                         (y-0.5f*(a.y+b.y))*cosf(h);
    // Rails are +/-100 mm. At >700 mm, a few ground-foot pixels plus yaw
    // error can move the projection across their midpoint (B498/499).
    // Do not latch either side within this 70 mm ambiguity band; retain
    // UNKNOWN and collect closer observations, for both colours/directions.
    return fabsf(lateral)<70.0f;
}

ObstacleObservationResult obstacle_path_observe(const Blob *blob)
{
    ObstacleObservationResult result;
    result.injectionCount = injectionCount;
    // The first-lap map is authoritative once the repeated route is built.
    // Diagnostic callers must not reclassify or displace seats on laps 2/3.
    if (completedLaps > 0)
        return result;
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
        logRedSeatProjection(result, "outside_snap");
        return result;
    }

    if (!mappedParkingSeatFootMatches(static_cast<uint8_t>(result.seatId), pose, result.bottom))
    {
        expirePendingVotes();
        result.status = OBSTACLE_OBSERVATION_NO_SEAT;
        result.seatId = -1;
        logRedSeatProjection(result, "start_foot_mismatch");
        return result;
    }

    if (distantSeatSideAmbiguous(result.sightingXmm,result.sightingYmm,
            result.rangeMm,static_cast<uint8_t>(result.seatId)))
    {
        expirePendingVotes();
        result.status=OBSTACLE_OBSERVATION_NO_SEAT;
        result.seatId=-1;
        logRedSeatProjection(result,"distant_side_ambiguous");
        return result;
    }

    if (!runtimeTestMode && !parkingEntryActive && !parkingEntryObserving &&
        !parkingEntryScouting && !parkingEntryConnectorActive)
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
            logRedSeatProjection(result, "not_upcoming");
            return result;
        }
    }

    // A station has only one legal pillar seat, including in CHECK_ALL mode.
    // A late projection of the same pillar onto the opposite seat must not
    // reshape the path to pass a second, physically impossible pillar.
    const uint8_t oppositeSeat = static_cast<uint8_t>(result.seatId) ^ 1U;
    if (seats[oppositeSeat].confirmed)
    {
        if (!oppositeSeatConflictReported[result.seatId])
        {
            oppositeSeatConflictReported[result.seatId] = true;
            Serial.print("[PATH] Ignored contradictory opposite seat=");
            Serial.print(result.seatId);
            Serial.print(" confirmed_seat=");
            Serial.println(oppositeSeat);
        }
        expirePendingVotes();
        result.status = OBSTACLE_OBSERVATION_NO_SEAT;
        result.seatId = -1;
        logRedSeatProjection(result, "opposite_confirmed");
        return result;
    }

    prepareConsecutiveVote(result.seatId, blob->color);
    CandidateSeat &seat = seats[result.seatId];
    if (seat.confirmed)
    {
        result.status = OBSTACLE_OBSERVATION_ALREADY_CONFIRMED;
    }
    else if (recordSeatVote(seat, blob->color))
    {
        finalizeConfirmedSeat(result);
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
    logRedSeatProjection(result,
        result.status == OBSTACLE_OBSERVATION_CONFIRMED
            ? "confirmed" : "vote_or_repeat",
        result.status == OBSTACLE_OBSERVATION_CONFIRMED);
    return result;
}

void logParkingGreenCheck(uint8_t seatIndex, const PositionEstimate &pose,
                          const char *reason,
                          const GreenSeatCandidate *candidate = nullptr)
{
    if (!parkingSectionInnerSeatsOnly ||
        completedLaps != 0 || seatIndex >= 6 ||
        seats[seatIndex].y < seats[seatIndex ^ 1U].y ||
        (parkingEntryActive && !parkingEntryObserving))
        return;
    const uint8_t station = seatIndex / COURSE_SEATS_PER_STATION;
    const uint32_t now = millis();
    // At most 36 short cached-data records per run. No extra frame capture.
    if (parkingGreenTraceCount[station] >= 12 ||
        (parkingGreenTraceCount[station] != 0 &&
         now - parkingGreenTraceMs[station] < 500))
        return;
    float bearing = 0, range = 0;
    seatCameraGeometry(seatIndex, pose, bearing, range);
    if (!parkingEntryObserving && !parkingEntryScouting &&
        !parkingEntryConnectorActive && range > 750)
        return;
    ++parkingGreenTraceCount[station];
    parkingGreenTraceMs[station] = now;
    Serial.print("[GREEN START CHECK] t="); Serial.print(now);
    Serial.print(" seat="); Serial.print(seatIndex);
    Serial.print(" phase=");
    Serial.print(parkingEntryScouting ? "scout" :
        (parkingEntryObserving ? "observe" :
         (parkingEntryConnectorActive ? "connector" : "route")));
    Serial.print(" reason="); Serial.print(reason);
    Serial.print(" bearing/range="); Serial.print(bearing, 1);
    Serial.print("/"); Serial.print(range, 0);
    Serial.print(" votes=");
    Serial.print(seats[seatIndex].greenSeatCandidateFrames);
    Serial.print(" red_votes="); Serial.print(seats[seatIndex].redVotes);
    if (candidate != nullptr)
    {
        Serial.print(" x/foot="); Serial.print(candidate->blob.centerX);
        Serial.print("/"); Serial.print(candidate->blob.maxY);
        Serial.print(" samples/contrast="); Serial.print(candidate->greenSamples);
        Serial.print("/"); Serial.print(candidate->brightnessContrast);
    }
    Serial.println();
}

void processGreenSeatCandidates(
    const PositionEstimate &pose,
    const ObstacleObservationResult &normalObservation)
{
    const uint32_t startedUs = micros();
    lastGreenSeatProcessingUs = 0;
    memset(greenSeatCandidateThisFrame, 0,
           sizeof(greenSeatCandidateThisFrame));
    if (runtimeTestMode || completedLaps != 0 ||
        camera.getBuffer() == nullptr ||
        camera.getWidth() != 320 || camera.getHeight() != 240)
        return;

    const uint32_t now = millis();
    const float heading = pose.heading_deg * PI / 180.0f;
    const float cameraX =
        pose.x_mm + OBSTACLE_CAMERA_LOCAL_X_MM * cosf(heading) -
        OBSTACLE_CAMERA_LOCAL_Y_MM * sinf(heading);
    const float cameraY =
        pose.y_mm + OBSTACLE_CAMERA_LOCAL_X_MM * sinf(heading) +
        OBSTACLE_CAMERA_LOCAL_Y_MM * cosf(heading);

    for (uint8_t seatIndex = 0; seatIndex < OBSTACLE_SEAT_COUNT; ++seatIndex)
    {
        CandidateSeat &seat = seats[seatIndex];
        if (parkingSectionInnerSeatsOnly &&
            sectionLayoutMode == OBSTACLE_SECTION_LAYOUT_OFFICIAL &&
            seatIndex < 6 && seat.y < seats[seatIndex ^ 1U].y)
            continue;
        if (seat.confirmed || seats[seatIndex ^ 1U].confirmed)
            continue;
        if (!seatComfortablyVisible(seatIndex, pose))
        {
            seat.greenSeatCandidateFrames = 0;
            logParkingGreenCheck(seatIndex, pose, "outside_view");
            continue;
        }

        float bearingDeg = 0.0f;
        float expectedRangeMm = -1.0f;
        seatCameraGeometry(seatIndex, pose, bearingDeg, expectedRangeMm);
        const float bearingRad = bearingDeg * PI / 180.0f;
        const float expectedForwardMm =
            expectedRangeMm * cosf(bearingRad);
        if (expectedForwardMm <= 0.0f)
        {
            seat.greenSeatCandidateFrames = 0;
            continue;
        }
        const int16_t expectedX = static_cast<int16_t>(lroundf(
            OBSTACLE_CAMERA_PRINCIPAL_X_PX -
            OBSTACLE_CAMERA_FOCAL_X_PX * tanf(bearingRad)));
        const int16_t expectedFootY = static_cast<int16_t>(lroundf(
            OBSTACLE_CAMERA_GROUND_HORIZON_Y +
            OBSTACLE_CAMERA_GROUND_RANGE_SCALE_MM_PX /
                expectedForwardMm));

        GreenSeatCandidate candidate;
        if (!vision.findGreenSeatCandidate(
                camera.getBuffer(), camera.getWidth(), camera.getHeight(),
                expectedX, expectedFootY, candidate))
        {
            seat.greenSeatCandidateFrames = 0;
            // Unknown colour on a mapped upright object is not proof of an
            // empty place. Existing primary retry/scout obtains another view.
            greenSeatCandidateThisFrame[seatIndex] = candidate.silhouetteFound;
            logParkingGreenCheck(seatIndex, pose,
                candidate.silhouetteFound ? "occupied_colour_unknown" : "no_silhouette");
            continue;
        }

        greenSeatCandidateThisFrame[seatIndex] = candidate.silhouetteFound;
        // The silhouette's measured foot supplies independent distance;
        // angular overlap alone would confuse two aligned station seats.
        const float measuredRangeMm =
            obstacle_estimate_camera_range_mm(&candidate.blob);
        const float candidateBearingDeg =
            obstacle_camera_bearing_deg(&candidate.blob);
        const float globalBearing =
            (pose.heading_deg + candidateBearingDeg) * PI / 180.0f;
        const float sightingX =
            cameraX + measuredRangeMm * cosf(globalBearing);
        const float sightingY =
            cameraY + measuredRangeMm * sinf(globalBearing);
        float snapErrorMm = -1.0f;
        if (!isfinite(measuredRangeMm) || measuredRangeMm <= 0.0f ||
            fabsf(measuredRangeMm - expectedRangeMm) >
                OBSTACLE_GREEN_SEAT_RANGE_TOLERANCE_MM ||
            nearestSeatIndex(sightingX, sightingY, &snapErrorMm) != seatIndex ||
            (normalObservation.productionValid &&
             normalObservation.seatId == seatIndex &&
             normalObservation.color == ColorType::RED))
        {
            seat.greenSeatCandidateFrames = 0;
            const char *reason = !isfinite(measuredRangeMm) || measuredRangeMm <= 0
                ? "invalid_range"
                : (fabsf(measuredRangeMm - expectedRangeMm) >
                       OBSTACLE_GREEN_SEAT_RANGE_TOLERANCE_MM
                   ? "range_mismatch"
                   : (normalObservation.productionValid &&
                      normalObservation.seatId == seatIndex &&
                      normalObservation.color == ColorType::RED
                      ? "red_conflict" : "wrong_seat_snap"));
            logParkingGreenCheck(seatIndex, pose, reason, &candidate);
            continue;
        }

        greenSeatCandidateThisFrame[seatIndex] = true;
        if (seat.lastGreenSeatCandidateMs == 0 ||
            now - seat.lastGreenSeatCandidateMs >
                OBSTACLE_GREEN_SEAT_VOTE_WINDOW_MS)
            seat.greenSeatCandidateFrames = 0;
        seat.lastGreenSeatCandidateMs = now;
        if (seat.greenSeatCandidateFrames < 255)
            ++seat.greenSeatCandidateFrames;
        if (seat.greenSeatCandidateFrames <
            OBSTACLE_GREEN_SEAT_CONFIRM_FRAMES || seat.redVotes != 0)
        {
            logParkingGreenCheck(seatIndex, pose, "collecting_votes", &candidate);
            continue;
        }

        // This path uses three fresh, geometrically matched frames. The
        // ordinary blob votes and red recognition remain independent.
        seat.confirmed = true;
        seat.red = false;
        seat.greenVotes = seat.greenSeatCandidateFrames;
        ObstacleObservationResult result;
        result.productionValid = true;
        result.color = ColorType::GREEN;
        result.left = candidate.blob.minX;
        result.top = candidate.blob.minY;
        result.right = candidate.blob.maxX;
        result.bottom = candidate.blob.maxY;
        result.bearingDeg = candidateBearingDeg;
        result.rangeMm = measuredRangeMm;
        result.robotXmm = pose.x_mm;
        result.robotYmm = pose.y_mm;
        result.robotHeadingDeg = pose.heading_deg;
        result.cameraXmm = cameraX;
        result.cameraYmm = cameraY;
        result.sightingXmm = sightingX;
        result.sightingYmm = sightingY;
        result.seatId = seatIndex;
        result.snapErrorMm = snapErrorMm;
        finalizeConfirmedSeat(result);
        Serial.print("[GREEN SEAT] confirmed seat=");
        Serial.print(seatIndex);
        Serial.print(" frames=");
        Serial.print(seat.greenSeatCandidateFrames);
        Serial.print(" x/foot=");
        Serial.print(candidate.blob.centerX);
        Serial.print("/");
        Serial.print(candidate.blob.maxY);
        Serial.print(" green_samples/contrast=");
        Serial.print(candidate.greenSamples);
        Serial.print("/");
        Serial.print(candidate.brightnessContrast);
        Serial.print(" range/snap_mm=");
        Serial.print(measuredRangeMm, 0);
        Serial.print("/");
        Serial.println(snapErrorMm, 0);
        if (!normalObservation.productionValid)
            lastDiscoveryObservation = result;
    }
    lastGreenSeatProcessingUs = micros() - startedUs;
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

const char *obstacle_path_wall_feature_name(ObstacleWallFeature feature)
{
    switch (feature)
    {
    case OBSTACLE_WALL_OUTER_SOUTH: return "outer_south";
    case OBSTACLE_WALL_OUTER_EAST: return "outer_east";
    case OBSTACLE_WALL_OUTER_NORTH: return "outer_north";
    case OBSTACLE_WALL_OUTER_WEST: return "outer_west";
    case OBSTACLE_WALL_INNER_SOUTH: return "inner_south_face";
    case OBSTACLE_WALL_INNER_EAST: return "inner_east_face";
    case OBSTACLE_WALL_INNER_NORTH: return "inner_north_face";
    case OBSTACLE_WALL_INNER_WEST: return "inner_west_face";
    case OBSTACLE_WALL_INNER_CORNER_SW: return "inner_corner_SW";
    case OBSTACLE_WALL_INNER_CORNER_SE: return "inner_corner_SE";
    case OBSTACLE_WALL_INNER_CORNER_NE: return "inner_corner_NE";
    case OBSTACLE_WALL_INNER_CORNER_NW: return "inner_corner_NW";
    default: return "unknown";
    }
}

bool obstacle_path_sample_pose_clearance(
    uint8_t seat_id,
    float x_mm,
    float y_mm,
    float heading_deg,
    ObstacleClearanceSample &sample)
{
    if (!running || seat_id >= OBSTACLE_SEAT_COUNT)
        return false;
    return calculateClearanceAtPose(
        seats[seat_id], x_mm, y_mm, heading_deg, sample);
}

bool obstacle_path_get_planned_clearance(
    uint8_t seat_id,
    ObstacleClearanceSample &sample)
{
    if (!running || seat_id >= OBSTACLE_SEAT_COUNT || pathLength < 2)
        return false;
    if (plannedClearanceSnapshotValid[seat_id])
    {
        sample = plannedClearanceAtInjection[seat_id];
        return sample.valid;
    }
    const CandidateSeat &seat = seats[seat_id];
    const PathPoint *path = optimizedBuilt ? optimizedPath : livePath;
    sample = ObstacleClearanceSample{};
    for (uint8_t corner = 0; corner < 4; ++corner)
        sample.innerCornerMm[corner] = 1.0e9f;

    bool found = false;
    for (uint16_t i = 0; i < pathLength; ++i)
    {
        const float forward = cyclicDistanceForward(
            seat.pathDistanceMm, path[i].distanceMm);
        const bool inWindow =
            forward <= OBSTACLE_TOF_PASSAGE_AFTER_MM ||
            forward >= loopLengthMm - OBSTACLE_TOF_PASSAGE_BEFORE_MM;
        if (!inWindow)
            continue;
        const uint16_t before = (i + pathLength - 1) % pathLength;
        const uint16_t after = (i + 1) % pathLength;
        const float headingDeg = atan2f(
            path[after].y - path[before].y,
            path[after].x - path[before].x) * 180.0f / PI;
        ObstacleClearanceSample instant;
        if (!calculateClearanceAtPose(
                seat, path[i].x, path[i].y, headingDeg, instant))
            continue;
        if (!found || instant.pillarMm < sample.pillarMm)
        {
            sample.pillarMm = instant.pillarMm;
            sample.robotXmm = instant.robotXmm;
            sample.robotYmm = instant.robotYmm;
            sample.robotHeadingDeg = instant.robotHeadingDeg;
        }
        if (!found || instant.wallMm < sample.wallMm)
        {
            sample.wallMm = instant.wallMm;
            sample.wallFeature = instant.wallFeature;
            sample.wallXmm = instant.wallXmm;
            sample.wallYmm = instant.wallYmm;
            sample.wallRobotXmm = instant.wallRobotXmm;
            sample.wallRobotYmm = instant.wallRobotYmm;
            sample.wallRobotHeadingDeg = instant.wallRobotHeadingDeg;
        }
        for (uint8_t corner = 0; corner < 4; ++corner)
            sample.innerCornerMm[corner] = fminf(
                sample.innerCornerMm[corner], instant.innerCornerMm[corner]);
        found = true;
    }
    sample.valid = found;
    return found;
}

void obstacle_path_clear_observations()
{
    if (!running)
        return;
    if (completedLaps > 0)
    {
        Serial.println("[LAPS] Clear rejected: lap-2/3 map frozen; restart run to relearn");
        return;
    }
    memcpy(livePath, baselinePath, sizeof(PathPoint) * pathLength);
    memcpy(optimizedPath, baselinePath, sizeof(PathPoint) * pathLength);
    optimizedBuilt = false;
    injectionCount = 0;
    parkingCwStoredSeat = -1;
    parkingCcwStoredSeatMask = 0;
    lastConfirmedSeatIndex = -1;
    extremeAdjacentReleasePending = false;
    deferredInjectionSeatIndex = -1;
    discoveryBlocked = false;
    discoveryBlockedStation = -1;
    discoveryHolding = false;
    discoveryHoldStation = -1;
    discoveryHoldStartMs = 0;
    memset(
        plannedClearanceSnapshotValid,
        0,
        sizeof(plannedClearanceSnapshotValid));
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
    const float expectedTestLookaheadMm =
        OBSTACLE_LOOKAHEAD_MIN_MM +
        (OBSTACLE_LOOKAHEAD_MAX_MM - OBSTACLE_LOOKAHEAD_MIN_MM) *
            (OBSTACLE_PATH_TEST_MAX_SPEED_MM_S -
             OBSTACLE_PATH_MIN_SPEED) /
            (OBSTACLE_PATH_MAX_SPEED - OBSTACLE_PATH_MIN_SPEED);
    if (!obstacle_path_geometry_valid() ||
        OBSTACLE_SEAT_COUNT != 24 ||
        OBSTACLE_SEAT_CONFIRM_VOTES != 2 ||
        fabsf(adaptiveLookahead(OBSTACLE_PATH_TEST_MAX_SPEED_MM_S) -
              expectedTestLookaheadMm) > 0.1f ||
        fabsf(adaptiveLookahead(OBSTACLE_PATH_MIN_SPEED) -
              OBSTACLE_LOOKAHEAD_MIN_MM) > 0.1f ||
        fabsf(adaptiveLookahead(OBSTACLE_PATH_MAX_SPEED) -
              OBSTACLE_LOOKAHEAD_MAX_MM) > 0.1f ||
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
        obstacle_blob_valid_for_acquisition(&floorFragment) ||
        !rejectedBlobBlocksSeatClear(&floorFragment, 20.0f))
        return false;
    floorFragment.maxY = OBSTACLE_MIN_BOTTOM_Y - 1;
    if (rejectedBlobBlocksSeatClear(&floorFragment, 20.0f))
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

    CandidateSeat outerRed = seats[6];
    CandidateSeat outerGreen = seats[9];
    CandidateSeat moderateRed = seats[7];
    CandidateSeat moderateGreen = seats[8];
    CandidateSeat separatedGreen = seats[11];
    outerRed.red = true;
    outerGreen.red = false;
    moderateRed.red = true;
    moderateGreen.red = false;
    separatedGreen.red = false;
    const float reducedOuterReversalMm = fabsf(
        targetLateralForSeat(
            outerGreen,
            OBSTACLE_EXTREME_ADJACENT_SECOND_CLEARANCE_MM) -
        targetLateralForSeat(
            outerRed,
            OBSTACLE_EXTREME_ADJACENT_CLEARANCE_MM));
    const float nominalWheelMarginMm =
        OBSTACLE_EXTREME_ADJACENT_CLEARANCE_MM -
        OBSTACLE_PILLAR_MOVEMENT_RADIUS_MM -
        OBSTACLE_MAX_WHEEL_HALF_WIDTH_MM;
    const float outerHeading = outerRed.headingDeg * PI / 180.0f;
    const float outerNormalX = -sinf(outerHeading);
    const float outerNormalY = cosf(outerHeading);
    const float outerCenterX =
        outerRed.x - outerNormalX * outerRed.lateralMm;
    const float outerCenterY =
        outerRed.y - outerNormalY * outerRed.lateralMm;
    const float outerTarget = targetLateralForSeat(
        outerRed, OBSTACLE_EXTREME_ADJACENT_CLEARANCE_MM);
    ObstacleClearanceSample envelopeCheck;
    if (!calculateClearanceAtPose(
            outerRed,
            outerCenterX + outerNormalX * outerTarget,
            outerCenterY + outerNormalY * outerTarget,
            outerRed.headingDeg,
            envelopeCheck))
        return false;
    if (!isExtremeAdjacentPair(6, outerRed, 9, outerGreen) ||
        isExtremeAdjacentPair(7, moderateRed, 8, moderateGreen) ||
        isExtremeAdjacentPair(6, outerRed, 11, separatedGreen) ||
        !targetsOuterExtreme(outerRed) ||
        targetsOuterExtreme(moderateRed) ||
        !upcomingAdjacentStationUnresolved(6) ||
        upcomingAdjacentStationUnresolved(10) ||
        fabsf(
            validatedClearanceForSeat(6) -
            OBSTACLE_OUTER_SAFE_CLEARANCE_MM) > 0.1f ||
        fabsf(
            optimizedClearanceForSeat(6) -
            OBSTACLE_OPTIMIZED_OUTER_CLEARANCE_MM) > 0.1f ||
        fabsf(
            optimizedClearanceForSeat(7) -
            OBSTACLE_LAP1_CLEARANCE_MM) > 0.1f ||
        !optimizedUsesOuterPlateau(6) ||
        optimizedUsesOuterPlateau(7) ||
        OBSTACLE_OPTIMIZED_OUTER_CLEARANCE_MM <=
            OBSTACLE_EXTREME_ADJACENT_CLEARANCE_MM ||
        OBSTACLE_OPTIMIZED_OUTER_CLEARANCE_MM >=
            OBSTACLE_OUTER_SAFE_CLEARANCE_MM ||
        OBSTACLE_OUTER_SAFE_CLEARANCE_MM <=
            OBSTACLE_EXTREME_ADJACENT_CLEARANCE_MM ||
        OBSTACLE_OUTER_SAFE_CLEARANCE_MM >=
            OBSTACLE_LAP1_CLEARANCE_MM ||
        OBSTACLE_OUTER_SAFE_EXIT_HOLD_WAYPOINTS != 1 ||
        OBSTACLE_OUTER_SAFE_EXIT_HOLD_WAYPOINTS >=
            OBSTACLE_PATH_TAPER_WAYPOINTS ||
        OBSTACLE_OUTER_SAFE_APPROACH_LEAD_WAYPOINTS != 1 ||
        OBSTACLE_OUTER_SAFE_APPROACH_LEAD_WAYPOINTS >=
            OBSTACLE_PATH_TAPER_WAYPOINTS ||
        OBSTACLE_DISCOVERY_HOLD_GRACE_MS <
            4UL * OBSTACLE_CAMERA_INTERVAL_MS ||
        OBSTACLE_DISCOVERY_HOLD_GRACE_MS > 1000UL ||
        OBSTACLE_DISCOVERY_CLEAR_FOV_MARGIN_DEG <= 0.0f ||
        OBSTACLE_DISCOVERY_CLEAR_FOV_MARGIN_DEG >=
            OBSTACLE_LOOK_FOV_MARGIN_DEG ||
        fabsf(reducedOuterReversalMm - 610.0f) > 0.1f ||
        OBSTACLE_EXTREME_ADJACENT_INJECTION_DELAY_MM <
            OBSTACLE_MAX_WHEEL_HALF_WIDTH_MM ||
        OBSTACLE_EXTREME_ADJACENT_INJECTION_DELAY_MM >=
            OBSTACLE_STRAIGHT_LENGTH_MM * 0.5f ||
        OBSTACLE_EXTREME_ADJACENT_SECOND_CLEARANCE_MM -
                OBSTACLE_PILLAR_MOVEMENT_RADIUS_MM -
                OBSTACLE_MAX_WHEEL_HALF_WIDTH_MM <
            50.0f ||
        nominalWheelMarginMm < 30.0f ||
        !envelopeCheck.valid ||
        fabsf(envelopeCheck.pillarMm - nominalWheelMarginMm) > 0.1f ||
        !isfinite(envelopeCheck.wallMm))
        return false;

    return fabsf(
               targetLateralForSeat(redTest, OBSTACLE_LAP1_CLEARANCE_MM) -
               redTest.lateralMm + OBSTACLE_LAP1_CLEARANCE_MM) < 0.1f &&
           fabsf(
               targetLateralForSeat(greenTest, OBSTACLE_LAP1_CLEARANCE_MM) -
               greenTest.lateralMm - OBSTACLE_LAP1_CLEARANCE_MM) < 0.1f;
}
