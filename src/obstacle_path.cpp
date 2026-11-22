#include "obstacle_path.h"

#include <math.h>
#include <string.h>

#include "config.h"
#include "course_map.h"
#include "obstacle_section_layout.h"
#include "obstacle_sign_height.h"
#include "motor_control.h"
#include "obstacle.h"
#include "position_estimator.h"
#include "sensors.h"
#include "vision.h"
#include "logger.h"
#include "run_telemetry.h"
#include "parking_start_footprint.h"
#include "parking_exit_observer.h"
#include "ackermann_kinematics.h"
#include "drive_profile.h"

#undef Serial
#define Serial robot_logger

void processGreenSeatCandidates(
    const PositionEstimate &pose,
    const ObstacleObservationResult &normalObservation);

namespace
{
// Pure Pursuit uses positive curvature for a left turn; the servo is positive
// to the right. Use the calibrated linkage on every lap and in its preflight.
float pursuitServoForCurvature(float curvature)
{
    if (!isfinite(curvature)) return NAN;
    if (fabsf(curvature) < 1.0e-6f) return 0.0f;
    return Ackermann::getServoAngleForRadius(-1.0f / curvature);
}

float pursuitCurvatureForServo(int steering)
{
    if (abs(steering) < 2) return 0.0f;
    return -1.0f / Ackermann::getTurnRadius(steering);
}

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
    bool visibleRed = false;
    bool modifiedSign = false;
    uint8_t heightMm = 100;
    uint8_t heightClass = 0, heightFrames = 0;
    uint8_t heightColor = 0;
    uint32_t lastHeightFrame = 0;
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
PathPoint parkingApproachPath[OBSTACLE_MAX_PATH_WAYPOINTS];
PathPoint smoothingBuffer[OBSTACLE_MAX_PATH_WAYPOINTS];
PathPoint parkingEntryPath[OBSTACLE_PARKING_ENTRY_MAX_WAYPOINTS];
PathPoint parkingEntryConnector[
    OBSTACLE_PARKING_ENTRY_CONNECTOR_MAX_WAYPOINTS];
CandidateSeat seats[OBSTACLE_SEAT_COUNT];
int8_t surpriseSeatIndex = -1;
DiscoveryStation discoveryStations[OBSTACLE_SEAT_COUNT / 2];
bool greenSeatCandidateThisFrame[OBSTACLE_SEAT_COUNT] = {};
uint32_t greenSeatCandidateLastSeenMs[OBSTACLE_SEAT_COUNT] = {};
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
bool rollingFinishAllowed = false;
bool optimizedBuilt = false;
float laterLapSpeedLimits[OBSTACLE_MAX_PATH_WAYPOINTS] = {};
bool lapBoundaryPending = false;
// A start connector may join just BEFORE phase zero. That first seam crossing
// is not a lap; require progress through the middle half of the course first.
bool lapCountingArmed = false;
bool laterLapPlanRejected = false;
bool lapBoundaryHoldLogged = false;
bool lapFinishPending = false;
// Only the first recorded lap hands over from the discovery controller.
bool lapHandoffActive = false, lap1CommandValid = false;
float lap1LookaheadMm = 0.0f, lap1SpeedMmS = 0.0f;
float lapHandoffEncoderMm = 0.0f, lapHandoffTravelMm = 0.0f;
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
// A first straight peek per corner station during discovery lap 1. If that
// view stays ambiguous, individually preflighted parallax peeks remain active
// until the station is resolved; an evidence miss is not a terminal stop.
enum CornerViewPhase : uint8_t { CORNER_VIEW_IDLE, CORNER_VIEW_SETTLE,
    CORNER_VIEW_REVERSE, CORNER_VIEW_OBSERVE, CORNER_VIEW_RETURN,
    CORNER_VIEW_RETURN_BRAKE, CORNER_VIEW_EXTRA_SETTLE, CORNER_VIEW_EXTRA_DRIVE,
    CORNER_VIEW_LOCKED,
    CORNER_VIEW_REBASE_BRAKE };
CornerViewPhase cornerViewPhase = CORNER_VIEW_IDLE;
bool cornerViewAttempted[OBSTACLE_SEAT_COUNT / 2] = {};
uint8_t cornerViewStation = 0;
uint32_t cornerViewPhaseMs = 0;
bool cornerViewSensorPaused = false;
uint32_t cornerViewSensorPauseMs = 0;
PositionEstimate cornerViewOrigin;
float cornerViewOriginEncoder = 0.0f, cornerViewReturnEncoder = 0.0f;
float cornerViewProgressEncoder = 0.0f;
float cornerViewDistanceMm = 0.0f, cornerViewMeasuredReverseMm = 0.0f;
float cornerViewRequestedMinRange = OBSTACLE_DISCOVERY_VIEW_MIN_MM;
uint8_t cornerViewStoppedFrames = 0;
uint8_t cornerViewObservationTraceCount = 0;
bool cornerViewCanResume = false;
uint8_t cornerViewExtraTriedMask = 0;
bool cornerViewForwardTried = false;
uint16_t cornerViewSearchCycle = 0;
uint32_t cornerViewLastNoSafeTraceMs = 0;
int cornerViewExtraSteering = 0;
int cornerViewExtraDirection = -1;
PositionEstimate cornerViewExtraOrigin;
float cornerViewExtraStartEncoder = 0;
float cornerViewExtraDistanceMm = 0;
float cornerViewSearchStartEncoder = 0;
uint32_t cornerViewLastSearchMs = 0;
PathPoint
findLookahead(const PathPoint *path, const PositionEstimate &pose,
                        float lookaheadMm);
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
float parkingCcwScanArcMm = 0.0f;
uint8_t parkingCcwRefinePhase = 0;
bool parkingCcwRefineUsed = false;
uint32_t parkingCcwRefineMs = 0;
float parkingCcwRefineEncoder = 0.0f;
float parkingCcwRefineTravel = 0.0f;
int8_t parkingCcwRefineDirection = 1, parkingCcwRefineSteering = -50;
bool parkingCcwViewAlternate = false;
bool parkingCcwRefineRecovery = false, parkingEntryNeedsJoinView = false;
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
        return parkingCcwScanArcMm > 0.0f ? parkingCcwScanArcMm :
            OBSTACLE_PARKING_CCW_FIRST_EDGE_SCAN_ARC_MM;
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
float parkingEntryScoutTargetMm = 0.0f;
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

float firstLapLookahead(float speedMmS)
{
    return fminf(adaptiveLookahead(speedMmS),
                 OBSTACLE_LAP1_LOOKAHEAD_MAX_MM);
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

bool stationDirectlyResolved(uint8_t stationIndex)
{
    if (stationIndex >= OBSTACLE_SEAT_COUNT / 2)
        return true;
    const uint8_t firstSeat = stationIndex * 2;
    return discoveryStations[stationIndex].observedClear ||
           seats[firstSeat].confirmed || seats[firstSeat + 1].confirmed;
}

bool stationResolved(uint8_t stationIndex)
{
    return stationDirectlyResolved(stationIndex) ||
        (sectionInferredEmpty(stationIndex / COURSE_STATIONS_PER_SECTION) &
         (1U << (stationIndex % COURSE_STATIONS_PER_SECTION)));
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

bool recordSeatVote(CandidateSeat &seat, ColorType color, bool allowConfirmation=true)
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
    if (winningVotes < OBSTACLE_SEAT_CONFIRM_VOTES || !allowConfirmation)
        return false;

    seat.confirmed = true;
    seat.red = seat.redVotes > seat.greenVotes;
    seat.visibleRed = seat.red;
    return true;
}

// Confirmation order is not encounter order: exit frames can see a far sign
// before the first one. Require a confirmed earlier sign, or directly/legally
// resolved empty earlier stations, before deciding whether this is the first.
enum SurpriseOrder : uint8_t { SURPRISE_FIRST, SURPRISE_LATER, SURPRISE_ORDER_UNKNOWN };
SurpriseOrder surpriseOrderForSeat(uint8_t id)
{
    const float target=seats[id].pathDistanceMm;
    bool unresolved=false;
    for(uint8_t station=0;station<OBSTACLE_SEAT_COUNT/2;++station)
    {
        const uint8_t first=station*2;
        const float distance=seats[first].pathDistanceMm;
        if(distance>=target-1)continue;
        if(distance<=1)
        {
            if((seats[first].confirmed && seats[first].injected) ||
               (seats[first+1].confirmed && seats[first+1].injected))return SURPRISE_LATER;
            continue; // a stored behind-start sign is not the first encountered
        }
        if(seats[first].confirmed || seats[first+1].confirmed)return SURPRISE_LATER;
        const uint8_t inferred=sectionInferredEmpty(first/6);
        if(!discoveryStations[station].observedClear &&
           !(inferred & (1U<<((first%6)/2))))unresolved=true;
    }
    return unresolved ? SURPRISE_ORDER_UNKNOWN:SURPRISE_FIRST;
}

// Store the one special identity independently of visible colour. seat.red is
// the effective/original colour consumed by the existing map and bypass code
// on all laps; visibleRed remains the actual camera observation for diagnostics.
void rememberModifiedSign(uint8_t id,obstacle_sign_height::Class height)
{
    if(surpriseSeatIndex>=0 || id>=OBSTACLE_SEAT_COUNT)return;
    CandidateSeat &seat=seats[id];
    seat.red=!seat.visibleRed;
    seat.modifiedSign=true;
    seat.heightMm=height;
    surpriseSeatIndex=static_cast<int8_t>(id);
    Serial.print("[SURPRISE] locked seat/height/visible/original=");
    Serial.print(id);Serial.print('/');Serial.print(seat.heightMm);Serial.print('/');
    Serial.print(seat.visibleRed ? "RED":"GREEN");Serial.print('/');
    Serial.println(seat.red ? "RED; pass RIGHT":"GREEN; pass LEFT");
}

bool recordObservedSeatVote(uint8_t id,const Blob &blob)
{
    CandidateSeat &seat=seats[id];
    if(seat.confirmed)return false;
    if(blob.frameSequence && seat.lastHeightFrame==blob.frameSequence)return false;
#if !OBSTACLE_SURPRISE_RULE_ENABLED
    seat.lastHeightFrame=blob.frameSequence;
    return recordSeatVote(seat,blob.color);
#else
    if(surpriseSeatIndex>=0)
    {
        seat.lastHeightFrame=blob.frameSequence;
        return recordSeatVote(seat,blob.color);
    }
    const auto order=surpriseOrderForSeat(id);
    auto height=obstacle_sign_height::classify(blob);
    // CW46 accepted a stationary, short-looking RED fragment as the first
    // middle pillar. The same setup has the first sign at the far end
    // (CW47). In this one parked view require the full normal first sign.
    const bool cwStartMiddleRed = routeTurnSign<0 && id==2 &&
        sectionLayoutMode==OBSTACLE_SECTION_LAYOUT_OFFICIAL && parkingEntryObserving &&
        order==SURPRISE_FIRST && blob.color==ColorType::RED;
    if (cwStartMiddleRed && height!=obstacle_sign_height::NORMAL)
        return false;
    // A far, oblique short component can be the visible part of a normal
    // pillar. Leave the seat open until it can be seen nearer or more centrally.
    const float imageOffset=fabsf(blob.centerX-OBSTACLE_CAMERA_PRINCIPAL_X_PX);
    const float adjustedFoot=blob.maxY-OBSTACLE_CAMERA_FOOT_EDGE_SLOPE*imageOffset;
    // CCW42: the normal GREEN at the first middle station presented a
    // repeatable short fragment at 24 degrees. Do not spend the one surprise
    // identity on this station until the short silhouette is more central.
    const uint8_t station=id/COURSE_SEATS_PER_STATION;
    if (order!=SURPRISE_FIRST && height==obstacle_sign_height::SHORT &&
        routeTurnSign>0 && (station==4 || (station==3 && blob.color==ColorType::GREEN)) &&
        imageOffset>88.0f) return false;
    if (order!=SURPRISE_FIRST && height==obstacle_sign_height::SHORT && imageOffset>96.0f &&
        adjustedFoot-OBSTACLE_CAMERA_GROUND_HORIZON_Y<OBSTACLE_CAMERA_GROUND_RANGE_SCALE_MM_PX/480.0f)
        return false;
    // A short red component can acquire an unrelated upper edge in the
    // reconstructed silhouette. Defer the ordinary vote until the component
    // has a usable view and the encounter order permits a modified sign.
    if (blob.color==ColorType::RED &&
        (height==obstacle_sign_height::NORMAL || height==obstacle_sign_height::UNKNOWN) &&
        obstacle_sign_height::classify(blob,true)==obstacle_sign_height::SHORT &&
        (order!=SURPRISE_FIRST || (routeTurnSign<0 && station%3==1 && !cwStartMiddleRed)))
    {
        if (order!=SURPRISE_LATER || imageOffset>78.0f ||
            adjustedFoot-OBSTACLE_CAMERA_GROUND_HORIZON_Y<
                OBSTACLE_CAMERA_GROUND_RANGE_SCALE_MM_PX/600.0f) return false;
        height=obstacle_sign_height::SHORT;
    }
    // An unusable primary top must not consume this frame before the mapped
    // GREEN fallback can supply a narrower, valid measurement. Only useful
    // height evidence votes; the existing400ms window still expires old votes.
    // The protected first sign needs no height to retain its ordinary colour.
    if(order!=SURPRISE_FIRST && height==obstacle_sign_height::UNKNOWN)return false;
    seat.lastHeightFrame=blob.frameSequence;
    const auto now=millis();
    if(height==obstacle_sign_height::UNKNOWN || seat.heightClass!=height ||
       seat.heightColor!=static_cast<uint8_t>(blob.color) ||
       (seat.lastVoteMs && now-seat.lastVoteMs>OBSTACLE_SEAT_VOTE_WINDOW_MS))
        seat.heightFrames=0;
    seat.heightClass=height;seat.heightColor=static_cast<uint8_t>(blob.color);
    if(height!=obstacle_sign_height::UNKNOWN && seat.heightFrames<255)++seat.heightFrames;
    const bool modified=height==obstacle_sign_height::SHORT || height==obstacle_sign_height::TALL;
    const bool first=order==SURPRISE_FIRST;
    const bool ready=cwStartMiddleRed
        ? seat.heightFrames>=OBSTACLE_SURPRISE_HEIGHT_CONFIRM_FRAMES
        : first ||
          (height==obstacle_sign_height::NORMAL && seat.heightFrames>=OBSTACLE_SEAT_CONFIRM_VOTES) ||
          (modified && order==SURPRISE_LATER && seat.heightFrames>=OBSTACLE_SURPRISE_HEIGHT_CONFIRM_FRAMES);
    if(!recordSeatVote(seat,blob.color,ready))return false;
    if(!first && modified)
        rememberModifiedSign(id,height);
    return true;
#endif
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
            seat.heightFrames=seat.heightClass=0;
            seat.heightColor=0;
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
            seats[i].heightFrames=seats[i].heightClass=0;
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
            seat.heightFrames=seat.heightClass=0;
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

float pointSegmentDistance(float,float,float,float,float,float,float*,float*);

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
                // The reference calculation already checked identical field
                // walls for this capsule. Only the pillar depends on seat.
                clearance.pillarMm = pointSegmentDistance(seats[seat].x, seats[seat].y,
                    px,py,px+OBSTACLE_ROBOT_ENVELOPE_AXIS_FRONT_MM*cosf(heading),
                    py+OBSTACLE_ROBOT_ENVELOPE_AXIS_FRONT_MM*sinf(heading),nullptr,nullptr) -
                    OBSTACLE_MAX_WHEEL_HALF_WIDTH_MM - OBSTACLE_PILLAR_MOVEMENT_RADIUS_MM;
                const bool seatValid = isfinite(clearance.pillarMm);
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
        const float steering = pursuitServoForCurvature(curvature);
        if (!isfinite(forward) || !isfinite(steering) || forward <= 1.0f ||
            fabsf(steering) > (parkingCcwShortStart ? OBSTACLE_PARKING_CONNECTOR_PLAN_STEERING_DEG :
                                 OBSTACLE_MAX_PURSUIT_STEERING_DEG))
            return reject("tracking", travel, -1, noClearance,
                forward, steering);
        // Runtime sends integer servo degrees. Predict that same command through
        // the CAD linkage instead of integrating the ideal requested curvature.
        const float actualCurvature = pursuitCurvatureForServo(
            static_cast<int>(steering));
        const float nextHeading = heading + actualCurvature * stepMm;
        if (fabsf(actualCurvature) > 1.0e-6f)
        {
            x += (sinf(nextHeading) - sinf(heading)) / actualCurvature;
            y += (cosf(heading) - cosf(nextHeading)) / actualCurvature;
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

// Recover tracking reserve without relaxing the steering or footprint gates.
// The 508 measured pose needs more horizon than the original 150 mm target.
bool widenParkingConnectorLookahead(const PositionEstimate &pose,
    const PathPoint *route, uint8_t progress, uint16_t mergeIndex,
    bool continueIntoRoute)
{
    const float h = pose.heading_deg * PI / 180.0f;
    const auto commandAt = [&](float distance) -> float {
        const PathPoint target = connectorLookaheadFrom(progress, distance,
            route, mergeIndex, continueIntoRoute);
        const float dx = target.x - pose.x_mm, dy = target.y - pose.y_mm;
        if (dx*cosf(h)+dy*sinf(h) <= 1.0f) return NAN;
        return pursuitServoForCurvature(2.0f*(-dx*sinf(h)+dy*cosf(h)) /
            fmaxf(1.0f,dx*dx+dy*dy));
    };
    const float current = commandAt(parkingEntryConnectorLookaheadMm);
    if (isfinite(current) && fabsf(current) <= 38.0f) return false;
    const uint8_t reference = parkingCcwShortStart ? 5 : 2;
    const int8_t confirmed = seats[reference].confirmed ? reference : -1;
    const uint8_t hidden = parkingCcwShortStart ? 3 : 0;
    const int8_t guard = seats[hidden].confirmed ||
        !discoveryStations[hidden/2].observedClear ? hidden : -1;
    for (float distance=parkingEntryConnectorLookaheadMm+25.0f;
         distance<=250.0f; distance+=25.0f)
    {
        const float command = commandAt(distance);
        if (!isfinite(command) || fabsf(command)>38.0f) continue;
        if (!connectorRolloutFeasible(pose,distance,reference,confirmed,guard,
                route,mergeIndex,continueIntoRoute)) continue;
        parkingEntryConnectorLookaheadMm=distance;
        Serial.print("[PARK ENTRY CONNECTOR] Checked tracking horizon_mm=");
        Serial.println(distance,0);
        return true;
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

    // Test the actual tracked poses, not hypothetical exact on-curve poses.
    // A feasible pursuit track can differ from the Hermite centreline. The
    // complete calibrated rollout below checks forward targets, integer servo
    // commands, all swept footprints and the terminal merge for every retry.
    float desiredLookahead = adaptiveLookahead(
        OBSTACLE_PARKING_ENTRY_RECOVERY_SPEED_MM_S);
    if (!parkingShortStart() && confirmedSeat >= 0 && !seats[confirmedSeat].red)
        desiredLookahead *=
            OBSTACLE_PARKING_ENTRY_GREEN_JOIN_LOOKAHEAD_SCALE;
    parkingEntryConnectorLookaheadMm = 0.0f;
    for (float candidateLookahead = desiredLookahead;
         candidateLookahead >=
             OBSTACLE_PARKING_ENTRY_CONNECTOR_SAMPLE_MM - 0.1f;
         candidateLookahead -= OBSTACLE_PARKING_ENTRY_CONNECTOR_SAMPLE_MM)
    {
        const bool feasible = connectorRolloutFeasible(
            start, candidateLookahead, referenceSeat, confirmedSeat,
            guardSeat, route, best, continueIntoRoute);
        if (!feasible)
        {
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
            "[PARK ENTRY CONNECTOR] Preflight FAIL calibrated_tracking min_lookahead_mm=");
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

bool
preflightCwStartArc(const PositionEstimate &start, float travelMm);

void buildParkingEntryPath(const PositionEstimate &start)
{
    if (parkingCcwFirstEdgeStart)
    {
        // Keep the longest safe observation arc.580/581's settled Y/heading
        // needs60/66mm; the old68mm search floor rejected both before looking.
        // Short views can use the separately checked forward camera refinement.
        // Keep all footprint, heading, braking and hidden-pillar budgets.
        parkingCcwScanArcMm = 0.0f;
        for (float arc=OBSTACLE_PARKING_CCW_FIRST_EDGE_SCAN_ARC_MM;
             arc>=40.0f; arc-=2.0f)
            if (preflightCwStartArc(start,arc))
            { parkingCcwScanArcMm=arc; break; }
        if (parkingCcwScanArcMm > 0.0f && parkingCcwScanArcMm <
                OBSTACLE_PARKING_CCW_FIRST_EDGE_SCAN_ARC_MM)
        {
            Serial.print("[CCW START] Fitted observation arc_mm=");
            Serial.println(parkingCcwScanArcMm, 1);
        }
    }
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
    const bool compactFirstLap = !runtimeTestMode && completedLaps == 0 &&
        sectionLayoutMode == OBSTACLE_SECTION_LAYOUT_OFFICIAL &&
        seatIndex >= 6;
    if (targetsOuterExtreme(seats[seatIndex]))
    {
        if (compactFirstLap || upcomingAdjacentStationUnresolved(seatIndex))
            return OBSTACLE_OUTER_SAFE_CLEARANCE_MM;
    }
    else if (compactFirstLap)
        return OBSTACLE_LAP1_MODERATE_CLEARANCE_MM;
    return OBSTACLE_LAP1_CLEARANCE_MM;
}

float optimizedClearanceForSeat(uint8_t seatIndex)
{
    //CW GREEN outside -> RED inside: the old210mm RED offset puts the
    //rear axle at x=-690 on the west leg. Recorded tracking overshoot in623
    //reached x=-627, consuming the inner-wall room for the angled nose.
    //Use30mm less RED offset here; retain the pillar/swept-route preflight.
    if(routeTurnSign<0 && sectionLayoutMode==OBSTACLE_SECTION_LAYOUT_OFFICIAL &&
       seats[seatIndex].red && seats[seatIndex].lateralMm<0.0f)
    {
        const uint8_t sectionStart=(seatIndex/6)*6;
        for(uint8_t prior=sectionStart;prior<seatIndex;++prior)
            if(seats[prior].confirmed && !seats[prior].red &&
               seats[prior].lateralMm>0.0f)
                return 180.0f;
    }
    if (hasConfirmedExtremeAdjacentPair(seatIndex))
    {
        const float clearance=isSecondExtremeAdjacentSeat(seatIndex)
            ? OBSTACLE_EXTREME_ADJACENT_SECOND_CLEARANCE_MM
            : OBSTACLE_EXTREME_ADJACENT_CLEARANCE_MM;
        // log2CCW: light contact at a start GREEN on lap2. Retain the pair's
        // continuous bend, but add15mm only to this return-straight geometry.
        return clearance + (routeTurnSign>0 &&
            sectionLayoutMode==OBSTACLE_SECTION_LAYOUT_OFFICIAL &&
            seatIndex<6 && !seats[seatIndex].red ? 15.0f:0.0f);
    }
    if (routeTurnSign < 0 &&
        sectionLayoutMode == OBSTACLE_SECTION_LAYOUT_OFFICIAL &&
        seatIndex < 6 && !seats[seatIndex].red &&
        seats[seatIndex].lateralMm < 0.0f)
        // The protected seam stays on the learned path. The far start GREEN
        // needs extra swept-wheel room as the line bends into the first turn.
        return seatIndex == 4 ? OBSTACLE_PARKING_CW_INNER_GREEN_CLEARANCE_MM + 15.0f
                              : OBSTACLE_PARKING_CW_INNER_GREEN_CLEARANCE_MM;
    if (sectionLayoutMode == OBSTACLE_SECTION_LAYOUT_OFFICIAL &&
        seatIndex >= 6 && seatIndex % 6 / 2 == 1 && targetsOuterExtreme(seats[seatIndex]))
        return OBSTACLE_OPTIMIZED_MIDDLE_CLEARANCE_MM;
    if (targetsOuterExtreme(seats[seatIndex]))
        return OBSTACLE_OPTIMIZED_OUTER_CLEARANCE_MM;
    return sectionLayoutMode == OBSTACLE_SECTION_LAYOUT_OFFICIAL &&
            seatIndex >= 6 && seatIndex % 6 / 2 == 1
        ? OBSTACLE_OPTIMIZED_MODERATE_CLEARANCE_MM
        : (sectionLayoutMode == OBSTACLE_SECTION_LAYOUT_OFFICIAL && seatIndex >= 6 &&
           routeTurnSign < 0 && !seats[seatIndex].red
            ? OBSTACLE_OPTIMIZED_END_CLEARANCE_MM : OBSTACLE_LAP1_CLEARANCE_MM);
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
            if (fabsf(pursuitServoForCurvature(curvature)) >
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

void joinSameColourStraightEnds(PathPoint *route, bool injectedOnly, bool mixedColours = false);
void roundKnownCornerPairs(PathPoint *route, bool injectedOnly = false, bool mixedColours = false);
float smoothLaneBlend(float distance, float start, float end)
{
    const float t = clampFloat((distance - start) / (end - start), 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

int ccwDiscoveryTallRedExitSeat()
{
    if (routeTurnSign <= 0 || completedLaps != 0 ||
        sectionLayoutMode != OBSTACLE_SECTION_LAYOUT_OFFICIAL)
        return -1;
    // A visible tall GREEN is driven as RED. On the last straight it
    // pushes the car out of the central view of the two third positions.
    constexpr uint8_t innerFirst = 19;
    const auto &sign = seats[innerFirst];
    return sign.confirmed && sign.injected && sign.red &&
        sign.modifiedSign && sign.heightMm == 150 ? innerFirst : -1;
}

void shapeCcwDiscoveryAfterTallSign(PathPoint *route);
void shapeCcwDiscoveryReturnCorner(PathPoint *route);

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

void rebuildDiscoveryPath()
{
    rebuildLivePath();
    if (ccwDiscoveryTallRedExitSeat() < 0)
        return;
    shapeCcwDiscoveryAfterTallSign(livePath);
    shapeCcwDiscoveryReturnCorner(livePath);
    recomputeSpeedProfile(livePath);
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
    rebuildDiscoveryPath();
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
        const float approach = OBSTACLE_PARKING_CCW_STORED_SEAT_APPROACH_MM +
            (seats[seatIndex].red ? 0.0f : 300.0f);
        if (!(parkingCcwStoredSeatMask & bit) ||
            cyclicDistanceForward(currentDistanceMm,
                seats[seatIndex].pathDistanceMm) > approach)
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
                       float *minimumWall = nullptr, float *minimumPillar = nullptr,
                       uint32_t startedUs = 0, uint32_t budgetUs = 0,
                       uint16_t first = 0, uint16_t end = 0)
{
    if (minimumWall) *minimumWall=INFINITY;
    if (minimumPillar) *minimumPillar=INFINITY;
    if (pathLength < 3)
        return false;
    // Check the physical pose implied by each route tangent, including the
    // seam and every confirmed pillar. This is a geometric preflight, not a
    // substitute for motor/sensor tracking validation during the test laps.
    if (end == 0) end = pathLength;
    for (uint16_t i = first; i < end; ++i)
    {
        if(budgetUs && uint32_t(micros()-startedUs)>=budgetUs)return false;
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
        // Walls do not depend on the reference pillar. Compute them once per
        // pose and retain the identical capsule distance for every pillar.
        ObstacleClearanceSample reference{};
        if(!calculateClearanceAtPose(seats[0],route[i].x,route[i].y,heading,reference) ||
            !reference.valid || reference.wallMm<=OBSTACLE_LATER_LAP_ROUTE_RESERVE_MM)
        {
            Serial.print("[LAPS] Route preflight rejected index/seat/wall/pillar=");
            Serial.print(i);Serial.print("/0/");Serial.print(reference.wallMm,1);
            Serial.print('/');Serial.println(reference.pillarMm,1);
            return false;
        }
        if(minimumWall)*minimumWall=fminf(*minimumWall,reference.wallMm);
        const float h=heading*PI/180.0f;
        const float endX=route[i].x+OBSTACLE_ROBOT_ENVELOPE_AXIS_FRONT_MM*cosf(h);
        const float endY=route[i].y+OBSTACLE_ROBOT_ENVELOPE_AXIS_FRONT_MM*sinf(h);
        for (uint8_t seatIndex = 0; seatIndex < OBSTACLE_SEAT_COUNT; ++seatIndex)
        {
            if(!seats[seatIndex].confirmed)continue;
            const float gap=pointSegmentDistance(seats[seatIndex].x,seats[seatIndex].y,
                route[i].x,route[i].y,endX,endY)-OBSTACLE_MAX_WHEEL_HALF_WIDTH_MM-
                OBSTACLE_PILLAR_MOVEMENT_RADIUS_MM;
            if(!isfinite(gap) || gap<=OBSTACLE_LATER_LAP_ROUTE_RESERVE_MM)
            {
                Serial.print("[LAPS] Route preflight rejected index/seat/wall/pillar=");
                Serial.print(i);Serial.print('/');Serial.print(seatIndex);
                Serial.print('/');Serial.print(reference.wallMm,1);
                Serial.print('/');Serial.println(gap,1);
                return false;
            }
            if(minimumPillar)*minimumPillar=fminf(*minimumPillar,gap);
        }
    }
    return true;
}

// Two confirmed end signs with the same passing colour need one continuous
// straight bypass. Avoid returning to the centre between their tapers.
// Official cards exclude a middle sign. O3 must retain independent geometry.
void joinSameColourStraightEnds(PathPoint *route, bool injectedOnly, bool mixedColours)
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
        if (a<0 || b<0 || (!mixedColours && seats[a].red!=seats[b].red) ||
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
            if (!injectedOnly && !mixedColours && section!=0)
            {
                // Use one straight line between the passing lanes, with
                // tangent-continuous transitions onto the two plateaus.
                constexpr float blendMm=180.0f;
                const float fromEnd=fminf(d,span-d);
                const float travel=fromEnd<blendMm
                    ? fromEnd*fromEnd/(2.0f*blendMm) : fromEnd-0.5f*blendMm;
                t=travel/(span-blendMm);
                if(d>0.5f*span)t=1.0f-t;
            }
            const float offset=offsetA+t*(offsetB-offsetA);
            route[i].x=baselinePath[i].x+nx*offset;
            route[i].y=baselinePath[i].y+ny*offset;
        }
        if(injectedOnly){Serial.print("[PATH] Same-colour straight joined seats=");Serial.print(a);Serial.print('/');Serial.println(b);}
    }
}

void roundKnownCornerPairs(PathPoint *route, bool injectedOnly, bool mixedColours)
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
            (!mixedColours && seats[endSeat].red != seats[nextSeat].red))
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
void carryKnownInnerLaneToMiddle(PathPoint *route, bool mixedColours = false)
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
            (!mixedColours && seats[endSeat].red != seats[middleSeat].red) ||
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

float laterLapBending(const PathPoint *route, float &length, float &peak,
                     float *absoluteTurn = nullptr,
                     uint16_t first = 0, uint16_t end = 0)
{
    length=peak=0.0f;
    if(absoluteTurn)*absoluteTurn=0;
    float bending=0.0f;
    if (end == 0) end = pathLength;
    for(uint16_t i=first;i<end;++i)
    {
        const uint16_t p=(i+pathLength-1)%pathLength,n=(i+1)%pathLength;
        const float a=hypotf(route[i].x-route[p].x,route[i].y-route[p].y);
        const float b=hypotf(route[n].x-route[i].x,route[n].y-route[i].y);
        length+=b;
        if(a<1.0f || b<1.0f)continue; // coincident lap-seam point
        const float h1=atan2f(route[i].y-route[p].y,route[i].x-route[p].x);
        const float h2=atan2f(route[n].y-route[i].y,route[n].x-route[i].x);
        const float turn=wrap180((h2-h1)*180.0f/PI)*PI/180.0f;
        if(absoluteTurn)*absoluteTurn+=fabsf(turn);
        const float ds=0.5f*(a+b);
        bending+=turn*turn/ds;
        peak=fmaxf(peak,fabsf(turn)/ds);
    }
    return bending;
}

// Bounded local shortening after the map is complete. Try the largest smooth
// candidate first, then back off to the previous 6 mm refinement if needed.
// Passing plateaus, lap seam, metadata and geometric reserves stay protected.
void smoothKnownLaterLapRoute(PathPoint *route, PathPoint *rollback,
                             float oldWall, float oldPillar)
{
    memcpy(rollback,route,sizeof(PathPoint)*pathLength);
    float oldLength,oldPeak;
    const float oldBending=laterLapBending(route,oldLength,oldPeak);
    constexpr float weights[]={1,4,6,4,1};
    constexpr uint8_t passes[]={24,12,6,4};
    const float shifts[]={OBSTACLE_LATER_LAP_SMOOTH_MAX_SHIFT_MM,
        20.0f,10.0f,6.0f};
    // Seat stations and the seam are immutable during this build. Compute
    // their protection weights once, not again in every relaxation pass.
    // The planner is non-reentrant and already owns static scratch arrays.
    // Avoid adding to its nested route-validation stack; fill every entry.
    static float protection[OBSTACLE_MAX_PATH_WAYPOINTS];
    for(uint16_t i=0;i<pathLength;++i)
    {
        const float seam=fminf(baselinePath[i].distanceMm,
            loopLengthMm-baselinePath[i].distanceMm);
        const float keep=OBSTACLE_LATER_LAP_SEAM_KEEP_MM+OBSTACLE_LATER_LAP_SEAM_BLEND_MM;
        float fade=clampFloat((seam-keep)/150.0f,0.0f,1.0f);
        fade=fade*fade*(3.0f-2.0f*fade);
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
        protection[i]=fade;
    }
    for(uint8_t trial=0;trial<4;++trial)
    {
        memcpy(route,rollback,sizeof(PathPoint)*pathLength);
        for(uint8_t pass=0;pass<passes[trial];++pass)
        {
            memcpy(smoothingBuffer,route,sizeof(PathPoint)*pathLength);
            for(uint16_t i=0;i<pathLength;++i)
            {
                const float fade=protection[i];
                if(fade<=0.0f)continue;
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
                const float relative=shift/shifts[trial];
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
            continue;
        }
        Serial.print("[LAPS] Smooth route length_before_after_mm=");
        Serial.print(oldLength,1);Serial.print('/');Serial.print(length,1);
        Serial.print(" bending_reduction_pct=");
        Serial.println(100.0f*(oldBending-bending)/fmaxf(oldBending,1e-9f),1);
        return;
    }
    memcpy(route,rollback,sizeof(PathPoint)*pathLength);
    Serial.println("[LAPS] Smoothing rejected; checked prior shape retained");
}

float laterLapCurvatureSpeed(float curvature)
{
    if (!isfinite(curvature)) return 0.0f;
    const float maximum = OBSTACLE_PATH_MAX_SPEED * OBSTACLE_LATER_LAP_SPEED_FACTOR;
    const float k = fabsf(curvature) * OBSTACLE_LATER_LAP_CURVATURE_TOLERANCE;
    return k < 1e-7f ? maximum : fminf(maximum,
        sqrtf(OBSTACLE_LATER_LAP_LATERAL_ACCEL_MMSS / k) /
            OBSTACLE_LATER_LAP_SPEED_TOLERANCE);
}

float laterLapInwardTrackingSpeed(float speed, const PositionEstimate &pose,
                                 const PathPoint *route, uint16_t index)
{
    if(!isfinite(speed)||speed<0||!isfinite(pose.x_mm)||!isfinite(pose.y_mm)||
       !isfinite(pose.heading_deg))return 0;
    const float tangent=connectorRouteHeading(route,index)*PI/180.0f;
    const float nx=-sinf(tangent),ny=cosf(tangent);
    const float crossTrack=nx*(pose.x_mm-route[index].x)+ny*(pose.y_mm-route[index].y);
    const float ordinaryMaximum=OBSTACLE_PATH_MAX_SPEED*OBSTACLE_LATER_LAP_SPEED_FACTOR;
    if(speed>ordinaryMaximum)
    {
        const float angle=fabsf(wrap180(pose.heading_deg-tangent*180.0f/PI));
        const float aligned=fminf(clampFloat((6.0f-angle)/3.0f,0.0f,1.0f),
            clampFloat((35.0f-fabsf(crossTrack))/20.0f,0.0f,1.0f));
        speed=ordinaryMaximum+aligned*(speed-ordinaryMaximum);
    }
    const float towardCentre=-nx*route[index].x-ny*route[index].y;
    if(fabsf(towardCentre)<1.0f)return speed;
    const float sign=towardCentre>0 ? 1.0f:-1.0f;
    const float h=pose.heading_deg*PI/180.0f;
    const float inwardHeading=sign*(nx*cosf(h)+ny*sinf(h));
    if(inwardHeading<=0)return speed;
    const float inwardError=sign*crossTrack;
    //617 diverged toward the inner wall while the same route worked in618.
    //Anticipate the measured heading error over the existing response budget,
    //rather than maintaining full speed until the correction is too tight.
    const float preview=inwardError+speed*OBSTACLE_LATER_LAP_RESPONSE_S*inwardHeading;
    if(preview<=40.0f)return speed;
    return fminf(speed,fmaxf(180.0f,speed/(1.0f+(preview-40.0f)/60.0f)));
}

float laterLapTrackingSpeed(float speed, float requestedCurvature, int steering)
{
    if (!isfinite(speed) || speed < 0 || !isfinite(requestedCurvature)) return 0;
    // Include the quantized CAD command as well as the pursuit request.
    const float actualCurvature=abs(steering)<2 ? 0.0f :
        1.0f/Ackermann::getTurnRadius(steering);
    const float curvature=fmaxf(fabsf(requestedCurvature),fabsf(actualCurvature));
    const float ordinary=OBSTACLE_PATH_MAX_SPEED*OBSTACLE_LATER_LAP_SPEED_FACTOR;
    if(speed>ordinary && curvature<OBSTACLE_LATER_LAP_STRAIGHT_CURVATURE_PER_MM)
    {
        const float straightness=clampFloat(2.0f*(1.0f-
            curvature/OBSTACLE_LATER_LAP_STRAIGHT_CURVATURE_PER_MM),0.0f,1.0f);
        return fminf(speed,ordinary+straightness*(OBSTACLE_LATER_LAP_STRAIGHT_SPEED_MM_S-ordinary));
    }
    return fminf(speed,laterLapCurvatureSpeed(curvature));
}

void planLaterLapSpeeds(const PathPoint *route, bool physicalLimitsOnly=false,
                       bool report=true)
{
    float curveLimits[OBSTACLE_MAX_PATH_WAYPOINTS];
    bool clearStraight[OBSTACLE_MAX_PATH_WAYPOINTS]={};
    const float maximum = OBSTACLE_PATH_MAX_SPEED * OBSTACLE_LATER_LAP_SPEED_FACTOR;
    float maximumSegment = OBSTACLE_PATH_SAMPLE_MM;
    for (uint16_t i=0;i<pathLength;++i)
    {
        const uint16_t next=(i+1)%pathLength;
        maximumSegment=fmaxf(maximumSegment,hypotf(route[next].x-route[i].x,
            route[next].y-route[i].y));
        uint16_t p=i,n=i;
        float beforeDistance=0,afterDistance=0;
        for(uint16_t step=1;step<pathLength;++step)
        {
            const uint16_t old=p;
            p=(p+pathLength-1)%pathLength;
            beforeDistance+=hypotf(route[old].x-route[p].x,route[old].y-route[p].y);
            if(beforeDistance>=OBSTACLE_LATER_LAP_CURVATURE_WINDOW_MM)break;
        }
        for(uint16_t step=1;step<pathLength;++step)
        {
            const uint16_t old=n;
            n=(n+1)%pathLength;
            afterDistance+=hypotf(route[n].x-route[old].x,route[n].y-route[old].y);
            if(afterDistance>=OBSTACLE_LATER_LAP_CURVATURE_WINDOW_MM)break;
        }
        // Circumcircle curvature over a pursuit-scale window. A polyline
        // vertex is not an instantaneous physical steering angle. Runtime
        // still caps the actual pursuit/CAD curvature on every control update.
        const float a=hypotf(route[i].x-route[p].x,route[i].y-route[p].y);
        const float b=hypotf(route[n].x-route[i].x,route[n].y-route[i].y);
        const float c=hypotf(route[n].x-route[p].x,route[n].y-route[p].y);
        const float cross=(route[i].x-route[p].x)*(route[n].y-route[i].y)-
            (route[i].y-route[p].y)*(route[n].x-route[i].x);
        const float k=2*cross/fmaxf(1,a*b*c);
        // The lap-1 pointwise heuristic is intentionally cautious while
        // discovering. Known laps use their physical curvature/braking plan
        // instead of carrying that second, unrelated slow-down into lap2/3.
        curveLimits[i]=physicalLimitsOnly ? laterLapCurvatureSpeed(k) :
            fminf(ceilf(route[i].speedMmS*OBSTACLE_LATER_LAP_SPEED_FACTOR),
                  laterLapCurvatureSpeed(k));
        const float phase=baselinePath[i].distanceMm;
        if(physicalLimitsOnly && phase>=OBSTACLE_LATER_LAP_HANDOFF_MM &&
           phase<corners[3].pathStartMm &&
           fabsf(k)<=OBSTACLE_LATER_LAP_STRAIGHT_CURVATURE_PER_MM &&
           beforeDistance+afterDistance-c<=2.0f)
        {
            clearStraight[i]=true;
            constexpr float radius=OBSTACLE_LATER_LAP_STRAIGHT_PILLAR_DISTANCE_MM;
            for(const auto &seat:seats)
            {
                if(seat.confirmed && distanceSquared(route[i].x,route[i].y,seat.x,seat.y)<radius*radius)
                { clearStraight[i]=false;break; }
            }
        }
    }
    // Include a full actual waypoint interval; displacement can stretch the
    // baseline's nominal 50 mm spacing. Never use baseline station distances.
    const float responseDistance = maximum * OBSTACLE_LATER_LAP_SPEED_TOLERANCE *
        OBSTACLE_LATER_LAP_RESPONSE_S + maximumSegment;
    const float straightResponseDistance =
        OBSTACLE_LATER_LAP_STRAIGHT_SPEED_MM_S*OBSTACLE_LATER_LAP_SPEED_TOLERANCE*
        OBSTACLE_LATER_LAP_RESPONSE_S+maximumSegment;
    // Every upcoming turn constrains the entry speed. This includes braking
    // across the cyclic seam and accounts for the finite motor response.
    for(uint16_t i=0;i<pathLength;++i)
    {
        float distance=0,limit=curveLimits[i];
        float straightLimit=clearStraight[i] ? OBSTACLE_LATER_LAP_STRAIGHT_SPEED_MM_S:0;
        for(uint16_t step=1;step<pathLength;++step)
        {
            const uint16_t p=(i+step-1)%pathLength,n=(i+step)%pathLength;
            distance+=hypotf(route[n].x-route[p].x,route[n].y-route[p].y);
            const float usable=fmaxf(0,distance-responseDistance);
            // All later distances are greater and all squared curve speeds
            // nonnegative. Beyond this point no future curve can lower limit.
            const float braking=2*OBSTACLE_LATER_LAP_PLAN_DECEL_MMSS*usable;
            if(braking<limit*limit)
                limit=fminf(limit,sqrtf(curveLimits[n]*curveLimits[n]+braking));
            if(straightLimit>maximum && !clearStraight[n])
            {
                const float straightBraking=2*OBSTACLE_LATER_LAP_PLAN_DECEL_MMSS*
                    fmaxf(0,distance-straightResponseDistance);
                straightLimit=fminf(straightLimit,sqrtf(
                    curveLimits[n]*curveLimits[n]+straightBraking));
            }
            if(braking>=limit*limit &&
               (straightLimit<=maximum || 2*OBSTACLE_LATER_LAP_PLAN_DECEL_MMSS*
                (distance-straightResponseDistance)>=straightLimit*straightLimit))break;
        }
        laterLapSpeedLimits[i]=straightLimit>maximum ? straightLimit:limit;
    }
    // Two cyclic forward sweeps limit planned acceleration after a slow turn.
    for(uint8_t pass=0;pass<2;++pass)
        for(uint16_t i=0;i<pathLength;++i)
        {
            const uint16_t n=(i+1)%pathLength;
            const float ds=hypotf(route[n].x-route[i].x,route[n].y-route[i].y);
            laterLapSpeedLimits[n]=fminf(laterLapSpeedLimits[n],sqrtf(
                laterLapSpeedLimits[i]*laterLapSpeedLimits[i]+
                2*OBSTACLE_LATER_LAP_PLAN_ACCEL_MMSS*ds));
        }
    if (!report) return;
    Serial.print("[LAPS] Dynamics max/lateral/decel/response=");
    Serial.print(maximum,0);Serial.print('/');
    Serial.print(OBSTACLE_LATER_LAP_LATERAL_ACCEL_MMSS,0);Serial.print('/');
    Serial.print(OBSTACLE_LATER_LAP_PLAN_DECEL_MMSS,0);Serial.print('/');
    Serial.println(OBSTACLE_LATER_LAP_RESPONSE_S,2);
    if(physicalLimitsOnly)
    {
        float plannedPeak=0;
        for(uint16_t i=0;i<pathLength;++i)plannedPeak=fmaxf(plannedPeak,laterLapSpeedLimits[i]);
        Serial.print("[LAPS] clear_straight_limit/peak_mm_s=");
        Serial.print(OBSTACLE_LATER_LAP_STRAIGHT_SPEED_MM_S,0);Serial.print('/');
        Serial.println(plannedPeak,0);
    }
}

PathPoint laterLapLookaheadTarget(const PathPoint *route, const PositionEstimate &pose,
                                 uint16_t index, float lookaheadMm)
{
    // Locate the axle on either segment incident to the nearest waypoint.
    // Then interpolate along the route instead of jumping whole waypoints.
    const uint16_t before=(index+pathLength-1)%pathLength;
    uint16_t start=index;
    float bestError=INFINITY, intoSegment=0.0f;
    for(uint16_t from:{before,index})
    {
        const auto &a=route[from];
        const auto &b=route[(from+1)%pathLength];
        const float dx=b.x-a.x,dy=b.y-a.y;
        const float lengthSquared=dx*dx+dy*dy;
        if(lengthSquared<1.0f)continue;
        const float t=clampFloat(((pose.x_mm-a.x)*dx+(pose.y_mm-a.y)*dy)/
            lengthSquared,0.0f,1.0f);
        const float ex=pose.x_mm-(a.x+t*dx),ey=pose.y_mm-(a.y+t*dy);
        const float error=ex*ex+ey*ey;
        if(error>=bestError)continue;
        bestError=error;start=from;intoSegment=t*sqrtf(lengthSquared);
    }
    // Retain the previous average horizon, which rounded up to a sample.
    float remaining=lookaheadMm+0.5f*OBSTACLE_PATH_SAMPLE_MM+intoSegment;
    for(uint16_t visited=0;visited<pathLength;++visited)
    {
        const uint16_t next=(start+1)%pathLength;
        const float dx=route[next].x-route[start].x,dy=route[next].y-route[start].y;
        const float length=hypotf(dx,dy);
        if(length>=1.0f && remaining<=length)
        {
            PathPoint target=route[next];
            const float t=remaining/length;
            target.x=route[start].x+t*dx;
            target.y=route[start].y+t*dy;
            return target;
        }
        remaining-=length;start=next;
    }
    return route[index];
}

// Run the same quantized pursuit controller around the candidate before using
// an optional shortcut. Wall geometry is sampled once per pose; other pillars
// use the identical capsule distance directly, avoiding repeated wall work.
bool laterLapTrackedRouteSafe(const PathPoint *route,uint32_t startedUs,uint32_t budgetUs,
                              uint16_t first = 0, uint16_t end = 0,
                              uint16_t sectionSteps = 360, float minimumClearance = 0.0f,
                              float minimumWall = 0.0f)
{
    const bool sectionOnly = end != 0;
    const bool sectionWrap = sectionOnly && end < first;
    const PathPoint *entry = sectionOnly ? route : livePath;
    const uint16_t maximumSteps = sectionOnly ? sectionSteps : 6000;
    const float wallFloor = fmaxf(40.0f, fmaxf(minimumClearance, minimumWall));
    const float pillarFloor = fmaxf(32.0f, minimumClearance);
    for(int perturbation=-1;perturbation<=1;++perturbation)
    {
        float h=connectorRouteHeading(entry,first)*PI/180.0f;
        const float offset=sectionOnly ? 20.0f : 10.0f;
        float x=entry[first].x-perturbation*offset*sinf(h);
        float y=entry[first].y+perturbation*offset*cosf(h);
        h+=perturbation*(sectionOnly ? 3.0f : 1.0f)*PI/180.0f;
        uint16_t index=first;
        float previousForward[OBSTACLE_SEAT_COUNT];
        for(float &value:previousForward)value=NAN;
        uint8_t wraps=0;
        bool completed=false;
        float travel=0,speed=sectionOnly ? laterLapSpeedLimits[first] : OBSTACLE_PATH_MAX_SPEED;
        float acceleration=0,limit=0;
        constexpr float dt=0.020f;
        for(uint16_t step=0;step<maximumSteps;++step)
        {
            if(budgetUs && uint32_t(micros()-startedUs)>=budgetUs)return false;
            const uint16_t previous=index;
            index=nearestPathIndex(route,x,y,index,OBSTACLE_PATH_PROGRESS_WINDOW);
            if (sectionWrap && previous > pathLength * 3 / 4 && index < pathLength / 4)
                ++wraps;
            if(sectionOnly && (!sectionWrap || wraps) &&
                index>=end && index<=end+OBSTACLE_PATH_PROGRESS_WINDOW)
            {completed=true;break;}
            if(!sectionOnly && previous>pathLength*3/4 && index<pathLength/4 &&
                travel>loopLengthMm*0.5f && ++wraps>=2){completed=true;break;}
            ObstacleClearanceSample reference{};
            if(!calculateClearanceAtPose(seats[0],x,y,h*180.0f/PI,reference) ||
                !reference.valid || reference.wallMm<wallFloor)return false;
            const float endX=x+OBSTACLE_ROBOT_ENVELOPE_AXIS_FRONT_MM*cosf(h);
            const float endY=y+OBSTACLE_ROBOT_ENVELOPE_AXIS_FRONT_MM*sinf(h);
            for(uint8_t seat=0;seat<OBSTACLE_SEAT_COUNT;++seat)
            {
                if(!seats[seat].confirmed)continue;
                const float gap=pointSegmentDistance(seats[seat].x,seats[seat].y,
                    x,y,endX,endY)-OBSTACLE_MAX_WHEEL_HALF_WIDTH_MM-
                    OBSTACLE_PILLAR_MOVEMENT_RADIUS_MM;
                if(!isfinite(gap) || gap<pillarFloor)return false;
                // Witness correct passing once the complete steered footprint
                // crosses the sign's transverse plane, not just the rear axle.
                if(distanceSquared(x,y,seats[seat].x,seats[seat].y)>400.0f*400.0f)continue;
                const float sh=seats[seat].headingDeg*PI/180.0f;
                const float shCos=cosf(sh), shSin=sinf(sh);
                if (sectionOnly && fabsf((x-seats[seat].x)*shCos+
                        (y-seats[seat].y)*shSin)>210.0f) continue;
                float minForward=INFINITY;
                float minSide=INFINITY,maxSide=-INFINITY;
                for(int wheel:{-1,1})
                {
                    parking_start_footprint::Quad bodies[6];
                    parking_start_footprint::robotQuads(x,y,h*180.0f/PI,wheel,0,bodies);
                    for(const auto &body:bodies)for(const auto &p:body.p)
                    {
                        const float dx=p.x-seats[seat].x,dy=p.y-seats[seat].y;
                        const float forward=dx*shCos+dy*shSin;
                        const float side=-dx*shSin+dy*shCos;
                        minForward=fminf(minForward,forward);
                        minSide=fminf(minSide,side);maxSide=fmaxf(maxSide,side);
                    }
                }
                if(isfinite(previousForward[seat]) && previousForward[seat]<=0 && minForward>0 &&
                    (seats[seat].red ? maxSide>=-25.0f : minSide<=25.0f))return false;
                previousForward[seat]=minForward;
            }
            if(y<-1100.0f &&
                (!parking_start_footprint::safe(x,y,h*180.0f/PI,-1,false,20.0f) ||
                 !parking_start_footprint::safe(x,y,h*180.0f/PI,1,false,20.0f)))return false;
            float lookahead=OBSTACLE_LATER_LAP_LOOKAHEAD_MM;
            if(nearCorner(baselinePath[index].distanceMm))lookahead*=OBSTACLE_LOOKAHEAD_CORNER_SCALE;
            const auto target=laterLapLookaheadTarget(route,{x,y,0.0f},index,lookahead);
            const float dx=target.x-x,dy=target.y-y;
            const float forward=dx*cosf(h)+dy*sinf(h);
            const float curvature=2.0f*(-dx*sinf(h)+dy*cosf(h))/fmaxf(1.0f,dx*dx+dy*dy);
            const float requested=pursuitServoForCurvature(curvature);
            if(!isfinite(requested) || forward<=1.0f)return false;
            const float steering=clampFloat(requested,-OBSTACLE_MAX_PURSUIT_STEERING_DEG,
                OBSTACLE_MAX_PURSUIT_STEERING_DEG);
            const float k=pursuitCurvatureForServo(static_cast<int>(steering));
            const int command=static_cast<int>(laterLapTrackingSpeed(
                laterLapSpeedLimits[index],curvature,static_cast<int>(steering)));
            // Same finite jerk/release/braking profile as drive_loop. A new
            // lower target is not an instantaneous lower physical speed.
            float stepMm=DriveProfile::advance(speed,acceleration,limit,command,dt,DEFAULT_ACCELERATION);
            if(perturbation!=0)stepMm*=OBSTACLE_LATER_LAP_SPEED_TOLERANCE;
            const float actualSpeed=stepMm/dt;
            if(actualSpeed*actualSpeed*fabsf(k)>OBSTACLE_LATER_LAP_LATERAL_ACCEL_MMSS)return false;
            travel+=stepMm;
            if(travel>loopLengthMm*2.5f)return false;
            const float nextH=h+k*stepMm;
            if(fabsf(k)>1e-6f){x+=(sinf(nextH)-sinf(h))/k;y+=(cosf(h)-cosf(nextH))/k;}
            else{x+=stepMm*cosf(h);y+=stepMm*sinf(h);}
            h=nextH;
        }
        if(!completed)return false;
    }
    return true;
}

float laterLapTargetTime(PathPoint *route, bool physicalLimitsOnly = false)
{
    if (!physicalLimitsOnly) recomputeSpeedProfile(route);
    planLaterLapSpeeds(route, physicalLimitsOnly, !physicalLimitsOnly);
    float seconds=0;
    for(uint16_t i=0;i<pathLength;++i)
    {
        const uint16_t n=(i+1)%pathLength;
        seconds+=2.0f*hypotf(route[n].x-route[i].x,route[n].y-route[i].y)/
            fmaxf(1.0f,laterLapSpeedLimits[i]+laterLapSpeedLimits[n]);
    }
    return seconds;
}

// A lateral elastic line on the recorded route: retain the complete passing
// plateaus, then connect them through free space towards the inner corridor.
// There is no colour-pair restriction and no forced return to the centre.
void shortenKnownInnerRoute(PathPoint *route,PathPoint *rollback)
{
    const uint32_t startedUs=micros();
    const uint32_t budgetUs=OBSTACLE_LATER_LAP_INNER_PLAN_BUDGET_US;
    memcpy(rollback,route,sizeof(PathPoint)*pathLength);
    float oldLength,oldPeak,oldWall,oldPillar,oldTurn;
    const float oldBending=laterLapBending(route,oldLength,oldPeak,&oldTurn);
    if(!laterLapRouteSafe(route,&oldWall,&oldPillar,startedUs,budgetUs))return;
    const float oldTime=laterLapTargetTime(route);
    float bestTime=oldTime,bestLength=oldLength;
    // Scratch storage is static: do not grow the main thread's stack on top
    // of buildOptimizedPath's existing full-route rollback array.
    static PathPoint best[OBSTACLE_MAX_PATH_WAYPOINTS];
    struct Node {float nx,ny,fade,minimum,q,next;};
    static Node nodes[OBSTACLE_MAX_PATH_WAYPOINTS];
    memcpy(best,route,sizeof(PathPoint)*pathLength);
    // Restore the tighter turn budget used before the direct connectors.
    constexpr float maximumExtraTurn=0.01f;
    bool innerPass[OBSTACLE_SEAT_COUNT]={};
    for(uint8_t seat=0;seat<OBSTACLE_SEAT_COUNT;++seat)
        if(seats[seat].confirmed)innerPass[seat]=routeTurnSign*
            targetLateralForSeat(seats[seat],optimizedClearanceForSeat(seat))>0;
    for(uint16_t i=0;i<pathLength;++i)
    {
        const float h=baselinePath[i].headingDeg*PI/180.0f;
        Node &node=nodes[i];node.nx=-sinf(h);node.ny=cosf(h);node.minimum=-INFINITY;
        const float seam=fminf(baselinePath[i].distanceMm,loopLengthMm-baselinePath[i].distanceMm);
        float fade=clampFloat((seam-400.0f)/200.0f,0.0f,1.0f);
        for(uint8_t seat=0;seat<OBSTACLE_SEAT_COUNT;++seat)
        {
            if(!seats[seat].confirmed)continue;
            const float ahead=cyclicDistanceForward(seats[seat].pathDistanceMm,baselinePath[i].distanceMm);
            const float distance=fminf(ahead,loopLengthMm-ahead);
            if(innerPass[seat])
            {
                if(distance<=150.0f)node.minimum=routeTurnSign*
                    ((rollback[i].x-baselinePath[i].x)*node.nx+(rollback[i].y-baselinePath[i].y)*node.ny);
            }
            else fade=fminf(fade,clampFloat((distance-150.0f)/200.0f,0.0f,1.0f));
        }
        node.fade=fade*fade*(3.0f-2.0f*fade);
    }
    constexpr float innerTargets[]={300.0f,240.0f,180.0f,120.0f,0.0f};
    constexpr float weights[]={1,4,6,4,1};
    for(float inner:innerTargets)
    {
        if(uint32_t(micros()-startedUs)>=budgetUs)break;
        memcpy(route,rollback,sizeof(PathPoint)*pathLength);
        joinSameColourStraightEnds(route,false,true);
        roundKnownCornerPairs(route,false,true);
        carryKnownInnerLaneToMiddle(route,true);
        preserveLaterLapSeam(route);
        for(uint16_t i=0;i<pathLength;++i)nodes[i].q=
            (route[i].x-baselinePath[i].x)*nodes[i].nx+(route[i].y-baselinePath[i].y)*nodes[i].ny;
        for(uint16_t pass=0;inner>0 && pass<240;++pass)
        {
            if(uint32_t(micros()-startedUs)>=budgetUs)break;
            for(uint16_t i=0;i<pathLength;++i)
            {
                Node &node=nodes[i];
                float average=0;
                for(int k=-2;k<=2;++k)
                {
                    const uint16_t j=(i+pathLength+k)%pathLength;
                    average+=weights[k+2]*nodes[j].q/16.0f;
                }
                node.next=node.q+node.fade*(0.8f*(average-node.q)+0.0015f*(routeTurnSign*inner-node.q));
                if(node.fade>0)node.next=routeTurnSign*fmaxf(routeTurnSign*node.next,node.minimum);
            }
            for(uint16_t i=0;i<pathLength;++i)nodes[i].q=nodes[i].next;
        }
        if(uint32_t(micros()-startedUs)>=budgetUs)break;
        if(inner>0)for(uint16_t i=0;i<pathLength;++i)
        {
            const Node &node=nodes[i];
            const float initial=(route[i].x-baselinePath[i].x)*node.nx+(route[i].y-baselinePath[i].y)*node.ny;
            route[i].x+=node.nx*(node.q-initial);route[i].y+=node.ny*(node.q-initial);
        }
        float length,peak,wall,pillar,turn;
        const float bending=laterLapBending(route,length,peak,&turn);
        if(length>=oldLength-10.0f || peak>fmaxf(oldPeak,1.0f/250.0f)+1e-7f ||
            turn>oldTurn+maximumExtraTurn || bending>fmaxf(oldBending*1.30f,2.0f*PI/330.0f) ||
            !laterLapRouteSafe(route,&wall,&pillar,startedUs,budgetUs) ||
            wall<fmaxf(60.0f,fminf(oldWall,100.0f)) ||
            pillar<fmaxf(40.0f,fminf(oldPillar,70.0f)))continue;
        const float seconds=laterLapTargetTime(route);
        if(seconds>bestTime-0.01f)continue;
        if(!laterLapTrackedRouteSafe(route,startedUs,budgetUs))continue;
        bestTime=seconds;bestLength=length;
        memcpy(best,route,sizeof(PathPoint)*pathLength);
    }
    memcpy(route,best,sizeof(PathPoint)*pathLength);
    if(bestTime<oldTime)
    {
        Serial.print("[LAPS] Inner route length_before_after_mm=");
        Serial.print(oldLength,1);Serial.print('/');Serial.print(bestLength,1);
        Serial.print(" target_s_before_after=");Serial.print(oldTime,3);
        Serial.print('/');Serial.print(bestTime,3);
        Serial.print(" planning_us=");Serial.println(uint32_t(micros()-startedUs));
    }
    else
    {
        Serial.print("[LAPS] Inner search retained checked shape planning_us=");
        Serial.println(uint32_t(micros()-startedUs));
    }
}

void addLaterLapStartMiddleReserve(PathPoint *route,PathPoint *backup)
{
    if(sectionLayoutMode!=OBSTACLE_SECTION_LAYOUT_OFFICIAL)return;
    const int middle=seats[2].confirmed ? 2:seats[3].confirmed ? 3:-1;
    if(middle<0)return;
    float oldWall,oldPillar;
    if(!laterLapRouteSafe(route,&oldWall,&oldPillar))return;
    memcpy(backup,route,sizeof(PathPoint)*pathLength);
    const float h=seats[middle].headingDeg*PI/180.0f;
    const float side=seats[middle].red ? -1.0f:1.0f;
    for(uint16_t i=0;i<pathLength;++i)
    {
        float d=cyclicDistanceForward(seats[middle].pathDistanceMm,baselinePath[i].distanceMm);
        d=fminf(d,loopLengthMm-d);
        if(d>=400.0f)continue;
        const float blend=sinf(PI*d/400.0f);
        const float extra=OBSTACLE_LATER_LAP_START_MIDDLE_EXTRA_MM*blend*blend*side;
        route[i].x-=sinf(h)*extra;route[i].y+=cosf(h)*extra;
    }
    float wall,pillar;
    if(!laterLapRouteSafe(route,&wall,&pillar) ||
       wall<fmaxf(60.0f,oldWall-20.0f) || pillar<oldPillar-1.0f)
    {
        memcpy(route,backup,sizeof(PathPoint)*pathLength);
        Serial.println("[LAPS] Start-middle reserve rejected; checked shape retained");
    }
    else Serial.println("[LAPS] Smooth start-middle passing reserve applied");
}

int solitaryMiddleSeat(uint8_t section)
{
    const int first = 6*section;
    int middle = -1;
    for (int offset=0;offset<6;++offset)
    {
        if (!seats[first+offset].confirmed) continue;
        if ((offset!=2 && offset!=3) || middle>=0) return -1;
        middle = first+offset;
    }
    return middle;
}

bool isCwSolitaryOuterRed(int seat)
{
    return routeTurnSign < 0 && seat >= 6 && seats[seat].red &&
        seats[seat].lateralMm > 0.0f && solitaryMiddleSeat(seat / 6) == seat;
}

void refineCcwSolitaryPillars(PathPoint *route, PathPoint *rollback)
{
    if (routeTurnSign <= 0) return;
    int middle[4] = {-1,-1,-1,-1};
    bool hasRed = false;
    for (uint8_t section=1;section<4;++section)
    {
        middle[section] = solitaryMiddleSeat(section);
        hasRed |= middle[section]>=0 && seats[middle[section]].red;
    }
    if (!hasRed) return;

    const uint32_t started = micros();
    const uint32_t budget = OBSTACLE_LATER_LAP_INNER_PLAN_BUDGET_US;
    memcpy(rollback,route,sizeof(PathPoint)*pathLength);
    float oldLength,oldPeak,oldTurn;
    laterLapBending(route,oldLength,oldPeak,&oldTurn);
    const float oldTime = laterLapTargetTime(route);
    for (float handleScale:{0.56f,0.72f,0.0f})
    {
        if (uint32_t(micros()-started)>=budget) break;
        memcpy(route,rollback,sizeof(PathPoint)*pathLength);
        for (uint8_t section=1;section<4;++section)
        {
            const int red = middle[section], green = middle[section-1];
            if (red<0 || !seats[red].red) continue;
            const float pass = seats[red].pathDistanceMm;
            constexpr float taper = 425.0f, hold = 105.0f;
            for (uint16_t i=0;i<pathLength;++i)
            {
                const float d = baselinePath[i].distanceMm;
                if (fabsf(d-pass)>=taper) continue;
                const float heading = baselinePath[i].headingDeg*PI/180.0f;
                const float nx=-sinf(heading), ny=cosf(heading);
                const float offset = 30.0f*smoothLaneBlend(d,pass-taper,pass-hold)*
                    (1.0f-smoothLaneBlend(d,pass+hold,pass+taper));
                route[i].x += nx*offset;
                route[i].y += ny*offset;
            }
            if (handleScale==0.0f || green<0 || seats[green].red) continue;

            // Keep the exit tangent of GREEN's left pass and the entry
            // tangent of RED's right pass. One cubic connects those
            // straights around the corner without two independent tapers.
            uint16_t first=0,last=0;
            const float greenExit = seats[green].pathDistanceMm+100.0f;
            const float redEntry = pass-100.0f;
            while (first+1<pathLength && baselinePath[first+1].distanceMm<=greenExit+0.1f) ++first;
            while (last+1<pathLength && baselinePath[last].distanceMm<redEntry-0.1f) ++last;
            if (last<=first) continue;
            const PathPoint &a = route[first], &b = route[last];
            const float h0 = baselinePath[first].headingDeg*PI/180.0f;
            const float h1 = baselinePath[last].headingDeg*PI/180.0f;
            const float fx=cosf(h0),fy=sinf(h0),tx=cosf(h1),ty=sinf(h1);
            const float dx=b.x-a.x,dy=b.y-a.y;
            const float lead = handleScale*(dx*fx+dy*fy);
            const float tail = handleScale*(dx*tx+dy*ty);
            const float span=baselinePath[last].distanceMm-baselinePath[first].distanceMm;
            for (uint16_t i=first+1;i<last;++i)
            {
                const float t=(baselinePath[i].distanceMm-baselinePath[first].distanceMm)/span;
                const float u=1.0f-t;
                const float blend=t*t*(3.0f-2.0f*t);
                route[i].x=a.x+blend*dx+3.0f*u*t*(u*lead*fx-t*tail*tx);
                route[i].y=a.y+blend*dy+3.0f*u*t*(u*lead*fy-t*tail*ty);
            }
        }
        float length,peak,turn,wall,pillar;
        laterLapBending(route,length,peak,&turn);
        if (length>oldLength || peak>oldPeak+1e-6f || turn>oldTurn+0.04f ||
            !laterLapRouteSafe(route,&wall,&pillar,started,budget) ||
            wall<50.0f || pillar<50.0f || laterLapTargetTime(route)>oldTime+0.05f ||
            !laterLapTrackedRouteSafe(route,started,budget)) continue;
        Serial.print("[LAPS] Solitary pillars single_corner=");
        Serial.print(handleScale>0.0f ? 1:0);
        Serial.print(" length_saved_mm=");
        Serial.println(oldLength-length,1);
        return;
    }
    memcpy(route,rollback,sizeof(PathPoint)*pathLength);
    Serial.println("[LAPS] Solitary pillars kept prior checked route");
}

void refineCwSolitaryRed(PathPoint *route, PathPoint *rollback)
{
    if (routeTurnSign >= 0) return;
    for (uint8_t section=1; section<4; ++section)
    {
        const int red = solitaryMiddleSeat(section);
        // The inner-row RED already has its own pass next to the inner wall.
        if (!isCwSolitaryOuterRed(red)) continue;
        const float pass = seats[red].pathDistanceMm;
        uint16_t first = 0, end = 0;
        while (first+1 < pathLength && baselinePath[first+1].distanceMm < pass-1000.0f) ++first;
        while (end+1 < pathLength && baselinePath[end].distanceMm < pass+1050.0f) ++end;
        memcpy(rollback, route, sizeof(PathPoint)*pathLength);
        bool accepted = false, sweptClear = false;
        for (float clearance : {345.0f, 315.0f, 285.0f})
        {
            const float lane = targetLateralForSeat(seats[red], clearance);
            for (uint16_t i=first; i<=end; ++i)
            {
                const float d = baselinePath[i].distanceMm - pass;
                const float blend = smoothLaneBlend(d, -830.0f, -365.0f) *
                    (1.0f-smoothLaneBlend(d, 365.0f, 830.0f));
                if (blend <= 0.0f) continue;
                const float heading = baselinePath[i].headingDeg*PI/180.0f;
                const float nx = -sinf(heading), ny = cosf(heading);
                const float previous = (rollback[i].x-baselinePath[i].x)*nx +
                    (rollback[i].y-baselinePath[i].y)*ny;
                const float shift = blend*(lane-previous);
                route[i].x = rollback[i].x + nx*shift;
                route[i].y = rollback[i].y + ny*shift;
            }
            float wall, pillar;
            if (laterLapRouteSafe(route,&wall,&pillar,0,0,first,end+1) &&
                wall >= 50.0f && pillar >= 50.0f)
            {
                sweptClear = true;
                planLaterLapSpeeds(route,true);
                // The local rollout has a fixed step bound. An open corridor
                // can finish validation even when the rest of lap planning
                // already used its optional time budgets.
                if (laterLapTrackedRouteSafe(route,0,0,first,end))
                {
                    Serial.print("[LAPS] CW solitary RED inside seat/offset_mm=");
                    Serial.print(red); Serial.print('/'); Serial.println(lane,0);
                    accepted = true;
                    break;
                }
            }
            memcpy(route+first,rollback+first,sizeof(PathPoint)*(end-first+1));
        }
        if (!accepted)
        {
            Serial.print("[LAPS] CW solitary RED retained checked route seat/swept_clear=");
            Serial.print(red); Serial.print('/');
            Serial.println(sweptClear ? 1 : 0);
        }
    }
}

// Connect consecutive passes using their actual entry and exit tangents.
// Collinear controls give a straight. In a corner the first and last three
// controls are collinear, so the new bend starts and ends with zero curvature.
bool connectLaterLapPasses(PathPoint *route, const PathPoint *original,
                           uint16_t first, uint16_t last, float handleScale = 0.38f)
{
    const PathPoint &a = original[first], &b = original[last];
    const float dx = b.x - a.x, dy = b.y - a.y;
    const float distance = hypotf(dx, dy);
    const float entry = connectorRouteHeading(original, first - 1) * PI / 180.0f;
    const float exit = connectorRouteHeading(original, last) * PI / 180.0f;
    const float fx = cosf(entry), fy = sinf(entry), tx = cosf(exit), ty = sinf(exit);
    const float alongEntry = dx * fx + dy * fy;
    const float alongExit = dx * tx + dy * ty;
    if (distance < 250.0f || alongEntry < 0.2f * distance || alongExit < 0.2f * distance)
        return false;

    const float handleA = handleScale * alongEntry;
    const float handleB = handleScale * alongExit;
    const auto pointAt = [&](float t, float &x, float &y)
    {
        const float u = 1.0f - t;
        const float fromA = 5.0f * t * u * u * u * (u + 4.0f * t) * handleA;
        const float fromB = 5.0f * u * t * t * t * (t + 4.0f * u) * handleB;
        const float blend = t * t * t * (10.0f + t * (-15.0f + 6.0f * t));
        x = a.x + blend * dx + fromA * fx - fromB * tx;
        y = a.y + blend * dy + fromA * fy - fromB * ty;
    };
    float previousT = 0.0f, previousX = a.x, previousY = a.y;
    for (uint16_t i = first + 1; i < last; ++i)
    {
        // Keep each sample in its original cross-section. Lap progress,
        // pillar passage and corner lookahead all use the baseline indices.
        const auto &station = baselinePath[i];
        const float heading = station.headingDeg * PI / 180.0f;
        const float hx = cosf(heading), hy = sinf(heading);
        const auto forward = [&](float x, float y)
        {
            return (x - station.x) * hx + (y - station.y) * hy;
        };
        if (forward(previousX, previousY) >= 0.0f || forward(b.x, b.y) <= 0.0f)
            return false;
        float low = previousT, high = 1.0f;
        for (uint8_t step = 0; step < 13; ++step)
        {
            const float t = 0.5f * (low + high);
            float x, y;
            pointAt(t, x, y);
            if (forward(x, y) < 0.0f) low = t;
            else high = t;
        }
        previousT = 0.5f * (low + high);
        pointAt(previousT, previousX, previousY);
        const float error = forward(previousX, previousY);
        route[i].x = previousX - error * hx;
        route[i].y = previousY - error * hy;
    }
    return true;
}

bool tryLaterLapPassConnection(PathPoint *route, PathPoint *original,
                               uint16_t first, uint16_t last, float handleScale,
                               uint32_t started, uint32_t budget, uint16_t trackingSteps,
                               float &checkedTime, float &savedTurn,
                               uint8_t trackingTailPoints = 5, bool allowWiderBend = false)
{
    const float wallReserve = allowWiderBend ? 55.0f : 50.0f;
    if (connectLaterLapPasses(route, original, first, last, handleScale))
    {
        float oldLength, oldPeak, oldTurn, length, peak, turn, wall, pillar;
        const float oldBending = laterLapBending(original, oldLength, oldPeak, &oldTurn,
            first, last + 1);
        const float bending = laterLapBending(route, length, peak, &turn, first, last + 1);
        const bool reducedTurn = turn < oldTurn - 0.025f;
        const bool widerBend = allowWiderBend && turn <= oldTurn + 0.025f &&
            peak <= 0.95f * oldPeak;
        if (bending < (allowWiderBend ? 0.97f : 0.9f) * oldBending &&
            (reducedTurn || widerBend) &&
            length <= 1.1f * oldLength && peak <= oldPeak + 1e-6f &&
            laterLapRouteSafe(route, &wall, &pillar, started, budget, first - 1, last + 2) &&
            wall >= wallReserve && pillar >= 50.0f)
        {
            if (checkedTime < 0.0f) checkedTime = laterLapTargetTime(original, true);
            const float candidateTime = laterLapTargetTime(route, true);
            const uint16_t trackingStart = first > 4 ? first - 4 : 0;
            const uint16_t tail = last + trackingTailPoints;
            uint16_t trackingEnd = tail < pathLength - 1 ? tail : pathLength - 2;
            const float tailToSeam = loopLengthMm - baselinePath[last].distanceMm;
            if (allowWiderBend && ((tailToSeam < 500.0f &&
                (seats[2].confirmed || seats[3].confirmed)) ||
                tailToSeam < OBSTACLE_LATER_LAP_SEAM_KEEP_MM - 1.0f))
            {
                // A changed entry tangent can affect the vehicle beyond the
                // seam even when the last overwritten sample is clear.
                trackingEnd = 1;
                while (baselinePath[trackingEnd].distanceMm < 250.0f) ++trackingEnd;
            }
            // Require an actual time gain for a slightly longer curve.
            const float timeMargin = length > oldLength + 1.0f ? -0.01f : 0.0f;
            if (candidateTime <= checkedTime + timeMargin &&
                laterLapTrackedRouteSafe(route, started, budget, trackingStart,
                    trackingEnd, trackingSteps, 50.0f, wallReserve))
            {
                checkedTime = candidateTime;
                savedTurn = oldTurn - turn;
                return true;
            }
        }
    }
    memcpy(route + first, original + first, sizeof(PathPoint) * (last - first + 1));
    return false;
}

void refineLaterLapPassConnections(PathPoint *route, PathPoint *rollback)
{
    // Seat order follows travel in both directions. Leave the final corner
    // used by parking and the shared start/finish geometry outside this pass.
    const uint32_t started = micros();
    const uint32_t budget = OBSTACLE_LATER_LAP_INNER_PLAN_BUDGET_US;
    const float protectedStart = OBSTACLE_LATER_LAP_SEAM_KEEP_MM +
        OBSTACLE_LATER_LAP_SEAM_BLEND_MM;
    int previous = -1;
    float checkedTime = -1.0f;
    for (uint8_t next = 0; next < OBSTACLE_SEAT_COUNT; ++next)
    {
        if (!seats[next].confirmed) continue;
        const int from = previous;
        previous = next;
        if (from < 0 || next / 6 - from / 6 > 1) continue;
        const bool changesSection = next / 6 != from / 6;
        const float start = seats[from].pathDistanceMm;
        const float finish = seats[next].pathDistanceMm;
        if (finish <= start || finish - start > 2200.0f) continue;
        const bool keepEntryStraight = isCwSolitaryOuterRed(from);
        const bool keepExitStraight = isCwSolitaryOuterRed(next);

        memcpy(rollback, route, sizeof(PathPoint) * pathLength);
        // For a sign at the end of a straight, also try joining from its
        // passing lane before the old taper starts to steer into the corner.
        constexpr float trims[][2] = {
            {-150.0f, 0.0f}, {-100.0f, 100.0f}, {0.0f, -100.0f},
            {150.0f, 150.0f}, {100.0f, 100.0f}, {200.0f, 200.0f},
        };
        for (const auto &trim : trims)
        {
            if (uint32_t(micros() - started) >= budget) return;
            if (!changesSection && (trim[0] <= 0.0f || trim[1] <= 0.0f)) continue;
            // Keep the established straight through an isolated outer CW RED.
            const float startTrim = keepEntryStraight ? 365.0f : trim[0];
            const float finishTrim = keepExitStraight ? 365.0f : trim[1];
            uint16_t first = 1, last = 1;
            while (first + 1 < pathLength && baselinePath[first].distanceMm < start + startTrim)
                ++first;
            while (last + 2 < pathLength && baselinePath[last + 1].distanceMm <= finish - finishTrim)
                ++last;
            if (last <= first + 3 || baselinePath[first - 1].distanceMm < protectedStart ||
                baselinePath[last + 1].distanceMm >= corners[3].pathStartMm)
                continue;
            float savedTurn;
            if (tryLaterLapPassConnection(route, rollback, first, last, 0.38f,
                    started, budget, 480, checkedTime, savedTurn))
            {
                Serial.print("[LAPS] Smooth pass connection from/to/turn_saved_deg=");
                Serial.print(from); Serial.print('/'); Serial.print(next); Serial.print('/');
                Serial.println(savedTurn * 180.0f / PI, 1);
                break;
            }
        }
    }
}

void refineCcwParallelStart(PathPoint *route, PathPoint *rollback,
                            int previousSign, int green, int nextSign)
{
    const uint32_t started = micros();
    // This local rollout spans both corners and the start crossing.
    constexpr uint32_t budget = 45000UL;
    const auto waypoint = [](float distance)
    {
        uint16_t index = 0;
        while (index + 1 < pathLength && baselinePath[index + 1].distanceMm <= distance + 0.1f)
            ++index;
        return index;
    };
    const float greenPass = seats[green].pathDistanceMm;
    const float exitLane = targetLateralForSeat(seats[nextSign], optimizedClearanceForSeat(nextSign));
    const bool innerExit = exitLane > 0.0f;
    const uint16_t first = waypoint(seats[previousSign].pathDistanceMm + (innerExit ? 0.0f : 50.0f));
    const uint16_t straightIn = waypoint(loopLengthMm + greenPass - (innerExit ? 200.0f : 250.0f));
    const uint16_t straightOut = waypoint(greenPass + (innerExit ? 250.0f : 200.0f));
    const uint16_t last = waypoint(seats[nextSign].pathDistanceMm + (innerExit ? 100.0f : 150.0f));
    if (first < 5 || straightIn <= first + 4 || last <= straightOut + 4 ||
        last + 5 >= first || straightOut < 2)
        return;

    float length, peak, turn;
    const float oldBending = laterLapBending(route, length, peak, &turn);
    const float oldTime = laterLapTargetTime(route, true);
    const float heading = baselinePath[0].headingDeg * PI / 180.0f;
    const float nx = -sinf(heading), ny = cosf(heading);
    memcpy(rollback, route, sizeof(PathPoint) * pathLength);
    // Use one lateral offset on both sides of the duplicated lap waypoint.
    // The two tangent-matched connections leave a straight across the green.
    for (float clearance : {205.0f, 215.0f})
    {
        if (uint32_t(micros() - started) >= budget) break;
        const float offset = seats[green].lateralMm + clearance;
        if (hypotf(baselinePath[0].x + nx * offset - rollback[0].x,
                  baselinePath[0].y + ny * offset - rollback[0].y) > 55.0f)
            continue;
        for (uint16_t i = 0; i < pathLength; ++i)
        {
            if (i > straightOut && i < straightIn) continue;
            route[i].x = baselinePath[i].x + nx * offset;
            route[i].y = baselinePath[i].y + ny * offset;
        }
        if (connectLaterLapPasses(route, route, first, straightIn, innerExit ? 0.23f : 0.26f) &&
            connectLaterLapPasses(route, route, straightOut, last, innerExit ? 0.29f : 0.50f))
        {
            float newLength, newPeak, newTurn, wallIn, pillarIn, wallOut, pillarOut;
            if (laterLapBending(route, newLength, newPeak, &newTurn) < oldBending * 0.98f &&
                newTurn < turn - 0.025f && newPeak <= peak + 1e-5f &&
                laterLapRouteSafe(route, &wallIn, &pillarIn, started, budget, first - 1, pathLength) &&
                laterLapRouteSafe(route, &wallOut, &pillarOut, started, budget, 0, last + 2) &&
                fminf(wallIn, wallOut) >= 50.0f && fminf(pillarIn, pillarOut) >= 50.0f &&
                laterLapTargetTime(route, true) <= oldTime &&
                laterLapTrackedRouteSafe(route, started, budget, first - 4, last + 5, 800, 50.0f))
            {
                Serial.print("[LAPS] Parallel CCW start GREEN seat/turn_saved_deg=");
                Serial.print(green); Serial.print('/');
                Serial.println((turn - newTurn) * 180.0f / PI, 1);
                return;
            }
        }
        memcpy(route, rollback, sizeof(PathPoint) * pathLength);
    }
}

void refineLaterLapLongCorners(PathPoint *route, PathPoint *rollback)
{
    // The two first start-section seats can have a phase near the *end* of
    // the lap. Traverse placed signs in phase order across that boundary.
    uint8_t order[OBSTACLE_SEAT_COUNT], count = 0;
    for (uint8_t seat = 0; seat < OBSTACLE_SEAT_COUNT; ++seat)
    {
        if (!seats[seat].confirmed) continue;
        uint8_t i = count++;
        while (i > 0 && seats[order[i - 1]].pathDistanceMm > seats[seat].pathDistanceMm)
        {
            order[i] = order[i - 1];
            --i;
        }
        order[i] = seat;
    }
    if (count == 0) return;

    uint32_t started = micros();
    const uint32_t budget = OBSTACLE_LATER_LAP_INNER_PLAN_BUDGET_US;
    const float keepStart = OBSTACLE_LATER_LAP_HANDOFF_MM;
    const float keepEnd = loopLengthMm - OBSTACLE_LATER_LAP_SEAM_KEEP_MM;
    float checkedTime = -1.0f;
    const auto tryWindow = [&](int from, int to, float begin, float end, float scale,
                               uint8_t trackingTail = 5)
    {
        if (uint32_t(micros() - started) >= budget || begin < keepStart - 0.1f ||
            end > keepEnd + 0.1f || end - begin > 2650.0f)
            return false;
        uint16_t first = 1, last = 1;
        while (first + 1 < pathLength && baselinePath[first].distanceMm < begin - 0.1f) ++first;
        while (last + 2 < pathLength && baselinePath[last + 1].distanceMm <= end + 0.1f) ++last;
        if (last <= first + 5) return false;

        memcpy(rollback, route, sizeof(PathPoint) * pathLength);
        float savedTurn;
        if (!tryLaterLapPassConnection(route, rollback, first, last, scale,
                started, budget, 650, checkedTime, savedTurn, trackingTail))
            return false;
        Serial.print("[LAPS] Long corner from/to/turn_saved_deg=");
        Serial.print(from); Serial.print('/'); Serial.print(to); Serial.print('/');
        Serial.println(savedTurn * 180.0f / PI, 1);
        return true;
    };

    for (uint8_t i = 1; i < count; ++i)
    {
        if (uint32_t(micros() - started) >= budget) break;
        const int from = order[i - 1], to = order[i];
        if ((from / 6 + 1) % 4 != to / 6) continue;
        const float begin = seats[from].pathDistanceMm, end = seats[to].pathDistanceMm;
        const bool finalCorner = begin < corners[3].pathStartMm &&
            end >= corners[3].pathEndMm - 100.0f && !seats[from].red && seats[to].red;
        const bool longGap = end - begin >= 2000.0f;
        const bool middleReds = solitaryMiddleSeat(from / 6) == from &&
            solitaryMiddleSeat(to / 6) == to && seats[from].red && seats[to].red;
        if (!finalCorner && !longGap && !middleReds) continue;

        // Preserve the flat CW line at a lone outer RED if it bounds a gap.
        const bool keepEntry = isCwSolitaryOuterRed(from), keepExit = isCwSolitaryOuterRed(to);
        struct Trial { float entryOffset, exitOffset, handleScale; };
        constexpr Trial trials[] = {
            {50.0f, 100.0f, 0.38f}, {0.0f, 100.0f, 0.26f},
            {-100.0f, 100.0f, 0.26f}, {0.0f, 200.0f, 0.32f},
            {150.0f, 50.0f, 0.32f}, {0.0f, -150.0f, 0.26f}
        };
        if ((finalCorner || (longGap && seats[to].red)) && !keepEntry && !keepExit &&
            tryWindow(from, to, begin - 100.0f, end + 100.0f, 0.26f))
            continue;
        if (middleReds && !keepEntry && !keepExit &&
            tryWindow(from, to, begin, end + 100.0f, 0.26f))
            continue;
        for (const auto &trial : trials)
        {
            if (uint32_t(micros() - started) >= budget) break;
            const float start = begin + (keepEntry ? 365.0f : trial.entryOffset);
            const float finish = end + (keepExit ? -365.0f : trial.exitOffset);
            if (tryWindow(from, to, start, finish, trial.handleScale)) break;
        }
    }

    const int firstSign = order[0], lastSign = order[count - 1];
    const float firstPass = seats[firstSign].pathDistanceMm;
    const float lastPass = seats[lastSign].pathDistanceMm;
    // Join the first unobstructed corner to the next known passing line.
    if (firstPass >= 850.0f && firstPass <= corners[0].pathEndMm + 525.0f &&
        !tryWindow(-1, firstSign, keepStart, firstPass, 0.26f))
        tryWindow(-1, firstSign, keepStart + 100.0f, firstPass + 100.0f, 0.32f);
    // Return from a sign at the last corner to the unchanged 200 mm seam.
    if (seats[lastSign].red && fabsf(lastPass - corners[3].pathEndMm) <= 100.0f &&
        !tryWindow(lastSign, -1, lastPass - 50.0f, keepEnd, 0.26f))
        tryWindow(lastSign, -1, lastPass - 100.0f, keepEnd, 0.26f);

    // Give the lane transitions not covered by the normal sign-to-sign
    // windows a separate bounded check. Preserve the start/finish anchors.
    started = micros();
    const bool ccw = routeTurnSign > 0;
    const bool startMiddleGreen = ccw && solitaryMiddleSeat(0) == firstSign &&
        !seats[firstSign].red && firstPass < keepStart;
    const bool clearStart = firstSign >= 6 && lastSign >= 6;
    if (startMiddleGreen)
    {
        // Complete the RED-to-start transition early enough to settle on the
        // checked start bypass before it passes GREEN.
        if (seats[lastSign].red && lastSign / 6 == 3 && lastPass < corners[3].pathStartMm)
            tryWindow(lastSign, firstSign, lastPass + 50.0f, keepEnd - 150.0f, 0.50f, 1);
        if (count > 1 && order[1] / 6 == 1 && seats[order[1]].red)
        {
            const float nextPass = seats[order[1]].pathDistanceMm;
            if (!tryWindow(firstSign, order[1], keepStart, nextPass + 200.0f, 0.38f))
                tryWindow(firstSign, order[1], keepStart, nextPass + 150.0f, 0.44f);
        }
    }
    else if (ccw && clearStart && solitaryMiddleSeat(3) == lastSign && !seats[lastSign].red)
    {
        // Continue from GREEN's passing lane through the open return up to
        // the start-section anchor.
        tryWindow(lastSign, -1, lastPass, keepEnd, 0.26f, 1);
    }

    for (uint8_t i = 1; i < count; ++i)
    {
        if (uint32_t(micros() - started) >= budget) break;
        const int from = order[i - 1], to = order[i];
        if ((from / 6 + 1) % 4 != to / 6) continue;
        const float fromPass = seats[from].pathDistanceMm, toPass = seats[to].pathDistanceMm;
        const bool destinationGreen = !seats[to].red;
        if (ccw && !seats[from].red && !destinationGreen && to % 6 < 2 &&
            solitaryMiddleSeat(from / 6) == from && solitaryMiddleSeat(0) != from)
        {
            // Keep the gentler middle-GREEN exit all the way to the early
            // RED in the following section.
            tryWindow(from, to, fromPass, toPass - 75.0f, 0.26f);
        }
        if (!seats[from].red) continue;
        if (ccw && destinationGreen && from % 6 >= 4 && solitaryMiddleSeat(to / 6) == to)
        {
            const uint8_t section = (from / 6) * 6;
            const bool redPair = (seats[section].confirmed && seats[section].red) ||
                (seats[section + 1].confirmed && seats[section + 1].red);
            if (redPair && tryWindow(from, to, fromPass - 150.0f, toPass - 300.0f, 0.44f))
                continue;
            if (!tryWindow(from, to, fromPass - 250.0f, toPass - 50.0f, 0.50f))
                tryWindow(from, to, fromPass + 50.0f, toPass, 0.26f);
        }
        else if (!ccw && destinationGreen && to % 6 >= 4 &&
                 solitaryMiddleSeat(from / 6) == from && seats[from].lateralMm < 0.0f)
        {
            // A late GREEN leaves room to carry one right bend through the
            // corner and onto its lane, without an intermediate recenter.
            if (!tryWindow(from, to, fromPass + 100.0f, toPass + 50.0f, 0.38f))
                tryWindow(from, to, fromPass, toPass, 0.38f);
        }
        else if (!ccw && !destinationGreen && from / 6 == 3 && to < 6 &&
                 isCwSolitaryOuterRed(from) && fabsf(toPass - corners[3].pathEndMm) <= 100.0f)
        {
            // Continue the straight inside RED's lane into one last corner.
            // Stop the connector before the existing parking handoff.
            tryWindow(from, to, fromPass + 400.0f, toPass + 50.0f, 0.32f, 1);
        }
    }
    if (startMiddleGreen && firstPass <= 100.0f && count > 1 &&
        order[1] / 6 == 1 &&
        lastSign / 6 == 3 && lastPass < corners[3].pathStartMm)
        refineCcwParallelStart(route, rollback, lastSign, firstSign, order[1]);
}

void refineLaterLapCornerGeometry(PathPoint *route, PathPoint *rollback)
{
    struct Trial { float entryOffset, exitOffset, scale; bool useCornerEntry; };
    // Sample around the actual passing lanes and the corner tangent.
    // Neither the candidate windows nor their ranking use the sign colours.
    constexpr Trial trials[] = {
        {100, -100, 0.23f, false}, {0, -125, 0.23f, false},
        {-50, -150, 0.30f, false}, {-100, -150, 0.36f, false},
        {150, 100, 0.23f, false}, {-200, -400, 0.54f, false},
        {150, 300, 0.36f, false}, {50, 100, 0.36f, false},
        {-150, -50, 0.23f, false}, {-400, 0, 0.48f, false},
        {-400, -100, 0.42f, false}, {100, 50, 0.30f, false},
        {-50, 150, 0.42f, false}, {-150, -300, 0.44f, false},
        {200, 250, 0.36f, true}, {350, 250, 0.48f, true},
        {200, 200, 0.48f, true}, {50, 200, 0.36f, true},
        {-150, 100, 0.32f, true}, {100, -50, 0.38f, true},
    };
    struct Choice { uint16_t first, last; float scale, score; };
    const uint32_t started = micros();
    constexpr uint32_t budget = 65000UL;
    const float keepStart = OBSTACLE_LATER_LAP_HANDOFF_MM;
    const float keepEnd = loopLengthMm - OBSTACLE_LATER_LAP_SEAM_KEEP_MM;
    struct Span { float begin, end; };
    // Keep the already checked CCW link between two solitary middle signs.
    // Its entry line and RED passing plateau form one connected corridor.
    Span verified[COURSE_SECTION_COUNT]{};
    uint8_t verifiedCount = 0;
    if (routeTurnSign > 0)
    {
        for (uint8_t section = 1; section + 1 < COURSE_SECTION_COUNT; ++section)
        {
            const int green = solitaryMiddleSeat(section), red = solitaryMiddleSeat(section + 1);
            if (green >= 0 && red >= 0 && !seats[green].red && seats[red].red)
                verified[verifiedCount++] = {seats[green].pathDistanceMm - 100.0f,
                                             seats[red].pathDistanceMm + 100.0f};
        }
    }
    float checkedTime = -1.0f;
    for (uint8_t corner = 0; corner < COURSE_SECTION_COUNT; ++corner)
    {
        const float entry = corners[corner].pathStartMm;
        const float exit = corners[corner].pathEndMm;
        int previous = -1, next = -1;
        for (uint8_t id = 0; id < OBSTACLE_SEAT_COUNT; ++id)
        {
            if (!seats[id].confirmed) continue;
            const float pass = seats[id].pathDistanceMm;
            if (pass <= entry + 0.1f && pass >= entry - 1050.0f &&
                (previous < 0 || pass > seats[previous].pathDistanceMm))
                previous = id;
            if (pass >= exit - 0.1f && pass <= exit + 1050.0f &&
                (next < 0 || pass < seats[next].pathDistanceMm))
                next = id;
        }
        if (previous < 0 && next < 0) continue;

        const float entryReference = previous < 0 ? entry - 150.0f : seats[previous].pathDistanceMm;
        const float exitReference = next < 0 ? fminf(exit + 300.0f, keepEnd) : seats[next].pathDistanceMm;
        const float entryLimit = isCwSolitaryOuterRed(previous)
            ? fmaxf(keepStart, entryReference + 365.0f) : keepStart;
        const float exitLimit = isCwSolitaryOuterRed(next)
            ? fminf(keepEnd, exitReference - 365.0f) : keepEnd;
        memcpy(rollback, route, sizeof(PathPoint) * pathLength);
        Choice shortlist[5]{};
        for (const auto &trial : trials)
        {
            if (uint32_t(micros() - started) >= budget) return;
            const float begin = fmaxf(entryLimit,
                (trial.useCornerEntry ? entry : entryReference) + trial.entryOffset);
            const float end = fminf(exitLimit, exitReference + trial.exitOffset);
            if (begin > entry + 400.0f || end < exit - 160.0f ||
                end - begin < 550.0f || end - begin > 2450.0f)
                continue;
            bool overlapsVerified = false;
            for (uint8_t i = 0; i < verifiedCount; ++i)
                overlapsVerified |= begin < verified[i].end && end > verified[i].begin;
            if (overlapsVerified) continue;
            uint16_t first = 1, last = 1;
            while (first + 1 < pathLength && baselinePath[first].distanceMm < begin - 0.1f) ++first;
            while (last + 2 < pathLength && baselinePath[last + 1].distanceMm <= end + 0.1f) ++last;
            if (last <= first + 5) continue;
            float score = 0.0f;
            if (connectLaterLapPasses(route, rollback, first, last, trial.scale))
            {
                float oldLength, oldPeak, oldTurn, length, peak, turn;
                const float oldBending = laterLapBending(rollback, oldLength, oldPeak, &oldTurn,
                    first, last + 1);
                const float bending = laterLapBending(route, length, peak, &turn, first, last + 1);
                if (bending < 0.97f * oldBending && peak <= oldPeak + 1e-6f &&
                    (turn < oldTurn - 0.025f || (turn <= oldTurn + 0.025f && peak <= 0.95f * oldPeak)) &&
                    length <= 1.1f * oldLength)
                    score = (oldTurn - turn) + 200.0f * (oldPeak - peak) +
                        60.0f * (oldBending - bending) + 0.0005f * (oldLength - length);
            }
            memcpy(route + first, rollback + first, sizeof(PathPoint) * (last - first + 1));
            if (score <= 0.0f) continue;
            bool duplicate = false;
            for (const auto &choice : shortlist)
                duplicate |= choice.first == first && choice.last == last && choice.scale == trial.scale;
            if (duplicate) continue;
            for (auto &choice : shortlist)
            {
                if (score > choice.score)
                {
                    Choice candidate{first, last, trial.scale, score};
                    for (auto *slot = &choice; slot != shortlist + 5; ++slot)
                    {
                        const Choice displaced = *slot;
                        *slot = candidate;
                        candidate = displaced;
                    }
                    break;
                }
            }
        }
        for (const auto &choice : shortlist)
        {
            if (choice.score <= 0.0f || uint32_t(micros() - started) >= budget) break;
            float savedTurn;
            if (!tryLaterLapPassConnection(route, rollback, choice.first, choice.last,
                    choice.scale, started, budget, 650, checkedTime, savedTurn,
                    corner == 3 ? 1 : 5, true))
                continue;
            Serial.print("[LAPS] Corner geometry corner/turn_saved_deg=");
            Serial.print(corner); Serial.print('/');
            Serial.println(savedTurn * 180.0f / PI, 1);
            break;
        }
    }
}

uint16_t laterLapWaypointAt(float phase)
{
    uint16_t index = 0;
    while (index + 1 < pathLength && baselinePath[index + 1].distanceMm <= phase + 0.1f)
        ++index;
    return index;
}

bool tryLaterLapStraightPass(PathPoint *route, PathPoint *backup, int seat,
                             uint16_t first, uint16_t lineStart, uint16_t lineEnd,
                             uint16_t last, float clearance, float scale,
                             uint32_t started, uint32_t budget)
{
    const bool crossesStart = lineStart > lineEnd;
    const uint16_t entryLimit = crossesStart ? pathLength - 1 : lineEnd;
    if (first < 5 || lineStart <= first + 4 || last <= lineEnd + 4 ||
        (crossesStart && last + 5 >= first))
        return false;
    if (uint32_t(micros() - started) >= budget) return false;

    const float h = seats[seat].headingDeg * PI / 180.0f;
    const float offset = targetLateralForSeat(seats[seat], clearance);
    const float dx = -sinf(h) * offset, dy = cosf(h) * offset;
    if (crossesStart && hypotf(baselinePath[0].x + dx - backup[0].x,
                              baselinePath[0].y + dy - backup[0].y) > 55.0f)
        return false;
    float oldLength, oldPeak;
    const float oldBending = laterLapBending(backup, oldLength, oldPeak);
    const float oldTime = laterLapTargetTime(backup, true);
    for (uint16_t i = lineStart; i <= entryLimit; ++i)
    {
        route[i].x = baselinePath[i].x + dx;
        route[i].y = baselinePath[i].y + dy;
    }
    if (crossesStart)
    {
        for (uint16_t i = 0; i <= lineEnd; ++i)
        {
            route[i].x = baselinePath[i].x + dx;
            route[i].y = baselinePath[i].y + dy;
        }
    }
    const auto safeSpan = [&](uint16_t start, uint16_t end)
    {
        float wall, pillar;
        return laterLapRouteSafe(route, &wall, &pillar, started, budget, start, end) &&
            wall >= 50.0f && pillar >= 50.0f;
    };
    if (connectLaterLapPasses(route, route, first, lineStart, scale) &&
        connectLaterLapPasses(route, route, lineEnd, last, scale))
    {
        float length, peak;
        if (laterLapBending(route, length, peak) < oldBending * 0.99f &&
            laterLapTargetTime(route, true) < oldTime - 0.01f &&
            safeSpan(first - 1, crossesStart ? pathLength : last + 2) &&
            (!crossesStart || safeSpan(0, last + 2)) &&
            laterLapTrackedRouteSafe(route, started, budget, first - 4, last + 3, 1000, 50.0f))
        {
            Serial.print("[LAPS] Straight passing line seat=");
            Serial.println(seat);
            return true;
        }
    }
    memcpy(route, backup, sizeof(PathPoint) * pathLength);
    return false;
}

void refineLaterLapStartStraight(PathPoint *route, PathPoint *backup)
{
    const int middle = solitaryMiddleSeat(0);
    if (middle < 0 || seats[middle].red != (routeTurnSign < 0)) return;
    float low = INFINITY, high = -INFINITY;
    for (uint16_t i = 0; i < pathLength; ++i)
    {
        const float phase = baselinePath[i].distanceMm;
        if (phase > 150.0f && phase < loopLengthMm - 150.0f) continue;
        low = fminf(low, route[i].y);
        high = fmaxf(high, route[i].y);
    }
    if (high - low < 15.0f) return;

    int previous = -1, next = -1;
    for (uint8_t i = 6; i < OBSTACLE_SEAT_COUNT; ++i)
    {
        if (!seats[i].confirmed) continue;
        if (i / 6 == 1 && next < 0) next = i;
        if (i / 6 == 3) previous = i;
    }
    if (previous < 0 || next < 0) return;

    const uint32_t started = micros();
    constexpr uint32_t budget = 45000UL;
    const float from = seats[previous].pathDistanceMm, to = seats[next].pathDistanceMm;
    const bool nextIsMiddle = solitaryMiddleSeat(1) == next;
    const uint16_t first = laterLapWaypointAt(from - (nextIsMiddle ? 100.0f : 0.0f));
    const uint16_t lineStart = laterLapWaypointAt(loopLengthMm - (nextIsMiddle ? 300.0f : 200.0f));
    const uint16_t lineEnd = laterLapWaypointAt(200.0f);
    const uint16_t last = laterLapWaypointAt(to + (nextIsMiddle ? 150.0f : -50.0f));
    memcpy(backup, route, sizeof(PathPoint) * pathLength);
    for (float clearance : {205.0f, 215.0f, 225.0f})
    {
        if (tryLaterLapStraightPass(route, backup, middle, first, lineStart, lineEnd,
                                    last, clearance, 0.23f, started, budget))
            return;
    }
    // A late sign on an outside pass needs its exit before the start turn.
    const float previousLane = targetLateralForSeat(seats[previous], optimizedClearanceForSeat(previous));
    if (routeTurnSign * previousLane < 0.0f && !nextIsMiddle)
        tryLaterLapStraightPass(route, backup, middle, laterLapWaypointAt(from + 200.0f),
            lineStart, lineEnd, laterLapWaypointAt(to - 200.0f), 205.0f, 0.18f, started, budget);
}

void refineLaterLapOpenStart(PathPoint *route, PathPoint *backup)
{
    // When the entry half of the start straight is clear, the return can
    // join it without keeping the old corner bulge. With the whole start
    // clear, also carry an isolated inside pass on a straight after it.
    for (uint8_t i = 0; i < 4; ++i)
        if (seats[i].confirmed) return;

    const uint32_t started = micros();
    constexpr uint32_t budget = 40000UL;
    const bool lateStartSign = seats[4].confirmed || seats[5].confirmed;
    if (!lateStartSign)
    {
        const int firstSign = solitaryMiddleSeat(1);
        if (firstSign < 0 || seats[firstSign].red != (routeTurnSign < 0)) return;
        const float pass = seats[firstSign].pathDistanceMm;
        memcpy(backup, route, sizeof(PathPoint) * pathLength);
        tryLaterLapStraightPass(route, backup, firstSign,
                               laterLapWaypointAt(pass - 650.0f), laterLapWaypointAt(pass - 150.0f),
                               laterLapWaypointAt(pass + 400.0f), laterLapWaypointAt(pass + 900.0f),
                               175.0f, 0.21f, started, budget);
    }

    const int side = routeTurnSign > 0 ? 1 : 0;
    const int finalSign = 22 + side;
    if (!seats[finalSign].confirmed || seats[finalSign].red == (routeTurnSign > 0)) return;
    const uint16_t first = laterLapWaypointAt(seats[finalSign].pathDistanceMm - 100.0f);
    const float keep = lateStartSign ? 100.0f : OBSTACLE_LATER_LAP_SEAM_KEEP_MM;
    const uint16_t last = laterLapWaypointAt(loopLengthMm - keep);
    memcpy(backup, route, sizeof(PathPoint) * pathLength);
    float time = -1.0f, savedTurn;
    if (tryLaterLapPassConnection(route, backup, first, last, 0.23f,
                                  started, budget, 650, time, savedTurn, 5, true))
        Serial.println("[LAPS] Direct corner to clear start entry");
}

void refineLaterLapEarlyStartExit(PathPoint *route, PathPoint *backup)
{
    const int sign = seats[0].confirmed ? 0 : seats[1].confirmed ? 1 : -1;
    const int next = seats[6].confirmed ? 6 : seats[7].confirmed ? 7 : -1;
    if (sign < 0 || next < 0 || seats[sign].red == seats[next].red ||
        seats[next].red != (routeTurnSign < 0)) return;
    for (uint8_t i = 2; i < 6; ++i)
        if (seats[i].confirmed) return;

    // Let the passing lane of the first start sign lead into the corner
    // without tapering back to the wide centre line at the lap seam.
    const uint32_t started = micros();
    constexpr uint32_t budget = 35000UL;
    memcpy(backup, route, sizeof(PathPoint) * pathLength);
    tryLaterLapStraightPass(route, backup, sign,
                           laterLapWaypointAt(seats[sign].pathDistanceMm + 15.0f),
                           laterLapWaypointAt(loopLengthMm - 100.0f), laterLapWaypointAt(100.0f),
                           laterLapWaypointAt(seats[next].pathDistanceMm + 50.0f),
                           120.0f, 0.23f, started, budget);
}

void refineLaterLapInnerMiddleCorners(PathPoint *route, PathPoint *backup)
{
    const uint32_t started = micros();
    constexpr uint32_t budget = 35000UL;
    for (uint8_t section = 1; section + 1 < COURSE_SECTION_COUNT; ++section)
    {
        const int from = solitaryMiddleSeat(section);
        const int to = solitaryMiddleSeat(section + 1);
        if (from < 0 || to < 0 || seats[from].red != (routeTurnSign < 0) ||
            seats[to].red != seats[from].red ||
            isCwSolitaryOuterRed(from) || isCwSolitaryOuterRed(to)) continue;

        // Both signs can be passed on the same side next to the island.
        const uint16_t first = laterLapWaypointAt(seats[from].pathDistanceMm + 150.0f);
        const uint16_t last = laterLapWaypointAt(seats[to].pathDistanceMm - 150.0f);
        memcpy(backup, route, sizeof(PathPoint) * pathLength);
        float time = -1.0f, savedTurn;
        if (tryLaterLapPassConnection(route, backup, first, last, 0.50f,
                                     started, budget, 850, time, savedTurn, 5, true))
            Serial.println("[LAPS] Inner corner between solitary middle signs");
    }
}

void refineLaterLapStartExit(PathPoint *route, PathPoint *backup)
{
    const int start = seats[4].confirmed ? 4 : seats[5].confirmed ? 5 : -1;
    const int late = seats[10].confirmed ? 10 : seats[11].confirmed ? 11 : -1;
    const int early = seats[12].confirmed ? 12 : seats[13].confirmed ? 13 : -1;
    if (start < 0 || late < 0 || early < 0) return;
    for (uint8_t i = 6; i < 10; ++i)
        if (seats[i].confirmed) return;
    const auto inside = [](int i)
    {
        return routeTurnSign *
            targetLateralForSeat(seats[i], optimizedClearanceForSeat(i)) > 0.0f;
    };
    if (inside(start) || !inside(late) || !inside(early)) return;

    // With the next two passes on the inside there is room to steer in
    // once, instead of running out to the middle of the free straight.
    const uint32_t started = micros();
    constexpr uint32_t budget = 30000UL;
    const uint16_t last = laterLapWaypointAt(seats[early].pathDistanceMm + 120.0f);
    memcpy(backup, route, sizeof(PathPoint) * pathLength);
    float time = -1.0f, savedTurn;
    for (float scale : {0.36f, 0.43f, 0.26f})
    {
        const uint16_t first = laterLapWaypointAt(seats[start].pathDistanceMm +
                                                  (scale < 0.3f ? 400.0f : 200.0f));
        if (tryLaterLapPassConnection(route, backup, first, last, scale,
                                      started, budget, 650, time, savedTurn, 5, true))
        {
            Serial.println("[LAPS] Direct link from last start sign to inside passes");
            return;
        }
    }
}

void refineCcwFirstSeatCorner(PathPoint *route, PathPoint *backup)
{
    if (routeTurnSign<0) return;
    for (uint8_t section=1;section+1<COURSE_SECTION_COUNT;++section)
    {
        const uint8_t from=section*6+1;
        const uint8_t green=from+6;
        if (!seats[from].confirmed || !seats[green].confirmed ||
            seats[green].red) continue;
        bool clearStraight=true;
        for (uint8_t i=from+1;i<green-1;++i) clearStraight &= !seats[i].confirmed;
        if (!clearStraight) continue;
        // Start joining at the preceding pass. Waiting until the corner retains
        // the opposite steering from the inward return on the empty straight.
        const uint16_t first=laterLapWaypointAt(seats[from].pathDistanceMm-150.0f);
        const uint16_t last=laterLapWaypointAt(seats[green].pathDistanceMm+100.0f);
        memcpy(backup,route,sizeof(PathPoint)*pathLength);
        if (!seats[from].red)
        {
            // Both signs are passed on the island side. Follow the same
            // lane between them so the empty straight cannot pull the car
            // back to the centre before it turns in for the second sign.
            const float begin=seats[from].pathDistanceMm;
            const float finish=seats[green].pathDistanceMm;
            const uint16_t before=laterLapWaypointAt(begin-300.0f);
            const uint16_t after=laterLapWaypointAt(finish+300.0f);
            float oldLength,oldPeak;
            const float oldBending=laterLapBending(backup,oldLength,oldPeak,nullptr,before,after+1);
            const float oldTime=laterLapTargetTime(backup,true);
            for (float radius:{225.0f,210.0f,240.0f})
            {
                for (uint16_t i=before;i<=after;++i)
                {
                    const auto &base=baselinePath[i];
                    const float h=base.headingDeg*PI/180.0f;
                    const float offset=OBSTACLE_CORNER_RADIUS_MM-radius;
                    const float blend=smoothLaneBlend(base.distanceMm,begin-300.0f,begin+200.0f)*
                        (1.0f-smoothLaneBlend(base.distanceMm,finish+30.0f,finish+300.0f));
                    route[i].x=backup[i].x+blend*(base.x-offset*sinf(h)-backup[i].x);
                    route[i].y=backup[i].y+blend*(base.y+offset*cosf(h)-backup[i].y);
                }
                float length,peak,wall,pillar;
                const float bending=laterLapBending(route,length,peak,nullptr,before,after+1);
                if (bending<oldBending && peak<=oldPeak && length<oldLength &&
                    laterLapRouteSafe(route,&wall,&pillar,0,0,before-1,after+2) &&
                    wall>=55.0f && pillar>=50.0f && laterLapTargetTime(route,true)<=oldTime &&
                    laterLapTrackedRouteSafe(route,0,0,before-4,after+5,850,50.0f,55.0f))
                {
                    Serial.print("[LAPS] Inner first-seat corner seats=");
                    Serial.print(from);Serial.print('/');Serial.println(green);
                    break;
                }
                memcpy(route+before,backup+before,sizeof(PathPoint)*(after-before+1));
            }
            continue;
        }
        float time=-1.0f,savedTurn=0.0f;
        if (tryLaterLapPassConnection(route,backup,first,last,0.23f,0,0,
                                     850,time,savedTurn,6,true))
        {
            Serial.print("[LAPS] Tangent first-seat corner seats=");
            Serial.print(from);Serial.print('/');Serial.println(green);
        }
    }
}

void refineCcwGreenRedStraight(PathPoint *route, PathPoint *backup)
{
    if (routeTurnSign<0) return;
    for (uint8_t green : {7,13,19})
    {
        const uint8_t red=green+3;
        if (!seats[green].confirmed || seats[green].red ||
            !seats[red].confirmed || !seats[red].red ||
            seats[green+1].confirmed || seats[green+2].confirmed) continue;
        const float begin=seats[green].pathDistanceMm, end=seats[red].pathDistanceMm;
        const uint16_t first=laterLapWaypointAt(begin-200.0f);
        const uint16_t last=laterLapWaypointAt(end+440.0f);
        if (first<4 || last+5>=pathLength) continue;
        memcpy(backup,route,sizeof(PathPoint)*pathLength);
        const float oldTime=laterLapTargetTime(backup,true);
        const float h=seats[green].headingDeg*PI/180.0f;
        for (float greenShift : {20.0f,10.0f})
        {
            for (uint16_t i=first;i<=last;++i)
            {
                const auto &base=baselinePath[i];
                const float shift=greenShift+(35.0f-greenShift)*
                    smoothLaneBlend(base.distanceMm,begin+50.0f,end-140.0f);
                const float offset=shift*smoothLaneBlend(base.distanceMm,begin-200.0f,begin-10.0f)*
                    (1.0f-smoothLaneBlend(base.distanceMm,end+80.0f,end+440.0f));
                route[i].x=backup[i].x-offset*sinf(h);
                route[i].y=backup[i].y+offset*cosf(h);
            }
            float wall,pillar;
            if (laterLapRouteSafe(route,&wall,&pillar,0,0,first-1,last+2) &&
                wall>=55.0f && pillar>=50.0f &&
                laterLapTargetTime(route,true)<=oldTime+0.05f &&
                laterLapTrackedRouteSafe(route,0,0,first-4,last+5,1200,50.0f,55.0f))
            {
                Serial.print("[LAPS] Green-red straight inward shift green/red mm=");
                Serial.print(greenShift,0);Serial.println("/35");
                break;
            }
            memcpy(route+first,backup+first,sizeof(PathPoint)*(last-first+1));
        }
    }
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
    {
        smoothKnownLaterLapRoute(optimizedPath,beforeLaneCarry,routeWall,routePillar);
        shortenKnownInnerRoute(optimizedPath,beforeLaneCarry);
        addLaterLapStartMiddleReserve(optimizedPath,beforeLaneCarry);
        refineCcwSolitaryPillars(optimizedPath,beforeLaneCarry);
        refineCwSolitaryRed(optimizedPath,beforeLaneCarry);
        refineLaterLapPassConnections(optimizedPath,beforeLaneCarry);
    }
    // Parking can drive back along the start section in CW. Its checked
    // bypass stays independent of the faster forward racing connections.
    memcpy(parkingApproachPath, optimizedPath, sizeof(PathPoint) * pathLength);
    if (sectionLayoutMode == OBSTACLE_SECTION_LAYOUT_OFFICIAL)
    {
        refineLaterLapLongCorners(optimizedPath, beforeLaneCarry);
        refineLaterLapCornerGeometry(optimizedPath, beforeLaneCarry);
        refineLaterLapStartStraight(optimizedPath, beforeLaneCarry);
        refineLaterLapOpenStart(optimizedPath, beforeLaneCarry);
        refineLaterLapEarlyStartExit(optimizedPath, beforeLaneCarry);
        refineLaterLapInnerMiddleCorners(optimizedPath, beforeLaneCarry);
        refineLaterLapStartExit(optimizedPath, beforeLaneCarry);
        refineCcwFirstSeatCorner(optimizedPath, beforeLaneCarry);
        refineCcwGreenRedStraight(optimizedPath, beforeLaneCarry);
    }
    recomputeSpeedProfile(optimizedPath);
    planLaterLapSpeeds(optimizedPath,true);
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
        "clearance_policy=20261122_lap23_first_seat_corner");
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
    if (!runtimeTestMode && completedLaps == 0 && runtimeLapTarget > 1)
    {
        const uint32_t planningStartUs = micros();
        if (!buildOptimizedPath()) return false;
        Serial.print("[LAPS] Handoff planning_us=");
        Serial.println(uint32_t(micros() - planningStartUs));
        lapHandoffActive = lap1CommandValid;
        lapHandoffEncoderMm = get_distance();
        lapHandoffTravelMm = 0.0f;
    }
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

bool recordParkingFinish(const PositionEstimate &pose)
{
    if (!runtimeTestMode && runtimeLapTarget==3 && completedLaps==2 &&
        lapCountingArmed && optimizedBuilt && finalCornerVehicleClear(pose))
    {
        lapCountingArmed=lapBoundaryPending=lapFinishPending=false;
        completedLaps=3;finished=true;
        run_telemetry_lap(3);
        Serial.println("[PATH] Completed lap 3");
        Serial.println("[LAPS] stage=FINISH; complete vehicle clear of last corner; direct parking");
    }
    return finished && completedLaps==3;
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
    if (baselinePath[progressIndex].distanceMm>=corners[3].pathEndMm &&
        recordParkingFinish(pose)) return;

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
bool discoveryAimCommandSafe(const PositionEstimate &start, int steering);

const Blob *getDiscoveryObstacle()
{
    const Blob *primary=getLargestValidObstacle();
    if (!primary) return nullptr;
    const PositionEstimate pose=get_position_struct();
    const float h=pose.heading_deg*PI/180.0f;
    const float range=obstacle_estimate_camera_range_mm(primary);
    const float ray=h+obstacle_camera_bearing_deg(primary)*PI/180.0f;
    const float x=pose.x_mm+OBSTACLE_CAMERA_LOCAL_X_MM*cosf(h)-OBSTACLE_CAMERA_LOCAL_Y_MM*sinf(h)+range*cosf(ray);
    const float y=pose.y_mm+OBSTACLE_CAMERA_LOCAL_X_MM*sinf(h)+OBSTACLE_CAMERA_LOCAL_Y_MM*cosf(h)+range*sinf(ray);
    const int seat=nearestSeatIndex(x,y,nullptr);
    if (seat<0 || !seats[seat].confirmed) return primary;
    // A nearby already-known RED must not hide the smaller next GREEN for
    // every frame merely because it has the lower foot in the image.
    const VisionResult &v=vision.getResult();
    const Blob *other=primary->color==ColorType::RED ? &v.greenPillar:&v.redPillar;
    if (!obstacle_blob_valid_for_acquisition(other))
        other=primary->color==ColorType::RED ? &v.green:&v.red;
    return obstacle_blob_valid_for_acquisition(other) ? other:primary;
}

void applyDiscoveryTargetNudge(
    PathPoint &target,
    const PositionEstimate &pose)
{
    float desiredNudgeDeg = 0.0f;
    bool checkedAimHint=false;
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
        const bool centreThirdPair = bestStation == 11 && ccwDiscoveryTallRedExitSeat() >= 0 &&
            bestForward >= 330.0f;
        const bool firstCcwMiddle = routeTurnSign > 0 && bestStation == 4;
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
                comfortableBearingDeg - 4.0f - 0.5f * separationDeg);
            if (firstCcwMiddle)
                allowedBearingDeg = fminf(allowedBearingDeg, 5.0f);
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
            const auto safeNudge = [&](float candidate) {
                const float a=candidate*PI/180.0f;
                const float dx=target.x-pose.x_mm, dy=target.y-pose.y_mm;
                const float tx=dx*cosf(a)-dy*sinf(a);
                const float ty=dx*sinf(a)+dy*cosf(a);
                const float k=2.0f*(-tx*sinf(heading)+ty*cosf(heading)) /
                    fmaxf(1.0f,tx*tx+ty*ty);
                return discoveryAimCommandSafe(pose,
                    static_cast<int>(clampFloat(pursuitServoForCurvature(k),
                        -OBSTACLE_MAX_PURSUIT_STEERING_DEG,OBSTACLE_MAX_PURSUIT_STEERING_DEG)));
            };
            // 547/549/550: a genuine GREEN component was visible before its
            // foot could snap, then left the image while the pair-centering
            // rule requested only 2--3 degrees. Keep this evidence in view;
            // it is an aiming hint, never a colour/seat confirmation.
            const auto &observation = lastDiscoveryObservation;
            bool greenHint=observation.productionValid &&
                observation.color==ColorType::GREEN && observation.seatId<0 &&
                observation.rangeMm>=OBSTACLE_DISCOVERY_VIEW_MIN_MM &&
                observation.rangeMm<=OBSTACLE_DISCOVERY_VIEW_MAX_MM;
            float hintBearing=observation.bearingDeg, hintRange=observation.rangeMm;
            // A component at a horizontal edge can steer the view inward,
            // including when range carries the synthetic clipped170mm value.
            // All other shape gates remain. This hint cannot vote/inject/clear.
            const Blob *raw=getLargestObstacle();
            if (!greenHint && raw && raw->found && raw->color==ColorType::GREEN &&
                (raw->centerX<OBSTACLE_START_MIN_X || raw->centerX>OBSTACLE_START_MAX_X ||
                 raw->minX<=2 || raw->maxX>=317))
            {
                Blob centred=*raw;
                centred.centerX=static_cast<int16_t>(clampFloat(raw->centerX,
                    OBSTACLE_START_MIN_X,OBSTACLE_START_MAX_X));
                if (obstacle_blob_valid_for_acquisition(&centred))
                {
                    hintBearing=obstacle_camera_bearing_deg(raw);
                    const float foot=raw->maxY-OBSTACLE_CAMERA_FOOT_EDGE_SLOPE*
                        fabsf(raw->centerX-OBSTACLE_CAMERA_PRINCIPAL_X_PX);
                    hintRange=foot>OBSTACLE_CAMERA_GROUND_HORIZON_Y+1.0f ?
                        OBSTACLE_CAMERA_GROUND_RANGE_SCALE_MM_PX /
                        ((foot-OBSTACLE_CAMERA_GROUND_HORIZON_Y)*cosf(hintBearing*PI/180.0f)) : 0.0f;
                    greenHint=true;
                }
            }
            if (unresolved0 && unresolved1 && !centreThirdPair && greenHint &&
                hintRange >= OBSTACLE_DISCOVERY_VIEW_MIN_MM &&
                hintRange <= OBSTACLE_DISCOVERY_VIEW_MAX_MM &&
                fabsf(hintBearing) >= 20.0f &&
                fminf(fabsf(wrap180(bearing[0]-hintBearing)),
                      fabsf(wrap180(bearing[1]-hintBearing))) <= 6.0f)
            {
                const float candidate = clampFloat(hintBearing * taper,
                    -OBSTACLE_LOOK_MAX_TARGET_NUDGE_DEG,
                     OBSTACLE_LOOK_MAX_TARGET_NUDGE_DEG);
                if (safeNudge(candidate))
                {
                    desiredNudgeDeg = candidate;
                    checkedAimHint=true;
                }
            }
            // Once the bypass guard has released, favour a single recent,
            // geometrically matched candidate over the midpoint of both rows.
            // This hint steers the view only. The next frame still has to vote.
            if (unresolved0 && unresolved1 && !checkedAimHint && !centreThirdPair)
            {
                const uint8_t first = bestStation * COURSE_SEATS_PER_STATION;
                const uint32_t now = millis();
                const auto hasRecentVote = [now](const CandidateSeat &seat) {
                    return seat.lastVoteMs != 0 &&
                        now - seat.lastVoteMs <= OBSTACLE_SEAT_VOTE_WINDOW_MS;
                };
                const bool leftPending = hasRecentVote(seats[first]);
                const bool rightPending = hasRecentVote(seats[first+1]);
                if (leftPending != rightPending)
                {
                    const float pendingBearing = bearing[rightPending ? 1 : 0];
                    const float excess = fmaxf(0.0f,fabsf(pendingBearing)-8.0f);
                    const float candidate = copysignf(fminf(15.0f,excess*taper),
                        pendingBearing);
                    if (fabsf(candidate) > fabsf(desiredNudgeDeg) && safeNudge(candidate))
                    {
                        desiredNudgeDeg = candidate;
                        checkedAimHint = true;
                    }
                }
            }
            // After a corner the two rows spread across the image rapidly.
            // Start centring while forward space is available. The normal
            // aim takes over again at the hold distance where the pair can
            // already be wider than the complete-pillar view.
            if (unresolved0 && unresolved1 && bestStation>=3 && !checkedAimHint)
            {
                const float spread=fabsf(wrap180(bearing[1]-bearing[0]));
                const bool afterTallSign=routeTurnSign>0 && bestStation%3==2 &&
                    ((seats[bestStation*2-4].modifiedSign && seats[bestStation*2-4].heightMm==150) ||
                     (seats[bestStation*2-3].modifiedSign && seats[bestStation*2-3].heightMm==150));
                const bool cwFirstCornerEntry=routeTurnSign<0 && bestStation==3;
                const bool ccwSecondStraightEntry=routeTurnSign>0 && bestStation==6;
                const bool shortRedView=OBSTACLE_SURPRISE_ENABLED && surpriseSeatIndex<0 &&
                    ((routeTurnSign<0 && bestStation==4) || (routeTurnSign>0 && bestStation==8));
                const bool earlyPair=cwFirstCornerEntry || ccwSecondStraightEntry || shortRedView;
                const float nearBlendStart=(routeTurnSign<0 && bestStation==9) ||
                    afterTallSign || earlyPair
                    ? 275.0f : 330.0f;
                const float spreadStart=earlyPair ? 12.0f : 20.0f;
                const float weight=clampFloat((spread-spreadStart)/15.0f,0.0f,1.0f)*
                    clampFloat((bestForward-nearBlendStart)/120.0f,0.0f,1.0f);
                const float centre=fmaxf(0.0f,fabsf(aimBearingDeg)-(earlyPair ? 1.0f : 4.0f));
                const float gain=earlyPair ? 2.1f : 1.25f;
                const float candidate=copysignf(fminf(earlyPair ? 38.0f : 30.0f,gain*centre*taper),
                    aimBearingDeg);
                if (weight>0.0f && fabsf(candidate)>fabsf(desiredNudgeDeg))
                {
                    const float aimed=desiredNudgeDeg+weight*(candidate-desiredNudgeDeg);
                    if (safeNudge(aimed))
                    { desiredNudgeDeg=aimed;checkedAimHint=true; }
                }
            }
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
    if (checkedAimHint)
    {
        // The slew-limited command can differ from the checked endpoint.
        // Validate the actual clamped/quantized steering as well.
        const float h=pose.heading_deg*PI/180.0f;
        const float tx=dx*cosf(nudge)-dy*sinf(nudge);
        const float ty=dx*sinf(nudge)+dy*cosf(nudge);
        const float k=2.0f*(-tx*sinf(h)+ty*cosf(h))/fmaxf(1.0f,tx*tx+ty*ty);
        const int steering=static_cast<int>(clampFloat(pursuitServoForCurvature(k),
            -OBSTACLE_MAX_PURSUIT_STEERING_DEG,OBSTACLE_MAX_PURSUIT_STEERING_DEG));
        if (!discoveryAimCommandSafe(pose,steering))
        { lastDiscoveryTargetNudgeDeg=0.0f;return; }
    }
    target.x = pose.x_mm + dx * cosf(nudge) - dy * sinf(nudge);
    target.y = pose.y_mm + dx * sinf(nudge) + dy * cosf(nudge);
}

int nearestUpcomingUnresolvedStation(float &forwardMm, uint16_t routeIndex,
                                     float minimumForwardMm = 50.0f)
{
    const float currentDistance = baselinePath[routeIndex].distanceMm;
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
        if (forward <= minimumForwardMm || forward >= forwardMm)
            continue;
        forwardMm = forward;
        bestStation = station;
    }
    return bestStation;
}

int nearestUpcomingUnresolvedStation(float &forwardMm)
{
    return nearestUpcomingUnresolvedStation(forwardMm, progressIndex);
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

bool confirmedPillarOccludesSeat(uint8_t seatIndex, const PositionEstimate &pose,
                                float bearingDeg, float rangeMm)
{
    const float ray = (pose.heading_deg+bearingDeg)*PI/180.0f;
    const float ux = cosf(ray), uy = sinf(ray);
    const CandidateSeat &target = seats[seatIndex];
    const float radius = OBSTACLE_PILLAR_MOVEMENT_RADIUS_MM;
    for (uint8_t other = 0; other < OBSTACLE_SEAT_COUNT; ++other)
    {
        if (other == seatIndex || !seats[other].confirmed)
            continue;
        const float dx = seats[other].x-target.x, dy = seats[other].y-target.y;
        const float ahead = dx*ux+dy*uy+rangeMm;
        const float cross = dx*uy-dy*ux;
        if (ahead > 0.0f && ahead < rangeMm && fabsf(cross) <= radius)
            return true;
    }
    return false;
}

// A distant GREEN component can overlap a nearby slot in the image. Keep
// the footprint and range tests even when shape alone cannot separate them.
bool rejectedGreenBehindSeat(const Blob *raw, float bearing, float range)
{
    if (raw==nullptr || !raw->found || raw->color!=ColorType::GREEN ||
        raw->height()<=0 ||
        raw->maxY<=OBSTACLE_CAMERA_GROUND_HORIZON_Y) return false;
    const bool broad=raw->width()>=2*raw->height();
    const bool alignedNearSeat=fabsf(bearing)<=15.0f && range<=450.0f &&
        raw->minX>2 && raw->maxX<317 && raw->minY>2;
    if (!broad && !alignedNearSeat) return false;
    const float forward=range*cosf(bearing*PI/180.0f);
    if (!isfinite(forward) || forward<=0.0f) return false;
    const float expectedFoot=OBSTACLE_CAMERA_GROUND_HORIZON_Y+
        OBSTACLE_CAMERA_GROUND_RANGE_SCALE_MM_PX/forward;
    const float rawForward=OBSTACLE_CAMERA_GROUND_RANGE_SCALE_MM_PX/
        (raw->maxY-OBSTACLE_CAMERA_GROUND_HORIZON_Y);
    return expectedFoot-raw->maxY>=OBSTACLE_PARKING_SEAT_FOOT_TOLERANCE_PX &&
        rawForward>=forward+OBSTACLE_DISCOVERY_BEHIND_SEAT_MARGIN_MM;
}

bool closeUnmappedObstacle(const ObstacleObservationResult &observation)
{
    // Edge-clipped blobs carry a synthetic170mm range. Require an actual low
    // ground foot as well, so a distant component at the image edge cannot
    // trigger this stop solely from that placeholder distance.
    const float centerX = 0.5f * (observation.left + observation.right);
    const float correctedFoot = observation.bottom -
        OBSTACLE_CAMERA_FOOT_EDGE_SLOPE *
            fabsf(centerX - OBSTACLE_CAMERA_PRINCIPAL_X_PX);
    return observation.productionValid && observation.seatId < 0 &&
        isfinite(observation.rangeMm) && observation.rangeMm > 0 &&
        observation.rangeMm <= 400.0f &&
        correctedFoot >= OBSTACLE_CAMERA_GROUND_HORIZON_Y +
            OBSTACLE_CAMERA_GROUND_RANGE_SCALE_MM_PX / 400.0f;
}

// A low upright colour region with no trusted map association cannot prove an
// empty seat from its disputed bearing. In512/514, edge/oversized GREEN was
// cleared before it became a valid component. Broad high wall strips remain
// covered by the separate distant-background test.
bool unmappedForegroundBlocksClear(const Blob *raw,
    const ObstacleObservationResult &observation, float bearing, float range)
{
    if (raw == nullptr || !raw->found ||
        (raw->color != ColorType::GREEN && raw->color != ColorType::RED) ||
        raw->height() < OBSTACLE_GREEN_MIN_HEIGHT ||
        raw->width() > 2 * raw->height() ||
        raw->maxY < OBSTACLE_MIN_BOTTOM_Y)
        return false;
    if (observation.productionValid && observation.seatId >= 0 &&
        observation.color == raw->color)
        return false;
    // 565: a lower-image GREEN fragment (top148, foot192) in another
    // direction vetoed every UNKNOWN seat, even after a clear local search.
    // Keep the global veto for full-height/upper silhouettes (512/514).
    // For a late-starting fragment, retain a generous12deg angular guard;
    // only an unrelated bearing may stop blocking this particular seat.
    if (raw->minY>OBSTACLE_MAX_TOP_Y && isfinite(bearing))
    {
        const float a=atanf((OBSTACLE_CAMERA_PRINCIPAL_X_PX-raw->minX)/
            OBSTACLE_CAMERA_FOCAL_X_PX)*180.0f/PI;
        const float b=atanf((OBSTACLE_CAMERA_PRINCIPAL_X_PX-raw->maxX)/
            OBSTACLE_CAMERA_FOCAL_X_PX)*180.0f/PI;
        if (bearing<fminf(a,b)-12.0f || bearing>fmaxf(a,b)+12.0f)
            return false;
    }
    const float forward = range * cosf(bearing * PI / 180.0f);
    if (!isfinite(forward) || forward <= 0) return true;
    const float expectedFoot = OBSTACLE_CAMERA_GROUND_HORIZON_Y +
        OBSTACLE_CAMERA_GROUND_RANGE_SCALE_MM_PX / forward;
    return raw->maxY >= expectedFoot -
        OBSTACLE_PARKING_SEAT_FOOT_TOLERANCE_PX;
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
            const bool distantRawGreen =
                rejectedGreenBehindSeat(rawBlob, seatBearingDeg, seatRangeMm);
            const bool distantGreenBackground = distantRawGreen &&
                !observation.productionValid &&
                (observation.status == OBSTACLE_OBSERVATION_NO_BLOB ||
                 (observation.status == OBSTACLE_OBSERVATION_REJECTED_BLOB &&
                  observation.color == ColorType::GREEN));
            const uint32_t now = millis();
            const CandidateSeat &seat = seats[seatIndex];
            const bool recentSeatVote = seat.lastVoteMs != 0 &&
                now-seat.lastVoteMs <= OBSTACLE_SEAT_VOTE_WINDOW_MS;
            const bool recentGreenSeatCandidate =
                greenSeatCandidateLastSeenMs[seatIndex] != 0 &&
                now - greenSeatCandidateLastSeenMs[seatIndex] <=
                    OBSTACLE_GREEN_SEAT_OCCUPANCY_HOLD_MS;
            // CCW41: the edge view at 422 mm declared a real middle GREEN
            // empty. Keep the station in the checked search until a nearer
            // view, while accepting a correctly matched colour at full range.
            const bool firstCcwMiddleClearReady = routeTurnSign <= 0 || station != 4 ||
                seatRangeMm <= 350.0f || fabsf(seatBearingDeg) <= 17.0f;
            const bool clearEvidence =
                !deferParkingTargetClear && comfortablyVisible && !recentSeatVote &&
                firstCcwMiddleClearReady &&
                // Log20: clearing both views at 504–567 mm let discovery
                // accelerate before getting a closer look at the middle slot.
                // Keep distant colour votes, revisit distant EMPTY.
                (parkingEntryActive || parkingEntryObserving || parkingEntryScouting ||
                 seatRangeMm<=OBSTACLE_DISCOVERY_CLEAR_MAX_MM) &&
                !confirmedPillarOccludesSeat(seatIndex,pose,seatBearingDeg,seatRangeMm) &&
                //621 marked the first place after the corner CLEAR from
                //the preceding straight (~600mm oblique view). Colour votes
                //remain allowed at full range; absence needs a nearer, aligned view.
                (station % COURSE_STATIONS_PER_SECTION != 0 ||
                    (seatRangeMm<=OBSTACLE_DISCOVERY_CLEAR_MAX_MM &&
                     (parkingEntryActive || parkingEntryObserving || parkingEntryScouting ||
                      (cornerViewPhase!=CORNER_VIEW_IDLE && cornerViewStation==station) ||
                      fabsf(wrap180(pose.heading_deg-seats[seatIndex].headingDeg))<=25.0f ||
                      (seatRangeMm>=350.0f && fabsf(seatBearingDeg)<=12.0f &&
                       fabsf(wrap180(pose.heading_deg-seats[seatIndex].headingDeg))<=38.0f)))) &&
                !greenSeatCandidateThisFrame[seatIndex] &&
                !recentGreenSeatCandidate &&
                (distantRawGreen ||
                 (!unmappedForegroundBlocksClear(rawBlob, observation,
                    seatBearingDeg, seatRangeMm) &&
                  !rejectedBlobBlocksSeatClear(rawBlob, seatBearingDeg))) &&
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
                Serial.print("[SEAT CLEAR] t=");Serial.print(now);
                Serial.print(" seat=");Serial.print(seatIndex);
                Serial.print(" bearing/range=");Serial.print(seatBearingDeg,1);
                Serial.print("/");Serial.print(seatRangeMm,1);
                Serial.print(" pose=");Serial.print(pose.x_mm,1);Serial.print(",");
                Serial.print(pose.y_mm,1);Serial.print(",");Serial.println(pose.heading_deg,1);
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
    if(!forced && run_telemetry_pose_capture_active())return;
    if(!forced && robot_logger.remaining()<112U*1024U)return;
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
    Serial.print(" near_unmapped="); Serial.print(closeUnmappedObstacle(lastDiscoveryObservation) ? 1 : 0);
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
        Serial.print(","); Serial.print((rejectedBlobBlocksSeatClear(raw, bearing) ||
            unmappedForegroundBlocksClear(raw, lastDiscoveryObservation,
                bearing, range)) ? 1 : 0);
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
    if (seats[seat ^ 1U].confirmed) return false;
    if (discoveryStations[seat/2].seatObservedClear[seat%2]) return false;
    return !(sectionInferredEmpty(seat/6) & (1U << ((seat%6)/2)));
}

bool cornerViewRetreatSafe(const PositionEstimate &start,int steering,float travel)
{
    //604 stopped with14.4mm conservative pillar reserve, below the ordinary
    //40mm planning margin. Rejecting the initial sample prohibited even
    //movement away from that possible pillar. Allow only a measured reverse
    //escape with positive>=10mm reserve, no decreasing per-pillar clearance,
    //unchanged40mm walls and a fully restored40mm endpoint reserve.
    if (!isfinite(travel) || travel>=0 || travel< -260 || abs(steering)>20) return false;
    const float k=pursuitCurvatureForServo(steering);
    const float h0=start.heading_deg*PI/180.0f;
    float previous[2][OBSTACLE_SEAT_COUNT]{};
    const unsigned steps=static_cast<unsigned>(ceilf(-travel/3.0f));
    for(unsigned step=0;step<=steps;++step)
    {
        const float ds=travel*step/steps,h=h0+k*ds;
        const float x=start.x_mm+(fabsf(k)>1e-6f ? (sinf(h)-sinf(h0))/k:ds*cosf(h0));
        const float y=start.y_mm+(fabsf(k)>1e-6f ? (cosf(h0)-cosf(h))/k:ds*sinf(h0));
        for(uint8_t rear=0;rear<2;++rear)
        {
            const float offset=rear ? OBSTACLE_MAX_WHEEL_HALF_WIDTH_MM:0;
            ObstacleClearanceSample walls{};
            const float px=x-offset*cosf(h),py=y-offset*sinf(h);
            if(!calculateClearanceAtPose(seats[0],px,py,h*180/PI,walls)||walls.wallMm<=40) return false;
            for(uint8_t seat=0;seat<OBSTACLE_SEAT_COUNT;++seat)
            {
                if(!cornerViewSeatMayBeOccupied(seat))continue;
                const float gap=pointSegmentDistance(seats[seat].x,seats[seat].y,
                    px,py,px+OBSTACLE_ROBOT_ENVELOPE_AXIS_FRONT_MM*cosf(h),
                    py+OBSTACLE_ROBOT_ENVELOPE_AXIS_FRONT_MM*sinf(h))-
                    OBSTACLE_MAX_WHEEL_HALF_WIDTH_MM-OBSTACLE_PILLAR_MOVEMENT_RADIUS_MM;
                if(!isfinite(gap)||gap<10 || (step && gap<fminf(40,previous[rear][seat])-0.25f) ||
                    (step==steps && travel<=-60 && gap<40))return false;
                previous[rear][seat]=fmaxf(previous[rear][seat],gap);
            }
        }
    }
    return true;
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
            // The walls are identical for every pillar at this pose. Keep
            // the same samples and 40-mm reserve without doing 24 wall scans.
            ObstacleClearanceSample walls{};
            if (!calculateClearanceAtPose(seats[0], x, y,
                    start.heading_deg, walls) || walls.wallMm <= 40.0f)
                return false;
            const float endX = x + OBSTACLE_ROBOT_ENVELOPE_AXIS_FRONT_MM*cosf(heading);
            const float endY = y + OBSTACLE_ROBOT_ENVELOPE_AXIS_FRONT_MM*sinf(heading);
            for (uint8_t seat = 0; seat < OBSTACLE_SEAT_COUNT; ++seat)
            {
                if (!cornerViewSeatMayBeOccupied(seat)) continue;
                const float clearance = pointSegmentDistance(seats[seat].x,seats[seat].y,
                    x,y,endX,endY) - OBSTACLE_MAX_WHEEL_HALF_WIDTH_MM -
                    OBSTACLE_PILLAR_MOVEMENT_RADIUS_MM;
                if (!isfinite(clearance) || clearance <= 40.0f)
                    return signedTravel<0 && cornerViewRetreatSafe(start,0,signedTravel);
            }
        }
    }
    return true;
}

bool ccwDiscoveryRouteSectionSafe(const PathPoint *route, uint16_t first, uint16_t last)
{
    const float extraNose = fmaxf(0.0f,
        OBSTACLE_FINAL_PARKING_STEERED_FRONT_MM - ROBOT_FRONT_FROM_REAR_AXLE_MM);
    for (uint16_t i = first; i <= last; ++i)
    {
        const uint16_t before = (i + pathLength - 1) % pathLength;
        const uint16_t after = (i + 1) % pathLength;
        const float dx = route[after].x - route[i].x, dy = route[after].y - route[i].y;
        const float beforeRad = atan2f(route[i].y - route[before].y,
                                      route[i].x - route[before].x);
        const float afterRad = atan2f(dy, dx);
        const float segment = hypotf(dx, dy);
        const float bend = wrap180((afterRad - beforeRad) * 180.0f / PI);
        if (segment < 3.0f || fabsf(pursuitServoForCurvature(
                bend * PI / (180.0f * segment))) > OBSTACLE_MAX_PURSUIT_STEERING_DEG)
            return false;
        for (float fraction : {0.0f, 0.5f})
        {
            PositionEstimate pose{};
            pose.x_mm = route[i].x + fraction * dx;
            pose.y_mm = route[i].y + fraction * dy;
            pose.heading_deg = afterRad * 180.0f / PI - (0.5f - fraction) * bend;
            if (!cornerViewSweepSafe(pose, extraNose) ||
                !parking_start_footprint::safe(pose.x_mm, pose.y_mm, pose.heading_deg, -1, false) ||
                !parking_start_footprint::safe(pose.x_mm, pose.y_mm, pose.heading_deg, 1, false))
                return false;
        }
    }
    return true;
}

void shapeCcwDiscoveryAfterTallSign(PathPoint *route)
{
    const int tall = ccwDiscoveryTallRedExitSeat();
    if (tall < 0) return;
    const float firstSign = seats[tall].pathDistanceMm;
    const uint16_t tallIndex = nearestPathIndex(baselinePath, seats[tall].x,
        seats[tall].y, 0, pathLength);
    const uint16_t first = tallIndex;
    const bool thirdKnown = seats[22].confirmed || seats[23].confirmed;
    const uint16_t last = tallIndex + (thirdKnown ? 17 : 11);
    if (last + 1 >= pathLength) return;

    // Carry the actual passing offset 150 mm past the pillar before gently
    // returning to the centre. A taper that starts at the pillar itself makes
    // pursuit turn in while the rear wheels and camera are still in the pass.
    memcpy(smoothingBuffer, route, sizeof(PathPoint) * pathLength);
    const float passingOffset = route[tallIndex].x - baselinePath[tallIndex].x;
    for (uint16_t i = first; i <= last; ++i)
    {
        const float d = baselinePath[i].distanceMm - firstSign;
        const float leave = smoothLaneBlend(d, 150.0f, 500.0f);
        const float rejoin = smoothLaneBlend(d, 550.0f, 850.0f);
        const float offset = route[i].x - baselinePath[i].x;
        route[i].x = baselinePath[i].x + passingOffset * (1.0f - leave) + offset * rejoin;
    }
    if (ccwDiscoveryRouteSectionSafe(route, first, last))
        Serial.println("[PATH] CCW tall sign: centred third-position approach");
    else
        memcpy(route + first, smoothingBuffer + first, sizeof(PathPoint) * (last - first + 1));
}

void shapeCcwDiscoveryReturnCorner(PathPoint *route)
{
    // Both passes already have mapped sides and active bypasses. Preserve
    // the initial departure and every other layout.
    if (ccwDiscoveryTallRedExitSeat() < 0 ||
        !seats[23].confirmed || !seats[23].injected || !seats[23].red ||
        !seats[1].confirmed || !seats[1].injected || seats[1].red)
        return;
    const uint16_t red = nearestPathIndex(baselinePath, seats[23].x, seats[23].y, 0, pathLength);
    const uint16_t green = nearestPathIndex(baselinePath, seats[1].x, seats[1].y, 0, pathLength);
    const uint16_t turnEnd = red + 8;
    if (red < 7 || green <= turnEnd || green + 9 >= pathLength) return;
    const uint16_t first = red - 7;
    const uint16_t last = green + 8;
    const float approachStart = baselinePath[first].distanceMm;
    const float turnStart = baselinePath[red].distanceMm;
    const float turnSpan = baselinePath[turnEnd].distanceMm - turnStart;
    const PathPoint greenPass = route[green];
    const float endY = greenPass.y - 30.0f;
    const float startX = route[red].x - 20.0f, startY = route[red].y;
    const float dy = endY - startY, dx = -1.05f * dy;
    const float endX = startX + dx;
    if (dy >= -170.0f || greenPass.x <= endX + 100.0f) return;

    memcpy(smoothingBuffer, route, sizeof(PathPoint) * pathLength);
    // Make room for the nose at RED, then finish one tangential turn
    // before reaching GREEN. The old overlapping tapers were still
    // changing heading at GREEN and immediately demanded countersteer.
    for (uint16_t i = first; i <= red; ++i)
        route[i].x -= 20.0f * smoothLaneBlend(
            baselinePath[i].distanceMm, approachStart, turnStart - 50.0f);
    for (uint16_t i = red + 1; i < turnEnd; ++i)
    {
        const float t = (baselinePath[i].distanceMm - turnStart) / turnSpan;
        const float u = 1.0f - t;
        const float blend = t * t * (3.0f - 2.0f * t);
        constexpr float handle = 0.55f; // tangents of a quarter-turn cubic
        route[i].x = startX + (blend - 3.0f * u * t * t * handle) * dx;
        route[i].y = startY + (blend + 3.0f * u * u * t * handle) * dy;
    }
    const float straightSpan = baselinePath[green].distanceMm - baselinePath[turnEnd].distanceMm;
    for (uint16_t i = turnEnd; i <= green; ++i)
    {
        const float t = (baselinePath[i].distanceMm - baselinePath[turnEnd].distanceMm) / straightSpan;
        route[i].x = endX + t * (greenPass.x - endX);
        route[i].y = endY;
    }
    // Leave GREEN parallel, then blend back to the existing start straight
    // without moving the seam.
    for (uint16_t i = green + 1; i < last; ++i)
    {
        const float d = baselinePath[i].distanceMm - baselinePath[green].distanceMm;
        route[i].y = endY + smoothLaneBlend(d, 50.0f, 400.0f) * (route[i].y - endY);
    }
    if (ccwDiscoveryRouteSectionSafe(route, first, last))
        Serial.println("[PATH] CCW tall sign: tangent RED-GREEN return");
    else
        memcpy(route + first, smoothingBuffer + first, sizeof(PathPoint) * (last - first + 1));
}

bool discoveryAimCommandSafe(const PositionEstimate &start, int steering)
{
    // Aiming never grants a new drive permission. Retain 40-mm clearance to
    // every still-possible pillar and wall over 80 mm of the actual CAD arc.
    const float radius = Ackermann::getTurnRadius(static_cast<float>(steering));
    const float k = fabsf(radius)>100000.0f ? 0.0f : -1.0f/radius;
    const float h = start.heading_deg*PI/180.0f;
    for (int step=0; step<=27; ++step)
    {
        const float travel = 80.0f*step/27.0f;
        PositionEstimate p = start;
        const float next = h+k*travel;
        if (fabsf(k)>1.0e-6f) {
            p.x_mm += (sinf(next)-sinf(h))/k;
            p.y_mm += (cosf(h)-cosf(next))/k;
        } else {
            p.x_mm += travel*cosf(h); p.y_mm += travel*sinf(h);
        }
        p.heading_deg = next*180.0f/PI;
        if (!cornerViewSweepSafe(p,0.0f)) return false;
    }
    return true;
}

bool earlyDiscoveryAimCommandSafe(const PositionEstimate &start, int steering)
{
    const float k=pursuitCurvatureForServo(steering);
    const float h=start.heading_deg*PI/180.0f;
    const float extraNose=fmaxf(0.0f,
        OBSTACLE_FINAL_PARKING_STEERED_FRONT_MM-ROBOT_FRONT_FROM_REAR_AXLE_MM);
    for (int step=0; step<=27; ++step)
    {
        const float travel=80.0f*step/27.0f, next=h+k*travel;
        PositionEstimate p=start;
        p.x_mm+=fabsf(k)>1.0e-6f ? (sinf(next)-sinf(h))/k : travel*cosf(h);
        p.y_mm+=fabsf(k)>1.0e-6f ? (cosf(h)-cosf(next))/k : travel*sinf(h);
        p.heading_deg=next*180.0f/PI;
        // Union of the original front/rear capsule and its forward extension
        // encloses the measured135mm nose without removing the old rear reserve.
        if (!cornerViewSweepSafe(p,extraNose)) return false;
    }
    return true;
}

void applyLap1DiscoveryAim(PathPoint &target, const PositionEstimate &pose)
{
    if (traversingInjectedAvoidance(baselinePath[progressIndex].distanceMm))
    {
        lastDiscoveryTargetNudgeDeg=0.0f;
        lastDiscoveryNudgeUpdateMs=millis();
        discoveryScanStation=discoveryScanSide=-1;
        return;
    }
    const PathPoint baselineTarget=target;
    const float previousNudge=lastDiscoveryTargetNudgeDeg;
    const float dt=lastDiscoveryNudgeUpdateMs==0 ? 0.02f :
        fminf(0.25f,(millis()-lastDiscoveryNudgeUpdateMs)/1000.0f);
    applyDiscoveryTargetNudge(target,pose);
    // CCW returned633/648/655/656: the last unresolved place stayed at the
    // image edge while ordinary target rotation was already saturated.
    // Preserve CW, the complete bypass guard and the normal two-place scan.
    float angle=lastDiscoveryTargetNudgeDeg;
    if (completedLaps==0 && routeTurnSign>0 && discoveryScanStation>=0 &&
        discoveryScanSide>=0 && fabsf(angle)>=44.0f)
    {
        float bearing=0.0f,range=0.0f;
        seatCameraGeometry(discoveryScanStation*COURSE_SEATS_PER_STATION+discoveryScanSide,
            pose,bearing,range);
        const float viewLimit=OBSTACLE_CAMERA_HORIZONTAL_FOV_DEG*
            OBSTACLE_DISCOVERY_FOV_FRACTION-OBSTACLE_DISCOVERY_CLEAR_FOV_MARGIN_DEG;
        if (range>=300.0f && range<=750.0f && fabsf(bearing)>viewLimit && bearing*angle>0.0f)
        {
            const float desired=copysignf(OBSTACLE_LOOK_MAX_TARGET_NUDGE_DEG+5.0f,bearing);
            const float change=OBSTACLE_LOOK_NUDGE_SLEW_DEG_S*dt;
            const float boosted=previousNudge+clampFloat(desired-previousNudge,-change,change);
            if (fabsf(boosted)>fabsf(angle)) angle=boosted;
        }
    }
    if (fabsf(angle)<=OBSTACLE_LOOK_MAX_TARGET_NUDGE_DEG) return;
    const float a=angle*PI/180.0f,h=pose.heading_deg*PI/180.0f;
    const float x=baselineTarget.x-pose.x_mm,y=baselineTarget.y-pose.y_mm;
    PathPoint candidate=baselineTarget;
    candidate.x=pose.x_mm+x*cosf(a)-y*sinf(a);
    candidate.y=pose.y_mm+x*sinf(a)+y*cosf(a);
    const float dx=candidate.x-pose.x_mm,dy=candidate.y-pose.y_mm;
    const float k=2.0f*(-dx*sinf(h)+dy*cosf(h))/fmaxf(1.0f,dx*dx+dy*dy);
    const int steering=static_cast<int>(clampFloat(pursuitServoForCurvature(k),
        -OBSTACLE_MAX_PURSUIT_STEERING_DEG,OBSTACLE_MAX_PURSUIT_STEERING_DEG));
    // Recheck the extra angle also while it slews back after losing eligibility.
    if (completedLaps==0 && routeTurnSign>0 && earlyDiscoveryAimCommandSafe(pose,steering))
    { lastDiscoveryTargetNudgeDeg=angle; target=candidate; }
    else
    {
        lastDiscoveryTargetNudgeDeg=clampFloat(lastDiscoveryTargetNudgeDeg,
            -OBSTACLE_LOOK_MAX_TARGET_NUDGE_DEG,OBSTACLE_LOOK_MAX_TARGET_NUDGE_DEG);
        const float normal=lastDiscoveryTargetNudgeDeg*PI/180.0f;
        target.x=pose.x_mm+x*cosf(normal)-y*sinf(normal);
        target.y=pose.y_mm+x*sinf(normal)+y*cosf(normal);
    }
}

float cornerViewReverseDistance(const PositionEstimate &pose, uint8_t station,
                               float minimumViewRangeMm = OBSTACLE_DISCOVERY_VIEW_MIN_MM)
{
    const uint8_t first=station*COURSE_SEATS_PER_STATION;
    const bool needFirst=!discoveryStations[station].seatObservedClear[0];
    const bool needSecond=!discoveryStations[station].seatObservedClear[1];
    const float h=pose.heading_deg*PI/180.0f;
    const float preferred = needFirst && needSecond ? 180.0f : 120.0f;
    const auto viewUsable = [&](float distance, bool &preferredView)
    {
        PositionEstimate view=pose;
        view.x_mm-=distance*cosf(h); view.y_mm-=distance*sinf(h);
        float bearing0=0,range0=0,bearing1=0,range1=0;
        seatCameraGeometry(first,view,bearing0,range0);
        seatCameraGeometry(first+1,view,bearing1,range1);
        preferredView=(!needFirst || (range0>=300.0f && fabsf(bearing0)<=22.0f)) &&
                      (!needSecond || (range1>=300.0f && fabsf(bearing1)<=22.0f));
        return (!needFirst || seatComfortablyVisible(first,view)) &&
            (!needSecond || seatComfortablyVisible(first+1,view)) &&
            (!needFirst || range0 >= minimumViewRangeMm) &&
            (!needSecond || range1 >= minimumViewRangeMm) &&
            cornerViewSweepSafe(pose,-distance-20.0f);
    };
    float chosen=0.0f;
    for (float distance=60.0f; distance<=220.0f; distance+=10.0f)
    {
        bool preferredView=false;
        if (viewUsable(distance,preferredView))
        {
            chosen=distance;
            if (distance>=preferred && preferredView) break;
        }
    }
    return chosen;
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

bool cornerViewSteeredCommandSafe(PositionEstimate pose, int steering, int direction,
                                  int previewMm=15)
{
    if(direction<0 && !cornerViewSweepSafe(pose,0))
        return cornerViewRetreatSafe(pose,steering,-static_cast<float>(previewMm));
    const float radius=Ackermann::getTurnRadius(static_cast<float>(steering));
    const float curvature=fabsf(radius)>100000.0f ? 0.0f : -1.0f/radius;
    for (int mm=0; mm<=previewMm; ++mm)
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

// A newly learned bypass starts behind the old stop pose. Returning to that
// pose discards the approach needed to reach the correct side (log521).
// Rebase only this recovery's progress, then check the actual CAD pursuit from
// the stopped observation pose. Ordinary lap progress remains forward-only.
bool discoveryResumePoseSafe(const PositionEstimate &pose)
{
    if (!isfinite(pose.x_mm) || !isfinite(pose.y_mm) ||
        !isfinite(pose.heading_deg)) return false;
    const float h = pose.heading_deg * PI / 180.0f;
    const float c = cosf(h), s = sinf(h);
    for (uint8_t rear = 0; rear < 2; ++rear)
    {
        const float offset = rear ? OBSTACLE_MAX_WHEEL_HALF_WIDTH_MM : 0;
        const float x = pose.x_mm-offset*c, y = pose.y_mm-offset*s;
        // Walls do not depend on the pillar seat. Computing all wall/corner
        // distances again for every seat blocked gyro service for~815ms.
        ObstacleClearanceSample walls{};
        if (!calculateClearanceAtPose(seats[0], x, y, pose.heading_deg, walls) ||
            walls.wallMm <= 30) return false;
        const float endX = x+OBSTACLE_ROBOT_ENVELOPE_AXIS_FRONT_MM*c;
        const float endY = y+OBSTACLE_ROBOT_ENVELOPE_AXIS_FRONT_MM*s;
        for (uint8_t seat = 0; seat < OBSTACLE_SEAT_COUNT; ++seat)
        {
            if (!seats[seat].confirmed &&
                (seats[seat ^ 1U].confirmed || !cornerViewSeatMayBeOccupied(seat)))
                continue;
            // Identical capsule/pillar expression to calculateClearanceAtPose;
            // retain every possible seat and the same30mm reserve.
            const float clearance = pointSegmentDistance(seats[seat].x, seats[seat].y,
                x, y, endX, endY) - OBSTACLE_MAX_WHEEL_HALF_WIDTH_MM -
                OBSTACLE_PILLAR_MOVEMENT_RADIUS_MM;
            if (!isfinite(clearance) || clearance <= 30) return false;
        }
    }
    return true;
}

bool discoveryResumeFromViewSafe(const PositionEstimate &start, uint16_t &resumeIndex)
{
    if (pathLength < 2 || !isfinite(start.x_mm) || !isfinite(start.y_mm) ||
        !isfinite(start.heading_deg)) return false;
    const uint16_t saved = progressIndex;
    const uint16_t back = 8;
    uint16_t simulatedIndex = nearestPathIndex(livePath, start.x_mm, start.y_mm,
        (saved + pathLength - back) % pathLength,
        min(static_cast<uint16_t>(pathLength), static_cast<uint16_t>(back + OBSTACLE_PATH_PROGRESS_WINDOW)));
    resumeIndex = simulatedIndex;
    PositionEstimate pose = start;
    bool safe = true;
    for (float travel = 0; travel <= 700 && safe; travel += 5)
    {
        simulatedIndex = nearestPathIndex(livePath, pose.x_mm, pose.y_mm,
            simulatedIndex, OBSTACLE_PATH_PROGRESS_WINDOW);
        float h = pose.heading_deg * PI / 180.0f;
        safe = discoveryResumePoseSafe(pose);
        if (!safe) break;
        progressIndex = simulatedIndex;
        float lookahead = firstLapLookahead(cappedPathSpeed(
            recordedLapSpeed(livePath[simulatedIndex].speedMmS)));
        if (nearCorner(baselinePath[simulatedIndex].distanceMm))
            lookahead *= OBSTACLE_LOOKAHEAD_CORNER_SCALE;
        const PathPoint target = findLookahead(livePath, pose, lookahead);
        const float dx = target.x-pose.x_mm, dy = target.y-pose.y_mm;
        const float forward = dx*cosf(h)+dy*sinf(h);
        const float curvature = 2*(-dx*sinf(h)+dy*cosf(h))/fmaxf(1,dx*dx+dy*dy);
        const float command = pursuitServoForCurvature(curvature);
        if (!isfinite(command) || forward <= 1) { safe = false; break; }
        const float k = pursuitCurvatureForServo(static_cast<int>(clampFloat(command,
            -OBSTACLE_MAX_PURSUIT_STEERING_DEG, OBSTACLE_MAX_PURSUIT_STEERING_DEG)));
        const float next = h+k*5;
        if (fabsf(k)>1e-6f)
        { pose.x_mm+=(sinf(next)-sinf(h))/k; pose.y_mm+=(cosf(h)-cosf(next))/k; }
        else { pose.x_mm+=5*cosf(h); pose.y_mm+=5*sinf(h); }
        pose.heading_deg=next*180.0f/PI;
    }
    progressIndex = saved;
    return safe;
}

bool resumeDiscoveryFromView(const PositionEstimate &pose)
{
    const uint32_t startedUs = micros();
    uint16_t resumeIndex = 0;
    if (!discoveryResumeFromViewSafe(pose, resumeIndex)) return false;
    progressIndex = resumeIndex;
    cornerViewPhase = CORNER_VIEW_IDLE;
    discoveryHolding = discoveryBlocked = false;
    discoveryHoldStation = discoveryBlockedStation = -1;
    discoveryHoldStartMs = 0;
    lastDiscoveryTargetNudgeDeg = 0;
    lastDiscoveryNudgeUpdateMs = millis();
    servo_disabled = false;
    Serial.print("[CORNER VIEW] Resolved; resume from observation pose index=");
    Serial.print(resumeIndex);
    Serial.print(" check_us="); Serial.println(micros()-startedUs);
    return true;
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

void reobserveCornerView(const char *reason,const PositionEstimate &pose,float encoder)
{
    // A rejected old motion model is not failed perception. Brake first;
    // then obtain fresh evidence and preflight every new movement from the
    // actual settled pose. Never return blindly to the old origin.
    holdCornerViewCentered();
    cornerViewOrigin=pose;
    cornerViewOriginEncoder=encoder;
    cornerViewDistanceMm=cornerViewMeasuredReverseMm=0;
    cornerViewStoppedFrames=0;
    discoveryStations[cornerViewStation].clearFrames[0]=0;
    discoveryStations[cornerViewStation].clearFrames[1]=0;
    cornerViewPhase=CORNER_VIEW_REBASE_BRAKE;
    cornerViewPhaseMs=millis();
    Serial.print("[CORNER VIEW] Brake and reobserve: ");Serial.println(reason);
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
    const float curvature = pursuitCurvatureForServo(steering);
    const float angle = start.heading_deg * PI / 180.0f;
    if (fabsf(curvature) < 1.0e-6f)
    {
        result.x_mm += travel * cosf(angle);
        result.y_mm += travel * sinf(angle);
        return result;
    }
    const float end = angle + curvature * travel;
    result.x_mm += (sinf(end) - sinf(angle)) / curvature;
    result.y_mm += (cosf(angle) - cosf(end)) / curvature;
    result.heading_deg = wrap180(end * 180.0f / PI);
    return result;
}

bool cornerViewArcSafe(const PositionEstimate &start, int steering, float travel)
{
    if ((steering != 0 && abs(steering) != 20) ||
        !isfinite(travel) || fabsf(travel) > 160) return false;
    const float extraNose = fmaxf(0.0f,
        OBSTACLE_FINAL_PARKING_STEERED_FRONT_MM - ROBOT_FRONT_FROM_REAR_AXLE_MM);
    const float offsets[] = {0.0f, OBSTACLE_MAX_WHEEL_HALF_WIDTH_MM, -extraNose};
    const uint8_t offsetCount = travel > 0.0f ? 3 : 2;
    const unsigned steps = static_cast<unsigned>(ceilf(fabsf(travel) / 3));
    for (unsigned step = 0; step <= steps; ++step)
    {
        const PositionEstimate pose = cornerViewArcPose(start, steering,
            steps ? travel * step / steps : 0);
        const float h = pose.heading_deg * PI / 180.0f;
        const float c = cosf(h), s = sinf(h);
        for (uint8_t offset = 0; offset < offsetCount; ++offset)
        {
            const float x = pose.x_mm - offsets[offset]*c;
            const float y = pose.y_mm - offsets[offset]*s;
            ObstacleClearanceSample walls{};
            if (!calculateClearanceAtPose(seats[0],x,y,pose.heading_deg,walls) || walls.wallMm<=40)
                return travel<0 && cornerViewRetreatSafe(start,steering,travel);
            const float endX = x + OBSTACLE_ROBOT_ENVELOPE_AXIS_FRONT_MM*c;
            const float endY = y + OBSTACLE_ROBOT_ENVELOPE_AXIS_FRONT_MM*s;
            for (uint8_t seat = 0; seat < OBSTACLE_SEAT_COUNT; ++seat)
            {
                if (!cornerViewSeatMayBeOccupied(seat)) continue;
                const float gap = pointSegmentDistance(seats[seat].x,seats[seat].y,x,y,endX,endY) -
                    OBSTACLE_MAX_WHEEL_HALF_WIDTH_MM - OBSTACLE_PILLAR_MOVEMENT_RADIUS_MM;
                if (!isfinite(gap) || gap<=40)
                    return travel<0 && cornerViewRetreatSafe(start,steering,travel);
            }
        }
    }
    return true;
}

void collectCornerViewFrame(const PositionEstimate &pose, bool newFrame)
{
    if (!newFrame) return;
    lastDiscoveryObservation = obstacle_path_observe(getDiscoveryObstacle());
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
            Serial.print(" bounds/centroid=");
            Serial.print(raw->minX); Serial.print("/"); Serial.print(raw->minY);
            Serial.print("/"); Serial.print(raw->maxX); Serial.print("/");
            Serial.print(raw->maxY); Serial.print("/"); Serial.print(raw->centerX);
        }
        Serial.println();
    }
    logDiscoveryTrace(cornerViewStation, "reverse_observe", false);
}

bool beginCornerViewExtra(const PositionEstimate &pose)
{
    const DiscoveryStation &coverage = discoveryStations[cornerViewStation];
    uint8_t unresolvedMask = 0;
    for (uint8_t side = 0; side < COURSE_SEATS_PER_STATION; ++side)
        if (!coverage.seatObservedClear[side])
            unresolvedMask |= static_cast<uint8_t>(1U << side);

    // Both directions stay within the original scan area after rebasing.
    const float backedAway = cornerViewSearchStartEncoder-get_distance();
    const float remaining = 420.0f - backedAway;
    const uint32_t now=millis();
    if (now-cornerViewLastSearchMs<250U) return false;
    cornerViewLastSearchMs=now;
    int selectedSteering = 0;
    float selectedDistance = 0.0f;
    float selectedCost = INFINITY;
    const auto viewCost = [&](const PositionEstimate &view) {
        float cost = 0.0f;
        for (uint8_t side=0; side<COURSE_SEATS_PER_STATION; ++side)
        {
            if (unresolvedMask && !(unresolvedMask & (1U << side))) continue;
            const uint8_t seat=cornerViewStation*COURSE_SEATS_PER_STATION+side;
            float bearing=0.0f, range=0.0f;
            seatCameraGeometry(seat,view,bearing,range);
            cost += (seatComfortablyVisible(seat,view) ? 0.0f : 40.0f) +
                0.7f*fabsf(bearing) + 0.09f*fabsf(range-365.0f);
        }
        return cost;
    };
    const float currentCost=viewCost(pose);
    const bool resolved = stationDirectlyResolved(cornerViewStation);
    for (int steering : {-20,20})
    {
        const uint8_t bit = steering==0 ? 1U : (steering<0 ? 2U : 4U);
        if (cornerViewExtraTriedMask & bit) continue;
        for (float distance : {130.0f,110.0f,90.0f,70.0f})
        {
            if (distance+20.0f>remaining) continue;
            const PositionEstimate view=cornerViewArcPose(pose,steering,-distance);
            const float cost=viewCost(view)+(130.0f-distance)*0.08f;
            // Once direct evidence is known, a retreat can still provide
            // joining room. During discovery require a meaningfully new view.
            if (cost>=selectedCost ||
                (!resolved && cost>currentCost+10.0f) ||
                !cornerViewArcSafe(pose,steering,-distance-20.0f)) continue;
            selectedCost=cost;
            selectedDistance=distance;
            selectedSteering=steering;
        }
    }
    int direction = -1;
    if (selectedDistance==0.0f && !resolved && !cornerViewForwardTried && backedAway>=120.0f)
    {
        // A distant look can confirm colour but cannot prove that a place is
        // empty. Leave 30 mm before the old stop, including braking allowance.
        direction=1;
        selectedCost=currentCost-2.0f;
        for (int steering : {0,-20,20})
        {
            for (float distance : {70.0f,100.0f,130.0f})
            {
                if (distance+50.0f>backedAway) continue;
                const PositionEstimate view=cornerViewArcPose(pose,steering,distance);
                const float cost=viewCost(view)+distance*0.04f+abs(steering)*0.1f;
                if (cost>=selectedCost) continue;
                bool closerView=false;
                for (uint8_t side=0; side<COURSE_SEATS_PER_STATION; ++side)
                {
                    if (!(unresolvedMask & (1U << side))) continue;
                    const uint8_t seat=cornerViewStation*COURSE_SEATS_PER_STATION+side;
                    float bearing,range;
                    seatCameraGeometry(seat,view,bearing,range);
                    if (range>=OBSTACLE_DISCOVERY_VIEW_MIN_MM &&
                        range<=OBSTACLE_DISCOVERY_CLEAR_MAX_MM-15.0f && fabsf(bearing)<=18.0f)
                        closerView=true;
                }
                if (!closerView || !cornerViewArcSafe(pose,steering,distance+20.0f)) continue;
                selectedCost=cost;
                selectedDistance=distance;
                selectedSteering=steering;
            }
        }
    }
    if (selectedDistance==0.0f) return false;

    cornerViewExtraDistanceMm=selectedDistance;
    cornerViewExtraSteering=selectedSteering;
    cornerViewExtraDirection=direction;
    if (direction>0) cornerViewForwardTried=true;
    else cornerViewExtraTriedMask |= selectedSteering<0 ? 2U : 4U;
    holdCornerViewSteering(selectedSteering);
    cornerViewPhase=CORNER_VIEW_EXTRA_SETTLE;
    cornerViewPhaseMs=millis();
    if (direction>0) Serial.println("[CORNER VIEW] Closer forward viewpoint");
    Serial.print("[CORNER VIEW] New viewpoint cycle/steering/mm/unresolved=");
    Serial.print(cornerViewSearchCycle); Serial.print("/");
    Serial.print(selectedSteering); Serial.print("/");
    Serial.print(selectedDistance,0); Serial.print("/");
    Serial.println(unresolvedMask);
    return true;
}

void updateCornerViewExtra()
{
    const PositionEstimate pose=get_position_struct();
    const float encoder=get_distance();
    const uint32_t age=millis()-cornerViewPhaseMs;
    const int direction=cornerViewExtraDirection;
    if (!isfinite(encoder) || !isfinite(pose.x_mm) || !isfinite(pose.y_mm) ||
        !isfinite(pose.heading_deg)) { lockCornerView("extra invalid pose"); return; }

    if (cornerViewPhase==CORNER_VIEW_EXTRA_SETTLE)
    {
        holdCornerViewSteering(cornerViewExtraSteering);
        if (age<300) return;
        if (!cornerViewArcSafe(pose,cornerViewExtraSteering,direction*(cornerViewExtraDistanceMm+20.0f)))
        { reobserveCornerView("extra settled preflight",pose,encoder); return; }
        cornerViewExtraOrigin=pose;
        cornerViewExtraStartEncoder=encoder;
        cornerViewPhase=CORNER_VIEW_EXTRA_DRIVE;
        cornerViewPhaseMs=millis();
        logCornerViewPose("extra_origin",pose,encoder);
        return;
    }

    const float travelled=direction*(encoder-cornerViewExtraStartEncoder);
    if (travelled < -5.0f || travelled>cornerViewExtraDistanceMm+20.0f || age>4500)
    { lockCornerView("extra viewpoint travel/time/direction"); return; }
    const PositionEstimate expected=cornerViewArcPose(
        cornerViewExtraOrigin,cornerViewExtraSteering,direction*travelled);
    const float positionError=hypotf(pose.x_mm-expected.x_mm,pose.y_mm-expected.y_mm);
    const float headingError=fabsf(wrap180(pose.heading_deg-expected.heading_deg));
    if (positionError>25.0f || headingError>12.0f)
    { lockCornerView("extra arc tracking limit"); return; }
    if (travelled>=cornerViewExtraDistanceMm)
    {
        // The next search starts here. A blocked or tracking-failed arc
        // retains its tried bit; a completed movement opens new viewpoints.
        cornerViewExtraTriedMask=0;
        if (cornerViewSearchCycle<UINT16_MAX) ++cornerViewSearchCycle;
        reobserveCornerView("new viewpoint reached",pose,encoder);
        return;
    }
    if (positionError>15.0f || headingError>5.0f ||
        !cornerViewArcSafe(pose,cornerViewExtraSteering,direction*15.0f))
    { reobserveCornerView("extra tracking/swept clearance",pose,encoder); return; }
    set_steering(cornerViewExtraSteering);
    servo_disabled=false;
    const float cruise = direction<0 && cornerViewExtraDistanceMm >= 110.0f
        ? OBSTACLE_CORNER_VIEW_ARC_CRUISE_MM_S : OBSTACLE_CORNER_VIEW_REVERSE_SPEED_MM_S;
    int speed = DriveProfile::approachSpeed(cornerViewExtraDistanceMm - travelled, cruise,
        OBSTACLE_CORNER_VIEW_REVERSE_SPEED_MM_S, 22.0f, 75.0f);
    if (speed > OBSTACLE_CORNER_VIEW_REVERSE_SPEED_MM_S &&
        !cornerViewArcSafe(pose, cornerViewExtraSteering, direction*25.0f))
        speed = OBSTACLE_CORNER_VIEW_REVERSE_SPEED_MM_S;
    set_speed(direction*speed);
}

void updateCornerView(bool newCameraFrame)
{
    //620: the mismatched arc was still braking when its pose became the
    //straight-observation reference. Another3.17deg of settling then locked
    //the next tick. Finish braking before choosing the new reference.
    if(cornerViewPhase==CORNER_VIEW_REBASE_BRAKE)
    {
        holdCornerViewCentered();
        const PositionEstimate pose=get_position_struct();
        const float encoder=get_distance();
        if(!isfinite(encoder)||!isfinite(pose.x_mm)||!isfinite(pose.y_mm)||
           !isfinite(pose.heading_deg)||
           hypotf(pose.x_mm-cornerViewOrigin.x_mm,pose.y_mm-cornerViewOrigin.y_mm)>20 ||
           fabsf(wrap180(pose.heading_deg-cornerViewOrigin.heading_deg))>12)
        {lockCornerView("reobserve braking limit");return;}
        if(millis()-cornerViewPhaseMs<300)return;
        cornerViewOrigin=pose;cornerViewOriginEncoder=encoder;
        cornerViewDistanceMm=cornerViewMeasuredReverseMm=0;
        cornerViewStoppedFrames=0;
        discoveryStations[cornerViewStation].clearFrames[0]=0;
        discoveryStations[cornerViewStation].clearFrames[1]=0;
        cornerViewPhase=CORNER_VIEW_OBSERVE;cornerViewPhaseMs=millis();
        logCornerViewPose("reobserve_settled",pose,encoder);
        return;
    }
    if (cornerViewPhase == CORNER_VIEW_EXTRA_SETTLE || cornerViewPhase == CORNER_VIEW_EXTRA_DRIVE)
    {
        updateCornerViewExtra();
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
            if(fabsf(wrap180(pose.heading_deg-cornerViewOrigin.heading_deg))<=12.0f &&
               cross<=25.0f)
                reobserveCornerView("straight heading/cross-track mismatch",pose,encoder);
            else
                lockCornerView("heading/cross-track limit");
            return;
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
        cornerViewProgressEncoder = encoder;
        cornerViewSearchStartEncoder = encoder;
        cornerViewDistanceMm = cornerViewReverseDistance(pose,cornerViewStation,
            cornerViewRequestedMinRange);
        if (cornerViewDistanceMm == 0)
        {
            // Motion is allowed only after a swept preflight, but lack of one
            // must not turn a perception miss into a permanent dead stop.
            // Continue evaluating fresh frames and retry both safe parallax
            // choices from this pose.
            cornerViewDistanceMm = 0.0f;
            cornerViewPhase = CORNER_VIEW_OBSERVE;
            cornerViewPhaseMs = millis();
            cornerViewStoppedFrames = 0;
            discoveryStations[cornerViewStation].clearFrames[0] = 0;
            discoveryStations[cornerViewStation].clearFrames[1] = 0;
            Serial.println("[CORNER VIEW] No safe straight reverse; active observation continues");
            logCornerViewPose("stationary_origin", pose, encoder);
            return;
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
            cornerViewOriginEncoder - encoder > cornerViewDistanceMm + 20)
        {
            lockCornerView("reverse travel/direction limit"); return;
        }
        if (cornerViewOriginEncoder - encoder >= cornerViewDistanceMm-2.0f)
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
        // Each healthy interval can advance the same retreat after a sensor
        // restart. Time out lack of progress, rather than the entire move.
        if (cornerViewProgressEncoder-encoder>=5.0f)
        {
            cornerViewProgressEncoder=encoder;
            cornerViewPhaseMs=millis();
        }
        else if (age>6000)
        {
            reobserveCornerView("reverse without progress",pose,encoder);
            return;
        }
        {
            const int steering=cornerViewStraightSteering(pose,-1);
            if (!cornerViewSteeredCommandSafe(pose,steering,-1))
            { reobserveCornerView("reverse command clearance",pose,encoder); return; }
            const float remaining=cornerViewDistanceMm-(cornerViewOriginEncoder-encoder);
            int speed=DriveProfile::approachSpeed(remaining,125,
                OBSTACLE_CORNER_VIEW_REVERSE_SPEED_MM_S,35.0f,130.0f);
            if (speed>OBSTACLE_CORNER_VIEW_REVERSE_SPEED_MM_S &&
                !cornerViewSteeredCommandSafe(pose,steering,-1,32))
                speed=OBSTACLE_CORNER_VIEW_REVERSE_SPEED_MM_S;
            servo_disabled = false;
            set_steering(steering);
            set_speed(-speed);
        }
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
        if (!stationDirectlyResolved(cornerViewStation) && newCameraFrame &&
            cornerViewRequestedMinRange < 300.0f &&
            cornerViewMinimumRange(getLargestObstacle()) >= 300.0f &&
            !obstacle_blob_valid_for_acquisition(getLargestObstacle()))
        {
            cornerViewRequestedMinRange=300.0f;
            const float deeper=cornerViewReverseDistance(cornerViewOrigin,
                cornerViewStation,cornerViewRequestedMinRange);
            if (deeper>cornerViewDistanceMm &&
                cornerViewSearchStartEncoder-cornerViewOriginEncoder+deeper+20.0f<=420.0f &&
                cornerViewSweepSafe(pose,-(deeper-cornerViewMeasuredReverseMm+20.0f)))
            {
                cornerViewDistanceMm=deeper;
                cornerViewProgressEncoder=encoder;
                cornerViewPhase=CORNER_VIEW_REVERSE;
                cornerViewPhaseMs=millis();
                Serial.print("[CORNER VIEW] Near silhouette; extend straight reverse total_mm=");
                Serial.println(deeper,0);
                return;
            }
        }
        if (age < 600 || (age < 1800 &&
            (!stationDirectlyResolved(cornerViewStation) || cornerViewStoppedFrames < 2))) return;
        // Once an active recovery has started, resume only from evidence for
        // this exact station: both seats observed CLEAR or either colour/seat
        // confirmed. Section inference alone is not enough for this fallback.
        cornerViewCanResume = stationDirectlyResolved(cornerViewStation) &&
            cornerViewStoppedFrames >= 2;
        if (cornerViewCanResume)
        {
            if (resumeDiscoveryFromView(pose)) return;
            // Obtain more approach distance within the already bounded and
            // swept straight reverse, rather than returning towards the sign.
            const float deeper = cornerViewDistanceMm + 40;
            if (deeper <= 220 &&
                cornerViewSearchStartEncoder-cornerViewOriginEncoder+deeper+20.0f<=420.0f &&
                cornerViewSweepSafe(pose,
                    -(deeper-cornerViewMeasuredReverseMm+20)))
            {
                cornerViewDistanceMm = deeper;
                cornerViewProgressEncoder=encoder;
                cornerViewPhase = CORNER_VIEW_REVERSE;
                cornerViewPhaseMs = millis();
                Serial.println("[CORNER VIEW] Resolved; extend approach before route join");
                return;
            }
            //627: colour is known but straight retreat exhausted its bound.
            //Try existing swept parallax; never accept the unsafe join.
            if(beginCornerViewExtra(pose))return;
            const uint32_t now=millis();
            if(now-cornerViewLastNoSafeTraceMs>=1000UL)
            {
                cornerViewLastNoSafeTraceMs=now;
                Serial.println("[CORNER VIEW] Resolved; retrying checked approach viewpoints");
            }
            return;
        }
        if (!cornerViewCanResume)
        {
            if (beginCornerViewExtra(pose)) return;
            // No currently safe movement is available. Keep consuming camera
            // frames and re-run the complete preflight rather than returning
            // and locking only because the station is still UNKNOWN.
            const uint32_t now = millis();
            if (now - cornerViewLastNoSafeTraceMs >= 1000UL)
            {
                cornerViewLastNoSafeTraceMs = now;
                Serial.println("[CORNER VIEW] Unresolved; observing and retrying safe viewpoints");
            }
            return;
        }
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
            { reobserveCornerView("return command clearance",pose,encoder); return; }
            servo_disabled = false;
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
            // Defensive recovery for an interrupted/legacy return state: once
            // back at the checked origin, restart active observation instead
            // of converting missing evidence into a permanent lock.
            cornerViewOrigin = pose;
            cornerViewOriginEncoder = encoder;
            cornerViewDistanceMm = 0.0f;
            cornerViewMeasuredReverseMm = 0.0f;
            cornerViewPhase = CORNER_VIEW_OBSERVE;
            cornerViewPhaseMs = millis();
            cornerViewStoppedFrames = 0;
            Serial.println("[CORNER VIEW] Returned unresolved; active search restarts");
            return;
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
        if (parkingCcwShortStart)
        {
            stop(false);
            parkingEntryActive=false;parkingEntryScouting=false;parkingEntryObserving=true;
            parkingEntryNeedsJoinView=true;parkingCcwRefineRecovery=true;
            parkingCcwRefineUsed=false;parkingEntryObserveStartMs=millis();
            Serial.println("[CCW START] Join refused; search another checked view");
            return;
        }
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
    parkingEntryNeedsJoinView=false;
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
            // The stopped scan must also allow the opposite wheel sweep at
            // connector launch. 551/553/555 passed the scan-only footprint,
            // then rejected every connector at travel=0.
            if (parkingCcwFirstEdgeStart && travel >= travelMm-4.0f &&
                !parkingStartFootprintSafe(x,y,heading*180.0f/PI,-routeTurnSign,5.0f))
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

bool preflightCcwFrontView(const PositionEstimate &start, float travelMm,
                           int direction=1, int steering=-50)
{
    if (!parkingCcwShortStart || routeTurnSign != 1 ||
        !isfinite(travelMm) || travelMm <= 0.0f || travelMm > 35.0f) return false;
    const float k = pursuitCurvatureForServo(steering);
    for (int error = -1; error <= 1; ++error)
    {
        const float h = (start.heading_deg + error) * PI / 180.0f;
        for (float travel = 0.0f; travel <= travelMm + 8.0f; travel += 2.0f)
        {
            const float next = h + k * direction * travel;
            const float x = start.x_mm + (sinf(next) - sinf(h)) / k;
            const float y = start.y_mm + (cosf(h) - cosf(next)) / k;
            const float heading = next * 180.0f / PI;
            if (!parkingStartFootprintSafe(x, y, heading, -1, 5.0f) ||
                !parkingStartFootprintSafe(x, y, heading, 1, 5.0f)) return false;
            for (uint8_t seat = 1; seat < 6; seat += 2)
            {
                ObstacleClearanceSample clearance{};
                if (!calculateClearanceAtPose(seats[seat], x, y, heading, clearance) ||
                    clearance.wallMm <= 10.0f || clearance.pillarMm <= 10.0f)
                    return false;
            }
        }
    }
    return true;
}

float ccwFrontViewExtensionMm(const PositionEstimate &pose)
{
    if (!parkingCcwShortStart || routeTurnSign != 1) return 0.0f;
    float bearing = 0.0f, range = 0.0f;
    seatCameraGeometry(5, pose, bearing, range);
    if (!parkingCcwRefineRecovery)
    {
        // Keep the first successful refinement and its known-colour joins.
        // Broader parallax alternatives are only needed after a failed view.
        if (fabsf(bearing)<=10.0f) return 0.0f;
        const float k=pursuitCurvatureForServo(-50),h=pose.heading_deg*PI/180.0f;
        for (float extra=5.0f;extra<=35.0f;extra+=5.0f)
        {
            const float next=h+k*extra;
            PositionEstimate view=pose;
            view.x_mm+=(sinf(next)-sinf(h))/k;view.y_mm+=(cosf(h)-cosf(next))/k;
            view.heading_deg=next*180.0f/PI;
            seatCameraGeometry(5,view,bearing,range);
            if (fabsf(bearing)<=10.0f && seatComfortablyVisible(5,view) && preflightCcwFrontView(pose,extra))
            { parkingCcwRefineDirection=1;parkingCcwRefineSteering=-50;return extra; }
        }
        return 0.0f;
    }
    const float target = fabsf(bearing)>5.0f ? 0.0f : (parkingCcwViewAlternate ? -8.0f:8.0f);
    parkingCcwViewAlternate=!parkingCcwViewAlternate;
    float bestError=fabsf(bearing-target)-2.0f, bestTravel=0.0f;
    const float h = pose.heading_deg * PI / 180.0f;
    // Missing evidence requests another small, fully checked parallax view.
    // Never extrapolate a fixed reverse turn towards the pink pieces.
    for (int direction:{1,-1}) for (int steering:{-50,50})
    for (float extra = 10.0f; extra <= 35.0f; extra += 5.0f)
    {
        const float k=pursuitCurvatureForServo(steering);
        const float next = h + k * direction * extra;
        PositionEstimate view = pose;
        view.x_mm += (sinf(next) - sinf(h)) / k;
        view.y_mm += (cosf(h) - cosf(next)) / k;
        view.heading_deg = next * 180.0f / PI;
        seatCameraGeometry(5, view, bearing, range);
        const float error=fabsf(bearing-target)+0.02f*extra;
        if (error<bestError && seatComfortablyVisible(5, view) &&
            preflightCcwFrontView(pose, extra,direction,steering))
        {
            bestError=error;bestTravel=extra;
            parkingCcwRefineDirection=direction;parkingCcwRefineSteering=steering;
        }
    }
    return bestTravel;
}

float cwScoutViewExtensionMm(const PositionEstimate &pose, uint8_t seat, float travelled)
{
    if (!parkingCwShortStart || seat>=6) return 0.0f;
    float bearing=0.0f,range=0.0f;
    seatCameraGeometry(seat,pose,bearing,range);
    if (seatComfortablyVisible(seat,pose) && fabsf(bearing)<=5.0f) return 0.0f;
    const float maximum=parkingEntryScoutArcMm()+30.0f-travelled;
    const float k=-routeTurnSign/OBSTACLE_PARKING_ENTRY_SCAN_RADIUS_MM;
    const float h=pose.heading_deg*PI/180.0f;
    for (float extra=5.0f; extra<=maximum; extra+=5.0f)
    {
        const float next=h-k*extra;
        PositionEstimate view=pose;
        view.x_mm+=(sinf(next)-sinf(h))/k;
        view.y_mm+=(cosf(h)-cosf(next))/k;
        view.heading_deg=next*180.0f/PI;
        seatCameraGeometry(seat,view,bearing,range);
        if (seatComfortablyVisible(seat,view) && fabsf(bearing)<=5.0f && preflightCwStartArc(pose,extra))
            return extra;
    }
    return 0.0f;
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
            parkingEntryPrimaryRetryUsed = true;
            startParkingEntryPrimaryRetry();
            Serial.println("[CCW START] Front place unresolved; fresh observation retry");
        }
        return;
    }
    if (parkingEntryTargetStation <= 0)
    {
        armParkingEntryConnectorFromPose(pose);
        return;
    }

    parkingEntryScoutStation = parkingEntryTargetStation - 1;
    // The observation flags have just been cleared by the primary-result
    // transition. Layout inference is enabled in that gap, but is not camera
    // evidence for the preceding start place (576). Check it explicitly.
    if (stationDirectlyResolved(static_cast<uint8_t>(parkingEntryScoutStation)))
    {
        Serial.print("[PARK ENTRY SCOUT] Station already resolved station=");
        Serial.println(parkingEntryScoutStation);
        if (stationResolved(static_cast<uint8_t>(parkingEntryTargetStation)))
            armParkingEntryConnectorFromPose(pose);
        else if (parkingEntryPrimaryRetryUsed)
            startParkingEntryPrimaryRetry();
        else
        {
            parkingEntryPrimaryRetryUsed = true;
            startParkingEntryPrimaryRetry();
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
    parkingEntryScoutTargetMm = parkingEntryScoutArcMm();
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

int parkingEntryScoutMotionSpeed(float arcMm, float travelledMm)
{
    const float terminal=OBSTACLE_PARKING_ENTRY_SCOUT_SPEED_MM_S;
    if(!parkingCwShortStart || routeTurnSign>=0 || arcMm<115.0f)
        return static_cast<int>(terminal);
    return DriveProfile::approachSpeed(arcMm - travelledMm,
        OBSTACLE_PARKING_CW_SCOUT_CRUISE_MM_S, terminal, 18.0f, 70.0f);
}

int parkingEntryScanMotionSpeed(float remaining)
{
    const float terminal = OBSTACLE_PARKING_ENTRY_SPEED_MM_S;
    if (!parkingShortStart()) return static_cast<int>(terminal);
    // A short-start arc can end 3 mm before its nominal encoder target.
    return DriveProfile::approachSpeed(remaining - 3.0f,
        OBSTACLE_PARKING_START_ARC_CRUISE_MM_S, terminal, 22.0f, 90.0f);
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
            obstacle_path_observe(getDiscoveryObstacle());
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
        set_speed(-parkingEntryScoutMotionSpeed(parkingEntryScoutTargetMm,phaseTravel));
        if (phaseTravel >= parkingEntryScoutTargetMm)
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
        if (!resolved && parkingEntryScoutStation>=0 && discoveryScanSide>=0)
        {
            const float extra=cwScoutViewExtensionMm(pose,
                static_cast<uint8_t>(parkingEntryScoutStation*COURSE_SEATS_PER_STATION+discoveryScanSide),phaseTravel);
            if (extra>0.0f)
            {
                parkingEntryScoutTargetMm=phaseTravel+extra;
                parkingEntryScoutPhase=PARKING_ENTRY_SCOUT_REVERSE;
                Serial.print("[CW START] Checked scout view extension_mm=");Serial.println(extra,1);
                return;
            }
        }
        if (!resolved &&
            millis() - parkingEntryScoutPhaseStartMs <
                OBSTACLE_PARKING_ENTRY_SCOUT_OBSERVE_MS)
            return;
        if (!resolved)
        {
            // Return along the checked scout arc, then acquire another fresh
            // view. UNKNOWN alone must not latch the motor off (595).
            parkingEntryScoutPhase=PARKING_ENTRY_SCOUT_RETURN_BRAKE;
            parkingEntryScoutPhaseStartMs=millis();
            Serial.println("[PARK ENTRY SCOUT] Unresolved; measured return then fresh view");
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
                parkingEntryScoutTargetMm + 8.0f)
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
        set_speed(parkingEntryScoutMotionSpeed(parkingEntryScoutReturnTravelMm,phaseTravel));
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
            if (parkingEntryScoutStation>=0 &&
                !stationResolved(static_cast<uint8_t>(parkingEntryScoutStation)))
            {
                startParkingEntryScout(returnedPose);
                return;
            }
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
                parkingEntryPrimaryRetryUsed = true;
                startParkingEntryPrimaryRetry();
            }
        }
        return;
    }
}

bool updateCcwFrontViewRefinement(const PositionEstimate &pose, bool resolved)
{
    if (parkingCcwRefinePhase == 0)
    {
        if (resolved || parkingCcwRefineUsed || !parkingCcwShortStart ||
            parkingEntryTargetStation != 2 || parkingEntryPathFailed ||
            millis() - parkingEntryObserveStartMs < OBSTACLE_PARKING_ENTRY_SCOUT_MIN_OBSERVE_MS)
            return false;
        parkingCcwRefineUsed = true;
        parkingCcwRefineTravel = ccwFrontViewExtensionMm(pose);
        if (parkingCcwRefineTravel <= 0.0f) return false;
        parkingCcwRefinePhase = 1;
        parkingCcwRefineMs = millis();
        Serial.print("[CCW START] Checked front-view refinement_mm=");
        Serial.print(parkingCcwRefineTravel, 1);
        Serial.print(" direction/steering=");Serial.print(parkingCcwRefineDirection);
        Serial.print("/");Serial.println(parkingCcwRefineSteering);
    }
    const int steering = parkingCcwRefineSteering;
    if (parkingCcwRefinePhase == 1)
    {
        stop(false);
        servo_disabled = false;
        set_steering(steering);
        steer(steering);
        if (millis() - parkingCcwRefineMs < OBSTACLE_PARKING_EXIT_STEER_SETTLE_MS)
            return true;
        if (!preflightCcwFrontView(pose, parkingCcwRefineTravel,
                                   parkingCcwRefineDirection,steering))
        {
            parkingCcwRefinePhase = 0;
            return false;
        }
        parkingCcwRefineEncoder = get_distance();
        parkingCcwRefineMs = millis();
        parkingCcwRefinePhase = 2;
    }
    if (parkingCcwRefinePhase == 2)
    {
        const float travelled = parkingCcwRefineDirection*(get_distance() - parkingCcwRefineEncoder);
        if (!isfinite(travelled) || travelled < -5.0f ||
            travelled > parkingCcwRefineTravel + 8.0f)
        {
            stop(false);
            parkingEntryTestHold = true;
            Serial.println("[CCW START] Front-view travel guard");
            return true;
        }
        if (!resolved && travelled < parkingCcwRefineTravel && millis()-parkingCcwRefineMs<6000)
        {
            servo_disabled = false;
            set_steering(steering);
            steer(steering);
            set_speed(parkingCcwRefineDirection*static_cast<int>(OBSTACLE_PARKING_ENTRY_SPEED_MM_S));
            return true;
        }
        stop(true);
        parkingCcwRefinePhase = 3;
        parkingCcwRefineMs = millis();
    }
    stop(true);
    servo_disabled = false;
    set_steering(steering);
    steer(steering);
    const float stoppedTravel = parkingCcwRefineDirection*(get_distance() - parkingCcwRefineEncoder);
    if (!isfinite(stoppedTravel) || stoppedTravel < -5.0f ||
        stoppedTravel > parkingCcwRefineTravel + 8.0f)
    {
        stop(false);
        parkingEntryTestHold = true;
        Serial.println("[CCW START] Front-view braking guard");
        return true;
    }
    if (millis() - parkingCcwRefineMs < OBSTACLE_PARKING_EXIT_STEER_SETTLE_MS)
        return true;
    stop(false);
    set_steering(0);
    parkingCcwRefinePhase = 0;
    parkingEntryObserveStartMs = millis();
    return true;
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
    const int scanWheelSign = parkingCcwRefinePhase != 0 ? (parkingCcwRefineSteering>0 ? 1:-1) : routeTurnSign;
    if (parkingShortStart() && !parkingStartFootprintSafe(
            pose.x_mm, pose.y_mm, pose.heading_deg, scanWheelSign))
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
            obstacle_path_observe(getDiscoveryObstacle());
        lastDiscoveryObservation = observation;
        processGreenSeatCandidates(pose, observation);
        updateDiscoveryCoverage(lastDiscoveryObservation, pose, rawBlob);
    }

    if (parkingEntryObserving)
    {
        const bool resolved =
            !parkingEntryPathFailed &&
            parkingEntryTargetStation >= 0 &&
            stationResolved(static_cast<uint8_t>(parkingEntryTargetStation));
        if (updateCcwFrontViewRefinement(pose, resolved && !parkingEntryNeedsJoinView)) return;
        stop(false);
        set_steering(0);
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
                parkingCcwRefineUsed=false;
                parkingCcwRefineRecovery=true;
                startParkingEntryPrimaryRetry();
                Serial.println("[PARK ENTRY] Unknown; continue checked camera-view search");
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
        set_speed(-parkingEntryScanMotionSpeed(plannedTravel - entryTravel));
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
        // Do not add 12--14 mm reverse just to acquire a two-degree model
        // heading (551/553/555). The actual stopped pose is fully checked
        // again by the connector; this only ends the camera observation arc.
        finishHeadingError <= (parkingCcwFirstEdgeStart ? 12.0f : parkingShortStart() ? 2.0f :
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
    set_speed(-(parkingEntryDrivePhase == PARKING_ENTRY_REVERSE_ARC
        ? parkingEntryScanMotionSpeed(plannedTravel - entryTravel)
        : static_cast<int>(OBSTACLE_PARKING_ENTRY_SPEED_MM_S)));
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

float lapHandoffWeight()
{
    if (!lapHandoffActive) return 1.0f;
    if (completedLaps != 1 || runtimeTestMode || lapFinishPending)
    {
        lapHandoffActive = false;
        return 1.0f;
    }
    // Measured forward travel, not elapsed time or discrete waypoint indices:
    // waiting must not consume the handover, and odometry noise cannot undo it.
    const float travel = get_distance() - lapHandoffEncoderMm;
    if (isfinite(travel)) lapHandoffTravelMm = fmaxf(lapHandoffTravelMm, travel);
    const float t = clampFloat(lapHandoffTravelMm /
        OBSTACLE_LATER_LAP_HANDOFF_MM, 0.0f, 1.0f);
    if (t >= 1.0f) lapHandoffActive = false;
    return t*t*(3.0f-2.0f*t);
}

PathPoint lapHandoffTarget(const PathPoint *path, const PositionEstimate &pose,
                          float lookaheadMm, float weight)
{
    PathPoint target = optimizedBuilt && completedLaps>0 && !lapFinishPending && !runtimeTestMode
        ? laterLapLookaheadTarget(path,pose,progressIndex,lookaheadMm)
        : findLookahead(path,pose,lookaheadMm);
    if (weight >= 1.0f) return target;
    // Blend pursuit targets, not a changing lookahead threshold: the latter
    // would still jump a whole 50 mm waypoint as each threshold is crossed.
    const PathPoint outgoing = findLookahead(livePath, pose, lap1LookaheadMm);
    target.x = outgoing.x + weight*(target.x-outgoing.x);
    target.y = outgoing.y + weight*(target.y-outgoing.y);
    return target;
}

float lapHandoffSpeed(float requestedMmS, float weight)
{
    // Never lift a new curvature/route cap, even when lap 1 was faster.
    return fminf(requestedMmS,
        lap1SpeedMmS + weight*(requestedMmS-lap1SpeedMmS));
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

void obstacle_path_sensor_hold()
{
    if (cornerViewPhase == CORNER_VIEW_IDLE || cornerViewSensorPaused) return;
    cornerViewSensorPaused = true;
    cornerViewSensorPauseMs = millis();
}

void obstacle_path_sensor_resume()
{
    if (!cornerViewSensorPaused) return;
    cornerViewPhaseMs += uint32_t(millis() - cornerViewSensorPauseMs);
    cornerViewSensorPaused = false;
}

void obstacle_path_reset()
{
    surpriseSeatIndex=-1;
    telemetryRouteDirty=true;
    pathLength = 0;
    progressIndex = 0;
    completedLaps = 0;
    lapHandoffActive = lap1CommandValid = false;
    lap1LookaheadMm = lap1SpeedMmS = 0.0f;
    lapHandoffEncoderMm = lapHandoffTravelMm = 0.0f;
    laterTrackingTraceMs = 0;
    laterTrackingTraceCount = 0;
    parkingStartRoiLogged = false;
    routeTurnSign = 1;
    running = false;
    finished = false;
    rollingFinishAllowed = false;
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
    memset(greenSeatCandidateLastSeenMs, 0,
           sizeof(greenSeatCandidateLastSeenMs));
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
    parkingCcwScanArcMm = 0.0f;
    parkingCcwRefinePhase = 0;
    parkingCcwRefineUsed = false;
    parkingCcwRefineDirection=1;parkingCcwRefineSteering=-50;parkingCcwViewAlternate=false;
    parkingCcwRefineRecovery=false;parkingEntryNeedsJoinView=false;
    parkingCcwRefineMs = 0;
    parkingCcwRefineEncoder = parkingCcwRefineTravel = 0.0f;
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
    parkingEntryScoutTargetMm = 0.0f;
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
    cornerViewSensorPaused = false;
    cornerViewCanResume = false;
    cornerViewExtraTriedMask = 0;
    cornerViewForwardTried = false;
    cornerViewSearchCycle = 0;
    cornerViewLastNoSafeTraceMs = 0;
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
    Serial.print("[SURPRISE] revision=20261122_short_red_crosscheck enabled=");
    Serial.println(OBSTACLE_SURPRISE_ENABLED ? 1:0);
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

    Serial.print("[PATH] control_revision=20261122_progress_based_reobserve discovery_mm_s=");
    Serial.print(OBSTACLE_DISCOVERY_SPEED_MM_S,0);
    Serial.print(" confirmed_pass_mm_s=");
    Serial.println(OBSTACLE_LAP1_CONFIRMED_PASS_SPEED_MM_S,0);
    Serial.print("[PATH] Known geometry ready points=");
    Serial.print(pathLength);
    Serial.print(" length_mm=");
    Serial.print(loopLengthMm, 0);
    Serial.print(" turns=");
    Serial.print(routeTurnSign > 0 ? "LEFT" : "RIGHT");
    Serial.print(" first_corner_mm=");
    Serial.println(firstCornerDistanceMm, 0);
}

bool obstacle_path_parking_handoff_pose(PositionEstimate &pose,float *distanceMm)
{
    if(!optimizedBuilt || pathLength<3)return false;
    for(uint16_t i=0;i<pathLength;++i)
    {
        if(baselinePath[i].distanceMm<corners[3].pathEndMm)continue;
        const PositionEstimate p{optimizedPath[i].x,optimizedPath[i].y,
            connectorRouteHeading(optimizedPath,i)};
        if(!finalCornerVehicleClear(p))continue;
        pose=p;if(distanceMm)*distanceMm=baselinePath[i].distanceMm;return true;
    }
    return false;
}

void obstacle_path_allow_rolling_finish(bool allowed){rollingFinishAllowed=allowed;}

bool obstacle_path_record_parking_finish(const PositionEstimate &pose)
{
    return recordParkingFinish(pose);
}

bool parkingStartStationHasColour(uint8_t station,bool red)
{
    for (uint8_t id=2*station;id<2*station+2;++id)
        if (seats[id].confirmed && seats[id].red==red) return true;
    return false;
}

bool parkingCanUseWallArc()
{
    return routeTurnSign<0 ? !parkingStartStationHasColour(0,true) :
        !parkingStartStationHasColour(0,false) && !parkingStartStationHasColour(1,false);
}

bool obstacle_path_parking_corner_window(const PositionEstimate &pose,bool &wallArc)
{
    if (runtimeTestMode || runtimeLapTarget!=3 || completedLaps!=2 ||
        !lapCountingArmed || !optimizedBuilt || finished) return false;
    const float progress=baselinePath[progressIndex].distanceMm;
    wallArc=parkingCanUseWallArc();
    if (!wallArc)
    {
        if (routeTurnSign<0)
        {
            // The red blocks the normal right-side scan. After clearing the
            // last colour pass, hand off rolling into the left wall approach.
            return parkingStartStationHasColour(0,true) &&
                progress>=corners[3].pathEndMm-100.0f &&
                finalCornerVehicleClear(pose) && pose.x_mm>=-50.0f;
        }
        // Once the rear has passed the first CCW green, merge toward the
        // wall in the same direction. The actual sweep still has to fit.
        return routeTurnSign>0 && parkingStartStationHasColour(0,false) &&
            !parkingStartStationHasColour(1,false) && pose.x_mm>=-415.0f &&
            progress>=corners[3].pathEndMm &&
            fabsf(wrap180(pose.heading_deg))<40.0f;
    }
    if (progress<corners[3].pathStartMm-65.0f ||
        progress>corners[3].pathStartMm+245.0f ||
        fabsf(wrap180(pose.heading_deg+90.0f))>42.0f) return false;
    for (uint8_t id=18;id<24;++id)
    {
        const CandidateSeat &seat=seats[id];
        if (!seat.confirmed) continue;
        const float heading=seat.headingDeg*PI/180.0f;
        const float passed=(pose.x_mm-seat.x)*cosf(heading)+(pose.y_mm-seat.y)*sinf(heading);
        if (passed<125.0f) return false;
    }
    return true;
}

float parkingFinishSpeed(float speed,const PositionEstimate &)
{
    if(runtimeTestMode || !optimizedBuilt)return speed;
    if(lapFinishPending)return fminf(speed,static_cast<float>(OBSTACLE_FINAL_PARKING_APPROACH_SPEED));
    if(completedLaps!=2)return speed;
    const float progress=baselinePath[progressIndex].distanceMm;
    const float cornerStart=corners[3].pathStartMm;
    if (OBSTACLE_FINAL_PARKING_ENABLED && runtimeLapTarget==3 && parkingCanUseWallArc() &&
        progress>=cornerStart-450.0f && progress<=cornerStart+245.0f)
    {
        const float remaining=fmaxf(0.0f,cornerStart-65.0f-progress);
        return fminf(speed,sqrtf(205.0f*205.0f+
            2*OBSTACLE_LATER_LAP_PLAN_DECEL_MMSS*remaining));
    }
    if(progress<cornerStart)return speed;
    PositionEstimate arrival;float arrivalDistance=0;
    if(!obstacle_path_parking_handoff_pose(arrival,&arrivalDistance))return speed;
    float remaining=cyclicDistanceForward(baselinePath[progressIndex].distanceMm,arrivalDistance);
    if(remaining>loopLengthMm*.5f)remaining=0;
    remaining=fmaxf(0,remaining-80.0f);
    return fminf(speed,sqrtf(OBSTACLE_FINAL_PARKING_APPROACH_SPEED*
        OBSTACLE_FINAL_PARKING_APPROACH_SPEED+2*OBSTACLE_LATER_LAP_PLAN_DECEL_MMSS*remaining));
}

float lap1ClearRouteSpeed(float speed, const PositionEstimate &pose,
                             const PathPoint *path, float unknownForward,
                             float requestedCurvature, int steering, bool clearStartDeparture)
{
    // Leave a waypoint of reserve and up to 200 mm to shed the extra
    // 60 mm/s before the existing discovery aiming window opens.
    constexpr float brakeLeadMm = 200.0f;
    const float beforeAim = unknownForward - OBSTACLE_LOOK_START_MM - OBSTACLE_PATH_SAMPLE_MM;
    if (beforeAim <= 0.0f || discoveryScanStation >= 0 || discoveryHolding || discoveryBlocked ||
        (!clearStartDeparture && !traversingInjectedAvoidance(baselinePath[progressIndex].distanceMm)))
        return speed;

    const float tangent = connectorRouteHeading(path, progressIndex);
    if (!isfinite(pose.heading_deg) || !isfinite(tangent)) return speed;
    const float h = tangent * PI / 180.0f;
    const float lateral = -(pose.x_mm - path[progressIndex].x) * sinf(h) +
                           (pose.y_mm - path[progressIndex].y) * cosf(h);
    const float headingError = fabsf(wrap180(pose.heading_deg - tangent));
    if (!isfinite(lateral) || !isfinite(headingError)) return speed;
    const float aligned = fminf(clampFloat((40.0f - fabsf(lateral)) / 25.0f, 0.0f, 1.0f),
                                clampFloat((18.0f - headingError) / 12.0f, 0.0f, 1.0f));
    const float blend = aligned * clampFloat(beforeAim / brakeLeadMm, 0.0f, 1.0f);
    const float bonus = OBSTACLE_LAP1_CONFIRMED_PASS_SPEED_MM_S - OBSTACLE_PATH_MAX_SPEED;
    const float target = cappedPathSpeed(speed + bonus * blend);
    return fmaxf(speed, laterLapTrackingSpeed(target, requestedCurvature, steering));
}

float parkingConnectorDepartureSpeed(float baseSpeed, float pathSpeed,
                                     const PositionEstimate &pose, int steering)
{
    // Below this lateral line a wheel can still run beside the pink ends.
    if (runtimeTestMode || !parkingShortStart() || parkingEntryFarGreenFollowup ||
        parkingEntryTargetStation < 0 || !stationResolved(parkingEntryTargetStation) ||
        pose.y_mm <= -1175.0f || abs(steering) > 20)
        return baseSpeed;

    // progressIndex is the merge point while the connector owns steering.
    // Use the current position to avoid skipping a station before the merge.
    const uint16_t index = nearestPathIndex(baselinePath, pose.x_mm, pose.y_mm, 0, pathLength);
    float unknownForward;
    const bool unknownAhead = nearestUpcomingUnresolvedStation(unknownForward, index, -1.0f) >= 0;
    const float cruise = fminf(pathSpeed, OBSTACLE_PARKING_START_CONNECTOR_CRUISE_MM_S);
    if (cruise <= baseSpeed) return baseSpeed;
    if (!unknownAhead) return cruise;
    return DriveProfile::approachSpeed(unknownForward - OBSTACLE_LOOK_START_MM,
        cruise, baseSpeed, OBSTACLE_PATH_SAMPLE_MM, 75.0f);
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
        if(!rollingFinishAllowed)stop(true);
        return;
    }

    if (!runtimeTestMode && new_camera_frame)
    {
        if (completedLaps == 0)
        {
            const ObstacleObservationResult observation =
                obstacle_path_observe(getDiscoveryObstacle());
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
    float commandedSpeed = cappedPathSpeed(lapFinishPending
        ? fminf(progress.speedMmS, OBSTACLE_LATER_LAP_CORNER_SPEED)
        : recordedLapSpeed(progress.speedMmS));
    if (completedLaps > 0 && optimizedBuilt && !runtimeTestMode &&
        !lapFinishPending && !parkingEntryConnectorActive)
        commandedSpeed=cappedPathSpeed(laterLapSpeedLimits[progressIndex]);
    const float handoffWeight = parkingEntryConnectorActive ? 1.0f : lapHandoffWeight();
    if (handoffWeight < 1.0f)
        commandedSpeed = lapHandoffSpeed(commandedSpeed, handoffWeight);
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
    if (parkingEntryConnectorActive)
        widenParkingConnectorLookahead(pose,path,parkingEntryConnectorProgress,
            parkingEntryConnectorMergeIndex,parkingEntryConnectorRouteLookahead);
    float lookahead = parkingEntryConnectorActive
        ? parkingEntryConnectorLookaheadMm
        : (completedLaps > 0 ? OBSTACLE_LATER_LAP_LOOKAHEAD_MM
                             : firstLapLookahead(commandedSpeed));
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
        target = lapHandoffTarget(path, pose, lookahead, handoffWeight);
    }
    if (!runtimeTestMode && !parkingEntryActive && !parkingEntryObserving &&
        !parkingEntryJoining && !parkingEntryConnectorActive)
    {
        applyLap1DiscoveryAim(target, pose);
    }
    const float dx = target.x - pose.x_mm;
    const float dy = target.y - pose.y_mm;
    const float heading = pose.heading_deg * PI / 180.0f;
    const float localX = dx * cosf(heading) + dy * sinf(heading);
    const float localY = -dx * sinf(heading) + dy * cosf(heading);
    const float targetDistanceSquared = fmaxf(1.0f, dx * dx + dy * dy);
    const float curvature = 2.0f * localY / targetDistanceSquared;
    // Positive geometric curvature is left; positive servo command is right.
    const float requiredSteering = pursuitServoForCurvature(curvature);
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
    float steering = clampFloat(
        requiredSteering,
        -OBSTACLE_MAX_PURSUIT_STEERING_DEG,
        OBSTACLE_MAX_PURSUIT_STEERING_DEG);

    const bool returningPastParking = !parkingEntryActive &&
        !parkingEntryObserving && !parkingEntryJoining &&
        !parkingEntryConnectorActive &&
        (completedLaps > 0 || baselinePath[progressIndex].distanceMm > loopLengthMm - 1800.0f);
    // Use the side wall as well as odometry on the first CCW return.
    // Logs 50/51 ended in a stall with a close side return even though the
    // estimated position still showed room. Never steer into that return.
    float startReturnSpeed=INFINITY;
    if (returningPastParking && completedLaps==0 && parkingCcwShortStart &&
        pose.x_mm>=-670.0f && pose.x_mm<=50.0f && pose.y_mm<-650.0f)
    {
        const float heading=wrap180(pose.heading_deg);
        if (fabsf(heading)>14.0f || fabsf(steering)>20.0f)
            startReturnSpeed=135.0f;
        int away=0;
        float nearest=115.0f;
        for (const TofSensor sensor:{TOF_LEFT,TOF_RIGHT})
        {
            TofDiagnosticSnapshot side{};
            if (fabsf(heading)>35.0f ||
                !get_tof_diagnostic_snapshot(sensor,side) ||
                millis()-side.sampled_ms>100U || side.selected_signal_mcps<=0.0f ||
                side.selected_sigma_mm<=0.0f || side.selected_sigma_mm>10.0f ||
                side.filtered_distance_mm<=0.0f || side.filtered_distance_mm>=nearest)
                continue;
            away=sensor==TOF_LEFT ? 1 : -1;
            nearest=side.filtered_distance_mm;
        }
        if (away)
        {
            startReturnSpeed=fminf(startReturnSpeed,100.0f);
            if (away*steering<8.0f)
            {
                const int correction=away*14;
                if (cornerViewSteeredCommandSafe(pose,correction,1))
                    steering=correction;
                else
                    startReturnSpeed=0.0f;
            }
        }
    }
    if (returningPastParking && !parkingReturnMotionSafe(pose, static_cast<int>(steering)))
    {
        stop(false); set_steering(0); parkingEntryTestHold = true;
        Serial.print("[PARK RETURN] Footprint guard held pose=");
        Serial.print(pose.x_mm, 1); Serial.print(",");
        Serial.print(pose.y_mm, 1); Serial.print(",");
        Serial.println(pose.heading_deg, 1);
        return;
    }
    // main() invokes the route only with a fresh healthy gyro. A temporary
    // sensor hold called stop(false),which also disabled servo writes. Restore
    // steering at this successful drive commit,after all rejecting guards.
    servo_disabled = false;
    set_steering(static_cast<int>(steering));
    const char *discoveryTraceReason = nullptr;
    int discoveryTraceStation = -1;
    float safeSpeed = commandedSpeed;
    if (completedLaps > 0 && !runtimeTestMode && !lapFinishPending &&
        !parkingEntryConnectorActive)
    {
        // Actual pursuit corrections can be tighter than the stored route.
        // Cover both requested curvature and the quantized CAD servo command.
        safeSpeed=laterLapTrackingSpeed(safeSpeed,curvature,
            static_cast<int>(steering));
        safeSpeed=laterLapInwardTrackingSpeed(safeSpeed,pose,path,progressIndex);
    }
    if (parkingEntryConnectorActive)
    {
        safeSpeed = fminf(
            safeSpeed,
            OBSTACLE_PARKING_ENTRY_RECOVERY_SPEED_MM_S);
        // The tight parking-piece departure retains 80 mm/s. Once the full
        // body has cleared the tips, a confirmed start GREEN does not require
        // that recovery pace along the gentle, checked connector to RED.
        if (routeTurnSign < 0 && seats[0].confirmed && !seats[0].red &&
            pose.y_mm > -1200.0f && fabsf(steering) <= 20.0f)
            safeSpeed = runtimeSpeedCapMmS > 0.0f
                ? fminf(runtimeSpeedCapMmS,OBSTACLE_PARKING_ENTRY_GREEN_JOIN_SPEED_MM_S)
                : OBSTACLE_PARKING_ENTRY_GREEN_JOIN_SPEED_MM_S;
        safeSpeed = parkingConnectorDepartureSpeed(safeSpeed, commandedSpeed,
            pose, static_cast<int>(steering));
    }
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
        if (!parkingEntryFarGreenFollowup)
        {
            const float phase = baselinePath[progressIndex].distanceMm;
            const bool clearStartDeparture = parkingShortStart() &&
                parkingEntryConnectorLength >= 2 && !lapCountingArmed &&
                pose.y_mm > -1175.0f && (phase <= corners[0].pathStartMm ||
                                       phase >= loopLengthMm - 500.0f);
            float bonusForward = unresolvedStation < 0 ? INFINITY : unresolvedForward;
            if (clearStartDeparture)
            {
                float departureForward;
                if (nearestUpcomingUnresolvedStation(departureForward, progressIndex, -1.0f) >= 0)
                    bonusForward = departureForward;
            }
            safeSpeed = lap1ClearRouteSpeed(safeSpeed, pose, path,
                bonusForward, curvature, static_cast<int>(steering), clearStartDeparture);
        }
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
    // UNKNOWN stations retain the existing station-aware hold and checked
    // observation recovery. The additional unassigned-blob-only stop caused
    // stop/go on fluctuating projections even outside the planned lane.
    safeSpeed=parkingFinishSpeed(safeSpeed,pose);
    safeSpeed=fminf(safeSpeed,startReturnSpeed);
    set_speed(static_cast<int>(safeSpeed));
    if (completedLaps == 0 && !runtimeTestMode && !parkingEntryConnectorActive)
    {
        lap1CommandValid = isfinite(lookahead) && isfinite(safeSpeed) && safeSpeed > 0;
        lap1LookaheadMm = lookahead;
        lap1SpeedMmS = safeSpeed;
    }
    // Bounded onboard diagnostics: no driving USB cable required. 200 records
    // across lap1 return and laps2/3 share the existing ~40kB budget.
    if (!run_telemetry_pose_capture_active() && returningPastParking && !lapFinishPending && robot_logger.remaining()>=80U*1024U &&
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
    // A middle/end station can also leave both seats outside the near camera
    // window (log522). Use the same swept, bounded view recovery there.
    if (discoveryTraceReason != nullptr &&
        strcmp(discoveryTraceReason, "hold_expired") == 0 &&
        discoveryHoldStation >= 0 &&
        !cornerViewAttempted[discoveryHoldStation])
    {
        holdCornerViewCentered();
        cornerViewStation = static_cast<uint8_t>(discoveryHoldStation);
        cornerViewRequestedMinRange=cornerViewMinimumRange(getLargestObstacle());
        cornerViewObservationTraceCount=0;
        cornerViewAttempted[cornerViewStation] = true;
        cornerViewExtraTriedMask = 0;
        cornerViewForwardTried = false;
        cornerViewSearchCycle = 0;
        cornerViewLastSearchMs = 0;
        cornerViewLastNoSafeTraceMs = 0;
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
    if (!RUN_TELEMETRY_DETAILED && !run_telemetry_pose_capture_active()) return;
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
    const PathPoint *route = parkingApproachPath;
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
    run_telemetry_obstacle(seatIndex,seat.red,seat.x,seat.y);
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
        rebuildDiscoveryPath();
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
                                  const PositionEstimate &pose, int footY,
                                  bool green)
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
    const uint8_t localSeat = seatIndex % 6U;
    const bool endStationGreen = green &&
        (localSeat < 2 || localSeat >= 4);
    const float tolerance = endStationGreen
        ? OBSTACLE_PARKING_END_GREEN_FOOT_TOLERANCE_PX
        : OBSTACLE_PARKING_SEAT_FOOT_TOLERANCE_PX;
    return isfinite(expectedFoot) && fabsf(footY-expectedFoot)<=tolerance;
}

void logRedSeatProjection(const ObstacleObservationResult &result,
                          const char *decision, bool force = false)
{
    if(!force && run_telemetry_pose_capture_active())return;
    if(!force && robot_logger.remaining()<112U*1024U)return;
    if (!result.productionValid ||
        (result.color != ColorType::RED && result.color != ColorType::GREEN))
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
    Serial.print(result.color == ColorType::RED ? "[RED SEAT] t=" : "[GREEN SEAT PRIMARY] t=");
    Serial.print(now);
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

bool distantSeatSideAmbiguous(float x, float y, float range, uint8_t seatId,
                             float cameraX = NAN, float cameraY = NAN)
{
    if (!isfinite(range) || range <= 0.0f ||
        seatId >= OBSTACLE_SEAT_COUNT || seats[seatId].confirmed) return false;
    const CandidateSeat &a=seats[seatId], &b=seats[seatId^1U];
    const float h=a.headingDeg*PI/180.0f;
    const float lateral=-(x-0.5f*(a.x+b.x))*sinf(h)+
                         (y-0.5f*(a.y+b.y))*cosf(h);
    // Retain the midpoint guard: distance alone cannot decide the row (514).
    if (fabsf(lateral)>=70.0f) return false;
    if (fabsf(lateral)<30.0f || !isfinite(cameraX) || !isfinite(cameraY) ||
        range<OBSTACLE_DISCOVERY_VIEW_MIN_MM || range>OBSTACLE_DISCOVERY_VIEW_MAX_MM)
        return true;
    // Corner views can resolve the two rows by bearing even when the ground
    // foot underestimates distance. 541--543: outer row differs by 1.9--5.9deg,
    // other row by 19.7--23.7deg. Never decide aligned/far/central projections.
    const float vx=x-cameraX, vy=y-cameraY;
    const float ax=a.x-cameraX, ay=a.y-cameraY;
    const float bx=b.x-cameraX, by=b.y-cameraY;
    const float errorA=fabsf(atan2f(vx*ay-vy*ax,vx*ax+vy*ay))*180.0f/PI;
    const float errorB=fabsf(atan2f(vx*by-vy*bx,vx*bx+vy*by))*180.0f/PI;
    const float separation=fabsf(atan2f(ax*by-ay*bx,ax*bx+ay*by))*180.0f/PI;
    const bool resolved=separation>=12.0f && errorA<=6.0f && errorB-errorA>=8.0f &&
        fabsf(hypotf(ax,ay)-range)<=100.0f;
    return !resolved;
}

int resolveSeatRowByBearing(const ObstacleObservationResult &observation, int nearest)
{
    // Only resolve the two rows of the same non-parking station. At an
    // oblique corner the coloured foot can underestimate depth enough to
    // snap to the nearer row (556/558). Range alone must not choose that row.
    if (routeTurnSign >= 0 || nearest < 6 || nearest >= OBSTACLE_SEAT_COUNT ||
        observation.color != ColorType::GREEN ||
        observation.rangeMm < OBSTACLE_DISCOVERY_VIEW_MIN_MM ||
        observation.rangeMm > OBSTACLE_DISCOVERY_VIEW_MAX_MM)
        return -1;
    const int first = nearest & ~1;
    const float vx=observation.sightingXmm-observation.cameraXmm;
    const float vy=observation.sightingYmm-observation.cameraYmm;
    const float ax=seats[first].x-observation.cameraXmm;
    const float ay=seats[first].y-observation.cameraYmm;
    const float bx=seats[first+1].x-observation.cameraXmm;
    const float by=seats[first+1].y-observation.cameraYmm;
    const float errorA=fabsf(atan2f(vx*ay-vy*ax,vx*ax+vy*ay))*180.0f/PI;
    const float errorB=fabsf(atan2f(vx*by-vy*bx,vx*bx+vy*by))*180.0f/PI;
    const float separation=fabsf(atan2f(ax*by-ay*bx,ax*bx+ay*by))*180.0f/PI;
    const bool firstBetter=errorA<errorB;
    const float best=firstBetter ? errorA:errorB;
    const float other=firstBetter ? errorB:errorA;
    const float range=firstBetter ? hypotf(ax,ay):hypotf(bx,by);
    if (best>4.5f) return -1;
    const int bestSeat=first+(firstBetter ? 0:1);
    const bool decisiveRange=range<=800.0f && range>=observation.rangeMm-80.0f &&
        range<=observation.rangeMm+250.0f && range<=1.6f*observation.rangeMm;
    // 563/566: the earlier, less separated views favour the other row too.
    // Falling back to the depth snap here committed the wrong route before
    // the later decisive bearing arrived. Defer this disputed row, not GREEN.
    if (!decisiveRange || separation<7.0f || other-best<7.0f)
    {
        const bool plausibleAlternative=range<=1000.0f &&
            range>=observation.rangeMm-80.0f && range<=observation.rangeMm+350.0f &&
            range<=1.8f*observation.rangeMm;
        // CW582/583: an early projection can favour the depth-nearest row even
        // though the other legal row remains plausible. Committing that row
        // made the later, decisive outer-row observation an opposite conflict.
        // Keep the row pending regardless of which row won the depth snap.
        return plausibleAlternative ? -2 : -1;
    }
    return bestSeat;
}

bool officialMiddleConflictsWithEnd(uint8_t seatId)
{
    const uint8_t local=seatId%6U, first=seatId/6U*6U;
    return sectionLayoutMode==OBSTACLE_SECTION_LAYOUT_OFFICIAL &&
        (local==2 || local==3) &&
        (seats[first].confirmed || seats[first+1].confirmed ||
         seats[first+4].confirmed || seats[first+5].confirmed);
}

int resolveCcwReturnRedByBearing(const ObstacleObservationResult &observation)
{
    // 567/569: the visible RED after end GREEN22 underestimates depth by
    // 200--300mm across the final corner. This is a returning course view,
    // not the initial parking-boundary view that needs the strict foot gate.
    if (routeTurnSign<=0 || completedLaps!=0 ||
        sectionLayoutMode!=OBSTACLE_SECTION_LAYOUT_OFFICIAL ||
        !parkingSectionInnerSeatsOnly || parkingEntryActive ||
        parkingEntryObserving || parkingEntryScouting || parkingEntryConnectorActive ||
        !seats[22].confirmed || seats[22].red ||
        observation.color!=ColorType::RED || observation.rangeMm<400.0f ||
        observation.rangeMm>600.0f || observation.robotXmm>-750.0f ||
        observation.robotYmm<-500.0f || observation.robotYmm>100.0f)
        return -1;
    const uint8_t inner=seats[0].y>seats[1].y ? 0:1;
    const float sx=seats[inner].x-observation.cameraXmm;
    const float sy=seats[inner].y-observation.cameraYmm;
    const float vx=observation.sightingXmm-observation.cameraXmm;
    const float vy=observation.sightingYmm-observation.cameraYmm;
    const float range=hypotf(sx,sy);
    const float error=fabsf(atan2f(vx*sy-vy*sx,vx*sx+vy*sy))*180.0f/PI;
    if (range>850.0f || range<observation.rangeMm-80.0f ||
        range>observation.rangeMm+350.0f || range>1.8f*observation.rangeMm ||
        error>4.5f) return -1;
    // Other legal longitudinal positions must not explain the same ray.
    for (uint8_t i=2; i<6; ++i)
    {
        if (seats[i].y<seats[i^1U].y) continue;
        const float ax=seats[i].x-observation.cameraXmm;
        const float ay=seats[i].y-observation.cameraYmm;
        const float other=fabsf(atan2f(vx*ay-vy*ax,vx*ax+vy*ay))*180.0f/PI;
        if (other<12.0f) return -1;
    }
    return inner;
}

bool ccwReturnGreenSeatPlausible(uint8_t seat, float snapErrorMm)
{
    // Log 51 associated a background return 86 mm away with the inner
    // first start seat. Do not insert its high-curvature avoidance on that
    // projection; the normal observation can continue from a better view.
    return !parkingCcwShortStart || !lapCountingArmed || seat>=2 ||
        (isfinite(snapErrorMm) && snapErrorMm<=65.0f);
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

    // The clipped-image range is a conservative avoidance placeholder,
    // not measured depth.603:170mm forward snapped an outer-middle GREEN
    // onto end seat13 and inferred the real middle empty. Wait for an
    // uncropped frame/aimed seat view before committing any map position.
    if (blob->minX<=2 || blob->maxX>=317)
    {
        expirePendingVotes();
        result.status=OBSTACLE_OBSERVATION_NO_SEAT;
        result.seatId=-1;
        logRedSeatProjection(result,"edge_clipped_depth_pending");
        return result;
    }

    result.seatId = static_cast<int8_t>(nearestSeatIndex(
        result.sightingXmm,
        result.sightingYmm,
        &result.snapErrorMm));
    const int returnSeat=resolveCcwReturnRedByBearing(result);
    const bool returnBearingResolved=returnSeat>=0;
    if (returnBearingResolved)
    {
        result.seatId=static_cast<int8_t>(returnSeat);
        result.snapErrorMm=hypotf(result.sightingXmm-seats[returnSeat].x,
                                 result.sightingYmm-seats[returnSeat].y);
    }
    if (result.seatId < 0)
    {
        expirePendingVotes();
        result.status = OBSTACLE_OBSERVATION_NO_SEAT;
        logRedSeatProjection(result, "outside_snap");
        return result;
    }

    const int bearingSeat=resolveSeatRowByBearing(result,result.seatId);
    if (bearingSeat == -2 && !seats[result.seatId].confirmed)
    {
        expirePendingVotes();
        result.status=OBSTACLE_OBSERVATION_NO_SEAT;
        result.seatId=-1;
        logRedSeatProjection(result,"row_bearing_pending");
        return result;
    }
    const bool bearingResolved=bearingSeat>=0;
    if (bearingResolved)
    {
        result.seatId=static_cast<int8_t>(bearingSeat);
        result.snapErrorMm=hypotf(result.sightingXmm-seats[bearingSeat].x,
                                 result.sightingYmm-seats[bearingSeat].y);
    }

    if (!returnBearingResolved &&
        !mappedParkingSeatFootMatches(static_cast<uint8_t>(result.seatId), pose,
            result.bottom, result.color == ColorType::GREEN))
    {
        expirePendingVotes();
        result.status = OBSTACLE_OBSERVATION_NO_SEAT;
        result.seatId = -1;
        logRedSeatProjection(result, "start_foot_mismatch");
        return result;
    }

    if (!bearingResolved && distantSeatSideAmbiguous(result.sightingXmm,result.sightingYmm,
            result.rangeMm,static_cast<uint8_t>(result.seatId),
            result.cameraXmm,result.cameraYmm))
    {
        expirePendingVotes();
        result.status=OBSTACLE_OBSERVATION_NO_SEAT;
        result.seatId=-1;
        logRedSeatProjection(result,"distant_side_ambiguous");
        return result;
    }

    // 561: the next section's distant RED projected onto the near end seat
    // at range545/foot114, but its ray missed that seat by11.5 degrees.
    // Do not commit a low-foot, distant RED from depth alone. Close views
    // and the parking section retain their existing separate geometry gates.
    if (result.seatId >= 6 && !seats[result.seatId].confirmed &&
        result.bottom<=(result.color==ColorType::GREEN ? 125:120) && result.rangeMm>=450.0f)
    {
        const float sx=seats[result.seatId].x-result.cameraXmm;
        const float sy=seats[result.seatId].y-result.cameraYmm;
        const float vx=result.sightingXmm-result.cameraXmm;
        const float vy=result.sightingYmm-result.cameraYmm;
        const float error=fabsf(atan2f(vx*sy-vy*sx,vx*sx+vy*sy))*180.0f/PI;
        // 600/601: a high GREEN component snapped to end seat12 despite
        // missing its ray, then the official layout inferred the real middle
        // place empty. Depth alone must not establish that layout either.
        const float maximumError=result.color==ColorType::GREEN ? 6.0f:10.0f;
        //625: distant room RED at foot112/range617 committed actual GREEN22
        //as RED and selected the wrong passing side. Require a consistent
        //seat ray AND depth for these far/high RED votes; near views remain.
        const bool distantRedMismatch=result.color==ColorType::RED &&
            error>4.5f && fabsf(hypotf(sx,sy)-result.rangeMm)>100.0f;
        const bool distantGreenEndMismatch=result.color==ColorType::GREEN &&
            (result.seatId%6)/2!=1 && error>3.0f &&
            fabsf(hypotf(sx,sy)-result.rangeMm)>60.0f;
        if (distantRedMismatch || distantGreenEndMismatch || (error>maximumError &&
            (result.color==ColorType::RED || !bearingResolved)))
        {
            expirePendingVotes();result.status=OBSTACLE_OBSERVATION_NO_SEAT;
            result.seatId=-1;logRedSeatProjection(result,"distant_ray_mismatch");return result;
        }
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
        float maximumRelevantForward =
            OBSTACLE_CAMERA_LOCAL_X_MM +
            OBSTACLE_DISCOVERY_VIEW_MAX_MM +
            OBSTACLE_SEAT_SNAP_RADIUS_MM;
        // Across a corner the path distance exceeds the camera's straight
        // range. 544 rejected the visible start RED as not_upcoming while
        // still before the final corner, until it became oversized.
        const CandidateSeat &observedSeat = seats[result.seatId];
        const float sx = observedSeat.x-result.cameraXmm;
        const float sy = observedSeat.y-result.cameraYmm;
        const float vx = result.sightingXmm-result.cameraXmm;
        const float vy = result.sightingYmm-result.cameraYmm;
        const float bearingError = fabsf(atan2f(vx*sy-vy*sx,vx*sx+vy*sy))*180.0f/PI;
        const float headingCos = cosf((observedSeat.headingDeg-pose.heading_deg)*PI/180.0f);
        // log3CW: RED18 was already geometrically visible at934--1255mm,
        // but the600mm EMPTY-view limit suppressed this corner allowance.
        // Keep the seat snap/ray/votes; colour acquisition has its own range.
        if (result.rangeMm <= CAMERA_DRIVE_CAL_MAX_RANGE_MM &&
            fabsf(headingCos) <= 0.7072f && bearingError <= 6.0f)
            maximumRelevantForward += 0.5f*PI*OBSTACLE_CORNER_RADIUS_MM;
        else if (observedStation%3==1 && fabsf(headingCos)<=0.88f &&
            result.rangeMm <= CAMERA_DRIVE_CAL_MAX_RANGE_MM &&
            bearingError<=4.0f && fabsf(hypotf(sx,sy)-result.rangeMm)<=110.0f)
        {
            // Log 9: the middle GREEN had a good projection from the end
            // of the corner, where heading alone no longer granted leeway.
            maximumRelevantForward += 150.0f;
        }
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
    if (blob->color==ColorType::GREEN && !seats[result.seatId].confirmed &&
        !ccwReturnGreenSeatPlausible(result.seatId,result.snapErrorMm))
    {
        expirePendingVotes();
        result.status=OBSTACLE_OBSERVATION_NO_SEAT;
        result.seatId=-1;
        logRedSeatProjection(result,"start_return_projection");
        return result;
    }
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

    // 554/556: range aliasing also occurs inside 600 mm. Apply the existing
    // official end-pair constraint at acquisition, not only at lap2 validation.
    // CHECK_ALL retains arbitrary layouts and independent station evidence.
    const bool middle=(static_cast<uint8_t>(result.seatId)%6U)/2U==1U;
    const bool transientFarMiddle=sectionLayoutMode==OBSTACLE_SECTION_LAYOUT_OFFICIAL &&
        parkingEntryConnectorActive && middle &&
        result.rangeMm>OBSTACLE_DISCOVERY_VIEW_MAX_MM;
    if (!seats[result.seatId].confirmed &&
        (officialMiddleConflictsWithEnd(static_cast<uint8_t>(result.seatId)) || transientFarMiddle))
    {
        expirePendingVotes();
        result.status = OBSTACLE_OBSERVATION_NO_SEAT;
        result.seatId = -1;
        logRedSeatProjection(result,"official_middle_conflict");
        return result;
    }
    CandidateSeat &seat = seats[result.seatId];
    if (seat.confirmed)
    {
        // A mapped foreground blob can alternate with the approaching sign.
        // It supplies no new vote and lets pending votes age normally.
        expirePendingVotes();
        result.status = OBSTACLE_OBSERVATION_ALREADY_CONFIRMED;
    }
    else
    {
        prepareConsecutiveVote(result.seatId, blob->color);
        if (recordObservedSeatVote(static_cast<uint8_t>(result.seatId), *blob))
            finalizeConfirmedSeat(result);
        else
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
        if (officialMiddleConflictsWithEnd(seatIndex))
            continue;
        float bearingDeg = 0.0f;
        float expectedRangeMm = -1.0f;
        seatCameraGeometry(seatIndex, pose, bearingDeg, expectedRangeMm);
        //624-627: supplement the primary blob before the600mm empty-seat
        //window. This only searches colour at a mapped non-parking seat;
        //CLEAR, silhouette/colour, snap and three-frame gates stay unchanged.
        const float greenBearingLimit=OBSTACLE_CAMERA_HORIZONTAL_FOV_DEG*
            OBSTACLE_DISCOVERY_FOV_FRACTION-OBSTACLE_DISCOVERY_CLEAR_FOV_MARGIN_DEG;
        const bool earlyGreenView=seatIndex>=6 &&
            expectedRangeMm>=OBSTACLE_DISCOVERY_VIEW_MIN_MM && expectedRangeMm<=800.0f &&
            fabsf(bearingDeg)<=greenBearingLimit;
        if (!seatComfortablyVisible(seatIndex, pose) && !earlyGreenView)
        {
            seat.greenSeatCandidateFrames = 0;
            logParkingGreenCheck(seatIndex, pose, "outside_view");
            continue;
        }

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
            if (candidate.silhouetteFound)
                greenSeatCandidateLastSeenMs[seatIndex] = now;
            logParkingGreenCheck(seatIndex, pose,
                candidate.silhouetteFound ? "occupied_colour_unknown" : "no_silhouette");
            continue;
        }

        greenSeatCandidateThisFrame[seatIndex] = candidate.silhouetteFound;
        if (candidate.silhouetteFound)
            greenSeatCandidateLastSeenMs[seatIndex] = now;
        if (candidate.blob.minX<=2 || candidate.blob.maxX>=317)
        {
            seat.greenSeatCandidateFrames=0;
            logParkingGreenCheck(seatIndex,pose,"edge_clipped_depth_pending",&candidate);
            continue;
        }
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
            !ccwReturnGreenSeatPlausible(seatIndex,snapErrorMm) ||
            distantSeatSideAmbiguous(sightingX, sightingY, measuredRangeMm,
                seatIndex,cameraX,cameraY) ||
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
        if(OBSTACLE_SURPRISE_ENABLED && surpriseSeatIndex<0)
        {
            vision.measurePillarHeight(camera.getBuffer(),camera.getWidth(),
                                       camera.getHeight(),candidate.blob);
            prepareConsecutiveVote(seatIndex,ColorType::GREEN);
            if(!recordObservedSeatVote(seatIndex,candidate.blob))
            {logParkingGreenCheck(seatIndex,pose,"height_pending",&candidate);continue;}
        }
        else
        {
            seat.confirmed = true;
            seat.visibleRed=seat.red = false;
            seat.greenVotes = seat.greenSeatCandidateFrames;
        }
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

// Buffered exit frames use their corrected historical pose, never the current
// end-of-exit pose. This is deliberately stricter than live acquisition: three
// agreeing frames plus a stable association under pose perturbations are needed.
int parkingExitObservationSeat(const Blob &blob, const PositionEstimate &pose,
                               ObstacleObservationResult &result)
{
    if (!obstacle_blob_valid_for_acquisition(&blob) || blob.minX<=2 || blob.maxX>=317) return -1;
    result = ObstacleObservationResult{};
    result.color = blob.color;
    result.productionValid = true;
    result.left = blob.minX; result.right = blob.maxX;
    result.top = blob.minY; result.bottom = blob.maxY;
    result.bearingDeg = obstacle_camera_bearing_deg(&blob);
    result.rangeMm = obstacle_estimate_camera_range_mm(&blob);
    if (!isfinite(result.rangeMm) || result.rangeMm < 120.0f ||
        result.rangeMm > 900.0f || fabsf(result.bearingDeg) > 28.0f) return -1;
    const float heading = pose.heading_deg * PI / 180.0f;
    result.robotXmm = pose.x_mm; result.robotYmm = pose.y_mm;
    result.robotHeadingDeg = pose.heading_deg;
    result.cameraXmm = pose.x_mm + OBSTACLE_CAMERA_LOCAL_X_MM * cosf(heading) -
        OBSTACLE_CAMERA_LOCAL_Y_MM * sinf(heading);
    result.cameraYmm = pose.y_mm + OBSTACLE_CAMERA_LOCAL_X_MM * sinf(heading) +
        OBSTACLE_CAMERA_LOCAL_Y_MM * cosf(heading);
    const float ray = heading + result.bearingDeg * PI / 180.0f;
    result.sightingXmm = result.cameraXmm + result.rangeMm * cosf(ray);
    result.sightingYmm = result.cameraYmm + result.rangeMm * sinf(ray);
    const int seat = nearestSeatIndex(result.sightingXmm, result.sightingYmm,
                                      &result.snapErrorMm);
    if (seat < 0 || seat >= 6 || result.snapErrorMm > 100.0f ||
        !mappedParkingSeatFootMatches(seat, pose, blob.maxY, blob.color == ColorType::GREEN) ||
        distantSeatSideAmbiguous(result.sightingXmm, result.sightingYmm, result.rangeMm,
            seat, result.cameraXmm, result.cameraYmm)) return -1;
    float expectedBearing = 0.0f, expectedRange = 0.0f;
    seatCameraGeometry(seat, pose, expectedBearing, expectedRange);
    if (fabsf(wrap180(expectedBearing - result.bearingDeg)) > 6.0f ||
        fabsf(expectedRange - result.rangeMm) > 100.0f) return -1;
    result.seatId = seat;
    return seat;
}

bool parkingExitObservationStable(const Blob &blob, const PositionEstimate &pose,
                                  ObstacleObservationResult &result)
{
    const int seat = parkingExitObservationSeat(blob, pose, result);
    if (seat < 0) return false;
    // Allow for exposure-time approximation and residual exit odometry drift.
    // These margins test the association; they do not change the estimator.
    for (float dx : {-20.0f, 0.0f, 20.0f})
        for (float dy : {-20.0f, 0.0f, 20.0f})
            for (float dh : {-3.0f, 0.0f, 3.0f}) {
                PositionEstimate varied = pose;
                varied.x_mm += dx; varied.y_mm += dy; varied.heading_deg += dh;
                // Perturbations must retain the same seat, but nominal ray/foot
                // residuals must not be charged the uncertainty a second time.
                const float h = varied.heading_deg * PI / 180.0f;
                const float cx = varied.x_mm + OBSTACLE_CAMERA_LOCAL_X_MM*cosf(h) -
                    OBSTACLE_CAMERA_LOCAL_Y_MM*sinf(h);
                const float cy = varied.y_mm + OBSTACLE_CAMERA_LOCAL_X_MM*sinf(h) +
                    OBSTACLE_CAMERA_LOCAL_Y_MM*cosf(h);
                const float ray = h + result.bearingDeg*PI/180.0f;
                const float x = cx + result.rangeMm*cosf(ray);
                const float y = cy + result.rangeMm*sinf(ray);
                if (nearestSeatIndex(x,y,nullptr) != seat ||
                    hypotf(x-seats[seat ^ 1U].x,y-seats[seat ^ 1U].y) -
                    hypotf(x-seats[seat].x,y-seats[seat].y) < 40.0f) return false;
            }
    return true;
}

void obstacle_path_apply_exit_observations(const ParkingExitCameraBuffer &buffer)
{
    if (!running || runtimeTestMode || completedLaps != 0 ||
        !parkingSectionInnerSeatsOnly || !parkingEntryActive || parkingEntryTestHold ||
        buffer.direction() != routeTurnSign) return;
    struct Evidence {
        ObstacleObservationResult result{};
        uint32_t firstUs = 0, lastUs = 0;
        uint8_t colors = 0, count = 0;
        bool qualified = false;
    } evidence[6];
    bool conflict[3] = {};
    const uint32_t now = micros();
    for (uint16_t i = 0; i < buffer.size(); ++i) {
        const ParkingExitCameraFrame &frame = buffer.frame(i);
        if (now - frame.sampledUs > 20000000U) continue;
        for (const Blob *blob : {&frame.red, &frame.green}) {
            ObstacleObservationResult observation;
            if (!parkingExitObservationStable(*blob, frame.pose, observation)) continue;
            const uint8_t seat = static_cast<uint8_t>(observation.seatId);
            Evidence &e = evidence[seat];
            const uint8_t color = blob->color == ColorType::RED ? 1 : 2;
            e.colors |= color;
            if (e.colors == 3 || evidence[seat ^ 1U].colors) conflict[seat / 2] = true;
            if (e.qualified) continue;
            if (!e.count || frame.sampledUs - e.lastUs > OBSTACLE_SEAT_VOTE_WINDOW_MS*1000U ||
                observation.color != e.result.color ||
                hypotf(observation.sightingXmm - e.result.sightingXmm,
                       observation.sightingYmm - e.result.sightingYmm) > 60.0f) {
                e.count = 0;
                e.firstUs = frame.sampledUs;
                e.result = observation;
            }
            e.lastUs = frame.sampledUs;
            if (e.count < 255) ++e.count;
            e.qualified = e.count >= 3 && e.lastUs - e.firstUs >= 120000U;
        }
    }
    // Reject a contradictory official middle/end batch in its entirety;
    // import order must never decide which placement becomes authoritative.
    const bool middle = evidence[2].qualified || evidence[3].qualified;
    const bool end = evidence[0].qualified || evidence[1].qualified ||
        evidence[4].qualified || evidence[5].qualified;
    if (sectionLayoutMode == OBSTACLE_SECTION_LAYOUT_OFFICIAL && middle && end) {
        Serial.println("[EXIT VIEW] Conflicting middle/end evidence; normal scan retained");
        return;
    }
    uint8_t accepted = 0;
    for (uint8_t i = 0; i < 6; ++i) {
        Evidence &e = evidence[i];
        if (!e.qualified || conflict[i/2] || seats[i].confirmed ||
            seats[i ^ 1U].confirmed || officialMiddleConflictsWithEnd(i)) continue;
        CandidateSeat &seat = seats[i];
        //610: an internally consistent historical projection put an end
        //GREEN onto the middle seat. Committing it here skipped the normal
        //scan and left an impossible three-pillar section at the lap boundary.
        //Keep the cached evidence as one pending vote only. A fresh matched
        //observation must still pass normal association/expiry before it can
        //alter the map, infer empty places or inject a bypass.
        const bool red = e.result.color == ColorType::RED;
        const uint8_t votes = OBSTACLE_SEAT_CONFIRM_VOTES > 1 ? 1 : 0;
        seat.redVotes = red ? votes : 0;
        seat.greenVotes = red ? 0 : votes;
        seat.lastVoteMs = millis();
        ++accepted;
        Serial.print("[EXIT VIEW] Pending seat/color/frames="); Serial.print(i);
        Serial.print("/"); Serial.print(red ? "RED" : "GREEN");
        Serial.print("/"); Serial.print(e.count);
        Serial.print(" corrected_pose="); Serial.print(e.result.robotXmm,1);
        Serial.print("/"); Serial.print(e.result.robotYmm,1);
        Serial.print("/"); Serial.print(e.result.robotHeadingDeg,1);
        Serial.print(" bearing/range/snap="); Serial.print(e.result.bearingDeg,1);
        Serial.print("/"); Serial.print(e.result.rangeMm,1);
        Serial.print("/"); Serial.println(e.result.snapErrorMm,1);
    }
    Serial.print("[EXIT VIEW] Pending hints="); Serial.print(accepted);
    Serial.println("; fresh match required, normal scan retained, no cached CLEAR votes");
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
    info.visibleRed=seat.visibleRed;
    info.modifiedSign=seat.modifiedSign;
    info.heightMm=seat.heightMm;
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
    surpriseSeatIndex = -1;
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
        seats[i].heightFrames = seats[i].heightClass = seats[i].heightColor = 0;
        seats[i].lastHeightFrame = 0;
        seats[i].heightMm = 100;
        seats[i].modifiedSign = seats[i].visibleRed = false;
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
