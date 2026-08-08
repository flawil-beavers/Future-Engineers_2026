#include "obstacle_path.h"

#include <math.h>
#include <string.h>

#include "config.h"
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
    uint8_t redVotes = 0;
    uint8_t greenVotes = 0;
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

PathPoint baselinePath[OBSTACLE_MAX_PATH_WAYPOINTS];
PathPoint livePath[OBSTACLE_MAX_PATH_WAYPOINTS];
PathPoint optimizedPath[OBSTACLE_MAX_PATH_WAYPOINTS];
PathPoint smoothingBuffer[OBSTACLE_MAX_PATH_WAYPOINTS];
CandidateSeat seats[OBSTACLE_SEAT_COUNT];

uint16_t pathLength = 0;
uint16_t progressIndex = 0;
uint8_t completedLaps = 0;
int8_t routeTurnSign = 1;
bool running = false;
bool finished = false;
bool optimizedBuilt = false;
float loopLengthMm = 0.0f;
CornerGeometry corners[4];

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
        loopLengthMm - 0.5f * OBSTACLE_STRAIGHT_LENGTH_MM,
        0.5f * OBSTACLE_STRAIGHT_LENGTH_MM + cornerArc,
        1.5f * OBSTACLE_STRAIGHT_LENGTH_MM + 2.0f * cornerArc,
        2.5f * OBSTACLE_STRAIGHT_LENGTH_MM + 3.0f * cornerArc};

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
    const float passSide = seat.red ? -1.0f : 1.0f;
    const float targetLateral =
        seat.lateralMm + passSide * clearanceMm;
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

void registerDetection(const Blob *blob)
{
    if (blob == nullptr || !blob->found ||
        (blob->color != ColorType::RED &&
         blob->color != ColorType::GREEN) ||
        blob->height() <= 0)
    {
        return;
    }

    const PositionEstimate pose = get_position_struct();
    const float bearing =
        (static_cast<float>(blob->centerX) - 160.0f) *
        (OBSTACLE_CAMERA_HORIZONTAL_FOV_DEG / 320.0f);
    const bool edgeClipped = blob->minX <= 2 || blob->maxX >= 317;
    const float range = edgeClipped
                            ? OBSTACLE_EDGE_CLIPPED_RANGE_MM
                            : OBSTACLE_CAMERA_FOCAL_LENGTH_PX *
                                  OBSTACLE_PILLAR_HEIGHT_MM /
                                  blob->height();
    const float globalBearing =
        (pose.heading_deg + bearing) * PI / 180.0f;
    const float sightingX = pose.x_mm + range * cosf(globalBearing);
    const float sightingY = pose.y_mm + range * sinf(globalBearing);

    int bestSeat = -1;
    float bestDistance =
        OBSTACLE_SEAT_SNAP_RADIUS_MM * OBSTACLE_SEAT_SNAP_RADIUS_MM;
    for (uint8_t i = 0; i < OBSTACLE_SEAT_COUNT; ++i)
    {
        const float candidateDistance = distanceSquared(
            sightingX,
            sightingY,
            seats[i].x,
            seats[i].y);
        if (candidateDistance < bestDistance)
        {
            bestDistance = candidateDistance;
            bestSeat = i;
        }
    }

    if (bestSeat < 0)
        return;

    CandidateSeat &seat = seats[bestSeat];
    uint8_t &votes =
        blob->color == ColorType::RED ? seat.redVotes : seat.greenVotes;
    if (votes < 255)
        ++votes;

    const uint8_t winningVotes =
        seat.redVotes > seat.greenVotes ? seat.redVotes : seat.greenVotes;
    if (!seat.confirmed && winningVotes >= OBSTACLE_SEAT_CONFIRM_VOTES)
    {
        seat.confirmed = true;
        seat.red = seat.redVotes > seat.greenVotes;
        displaceForSeat(livePath, seat, OBSTACLE_LAP1_CLEARANCE_MM);
        seat.injected = true;

        Serial.print("[PATH] Live avoidance injected seat=");
        Serial.print(bestSeat);
        Serial.print(" color=");
        Serial.println(seat.red ? "RED" : "GREEN");
    }
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
        if (completedLaps < 255)
            ++completedLaps;
        Serial.print("[PATH] Completed lap ");
        Serial.println(completedLaps);

        if (completedLaps == 1)
            buildOptimizedPath();
        else if (completedLaps >= 3)
            finished = true;
    }
}

float computeLookSteering(const PositionEstimate &pose)
{
    if (completedLaps != 0)
        return 0.0f;

    const float currentDistance = baselinePath[progressIndex].distanceMm;
    float bestForward = OBSTACLE_LOOK_START_MM + 1.0f;
    float steering = 0.0f;

    for (uint8_t i = 0; i < OBSTACLE_SEAT_COUNT; ++i)
    {
        if (seats[i].confirmed)
            continue;
        const float forward = cyclicDistanceForward(
            currentDistance,
            seats[i].pathDistanceMm);
        if (forward < OBSTACLE_LOOK_END_MM ||
            forward > OBSTACLE_LOOK_START_MM ||
            forward >= bestForward)
        {
            continue;
        }

        const float expectedBearing = atan2f(
            seats[i].y - pose.y_mm,
            seats[i].x - pose.x_mm) *
            180.0f / PI;
        const float bearingError =
            wrap180(expectedBearing - pose.heading_deg);
        if (fabsf(bearingError) <=
            OBSTACLE_CAMERA_HORIZONTAL_FOV_DEG * 0.42f)
        {
            continue;
        }

        const float taper = clampFloat(
            (OBSTACLE_LOOK_START_MM - forward) /
                (OBSTACLE_LOOK_START_MM - OBSTACLE_LOOK_END_MM),
            0.0f,
            1.0f);
        // Positive bearing is left; positive servo command is right.
        steering = clampFloat(
            -bearingError * OBSTACLE_LOOK_HEADING_KP * taper,
            -OBSTACLE_LOOK_MAX_STEERING_DEG,
            OBSTACLE_LOOK_MAX_STEERING_DEG);
        bestForward = forward;
    }
    return steering;
}

float residualVisionSteering()
{
    if (completedLaps == 0)
        return 0.0f;

    const Blob *blob = getLargestObstacle();
    if (blob == nullptr || !blob->found)
        return 0.0f;

    const int targetX =
        blob->color == ColorType::RED
            ? OBSTACLE_RED_TARGET_X
            : OBSTACLE_GREEN_TARGET_X;
    return clampFloat(
        (blob->centerX - targetX) * OBSTACLE_RESIDUAL_VISION_KP,
        -OBSTACLE_RESIDUAL_VISION_MAX_DEG,
        OBSTACLE_RESIDUAL_VISION_MAX_DEG);
}

void applyTofCorrection(const PositionEstimate &pose)
{
    const float pathDistance = baselinePath[progressIndex].distanceMm;
    int cornerIndex = -1;
    for (uint8_t corner = 0; corner < 4; ++corner)
    {
        if (withinCornerGate(pathDistance, corner))
        {
            cornerIndex = corner;
            break;
        }
    }

    const PathPoint center = baselinePath[progressIndex];
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
        // The inside wall opens at a known corner: left for a left-turning
        // route, right for a right-turning route. This is the precomputed
        // corner-side geometry gate; no measurement-jump detector is used.
        const bool recedingAtCorner =
            cornerIndex >= 0 &&
            (left
                 ? corners[cornerIndex].recedesOnLeft
                 : corners[cornerIndex].recedesOnRight);
        if (recedingAtCorner)
            continue;

        const float reading = get_tof_distance(
            left ? TOF_LEFT : TOF_RIGHT);
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

        float lateralError =
            (expectedWallX - measuredWallX) * wallNormalX +
            (expectedWallY - measuredWallY) * wallNormalY;
        lateralError = clampFloat(
            lateralError * OBSTACLE_TOF_CORRECTION_GAIN,
            -OBSTACLE_TOF_CORRECTION_MAX_STEP_MM,
            OBSTACLE_TOF_CORRECTION_MAX_STEP_MM);
        correctionX += wallNormalX * lateralError;
        correctionY += wallNormalY * lateralError;
        ++corrections;
    }

    if (corrections > 0)
    {
        position_apply_xy_correction(
            correctionX / corrections,
            correctionY / corrections);
    }
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
    loopLengthMm = 0.0f;
    memset(seats, 0, sizeof(seats));
}

void obstacle_path_start(int8_t turn_sign)
{
    obstacle_path_reset();
    routeTurnSign = turn_sign < 0 ? -1 : 1;
    const PositionEstimate anchor = get_position_struct();

    float x = 0.0f;
    float y = 0.0f;
    float heading = 0.0f;
    float distance = 0.0f;
    appendLocalPoint(x, y, 0.0f, distance, anchor);
    appendStraight(
        OBSTACLE_STRAIGHT_LENGTH_MM * 0.5f,
        x, y, heading, distance, anchor);
    for (uint8_t corner = 0; corner < 4; ++corner)
    {
        appendCorner(corner, x, y, heading, distance, anchor);
        appendStraight(
            corner == 3
                ? OBSTACLE_STRAIGHT_LENGTH_MM * 0.5f
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
    Serial.println(routeTurnSign > 0 ? "LEFT" : "RIGHT");
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

    if (new_camera_frame)
    {
        if (completedLaps == 0)
            registerDetection(getLargestObstacle());
    }

    applyTofCorrection(pose);
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

    const PathPoint target = findLookahead(path, pose, lookahead);
    const float dx = target.x - pose.x_mm;
    const float dy = target.y - pose.y_mm;
    const float heading = pose.heading_deg * PI / 180.0f;
    const float localY = -dx * sinf(heading) + dy * cosf(heading);
    const float targetDistanceSquared = fmaxf(1.0f, dx * dx + dy * dy);
    const float curvature = 2.0f * localY / targetDistanceSquared;
    // Positive geometric curvature is left; positive servo command is right.
    float steering =
        -atanf(OBSTACLE_WHEELBASE_MM * curvature) * 180.0f / PI;
    steering += computeLookSteering(pose);
    if (new_camera_frame)
        steering += residualVisionSteering();
    steering = clampFloat(
        steering,
        -OBSTACLE_MAX_PURSUIT_STEERING_DEG,
        OBSTACLE_MAX_PURSUIT_STEERING_DEG);

    set_steering(static_cast<int>(steering));
    set_speed(static_cast<int>(progress.speedMmS));
}

bool obstacle_path_started()
{
    return running;
}

bool obstacle_path_complete()
{
    return finished;
}

uint8_t obstacle_path_lap()
{
    return completedLaps;
}

uint16_t obstacle_path_progress_index()
{
    return progressIndex;
}
