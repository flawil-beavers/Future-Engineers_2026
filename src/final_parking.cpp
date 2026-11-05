#include "final_parking.h"

#include <math.h>
#include <initializer_list>

#include "config.h"
#include "logger.h"
#include "run_telemetry.h"
#include "motor_control.h"
#include "obstacle_path.h"
#include "position_estimator.h"
#include "sensors.h"
#include "ackermann_kinematics.h"
#include "parking_start_footprint.h"

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
    FP_ABORT,
    FP_SCAN_ALIGN_DRIVE,
    FP_SCAN_ALIGN_BRAKE,
    FP_CAPTURE_ALIGN_DRIVE,
    FP_CAPTURE_ALIGN_BRAKE,
    FP_REPLAN_BRAKE
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
    250.26f-OBSTACLE_FINAL_PARKING_REARWARD_SHIFT_MM,
    153.07f-OBSTACLE_FINAL_PARKING_REARWARD_SHIFT_MM,
    116.85f-OBSTACLE_FINAL_PARKING_REARWARD_SHIFT_MM,
    72.36f-OBSTACLE_FINAL_PARKING_REARWARD_SHIFT_MM,
    90.65f-OBSTACLE_FINAL_PARKING_REARWARD_SHIFT_MM,
    56.25f-OBSTACLE_FINAL_PARKING_REARWARD_SHIFT_MM,
    81.25f-OBSTACLE_FINAL_PARKING_REARWARD_SHIFT_MM};
constexpr float SEGMENT_LOCAL_Y_MM[
    OBSTACLE_FINAL_PARKING_SEGMENT_COUNT] = {
    294.60f, 234.95f, 163.62f, 117.56f, 125.57f, 120.00f, 120.00f};
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
bool scanLineEstablished = false;
bool scanWallRegistered = false;
int8_t scanAlignSign = 1;
uint8_t scanAlignTurns = 0;
uint32_t scanAlignStartMs = 0;

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
bool captureAlignPending = false;
bool completed = false;
bool aborted = false;
bool practiceMode = false;
bool initialBrakeStarted = false;
float approachOuterShiftX = OBSTACLE_FINAL_PARKING_OUTER_SHIFT_X_MM;
bool approachDirectLane = false;
uint32_t wallSequence = 0, finalTraceMs = 0;
uint16_t finalTraceCount = 0;
uint8_t initialWallFrames = 0;
uint8_t sensorHoldTraceCount = 0;
bool sensorHoldPending = false;
uint32_t sensorHoldStartMs = 0;
float initialWallCorrection = 0.0f;
float lastWallCorrection = 0.0f;
uint32_t wallCorrectionSequence = 0;
uint32_t stoppedWallSequence = 0;
uint8_t stoppedWallFrames = 0;
float stoppedWallRange = 0.0f;
constexpr uint8_t ENTRY_PLAN_CAPACITY = 16;
ParkingSegment adaptiveEntry[ENTRY_PLAN_CAPACITY]{};
bool entryQuickRecipe = false;
PositionEstimate adaptiveEndpoints[ENTRY_PLAN_CAPACITY]{};
uint8_t adaptiveEntryCount = 0;
uint8_t entryReplans = 0;
uint8_t entryStalledReplans = 0;
float entryBestReplanError = INFINITY;
float entrySafetyMargin = 3.0f;
const char *entryReplanReason = nullptr;
float entryRadiusScale = 1.0f;
float entryStartHeading = 0.0f;
bool entryRadiusMeasured = false;
bool entryReplanHadMotion = false;
float entryBrakeStartDistance = 0.0f;
float entryBrakeTravelMm = 0.0f;
int8_t entryBrakeDirection = 1;
uint32_t entryPlanningBudget = 0;
uint32_t entryPlannerLastPollMs = 0;
uint32_t rearBrakeEpoch=0,rearReferenceSequence=0;
uint8_t rearReferenceFrames=0;
float rearReferenceAxleMm=0;
bool entryBrakeEvidenceValid=false;
uint32_t entryBrakeEpoch=0,entryBrakeStillMs=0;
uint8_t entryBrakeEvidenceState=0;
unsigned long entryBrakeSampleCount=0;
float entryBrakeStillDistance=0,entryBrakeStillHeading=0;

void serviceEntryPlanner()
{
    // Geometry samples vary greatly in cost on the MCU. Poll by elapsed time,
    // not every256 samples (594/596: >200ms gaps and SH2 restarts while stopped).
    const uint32_t now=millis();
    if (now-entryPlannerLastPollMs>=10U)
    { entryPlannerLastPollMs=now;update_gyro(); }
}

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
           (scanPhase == 5 && measuredGapMm > 0.0f
                ? measuredGapMm : OBSTACLE_FINAL_PARKING_GAP_MM);
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

BeamFootprint markerEdgeFootprint(TofSensor sensor, const PositionEstimate &pose,
    float planeY = OBSTACLE_SOUTH_OUTER_WALL_Y_MM + OBSTACLE_PARKING_WIDTH_MM)
{
    // At a piece edge the accepted range can mix the nearby piece and the
    // wall. Its value classifies occupancy, but is not the depth of the tip.
    // Intersect both cone rays with the known marker-tip plane instead of
    // expanding the cone with that mixed range (531/533/534: oversized gap).
    const float localX = sensor == TOF_LEFT
        ? OBSTACLE_TOF_LEFT_LOCAL_X_MM : OBSTACLE_TOF_RIGHT_LOCAL_X_MM;
    const float localY = sensor == TOF_LEFT
        ? OBSTACLE_TOF_LEFT_LOCAL_Y_MM : OBSTACLE_TOF_RIGHT_LOCAL_Y_MM;
    const float h = pose.heading_deg * PI / 180.0f;
    const float sx = pose.x_mm + cosf(h)*localX - sinf(h)*localY;
    const float sy = pose.y_mm + sinf(h)*localX + cosf(h)*localY;
    const float ray = h + (sensor == TOF_LEFT ? 0.5f*PI : -0.5f*PI);
    const float half = 0.5f*OBSTACLE_PARKING_EXIT_TOF_DETECTION_FOV_DEG*PI/180.0f;
    const float dy1 = sinf(ray-half), dy2 = sinf(ray+half);
    if (!isfinite(sx) || !isfinite(sy) || fabsf(dy1)<0.1f || fabsf(dy2)<0.1f)
        return {NAN, NAN};
    const float r1 = (planeY-sy)/dy1, r2 = (planeY-sy)/dy2;
    if (r1<=0.0f || r2<=0.0f) return {NAN, NAN};
    const float x1 = sx+r1*cosf(ray-half), x2 = sx+r2*cosf(ray+half);
    return {fminf(x1,x2), fmaxf(x1,x2)};
}

bool earlyWallReferenceClearOfMarkers(const PositionEstimate &pose)
{
    // Before the marker scan, X may still carry the full registration error.
    // Mixed piece/wall returns can pass the80mm initial residual gate (564).
    // Only use an early reference if the whole fan misses both possible rails,
    // from their open tips to the wall. Skipping a correction does not stop drive.
    const BeamFootprint tip=markerEdgeFootprint(outerSensor,pose);
    const BeamFootprint back=markerEdgeFootprint(outerSensor,pose,OBSTACLE_SOUTH_OUTER_WALL_Y_MM);
    if (!isfinite(tip.minimumX) || !isfinite(tip.maximumX) ||
        !isfinite(back.minimumX) || !isfinite(back.maximumX)) return false;
    const float low=fminf(tip.minimumX,back.minimumX);
    const float high=fmaxf(tip.maximumX,back.maximumX);
    const float uncertainty=OBSTACLE_FINAL_PARKING_MAX_X_POSE_CORRECTION_MM;
    return high < movingInsideFaceX()-20.0f-uncertainty ||
           low > OBSTACLE_PARKING_FIXED_INNER_FACE_X_MM+20.0f+uncertainty;
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

float parkingOuterRange(const TofDiagnosticSnapshot &snapshot)
{
    // General navigation selects the furthest accepted object. Here a nearby
    // pink piece must take precedence over the wall in the same ToF cone.
    float nearest = 1.0e9f;
    for (uint8_t i=0; i<snapshot.stored_object_count; ++i)
    {
        const TofObjectDiagnostic &object = snapshot.objects[i];
        if (object.hardware_valid && object.filter_accepted && object.distance_mm>0)
            nearest = fminf(nearest,static_cast<float>(object.distance_mm));
    }
    // The diagnostic selection is not independent quality evidence. Without
    // an accepted object, neither wall registration nor marker classification
    // may consume a fallback number from this frame.
    return nearest < 1.0e9f ? nearest : -1.0f;
}

// Only fresh, wall-consistent returns may correct scan-line odometry. Marker
// edges mix wall/piece returns, so they deliberately do not move the pose.
bool scanAlignmentPreflight(PositionEstimate pose);
bool updateOuterWallReference(bool initial)
{
    TofDiagnosticSnapshot snapshot;
    if (!get_tof_diagnostic_snapshot(outerSensor, snapshot) ||
        snapshot.sequence == wallSequence ||
        millis() - snapshot.sampled_ms > 250)
        return false;
    wallSequence = snapshot.sequence;
    const PositionEstimate pose = get_position_struct();
    // Once both pieces are registered, keep that reference during capture.
    //612: mixed returns close to a pink edge repeatedly shifted Y and the
    //line controller steered away from an already good alignment.
    if (!initial && (scanPhase == 5 || !earlyWallReferenceClearOfMarkers(pose)))
        return false;
    if (initial && state==FP_APPROACH_LANE && !earlyWallReferenceClearOfMarkers(pose))
    {
        initialWallFrames=0;
        return false;
    }
    const float raw = parkingOuterRange(snapshot);
    const float expected = expectedOuterWallRange(outerSensor, pose);
    const float residual = raw - expected;
    const float maxResidual = initial ? 150.0f : 10.0f;
    if (!isfinite(raw) || raw < OBSTACLE_FINAL_PARKING_WALL_MIN_MM ||
        raw > (initial ? 700.0f : OBSTACLE_FINAL_PARKING_WALL_MAX_MM) || expected <= 0.0f ||
        fabsf(residual) > maxResidual ||
        fabsf(wrap180(pose.heading_deg-baseHeadingDeg())) > 10.0f)
    {
        initialWallFrames = 0;
        return false;
    }
    if (initial && (initialWallFrames == 0 ||
                    fabsf(residual-initialWallCorrection) > 8.0f))
    {
        initialWallFrames = 1;
        initialWallCorrection = residual;
        return false;
    }
    const float rayY = sinf(pose.heading_deg*PI/180.0f +
        (outerSensor == TOF_LEFT ? 0.5f*PI : -0.5f*PI));
    const float correction = -rayY * (initial
        ? 0.5f*(residual+initialWallCorrection)
        : clampFloat(residual*0.2f,-1.0f,1.0f));
    if (initial && fabsf(residual)>80.0f)
    {
        // 567/577: stable wall up to387mm versus predicted258mm after three laps.
        // Only the cleared outer corridor may register this larger drift,
        // and only if its complete bounded line alignment has a safe sweep.
        PositionEstimate corrected=pose;
        corrected.y_mm+=correction;
        if (!scanAlignmentPreflight(corrected)) return false;
    }
    position_apply_xy_correction(0.0f,correction);
    lastWallCorrection = correction;
    if (correction != 0.0f) ++wallCorrectionSequence;
    if (initial)
    {
        initialWallFrames = 2;
        Serial.print("[FINAL PARK WALL] initial_y_correction_mm=");
        Serial.println(correction,1);
    }
    return true;
}

// Bounded on-stick diagnostics replace unrelated vision chatter while parking.
void traceParking(bool force = false)
{
    if(!force && run_telemetry_pose_capture_active())return;
    if (finalTraceCount >= 240 ||
        (!force && robot_logger.remaining()<8192U) ||
        (!force && millis()-finalTraceMs < 200)) return;
    finalTraceMs = millis();
    ++finalTraceCount;
    const PositionEstimate pose = get_position_struct();
    TofDiagnosticSnapshot snapshot{};
    const bool available = get_tof_diagnostic_snapshot(outerSensor,snapshot);
    Serial.print("[FINAL_PARK_TRACE] t="); Serial.print(millis());
    Serial.print(" state/phase="); Serial.print(static_cast<int>(state));
    Serial.print("/"); Serial.print(scanPhase);
    Serial.print(" xyh="); Serial.print(pose.x_mm,1); Serial.print("/");
    Serial.print(pose.y_mm,1); Serial.print("/"); Serial.print(pose.heading_deg,1);
    Serial.print(" steering="); Serial.print(set_degree);
    Serial.print(" distance/speed="); Serial.print(get_distance(),1);
    Serial.print("/"); Serial.print(current_speed,1);
    Serial.print(" loop_ms="); Serial.print(last_loop_time*1000.0f,1);
    Serial.print(" tof_seq/age/raw/expected="); Serial.print(snapshot.sequence);
    Serial.print("/"); Serial.print(available ? millis()-snapshot.sampled_ms : 9999);
    Serial.print("/"); Serial.print(parkingOuterRange(snapshot),1);
    Serial.print("/"); Serial.print(expectedOuterWallRange(outerSensor,pose),1);
    Serial.print(" selected_raw="); Serial.print(snapshot.selected_raw_distance_mm,1);
    Serial.print(" cs="); Serial.print(wallCorrectionSequence);
    Serial.print(" wall_correction="); Serial.println(lastWallCorrection,1);
}

bool connectorClearanceSafe(const PositionEstimate &pose)
{
    if (!isfinite(pose.x_mm) || !isfinite(pose.y_mm) || !isfinite(pose.heading_deg)) return false;
    const float h = pose.heading_deg * PI / 180.0f;
    const float dx = OBSTACLE_ROBOT_ENVELOPE_AXIS_FRONT_MM * cosf(h);
    const float dy = OBSTACLE_ROBOT_ENVELOPE_AXIS_FRONT_MM * sinf(h);
    const float outer=OBSTACLE_CENTERLINE_HALF_EXTENT_MM+OBSTACLE_CORRIDOR_HALF_WIDTH_MM;
    const float inner=OBSTACLE_CENTERLINE_HALF_EXTENT_MM-OBSTACLE_CORRIDOR_HALF_WIDTH_MM;
    const float reserve=OBSTACLE_MAX_WHEEL_HALF_WIDTH_MM+5.0f;
    // In the south parking strip the whole inflated axis capsule is below
    // the inner square and inside all outer walls. This exact sufficient
    // bound avoids repeating eight wall/four corner distance calculations
    // at every millimetre of a stopped entry search. Else use the old test.
    const bool southStrip=fminf(pose.x_mm,pose.x_mm+dx)>=-outer+reserve &&
        fmaxf(pose.x_mm,pose.x_mm+dx)<=outer-reserve &&
        fminf(pose.y_mm,pose.y_mm+dy)>=-outer+reserve &&
        fmaxf(pose.y_mm,pose.y_mm+dy)<=-inner-reserve;
    // Walls do not depend on the selected pillar. Scan them once, rather
    // than repeating the same wall geometry for every confirmed seat at
    // every 2-mm approach-preflight sample.
    ObstacleClearanceSample walls;
    if (!southStrip && (!obstacle_path_sample_pose_clearance(0, pose.x_mm, pose.y_mm,
            pose.heading_deg, walls) || !walls.valid || walls.wallMm < 5.0f))
        return false;
    const float lengthSquared = dx*dx + dy*dy;
    const float pillarReserve=OBSTACLE_MAX_WHEEL_HALF_WIDTH_MM+
        OBSTACLE_PILLAR_MOVEMENT_RADIUS_MM+10.0f;
    for (uint8_t seatId = 0; seatId < obstacle_path_seat_count(); ++seatId)
    {
        ObstacleSeatInfo seat;
        if (!obstacle_path_get_seat(seatId, seat) || !seat.confirmed)
            continue;
        const float t = clampFloat(((seat.xMm-pose.x_mm)*dx +
            (seat.yMm-pose.y_mm)*dy) / fmaxf(lengthSquared,1.0e-6f),0.0f,1.0f);
        const float px=seat.xMm-pose.x_mm-t*dx,py=seat.yMm-pose.y_mm-t*dy;
        const float squared=px*px+py*py;
        if (!isfinite(squared) || squared<pillarReserve*pillarReserve) return false;
    }
    // The three-lap path cannot complete until every station is resolved.
    return true;
}

void abortParking(const char *reason)
{
    traceParking(true);
    stop(false);
    set_steering(0);
    state = FP_ABORT;
    aborted = true;
    Serial.print("[FINAL PARK ABORT] ");
    Serial.println(reason);
    run_telemetry_finish("aborted",reason);
    robot_logger.write_to_usb();
}

bool parkingBrakeReady(uint32_t minimumMs)
{
    const uint32_t elapsed=millis()-stateStartMs;
    if (elapsed<minimumMs) return false;
    if (isfinite(measured_speed) &&
        fabsf(measured_speed)<=SOFT_STOP_SPEED_THRESHOLD_MMS)
        return true;
    if (elapsed>=minimumMs+OBSTACLE_FINAL_PARKING_BRAKE_SETTLE_TIMEOUT_MS)
        abortParking("brake_not_settled");
    return false;
}

bool parkingEntryBrakeReady()
{
    // Retain the proven300ms path and timeout. Earlier continuation requires
    // independent encoder/yaw stillness AND fresh low-speed measurements.
    if (parkingBrakeReady(OBSTACLE_FINAL_PARKING_HOLD_BRAKE_MS)) return true;
    if (aborted) return false;
    const uint32_t now=millis();
    const float distance=get_distance();
    const float heading=get_position_struct().heading_deg;
    if (!isfinite(distance) || !isfinite(heading) || !isfinite(measured_speed))
    {entryBrakeEvidenceValid=false;return false;}
    if (!entryBrakeEvidenceValid || entryBrakeEpoch!=stateStartMs ||
        entryBrakeEvidenceState!=state ||
        fabsf(distance-entryBrakeStillDistance)>0.5f ||
        fabsf(wrap180(heading-entryBrakeStillHeading))>0.25f)
    {
        entryBrakeEvidenceValid=true;entryBrakeEpoch=stateStartMs;
        entryBrakeEvidenceState=state;entryBrakeStillMs=now;
        entryBrakeStillDistance=distance;entryBrakeStillHeading=heading;
        entryBrakeSampleCount=speed_measurement_count;
        return false;
    }
    return uint32_t(now-stateStartMs)>=150U &&
        uint32_t(now-entryBrakeStillMs)>=120U &&
        (speed_measurement_count-entryBrakeSampleCount)>=2UL &&
        fabsf(measured_speed)<=SOFT_STOP_SPEED_THRESHOLD_MMS;
}

bool checkMotionHealth(const PositionEstimate &pose)
{
    if (!gyro_is_healthy())
    {
        stop(false);
        final_parking_sensor_hold();
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

int fieldLineSteering(
    const PositionEstimate &pose,
    int8_t fieldMotionSign,
    float targetY)
{
    if (targetY==OBSTACLE_FINAL_PARKING_APPROACH_LANE_Y_MM)
    {
        // Preserve the established lane acquisition from an oblique final
        // corner. The new line pursuit is for the cleared outer corridor.
        const float angle=fieldMotionSign*atan2f(targetY-pose.y_mm,
            OBSTACLE_FINAL_PARKING_LINE_LOOKAHEAD_MM)*180.0f/PI;
        const float desired=baseHeadingDeg()+clampFloat(angle,
            -OBSTACLE_FINAL_PARKING_LINE_MAX_ANGLE_DEG,
            OBSTACLE_FINAL_PARKING_LINE_MAX_ANGLE_DEG);
        return static_cast<int>(clampFloat(-motorDirectionForFieldMotion(fieldMotionSign)*
            wrap180(desired-pose.heading_deg)*OBSTACLE_FINAL_PARKING_LINE_HEADING_KP,
            -OBSTACLE_FINAL_PARKING_LINE_MAX_STEERING,
             OBSTACLE_FINAL_PARKING_LINE_MAX_STEERING));
    }
    // Track the field line with the measured Ackermann model rather than
    // multiplying a bearing error into sustained full lock (648 stalled at42).
    // Target behind the vehicle for reverse travel; CW/CCW mirror naturally.
    const float dy=targetY-pose.y_mm;
    const float horizon=OBSTACLE_FINAL_PARKING_LINE_LOOKAHEAD_MM+
        fminf(fabsf(dy),OBSTACLE_FINAL_PARKING_LINE_LOOKAHEAD_MM);
    const float dx=fieldMotionSign*horizon;
    const float h=pose.heading_deg*PI/180.0f;
    const float curvature=2.0f*(-dx*sinf(h)+dy*cosf(h))/fmaxf(1.0f,dx*dx+dy*dy);
    const float steering = clampFloat(fabsf(curvature)<1.0e-6f ? 0.0f :
        Ackermann::getServoAngleForRadius(-1.0f/curvature),
        -OBSTACLE_FINAL_PARKING_LINE_MAX_STEERING,
        OBSTACLE_FINAL_PARKING_LINE_MAX_STEERING);
    return static_cast<int>(steering);
}

void driveFieldLine(int8_t fieldMotionSign, float targetY, int speed)
{
    const int motorDirection = motorDirectionForFieldMotion(fieldMotionSign);
    servo_disabled = false;
    set_speed(motorDirection * speed);
    set_steering(fieldLineSteering(get_position_struct(),fieldMotionSign,targetY));
}

bool linePoseReady(const PositionEstimate &pose, float targetY)
{
    return fabsf(pose.y_mm - targetY) <=
               OBSTACLE_FINAL_PARKING_LINE_Y_TOLERANCE_MM &&
           fabsf(wrap180(baseHeadingDeg() - pose.heading_deg)) <=
               OBSTACLE_FINAL_PARKING_CAPTURE_HEADING_TOLERANCE_DEG;
}

// Follow the stored bypass towards canonical +X before moving out to the
// marker scan line. In CW this is reverse motion, retaining the body heading.
bool parkingApproachSteering(const PositionEstimate &pose, bool outer, int &steering)
{
    if (!outer)
    {
        if (approachDirectLane)
        {
            steering=fieldLineSteering(pose,+1,OBSTACLE_FINAL_PARKING_APPROACH_LANE_Y_MM);
            return true;
        }
        float tx=0.0f, ty=0.0f;
        // Preserve the learned pillar bypass inside the start section. Past
        // it, continue straight instead of following the next corner bend.
        if (pose.x_mm >= 500.0f)
        { tx=pose.x_mm+100.0f; ty=pose.y_mm; }
        else if (!obstacle_path_parking_approach_target(pose.x_mm,pose.y_mm,tx,ty))
            return false;
        const float h=pose.heading_deg*PI/180.0f;
        const float dx=tx-pose.x_mm, dy=ty-pose.y_mm;
        const float curvature=2.0f*(-dx*sinf(h)+dy*cosf(h))/fmaxf(1.0f,dx*dx+dy*dy);
        steering=static_cast<int>(fabsf(curvature)<1e-6f ? 0.0f :
            clampFloat(Ackermann::getServoAngleForRadius(-1.0f/curvature),-42.0f,42.0f));
    }
    else
    {
        // Keep the wide-angle corner departure controller. The gentler line
        // pursuit takes over once this outward bend has reached the scan line.
        const float angle=clampFloat(atan2f(
            OBSTACLE_FINAL_PARKING_SCAN_Y_MM-pose.y_mm,
            OBSTACLE_FINAL_PARKING_LINE_LOOKAHEAD_MM)*180.0f/PI,-60.0f,60.0f);
        const float desired=baseHeadingDeg()+angle;
        steering=static_cast<int>(clampFloat(
            -motorDirectionForFieldMotion(+1)*wrap180(desired-pose.heading_deg)*
                OBSTACLE_FINAL_PARKING_LINE_HEADING_KP,-42.0f,42.0f));
    }
    return true;
}

void advanceParkingPose(PositionEstimate &pose, int steering, float distance)
{
    const float radius=Ackermann::getTurnRadius(static_cast<float>(steering));
    const float k=fabsf(radius)>100000.0f ? 0.0f : -1.0f/radius;
    const float h=pose.heading_deg*PI/180.0f, next=h+k*distance;
    if (fabsf(k)>1e-6f)
    {
        pose.x_mm+=(sinf(next)-sinf(h))/k;
        pose.y_mm+=(cosf(h)-cosf(next))/k;
    }
    else { pose.x_mm+=distance*cosf(h); pose.y_mm+=distance*sinf(h); }
    pose.heading_deg=wrap180(next*180.0f/PI);
}

void advanceEntryModel(PositionEstimate &pose,int steering,float distance,float scale)
{
    const float radius=Ackermann::getTurnRadius(static_cast<float>(steering))*scale;
    const float k=steering ? -1.0f/radius:0.0f;
    const float h=pose.heading_deg*PI/180.0f,next=h+k*distance;
    if (fabsf(k)>1.0e-6f)
    { pose.x_mm+=(sinf(next)-sinf(h))/k;pose.y_mm+=(cosf(h)-cosf(next))/k; }
    else
    { pose.x_mm+=distance*cosf(h);pose.y_mm+=distance*sinf(h); }
    pose.heading_deg=wrap180(next*180.0f/PI);
}

bool parkingApproachPoseSafe(const PositionEstimate &pose)
{
    // An earlier outward shift must retain the same steering-nose reserve
    // as the entry checks, including the registered +/-40mm lane uncertainty.
    const float h=pose.heading_deg*PI/180.0f;
    PositionEstimate nose=pose;
    const float reserve=OBSTACLE_FINAL_PARKING_STEER_FRONT_RESERVE_MM;
    nose.x_mm+=reserve*cosf(h);
    nose.y_mm+=reserve*sinf(h);
    return connectorClearanceSafe(pose) &&
        connectorClearanceSafe(nose) &&
        parking_start_footprint::safe(pose.x_mm,pose.y_mm,pose.heading_deg,-1,false,5.0f,false,
            OBSTACLE_FINAL_PARKING_STRAIGHT_WIDTH_MM,OBSTACLE_FINAL_PARKING_REAR_OVERHANG_MM,
            PHYSICAL_WHEELBASE_MM,OBSTACLE_FINAL_PARKING_FRONT_OVERHANG_MM,
            OBSTACLE_FINAL_PARKING_STEERED_FRONT_MM) &&
        parking_start_footprint::safe(pose.x_mm,pose.y_mm,pose.heading_deg,+1,false,5.0f,false,
            OBSTACLE_FINAL_PARKING_STRAIGHT_WIDTH_MM,OBSTACLE_FINAL_PARKING_REAR_OVERHANG_MM,
            PHYSICAL_WHEELBASE_MM,OBSTACLE_FINAL_PARKING_FRONT_OVERHANG_MM,
            OBSTACLE_FINAL_PARKING_STEERED_FRONT_MM) &&
        parking_start_footprint::safe(nose.x_mm,nose.y_mm,nose.heading_deg,-1,false,5.0f,false,
            OBSTACLE_FINAL_PARKING_STRAIGHT_WIDTH_MM,OBSTACLE_FINAL_PARKING_REAR_OVERHANG_MM,
            PHYSICAL_WHEELBASE_MM,OBSTACLE_FINAL_PARKING_FRONT_OVERHANG_MM,
            OBSTACLE_FINAL_PARKING_STEERED_FRONT_MM) &&
        parking_start_footprint::safe(nose.x_mm,nose.y_mm,nose.heading_deg,+1,false,5.0f,false,
            OBSTACLE_FINAL_PARKING_STRAIGHT_WIDTH_MM,OBSTACLE_FINAL_PARKING_REAR_OVERHANG_MM,
            PHYSICAL_WHEELBASE_MM,OBSTACLE_FINAL_PARKING_FRONT_OVERHANG_MM,
            OBSTACLE_FINAL_PARKING_STEERED_FRONT_MM);
}

bool parkingApproachPreflight(PositionEstimate pose)
{
    bool outer=false;
    for (int step=0; step<1200; ++step)
    {
        if (!parkingApproachPoseSafe(pose)) return false;
        if (approachDirectLane)
        {
            for (float dy:{-40.0f,40.0f})
            {
                PositionEstimate uncertain=pose;uncertain.y_mm+=dy;
                if (!parkingApproachPoseSafe(uncertain)) return false;
            }
        }
        outer=outer || pose.x_mm>=approachOuterShiftX;
        if (outer && pose.x_mm>=OBSTACLE_FINAL_PARKING_SCAN_START_X_MM &&
            linePoseReady(pose,OBSTACLE_FINAL_PARKING_SCAN_Y_MM)) return true;
        if (pose.x_mm>1100.0f) return false;
        int steering=0;
        if (!parkingApproachSteering(pose,outer,steering)) return false;
        advanceParkingPose(pose,steering,2.0f*motorDirectionForFieldMotion(+1));
    }
    return false;
}

bool chooseParkingApproach(const PositionEstimate &pose)
{
    // Stay on the extended straight instead of entering the next corner.
    // A late shift may be needed behind an inner start RED. Every candidate checks
    // the complete motion against all known pillars, walls and both pieces.
    // Keep the shift early enough to settle before the next outer wall;
    // preflight rejects a candidate intersecting a known pillar or marker.
    const float candidates[]={350.0f,400.0f,450.0f,500.0f,550.0f,600.0f,
        OBSTACLE_FINAL_PARKING_OUTER_SHIFT_X_MM};
    for (bool direct:{true,false}) for (float candidate:candidates)
    {
        // Only the direct lane has the full +/-40mm lateral uncertainty
        // sweep. Preserve the learned bypass's established shift interval.
        if (!direct && candidate<500.0f) continue;
        approachDirectLane=direct;
        approachOuterShiftX=candidate;
        if (parkingApproachPreflight(pose)) return true;
    }
    return false;
}

bool parkingApproachCommandSafe(PositionEstimate pose, int steering, int direction)
{
    for (int i=0; i<=15; ++i)
    {
        if (!parkingApproachPoseSafe(pose)) return false;
        advanceParkingPose(pose,steering,2.0f*direction);
    }
    return true;
}

// The measured gap replaces the nominal gap after the marker scan. Test the
// actual steered wheels, not an enclosing rectangle, at the marker tips.
bool parkingEntryPoseSafe(const PositionEstimate &pose, int steering, float margin=-1.0f)
{
    if (!connectorClearanceSafe(pose)) return false;
    const float safetyMargin=margin<0 ? entrySafetyMargin:margin;
    using namespace parking_start_footprint;
    Quad bodies[6];
    robotQuads(pose.x_mm,pose.y_mm,pose.heading_deg,
               steering>0 ? 1 : (steering<0 ? -1 : 0),safetyMargin,bodies,
               OBSTACLE_FINAL_PARKING_STRAIGHT_WIDTH_MM,
               OBSTACLE_FINAL_PARKING_REAR_OVERHANG_MM,PHYSICAL_WHEELBASE_MM,
               OBSTACLE_FINAL_PARKING_FRONT_OVERHANG_MM,
               OBSTACLE_FINAL_PARKING_STEERED_FRONT_MM,
               Ackermann::getLeftWheelAngleLUT(steering),
               Ackermann::getRightWheelAngleLUT(steering));
    const Quad fixed=rectangle(490.0f,-1400.0f,10.0f+safetyMargin,100.0f+safetyMargin,0.0f);
    const Quad moving=rectangle(movingInsideFaceX()-10.0f,-1400.0f,10.0f+safetyMargin,100.0f+safetyMargin,0.0f);
    // Extra nose room while steering is a swept-motion reserve, not a new
    //parked X target. Check the real footprint AND a forward-shifted copy;
    //the ordinary rear clearance remains unchanged.
    const float h=pose.heading_deg*PI/180.0f;
    // A tight escape must still be possible if the current pose already
    //lacks this preferred nose reserve;1mm mode retains the physical sweep.
    for(int offset=0;offset<(steering && safetyMargin>=3.0f ? 2:1);++offset)
    for (Quad body:bodies)
    {
        if(offset)for(Point &p:body.p)
        {
            p.x+=OBSTACLE_FINAL_PARKING_STEER_FRONT_RESERVE_MM*cosf(h);
            p.y+=OBSTACLE_FINAL_PARKING_STEER_FRONT_RESERVE_MM*sinf(h);
        }
        float minimumY=INFINITY,minimumX=INFINITY,maximumX=-INFINITY;
        for (const Point &p:body.p) if (p.y<=OBSTACLE_SOUTH_OUTER_WALL_Y_MM+safetyMargin) return false;
        for (const Point &p:body.p)
        {
            minimumY=fminf(minimumY,p.y);
            minimumX=fminf(minimumX,p.x);maximumX=fmaxf(maximumX,p.x);
        }
        // Most approach/first-arc polygons are entirely beyond both rail tips.
        // Avoid expensive SAT tests there; wall and pillar checks still apply.
        if (minimumY>OBSTACLE_PARKING_OPEN_END_FIELD_Y_MM+safetyMargin) continue;
        // A separating X axis proves no overlap just like the SAT itself.
        // Most samples inside/beside the gap need no further SAT axes.
        if ((maximumX>=480.0f-safetyMargin && minimumX<=500.0f+safetyMargin && overlaps(body,fixed)) ||
            (maximumX>=movingInsideFaceX()-20.0f-safetyMargin &&
             minimumX<=movingInsideFaceX()+safetyMargin && overlaps(body,moving))) return false;
    }
    return true;
}

bool parkingEntryPreflight(PositionEstimate pose)
{
    for (const ParkingSegment &segment:PARKING_SEGMENTS)
    {
        const int steering=segment.steering*routeTurnSign;
        for (float travel=0.0f; travel<segment.distanceMm; travel+=1.0f)
        {
            if (!parkingEntryPoseSafe(pose,steering)) return false;
            advanceParkingPose(pose,steering,segment.direction);
        }
        if (!parkingEntryPoseSafe(pose,steering)) return false;
    }
    return true;
}

bool scanCommandSafe(PositionEstimate pose, int direction)
{
    for (int mm=0; mm<=15; ++mm)
    {
        if (!parkingEntryPoseSafe(pose,set_degree)) return false;
        advanceParkingPose(pose,set_degree,static_cast<float>(direction));
    }
    return true;
}

bool captureAlignmentPreflight(PositionEstimate pose)
{
    // Registration can leave only17mm to capture (564), insufficient to remove
    // a10mm lateral error. Establish the line farther along +X, then return.
    // Check the entire out-and-back against the measured bay and known pillars.
    bool returning=false;
    int returnStart=-1;
    const int limit=static_cast<int>(OBSTACLE_FINAL_PARKING_CAPTURE_MAX_MM);
    for (int mm=0; mm<=2*limit; ++mm)
    {
        if (!returning && linePoseReady(pose,OBSTACLE_FINAL_PARKING_SCAN_Y_MM) &&
            pose.x_mm>=captureTargetX+80.0f)
        { returning=true;returnStart=mm; }
        if ((!returning && mm>limit) || (returning && mm-returnStart>limit))
            return false;
        const int direction=motorDirectionForFieldMotion(returning ? -1 : 1);
        const int steering=fieldLineSteering(pose,returning ? -1 : 1,
            OBSTACLE_FINAL_PARKING_SCAN_Y_MM);
        if (!parkingEntryPoseSafe(pose,steering)) return false;
        if (returning && pose.x_mm<=captureTargetX+1.5f)
            return linePoseReady(pose,OBSTACLE_FINAL_PARKING_SCAN_Y_MM);
        advanceParkingPose(pose,steering,static_cast<float>(direction));
    }
    return false;
}

bool captureDirectPreflight(PositionEstimate pose)
{
    // A registered lateral error does not always require an out-and-back.
    // In CCW the ordinary capture often already has several hundred mm to
    // establish the line. Accept it only with the same controller, measured
    // gap, endpoint gates and complete subsequent entry sweep.
    const int sign = captureFieldMotionSign;
    for (int mm=0; mm<=static_cast<int>(OBSTACLE_FINAL_PARKING_CAPTURE_MAX_MM); ++mm)
    {
        const int steering=fieldLineSteering(pose,sign,OBSTACLE_FINAL_PARKING_SCAN_Y_MM);
        if (!parkingEntryPoseSafe(pose,steering)) return false;
        const bool reached=sign>0 ? pose.x_mm>=captureTargetX-1.5f
                                  : pose.x_mm<=captureTargetX+1.5f;
        if (reached)
            return fabsf(pose.x_mm-captureTargetX)<=OBSTACLE_FINAL_PARKING_CAPTURE_X_TOLERANCE_MM &&
                linePoseReady(pose,OBSTACLE_FINAL_PARKING_SCAN_Y_MM) &&
                parkingEntryPreflight(pose);
        advanceParkingPose(pose,steering,static_cast<float>(motorDirectionForFieldMotion(sign)));
    }
    return false;
}

bool scanAlignmentPreflight(PositionEstimate pose)
{
    int sign=pose.x_mm<1000.0f ? 1:-1;
    int turns=0;
    for (int mm=0; mm<=1440; mm+=2)
    {
        if (pose.x_mm>=800.0f && linePoseReady(pose,OBSTACLE_FINAL_PARKING_SCAN_Y_MM))
            return true;
        if ((sign>0 && pose.x_mm>=1080.0f) || (sign<0 && pose.x_mm<=800.0f))
        { sign=-sign;if (++turns>6) return false; }
        const int steering=fieldLineSteering(pose,sign,OBSTACLE_FINAL_PARKING_SCAN_Y_MM);
        if (!parkingEntryPoseSafe(pose,steering)) return false;
        advanceParkingPose(pose,steering,2.0f*motorDirectionForFieldMotion(sign));
    }
    return false;
}

void resetScanClassFrames()
{
    classFrames = 0;
    movingCandidateStored = false;
}

void processScanFrame(const TofDiagnosticSnapshot &snapshot)
{
    const PositionEstimate pose = get_position_struct();
    const float raw = parkingOuterRange(snapshot);
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
                measuredFixedInsideX = markerEdgeFootprint(
                    outerSensor, lastFixedMarkerPose).maximumX;
                Serial.print("[FINAL PARK EDGE] fixed_x/raw_mm=");
                Serial.print(measuredFixedInsideX, 1); Serial.print("/");
                Serial.println(lastFixedMarkerRange, 1);
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
                measuredMovingInsideX = markerEdgeFootprint(
                    outerSensor, firstMovingMarkerPose).minimumX;
                Serial.print("[FINAL PARK EDGE] moving_x/raw_mm=");
                Serial.print(measuredMovingInsideX, 1); Serial.print("/");
                Serial.println(firstMovingMarkerRange, 1);
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

bool refreshStoppedScanWallReference()
{
    TofDiagnosticSnapshot snapshot{};
    if (!get_tof_diagnostic_snapshot(outerSensor,snapshot) ||
        snapshot.sequence==stoppedWallSequence || millis()-snapshot.sampled_ms>250 ||
        static_cast<int32_t>(snapshot.sampled_ms-stateStartMs)<
            static_cast<int32_t>(OBSTACLE_FINAL_PARKING_HOLD_BRAKE_MS)) return false;
    stoppedWallSequence=snapshot.sequence;
    const PositionEstimate pose=get_position_struct();
    const float raw=parkingOuterRange(snapshot);
    const float expected=expectedOuterWallRange(outerSensor,pose);
    if (!isfinite(raw) || raw<OBSTACLE_FINAL_PARKING_WALL_MIN_MM ||
        raw>OBSTACLE_FINAL_PARKING_WALL_MAX_MM || expected<=0.0f ||
        fabsf(raw-expected)>40.0f)
    { stoppedWallFrames=0;return false; }
    if (stoppedWallFrames==0 || fabsf(raw-stoppedWallRange)>8.0f)
    { stoppedWallRange=raw;stoppedWallFrames=1;return false; }
    if (stoppedWallFrames<3) ++stoppedWallFrames;
    if (fabsf(raw-expected)>OBSTACLE_FINAL_PARKING_MAX_POSE_CORRECTION_MM &&
        stoppedWallFrames<3) return false;
    //571: the first two wall-like edge returns gave -17.6mm, while the
    //stopped wall was261mm versus258mm predicted. Never register Y from that
    //transitional edge sample after a newer stationary reference is available.
    latestWallPose=pose;
    latestWallRange=0.5f*(raw+stoppedWallRange);
    Serial.print("[FINAL PARK WALL] stopped_scan_range_mm=");Serial.println(latestWallRange,1);
    return true;
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
            OBSTACLE_FINAL_PARKING_MAX_X_POSE_CORRECTION_MM &&
        expectedWall > 0.0f &&
        fabsf(yCorrection) <= (stoppedWallFrames>=3 ? 40.0f:
            OBSTACLE_FINAL_PARKING_MAX_POSE_CORRECTION_MM);

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

bool finalFootprintContained(float additionalMargin=0.0f)
{
    const PositionEstimate pose = get_position_struct();
    if (!isfinite(pose.x_mm) || !isfinite(pose.y_mm) ||
        !isfinite(pose.heading_deg))
        return false;
    const float heading = pose.heading_deg * PI / 180.0f;
    const float c = cosf(heading);
    const float s = sinf(heading);
    const float localX[2] = {
        -OBSTACLE_FINAL_PARKING_REAR_OVERHANG_MM,
        OBSTACLE_FINAL_PARKING_FRONT_OVERHANG_MM};
    const float halfWidth =
        OBSTACLE_FINAL_PARKING_STRAIGHT_WIDTH_MM * 0.5f;
    const float localY[2] = {-halfWidth, halfWidth};
    const float margin = OBSTACLE_FINAL_PARKING_FINAL_BOUNDARY_MARGIN_MM + additionalMargin;
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

struct EntryCandidate
{
    ParkingSegment segments[ENTRY_PLAN_CAPACITY]{};
    uint8_t count = 0;
};

// Candidate construction is in the mirrored bay frame. Only complete,
// collision-checked candidates are committed to motor control.
void appendEntry(EntryCandidate &candidate, PositionEstimate &local,
                 int direction, int steering, float distance)
{
    if (distance < 0.5f) return;
    if (candidate.count && candidate.segments[candidate.count-1].steering==steering)
    {
        ParkingSegment &previous=candidate.segments[candidate.count-1];
        const float net=previous.direction*previous.distanceMm+direction*distance;
        if (fabsf(net)<0.5f) --candidate.count;
        else { previous.direction=net>0 ? 1:-1;previous.distanceMm=fabsf(net); }
    }
    else
    {
        if (candidate.count>=ENTRY_PLAN_CAPACITY) return;
        candidate.segments[candidate.count++] = {
            static_cast<int8_t>(direction), static_cast<int8_t>(steering), distance};
    }
    advanceEntryModel(local,steering,direction*distance,entryRadiusScale);
}

void appendEntryArc(EntryCandidate &candidate, PositionEstimate &local,
                    int direction, int steering, float heading)
{
    const float distance = -Ackermann::getTurnRadius(static_cast<float>(steering))*entryRadiusScale*
        wrap180(heading-local.heading_deg)*PI/180.0f/direction;
    if (distance > 0.0f)
        appendEntry(candidate,local,direction,steering,distance);
}

PositionEstimate entryLocalPose(const PositionEstimate &field)
{
    return PositionEstimate{
        routeTurnSign > 0 ? field.x_mm-movingInsideFaceX()
                         : OBSTACLE_PARKING_FIXED_INNER_FACE_X_MM-field.x_mm,
        field.y_mm-OBSTACLE_SOUTH_OUTER_WALL_Y_MM,
        wrap180(routeTurnSign > 0 ? field.heading_deg : 180.0f-field.heading_deg)};
}

bool entryEndReserveReady()
{
    const PositionEstimate p=entryLocalPose(get_position_struct());
    const float h=p.heading_deg*PI/180.0f;
    const float extra=0.5f*OBSTACLE_FINAL_PARKING_STRAIGHT_WIDTH_MM*fabsf(sinf(h));
    const float rear=p.x_mm-OBSTACLE_FINAL_PARKING_REAR_OVERHANG_MM*cosf(h)-extra;
    const float front=measuredGapMm-p.x_mm-OBSTACLE_FINAL_PARKING_FRONT_OVERHANG_MM*cosf(h)-extra;
    // Once parallel and fully contained, accept the actual parked position.
    // No centring move or preferred 30mm nose reserve is needed at standstill.
    // Retain the boundary margin and clearance for centring the front wheels.
    const float margin=OBSTACLE_FINAL_PARKING_FINAL_BOUNDARY_MARGIN_MM+3.0f;
    return finalFootprintContained(1.0f) && rear>=margin && front>=margin &&
        parkingEntryPoseSafe(get_position_struct(),0);
}

void sampleRearEntryReference(const PositionEstimate &pose)
{
    if(rearBrakeEpoch!=stateStartMs)
    {rearBrakeEpoch=stateStartMs;rearReferenceFrames=0;rearReferenceSequence=0;}
    if(!isfinite(measured_speed)||fabsf(measured_speed)>SOFT_STOP_SPEED_THRESHOLD_MMS)
    {rearReferenceFrames=0;return;}
    const PositionEstimate p=entryLocalPose(pose);
    if(fabsf(p.heading_deg)>12 || p.y_mm<30 || p.y_mm>185)
    {rearReferenceFrames=0;return;}
    const float h=p.heading_deg*PI/180.0f;
    const float sx=p.x_mm-OBSTACLE_REAR_TOF_BEHIND_AXLE_MM*cosf(h);
    const float sy=p.y_mm-OBSTACLE_REAR_TOF_BEHIND_AXLE_MM*sinf(h);
    // Entire conservative30deg fan must hit the rear pink plane, away from
    // both tips. At larger yaw it may see a wall/gap: never interpret that as
    // available reverse space. Uses cached M4 samples, no extra acquisition.
    for(float offset:{-15.0f,15.0f})
    {
        const float ray=h+offset*PI/180.0f;
        const float range=sx/cosf(ray),hitY=sy-range*sinf(ray);
        if(range<20 || range>REAR_TOF_MAX_RELIABLE_DISTANCE_MM || hitY<15 || hitY>185)
        {rearReferenceFrames=0;return;}
    }
    TofDiagnosticSnapshot s{};
    if(!get_tof_diagnostic_snapshot(TOF_REAR,s)||millis()-s.sampled_ms>200)
    {rearReferenceFrames=0;return;}
    if(s.sequence==rearReferenceSequence)return;
    rearReferenceSequence=s.sequence;
    // The rear RPC diagnostic has no object list; use its transport-validated
    // selected range and quality plus agreement with the M4 filtered value.
    const float raw=s.selected_raw_distance_mm;
    if(!isfinite(raw)||raw<20 || raw>REAR_TOF_MAX_RELIABLE_DISTANCE_MM ||
       !isfinite(s.filtered_distance_mm)||fabsf(raw-s.filtered_distance_mm)>8 ||
       !isfinite(s.selected_signal_mcps)||s.selected_signal_mcps<=0 ||
       !isfinite(s.selected_sigma_mm)||s.selected_sigma_mm<0 || s.selected_sigma_mm>15)
    {rearReferenceFrames=0;return;}
    const float axle=(raw+OBSTACLE_REAR_TOF_BEHIND_AXLE_MM)*cosf(h);
    for(float offset:{-15.0f,15.0f})
    {
        const float ray=h+offset*PI/180.0f;
        const float distance=(axle-OBSTACLE_REAR_TOF_BEHIND_AXLE_MM*cosf(h))/cosf(ray);
        const float hitY=sy-distance*sinf(ray);
        if(hitY<15 || hitY>185){rearReferenceFrames=0;return;}
    }
    if(!rearReferenceFrames || fabsf(axle-rearReferenceAxleMm)>4)
    {rearReferenceAxleMm=axle;rearReferenceFrames=1;return;}
    rearReferenceAxleMm=(rearReferenceAxleMm*rearReferenceFrames+axle)/(rearReferenceFrames+1);
    if(rearReferenceFrames<3)++rearReferenceFrames;
}

bool applyRearEntryReference()
{
    if(rearReferenceFrames<3)return false;
    const PositionEstimate pose=get_position_struct();
    const float correction=rearReferenceAxleMm-entryLocalPose(pose).x_mm;
    rearReferenceFrames=0;
    if(!isfinite(correction)||fabsf(correction)<1 || fabsf(correction)>45)return false;
    PositionEstimate corrected=pose;corrected.x_mm+=routeTurnSign*correction;
    if(!parkingEntryPoseSafe(corrected,set_degree,1.0f))return false;
    position_apply_xy_correction(routeTurnSign*correction,0);
    Serial.print("[FINAL PARK REAR] accepted_local_x_correction_mm=");Serial.println(correction,2);
    return true;
}

// Additional registration reserve for normal entry; a partly parked car
// retains the physical footprint even when it has already spent that reserve.
float entryPlanMargin(float margin,const PositionEstimate &field)
{
    return margin+(margin>=3 ? 3.0f:(entryLocalPose(field).y_mm>=250 ? 1.5f:0.0f));
}

bool entryCandidateSafe(const EntryCandidate &candidate, PositionEstimate field,
                        PositionEstimate *endpoints = nullptr, float sampleStep=1.0f)
{
    if (!endpoints && sampleStep==1.0f && candidate.count)
    {
        const float spread=entryRadiusMeasured ? 0.05f:0.15f;
        const float margin=entrySafetyMargin>=4.0f ? entrySafetyMargin-3.0f:
            (entrySafetyMargin==2.5f || entrySafetyMargin==2.0f ? 1.0f:entrySafetyMargin);
        PositionEstimate segmentStart=field;
        // A straight prefix must not hide an unusable following arc. Check
        //the same short radius envelope at every steering transition, not
        //just when the very first segment happens to be an arc.
        for(uint8_t i=0;i<candidate.count;++i)
        {
            const ParkingSegment &first=candidate.segments[i];
            // Tiny positioning prefixes can disappear into brake/settle
            // tolerance (632). Check the following arc from a position up
            // to 2mm short as well, rather than accepting the same trap again.
            for(float shortfall:{0.0f,2.0f})
            {
                if(shortfall && (!i || candidate.segments[i-1].steering ||
                    candidate.segments[i-1].distanceMm>=5.0f))continue;
                if(first.steering)for(float factor:{1.0f-spread,1.0f+spread})
                {
                    PositionEstimate predicted=segmentStart;
                    if(shortfall)advanceEntryModel(predicted,0,
                        -candidate.segments[i-1].direction*fminf(shortfall,candidate.segments[i-1].distanceMm),entryRadiusScale);
                    const float length=fminf(15.0f,first.distanceMm*factor);
                    for(float travel=0;travel<length;travel+=1.0f)
                    {
                        if(!entryPlanningBudget)return false;
                        --entryPlanningBudget;serviceEntryPlanner();
                        if(!parkingEntryPoseSafe(predicted,first.steering*routeTurnSign,margin))return false;
                        advanceEntryModel(predicted,first.steering*routeTurnSign,
                            first.direction*fminf(1.0f,length-travel),entryRadiusScale*factor);
                    }
                    if(!parkingEntryPoseSafe(predicted,first.steering*routeTurnSign,margin))return false;
                }
            }
            advanceEntryModel(segmentStart,first.steering*routeTurnSign,
                first.direction*first.distanceMm,entryRadiusScale);
        }
    }
    // Reject obviously blocked families cheaply. Every surviving candidate is
    // still checked again at1mm before it can become a motor command.
    if (!endpoints && sampleStep==1.0f &&
        !entryCandidateSafe(candidate,field,nullptr,10.0f)) return false;
    float walked=0.0f;
    const float extraReserve=entrySafetyMargin>=4.0f ? 3.0f:
        (entrySafetyMargin==2.5f ? 1.5f:(entrySafetyMargin==2.0f ? 1.0f:0.0f));
    for (uint8_t index=0; index<candidate.count; ++index)
    {
        const ParkingSegment &segment=candidate.segments[index];
        const int steering=segment.steering*routeTurnSign;
        // Include stationary steering changes and the complete swept motion.
        for (float travel=0; travel<segment.distanceMm; travel+=sampleStep)
        {
            if (!endpoints)
            {
                if (!entryPlanningBudget) return false;
                --entryPlanningBudget;
                serviceEntryPlanner();
            }
            const float margin=entrySafetyMargin-extraReserve*
                (1.0f-clampFloat(walked/20.0f,0.0f,1.0f));
            if (!parkingEntryPoseSafe(field,steering,margin)) return false;
            advanceEntryModel(field,steering,segment.direction*
                fminf(sampleStep,segment.distanceMm-travel),entryRadiusScale);
            walked+=fminf(sampleStep,segment.distanceMm-travel);
        }
        if (!parkingEntryPoseSafe(field,steering,entrySafetyMargin-extraReserve*
            (1.0f-clampFloat(walked/20.0f,0.0f,1.0f)))) return false;
        if (endpoints) endpoints[index]=field;
    }
    // Final check uses the whole bounding footprint, including the open edge.
    const PositionEstimate local=entryLocalPose(field);
    const float h=local.heading_deg*PI/180.0f,c=cosf(h),s=sinf(h);
    const float xs[]={-OBSTACLE_FINAL_PARKING_REAR_OVERHANG_MM,
                       OBSTACLE_FINAL_PARKING_FRONT_OVERHANG_MM};
    const float ys[]={-0.5f*OBSTACLE_FINAL_PARKING_STRAIGHT_WIDTH_MM,
                       0.5f*OBSTACLE_FINAL_PARKING_STRAIGHT_WIDTH_MM};
    const float margin=OBSTACLE_FINAL_PARKING_FINAL_BOUNDARY_MARGIN_MM+1.0f;
    for (float x:xs) for (float y:ys)
    {
        const float px=local.x_mm+c*x-s*y,py=local.y_mm+s*x+c*y;
        if (px<=margin || px>=measuredGapMm-margin || py<=margin ||
            py>=OBSTACLE_PARKING_WIDTH_MM-margin) return false;
    }
    return fabsf(local.heading_deg)<=OBSTACLE_FINAL_PARKING_FINAL_HEADING_TOLERANCE_DEG;
}

float entryCandidateCost(const EntryCandidate &candidate)
{
    float cost=0; int previous=0;
    for (uint8_t i=0;i<candidate.count;++i)
    {
        const ParkingSegment &segment=candidate.segments[i];
        cost+=segment.distanceMm+10.0f;
        if (previous && previous!=segment.direction) cost+=20.0f;
        previous=segment.direction;
    }
    return cost;
}

// Measured-pose free travel, including stationary wheel clearance and every
// intermediate footprint. This is a usable path length, not gap minus length:
// outside the rail tips a diagonal vehicle can have additional room.
float entryFreeTravel(const PositionEstimate &field, int direction, int steering,
                      float maximum=80.0f, bool countBudget=false)
{
    PositionEstimate predicted=field;
    const float reserve=fmaxf(2.0f,entryBrakeTravelMm);
    float walked=0;
    while(walked<=maximum)
    {
        if(countBudget){if(!entryPlanningBudget)break;--entryPlanningBudget;}
        serviceEntryPlanner();
        if(!parkingEntryPoseSafe(predicted,steering*routeTurnSign))break;
        if(walked==maximum)return fmaxf(0,walked-reserve);
        advanceEntryModel(predicted,steering*routeTurnSign,direction,entryRadiusScale);
        walked+=1;
    }
    return fmaxf(0,walked-1.0f-reserve);
}

bool planShortEntry(const PositionEstimate &field)
{
    const PositionEstimate start=entryLocalPose(field);
    if(start.y_mm<250 || fabsf(start.heading_deg)>10)return false;
    const uint32_t started=millis();
    for(float margin:{3.0f})
    {
        entrySafetyMargin=entryPlanMargin(margin,field);
        entryPlanningBudget=3500;entryPlannerLastPollMs=millis();
        EntryCandidate best;float cost=1.0e9f;
        // Two reverse arcs with a solved diagonal straight. No forward
        // correction arc is needed when this complete swept path fits.
        for(float high:{60.0f,65.0f,55.0f,70.0f,75.0f,80.0f})
        for(float initial:{0.0f,10.0f,20.0f,30.0f,40.0f})
        {
            if(!entryPlanningBudget)goto shortEntrySelected;
            PositionEstimate local=start;EntryCandidate arcs;
            appendEntry(arcs,local,-1,0,initial);
            appendEntryArc(arcs,local,-1,50,high);
            appendEntryArc(arcs,local,-1,-50,0);
            const float distance=(local.y_mm-OBSTACLE_FINAL_PARKING_TARGET_LOCAL_Y_MM)/sinf(high*PI/180);
            if(distance<0 || distance>160)continue;
            local=start;EntryCandidate candidate;
            appendEntry(candidate,local,-1,0,initial);
            appendEntryArc(candidate,local,-1,50,high);
            appendEntry(candidate,local,-1,0,distance);
            appendEntryArc(candidate,local,-1,-50,0);
            const float dx=OBSTACLE_FINAL_PARKING_TARGET_LOCAL_X_MM-local.x_mm;
            if(fabsf(dx)>100)continue;
            appendEntry(candidate,local,dx>0?1:-1,0,fabsf(dx));
            const float score=entryCandidateCost(candidate);
            if(score>=cost || !entryCandidateSafe(candidate,field))continue;
            best=candidate;cost=score;
        }
shortEntrySelected:
        if(!best.count)continue;
        entryCandidateSafe(best,field,adaptiveEndpoints);
        adaptiveEntryCount=best.count;
        for(uint8_t i=0;i<best.count;++i)adaptiveEntry[i]=best.segments[i];
        segmentIndex=0;entrySafetyMargin=margin;entryQuickRecipe=false;
        Serial.print("[FINAL PARK PLAN] Two-arc entry segments/margin/search_ms=");
        Serial.print(best.count);Serial.print("/");Serial.print(margin);
        Serial.print("/");Serial.println(millis()-started);
        return true;
    }
    entrySafetyMargin=3.0f;
    return false;
}

bool planEntryTail(const PositionEstimate &field, float targetY,
                   float oldMiddle, float oldLow, bool allowPrefix)
{
    const uint32_t startedMs=millis();
    const PositionEstimate local=entryLocalPose(field);
    if(local.heading_deg<12.0f || local.heading_deg>85.0f)return false;
    const float margin=entrySafetyMargin;
    // Try the accepted tail first, then nearby analytic tails. A measured
    // first arc can make the original intermediate headings infeasible.
    // Share a bounded budget across alternatives instead of restarting the
    // large general search for every small curvature correction.
    for(float checkedMargin:{margin,1.0f})
    {
        entryPlanningBudget=12000;entryPlannerLastPollMs=millis();
        entrySafetyMargin=entryPlanMargin(checkedMargin,field);
        for(float initial:{0.0f,10.0f,-10.0f,20.0f,-20.0f})
        for(float high:{local.heading_deg,70.0f,80.0f})
        for(float middle:{oldMiddle,20.0f,30.0f,40.0f,10.0f,0.0f})
        for(float low:{oldLow,10.0f,0.0f,20.0f})
        for(float solvedY:{targetY,OBSTACLE_FINAL_PARKING_TARGET_LOCAL_Y_MM,127.0f,125.0f,120.0f,115.0f,110.0f})
        {
            if(!entryPlanningBudget)break;
            if(!allowPrefix && initial!=0)continue;
            if(high<local.heading_deg || middle>high || low>middle)continue;
            PositionEstimate end=entryLocalPose(field);EntryCandidate arcs;
            appendEntry(arcs,end,initial>0?1:-1,0,fabsf(initial));
            appendEntryArc(arcs,end,-1,50,high);
            appendEntryArc(arcs,end,-1,-50,middle);
            appendEntryArc(arcs,end,1,50,low);
            appendEntryArc(arcs,end,-1,-50,0);
            const float distance=(end.y_mm-solvedY)/sinf(high*PI/180.0f);
            if(distance<0 || distance>160)continue;
            EntryCandidate candidate;end=entryLocalPose(field);
            appendEntry(candidate,end,initial>0?1:-1,0,fabsf(initial));
            appendEntryArc(candidate,end,-1,50,high);
            appendEntry(candidate,end,-1,0,distance);
            appendEntryArc(candidate,end,-1,-50,middle);
            appendEntryArc(candidate,end,1,50,low);
            appendEntryArc(candidate,end,-1,-50,0);
            const float dx=OBSTACLE_FINAL_PARKING_TARGET_LOCAL_X_MM-end.x_mm;
            if(fabsf(dx)>100)continue;
            appendEntry(candidate,end,dx>0?1:-1,0,fabsf(dx));
            if(!candidate.count || !entryCandidateSafe(candidate,field))continue;
            entryCandidateSafe(candidate,field,adaptiveEndpoints);
            adaptiveEntryCount=candidate.count;
            for(uint8_t i=0;i<candidate.count;++i)adaptiveEntry[i]=candidate.segments[i];
            segmentIndex=0;entryQuickRecipe=false;entrySafetyMargin=checkedMargin;
            Serial.print("[FINAL PARK PLAN] Remaining recipe adapted; full swept preflight PASS search_ms=");
            Serial.println(millis()-startedMs);
            return true;
        }
    }
    entrySafetyMargin=margin;
    return false;
}

// Bounded recovery for a diagonal car: the analytic recipe has only a few
// cusps and can miss a valid multi-point alignment with a larger real radius.
// Keep this stopped-only scratch off the control stack. Every accepted path
// still receives the ordinary complete 1mm/radius-envelope preflight.
bool planBoundedParkingRecovery(const PositionEstimate &field)
{
    struct Node { PositionEstimate pose; float cost,score; uint16_t parent;
        ParkingSegment move; uint8_t segments; bool closed; };
    static Node nodes[512];
    const float savedMargin=entrySafetyMargin;
    entrySafetyMargin=2.0f;
    entryPlanningBudget=65000;entryPlannerLastPollMs=millis();
    auto score=[](const PositionEstimate &p,float cost) {
        return .05f*cost+4.0f*fabsf(p.y_mm-OBSTACLE_FINAL_PARKING_TARGET_LOCAL_Y_MM)+
            1.0f*fabsf(p.heading_deg)+.4f*fabsf(p.x_mm-85.0f);
    };
    auto start=entryLocalPose(field);
    nodes[0]={start,0,score(start,0),0,{0,0,0},0,false};
    int count=1;
    for(int expansion=0;expansion<512 && entryPlanningBudget; ++expansion)
    {
        int selected=-1;float best=INFINITY;
        for(int i=0;i<count;++i)if(!nodes[i].closed && nodes[i].score<best)
        {selected=i;best=nodes[i].score;}
        if(selected<0)break;
        const Node current=nodes[selected];nodes[selected].closed=true;
        if(current.segments && fabsf(current.pose.heading_deg)<=2.0f &&
           current.pose.y_mm<140.0f)
        {
            ParkingSegment reverse[24];int n=0,at=selected;
            while(at && n<24){reverse[n++]=nodes[at].move;at=nodes[at].parent;}
            if(!at)
            {
                EntryCandidate candidate;auto local=start;
                while(n){auto m=reverse[--n];appendEntry(candidate,local,m.direction,m.steering,m.distanceMm);}
                if(hypotf(local.x_mm-current.pose.x_mm,local.y_mm-current.pose.y_mm)<0.1f &&
                   fabsf(wrap180(local.heading_deg-current.pose.heading_deg))<0.1f &&
                   entryCandidateSafe(candidate,field))
                {
                    entryCandidateSafe(candidate,field,adaptiveEndpoints);
                    adaptiveEntryCount=candidate.count;
                    for(uint8_t i=0;i<candidate.count;++i)adaptiveEntry[i]=candidate.segments[i];
                    segmentIndex=0;entryQuickRecipe=false;entrySafetyMargin=1.0f;
                    Serial.println("[FINAL PARK PLAN] Bounded multi-point recovery; complete swept preflight PASS");
                    return true;
                }
            }
        }
        for(int direction:{-1,1})for(int steering:{-50,0,50})
        for(int lengthChoice=0;lengthChoice<4;++lengthChoice)
        {
            float distance=lengthChoice==0 ? 20.0f:40.0f;
            if(lengthChoice==2)
            {
                if(!steering)continue;
                distance=-Ackermann::getTurnRadius(float(steering))*entryRadiusScale*
                    (-current.pose.heading_deg)*PI/180.0f/direction;
                if(distance<2 || distance>100)continue;
            }
            if(current.segments>=24)continue;
            PositionEstimate end;
            localToField(current.pose.x_mm,current.pose.y_mm,current.pose.heading_deg,
                end.x_mm,end.y_mm,end.heading_deg);
            if(lengthChoice==3)
            {
                entrySafetyMargin=current.cost<1 ? 1.0f:2.0f;
                distance=entryFreeTravel(end,direction,steering,80.0f,true);
                entrySafetyMargin=2.0f;
                if(distance<4)continue;
            }
            bool safe=true;
            for(float walked=0;walked<=distance;walked+=2)
            {
                if(!entryPlanningBudget){safe=false;break;}
                --entryPlanningBudget;serviceEntryPlanner();
                if(!parkingEntryPoseSafe(end,steering*routeTurnSign,
                    1.0f+clampFloat((current.cost+walked)/20.0f,0.0f,1.0f))){safe=false;break;}
                if(walked<distance)advanceEntryModel(end,steering*routeTurnSign,
                    direction*fminf(2,distance-walked),entryRadiusScale);
            }
            if(!safe || !parkingEntryPoseSafe(end,steering*routeTurnSign,2.0f))continue;
            const auto local=entryLocalPose(end);
            if(local.y_mm<65 || local.y_mm>320 || fabsf(local.heading_deg)>85)continue;
            const float cost=current.cost+distance+15+
                (current.segments && current.move.direction!=direction ? 20:0);
            int duplicate=-1;
            for(int i=0;i<count;++i)if(fabsf(nodes[i].pose.x_mm-local.x_mm)<7 &&
                fabsf(nodes[i].pose.y_mm-local.y_mm)<7 &&
                fabsf(nodes[i].pose.heading_deg-local.heading_deg)<4)
            {duplicate=i;break;}
            if(duplicate>=0)continue;
            int slot=count;
            if(count>=512)
            {
                float worst=-INFINITY;slot=-1;
                for(int i=0;i<count;++i)if(!nodes[i].closed && nodes[i].score>worst)
                {worst=nodes[i].score;slot=i;}
                if(slot<0 || score(local,cost)>=worst)continue;
            }
            else ++count;
            nodes[slot]={local,cost,score(local,cost),uint16_t(selected),
                {int8_t(direction),int8_t(steering),distance},uint8_t(current.segments+1),false};
        }
    }
    entrySafetyMargin=savedMargin;
    return false;
}

bool planParkingEntry(const PositionEstimate &field, bool allowEscape=true,
                      float requestedMargin=0.0f)
{
    const uint32_t planningStartedMs=millis();
    entryQuickRecipe=false;
    const PositionEstimate start=entryLocalPose(field);
    if (!isfinite(start.heading_deg) || fabsf(start.heading_deg)>85.0f)
        return false;
    if(allowEscape && requestedMargin==0 && planShortEntry(field))return true;
    // Partial entries use the same bounded continuation family before the
    // expensive general search. Prefixes are ordinary checked alternatives.
    if(allowEscape && requestedMargin==0 && entryRadiusMeasured && start.y_mm>130.0f && start.y_mm<280.0f &&
       planEntryTail(field,OBSTACLE_FINAL_PARKING_TARGET_LOCAL_Y_MM,30,20,true))
        return true;
    EntryCandidate best;
    float bestCost=1.0e9f;
    // Try a small repeatable parking recipe before the expensive general
    //search. Solve its straight and final X from the measured pose/gap;
    //every accepted recipe still passes the full1mm swept preflight.
    if (allowEscape && requestedMargin==0.0f)
    {
        for (float margin:{3.0f,1.0f})
        {
            // Tight quick recipes are for the normal capture only; partial
            //entry recovery retains the general escape search ordering.
            if(margin<3.0f && start.y_mm<250.0f)continue;
            entryPlanningBudget=6000;entryPlannerLastPollMs=millis();
            entrySafetyMargin=entryPlanMargin(margin,field);
            EntryCandidate quickBest;float quickCost=1.0e9f;
            for(float high:{55.0f,60.0f,70.0f,50.0f,80.0f})
            for(float middle:{30.0f,20.0f,40.0f})
            for(float low:{20.0f,10.0f,0.0f})
            for(float initial:{-32.0f,-20.0f,-40.0f,0.0f})
            for(float targetY:{OBSTACLE_FINAL_PARKING_TARGET_LOCAL_Y_MM,127.0f,125.0f,120.0f,115.0f})
            {
                if(!entryPlanningBudget)goto quickEntrySelected;
                if(high<start.heading_deg)continue;
                EntryCandidate arcs;PositionEstimate end=start;
                appendEntry(arcs,end,-1,0,fabsf(initial));
                appendEntryArc(arcs,end,-1,50,high);
                appendEntryArc(arcs,end,-1,-50,middle);
                appendEntryArc(arcs,end,1,50,low);
                appendEntryArc(arcs,end,-1,-50,0);
                const float length=(end.y_mm-targetY)/sinf(high*PI/180.0f);
                if(length<0 || length>160)continue;
                EntryCandidate candidate;PositionEstimate local=start;
                appendEntry(candidate,local,-1,0,fabsf(initial));
                appendEntryArc(candidate,local,-1,50,high);
                appendEntry(candidate,local,-1,0,length);
                appendEntryArc(candidate,local,-1,-50,middle);
                appendEntryArc(candidate,local,1,50,low);
                appendEntryArc(candidate,local,-1,-50,0);
                const float dx=OBSTACLE_FINAL_PARKING_TARGET_LOCAL_X_MM-local.x_mm;
                if(fabsf(dx)>100)continue;
                appendEntry(candidate,local,dx>0?1:-1,0,fabsf(dx));
                const float score=entryCandidateCost(candidate);
                if(!candidate.count || score>=quickCost || !entryCandidateSafe(candidate,field))continue;
                // Improve the first safe recipe within a small additional
                // budget; never exhaust the broad search just to rank it.
                if(!quickBest.count && entryPlanningBudget>1500)entryPlanningBudget=1500;
                quickBest=candidate;quickCost=score;
            }
quickEntrySelected:
            if(quickBest.count)
            {
                adaptiveEntryCount=quickBest.count;
                for(uint8_t i=0;i<quickBest.count;++i)adaptiveEntry[i]=quickBest.segments[i];
                entryCandidateSafe(quickBest,field,adaptiveEndpoints);
                entrySafetyMargin=margin;segmentIndex=0;
                entryQuickRecipe=true;
                Serial.print("[FINAL PARK PLAN] Checked quick recipe segments/margin/search_ms=");
                Serial.print(quickBest.count);Serial.print("/");Serial.print(margin,1);
                Serial.print("/");Serial.println(millis()-planningStartedMs);
                update_gyro();return true;
            }
        }
    }
    // First exhaust the ordinary reserve. Tight mode still preserves positive
    // body/rail clearance and the normal final containment margin.
    for (float margin:{3.0f,1.0f})
    {
        if (requestedMargin>0.0f && margin!=requestedMargin) continue;
        // Bound worst-case work while stopped, including escape children.
        // Each reserve level gets its own budget, so a difficult normal search
        // cannot prevent the requested last-resort tight-clearance attempt.
        if (requestedMargin==0.0f) { entryPlanningBudget=20000;entryPlannerLastPollMs=millis(); }
        // Reserve an additional3mm on both footprint and pieces while
        // planning for localization/endpoint rounding. Runtime retains margin.
        entrySafetyMargin=entryPlanMargin(margin,field);
        // ENTRY_FIRST_STEERING_CHECKED_BY_SWEEP
        // A partly parked robot may fit with its current steering but not
        // with neutral wheels. Do not require an uncommanded neutral pose:
        // each candidate checks its first steering and entire swept path.
        // Already entering the bay: try the short remaining alignment first.
        for (int direction:{-1,1})
        {
            EntryCandidate candidate; PositionEstimate local=start;
            if (fabsf(local.heading_deg)>0.2f)
                appendEntryArc(candidate,local,direction,
                    local.heading_deg*direction>0 ? 50:-50,0.0f);
            const float dx=OBSTACLE_FINAL_PARKING_TARGET_LOCAL_X_MM-local.x_mm;
            appendEntry(candidate,local,dx>0?1:-1,0,fabsf(dx));
            const float cost=entryCandidateCost(candidate);
            if (candidate.count && cost<bestCost && entryCandidateSafe(candidate,field))
            { best=candidate;bestCost=cost; }
        }
        if (!best.count)
        {
        // Analytic multi-point families. The high-angle straight is solved
        // from the desired final Y; the last straight centres between rails.
        // The bounded enumeration needs only a small candidate on the stack.
        const float highs[]={start.heading_deg,10,20,30,40,50,55,60,65,70,75,80};
        const float rearRoom=entryFreeTravel(field,-1,0,measuredGapMm);
        const float frontRoom=entryFreeTravel(field,1,0,measuredGapMm);
        for (float initial:{0.0f,-10.0f,-20.0f,-30.0f,-40.0f,10.0f,20.0f,30.0f,40.0f,
                           -rearRoom,frontRoom,-rearRoom*0.5f,frontRoom*0.5f})
        for (float high:highs)
        for (float middle:{0.0f,5.0f,10.0f,15.0f,20.0f,25.0f,30.0f,35.0f,40.0f})
        for (float low:{0.0f,5.0f,10.0f,15.0f,20.0f,25.0f})
        {
            if (!entryPlanningBudget || (allowEscape && requestedMargin==0.0f && entryPlanningBudget<10000))
                goto entryPlanSelected;
            if (high<start.heading_deg-0.1f || high<middle || middle<low || high<1.0f)
                continue;
            serviceEntryPlanner();
            EntryCandidate arcs; PositionEstimate arcEnd=start;
            appendEntry(arcs,arcEnd,initial>0?1:-1,0,fabsf(initial));
            appendEntryArc(arcs,arcEnd,-1,50,high);
            appendEntryArc(arcs,arcEnd,-1,-50,middle);
            appendEntryArc(arcs,arcEnd,1,50,low);
            appendEntryArc(arcs,arcEnd,-1,-50,0);
            // Prefer the shallow open-edge target. Keep deeper checked tails
            // available when an already diagonal car cannot safely reach it.
            const float targetYs[]={start.y_mm<250 ? 115.0f:OBSTACLE_FINAL_PARKING_TARGET_LOCAL_Y_MM,
                start.y_mm<250 ? 110.0f:127.0f,
                OBSTACLE_FINAL_PARKING_TARGET_LOCAL_Y_MM,125.0f,120.0f,110.0f};
            for (uint8_t targetIndex=0;targetIndex<6;++targetIndex)
            {
                const float targetY=targetYs[targetIndex];
                const float length=(arcEnd.y_mm-targetY)/sinf(high*PI/180.0f);
                if (length<0 || length>160) continue;
                EntryCandidate candidate; PositionEstimate local=start;
                appendEntry(candidate,local,initial>0?1:-1,0,fabsf(initial));
                appendEntryArc(candidate,local,-1,50,high);
                appendEntry(candidate,local,-1,0,length);
                appendEntryArc(candidate,local,-1,-50,middle);
                appendEntryArc(candidate,local,1,50,low);
                appendEntryArc(candidate,local,-1,-50,0);
                const float dx=OBSTACLE_FINAL_PARKING_TARGET_LOCAL_X_MM-local.x_mm;
                if (fabsf(dx)>100.0f) continue;
                appendEntry(candidate,local,dx>0?1:-1,0,fabsf(dx));
                const float cost=entryCandidateCost(candidate);
                if (cost>=650.0f || !entryCandidateSafe(candidate,field)) continue;
                best=candidate;bestCost=cost;
                goto entryPlanSelected;
            }
        }
        }
entryPlanSelected:
        if (best.count)
        {
            adaptiveEntryCount=best.count;
            for (uint8_t i=0;i<best.count;++i) adaptiveEntry[i]=best.segments[i];
            entryCandidateSafe(best,field,adaptiveEndpoints);
            entrySafetyMargin=margin;
            segmentIndex=0;
            Serial.print("[FINAL PARK PLAN] segments/margin/cost=");
            Serial.print(best.count);Serial.print("/");Serial.print(margin,1);
            Serial.print("/");Serial.print(bestCost,1);
            Serial.print(" search_ms=");Serial.println(millis()-planningStartedMs);
            update_gyro();
            return true;
        }
    if (allowEscape)
    {
        // A late partial entry may first need to undo a short arc. Check that
        // prefix as well as a complete subsequent entry; never commit a blind
        // retreat merely because the ordinary family did not fit.
        // Prefer the forward arc away from the wall in the mirrored bay frame.
        for (int direction:{1,-1}) for (int steering:{-50,0,50})
        for (float distance:{40.0f,60.0f,20.0f,30.0f,10.0f,80.0f})
        {
            if (!entryPlanningBudget) break;
            entrySafetyMargin=margin;
            PositionEstimate end=field; bool safe=true;
            for (int mm=0;mm<=static_cast<int>(distance);++mm)
            {
                serviceEntryPlanner();
                if (!parkingEntryPoseSafe(end,steering*routeTurnSign)) {safe=false;break;}
                if (mm<distance) advanceEntryModel(end,steering*routeTurnSign,static_cast<float>(direction),entryRadiusScale);
            }
            if (!safe) continue;
            const uint32_t remainingBudget=entryPlanningBudget;
            entryPlanningBudget=remainingBudget<5000 ? remainingBudget:5000;
            const uint32_t childBudget=entryPlanningBudget;
            const bool planned=planParkingEntry(end,false,margin);
            const uint32_t usedBudget=childBudget-entryPlanningBudget;
            entryPlanningBudget=remainingBudget-usedBudget;
            if (!planned || adaptiveEntryCount>=ENTRY_PLAN_CAPACITY) continue;
            EntryCandidate combined;PositionEstimate local=entryLocalPose(field);
            appendEntry(combined,local,direction,steering,distance);
            for (uint8_t i=0;i<adaptiveEntryCount;++i)
                appendEntry(combined,local,adaptiveEntry[i].direction,
                    adaptiveEntry[i].steering,adaptiveEntry[i].distanceMm);
            if (!entryCandidateSafe(combined,field,adaptiveEndpoints)) continue;
            adaptiveEntryCount=combined.count;
            for (uint8_t i=0;i<combined.count;++i) adaptiveEntry[i]=combined.segments[i];
            segmentIndex=0;
            Serial.print("[FINAL PARK PLAN] Complete swept alternative with escape prefix search_ms=");
            Serial.println(millis()-planningStartedMs);
            return true;
        }
    }
    }
    if(allowEscape && requestedMargin==0 && start.y_mm<250.0f &&
       planBoundedParkingRecovery(field))return true;
    adaptiveEntryCount=0;entrySafetyMargin=3.0f;
    return false;
}

const ParkingSegment &activeEntrySegment()
{
    return adaptiveEntryCount ? adaptiveEntry[segmentIndex] : PARKING_SEGMENTS[segmentIndex];
}

// After the first recipe arc, solve the remaining straight from the actual
//pose and measured radius.616 otherwise restarted the general8.8s search.
bool refreshRemainingEntry(const PositionEstimate &field);
bool adaptRemainingEntryRecipe(const PositionEstimate &field)
{
    if(!entryQuickRecipe)
        return refreshRemainingEntry(field) ||
            planEntryTail(field,OBSTACLE_FINAL_PARKING_TARGET_LOCAL_Y_MM,30,20,false);
    if(!entryQuickRecipe || segmentIndex>1 || segmentIndex+3>=adaptiveEntryCount ||
       !adaptiveEntry[segmentIndex].steering ||
       adaptiveEntry[segmentIndex+1].steering ||
       adaptiveEntry[segmentIndex+1].direction!=-1)return false;
    const float targetY=entryLocalPose(adaptiveEndpoints[adaptiveEntryCount-1]).y_mm;
    PositionEstimate local=entryLocalPose(field);
    const float sine=sinf(local.heading_deg*PI/180.0f);
    if(sine<0.2f)return false;
    return planEntryTail(field,targetY,
        entryLocalPose(adaptiveEndpoints[segmentIndex+2]).heading_deg,
        entryLocalPose(adaptiveEndpoints[segmentIndex+3]).heading_deg,false);
}

// Reuse the already accepted remaining steering sequence after a measured
// deviation. Re-solve its first diagonal straight and final centring movement;
// accept only a complete fresh sweep, never blindly keep stale endpoints.
bool refreshRemainingEntry(const PositionEstimate &field)
{
    if(!adaptiveEntryCount || segmentIndex+1>=adaptiveEntryCount)return false;
    const uint8_t first=segmentIndex+1;
    const float targetY=entryLocalPose(adaptiveEndpoints[adaptiveEntryCount-1]).y_mm;
    const bool solveStraight=!adaptiveEntry[first].steering && first+1<adaptiveEntryCount;
    PositionEstimate end=entryLocalPose(field);EntryCandidate arcs;
    for(uint8_t i=first;i<adaptiveEntryCount;++i)
    {
        const ParkingSegment &segment=adaptiveEntry[i];
        if(segment.steering)appendEntryArc(arcs,end,segment.direction,segment.steering,
            entryLocalPose(adaptiveEndpoints[i]).heading_deg);
        else if(i+1<adaptiveEntryCount && !(solveStraight && i==first))
            appendEntry(arcs,end,segment.direction,0,segment.distanceMm);
    }
    float solved=0;
    if(solveStraight)
    {
        const float dy=adaptiveEntry[first].direction*sinf(entryLocalPose(field).heading_deg*PI/180);
        if(fabsf(dy)<0.2f)return false;
        solved=(targetY-end.y_mm)/dy;
        if(solved<0 || solved>160)return false;
    }
    EntryCandidate candidate;end=entryLocalPose(field);
    for(uint8_t i=first;i<adaptiveEntryCount;++i)
    {
        const ParkingSegment &segment=adaptiveEntry[i];
        if(segment.steering)appendEntryArc(candidate,end,segment.direction,segment.steering,
            entryLocalPose(adaptiveEndpoints[i]).heading_deg);
        else if(i+1<adaptiveEntryCount)
            appendEntry(candidate,end,segment.direction,0,solveStraight&&i==first ? solved:segment.distanceMm);
    }
    if(fabsf(end.heading_deg)>1.0f)return false;
    const float dx=entryLocalPose(adaptiveEndpoints[adaptiveEntryCount-1]).x_mm-end.x_mm;
    if(fabsf(dx)>100)return false;
    appendEntry(candidate,end,dx>0?1:-1,0,fabsf(dx));
    const float margin=entrySafetyMargin;
    for(float check:{margin,1.0f})
    {
        entrySafetyMargin=entryPlanMargin(check,field);
        entryPlanningBudget=2500;entryPlannerLastPollMs=millis();
        if(!candidate.count || !entryCandidateSafe(candidate,field))continue;
        entryCandidateSafe(candidate,field,adaptiveEndpoints);
        adaptiveEntryCount=candidate.count;
        for(uint8_t i=0;i<candidate.count;++i)adaptiveEntry[i]=candidate.segments[i];
        segmentIndex=0;entrySafetyMargin=check;entryQuickRecipe=false;
        Serial.println("[FINAL PARK PLAN] Remaining steering sequence refreshed; full swept preflight PASS");
        return true;
    }
    entrySafetyMargin=margin;return false;
}

bool updateEntryRadiusFromMotion()
{
    const ParkingSegment &segment=activeEntrySegment();
    const float angle=fabsf(wrap180(get_position_struct().heading_deg-entryStartHeading))*PI/180.0f;
    const float travel=distanceSince(stateStartDistance);
    if (!segment.steering || travel<=15.0f || angle<=0.08f) return false;
    const float scale=clampFloat(travel/(angle*fabsf(Ackermann::getTurnRadius(
        static_cast<float>(segment.steering)))),0.80f,1.50f);
    const bool changed=fabsf(scale-entryRadiusScale)>0.025f;
    entryRadiusScale=scale;entryRadiusMeasured=true;
    Serial.print("[FINAL PARK CURVATURE] measured_radius_scale=");Serial.println(scale,3);
    return changed;
}

void updateEntryBrakeTravel()
{
    // A hold controller may pull back after braking. Only settled movement
    // in the preceding drive direction is an overrun, not an opposite rebound.
    const float travel=(get_distance()-entryBrakeStartDistance)*entryBrakeDirection;
    entryBrakeTravelMm=isfinite(travel) ? clampFloat(travel,0.0f,8.0f):0.0f;
    Serial.print("[FINAL PARK BRAKE] settled_lead_mm=");Serial.println(entryBrakeTravelMm,2);
}

void requestEntryReplan(const char *reason)
{
    entryReplanHadMotion=state==FP_SEGMENT_DRIVE;
    if (entryReplanHadMotion)
    { entryBrakeStartDistance=get_distance();entryBrakeDirection=activeEntrySegment().direction; }
    stop(true);
    entryReplanReason=reason;
    stateStartMs=millis();
    state=FP_REPLAN_BRAKE;
    Serial.print("[FINAL PARK REPLAN] ");Serial.println(reason);
}

void beginEntrySettle()
{
    const int steering=activeEntrySegment().steering*routeTurnSign;
    if (!parkingEntryPoseSafe(get_position_struct(),steering))
    {requestEntryReplan("stationary_steering_clearance");return;}
    servo_disabled=false;set_steering(steering);steer(steering);
    stateStartMs=millis();state=FP_SEGMENT_SETTLE;
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
    scanLineEstablished = false;
    scanWallRegistered = false;
    scanAlignSign = 1;
    scanAlignTurns = 0;
    scanAlignStartMs = 0;
    movingCandidateStored = false;
    latestWallRange = 0.0f;
    measuredFixedInsideX = 0.0f;
    measuredMovingInsideX = 0.0f;
    measuredGapMm = 0.0f;
    captureTargetX = 0.0f;
    captureFieldMotionSign = 1;
    captureAlignPending = false;
    completed = false;
    aborted = false;
    practiceMode = false;
    initialBrakeStarted = false;
    approachOuterShiftX = OBSTACLE_FINAL_PARKING_OUTER_SHIFT_X_MM;
    approachDirectLane=false;
    wallSequence = finalTraceMs = 0;
    finalTraceCount = initialWallFrames = 0;
    initialWallCorrection = 0.0f;
    lastWallCorrection = 0.0f;
    wallCorrectionSequence = 0;
    stoppedWallSequence=0;stoppedWallFrames=0;stoppedWallRange=0.0f;
    adaptiveEntryCount=0;entryReplans=0;entrySafetyMargin=3.0f;entryQuickRecipe=false;
    entryStalledReplans=0;entryBestReplanError=INFINITY;
    entryReplanReason=nullptr;
    entryRadiusScale=1.0f;entryStartHeading=0.0f;
    entryRadiusMeasured=false;
    entryReplanHadMotion=false;
    entryBrakeEvidenceValid=false;
    entryBrakeStartDistance=entryBrakeTravelMm=0.0f;
    entryBrakeDirection=1;
    entryPlanningBudget=0;
    rearBrakeEpoch=rearReferenceSequence=0;rearReferenceFrames=0;rearReferenceAxleMm=0;
    sensorHoldTraceCount = 0;
    sensorHoldPending = false;
    sensorHoldStartMs = 0;
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

void final_parking_start_entry_practice(int8_t turn_sign)
{
    final_parking_reset();
    practiceMode=true;
    routeTurnSign=turn_sign<0 ? -1:1;
    outerSensor=routeTurnSign>0 ? TOF_RIGHT:TOF_LEFT;
    // The tester sets the measured rail gap and axle pose from the setup
    // sheet. Unlike normal parking, this mode deliberately omits localization.
    scanPhase=5;
    measuredGapMm=OBSTACLE_FINAL_PARKING_GAP_MM;
    PositionEstimate start{};
    localToField(OBSTACLE_FINAL_PARKING_CAPTURE_LOCAL_X_MM,
        OBSTACLE_FINAL_PARKING_CAPTURE_LOCAL_Y_MM,0,
        start.x_mm,start.y_mm,start.heading_deg);
    position_reset(start.x_mm,start.y_mm,start.heading_deg);
    captureTargetX=start.x_mm;
    runStartDistance=stateStartDistance=get_distance();
    stateStartMs=millis();
    state=FP_CAPTURE_BRAKE;
    stop(true);
    Serial.println("[FINAL PARK] revision=20261105_measured_parking");
    Serial.print("[FINAL PARK PRACTICE] entry_only=yes turn=");
    Serial.print(routeTurnSign>0 ? "CCW":"CW");
    Serial.print(" gap_mm=");Serial.print(measuredGapMm,1);
    Serial.print(" local_axle_x_y_mm=");
    Serial.print(OBSTACLE_FINAL_PARKING_CAPTURE_LOCAL_X_MM,2);Serial.print("/");
    Serial.println(OBSTACLE_FINAL_PARKING_CAPTURE_LOCAL_Y_MM,2);
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
    if (state == FP_IDLE && !initialBrakeStarted)
    {
        routeTurnSign = turn_sign < 0 ? -1 : 1;
        outerSensor = routeTurnSign > 0 ? TOF_RIGHT : TOF_LEFT;
        initialBrakeStarted = true;
        stateStartMs = millis();
        // Retain the challenge's existing position-hold target while settling.
        // Reissuing stop(true) each tick would move that target with the coast.
        if (dc_state != DC_HOLDING) stop(true);
    }
    // Include the initial preflight and final verification in the same health
    // gate as motion. NaN coordinates must never count as contained/stopped.
    if ((state == FP_IDLE || state == FP_VERIFY) && !checkMotionHealth(pose))
        return true;
    if (state == FP_IDLE)
    {
        if (!parkingBrakeReady(0)) return true;
        // Position HOLD retains the old driving speed profile. Only after
        // measured standstill, clear it before the low-speed parking command
        // (CW may reverse relative to the completed lap).
        stop(false);
        if (!chooseParkingApproach(pose))
        {
            abortParking("approach_swept_preflight");
            return true;
        }
        Serial.println("[FINAL PARK] Learned-route approach swept preflight PASS");
        Serial.println("[FINAL PARK] revision=20261105_measured_parking");
        // The stopped geometric search may outlast the 200-ms sensor age.
        // Service queued reports before attempting the first motion; a
        // genuinely stale/missing report still follows the normal hold.
        update_gyro();
        Serial.print("[FINAL PARK] Outward shift x="); Serial.println(approachOuterShiftX,1);
        Serial.print("[FINAL PARK] Direct staging lane=");Serial.println(approachDirectLane ? "yes":"no");
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
    traceParking();
    if (((state >= FP_APPROACH_LANE && state <= FP_SEGMENT_BRAKE) ||
         state == FP_SCAN_ALIGN_DRIVE || state == FP_SCAN_ALIGN_BRAKE ||
         state == FP_CAPTURE_ALIGN_DRIVE || state == FP_CAPTURE_ALIGN_BRAKE ||
         state == FP_REPLAN_BRAKE) &&
        !checkMotionHealth(pose))
        return true;

    if (state == FP_APPROACH_LANE)
    {
        // Register lateral drift on the cleared, straight approach before
        // the outward bend; correcting it only at the distant scan endpoint
        // can consume the remaining room beside the next corner.
        if (initialWallFrames < 2) updateOuterWallReference(true);
        pose = get_position_struct();
        if (practiceMode)
            driveFieldLine(+1, OBSTACLE_FINAL_PARKING_APPROACH_LANE_Y_MM,
                           OBSTACLE_FINAL_PARKING_APPROACH_SPEED);
        else
        {
            int steering=0;
            if (!parkingApproachSteering(pose,false,steering))
            { abortParking("learned_approach_target"); return true; }
            if (!parkingApproachCommandSafe(pose,steering,motorDirectionForFieldMotion(+1)))
            { abortParking("approach_motion_clearance"); return true; }
            servo_disabled = false;
            set_steering(steering);
            set_speed(motorDirectionForFieldMotion(+1)*OBSTACLE_FINAL_PARKING_APPROACH_SPEED);
        }
        if (pose.x_mm >= approachOuterShiftX)
        {
            state = FP_APPROACH_OUTER;
            Serial.println("[FINAL PARK] Swept-checked outward shift");
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
        int steering=0;
        if (!parkingApproachSteering(pose,true,steering))
        { abortParking("outer_approach_target"); return true; }
        if (!practiceMode && !parkingApproachCommandSafe(pose,steering,motorDirectionForFieldMotion(+1)))
        { abortParking("outer_motion_clearance"); return true; }
        servo_disabled = false;
        set_steering(steering);
        set_speed(motorDirectionForFieldMotion(+1)*OBSTACLE_FINAL_PARKING_APPROACH_SPEED);
        if (pose.x_mm >= OBSTACLE_FINAL_PARKING_SCAN_START_X_MM &&
            linePoseReady(pose, OBSTACLE_FINAL_PARKING_SCAN_Y_MM))
        {
            stop(true);
            stateStartMs = millis();
            if (!scanWallRegistered) initialWallFrames = 0;
            state = FP_APPROACH_BRAKE;
        }
        else if (distanceSince(runStartDistance) >=
                     OBSTACLE_FINAL_PARKING_APPROACH_MAX_MM ||
                 pose.x_mm > 1100.0f ||
                 millis() - stateStartMs >=
                     OBSTACLE_FINAL_PARKING_APPROACH_TIMEOUT_MS)
            abortParking("approach_outer_gate");
        return true;
    }

    if (state == FP_APPROACH_BRAKE)
    {
        if (!parkingBrakeReady(OBSTACLE_FINAL_PARKING_HOLD_BRAKE_MS))
            return true;
        if (!scanWallRegistered && initialWallFrames < 2 && !updateOuterWallReference(true))
        {
            if (millis()-stateStartMs > 2500)
                abortParking("initial_outer_wall_reference");
            return true;
        }
        scanWallRegistered = true;
        // Registration can move Y by tens of millimetres (log 510: -25.4).
        // Regain the scan line in the checked outward corridor BEFORE passing
        // either marker. Keep the existing distance/X/time bounds in force.
        if (!linePoseReady(get_position_struct(), OBSTACLE_FINAL_PARKING_SCAN_Y_MM))
        {
            stop(false);
            servo_disabled = false;
            scanAlignSign = pose.x_mm < 1000.0f ? 1 : -1;
            scanAlignStartMs = millis();
            scanAlignTurns = 0;
            state = FP_SCAN_ALIGN_DRIVE;
            Serial.println("[FINAL PARK] Re-align outward after wall registration");
            return true;
        }
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
        // Odometry drifts longitudinally over three laps. Start looking while
        // still on the established wall baseline, not at a guessed marker X.
        scanArmed = true;
        scanLineEstablished = false;
        movingCandidateStored = false;
        stateStartDistance = get_distance();
        stateStartMs = millis();
        state = FP_SCAN_DRIVE;
        Serial.println("[FINAL PARK] Reverse field scan started geometry=tip_plane_v1");
        return true;
    }

    if (state == FP_SCAN_ALIGN_DRIVE || state == FP_SCAN_ALIGN_BRAKE)
    {
        // Stay beyond even a +250 mm longitudinal registration error plus
        // sensor/cone reach. Reversals only use the checked free outer corridor.
        if (millis()-scanAlignStartMs > 24000UL || scanAlignTurns > 6)
        { abortParking("scan_alignment_limit"); return true; }
        if (state == FP_SCAN_ALIGN_BRAKE)
        {
            if (!parkingBrakeReady(OBSTACLE_FINAL_PARKING_HOLD_BRAKE_MS)) return true;
            stop(false);
            if (linePoseReady(pose,OBSTACLE_FINAL_PARKING_SCAN_Y_MM))
            {
                servo_disabled=false;set_steering(0);steer(0);
                stateStartMs=millis();state=FP_SCAN_SETTLE;return true;
            }
            scanAlignSign=-scanAlignSign;
            ++scanAlignTurns;
            state=FP_SCAN_ALIGN_DRIVE;
        }
        driveFieldLine(scanAlignSign,OBSTACLE_FINAL_PARKING_SCAN_Y_MM,
            OBSTACLE_FINAL_PARKING_ALIGN_SPEED);
        if (!scanCommandSafe(pose,motorDirectionForFieldMotion(scanAlignSign)))
        { abortParking("scan_alignment_clearance"); return true; }
        const bool boundary=scanAlignSign>0 ? pose.x_mm>=1080.0f : pose.x_mm<=800.0f;
        const bool ready=pose.x_mm>=800.0f && linePoseReady(pose,OBSTACLE_FINAL_PARKING_SCAN_Y_MM);
        if (boundary || ready)
        { stop(true);stateStartMs=millis();state=FP_SCAN_ALIGN_BRAKE; }
        return true;
    }

    if (state == FP_SCAN_DRIVE)
    {
        updateOuterWallReference(false);
        pose = get_position_struct();
        driveFieldLine(-1, OBSTACLE_FINAL_PARKING_SCAN_Y_MM,
                       OBSTACLE_FINAL_PARKING_SCAN_SPEED);
        if (!scanCommandSafe(pose,motorDirectionForFieldMotion(-1)))
        {
            // A small line error can make the steered wheel sweep unsafe
            // although a neutral retreat still clears the tips (601).
            set_steering(0);
            if (!scanCommandSafe(pose,motorDirectionForFieldMotion(-1)))
            { abortParking("scan_motion_clearance"); return true; }
        }
        if (!scanArmed)
        {
            scanArmed = true;
            scanPhase = 0;
            classFrames = 0;
            Serial.println("[FINAL PARK] Dual-marker scan armed");
        }
        TofDiagnosticSnapshot snapshot;
        if (!scanLineEstablished &&
            linePoseReady(pose,OBSTACLE_FINAL_PARKING_SCAN_Y_MM))
            scanLineEstablished = true;
        if (scanArmed && scanLineEstablished &&
            get_tof_diagnostic_snapshot(outerSensor, snapshot) &&
            snapshot.sequence != tofSequence &&
            millis()-snapshot.sampled_ms <= 250)
        {
            tofSequence = snapshot.sequence;
            processScanFrame(snapshot);
        }
        if (scanPhase == 5)
        {
            stop(true);
            stoppedWallSequence=0;stoppedWallFrames=0;
            stateStartMs = millis();
            state = FP_SCAN_BRAKE;
        }
        // The markers, not accumulated three-lap X odometry, define the bay.
        // Permit the same bounded registration error at the search endpoint.
        else if (pose.x_mm <= OBSTACLE_FINAL_PARKING_SCAN_END_X_MM -
                                  OBSTACLE_FINAL_PARKING_MAX_X_POSE_CORRECTION_MM ||
                 distanceSince(stateStartDistance) >=
                     OBSTACLE_FINAL_PARKING_SCAN_MAX_MM ||
                 millis() - stateStartMs >=
                     OBSTACLE_FINAL_PARKING_SCAN_TIMEOUT_MS)
            abortParking("dual_marker_scan_incomplete");
        return true;
    }

    if (state == FP_SCAN_BRAKE)
    {
        if (!parkingBrakeReady(OBSTACLE_FINAL_PARKING_HOLD_BRAKE_MS))
            return true;
        stop(false);
        if (!refreshStoppedScanWallReference())
        {
            if (millis()-stateStartMs>2500UL) abortParking("stopped_scan_wall_reference");
            return true;
        }
        if (!applyScanLocalization())
        {
            abortParking("dual_marker_geometry");
            return true;
        }
        captureAlignPending=!linePoseReady(get_position_struct(),
            OBSTACLE_FINAL_PARKING_SCAN_Y_MM);
        if (captureAlignPending && captureDirectPreflight(get_position_struct()))
        {
            captureAlignPending=false;
            Serial.println("[FINAL PARK] Registered capture: direct alignment swept preflight PASS");
        }
        if (captureAlignPending && !captureAlignmentPreflight(get_position_struct()))
        { abortParking("capture_alignment_preflight"); return true; }
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
        state = captureAlignPending ? FP_CAPTURE_ALIGN_DRIVE : FP_CAPTURE_DRIVE;
        Serial.print("[FINAL PARK] Capture target x=");
        Serial.println(captureTargetX, 2);
        return true;
    }

    if (state == FP_CAPTURE_ALIGN_DRIVE)
    {
        pose=get_position_struct();
        driveFieldLine(+1,OBSTACLE_FINAL_PARKING_SCAN_Y_MM,
            OBSTACLE_FINAL_PARKING_CAPTURE_SPEED);
        if (!scanCommandSafe(pose,motorDirectionForFieldMotion(+1)))
        { abortParking("capture_alignment_clearance"); return true; }
        if (linePoseReady(pose,OBSTACLE_FINAL_PARKING_SCAN_Y_MM) &&
            pose.x_mm>=captureTargetX+80.0f)
        { stop(true);stateStartMs=millis();state=FP_CAPTURE_ALIGN_BRAKE; }
        else if (distanceSince(stateStartDistance)>=OBSTACLE_FINAL_PARKING_CAPTURE_MAX_MM ||
                 millis()-stateStartMs>=OBSTACLE_FINAL_PARKING_CAPTURE_TIMEOUT_MS)
            abortParking("capture_alignment_limit");
        return true;
    }

    if (state == FP_CAPTURE_ALIGN_BRAKE)
    {
        if (!parkingBrakeReady(OBSTACLE_FINAL_PARKING_HOLD_BRAKE_MS)) return true;
        stop(false);
        if (!linePoseReady(get_position_struct(),OBSTACLE_FINAL_PARKING_SCAN_Y_MM))
        { abortParking("capture_alignment_pose"); return true; }
        captureAlignPending=false;
        captureFieldMotionSign=-1;
        servo_disabled=false;set_steering(0);steer(0);
        stateStartMs=millis();state=FP_CAPTURE_SETTLE;
        Serial.println("[FINAL PARK] Registered capture line aligned; return to entry");
        return true;
    }

    if (state == FP_CAPTURE_DRIVE)
    {
        updateOuterWallReference(false);
        pose = get_position_struct();
        driveFieldLine(captureFieldMotionSign,
                       OBSTACLE_FINAL_PARKING_SCAN_Y_MM,
                       OBSTACLE_FINAL_PARKING_CAPTURE_SPEED);
        if (!scanCommandSafe(pose,motorDirectionForFieldMotion(captureFieldMotionSign)))
        { abortParking("capture_motion_clearance"); return true; }
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
            //615: a slow but progressing capture hit the old15s timer.
            //Brake and solve a complete entry from the registered actual
            //pose instead of discarding the measured bay at a timer gate.
            requestEntryReplan("capture_distance");
        return true;
    }

    if (state == FP_CAPTURE_BRAKE)
    {
        if (!parkingEntryBrakeReady())
            return true;
        stop(false);
        pose = get_position_struct();
        const bool captureReady=fabsf(pose.x_mm-captureTargetX)<=
            OBSTACLE_FINAL_PARKING_CAPTURE_X_TOLERANCE_MM &&
            linePoseReady(pose,OBSTACLE_FINAL_PARKING_SCAN_Y_MM);
        // A small residual (590:1.4deg) is a planning input, not a reason to
        // discard a registered bay. Only a complete checked entry may accept it.
        const bool planned=planParkingEntry(pose);
        if (!captureReady && !planned)
        { requestEntryReplan("capture_pose");return true; }
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
        if (!planned && !parkingEntryPreflight(pose))
        { requestEntryReplan("entry_swept_preflight"); return true; }
        Serial.println("[FINAL PARK] Measured-gap entry swept preflight PASS");
        beginEntrySettle();
        return true;
    }

    if (state == FP_SEGMENT_SETTLE)
    {
        if (dc_state != DC_DISABLED)
            stop(false);
        if (finalFootprintContained(3.0f) && entryEndReserveReady() &&
            isfinite(measured_speed) &&
            fabsf(measured_speed)<=SOFT_STOP_SPEED_THRESHOLD_MMS)
        {
            state=FP_VERIFY;
            return true;
        }
        const int steering = activeEntrySegment().steering *
                             routeTurnSign;
        if (!parkingEntryPoseSafe(pose,steering))
        { requestEntryReplan("stationary_steering_clearance");return true; }
        servo_disabled = false;
        set_steering(steering);
        steer(steering);
        // Reuse elapsed settling only for the identical physically written
        // command. A new command still gets the complete actuator wait.
        if (!steering_command_settled(steering,
            OBSTACLE_FINAL_PARKING_STEER_SETTLE_MS))
            return true;
        stateStartDistance = get_distance();
        stateStartMs = millis();
        state = FP_SEGMENT_DRIVE;
        entryStartHeading=pose.heading_deg;
        Serial.print("[FINAL PARK SEGMENT] index=");
        Serial.print(segmentIndex + 1);
        Serial.print(" direction=");
        Serial.print(activeEntrySegment().direction);
        Serial.print(" steering=");
        Serial.print(steering);
        Serial.print(" distance_mm=");
        Serial.println(activeEntrySegment().distanceMm, 1);
        return true;
    }

    if (state == FP_SEGMENT_DRIVE)
    {
        const ParkingSegment &segment = activeEntrySegment();
        if (finalFootprintContained(3.0f) && entryEndReserveReady())
        {
            // Finish at the first valid parallel pose, including mid-segment.
            // Remove curvature before the existing measured-standstill gate;
            // never execute a remaining straight just to return to a target X.
            entryBrakeStartDistance=get_distance();
            entryBrakeDirection=segment.direction;
            servo_disabled=false;set_steering(0);steer(0);
            stop(true);stateStartMs=millis();state=FP_SEGMENT_BRAKE;
            Serial.println("[FINAL PARK] Parallel inside bay; braking without centring move");
            return true;
        }
        const float travelled=distanceSince(stateStartDistance);
        // Use only this entry's measured, settled brake overrun. Retain a
        // small commanded movement for tiny corrections; never skip an arc
        // merely because its length is smaller than the learned overrun.
        const float brakeLead=adaptiveEntryCount && travelled>=0.5f ? entryBrakeTravelMm:0.0f;
        bool reached=travelled>=segment.distanceMm-0.5f-brakeLead;
        if (adaptiveEntryCount && segment.steering!=0)
        {
            const float target=adaptiveEndpoints[segmentIndex].heading_deg;
            const float rate=-segment.direction/Ackermann::getTurnRadius(
                static_cast<float>(segment.steering*routeTurnSign));
            // A sub-degree final correction must actually move. A fixed
            //0.75deg allowance previously skipped the whole small arc and
            // repeatedly planned the same impossible final alignment.
            const float turn=fabsf(wrap180(target-entryStartHeading));
            const float tolerance=fminf(0.75f,fmaxf(0.1f,0.15f*turn));
            const float radius=fabsf(Ackermann::getTurnRadius(
                static_cast<float>(segment.steering*routeTurnSign))*entryRadiusScale);
            reached=wrap180(target-pose.heading_deg)*(rate>0?1.0f:-1.0f)<=
                tolerance+brakeLead/radius*180.0f/PI;
        }
        if (reached)
        {
            entryBrakeStartDistance=get_distance();
            entryBrakeDirection=segment.direction;
            stop(true);stateStartMs=millis();state=FP_SEGMENT_BRAKE;
            return true;
        }
        float remaining=segment.distanceMm-distanceSince(stateStartDistance);
        const float prediction=adaptiveEntryCount && segment.steering ? 15.0f:
            fminf(15.0f,fmaxf(0.0f,remaining));
        const float spread=entryRadiusMeasured ? 0.05f:0.15f;
        for (float radiusScale:{entryRadiusScale*(1.0f-spread),entryRadiusScale,entryRadiusScale*(1.0f+spread)})
        {
            PositionEstimate predicted=pose;
            for (int travel=0; travel<=static_cast<int>(ceilf(prediction)); ++travel)
            {
                if (!parkingEntryPoseSafe(predicted,segment.steering*routeTurnSign))
                { requestEntryReplan("entry_motion_clearance"); return true; }
                float step=fminf(1.0f,prediction-travel);
                if (adaptiveEntryCount && segment.steering)
                {
                    const float radius=Ackermann::getTurnRadius(static_cast<float>(segment.steering*routeTurnSign))*radiusScale;
                    const float rate=-segment.direction/radius;
                    const float angle=wrap180(adaptiveEndpoints[segmentIndex].heading_deg-predicted.heading_deg)*(rate>0 ? 1.0f:-1.0f);
                    step=fminf(step,fmaxf(0.0f,angle)*PI/180.0f*fabsf(radius));
                }
                if (step<=0.0001f) break;
                advanceEntryModel(predicted,segment.steering*routeTurnSign,
                    segment.direction*step,radiusScale);
                if (!parkingEntryPoseSafe(predicted,segment.steering*routeTurnSign))
                { requestEntryReplan("entry_motion_clearance"); return true; }
                if (step<0.999f) break;
            }
        }
        // The main gyro gate can pause an already active segment with
        // stop(false). Only restore servo writes after health and the swept
        // motion checks above have accepted the resumed drive command.
        servo_disabled = false;
        set_speed(segment.direction * (entrySafetyMargin<3.0f ? 40:OBSTACLE_FINAL_PARKING_ENTRY_SPEED));
        set_steering(segment.steering * routeTurnSign);
        const float travelLimit=adaptiveEntryCount && segment.steering ?
            fmaxf(segment.distanceMm+35.0f,segment.distanceMm*1.6f+10.0f):segment.distanceMm+35.0f;
        if (distanceSince(stateStartDistance)>travelLimit ||
            millis()-stateStartMs>=OBSTACLE_FINAL_PARKING_SEGMENT_TIMEOUT_MS)
            requestEntryReplan("segment_progress");
        return true;
    }

    if (state == FP_SEGMENT_BRAKE)
    {
        sampleRearEntryReference(pose);
        if (!parkingEntryBrakeReady())
            return true;
        stop(false);
        updateEntryBrakeTravel();
        const bool radiusChanged=updateEntryRadiusFromMotion();
        const bool rearCorrected=applyRearEntryReference();
        if (finalFootprintContained() && entryEndReserveReady())
        {
            state=FP_VERIFY;
            return true;
        }
        if(rearCorrected){requestEntryReplan("rear_marker_reference");return true;}
        if(radiusChanged && adaptRemainingEntryRecipe(get_position_struct()))
        {beginEntrySettle();return true;}
        const bool withinLegacyGate=adaptiveEntryCount ? true:endpointWithinGate(segmentIndex);
        PositionEstimate expected{};
        if (adaptiveEntryCount) expected=adaptiveEndpoints[segmentIndex];
        else localToField(SEGMENT_LOCAL_X_MM[segmentIndex],SEGMENT_LOCAL_Y_MM[segmentIndex],
            SEGMENT_LOCAL_HEADING_DEG[segmentIndex],expected.x_mm,expected.y_mm,expected.heading_deg);
        pose=get_position_struct();
        const uint8_t count=adaptiveEntryCount ? adaptiveEntryCount:OBSTACLE_FINAL_PARKING_SEGMENT_COUNT;
        if (segmentIndex+1<count && !parkingEntryPoseSafe(pose,
                (adaptiveEntryCount ? adaptiveEntry[segmentIndex+1]:PARKING_SEGMENTS[segmentIndex+1]).steering*routeTurnSign))
        { requestEntryReplan("next_steering_clearance");return true; }
        if (radiusChanged || !withinLegacyGate || hypotf(pose.x_mm-expected.x_mm,pose.y_mm-expected.y_mm)>5.0f ||
            fabsf(wrap180(pose.heading_deg-expected.heading_deg))>1.5f)
        {
            if(refreshRemainingEntry(pose))
            {beginEntrySettle();return true;}
            requestEntryReplan("segment_pose");return true;
        }
        ++segmentIndex;
        if (segmentIndex >= (adaptiveEntryCount ? adaptiveEntryCount:OBSTACLE_FINAL_PARKING_SEGMENT_COUNT))
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
        beginEntrySettle();
        return true;
    }

    if (state == FP_VERIFY)
    {
        // stop(false) disables writes but retains the servo PWM. Do not wait
        // another400ms when the preceding straight already centred the wheels.
        if (servo_disabled)
        {
            if (!parkingEntryPoseSafe(pose,0))
            { requestEntryReplan("final_neutral_clearance");return true; }
            servo_disabled=false;set_steering(0);steer(0);
        }
        if (!steering_command_settled(0,OBSTACLE_FINAL_PARKING_STEER_SETTLE_MS)) return true;
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
        if (!contained && stopped)
        { requestEntryReplan("final_alignment");return true; }
        if (contained && stopped && !entryEndReserveReady())
        { requestEntryReplan("front_end_reserve");return true; }
        if (!contained || !stopped)
        {
            abortParking(
                contained ? "final_speed_gate" : "final_containment_gate");
            return true;
        }
        stop(true);
        completed = true;
        state = FP_HOLD;
        Serial.println("[FINAL PARK] Complete; motor hold active");
        run_telemetry_finish("completed","final_parking_stop");
        robot_logger.write_to_usb();
        return true;
    }

    if (state==FP_REPLAN_BRAKE)
    {
        sampleRearEntryReference(pose);
        if (!parkingEntryBrakeReady()) return true;
        stop(false);
        if (entryReplanHadMotion)
        {
            updateEntryBrakeTravel();
            updateEntryRadiusFromMotion();
        }
        entryReplanHadMotion=false;
        applyRearEntryReference();
        // Containment alone can pass at5.5mm while the normal inflated
        // neutral-wheel check needs more room. Returning straight to VERIFY
        // in that case loops forever without ever searching a centring move.
        if (finalFootprintContained() && parkingEntryPoseSafe(get_position_struct(),0) && entryEndReserveReady())
        { state=FP_VERIFY;return true; }
        const PositionEstimate local=entryLocalPose(get_position_struct());
        const float error=hypotf(local.x_mm-OBSTACLE_FINAL_PARKING_TARGET_LOCAL_X_MM,
            local.y_mm-OBSTACLE_FINAL_PARKING_TARGET_LOCAL_Y_MM)+
            fabsf(Ackermann::getTurnRadius(50.0f))*entryRadiusScale*
                fabsf(local.heading_deg)*PI/180.0f;
        // Count consecutive failures to make net progress, not all corrections
        // since entry. A robot that is steadily parking must not hit an
        // arbitrary eight-correction lifetime limit.
        if(error+3.0f<entryBestReplanError)
        {entryBestReplanError=error;entryStalledReplans=0;}
        else if(entryStalledReplans<255)++entryStalledReplans;
        if(entryReplans<255)++entryReplans;
        if (entryStalledReplans>8 || !planParkingEntry(get_position_struct()))
        {
            abortParking(entryStalledReplans>8 ? "entry_no_progress":"entry_no_collision_free_plan");
            return true;
        }
        Serial.print("[FINAL PARK REPLAN] accepted attempt=");Serial.print(entryReplans);
        Serial.print(" reason=");Serial.println(entryReplanReason ? entryReplanReason:"entry");
        beginEntrySettle();
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

bool final_parking_active()
{
    return state != FP_IDLE || initialBrakeStarted;
}

void final_parking_sensor_hold()
{
    entryBrakeEvidenceValid=false;
    if (!final_parking_active()) return;
    if (!sensorHoldPending) {
        sensorHoldPending = true;
        sensorHoldStartMs = millis();
    }
    if (sensorHoldTraceCount>=6) return;
    ++sensorHoldTraceCount;
    Serial.println("[FINAL PARK SENSOR HOLD] main gyro unhealthy; drive stopped");
    traceParking(true);
}

void final_parking_sensor_resume()
{
    if (!sensorHoldPending) return;
    // A sensor pause is not time spent attempting the current parking motion.
    const uint32_t paused = uint32_t(millis() - sensorHoldStartMs);
    stateStartMs += paused;
    if (scanAlignStartMs) scanAlignStartMs += paused;
    sensorHoldPending = false;
}

const char *final_parking_trace_phase() {
    static char phase[32];
    snprintf(phase,sizeof(phase),"final_park_%u",static_cast<unsigned>(state));
    return phase;
}
