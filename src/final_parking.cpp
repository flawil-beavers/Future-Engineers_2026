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
#include "drive_profile.h"

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
    FP_VERIFY = 13,
    FP_HOLD = 15,
    FP_ABORT,
    FP_SCAN_ALIGN_DRIVE,
    FP_SCAN_ALIGN_BRAKE,
    FP_CAPTURE_ALIGN_DRIVE,
    FP_CAPTURE_ALIGN_BRAKE,
    FP_FIXED_APPROACH = 22,
    FP_FIXED_SETTLE,
    FP_FIXED_DRIVE,
    FP_FIXED_BRAKE,
    FP_APPROACH_WALL_CLEAR,
    FP_CORNER_TO_WALL,
    FP_LEFT_TO_WALL
};

FinalParkingState state = FP_IDLE;
int8_t routeTurnSign = 1;
TofSensor outerSensor = TOF_RIGHT;
float stateStartDistance = 0.0f;
float runStartDistance = 0.0f;
uint32_t stateStartMs = 0;
uint8_t segmentIndex = 0;
uint32_t tofSequence = 0;
uint32_t scanSampleMs = 0;
bool scanFastCadence = false;
uint8_t scanPhase = 0;
uint8_t classFrames = 0;
bool scanArmed = false;
bool scanLineEstablished = false;
bool scanWallRegistered = false;
int8_t scanFieldSign = -1;
float scanWallOffsetMm = 0.0f;
bool cornerUsesWallArc = false;
bool cornerFinishPending = false;
uint32_t cornerCheckMs = 0;
int8_t scanAlignSign = 1;
uint8_t scanAlignTurns = 0;
uint8_t scanGeometryRetries = 0;
uint32_t scanAlignStartMs = 0;

PositionEstimate lastLeadingMarkerPose{};
float lastLeadingMarkerRange = 0.0f;
PositionEstimate firstTrailingMarkerPose{};
float firstTrailingMarkerRange = 0.0f;
bool trailingCandidateStored = false;
PositionEstimate latestWallPose{};
float latestWallRange = 0.0f;
PositionEstimate trustedGapWallPose{};
float trustedGapWallRange = 0.0f;
float measuredFixedInsideX = 0.0f;
float measuredMovingInsideX = 0.0f;
float measuredGapMm = 0.0f;
float captureTargetX = 0.0f;
float captureLineY = 0.0f;
bool captureTipSteeringLimited = false;
int8_t captureFieldMotionSign = 1;
bool captureAlignPending = false;
bool completed = false;
bool aborted = false;
bool practiceMode = false;
bool fixedCwPractice = false;
constexpr float FIXED_ENTRY_RAIL_CLEARANCE_MM = 30.0f;
constexpr float FIXED_ENTRY_CAPTURE_HEADING_TOLERANCE_DEG = 10.0f;
constexpr int FIXED_ENTRY_APPROACH_SPEED = OBSTACLE_PARKING_EXIT_EDGE_LOCALIZATION_APPROACH_SPEED*11/10;
constexpr int FIXED_ENTRY_SCAN_SPEED = OBSTACLE_FINAL_PARKING_SCAN_SPEED*11/10;
constexpr int FIXED_ENTRY_CAPTURE_SPEED = OBSTACLE_FINAL_PARKING_CAPTURE_SPEED*11/10;
constexpr int FREE_PARK_APPROACH_SPEED = 205;
constexpr int FREE_SCAN_APPROACH_SPEED = 145;
constexpr int FREE_NORMAL_SCAN_APPROACH_SPEED = 155;
constexpr int FRESH_MARKER_SCAN_SPEED = 84;
constexpr int FREE_CAPTURE_SPEED = 145;
constexpr char PARKING_APPROACH_REVISION[] = "[FINAL PARK] revision=20261122_two_sided_wall_line";
uint32_t fixedProgressMs = 0;
float fixedProgress = 0.0f;
float fixedStepStartHeading = 0.0f;
bool fixedSlipReserveLogged = false;
bool initialBrakeStarted = false;
float approachOuterShiftX = OBSTACLE_FINAL_PARKING_OUTER_SHIFT_X_MM;
bool approachDirectLane = false;
bool approachDirectScan = false;
bool approachSmoothCcw = false;
constexpr float DIRECT_SCAN_SHIFTS[] = {-500.0f, -150.0f, 0.0f, 175.0f, 350.0f};
constexpr unsigned DIRECT_SCAN_SHIFT_COUNT = sizeof(DIRECT_SCAN_SHIFTS)/sizeof(DIRECT_SCAN_SHIFTS[0]);
const float APPROACH_SHIFTS[]={-100.0f,100.0f,200.0f,350.0f,400.0f,450.0f,500.0f,550.0f,600.0f,650.0f};
constexpr unsigned APPROACH_SHIFT_COUNT = sizeof(APPROACH_SHIFTS)/sizeof(APPROACH_SHIFTS[0]);
struct ApproachPreparation {
    PositionEstimate arrival{},sample{};
    uint8_t candidate=0,variant=0;
    uint16_t steps=0;
    bool started=false,ready=false,failed=false,outer=false;
    int8_t direction=1;
} approachPreparation;

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
uint32_t entryPlannerLastPollMs = 0;
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

float parkingScanLineY()
{
    return OBSTACLE_PARKING_OPEN_END_FIELD_Y_MM +
        0.5f*OBSTACLE_FINAL_PARKING_STRAIGHT_WIDTH_MM + FIXED_ENTRY_RAIL_CLEARANCE_MM;
}

float activeScanLineY()
{
    return parkingScanLineY()-scanWallOffsetMm;
}

float parkingScanStartX()
{
    if (practiceMode) return OBSTACLE_FINAL_PARKING_SCAN_START_X_MM;
    // The scan travels towards -X. Measure from the leading edge of the
    // parallel robot to the outer face of the fixed pink rail.
    constexpr float railStartGapMm = 300.0f;
    const float reach = routeTurnSign > 0 ? OBSTACLE_FINAL_PARKING_REAR_OVERHANG_MM
                                         : OBSTACLE_FINAL_PARKING_FRONT_OVERHANG_MM;
    return OBSTACLE_PARKING_FIXED_DOTTED_LINE_X_MM + railStartGapMm + reach;
}

float captureRearAxleLocalX()
{
    // At the staging line the rear edge is level with the inner face of the
    // entry rail. The wheels and body are still outside the rail tips.
    return measuredGapMm + OBSTACLE_FINAL_PARKING_REAR_OVERHANG_MM;
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

bool registeredRailFanClear(const PositionEstimate &pose, bool bothRails)
{
    const auto tip=markerEdgeFootprint(outerSensor,pose);
    const auto wall=markerEdgeFootprint(outerSensor,pose,OBSTACLE_SOUTH_OUTER_WALL_Y_MM);
    if (!isfinite(tip.minimumX) || !isfinite(wall.minimumX)) return false;
    const float low=fminf(tip.minimumX,wall.minimumX);
    const float high=fmaxf(tip.maximumX,wall.maximumX);
    constexpr float reserve=12.0f;
    constexpr float minimumGap =
        OBSTACLE_FINAL_PARKING_GAP_MM-OBSTACLE_FINAL_PARKING_GAP_TOLERANCE_MM;
    const float movingFace=bothRails || scanFieldSign>0 ? measuredMovingInsideX
        : measuredFixedInsideX-minimumGap;
    const float fixedFace=bothRails || scanFieldSign<0 ? measuredFixedInsideX
        : measuredMovingInsideX+minimumGap;
    if (low>movingFace+reserve && high<fixedFace-reserve) return true;
    return bothRails && (high<movingFace-OBSTACLE_PARKING_LIMIT_THICKNESS_MM-reserve ||
        low>measuredFixedInsideX+OBSTACLE_PARKING_LIMIT_THICKNESS_MM+reserve);
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
bool scanAlignmentPreflight(PositionEstimate pose, int8_t *firstDirection = nullptr);
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
    const bool acrossClearGap=!initial && state==FP_SCAN_DRIVE &&
        scanPhase==3 && classFrames==0 && registeredRailFanClear(pose,false);
    if (!initial && !acrossClearGap &&
        (scanPhase >= 2 || (scanPhase==1 && classFrames>0) || !earlyWallReferenceClearOfMarkers(pose)))
        return false;
    const bool clearOfRails = initial && earlyWallReferenceClearOfMarkers(pose);
    const bool ccwClearApproach = initial && routeTurnSign > 0 && !practiceMode &&
        (state == FP_APPROACH_BRAKE || state == FP_APPROACH_WALL_CLEAR);
    const bool rollingReference = initial && !practiceMode &&
        (state==FP_APPROACH_OUTER || state==FP_CORNER_TO_WALL || state==FP_LEFT_TO_WALL);
    if (initial && (state == FP_APPROACH_LANE || ccwClearApproach || rollingReference) && !clearOfRails)
    {
        initialWallFrames=0;
        return false;
    }
    const float raw = parkingOuterRange(snapshot);
    const float expected = expectedOuterWallRange(outerSensor, pose);
    const float residual = raw - expected;
    const float maxResidual = rollingReference ? (routeTurnSign<0 ? 45.0f : 75.0f) :
        (initial ? 150.0f : (acrossClearGap ? 18.0f : 10.0f));
    // Outside the full rail fan, a short reading can be the actual outer
    // wall. Keep the normal wall/rail classification threshold during scan.
    const float minimumWallRange = ccwClearApproach || (rollingReference && routeTurnSign>0) ? 145.0f
        : OBSTACLE_FINAL_PARKING_WALL_MIN_MM;
    if (!isfinite(raw) || raw < minimumWallRange ||
        raw > (initial ? 700.0f : OBSTACLE_FINAL_PARKING_WALL_MAX_MM) || expected <= 0.0f ||
        fabsf(residual) > maxResidual ||
        fabsf(wrap180(pose.heading_deg-baseHeadingDeg())) >
            (rollingReference && routeTurnSign<0 ? 8.0f : 10.0f))
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
    if (initial && (fabsf(residual)>80.0f ||
        (ccwClearApproach && fabsf(correction)>OBSTACLE_FINAL_PARKING_LINE_Y_TOLERANCE_MM)))
    {
        // 567/577: stable wall up to387mm versus predicted258mm after three laps.
        // Only the cleared outer corridor may register this larger drift,
        // and only if its complete bounded line alignment has a safe sweep.
        PositionEstimate corrected=pose;
        corrected.y_mm+=correction;
        if (!scanAlignmentPreflight(corrected)) return false;
    }
    position_apply_xy_correction(0.0f,correction);
    if (acrossClearGap)
    {
        // Remember an actual wall return and the pose after this correction
        // so registration never applies the same lateral correction twice.
        trustedGapWallPose=pose;
        trustedGapWallPose.y_mm+=correction;
        trustedGapWallRange=raw;
    }
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

bool checkMotionHealth(const PositionEstimate &pose, bool onApproach = true)
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
    if (onApproach && !practiceMode && !connectorClearanceSafe(pose))
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

int parkingParallelSteering(const PositionEstimate &pose, int motorDirection)
{
    const float error=wrap180(baseHeadingDeg()-pose.heading_deg);
    return static_cast<int>(clampFloat(-motorDirection*3.0f*error,-10.0f,10.0f));
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
    else if (scanWallRegistered || approachDirectScan)
        steering=fieldLineSteering(pose,+1,parkingScanLineY());
    else if (approachSmoothCcw)
    {
        // Run alongside the rails, then merge progressively onto the scan
        // line. Preview Y on the ramp instead of requesting a full shift at
        // one waypoint and countersteering as soon as the nose crosses it.
        constexpr float preview = 115.0f;
        constexpr float shiftLength = 285.0f;
        const float t = clampFloat((pose.x_mm-approachOuterShiftX)/shiftLength,0.0f,1.0f);
        const float blend = t*t*(3.0f-2.0f*t);
        const float targetY = OBSTACLE_FINAL_PARKING_APPROACH_LANE_Y_MM +
            blend*(parkingScanLineY()-OBSTACLE_FINAL_PARKING_APPROACH_LANE_Y_MM);
        const float h = pose.heading_deg*PI/180.0f, dy = targetY-pose.y_mm;
        const float curvature = 2.0f*(-preview*sinf(h)+dy*cosf(h))/(preview*preview+dy*dy);
        steering = static_cast<int>(fabsf(curvature)<1.0e-6f ? 0.0f :
            clampFloat(Ackermann::getServoAngleForRadius(-1.0f/curvature),-22.0f,22.0f));
    }
    else
    {
        // Keep the wide-angle corner departure controller. The gentler line
        // pursuit takes over once this outward bend has reached the scan line.
        const float angle=clampFloat(atan2f(
            parkingScanLineY()-pose.y_mm,
            OBSTACLE_FINAL_PARKING_LINE_LOOKAHEAD_MM)*180.0f/PI,-60.0f,60.0f);
        const float desired=baseHeadingDeg()+angle;
        steering=static_cast<int>(clampFloat(
            -motorDirectionForFieldMotion(+1)*wrap180(desired-pose.heading_deg)*
                OBSTACLE_FINAL_PARKING_LINE_HEADING_KP,-42.0f,42.0f));
    }
    return true;
}

// Faster while the acquisition still has useful travel; approach a reversal
// at60mm/s. Include the longitudinal gate so an already parallel robot does
// not crawl all the way to the scan start merely because its Y is correct.
int parkingLineAcquisitionSpeed(const PositionEstimate &pose, int maximum,
    float minimumX, float lateralWindow, int minimum = FIXED_ENTRY_SCAN_SPEED)
{
    const float lateral=fabsf(pose.y_mm-parkingScanLineY())/lateralWindow;
    const float longitudinal=fmaxf(0.0f,minimumX-pose.x_mm)/100.0f;
    const float blend=clampFloat(fmaxf(lateral,longitudinal),0.0f,1.0f);
    return static_cast<int>(minimum+blend*(maximum-minimum));
}

int parkingApproachSpeed(const PositionEstimate &pose, int maximum)
{
    if (pose.x_mm < approachOuterShiftX)
        return maximum;
    // Keep making progress after the nominal scan X if the wall reference
    // or heading is still settling. Leave the original slow endpoint.
    const int minimum = static_cast<int>(FIXED_ENTRY_SCAN_SPEED +
        (105-FIXED_ENTRY_SCAN_SPEED)*clampFloat((1030.0f-pose.x_mm)/90.0f,0.0f,1.0f));
    return parkingLineAcquisitionSpeed(pose,maximum,
        parkingScanStartX(),100.0f,minimum);
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

// The same checked sample is used by stopped and incremental planning.
int advanceApproachPreflight(PositionEstimate &pose,bool &outer)
{
    if(!parkingApproachPoseSafe(pose))return -1;
    if(approachDirectLane || approachDirectScan)for(float dy:{-40.0f,40.0f})
    {
        PositionEstimate uncertain=pose;uncertain.y_mm+=dy;
        if(!parkingApproachPoseSafe(uncertain))return -1;
    }
    outer=outer || pose.x_mm>=approachOuterShiftX;
    if(outer && pose.x_mm>=parkingScanStartX() &&
       linePoseReady(pose,parkingScanLineY()))return 1;
    if(pose.x_mm>1100.0f)return -1;
    int steering=0;if(!parkingApproachSteering(pose,outer,steering))return -1;
    advanceParkingPose(pose,steering,2.0f*motorDirectionForFieldMotion(+1));
    return 0;
}

bool parkingApproachPreflight(PositionEstimate pose)
{
    bool outer=false;
    for(int step=0;step<1200;++step)
    {
        serviceEntryPlanner();
        const int result=advanceApproachPreflight(pose,outer);
        if(result)return result>0;
    }
    return false;
}

unsigned parkingApproachCandidateCount()
{
    return (routeTurnSign>0 && !practiceMode ? 4 : 2)*APPROACH_SHIFT_COUNT +
        (practiceMode ? 0 : DIRECT_SCAN_SHIFT_COUNT);
}

bool selectParkingApproachCandidate(unsigned candidate)
{
    approachDirectScan = !practiceMode && candidate < DIRECT_SCAN_SHIFT_COUNT;
    if (approachDirectScan)
    {
        approachDirectLane = false;
        approachSmoothCcw = false;
        approachOuterShiftX = DIRECT_SCAN_SHIFTS[candidate];
        return true;
    }
    if (!practiceMode) candidate -= DIRECT_SCAN_SHIFT_COUNT;
    approachSmoothCcw = routeTurnSign > 0 && !practiceMode &&
        candidate<2*APPROACH_SHIFT_COUNT;
    approachDirectLane = (candidate/APPROACH_SHIFT_COUNT)%2==0;
    approachOuterShiftX = APPROACH_SHIFTS[candidate%APPROACH_SHIFT_COUNT];
    return approachDirectLane || approachOuterShiftX>=500.0f;
}

bool chooseParkingApproach(const PositionEstimate &pose)
{
    for(unsigned candidate=0;candidate<parkingApproachCandidateCount();++candidate)
    {
        if(selectParkingApproachCandidate(candidate) && parkingApproachPreflight(pose))return true;
    }
    return false;
}

bool preparedApproachFits(const PositionEstimate &pose)
{
    return approachPreparation.ready &&
        fabsf(pose.x_mm-approachPreparation.arrival.x_mm)<=40.0f &&
        fabsf(pose.y_mm-approachPreparation.arrival.y_mm)<=40.0f &&
        fabsf(wrap180(pose.heading_deg-approachPreparation.arrival.heading_deg))<=5.0f;
}

void resetPreparationVariant()
{
    auto &p=approachPreparation;
    p.sample=p.arrival;p.outer=false;p.steps=0;
    if(p.variant)
    {
        int cell=p.variant-1;if(cell>=13)++cell; // centre was checked first
        p.sample.x_mm+=(cell%3-1)*40.0f;
        p.sample.y_mm+=((cell/3)%3-1)*40.0f;
        p.sample.heading_deg+=(cell/9-1)*5.0f;
    }
}

bool parkingApproachCommandSafe(PositionEstimate pose, int steering, int direction,
                                int samples = 15)
{
    for (int i=0; i<=samples; ++i)
    {
        if (!parkingApproachPoseSafe(pose)) return false;
        advanceParkingPose(pose,steering,2.0f*direction);
    }
    return true;
}

int wallMergeSteering(const PositionEstimate &pose,int fieldSign)
{
    const float dy=activeScanLineY()-pose.y_mm;
    const float dx=fieldSign*(65.0f+fminf(fabsf(dy)*0.15f,35.0f));
    const float heading=pose.heading_deg*PI/180.0f;
    const float curvature=2.0f*(-dx*sinf(heading)+dy*cosf(heading))/(dx*dx+dy*dy);
    return fabsf(curvature)<1.0e-6f ? 0 : static_cast<int>(clampFloat(
        Ackermann::getServoAngleForRadius(-1.0f/curvature),-42.0f,42.0f));
}

int cornerWallSteering(const PositionEstimate &pose, bool wallArc)
{
    const float lateral=pose.y_mm-parkingScanLineY();
    if (!wallArc) return wallMergeSteering(pose,routeTurnSign);
    if (lateral<40.0f) return fieldLineSteering(pose,routeTurnSign,parkingScanLineY());
    // Hold the southbound tangent until there is just enough Y for a
    // continuous turn onto the wall line. The same arc mirrors for CCW.
    constexpr float turnRadius=220.0f;
    const float angle=acosf(clampFloat(1.0f-(lateral-16.0f)/turnRadius,0.0f,1.0f))*180.0f/PI;
    const float blend=clampFloat((lateral-turnRadius-20.0f)/130.0f,0.0f,1.0f);
    const float southboundCorrection=routeTurnSign<0 ? blend*clampFloat(
        (parkingScanStartX()+200.0f-pose.x_mm)/6.0f,-15.0f,15.0f) : 0.0f;
    const float target=baseHeadingDeg()-routeTurnSign*angle+southboundCorrection;
    return static_cast<int>(clampFloat(-2.8f*wrap180(target-pose.heading_deg),-42.0f,42.0f));
}

float scanLeadRemaining(const PositionEstimate &pose)
{
    // Distance from the first rail to the side sensor, before its cone and
    // two fresh baseline frames enter the marker zone.
    const float sensorX=pose.x_mm+routeTurnSign*(routeTurnSign>0 ?
        OBSTACLE_TOF_RIGHT_LOCAL_X_MM : OBSTACLE_TOF_LEFT_LOCAL_X_MM);
    return scanFieldSign>0 ? movingInsideFaceX()-OBSTACLE_PARKING_LIMIT_THICKNESS_MM-sensorX
                          : sensorX-OBSTACLE_PARKING_FIXED_DOTTED_LINE_X_MM;
}

bool cornerScanReady(const PositionEstimate &pose)
{
    return scanLeadRemaining(pose)>=115.0f && linePoseReady(pose,parkingScanLineY());
}

bool parkingEntryPoseSafe(const PositionEstimate &pose,int steering,float safetyMargin);

bool checkedWallScan(const PositionEstimate &pose,int sign,uint32_t cornerStartUs=0)
{
    // Include the entire scan and the short overshoot past the second ToF
    // edge. The normal corridor test includes learned block movement.
    const float end=sign>0 ? 580.0f : 140.0f;
    for (float x=pose.x_mm;sign*(end-x)>0.0f;x+=sign*10.0f)
    {
        serviceEntryPlanner();
        if (cornerStartUs && uint32_t(micros()-cornerStartUs)>16000U) return false;
        for (float dy:{-4.0f,4.0f})
        {
            const PositionEstimate sample{x,activeScanLineY()+dy,baseHeadingDeg()};
            if (!parkingEntryPoseSafe(sample,0,3.0f)) return false;
        }
    }
    return true;
}

bool cornerWallPreflight(PositionEstimate pose, bool wallArc)
{
    const uint32_t startUs=micros();
    for (int mm=0;mm<=1150;mm+=5)
    {
        serviceEntryPlanner();
        if (uint32_t(micros()-startUs)>16000U) return false;
        PositionEstimate swept=pose;
        for (float dy:{-5.0f,5.0f})
        {
            swept.y_mm=pose.y_mm+dy;
            if (!parkingApproachPoseSafe(swept)) return false;
        }
        if (cornerScanReady(pose)) return checkedWallScan(pose,scanFieldSign,startUs);
        if (scanLeadRemaining(pose)<115.0f) return false;
        advanceParkingPose(pose,cornerWallSteering(pose,wallArc),5.0f);
    }
    return false;
}

bool leftWallReady(const PositionEstimate &pose)
{
    const float leading=routeTurnSign>0 ? OBSTACLE_FINAL_PARKING_FRONT_OVERHANG_MM
                                       : OBSTACLE_FINAL_PARKING_REAR_OVERHANG_MM;
    const float target=movingInsideFaceX()-OBSTACLE_PARKING_LIMIT_THICKNESS_MM-300.0f-leading;
    return pose.x_mm<=target && linePoseReady(pose,activeScanLineY()) &&
        (scanWallOffsetMm==0.0f || fabsf(wrap180(baseHeadingDeg()-pose.heading_deg))<=2.0f);
}

bool leftWallPreflight(PositionEstimate pose)
{
    for (int mm=0;mm<=950;mm+=5)
    {
        serviceEntryPlanner();
        const int steering=wallMergeSteering(pose,-1);
        if (!parkingEntryPoseSafe(pose,steering,5.0f)) return false;
        if (leftWallReady(pose)) return checkedWallScan(pose,1);
        advanceParkingPose(pose,steering,
            5.0f*motorDirectionForFieldMotion(-1));
    }
    return false;
}

bool cwRedBeforeBay()
{
    if (practiceMode || routeTurnSign>0) return false;
    for (uint8_t id=0;id<4;++id)
    {
        ObstacleSeatInfo seat;
        if (obstacle_path_get_seat(id,seat) && seat.confirmed && seat.red) return true;
    }
    return false;
}

bool driveCheckedParkingApproach(const PositionEstimate &pose, bool outer)
{
    int steering = 0;
    if (!parkingApproachSteering(pose,outer,steering))
        return false;
    const int direction = motorDirectionForFieldMotion(+1);
    int speed = parkingApproachSpeed(pose,OBSTACLE_FINAL_PARKING_APPROACH_SPEED);
    // Keep the longer swept preview at higher speed. For the unregistered
    // open approach the same corridor preflight covers lateral uncertainty.
    if (!practiceMode && (initialWallFrames>=2 || !outer) && abs(steering)<=20 &&
        parkingApproachCommandSafe(pose,steering,direction,22))
        speed = parkingApproachSpeed(pose,FREE_PARK_APPROACH_SPEED);
    else if ((!practiceMode || !outer) && !parkingApproachCommandSafe(pose,steering,direction))
        return false;
    if (outer)
        speed=static_cast<int>(fminf(speed,130.0f+(FREE_PARK_APPROACH_SPEED-130.0f)*
            clampFloat((1030.0f-pose.x_mm)/80.0f,0.0f,1.0f)));
    servo_disabled = false;
    set_steering(steering);
    set_speed(direction*speed);
    return true;
}

// The measured gap replaces the nominal gap after the marker scan. Test the
// actual steered wheels, not an enclosing rectangle, at the marker tips.
bool parkingEntryPoseSafe(const PositionEstimate &pose, int steering, float safetyMargin=3.0f)
{
    if (!connectorClearanceSafe(pose)) return false;
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

bool scanCommandSafe(PositionEstimate pose, int direction, int steering = set_degree,
                     int previewMm = 15)
{
    for (int mm=0; mm<=previewMm; ++mm)
    {
        if (!parkingEntryPoseSafe(pose,steering)) return false;
        advanceParkingPose(pose,steering,static_cast<float>(direction));
    }
    return true;
}

int parkingRollingPreviewMm()
{
    const float speed=fmaxf(fabsf(measured_speed),fabsf(current_speed));
    const float bounded=isfinite(speed) ? fminf(speed,FREE_PARK_APPROACH_SPEED)
                                       : FREE_PARK_APPROACH_SPEED;
    return static_cast<int>(ceilf(bounded*17.0f/FIXED_ENTRY_SCAN_SPEED));
}

bool captureSteering(const PositionEstimate &pose, int8_t fieldSign, int &steering,
                     bool preview=true)
{
    // Leave the extra clearance alongside the obstacle until the rear has
    // cleared it. The last 180 mm blend back towards the normal entry line.
    const float keepOffset=clampFloat((pose.x_mm-captureTargetX-25.0f)/180.0f,0.0f,1.0f);
    const float line=captureLineY-scanWallOffsetMm*keepOffset;
    const int requested=fieldLineSteering(pose,fieldSign,line);
    const int direction=motorDirectionForFieldMotion(fieldSign);
    const int previewMm=preview ? static_cast<int>(fmaxf(17,parkingRollingPreviewMm())) : 15;
    if(!captureTipSteeringLimited)
    {
        steering=requested;
        return preview ? scanCommandSafe(pose,direction,steering,previewMm)
                       : parkingEntryPoseSafe(pose,steering);
    }
    // An angled wheel can sweep across a tip even when the straight body
    // fits. Ease the turn in as far as the checked preview permits.
    for (int candidate : {requested,requested/2,requested/4,0})
    {
        if(scanCommandSafe(pose,direction,candidate,previewMm)){steering=candidate;return true;}
    }
    return false;
}

bool driveCaptureLine(const PositionEstimate &pose, int8_t fieldSign)
{
    int steering=0;
    if(!captureSteering(pose,fieldSign,steering))return false;
    const int direction=motorDirectionForFieldMotion(fieldSign);
    const float goal=captureTargetX+(captureAlignPending && fieldSign>0 ? 80.0f : 0.0f);
    const float remaining=fieldSign*(goal-pose.x_mm);
    int speed=FIXED_ENTRY_CAPTURE_SPEED;
    if (remaining<45.0f)
        speed=static_cast<int>(80.0f+(FIXED_ENTRY_CAPTURE_SPEED-80.0f)*
            clampFloat(remaining/45.0f,0.0f,1.0f));
    else if (remaining>75.0f && abs(steering)<=15 &&
             scanCommandSafe(pose,direction,steering,36))
        speed=FREE_CAPTURE_SPEED;
    servo_disabled=false;
    set_steering(steering);
    set_speed(direction*speed);
    return true;
}

bool fixedEntryCaptureReady(const PositionEstimate &pose);

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
        if (!returning && linePoseReady(pose,captureLineY) &&
            pose.x_mm>=captureTargetX+80.0f)
        { returning=true;returnStart=mm; }
        if ((!returning && mm>limit) || (returning && mm-returnStart>limit))
            return false;
        serviceEntryPlanner();
        const int8_t sign=returning ? -1 : 1;
        int steering=0;
        if(!captureSteering(pose,sign,steering,false))return false;
        if (returning && pose.x_mm<=captureTargetX+1.5f)
            return fixedEntryCaptureReady(pose);
        advanceParkingPose(pose,steering,static_cast<float>(motorDirectionForFieldMotion(sign)));
    }
    return false;
}

bool fixedEntryCaptureReady(const PositionEstimate &pose)
{
    return fabsf(pose.x_mm-captureTargetX)<=12.0f &&
        fabsf(pose.y_mm-captureLineY)<=10.0f &&
        fabsf(wrap180(pose.heading_deg-baseHeadingDeg()))<=FIXED_ENTRY_CAPTURE_HEADING_TOLERANCE_DEG;
}

bool captureDirectPreflight(PositionEstimate pose)
{
    const int sign = captureFieldMotionSign;
    for (int mm=0; mm<=static_cast<int>(OBSTACLE_FINAL_PARKING_CAPTURE_MAX_MM); ++mm)
    {
        serviceEntryPlanner();
        const bool reached=sign>0 ? pose.x_mm>=captureTargetX-1.5f
                                  : pose.x_mm<=captureTargetX+1.5f;
        if (reached) return fixedEntryCaptureReady(pose);
        int steering=0;
        if (!captureSteering(pose,sign,steering,false)) return false;
        advanceParkingPose(pose,steering,static_cast<float>(motorDirectionForFieldMotion(sign)));
    }
    return false;
}

bool chooseRegisteredCapture(const PositionEstimate &pose)
{
    captureLineY=parkingScanLineY();
    if (fixedEntryCaptureReady(pose))
    {
        captureAlignPending=false;
        return true;
    }
    const int requested=fieldLineSteering(pose,captureFieldMotionSign,captureLineY);
    captureTipSteeringLimited=routeTurnSign>0 && captureFieldMotionSign>0 &&
        pose.y_mm<captureLineY-10.0f &&
        measuredGapMm>=OBSTACLE_FINAL_PARKING_GAP_MM+8.0f &&
        !parkingEntryPoseSafe(pose,requested);
    captureAlignPending=!linePoseReady(pose,captureLineY);
    if (!captureAlignPending) return true;
    if (captureDirectPreflight(pose))
    {
        captureAlignPending=false;
        Serial.println("[FINAL PARK] Registered capture: direct alignment swept preflight PASS");
        return true;
    }
    return captureAlignmentPreflight(pose);
}

int scanAlignmentTravel(PositionEstimate pose, int sign)
{
    int turns=0;
    const int outward=-scanFieldSign;
    const float nearX=scanFieldSign<0 ? 800.0f : 260.0f;
    const float farX=scanFieldSign<0 ? 1080.0f : 660.0f;
    for (int mm=0; mm<=1440; mm+=2)
    {
        serviceEntryPlanner();
        const float sideX=outward*pose.x_mm;
        if (sideX>=nearX && linePoseReady(pose,activeScanLineY()))
            return mm;
        if ((sign==outward && sideX>=farX) || (sign!=outward && sideX<=nearX))
        { sign=-sign;if (++turns>6) return -1; }
        const int steering=fieldLineSteering(pose,sign,activeScanLineY());
        if (!parkingEntryPoseSafe(pose,steering)) return -1;
        advanceParkingPose(pose,steering,2.0f*motorDirectionForFieldMotion(sign));
    }
    return -1;
}

bool scanAlignmentPreflight(PositionEstimate pose, int8_t *firstDirection)
{
    const int8_t outward=-scanFieldSign;
    const float midpoint=scanFieldSign<0 ? 1000.0f : 500.0f;
    const int8_t preferred=outward*pose.x_mm<midpoint ? outward : -outward;
    const int first=scanAlignmentTravel(pose,preferred);
    if (first>=0 && (!firstDirection || first<=60))
    {
        if (firstDirection) *firstDirection=preferred;
        return true;
    }
    const int other=scanAlignmentTravel(pose,-preferred);
    if (first<0 && other<0) return false;
    if (firstDirection)
        *firstDirection=first<0 || (other>=0 && other+20<first) ? -preferred : preferred;
    return true;
}

void resetScanClassFrames()
{
    classFrames = 0;
    trailingCandidateStored = false;
}

void registerScannedEdge(const PositionEstimate &pose, float range, bool leading)
{
    const auto footprint=markerEdgeFootprint(outerSensor,pose);
    const bool fixed=(scanFieldSign<0)==leading;
    const float edge=fixed ? footprint.maximumX : footprint.minimumX;
    if (fixed) measuredFixedInsideX=edge;
    else measuredMovingInsideX=edge;
    Serial.print(fixed ? "[FINAL PARK EDGE] fixed_x/raw_mm=" :
                         "[FINAL PARK EDGE] moving_x/raw_mm=");
    Serial.print(edge,1);Serial.print("/");Serial.println(range,1);
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
            lastLeadingMarkerPose = pose;
            lastLeadingMarkerRange = raw;
            scanPhase = 2;
            classFrames = 0;
        }
        return;
    }

    if (scanPhase == 2)
    {
        if (marker)
        {
            lastLeadingMarkerPose = pose;
            lastLeadingMarkerRange = raw;
            classFrames = 0;
        }
        else if (wall)
        {
            latestWallPose = pose;
            latestWallRange = raw;
            ++classFrames;
            if (classFrames >= OBSTACLE_FINAL_PARKING_CLASS_CONFIRM_FRAMES)
            {
                registerScannedEdge(lastLeadingMarkerPose,lastLeadingMarkerRange,true);
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
            if (!trailingCandidateStored)
            {
                firstTrailingMarkerPose = pose;
                firstTrailingMarkerRange = raw;
                trailingCandidateStored = true;
            }
            ++classFrames;
            if (classFrames >= OBSTACLE_FINAL_PARKING_CLASS_CONFIRM_FRAMES)
            {
                registerScannedEdge(firstTrailingMarkerPose,firstTrailingMarkerRange,false);
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
    // At the second rail the centre ray can see the wall while the outer
    // part of its fan still sees pink. Log23 drove back to undo that mixed
    // reading. Keep the unobstructed wall reference from between the rails.
    if (trustedGapWallRange>0.0f &&
        !registeredRailFanClear(get_position_struct(),true))
    {
        latestWallPose=trustedGapWallPose;
        latestWallRange=trustedGapWallRange;
        Serial.print("[FINAL PARK WALL] clear_gap_range_mm=");
        Serial.println(latestWallRange,1);
        return true;
    }
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
    // In the mirrored frame +X points towards the nose. Reference the rear
    // edge to the measured inner face of the entry-side rail.
    const float captureLocalX=captureRearAxleLocalX();
    captureTargetX = routeTurnSign > 0
        ? movingInsideFaceX() + captureLocalX
        : OBSTACLE_PARKING_FIXED_INNER_FACE_X_MM - captureLocalX;
    captureFieldMotionSign =
        captureTargetX >= get_position_struct().x_mm ? 1 : -1;
    return true;
}

bool finalFootprintContained()
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

PositionEstimate entryLocalPose(const PositionEstimate &field)
{
    return PositionEstimate{
        routeTurnSign > 0 ? field.x_mm-movingInsideFaceX()
                         : OBSTACLE_PARKING_FIXED_INNER_FACE_X_MM-field.x_mm,
        field.y_mm-OBSTACLE_SOUTH_OUTER_WALL_Y_MM,
        wrap180(routeTurnSign > 0 ? field.heading_deg : 180.0f-field.heading_deg)};
}

} // namespace

void final_parking_prepare_approach(int8_t turn_sign,const PositionEstimate &arrival)
{
    if(state!=FP_IDLE || initialBrakeStarted)return;
    auto &p=approachPreparation;
    if(!p.started)
    {
        p.started=true;p.arrival=arrival;p.direction=turn_sign<0?-1:1;
        routeTurnSign=p.direction;outerSensor=routeTurnSign>0?TOF_RIGHT:TOF_LEFT;
        resetPreparationVariant();
    }
    if(p.ready || p.failed)return;
    const uint32_t startedUs=micros();
    for(int work=0;work<12 && uint32_t(micros()-startedUs)<1200U;++work)
    {
        if(!selectParkingApproachCandidate(p.candidate))
        {++p.candidate;p.variant=0;resetPreparationVariant();continue;}
        const int result=advanceApproachPreflight(p.sample,p.outer);
        if(result<0 || ++p.steps>=1200)
        {
            ++p.candidate;p.variant=0;
            if(p.candidate>=parkingApproachCandidateCount()){p.failed=true;return;}
            resetPreparationVariant();
        }
        else if(result>0)
        {
            if(++p.variant==27){p.ready=true;return;}
            resetPreparationVariant();
        }
    }
}

bool final_parking_rolling_ready(const PositionEstimate &pose)
{
    return state==FP_IDLE && !initialBrakeStarted && approachPreparation.direction>0 &&
        preparedApproachFits(pose) && gyro_is_healthy() && dc_state==DC_ENABLED &&
        current_speed>=0.0f && current_speed<=190.0f &&
        measured_speed>=0.0f && measured_speed<=190.0f;
}

bool final_parking_start_rolling(int8_t turn_sign)
{
    const PositionEstimate pose=get_position_struct();
    if(turn_sign<0 || !final_parking_rolling_ready(pose))return false;
    const bool directOutward = approachDirectScan && pose.x_mm >= approachOuterShiftX;
    int steering=0;
    if(!parkingApproachSteering(pose,directOutward,steering) ||
       !parkingApproachCommandSafe(pose,steering,+1))return false;
    initialBrakeStarted=true;
    runStartDistance=stateStartDistance=get_distance();stateStartMs=millis();
    state=directOutward ? FP_APPROACH_OUTER : FP_APPROACH_LANE;
    Serial.println(PARKING_APPROACH_REVISION);
    Serial.print("[FINAL PARK] Prepared rolling handoff; outward shift x=");
    Serial.println(approachOuterShiftX,1);
    Serial.print("[FINAL PARK] Progressive outward shift=");
    Serial.println(approachSmoothCcw ? "yes" : "swept fallback");
    Serial.print("[FINAL PARK] Direct scan line=");Serial.println(approachDirectScan ? "yes" : "no");
    return true;
}

bool final_parking_try_corner_approach(int8_t turn_sign, bool wallArc)
{
    if (state!=FP_IDLE || initialBrakeStarted || dc_state!=DC_ENABLED ||
        !gyro_is_healthy() || current_speed<=0.0f || measured_speed<0.0f ||
        current_speed>220.0f || measured_speed>220.0f ||
        millis()-cornerCheckMs<180U) return false;
    cornerCheckMs=millis();
    routeTurnSign=turn_sign<0 ? -1 : 1;
    outerSensor=routeTurnSign>0 ? TOF_RIGHT : TOF_LEFT;
    scanFieldSign=routeTurnSign;
    const PositionEstimate pose=get_position_struct();
    if (!cornerWallPreflight(pose,wallArc))
    {
        scanFieldSign=-1;
        return false;
    }
    cornerUsesWallArc=wallArc;
    cornerFinishPending=true;
    initialBrakeStarted=true;
    runStartDistance=stateStartDistance=get_distance();stateStartMs=millis();
    state=FP_CORNER_TO_WALL;
    Serial.println(PARKING_APPROACH_REVISION);
    Serial.println(wallArc ? "[FINAL PARK] Rolling from final corner to wall" :
                             "[FINAL PARK] Rolling to wall after first green");
    return true;
}

void final_parking_notify_lap_finish(bool complete)
{
    if (complete) cornerFinishPending=false;
}

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
    scanSampleMs = 0;
    scanFastCadence = false;
    scanPhase = 0;
    classFrames = 0;
    scanArmed = false;
    scanLineEstablished = false;
    scanWallRegistered = false;
    scanFieldSign = -1;
    scanWallOffsetMm = 0.0f;
    cornerUsesWallArc = cornerFinishPending = false;
    cornerCheckMs = 0;
    scanAlignSign = 1;
    scanAlignTurns = 0;
    scanGeometryRetries = 0;
    scanAlignStartMs = 0;
    trailingCandidateStored = false;
    latestWallRange = 0.0f;
    trustedGapWallRange = 0.0f;
    measuredFixedInsideX = 0.0f;
    measuredMovingInsideX = 0.0f;
    measuredGapMm = 0.0f;
    captureTargetX = 0.0f;
    captureLineY = parkingScanLineY();
    captureTipSteeringLimited = false;
    captureFieldMotionSign = 1;
    captureAlignPending = false;
    completed = false;
    aborted = false;
    practiceMode = false;
    fixedCwPractice = false;

    initialBrakeStarted = false;
    approachOuterShiftX = OBSTACLE_FINAL_PARKING_OUTER_SHIFT_X_MM;
    approachSmoothCcw = false;
    approachDirectLane=false;
    approachDirectScan=false;
    approachPreparation=ApproachPreparation{};
    wallSequence = finalTraceMs = 0;
    finalTraceCount = initialWallFrames = 0;
    initialWallCorrection = 0.0f;
    lastWallCorrection = 0.0f;
    wallCorrectionSequence = 0;
    stoppedWallSequence=0;stoppedWallFrames=0;stoppedWallRange=0.0f;
    entryBrakeEvidenceValid=false;
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
    Serial.println(" full_approach_scan=yes");
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
    const float localX=captureRearAxleLocalX();
    const float localY=parkingScanLineY()-OBSTACLE_SOUTH_OUTER_WALL_Y_MM;
    localToField(localX,localY,0,
        start.x_mm,start.y_mm,start.heading_deg);
    position_reset(start.x_mm,start.y_mm,start.heading_deg);
    captureTargetX=start.x_mm;
    runStartDistance=stateStartDistance=get_distance();
    stateStartMs=millis();
    state=FP_CAPTURE_BRAKE;
    stop(true);
    Serial.println(PARKING_APPROACH_REVISION);
    Serial.print("[FINAL PARK PRACTICE] entry_only=yes turn=");
    Serial.print(routeTurnSign>0 ? "CCW":"CW");
    Serial.print(" gap_mm=");Serial.print(measuredGapMm,1);
    Serial.print(" local_axle_x_y_mm=");
    Serial.print(localX,2);Serial.print("/");
    Serial.println(localY,2);
}

void final_parking_start_cw_sequence_practice()
{
    final_parking_reset();
    practiceMode = fixedCwPractice = true;
    routeTurnSign = -1;
    outerSensor = TOF_LEFT;
    const float cornerWallX=OBSTACLE_CENTERLINE_HALF_EXTENT_MM+
        OBSTACLE_CORRIDOR_HALF_WIDTH_MM;
    const float startX=cornerWallX-OBSTACLE_FINAL_PARKING_REAR_OVERHANG_MM-20.0f;
    const float scanY=parkingScanLineY();
    position_reset(startX,scanY,baseHeadingDeg());
    runStartDistance=stateStartDistance=get_distance();
    stateStartMs=millis();
    state=FP_FIXED_APPROACH;
    stop(false);
    set_steering(0);
    Serial.println(PARKING_APPROACH_REVISION);
    Serial.print("[FINAL PARK FIXED] CW corner start/rear clearance/axle wall mm=");
    Serial.print(startX,1); Serial.print("/20.0/");
    Serial.println(scanY-OBSTACLE_SOUTH_OUTER_WALL_Y_MM,1);
}

namespace
{
struct FixedParkingMove
{
    int direction;
    int steering;
    float headingDeg;
    float distanceMm;
    float maxTravelMm;
};

// Steering is in the mirrored bay frame. Forward with the SAME steering
// as the initial reverse arc turns the heading back towards zero.
constexpr FixedParkingMove FIXED_ENTRY_MOVES[] = {
    {-1,  50, 47.0f,   0.0f, 150.0f},
    {-1,   0, 47.0f, 125.0f, 140.0f},
    {-1, -50, 22.5f,  0.0f,  95.0f},
    { 1,  50,  0.0f,  0.0f,  95.0f},
};
constexpr uint8_t FIXED_ENTRY_MOVE_COUNT=sizeof(FIXED_ENTRY_MOVES)/sizeof(FIXED_ENTRY_MOVES[0]);
constexpr float FIXED_TURN_SLIP_RESERVE_MM = 35.0f;
constexpr uint32_t FIXED_MOVE_LIMIT_MS = 5000U;
constexpr uint32_t FIXED_TURN_SLIP_LIMIT_MS = 8500U;

void beginFixedEntry(const PositionEstimate &pose)
{
    segmentIndex=0;
    state=FP_FIXED_SETTLE;
    Serial.print("[FINAL PARK FIXED] Capture accepted; heading error_deg=");
    Serial.println(wrap180(pose.heading_deg-baseHeadingDeg()),1);
}

void updateFixedEntry(const PositionEstimate &pose)
{
    const auto &move=FIXED_ENTRY_MOVES[segmentIndex];
    const int steering=move.steering*routeTurnSign;
    const float heading=entryLocalPose(pose).heading_deg;
    const float encoder=get_distance();
    if (!isfinite(encoder)) { abortParking("fixed_invalid_encoder"); return; }
    if (state==FP_FIXED_SETTLE)
    {
        servo_disabled=false;
        set_steering(steering);
        steer(steering);
        if (!steering_command_settled(steering,OBSTACLE_FINAL_PARKING_STEER_SETTLE_MS)) return;
        stateStartDistance=encoder;
        stateStartMs=fixedProgressMs=millis();
        fixedProgress=0.0f;
        fixedStepStartHeading=heading;
        fixedSlipReserveLogged=false;
        state=FP_FIXED_DRIVE;
        Serial.print("[FINAL PARK FIXED] start step="); Serial.print(segmentIndex+1);
        Serial.print(" steering="); Serial.print(steering);
        Serial.print(" target_deg/mm="); Serial.print(move.headingDeg,1);
        Serial.print("/"); Serial.println(move.distanceMm,1);
        return;
    }

    const float travel=move.direction*(encoder-stateStartDistance);
    if (state==FP_FIXED_BRAKE)
    {
        if (!parkingEntryBrakeReady()) return;
        Serial.print("[FINAL PARK FIXED] end step="); Serial.print(segmentIndex+1);
        Serial.print(" heading/travel_mm="); Serial.print(heading,1);
        Serial.print("/"); Serial.println(travel,1);
        if (++segmentIndex<FIXED_ENTRY_MOVE_COUNT)
        {
            stop(false);
            state=FP_FIXED_SETTLE;
        }
        else
        {
            servo_disabled=false;
            set_steering(0);
            steer(0);
            state=FP_VERIFY;
        }
        return;
    }

    const bool turning=move.steering!=0;
    const uint32_t now=millis();
    const uint32_t elapsed=now-stateStartMs;
    const bool normalLimit=travel>move.maxTravelMm || elapsed>FIXED_MOVE_LIMIT_MS;
    if (travel < -6.0f || (!turning && normalLimit) ||
        (turning && (travel>move.maxTravelMm+FIXED_TURN_SLIP_RESERVE_MM ||
                     elapsed>FIXED_TURN_SLIP_LIMIT_MS)))
    { abortParking("fixed_travel_or_time_limit"); return; }
    const float turnSign=segmentIndex==0 ? 1.0f : -1.0f;
    const float headingRemaining=(move.headingDeg-heading)*turnSign;
    const bool reached=turning ? headingRemaining<=0.5f : travel>=move.distanceMm;
    if (reached)
    {
        stop(true);
        stateStartMs=millis();
        state=FP_FIXED_BRAKE;
        return;
    }
    const float progress=turning ? (heading-fixedStepStartHeading)*turnSign : travel;
    if (progress>fixedProgress+(turning ? 1.0f:2.0f))
    { fixedProgress=progress;fixedProgressMs=now; }
    else if (now-fixedProgressMs>1300U)
    { abortParking("fixed_no_measured_progress"); return; }
    if (move.direction<0)
    {
        TofDiagnosticSnapshot rear{};
        if (get_tof_diagnostic_snapshot(TOF_REAR,rear) && millis()-rear.sampled_ms<=200 &&
            rear.selected_signal_mcps>0 && rear.selected_raw_distance_mm>0 &&
            rear.selected_raw_distance_mm<=20.0f)
        {
            abortParking("fixed_rear_tof_close");
            return;
        }
    }
    if (normalLimit)
    {
        const float requiredTurn=fabsf(move.headingDeg-fixedStepStartHeading);
        if (headingRemaining>18.0f || progress<fminf(12.0f,0.5f*requiredTurn) ||
            progress<fixedProgress-1.0f || now-fixedProgressMs>650U ||
            !scanCommandSafe(pose,move.direction,steering,12))
        { abortParking("fixed_travel_or_time_limit"); return; }
        if (!fixedSlipReserveLogged)
        {
            fixedSlipReserveLogged=true;
            Serial.print("[FINAL PARK SLIP] step/yaw_remaining/encoder_mm=");
            Serial.print(segmentIndex+1);Serial.print("/");
            Serial.print(headingRemaining,1);Serial.print("/");
            Serial.println(travel,1);
        }
    }
    servo_disabled=false;
    set_steering(steering);
    const int speed=turning && headingRemaining<6.0f ? 35 : 45;
    set_speed(move.direction*speed);
}
}

bool final_parking_update(int8_t turn_sign)
{
    if (!OBSTACLE_FINAL_PARKING_ENABLED)
        return false;

    if (state == FP_ABORT)
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
    if ((state == FP_IDLE || state == FP_VERIFY) && !checkMotionHealth(pose,state==FP_IDLE))
        return true;
    if (state == FP_IDLE)
    {
        if (!parkingBrakeReady(0)) return true;
        // Position HOLD retains the old driving speed profile. Only after
        // measured standstill, clear it before the low-speed parking command
        // (CW may reverse relative to the completed lap).
        stop(false);
        bool fromLeft=cwRedBeforeBay() && leftWallPreflight(pose);
        if (!fromLeft && cwRedBeforeBay())
        {
            // A close outside block can consume the 23-cm line. Try the
            // remaining corridor with a straight-wheel 10-mm rail gap.
            scanWallOffsetMm=19.0f;
            fromLeft=leftWallPreflight(pose);
            if (!fromLeft) scanWallOffsetMm=0.0f;
        }
        if (!fromLeft && !(preparedApproachFits(pose) || chooseParkingApproach(pose)))
        {
            abortParking("approach_swept_preflight");
            return true;
        }
        Serial.println("[FINAL PARK] Learned-route approach swept preflight PASS");
        Serial.println(PARKING_APPROACH_REVISION);
        // The stopped geometric search may outlast the 200-ms sensor age.
        // Service queued reports before attempting the first motion; a
        // genuinely stale/missing report still follows the normal hold.
        update_gyro();
        Serial.print("[FINAL PARK] Outward shift x="); Serial.println(approachOuterShiftX,1);
        Serial.print("[FINAL PARK] Direct staging lane=");Serial.println(approachDirectLane ? "yes":"no");
        Serial.print("[FINAL PARK] Direct scan line=");Serial.println(approachDirectScan ? "yes" : "no");
        if (routeTurnSign>0)
        {
            Serial.print("[FINAL PARK] Progressive outward shift=");
            Serial.println(approachSmoothCcw ? "yes" : "swept fallback");
        }
        runStartDistance = get_distance();
        stateStartDistance = runStartDistance;
        stateStartMs = millis();
        state = approachDirectScan && pose.x_mm >= approachOuterShiftX
            ? FP_APPROACH_OUTER : FP_APPROACH_LANE;
        if (fromLeft)
        {
            scanFieldSign=1;
            state=FP_LEFT_TO_WALL;
            Serial.print("[FINAL PARK] Wall approach and rail scan from left; rail gap_mm=");
            Serial.println(FIXED_ENTRY_RAIL_CLEARANCE_MM-scanWallOffsetMm,1);
        }
        Serial.print("[FINAL PARK] Start turn=");
        Serial.print(routeTurnSign > 0 ? "CCW" : "CW");
        Serial.print(" rail_clearance_mm=");
        Serial.println(FIXED_ENTRY_RAIL_CLEARANCE_MM, 1);
    }

    pose = get_position_struct();
    traceParking();
    if (((state >= FP_APPROACH_LANE && state <= FP_CAPTURE_DRIVE) ||
         state == FP_SCAN_ALIGN_DRIVE || state == FP_SCAN_ALIGN_BRAKE ||
         state == FP_CAPTURE_ALIGN_DRIVE || state == FP_CAPTURE_ALIGN_BRAKE ||
         state == FP_APPROACH_WALL_CLEAR || state == FP_FIXED_APPROACH ||
         state == FP_CORNER_TO_WALL || state == FP_LEFT_TO_WALL) &&
        !checkMotionHealth(pose))
        return true;

    if (state==FP_CORNER_TO_WALL || state==FP_LEFT_TO_WALL)
    {
        const bool rolling=state==FP_CORNER_TO_WALL;
        const float travelLimit=rolling && scanLineEstablished ? 1850.0f : 1150.0f;
        if (millis()-stateStartMs>14000U || distanceSince(stateStartDistance)>travelLimit)
        { abortParking("wall_line_acquisition_limit");return true; }
        if (rolling && cornerScanReady(pose)) scanLineEstablished=true;
        if ((rolling ? cornerScanReady(pose) : leftWallReady(pose)))
        {
            updateOuterWallReference(true);
            pose=get_position_struct();
            if (rolling && cornerScanReady(pose) && scanLeadRemaining(pose)<=450.0f)
            {
                stateStartMs=millis();stateStartDistance=get_distance();
                state=FP_SCAN_DRIVE;
                return true;
            }
            if (!rolling)
            {
                stop(true);stateStartMs=millis();state=FP_APPROACH_BRAKE;return true;
            }
        }
        const int direction=rolling ? 1 : motorDirectionForFieldMotion(-1);
        const int steering=!rolling ? wallMergeSteering(pose,-1) : scanLineEstablished
            ? fieldLineSteering(pose,routeTurnSign,parkingScanLineY())
            : cornerWallSteering(pose,cornerUsesWallArc);
        const int preview=parkingRollingPreviewMm();
        const bool clear=rolling ?
            parkingApproachCommandSafe(pose,steering,direction,(preview+1)/2) :
            scanCommandSafe(pose,direction,steering,preview);
        if (!clear)
        { abortParking("wall_line_acquisition_clearance");return true; }
        if (rolling && scanLeadRemaining(pose)<75.0f)
        { abortParking("wall_line_acquisition_before_rail");return true; }
        const int speed=abs(steering)<24 ?
            (rolling && scanLineEstablished && scanLeadRemaining(pose)>450.0f ?
                FREE_PARK_APPROACH_SPEED : FREE_NORMAL_SCAN_APPROACH_SPEED) : 130;
        servo_disabled=false;
        set_steering(steering);
        set_speed(direction*speed);
        return true;
    }

    if (state==FP_FIXED_APPROACH)
    {
        if (millis()-stateStartMs>15000U || get_distance()-runStartDistance>650.0f)
        { abortParking("fixed_corner_approach_limit");return true; }
        if (pose.x_mm<=1250.0f && initialWallFrames<2)
            updateOuterWallReference(true);
        pose=get_position_struct();
        if (pose.x_mm<=900.0f)
        {
            stateStartMs=millis();
            if (initialWallFrames>=2 && linePoseReady(pose,parkingScanLineY()))
            {
                stateStartDistance=get_distance();
                state=FP_SCAN_DRIVE;
                Serial.println("[FINAL PARK FIXED] Rolling into normal rail scan");
            }
            else
            {
                stop(true);
                state=FP_APPROACH_BRAKE;
            }
            return true;
        }
        // Move away from the corner before applying lateral line corrections.
        const int steering=pose.x_mm>1400.0f ? parkingParallelSteering(pose,1) :
            fieldLineSteering(pose,-1,parkingScanLineY());
        if (pose.x_mm<=1400.0f && !scanCommandSafe(pose,1,steering,28))
        { abortParking("fixed_corner_approach_clearance");return true; }
        const bool freeStraight = pose.x_mm>1000.0f && pose.x_mm<=1400.0f &&
            abs(steering)<=15 && scanCommandSafe(pose,1,steering,36);
        servo_disabled=false;
        set_steering(steering);
        set_speed(freeStraight ? FREE_SCAN_APPROACH_SPEED : FIXED_ENTRY_APPROACH_SPEED);
        return true;
    }

    if (state>=FP_FIXED_SETTLE && state<=FP_FIXED_BRAKE)
    {
        if (checkMotionHealth(pose,false)) updateFixedEntry(pose);
        return true;
    }

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
        else if (!driveCheckedParkingApproach(pose,false))
        { abortParking("approach_motion_clearance"); return true; }
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
        // Register the wall while still moving so the last part of the
        // outward curve converges on the physical scan line.
        if (!practiceMode && !scanWallRegistered && updateOuterWallReference(true))
            scanWallRegistered=true;
        pose=get_position_struct();
        if (!driveCheckedParkingApproach(pose,true))
        { abortParking("outer_motion_clearance"); return true; }
        const bool lineReady=pose.x_mm>=parkingScanStartX() &&
            linePoseReady(pose,parkingScanLineY());
        const bool atAlignmentLimit=pose.x_mm>=1030.0f;
        if ((lineReady && (scanWallRegistered || practiceMode)) || atAlignmentLimit)
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

    if (state == FP_APPROACH_WALL_CLEAR)
    {
        if (updateOuterWallReference(true))
        {
            stop(true);
            scanWallRegistered = true;
            stateStartMs = millis();
            state = FP_APPROACH_BRAKE;
        }
        else
        {
            const int steering = parkingParallelSteering(pose,+1);
            if (pose.x_mm>=1030.0f || millis()-stateStartMs>5000U ||
                !parkingApproachCommandSafe(pose,steering,+1))
                abortParking("ccw_clear_wall_reference");
            else
            {
                set_steering(steering);
                const bool free = pose.x_mm<990.0f &&
                    parkingApproachCommandSafe(pose,steering,+1,20);
                set_speed(free ? 90 : OBSTACLE_FINAL_PARKING_SCAN_SPEED);
            }
        }
        return true;
    }

    if (state == FP_APPROACH_BRAKE)
    {
        if (!parkingEntryBrakeReady())
            return true;
        if (!scanWallRegistered && initialWallFrames < 2 && !updateOuterWallReference(true))
        {
            if (scanFieldSign<0 && routeTurnSign>0 && !practiceMode && pose.x_mm<1000.0f &&
                millis()-stateStartMs>400U)
            {
                initialWallFrames = 0;
                state = FP_APPROACH_WALL_CLEAR;
                stateStartMs = millis();
                Serial.println("[FINAL PARK] Continue straight beyond rail fan for wall reference");
            }
            else if (millis()-stateStartMs > 2500)
                abortParking("initial_outer_wall_reference");
            return true;
        }
        scanWallRegistered = true;
        // Registration can move Y by tens of millimetres (log 510: -25.4).
        // Regain the scan line in the checked outward corridor BEFORE passing
        // either marker. Keep the existing distance/X/time bounds in force.
        if (!linePoseReady(get_position_struct(),activeScanLineY()))
        {
            stop(false);
            servo_disabled = false;
            scanAlignSign = scanFieldSign<0 && pose.x_mm < 1000.0f ? 1 : -1;
            scanAlignmentPreflight(get_position_struct(),&scanAlignSign);
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
        if (!steering_command_settled(0,OBSTACLE_FINAL_PARKING_STEER_SETTLE_MS))
            return true;
        TofDiagnosticSnapshot snapshot;
        tofSequence = get_tof_diagnostic_snapshot(outerSensor, snapshot)
            ? snapshot.sequence
            : 0;
        scanSampleMs = 0;
        scanFastCadence = false;
        scanPhase = 0;
        classFrames = 0;
        trustedGapWallRange = 0.0f;
        // Odometry drifts longitudinally over three laps. Start looking while
        // still on the established wall baseline, not at a guessed marker X.
        scanArmed = true;
        scanLineEstablished = false;
        trailingCandidateStored = false;
        stateStartDistance = get_distance();
        stateStartMs = millis();
        state = FP_SCAN_DRIVE;
        Serial.println(scanFieldSign<0 ? "[FINAL PARK] Rail scan from right" :
                                         "[FINAL PARK] Rail scan from left");
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
            if (!parkingEntryBrakeReady()) return true;
            stop(false);
            if (linePoseReady(pose,activeScanLineY()))
            {
                servo_disabled=false;set_steering(0);steer(0);
                stateStartMs=millis();state=FP_SCAN_SETTLE;return true;
            }
            scanAlignSign=-scanAlignSign;
            ++scanAlignTurns;
            state=FP_SCAN_ALIGN_DRIVE;
        }
        const int direction=motorDirectionForFieldMotion(scanAlignSign);
        const int steering=fieldLineSteering(pose,scanAlignSign,activeScanLineY());
        const int rollingPreview=static_cast<int>(fmaxf(15,parkingRollingPreviewMm()));
        if (!scanCommandSafe(pose,direction,steering,rollingPreview))
        { abortParking("scan_alignment_clearance"); return true; }
        const int outward=-scanFieldSign;
        const float sideX=outward*pose.x_mm;
        const float nearX=scanFieldSign<0 ? 800.0f : 260.0f;
        const float farX=scanFieldSign<0 ? 1080.0f : 660.0f;
        const float toBoundary=scanAlignSign==outward ? farX-sideX : sideX-nearX;
        const bool faster=toBoundary>60.0f && abs(steering)<=24 &&
            scanCommandSafe(pose,direction,steering,36);
        const int speed=parkingLineAcquisitionSpeed(pose,
            faster ? 155 : OBSTACLE_FINAL_PARKING_ALIGN_SPEED,
            scanFieldSign<0 ? 800.0f : pose.x_mm,30.0f,
            faster ? 88 : FIXED_ENTRY_SCAN_SPEED);
        servo_disabled=false;
        set_steering(steering);
        set_speed(direction*speed);
        const bool boundary=toBoundary<=0.0f;
        const bool ready=sideX>=nearX && linePoseReady(pose,activeScanLineY());
        if (boundary || ready)
        { stop(true);stateStartMs=millis();state=FP_SCAN_ALIGN_BRAKE; }
        return true;
    }

    if (state == FP_SCAN_DRIVE)
    {
        updateOuterWallReference(false);
        pose = get_position_struct();
        const int direction=motorDirectionForFieldMotion(scanFieldSign);
        const float scanY=activeScanLineY();
        TofDiagnosticSnapshot snapshot;
        const bool hasSnapshot=get_tof_diagnostic_snapshot(outerSensor, snapshot);
        const bool freshFrame=hasSnapshot &&
            snapshot.sequence!=tofSequence && millis()-snapshot.sampled_ms<=250U;
        if (freshFrame)
        {
            scanFastCadence=scanSampleMs!=0 && snapshot.sampled_ms-scanSampleMs<=45U;
            scanSampleMs=snapshot.sampled_ms;
            tofSequence=snapshot.sequence;
        }
        int steering=fieldLineSteering(pose,scanFieldSign,scanY);
        if (scanLineEstablished && (scanPhase>=2 || (scanPhase==1 && classFrames>0)))
        {
            // The first rail can produce a mixed ToF return for many frames.
            // Keep the already established parallel heading between the rails.
            steering=static_cast<int>(clampFloat(steering,-8.0f,8.0f));
        }
        const float freeLead=scanFieldSign<0
            ? pose.x_mm-OBSTACLE_FINAL_PARKING_SCAN_START_X_MM
            : 65.0f-pose.x_mm;
        const bool beforeMarkers=scanPhase<2 && classFrames==0 && freeLead>0.0f;
        int scanSpeed=FIXED_ENTRY_SCAN_SPEED;
        if (beforeMarkers)
        {
            PositionEstimate afterBrake=pose;
            advanceParkingPose(afterBrake,steering,60.0f*direction);
            const bool clearFan=earlyWallReferenceClearOfMarkers(pose);
            if (clearFan && abs(steering)<=15 && earlyWallReferenceClearOfMarkers(afterBrake))
            {
                const float expected=expectedOuterWallRange(outerSensor,pose);
                const bool measuredWall=hasSnapshot && millis()-snapshot.sampled_ms<=120U &&
                    expected>0.0f && fabsf(parkingOuterRange(snapshot)-expected)<=30.0f;
                scanSpeed=!practiceMode && measuredWall ? FREE_NORMAL_SCAN_APPROACH_SPEED
                                                       : FREE_SCAN_APPROACH_SPEED;
            }
            else if (fixedCwPractice || clearFan)
                scanSpeed=FIXED_ENTRY_APPROACH_SPEED;
            if (!practiceMode)
                scanSpeed=DriveProfile::approachSpeed(
                    freeLead, scanSpeed,
                    FIXED_ENTRY_SCAN_SPEED, 30.0f, 75.0f);
        }
        if (scanSpeed==FIXED_ENTRY_SCAN_SPEED && scanPhase>0 && scanPhase<4 && scanFastCadence &&
            scanLineEstablished && millis()-scanSampleMs<=50U)
            scanSpeed=FRESH_MARKER_SCAN_SPEED;
        const int rollingPreview=parkingRollingPreviewMm();
        const int previewMm=static_cast<int>(fmaxf(rollingPreview,
            (scanSpeed*17+FIXED_ENTRY_SCAN_SPEED-1)/FIXED_ENTRY_SCAN_SPEED));
        if (!scanCommandSafe(pose,direction,steering,previewMm))
        {
            // Check the shorter preview at scan speed before giving up the
            // steering correction. Neutral remains the last checked option.
            scanSpeed=FIXED_ENTRY_SCAN_SPEED;
            const int slowPreview=static_cast<int>(fmaxf(rollingPreview,17));
            if (!scanCommandSafe(pose,direction,steering,slowPreview))
            {
                steering=0;
                if (!scanCommandSafe(pose,direction,steering,slowPreview))
                { abortParking("scan_motion_clearance"); return true; }
            }
        }
        servo_disabled=false;
        set_steering(steering);
        set_speed(direction*scanSpeed);
        if (!scanArmed)
        {
            scanArmed = true;
            scanPhase = 0;
            classFrames = 0;
            Serial.println("[FINAL PARK] Dual-marker scan armed");
        }
        if (!scanLineEstablished &&
            linePoseReady(pose,scanY))
            scanLineEstablished = true;
        if (scanArmed && scanLineEstablished && freshFrame)
            processScanFrame(snapshot);
        if (scanPhase == 5 && !cornerFinishPending)
        {
            stop(true);
            stoppedWallSequence=0;stoppedWallFrames=0;
            stateStartMs = millis();
            state = FP_SCAN_BRAKE;
        }
        // The markers, not accumulated three-lap X odometry, define the bay.
        // Permit the same bounded registration error at the search endpoint.
        else if ((scanFieldSign<0 ? pose.x_mm<=OBSTACLE_FINAL_PARKING_SCAN_END_X_MM-
                                    OBSTACLE_FINAL_PARKING_MAX_X_POSE_CORRECTION_MM
                                  : pose.x_mm>=OBSTACLE_PARKING_FIXED_DOTTED_LINE_X_MM+
                                    100.0f+OBSTACLE_FINAL_PARKING_MAX_X_POSE_CORRECTION_MM) ||
                 distanceSince(stateStartDistance) >=
                     OBSTACLE_FINAL_PARKING_SCAN_MAX_MM ||
                 millis() - stateStartMs >=
                     OBSTACLE_FINAL_PARKING_SCAN_TIMEOUT_MS)
            abortParking("dual_marker_scan_incomplete");
        return true;
    }

    if (state == FP_SCAN_BRAKE)
    {
        if (cornerFinishPending)
        { abortParking("lap_finish_before_parking_entry");return true; }
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
            // A mixed/grazing edge can shorten the apparent bay (log6:222mm).
            // Do not widen the accepted gap or enter using that geometry.
            // Retrace the checked outer corridor and acquire both rails anew.
            if (scanGeometryRetries<1)
            {
                const PositionEstimate retryPose=get_position_struct();
                scanPhase=0;classFrames=0;trailingCandidateStored=false;
                measuredGapMm=0.0f;
                // A left-to-right scan already finishes on the right.
                // Rescan from that side instead of recrossing both markers.
                scanFieldSign=-1;
                if (scanAlignmentPreflight(retryPose,&scanAlignSign))
                {
                    ++scanGeometryRetries;
                    scanAlignStartMs=millis();scanAlignTurns=0;
                    servo_disabled=false;
                    state=FP_SCAN_ALIGN_DRIVE;
                    Serial.println("[FINAL PARK] Uncertain marker geometry; checked rescan");
                    return true;
                }
            }
            abortParking("dual_marker_geometry");
            return true;
        }
        pose=get_position_struct();
        if (!chooseRegisteredCapture(pose))
        { abortParking("capture_alignment_preflight"); return true; }
        if (fixedEntryCaptureReady(pose))
        { beginFixedEntry(pose);return true; }
        servo_disabled = false;
        set_steering(0);
        steer(0);
        stateStartMs = millis();
        state = FP_CAPTURE_SETTLE;
        return true;
    }

    if (state == FP_CAPTURE_SETTLE)
    {
        // The stop and wall-reference checks already ran. Count any time the
        // same neutral steering command has actually been held.
        if (!steering_command_settled(0,OBSTACLE_FINAL_PARKING_STEER_SETTLE_MS))
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
        if (!driveCaptureLine(pose,+1))
        { abortParking("capture_alignment_clearance"); return true; }
        if (linePoseReady(pose,captureLineY) &&
            pose.x_mm>=captureTargetX+80.0f)
        { stop(true);stateStartMs=millis();state=FP_CAPTURE_ALIGN_BRAKE; }
        else if (distanceSince(stateStartDistance)>=OBSTACLE_FINAL_PARKING_CAPTURE_MAX_MM ||
                 millis()-stateStartMs>=OBSTACLE_FINAL_PARKING_CAPTURE_TIMEOUT_MS)
            abortParking("capture_alignment_limit");
        return true;
    }

    if (state == FP_CAPTURE_ALIGN_BRAKE)
    {
        if (!parkingEntryBrakeReady()) return true;
        stop(false);
        if (!linePoseReady(get_position_struct(),captureLineY))
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
        const bool targetReached = captureFieldMotionSign > 0
            ? pose.x_mm >= captureTargetX - 1.5f
            : pose.x_mm <= captureTargetX + 1.5f;
        if (!targetReached && !driveCaptureLine(pose,captureFieldMotionSign))
        { abortParking("capture_motion_clearance"); return true; }
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
            abortParking("fixed_capture_limit");
        return true;
    }

    if (state == FP_CAPTURE_BRAKE)
    {
        if (!checkMotionHealth(pose,false)) return true;
        if (!parkingEntryBrakeReady()) return true;
        stop(false);
        pose=get_position_struct();
        if (fixedEntryCaptureReady(pose)) beginFixedEntry(pose);
        else abortParking("fixed_capture_not_aligned");
        return true;
    }

    if (state == FP_VERIFY)
    {
        servo_disabled=false;
        set_steering(0);
        steer(0);
        if (!steering_command_settled(0,OBSTACLE_FINAL_PARKING_STEER_SETTLE_MS)) return true;
        set_steering(0);
        // In DC_HOLDING the drive profile retains its last commanded speed.
        // Use measured axle stillness before verifying the parked footprint.
        if (!parkingBrakeReady(0)) return true;
        pose = get_position_struct();
        const bool contained = finalFootprintContained();
        Serial.print("[FINAL PARK RESULT] contained=");
        Serial.print(contained ? "yes" : "no");
        Serial.print(" stopped=");
        Serial.print("yes");
        Serial.print(" pose_x_y_heading=");
        Serial.print(pose.x_mm, 1);
        Serial.print("/");
        Serial.print(pose.y_mm, 1);
        Serial.print("/");
        Serial.print(pose.heading_deg, 1);
        Serial.print(" travel_mm=");
        Serial.println(distanceSince(runStartDistance), 1);
        if (!contained)
        { abortParking("fixed_final_pose");return true; }
        stop(true);
        completed = true;
        state = FP_HOLD;
        Serial.println("[FINAL PARK] Complete; motor hold active");
        run_telemetry_finish("completed","final_parking_stop");
        robot_logger.write_to_usb();
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
    if (state==FP_FIXED_DRIVE) fixedProgressMs += paused;
    if (scanAlignStartMs) scanAlignStartMs += paused;
    sensorHoldPending = false;
}

const char *final_parking_trace_phase() {
    static char phase[32];
    snprintf(phase,sizeof(phase),"final_park_%u",static_cast<unsigned>(state));
    return phase;
}
