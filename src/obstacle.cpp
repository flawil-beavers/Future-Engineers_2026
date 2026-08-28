#include "obstacle.h"

#include "config.h"
#include "motor_control.h"
#include "sensors.h"
#include "navigation_controller.h"
#include "course_map.h"
#include "obstacle_path.h"
#include "logger.h"
#include "position_estimator.h"

#define Serial robot_logger

// ============================================================
// OBSTACLE AVOIDANCE STATE
// ============================================================

static ObstacleAvoidanceState oa_state =
    OA_IDLE;

static ColorType oa_color =
    ColorType::NONE;

// Gyro heading that was active before starting avoidance.
static float oa_base_heading = 0.0f;

// Encoder distance at beginning of a state.
static float oa_state_start_distance = 0.0f;

// Used to avoid immediately detecting the same obstacle again.
static float oa_last_finish_distance = 0.0f;
static bool oa_has_finished_obstacle = false;

static uint8_t oa_confirm_frames = 0;
static uint8_t oa_lost_frames = 0;
static ColorType oa_candidate_color = ColorType::NONE;

static float oa_last_camera_error = 0.0f;

// ============================================================
// OBSTACLE CHALLENGE STATE
// ============================================================

static bool oc_was_enabled = false;
static bool oc_bench_test = false;
static bool oc_finish_requested = false;
static bool oc_complete = false;

enum ParkingExitState : uint8_t
{
    PARKING_EXIT_IDLE,
    PARKING_EXIT_REAR_SETTLE,
    PARKING_EXIT_REAR_DRIVE,
    PARKING_EXIT_REAR_BRAKE,
    PARKING_EXIT_SEGMENT_SETTLE,
    PARKING_EXIT_SEGMENT_DRIVE,
    PARKING_EXIT_SEGMENT_BRAKE,
    PARKING_EXIT_LOCALIZE_SETTLE,
    PARKING_EXIT_LOCALIZE_DRIVE,
    PARKING_EXIT_LOCALIZE_BRAKE,
    PARKING_EXIT_TEST_HOLD,
    PARKING_EXIT_DONE
};

struct ParkingExitSegment
{
    int8_t direction;
    int8_t steeringRelativeToAway;
    float distanceMm;
};

// Steering is expressed relative to the direction away from the outer wall,
// so the same path mirrors automatically for CW and CCW starts.
static constexpr ParkingExitSegment PARKING_EXIT_SEGMENTS[
    OBSTACLE_PARKING_EXIT_SEGMENT_COUNT] = {
    {-1, -1, 20.0f},
    {+1, +1, 25.0f},
    {-1, -1, 20.0f},
    {+1, +1, 85.0f},
    {+1, -1, OBSTACLE_PARKING_EXIT_FINAL_ALIGN_MODEL_MM}};

static ParkingExitState oc_parking_exit_state =
    PARKING_EXIT_IDLE;
static float oc_parking_exit_state_distance = 0.0f;
static float oc_parking_exit_run_start_distance = 0.0f;
static float oc_parking_exit_start_heading = 0.0f;
static float oc_parking_exit_start_left_mm = 0.0f;
static float oc_parking_exit_start_right_mm = 0.0f;
static int oc_parking_exit_steering = 0;
static uint32_t oc_parking_exit_brake_start_ms = 0;
static uint32_t oc_parking_exit_settle_start_ms = 0;
static uint8_t oc_parking_exit_segment = 0;
static bool oc_parking_field_pose_initialized = false;
static float oc_parking_rear_start_distance = 0.0f;
static float oc_parking_rear_start_range = -1.0f;
static float oc_parking_rear_last_range = -1.0f;
static uint32_t oc_parking_rear_tof_sequence = 0;
static uint8_t oc_parking_rear_confirm_frames = 0;
static float oc_parking_rear_sample_sum = 0.0f;
static float oc_parking_rear_sample_min = 0.0f;
static float oc_parking_rear_sample_max = 0.0f;
static float oc_parking_rear_planned_travel = 0.0f;
static float oc_parking_rear_cumulative_travel = 0.0f;
static float oc_parking_rear_last_encoder_distance = 0.0f;
static int8_t oc_parking_rear_direction = 0;
static bool oc_parking_rear_verifying = false;
static uint8_t oc_parking_rear_discard_frames = 0;
static TofSensor oc_parking_localization_sensor = TOF_RIGHT;
static float oc_parking_localization_start_distance = 0.0f;
static float oc_parking_localization_initial_piece_range = 0.0f;
static float oc_parking_localization_last_piece_range = 0.0f;
static float oc_parking_localization_wall_range = 0.0f;
static PositionEstimate oc_parking_localization_last_piece_pose;
static PositionEstimate oc_parking_localization_latest_wall_pose;
static uint32_t oc_parking_localization_tof_sequence = 0;
static uint8_t oc_parking_localization_wall_frames = 0;
static bool oc_parking_localization_transition_found = false;
static bool oc_parking_localization_piece_seen = false;

static uint8_t oc_current_section = 0;
static uint8_t oc_current_lap = 0;
static float oc_section_start_distance = 0.0f;
static NavigationState oc_last_navigation_state = NAV_IDLE;
static bool oc_corner_settling = false;
static int oc_last_completed_turn = 0;
static bool oc_known_obstacle_used[
    COURSE_MAX_OBSTACLES_PER_SECTION] = {false, false};

// ============================================================
// GENERAL HELPERS
// ============================================================

static float clampValue(
    float value,
    float minimum,
    float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }

    if (value > maximum)
    {
        return maximum;
    }

    return value;
}

static float distanceSince(
    float startDistance)
{
    return fabsf(
        get_distance() - startDistance);
}

static float applyWallGuard(float steering)
{
    const float left = get_tof_distance(TOF_LEFT);
    const float right = get_tof_distance(TOF_RIGHT);
    float correction = 0.0f;

    if (left > 0.0f && left < OBSTACLE_WALL_GUARD_DISTANCE_MM)
    {
        correction +=
            (OBSTACLE_WALL_GUARD_DISTANCE_MM - left) *
            OBSTACLE_WALL_GUARD_KP;
    }

    if (right > 0.0f && right < OBSTACLE_WALL_GUARD_DISTANCE_MM)
    {
        correction -=
            (OBSTACLE_WALL_GUARD_DISTANCE_MM - right) *
            OBSTACLE_WALL_GUARD_KP;
    }

    correction = clampValue(
        correction,
        -OBSTACLE_WALL_GUARD_MAX_STEERING,
        OBSTACLE_WALL_GUARD_MAX_STEERING);

    return steering + correction;
}

static void setAvoidanceSpeed(int speed)
{
    if (!oc_bench_test)
    {
        set_speed(speed);
    }
}

static float obstacleSectionDistance()
{
    return fabsf(get_distance() - oc_section_start_distance);
}

static void resetParkingExit()
{
    oc_parking_exit_state =
        OBSTACLE_PARKING_EXIT_ENABLED
            ? PARKING_EXIT_IDLE
            : PARKING_EXIT_DONE;
    oc_parking_exit_state_distance = get_distance();
    oc_parking_exit_run_start_distance = get_distance();
    oc_parking_exit_start_heading = get_angle();
    oc_parking_exit_start_left_mm = 0.0f;
    oc_parking_exit_start_right_mm = 0.0f;
    oc_parking_exit_steering = 0;
    oc_parking_exit_brake_start_ms = 0;
    oc_parking_exit_settle_start_ms = 0;
    oc_parking_exit_segment = 0;
    oc_parking_field_pose_initialized = false;
    oc_parking_rear_start_distance = get_distance();
    oc_parking_rear_start_range = -1.0f;
    oc_parking_rear_last_range = -1.0f;
    oc_parking_rear_tof_sequence = 0;
    oc_parking_rear_confirm_frames = 0;
    oc_parking_rear_sample_sum = 0.0f;
    oc_parking_rear_sample_min = 0.0f;
    oc_parking_rear_sample_max = 0.0f;
    oc_parking_rear_planned_travel = 0.0f;
    oc_parking_rear_cumulative_travel = 0.0f;
    oc_parking_rear_last_encoder_distance = get_distance();
    oc_parking_rear_direction = 0;
    oc_parking_rear_verifying = false;
    oc_parking_rear_discard_frames = 0;
    oc_parking_localization_start_distance = get_distance();
    oc_parking_localization_initial_piece_range = 0.0f;
    oc_parking_localization_last_piece_range = 0.0f;
    oc_parking_localization_wall_range = 0.0f;
    oc_parking_localization_tof_sequence = 0;
    oc_parking_localization_wall_frames = 0;
    oc_parking_localization_transition_found = false;
    oc_parking_localization_piece_seen = false;
}

static void initializeParkingFieldPose(float rearTofRangeMm)
{
    const int8_t turnSign =
        oc_parking_exit_steering > 0 ? -1 : 1;
    const float gapMm =
        OBSTACLE_PARKING_EXIT_PROTOTYPE_GAP_MM;
    const float rearReferenceX =
        turnSign > 0
            ? OBSTACLE_PARKING_FIXED_INNER_FACE_X_MM - gapMm
            : OBSTACLE_PARKING_FIXED_INNER_FACE_X_MM;
    const bool rearReferenceUsable =
        rearTofRangeMm > 0.0f &&
        rearTofRangeMm < TOF_OUT_OF_RANGE_MM;
    const float measuredRearClearance =
        rearReferenceUsable
            ? rearTofRangeMm -
                  OBSTACLE_PARKING_REAR_TOF_SENSOR_TO_BODY_MM
            : OBSTACLE_PARKING_EXIT_START_REAR_CLEARANCE_MM;
    const float startX =
        turnSign > 0
            ? rearReferenceX +
                OBSTACLE_PARKING_EXIT_PROTOTYPE_REAR_MM +
                measuredRearClearance
            : rearReferenceX -
                OBSTACLE_PARKING_EXIT_PROTOTYPE_REAR_MM -
                measuredRearClearance;
    const float nominalStartY =
        OBSTACLE_PARKING_OPEN_END_FIELD_Y_MM -
        OBSTACLE_WHEEL_OUTSIDE_WIDTH_MM * 0.5f;
    const TofSensor outerWallSensor =
        oc_parking_exit_steering > 0 ? TOF_LEFT : TOF_RIGHT;
    const float outerWallRange = get_tof_distance(outerWallSensor);
    const float outerWallSensorOffset =
        outerWallSensor == TOF_LEFT
            ? fabsf(OBSTACLE_TOF_LEFT_LOCAL_Y_MM)
            : fabsf(OBSTACLE_TOF_RIGHT_LOCAL_Y_MM);
    const bool outerWallReferenceUsable =
        outerWallRange > 0.0f &&
        outerWallRange <= OBSTACLE_PARKING_EXIT_TOF_REFERENCE_MAX_MM;
    const float startY =
        outerWallReferenceUsable
            ? OBSTACLE_SOUTH_OUTER_WALL_Y_MM +
                  outerWallRange + outerWallSensorOffset
            : nominalStartY;
    const float startHeading = turnSign > 0 ? 0.0f : 180.0f;

    position_reset(startX, startY, startHeading);
    oc_parking_field_pose_initialized = true;

    Serial.print("[PARK FIELD START] turn=");
    Serial.print(turnSign > 0 ? "CCW" : "CW");
    Serial.print(" fixed_line_x=");
    Serial.print(OBSTACLE_PARKING_FIXED_DOTTED_LINE_X_MM, 1);
    Serial.print(" rear_axle_x_y_heading=");
    Serial.print(startX, 1);
    Serial.print("/");
    Serial.print(startY, 1);
    Serial.print("/");
    Serial.print(startHeading, 1);
    Serial.print(" start_y_source=");
    Serial.print(outerWallReferenceUsable ? "outer_wall_tof" : "nominal");
    Serial.print(" range_mm=");
    Serial.print(outerWallRange, 1);
    Serial.print(" start_x_source=");
    Serial.print(rearReferenceUsable ? "rear_tof" : "nominal");
    Serial.print(" rear_range/clearance_mm=");
    Serial.print(rearTofRangeMm, 1);
    Serial.print("/");
    Serial.println(measuredRearClearance, 1);
}

static void printParkingExitGeometry()
{
    Serial.print("[PARK EXIT] Geometry width_mm=");
    Serial.print(OBSTACLE_PARKING_WIDTH_MM, 0);
    Serial.print(" robot_length_mm=");

    if (OBSTACLE_FINAL_ROBOT_LENGTH_MM > 0.0f)
    {
        Serial.print(OBSTACLE_FINAL_ROBOT_LENGTH_MM, 1);
        Serial.print(" parking_length_mm=");
        Serial.println(
            OBSTACLE_FINAL_ROBOT_LENGTH_MM *
                OBSTACLE_PARKING_LENGTH_FACTOR,
            1);
    }
    else
    {
        Serial.println(
            "UNSET parking_length=1.5*robot_length prototype_only=yes");
    }

    Serial.print("[PARK EXIT] Prototype footprint length/front/rear/width_mm=");
    Serial.print(OBSTACLE_PARKING_EXIT_PROTOTYPE_LENGTH_MM, 1);
    Serial.print("/");
    Serial.print(OBSTACLE_PARKING_EXIT_PROTOTYPE_FRONT_MM, 1);
    Serial.print("/");
    Serial.print(OBSTACLE_PARKING_EXIT_PROTOTYPE_REAR_MM, 1);
    Serial.print("/");
    Serial.print(OBSTACLE_PARKING_EXIT_PROTOTYPE_WIDTH_MM, 1);
    Serial.print(" gap_mm=");
    Serial.print(OBSTACLE_PARKING_EXIT_PROTOTYPE_GAP_MM, 1);
    Serial.print(" rear_clearance_mm=");
    Serial.print(OBSTACLE_PARKING_EXIT_START_REAR_CLEARANCE_MM, 1);
    Serial.print(" stage_segments=");
    Serial.print(OBSTACLE_PARKING_EXIT_TEST_SEGMENT_LIMIT);
    Serial.print("/");
    Serial.println(OBSTACLE_PARKING_EXIT_SEGMENT_COUNT);

    Serial.print("[PARK REAR] enabled=");
    Serial.print(
        OBSTACLE_PARKING_REAR_TOF_POSITIONING_ENABLED ? "yes" : "no");
    Serial.print(" sensor_behind_axle_mm=");
    Serial.print(OBSTACLE_REAR_TOF_BEHIND_AXLE_MM, 1);
    Serial.print(" target_range/clearance_mm=");
    Serial.print(OBSTACLE_PARKING_REAR_TOF_TARGET_RANGE_MM, 1);
    Serial.print("/");
    Serial.println(OBSTACLE_PARKING_REAR_TOF_TARGET_CLEARANCE_MM, 1);
}

static bool parkingRearRangeValid(float rangeMm)
{
    const float availableClearance =
        OBSTACLE_PARKING_EXIT_PROTOTYPE_GAP_MM -
        OBSTACLE_PARKING_EXIT_PROTOTYPE_LENGTH_MM;
    const float clearanceMm =
        rangeMm - OBSTACLE_PARKING_REAR_TOF_SENSOR_TO_BODY_MM;
    // Allow the documented +/-5 mm physical placement tolerance at the two
    // limits, but reject a return that cannot be the rear magenta piece.
    return rangeMm > 0.0f &&
           rangeMm < TOF_OUT_OF_RANGE_MM &&
           clearanceMm >= -5.0f &&
           clearanceMm <= availableClearance + 5.0f;
}

static void resetParkingRearSamples()
{
    oc_parking_rear_confirm_frames = 0;
    oc_parking_rear_sample_sum = 0.0f;
    oc_parking_rear_sample_min = 0.0f;
    oc_parking_rear_sample_max = 0.0f;
}

static bool addParkingRearSample(float rangeMm, float &averageMm)
{
    if (oc_parking_rear_confirm_frames == 0)
    {
        oc_parking_rear_confirm_frames = 1;
        oc_parking_rear_sample_sum = rangeMm;
        oc_parking_rear_sample_min = rangeMm;
        oc_parking_rear_sample_max = rangeMm;
        return false;
    }

    const float nextMin = fminf(oc_parking_rear_sample_min, rangeMm);
    const float nextMax = fmaxf(oc_parking_rear_sample_max, rangeMm);
    if (
        nextMax - nextMin >
        OBSTACLE_PARKING_REAR_TOF_SAMPLE_SPAN_MM)
    {
        // Restart the stationary window on a jump; never average two targets.
        resetParkingRearSamples();
        oc_parking_rear_confirm_frames = 1;
        oc_parking_rear_sample_sum = rangeMm;
        oc_parking_rear_sample_min = rangeMm;
        oc_parking_rear_sample_max = rangeMm;
        return false;
    }

    oc_parking_rear_sample_min = nextMin;
    oc_parking_rear_sample_max = nextMax;
    oc_parking_rear_sample_sum += rangeMm;
    ++oc_parking_rear_confirm_frames;
    if (
        oc_parking_rear_confirm_frames <
        OBSTACLE_PARKING_REAR_TOF_CONFIRM_FRAMES)
    {
        return false;
    }

    averageMm =
        oc_parking_rear_sample_sum /
        static_cast<float>(oc_parking_rear_confirm_frames);
    return true;
}

static void holdParkingRearPositioning(const char *reason)
{
    stop(false);
    set_steering(0);
    oc_parking_exit_state = PARKING_EXIT_TEST_HOLD;
    Serial.print("[PARK REAR] Failed reason=");
    Serial.print(reason);
    Serial.print(" range_mm=");
    Serial.print(oc_parking_rear_last_range, 1);
    Serial.print(" travel_mm=");
    Serial.println(oc_parking_rear_cumulative_travel, 1);
    TofDiagnosticSnapshot snapshot;
    if (get_tof_diagnostic_snapshot(TOF_REAR, snapshot))
    {
        Serial.print("[PARK REAR DIAG] sequence/filtered/raw_mm=");
        Serial.print(snapshot.sequence);
        Serial.print("/");
        Serial.print(snapshot.filtered_distance_mm, 1);
        Serial.print("/");
        Serial.print(snapshot.selected_raw_distance_mm, 1);
        Serial.print(" signal_mcps=");
        Serial.print(snapshot.selected_signal_mcps, 3);
        Serial.print(" sigma_mm=");
        Serial.println(snapshot.selected_sigma_mm, 1);
    }
    Serial.println("[PARK REAR] Drive motor locked off");
    robot_logger.write_to_usb();
}

static void beginParkingExitSegments(float rearRangeMm)
{
    oc_parking_exit_state_distance = get_distance();
    oc_parking_exit_run_start_distance = get_distance();
    oc_parking_exit_start_heading = get_angle();
    initializeParkingFieldPose(rearRangeMm);
    oc_parking_exit_segment = 0;
    oc_parking_exit_settle_start_ms = millis();
    oc_parking_exit_state = PARKING_EXIT_SEGMENT_SETTLE;

    Serial.print("[PARK REAR RESULT] start/final_range_mm=");
    Serial.print(oc_parking_rear_start_range, 1);
    Serial.print("/");
    Serial.print(rearRangeMm, 1);
    Serial.print(" start/final_clearance_mm=");
    Serial.print(
        oc_parking_rear_start_range -
            OBSTACLE_PARKING_REAR_TOF_SENSOR_TO_BODY_MM,
        1);
    Serial.print("/");
    Serial.print(
        rearRangeMm - OBSTACLE_PARKING_REAR_TOF_SENSOR_TO_BODY_MM,
        1);
    Serial.print(" correction_travel_mm=");
    Serial.println(oc_parking_rear_cumulative_travel, 1);

    if (OBSTACLE_PARKING_REAR_TOF_POSITIONING_TEST_ONLY)
    {
        stop(false);
        set_steering(0);
        oc_parking_exit_state = PARKING_EXIT_TEST_HOLD;
        Serial.println(
            "[PARK REAR] Positioning test complete - drive motor locked off");
        robot_logger.write_to_usb();
        return;
    }
}

struct ParkingBeamFootprint
{
    float centerX;
    float minimumX;
    float maximumX;
};

static ParkingBeamFootprint parkingBeamFootprint(
    TofSensor sensor,
    float rangeMm,
    const PositionEstimate &pose)
{
    const float localX =
        sensor == TOF_LEFT
            ? OBSTACLE_TOF_LEFT_LOCAL_X_MM
            : OBSTACLE_TOF_RIGHT_LOCAL_X_MM;
    const float localY =
        sensor == TOF_LEFT
            ? OBSTACLE_TOF_LEFT_LOCAL_Y_MM
            : OBSTACLE_TOF_RIGHT_LOCAL_Y_MM;
    const float headingRad = pose.heading_deg * PI / 180.0f;
    const float sensorX =
        pose.x_mm + cosf(headingRad) * localX -
        sinf(headingRad) * localY;
    const float rayHeadingRad =
        headingRad + (sensor == TOF_LEFT ? 0.5f * PI : -0.5f * PI);
    const float halfFovRad =
        0.5f * OBSTACLE_PARKING_EXIT_TOF_DETECTION_FOV_DEG *
        PI / 180.0f;
    const float centerX =
        sensorX + rangeMm * cosf(rayHeadingRad);
    const float edgeX1 =
        sensorX + rangeMm * cosf(rayHeadingRad - halfFovRad);
    const float edgeX2 =
        sensorX + rangeMm * cosf(rayHeadingRad + halfFovRad);
    return {
        centerX,
        fminf(edgeX1, edgeX2),
        fmaxf(edgeX1, edgeX2)};
}

static float expectedParkingOuterWallRange(
    TofSensor sensor,
    const PositionEstimate &pose)
{
    const float localX =
        sensor == TOF_LEFT
            ? OBSTACLE_TOF_LEFT_LOCAL_X_MM
            : OBSTACLE_TOF_RIGHT_LOCAL_X_MM;
    const float localY =
        sensor == TOF_LEFT
            ? OBSTACLE_TOF_LEFT_LOCAL_Y_MM
            : OBSTACLE_TOF_RIGHT_LOCAL_Y_MM;
    const float headingRad = pose.heading_deg * PI / 180.0f;
    const float sensorY =
        pose.y_mm + sinf(headingRad) * localX +
        cosf(headingRad) * localY;
    const float rayHeadingRad =
        headingRad + (sensor == TOF_LEFT ? 0.5f * PI : -0.5f * PI);
    const float rayY = sinf(rayHeadingRad);
    if (fabsf(rayY) < 0.1f)
        return -1.0f;

    return (OBSTACLE_SOUTH_OUTER_WALL_Y_MM - sensorY) / rayY;
}

static void completeParkingExit(bool stagedTest)
{
    if (stagedTest ||
        (OBSTACLE_PARKING_EXIT_TEST_ONLY &&
         !OBSTACLE_PARKING_ENTRY_DISCOVERY_ENABLED))
    {
        oc_parking_exit_state = PARKING_EXIT_TEST_HOLD;
        Serial.println(
            stagedTest
                ? "[PARK EXIT] Staged test complete - drive motor locked off"
                : "[PARK EXIT] Test complete - drive motor locked off");
        robot_logger.write_to_usb();
        return;
    }

    oc_parking_exit_state = PARKING_EXIT_DONE;
    if (!OBSTACLE_PARKING_ENTRY_DISCOVERY_ENABLED)
        navigation_enable();
    oc_section_start_distance = get_distance();
    oc_last_navigation_state = navigation_get_state();
    oc_last_completed_turn = navigation_get_turn_count();

    Serial.println(
        OBSTACLE_PARKING_ENTRY_DISCOVERY_ENABLED
            ? "[PARK EXIT] Complete - parking-entry discovery next"
            : "[PARK EXIT] Complete - normal Obstacle navigation");
}

static void finishParkingExit(bool stagedTest)
{
    const float finalHeadingError =
        fabsf(get_angle() - oc_parking_exit_start_heading);
    const float finalLeft = get_tof_distance(TOF_LEFT);
    const float finalRight = get_tof_distance(TOF_RIGHT);
    const float totalDistance =
        distanceSince(oc_parking_exit_run_start_distance);

    stop(false);
    set_steering(0);

    Serial.print("[PARK EXIT RESULT] completed_segments=");
    Serial.print(oc_parking_exit_segment);
    Serial.print("/");
    Serial.print(OBSTACLE_PARKING_EXIT_SEGMENT_COUNT);
    Serial.print(" travel_mm=");
    Serial.print(totalDistance, 1);
    Serial.print(" heading_error_deg=");
    Serial.print(finalHeadingError, 1);
    Serial.print(" tof_start_left_right_mm=");
    Serial.print(oc_parking_exit_start_left_mm, 0);
    Serial.print("/");
    Serial.print(oc_parking_exit_start_right_mm, 0);
    Serial.print(" tof_end_left_right_mm=");
    Serial.print(finalLeft, 0);
    Serial.print("/");
    Serial.println(finalRight, 0);

    // At the aligned exit pose, the outer-wall-side sensor is expected to sit
    // over one magenta piece and face its exact 200 mm open end. Accept the
    // resulting field reference only when the odometry-derived beam position
    // is over that particular 20 mm piece.
    const TofSensor referenceSensor =
        oc_parking_exit_steering > 0
            ? TOF_LEFT
            : TOF_RIGHT;
    const float referenceRange =
        get_tof_distance(referenceSensor);
    const float signedHeadingDelta =
        get_angle() - oc_parking_exit_start_heading;
    const float headingRad = signedHeadingDelta * PI / 180.0f;
    const float awayAxisSign =
        oc_parking_exit_steering < 0 ? 1.0f : -1.0f;
    const float sensorLocalX =
        referenceSensor == TOF_LEFT
            ? OBSTACLE_TOF_LEFT_LOCAL_X_MM
            : OBSTACLE_TOF_RIGHT_LOCAL_X_MM;
    const float sensorWallOffset =
        referenceSensor == TOF_LEFT
            ? fabsf(OBSTACLE_TOF_LEFT_LOCAL_Y_MM)
            : fabsf(OBSTACLE_TOF_RIGHT_LOCAL_Y_MM);
    const bool headingUsable =
        finalHeadingError <=
            OBSTACLE_PARKING_EXIT_FINAL_HEADING_TOLERANCE_DEG;
    const bool pieceRangeUsable =
        referenceRange > 0.0f &&
        referenceRange <=
            OBSTACLE_PARKING_EXIT_TOF_REFERENCE_MAX_MM;
    const float rearBeyondParkingEnd =
        (referenceRange + sensorWallOffset) * cosf(headingRad) -
        awayAxisSign * sensorLocalX * sinf(headingRad);
    const float remainingToCenterline =
        OBSTACLE_CORRIDOR_HALF_WIDTH_MM -
        OBSTACLE_PARKING_WIDTH_MM -
        rearBeyondParkingEnd;

    PositionEstimate fieldPose = get_position_struct();
    const float sensorLocalY =
        referenceSensor == TOF_LEFT
            ? OBSTACLE_TOF_LEFT_LOCAL_Y_MM
            : OBSTACLE_TOF_RIGHT_LOCAL_Y_MM;
    const float fieldHeadingRad = fieldPose.heading_deg * PI / 180.0f;
    const float sensorFieldX =
        fieldPose.x_mm + cosf(fieldHeadingRad) * sensorLocalX -
        sinf(fieldHeadingRad) * sensorLocalY;
    const float sensorRayHeadingRad =
        fieldHeadingRad +
        (referenceSensor == TOF_LEFT ? 0.5f * PI : -0.5f * PI);
    const float halfDetectionFovRad =
        0.5f * OBSTACLE_PARKING_EXIT_TOF_DETECTION_FOV_DEG *
        PI / 180.0f;
    const float beamCenterFieldX =
        sensorFieldX + referenceRange * cosf(sensorRayHeadingRad);
    const float beamEdgeFieldX1 =
        sensorFieldX + referenceRange *
            cosf(sensorRayHeadingRad - halfDetectionFovRad);
    const float beamEdgeFieldX2 =
        sensorFieldX + referenceRange *
            cosf(sensorRayHeadingRad + halfDetectionFovRad);
    const float beamFootprintMinX =
        fminf(beamEdgeFieldX1, beamEdgeFieldX2);
    const float beamFootprintMaxX =
        fmaxf(beamEdgeFieldX1, beamEdgeFieldX2);
    const bool counterClockwiseExit = oc_parking_exit_steering < 0;
    const float expectedPieceMaxX =
        counterClockwiseExit
            ? OBSTACLE_PARKING_FIXED_DOTTED_LINE_X_MM
            : OBSTACLE_PARKING_FIXED_INNER_FACE_X_MM -
                  OBSTACLE_PARKING_EXIT_PROTOTYPE_GAP_MM;
    const float expectedPieceMinX =
        expectedPieceMaxX - OBSTACLE_PARKING_LIMIT_THICKNESS_MM;
    const bool beamOverExpectedPiece =
        beamFootprintMaxX >= expectedPieceMinX -
                                 OBSTACLE_PARKING_EXIT_BEAM_X_TOLERANCE_MM &&
        beamFootprintMinX <= expectedPieceMaxX +
                                 OBSTACLE_PARKING_EXIT_BEAM_X_TOLERANCE_MM;
    const bool referenceUsable =
        headingUsable && pieceRangeUsable && beamOverExpectedPiece;
    // Multi-object ToF returns at the end of the thin parking piece can jump
    // between the piece, an oblique edge, and the outer wall. A geometrically
    // valid intermediate return may seed the bounded edge search, but it must
    // not be used as a piece measurement or field-y correction.
    const bool localizationSeedUsable =
        headingUsable && referenceRange > 0.0f &&
        referenceRange <= OBSTACLE_PARKING_EXIT_WALL_REFERENCE_MAX_MM &&
        beamOverExpectedPiece;
    const float tofFieldY =
        OBSTACLE_PARKING_OPEN_END_FIELD_Y_MM +
        rearBeyondParkingEnd;
    const float tofCorrectionY = tofFieldY - fieldPose.y_mm;
    const bool applyFieldY =
        referenceUsable && oc_parking_field_pose_initialized &&
        fabsf(tofCorrectionY) <=
            OBSTACLE_PARKING_EXIT_MAX_Y_CORRECTION_MM;
    if (applyFieldY)
    {
        position_apply_xy_correction(0.0f, tofCorrectionY);
        fieldPose = get_position_struct();
    }

    Serial.print("[PARK EXIT TOF REF] sensor=");
    Serial.print(referenceSensor == TOF_LEFT ? "L" : "R");
    Serial.print(" filtered/raw_mm=");
    Serial.print(referenceRange, 1);
    Serial.print("/");
    Serial.print(get_tof_raw_distance(referenceSensor), 1);
    Serial.print(" signal_mcps=");
    Serial.print(get_tof_signal_rate(referenceSensor), 2);
    Serial.print(" sigma_mm=");
    Serial.print(get_tof_sigma(referenceSensor), 1);
    Serial.print(" rear_beyond_parking_end_mm=");
    Serial.print(rearBeyondParkingEnd, 1);
    Serial.print(" remaining_to_centerline_mm=");
    Serial.print(remainingToCenterline, 1);
    Serial.print(" beam_x_mm=");
    Serial.print(beamCenterFieldX, 1);
    Serial.print(" beam_footprint_x_mm=");
    Serial.print(beamFootprintMinX, 1);
    Serial.print("..");
    Serial.print(beamFootprintMaxX, 1);
    Serial.print(" expected_piece_x_mm=");
    Serial.print(expectedPieceMinX, 1);
    Serial.print("..");
    Serial.print(expectedPieceMaxX, 1);
    Serial.print(" beam_over_piece=");
    Serial.print(beamOverExpectedPiece ? "yes" : "no");
    Serial.print(" usable=");
    Serial.print(referenceUsable ? "yes" : "no");
    Serial.print(" localization_seed=");
    Serial.print(localizationSeedUsable ? "yes" : "no");
    Serial.print(" apply_y=");
    Serial.println(applyFieldY ? "yes" : "no");

    Serial.print("[PARK EXIT FIELD POSE] x_y_heading=");
    Serial.print(fieldPose.x_mm, 1);
    Serial.print("/");
    Serial.print(fieldPose.y_mm, 1);
    Serial.print("/");
    Serial.print(fieldPose.heading_deg, 1);
    Serial.print(" tof_y_correction_mm=");
    Serial.println(applyFieldY ? tofCorrectionY : 0.0f, 1);

    if (OBSTACLE_PARKING_REAR_TOF_EXIT_TEST_ONLY)
    {
        stop(false);
        set_steering(0);
        oc_parking_exit_state = PARKING_EXIT_TEST_HOLD;
        Serial.println(
            "[PARK EXIT] Rear-positioned exit test complete - "
            "drive motor locked off");
        robot_logger.write_to_usb();
        return;
    }

    if (
        !stagedTest &&
        OBSTACLE_PARKING_EXIT_EDGE_LOCALIZATION_ENABLED &&
        localizationSeedUsable)
    {
        oc_parking_localization_sensor = referenceSensor;
        oc_parking_localization_initial_piece_range =
            referenceUsable
                ? referenceRange
                : OBSTACLE_PARKING_EXIT_TOF_REFERENCE_MAX_MM;
        oc_parking_localization_last_piece_range =
            referenceUsable ? referenceRange : 0.0f;
        oc_parking_localization_last_piece_pose = fieldPose;
        oc_parking_localization_wall_range = 0.0f;
        oc_parking_localization_wall_frames = 0;
        oc_parking_localization_transition_found = false;
        oc_parking_localization_piece_seen = referenceUsable;
        TofDiagnosticSnapshot snapshot;
        oc_parking_localization_tof_sequence =
            get_tof_diagnostic_snapshot(referenceSensor, snapshot)
                ? snapshot.sequence
                : 0;
        oc_parking_exit_settle_start_ms = millis();
        oc_parking_exit_state = PARKING_EXIT_LOCALIZE_SETTLE;
        Serial.print("[PARK LOCALIZE] Armed sensor=");
        Serial.print(referenceSensor == TOF_LEFT ? "L" : "R");
        Serial.print(" piece_range_mm=");
        Serial.print(
            referenceUsable ? referenceRange : 0.0f,
            1);
        Serial.print(" direction=");
        Serial.print(
            OBSTACLE_PARKING_EXIT_EDGE_LOCALIZATION_DIRECTION < 0
                ? "reverse"
                : "forward");
        Serial.print(" max_creep_mm=");
        Serial.println(OBSTACLE_PARKING_EXIT_EDGE_LOCALIZATION_MAX_MM, 1);
        return;
    }

    if (!stagedTest && OBSTACLE_PARKING_ENTRY_DISCOVERY_ENABLED)
    {
        stop(false);
        oc_parking_exit_state = PARKING_EXIT_TEST_HOLD;
        Serial.println(
            "[PARK ENTRY] Not armed - no usable parking-end reference; "
            "drive motor locked off");
        robot_logger.write_to_usb();
        return;
    }

    completeParkingExit(stagedTest);
}

static void finishParkingEdgeLocalization()
{
    const float creepDistance =
        distanceSince(oc_parking_localization_start_distance);
    float xCorrection = 0.0f;
    float yCorrection = 0.0f;
    float predictedEdgeX = 0.0f;
    float knownEdgeX = 0.0f;
    float expectedWallRange = 0.0f;
    bool applyX = false;
    bool applyY = false;

    if (oc_parking_localization_transition_found)
    {
        const PositionEstimate transitionPose =
            oc_parking_localization_last_piece_pose;

        const ParkingBeamFootprint footprint = parkingBeamFootprint(
            oc_parking_localization_sensor,
            oc_parking_localization_last_piece_range,
            transitionPose);
        const bool counterClockwiseExit = oc_parking_exit_steering < 0;
        const bool reverseSearch =
            OBSTACLE_PARKING_EXIT_EDGE_LOCALIZATION_DIRECTION < 0;
        if (counterClockwiseExit && !reverseSearch)
        {
            predictedEdgeX = footprint.minimumX;
            knownEdgeX = OBSTACLE_PARKING_FIXED_DOTTED_LINE_X_MM;
        }
        else if (!counterClockwiseExit && !reverseSearch)
        {
            predictedEdgeX = footprint.maximumX;
            knownEdgeX =
                OBSTACLE_PARKING_FIXED_INNER_FACE_X_MM -
                OBSTACLE_PARKING_EXIT_PROTOTYPE_GAP_MM -
                OBSTACLE_PARKING_LIMIT_THICKNESS_MM;
        }
        else if (counterClockwiseExit)
        {
            predictedEdgeX = footprint.maximumX;
            knownEdgeX = OBSTACLE_PARKING_FIXED_INNER_FACE_X_MM;
        }
        else
        {
            predictedEdgeX = footprint.minimumX;
            knownEdgeX =
                OBSTACLE_PARKING_FIXED_INNER_FACE_X_MM -
                OBSTACLE_PARKING_EXIT_PROTOTYPE_GAP_MM;
        }
        xCorrection = knownEdgeX - predictedEdgeX;
        applyX =
            oc_parking_localization_piece_seen &&
            fabsf(xCorrection) <=
            OBSTACLE_PARKING_EXIT_MAX_X_CORRECTION_MM;

        const PositionEstimate wallPose =
            oc_parking_localization_latest_wall_pose;
        expectedWallRange = expectedParkingOuterWallRange(
            oc_parking_localization_sensor, wallPose);
        const float headingRad = wallPose.heading_deg * PI / 180.0f;
        const float sensorLocalX =
            oc_parking_localization_sensor == TOF_LEFT
                ? OBSTACLE_TOF_LEFT_LOCAL_X_MM
                : OBSTACLE_TOF_RIGHT_LOCAL_X_MM;
        const float sensorLocalY =
            oc_parking_localization_sensor == TOF_LEFT
                ? OBSTACLE_TOF_LEFT_LOCAL_Y_MM
                : OBSTACLE_TOF_RIGHT_LOCAL_Y_MM;
        const float sensorY =
            wallPose.y_mm + sinf(headingRad) * sensorLocalX +
            cosf(headingRad) * sensorLocalY;
        const float rayHeadingRad =
            headingRad +
            (oc_parking_localization_sensor == TOF_LEFT
                 ? 0.5f * PI
                 : -0.5f * PI);
        const float predictedWallY =
            sensorY + oc_parking_localization_wall_range *
                          sinf(rayHeadingRad);
        yCorrection =
            OBSTACLE_SOUTH_OUTER_WALL_Y_MM - predictedWallY;
        applyY =
            fabsf(yCorrection) <=
            OBSTACLE_PARKING_EXIT_MAX_Y_CORRECTION_MM;

        position_apply_xy_correction(
            applyX ? xCorrection : 0.0f,
            applyY ? yCorrection : 0.0f);
    }

    const PositionEstimate finalPose = get_position_struct();
    Serial.print("[PARK LOCALIZE RESULT] transition=");
    Serial.print(
        oc_parking_localization_transition_found ? "yes" : "no_max_distance");
    Serial.print(" creep_mm=");
    Serial.print(creepDistance, 1);
    Serial.print(" piece_range_mm=");
    Serial.print(oc_parking_localization_last_piece_range, 1);
    Serial.print(" piece_seen=");
    Serial.print(oc_parking_localization_piece_seen ? "yes" : "no");
    Serial.print(" wall_range_mm=");
    Serial.print(oc_parking_localization_wall_range, 1);
    Serial.print(" expected_wall_range_mm=");
    Serial.print(expectedWallRange, 1);
    Serial.print(" wall_residual_mm=");
    Serial.print(
        oc_parking_localization_wall_range - expectedWallRange,
        1);
    Serial.print(" predicted/known_edge_x_mm=");
    Serial.print(predictedEdgeX, 1);
    Serial.print("/");
    Serial.print(knownEdgeX, 1);
    Serial.print(" x_correction_mm=");
    Serial.print(xCorrection, 1);
    Serial.print(" apply_x=");
    Serial.print(applyX ? "yes" : "no");
    Serial.print(" wall_y_correction_mm=");
    Serial.print(yCorrection, 1);
    Serial.print(" apply_y=");
    Serial.println(applyY ? "yes" : "no");

    Serial.print("[PARK LOCALIZE POSE] x_y_heading=");
    Serial.print(finalPose.x_mm, 1);
    Serial.print("/");
    Serial.print(finalPose.y_mm, 1);
    Serial.print("/");
    Serial.println(finalPose.heading_deg, 1);

    if (OBSTACLE_PARKING_ENTRY_DISCOVERY_ENABLED &&
        (!oc_parking_localization_transition_found || !applyX || !applyY))
    {
        stop(false);
        oc_parking_exit_state = PARKING_EXIT_TEST_HOLD;
        Serial.println(
            "[PARK ENTRY] Not armed - parking localization invalid; "
            "drive motor locked off");
        robot_logger.write_to_usb();
        return;
    }

    completeParkingExit(false);
}

static void processParkingEdgeLocalizationTof()
{
    TofDiagnosticSnapshot snapshot;
    if (
        !get_tof_diagnostic_snapshot(
            oc_parking_localization_sensor, snapshot) ||
        snapshot.sequence == oc_parking_localization_tof_sequence)
    {
        return;
    }

    oc_parking_localization_tof_sequence = snapshot.sequence;
    const float rawRange = snapshot.selected_raw_distance_mm;
    const PositionEstimate samplePose = get_position_struct();
    const float markerRangeMaximum = fminf(
        OBSTACLE_PARKING_EXIT_TOF_REFERENCE_MAX_MM,
        oc_parking_localization_initial_piece_range +
            OBSTACLE_PARKING_EXIT_MARKER_RANGE_MARGIN_MM);
    const float expectedWallRange = expectedParkingOuterWallRange(
        oc_parking_localization_sensor, samplePose);
    const bool wallRangeConsistent =
        rawRange >= OBSTACLE_PARKING_EXIT_WALL_REFERENCE_MIN_MM &&
        rawRange <= OBSTACLE_PARKING_EXIT_WALL_REFERENCE_MAX_MM &&
        expectedWallRange > 0.0f &&
        fabsf(rawRange - expectedWallRange) <=
            OBSTACLE_PARKING_EXIT_WALL_RANGE_RESIDUAL_MM;
    if (rawRange > 0.0f && rawRange <= markerRangeMaximum)
    {
        oc_parking_localization_piece_seen = true;
        oc_parking_localization_last_piece_range = rawRange;
        oc_parking_localization_last_piece_pose = samplePose;
        oc_parking_localization_wall_frames = 0;
    }
    else if (wallRangeConsistent)
    {
        // Confirm a sequence, then localize from its newest sample. The first
        // in-range return can still contain the fading edge of the magenta
        // piece even though it passes the residual gate.
        oc_parking_localization_latest_wall_pose = samplePose;
        oc_parking_localization_wall_range = rawRange;
        ++oc_parking_localization_wall_frames;
    }
    else
    {
        // Intermediate oblique returns are neither the short magenta end nor
        // a wall range consistent with the current pose.
        oc_parking_localization_wall_frames = 0;
    }

    oc_parking_localization_transition_found =
        oc_parking_localization_wall_frames >=
        OBSTACLE_PARKING_EXIT_WALL_CONFIRM_FRAMES;
}

static bool updateParkingExit()
{
    if (oc_parking_exit_state == PARKING_EXIT_DONE)
        return false;

    if (oc_parking_exit_state == PARKING_EXIT_TEST_HOLD)
    {
        // Keep the drive motor de-energized until the enable switch is turned
        // off. This prevents the remaining Obstacle Challenge from starting.
        if (dc_state != DC_DISABLED)
            stop(false);
        return true;
    }

    if (oc_parking_exit_state == PARKING_EXIT_REAR_SETTLE)
    {
        if (dc_state != DC_DISABLED)
            stop(false);
        servo_disabled = false;
        set_steering(0);
        steer(0);

        const uint32_t requiredSettleMs =
            oc_parking_rear_verifying
                ? OBSTACLE_PARKING_REAR_TOF_POST_MOVE_SETTLE_MS
                : OBSTACLE_PARKING_REAR_TOF_SETTLE_MS;
        if (
            millis() - oc_parking_exit_settle_start_ms <
                requiredSettleMs)
        {
            return true;
        }

        TofDiagnosticSnapshot snapshot;
        if (
            get_tof_diagnostic_snapshot(TOF_REAR, snapshot) &&
            snapshot.sequence != oc_parking_rear_tof_sequence)
        {
            oc_parking_rear_tof_sequence = snapshot.sequence;
            const float range = snapshot.filtered_distance_mm;
            if (parkingRearRangeValid(range))
            {
                oc_parking_rear_last_range = range;
                if (
                    oc_parking_rear_verifying &&
                    oc_parking_rear_discard_frames > 0)
                {
                    --oc_parking_rear_discard_frames;
                    return true;
                }
                float averageRange = 0.0f;
                if (!addParkingRearSample(range, averageRange))
                {
                    if (
                        millis() - oc_parking_exit_settle_start_ms >=
                        requiredSettleMs +
                            OBSTACLE_PARKING_REAR_TOF_TIMEOUT_MS)
                    {
                        holdParkingRearPositioning(
                            "unstable_stationary_range");
                    }
                    return true;
                }

                if (!oc_parking_rear_verifying)
                {
                    oc_parking_rear_start_range = averageRange;
                    const float signedCorrection =
                        OBSTACLE_PARKING_REAR_TOF_TARGET_RANGE_MM -
                        averageRange;
                    oc_parking_rear_planned_travel =
                        fabsf(signedCorrection);

                    if (
                        oc_parking_rear_planned_travel <=
                        OBSTACLE_PARKING_REAR_TOF_TOLERANCE_MM)
                    {
                        beginParkingExitSegments(averageRange);
                        return true;
                    }
                    if (
                        oc_parking_rear_planned_travel >
                        OBSTACLE_PARKING_REAR_TOF_MAX_TRAVEL_MM)
                    {
                        holdParkingRearPositioning(
                            "required_correction_too_large");
                        return true;
                    }

                    oc_parking_rear_direction =
                        signedCorrection > 0.0f ? 1 : -1;
                    oc_parking_rear_start_distance = get_distance();
                    oc_parking_rear_last_encoder_distance = get_distance();
                    oc_parking_rear_cumulative_travel = 0.0f;
                    oc_parking_exit_state = PARKING_EXIT_REAR_DRIVE;
                    Serial.print(
                        "[PARK REAR] One-shot correction start_range_mm=");
                    Serial.print(averageRange, 1);
                    Serial.print(" direction=");
                    Serial.print(
                        oc_parking_rear_direction > 0
                            ? "forward"
                            : "reverse");
                    Serial.print(" planned_mm=");
                    Serial.println(oc_parking_rear_planned_travel, 1);
                    return true;
                }

                const float measuredRangeChange =
                    averageRange - oc_parking_rear_start_range;
                const float expectedRangeChange =
                    oc_parking_rear_direction *
                    oc_parking_rear_cumulative_travel;
                const float targetError = fabsf(
                    averageRange -
                    OBSTACLE_PARKING_REAR_TOF_TARGET_RANGE_MM);
                const float motionAgreementError = fabsf(
                    measuredRangeChange - expectedRangeChange);
                const bool accepted =
                    targetError <=
                        OBSTACLE_PARKING_REAR_TOF_FINAL_TOLERANCE_MM &&
                    motionAgreementError <=
                        OBSTACLE_PARKING_REAR_TOF_MOTION_AGREEMENT_MM;

                Serial.print("[PARK REAR VERIFY] final_range_mm=");
                Serial.print(averageRange, 1);
                Serial.print(" target_error_mm=");
                Serial.print(targetError, 1);
                Serial.print(" measured/expected_change_mm=");
                Serial.print(measuredRangeChange, 1);
                Serial.print("/");
                Serial.print(expectedRangeChange, 1);
                Serial.print(" accepted=");
                Serial.println(accepted ? "yes" : "no");

                if (!accepted)
                {
                    holdParkingRearPositioning(
                        "stationary_verification_failed");
                    return true;
                }
                beginParkingExitSegments(averageRange);
                return true;
            }
        }

        if (
            millis() - oc_parking_exit_settle_start_ms >=
            requiredSettleMs +
                OBSTACLE_PARKING_REAR_TOF_TIMEOUT_MS)
        {
            holdParkingRearPositioning("no_valid_rear_range");
        }
        return true;
    }

    if (oc_parking_exit_state == PARKING_EXIT_REAR_DRIVE)
    {
        const float encoderDistance = get_distance();
        oc_parking_rear_cumulative_travel += fabsf(
            encoderDistance - oc_parking_rear_last_encoder_distance);
        oc_parking_rear_last_encoder_distance = encoderDistance;
        set_steering(0);

        if (
            oc_parking_rear_cumulative_travel >=
            oc_parking_rear_planned_travel)
        {
            stop(true);
            oc_parking_exit_brake_start_ms = millis();
            oc_parking_exit_state = PARKING_EXIT_REAR_BRAKE;
            return true;
        }
        if (
            oc_parking_rear_cumulative_travel >=
            OBSTACLE_PARKING_REAR_TOF_MAX_TRAVEL_MM)
        {
            holdParkingRearPositioning("cumulative_travel_limit");
            return true;
        }

        set_speed(
            oc_parking_rear_direction *
            OBSTACLE_PARKING_REAR_TOF_SPEED_MM_S);
        return true;
    }

    if (oc_parking_exit_state == PARKING_EXIT_REAR_BRAKE)
    {
        if (
            millis() - oc_parking_exit_brake_start_ms <
                OBSTACLE_PARKING_EXIT_HOLD_BRAKE_MS)
        {
            return true;
        }

        stop(false);
        set_steering(0);
        resetParkingRearSamples();
        oc_parking_rear_verifying = true;
        oc_parking_rear_discard_frames =
            OBSTACLE_PARKING_REAR_TOF_POST_MOVE_DISCARD_FRAMES;
        oc_parking_exit_settle_start_ms = millis();
        TofDiagnosticSnapshot snapshot;
        oc_parking_rear_tof_sequence =
            get_tof_diagnostic_snapshot(TOF_REAR, snapshot)
                ? snapshot.sequence
                : oc_parking_rear_tof_sequence;
        oc_parking_exit_state = PARKING_EXIT_REAR_SETTLE;
        return true;
    }

    if (oc_parking_exit_state == PARKING_EXIT_LOCALIZE_SETTLE)
    {
        if (dc_state != DC_DISABLED)
            stop(false);
        servo_disabled = false;
        set_steering(0);
        steer(0);

        // Consume fresh stationary samples here as well as while driving. An
        // intermediate edge return at exit completion must not prevent a
        // subsequent short piece return from seeding localization.
        processParkingEdgeLocalizationTof();

        if (
            millis() - oc_parking_exit_settle_start_ms <
                OBSTACLE_PARKING_EXIT_EDGE_LOCALIZATION_SETTLE_MS)
        {
            return true;
        }

        oc_parking_localization_start_distance = get_distance();
        oc_parking_exit_state = PARKING_EXIT_LOCALIZE_DRIVE;
        Serial.println("[PARK LOCALIZE] Straight edge search started");
        return true;
    }

    if (oc_parking_exit_state == PARKING_EXIT_LOCALIZE_DRIVE)
    {
        set_speed(
            OBSTACLE_PARKING_EXIT_EDGE_LOCALIZATION_DIRECTION *
            OBSTACLE_PARKING_EXIT_EDGE_LOCALIZATION_SPEED);
        set_steering(0);

        processParkingEdgeLocalizationTof();

        const float creepDistance =
            distanceSince(oc_parking_localization_start_distance);
        const bool distanceLimitReached =
            creepDistance >=
            OBSTACLE_PARKING_EXIT_EDGE_LOCALIZATION_MAX_MM;
        if (
            oc_parking_localization_transition_found ||
            distanceLimitReached)
        {
            stop(true);
            oc_parking_exit_brake_start_ms = millis();
            oc_parking_exit_state = PARKING_EXIT_LOCALIZE_BRAKE;
        }
        return true;
    }

    if (oc_parking_exit_state == PARKING_EXIT_LOCALIZE_BRAKE)
    {
        // The finite ToF footprint can keep returning the parking piece until
        // the chassis has stopped and settled. Continue accepting fresh frames
        // during the existing brake hold, without extending creep distance.
        processParkingEdgeLocalizationTof();
        if (
            millis() - oc_parking_exit_brake_start_ms <
                OBSTACLE_PARKING_EXIT_HOLD_BRAKE_MS)
        {
            return true;
        }

        stop(false);
        finishParkingEdgeLocalization();
        return true;
    }

    if (oc_parking_exit_state == PARKING_EXIT_SEGMENT_BRAKE)
    {
        if (
            millis() - oc_parking_exit_brake_start_ms <
                OBSTACLE_PARKING_EXIT_HOLD_BRAKE_MS)
        {
            return true;
        }

        const float actualDistance =
            distanceSince(oc_parking_exit_state_distance);
        const ParkingExitSegment &completed =
            PARKING_EXIT_SEGMENTS[oc_parking_exit_segment];

        stop(false);
        Serial.print("[PARK EXIT] Segment ");
        Serial.print(oc_parking_exit_segment + 1);
        Serial.print(" stopped target_mm=");
        Serial.print(completed.distanceMm, 1);
        Serial.print(" actual_mm=");
        Serial.print(actualDistance, 1);
        Serial.print(" heading_delta_deg=");
        Serial.println(
            get_angle() - oc_parking_exit_start_heading,
            1);

        ++oc_parking_exit_segment;
        if (
            oc_parking_exit_segment >=
                OBSTACLE_PARKING_EXIT_TEST_SEGMENT_LIMIT)
        {
            finishParkingExit(
                oc_parking_exit_segment <
                    OBSTACLE_PARKING_EXIT_SEGMENT_COUNT);
            return true;
        }

        oc_parking_exit_settle_start_ms = millis();
        oc_parking_exit_state = PARKING_EXIT_SEGMENT_SETTLE;
        return true;
    }

    if (oc_parking_exit_state == PARKING_EXIT_IDLE)
    {
        const float left = get_tof_distance(TOF_LEFT);
        const float right = get_tof_distance(TOF_RIGHT);

        // The parking lot is against the outer wall. Steer away from the
        // clearly nearer side wall. Positive steering is right.
        if (
            left > 0.0f &&
            right > 0.0f &&
            fabsf(left - right) >=
                OBSTACLE_PARKING_EXIT_MIN_WALL_DIFFERENCE_MM)
        {
            oc_parking_exit_steering =
                left < right
                    ? OBSTACLE_PARKING_EXIT_STEERING
                    : -OBSTACLE_PARKING_EXIT_STEERING;
        }
        else
        {
            // Do not guess the driving direction if the side sensors do not
            // identify the outer wall reliably.
            set_speed(0);
            set_steering(0);
            return true;
        }

        oc_parking_exit_start_left_mm = left;
        oc_parking_exit_start_right_mm = right;

        Serial.print("[PARK EXIT] Start wall left=");
        Serial.print(left, 0);
        Serial.print(" right=");
        Serial.println(right, 0);
        Serial.print("[PARK EXIT] Away steering=");
        Serial.println(oc_parking_exit_steering);

        if (OBSTACLE_PARKING_REAR_TOF_POSITIONING_ENABLED)
        {
            stop(false);
            servo_disabled = false;
            set_steering(0);
            steer(0);
            oc_parking_rear_start_distance = get_distance();
            oc_parking_rear_start_range = -1.0f;
            oc_parking_rear_last_range = get_tof_distance(TOF_REAR);
            resetParkingRearSamples();
            oc_parking_rear_planned_travel = 0.0f;
            oc_parking_rear_cumulative_travel = 0.0f;
            oc_parking_rear_last_encoder_distance = get_distance();
            oc_parking_rear_direction = 0;
            oc_parking_rear_verifying = false;
            oc_parking_rear_discard_frames = 0;
            TofDiagnosticSnapshot snapshot;
            oc_parking_rear_tof_sequence =
                get_tof_diagnostic_snapshot(TOF_REAR, snapshot)
                    ? snapshot.sequence
                    : 0;
            oc_parking_exit_settle_start_ms = millis();
            oc_parking_exit_state = PARKING_EXIT_REAR_SETTLE;

            Serial.print("[PARK REAR] Armed initial_range_mm=");
            Serial.print(oc_parking_rear_last_range, 1);
            Serial.print(" target_range_mm=");
            Serial.print(OBSTACLE_PARKING_REAR_TOF_TARGET_RANGE_MM, 1);
            Serial.print(" tolerance_mm=");
            Serial.println(OBSTACLE_PARKING_REAR_TOF_TOLERANCE_MM, 1);
            return true;
        }

        oc_parking_rear_start_range = -1.0f;
        beginParkingExitSegments(-1.0f);
    }

    const ParkingExitSegment &segment =
        PARKING_EXIT_SEGMENTS[oc_parking_exit_segment];
    const int segmentSteering =
        segment.steeringRelativeToAway *
        oc_parking_exit_steering;

    if (oc_parking_exit_state == PARKING_EXIT_SEGMENT_SETTLE)
    {
        if (dc_state != DC_DISABLED)
            stop(false);
        servo_disabled = false;
        set_steering(segmentSteering);
        steer(segmentSteering);

        if (
            millis() - oc_parking_exit_settle_start_ms <
                OBSTACLE_PARKING_EXIT_STEER_SETTLE_MS)
        {
            return true;
        }

        oc_parking_exit_state_distance = get_distance();
        oc_parking_exit_state = PARKING_EXIT_SEGMENT_DRIVE;
        Serial.print("[PARK EXIT] Segment ");
        Serial.print(oc_parking_exit_segment + 1);
        Serial.print(" direction=");
        Serial.print(segment.direction < 0 ? "reverse" : "forward");
        Serial.print(" steering=");
        Serial.print(segmentSteering);
        Serial.print(" target_mm=");
        Serial.println(segment.distanceMm, 1);
        return true;
    }

    set_speed(
        segment.direction *
        OBSTACLE_PARKING_EXIT_SPEED);
    set_steering(segmentSteering);

    const float stateDistance =
        distanceSince(oc_parking_exit_state_distance);
    const bool finalSegment =
        oc_parking_exit_segment + 1 ==
        OBSTACLE_PARKING_EXIT_SEGMENT_COUNT;
    const float headingError =
        fabsf(get_angle() - oc_parking_exit_start_heading);
    const bool finalAligned =
        finalSegment &&
        stateDistance >= OBSTACLE_PARKING_EXIT_FINAL_ALIGN_MIN_MM &&
        headingError <=
            OBSTACLE_PARKING_EXIT_FINAL_HEADING_TOLERANCE_DEG;
    const bool finalLimitReached =
        finalSegment &&
        stateDistance >= OBSTACLE_PARKING_EXIT_FINAL_ALIGN_MAX_MM;
    const bool regularTargetReached =
        !finalSegment &&
        stateDistance >= segment.distanceMm;

    if (regularTargetReached || finalAligned || finalLimitReached)
    {
        if (finalSegment)
        {
            Serial.print("[PARK EXIT] Final alignment distance_mm=");
            Serial.print(stateDistance, 1);
            Serial.print(" heading_error_deg=");
            Serial.print(headingError, 1);
            Serial.print(" aligned=");
            Serial.println(finalAligned ? "yes" : "no_max_distance");

            // Remove the full-lock curvature before holding the encoder
            // position. In log_123 the alignment trigger occurred at 1.6
            // degrees, but leaving the wheels at full lock during the final
            // 2 mm of settling changed the stopped heading to -2.3 degrees.
            // Commanding the servo directly here lets it begin centring before
            // stop(true) disables further steering updates.
            servo_disabled = false;
            set_steering(0);
            steer(0);
        }

        // The short first movements have little room for an uncontrolled
        // coast. Hold the encoder position briefly, then release the motor.
        stop(true);
        oc_parking_exit_brake_start_ms = millis();
        oc_parking_exit_state = PARKING_EXIT_SEGMENT_BRAKE;
        return true;
    }

    return true;
}

static WallSide wallForColor(ColorType color)
{
    return color == ColorType::GREEN
        ? SIDE_LEFT
        : SIDE_RIGHT;
}

static uint8_t sortedKnownObstacles(
    uint8_t sectionIndex,
    const CourseObstacle *ordered[
        COURSE_MAX_OBSTACLES_PER_SECTION])
{
    const CourseSection &section =
        course_map_get_section(sectionIndex);
    uint8_t count = 0;

    for (uint8_t i = 0;
         i < COURSE_MAX_OBSTACLES_PER_SECTION;
         ++i)
    {
        if (section.obstacles[i].known)
            ordered[count++] = &section.obstacles[i];
    }

    if (count == 2 &&
        ordered[0]->firstDetectionDistanceMm >
            ordered[1]->firstDetectionDistanceMm)
    {
        const CourseObstacle *temporary = ordered[0];
        ordered[0] = ordered[1];
        ordered[1] = temporary;
    }

    return count;
}

static bool updateLearnedLanePlan()
{
    if (oc_current_lap == 0)
        return false;

    const CourseObstacle *current[
        COURSE_MAX_OBSTACLES_PER_SECTION] = {nullptr, nullptr};
    const uint8_t currentCount =
        sortedKnownObstacles(
            oc_current_section,
            current);

    if (currentCount == 0)
    {
        const CourseSection &section =
            course_map_get_section(oc_current_section);
        WallSide guessedWall = SIDE_UNKNOWN;

        if (section.successfulLane < 0)
            guessedWall = SIDE_LEFT;
        else if (section.successfulLane > 0)
            guessedWall = SIDE_RIGHT;
        else
        {
            guessedWall =
                navigation_get_following_wall();
            if (guessedWall == SIDE_UNKNOWN)
                guessedWall = SIDE_LEFT;
        }

        navigation_select_wall(
            guessedWall,
            OBSTACLE_PLANNED_LANE_WALL_MM);

        static int lastGuessSection = -1;
        if (lastGuessSection != oc_current_section)
        {
            lastGuessSection = oc_current_section;
            Serial.print("[MAP] EXCEPTION: guessing S");
            Serial.print(oc_current_section);
            Serial.print(" from traversed lane ");
            Serial.println(
                guessedWall == SIDE_LEFT
                    ? "LEFT"
                    : "RIGHT");
        }
        return true;
    }

    ColorType desiredColor = ColorType::NONE;
    const float sectionDistance = obstacleSectionDistance();

    if (currentCount > 0)
    {
        desiredColor = current[0]->color;

        if (currentCount == 2 &&
            sectionDistance >=
                (oc_current_section == 0
                     ? OBSTACLE_START_SECTION_SWITCH_MM
                     : current[0]->firstDetectionDistanceMm +
                           OBSTACLE_PLANNED_SWITCH_AFTER_MM))
        {
            desiredColor = current[1]->color;
        }

        const CourseObstacle *last =
            current[currentCount - 1];
        if (sectionDistance >=
                (oc_current_section == 0
                     ? OBSTACLE_START_SECTION_NEXT_PLAN_MM
                     : last->firstDetectionDistanceMm +
                           OBSTACLE_PLANNED_NEXT_SECTION_MM))
        {
            const CourseObstacle *next[
                COURSE_MAX_OBSTACLES_PER_SECTION] =
                    {nullptr, nullptr};
            const uint8_t nextCount =
                sortedKnownObstacles(
                    (oc_current_section + 1) %
                        COURSE_SECTION_COUNT,
                    next);
            if (nextCount > 0)
                desiredColor = next[0]->color;
        }
    }

    if (desiredColor != ColorType::NONE)
    {
        navigation_select_wall(
            wallForColor(desiredColor),
            OBSTACLE_PLANNED_LANE_WALL_MM);
    }

    return true;
}

static void updateCourseProgress()
{
    const NavigationState navigationState =
        navigation_get_state();

    if (oc_current_lap == 0 &&
        oc_last_navigation_state == NAV_FOLLOWING &&
        navigationState == NAV_TURNING)
    {
        const WallSide wall =
            navigation_get_following_wall();
        if (wall == SIDE_LEFT || wall == SIDE_RIGHT)
        {
            course_map_record_successful_lane(
                oc_current_section,
                wall == SIDE_LEFT ? -1 : 1);
        }
    }

    // A new straight section starts only after the gyro-controlled 90 degree
    // turn has completed. This makes the section reference repeatable.
    const int completedTurns =
        navigation_get_turn_count();

    if (
        navigationState == NAV_FOLLOWING &&
        completedTurns > oc_last_completed_turn)
    {
        oc_last_completed_turn = completedTurns;
        oc_current_section =
            (oc_current_section + 1) % COURSE_SECTION_COUNT;
        oc_current_lap =
            static_cast<uint8_t>(
                navigation_get_turn_count() /
                COURSE_SECTION_COUNT);
        oc_section_start_distance = get_distance();
        oc_corner_settling = true;
        for (uint8_t i = 0;
             i < COURSE_MAX_OBSTACLES_PER_SECTION;
             ++i)
        {
            oc_known_obstacle_used[i] = false;
        }

        course_map_enter_section(
            oc_current_section,
            true);

        Serial.print("[OC] Lap ");
        Serial.print(oc_current_lap);
        Serial.print(" section ");
        Serial.println(oc_current_section);

        if (oc_current_section == 0)
        {
            course_map_print();
        }
    }

    oc_last_navigation_state = navigationState;
}

// ============================================================
// VISION DEBUG
// ============================================================

void printVisionDebug()
{
    static uint32_t lastPrint = 0;

    if (
        millis() - lastPrint <
        500)
    {
        return;
    }

    lastPrint = millis();

    const VisionResult &v =
        vision.getResult();

    Serial.println();
    Serial.println("======================");
    Serial.println("VISION");
    Serial.println("======================");

    Serial.print("Processing: ");
    Serial.print(
        v.processingTimeUs /
        1000.0f);
    Serial.println(" ms");

    // --------------------------------------------------------
    // RED
    // --------------------------------------------------------

    if (v.red.found)
    {
        Serial.println();
        Serial.println("RED:");

        Serial.print("X: ");
        Serial.println(
            v.red.centerX);

        Serial.print("Y: ");
        Serial.println(
            v.red.centerY);

        Serial.print("Width: ");
        Serial.println(
            v.red.width());

        Serial.print("Height: ");
        Serial.println(
            v.red.height());

        Serial.print("Bottom Y: ");
        Serial.println(
            v.red.maxY);

        Serial.print("Area: ");
        Serial.println(
            v.red.area);
    }

    // --------------------------------------------------------
    // GREEN
    // --------------------------------------------------------

    if (v.green.found)
    {
        Serial.println();
        Serial.println("GREEN:");

        Serial.print("X: ");
        Serial.println(
            v.green.centerX);

        Serial.print("Y: ");
        Serial.println(
            v.green.centerY);

        Serial.print("Width: ");
        Serial.println(
            v.green.width());

        Serial.print("Height: ");
        Serial.println(
            v.green.height());

        Serial.print("Bottom Y: ");
        Serial.println(
            v.green.maxY);

        Serial.print("Area: ");
        Serial.println(
            v.green.area);
    }

    // Orange and blue are still available for debugging,
    // but they are NOT used for navigation or lap counting.

    if (v.orange.found)
    {
        Serial.print(
            "ORANGE debug area: ");

        Serial.println(
            v.orange.area);
    }

    if (v.blue.found)
    {
        Serial.print(
            "BLUE debug area: ");

        Serial.println(
            v.blue.area);
    }

    Serial.print(
        "OA state: ");

    Serial.println(
        obstacle_avoidance_state_string(
            oa_state));

    Serial.print(
        "Open/Obstacle turns: ");

    Serial.println(
        navigation_get_turn_count());

    Serial.print(
        "Target heading: ");

    Serial.println(
        navigation_get_target_heading());
}

// ============================================================
// SIMPLE DETECTION DEBUG
// ============================================================

void handleObstacleDetection()
{
    const Blob *obstacle =
        getLargestObstacle();

    if (obstacle == nullptr)
    {
        return;
    }

    Serial.print(
        "Obstacle X: ");

    Serial.print(
        obstacle->centerX);

    Serial.print(
        " Area: ");

    Serial.print(
        obstacle->area);

    Serial.print(
        " Color: ");

    if (
        obstacle->color ==
        ColorType::RED)
    {
        Serial.println("RED");
    }
    else if (
        obstacle->color ==
        ColorType::GREEN)
    {
        Serial.println("GREEN");
    }
}

// ============================================================
// VALIDATE OBSTACLE
// ============================================================

bool obstacle_blob_valid_for_acquisition(
    const Blob *obstacle)
{
    if (obstacle == nullptr)
    {
        return false;
    }

    if (!obstacle->found)
    {
        return false;
    }

    if (
        obstacle->color !=
            ColorType::RED &&
        obstacle->color !=
            ColorType::GREEN)
    {
        return false;
    }

    const uint32_t minimumArea =
        (obstacle->color == ColorType::RED)
            ? OBSTACLE_RED_MIN_AREA
            : OBSTACLE_GREEN_MIN_AREA;

    const int minimumHeight =
        (obstacle->color == ColorType::RED)
            ? OBSTACLE_RED_MIN_HEIGHT
            : OBSTACLE_GREEN_MIN_HEIGHT;

    if (obstacle->area < minimumArea)
    {
        return false;
    }

    if (obstacle->height() < minimumHeight)
    {
        return false;
    }

    if (obstacle->maxY < OBSTACLE_MIN_BOTTOM_Y)
    {
        return false;
    }

    if (obstacle->minY > OBSTACLE_MAX_TOP_Y)
    {
        return false;
    }

    if (
        obstacle->centerX < OBSTACLE_START_MIN_X ||
        obstacle->centerX > OBSTACLE_START_MAX_X ||
        obstacle->width() > OBSTACLE_MAX_START_WIDTH ||
        obstacle->height() > OBSTACLE_MAX_START_HEIGHT)
    {
        return false;
    }

    if (
        static_cast<float>(obstacle->width()) >
        static_cast<float>(obstacle->height()) *
            OBSTACLE_MAX_WIDTH_HEIGHT_RATIO)
    {
        return false;
    }

    return true;
}

static bool validTrackedObstacle(
    const Blob *obstacle)
{
    if (obstacle_blob_valid_for_acquisition(obstacle))
    {
        return true;
    }

    if (
        obstacle == nullptr ||
        !obstacle->found ||
        obstacle->color != oa_color)
    {
        return false;
    }

    // A confirmed block becomes smaller when it is clipped by a side of the
    // image. These relaxed limits are tracking-only: they can never start a
    // maneuver, so orange/blue line fragments remain unable to trigger one.
    const bool nearSide =
        obstacle->centerX <= 75 ||
        obstacle->centerX >= 245;

    if (!nearSide)
    {
        return false;
    }

    if (
        obstacle->area < 100 ||
        obstacle->height() < 15 ||
        obstacle->maxY < OBSTACLE_MIN_BOTTOM_Y)
    {
        return false;
    }

    return
        static_cast<float>(obstacle->width()) <=
        static_cast<float>(obstacle->height()) *
            OBSTACLE_MAX_WIDTH_HEIGHT_RATIO;
}

// ============================================================
// GET CURRENTLY TRACKED COLOUR
// ============================================================

static const Blob *getTrackedObstacle()
{
    const VisionResult &v =
        vision.getResult();

    if (
        oa_color ==
        ColorType::RED)
    {
        if (v.red.found)
        {
            return &v.red;
        }

        return nullptr;
    }

    if (
        oa_color ==
        ColorType::GREEN)
    {
        if (v.green.found)
        {
            return &v.green;
        }

        return nullptr;
    }

    return nullptr;
}

// ============================================================
// TARGET X POSITION
//
// RED stays LEFT in the camera image.
// The vehicle therefore passes RED on the RIGHT.
//
// GREEN stays RIGHT in the image.
// The vehicle therefore passes GREEN on the LEFT.
// ============================================================

static int obstacleTargetX(
    ColorType color)
{
    if (
        color ==
        ColorType::RED)
    {
        return OBSTACLE_RED_TARGET_X;
    }

    return OBSTACLE_GREEN_TARGET_X;
}

// ============================================================
// PASSING DIRECTION
// ============================================================

static int obstacleAvoidDirection(
    ColorType color)
{
    // Current assumption:
    //
    // positive steering -> RIGHT
    // negative steering -> LEFT
    //
    // Verify this once on the real robot.

    if (
        color ==
        ColorType::RED)
    {
        return 1;
    }

    return -1;
}

static int expectedKnownObstacleIndex(
    float sectionDistance,
    ColorType &expectedColor)
{
    expectedColor = ColorType::NONE;
    if (oc_current_lap == 0)
        return -1;

    const CourseSection &section =
        course_map_get_section(oc_current_section);

    int bestIndex = -1;
    float bestDifference = 100000.0f;
    for (uint8_t i = 0;
         i < COURSE_MAX_OBSTACLES_PER_SECTION;
         ++i)
    {
        const CourseObstacle &known = section.obstacles[i];
        if (!known.known ||
            oc_known_obstacle_used[i] ||
            known.firstLap >= oc_current_lap)
            continue;

        const float difference =
            sectionDistance -
            known.firstDetectionDistanceMm;

        // Start looking before the former camera trigger, but do not keep a
        // stale prediction active throughout the rest of the section.
        if (difference < -220.0f || difference > 260.0f)
            continue;

        if (fabsf(difference) < bestDifference)
        {
            bestDifference = fabsf(difference);
            bestIndex = i;
            expectedColor = known.color;
        }
    }

    return bestIndex;
}

static bool validKnownColorObservation(
    const Blob *obstacle,
    ColorType expectedColor)
{
    if (obstacle == nullptr ||
        !obstacle->found ||
        obstacle->color != expectedColor)
        return false;

    return obstacle->area >= 100 &&
           obstacle->height() >= 12 &&
           obstacle->maxY >= 90 &&
           obstacle->width() <= 120;
}

// ============================================================
// AVOIDANCE GETTERS
// ============================================================

bool obstacle_avoidance_active()
{
    return oa_state != OA_IDLE;
}

ObstacleAvoidanceState
obstacle_avoidance_get_state()
{
    return oa_state;
}

const char *
obstacle_avoidance_state_string(
    ObstacleAvoidanceState state)
{
    switch (state)
    {
    case OA_IDLE:
        return "IDLE";

    case OA_TRACKING:
        return "TRACKING";

    case OA_PASSING:
        return "PASSING";

    case OA_RECOVERING:
        return "RECOVERING";

    default:
        return "UNKNOWN";
    }
}

// ============================================================
// RESET AVOIDANCE
// ============================================================

void obstacle_avoidance_reset()
{
    oa_state =
        OA_IDLE;

    oa_color =
        ColorType::NONE;

    oa_base_heading = 0;

    oa_state_start_distance = 0;

    oa_last_finish_distance = 0;

    oa_has_finished_obstacle =
        false;

    oa_confirm_frames = 0;

    oa_lost_frames = 0;
    oa_candidate_color = ColorType::NONE;

    oa_last_camera_error = 0;
}

// ============================================================
// OBSTACLE AVOIDANCE UPDATE
// ============================================================

bool obstacle_avoidance_update(
    bool enabled,
    bool newCameraFrame)
{
    if (!enabled)
    {
        obstacle_avoidance_reset();

        return false;
    }

    // ========================================================
    // IDLE
    //
    // Wait for a stable red or green object.
    // ========================================================

    if (oa_state == OA_IDLE)
    {
        // Only start avoidance while the normal navigation
        // controller is actually driving a straight section.

        if (
            !oc_bench_test &&
            navigation_get_state() !=
            NAV_FOLLOWING)
        {
            oa_confirm_frames = 0;

            return false;
        }

        // Do not detect the same obstacle again immediately
        // after passing it.

        if (
            oa_has_finished_obstacle &&
            distanceSince(
                oa_last_finish_distance) <
                OBSTACLE_REARM_DISTANCE_MM)
        {
            return false;
        }

        if (!newCameraFrame)
        {
            return false;
        }

        const Blob *obstacle =
            getLargestObstacle();

        ColorType expectedColor = ColorType::NONE;
        const int expectedIndex =
            expectedKnownObstacleIndex(
                obstacleSectionDistance(),
                expectedColor);
        const bool memoryConfirmed =
            expectedIndex >= 0 &&
            validKnownColorObservation(
                obstacle,
                expectedColor);

        if (
            !memoryConfirmed &&
            !obstacle_blob_valid_for_acquisition(obstacle))
        {
            oa_confirm_frames = 0;
            oa_candidate_color = ColorType::NONE;

            return false;
        }

        if (memoryConfirmed)
        {
            oa_candidate_color = obstacle->color;
            oa_confirm_frames = OBSTACLE_CONFIRM_FRAMES;
            oc_known_obstacle_used[expectedIndex] = true;
            Serial.print("[MAP] Predicted ");
            Serial.print(
                expectedColor == ColorType::RED
                    ? "RED"
                    : "GREEN");
            Serial.print(" confirmed at ");
            Serial.println(obstacleSectionDistance(), 0);
        }
        else if (obstacle->color != oa_candidate_color)
        {
            oa_candidate_color = obstacle->color;
            oa_confirm_frames = 1;
        }
        else if (oa_confirm_frames < 255)
        {
            ++oa_confirm_frames;
        }

        if (
            oa_confirm_frames <
            OBSTACLE_CONFIRM_FRAMES)
        {
            return false;
        }

        // Stable obstacle confirmed.

        oa_color =
            obstacle->color;
        oa_candidate_color = ColorType::NONE;

        course_map_record_obstacle(
            oc_current_section,
            oc_current_lap,
            oa_color,
            obstacleSectionDistance(),
            obstacle->centerX,
            obstacle->maxY);

        // Save the exact heading target of the normal
        // wall follower before taking over steering.

        oa_base_heading =
            navigation_get_target_heading();

        oa_last_camera_error =
            obstacle->centerX -
            obstacleTargetX(
                oa_color);

        oa_lost_frames = 0;

        oa_state_start_distance =
            get_distance();

        oa_state =
            OA_TRACKING;

        Serial.print(
            "[OA] TRACKING ");

        if (
            oa_color ==
            ColorType::RED)
        {
            Serial.println("RED");
        }
        else
        {
            Serial.println("GREEN");
        }

        return true;
    }

    // ========================================================
    // TRACKING
    //
    // Camera error + gyro heading correction.
    // ========================================================

    if (oa_state == OA_TRACKING)
    {
        const Blob *obstacle =
            getTrackedObstacle();

        // Only count lost frames when a genuinely new
        // camera image has been processed.

        if (newCameraFrame)
        {
            if (
                validTrackedObstacle(
                    obstacle))
            {
                oa_lost_frames = 0;

                oa_last_camera_error =
                    obstacle->centerX -
                    obstacleTargetX(
                        oa_color);
            }
            else
            {
                if (
                    oa_lost_frames <
                    255)
                {
                    ++oa_lost_frames;
                }
            }
        }

        // Object disappeared for several camera frames:
        // assume the vehicle has reached/passed its side.

        if (
            oa_lost_frames >=
            OBSTACLE_LOST_FRAMES)
        {
            if (oc_bench_test)
            {
                oa_state = OA_PASSING;
                oa_state_start_distance = get_distance();
                Serial.println(
                    "[BENCH] OBJECT LOST -> PASSING HOLD");
                return true;
            }

            oa_state =
                OA_PASSING;

            oa_state_start_distance =
                get_distance();

            Serial.println(
                "[OA] TRACKING -> PASSING");

            return true;
        }

        const float headingError =
            get_angle() -
            oa_base_heading;

        float steering =
            OBSTACLE_CAMERA_KP *
                oa_last_camera_error +
            OBSTACLE_HEADING_KP *
                headingError;

        steering = applyWallGuard(steering);

        steering =
            clampValue(
                steering,
                -OBSTACLE_MAX_STEERING,
                OBSTACLE_MAX_STEERING);

        set_steering(
            static_cast<int>(
                steering));

        setAvoidanceSpeed(
            OBSTACLE_AVOID_SPEED);

        return true;
    }

    // ========================================================
    // PASSING
    //
    // Continue around the obstacle after it leaves the camera.
    // ========================================================

    if (oa_state == OA_PASSING)
    {
        const int direction =
            obstacleAvoidDirection(
                oa_color);

        const float headingError =
            get_angle() -
            oa_base_heading;

        float steering =
            direction *
                OBSTACLE_PASS_STEERING +
            OBSTACLE_HEADING_KP *
                headingError;

        steering = applyWallGuard(steering);

        steering =
            clampValue(
                steering,
                -OBSTACLE_MAX_STEERING,
                OBSTACLE_MAX_STEERING);

        set_steering(
            static_cast<int>(
                steering));

        setAvoidanceSpeed(
            OBSTACLE_AVOID_SPEED);

        if (
            distanceSince(
                oa_state_start_distance) >=
            OBSTACLE_PASS_DISTANCE_MM)
        {
            oa_state =
                OA_RECOVERING;

            Serial.println(
                "[OA] PASSING -> RECOVERING");
        }

        return true;
    }

    // ========================================================
    // RECOVERING
    //
    // Return to the original gyro heading.
    // ========================================================

    if (oa_state == OA_RECOVERING)
    {
        const float headingError =
            get_angle() -
            oa_base_heading;

        float steering =
            OBSTACLE_RECOVER_KP *
            headingError;

        steering = applyWallGuard(steering);

        steering =
            clampValue(
                steering,
                -OBSTACLE_RECOVER_MAX_STEERING,
                OBSTACLE_RECOVER_MAX_STEERING);

        set_steering(
            static_cast<int>(
                steering));

        setAvoidanceSpeed(
            OBSTACLE_RECOVER_SPEED);

        if (
            fabsf(headingError) <=
            OBSTACLE_RECOVER_TOLERANCE_DEG)
        {
            oa_last_finish_distance =
                get_distance();

            oa_has_finished_obstacle =
                true;

            oa_state =
                OA_IDLE;

            oa_color =
                ColorType::NONE;

            oa_confirm_frames = 0;
            oa_lost_frames = 0;

            set_steering(0);

            Serial.println(
                "[OA] RECOVERY COMPLETE");

            // Give the normal wall follower a fresh starting
            // point after the avoidance maneuver.
            navigation_rearm_after_obstacle();

            return false;
        }

        return true;
    }

    return false;
}

// ============================================================
// OBSTACLE CHALLENGE SETUP
// ============================================================

void obstacle_challenge_setup()
{
    navigation_set_obstacle_mode(true);
    navigation_set_speed(
        OBSTACLE_CRUISE_SPEED);

    oc_was_enabled =
        false;

    resetParkingExit();
    obstacle_avoidance_reset();
    obstacle_path_reset();
    oc_finish_requested = false;
    oc_complete = false;

    Serial.println(
        "===== OBSTACLE CHALLENGE READY =====");
}

// ============================================================
// IS OBSTACLE DETECTION ALREADY ACTIVE?
// ============================================================

bool obstacle_challenge_active()
{
    return oc_was_enabled;
}

bool obstacle_challenge_complete()
{
    return oc_complete;
}

bool obstacle_parking_exit_active()
{
    // Keep verbose camera output disabled in the isolated test hold too, so
    // the small logger cannot overwrite the manoeuvre's diagnostic lines.
    return oc_parking_exit_state != PARKING_EXIT_DONE;
}

void obstacle_bench_test_set(bool enable)
{
    oc_bench_test = enable;
    obstacle_avoidance_reset();
    navigation_set_speed(
        OBSTACLE_CRUISE_SPEED);

    // This is deliberately independent of the physical enable switch.
    // No obstacle-test path may energize the drive motor.
    stop(false);
    servo_disabled = !enable;
    set_steering(0);

    Serial.println(enable
        ? "[BENCH] ON - DRIVE MOTOR LOCKED OFF"
        : "[BENCH] OFF");
}

bool obstacle_bench_test_active()
{
    return oc_bench_test;
}

// ============================================================
// COMPLETE OBSTACLE CHALLENGE CONTROL
// ============================================================

void obstacle_challenge_update(
    bool enabled,
    bool newCameraFrame)
{
    if (oc_bench_test)
    {
        // Reassert the safety condition on every loop, even if the physical
        // enable switch or another serial command was used accidentally.
        if (dc_state != DC_DISABLED)
        {
            stop(false);
        }
        servo_disabled = false;

        obstacle_avoidance_update(
            true,
            newCameraFrame);
        return;
    }

    // ========================================================
    // DISABLED
    // ========================================================

    if (!enabled)
    {
        if (oc_was_enabled)
        {
            obstacle_avoidance_reset();
        }

        oc_was_enabled =
            false;
        resetParkingExit();
        obstacle_path_reset();
        oc_finish_requested = false;
        oc_complete = false;

        return;
    }

    // ========================================================
    // NEW RUN
    // ========================================================

    if (!oc_was_enabled)
    {
        oc_was_enabled =
            true;

        // The isolated parking test only needs manoeuvre diagnostics. Clearing
        // verbose startup output also keeps the useful data on USB drives that
        // currently truncate each write after about 1024 bytes.
        if (OBSTACLE_PARKING_EXIT_TEST_ONLY)
        {
            robot_logger.clear();
        }

        obstacle_avoidance_reset();
        obstacle_path_reset();
        course_map_reset();
        oc_finish_requested = false;
        oc_complete = false;

        oc_current_section = 0;
        oc_current_lap = 0;
        oc_section_start_distance = get_distance();
        oc_last_navigation_state =
            navigation_get_state();
        oc_last_completed_turn =
            navigation_get_turn_count();
        oc_corner_settling = false;
        for (uint8_t i = 0;
             i < COURSE_MAX_OBSTACLES_PER_SECTION;
             ++i)
        {
            oc_known_obstacle_used[i] = false;
        }

        // The rules allow starting in the parking lot or in the middle zone
        // above it. Store colour/order, but do not treat its encoder origin
        // like the repeatable origin after a corner.
        course_map_enter_section(
            oc_current_section,
            false);

        Serial.println(
            "[OC] New obstacle run");
        printParkingExitGeometry();
    }

    // This path exists only in the Obstacle Challenge. While it owns the
    // steering, camera avoidance and the regular challenge remain untouched.
    if (updateParkingExit())
    {
        return;
    }

    // The parking lot is mounted against the outer wall. The existing exit
    // turns away from that wall, which also reveals the required direction
    // around the inner field. Without a parking exit, use the explicit
    // pre-round fallback in config.h.
    if (!obstacle_path_started())
    {
        const int8_t turnSign =
            OBSTACLE_PARKING_EXIT_ENABLED
                ? (oc_parking_exit_steering > 0 ? -1 : 1)
                : OBSTACLE_DEFAULT_TURN_SIGN;
        const float firstCornerDistance =
            turnSign > 0
                ? OBSTACLE_PARKING_TO_FIRST_CORNER_CCW_MM
                : OBSTACLE_PARKING_TO_FIRST_CORNER_CW_MM;
        obstacle_path_start(
            turnSign,
            false,
            firstCornerDistance,
            0,
            0.0f,
            OBSTACLE_PARKING_EXIT_ENABLED &&
                OBSTACLE_PARKING_ENTRY_DISCOVERY_ENABLED);
    }

    if (!obstacle_path_complete())
    {
        obstacle_path_update(newCameraFrame);
        return;
    }

    // End-of-run parking is intentionally not guessed here: the repository
    // only contains the start parking exit. Stop safely in the start section
    // until a separately calibrated parking state machine is supplied.
    set_steering(0);
    set_speed(0);
    if (!oc_finish_requested)
    {
        oc_finish_requested = true;
        Serial.println(
            "[OC] Three laps complete; final parking is not implemented");
    }
    if (!oc_complete &&
        fabsf(current_speed) <= SOFT_STOP_SPEED_THRESHOLD_MMS &&
        fabsf(measured_speed) <= SOFT_STOP_SPEED_THRESHOLD_MMS)
    {
        stop(false);
        oc_complete = true;
        robot_logger.write_to_usb();
        Serial.println("[OC] Controlled stop complete");
    }
}
