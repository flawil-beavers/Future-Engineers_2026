#pragma once

#include <Arduino.h>
#include "vision.h"

enum ObstacleObservationStatus : uint8_t {
    OBSTACLE_OBSERVATION_NO_BLOB,
    OBSTACLE_OBSERVATION_REJECTED_BLOB,
    OBSTACLE_OBSERVATION_INVALID_RANGE,
    OBSTACLE_OBSERVATION_NO_SEAT,
    OBSTACLE_OBSERVATION_VOTE,
    OBSTACLE_OBSERVATION_CONFIRMED,
    OBSTACLE_OBSERVATION_ALREADY_CONFIRMED
};

struct ObstacleSeatInfo {
    uint8_t id = 0;
    uint8_t section = 0;
    uint8_t station = 0;
    char side = '?';
    float xMm = 0.0f;
    float yMm = 0.0f;
    float headingDeg = 0.0f;
    float pathDistanceMm = 0.0f;
    float lateralMm = 0.0f;
    uint8_t redVotes = 0;
    uint8_t greenVotes = 0;
    bool confirmed = false;
    bool red = false;
    bool injected = false;
};

struct ObstacleObservationResult {
    ObstacleObservationStatus status = OBSTACLE_OBSERVATION_NO_BLOB;
    bool productionValid = false;
    ColorType color = ColorType::NONE;
    int16_t left = 0;
    int16_t top = 0;
    int16_t right = 0;
    int16_t bottom = 0;
    float bearingDeg = 0.0f;
    float rangeMm = -1.0f;
    float robotXmm = 0.0f;
    float robotYmm = 0.0f;
    float robotHeadingDeg = 0.0f;
    float cameraXmm = 0.0f;
    float cameraYmm = 0.0f;
    float sightingXmm = 0.0f;
    float sightingYmm = 0.0f;
    int8_t seatId = -1;
    float snapErrorMm = -1.0f;
    uint8_t redVotes = 0;
    uint8_t greenVotes = 0;
    bool confirmed = false;
    bool injected = false;
    uint16_t injectionCount = 0;
    char passSide = '?';
    float peakDisplacementMm = 0.0f;
    float movementCircleClearanceMm = 0.0f;
};

struct ObstacleDiscoveryTelemetry {
    int8_t station = -1;
    uint8_t visibleMask = 0; // bit 0 = right seat, bit 1 = left seat
    uint8_t clearEvidenceMask = 0; // current frame, same side bits as visible
    uint8_t clearFrames[2] = {};
    // Predicted geometry from the camera to the station's right/left seats.
    // This is independent of whether the image contains an obstacle blob.
    float seatBearingDeg[2] = {};
    float seatRangeMm[2] = {};
    ObstacleObservationStatus observationStatus =
        OBSTACLE_OBSERVATION_NO_BLOB;
    int8_t observationSeat = -1;
    int16_t left = 0;
    int16_t top = 0;
    int16_t right = 0;
    int16_t bottom = 0;
    float bearingDeg = 0.0f;
    float rangeMm = -1.0f;
};

struct ObstacleTofCorrectionResult {
    bool geometryReady = false;
    bool leftUsed = false;
    bool rightUsed = false;
    bool leftCornerGated = false;
    bool rightCornerGated = false;
    bool leftResidualGated = false;
    bool rightResidualGated = false;
    float leftReadingMm = -1.0f;
    float rightReadingMm = -1.0f;
    float leftResidualMm = 0.0f;
    float rightResidualMm = 0.0f;
    float correctionXmm = 0.0f;
    float correctionYmm = 0.0f;
};

enum ObstacleWallFeature : uint8_t {
    OBSTACLE_WALL_UNKNOWN,
    OBSTACLE_WALL_OUTER_SOUTH,
    OBSTACLE_WALL_OUTER_EAST,
    OBSTACLE_WALL_OUTER_NORTH,
    OBSTACLE_WALL_OUTER_WEST,
    OBSTACLE_WALL_INNER_SOUTH,
    OBSTACLE_WALL_INNER_EAST,
    OBSTACLE_WALL_INNER_NORTH,
    OBSTACLE_WALL_INNER_WEST,
    OBSTACLE_WALL_INNER_CORNER_SW,
    OBSTACLE_WALL_INNER_CORNER_SE,
    OBSTACLE_WALL_INNER_CORNER_NE,
    OBSTACLE_WALL_INNER_CORNER_NW
};

/** One geometric clearance sample for the conservative complete-robot
 * capsule. Distances are to the pillar movement circle or physical wall;
 * negative values mean that the two safety envelopes overlap. */
struct ObstacleClearanceSample {
    bool valid = false;
    float pillarMm = 1.0e9f;
    float wallMm = 1.0e9f;
    ObstacleWallFeature wallFeature = OBSTACLE_WALL_UNKNOWN;
    float wallXmm = 0.0f;
    float wallYmm = 0.0f;
    float robotXmm = 0.0f;
    float robotYmm = 0.0f;
    float robotHeadingDeg = 0.0f;
    float wallRobotXmm = 0.0f;
    float wallRobotYmm = 0.0f;
    float wallRobotHeadingDeg = 0.0f;
    // SW, SE, NE, NW inner-wall corner clearance.
    float innerCornerMm[4] = {};
};

/** Known-field waypoint planner and Pure Pursuit controller for the
 * WRO Future Engineers Obstacle Challenge.
 *
 * Production paths use a fixed field frame whose origin is the field centre,
 * +X runs along the south straight toward the CCW corner, and +Y points north.
 * Test mode deliberately remains anchored to the pose at test start. */
void obstacle_path_reset();
void obstacle_path_start(
    int8_t turn_sign,
    bool test_mode = false,
    float first_corner_distance_mm = 500.0f,
    uint8_t lap_target = 0,
    float speed_cap_mm_s = 0.0f);
void obstacle_path_update(bool new_camera_frame);
bool obstacle_path_started();
bool obstacle_path_complete();
/** Latched when lap-1 perception reaches an unresolved station's hold limit. */
bool obstacle_path_perception_blocked();
int8_t obstacle_path_blocked_station();
float obstacle_path_discovery_target_nudge_deg();
int8_t obstacle_path_discovery_scan_seat();
bool obstacle_path_get_discovery_telemetry(
    ObstacleDiscoveryTelemetry &telemetry);
uint8_t obstacle_path_lap();
uint16_t obstacle_path_progress_index();
uint16_t obstacle_path_waypoint_count();
float obstacle_path_loop_length_mm();
/** Baseline path distance travelled, unwrapped across completed laps. */
float obstacle_path_travel_distance_mm();
float obstacle_path_cross_track_error_mm();
float obstacle_path_heading_error_deg();
bool obstacle_path_geometry_valid();

/** Run one camera observation through the exact production seat-snapping and
 * live-path injection code. Pure Pursuit control is not run by this call. */
ObstacleObservationResult obstacle_path_observe(const Blob *blob);

uint8_t obstacle_path_seat_count();
bool obstacle_path_get_seat(uint8_t seat_id, ObstacleSeatInfo &info);
/** Minimum clearance of the current planned Pure Pursuit route over the same
 * longitudinal passage window used by the ToF logger. */
bool obstacle_path_get_planned_clearance(
    uint8_t seat_id,
    ObstacleClearanceSample &sample);
/** Clearance of one calculated rear-axle odometry pose. */
bool obstacle_path_sample_pose_clearance(
    uint8_t seat_id,
    float x_mm,
    float y_mm,
    float heading_deg,
    ObstacleClearanceSample &sample);
const char *obstacle_path_wall_feature_name(ObstacleWallFeature feature);
void obstacle_path_clear_observations();
uint16_t obstacle_path_injection_count();

/** Prepare a motor-independent diagnostic pose on the centreline of a known
 * straight or corner. The production fixed-field geometry is used. */
bool obstacle_path_prepare_tof_diagnostic(
    int8_t turn_sign,
    bool corner,
    uint8_t index,
    float initial_lateral_mm,
    float &path_distance_mm,
    float &center_x_mm,
    float &center_y_mm,
    float &heading_deg);

/** Apply one production ToF correction at an explicitly selected path
 * position. This changes only the position estimator; it never drives. */
ObstacleTofCorrectionResult obstacle_path_apply_tof_diagnostic(
    float path_distance_mm);

/** Most recent fresh ToF frame evaluated by production pose correction. */
ObstacleTofCorrectionResult obstacle_path_last_tof_correction();

/** Deterministic checks for the currently generated direction. */
bool obstacle_path_geometry_preflight();
