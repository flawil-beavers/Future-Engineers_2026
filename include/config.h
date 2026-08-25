#pragma once

/**
 * @file config.h
 * @brief Centralized configuration for all pin definitions, constants, and tuning parameters
 */

// ==========================================
// SERIAL CONFIGURATION
// ==========================================
constexpr auto SERIAL_BAUD = 115200;
constexpr auto BUFFER_SIZE = 64;

// ==========================================
// MOTOR & STEERING PINS
// ==========================================
constexpr auto SERVO_PIN = 4;
constexpr auto MOTOR_IN1_PIN = 5; // Direction control 1
constexpr auto MOTOR_IN2_PIN = 6; // Direction control 2
constexpr auto MOTOR_PWM_PIN = 7; // PWM speed control

// ==========================================
// ENCODER PINS
// ==========================================
constexpr auto ENCODER_PIN_A = 3; // Phase A
constexpr auto ENCODER_PIN_B = 2; // Phase B

// ==========================================
// BNO085 IMU (GYRO) - SPI CONFIGURATION
// ==========================================
constexpr auto BNO085_CS = 10;
constexpr auto BNO085_INT = A0;
constexpr auto BNO085_RST = A1;

// ==========================================
// ENABLE SWITCH
// ==========================================
constexpr auto ENABLE_SWITCH_PIN = A2; // Enable switch - HIGH to enable, LOW to disable

// ==========================================
// MOTOR CONTROL CONSTANTS
// ==========================================
constexpr auto GEAR_RATIO = 30; // Motor gear ratio
constexpr auto ENCODER_COUNTS_PER_REV = (GEAR_RATIO * 7 * 4); // CPR of the motor
constexpr auto ENCODER_COUNTS_PER_WHEEL_REV = (28.0 / 20.0 * ENCODER_COUNTS_PER_REV); // CPR of the wheel
constexpr auto COUNTER_TO_MM = (PI * 43.2 / ENCODER_COUNTS_PER_WHEEL_REV); // mm per encoder count

constexpr auto MOTOR_MAX_DC = 200; // Max duty cycle (0-255)
constexpr auto MOTOR_MIN_DC = 80; // Min duty cycle to overcome static friction
constexpr auto MOTOR_MAX_ACC_DC = 255; // Max acceleration duty cycle (DC/s)

// ==========================================
// STEERING CONFIGURATION
// ==========================================
constexpr auto SERVO_CENTER = 80; // Center neutral position
constexpr auto MAX_STEERING = 50;
constexpr auto SERVO_MAX_ANGLE = (SERVO_CENTER + MAX_STEERING); // Max right turn
constexpr auto SERVO_MIN_ANGLE = (SERVO_CENTER - MAX_STEERING); // Max left turn

// ==========================================
// PID CONTROLLER TUNING
// ==========================================
constexpr float CRUISE_KP = 0.12f;
constexpr float CRUISE_KI = 0.04f;
constexpr float LOW_SPEED_CRUISE_KP = 0.035f;
constexpr float LOW_SPEED_CRUISE_KI = 0.020f;
constexpr float MID_SPEED_CRUISE_KP = 0.08f;
constexpr float MID_SPEED_CRUISE_KI = 0.03f;
constexpr float CRUISE_ENTRY_INTEGRAL_MIN = -2.0f;
constexpr float CRUISE_ENTRY_INTEGRAL_MAX = 2.0f;
constexpr float LOW_SPEED_GAIN_END_MMS = 120.0f;
constexpr float MID_SPEED_GAIN_END_MMS = 220.0f;
constexpr float HIGH_SPEED_GAIN_START_MMS = 250.0f;
constexpr float ACCEL_KP = 0.050f;
constexpr float ACCEL_KI = 0.080f;
constexpr float SPEED_INTEGRAL_PWM_MAX = 45.0f;
constexpr float ACCEL_INTEGRAL_PWM_MAX = 35.0f;
constexpr float ACCEL_SPEED_TRACKING_KP = 4.0f;
constexpr float MIN_PROFILE_ACCELERATION_MMSS = 120.0f;
constexpr float PROFILE_ACCEL_PER_TARGET_SPEED = 1.3f;
constexpr float MOTOR_STATIC_FF_DC = 86.0f;
constexpr float MOTOR_SPEED_FF_DC_PER_MMS = 0.113f;
constexpr float MOTOR_ACCEL_FF_DC_PER_MMSS = 0.015f;
constexpr float DRIVE_JERK_LIMIT_MMSSS = 2000.0f;
constexpr float DRIVE_ACCEL_RELEASE_JERK_MMSSS = 500.0f;
constexpr float CRUISE_ACCEL_THRESHOLD_MMSS = 20.0f;
constexpr float CRUISE_SPEED_ERROR_MMS = 30.0f;
constexpr unsigned long CRUISE_ENTRY_DWELL_US = 300000;
constexpr unsigned long SPEED_MEASUREMENT_WINDOW_US = 50000;
constexpr float SPEED_FILTER_ALPHA = 0.60f;
constexpr float LOW_SPEED_FILTER_ALPHA = 0.60f;
constexpr float ACCELERATION_FILTER_ALPHA = 0.30f;
constexpr float SOFT_STOP_SPEED_THRESHOLD_MMS = 5.0f;
constexpr float SOFT_STOP_DECELERATION_MMSS = 200.0f;
constexpr float HOLD_POSITION_KP = 0.8f;
constexpr float HOLD_POSITION_KI = 0.2f;
constexpr float HOLD_POSITION_KD = 0.1f;

// ==========================================
// ACCELERATION SETTINGS
// ==========================================
constexpr auto DEFAULT_ACCELERATION = 500; // mm/s^2

// ==========================================
// PID AUTOTUNE PARAMETERS
// ==========================================
constexpr auto PID_AT_RELAY_AMPLITUDE = 20;
constexpr auto PID_AT_TARGET_SPEED = 250;
constexpr float PID_AT_INITIAL_BASELINE_DC = 116.0f;
constexpr auto PID_AT_MIN_CYCLES = 6;
constexpr auto PID_AT_MAX_DISTANCE_MM = 4000;
constexpr auto PID_AT_MAX_TIME_US = 30000000;
constexpr float PID_AT_HYSTERESIS_MMS = 8.0f;
constexpr int PID_AT_WARMUP_CROSSINGS = 4;
constexpr float PID_AT_MAX_PERIOD_VARIATION = 0.25f;
constexpr float PID_AT_MIN_SPEED_AMPLITUDE = 12.0f;
constexpr float PID_AT_MAX_CENTER_ERROR_MMS = 12.0f;
constexpr float PID_AT_MAX_AMPLITUDE_ASYMMETRY = 0.35f;
// Abort after this time if a symmetric, unsaturated relay test cannot start.
constexpr unsigned long PID_AT_ACCEL_TIMEOUT_US = 8000000;
constexpr unsigned long PID_AT_BASELINE_SETTLE_US = 3000000;
constexpr unsigned long PID_AT_BASELINE_SAMPLE_START_US = 1500000;
constexpr float PID_AT_MAX_BASELINE_SPEED_VARIATION = 0.15f;
// Safety net: abort if a forced relay start happens at a near-zero speed,
// because tuning a stalled robot would produce meaningless gains.
constexpr float PID_AT_MIN_RELAY_SPEED_MMS = 40.0f;

// ==========================================
// SENSOR UPDATE RATES
// ==========================================
constexpr auto STATUS_PRINT_INTERVAL_US = 200000; // Print status every 200ms

// ==========================================
// STALL DETECTION
// ==========================================
constexpr auto STALL_SPEED_THRESHOLD_MMS = 1.0f; // Trigger stall if speed < 1.0 mm/s while demanding high torque
constexpr auto STALL_DC_THRESHOLD = 0.99; // Trigger at 99% of max DC
constexpr unsigned long STALL_DETECTION_WINDOW_US = 100000;
constexpr float HOLD_MAX_DC = 110.0f;
constexpr float HOLD_OVERLOAD_THRESHOLD = 0.95f;
constexpr unsigned long HOLD_OVERLOAD_WINDOW_US = 2000000;

// ==========================================
// ENABLE INTERRUPT DEBOUNCE
// ==========================================
constexpr auto ENABLE_SWITCH_POLL_INTERVAL_US = 50000; // 50ms polling interval
constexpr auto ENABLE_DEBOUNCE_TIME_US = 100000; // 100ms debounce

// ==========================================
// SENSOR READING MODES
// ==========================================
#define TOF_DISTANCE_MODE VL53L4CX_DISTANCEMODE_MEDIUM
constexpr auto TOF_I2C_CLOCK = 400000; // 400kHz I2C clock (standard for VL53L4CX)
constexpr auto TOF_TIMING_BUDGET_US = 30000UL; // Normal 30ms budget; explicitly applied during sensor initialization
constexpr auto TOF_MAX_RELIABLE_DISTANCE_MM = 600.0f; // Max distance for reliable wall detection (mm)
constexpr auto TOF_MAX_LONG_DISTANCE_MM = 4000.0f; // Max distance for long-range discovery (mm)
constexpr auto TOF_OUT_OF_RANGE_MM = 9999.0f; // Value returned when no object is detected or beyond reliable range (mm)
constexpr auto TOF_MAX_DELTA_MM = 100.0f; // Max change allowed between consecutive readings (mm)

// ==========================================
// CALIBRATION AND POSITION ESTIMATION
// ==========================================

// Physical axle-to-axle wheelbase (mm).
// Used for printing/documentation only.  The Ackermann kinematic model
// uses WHEELBASE_MM = 127 mm (Ackermann::WHEELBASE_MM in ackermann_kinematics.h)
// because the pin-slot joint geometry shifts the effective turning centre.
constexpr float PHYSICAL_WHEELBASE_MM = 100.0f;

constexpr auto CAL_SPEED_MMS = 250.0f;
constexpr unsigned long CAL_TURN_SERVO_SETTLE_MS = 600;
constexpr unsigned long CAL_TURN_STOP_SETTLE_MS = 1000;
constexpr unsigned long CAL_TURN_TIMEOUT_MS = 30000;
constexpr int CAL_TURN_MAX_RETRIES = 2;
constexpr float CAL_TURN_MIN_DISTANCE_MM = 50.0f;
constexpr float CAL_TURN_MIN_RADIUS_MM = 50.0f;
constexpr float CAL_TURN_MAX_RADIUS_MM = 5000.0f;
constexpr auto CAL_CENTER_DISTANCE_MM = 2000.0f;
constexpr auto CAL_CENTER_MAX_TIME_MS = 15000;
constexpr auto CAL_CENTER_DEBUG_INTERVAL_MS = 500;

// ==========================================
// MOTOR MIN DC CALIBRATION
// ==========================================
// Three-phase drive-verified calibration for the minimum PWM duty cycle.
// Phase 1 locates the stall threshold (encoder twitch). Phase 2 finds a
// DC that actually drives the robot (coarse steps of 5). Phase 3 fine-tunes
// the drive-verified DC (steps of 1). The final result is Phase 3's value.

// Phase 1 (stall locator): fast ramp from a base value
constexpr auto MC_P1_BASE_DC = 50;              // Starting DC (skips the definitely-stalled range)
constexpr auto MC_P1_STEP_DC = 2;               // DC increment per ramp step
constexpr auto MC_P1_STEP_MS = 50;              // Time per ramp step (ms)
constexpr auto MC_P1_TIMEOUT_MS = 2000;         // Max stall time per phase (ms)

// Phase 2 (coarse drive-find): starts at P1 threshold, steps by 5 until the
// robot actually drives (>= MC_DRIVE_MIN_MM in one window).
constexpr auto MC_P2_OFFSET_DC = 0;             // Start at P1 threshold
constexpr auto MC_P2_STEP_DC = 5;               // Coarse step per failed window
constexpr auto MC_P2_DRIVE_WINDOW_MS = 1000;    // Window to measure drive distance (ms)
constexpr auto MC_P2_TIMEOUT_MS = 10000;        // Total phase 2 time (ms)

// Phase 3 (fine drive-verify): starts a few DC below P2 result, steps by 1
// to find the exact drive-verified DC.
constexpr auto MC_P3_OFFSET_DC = 5;             // Start this many DC below P2 result
constexpr auto MC_P3_STEP_DC = 1;               // Fine step per failed window
constexpr auto MC_P3_DRIVE_WINDOW_MS = 1500;    // Window to measure drive distance (ms)
constexpr auto MC_P3_TIMEOUT_MS = 15000;        // Total phase 3 time (ms)

// Shared drive criteria and safety limits
constexpr auto MC_DRIVE_MIN_MM = 50.0f;         // Min distance per window = "really driving" (mm)
constexpr auto MC_MAX_DC = 150;                 // Hard cap on DC during calibration (below MOTOR_MAX_DC)
constexpr auto MC_MOVEMENT_THRESHOLD_MM = 1.0f; // Encoder distance that confirms movement (mm)
constexpr auto MC_SETTLE_MS = 1000;             // Settle/cooling time between phases (ms)
constexpr auto MC_MAX_PHASE_DISTANCE_MM = 100.0f;  // Per-phase distance limit (mm)
constexpr auto MC_MAX_TOTAL_DISTANCE_MM = 2000.0f; // Total distance limit across all phases (mm)

// Turn-radius polynomial: R(delta) = a0 + a1|delta| + a2|delta|^2 + a3|delta|^3
constexpr auto CAL_LEFT_A0 = 1481.4659f;
constexpr auto CAL_LEFT_A1 = -100.413338f;
constexpr auto CAL_LEFT_A2 = 2.75952291f;
constexpr auto CAL_LEFT_A3 = -0.02698299f;
constexpr auto CAL_RIGHT_A0 = 1422.9576f;
constexpr auto CAL_RIGHT_A1 = -100.615837f;
constexpr auto CAL_RIGHT_A2 = 2.91633201f;
constexpr auto CAL_RIGHT_A3 = -0.02982690f;
constexpr auto CAL_LEFT_K = 1.3101f;
constexpr auto CAL_RIGHT_K = 1.2688f;

constexpr auto POSITION_PRINT_INTERVAL_US = 500000;

// ==========================================
// ENABLE/DISABLE FLAGS MESSAGES
// ==========================================
constexpr auto EN_STATE_TRUE_MSG = "enable start";
constexpr auto EN_STATE_FALSE_MSG = "enable stop";

// ==========================================
// OBSTACLE AVOIDANCE
// ==========================================

// Optional start manoeuvre for an Obstacle Challenge run that begins inside
// the parking lot. Set to false when starting in the middle zone above it.
// This flag has no effect on the Open Challenge.
constexpr bool OBSTACLE_PARKING_EXIT_ENABLED = true;

// Development mode: execute only the parking exit and stop afterwards.
// Set to false once the isolated manoeuvre has been tuned successfully.
constexpr bool OBSTACLE_PARKING_EXIT_TEST_ONLY = false;

// The parking lot is 200 mm wide and the Obstacle track is 1000 mm wide.
// With the inner edge of the 120 mm wide car placed at the parking lot's open
// edge, its centre starts about 140 mm from the outer wall. The first three
// tests ended 60-80 mm beyond the track centre, so the outward arc is reduced.
// The counter-arc is gyro-terminated because the left/right steering radii and
// servo transition distances are not identical in practice.
constexpr auto OBSTACLE_PARKING_EXIT_STEERING = 40;
constexpr auto OBSTACLE_PARKING_EXIT_SPEED = 120;
constexpr auto OBSTACLE_PARKING_EXIT_COUNTER_SPEED = 80;
// Separate calibration is intentional: the measured Ackermann radii and the
// servo linkage are not perfectly symmetric.
constexpr auto OBSTACLE_PARKING_EXIT_FIRST_ARC_NEGATIVE_MM = 239.0f;
constexpr auto OBSTACLE_PARKING_EXIT_FIRST_ARC_POSITIVE_MM = 250.0f;
constexpr auto OBSTACLE_PARKING_EXIT_COUNTER_MIN_MM = 180.0f;
constexpr auto OBSTACLE_PARKING_EXIT_COUNTER_MAX_MM = 400.0f;
constexpr auto OBSTACLE_PARKING_EXIT_FINE_ALIGN_START_DEG = 30.0f;
constexpr auto OBSTACLE_PARKING_EXIT_FINE_ALIGN_SPEED = 50;
constexpr auto OBSTACLE_PARKING_EXIT_FINE_ALIGN_MIN_STEERING = 8.0f;
constexpr auto OBSTACLE_PARKING_EXIT_HEADING_TOLERANCE_DEG = 2.0f;
constexpr auto OBSTACLE_PARKING_EXIT_BRAKE_TIME_NEGATIVE_MS = 450;
constexpr auto OBSTACLE_PARKING_EXIT_BRAKE_TIME_POSITIVE_MS = 250;
constexpr auto OBSTACLE_PARKING_EXIT_MIN_WALL_DIFFERENCE_MM = 80.0f;

// Calibrated route phase after the parking manoeuvre: distance from the rear
// axle at the aligned exit pose to the first centreline corner. The parking
// bay is not centred on its straight, so clockwise and counter-clockwise runs
// deliberately have separate values. Measure these on the assembled field;
// 500 mm preserves the former midpoint assumption until then.
constexpr auto OBSTACLE_PARKING_TO_FIRST_CORNER_CCW_MM = 500.0f;
constexpr auto OBSTACLE_PARKING_TO_FIRST_CORNER_CW_MM = 500.0f;

// Camera processing
constexpr auto OBSTACLE_CAMERA_INTERVAL_MS = 50;

// HSV color classification. The dark-red limits include the measured
// log_16 samples (H=346..354, S=130..183, V=41..57). Keeping the low red hue
// at 0..8 leaves the orange range beginning at 9 degrees unambiguous.
constexpr auto VISION_RED_HUE_LOW_MAX = 8;
constexpr auto VISION_RED_HUE_HIGH_MIN = 340;
constexpr auto VISION_RED_MIN_SATURATION = 120;
constexpr auto VISION_RED_MIN_VALUE = 35;
constexpr auto VISION_ORANGE_HUE_MIN = 9;
constexpr auto VISION_ORANGE_HUE_MAX = 35;

// Detection validation
constexpr auto OBSTACLE_RED_MIN_AREA = 300;
constexpr auto OBSTACLE_RED_MIN_HEIGHT = 21;
constexpr auto OBSTACLE_GREEN_MIN_AREA = 400;
constexpr auto OBSTACLE_GREEN_MIN_HEIGHT = 21;
constexpr auto OBSTACLE_CONFIRM_FRAMES = 2;
constexpr auto OBSTACLE_LOST_FRAMES = 3;
constexpr uint8_t OPEN_CORNER_CONFIRM_SAMPLES = 3;

// Desired obstacle positions inside the 320 px image
// Red must stay on the LEFT -> robot passes on the right
constexpr auto OBSTACLE_RED_TARGET_X = 120;

// Green must stay on the RIGHT -> robot passes on the left
constexpr auto OBSTACLE_GREEN_TARGET_X = 200;

// Camera steering
constexpr auto OBSTACLE_CAMERA_KP = 0.10f;
constexpr auto OBSTACLE_HEADING_KP = 0.30f;
constexpr auto OBSTACLE_MAX_STEERING = 20.0f;

// Start slowly while tuning
constexpr auto OBSTACLE_AVOID_SPEED = 160;
constexpr auto OBSTACLE_CRUISE_SPEED = 220;

// Continue around obstacle after camera loses it
constexpr auto OBSTACLE_PASS_STEERING = 8.0f;
constexpr auto OBSTACLE_PASS_DISTANCE_MM = 60.0f;

// Return to original course heading
constexpr auto OBSTACLE_RECOVER_KP = 1.0f;
constexpr auto OBSTACLE_RECOVER_MAX_STEERING = 18.0f;
constexpr auto OBSTACLE_RECOVER_SPEED = 160;
constexpr auto OBSTACLE_RECOVER_TOLERANCE_DEG = 3.0f;

// Prevent detecting the same obstacle immediately again
constexpr auto OBSTACLE_REARM_DISTANCE_MM = 150.0f;

// Side-barrier protection while camera avoidance owns the steering.
constexpr auto OBSTACLE_WALL_GUARD_DISTANCE_MM = 170.0f;
constexpr auto OBSTACLE_WALL_GUARD_KP = 0.12f;
constexpr auto OBSTACLE_WALL_GUARD_MAX_STEERING = 18.0f;

// A block must extend this far down in the logical 320x240 image.
constexpr auto OBSTACLE_MIN_BOTTOM_Y = 100;
// A real 100 mm pillar intersects the obstacle ROI's upper boundary throughout
// the acquisition range. Floor-line fragments begin substantially lower in
// the image; requiring the blob to reach near the ROI top rejects them even
// when lighting shifts orange pixels into the red hue range.
constexpr auto OBSTACLE_MAX_TOP_Y = 100;

// Upright WRO obstacle blocks are taller than they are wide. This rejects
// broad greenish background regions and blobs merged with the horizon.
constexpr auto OBSTACLE_MAX_WIDTH_HEIGHT_RATIO = 1.25f;

// A new manoeuvre may only start from a reasonably complete pillar. Once a
// pillar is confirmed, the separate tracking rules remain tolerant at edges.
constexpr auto OBSTACLE_START_MIN_X = 30;
constexpr auto OBSTACLE_START_MAX_X = 290;
constexpr auto OBSTACLE_MAX_START_WIDTH = 80;
constexpr auto OBSTACLE_MAX_START_HEIGHT = 120;

// After every 90 degree corner, let the normal gyro controller remove turn
// overshoot before camera avoidance can take steering priority.
constexpr auto OBSTACLE_CORNER_SETTLE_MIN_DISTANCE_MM = 100.0f;
constexpr auto OBSTACLE_CORNER_SETTLE_MAX_DISTANCE_MM = 300.0f;
constexpr auto OBSTACLE_CORNER_SETTLE_HEADING_DEG = 4.0f;
constexpr auto OBSTACLE_CORNER_EARLY_TAKEOVER_HEADING_DEG = 6.0f;

// Measured Ackermann geometry:
// servo -40 -> R ~= 154.6 mm -> 90 degree arc ~= 242.8 mm
// servo +40 -> R ~= 158.8 mm -> 90 degree arc ~= 249.4 mm
constexpr auto OBSTACLE_CORNER_STEERING = 40.0f;
constexpr auto OBSTACLE_FIRST_LAP_CORNER_SPEED = 140.0f;
constexpr auto OBSTACLE_LATER_LAP_CORNER_SPEED = 180.0f;
constexpr auto OBSTACLE_FIRST_LAP_REVERSE_SPEED = 80.0f;
constexpr auto OBSTACLE_FIRST_LAP_REVERSE_STEERING = 40.0f;
constexpr auto OBSTACLE_FIRST_LAP_REVERSE_MIN_MM = 25.0f;
constexpr auto OBSTACLE_FIRST_LAP_REVERSE_MAX_MM = 55.0f;
constexpr auto OBSTACLE_FIRST_LAP_REVERSE_TOLERANCE_DEG = 2.0f;
constexpr auto OBSTACLE_FIRST_LAP_ALIGN_SPEED = 120.0f;
constexpr auto OBSTACLE_FIRST_LAP_ALIGN_MIN_MM = 120.0f;
constexpr auto OBSTACLE_FIRST_LAP_ALIGN_MAX_MM = 260.0f;
constexpr auto OBSTACLE_FIRST_LAP_ALIGN_TOLERANCE_DEG = 3.0f;

// Learned-lap lane planner. Official signs sit roughly 400 mm from a wall;
// a 190 mm vehicle-centre offset leaves useful clearance on both sides.
constexpr auto OBSTACLE_PLANNED_LANE_WALL_MM = 190.0f;
constexpr auto OBSTACLE_PLANNED_SWITCH_AFTER_MM = 200.0f;
constexpr auto OBSTACLE_PLANNED_NEXT_SECTION_MM = 180.0f;
constexpr auto OBSTACLE_START_SECTION_SWITCH_MM = 500.0f;
constexpr auto OBSTACLE_START_SECTION_NEXT_PLAN_MM = 750.0f;

// Known-geometry Pure Pursuit planner. The official obstacle field has a
// 3000 mm inner track square, 1000 mm corridors, 1000 mm straight sections,
// and 500 mm-radius centreline corners. Candidate seats lie at the three
// section stations and 100 mm to either side of the corridor centreline.
constexpr auto OBSTACLE_PATH_SAMPLE_MM = 50.0f;
constexpr auto OBSTACLE_STRAIGHT_LENGTH_MM = 1000.0f;
constexpr auto OBSTACLE_CORNER_RADIUS_MM = 500.0f;
constexpr auto OBSTACLE_SEAT_LATERAL_MM = 100.0f;
constexpr auto OBSTACLE_CORRIDOR_HALF_WIDTH_MM = 500.0f;
constexpr auto OBSTACLE_WHEELBASE_MM = 100.0f;
constexpr auto OBSTACLE_MAX_PATH_WAYPOINTS = 192;
constexpr auto OBSTACLE_SEAT_COUNT = 24;

// Canonical field frame used by production obstacle navigation. The origin is
// the geometric centre of the rounded-square centreline, +X points along the
// south straight toward its CCW corner, and +Y points toward the north side.
// With the known geometry the centreline tangencies and outer arc extrema fit
// inside +/-1000 mm on both axes.
constexpr auto OBSTACLE_FIELD_ORIGIN_X_MM = 0.0f;
constexpr auto OBSTACLE_FIELD_ORIGIN_Y_MM = 0.0f;
constexpr auto OBSTACLE_CENTERLINE_HALF_EXTENT_MM =
    OBSTACLE_STRAIGHT_LENGTH_MM * 0.5f +
    OBSTACLE_CORNER_RADIUS_MM;

// If the parking exit is disabled, the code cannot infer which way around
// the field the car was placed. Set +1 for left/CCW corners or -1 for
// right/CW corners before that test. A parking-lot start infers this from the
// measured outer-wall side and does not use the fallback.
constexpr int8_t OBSTACLE_DEFAULT_TURN_SIGN = 1;

constexpr auto OBSTACLE_LOOKAHEAD_MIN_MM = 150.0f;
constexpr auto OBSTACLE_LOOKAHEAD_MAX_MM = 330.0f;
constexpr auto OBSTACLE_LOOKAHEAD_CORNER_SCALE = 0.65f;
constexpr auto OBSTACLE_PATH_PROGRESS_WINDOW = 12;
constexpr auto OBSTACLE_MAX_PURSUIT_STEERING_DEG = 42.0f;
constexpr auto OBSTACLE_PATH_MIN_SPEED = 135.0f;
constexpr auto OBSTACLE_PATH_MAX_SPEED = 260.0f;
constexpr auto OBSTACLE_CURVATURE_SPEED_GAIN = 950.0f;

constexpr auto OBSTACLE_LAP1_CLEARANCE_MM = 200.0f;
constexpr auto OBSTACLE_OPTIMIZED_CLEARANCE_MM = 160.0f;
constexpr auto OBSTACLE_PATH_TAPER_WAYPOINTS = 6;
constexpr auto OBSTACLE_PATH_SMOOTH_RADIUS = 1;
constexpr auto OBSTACLE_SEAT_SNAP_RADIUS_MM = 140.0f;
constexpr auto OBSTACLE_SEAT_CONFIRM_VOTES = 2;
constexpr uint32_t OBSTACLE_SEAT_VOTE_WINDOW_MS = 400;

// First-lap discovery must resolve a station before the car reaches it. Empty
// stations require several camera frames in which both legal seats were well
// inside the calibrated field of view. If a station is still unknown near the
// decision point, slow down and finally hold rather than risk a wrong pass.
constexpr auto OBSTACLE_DISCOVERY_VIEW_MIN_MM = 260.0f;
constexpr auto OBSTACLE_DISCOVERY_VIEW_MAX_MM = 600.0f;
constexpr auto OBSTACLE_DISCOVERY_FOV_FRACTION = 0.40f;
// Three consecutive usable views reject a one-frame camera dropout while
// allowing an empty seat to resolve within the short corner-viewing window.
// Pillars still use their separate two-vote geometry and colour confirmation.
constexpr auto OBSTACLE_DISCOVERY_CLEAR_FRAMES = 3;
constexpr auto OBSTACLE_DISCOVERY_SLOW_DISTANCE_MM = 420.0f;
constexpr auto OBSTACLE_DISCOVERY_HOLD_DISTANCE_MM = 170.0f;
// The drivetrain oscillates below its continuous controllable range at
// 90 mm/s. Use the already validated test speed while approaching an unresolved
// station; the final hold remains available if perception cannot resolve it.
constexpr auto OBSTACLE_DISCOVERY_SPEED_MM_S = 175.0f;

// BO462 calibration values. Camera coordinates use the same robot frame as the
// ToF mounts: +X forward, +Y left, with the rear-axle midpoint as the origin.
// Horizontal pinhole calibration from surveyed +/-100 mm pillar offsets at
// 400 and 600 mm ray range. The resulting usable 320 px field is about 42 deg.
// Provisional full-sensor values. Recalibrate physically before driving.
constexpr auto OBSTACLE_CAMERA_HORIZONTAL_FOV_DEG = 65.0f;
constexpr auto OBSTACLE_CAMERA_PRINCIPAL_X_PX = 156.9f;
constexpr auto OBSTACLE_CAMERA_FOCAL_X_PX = 250.5f;
// Measure both values along the robot centreline whenever the camera or body
// changes. ROBOT_FRONT_FROM_REAR_AXLE_MM is the plane touched by the near face
// of the pillar at the start of a camdrive calibration.
constexpr auto ROBOT_FRONT_FROM_REAR_AXLE_MM = 130.0f;
constexpr auto OBSTACLE_CAMERA_FROM_REAR_AXLE_MM = 125.0f;
constexpr auto OBSTACLE_CAMERA_LOCAL_X_MM =
    OBSTACLE_CAMERA_FROM_REAR_AXLE_MM;
constexpr auto OBSTACLE_CAMERA_LOCAL_Y_MM = 0.0f;

// The official pillar distances are measured horizontally from the camera to
// the foot of the block. Its top is clipped by the obstacle ROI at longer
// ranges, so height is not a reliable range input. Ground-plane calibration
// Stationary red and green measurements from log_20 have maxY=150 at 400 mm
// and maxY=120 at 600 mm, giving
// distance = scale / (maxY - horizonY).
constexpr auto OBSTACLE_CAMERA_GROUND_HORIZON_Y = 84.0f;
constexpr auto OBSTACLE_CAMERA_GROUND_RANGE_SCALE_MM_PX = 21600.0f;
// The wide-angle lens moves the apparent ground contact lower toward either
// image edge. Remove that measured V-shaped bias before applying the centreline
// ground-plane fit. Units are vertical foot pixels per horizontal pixel.
constexpr auto OBSTACLE_CAMERA_FOOT_EDGE_SLOPE = 0.11f;

// Automated camera ground-plane calibration (serial: camdrive [reverse_mm]).
// The pillar starts touching ROBOT_FRONT_FROM_REAR_AXLE_MM; encoder travel and
// the camera offset above provide camera-to-pillar ground truth.
constexpr auto CAMERA_DRIVE_CAL_SPEED_MMS = 80;
constexpr auto CAMERA_DRIVE_CAL_DEFAULT_TRAVEL_MM = 1000.0f;
constexpr auto CAMERA_DRIVE_CAL_MAX_TRAVEL_MM = 1800.0f;
constexpr auto CAMERA_DRIVE_CAL_TIMEOUT_MS = 45000UL;
constexpr auto CAMERA_DRIVE_CAL_PRINT_INTERVAL_MS = 250UL;
constexpr auto CAMERA_DRIVE_CAL_MAX_HEADING_ERROR_DEG = 6.0f;
constexpr auto CAMERA_DRIVE_CAL_MAX_STEERING_DEG = 15.0f;
constexpr auto CAMERA_DRIVE_CAL_COLOR_DELAY_MM = 180.0f;
constexpr auto CAMERA_DRIVE_CAL_COLOR_CONFIRM_FRAMES = 3U;
constexpr auto CAMERA_DRIVE_CAL_GEOMETRY_CONFIRM_FRAMES = 5U;
// Stop periodically so each regression point is an average without motion
// blur, steering vibration, or unequal weighting from camera frame timing.
constexpr auto CAMERA_DRIVE_CAL_FIRST_CHECKPOINT_MM = 300.0f;
constexpr auto CAMERA_DRIVE_CAL_CHECKPOINT_INTERVAL_MM = 100.0f;
constexpr auto CAMERA_DRIVE_CAL_CHECKPOINT_SETTLE_MS = 700UL;
constexpr auto CAMERA_DRIVE_CAL_CHECKPOINT_SAMPLE_TIMEOUT_MS = 2500UL;
constexpr auto CAMERA_DRIVE_CAL_CHECKPOINT_SAMPLES = 10U;
constexpr auto CAMERA_DRIVE_CAL_CHECKPOINT_MIN_SAMPLES = 6U;
constexpr auto CAMERA_DRIVE_CAL_MIN_RANGE_MM = 120.0f;
constexpr auto CAMERA_DRIVE_CAL_MAX_RANGE_MM = 1600.0f;
constexpr auto CAMERA_DRIVE_CAL_MIN_SAMPLES = 5U;
// Acquire the close pillar low in the image, then require its foot to move
// upward as distance increases. This rejects unrelated colored components.
constexpr auto CAMERA_DRIVE_CAL_ACQUIRE_MIN_FOOT_Y = 160;
constexpr auto CAMERA_DRIVE_CAL_FOOT_Y_TOLERANCE_PX = 6;
constexpr auto CAMERA_DRIVE_CAL_MAX_CENTER_ERROR_PX = 90;
constexpr auto CAMERA_DRIVE_CAL_MIN_FIT_SCALE_MM_PX = 15000.0f;
constexpr auto CAMERA_DRIVE_CAL_MAX_FIT_SCALE_MM_PX = 60000.0f;
constexpr auto CAMERA_DRIVE_CAL_MIN_FIT_HORIZON_Y = -20.0f;
constexpr auto CAMERA_DRIVE_CAL_MAX_FIT_HORIZON_Y = 150.0f;
constexpr auto CAMERA_DRIVE_CAL_MAX_FIT_RMSE_PX = 5.0f;

// Retained as a diagnostic/fallback for a blob whose foot is above the
// calibrated horizon. Normal obstacle range uses the ground-plane model.
constexpr auto OBSTACLE_CAMERA_FOCAL_LENGTH_PX = 277.0f;
constexpr auto OBSTACLE_PILLAR_HEIGHT_MM = 100.0f;
constexpr auto OBSTACLE_EDGE_CLIPPED_RANGE_MM = 170.0f;
// A corner-exit inside seat is about 54 degrees off the nominal camera axis
// while still in the calibrated range. Begin a Pure-Pursuit target scan early
// enough to rotate the chassis/camera toward it before reaching the tangent.
constexpr auto OBSTACLE_LOOK_START_MM = 700.0f;
constexpr auto OBSTACLE_LOOK_FULL_NUDGE_MM = 550.0f;
constexpr auto OBSTACLE_LOOK_END_MM = 80.0f;
constexpr auto OBSTACLE_LOOK_TARGET_GAIN = 0.75f;
constexpr auto OBSTACLE_LOOK_TARGET_BEARING_DEG = 8.0f;
constexpr auto OBSTACLE_LOOK_MAX_TARGET_NUDGE_DEG = 35.0f;
constexpr auto OBSTACLE_LOOK_NUDGE_SLEW_DEG_S = 60.0f;

// ToF locations in the robot frame: +X forward, +Y left. The documented
// 120 mm body width places the side-facing sensor apertures about 60 mm from
// the pose origin; update DX after measuring the final printed mounts.
constexpr auto OBSTACLE_TOF_LEFT_LOCAL_X_MM = 40.0f;
constexpr auto OBSTACLE_TOF_LEFT_LOCAL_Y_MM = 35.0f;
constexpr auto OBSTACLE_TOF_RIGHT_LOCAL_X_MM = 40.0f;
constexpr auto OBSTACLE_TOF_RIGHT_LOCAL_Y_MM = -35.0f;
constexpr auto OBSTACLE_TOF_CORRECTION_MAX_RANGE_MM = 500.0f;
constexpr auto OBSTACLE_TOF_CORRECTION_GAIN = 0.18f;
constexpr auto OBSTACLE_TOF_CORRECTION_MAX_STEP_MM = 12.0f;
constexpr auto OBSTACLE_CORNER_GATE_BEFORE_MM = 120.0f;
constexpr auto OBSTACLE_CORNER_GATE_AFTER_MM = 280.0f;

// Empty-track Pure Pursuit test mode (serial: X1 left/CCW, X-1 right/CW).
// Vision steering and ToF pose correction are disabled during this test so
// it measures only path anchoring, direction, odometry and path tracking.
constexpr auto OBSTACLE_PATH_TEST_MAX_SPEED_MM_S = 175.0f;
constexpr auto OBSTACLE_PATH_TEST_TIMEOUT_MS = 75000UL;
// Valid only for the empty-track X test. The live Y test cannot distinguish a
// legal pillar from a wall using one raw side-ToF range.
constexpr auto OBSTACLE_PATH_TEST_WALL_STOP_MM = 120.0f;
constexpr auto OBSTACLE_PATH_TEST_ABORT_CROSS_TRACK_MM = 300.0f;
constexpr auto OBSTACLE_PATH_TEST_PASS_CROSS_TRACK_MM = 180.0f;
constexpr auto OBSTACLE_PATH_TEST_PASS_POSITION_MM = 200.0f;
constexpr auto OBSTACLE_PATH_TEST_PASS_HEADING_DEG = 12.0f;
constexpr auto OBSTACLE_PATH_TEST_TELEMETRY_MS = 500UL;
constexpr auto OBSTACLE_LIVE_TEST_TIMEOUT_MS = 120000UL;
constexpr auto OBSTACLE_LIVE_TEST_TELEMETRY_MS = 200UL;



// ==========================================
// CHALLENGE MODE
// ==========================================

// Default after power-on/reset. Serial mode letters override this only in
// RAM for the current power cycle. If the physical enable switch is LOW,
// the selected mode remains pending until the switch is enabled.
// Common choices:
//   MODE_OPEN_CHALLENGE, MODE_OBSTACLE_CHALLENGE,
//   MODE_TURN_RADIUS_CAL, MODE_SERVO_CENTER_CAL,
//   MODE_PID_AUTOTUNE, MODE_MOTOR_MIN_CAL
// Stationary safety boot for full-FOV asynchronous camera development.
#define CAMERA_ASYNC_STATIONARY_AUTOSTART true
#define CAMERA_ASYNC_CAPTURE_ENABLED true
#define STARTUP_ROBOT_MODE MODE_CAMERA_CALIBRATION
