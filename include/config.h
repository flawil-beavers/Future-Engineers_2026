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
// The game rotation vector is requested at 100 Hz. Missing 20 consecutive
// samples indicates a sensor-side stall, but a main-loop pause must not be
// mistaken for one: the host was not polling the BNO085 during that pause.
constexpr uint32_t GYRO_REPORT_INTERVAL_US = 10000UL;
constexpr uint32_t GYRO_REPORT_TIMEOUT_MS = 200UL;
// Allow a hardware-reset request time to boot and announce its reset before
// retrying. Repeated 200 ms resets could otherwise keep a slow boot in a loop.
constexpr uint32_t GYRO_RESET_RECOVERY_TIMEOUT_MS = 1000UL;

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
constexpr auto DRIVE_WHEEL_DIAMETER_MM = 43.2f;
constexpr auto COUNTER_TO_MM = (PI * DRIVE_WHEEL_DIAMETER_MM / ENCODER_COUNTS_PER_WHEEL_REV); // mm per encoder count

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
// The fast detector above catches a hard stall only after the controller has
// reached virtually maximum duty. Also stop when a real motion command makes
// less than 10 mm of encoder progress in one second, regardless of duty. Both
// target and ramped profile must be active, so acceleration startup, planned
// braking, perception holds, and position holding do not arm this watchdog.
constexpr float STALL_COMMAND_MIN_SPEED_MMS = 80.0f;
constexpr float STALL_NO_PROGRESS_MIN_DISTANCE_MM = 10.0f;
constexpr unsigned long STALL_NO_PROGRESS_WINDOW_US = 1000000UL;
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
// GetMeasurementDataReady() itself is an I2C transaction. Polling both 30 ms
// sensors on every sub-millisecond control iteration wastes bus and CPU time;
// 1 ms polling retains at most 1 ms of fresh-sample service latency.
constexpr auto TOF_READY_POLL_INTERVAL_US = 1000UL;
constexpr auto TOF_MAX_RELIABLE_DISTANCE_MM = 600.0f; // Max distance for reliable wall detection (mm)
// The rear position deliberately reuses the previously damaged Adafruit
// module. Physical black-wall testing found it accurate only through about
// 370 mm, so M4 must reject longer control distances even if hardware status
// and signal-quality checks pass.
constexpr auto REAR_TOF_MAX_RELIABLE_DISTANCE_MM = 370.0f;
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
constexpr bool OBSTACLE_PARKING_EXIT_TEST_ONLY = true;

// The rules define the parking-space length as 1.5 times the robot length.
// The mechanical length is intentionally left unset while the chassis design
// is still being decided. A value of 0 means "unknown" and prevents logs from
// presenting the current prototype setup as final competition geometry.
constexpr auto OBSTACLE_FINAL_ROBOT_LENGTH_MM = 0.0f;
constexpr auto OBSTACLE_PARKING_LENGTH_FACTOR = 1.5f;
constexpr auto OBSTACLE_PARKING_WIDTH_MM = 200.0f;
static_assert(
    OBSTACLE_FINAL_ROBOT_LENGTH_MM >= 0.0f,
    "Robot length must be zero (unset) or a positive millimetre value");
static_assert(
    OBSTACLE_PARKING_LENGTH_FACTOR > 1.0f,
    "Parking length factor must leave space beyond the robot length");

// Prototype-only multi-point exit. The former forward-first two-arc exit hit
// the front parking marker in the proportional gap and must not be restored.
// These values come from the current 165 mm footprint model and must be
// revalidated after any chassis, wheel-envelope, or steering change.
constexpr auto OBSTACLE_PARKING_EXIT_STEERING = 50;
constexpr auto OBSTACLE_PARKING_EXIT_SPEED = 80;
constexpr auto OBSTACLE_PARKING_EXIT_STEER_SETTLE_MS = 400;
constexpr auto OBSTACLE_PARKING_EXIT_HOLD_BRAKE_MS = 300;
constexpr auto OBSTACLE_PARKING_EXIT_SEGMENT_COUNT = 5;

// Safety gate for powered development. A value below SEGMENT_COUNT stops and
// saves the log after that many segments. Increase it only after reviewing the
// preceding stage's actual travel and physical clearance.
constexpr auto OBSTACLE_PARKING_EXIT_TEST_SEGMENT_LIMIT = 5;
static_assert(
    OBSTACLE_PARKING_EXIT_TEST_SEGMENT_LIMIT >= 1 &&
        OBSTACLE_PARKING_EXIT_TEST_SEGMENT_LIMIT <=
            OBSTACLE_PARKING_EXIT_SEGMENT_COUNT,
    "Parking exit segment limit must select one or more valid segments");

constexpr auto OBSTACLE_PARKING_EXIT_PROTOTYPE_LENGTH_MM = 165.0f;
constexpr auto OBSTACLE_PARKING_EXIT_PROTOTYPE_FRONT_MM = 125.0f;
constexpr auto OBSTACLE_PARKING_EXIT_PROTOTYPE_REAR_MM = 40.0f;
constexpr auto OBSTACLE_PARKING_EXIT_PROTOTYPE_WIDTH_MM = 135.0f;
constexpr auto OBSTACLE_PARKING_EXIT_PROTOTYPE_GAP_MM = 247.5f;
constexpr auto OBSTACLE_PARKING_EXIT_START_REAR_CLEARANCE_MM = 50.0f;
constexpr auto OBSTACLE_PARKING_EXIT_FINAL_ALIGN_MIN_MM = 120.0f;
constexpr auto OBSTACLE_PARKING_EXIT_FINAL_ALIGN_MODEL_MM = 140.0f;
constexpr auto OBSTACLE_PARKING_EXIT_FINAL_ALIGN_MAX_MM = 180.0f;
constexpr auto OBSTACLE_PARKING_EXIT_FINAL_HEADING_TOLERANCE_DEG = 2.0f;
// A short side-ToF return after alignment is expected to be the exact 200 mm
// open end of the adjacent magenta parking piece, not the more distant outer
// field wall. Both mirrored exits have validated this geometry. Apply it only
// when the estimated sensor beam lies over the expected 20 mm magenta piece;
// the tolerance matches the measured parking-piece placement accuracy.
constexpr auto OBSTACLE_PARKING_EXIT_TOF_REFERENCE_MAX_MM = 180.0f;
constexpr auto OBSTACLE_PARKING_EXIT_BEAM_X_TOLERANCE_MM = 5.0f;
// ST documents an approximately 22-degree detection volume at a 100 mm
// target (18 degrees at 1000 mm). Parking-end references are only 40-70 mm
// away, so use the documented near-target value for the horizontal footprint.
constexpr auto OBSTACLE_PARKING_EXIT_TOF_DETECTION_FOV_DEG = 22.0f;
static_assert(
    OBSTACLE_PARKING_EXIT_TOF_DETECTION_FOV_DEG > 0.0f &&
        OBSTACLE_PARKING_EXIT_TOF_DETECTION_FOV_DEG < 90.0f,
    "Parking-exit ToF detection FoV must be a plausible full angle");
// After the five validated manoeuvre segments, reverse straight until the
// outer-wall-side ToF crosses the opposite edge of the magenta piece and sees
// the black wall. Reversing leaves more approach distance for starting-section
// discovery. This isolated test-stage motion is bounded and cannot start the
// lap until its swept path has also passed the physical test.
constexpr bool OBSTACLE_PARKING_EXIT_EDGE_LOCALIZATION_ENABLED = true;
constexpr auto OBSTACLE_PARKING_EXIT_EDGE_LOCALIZATION_SPEED = 60;
constexpr int8_t OBSTACLE_PARKING_EXIT_EDGE_LOCALIZATION_DIRECTION = -1;
constexpr auto OBSTACLE_PARKING_EXIT_EDGE_LOCALIZATION_MAX_MM = 60.0f;
constexpr auto OBSTACLE_PARKING_EXIT_EDGE_LOCALIZATION_SETTLE_MS = 200;
constexpr auto OBSTACLE_PARKING_EXIT_WALL_REFERENCE_MIN_MM = 190.0f;
constexpr auto OBSTACLE_PARKING_EXIT_WALL_REFERENCE_MAX_MM = 400.0f;
constexpr auto OBSTACLE_PARKING_EXIT_WALL_CONFIRM_FRAMES = 3;
constexpr auto OBSTACLE_PARKING_EXIT_MARKER_RANGE_MARGIN_MM = 50.0f;
constexpr auto OBSTACLE_PARKING_EXIT_WALL_RANGE_RESIDUAL_MM = 25.0f;
constexpr auto OBSTACLE_PARKING_EXIT_MAX_X_CORRECTION_MM = 25.0f;
constexpr auto OBSTACLE_PARKING_EXIT_MAX_Y_CORRECTION_MM = 25.0f;
static_assert(
    OBSTACLE_PARKING_EXIT_EDGE_LOCALIZATION_DIRECTION == -1 ||
        OBSTACLE_PARKING_EXIT_EDGE_LOCALIZATION_DIRECTION == 1,
    "Parking-edge localization direction must be reverse or forward");

// Test-only Pure-Pursuit discovery movement after the reverse edge reference.
// The parking section's outer-row seats are known empty by rule; this path
// rotates the forward camera toward the first upcoming inner-row seat. The
// fixed parking position is asymmetric along the straight, so the two travel
// directions use different field-x positions before the mirrored scan arc.
constexpr bool OBSTACLE_PARKING_ENTRY_DISCOVERY_ENABLED = true;
constexpr bool OBSTACLE_PARKING_ENTRY_DISCOVERY_TEST_ONLY = true;
constexpr auto OBSTACLE_PARKING_ENTRY_CCW_ARC_START_X_MM = 60.0f;
constexpr auto OBSTACLE_PARKING_ENTRY_CW_ARC_START_X_MM = 520.0f;
constexpr auto OBSTACLE_PARKING_ENTRY_SCAN_ARC_MM = 40.0f;
constexpr auto OBSTACLE_PARKING_ENTRY_SCAN_RADIUS_MM = 109.0f;
constexpr auto OBSTACLE_PARKING_ENTRY_SPEED_MM_S = 60.0f;
constexpr auto OBSTACLE_PARKING_ENTRY_LOOKAHEAD_MM = 70.0f;
constexpr auto OBSTACLE_PARKING_ENTRY_FINISH_TOLERANCE_MM = 18.0f;
constexpr auto OBSTACLE_PARKING_ENTRY_FINISH_HEADING_DEG = 5.0f;
constexpr auto OBSTACLE_PARKING_ENTRY_MAX_OVERRUN_MM = 20.0f;
constexpr unsigned long OBSTACLE_PARKING_ENTRY_OBSERVE_MS = 1200UL;
constexpr auto OBSTACLE_PARKING_ENTRY_MAX_WAYPOINTS = 32;
static_assert(
    OBSTACLE_PARKING_EXIT_FINAL_ALIGN_MIN_MM <
            OBSTACLE_PARKING_EXIT_FINAL_ALIGN_MODEL_MM &&
        OBSTACLE_PARKING_EXIT_FINAL_ALIGN_MODEL_MM <
            OBSTACLE_PARKING_EXIT_FINAL_ALIGN_MAX_MM,
    "Final parking alignment bounds must surround the modeled distance");
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

// Rules Figure 4: within the selected starting section, the right magenta
// parking limitation is fixed immediately to the left of the right-hand
// dotted section boundary. Only the left limitation moves to create the
// 1.5*robot-length gap. The canonical frame treats the randomly selected
// starting section as the south section, so that dotted boundary is x=+500.
constexpr auto OBSTACLE_PARKING_FIXED_DOTTED_LINE_X_MM =
    OBSTACLE_STRAIGHT_LENGTH_MM * 0.5f;
constexpr auto OBSTACLE_PARKING_LIMIT_THICKNESS_MM = 20.0f;
constexpr auto OBSTACLE_PARKING_FIXED_INNER_FACE_X_MM =
    OBSTACLE_PARKING_FIXED_DOTTED_LINE_X_MM -
    OBSTACLE_PARKING_LIMIT_THICKNESS_MM;
constexpr auto OBSTACLE_PARKING_OPEN_END_FIELD_Y_MM =
    -OBSTACLE_CENTERLINE_HALF_EXTENT_MM -
    OBSTACLE_CORRIDOR_HALF_WIDTH_MM +
    OBSTACLE_PARKING_WIDTH_MM;
constexpr auto OBSTACLE_SOUTH_OUTER_WALL_Y_MM =
    -OBSTACLE_CENTERLINE_HALF_EXTENT_MM -
    OBSTACLE_CORRIDOR_HALF_WIDTH_MM;

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

// Radius-1 smoothing reduces the requested displacement. 260 mm adds about
// 20 mm in the approach where the green-left rear wheel touched in log_54 and
// about 27 mm at the peak, while retaining estimated body-to-wall margin.
constexpr auto OBSTACLE_LAP1_CLEARANCE_MM = 260.0f;
// Safe lap-1 outer-extreme route while the following station is unresolved,
// preserving the option of a rare opposing adjacent pillar. Logs 101-104
// validated its 230 mm short plateau.
constexpr auto OBSTACLE_OUTER_SAFE_CLEARANCE_MM = 230.0f;
// Log_99 still approached the CCW red pillar within about 10 mm physically
// (19 mm by side ToF) even though confirmation occurred before the avoidance
// taper. Reach peak displacement one 50 mm waypoint earlier so the front
// envelope is protected before the rear-axle centre reaches the pillar.
constexpr auto OBSTACLE_OUTER_SAFE_APPROACH_LEAD_WAYPOINTS = 1;
// Log_92 still measured only 11 mm of rear-wheel clearance: the closest point
// occurred just after the rear axle passed the pillar. Keep the provisional
// route at its peak for one more 50 mm waypoint before beginning the exit
// taper. Together these form a short plateau around the complete vehicle pass.
// Confirmed adjacent-pair and moderate 260 mm paths retain their old shapes.
constexpr auto OBSTACLE_OUTER_SAFE_EXIT_HOLD_WAYPOINTS = 1;
// Log_106 proved that reusing the 230 mm plateau on optimized laps prevented
// contact but left only about 10 mm physical front-wheel clearance to the
// outer wall, while retaining 108-138 mm ToF-estimated pillar clearance.
// Move only isolated optimized outer routes 20 mm inward. This value happens
// to equal the adjacent second clearance, so route construction must pass an
// explicit plateau flag rather than infer the shape from clearance alone.
constexpr auto OBSTACLE_OPTIMIZED_OUTER_CLEARANCE_MM = 210.0f;
// The first member of the worst adjacent outer-seat reversal needs more than
// the former 160 mm reduced value. Log_84 measured -6 mm ToF wheel clearance
// at 160 mm while about 242 mm remained to the opposite wall. Request 200 mm to
// add roughly 40 mm of physical placement tolerance. Ordinary first-lap
// routes retain their proven 260 mm clearance.
constexpr auto OBSTACLE_EXTREME_ADJACENT_CLEARANCE_MM = 200.0f;
// The second member needs extra rear-wheel margin. Log_82 measured a -16 mm
// green wheel-envelope estimate while about 207 mm remained to the outer wall.
constexpr auto OBSTACLE_EXTREME_ADJACENT_SECOND_CLEARANCE_MM = 210.0f;
// Do not aim at the following station while Pure Pursuit is still completing
// the extreme adjacent transition. At release, that station remains about
// 400 mm ahead and inside the existing perception/hold window.
constexpr auto OBSTACLE_EXTREME_ADJACENT_RELEASE_MM = 100.0f;
// A newly confirmed second obstacle must not reshape the route while the rear
// of the robot is still clearing the first obstacle.  At 100 mm past the first
// seat, the conservative robot envelope is clear and 400 mm of approach to the
// adjacent station remains.
constexpr auto OBSTACLE_EXTREME_ADJACENT_INJECTION_DELAY_MM = 100.0f;
// Begin obstacle displacement 100 mm earlier than the former 6-point taper so
// Pure Pursuit tracks outward before the closest approach instead of cutting
// inside the green-left path as observed in log_55.
constexpr auto OBSTACLE_PATH_TAPER_WAYPOINTS = 8;
constexpr auto OBSTACLE_PATH_SMOOTH_RADIUS = 1;
constexpr auto OBSTACLE_SEAT_SNAP_RADIUS_MM = 140.0f;
constexpr auto OBSTACLE_SEAT_CONFIRM_VOTES = 2;
constexpr uint32_t OBSTACLE_SEAT_VOTE_WINDOW_MS = 400;

// First-lap discovery must resolve a station before the car reaches it. Empty
// stations require several camera frames in which both legal seats were well
// inside the calibrated field of view. If a station is still unknown near the
// decision point, slow down and finally hold rather than risk a wrong pass.
constexpr auto OBSTACLE_DISCOVERY_VIEW_MIN_MM = 230.0f;
constexpr auto OBSTACLE_DISCOVERY_VIEW_MAX_MM = 600.0f;
// Clear evidence is evaluated for each seat independently. Include the
// calibrated bearing uncertainty around the detected blob, and accept an
// overlapping blob as evidence of a clear nearer seat only when it is well
// behind that seat. Consecutive stations are 500 mm apart, so 180 mm remains
// conservative while separating the log_46 seat-3/seat-5 sight line.
constexpr auto OBSTACLE_DISCOVERY_BLOB_BEARING_MARGIN_DEG = 2.0f;
constexpr auto OBSTACLE_DISCOVERY_BEHIND_SEAT_MARGIN_MM = 180.0f;
// A complete official pillar remained visible and stable at +/-26.6 degrees
// in the full-sensor mode, with about 20 px clearance to the image edge. This
// remains the acquisition window. Empty-seat evidence uses a separate inward
// margin so NO_BLOB is never trusted at the acquisition edge. Log_89 showed
// that 3 degrees left no two-frame overlap. Log_100 then left the second empty
// seat within the 2-degree gate for only about one frame. A 1-degree margin
// still stays inside the validated +/-26.6-degree complete-pillar view and
// excludes the failed -28.8-degree acquisition case.
constexpr auto OBSTACLE_DISCOVERY_FOV_FRACTION = 0.42f;
constexpr auto OBSTACLE_DISCOVERY_CLEAR_FOV_MARGIN_DEG = 1.0f;
// Normal asynchronous full-FOV completion takes about 80 ms, so two
// consecutive usable views require at least about 160 ms while still rejecting
// a single-frame dropout. Pillars use their separate two-vote geometry and
// colour confirmation and can override an earlier clear observation.
constexpr auto OBSTACLE_DISCOVERY_CLEAR_FRAMES = 2;
constexpr auto OBSTACLE_DISCOVERY_SLOW_DISTANCE_MM = 420.0f;
// Stop early enough that an unresolved near-side pillar cannot overlap the
// chassis before the camera has produced a usable edge-of-view observation.
constexpr auto OBSTACLE_DISCOVERY_HOLD_DISTANCE_MM = 340.0f;
// Keep processing frames while stopped at the hold line. Log_93 obtained its
// first valid red vote in the same cycle that the former immediate abort ran;
// 400 ms covers several normal ~80 ms camera frames without allowing motion
// toward an unresolved station. The normal two-frame confirmation is retained.
constexpr unsigned long OBSTACLE_DISCOVERY_HOLD_GRACE_MS = 400UL;
// The drivetrain oscillates below its continuous controllable range at
// 90 mm/s. Use the already validated test speed while approaching an unresolved
// station; the final hold remains available if perception cannot resolve it.
constexpr auto OBSTACLE_DISCOVERY_SPEED_MM_S = 175.0f;

// BO462 calibration values. Camera coordinates use the same robot frame as the
// ToF mounts: +X forward, +Y left, with the rear-axle midpoint as the origin.
// Horizontal pinhole calibration from symmetric +/-200 mm offsets at 400 mm
// forward depth. The full-sensor centroids were x=39.9 and x=288.8 at
// +/-26.565 degrees. The GC2145 subsamples the full view directly to 320x240.
constexpr auto OBSTACLE_CAMERA_HORIZONTAL_FOV_DEG = 65.3f;
constexpr auto OBSTACLE_CAMERA_PRINCIPAL_X_PX = 164.4f;
constexpr auto OBSTACLE_CAMERA_FOCAL_X_PX = 248.9f;
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
// Full-sensor stationary red measurements have maxY=138 at 400 mm and
// maxY=118 at 600 mm, giving
// distance = scale / (maxY - horizonY).
constexpr auto OBSTACLE_CAMERA_GROUND_HORIZON_Y = 78.0f;
constexpr auto OBSTACLE_CAMERA_GROUND_RANGE_SCALE_MM_PX = 24000.0f;
// The former centred-crop mode needed a V-shaped off-axis correction. In the
// full-sensor mode, equal 400 mm forward-depth samples had maxY=136 (left) and
// maxY=138 (right), so the old 0.11 slope badly overcorrected their range.
constexpr auto OBSTACLE_CAMERA_FOOT_EDGE_SLOPE = 0.0f;

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
// A corner-exit inside seat is about 54 degrees off the nominal camera axis.
// Begin rotating the Pure-Pursuit target before the station enters the
// calibrated 600 mm camera range, and reach full response near that boundary,
// so the chassis/camera is already oriented when usable frames arrive.
constexpr auto OBSTACLE_LOOK_START_MM = 850.0f;
constexpr auto OBSTACLE_LOOK_FULL_NUDGE_MM = 650.0f;
constexpr auto OBSTACLE_LOOK_END_MM = 80.0f;
constexpr auto OBSTACLE_LOOK_TARGET_GAIN = 1.0f;
// Keep a complete pillar a few degrees inside the measured comfortable view.
// Discovery should use the wide camera view, not steer every seat to the old
// cropped-camera +/-8 degree target.
constexpr auto OBSTACLE_LOOK_FOV_MARGIN_DEG = 3.0f;
// Once one side has been cleared, bias the Pure-Pursuit target beyond the only
// unresolved seat's bearing. A 1.35 gain drives the logged 29.9-degree seat-6
// edge miss toward the target-nudge cap without widening camera acceptance.
// Both seats retain the gentler simultaneous wide-FOV rule while unresolved.
constexpr auto OBSTACLE_LOOK_SINGLE_SEAT_BEARING_DEG = 0.0f;
constexpr auto OBSTACLE_LOOK_SINGLE_SEAT_TARGET_GAIN = 1.35f;
// The green-right CCW detour left the next inside seat about 1.2 degrees beyond
// the validated view at 248 mm. Allow a short additional target rotation while
// retaining the existing slew limit and Pure Pursuit steering calculation.
// Log_87 reached a 38.9 degree target nudge but left the remaining seat at
// -28.8 degrees, outside the reliable complete-pillar region. Five additional
// degrees bring it inside the clear-evidence gate while Pure Pursuit remains
// the sole steering controller.
constexpr auto OBSTACLE_LOOK_MAX_TARGET_NUDGE_DEG = 45.0f;
constexpr auto OBSTACLE_LOOK_NUDGE_SLEW_DEG_S = 60.0f;

// ToF locations in the robot frame: +X forward, +Y left. The documented
// coordinates are the side-facing sensor aperture/measurement origins.
constexpr auto OBSTACLE_TOF_LEFT_LOCAL_X_MM = 40.0f;
constexpr auto OBSTACLE_TOF_LEFT_LOCAL_Y_MM = 35.0f;
constexpr auto OBSTACLE_TOF_RIGHT_LOCAL_X_MM = 40.0f;
constexpr auto OBSTACLE_TOF_RIGHT_LOCAL_Y_MM = -35.0f;
// Clearance diagnostic geometry. The 125 mm value is measured outside wheel
// to outside wheel. Steering can widen the wheel envelope by approximately
// 7.5 mm per side, so its conservative half-width is 125/2 + 7.5 = 70 mm.
// Each aperture is 35 mm from the centre, leaving a 70 - 35 = 35 mm inset.
// A side-facing ToF range minus this inset is a conservative geometry-based
// outer-wheel clearance estimate, not an exact swept-body minimum.
constexpr auto OBSTACLE_WHEEL_OUTSIDE_WIDTH_MM = 125.0f;
constexpr auto OBSTACLE_STEERING_ENVELOPE_PER_SIDE_MM = 7.5f;
constexpr auto OBSTACLE_MAX_WHEEL_HALF_WIDTH_MM =
    OBSTACLE_WHEEL_OUTSIDE_WIDTH_MM * 0.5f +
    OBSTACLE_STEERING_ENVELOPE_PER_SIDE_MM;
// Clearance logs model the complete safety envelope as a capsule.  Its radius
// is the maximum steered-wheel half-width.  The centre segment starts at the
// rear axle and ends 60 mm forward, so the capsule reaches the measured
// 130 mm front plane and conservatively reaches 70 mm behind the rear axle.
constexpr auto OBSTACLE_ROBOT_ENVELOPE_AXIS_FRONT_MM =
    ROBOT_FRONT_FROM_REAR_AXLE_MM - OBSTACLE_MAX_WHEEL_HALF_WIDTH_MM;
// Official 85 mm obstacle movement circle, expressed as a radius.
constexpr auto OBSTACLE_PILLAR_MOVEMENT_RADIUS_MM = 42.5f;
constexpr auto OBSTACLE_TOF_LEFT_TO_WHEEL_ENVELOPE_MM =
    OBSTACLE_MAX_WHEEL_HALF_WIDTH_MM - OBSTACLE_TOF_LEFT_LOCAL_Y_MM;
constexpr auto OBSTACLE_TOF_RIGHT_TO_WHEEL_ENVELOPE_MM =
    OBSTACLE_MAX_WHEEL_HALF_WIDTH_MM + OBSTACLE_TOF_RIGHT_LOCAL_Y_MM;
// Capture every fresh facing-side sample for 300 mm before and after the
// confirmed seat. Adjacent 500 mm stations intentionally overlap by 100 mm;
// their per-seat accumulators remain independent.
constexpr auto OBSTACLE_TOF_PASSAGE_BEFORE_MM = 300.0f;
constexpr auto OBSTACLE_TOF_PASSAGE_AFTER_MM = 300.0f;
constexpr auto OBSTACLE_TOF_CORRECTION_MAX_RANGE_MM = 500.0f;
// Reject a side return whose implied wall-position error is too large to be a
// credible localization error. This keeps the validated +/-100 mm correction
// cases but prevents a nearby pillar from being treated as the field wall.
constexpr auto OBSTACLE_TOF_CORRECTION_MAX_RESIDUAL_MM = 150.0f;
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
// Three-lap diagnostics must fit in the 128 KiB USB log buffer. Clearance
// events remain unthrottled; only the repetitive live status line is slower.
constexpr auto OBSTACLE_LIVE_TEST_THREE_LAP_TELEMETRY_MS = 600UL;



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
// Stationary safety boot for full-FOV asynchronous camera validation. This
// auto-start exception never enables a mode that can request motor movement.
#define CAMERA_ASYNC_STATIONARY_AUTOSTART true
#define CAMERA_ASYNC_CAPTURE_ENABLED true
// Keep snapshot async capture as a compile-time fallback while uninterrupted
// DCMI/DMA double-buffering is validated on the robot.
#define CAMERA_CONTINUOUS_CAPTURE_ENABLED true
// GC2145 timing profile. At 24 MHz XCLK the sensor's input divide-by-two bit
// preserves the proven 12 MHz internal timing. CAMERA_GC2145_PLL_DIVX4 can
// then be raised independently in controlled tests to increase frame rate.
#define CAMERA_SENSOR_XCLK_HZ 24000000UL
#define CAMERA_GC2145_PLL_MODE1 0x1F
#define CAMERA_GC2145_PLL_DIVX4 0x05
// Preserve the complete 1616 x 1208 readout and its horizontal timing. The
// stock profile adds 50 vertical blanking lines even though the measured 1080
// line exposure fits inside the active window; remove only those idle lines.
#define CAMERA_GC2145_HBLANK 0x011C
#define CAMERA_GC2145_VBLANK 0x0000
#define STARTUP_ROBOT_MODE MODE_CAMERA_CALIBRATION
