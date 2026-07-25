#include "obstacle.h"

#include "config.h"
#include "motor_control.h"
#include "sensors.h"
#include "wall_follower.h"
#include "course_map.h"
#include "logger.h"

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

static uint8_t oc_current_section = 0;
static uint8_t oc_current_lap = 0;
static float oc_section_start_distance = 0.0f;
static GyroFollowerState oc_last_navigation_state = GF_IDLE;
static bool oc_corner_settling = false;

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

static void updateCourseProgress()
{
    const GyroFollowerState navigationState =
        gyro_follower_get_state();

    // A new straight section starts only after the gyro-controlled 90 degree
    // turn has completed. This makes the section reference repeatable.
    if (
        oc_last_navigation_state == GF_TURNING &&
        navigationState == GF_FOLLOWING)
    {
        oc_current_section =
            (oc_current_section + 1) % COURSE_SECTION_COUNT;
        oc_current_lap =
            static_cast<uint8_t>(
                gyro_follower_get_turn_count() /
                COURSE_SECTION_COUNT);
        oc_section_start_distance = get_distance();
        oc_corner_settling = true;

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
// CAMERA UPDATE
// ============================================================

bool updateCameraVision()
{
    static uint32_t lastCameraUpdate = 0;

    if (
        millis() - lastCameraUpdate <
        OBSTACLE_CAMERA_INTERVAL_MS)
    {
        return false;
    }

    lastCameraUpdate = millis();

    if (!camera.capture())
    {
        Serial.println(
            "Camera capture failed.");

        return false;
    }

    return vision.update(
        camera.getBuffer(),
        camera.getWidth(),
        camera.getHeight());
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
        gyro_follower_get_turn_count());

    Serial.print(
        "Target heading: ");

    Serial.println(
        gyro_follower_get_target_heading());
}

// ============================================================
// CAMERA COLOUR CALIBRATION
// ============================================================

void printCameraCalibration()
{
    static uint32_t lastPrint = 0;

    if (
        millis() - lastPrint <
        500)
    {
        return;
    }

    lastPrint = millis();

    const uint16_t x =
        camera.getWidth() / 2;

    const uint16_t y =
        camera.getHeight() / 2;

    HSV hsv =
        vision.getHSVAt(
            camera.getBuffer(),
            camera.getWidth(),
            camera.getHeight(),
            x,
            y);

    Serial.print(
        "CENTER HSV -> H: ");

    Serial.print(hsv.h);

    Serial.print(
        " S: ");

    Serial.print(hsv.s);

    Serial.print(
        " V: ");

    Serial.println(hsv.v);
}

// ============================================================
// GET LARGEST OBSTACLE
// ============================================================

const Blob *getLargestObstacle()
{
    const VisionResult &v =
        vision.getResult();

    if (
        !v.red.found &&
        !v.green.found)
    {
        return nullptr;
    }

    if (
        v.red.found &&
        !v.green.found)
    {
        return &v.red;
    }

    if (
        v.green.found &&
        !v.red.found)
    {
        return &v.green;
    }

    // Prefer the object nearest the bottom of the image. It is normally the
    // next block, while blob area varies more strongly with segmentation.
    if (v.red.maxY != v.green.maxY)
    {
        return (v.red.maxY > v.green.maxY)
            ? &v.red
            : &v.green;
    }

    if (v.red.area >= v.green.area)
    {
        return &v.red;
    }

    return &v.green;
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

static bool validObstacle(
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
    if (validObstacle(obstacle))
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
            gyro_follower_get_state() !=
            GF_FOLLOWING)
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

        if (
            !validObstacle(
                obstacle))
        {
            oa_confirm_frames = 0;
            oa_candidate_color = ColorType::NONE;

            return false;
        }

        if (obstacle->color != oa_candidate_color)
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
            gyro_follower_get_target_heading();

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
            gyro_follower_rearm_after_obstacle();

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
    gyro_follower_set_obstacle_mode(true);
    gyro_follower_set_speed(
        OBSTACLE_CRUISE_SPEED);

    oc_was_enabled =
        false;

    obstacle_avoidance_reset();

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

void obstacle_bench_test_set(bool enable)
{
    oc_bench_test = enable;
    obstacle_avoidance_reset();
    gyro_follower_set_speed(
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

        return;
    }

    // ========================================================
    // NEW RUN
    // ========================================================

    if (!oc_was_enabled)
    {
        oc_was_enabled =
            true;

        obstacle_avoidance_reset();
        course_map_reset();

        oc_current_section = 0;
        oc_current_lap = 0;
        oc_section_start_distance = get_distance();
        oc_last_navigation_state =
            gyro_follower_get_state();
        oc_corner_settling = false;

        // The randomly selected starting zone can contain a relevant sign
        // before the first corner. Its local origin is therefore offset.
        course_map_enter_section(
            oc_current_section,
            false);

        Serial.println(
            "[OC] New obstacle run");
    }

    updateCourseProgress();

    // Give the unchanged wall/gyro follower exclusive steering control just
    // after a corner. It exits once both a minimum travel distance and a
    // small heading error are reached. The maximum distance prevents a bad
    // gyro sample from blocking obstacle detection indefinitely.
    if (
        oc_corner_settling &&
        gyro_follower_get_state() == GF_FOLLOWING)
    {
        const float settleDistance = obstacleSectionDistance();
        const float headingError =
            fabsf(
                get_angle() -
                gyro_follower_get_target_heading());

        gyro_follower_update(enabled);

        if (
            (settleDistance >=
                 OBSTACLE_CORNER_SETTLE_MIN_DISTANCE_MM &&
             headingError <=
                 OBSTACLE_CORNER_SETTLE_HEADING_DEG) ||
            settleDistance >=
                OBSTACLE_CORNER_SETTLE_MAX_DISTANCE_MM)
        {
            oc_corner_settling = false;
            gyro_follower_rearm_after_obstacle();

            Serial.print("[OC] Section aligned distance=");
            Serial.print(settleDistance, 0);
            Serial.print(" heading_error=");
            Serial.println(headingError, 1);
        }

        return;
    }

    // ========================================================
    // ALL STRAIGHT SECTIONS
    //
    // Detection is active immediately because the official starting zone is
    // random and a relevant sign can appear before the first corner.
    // ========================================================

    // --------------------------------------------------------
    // The Wall Follower is currently executing a corner,
    // stopping, or in another non-straight state.
    //
    // Camera obstacle avoidance must NOT interfere.
    // --------------------------------------------------------

    if (
        gyro_follower_get_state() !=
        GF_FOLLOWING)
    {
        gyro_follower_update(
            enabled);

        return;
    }

    // --------------------------------------------------------
    // Straight section:
    // obstacle avoidance gets first priority.
    // --------------------------------------------------------

    const bool avoiding =
        obstacle_avoidance_update(
            enabled,
            newCameraFrame);

    // --------------------------------------------------------
    // No obstacle:
    // run the unchanged Open Challenge navigation.
    // --------------------------------------------------------

    if (!avoiding)
    {
        gyro_follower_update(
            enabled);
    }
}
