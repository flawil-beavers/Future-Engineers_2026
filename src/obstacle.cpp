#include "obstacle.h"

#include "config.h"
#include "motor_control.h"
#include "sensors.h"
#include "navigation_controller.h"
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

enum ParkingExitState : uint8_t
{
    PARKING_EXIT_IDLE,
    PARKING_EXIT_FIRST_ARC,
    PARKING_EXIT_COUNTER_ARC,
    PARKING_EXIT_STRAIGHTENING,
    PARKING_EXIT_TEST_HOLD,
    PARKING_EXIT_DONE
};

static ParkingExitState oc_parking_exit_state =
    PARKING_EXIT_IDLE;
static float oc_parking_exit_state_distance = 0.0f;
static float oc_parking_exit_start_heading = 0.0f;
static int oc_parking_exit_steering = 0;
static uint32_t oc_parking_exit_brake_start_ms = 0;

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
    oc_parking_exit_start_heading = get_angle();
    oc_parking_exit_steering = 0;
    oc_parking_exit_brake_start_ms = 0;
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

    if (oc_parking_exit_state == PARKING_EXIT_STRAIGHTENING)
    {
        // Do not cut motor power at full speed. Centre the wheels immediately
        // and let the speed controller ramp its target to zero first.
        set_steering(0);
        steer(0);
        set_speed(0);

        const uint32_t brakeTimeMs =
            oc_parking_exit_steering < 0
                ? OBSTACLE_PARKING_EXIT_BRAKE_TIME_NEGATIVE_MS
                : OBSTACLE_PARKING_EXIT_BRAKE_TIME_POSITIVE_MS;

        if (
            millis() - oc_parking_exit_brake_start_ms <
                brakeTimeMs)
        {
            return true;
        }

        const float finalHeadingError =
            fabsf(get_angle() - oc_parking_exit_start_heading);

        stop(false);

        Serial.print("[PARK EXIT] Stopped heading_error=");
        Serial.println(finalHeadingError, 1);

        if (OBSTACLE_PARKING_EXIT_TEST_ONLY)
        {
            oc_parking_exit_state = PARKING_EXIT_TEST_HOLD;
            Serial.println(
                "[PARK EXIT] Test complete - drive motor locked off");
            robot_logger.write_to_usb();
            return true;
        }

        oc_parking_exit_state = PARKING_EXIT_DONE;
        navigation_enable();
        oc_section_start_distance = get_distance();
        oc_last_navigation_state = navigation_get_state();
        oc_last_completed_turn = navigation_get_turn_count();

        Serial.println("[PARK EXIT] Complete - normal Obstacle navigation");
        return false;
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

        oc_parking_exit_state_distance = get_distance();
        oc_parking_exit_start_heading = get_angle();
        oc_parking_exit_state = PARKING_EXIT_FIRST_ARC;

        Serial.print("[PARK EXIT] Start wall left=");
        Serial.print(left, 0);
        Serial.print(" right=");
        Serial.println(right, 0);
        Serial.print("[PARK EXIT] First arc, steering=");
        Serial.println(oc_parking_exit_steering);
    }

    const float stateDistance =
        distanceSince(oc_parking_exit_state_distance);

    if (oc_parking_exit_state == PARKING_EXIT_FIRST_ARC)
    {
        set_speed(OBSTACLE_PARKING_EXIT_SPEED);
        set_steering(oc_parking_exit_steering);

        const float firstArcTarget =
            oc_parking_exit_steering < 0
                ? OBSTACLE_PARKING_EXIT_FIRST_ARC_NEGATIVE_MM
                : OBSTACLE_PARKING_EXIT_FIRST_ARC_POSITIVE_MM;

        if (stateDistance >= firstArcTarget)
        {
            const float firstArcHeading =
                fabsf(
                    get_angle() -
                    oc_parking_exit_start_heading);

            Serial.print("[PARK EXIT] First distance=");
            Serial.print(stateDistance, 0);
            Serial.print(" heading=");
            Serial.println(firstArcHeading, 1);

            oc_parking_exit_state_distance = get_distance();
            oc_parking_exit_state = PARKING_EXIT_COUNTER_ARC;
            Serial.println("[PARK EXIT] Counter arc");
        }

        return true;
    }

    const float headingError =
        fabsf(get_angle() - oc_parking_exit_start_heading);

    float counterSteering =
        static_cast<float>(OBSTACLE_PARKING_EXIT_STEERING);
    int counterSpeed =
        OBSTACLE_PARKING_EXIT_COUNTER_SPEED;

    if (
        headingError <
            OBSTACLE_PARKING_EXIT_FINE_ALIGN_START_DEG)
    {
        counterSteering = clampValue(
            OBSTACLE_PARKING_EXIT_STEERING *
                headingError /
                OBSTACLE_PARKING_EXIT_FINE_ALIGN_START_DEG,
            OBSTACLE_PARKING_EXIT_FINE_ALIGN_MIN_STEERING,
            static_cast<float>(
                OBSTACLE_PARKING_EXIT_STEERING));
        counterSpeed =
            OBSTACLE_PARKING_EXIT_FINE_ALIGN_SPEED;
    }

    set_speed(counterSpeed);
    set_steering(
        oc_parking_exit_steering < 0
            ? static_cast<int>(counterSteering)
            : -static_cast<int>(counterSteering));

    const bool parallelAgain =
        stateDistance >=
            OBSTACLE_PARKING_EXIT_COUNTER_MIN_MM &&
        headingError <=
            OBSTACLE_PARKING_EXIT_HEADING_TOLERANCE_DEG;
    const bool safetyDistanceReached =
        stateDistance >=
            OBSTACLE_PARKING_EXIT_COUNTER_MAX_MM;

    if (parallelAgain || safetyDistanceReached)
    {
        set_steering(0);
        // Apply the straight-ahead command immediately. stop(false) disables
        // the servo, so waiting for the next drive_loop() would leave the
        // wheels at the counter-steering angle during measurement.
        steer(0);

        Serial.print("[PARK EXIT] Counter distance=");
        Serial.print(stateDistance, 0);
        Serial.print(" heading_error=");
        Serial.println(headingError, 1);

        oc_parking_exit_brake_start_ms = millis();
        oc_parking_exit_state = PARKING_EXIT_STRAIGHTENING;
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
            !validObstacle(obstacle))
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
        course_map_reset();

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
    }

    // This path exists only in the Obstacle Challenge. While it owns the
    // steering, camera avoidance and the regular challenge remain untouched.
    if (updateParkingExit())
    {
        return;
    }

    updateCourseProgress();
    const bool learnedLaneActive =
        updateLearnedLanePlan();

    // Let the gyro follower align the car just after a corner, while vision
    // already looks ahead. A confirmed sign at the beginning of the new
    // section must be allowed to take control before the normal settle
    // distance has elapsed.
    if (
        oc_corner_settling &&
        navigation_get_state() == NAV_FOLLOWING)
    {
        const float settleDistance = obstacleSectionDistance();
        const float headingError =
            fabsf(
                get_angle() -
                navigation_get_target_heading());

        // Vision may already see the next sign, but an Ackermann car must
        // first be nearly parallel to the new section. Otherwise a sign seen
        // far to one side during the turn commands a large, wrong arc.
        const bool alignedForEarlyTakeover =
            headingError <=
            OBSTACLE_CORNER_EARLY_TAKEOVER_HEADING_DEG;

        const bool obstacleNeedsControl =
            !learnedLaneActive &&
            alignedForEarlyTakeover &&
            obstacle_avoidance_update(
                enabled,
                newCameraFrame);

        if (obstacleNeedsControl)
        {
            oc_corner_settling = false;
            Serial.println(
                "[OC] Early obstacle during corner exit");
            return;
        }

        navigation_update(enabled);

        if (
            (settleDistance >=
                 OBSTACLE_CORNER_SETTLE_MIN_DISTANCE_MM &&
             headingError <=
                 OBSTACLE_CORNER_SETTLE_HEADING_DEG) ||
            settleDistance >=
                OBSTACLE_CORNER_SETTLE_MAX_DISTANCE_MM)
        {
            oc_corner_settling = false;
            navigation_rearm_after_obstacle();

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
        navigation_get_state() !=
        NAV_FOLLOWING)
    {
        navigation_update(
            enabled);

        return;
    }

    // --------------------------------------------------------
    // Straight section:
    // obstacle avoidance gets first priority.
    // --------------------------------------------------------

    const bool avoiding =
        oc_current_lap == 0
            ? obstacle_avoidance_update(
                  enabled,
                  newCameraFrame)
            : false;

    // --------------------------------------------------------
    // No obstacle:
    // run the unchanged Open Challenge navigation.
    // --------------------------------------------------------

    if (!avoiding)
    {
        navigation_update(
            enabled);
    }
}
