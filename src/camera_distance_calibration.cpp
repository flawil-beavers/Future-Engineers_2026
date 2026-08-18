#include "camera_distance_calibration.h"

#include "config.h"
#include "logger.h"
#include "motor_control.h"
#include "navigation_controller.h"
#include "obstacle.h"
#include "sensors.h"

#define Serial robot_logger

extern float last_loop_time;

namespace
{
struct LinearFit
{
    uint32_t count = 0;
    double sumX = 0.0;
    double sumY = 0.0;
    double sumXX = 0.0;
    double sumXY = 0.0;
    double sumYY = 0.0;

    void clear()
    {
        count = 0;
        sumX = sumY = sumXX = sumXY = sumYY = 0.0;
    }

    void add(float x, float y)
    {
        ++count;
        sumX += x;
        sumY += y;
        sumXX += static_cast<double>(x) * x;
        sumXY += static_cast<double>(x) * y;
        sumYY += static_cast<double>(y) * y;
    }

    bool coefficients(float &intercept, float &slope, float &rmse) const
    {
        if (count < 2)
            return false;
        const double n = static_cast<double>(count);
        const double denominator = n * sumXX - sumX * sumX;
        if (fabs(denominator) < 1.0e-9)
            return false;
        const double b = (n * sumXY - sumX * sumY) / denominator;
        const double a = (sumY - b * sumX) / n;
        double sse = sumYY + n * a * a + b * b * sumXX +
                     2.0 * a * b * sumX - 2.0 * a * sumY -
                     2.0 * b * sumXY;
        if (sse < 0.0)
            sse = 0.0;
        intercept = static_cast<float>(a);
        slope = static_cast<float>(b);
        rmse = static_cast<float>(sqrt(sse / n));
        return true;
    }
};

enum CalibrationPhase
{
    CAL_DRIVING,
    CAL_SETTLING,
    CAL_SAMPLING
};

float configured_travel_mm = CAMERA_DRIVE_CAL_DEFAULT_TRAVEL_MM;
float start_odometry_mm = 0.0f;
float start_heading_deg = 0.0f;
float next_checkpoint_mm = CAMERA_DRIVE_CAL_FIRST_CHECKPOINT_MM;
uint32_t start_time_ms = 0;
uint32_t phase_start_ms = 0;
uint32_t last_print_ms = 0;
uint32_t rejected_frames = 0;
uint32_t stationary_frames = 0;
ColorType locked_color = ColorType::NONE;
ColorType candidate_color = ColorType::NONE;
uint8_t candidate_color_frames = 0;
bool pillar_acquired = false;
uint8_t geometry_candidate_frames = 0;
int16_t geometry_candidate_max_foot_y = 0;
int16_t last_checkpoint_foot_y = 239;
float last_heading_error_deg = 0.0f;
uint16_t checkpoint_sample_count = 0;
double checkpoint_range_sum = 0.0;
int16_t checkpoint_foot_samples[CAMERA_DRIVE_CAL_CHECKPOINT_SAMPLES];
CalibrationPhase calibration_phase = CAL_DRIVING;
LinearFit stopped_fit;
LinearFit moving_fit;

float angleDifference(float angle, float reference)
{
    float difference = angle - reference;
    while (difference > 180.0f) difference -= 360.0f;
    while (difference < -180.0f) difference += 360.0f;
    return difference;
}

const char *colorName(ColorType color)
{
    return color == ColorType::RED ? "RED" :
           color == ColorType::GREEN ? "GREEN" : "NONE";
}

const Blob *calibrationBlob()
{
    const VisionResult &result = vision.getResult();
    if (locked_color == ColorType::RED)
        return result.red.found ? &result.red : nullptr;
    if (locked_color == ColorType::GREEN)
        return result.green.found ? &result.green : nullptr;
    return getLargestObstacle();
}

bool colorCandidateValid(const Blob *blob)
{
    if (blob == nullptr || !blob->found ||
        (blob->color != ColorType::RED &&
         blob->color != ColorType::GREEN))
        return false;

    const uint32_t minimumArea = blob->color == ColorType::RED
        ? OBSTACLE_RED_MIN_AREA
        : OBSTACLE_GREEN_MIN_AREA;
    const int minimumHeight = blob->color == ColorType::RED
        ? OBSTACLE_RED_MIN_HEIGHT
        : OBSTACLE_GREEN_MIN_HEIGHT;
    return blob->area >= minimumArea &&
           blob->height() >= minimumHeight &&
           blob->minX > 2 && blob->maxX < 317 &&
           abs(blob->centerX - 160) <=
               CAMERA_DRIVE_CAL_MAX_CENTER_ERROR_PX;
}

void updateColorLock(const Blob *blob, float travelMm)
{
    if (locked_color != ColorType::NONE ||
        travelMm < CAMERA_DRIVE_CAL_COLOR_DELAY_MM)
        return;

    if (colorCandidateValid(blob))
    {
        if (candidate_color == blob->color)
            ++candidate_color_frames;
        else
        {
            candidate_color = blob->color;
            candidate_color_frames = 1;
        }
        if (candidate_color_frames >= CAMERA_DRIVE_CAL_COLOR_CONFIRM_FRAMES)
        {
            locked_color = candidate_color;
            Serial.print("[CAM DRIVE] locked_color=");
            Serial.print(colorName(locked_color));
            Serial.print(" at_travel_mm=");
            Serial.println(travelMm, 1);
        }
    }
    else
    {
        candidate_color = ColorType::NONE;
        candidate_color_frames = 0;
    }
}

bool frameIsValid(const Blob *blob, float trueForwardMm)
{
    return blob != nullptr && blob->found &&
           locked_color != ColorType::NONE &&
           blob->color == locked_color &&
           blob->minX > 2 && blob->maxX < 317 &&
           blob->maxY < 238 &&
           abs(blob->centerX - 160) <=
               CAMERA_DRIVE_CAL_MAX_CENTER_ERROR_PX &&
           trueForwardMm >= CAMERA_DRIVE_CAL_MIN_RANGE_MM &&
           trueForwardMm <= CAMERA_DRIVE_CAL_MAX_RANGE_MM;
}

void printFit(const char *name, const LinearFit &fit)
{
    float horizon = 0.0f;
    float scalePer1000 = 0.0f;
    float rmse = 0.0f;
    Serial.print("[CAM DRIVE] ");
    Serial.print(name);
    Serial.print(" samples=");
    Serial.print(fit.count);
    if (fit.coefficients(horizon, scalePer1000, rmse))
    {
        Serial.print(" horizon_y="); Serial.print(horizon, 3);
        Serial.print(" scale_mm_px="); Serial.print(scalePer1000 * 1000.0f, 1);
        Serial.print(" foot_rmse_px="); Serial.println(rmse, 2);
    }
    else
        Serial.println(" fit=unavailable");
}

bool printResult()
{
    float horizon = 0.0f;
    float scalePer1000 = 0.0f;
    float rmsePx = 0.0f;

    Serial.println("[CAM DRIVE] result_begin");
    Serial.print("[CAM DRIVE] accepted_checkpoints=");
    Serial.print(stopped_fit.count);
    Serial.print(" stationary_frames=");
    Serial.print(stationary_frames);
    Serial.print(" rejected_frames=");
    Serial.println(rejected_frames);
    printFit("moving_cross_check", moving_fit);

    if (stopped_fit.count < CAMERA_DRIVE_CAL_MIN_SAMPLES ||
        !stopped_fit.coefficients(horizon, scalePer1000, rmsePx) ||
        scalePer1000 <= 0.0f)
    {
        Serial.println("[CAM DRIVE] fit=INVALID (need more valid stopped checkpoints)");
        Serial.println("[CAM DRIVE] result_end");
        return false;
    }

    const float scaleMmPx = scalePer1000 * 1000.0f;
    Serial.print("[CAM DRIVE] stopped_fit_horizon_y=");
    Serial.print(horizon, 3);
    Serial.print(" stopped_fit_scale_mm_px=");
    Serial.print(scaleMmPx, 1);
    Serial.print(" foot_y_rmse_px=");
    Serial.println(rmsePx, 2);
    Serial.print("[CAM DRIVE] stopped_fit_check foot_y_at_400_mm=");
    Serial.print(horizon + scaleMmPx / 400.0f, 1);
    Serial.print(" foot_y_at_600_mm=");
    Serial.println(horizon + scaleMmPx / 600.0f, 1);

    float movingHorizon = 0.0f;
    float movingScalePer1000 = 0.0f;
    float movingRmse = 0.0f;
    if (moving_fit.coefficients(movingHorizon, movingScalePer1000, movingRmse))
    {
        Serial.print("[CAM DRIVE] motion_bias horizon_delta_px=");
        Serial.print(movingHorizon - horizon, 2);
        Serial.print(" scale_delta_mm_px=");
        Serial.println((movingScalePer1000 - scalePer1000) * 1000.0f, 1);
    }

    const bool qualityValid =
        horizon >= CAMERA_DRIVE_CAL_MIN_FIT_HORIZON_Y &&
        horizon <= CAMERA_DRIVE_CAL_MAX_FIT_HORIZON_Y &&
        scaleMmPx >= CAMERA_DRIVE_CAL_MIN_FIT_SCALE_MM_PX &&
        scaleMmPx <= CAMERA_DRIVE_CAL_MAX_FIT_SCALE_MM_PX &&
        rmsePx <= CAMERA_DRIVE_CAL_MAX_FIT_RMSE_PX;
    if (!qualityValid)
    {
        Serial.println("[CAM DRIVE] fit=INVALID (horizon, scale, or RMSE outside quality limits)");
        Serial.println("[CAM DRIVE] Do not copy this fit into config.h.");
        Serial.println("[CAM DRIVE] result_end");
        return false;
    }

    Serial.print("constexpr auto OBSTACLE_CAMERA_GROUND_HORIZON_Y = ");
    Serial.print(horizon, 3);
    Serial.println("f;");
    Serial.print("constexpr auto OBSTACLE_CAMERA_GROUND_RANGE_SCALE_MM_PX = ");
    Serial.print(scaleMmPx, 1);
    Serial.println("f;");
    Serial.println("[CAM DRIVE] Copy the two constexpr lines above into include/config.h.");
    Serial.println("[CAM DRIVE] The stopped fit is authoritative; moving_cross_check is diagnostic only.");
    Serial.println("[CAM DRIVE] result_end");
    return true;
}

void finish(bool success, const char *reason)
{
    set_speed(0);
    stop(false);
    set_steering(0);
    Serial.print("[CAM DRIVE] stopped: ");
    Serial.println(reason);
    const bool fitValid = printResult();
    camera_distance_cal_state = success && fitValid
        ? CAMERA_DISTANCE_CAL_DONE
        : CAMERA_DISTANCE_CAL_FAILED;
}

void beginCheckpoint(float travelMm)
{
    set_speed(0);
    stop(false);
    set_steering(0);
    calibration_phase = CAL_SETTLING;
    phase_start_ms = millis();
    checkpoint_sample_count = 0;
    checkpoint_range_sum = 0.0;
    Serial.print("[CAM DRIVE] checkpoint_stop target_mm=");
    Serial.print(next_checkpoint_mm, 1);
    Serial.print(" actual_mm=");
    Serial.println(travelMm, 1);
}

void completeCheckpoint(float travelMm)
{
    if (checkpoint_sample_count >= CAMERA_DRIVE_CAL_CHECKPOINT_MIN_SAMPLES)
    {
        const float meanRange = checkpoint_range_sum / checkpoint_sample_count;
        int16_t sortedFoot[CAMERA_DRIVE_CAL_CHECKPOINT_SAMPLES];
        for (uint16_t i = 0; i < checkpoint_sample_count; ++i)
        {
            sortedFoot[i] = checkpoint_foot_samples[i];
            uint16_t position = i;
            while (position > 0 && sortedFoot[position] < sortedFoot[position - 1])
            {
                const int16_t temporary = sortedFoot[position - 1];
                sortedFoot[position - 1] = sortedFoot[position];
                sortedFoot[position] = temporary;
                --position;
            }
        }
        // Remove two low and two high detections. This retains an averaged
        // checkpoint while preventing brief false components from moving it.
        const uint16_t trim = checkpoint_sample_count >= 6 ? 2 : 0;
        double trimmedFootSum = 0.0;
        for (uint16_t i = trim; i < checkpoint_sample_count - trim; ++i)
            trimmedFootSum += sortedFoot[i];
        const float meanFoot = trimmedFootSum /
            static_cast<float>(checkpoint_sample_count - 2 * trim);
        stopped_fit.add(1000.0f / meanRange, meanFoot);
        last_checkpoint_foot_y = static_cast<int16_t>(roundf(meanFoot));
        stationary_frames += checkpoint_sample_count;
        Serial.print("[CAM DRIVE] checkpoint_accepted travel_mm=");
        Serial.print(travelMm, 1);
        Serial.print(" true_forward_mm=");
        Serial.print(meanRange, 1);
        Serial.print(" mean_foot_y=");
        Serial.print(meanFoot, 2);
        Serial.print(" frames=");
        Serial.println(checkpoint_sample_count);
        printFit("stopped_live_fit", stopped_fit);
    }
    else
    {
        Serial.print("[CAM DRIVE] checkpoint_rejected travel_mm=");
        Serial.print(travelMm, 1);
        Serial.print(" valid_frames=");
        Serial.println(checkpoint_sample_count);
    }

    float followingCheckpoint = next_checkpoint_mm +
                                CAMERA_DRIVE_CAL_CHECKPOINT_INTERVAL_MM;
    if (followingCheckpoint > configured_travel_mm &&
        configured_travel_mm - next_checkpoint_mm >= 50.0f)
        followingCheckpoint = configured_travel_mm;

    if (followingCheckpoint <= configured_travel_mm)
    {
        next_checkpoint_mm = followingCheckpoint;
        calibration_phase = CAL_DRIVING;
        last_heading_error_deg = angleDifference(get_angle(), start_heading_deg);
        set_speed(-CAMERA_DRIVE_CAL_SPEED_MMS);
        Serial.print("[CAM DRIVE] resume next_checkpoint_mm=");
        Serial.println(next_checkpoint_mm, 1);
    }
    else
        finish(stopped_fit.count >= CAMERA_DRIVE_CAL_MIN_SAMPLES,
               "final stopped checkpoint completed");
}
} // namespace

CameraDistanceCalState camera_distance_cal_state = CAMERA_DISTANCE_CAL_IDLE;

bool camera_distance_cal_configure(float reverse_travel_mm)
{
    if (!isfinite(reverse_travel_mm) ||
        reverse_travel_mm < CAMERA_DRIVE_CAL_FIRST_CHECKPOINT_MM ||
        reverse_travel_mm > CAMERA_DRIVE_CAL_MAX_TRAVEL_MM)
        return false;

    configured_travel_mm = reverse_travel_mm;
    return true;
}

void camera_distance_cal_start()
{
    if (ROBOT_FRONT_FROM_REAR_AXLE_MM <= OBSTACLE_CAMERA_FROM_REAR_AXLE_MM)
    {
        camera_distance_cal_state = CAMERA_DISTANCE_CAL_FAILED;
        Serial.println("[CAM DRIVE] Invalid config: robot front must be ahead of camera.");
        return;
    }

    stopped_fit.clear();
    moving_fit.clear();
    rejected_frames = 0;
    stationary_frames = 0;
    locked_color = ColorType::NONE;
    candidate_color = ColorType::NONE;
    candidate_color_frames = 0;
    pillar_acquired = false;
    geometry_candidate_frames = 0;
    geometry_candidate_max_foot_y = 0;
    last_checkpoint_foot_y = 239;
    last_heading_error_deg = 0.0f;
    checkpoint_sample_count = 0;
    calibration_phase = CAL_DRIVING;
    next_checkpoint_mm = min(CAMERA_DRIVE_CAL_FIRST_CHECKPOINT_MM,
                             configured_travel_mm);
    navigation_reset_filter();
    start_odometry_mm = get_distance();
    start_heading_deg = get_angle();
    start_time_ms = millis();
    phase_start_ms = start_time_ms;
    last_print_ms = 0;
    camera_distance_cal_state = CAMERA_DISTANCE_CAL_RUNNING;

    set_steering(0);
    set_speed(-CAMERA_DRIVE_CAL_SPEED_MMS);

    Serial.println("[CAM DRIVE] AUTOMATED STOPPED-CHECKPOINT CAMERA CALIBRATION");
    Serial.println("[CAM DRIVE] Place the pillar on the centreline, touching the robot front.");
    Serial.print("[CAM DRIVE] robot_front_from_rear_axle_mm=");
    Serial.print(ROBOT_FRONT_FROM_REAR_AXLE_MM, 1);
    Serial.print(" camera_from_rear_axle_mm=");
    Serial.print(OBSTACLE_CAMERA_FROM_REAR_AXLE_MM, 1);
    Serial.print(" initial_camera_range_mm=");
    Serial.println(ROBOT_FRONT_FROM_REAR_AXLE_MM -
                   OBSTACLE_CAMERA_FROM_REAR_AXLE_MM, 1);
    Serial.print("[CAM DRIVE] checkpoints first_mm=");
    Serial.print(next_checkpoint_mm, 1);
    Serial.print(" interval_mm=");
    Serial.print(CAMERA_DRIVE_CAL_CHECKPOINT_INTERVAL_MM, 1);
    Serial.print(" final_mm=");
    Serial.println(configured_travel_mm, 1);
    Serial.println("[CAM DRIVE] Gyro-stabilised reverse; each stop settles then averages camera frames.");
    Serial.println("[CAM DRIVE] Keep the lane clear; send z to abort.");
}

void camera_distance_cal_update(bool newCameraFrame)
{
    if (camera_distance_cal_state != CAMERA_DISTANCE_CAL_RUNNING)
        return;

    const uint32_t now = millis();
    const float travelMm = fabsf(get_distance() - start_odometry_mm);
    const float headingError = angleDifference(get_angle(), start_heading_deg);
    if (fabsf(headingError) > CAMERA_DRIVE_CAL_MAX_HEADING_ERROR_DEG)
    {
        finish(false, "heading changed too far; distance geometry is invalid");
        return;
    }
    if (now - start_time_ms >= CAMERA_DRIVE_CAL_TIMEOUT_MS)
    {
        finish(false, "timeout");
        return;
    }

    float steering = 0.0f;
    if (calibration_phase == CAL_DRIVING)
    {
        steering = -navigation_compute_steering(
            headingError, last_heading_error_deg, last_loop_time);
        last_heading_error_deg = headingError;
        steering = constrain(steering,
                             -CAMERA_DRIVE_CAL_MAX_STEERING_DEG,
                             CAMERA_DRIVE_CAL_MAX_STEERING_DEG);
        set_steering(static_cast<int>(roundf(steering)));
        if (travelMm >= next_checkpoint_mm)
        {
            beginCheckpoint(travelMm);
            return;
        }
    }
    else
        set_steering(0);

    if (calibration_phase == CAL_SETTLING &&
        now - phase_start_ms >= CAMERA_DRIVE_CAL_CHECKPOINT_SETTLE_MS)
    {
        calibration_phase = CAL_SAMPLING;
        phase_start_ms = now;
        Serial.println("[CAM DRIVE] checkpoint_sampling");
    }

    if (!newCameraFrame)
    {
        if (calibration_phase == CAL_SAMPLING &&
            now - phase_start_ms >= CAMERA_DRIVE_CAL_CHECKPOINT_SAMPLE_TIMEOUT_MS)
            completeCheckpoint(travelMm);
        return;
    }

    const float trueForwardMm = ROBOT_FRONT_FROM_REAR_AXLE_MM + travelMm -
                                OBSTACLE_CAMERA_FROM_REAR_AXLE_MM;
    const Blob *blob = calibrationBlob();
    updateColorLock(blob, travelMm);
    blob = calibrationBlob();
    const bool basicValid = frameIsValid(blob, trueForwardMm);

    if (!pillar_acquired)
    {
        if (basicValid && blob->maxY >= CAMERA_DRIVE_CAL_ACQUIRE_MIN_FOOT_Y)
        {
            ++geometry_candidate_frames;
            if (blob->maxY > geometry_candidate_max_foot_y)
                geometry_candidate_max_foot_y = blob->maxY;
            if (geometry_candidate_frames >= CAMERA_DRIVE_CAL_GEOMETRY_CONFIRM_FRAMES)
            {
                pillar_acquired = true;
                last_checkpoint_foot_y = geometry_candidate_max_foot_y;
                Serial.print("[CAM DRIVE] pillar_acquired foot_y=");
                Serial.print(last_checkpoint_foot_y);
                Serial.print(" at_travel_mm=");
                Serial.println(travelMm, 1);
            }
        }
        else
        {
            geometry_candidate_frames = 0;
            geometry_candidate_max_foot_y = 0;
        }
    }

    const bool accepted = basicValid && pillar_acquired &&
                          blob->maxY <= last_checkpoint_foot_y +
                              CAMERA_DRIVE_CAL_FOOT_Y_TOLERANCE_PX;
    if (accepted)
    {
        if (calibration_phase == CAL_SAMPLING)
        {
            checkpoint_range_sum += trueForwardMm;
            checkpoint_foot_samples[checkpoint_sample_count] = blob->maxY;
            ++checkpoint_sample_count;
        }
        else if (calibration_phase == CAL_DRIVING)
            moving_fit.add(1000.0f / trueForwardMm,
                           static_cast<float>(blob->maxY));
    }
    else
        ++rejected_frames;

    if (calibration_phase == CAL_SAMPLING &&
        (checkpoint_sample_count >= CAMERA_DRIVE_CAL_CHECKPOINT_SAMPLES ||
         now - phase_start_ms >= CAMERA_DRIVE_CAL_CHECKPOINT_SAMPLE_TIMEOUT_MS))
    {
        completeCheckpoint(travelMm);
        return;
    }

    if (now - last_print_ms < CAMERA_DRIVE_CAL_PRINT_INTERVAL_MS)
        return;
    last_print_ms = now;
    Serial.print("[CAM DRIVE] data phase=");
    Serial.print(calibration_phase == CAL_DRIVING ? "drive" :
                 calibration_phase == CAL_SETTLING ? "settle" : "sample");
    Serial.print(" travel_mm="); Serial.print(travelMm, 1);
    Serial.print(" true_forward_mm="); Serial.print(trueForwardMm, 1);
    Serial.print(" foot_y="); Serial.print(blob != nullptr ? blob->maxY : -1);
    Serial.print(" color="); Serial.print(blob != nullptr ? colorName(blob->color) : "NONE");
    Serial.print(" heading_error_deg="); Serial.print(headingError, 2);
    Serial.print(" steering_deg="); Serial.print(steering, 1);
    Serial.print(" accepted="); Serial.println(accepted ? "yes" : "no");
}

void camera_distance_cal_stop()
{
    if (camera_distance_cal_state == CAMERA_DISTANCE_CAL_RUNNING)
    {
        set_speed(0);
        stop(false);
        set_steering(0);
        Serial.println("[CAM DRIVE] aborted");
        (void)printResult();
    }
    camera_distance_cal_state = CAMERA_DISTANCE_CAL_IDLE;
}
