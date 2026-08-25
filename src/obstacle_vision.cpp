#include "obstacle.h"

#include "config.h"
#include "logger.h"

#define Serial robot_logger

namespace
{
float calibration_reference_distance_mm = 0.0f;
uint32_t red_calibration_samples = 0;
uint32_t green_calibration_samples = 0;
float red_focal_sum_px = 0.0f;
float green_focal_sum_px = 0.0f;
float red_range_error_sum_mm = 0.0f;
float green_range_error_sum_mm = 0.0f;

void printCalibrationBlob(
    const char *name,
    const Blob &blob,
    uint32_t &sample_count,
    float &focal_sum_px,
    float &range_error_sum_mm)
{
    if (!blob.found || blob.height() <= 0)
        return;

    const float bearing_deg = obstacle_camera_bearing_deg(&blob);
    const bool edge_clipped = blob.minX <= 2 || blob.maxX >= 317;
    const float runtime_range_mm =
        obstacle_estimate_camera_range_mm(&blob);
    const bool production_valid =
        obstacle_blob_valid_for_acquisition(&blob);

    Serial.print("[CAM CAL] color=");
    Serial.print(name);
    Serial.print(" x=");
    Serial.print(blob.centerX);
    Serial.print(" y=");
    Serial.print(blob.centerY);
    Serial.print(" w=");
    Serial.print(blob.width());
    Serial.print(" h=");
    Serial.print(blob.height());
    Serial.print(" area=");
    Serial.print(blob.area);
    Serial.print(" min_x=");
    Serial.print(blob.minX);
    Serial.print(" max_x=");
    Serial.print(blob.maxX);
    Serial.print(" min_y=");
    Serial.print(blob.minY);
    Serial.print(" max_y=");
    Serial.print(blob.maxY);
    Serial.print(" bearing_deg=");
    Serial.print(bearing_deg, 1);
    Serial.print(" range_est_mm=");
    Serial.print(runtime_range_mm, 1);
    Serial.print(" edge_clipped=");
    Serial.print(edge_clipped ? "yes" : "no");
    Serial.print(" production_valid=");
    Serial.print(production_valid ? "yes" : "no");

    if (calibration_reference_distance_mm > 0.0f && production_valid)
    {
        const float focal_sample_px =
            calibration_reference_distance_mm *
            static_cast<float>(blob.height()) /
            OBSTACLE_PILLAR_HEIGHT_MM;
        focal_sum_px += focal_sample_px;
        const float range_error_mm =
            runtime_range_mm - calibration_reference_distance_mm;
        range_error_sum_mm += range_error_mm;
        ++sample_count;

        Serial.print(" ref_mm=");
        Serial.print(calibration_reference_distance_mm, 0);
        Serial.print(" focal_sample_px=");
        Serial.print(focal_sample_px, 1);
        Serial.print(" focal_avg_px=");
        Serial.print(focal_sum_px / static_cast<float>(sample_count), 1);
        Serial.print(" range_error_mm=");
        Serial.print(range_error_mm, 1);
        Serial.print(" range_error_avg_mm=");
        Serial.print(
            range_error_sum_mm / static_cast<float>(sample_count),
            1);
        Serial.print(" samples=");
        Serial.print(sample_count);
        Serial.print(" sample_accepted=yes");
    }
    else if (calibration_reference_distance_mm > 0.0f)
    {
        Serial.print(" sample_accepted=no");
    }

    Serial.println();
}
} // namespace

bool updateCameraVision()
{
    static uint32_t last_camera_update = 0;
    if (millis() - last_camera_update < OBSTACLE_CAMERA_INTERVAL_MS)
        return false;

    last_camera_update = millis();
    if (!camera.capture()) {
        Serial.println("Camera capture failed.");
        return false;
    }

    return vision.update(
        camera.getBuffer(),
        camera.getWidth(),
        camera.getHeight());
}

void printCameraCalibration()
{
    static uint32_t last_print = 0;
    if (millis() - last_print < 500)
        return;

    last_print = millis();
    const uint16_t x = camera.getWidth() / 2;
    const uint16_t y = camera.getHeight() / 2;
    const HSV hsv = vision.getHSVAt(
        camera.getBuffer(),
        camera.getWidth(),
        camera.getHeight(),
        x,
        y);

    Serial.print("[CAM CAL] center_hsv_h=");
    Serial.print(hsv.h);
    Serial.print(" s=");
    Serial.print(hsv.s);
    Serial.print(" v=");
    Serial.println(hsv.v);

    const VisionResult &result = vision.getResult();
    Serial.print("[CAM PERF] source=");
    Serial.print(camera.getWidth());
    Serial.print("x");
    Serial.print(camera.getHeight());
    Serial.print(" capture_ms=");
    Serial.print(camera.getLastCaptureTimeUs() / 1000.0f, 2);
    Serial.print(" processing_ms=");
    Serial.print(result.processingTimeUs / 1000.0f, 2);
    Serial.print(" total_ms=");
    Serial.println(
        (camera.getLastCaptureTimeUs() + result.processingTimeUs) /
            1000.0f,
        2);
    if (!result.red.found && !result.green.found)
    {
        Serial.println("[CAM CAL] blob=NONE");
        return;
    }

    printCalibrationBlob(
        "RED",
        result.red,
        red_calibration_samples,
        red_focal_sum_px,
        red_range_error_sum_mm);
    printCalibrationBlob(
        "GREEN",
        result.green,
        green_calibration_samples,
        green_focal_sum_px,
        green_range_error_sum_mm);
}

void camera_calibration_set_reference_distance(float distance_mm)
{
    calibration_reference_distance_mm = distance_mm > 0.0f
        ? distance_mm
        : 0.0f;
    red_calibration_samples = 0;
    green_calibration_samples = 0;
    red_focal_sum_px = 0.0f;
    green_focal_sum_px = 0.0f;
    red_range_error_sum_mm = 0.0f;
    green_range_error_sum_mm = 0.0f;

    Serial.print("[CAM CAL] Reference distance: ");
    if (calibration_reference_distance_mm > 0.0f)
    {
        Serial.print(calibration_reference_distance_mm, 0);
        Serial.println(" mm; sample averages reset");
    }
    else
    {
        Serial.println("not set; blob diagnostics only");
    }
}

const Blob *getLargestObstacle()
{
    const VisionResult &result = vision.getResult();

    if (!result.red.found && !result.green.found)
        return nullptr;
    if (result.red.found && !result.green.found)
        return &result.red;
    if (result.green.found && !result.red.found)
        return &result.green;

    if (result.red.maxY != result.green.maxY) {
        return result.red.maxY > result.green.maxY
            ? &result.red
            : &result.green;
    }

    return result.red.area >= result.green.area
        ? &result.red
        : &result.green;
}

const Blob *getLargestValidObstacle()
{
    const VisionResult &result = vision.getResult();
    const Blob *red = obstacle_blob_valid_for_acquisition(&result.red)
        ? &result.red
        : nullptr;
    const Blob *green = obstacle_blob_valid_for_acquisition(&result.green)
        ? &result.green
        : nullptr;

    if (red == nullptr)
        return green;
    if (green == nullptr)
        return red;
    if (red->maxY != green->maxY)
        return red->maxY > green->maxY ? red : green;
    return red->area >= green->area ? red : green;
}

float obstacle_camera_bearing_deg(const Blob *obstacle)
{
    if (obstacle == nullptr || !obstacle->found)
        return 0.0f;

    // Image X grows to the right, while the path/global convention uses a
    // positive bearing to the robot's left. Use the surveyed optical centre
    // and focal length instead of assuming a centred, angle-linear image.
    return atanf(
               (OBSTACLE_CAMERA_PRINCIPAL_X_PX -
                static_cast<float>(obstacle->centerX)) /
               OBSTACLE_CAMERA_FOCAL_X_PX) *
           180.0f / PI;
}

float obstacle_estimate_camera_forward_mm(const Blob *obstacle)
{
    if (obstacle == nullptr || !obstacle->found)
        return 0.0f;

    if (obstacle->minX <= 2 || obstacle->maxX >= 317)
        return OBSTACLE_EDGE_CLIPPED_RANGE_MM;

    const float horizontalOffset = fabsf(
        static_cast<float>(obstacle->centerX) -
        OBSTACLE_CAMERA_PRINCIPAL_X_PX);
    const float correctedFootY =
        static_cast<float>(obstacle->maxY) -
        OBSTACLE_CAMERA_FOOT_EDGE_SLOPE * horizontalOffset;
    const float ground_denominator =
        correctedFootY -
        OBSTACLE_CAMERA_GROUND_HORIZON_Y;
    if (ground_denominator > 1.0f)
    {
        return OBSTACLE_CAMERA_GROUND_RANGE_SCALE_MM_PX /
               ground_denominator;
    }

    if (obstacle->height() <= 0)
        return 0.0f;
    return OBSTACLE_CAMERA_FOCAL_LENGTH_PX *
           OBSTACLE_PILLAR_HEIGHT_MM /
           static_cast<float>(obstacle->height());
}

float obstacle_estimate_camera_range_mm(const Blob *obstacle)
{
    const float forwardMm = obstacle_estimate_camera_forward_mm(obstacle);
    if (forwardMm <= 0.0f)
        return 0.0f;

    // The ground-plane fit estimates forward depth. Convert it to distance
    // along the bearing ray so obstacle_path.cpp can project off-centre
    // pillars into the field and snap them to the correct known seat.
    const float bearingRad = obstacle_camera_bearing_deg(obstacle) * PI / 180.0f;
    const float bearingCos = cosf(bearingRad);
    if (bearingCos <= 0.01f)
        return 0.0f;
    return forwardMm / bearingCos;
}
