#include "reverse_gyro_test.h"

#include <Arduino.h>
#include <math.h>

#include "config.h"
#include "logger.h"
#include "motor_control.h"
#include "position_estimator.h"
#include "sensors.h"

#define Serial robot_logger

namespace
{
constexpr int TEST_SPEEDS_MM_S[] = {40, 60, 80};
constexpr uint8_t TEST_SPEED_COUNT =
    sizeof(TEST_SPEEDS_MM_S) / sizeof(TEST_SPEEDS_MM_S[0]);
constexpr float TEST_DISTANCE_MM = 500.0f;
constexpr float TEST_HEADING_KP = 3.0f;
constexpr float TEST_MAX_STEERING_DEG = 15.0f;
constexpr unsigned long TEST_SETTLE_MS = 500UL;
constexpr unsigned long TEST_LOG_INTERVAL_MS = 100UL;

bool active = false;
bool finished = false;
uint8_t speed_index = 0;
float heading_target = 0.0f;
float start_distance = 0.0f;
float max_heading_error = 0.0f;
uint32_t start_ms = 0;
uint32_t settle_start_ms = 0;
uint32_t last_log_ms = 0;

float wrapped_error(float error)
{
    while (error > 180.0f)
        error -= 360.0f;
    while (error < -180.0f)
        error += 360.0f;
    return error;
}

void begin_speed_run()
{
    stop(false);
    servo_disabled = false;
    set_steering(0);
    steer(0);
    heading_target = get_angle();
    start_distance = get_distance();
    max_heading_error = 0.0f;
    start_ms = millis();
    last_log_ms = 0;
    Serial.print("[REVERSE GYRO] Run ");
    Serial.print(speed_index + 1);
    Serial.print("/");
    Serial.print(TEST_SPEED_COUNT);
    Serial.print(" speed_mm_s=");
    Serial.print(TEST_SPEEDS_MM_S[speed_index]);
    Serial.print(" target_heading_deg=");
    Serial.println(heading_target, 2);
}

void finish_speed_run()
{
    stop(true);
    set_steering(0);
    const float travel = fabsf(get_distance() - start_distance);
    Serial.print("[REVERSE GYRO RESULT] speed_mm_s=");
    Serial.print(TEST_SPEEDS_MM_S[speed_index]);
    Serial.print(" travel_mm=");
    Serial.print(travel, 1);
    Serial.print(" elapsed_ms=");
    Serial.print(millis() - start_ms);
    Serial.print(" final_heading_error_deg=");
    Serial.print(wrapped_error(get_angle() - heading_target), 2);
    Serial.print(" max_heading_error_deg=");
    Serial.println(max_heading_error, 2);

    ++speed_index;
    if (speed_index >= TEST_SPEED_COUNT)
    {
        stop(false);
        finished = true;
        active = false;
        Serial.println("[REVERSE GYRO] All speed runs complete - drive locked off");
        robot_logger.write_to_usb();
        return;
    }

    settle_start_ms = millis();
}
} // namespace

void reverse_gyro_test_start()
{
    robot_logger.clear();
    active = true;
    finished = false;
    speed_index = 0;
    settle_start_ms = millis();
    Serial.println("[REVERSE GYRO] Starting 40/60/80 mm/s straight-line test");
}

void reverse_gyro_test_update()
{
    if (!active)
        return;

    if (settle_start_ms != 0 && millis() - settle_start_ms < TEST_SETTLE_MS)
    {
        stop(false);
        return;
    }
    if (settle_start_ms != 0)
    {
        settle_start_ms = 0;
        begin_speed_run();
    }

    const float heading_error =
        wrapped_error(get_angle() - heading_target);
    max_heading_error = fmaxf(max_heading_error, fabsf(heading_error));
    const int steering = static_cast<int>(roundf(constrain(
        -TEST_HEADING_KP * heading_error,
        -TEST_MAX_STEERING_DEG,
        TEST_MAX_STEERING_DEG)));
    set_steering(steering);
    set_speed(-TEST_SPEEDS_MM_S[speed_index]);

    const unsigned long now = millis();
    if (last_log_ms == 0 || now - last_log_ms >= TEST_LOG_INTERVAL_MS)
    {
        last_log_ms = now;
        Serial.print("[REVERSE GYRO SAMPLE] speed_mm_s=");
        Serial.print(TEST_SPEEDS_MM_S[speed_index]);
        Serial.print(" travel_mm=");
        Serial.print(fabsf(get_distance() - start_distance), 1);
        Serial.print(" measured_mm_s=");
        Serial.print(measured_speed, 1);
        Serial.print(" heading_error_deg=");
        Serial.print(heading_error, 2);
        Serial.print(" steering_deg=");
        Serial.println(steering);
    }

    if (fabsf(get_distance() - start_distance) >= TEST_DISTANCE_MM)
        finish_speed_run();
}

void reverse_gyro_test_stop()
{
    active = false;
    finished = true;
    stop(false);
    set_steering(0);
}

bool reverse_gyro_test_finished()
{
    return finished;
}