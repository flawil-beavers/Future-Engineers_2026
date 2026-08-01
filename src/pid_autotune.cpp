/**
 * @file pid_autotune.cpp
 * @brief Relay-feedback tuning for the DC motor speed PID.
 *
 * Both the tuner and the normal DC controller operate on the same process:
 * speed error in mm/s -> motor PWM. This keeps the identified gains and the
 * runtime PID gains dimensionally compatible.
 */

#include "pid_autotune.h"
#include "motor_control.h"
#include "config.h"
#include "logger.h"
#define Serial robot_logger

PIDAtuneState pid_atune_state = AT_IDLE;
PIDAtuneResult pid_atune_result;

static float start_distance = 0.0f;
static float last_distance_sample = 0.0f;
static unsigned long start_time_us = 0;
static unsigned long accel_start_us = 0;
static unsigned long target_stable_since_us = 0;
static unsigned long last_zero_cross_us = 0;
static unsigned long last_accel_debug_us = 0;
static float baseline_dc = 0.0f;
static float relay_reference_speed = 0.0f;
static int warmup_crossings = 0;
static int measured_crossings = 0;
static float current_half_cycle_peak = 0.0f;
static float sum_peaks = 0.0f;
static float sum_half_periods = 0.0f;
static float sum_half_periods_squared = 0.0f;
static int period_count = 0;

static void print_results()
{
    Serial.println("\n=== PID AUTOTUNE RESULT ===");
    Serial.print("Status: ");
    Serial.println(
        pid_atune_result.applied
            ? "VALID AND APPLIED"
            : (pid_atune_result.aborted ? "ABORTED" : "INVALID"));
    Serial.print("Measured crossings: ");
    Serial.println(measured_crossings);
    Serial.print("Speed amplitude: ");
    Serial.print(pid_atune_result.speed_amplitude, 2);
    Serial.println(" mm/s");
    Serial.print("Ultimate period Tu: ");
    Serial.print(pid_atune_result.Tu, 4);
    Serial.println(" s");
    Serial.print("Period variation: ");
    Serial.print(pid_atune_result.period_variation * 100.0f, 1);
    Serial.println(" %");
    Serial.print("Ku: ");
    Serial.println(pid_atune_result.Ku, 4);
    Serial.print("Kp: ");
    Serial.println(pid_atune_result.Kp, 4);
    Serial.print("Ki: ");
    Serial.println(pid_atune_result.Ki, 4);
    Serial.print("Kd: ");
    Serial.println(pid_atune_result.Kd, 4);
    Serial.print("Distance forward/backward: ");
    Serial.print(pid_atune_result.distance_forward, 1);
    Serial.print(" / ");
    Serial.print(pid_atune_result.distance_backward, 1);
    Serial.println(" mm");
    Serial.println("===========================\n");
}

static void abort_tune(const char *reason)
{
    Serial.print("PID autotune aborted: ");
    Serial.println(reason);
    pid_atune_result.aborted = true;
    pid_atune_result.valid = false;
    pid_atune_state = AT_DONE;
    stop(false);
    print_results();
}

static void record_zero_crossing()
{
    const unsigned long now = micros();
    const float half_period =
        last_zero_cross_us == 0
            ? 0.0f
            : (now - last_zero_cross_us) / 1000000.0f;
    last_zero_cross_us = now;

    if (warmup_crossings < PID_AT_WARMUP_CROSSINGS) {
        ++warmup_crossings;
        current_half_cycle_peak = 0.0f;
        return;
    }

    if (half_period > 0.02f && half_period < 5.0f) {
        sum_half_periods += half_period;
        sum_half_periods_squared += half_period * half_period;
        ++period_count;
    }
    sum_peaks += current_half_cycle_peak;
    ++measured_crossings;
    current_half_cycle_peak = 0.0f;
}

static bool calculate_gains()
{
    if (measured_crossings < PID_AT_MIN_CYCLES * 2 ||
        period_count < PID_AT_MIN_CYCLES * 2)
        return false;

    const float amplitude =
        sum_peaks / measured_crossings;
    const float average_half_period =
        sum_half_periods / period_count;
    float variance =
        sum_half_periods_squared / period_count -
        average_half_period * average_half_period;
    if (variance < 0.0f)
        variance = 0.0f;

    const float period_variation =
        average_half_period > 0.0f
            ? sqrtf(variance) / average_half_period
            : 1.0f;
    const float ultimate_period =
        2.0f * average_half_period;

    pid_atune_result.speed_amplitude = amplitude;
    pid_atune_result.period_variation = period_variation;
    pid_atune_result.Tu = ultimate_period;
    pid_atune_result.zero_crossings = measured_crossings;
    pid_atune_result.overshoot =
        amplitude / (float)PID_AT_TARGET_SPEED;

    if (!isfinite(amplitude) ||
        !isfinite(ultimate_period) ||
        amplitude < PID_AT_MIN_SPEED_AMPLITUDE ||
        period_variation > PID_AT_MAX_PERIOD_VARIATION ||
        ultimate_period <= 0.04f)
        return false;

    pid_atune_result.Ku =
        4.0f * (float)PID_AT_RELAY_AMPLITUDE /
        (PI * amplitude);

    // Tyreus-Luyben PID rules are less aggressive than classic
    // Ziegler-Nichols and are better suited to a small traction motor.
    pid_atune_result.Kp = pid_atune_result.Ku / 2.2f;
    const float integral_time = 2.2f * ultimate_period;
    const float derivative_time = ultimate_period / 6.3f;
    pid_atune_result.Ki =
        pid_atune_result.Kp / integral_time;
    pid_atune_result.Kd =
        pid_atune_result.Kp * derivative_time;

    const bool gains_sane =
        isfinite(pid_atune_result.Kp) &&
        isfinite(pid_atune_result.Ki) &&
        isfinite(pid_atune_result.Kd) &&
        pid_atune_result.Kp > 0.0f &&
        pid_atune_result.Kp <= 10.0f &&
        pid_atune_result.Ki >= 0.0f &&
        pid_atune_result.Ki <= 10.0f &&
        pid_atune_result.Kd >= 0.0f &&
        pid_atune_result.Kd <= 5.0f;
    pid_atune_result.valid = gains_sane;
    return gains_sane;
}

void pid_autotune_start()
{
    if (pid_autotune_is_active())
        pid_autotune_stop();

    pid_atune_result = {};
    pid_atune_state = AT_ACCELERATING;
    start_distance = current_distance;
    last_distance_sample = current_distance;
    start_time_us = micros();
    accel_start_us = start_time_us;
    last_accel_debug_us = start_time_us;
    target_stable_since_us = 0;
    last_zero_cross_us = 0;
    warmup_crossings = 0;
    measured_crossings = 0;
    current_half_cycle_peak = 0.0f;
    sum_peaks = 0.0f;
    sum_half_periods = 0.0f;
    sum_half_periods_squared = 0.0f;
    period_count = 0;

    baseline_dc =
        MOTOR_MIN_DC +
        ((float)PID_AT_TARGET_SPEED / 500.0f) *
        ((float)MOTOR_MAX_DC - MOTOR_MIN_DC);
    // The baseline must be able to reach the target speed. Capping it any
    // lower would prevent the relay from ever starting if the motor needs
    // more than MAX_DC-RELAY_AMPLITUDE PWM to hold the target.
    baseline_dc = constrain(
        baseline_dc,
        MOTOR_MIN_DC + PID_AT_RELAY_AMPLITUDE,
        MOTOR_MAX_DC);

    relay_reference_speed = 0.0f;
    pid_integral = 0.0f;
    last_error = 0.0f;
    dc_state = DC_ENABLED;
    set_steering(0);

    Serial.println("\n=== PID AUTOTUNE START ===");
    Serial.print("Target speed: ");
    Serial.print(PID_AT_TARGET_SPEED);
    Serial.println(" mm/s");
    Serial.print("Initial baseline PWM: ");
    Serial.println(baseline_dc, 1);
    Serial.print("Relay amplitude: +/-");
    Serial.println(PID_AT_RELAY_AMPLITUDE);
}

void pid_autotune_update()
{
    if (!pid_autotune_is_active())
        return;

    set_steering(0);
    const unsigned long now = micros();
    const float actual_speed = measured_speed;
    const float error =
        (float)PID_AT_TARGET_SPEED - actual_speed;

    const float distance_delta =
        current_distance - last_distance_sample;
    if (distance_delta >= 0.0f)
        pid_atune_result.distance_forward += distance_delta;
    else
        pid_atune_result.distance_backward += -distance_delta;
    last_distance_sample = current_distance;

    if (fabsf(current_distance - start_distance) >
        PID_AT_MAX_DISTANCE_MM) {
        abort_tune("distance limit reached");
        return;
    }
    if (now - start_time_us > PID_AT_MAX_TIME_US) {
        abort_tune("time limit reached");
        return;
    }

    if (pid_atune_state == AT_ACCELERATING) {
        // Ramp the baseline toward the PWM needed to hold the target speed.
        // The gain and step clamp are chosen to reach the target within a
        // couple of seconds even with a large initial error.
        baseline_dc +=
            constrain(error * 1.2f * last_loop_time, -2.5f, 2.5f);
        baseline_dc = constrain(
            baseline_dc,
            MOTOR_MIN_DC + PID_AT_RELAY_AMPLITUDE,
            MOTOR_MAX_DC);
        set_dc(baseline_dc, false);

        if (fabsf(error) <= PID_AT_HYSTERESIS_MMS) {
            if (target_stable_since_us == 0)
                target_stable_since_us = now;
        } else {
            target_stable_since_us = 0;
        }

        // Periodic debug output so the acceleration behaviour is visible on
        // the serial monitor while tuning on real hardware.
        if (now - last_accel_debug_us >= 1000000) {
            last_accel_debug_us = now;
            Serial.print("Accel: speed=");
            Serial.print(actual_speed, 1);
            Serial.print(" mm/s, error=");
            Serial.print(error, 1);
            Serial.print(" mm/s, baseline=");
            Serial.println(baseline_dc, 1);
        }

        // Fast path: start the relay once the speed has been settled at the
        // target for a short while.
        if (target_stable_since_us != 0 &&
            now - target_stable_since_us >= 400000) {
            pid_atune_state = AT_RELAY_POS;
            relay_reference_speed = actual_speed;
            last_zero_cross_us = now;
            current_half_cycle_peak = 0.0f;
            Serial.print("Baseline learned: ");
            Serial.println(baseline_dc, 1);
            Serial.print("Relay reference speed: ");
            Serial.println(relay_reference_speed, 1);
            Serial.println("Relay oscillation started.");
            return;
        }

        // Fallback: if the target speed cannot be reached (motor/battery/load
        // limits), force the relay to start after the acceleration timeout so
        // tuning still completes at whatever speed the motor can sustain.
        if (now - accel_start_us >= PID_AT_ACCEL_TIMEOUT_US) {
            if (actual_speed < PID_AT_MIN_RELAY_SPEED_MMS) {
                abort_tune("speed too low to tune");
                return;
            }
            pid_atune_state = AT_RELAY_POS;
            relay_reference_speed = actual_speed;
            last_zero_cross_us = now;
            current_half_cycle_peak = 0.0f;
            Serial.print("Acceleration timeout; forcing relay start at ");
            Serial.print(relay_reference_speed, 1);
            Serial.println(" mm/s.");
            Serial.print("Baseline learned: ");
            Serial.println(baseline_dc, 1);
            Serial.println("Relay oscillation started.");
        }
        return;
    }

    // Measure the oscillation amplitude relative to where the relay
    // actually oscillates, not the nominal target speed.
    const float absolute_error =
        fabsf(relay_reference_speed - actual_speed);
    if (absolute_error > current_half_cycle_peak)
        current_half_cycle_peak = absolute_error;

    if (pid_atune_state == AT_RELAY_POS) {
        set_dc(
            baseline_dc + PID_AT_RELAY_AMPLITUDE,
            false);
        if (actual_speed >=
            relay_reference_speed + PID_AT_HYSTERESIS_MMS) {
            record_zero_crossing();
            pid_atune_state = AT_RELAY_NEG;
        }
    } else if (pid_atune_state == AT_RELAY_NEG) {
        set_dc(
            baseline_dc - PID_AT_RELAY_AMPLITUDE,
            false);
        if (actual_speed <=
            relay_reference_speed - PID_AT_HYSTERESIS_MMS) {
            record_zero_crossing();
            pid_atune_state = AT_RELAY_POS;
        }
    }

    if (measured_crossings < PID_AT_MIN_CYCLES * 2)
        return;

    pid_atune_state = AT_DONE;
    stop(false);

    if (!calculate_gains()) {
        pid_atune_result.valid = false;
        Serial.println(
            "PID autotune rejected unstable or insufficient oscillation data.");
        print_results();
        return;
    }

    pid_autotune_apply_gains();
    print_results();
}

void pid_autotune_stop()
{
    if (pid_atune_state == AT_IDLE)
        return;

    pid_atune_state = AT_IDLE;
    stop(false);
    Serial.println("PID autotune stopped.");
}

bool pid_autotune_is_active()
{
    return pid_atune_state != AT_IDLE &&
           pid_atune_state != AT_DONE;
}

const PIDAtuneResult& pid_autotune_get_result()
{
    return pid_atune_result;
}

void pid_autotune_apply_gains()
{
    if (pid_atune_state != AT_DONE ||
        !pid_atune_result.valid ||
        pid_atune_result.aborted) {
        Serial.println("No valid PID autotune result to apply.");
        return;
    }

    Kp = pid_atune_result.Kp;
    Ki = pid_atune_result.Ki;
    Kd = pid_atune_result.Kd;
    pid_integral = 0.0f;
    last_error = 0.0f;
    pid_atune_result.applied = true;

    Serial.println("Motor speed PID gains applied.");
}
