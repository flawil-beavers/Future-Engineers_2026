/**
 * @file pid_autotune.cpp
 * @brief Symmetric relay-feedback tuning for the DC motor speed PI.
 */

#include "pid_autotune.h"
#include "motor_control.h"
#include "config.h"
#include "logger.h"
#include <float.h>
#define Serial robot_logger

PIDAtuneState pid_atune_state = AT_IDLE;
PIDAtuneResult pid_atune_result;

static float start_distance = 0.0f;
static float last_distance_sample = 0.0f;
static unsigned long start_time_us = 0;
static unsigned long accel_start_us = 0;
static unsigned long last_crossing_us = 0;
static unsigned long last_accel_debug_us = 0;
static unsigned long last_processed_speed_sample = 0;
static float baseline_dc = 0.0f;
static float relay_reference_speed = 0.0f;
static float configured_target_speed = PID_AT_TARGET_SPEED;
static float configured_baseline_dc = PID_AT_INITIAL_BASELINE_DC;
static float configured_relay_amplitude = PID_AT_RELAY_AMPLITUDE;
static float baseline_speed_sum = 0.0f;
static float baseline_speed_sum_squared = 0.0f;
static int baseline_speed_samples = 0;
static int warmup_crossings = 0;
static int measured_crossings = 0;
static int high_peak_count = 0;
static int low_peak_count = 0;
static int period_count = 0;
static float high_peak = -FLT_MAX;
static float low_peak = FLT_MAX;
static float sum_high_peaks = 0.0f;
static float sum_low_peaks = 0.0f;
static float sum_half_periods = 0.0f;
static float sum_half_periods_squared = 0.0f;

static void print_results()
{
    Serial.println("\n=== PID AUTOTUNE RESULT ===");
    Serial.print("Status: ");
    Serial.println(pid_atune_result.applied
        ? "VALID AND APPLIED"
        : (pid_atune_result.aborted ? "ABORTED" : "INVALID"));
    Serial.print("Measured crossings: ");
    Serial.println(pid_atune_result.zero_crossings);
    Serial.print("Center speed: ");
    Serial.print(pid_atune_result.center_speed, 2);
    Serial.println(" mm/s");
    Serial.print("Speed amplitude: ");
    Serial.print(pid_atune_result.speed_amplitude, 2);
    Serial.println(" mm/s");
    Serial.print("Amplitude asymmetry: ");
    Serial.print(pid_atune_result.amplitude_asymmetry * 100.0f, 1);
    Serial.println(" %");
    Serial.print("Ultimate period Tu: ");
    Serial.print(pid_atune_result.Tu, 4);
    Serial.println(" s");
    Serial.print("Period variation: ");
    Serial.print(pid_atune_result.period_variation * 100.0f, 1);
    Serial.println(" %");
    Serial.print("Ku: "); Serial.println(pid_atune_result.Ku, 4);
    Serial.print("Kp: "); Serial.println(pid_atune_result.Kp, 4);
    Serial.print("Ki: "); Serial.println(pid_atune_result.Ki, 4);
    Serial.print("Kd: "); Serial.println(pid_atune_result.Kd, 4);
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

static void reset_phase_peak(bool positive_phase, float speed)
{
    if (positive_phase)
        high_peak = speed;
    else
        low_peak = speed;
}

static void record_crossing(bool completed_positive_phase)
{
    const unsigned long now = micros();
    const float half_period =
        (now - last_crossing_us) / 1000000.0f;
    last_crossing_us = now;

    if (warmup_crossings < PID_AT_WARMUP_CROSSINGS) {
        ++warmup_crossings;
        return;
    }

    if (half_period <= 0.02f || half_period >= 5.0f)
        return;

    sum_half_periods += half_period;
    sum_half_periods_squared += half_period * half_period;
    ++period_count;
    ++measured_crossings;

    if (completed_positive_phase) {
        sum_high_peaks += high_peak;
        ++high_peak_count;
    } else {
        sum_low_peaks += low_peak;
        ++low_peak_count;
    }
}

static bool enough_measurements()
{
    return high_peak_count >= PID_AT_MIN_CYCLES &&
           low_peak_count >= PID_AT_MIN_CYCLES &&
           period_count >= PID_AT_MIN_CYCLES * 2;
}

static bool calculate_gains()
{
    if (!enough_measurements())
        return false;

    const float average_high = sum_high_peaks / high_peak_count;
    const float average_low = sum_low_peaks / low_peak_count;
    const float upper_amplitude = average_high - relay_reference_speed;
    const float lower_amplitude = relay_reference_speed - average_low;
    const float amplitude = (average_high - average_low) * 0.5f;
    const float center = (average_high + average_low) * 0.5f;
    const float amplitude_sum = upper_amplitude + lower_amplitude;
    const float asymmetry = amplitude_sum > 0.0f
        ? fabsf(upper_amplitude - lower_amplitude) / amplitude_sum
        : 1.0f;
    const float average_half_period = sum_half_periods / period_count;
    float variance = sum_half_periods_squared / period_count -
                     average_half_period * average_half_period;
    if (variance < 0.0f)
        variance = 0.0f;
    const float period_variation = average_half_period > 0.0f
        ? sqrtf(variance) / average_half_period
        : 1.0f;
    const float ultimate_period = 2.0f * average_half_period;

    pid_atune_result.speed_amplitude = amplitude;
    pid_atune_result.center_speed = center;
    pid_atune_result.amplitude_asymmetry = asymmetry;
    pid_atune_result.period_variation = period_variation;
    pid_atune_result.Tu = ultimate_period;
    pid_atune_result.zero_crossings = measured_crossings;
    pid_atune_result.overshoot = amplitude / relay_reference_speed;

    const float hysteresis = PID_AT_HYSTERESIS_MMS;
    const float corrected_amplitude_squared =
        amplitude * amplitude - hysteresis * hysteresis;
    if (!isfinite(amplitude) || !isfinite(ultimate_period) ||
        amplitude < PID_AT_MIN_SPEED_AMPLITUDE ||
        corrected_amplitude_squared <= 0.0f ||
        fabsf(center - relay_reference_speed) > PID_AT_MAX_CENTER_ERROR_MMS ||
        asymmetry > PID_AT_MAX_AMPLITUDE_ASYMMETRY ||
        period_variation > PID_AT_MAX_PERIOD_VARIATION ||
        ultimate_period <= 0.04f)
        return false;

    // Relay describing function corrected for the switching hysteresis.
    pid_atune_result.Ku =
        4.0f * configured_relay_amplitude /
        (PI * sqrtf(corrected_amplitude_squared));

    // Tyreus-Luyben PI. Encoder-derived speed is too noisy for a useful D term.
    pid_atune_result.Kp = pid_atune_result.Ku / 3.2f;
    const float integral_time = 2.2f * ultimate_period;
    pid_atune_result.Ki = pid_atune_result.Kp / integral_time;
    pid_atune_result.Kd = 0.0f;

    pid_atune_result.valid =
        isfinite(pid_atune_result.Kp) &&
        isfinite(pid_atune_result.Ki) &&
        pid_atune_result.Kp > 0.0f && pid_atune_result.Kp <= 10.0f &&
        pid_atune_result.Ki > 0.0f && pid_atune_result.Ki <= 10.0f;
    return pid_atune_result.valid;
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
    last_crossing_us = 0;
    last_processed_speed_sample = speed_measurement_count;
    warmup_crossings = measured_crossings = 0;
    high_peak_count = low_peak_count = period_count = 0;
    high_peak = -FLT_MAX;
    low_peak = FLT_MAX;
    sum_high_peaks = sum_low_peaks = 0.0f;
    sum_half_periods = sum_half_periods_squared = 0.0f;
    baseline_speed_sum = 0.0f;
    baseline_speed_sum_squared = 0.0f;
    baseline_speed_samples = 0;

    // Start near the measured operating point. The slow learner below still
    // compensates battery voltage, floor and drivetrain differences.
    baseline_dc = configured_baseline_dc;
    baseline_dc = constrain(
        baseline_dc,
        MOTOR_MIN_DC + configured_relay_amplitude,
        MOTOR_MAX_DC - configured_relay_amplitude);

    relay_reference_speed = 0.0f;
    pid_integral = 0.0f;
    last_error = 0.0f;
    dc_state = DC_ENABLED;
    set_steering(0);

    Serial.println("\n=== PID AUTOTUNE START ===");
    Serial.print("Target band: "); Serial.print(configured_target_speed);
    Serial.println(" mm/s");
    Serial.print("Initial baseline PWM: "); Serial.println(baseline_dc, 1);
    Serial.print("Relay amplitude: +/-"); Serial.println(configured_relay_amplitude);
}

void pid_autotune_update()
{
    if (!pid_autotune_is_active())
        return;

    set_steering(0);
    const unsigned long now = micros();
    const float distance_delta = current_distance - last_distance_sample;
    if (distance_delta >= 0.0f)
        pid_atune_result.distance_forward += distance_delta;
    else
        pid_atune_result.distance_backward -= distance_delta;
    last_distance_sample = current_distance;

    if (fabsf(current_distance - start_distance) > PID_AT_MAX_DISTANCE_MM) {
        abort_tune("distance limit reached");
        return;
    }
    if (now - start_time_us > PID_AT_MAX_TIME_US) {
        abort_tune("time limit reached");
        return;
    }

    const float actual_speed = measured_speed;
    const bool new_speed_sample =
        last_processed_speed_sample != speed_measurement_count;

    if (pid_atune_state == AT_ACCELERATING) {
        // Keep PWM fixed. Closing another feedback loop around the baseline
        // would create its own oscillation and corrupt relay identification.
        set_dc(baseline_dc, false);
        if (!new_speed_sample)
            return;
        last_processed_speed_sample = speed_measurement_count;

        const unsigned long baseline_elapsed = now - accel_start_us;
        if (baseline_elapsed >= PID_AT_BASELINE_SAMPLE_START_US) {
            baseline_speed_sum += actual_speed;
            baseline_speed_sum_squared += actual_speed * actual_speed;
            ++baseline_speed_samples;
        }

        if (now - last_accel_debug_us >= 1000000) {
            last_accel_debug_us = now;
            Serial.print("Accel: speed="); Serial.print(actual_speed, 1);
            Serial.print(" mm/s at fixed PWM "); Serial.println(baseline_dc, 1);
        }

        if (baseline_elapsed < PID_AT_BASELINE_SETTLE_US)
            return;

        if (baseline_speed_samples < 5) {
            abort_tune("not enough fixed-PWM speed samples");
            return;
        }

        relay_reference_speed = baseline_speed_sum / baseline_speed_samples;
        float variance = baseline_speed_sum_squared / baseline_speed_samples -
                         relay_reference_speed * relay_reference_speed;
        if (variance < 0.0f)
            variance = 0.0f;
        const float relative_variation = relay_reference_speed > 0.0f
            ? sqrtf(variance) / relay_reference_speed
            : 1.0f;
        if (relay_reference_speed < PID_AT_MIN_RELAY_SPEED_MMS ||
            relative_variation > PID_AT_MAX_BASELINE_SPEED_VARIATION) {
            abort_tune("fixed-PWM baseline speed is too low or unstable");
            return;
        }

        pid_atune_state = AT_RELAY_POS;
        last_crossing_us = now;
        reset_phase_peak(true, actual_speed);
        Serial.print("Fixed baseline PWM: "); Serial.println(baseline_dc, 1);
        Serial.print("Relay reference speed: ");
        Serial.println(relay_reference_speed, 1);
        Serial.print("Baseline speed variation: ");
        Serial.print(relative_variation * 100.0f, 1);
        Serial.println(" %");
        Serial.println("Relay oscillation started.");
        return;
    }

    // Relay transitions and extrema are evaluated exactly once per new
    // encoder-speed sample, not repeatedly using stale filtered data.
    if (!new_speed_sample)
        return;
    last_processed_speed_sample = speed_measurement_count;

    if (pid_atune_state == AT_RELAY_POS) {
        set_dc(baseline_dc + configured_relay_amplitude, false);
        if (actual_speed > high_peak)
            high_peak = actual_speed;
        if (actual_speed >= relay_reference_speed + PID_AT_HYSTERESIS_MMS) {
            record_crossing(true);
            pid_atune_state = AT_RELAY_NEG;
            reset_phase_peak(false, actual_speed);
        }
    } else if (pid_atune_state == AT_RELAY_NEG) {
        set_dc(baseline_dc - configured_relay_amplitude, false);
        if (actual_speed < low_peak)
            low_peak = actual_speed;
        if (actual_speed <= relay_reference_speed - PID_AT_HYSTERESIS_MMS) {
            record_crossing(false);
            pid_atune_state = AT_RELAY_POS;
            reset_phase_peak(true, actual_speed);
        }
    }

    if (!enough_measurements())
        return;

    pid_atune_state = AT_DONE;
    stop(false);
    if (!calculate_gains()) {
        Serial.println("PID autotune rejected asymmetric or unstable data.");
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
    return pid_atune_state != AT_IDLE && pid_atune_state != AT_DONE;
}

const PIDAtuneResult& pid_autotune_get_result()
{
    return pid_atune_result;
}

void pid_autotune_apply_gains()
{
    if (pid_atune_state != AT_DONE || !pid_atune_result.valid ||
        pid_atune_result.aborted) {
        Serial.println("No valid PID autotune result to apply.");
        return;
    }
    const char *band = "HIGH";
    if (configured_target_speed <= low_speed_gain_end) {
        low_speed_cruise_kp = pid_atune_result.Kp;
        low_speed_cruise_ki = pid_atune_result.Ki;
        band = "LOW";
    } else if (configured_target_speed <= mid_speed_gain_end) {
        mid_speed_cruise_kp = pid_atune_result.Kp;
        mid_speed_cruise_ki = pid_atune_result.Ki;
        band = "MID";
    } else {
        Kp = pid_atune_result.Kp;
        Ki = pid_atune_result.Ki;
    }
    Kd = pid_atune_result.Kd;
    if (relay_reference_speed > 1.0f) {
        motor_speed_ff = fmaxf(
            0.0f,
            (baseline_dc - motor_static_ff) / relay_reference_speed);
    }
    pid_integral = 0.0f;
    last_error = 0.0f;
    pid_atune_result.applied = true;
    Serial.print("Motor speed feedforward applied, Kv: ");
    Serial.println(motor_speed_ff, 5);
    Serial.print("Motor cruise PI gains applied to ");
    Serial.print(band);
    Serial.println(" speed band (runtime only). Use 'pid export'.");
}

bool pid_autotune_configure(float target, float baseline, float relay)
{
    if (!isfinite(target) || !isfinite(baseline) || !isfinite(relay) ||
        target < PID_AT_MIN_RELAY_SPEED_MMS || baseline <= MOTOR_MIN_DC ||
        baseline >= MOTOR_MAX_DC || relay < 2.0f ||
        baseline - relay < MOTOR_MIN_DC ||
        baseline + relay > MOTOR_MAX_DC)
        return false;
    configured_target_speed = target;
    configured_baseline_dc = baseline;
    configured_relay_amplitude = relay;
    return true;
}

void pid_autotune_print_config()
{
    Serial.print("Autotune target band: ");
    Serial.print(configured_target_speed, 1);
    Serial.println(" mm/s");
    Serial.print("Fixed baseline PWM: ");
    Serial.println(configured_baseline_dc, 1);
    Serial.print("Relay step: +/-");
    Serial.println(configured_relay_amplitude, 1);
}
