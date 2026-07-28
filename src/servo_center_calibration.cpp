/**
 * @file servo_center_calibration.cpp
 * @brief Straight-line servo-center calibration implementation
 * 
 * Calibration state machine:
 *   SC_IDLE → SC_DRIVING → SC_DONE
 * 
 * The robot drives straight while using gyro-follow steering to maintain
 * its initial heading. Over the course of the run, it builds a weighted
 * running mean of the steering corrections, which directly estimates
 * the neutral servo position (the servo value that makes the robot
 * drive straight without heading drift).
 */

#include "servo_center_calibration.h"
#include "config.h"
#include "motor_control.h"
#include "sensors.h"
#include "wall_follower.h"
#include "logger.h"
#define Serial robot_logger

// ==========================================
// EXTERNAL DEPENDENCIES
// ==========================================

extern float current_distance;
extern float last_loop_time;

// ==========================================
// SERVO CENTER CALIBRATION STATE
// ==========================================

ServoCenterState servo_center_state = SC_IDLE;

// ==========================================
// INTERNAL STATE
// ==========================================

static int sc_center_best_value = SERVO_CENTER;
static float sc_center_target_heading = 0.0f;
static float sc_center_last_heading_error = 0.0f;
static float sc_center_weighted_mean = 0.0f;
static float sc_center_weight_sum = 0.0f;
static float sc_center_weighted_m2 = 0.0f;
static float sc_center_uncertainty = 0.0f;
static unsigned long sc_center_last_debug_time = 0;
static float sc_start_distance = 0;
static float sc_drive_start_time = 0;

// ==========================================
// INTERNAL HELPERS
// ==========================================

/**
 * @brief Update the weighted running mean estimate of the neutral servo position.
 * 
 * Each sample is weighted by how trustworthy it is. Samples taken when the
 * heading error is small are more reliable, while large errors or strong
 * steering saturation are treated as less certain. The resulting estimate
 * is therefore less sensitive to noisy outliers and gives a useful
 * uncertainty value for the final servo-center estimate.
 */
static void update_center_estimate(float steering_cmd, float heading_error)
{
    float abs_heading_error = fabsf(heading_error);
    float certainty = 1.0f / (1.0f + abs_heading_error / 8.0f);
    certainty = constrain(certainty, 0.05f, 1.0f);

    if (fabsf(steering_cmd) > 45.0f) {
        certainty *= 0.6f;
    }

    float sample_weight = constrain(certainty, 0.05f, 1.0f);

    float previous_mean = sc_center_weighted_mean;
    sc_center_weight_sum += sample_weight;
    float delta = steering_cmd - previous_mean;
    sc_center_weighted_mean += delta * sample_weight / sc_center_weight_sum;
    float delta2 = steering_cmd - sc_center_weighted_mean;
    sc_center_weighted_m2 += sample_weight * delta * delta2;

    float weighted_variance = (sc_center_weight_sum > 0.0f) ? (sc_center_weighted_m2 / sc_center_weight_sum) : 0.0f;
    float weighted_std_dev = sqrtf(fmaxf(0.0f, weighted_variance));
    float std_error_of_mean = (sc_center_weight_sum > 1.0f) ? (weighted_std_dev / sqrtf(sc_center_weight_sum)) : weighted_std_dev;
    sc_center_uncertainty = std_error_of_mean;
}

/**
 * @brief Initialize and start the straight drive phase
 */
static void start_center_drive()
{
    sc_center_target_heading = get_angle();
    sc_center_last_heading_error = 0.0f;
    gyro_follower_reset_filter();
    sc_center_weighted_mean = 0.0f;
    sc_center_weight_sum = 0.0f;
    sc_center_weighted_m2 = 0.0f;
    sc_center_uncertainty = 0.0f;
    sc_center_last_debug_time = millis();
    sc_start_distance = current_distance;
    sc_drive_start_time = millis();

    set_steering(0);
    set_speed(CAL_SPEED_MMS);
    servo_center_state = SC_DRIVING;

    Serial.println("Starting straight 2 m servo-center calibration...");
    Serial.println("The robot will keep the original heading and estimate the neutral servo position.");
    Serial.print("Driving for ");
    Serial.print(CAL_CENTER_DISTANCE_MM / 1000.0f, 1);
    Serial.println(" m or until the safety timeout is reached.");
}

// ==========================================
// PUBLIC FUNCTIONS
// ==========================================

void servo_center_cal_start()
{
    servo_center_state = SC_DRIVING;
    sc_center_best_value = SERVO_CENTER;

    Serial.println("\n\n========================================");
    Serial.println("STRAIGHT SERVO-CENTER CALIBRATION");
    Serial.println("========================================");
    Serial.print("Using gyro-follow steering gains Kp=");
    Serial.print(gyro_follower_get_gyro_kp(), 3);
    Serial.print(", Kd=");
    Serial.println(gyro_follower_get_gyro_kd(), 3);

    start_center_drive();
}

void servo_center_cal_update()
{
    if (servo_center_state != SC_DRIVING) {
        return;
    }

    float heading_error = get_angle() - sc_center_target_heading;
    float steering_cmd = gyro_follower_compute_steering(heading_error, sc_center_last_heading_error, last_loop_time);
    sc_center_last_heading_error = heading_error;

    if (steering_cmd > 60.0f) steering_cmd = 60.0f;
    if (steering_cmd < -60.0f) steering_cmd = -60.0f;

    int steering_command = (int)steering_cmd;
    set_steering(steering_command);

    // The ideal neutral servo position is the value that makes the gyro-follow
    // controller need zero steering over a straight run. We estimate it by
    // combining the steering corrections in a weighted average that gives more
    // trust to samples taken when the heading error is small and less trust to
    // noisy or saturated samples. That makes the result more robust than a plain
    // mean and gives a useful uncertainty value for how confident we are in the
    // final estimate.
    update_center_estimate(steering_cmd, heading_error);

    float distance_traveled = current_distance - sc_start_distance;
    bool timed_out = (millis() - sc_drive_start_time) >= CAL_CENTER_MAX_TIME_MS;

    if ((millis() - sc_center_last_debug_time) >= CAL_CENTER_DEBUG_INTERVAL_MS) {
        sc_center_last_debug_time = millis();
        float mean_steering = sc_center_weighted_mean;
        float certainty = 100.0f / (1.0f + sc_center_uncertainty);
        int current_estimate = constrain((int)roundf(SERVO_CENTER + mean_steering), SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);

        Serial.print("[CAL] servo=");
        Serial.print(SERVO_CENTER);
        Serial.print(" estimate=");
        Serial.print(current_estimate);
        Serial.print(" uncertainty=±");
        Serial.print(sc_center_uncertainty, 1);
        Serial.print(" deg certainty=");
        Serial.print(certainty, 0);
        Serial.print("% heading_error=");
        Serial.print(heading_error, 1);
        Serial.println(" deg");
    }

    if (distance_traveled >= CAL_CENTER_DISTANCE_MM || timed_out) {
        float average_steering = sc_center_weighted_mean;
        sc_center_best_value = constrain((int)roundf(SERVO_CENTER + average_steering), SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);

        set_steering(0);
        stop(false);
        servo_center_state = SC_DONE;

        Serial.println("Straight calibration complete.");
        Serial.print("Final estimate: servo center = ");
        Serial.print(sc_center_best_value);
        Serial.print(" (weighted steering = ");
        Serial.print(average_steering, 2);
        Serial.print(" deg, uncertainty = ±");
        Serial.print(sc_center_uncertainty, 2);
        Serial.print(" deg, certainty = ");
        Serial.print(100.0f / (1.0f + sc_center_uncertainty), 0);
        Serial.println("%)");
    }
}

void servo_center_cal_stop()
{
    servo_center_state = SC_IDLE;
    stop(false);
    Serial.println("Servo-center calibration stopped.");
}

bool servo_center_cal_is_active()
{
    return servo_center_state != SC_IDLE && servo_center_state != SC_DONE;
}