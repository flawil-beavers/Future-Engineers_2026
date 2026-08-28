#include "tof_diagnostic_test.h"

#include <Arduino.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "config.h"
#include "logger.h"
#include "mode_manager.h"
#include "motor_control.h"
#include "sensors.h"

#define Serial robot_logger

namespace {

constexpr uint8_t SWEEP_STAGE_COUNT = 3;
constexpr uint32_t SWEEP_BUDGETS_US[SWEEP_STAGE_COUNT] = {
    30000, 100000, 200000};
constexpr uint16_t MIN_SAMPLES = 3;
constexpr uint16_t MAX_SAMPLES = 100;
constexpr uint8_t MATRIX_STAGE_COUNT = 6;
constexpr VL53L4CX_DistanceModes MATRIX_MODES[MATRIX_STAGE_COUNT] = {
    VL53L4CX_DISTANCEMODE_MEDIUM, VL53L4CX_DISTANCEMODE_MEDIUM,
    VL53L4CX_DISTANCEMODE_MEDIUM, VL53L4CX_DISTANCEMODE_LONG,
    VL53L4CX_DISTANCEMODE_LONG, VL53L4CX_DISTANCEMODE_LONG};
constexpr uint32_t MATRIX_BUDGETS_US[MATRIX_STAGE_COUNT] = {
    30000, 100000, 200000, 30000, 100000, 200000};
constexpr uint8_t SETTLE_FRAMES_AFTER_RECONFIGURE = 5;

struct SensorStats {
  uint16_t samples = 0;
  uint16_t valid = 0;
  float distance_sum = 0.0f;
  float raw_sum = 0.0f;
  float signal_sum = 0.0f;
  float sigma_sum = 0.0f;
  float minimum = INFINITY;
  float maximum = -INFINITY;
};

struct TestState {
  bool active = false;
  bool sweep = false;
  bool matrix = false;
  char label[16] = {};
  uint16_t target_samples = 0;
  uint16_t captured_samples = 0;
  uint8_t stage = 0;
  uint8_t settle_frames = 0;
  uint32_t restore_budget_us = TOF_TIMING_BUDGET_US;
  VL53L4CX_DistanceModes restore_mode = TOF_DISTANCE_MODE;
  uint32_t last_sequence[TOF_SIDE_COUNT] = {};
  SensorStats stats[TOF_SIDE_COUNT];
};

TestState test;

void reset_stats()
{
  test.captured_samples = 0;
  test.stats[TOF_LEFT] = SensorStats{};
  test.stats[TOF_RIGHT] = SensorStats{};
}

void print_help()
{
  Serial.println("\n===== STATIONARY TOF DIAGNOSTICS =====");
  Serial.println("Keep the enable switch LOW; this test never drives the robot.");
  Serial.println("tof show                 : print the latest detailed sensor frame");
  Serial.println("tof capture <label> <n>  : capture 3-100 paired frames at current budget");
  Serial.println("tof sweep <label> <n>    : capture at 30, 100, and 200 ms");
  Serial.println("tof matrix <label> <n>   : test MEDIUM/LONG at 30/100/200 ms");
  Serial.println("tof budget <ms>          : set both sensors to 20-500 ms (RAM only)");
  Serial.println("tof stop                 : stop capture and restore the prior budget");
  Serial.println("Recommended sequence at a measured 500 mm:");
  Serial.println("  tof sweep black 10");
  Serial.println("  cover the same target with white paper, without moving the robot");
  Serial.println("  tof sweep white 10");
  Serial.println("For a deeper follow-up run: tof matrix black 10");
  Serial.println("Object flags: H=hardware-valid, A=accepted by software, S=selected");
  Serial.println("======================================\n");
}

void print_snapshot(const char *label, uint16_t sample_number,
                    TofSensor side, const TofDiagnosticSnapshot &snapshot)
{
  Serial.print("[TOF FRAME] label="); Serial.print(label);
  Serial.print(" sample="); Serial.print(sample_number);
  Serial.print(" side="); Serial.print(side == TOF_LEFT ? "L" : "R");
  Serial.print(" mode="); Serial.print(static_cast<int>(snapshot.distance_mode));
  Serial.print(" budget_us="); Serial.print(snapshot.timing_budget_us);
  Serial.print(" filtered_mm="); Serial.print(snapshot.filtered_distance_mm, 1);
  Serial.print(" selected_raw_mm="); Serial.print(snapshot.selected_raw_distance_mm, 1);
  Serial.print(" signal_mcps="); Serial.print(snapshot.selected_signal_mcps, 4);
  Serial.print(" sigma_mm="); Serial.print(snapshot.selected_sigma_mm, 2);
  Serial.print(" objects="); Serial.print(snapshot.reported_object_count);
  Serial.print(" candidates=[");
  for (uint8_t i = 0; i < snapshot.stored_object_count; ++i) {
    if (i > 0) Serial.print(";");
    const TofObjectDiagnostic &object = snapshot.objects[i];
    Serial.print(i); Serial.print(":");
    Serial.print(object.distance_mm); Serial.print("/");
    Serial.print(object.signal_mcps, 4); Serial.print("/");
    Serial.print(object.sigma_mm, 2); Serial.print("/st");
    Serial.print(object.range_status); Serial.print("/");
    if (object.hardware_valid) Serial.print("H");
    if (object.filter_accepted) Serial.print("A");
    if (snapshot.selected_object_index == static_cast<int8_t>(i))
      Serial.print("S");
    if (!object.hardware_valid && !object.filter_accepted)
      Serial.print("-");
  }
  Serial.println("]");
}

void add_stats(SensorStats &stats, const TofDiagnosticSnapshot &snapshot)
{
  ++stats.samples;
  const float distance = snapshot.filtered_distance_mm;
  if (distance <= 0.0f || distance >= TOF_OUT_OF_RANGE_MM)
    return;

  ++stats.valid;
  stats.distance_sum += distance;
  stats.raw_sum += snapshot.selected_raw_distance_mm;
  stats.signal_sum += snapshot.selected_signal_mcps;
  stats.sigma_sum += snapshot.selected_sigma_mm;
  stats.minimum = fminf(stats.minimum, distance);
  stats.maximum = fmaxf(stats.maximum, distance);
}

void print_sensor_summary(TofSensor side, const SensorStats &stats)
{
  Serial.print(" side="); Serial.print(side == TOF_LEFT ? "L" : "R");
  Serial.print(" valid="); Serial.print(stats.valid);
  Serial.print("/"); Serial.print(stats.samples);
  if (stats.valid == 0) {
    Serial.print(" mean_mm=INVALID");
    return;
  }
  const float divisor = static_cast<float>(stats.valid);
  Serial.print(" mean_mm="); Serial.print(stats.distance_sum / divisor, 1);
  Serial.print(" min_mm="); Serial.print(stats.minimum, 1);
  Serial.print(" max_mm="); Serial.print(stats.maximum, 1);
  Serial.print(" raw_mean_mm="); Serial.print(stats.raw_sum / divisor, 1);
  Serial.print(" signal_mean_mcps="); Serial.print(stats.signal_sum / divisor, 4);
  Serial.print(" sigma_mean_mm="); Serial.print(stats.sigma_sum / divisor, 2);
}

void print_summary(const TofDiagnosticSnapshot &left,
                   const TofDiagnosticSnapshot &right)
{
  Serial.print("[TOF SUMMARY] label="); Serial.print(test.label);
  Serial.print(" mode="); Serial.print(static_cast<int>(right.distance_mode));
  Serial.print(" budget_us="); Serial.print(right.timing_budget_us);
  print_sensor_summary(TOF_LEFT, test.stats[TOF_LEFT]);
  print_sensor_summary(TOF_RIGHT, test.stats[TOF_RIGHT]);
  Serial.println();
}

void set_last_sequences()
{
  TofDiagnosticSnapshot snapshot;
  for (int i = 0; i < TOF_SIDE_COUNT; ++i) {
    test.last_sequence[i] =
        get_tof_diagnostic_snapshot(static_cast<TofSensor>(i), snapshot)
            ? snapshot.sequence
            : 0;
  }
}

void finish_test(bool cancelled)
{
  if (test.sweep || test.matrix)
    sensors_configure_tof_for_test(test.restore_mode,
                                   test.restore_budget_us);
  Serial.print("[TOF TEST] ");
  Serial.print(cancelled ? "cancelled" : "complete");
  Serial.print("; restored_budget_us=");
  Serial.println(test.restore_budget_us);
  test.active = false;
}

bool configure_stage()
{
  VL53L4CX_DistanceModes mode = test.restore_mode;
  uint32_t budget_us = test.restore_budget_us;
  if (test.matrix) {
    mode = MATRIX_MODES[test.stage];
    budget_us = MATRIX_BUDGETS_US[test.stage];
  } else if (test.sweep) {
    budget_us = SWEEP_BUDGETS_US[test.stage];
  }
  return sensors_configure_tof_for_test(mode, budget_us);
}

bool parse_capture(const char *message, bool sweep, bool matrix)
{
  char action[12] = {};
  char label[16] = {};
  int samples = 0;
  // Do not use a trailing %c here: many serial terminals retain CR from CRLF.
  const int fields = sscanf(message, "tof %11s %15s %d",
                            action, label, &samples);
  if (fields != 3 || samples < MIN_SAMPLES || samples > MAX_SAMPLES) {
    if (matrix)
      Serial.println("Usage: tof matrix <label> <samples 3-100>");
    else
      Serial.println(sweep
          ? "Usage: tof sweep <label> <samples 3-100>"
          : "Usage: tof capture <label> <samples 3-100>");
    return true;
  }
  if (system_enabled) {
    Serial.println("Rejected: lower the physical enable switch first.");
    return true;
  }

  if (test.active)
    finish_test(true);
  mode_stop_all();
  test = TestState{};
  test.active = true;
  test.sweep = sweep;
  test.matrix = matrix;
  strncpy(test.label, label, sizeof(test.label) - 1);
  test.target_samples = static_cast<uint16_t>(samples);

  TofDiagnosticSnapshot current;
  if (get_tof_diagnostic_snapshot(TOF_RIGHT, current) &&
      current.timing_budget_us > 0) {
    test.restore_budget_us = current.timing_budget_us;
    test.restore_mode = current.distance_mode;
  }

  if (sweep || matrix) {
    if (!configure_stage()) {
      Serial.println("[TOF TEST] sensor reconfiguration failed");
      finish_test(true);
      return true;
    }
    test.settle_frames = SETTLE_FRAMES_AFTER_RECONFIGURE;
  }
  set_last_sequences();
  reset_stats();
  Serial.print("[TOF TEST] started label="); Serial.print(test.label);
  Serial.print(" samples_per_budget="); Serial.print(test.target_samples);
  Serial.print(" mode=");
  Serial.println(matrix ? "matrix" : sweep ? "sweep" : "capture");
  return true;
}

}  // namespace

bool tof_diagnostic_handle_command(const char *message)
{
  if (strncmp(message, "tof", 3) != 0 ||
      (message[3] != '\0' && message[3] != ' '))
    return false;

  char action[12] = {};
  if (sscanf(message, "tof %11s", action) != 1 ||
      strcmp(action, "help") == 0) {
    print_help();
    return true;
  }
  if (strcmp(action, "show") == 0) {
    TofDiagnosticSnapshot left, right;
    if (!get_tof_diagnostic_snapshot(TOF_LEFT, left) ||
        !get_tof_diagnostic_snapshot(TOF_RIGHT, right)) {
      Serial.println("No complete ToF frame is available yet.");
      return true;
    }
    print_snapshot("show", 1, TOF_LEFT, left);
    print_snapshot("show", 1, TOF_RIGHT, right);
    return true;
  }
  if (strcmp(action, "capture") == 0)
    return parse_capture(message, false, false);
  if (strcmp(action, "sweep") == 0)
    return parse_capture(message, true, false);
  if (strcmp(action, "matrix") == 0)
    return parse_capture(message, false, true);
  if (strcmp(action, "stop") == 0) {
    if (test.active) finish_test(true);
    else Serial.println("No ToF diagnostic capture is active.");
    return true;
  }
  if (strcmp(action, "budget") == 0) {
    int budget_ms = 0;
    if (sscanf(message, "tof budget %d", &budget_ms) != 1 ||
        budget_ms < 20 || budget_ms > 500) {
      Serial.println("Usage: tof budget <20-500 ms>");
      return true;
    }
    if (test.active) finish_test(true);
    TofDiagnosticSnapshot current;
    const VL53L4CX_DistanceModes mode =
        get_tof_diagnostic_snapshot(TOF_RIGHT, current)
            ? current.distance_mode
            : TOF_DISTANCE_MODE;
    if (!sensors_configure_tof_for_test(
            mode, static_cast<uint32_t>(budget_ms) * 1000UL)) {
      Serial.println("[TOF] sensor reconfiguration failed");
      return true;
    }
    Serial.print("[TOF] requested_budget_us=");
    Serial.println(static_cast<uint32_t>(budget_ms) * 1000UL);
    return true;
  }

  Serial.println("Unknown ToF command. Send 'tof help'.");
  return true;
}

void tof_diagnostic_update()
{
  if (!test.active)
    return;

  TofDiagnosticSnapshot snapshots[TOF_SIDE_COUNT];
  if (!get_tof_diagnostic_snapshot(TOF_LEFT, snapshots[TOF_LEFT]) ||
      !get_tof_diagnostic_snapshot(TOF_RIGHT, snapshots[TOF_RIGHT]))
    return;
  if (snapshots[TOF_LEFT].sequence == test.last_sequence[TOF_LEFT] ||
      snapshots[TOF_RIGHT].sequence == test.last_sequence[TOF_RIGHT])
    return;

  test.last_sequence[TOF_LEFT] = snapshots[TOF_LEFT].sequence;
  test.last_sequence[TOF_RIGHT] = snapshots[TOF_RIGHT].sequence;
  if (test.settle_frames > 0) {
    --test.settle_frames;
    return;
  }

  ++test.captured_samples;
  for (int i = 0; i < TOF_SIDE_COUNT; ++i) {
    const TofSensor side = static_cast<TofSensor>(i);
    add_stats(test.stats[i], snapshots[i]);
    print_snapshot(test.label, test.captured_samples, side, snapshots[i]);
  }

  if (test.captured_samples < test.target_samples)
    return;

  print_summary(snapshots[TOF_LEFT], snapshots[TOF_RIGHT]);
  const uint8_t stage_count = test.matrix
      ? MATRIX_STAGE_COUNT
      : test.sweep ? SWEEP_STAGE_COUNT : 1;
  if ((!test.sweep && !test.matrix) || ++test.stage >= stage_count) {
    finish_test(false);
    return;
  }

  if (!configure_stage()) {
    Serial.println("[TOF TEST] sensor reconfiguration failed");
    finish_test(true);
    return;
  }
  test.settle_frames = SETTLE_FRAMES_AFTER_RECONFIGURE;
  reset_stats();
  set_last_sequences();
  Serial.print("[TOF TEST] next_budget_us=");
  Serial.println(test.matrix
      ? MATRIX_BUDGETS_US[test.stage]
      : SWEEP_BUDGETS_US[test.stage]);
}
