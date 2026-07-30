#pragma once

/**
 * @file pid_autotune.h
 * @brief PID speed controller auto-tuning using relay feedback method
 * 
 * Uses the Åström-Hägglund relay feedback method to automatically
 * determine optimal PID gains for the motor speed controller.
 * 
 * The robot drives straight at a target speed while a relay controller
 * oscillates the DC around a baseline. From the resulting limit cycles,
 * the ultimate gain (Ku) and ultimate period (Tu) are measured, and
 * Ziegler-Nichols tuning rules are applied.
 * 
 * Safety constraints:
 *   - Steering is locked to 0° (straight) throughout
 *   - Robot must not travel more than 1m forward or backward from start
 *   - A maximum tuning time prevents runaway
 *   - Enable switch works normally (pause/resume)
 */

#include <Arduino.h>

// ==========================================
// PID AUTOTUNE STATE MACHINE
// ==========================================

/**
 * @brief States for the PID autotune state machine
 */
enum PIDAtuneState {
    AT_IDLE,            ///< Not running
    AT_ACCELERATING,    ///< Accelerating to target speed
    AT_RELAY_POS,       ///< Relay ON (above baseline DC)
    AT_RELAY_NEG,       ///< Relay OFF (below baseline DC)
    AT_MEASURING,       ///< Accumulating zero-crossing data
    AT_DONE             ///< Tuning complete, results available
};

// ==========================================
// PID AUTOTUNE RESULT STRUCTURE
// ==========================================

/**
 * @brief Results from the PID autotune process
 */
struct PIDAtuneResult {
    float Ku;               ///< Ultimate gain
    float Tu;               ///< Ultimate period (seconds)
    float Kp;               ///< Recommended proportional gain
    float Ki;               ///< Recommended integral gain
    float Kd;               ///< Recommended derivative gain
    float overshoot;        ///< Measured overshoot (fraction of target)
    int   zero_crossings;   ///< Number of zero-crossings measured
    float distance_forward; ///< Total forward distance traveled (mm)
    float distance_backward;///< Total backward distance traveled (mm)
    bool  aborted;          ///< True if tuning was aborted (distance/time limit)
};

// ==========================================
// EXTERNAL STATE
// ==========================================

extern PIDAtuneState pid_atune_state;
extern PIDAtuneResult pid_atune_result;

// ==========================================
// PUBLIC FUNCTIONS
// ==========================================

/**
 * @brief Start the PID autotune sequence
 * 
 * Resets all state, locks steering to straight, and begins
 * the relay feedback tuning process.
 */
void pid_autotune_start();

/**
 * @brief Main PID autotune update function. Call every loop.
 * 
 * Drives the state machine: accelerates to target, applies relay
 * control, measures limit cycles, and computes PID gains.
 */
void pid_autotune_update();

/**
 * @brief Stop PID autotune immediately. Resets state to AT_IDLE.
 * Safe to call even if tuning is not running.
 */
void pid_autotune_stop();

/**
 * @brief Check if PID autotune is currently running
 * @return true if state machine is active
 */
bool pid_autotune_is_active();

/**
 * @brief Get the current autotune result
 * @return Reference to the result struct
 */
const PIDAtuneResult& pid_autotune_get_result();

/**
 * @brief Apply the recommended PID gains to the motor controller
 * Sets Kp, Ki, Kd globals in motor_control to the tuned values.
 */
void pid_autotune_apply_gains();