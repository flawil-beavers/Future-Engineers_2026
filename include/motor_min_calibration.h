#pragma once

/**
 * @file motor_min_calibration.h
 * @brief Automatic motor minimum duty cycle calibration subsystem
 * 
 * Determines the minimum PWM duty cycle (MOTOR_MIN_DC) needed to actually
 * drive the robot (not just twitch the motor shaft). Uses a three-phase
 * drive-verified search:
 *   Phase 1: Stall locator — fast ramp to find where the encoder first twitches
 *   Phase 2: Coarse drive-find — steps by 5 DC until the robot really drives
 *   Phase 3: Fine drive-verify — steps by 1 DC below the Phase 2 result to
 *            find the exact drive-verified DC
 * 
 * The final result is the Phase 3 value (the fine drive-verified DC).
 * The calibration writes directly to the motor pins to bypass the
 * set_dc() dead-zone check, and keeps dc_state = DC_DISABLED so the
 * normal PID/drive loop cannot interfere.
 */

#include <Arduino.h>

// ==========================================
// MOTOR MIN DC CALIBRATION STATE MACHINE
// ==========================================

enum MotorMinCalState {
    MC_IDLE,           ///< Not running
    MC_P1_RAMPING,     ///< Phase 1: stall locator (ramp up)
    MC_SETTLING,       ///< Settling between phases
    MC_P2_DRIVING,     ///< Phase 2: coarse drive-find (step DC by 5 per window)
    MC_P3_DRIVING,     ///< Phase 3: fine drive-verify (step DC by 1 per window)
    MC_DONE            ///< Calibration complete
};

// ==========================================
// EXTERNAL STATE
// ==========================================

extern MotorMinCalState motor_min_cal_state;

// ==========================================
// PUBLIC FUNCTIONS
// ==========================================

/**
 * @brief Start the motor minimum DC calibration sequence.
 * Resets all state and begins the three-phase drive-verified search.
 */
void motor_min_cal_start();

/**
 * @brief Main motor min DC calibration update function. Call every loop.
 * 
 * Drives the state machine: Phase 1 finds the stall threshold, Phase 2
 * finds a DC that actually drives the robot, and Phase 3 fine-tunes the
 * drive-verified DC. Finally outputs the recommended MOTOR_MIN_DC.
 */
void motor_min_cal_update();

/**
 * @brief Stop motor min DC calibration immediately. Resets state to MC_IDLE.
 * Safe to call even if calibration is not running.
 */
void motor_min_cal_stop();

/**
 * @brief Check if motor min DC calibration is currently running
 * @return true if state machine is active
 */
bool motor_min_cal_is_active();