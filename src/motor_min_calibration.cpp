/**
 * @file motor_min_calibration.cpp
 * @brief Automatic motor minimum duty cycle calibration implementation
 * 
 * Calibration state machine:
 *   MC_IDLE → MC_P1_RAMPING → MC_SETTLING → MC_P2_DRIVING → MC_SETTLING → MC_P3_DRIVING → MC_SETTLING → MC_DONE
 * 
 * Phase 1 (stall locator): Fast ramp from base DC until the encoder first
 *   twitches (1mm). This only finds the stall threshold.
 * 
 * Phase 2 (coarse drive-find): Starts at the P1 threshold. Drives at a
 *   constant DC for a 1-second window and checks if the robot actually
 *   travels >= MC_DRIVE_MIN_MM. If not, increases DC by 5 and tries again.
 *   This quickly finds a DC that truly drives the car.
 * 
 * Phase 3 (fine drive-verify): Starts 5 DC below the P2 result. Drives at
 *   constant DC for a 1.5-second window with the same pass criterion, but
 *   increases DC by 1 per failed window. The result is the exact drive-
 *   verified DC and is the final recommended MOTOR_MIN_DC.
 * 
 * Safety:
 *   - Per-phase timeouts limit stall time
 *   - Per-phase and total distance limits prevent runaway
 *   - Direct pin writes bypass the set_dc() dead-zone check
 *   - dc_state is kept DC_DISABLED so the PID/drive loop cannot interfere
 */

#include "motor_min_calibration.h"
#include "config.h"
#include "motor_control.h"
#include "logger.h"
#define Serial robot_logger

// ==========================================
// EXTERNAL DEPENDENCIES
// ==========================================

extern float current_distance;
extern DCState dc_state;

// ==========================================
// MOTOR MIN DC CALIBRATION STATE
// ==========================================

MotorMinCalState motor_min_cal_state = MC_IDLE;

// ==========================================
// INTERNAL STATE
// ==========================================

static int mc_phase = 0;                    // 0=P1, 1=P2, 2=P3
static int mc_current_dc = 0;               // Current DC being applied
static int mc_p1_result = 0;                // Phase 1 stall threshold
static int mc_p2_result = 0;                // Phase 2 drive-find result
static int mc_p3_result = 0;                // Phase 3 drive-verify result
static unsigned long mc_phase_start_ms = 0; // Time when current phase started
static unsigned long mc_last_step_ms = 0;   // Time of last ramp step
static unsigned long mc_settle_start_ms = 0;// Time when settling started
static unsigned long mc_last_debug_ms = 0;  // Time of last debug print
static unsigned long mc_window_start_ms = 0;// Time when the current drive window started
static float mc_phase_start_distance = 0;   // Distance at phase start
static float mc_window_start_distance = 0;  // Distance at window start
static float mc_cal_start_distance = 0;     // Distance at calibration start (for total limit)
static float mc_total_distance = 0;         // Total distance traveled (for reporting)

// ==========================================
// INTERNAL HELPERS
// ==========================================

/**
 * @brief Apply a DC value directly to the motor pins.
 * 
 * Bypasses set_dc() to avoid the MOTOR_MIN_DC dead-zone check.
 * The motor direction is always forward (IN1=LOW, IN2=HIGH).
 */
static void apply_dc(int dc)
{
    if (dc <= 0) {
        digitalWrite(MOTOR_IN1_PIN, LOW);
        digitalWrite(MOTOR_IN2_PIN, LOW);
        analogWrite(MOTOR_PWM_PIN, 0);
    } else {
        analogWrite(MOTOR_PWM_PIN, dc);
        digitalWrite(MOTOR_IN1_PIN, LOW);
        digitalWrite(MOTOR_IN2_PIN, HIGH);
    }
}

/**
 * @brief Check the 2-metre total distance safety limit.
 * @return true if the limit is exceeded (abort), false if safe to continue
 */
static bool total_distance_exceeded()
{
    return fabsf(current_distance - mc_cal_start_distance) > MC_MAX_TOTAL_DISTANCE_MM;
}

/**
 * @brief Start a drive-verification window at the current DC.
 * 
 * Applies the DC and tracks the distance from which the window's
 * drive success will be measured.
 */
static void start_drive_window()
{
    mc_window_start_ms = millis();
    mc_window_start_distance = current_distance;
    apply_dc(mc_current_dc);

    Serial.print("  Window: DC=");
    Serial.print(mc_current_dc);
    Serial.print(" for ");
    Serial.print(mc_phase == 1 ? MC_P2_DRIVE_WINDOW_MS : MC_P3_DRIVE_WINDOW_MS);
    Serial.println(" ms");
}

/**
 * @brief Check the current drive window and evaluate pass/fail.
 * 
 * @param window_ms Duration of the drive window in ms
 * @param now Current time
 * @return true if the window is complete (pass or fail), false if still running
 */
static bool drive_window_complete(unsigned long window_ms, unsigned long now)
{
    if (now - mc_window_start_ms < window_ms) {
        return false;
    }

    const float window_distance = fabsf(current_distance - mc_window_start_distance);

    if (window_distance >= MC_DRIVE_MIN_MM) {
        Serial.print("  PASS: traveled ");
        Serial.print(window_distance, 1);
        Serial.print(" mm in ");
        Serial.print((now - mc_window_start_ms) / 1000.0f, 1);
        Serial.println(" s");
    } else {
        Serial.print("  FAIL: only ");
        Serial.print(window_distance, 1);
        Serial.print(" mm in ");
        Serial.print((now - mc_window_start_ms) / 1000.0f, 1);
        Serial.print(" s (need >= ");
        Serial.print(MC_DRIVE_MIN_MM, 0);
        Serial.println(" mm)");
    }
    return true;
}

/**
 * @brief Start the current phase.
 */
static void start_phase()
{
    switch (mc_phase) {
    case 0: // Phase 1: stall locator — ramp from base DC
        mc_current_dc = constrain(MC_P1_BASE_DC, 1, MC_MAX_DC);
        mc_phase_start_ms = millis();
        mc_last_step_ms = mc_phase_start_ms;
        mc_last_debug_ms = mc_phase_start_ms;
        mc_phase_start_distance = current_distance;
        apply_dc(mc_current_dc);

        Serial.print("Phase 1 (stall locator): starting at DC=");
        Serial.print(mc_current_dc);
        Serial.print(", step=");
        Serial.print(MC_P1_STEP_DC);
        Serial.print(" per ");
        Serial.print(MC_P1_STEP_MS);
        Serial.println(" ms");
        break;

    case 1: // Phase 2: coarse drive-find — start at P1 threshold
        mc_current_dc = constrain(mc_p1_result + MC_P2_OFFSET_DC, 1, MC_MAX_DC);
        mc_phase_start_ms = millis();
        mc_last_debug_ms = mc_phase_start_ms;
        mc_phase_start_distance = current_distance;
        Serial.print("Phase 2 (coarse drive-find): starting at DC=");
        Serial.print(mc_current_dc);
        Serial.print(", step=");
        Serial.print(MC_P2_STEP_DC);
        Serial.print(" per window (");
        Serial.print(MC_P2_DRIVE_WINDOW_MS);
        Serial.println(" ms)");
        start_drive_window();
        break;

    case 2: // Phase 3: fine drive-verify — start below P2 result
        mc_current_dc = constrain(mc_p2_result - MC_P3_OFFSET_DC, 1, MC_MAX_DC);
        mc_phase_start_ms = millis();
        mc_last_debug_ms = mc_phase_start_ms;
        mc_phase_start_distance = current_distance;
        Serial.print("Phase 3 (fine drive-verify): starting at DC=");
        Serial.print(mc_current_dc);
        Serial.print(", step=");
        Serial.print(MC_P3_STEP_DC);
        Serial.print(" per window (");
        Serial.print(MC_P3_DRIVE_WINDOW_MS);
        Serial.println(" ms)");
        start_drive_window();
        break;
    }
}

/**
 * @brief Record the result of the current phase.
 */
static void record_phase_result(int threshold_dc)
{
    switch (mc_phase) {
    case 0:
        mc_p1_result = threshold_dc;
        Serial.print("Phase 1 (stall locator) threshold: DC=");
        Serial.println(mc_p1_result);
        break;
    case 1:
        mc_p2_result = threshold_dc;
        Serial.print("Phase 2 (coarse drive-find) result: DC=");
        Serial.println(mc_p2_result);
        break;
    case 2:
        mc_p3_result = threshold_dc;
        Serial.print("Phase 3 (fine drive-verify) result: DC=");
        Serial.println(mc_p3_result);
        break;
    }
}

/**
 * @brief Stop the motor and transition to settling.
 */
static void stop_and_settle()
{
    apply_dc(0);
    mc_settle_start_ms = millis();
    motor_min_cal_state = MC_SETTLING;
}

/**
 * @brief Abort the calibration with a reason.
 */
static void abort_cal(const char* reason)
{
    Serial.print("Motor min DC calibration aborted: ");
    Serial.println(reason);
    apply_dc(0);
    motor_min_cal_state = MC_DONE;
}

/**
 * @brief Print the final calibration result.
 */
static void print_result()
{
    Serial.println("\n=== MOTOR MIN DC CALIBRATION RESULT ===");
    Serial.print("Phase 1 (stall locator): ");
    if (mc_p1_result > 0) {
        Serial.println(mc_p1_result);
    } else {
        Serial.println("FAILED");
    }
    Serial.print("Phase 2 (coarse drive-find): ");
    if (mc_p2_result > 0) {
        Serial.println(mc_p2_result);
    } else {
        Serial.println("FAILED");
    }
    Serial.print("Phase 3 (fine drive-verify): ");
    if (mc_p3_result > 0) {
        Serial.println(mc_p3_result);
    } else {
        Serial.println("FAILED");
    }

    // The final result is the Phase 3 value (the fine drive-verified DC)
    if (mc_p3_result > 0) {
        Serial.print("Recommended MOTOR_MIN_DC: ");
        Serial.println(mc_p3_result);
        Serial.print("Update config.h: constexpr auto MOTOR_MIN_DC = ");
        Serial.print(mc_p3_result);
        Serial.println(";");
    } else if (mc_p2_result > 0) {
        Serial.print("Fallback: using Phase 2 result. Recommended MOTOR_MIN_DC: ");
        Serial.println(mc_p2_result);
        Serial.print("Update config.h: constexpr auto MOTOR_MIN_DC = ");
        Serial.print(mc_p2_result);
        Serial.println(";");
    } else if (mc_p1_result > 0) {
        Serial.print("Fallback: using Phase 1 result. Recommended MOTOR_MIN_DC: ");
        Serial.println(mc_p1_result);
        Serial.print("Update config.h: constexpr auto MOTOR_MIN_DC = ");
        Serial.print(mc_p1_result);
        Serial.println(";");
    } else {
        Serial.println("No valid measurements. Check motor/battery connections.");
    }

    Serial.print("Total distance traveled: ");
    Serial.print(mc_total_distance, 1);
    Serial.println(" mm");
    Serial.println("======================================\n");
}

// ==========================================
// PUBLIC FUNCTIONS
// ==========================================

void motor_min_cal_start()
{
    if (motor_min_cal_is_active())
        motor_min_cal_stop();

    // Reset all state
    mc_phase = 0;
    mc_current_dc = 0;
    mc_p1_result = 0;
    mc_p2_result = 0;
    mc_p3_result = 0;
    mc_total_distance = 0;
    mc_cal_start_distance = current_distance;

    // Keep the motor in DISABLED state so drive_loop()/pid_speed() won't interfere
    dc_state = DC_DISABLED;
    servo_disabled = false;
    set_steering(0);

    Serial.println("\n\n========================================");
    Serial.println("MOTOR MIN DC CALIBRATION");
    Serial.println("========================================");
    Serial.println("Three-phase drive-verified search:");
    Serial.println("  P1: stall locator (find twitch threshold)");
    Serial.println("  P2: coarse drive-find (step 5 DC until robot drives)");
    Serial.println("  P3: fine drive-verify (step 1 DC for exact value)");
    Serial.print("Base DC: ");
    Serial.println(MC_P1_BASE_DC);
    Serial.print("Max DC: ");
    Serial.println(MC_MAX_DC);
    Serial.print("Drive criteria: >= ");
    Serial.print(MC_DRIVE_MIN_MM, 0);
    Serial.println(" mm per window");
    Serial.print("Total distance limit: ");
    Serial.print(MC_MAX_TOTAL_DISTANCE_MM);
    Serial.println(" mm");
    Serial.println("WARNING: The robot WILL drive forward during Phases 2 and 3.");
    Serial.println("Ensure there is clear space ahead.");
    Serial.println();

    motor_min_cal_state = MC_P1_RAMPING;
    start_phase();
}

void motor_min_cal_update()
{
    if (motor_min_cal_state == MC_IDLE || motor_min_cal_state == MC_DONE) {
        return;
    }

    const unsigned long now = millis();

    // ==========================================
    // PHASE 1: Stall locator (ramp up until twitch)
    // ==========================================
    if (motor_min_cal_state == MC_P1_RAMPING) {
        const float phase_distance = fabsf(current_distance - mc_phase_start_distance);

        // Per-phase distance limit
        if (phase_distance > MC_MAX_PHASE_DISTANCE_MM) {
            Serial.println("Phase 1 aborted: distance limit reached.");
            record_phase_result(mc_current_dc);
            mc_total_distance += phase_distance;
            stop_and_settle();
            return;
        }

        // Total distance limit
        if (total_distance_exceeded()) {
            abort_cal("total distance limit reached");
            return;
        }

        // Movement detection (stall threshold)
        if (phase_distance >= MC_MOVEMENT_THRESHOLD_MM) {
            record_phase_result(mc_current_dc);
            mc_total_distance += phase_distance;
            Serial.print("Stall threshold detected at DC=");
            Serial.println(mc_current_dc);
            stop_and_settle();
            return;
        }

        // Phase timeout
        if (now - mc_phase_start_ms >= MC_P1_TIMEOUT_MS) {
            Serial.println("Phase 1 timed out: no movement detected.");
            mc_total_distance += phase_distance;
            stop_and_settle();
            return;
        }

        // Ramp the DC
        if (now - mc_last_step_ms >= MC_P1_STEP_MS) {
            mc_last_step_ms = now;
            mc_current_dc += MC_P1_STEP_DC;
            mc_current_dc = constrain(mc_current_dc, 1, MC_MAX_DC);
            apply_dc(mc_current_dc);
        }

        // Periodic debug output
        if (now - mc_last_debug_ms >= 500) {
            mc_last_debug_ms = now;
            Serial.print("  DC=");
            Serial.print(mc_current_dc);
            Serial.print(", dist=");
            Serial.println(phase_distance, 2);
        }
    }

    // ==========================================
    // PHASE 2 & 3: Drive verification (windowed)
    // ==========================================
    else if (motor_min_cal_state == MC_P2_DRIVING ||
             motor_min_cal_state == MC_P3_DRIVING) {

        const unsigned long window_ms = (mc_phase == 1)
            ? MC_P2_DRIVE_WINDOW_MS
            : MC_P3_DRIVE_WINDOW_MS;
        const unsigned long timeout_ms = (mc_phase == 1)
            ? MC_P2_TIMEOUT_MS
            : MC_P3_TIMEOUT_MS;
        const int step_dc = (mc_phase == 1)
            ? MC_P2_STEP_DC
            : MC_P3_STEP_DC;

        // Total distance limit
        if (total_distance_exceeded()) {
            abort_cal("total distance limit reached");
            return;
        }

        // Phase timeout
        if (now - mc_phase_start_ms >= timeout_ms) {
            Serial.print("Phase ");
            Serial.print(mc_phase + 1);
            Serial.println(" timed out: no drive-verified DC found.");
            stop_and_settle();
            return;
        }

        // Check the current drive window
        if (drive_window_complete(window_ms, now)) {
            const float window_distance = fabsf(current_distance - mc_window_start_distance);
            mc_total_distance += window_distance;

            if (window_distance >= MC_DRIVE_MIN_MM) {
                // Record this DC as the result
                record_phase_result(mc_current_dc);
                stop_and_settle();
                return;
            } else {
                // Too slow — increase DC and start a new window
                mc_current_dc += step_dc;
                mc_current_dc = constrain(mc_current_dc, 1, MC_MAX_DC);

                if (mc_current_dc >= MC_MAX_DC) {
                    Serial.println("Reached max DC without drive success.");
                    record_phase_result(mc_current_dc);
                    stop_and_settle();
                    return;
                }

                start_drive_window();
            }
        }
    }

    // ==========================================
    // SETTLING: transition to the next phase
    // ==========================================
    else if (motor_min_cal_state == MC_SETTLING) {
        if (now - mc_settle_start_ms >= MC_SETTLE_MS) {
            if (mc_phase == 0) {
                if (mc_p1_result == 0) {
                    abort_cal("Phase 1 found no movement. Check motor/battery connections.");
                    return;
                }
                mc_phase = 1;
                motor_min_cal_state = MC_P2_DRIVING;
                start_phase();
            }
            else if (mc_phase == 1) {
                if (mc_p2_result == 0) {
                    // No drive-verified DC found in Phase 2 — fall back to P1
                    Serial.println("Phase 2 failed. Using Phase 1 result as fallback.");
                    mc_p2_result = mc_p1_result;
                }
                mc_phase = 2;
                motor_min_cal_state = MC_P3_DRIVING;
                start_phase();
            }
            else if (mc_phase == 2) {
                motor_min_cal_state = MC_DONE;
                apply_dc(0);
                print_result();
            }
        }
    }
}

void motor_min_cal_stop()
{
    motor_min_cal_state = MC_IDLE;
    apply_dc(0);
    Serial.println("Motor min DC calibration stopped.");
}

bool motor_min_cal_is_active()
{
    return motor_min_cal_state != MC_IDLE &&
           motor_min_cal_state != MC_DONE;
}