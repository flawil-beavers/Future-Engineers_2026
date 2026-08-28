#include "tof_pose_diagnostic.h"

#include <Arduino.h>
#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "logger.h"
#include "mode_manager.h"
#include "motor_control.h"
#include "obstacle_path.h"
#include "position_estimator.h"
#include "sensors.h"

#define Serial robot_logger

namespace
{
constexpr uint8_t CAPTURE_FRAMES = 12;
constexpr float PASS_FINAL_ERROR_MM = 20.0f;

struct DiagnosticState
{
    bool active = false;
    bool corner = false;
    int8_t turnSign = 1;
    uint8_t index = 0;
    uint8_t frames = 0;
    uint8_t correctionFrames = 0;
    uint8_t expectedGateFrames = 0;
    uint8_t duplicateBlockedFrames = 0;
    float actualLateralMm = 0.0f;
    float initialErrorMm = 0.0f;
    float pathDistanceMm = 0.0f;
    float centerXmm = 0.0f;
    float centerYmm = 0.0f;
    float headingDeg = 0.0f;
    uint32_t lastSequence[TOF_SIDE_COUNT] = {};
};

DiagnosticState state;

void forceMotorLock()
{
    if (dc_state != DC_DISABLED || target_speed != 0 || dc_out != 0)
        stop(false);
    set_steering(0);
    servo_disabled = true;
}

float estimatedLateral(const PositionEstimate &pose)
{
    const float headingRad = state.headingDeg * PI / 180.0f;
    const float normalX = -sinf(headingRad);
    const float normalY = cosf(headingRad);
    return (pose.x_mm - state.centerXmm) * normalX +
           (pose.y_mm - state.centerYmm) * normalY;
}

void readSequences()
{
    TofDiagnosticSnapshot snapshot;
    for (int i = 0; i < TOF_SIDE_COUNT; ++i)
    {
        state.lastSequence[i] =
            get_tof_diagnostic_snapshot(static_cast<TofSensor>(i), snapshot)
                ? snapshot.sequence
                : 0;
    }
}

void printHelp()
{
    Serial.println("\n===== STATIONARY TOF POSE CORRECTION =====");
    Serial.println("Keep the physical enable switch LOW; motor and servo stay disabled.");
    Serial.println("Positive lateral offset means left of the driving centreline.");
    Serial.println("tofpose arm <L|R> straight <section 0-3> <actual_lateral_mm>");
    Serial.println("tofpose arm <L|R> corner <corner 0-3> <actual_lateral_mm>");
    Serial.println("tofpose show : print current diagnostic state");
    Serial.println("tofpose stop : stop and clear the diagnostic");
    Serial.println("Each arm command applies exactly 12 fresh paired ToF frames.");
    Serial.println("================================================\n");
}

void stopDiagnostic(const char *reason)
{
    forceMotorLock();
    obstacle_path_reset();
    state.active = false;
    Serial.print("[TOFPOSE] stopped reason=");
    Serial.println(reason);
}

void printState()
{
    const PositionEstimate pose = get_position_struct();
    Serial.print("[TOFPOSE] active="); Serial.print(state.active ? "yes" : "no");
    Serial.print(" direction="); Serial.print(state.turnSign > 0 ? "L" : "R");
    Serial.print(" location="); Serial.print(state.corner ? "corner" : "straight");
    Serial.print(" index="); Serial.print(state.index);
    Serial.print(" frames="); Serial.print(state.frames);
    Serial.print(" actual_lateral="); Serial.print(state.actualLateralMm, 1);
    Serial.print(" estimated_lateral="); Serial.print(estimatedLateral(pose), 1);
    Serial.print(" pose="); Serial.print(pose.x_mm, 1);
    Serial.print(","); Serial.print(pose.y_mm, 1);
    Serial.print(","); Serial.println(pose.heading_deg, 1);
}

bool arm(const char *message)
{
    char directionText[4] = {};
    char location[10] = {};
    int index = -1;
    int actualLateral = 0;
    if (sscanf(message, "tofpose arm %3s %9s %d %d",
               directionText, location, &index, &actualLateral) != 4 ||
        (toupper(directionText[0]) != 'L' &&
         toupper(directionText[0]) != 'R') ||
        directionText[1] != '\0' || index < 0 || index > 3 ||
        abs(actualLateral) > 200 ||
        (strcmp(location, "straight") != 0 &&
         strcmp(location, "corner") != 0))
    {
        Serial.println("Usage: tofpose arm <L|R> <straight|corner> <index 0-3> <actual_lateral_mm>");
        return true;
    }
    if (system_enabled)
    {
        Serial.println("[TOFPOSE] Rejected: lower the physical enable switch first.");
        return true;
    }

    mode_stop_all();
    state = DiagnosticState{};
    state.turnSign = toupper(directionText[0]) == 'L' ? 1 : -1;
    state.corner = strcmp(location, "corner") == 0;
    state.index = static_cast<uint8_t>(index);
    state.actualLateralMm = static_cast<float>(actualLateral);
    if (!obstacle_path_prepare_tof_diagnostic(
            state.turnSign,
            state.corner,
            state.index,
            0.0f,
            state.pathDistanceMm,
            state.centerXmm,
            state.centerYmm,
            state.headingDeg))
    {
        Serial.println("[TOFPOSE] Failed to prepare field geometry.");
        obstacle_path_reset();
        return true;
    }

    forceMotorLock();
    readSequences();
    state.initialErrorMm = fabsf(state.actualLateralMm);
    state.active = true;
    Serial.print("[TOFPOSE] armed direction=");
    Serial.print(state.turnSign > 0 ? "L" : "R");
    Serial.print(" location="); Serial.print(state.corner ? "corner" : "straight");
    Serial.print(" index="); Serial.print(state.index);
    Serial.print(" actual_lateral="); Serial.print(state.actualLateralMm, 1);
    Serial.print(" initial_estimate=0.0 heading="); Serial.print(state.headingDeg, 1);
    Serial.print(" motor_lock=");
    Serial.println(
        dc_state == DC_DISABLED && target_speed == 0 && servo_disabled
            ? "PASS"
            : "FAIL");
    return true;
}
} // namespace

bool tof_pose_diagnostic_handle_command(const char *message)
{
    if (strncmp(message, "tofpose", 7) != 0 ||
        (message[7] != '\0' && message[7] != ' '))
        return false;

    char action[12] = {};
    if (sscanf(message, "tofpose %11s", action) != 1 ||
        strcmp(action, "help") == 0)
    {
        printHelp();
        return true;
    }
    if (strcmp(action, "arm") == 0)
        return arm(message);
    if (strcmp(action, "show") == 0)
    {
        printState();
        return true;
    }
    if (strcmp(action, "stop") == 0)
    {
        if (state.active)
            stopDiagnostic("command");
        else
            Serial.println("[TOFPOSE] No diagnostic is active.");
        return true;
    }

    Serial.println("Unknown ToF pose command. Send 'tofpose help'.");
    return true;
}

void tof_pose_diagnostic_update()
{
    if (!state.active)
        return;

    forceMotorLock();
    if (system_enabled)
    {
        stopDiagnostic("enable_switch_high");
        return;
    }

    TofDiagnosticSnapshot snapshots[TOF_SIDE_COUNT];
    if (!get_tof_diagnostic_snapshot(TOF_LEFT, snapshots[TOF_LEFT]) ||
        !get_tof_diagnostic_snapshot(TOF_RIGHT, snapshots[TOF_RIGHT]) ||
        snapshots[TOF_LEFT].sequence == state.lastSequence[TOF_LEFT] ||
        snapshots[TOF_RIGHT].sequence == state.lastSequence[TOF_RIGHT])
        return;

    state.lastSequence[TOF_LEFT] = snapshots[TOF_LEFT].sequence;
    state.lastSequence[TOF_RIGHT] = snapshots[TOF_RIGHT].sequence;
    const PositionEstimate before = get_position_struct();
    const float beforeLateral = estimatedLateral(before);
    const ObstacleTofCorrectionResult result =
        obstacle_path_apply_tof_diagnostic(state.pathDistanceMm);
    const PositionEstimate after = get_position_struct();
    const float afterLateral = estimatedLateral(after);
    const ObstacleTofCorrectionResult duplicate =
        obstacle_path_apply_tof_diagnostic(state.pathDistanceMm);
    const bool duplicateBlocked =
        !duplicate.leftUsed && !duplicate.rightUsed &&
        fabsf(duplicate.correctionXmm) < 0.01f &&
        fabsf(duplicate.correctionYmm) < 0.01f;
    ++state.frames;
    if (result.leftUsed || result.rightUsed)
        ++state.correctionFrames;
    const bool expectedGate = state.turnSign > 0
        ? result.leftCornerGated && !result.leftUsed
        : result.rightCornerGated && !result.rightUsed;
    if (expectedGate)
        ++state.expectedGateFrames;
    if (duplicateBlocked)
        ++state.duplicateBlockedFrames;

    Serial.print("[TOFPOSE FRAME] n="); Serial.print(state.frames);
    Serial.print(" tof=L"); Serial.print(result.leftReadingMm, 1);
    Serial.print("/R"); Serial.print(result.rightReadingMm, 1);
    Serial.print(" used=L"); Serial.print(result.leftUsed ? 1 : 0);
    Serial.print("/R"); Serial.print(result.rightUsed ? 1 : 0);
    Serial.print(" gated=L"); Serial.print(result.leftCornerGated ? 1 : 0);
    Serial.print("/R"); Serial.print(result.rightCornerGated ? 1 : 0);
    Serial.print(" residual_gated=L");
    Serial.print(result.leftResidualGated ? 1 : 0);
    Serial.print("/R"); Serial.print(result.rightResidualGated ? 1 : 0);
    Serial.print(" residual=L"); Serial.print(result.leftResidualMm, 1);
    Serial.print("/R"); Serial.print(result.rightResidualMm, 1);
    Serial.print(" lateral="); Serial.print(beforeLateral, 1);
    Serial.print("->"); Serial.print(afterLateral, 1);
    Serial.print(" correction="); Serial.print(result.correctionXmm, 1);
    Serial.print(","); Serial.print(result.correctionYmm, 1);
    Serial.print(" duplicate_blocked=");
    Serial.println(duplicateBlocked ? 1 : 0);

    if (state.frames < CAPTURE_FRAMES)
        return;

    const float finalError = fabsf(state.actualLateralMm - afterLateral);
    const bool straightPass = !state.corner &&
        state.correctionFrames > 0 &&
        state.duplicateBlockedFrames == CAPTURE_FRAMES &&
        finalError < state.initialErrorMm &&
        finalError <= PASS_FINAL_ERROR_MM;
    const bool cornerPass = state.corner &&
        state.expectedGateFrames == CAPTURE_FRAMES &&
        state.duplicateBlockedFrames == CAPTURE_FRAMES;
    Serial.print("[TOFPOSE RESULT] ");
    Serial.print(state.corner ? (cornerPass ? "PASS" : "FAIL")
                              : (straightPass ? "PASS" : "FAIL"));
    Serial.print(" actual_lateral="); Serial.print(state.actualLateralMm, 1);
    Serial.print(" final_lateral="); Serial.print(afterLateral, 1);
    Serial.print(" final_error="); Serial.print(finalError, 1);
    Serial.print(" correction_frames="); Serial.print(state.correctionFrames);
    Serial.print(" expected_gate_frames="); Serial.print(state.expectedGateFrames);
    Serial.print(" duplicate_blocked_frames=");
    Serial.println(state.duplicateBlockedFrames);
    stopDiagnostic("capture_complete");
}

bool tof_pose_diagnostic_active()
{
    return state.active;
}
