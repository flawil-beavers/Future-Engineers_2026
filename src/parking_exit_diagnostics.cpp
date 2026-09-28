#include "parking_exit_diagnostics.h"

#if PARKING_EXIT_DIAGNOSTICS_ENABLED

#include "ackermann_kinematics.h"
#include "config.h"
#include "logger.h"
#include "motor_control.h"
#include "sensors.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define Serial robot_logger

namespace {
constexpr uint32_t SCHEMA_VERSION = 2;
constexpr uint32_t SAMPLE_PERIOD_MS = 200;
constexpr uint16_t SAMPLE_LIMIT = 150;
constexpr size_t DIAGNOSTIC_BYTE_LIMIT = 64 * 1024;
constexpr size_t SAMPLE_BYTE_LIMIT = 57000;
constexpr size_t EVENT_BYTE_LIMIT = 8 * 1024;
constexpr size_t LINE_CAPACITY = 512;
static_assert(SAMPLE_BYTE_LIMIT + EVENT_BYTE_LIMIT <= DIAGNOSTIC_BYTE_LIMIT,
              "Diagnostic sub-budgets must fit the hard byte limit");

bool active = false;
bool truncated = false;
uint32_t lastSampleMs = 0;
uint16_t sampleCount = 0;
size_t emittedBytes = 0;
size_t sampleBytes = 0;
size_t eventBytes = 0;
char lastState[28] = "";
int8_t lastDirection = 0;
uint32_t lastTofSequence[TOF_COUNT] = {};
float lastEncoderMm = 0.0f;
PositionEstimate nominal = {};

int8_t commandDirection()
{
    return target_speed > 0 ? 1 : (target_speed < 0 ? -1 : 0);
}

float wrap180(float angle)
{
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

void integrateNominal()
{
    const float encoderMm = get_distance();
    const float delta = encoderMm - lastEncoderMm;
    lastEncoderMm = encoderMm;
    if (fabsf(delta) < 0.0001f)
        return;

    float headingRad = nominal.heading_deg * PI / 180.0f;
    const float radius = Ackermann::getTurnRadius(static_cast<float>(set_degree));
    if (fabsf(radius) >= Ackermann::STRAIGHT_RADIUS_MM * 0.5f)
    {
        nominal.x_mm += delta * cosf(headingRad);
        nominal.y_mm += delta * sinf(headingRad);
    }
    else
    {
        const float deltaHeading = -delta / radius;
        const float nextHeading = headingRad + deltaHeading;
        nominal.x_mm += (sinf(nextHeading) - sinf(headingRad)) * (-radius);
        nominal.y_mm += (cosf(nextHeading) - cosf(headingRad)) * radius;
        headingRad = nextHeading;
        nominal.heading_deg = wrap180(headingRad * 180.0f / PI);
    }
}

bool emitLine(const char *line, bool priority, bool terminal = false)
{
    // println emits CRLF; include both bytes in the hard budget. Reserve room
    // for one truncation marker and the final completion/abort event.
    const size_t length = strlen(line) + 2;
    const size_t eventLimit = terminal ? EVENT_BYTE_LIMIT
                                     : EVENT_BYTE_LIMIT - 2 * (LINE_CAPACITY + 1);
    if (emittedBytes + length > DIAGNOSTIC_BYTE_LIMIT ||
        (priority ? eventBytes + length > eventLimit
                  : sampleBytes + length > SAMPLE_BYTE_LIMIT))
        return false;
    Serial.println(line);
    emittedBytes += length;
    if (priority) eventBytes += length;
    else sampleBytes += length;
    return true;
}

void markTruncated(const char *reason)
{
    if (truncated)
        return;
    truncated = true;
    char line[LINE_CAPACITY];
    snprintf(line, sizeof(line),
             "[PARK_DIAG] v=2 type=truncated t=%lu reason=%s",
             static_cast<unsigned long>(millis()), reason);
    emitLine(line, true, true);
}

void emitEvent(const char *type, const char *detail, bool terminal = false)
{
    char line[LINE_CAPACITY];
    snprintf(line, sizeof(line),
             "[PARK_DIAG] v=%lu type=%s t=%lu detail=%s enc=%ld emm=%.2f gyro=%.2f steer=%d cmd=%d",
             static_cast<unsigned long>(SCHEMA_VERSION), type,
             static_cast<unsigned long>(millis()), detail,
             encoder_pos, get_distance(), get_angle(), set_degree, target_speed);
    if (!emitLine(line, true, terminal))
        markTruncated("event_budget");
}

void beginDiagnostics(int8_t turnSign)
{
    active = true;
    truncated = false;
    lastSampleMs = 0;
    sampleCount = 0;
    emittedBytes = 0;
    sampleBytes = 0;
    eventBytes = 0;
    lastState[0] = '\0';
    lastDirection = commandDirection();
    memset(lastTofSequence, 0, sizeof(lastTofSequence));
    lastEncoderMm = get_distance();
    nominal = get_position_struct();

    char buildIdentity[24];
    snprintf(buildIdentity, sizeof(buildIdentity), "%s_%s", __DATE__, __TIME__);
    for (char *character = buildIdentity; *character != '\0'; ++character)
        if (*character == ' ' || *character == ':')
            *character = '_';
    char line[LINE_CAPACITY];
    snprintf(line, sizeof(line),
             "[PARK_DIAG_CONFIG] v=%lu build=%s turn=%d center=%d mm_per_count=%.7f period_ms=%lu sample_limit=%u byte_limit=%lu buffer=%lu sample_bytes=%lu event_bytes=%lu line_capacity=%lu segments=%d exit_speed=%d brake_ms=%lu rear_target=%.1f localize_max=%.1f",
             static_cast<unsigned long>(SCHEMA_VERSION), buildIdentity,
             turnSign, SERVO_CENTER, COUNTER_TO_MM,
             static_cast<unsigned long>(SAMPLE_PERIOD_MS), SAMPLE_LIMIT,
             static_cast<unsigned long>(DIAGNOSTIC_BYTE_LIMIT),
             static_cast<unsigned long>(LOG_BUFFER_SIZE),
             static_cast<unsigned long>(SAMPLE_BYTE_LIMIT),
             static_cast<unsigned long>(EVENT_BYTE_LIMIT),
             static_cast<unsigned long>(LINE_CAPACITY),
             OBSTACLE_PARKING_EXIT_SEGMENT_COUNT, OBSTACLE_PARKING_EXIT_SPEED,
             static_cast<unsigned long>(OBSTACLE_PARKING_EXIT_HOLD_BRAKE_MS),
             OBSTACLE_PARKING_REAR_TOF_TARGET_RANGE_MM,
             OBSTACLE_PARKING_EXIT_EDGE_LOCALIZATION_MAX_MM);
    emitLine(line, true);
}

bool appendTof(char *line, size_t capacity, size_t &used, TofSensor sensor,
               uint32_t now)
{
    TofDiagnosticSnapshot snapshot;
    if (!get_tof_diagnostic_snapshot(sensor, snapshot))
    {
        const int added = snprintf(line + used, capacity - used, " s%d=none", sensor);
        if (added < 0 || static_cast<size_t>(added) >= capacity - used)
            return false;
        used += static_cast<size_t>(added);
        return true;
    }
    if (lastTofSequence[sensor] == snapshot.sequence)
    {
        const int added = snprintf(line + used, capacity - used, " s%d=same", sensor);
        if (added < 0 || static_cast<size_t>(added) >= capacity - used)
            return false;
        used += static_cast<size_t>(added);
        return true;
    }
    lastTofSequence[sensor] = snapshot.sequence;
    const int index = snapshot.selected_object_index;
    const bool selected = index >= 0 && index < snapshot.stored_object_count;
    const bool valid = sensor == TOF_REAR
        ? snapshot.selected_raw_distance_mm >= 0.0f
        : selected && snapshot.objects[index].hardware_valid;
    const bool accepted = sensor == TOF_REAR
        ? snapshot.filtered_distance_mm >= 0.0f
        : selected && snapshot.objects[index].filter_accepted;
    const uint32_t age = now - snapshot.sampled_ms;
    const int added = snprintf(line + used, capacity - used,
                     " s%d=%lu,%lu,%.1f,%.1f,%.3f,%.1f,%d,%d",
                     sensor, static_cast<unsigned long>(snapshot.sequence),
                     static_cast<unsigned long>(age),
                     snapshot.selected_raw_distance_mm,
                     snapshot.filtered_distance_mm,
                     snapshot.selected_signal_mcps, snapshot.selected_sigma_mm,
                     valid ? 1 : 0, accepted ? 1 : 0);
    if (added < 0 || static_cast<size_t>(added) >= capacity - used)
        return false;
    used += static_cast<size_t>(added);
    return true;
}
} // namespace

void parking_exit_diagnostics_reset()
{
    active = false;
    truncated = false;
    lastState[0] = '\0';
    emittedBytes = 0;
    sampleBytes = 0;
    eventBytes = 0;
    sampleCount = 0;
}

void parking_exit_diagnostics_update(const char *state, uint8_t segment,
                                     int8_t turnSign, float segmentTargetMm,
                                     const char *expectedReference)
{
    if (!active)
        beginDiagnostics(turnSign);

    integrateNominal();
    const uint32_t now = millis();
    const bool stateChanged = strncmp(lastState, state, sizeof(lastState)) != 0;
    const int8_t direction = commandDirection();
    if (stateChanged)
    {
        strncpy(lastState, state, sizeof(lastState) - 1);
        lastState[sizeof(lastState) - 1] = '\0';
        emitEvent("state", state);
    }
    if (direction != lastDirection)
    {
        char detail[32];
        snprintf(detail, sizeof(detail), "%d_to_%d", lastDirection, direction);
        emitEvent("direction", detail);
        lastDirection = direction;
    }
    if (truncated || (!stateChanged && now - lastSampleMs < SAMPLE_PERIOD_MS))
        return;
    if (sampleCount >= SAMPLE_LIMIT)
    {
        markTruncated("sample_limit");
        return;
    }

    lastSampleMs = now;
    const PositionEstimate pose = get_position_struct();
    char line[LINE_CAPACITY];
    const int formatted = snprintf(
        line, sizeof(line),
        "[PARK_DIAG] v=2 type=sample t=%lu state=%s seg=%u turn=%d target=%.1f enc=%ld emm=%.2f cmd=%d speed=%.2f steer=%d dc=%d gyro=%.2f pose=%.1f,%.1f,%.2f nominal=%.1f,%.1f,%.2f ref=%s",
        static_cast<unsigned long>(now), state, segment, turnSign,
        segmentTargetMm, encoder_pos, get_distance(), target_speed,
        measured_speed, set_degree, static_cast<int>(dc_state), get_angle(),
        pose.x_mm, pose.y_mm, pose.heading_deg,
        nominal.x_mm, nominal.y_mm, nominal.heading_deg, expectedReference);
    if (formatted < 0 || static_cast<size_t>(formatted) >= sizeof(line))
    {
        markTruncated("line_capacity");
        return;
    }
    size_t used = static_cast<size_t>(formatted);
    for (uint8_t sensor = 0; sensor < TOF_COUNT; ++sensor)
        if (!appendTof(line, sizeof(line), used, static_cast<TofSensor>(sensor), now))
        {
            markTruncated("line_capacity");
            return;
        }
    if (emitLine(line, false))
        ++sampleCount;
    else
        markTruncated("sample_budget");
}

void parking_exit_diagnostics_rebase(const char *reason)
{
    nominal = get_position_struct();
    lastEncoderMm = get_distance();
    if (active)
        emitEvent("rebase", reason);
}

void parking_exit_diagnostics_correction(const char *source,
                                         const PositionEstimate &before,
                                         const PositionEstimate &after)
{
    if (!active)
        return;
    char line[LINE_CAPACITY];
    snprintf(line, sizeof(line),
             "[PARK_DIAG] v=2 type=correction t=%lu source=%s before=%.1f,%.1f,%.2f after=%.1f,%.1f,%.2f delta=%.1f,%.1f,%.2f",
             static_cast<unsigned long>(millis()), source,
             before.x_mm, before.y_mm, before.heading_deg,
             after.x_mm, after.y_mm, after.heading_deg,
             after.x_mm - before.x_mm, after.y_mm - before.y_mm,
             wrap180(after.heading_deg - before.heading_deg));
    if (!emitLine(line, true))
        markTruncated("event_budget");
}

void parking_exit_diagnostics_finish(const char *result)
{
    if (!active)
        return;
    char detail[80];
    snprintf(detail, sizeof(detail), "%s_bytes_%lu_samples_%u%s", result,
             static_cast<unsigned long>(emittedBytes), sampleCount,
             truncated ? "_truncated" : "");
    emitEvent("finish", detail, true);
    active = false;
}

#endif
