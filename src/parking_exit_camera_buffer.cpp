#include "parking_exit_observer.h"
#include "obstacle_blob_validation.h"
#include <math.h>

namespace {
float wrapExitAngle(float a) {
    while (a > 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}
bool validExitPose(const PositionEstimate &p) {
    return isfinite(p.x_mm) && isfinite(p.y_mm) && isfinite(p.heading_deg);
}
}

void ParkingExitCameraBuffer::reset() {
    first = count = rejected = replaced = 0;
    historyFirst = historyCount = 0;
    lastSequence = lastCapturedUs = 0;
    enabled = haveSequence = false;
    turn = 0;
}

void ParkingExitCameraBuffer::begin(int8_t turnSign) {
    reset();
    if (turnSign != -1 && turnSign != 1) return;
    enabled = true;
    turn = turnSign;
}

void ParkingExitCameraBuffer::samplePose(uint32_t nowUs, const PositionEstimate &pose, bool frameBoundary) {
    if (!enabled) return;
    if (!validExitPose(pose)) { reset(); return; }
    if (historyCount) {
        const TimedPose &last = history[(historyFirst + historyCount - 1) % HISTORY_CAPACITY];
        if (nowUs == last.timeUs || (!frameBoundary && nowUs - last.timeUs < 10000U)) return;
    }
    if (historyCount == HISTORY_CAPACITY) {
        historyFirst = (historyFirst + 1) % HISTORY_CAPACITY;
        --historyCount;
    }
    TimedPose &sample = history[(historyFirst + historyCount++) % HISTORY_CAPACITY];
    sample.pose = pose;
    sample.timeUs = nowUs;
}

bool ParkingExitCameraBuffer::poseAt(uint32_t timeUs, PositionEstimate &pose) const {
    for (uint8_t i = 0; i < historyCount; ++i) {
        const TimedPose &sample = history[(historyFirst + i) % HISTORY_CAPACITY];
        if (sample.timeUs == timeUs) {
            pose = sample.pose;
            pose.heading_deg = wrapExitAngle(pose.heading_deg);
            return true;
        }
    }
    for (uint8_t i = 1; i < historyCount; ++i) {
        const TimedPose &a = history[(historyFirst + i - 1) % HISTORY_CAPACITY];
        const TimedPose &b = history[(historyFirst + i) % HISTORY_CAPACITY];
        if (static_cast<int32_t>(timeUs - a.timeUs) < 0 ||
            static_cast<int32_t>(b.timeUs - timeUs) < 0) continue;
        const uint32_t dt = b.timeUs - a.timeUs;
        if (!dt || dt > 70000U) return false;
        const float f = static_cast<float>(timeUs - a.timeUs) / dt;
        pose = a.pose;
        pose.x_mm += f * (b.pose.x_mm - a.pose.x_mm);
        pose.y_mm += f * (b.pose.y_mm - a.pose.y_mm);
        pose.heading_deg = wrapExitAngle(a.pose.heading_deg +
            f * wrapExitAngle(b.pose.heading_deg - a.pose.heading_deg));
        return true;
    }
    return false;
}

bool ParkingExitCameraBuffer::capture(uint32_t nowUs, uint32_t completedUs,
        uint32_t intervalUs, uint32_t sequence, const VisionResult &vision) {
    if (!enabled) return false;
    if (haveSequence && sequence == lastSequence) return false;
    // A camera restart must not join evidence from two sequence epochs.
    if (haveSequence && static_cast<int32_t>(sequence - lastSequence) <= 0) {
        const int8_t savedTurn = turn;
        begin(savedTurn);
        return false;
    }
    lastSequence = sequence;
    haveSequence = true;
    PositionEstimate start{}, middle{}, end{};
    // Frame completion is timestamped by DMA. Use the middle of the actual
    // frame interval; this approximates exposure, not an exact shutter time.
    // Large/missing intervals and excessive motion fall back to the normal scan.
    if (intervalUs < 20000U || intervalUs > 120000U ||
        nowUs - completedUs > 100000U ||
        !poseAt(completedUs - intervalUs, start) ||
        !poseAt(completedUs - intervalUs / 2U, middle) ||
        !poseAt(completedUs, end) ||
        hypotf(end.x_mm - start.x_mm, end.y_mm - start.y_mm) > 25.0f ||
        fabsf(wrapExitAngle(end.heading_deg - start.heading_deg)) > 8.0f) {
        if (rejected < 65535) ++rejected;
        return false;
    }
    const uint32_t sampledUs = completedUs - intervalUs / 2U;
    if (count && sampledUs - lastCapturedUs < 60000U) return false;
    const Blob *red = obstacle_blob_shape_valid(&vision.redPillar) ? &vision.redPillar : &vision.red;
    const Blob *green = obstacle_blob_shape_valid(&vision.greenPillar) ? &vision.greenPillar : &vision.green;
    const bool redValid = obstacle_blob_shape_valid(red);
    const bool greenValid = obstacle_blob_shape_valid(green);
    if (!redValid && !greenValid) return false;
    if (count == CAPACITY) {
        first = (first + 1) % CAPACITY;
        --count;
        if (replaced < 65535) ++replaced;
    }
    ParkingExitCameraFrame &stored = frames[(first + count++) % CAPACITY];
    stored.pose = middle;
    stored.red = redValid ? *red : Blob{};
    stored.green = greenValid ? *green : Blob{};
    stored.sampledUs = sampledUs;
    stored.sequence = sequence;
    lastCapturedUs = sampledUs;
    return true;
}

void ParkingExitCameraBuffer::translate(float dx, float dy) {
    if (!enabled) return;
    if (!isfinite(dx) || !isfinite(dy)) { reset(); return; }
    for (uint8_t i = 0; i < historyCount; ++i) {
        PositionEstimate &p = history[(historyFirst + i) % HISTORY_CAPACITY].pose;
        p.x_mm += dx; p.y_mm += dy;
    }
    for (uint16_t i = 0; i < count; ++i) {
        PositionEstimate &p = frames[(first + i) % CAPACITY].pose;
        p.x_mm += dx; p.y_mm += dy;
    }
}

const ParkingExitCameraFrame &ParkingExitCameraBuffer::frame(uint16_t index) const {
    return frames[(first + index) % CAPACITY];
}
