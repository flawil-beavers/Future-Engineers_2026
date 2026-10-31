#pragma once

#include <stdint.h>
#include "position_estimator.h"
#include "vision.h"

// Raw observations, not seat assignments. XY corrections translate both the
// pose history and previously captured frames before any map vote is allowed.
struct ParkingExitCameraFrame {
    PositionEstimate pose{};
    Blob red{}, green{};
    uint32_t sampledUs = 0;
    uint32_t sequence = 0;
};

class ParkingExitCameraBuffer {
public:
    static constexpr uint16_t CAPACITY = 160;
    static constexpr uint8_t HISTORY_CAPACITY = 32;
    void reset();
    void begin(int8_t turnSign);
    void samplePose(uint32_t nowUs, const PositionEstimate &pose);
    bool capture(uint32_t nowUs, uint32_t completedUs, uint32_t intervalUs,
                 uint32_t sequence, const VisionResult &vision);
    void translate(float dx, float dy);
    const ParkingExitCameraFrame &frame(uint16_t index) const;
    uint16_t size() const { return count; }
    bool active() const { return enabled; }
    int8_t direction() const { return turn; }
    uint16_t dropped() const { return rejected; }
    uint16_t overwritten() const { return replaced; }

private:
    struct TimedPose { PositionEstimate pose{}; uint32_t timeUs = 0; };
    bool poseAt(uint32_t timeUs, PositionEstimate &pose) const;
    TimedPose history[HISTORY_CAPACITY]{};
    ParkingExitCameraFrame frames[CAPACITY]{};
    uint16_t first = 0, count = 0, rejected = 0, replaced = 0;
    uint8_t historyFirst = 0, historyCount = 0;
    uint32_t lastSequence = 0, lastCapturedUs = 0;
    int8_t turn = 0;
    bool enabled = false, haveSequence = false;
};

void parking_exit_observer_reset();
void parking_exit_observer_begin(int8_t turnSign);
void parking_exit_observer_sample_pose();
void parking_exit_observer_capture();
void parking_exit_observer_translate(float dx, float dy);
void parking_exit_observer_apply();

// Called once, after obstacle_path_start has initialized the corrected field
// geometry. Never drives and never infers that an unobserved position is empty.
void obstacle_path_apply_exit_observations(const ParkingExitCameraBuffer &buffer);
