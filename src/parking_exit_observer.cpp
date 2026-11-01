#include "parking_exit_observer.h"
#include "obstacle.h"
#include "logger.h"

namespace { ParkingExitCameraBuffer exitCamera; }

void parking_exit_observer_reset() { exitCamera.reset(); }
void parking_exit_observer_begin(int8_t turnSign) {
    exitCamera.begin(turnSign);
    parking_exit_observer_sample_pose();
}
void parking_exit_observer_sample_pose() {
    if (exitCamera.active()) exitCamera.samplePose(micros(), get_position_struct());
}
void parking_exit_observer_capture() {
    if (!exitCamera.active()) return;
    // DMA often finishes just after the last throttled history sample. Add
    // the post-processing pose too, so completion is bracketed rather than
    // rejecting almost every otherwise usable moving frame (594--601).
    const uint32_t now = micros();
    exitCamera.samplePose(now, get_position_struct(), true);
    exitCamera.capture(now, camera.getLastFrameCompletedTimeUs(),
        camera.getLastFrameIntervalUs(), camera.getCompletedFrameCount(), vision.getResult());
}
void parking_exit_observer_translate(float dx, float dy) { exitCamera.translate(dx, dy); }
void parking_exit_observer_apply() {
    if (!exitCamera.active()) return;
    const uint32_t startedUs = micros();
    robot_logger.print("[EXIT VIEW] revision=20261031_frame_bracket frames/rejected/overwritten=");
    robot_logger.print(exitCamera.size()); robot_logger.print("/");
    robot_logger.print(exitCamera.dropped()); robot_logger.print("/");
    robot_logger.println(exitCamera.overwritten());
    obstacle_path_apply_exit_observations(exitCamera);
    robot_logger.print("[EXIT VIEW] import_us=");
    robot_logger.println(micros() - startedUs);
    exitCamera.reset();
}
