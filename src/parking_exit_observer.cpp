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
    exitCamera.capture(micros(), camera.getLastFrameCompletedTimeUs(),
        camera.getLastFrameIntervalUs(), camera.getCompletedFrameCount(), vision.getResult());
}
void parking_exit_observer_translate(float dx, float dy) { exitCamera.translate(dx, dy); }
void parking_exit_observer_apply() {
    if (!exitCamera.active()) return;
    const uint32_t startedUs = micros();
    robot_logger.print("[EXIT VIEW] revision=20261030_corrected_pose frames/rejected/overwritten=");
    robot_logger.print(exitCamera.size()); robot_logger.print("/");
    robot_logger.print(exitCamera.dropped()); robot_logger.print("/");
    robot_logger.println(exitCamera.overwritten());
    obstacle_path_apply_exit_observations(exitCamera);
    robot_logger.print("[EXIT VIEW] import_us=");
    robot_logger.println(micros() - startedUs);
    exitCamera.reset();
}
