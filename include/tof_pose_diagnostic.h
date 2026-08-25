#pragma once

/** Stationary, motor-locked validation of the production obstacle-course ToF
 * pose correction. Commands are handled directly and do not require a robot
 * drive mode. */
bool tof_pose_diagnostic_handle_command(const char *message);
void tof_pose_diagnostic_update();
bool tof_pose_diagnostic_active();
