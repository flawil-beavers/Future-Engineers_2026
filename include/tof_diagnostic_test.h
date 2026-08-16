#pragma once

/** Handle a complete `tof ...` serial command. */
bool tof_diagnostic_handle_command(const char *message);

/** Advance an active stationary capture without blocking the robot loop. */
void tof_diagnostic_update();
