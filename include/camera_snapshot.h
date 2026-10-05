#pragma once
// Diagnostic only: stationary camera mode, physical drive enable off.
void camera_snapshot_export();
// Stopped, drive-disabled ten-frame check of the mapped-seat green fallback.
void camera_green_seat_diagnostic(int expected_x, int expected_foot_y);
