# Obstacle Challenge Test Plan

Run the tests in order. Do not continue to the next phase until every required
check in the current phase passes repeatedly.

Keep this file current after every firmware change and analyzed robot log.
Check an item only when the implementation or recorded evidence demonstrates
it; leave visual and repetition requirements open until they are explicitly
confirmed.

## Safety before every driving test

- [ ] Battery is secured and sufficiently charged.
- [ ] Emergency disable switch is reachable.
- [ ] USB cable cannot catch the wheels or steering.
- [ ] Test area is clear of people and fragile objects.
- [ ] Begin with the lowest practical speed.
- [ ] Save the serial log and note the exact configuration used.

## 1. Physical geometry - in progress

- [x] Measure the ToF apertures relative to the robot pose origin.
- [x] Update the sensor offsets in `include/config.h`.

Current measured values:

```cpp
OBSTACLE_TOF_LEFT_LOCAL_X_MM = 40.0f;
OBSTACLE_TOF_LEFT_LOCAL_Y_MM = 35.0f;
OBSTACLE_TOF_RIGHT_LOCAL_X_MM = 40.0f;
OBSTACLE_TOF_RIGHT_LOCAL_Y_MM = -35.0f;
```

- [x] Confirm that the pose origin used for these measurements is the same
      origin used by odometry, normally the midpoint of the rear axle.
- [x] Measure the camera origin: 125 mm forward and 0 mm lateral from the
      rear-axle pose origin.
- [x] Apply the rotated camera mounting offset before projecting a detected
      block into global field coordinates.
- [ ] Confirm `OBSTACLE_WHEELBASE_MM` against the finished robot.

## 2. Empty-track path and direction - in progress

Use the dedicated on-robot test mode. It automatically disables camera
steering, look nudges, and ToF pose correction while retaining ToF emergency
wall stopping. It drives one lap, brakes, and prints a PASS/FAIL report.

Preparation:

- Remove every pillar from the track.
- Place the robot on the corridor centerline, parallel to the straight.
- Place it at the longitudinal midpoint of the starting straight. The test
  path expects the first corner approximately 500 mm ahead.
- Keep the physical disable switch and serial `z` command ready.

Commands after uploading the firmware:

```powershell
$pio = Join-Path $env:USERPROFILE '.platformio\penv\Scripts\platformio.exe'
& $pio run --environment giga_r1_m7 --target upload
```

Then open the 115200-baud serial monitor. For the safest start, leave the
physical enable switch disabled, send the desired command so the mode becomes
pending, position the robot, clear the track, and only then enable the switch.

```text
X1     start one lap with left/CCW corners
X-1    start one lap with right/CW corners
X0     stop the path test
z      emergency stop any active mode
```

The mode prints telemetry every 500 ms and aborts automatically for a wall
closer than the configured safety distance, excessive cross-track error, or a
timeout. Test limits are the `OBSTACLE_PATH_TEST_*` constants in
`include/config.h`.

- [x] Confirm the on-board geometry preflight reports `PASS` before motion.
- [x] Confirm the generated path turns in the required direction.
- [ ] Confirm the first corner begins at the correct place.
- [x] Confirm the robot remains approximately on the corridor centerline.
- [x] Confirm it returns close to its starting pose after one lap.
- [x] Confirm `[PATH] Completed lap 1` appears exactly once per physical lap.
- [ ] Repeat successfully at least three times clockwise.
- [ ] Repeat successfully at least three times counterclockwise.

Recorded evidence from `log_72`, `log_73`, `log_75`, and `log_76`:

- [x] Two successful left/CCW one-lap runs.
- [x] Two successful right/CW one-lap runs.
- [x] All four valid runs passed the automated limits.
- [x] Worst maximum cross-track error was 36.6 mm.
- [x] Average final-position error was 49.8 mm.
- [x] Average final-heading error was 2.1 degrees.

Pass criteria:

- No wall contact.
- No incorrect-direction corner.
- No premature or missing lap increment.
- The final pose error is measured and recorded after each lap.

If starting without the parking exit, set and verify
`OBSTACLE_DEFAULT_TURN_SIGN`: `+1` means left/CCW corners and `-1` means
right/CW corners.

## 3. Pure Pursuit tuning on an empty track - in progress

Tune in this order:

1. `OBSTACLE_PATH_MIN_SPEED`
2. `OBSTACLE_PATH_MAX_SPEED`
3. `OBSTACLE_LOOKAHEAD_MIN_MM`
4. `OBSTACLE_LOOKAHEAD_MAX_MM`
5. `OBSTACLE_LOOKAHEAD_CORNER_SCALE`
6. `OBSTACLE_MAX_PURSUIT_STEERING_DEG`

- [ ] Straight sections do not oscillate.
- [ ] Steering returns smoothly to center after corrections.
- [ ] The robot does not cut across the inside of corners.
- [ ] The robot does not run wide toward the outer wall.
- [ ] Speed decreases sufficiently at corners.
- [ ] One empty lap succeeds five consecutive times in each direction.

Adjustment guide:

- Straight oscillation: increase lookahead or reduce maximum speed.
- Corner cutting: reduce `OBSTACLE_LOOKAHEAD_CORNER_SCALE`.
- Running wide: reduce corner speed or lookahead.
- Jerky steering: increase lookahead or reduce maximum steering/speed.

## 4. Camera bearing and range calibration - in progress

Use one official 100 mm pillar. Test both colors at measured distances of
approximately 150, 250, 400, 600, and 800 mm.

The camera test is stationary and keeps the drive motor stopped. Place the
front of the camera at the measured distance from the pillar, then send the
matching command:

```text
c150
c250
c400
c600
c800
```

Changing the distance resets the per-color sample average. Allow at least ten
samples at each distance and save the serial log. Each `[CAM CAL]` record
contains blob position and size, bearing, the firmware's current range
estimate, whether the image edge clipped the blob, the focal-length sample,
the average for that color, the exact bounding limits, and
`production_valid=yes|no`. The faint green competition-mat edge may still be
reported for diagnostics, but it must always report `production_valid=no` and
cannot enter live seat snapping. Use `c0` for diagnostics without a known
distance and `z` to stop the mode.

Normal range estimation uses the calibrated image position of the block's foot
(`max_y`), because the top of a distant block is intentionally clipped by the
obstacle ROI. `range_error_mm` and `range_error_avg_mm` compare that production
estimate with the distance supplied in the `c<mm>` command. Measure that
distance horizontally from the camera to the foot of the block. Blob-height
focal samples remain diagnostic only.

- [x] Record blob height, bounds, center X, estimated bearing, and estimated
      range in `log_81`, `log_83`, and `log_85`.
- [x] Calculate diagnostic focal length samples:

  `focal_length_px = range_mm * blob_height_px / 100`

- [x] Confirm that height-based focal range is unsuitable when the obstacle ROI
      clips the top of the block; retain 277 px as a fallback only.
- [x] Calibrate production foot range from ruler measurements. Final restored-
      ROI values from `log_88` are 400 mm at `max_y=136` and 600 mm at
      `max_y=108`.
- [x] Implement the ground-plane range model and serial range-error reporting.
- [x] Verify the ground-plane model with a post-update `c400` and `c600`
      run for both colors.
- [ ] Confirm the final two-pixel horizon refinement (`52 px`) in the next
      camera or stationary seat-snapping log.
- [ ] Verify `OBSTACLE_CAMERA_HORIZONTAL_FOV_DEG` at both frame edges.
- [ ] Tune `OBSTACLE_EDGE_CLIPPED_RANGE_MM` using close, clipped pillars.
- [x] Red classification is stable under expected lighting at 400 and 600 mm
      (`log_85`, at least 25 accepted samples at each distance).
- [x] Green classification is stable under expected lighting at 400 and
      600 mm (`log_88`, repeated accepted stationary samples).
- [x] Faint green competition-mat/background components report
      `production_valid=no` and are excluded from calibration and live
      Pure Pursuit seat input.
- [x] A valid green pillar remains independently detectable when the mat line
      is visible under competition lighting.
- [ ] Background objects do not produce confirmed seats during the stationary
      seat-snapping test.
- [x] Two consecutive observations are normally available before the pillar
      reaches the minimum safe reaction distance.

Recorded camera evidence from `log_88`:

- [x] Red pillar repeatedly accepted at 400 and 600 mm.
- [x] Green pillar repeatedly accepted at 400 and 600 mm.
- [x] The faint left-edge green mat component remained
      `production_valid=no sample_accepted=no`.
- [x] Dominant 400 mm samples were within approximately 10-20 mm before the
      final two-pixel horizon adjustment.
- [x] Dominant 600 mm samples were within approximately 0-46 mm before the
      final adjustment; the stable clusters support the final horizon value.
- [x] Camera-to-block range is transformed from the camera origin 125 mm ahead
      of the rear axle before seat snapping.

## 5. Seat snapping while stationary or pushed by hand

Keep the drive motor disabled. Place a pillar on each of the six seat types in
one straight section and move the robot through the expected approach.

- [ ] Each detection snaps to the correct seat.
- [ ] No detection snaps to the neighboring longitudinal station.
- [ ] No detection snaps to the seat on the opposite side of the corridor.
- [ ] A pillar produces only one live-injection event.
- [ ] A single bad-color frame does not confirm the wrong color.
- [ ] Test all seat types with red.
- [ ] Test all seat types with green.

Tune only after checking camera calibration and odometry:

- `OBSTACLE_SEAT_SNAP_RADIUS_MM`
- `OBSTACLE_SEAT_CONFIRM_VOTES`

Keep the snap radius as small as practical. Increasing it can hide a pose or
camera-calibration error and select the wrong seat.

## 6. Live lap-1 avoidance

Start with one pillar in the middle of a straight and use minimum speed.

- [ ] Red pillar: robot passes on the right.
- [ ] Green pillar: robot passes on the left.
- [ ] Avoidance is injected immediately after confirmation.
- [ ] Serial output contains exactly one
      `[PATH] Live avoidance injected` event.
- [ ] Steering enters and leaves the detour smoothly.
- [ ] The robot clears the pillar and its 85 mm movement circle.
- [ ] The robot retains safe wall clearance.
- [ ] The robot returns smoothly to the nominal path.
- [ ] Repeat with the pillar at both lateral seat positions.
- [ ] Repeat at the first, middle, and last station in a straight.
- [ ] Repeat in all four straight sections.

Tune:

- `OBSTACLE_LAP1_CLEARANCE_MM`
- `OBSTACLE_PATH_TAPER_WAYPOINTS`
- `OBSTACLE_PATH_SMOOTH_RADIUS`
- Minimum speed and short lookahead

## 7. Look-heading nudges

Test candidate seats that are outside the camera view on the nominal path.

- [ ] The chassis begins looking toward the seat before the reaction point.
- [ ] The seat enters the camera frame reliably.
- [ ] The nudge does not make the robot cross an unsafe lane position.
- [ ] The nudge tapers out smoothly.
- [ ] A confirmed seat is not scanned repeatedly.

Tune:

- `OBSTACLE_LOOK_START_MM`
- `OBSTACLE_LOOK_END_MM`
- `OBSTACLE_LOOK_HEADING_KP`
- `OBSTACLE_LOOK_MAX_STEERING_DEG`

## 8. ToF odometry correction

### TODO: Resolve unreliable right-ToF readings on black walls

This is deliberately deferred. Do not enable or tune ToF odometry correction
until it is resolved.

Observed at a measured 500 mm right-sensor-to-wall distance:

- Black wall: 10 readings averaged 354.0 mm (335-367 mm).
- White paper over the same wall, without moving the robot: 10 readings
  averaged 504.8 mm (503-507 mm).
- The right sensor was also accurate at approximately 300 mm from the black
  wall, so this is not a constant distance offset.

The white-target result confirms that the sensor position and aiming can
produce an accurate measurement. The current evidence points to weak black-wall
returns, filtering, or timing-budget configuration. Do not compensate with a
fixed calibration offset.

- [ ] Add diagnostics for selected distance, raw distance, signal rate, sigma,
      and active timing budget for both sensors.
- [ ] Correct initialization so `TOF_TIMING_BUDGET_US` is explicitly applied;
      it is currently declared but not applied during normal initialization.
- [ ] Record a black-versus-white baseline at 300, 400, and 500 mm.
- [ ] Repeat with a 100 ms timing budget and use 200 ms if necessary.
- [ ] Review `min_accept_signal` and `max_accept_sigma` only after collecting
      the diagnostic data.
- [ ] Verify both sensors on black walls within an agreed tolerance before
      enabling ToF pose correction.

Restore `OBSTACLE_TOF_CORRECTION_GAIN` and mark several ground-truth robot
positions on each straight.

- [ ] Compare the reported pose before and after correction.
- [ ] Correction moves the estimate toward ground truth.
- [ ] Correction is primarily lateral on straight sections.
- [ ] Readings at or above 500 mm do not correct the pose.
- [ ] Left and right sensors correct in the proper direction.
- [ ] Repeated readings converge instead of pulling the estimate past truth.
- [ ] The inside/receding sensor is ignored around every corner.
- [ ] The valid outer sensor does not introduce a corner jump.
- [ ] Test all four corners in both driving directions.

Tune:

- `OBSTACLE_TOF_CORRECTION_GAIN`
- `OBSTACLE_TOF_CORRECTION_MAX_STEP_MM`
- `OBSTACLE_CORNER_GATE_BEFORE_MM`
- `OBSTACLE_CORNER_GATE_AFTER_MM`

If correction has the wrong sign, verify the sensor coordinates and left/right
convention before changing the gain.

## 9. Optimized laps 2 and 3

- [ ] After lap 1, serial output reports
      `[PATH] Optimized laps 2-3 path built`.
- [ ] Only confirmed occupied seats affect the optimized path.
- [ ] Look-heading scanning is inactive on laps 2 and 3.
- [ ] The tighter optimized clearance remains safe at every seat position.
- [ ] Residual vision trim improves small errors without overpowering pursuit.
- [ ] Lap progress wraps exactly once per physical lap.
- [ ] The robot completes three laps without recomputing the optimized path.

Tune:

- `OBSTACLE_OPTIMIZED_CLEARANCE_MM`
- `OBSTACLE_RESIDUAL_VISION_KP`
- `OBSTACLE_RESIDUAL_VISION_MAX_DEG`

## 10. Full-field regression

- [ ] Clockwise three-lap run.
- [ ] Counterclockwise three-lap run.
- [ ] Every intended starting configuration.
- [ ] Pillars on every seat type.
- [ ] Two nearby pillars with opposing colors.
- [ ] Maximum expected number of pillars.
- [ ] Bright lighting, dim lighting, and shadows.
- [ ] Partially occluded and edge-clipped pillars.
- [ ] Low and high battery conditions.
- [ ] Ten consecutive three-lap runs without a wrong-side pass, moved pillar,
      wall contact, or lap-count error.
- [ ] Controlled stop occurs after exactly three laps.

## 11. Final parking - not implemented yet

The current code stops safely in the starting section after three laps. It does
not perform the required final parallel-parking maneuver.

- [ ] Design and implement the final parking state machine.
- [ ] Detect and approach the correct parking gap.
- [ ] Reverse into the lot without touching either magenta boundary.
- [ ] Finish fully inside the parking rectangle.
- [ ] Verify parallel alignment: side-distance difference no greater than
      20 mm.
- [ ] Test parking independently before adding it to a three-lap run.

## Test record

Copy this block for every test session:

```text
Date/time:
Firmware commit:
Driving direction:
Starting configuration:
Pillar layout:
Battery voltage:
Configuration changes:
Expected result:
Actual result:
Pass/fail:
Pose error after lap:
Relevant serial messages:
Video/log filename:
Next adjustment:
```
