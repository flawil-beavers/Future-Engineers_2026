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

### Rules basis for this phase

- [x] Confirm each straight section contains six traffic-sign seats: three
      longitudinal stations by two lateral positions (rules Figure 3).
- [x] Confirm every seat is 50 x 50 mm (rule 13.13).
- [x] Confirm every traffic sign is 50 x 50 x 100 mm (rule 13.19).
- [x] Confirm the movement-evaluation circle is 85 mm in diameter
      (rules 13.14-13.15).
- [x] Confirm red must be passed on the right and green on the left
      (rule 9.19).
- [x] Keep the test compatible with either randomized driving direction and
      all official seat positions.

### Firmware implementation plan

1. **Separate observation from control**

   - [x] Refactor the current private `registerDetection()` logic into one
         shared observation function used by both competition mode and test
         mode.
   - [x] Keep production validation, camera-foot range, the rotated 125 mm
         camera offset, nearest-seat lookup, voting, confirmation, and tapered
         live-path injection in that single code path.
   - [x] Return a read-only result structure containing rejection reason,
         projected global point, selected seat, snap error, votes,
         confirmation state, and injection count.
   - [x] Expose read-only seat metadata: seat ID, section, station, lateral
         side, path distance, global position, and confirmed color.

2. **Add deterministic geometry checks**

   - [x] Verify all 24 generated seats are finite, unique, and within the
         known straight-section geometry for left/CCW paths.
   - [x] Repeat the same checks for right/CW paths.
   - [x] Verify a point at every seat center selects that exact seat.
   - [x] Verify points just inside/outside `OBSTACLE_SEAT_SNAP_RADIUS_MM`
         behave correctly.
   - [x] Verify neighboring longitudinal and opposite-lateral seats cannot win
         for a perfect observation.
   - [x] Verify red selects the right-pass displacement and green selects the
         left-pass displacement.
   - [x] Verify two matching votes confirm once, while one vote cannot confirm
         or inject.

3. **Create a motor-locked on-robot mode**

   - [x] Add `MODE_OBSTACLE_SEAT_TEST` with its own small source/header module.
   - [x] Initialize the real camera and known Pure Pursuit geometry, but never
         call the steering/speed controller.
   - [x] Reassert motor stop, zero target speed, and centered/disabled steering
         on every update, regardless of the physical enable-switch state.
   - [x] Disable ToF pose correction, look-heading nudges, lap advancement,
         parking behavior, and normal challenge state transitions.
   - [x] Permit path displacement only in the isolated test path so the real
         injection code is exercised without vehicle motion.
   - [x] Keep `z`, the physical disable switch, and a dedicated stop command
         effective at all times.

4. **Add serial controls**

   - [x] `S1`: start the seat test with left/CCW field geometry.
   - [x] `S-1`: start the seat test with right/CW field geometry.
   - [x] `S0`: stop and clear the seat test.
   - [x] `seat expect <section> <station> <L|R> <range_mm>`: arm one expected
         seat and set the test-only odometry pose to the known approach pose.
   - [x] `seat clear`: clear votes/injection state before changing the pillar.
   - [x] `seat show`: print all seat coordinates and current states.
   - [x] Reject malformed indices, sides, ranges, or commands without changing
         the active test state.

5. **Add concise diagnostic output**

   - [x] Print blob color, bounds, production-valid flag, bearing, and
         camera-to-foot range.
   - [x] Print robot-origin pose, rotated camera position, projected sighting
         position, nearest seat label, and snap error in millimetres.
   - [x] Print red/green vote counts and explicit events for `VOTE`,
         `CONFIRMED`, `INJECTED`, `REJECTED`, and `WRONG_SEAT`.
   - [x] Print the displaced waypoint peak, pass side, taper extent, and
         clearance from the 85 mm movement circle.
   - [x] Throttle repeated frame telemetry while never suppressing state-change
         or failure events.

6. **Compile and bench-verify before field testing**

   - [x] Build incrementally with the IDE-managed PlatformIO installation;
         preserve `.pio` and the package cache.
   - [x] Confirm the geometry preflight and motor-lock preflight both pass.
   - [ ] Confirm the mode remains safe through start, pause, resume, malformed
         commands, `S0`, `z`, and USB disconnect.
   - [x] Confirm the existing empty-track `X1`/`X-1`, camera `c<mm>`, and
         competition modes still compile and retain their commands.

First on-robot smoke logs: both preflights passed and the test mode stayed
motor-locked. Two serial portability issues caused valid `seat expect` commands
to be rejected: CRLF left trailing whitespace, and this embedded C library does
not link floating-point `scanf` support. Command input is now trimmed and the
whole-millimetre range is parsed as an integer before conversion to the internal
float. Repeat the first seat observation with the rebuilt firmware before
checking the remaining safety and physical items.

The next smoke log armed `0/0/L` successfully and proved the confirmation guard:
the red pillar produced two votes, exactly one injection, and then remained at
`injections=1`. It also exposed an image-to-path sign mismatch: an image-left
blob at `centerX` about 97 was projected with a negative/right bearing and
therefore snapped to seat `0/R` instead of expected seat `1/L`. Camera bearing
now converts image-left to positive/path-left in one shared helper used by both
calibration diagnostics and live seat projection. Repeat `0/0/L` before
expanding the matrix.

The corrected `0/0/L` repeat passed with the camera on the field centerline and
the red pillar's foot measured 400 mm from the camera. Valid observations used
a positive-left bearing of about 16.1-16.3 degrees, selected
`seat=1 expected=1` with 29-37 mm snap error, confirmed after two red votes,
injected a right-pass path exactly once, and remained at `injections=1`.
Intermittent smaller red fragments were rejected after confirmation and did not
change the seat or injection state.

The first `0/0/R` run initially selected the expected right seat with a
negative bearing and injected the correct red right-pass path. It later failed
the overall case: two isolated false red blobs on the far image-left accumulated
stale votes around many correct/rejected frames, confirmed seat `1/L`, and
raised the injection count to two. Voting now requires consecutive accepted
observations of the same seat and color; another seat, invalid blob, invalid
range, or no-seat observation clears pending unconfirmed votes. Repeat `0/0/R`
before marking it complete.

The false component was suspected to be the distant orange boundary line. It
appeared at image `x=32-56`, with `minY=80` exactly on the
obstacle ROI boundary, while the real right-side pillar was at `x=228-276`.
The classifier currently checks red first at hue <=18 and orange second at hue
9-35, so camera/lighting shifts can classify the overlapping 9-18 range as red.
Before changing HSV thresholds, run `c0` with no pillar and the orange line in
the same top-left position; save whether it reports a production-valid RED blob
with matching bounds.

The no-pillar `c0` control did **not** reproduce the suspected orange-line
misclassification: every sampled frame reported `blob=NONE`, including after
the robot was moved slightly. The printed `center_hsv` samples describe only
the image-centre pixel, not the top-left line, but `blob=NONE` confirms that no
qualifying red/green component was found anywhere in those frames. Do not tune
HSV from this hypothesis. Treat the earlier left-side red components as
intermittent/pillar-dependent until another controlled test reproduces them;
the consecutive-vote protection must still prevent them from injecting.

The rebuilt `0/0/R` repeat passed. The real pillar selected
`seat=0 expected=0` at a negative 12-degree bearing with 20-27 mm snap error,
confirmed after two consecutive red votes, and injected the right-pass path
once. Two later false left-seat observations each reached only `R1`; correct or
rejected intervening frames cleared the pending vote, so seat 1 never confirmed
and the run remained at `injections=1`. This validates the consecutive-vote
regression fix under the artifact that previously caused a second injection.

The first station-1 left/right attempts did not confirm. Valid pillar frames
were interleaved with rejected small fragments, so the strict consecutive-frame
implementation repeatedly returned correct-seat evidence to `R1`. Voting now
uses a 400 ms evidence window: rejected/no-blob/no-seat flicker only expires a
vote after that window, while an accepted different seat or different color
still resets incompatible pending evidence immediately. This retains the
false-seat protection without requiring two adjacent camera frames. The
station-1 left observation also estimated only 251-263 mm and reached the
140 mm snap boundary, so repeat it only after repositioning the robot—not just
the block—to restore the measured 400 mm camera-to-foot distance and centreline
alignment. Neither station-1 matrix entry is complete yet.

### Physical test matrix after implementation

For each case, place the robot on the straight centerline at the commanded
400-600 mm approach pose, parallel to the straight, and place one official
pillar squarely on the selected 50 mm seat. Keep the wheels off powered drive;
re-arm/reset the test between cases. `range_mm` is the ruler distance from the
camera to the foot of the pillar. `L` and `R` name the seat as seen in the
driving direction; they do not name the required passing side.

Upload the firmware, leave the robot supported so the drive wheels cannot move,
and run this first smoke test:

1. Send `S1`; require both `[SEAT] geometry preflight LEFT+RIGHT: PASS` and
   `[SEAT] motor lock: PASS`.
2. Send `seat show` and save the 24-seat table.
3. Put a pillar at section 0, station 0, left seat, measure 400 mm from the
   camera to its foot, then send `seat expect 0 0 L 400`.
4. Require one `VOTE`, then one `CONFIRMED_INJECTED`, with `seat=1`,
   `expected=1`, and `injections=1`. Continued frames must remain at one
   injection.
5. Send `seat clear`, move the pillar, and arm the next matrix entry. Use `S0`
   or `z` at any time to stop and clear the mode.

- Red pillar at the left lateral position:
  - [x] Station 0.
  - [ ] Station 1.
  - [ ] Station 2.
- Red pillar at the right lateral position:
  - [x] Station 0.
  - [ ] Station 1.
  - [ ] Station 2.
- Green pillar at the left lateral position:
  - [ ] Station 0.
  - [ ] Station 1.
  - [ ] Station 2.
- Green pillar at the right lateral position:
  - [ ] Station 0.
  - [ ] Station 1.
  - [ ] Station 2.
- [ ] Repeat representative near/far seats with right/CW geometry.
- [ ] Run with no pillar and the faint green mat line visible; no seat may
      receive a vote.
- [ ] Present one isolated bad-color frame between correct frames; it must not
      confirm or inject the wrong color.
- [ ] Save serial logs and record the expected and selected seat for every
      matrix entry.

### Phase pass criteria

- [ ] Each detection snaps to the correct seat.
- [ ] No detection snaps to the neighboring longitudinal station.
- [ ] No detection snaps to the seat on the opposite side of the corridor.
- [x] A pillar produces only one live-injection event. (Passed `0/0/L` and the
      rebuilt `0/0/R`; isolated false-seat observations remained at one vote.)
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

### ToF sensor validation and remaining runtime work

The original unreliable right-hand module has been isolated and replaced. The
fault followed that physical module when the two sensors and I2C ports were
swapped, so it was not caused by the port, mounting side, or a constant software
offset. Keep the defective module out of navigation-critical use.

Observed at a measured 500 mm right-sensor-to-wall distance:

- Black wall: 10 readings averaged 354.0 mm (335-367 mm).
- White paper over the same wall, without moving the robot: 10 readings
  averaged 504.8 mm (503-507 mm).
- The right sensor was also accurate at approximately 300 mm from the black
  wall, so this is not a constant distance offset.

The white-target result confirmed that the sensor position and aiming could
produce an accurate measurement. The later swap test isolated the fault to the
module itself. Do not compensate for the discarded module with a fixed offset.

The replacement sensor passed controlled black-wall captures with 20/20 valid
samples at each distance: 304.5 mm at 300 mm, 403.0 mm at 400 mm, and 498.4 mm
at 500 mm. Its 500 mm mean was accurate, although individual unfiltered samples
ranged from 467-528 mm. MEDIUM mode at a 30 ms timing budget is therefore the
accepted provisional normal configuration.

- [x] Add diagnostics for selected distance, raw distance, signal rate, sigma,
      range status, object candidates, active distance mode, and timing budget
      for both sensors.
- [x] Correct initialization so `TOF_TIMING_BUDGET_US` is explicitly applied
      and the actual mode and budget are read back and printed.
- [x] Isolate the original failure by swapping both modules and I2C ports.
- [x] Replace the defective module and verify the replacement on a black wall
      at controlled 300, 400, and 500 mm distances.
- [ ] Complete matched black-versus-white captures at 300, 400, and 500 mm if
      surface-color characterization is still desired.
- [ ] Repeat with a 100 ms timing budget and use 200 ms if necessary.
- [x] Review `min_accept_signal` and `max_accept_sigma` after collecting the
      diagnostic data. Retain `0.3 Mcps / 20 mm` for normal operation and
      `0.3 Mcps / 30 mm` provisionally for discovery mode.
- [ ] Verify the other navigation sensor at controlled 300, 400, and 500 mm
      black-wall distances so both installed modules have equivalent evidence.
- [ ] Confirm whether runtime discovery intentionally uses MEDIUM mode at
      300 ms or should switch to hardware LONG mode; `nav_long_range_active`
      currently changes the budget and filters but not the distance mode.
- [ ] Repeatedly switch runtime discovery from 30 ms to 300 ms and back while
      checking the reported actual budget and continued frame delivery.
- [ ] Make corner/opening confirmation count distinct ToF frame sequence
      numbers rather than repeated control-loop uses of one cached reading.
- [ ] Perform a moving black-wall disappearance and reacquisition test at the
      intended driving speed before treating long-range discovery as final.
- [x] Confirm USB saves retain the requested RAM data until the completed file
      has been flushed, closed, reopened, and size-verified.

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
