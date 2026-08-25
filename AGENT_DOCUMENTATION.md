# Agent documentation and engineering handoffs

This is the repository's durable handoff log for coding agents. It records
verified project state, important decisions, test evidence, operational
constraints, and the next concrete action. It complements `AGENTS.md`, which
  contains mandatory workspace instructions. Focused test plans contain only
  the current ordered tests and their go/no-go criteria; engineering history,
  calibration, rationale, and completed-test evidence belong here.

## How to maintain this file

- Add a dated entry for a substantial investigation or development session.
- Put the newest entry first.
- Separate measured facts from hypotheses and proposed changes.
- Include source files, log names, build results, and reproducible next steps.
- Ask the user when an unresolved question could materially affect safety,
  architecture, hardware placement, calibration, the validity of a test, or
  how a change should be implemented. If the intended behaviour, design choice,
  or tradeoff is unclear, ask before implementing it.
- If work can safely continue with a reasonable assumption, state the
  assumption to the user and record it in the relevant dated entry. Never
  present an assumption as a measured or verified fact.
- Preserve still-relevant earlier entries; consolidate them only when their
  conclusions have been superseded and state what replaced them.
- Never store credentials, personal access tokens, or large raw logs here.

---

## 2026-08-25 - Obstacle test-plan documentation consolidation

`OBSTACLE_CHALLENGE_TEST_PLAN.md` was reduced to the tests that remain to be
run. Its former development narrative, calibration notes, completed gate
evidence, failure analysis, and tuning rationale are retained in the obstacle
handoffs below. This file is now authoritative for that durable engineering
context; the test plan is authoritative for test order and acceptance criteria.

Current obstacle state at consolidation:

- Empty-track Pure Pursuit is accepted at the 175 mm/s validation cap in both
  directions. Camera calibration, asynchronous acquisition, stationary
  perception, and ToF localization/residual gating are validated.
- Representative red-left and green-right left/CCW one-lap layouts pass.
- Red-right right/CW passes without contact with 260 mm first-lap clearance.
- Green-left right/CW completed without a stall at 260 mm clearance, but the
  observed approximately 1 mm physical gap is not a robust margin.
- `OBSTACLE_PATH_TAPER_WAYPOINTS` was therefore increased from 6 to 8. The
  IDE-managed `giga_r1_m7` build passed with 291176 bytes RAM and 352192 bytes
  flash. It has not been uploaded by an agent.

Exact next tests, in order:

1. After the user uploads, repeat green-left right/CW with the eight-waypoint
   taper. Require a practical visible gap at the wheels/body and wall,
   continuous motion, the correct green seat-5 detection, exactly one
   injection, all stations clear, and one completed lap.
2. If that passes, regress red-right right/CW because the wider taper changes
   both pass sides. Require the same no-contact/no-stall conditions, correct
   red seat-4 detection, one injection, all stations clear, and a completed
   lap.
3. Then exercise station 0 and station 2 placements, followed by two adjacent
   opposing-colour pillars. These tests specifically cover detour overlap and
   different longitudinal seat positions.
4. Only after those one-lap tests pass, proceed to three-lap optimized-path
   validation, full-field reliability regression, speed optimization, and
   final parking, in that order.

The live-test controls remain `Y1` for left/CCW, `Y-1` for right/CW, and `Y0`
to abort and brake. Arm with the physical enable switch LOW and keep the switch
reachable. The user performs powered runs and provides the USB log. Do not
upload firmware or make a new tuning change without explicit consent. Treat
physical contact as failure even if firmware reports `PASS`, and change only
one diagnosed cause at a time.

---

## 2026-08-25 - Asynchronous camera branch integration

`camera-async-buffering` was merged into `pure-pursuit`. Conflict resolution
kept the measured full-FOV calibration and the latest obstacle-discovery nudge,
while adding the project-owned Arducam driver, two fixed SDRAM framebuffers,
and non-blocking DCMI/DMA capture. `CAMERA_ASYNC_CAPTURE_ENABLED` and the safe
stationary camera auto-start are enabled. The stored routes and Pure Pursuit
steering calculation were not changed by this merge.

The merged build succeeds. Before the pending powered red-left run, repeat the
representative official-pillar seat test and field-clear test with motors
disabled. Confirm `[CAM PERF]` reports `async=yes`, advancing frame numbers,
stable detections, and no capture stalls. Only then resume the exact powered
test described in the next handoff entry.

`D:\log_39.txt` passed the async and official-pillar portions. Frames advanced
continuously through 1929; normal capture spans were about 76-84 ms, occasional
missed sensor-frame spans were about 150-159 ms, service cost was 75-83 us, and
typical processing/control blockage was about 6.2-6.8 ms. Seat 3 confirmed in
two frames with one injection and 12.6-22.0 mm snap error. A direct robot-side
attempt verified that the red pillar was still physically present, so a valid
field-clear control was impossible. The mode was stopped, the drive motor
remained locked, and COM4 was released. Remove the pillar; an agent may then
reconnect and perform the five-second field-clear test without another upload.

That direct continuation is now complete. With the pillar removed, the agent
resumed the motor-locked seat mode, armed `seat expect 0 1 L 400`, and observed
for more than seven seconds. Results alternated between no blob and tiny green
fragments around y=94-100; every fragment was rejected, votes remained `R0/G0`,
and injections remained zero. The mode was stopped and COM4 released. The next
step is the user-operated, post-async red-left powered run; do not retune the
nudge before analyzing it.

Three post-async powered repetitions are now available as `D:\log_40.txt`
through `D:\log_42.txt`. All three correctly injected red seat 5 exactly once,
then failed identically at S1 station 0: its right seat cleared, left seat 7
never appeared in `vis`, observations remained `NONE`, and the 35-degree nudge
cap arrived only after the viewing window. Maximum CTE was 69.0/67.9/67.8 mm
and maximum heading error 15.3/16.7/16.6 degrees. Async capture is not the
blocker. The old blocking loop sometimes produced more heading overshoot; the
smoother async controller no longer happens to swing the camera far enough.

Proposed next implementation, requiring user approval: only when one seat is
left unresolved, target it nearer the optical axis and use full bearing-error
gain so the existing 35-degree cap is reached sooner. Do not modify the stored
route, the simultaneous two-seat rule, perception confirmation, scan start
distance, or the Pure Pursuit steering formula. The user explicitly prohibited
firmware downloads without consent; code changes and uploads should be
separately confirmed if the requested scope is unclear.

The user approved the code change but not a firmware upload. The single-seat
target bearing is now 0 degrees and its dedicated gain is 1.0; the simultaneous
two-seat rule still uses the existing 0.75 gain. The scan start, 35-degree cap,
routes, perception, and Pure Pursuit formula are unchanged. Build locally and
request separate consent before uploading to the robot.

The local `giga_r1_m7` build passed at 291096 bytes RAM and 349952 bytes flash.
No upload or robot connection was performed.

The user subsequently uploaded and ran this firmware themselves. `D:\log_43.txt`
shows that the stronger single-seat response fixed the earlier first-corner
failure: S1 stations 0, 1, and 2 all cleared. At S2 station 0 the right seat
cleared, the nudge reached the unchanged 35-degree cap, and left seat 13 never
entered `vis`; S2 station 1 cleared independently. The robot stopped at the
135 mm perception limit. It made one correct obstacle injection, with 92.9 mm
maximum CTE and 20.9 degrees maximum heading error.

Do not infer the next controller change from `log_43` alone. `vis` combines the
predicted bearing window with the 260-600 mm range window, so the log cannot
distinguish an angular miss from seat 13 becoming too close. The next proposed
change is diagnostic only: expose predicted camera-relative bearing and range
for both seats of the active discovery station in `[LIVE PATH]` telemetry.
Ask before implementing if the intended diagnostic scope is uncertain, and
obtain explicit consent before any firmware upload.

The user approved that diagnostic implementation. `ObstacleDiscoveryTelemetry`
now carries predicted bearing and range for the right and left seats, and live
telemetry prints them as
`seat_geom=R<bearing_deg>/<range_mm>,L<bearing_deg>/<range_mm>`. It uses the
same camera/seat geometry as `vis` and does not change any controller,
perception, route, visibility, or timing decision. The IDE-managed build passed
at 291096 bytes RAM and 350208 bytes flash. It has not been uploaded; explicit
user consent is still required before uploading or connecting to the robot.

The user uploaded the merged 24 MHz/diagnostic firmware and produced
`D:\log_44.txt`. The geometry diagnostic isolates the S2 station 0 failure:
left seat 13 first entered the angular window at `L27.3/272`, then was already
below the current 260 mm range gate (`L25.3/238`) 201 ms later. Interpolation
leaves only about 70 ms of valid overlap, less than one normal 79.62 ms frame
interval and therefore insufficient for two clear frames. The run injected the
red pillar once, cleared all S1 stations and S2 station 1, then stopped safely;
maximum CTE was 94.3 mm and maximum heading error was 19.5 degrees.

Before lowering the view minimum, validate that both official pillar colours
remain production-valid at 220 mm slant range and about +/-25 degrees. Place
them relative to the camera lens about 200 mm forward and 93 mm left/right in
the motor-disabled camera mode. If both remain `production_valid=yes` after
settling, the proposed next code change is 260 to 220 mm for
`OBSTACLE_DISCOVERY_VIEW_MIN_MM`; ask before implementing and obtain separate
consent before uploading.

The direct COM4 test at the nominal 220 mm placement measured red at
+25.1 degrees/224.5 mm and green at -24.0 degrees/226.4 mm. Neither pillar was
edge-clipped and both were usually production-valid, but the boundary was not
fully reliable. Red intermittently lost its upper segment and failed the
`max_top_y` rule; green intermittently measured 81 px wide against the 80 px
acquisition maximum. The serial listener was stopped and COM4 released; no
drive command or upload occurred.

Do not lower the gate to 220 mm or test nearer yet. The next stationary check
is 230 mm at about +/-25 degrees: approximately 208 mm forward and 97 mm
left/right from the camera lens. A 230 mm gate would retain about 250 ms of the
S2 visibility overlap from `log_44`, enough for two frames even with one normal
snapshot miss. If both colours are stable there, ask before implementing a
260-to-230 mm change and separately before any upload.

The direct 230 mm COM4 check passed. The camera measured red at +25.4 degrees
and 237.3 mm, and green near -24.3 degrees and 235.2-239.8 mm. Neither was
edge-clipped; red was normally 77x111 px and green 73-77x109-111 px, below the
production size maxima. Over a settled 383-frame interval, green was valid
383/383 times and red 367/383 (95.8%). Red's occasional invalid frame was a
top-segmentation flicker rather than clipping. The available S2 window still
contains about three frames, enough for the independent two-vote obstacle and
two-clear-frame rules. COM4 was released without commands or upload. The next
proposed implementation is only `OBSTACLE_DISCOVERY_VIEW_MIN_MM` 260 to 230;
ask before changing it and separately before uploading.

The user approved the code change. `OBSTACLE_DISCOVERY_VIEW_MIN_MM` is now
230 mm; steering, stored routes, FOV, camera timing, and confirmation counts
are unchanged. Build locally and obtain separate permission before any upload.

The IDE-managed `giga_r1_m7` build passed at 291128 bytes RAM and 351136 bytes
flash. No upload or robot connection occurred. After explicit upload consent,
repeat one user-operated left/CCW run on the `log_44` layout and verify that S2
station 0 reaches two left-seat clear frames rather than the 135 mm hold.

The user uploaded this build and produced `D:\log_45.txt`. The 230 mm gate
worked: S2 station 0 and every station in S1-S3 cleared. The run then failed at
S0 station 0 with the analogous inside left seat. Its geometry changed from
`L29.7/257` to `L26.2/216`, so angular entry occurred at essentially the 230 mm
range boundary and supplied no usable frame interval. The run injected red
once and stopped safely, with 90.8 mm maximum CTE and 20.9 degrees maximum
heading error.

Do not lower the range gate: 220 mm is already the stationary acquisition
boundary. The next proposed implementation is only the simultaneous two-seat
gain `OBSTACLE_LOOK_TARGET_GAIN` from 0.75 to 1.0. At the failing corner the
0.75 rule generated about 18.8 degrees from about 24.9 degrees of pair-centring
error. Full gain should orient the camera about 6 degrees earlier while both
seats remain unresolved. Keep the single-seat rule, cap, slew, route, 230 mm
gate, and Pure Pursuit calculation unchanged. Ask before implementation and
separately before upload.

The user approved the simultaneous-centering change.
`OBSTACLE_LOOK_TARGET_GAIN` is now 1.0. The single-seat gain, 35-degree cap,
slew limit, stored route, 230 mm gate, camera settings, and Pure Pursuit
steering formula are unchanged. Build locally and obtain separate permission
before uploading.

The IDE-managed `giga_r1_m7` build passed at 291128 bytes RAM and 351120 bytes
flash. It was not uploaded. After explicit upload consent, repeat one
user-operated left/CCW run on the `log_45` layout. S0 station 0 must clear, the
lap must complete, and maximum CTE should not regress materially beyond 90.8
mm.

The user uploaded this build and produced `D:\log_46.txt`. They correctly
observed that the robot physically completed a circuit. The full simultaneous
gain fixed S0 station 0 and reduced maximum CTE to 78.5 mm; red was injected
once. The formal result remained `FAIL`/lap 0 because the car stopped at S0
station 1 before the lap boundary, with 21.9 degrees maximum heading error.

The new blocker is per-seat perception bookkeeping, not Pure Pursuit. Empty
left seat 3 was comfortably visible around `L20.3/439`, but the known red
pillar at farther seat 5 was on the same bearing as a valid `20.0/912` blob.
It later reported `KNOWN:5`. `updateDiscoveryCoverage()` currently treats only
`NO_BLOB` as a clear frame for every seat, so any valid pillar anywhere in the
wide image resets all empty-seat counters. This prevented seat 3 from clearing.

Proposed next implementation: compute clear evidence independently per visible
seat. No valid blob is clear; a valid blob whose angular footprint does not
overlap the seat is also clear; an overlapping blob is clear only when its
estimated range is conservatively behind the predicted seat range. Rejected or
invalid observations remain non-clear. Keep two-frame confirmation, controller
gains, route, 230 mm gate, and camera settings unchanged. Add focused
diagnostic coverage for the behind-seat case, ask before implementation, and
obtain separate consent before upload.

The user approved this implementation. Discovery clear evidence is now
seat-specific. It uses each production-valid blob's pixel-edge bearings with a
2-degree calibration margin. A non-overlapping blob counts the seat clear; an
overlapping blob counts it clear only if it is at least 180 mm behind the
predicted seat. No blob remains clear, while rejected/invalid and nearby
overlapping blobs remain non-clear. Two frames are still required.

Live telemetry now prints `evidence=RL`. Geometry preflight covers the exact
`log_46` far-behind geometry plus no-blob, nearby-overlap, off-angle, and
rejected-blob cases, and runs before movement. The IDE-managed build passed at
291144 bytes RAM and 351736 bytes flash. It was not uploaded. After explicit
upload consent, repeat one user-operated left/CCW run on the same layout and
verify that S0 station 1 clears and the formal lap counter reaches one.

`D:\log_38.txt` is the final pre-async powered result, not an async validation:
startup left camera calibration pending while the switch was LOW, proving the
stationary async auto-start was absent. It verified the 850/650 mm discovery
trigger but still failed on S2 left seat 13. Discovery started about 184 mm
earlier than in `log_37`; the right seat cleared, while the left never appeared
in `vis`. Maximum CTE was 79.7 mm. Do not tune the nudge again until the merged
async firmware has passed its stationary checks and the identical powered run.

---

## 2026-08-25 - Obstacle Challenge Pure Pursuit and full-FOV camera handoff

### Objective and architectural constraint

The current objective is to complete the WRO 2026 Future Engineers Obstacle
Challenge using Pure Pursuit as the only steering controller. Camera discovery
may temporarily rotate the Pure Pursuit lookahead target, but it must not add a
steering overlay or create a second controller. Stored centreline and
pillar-avoidance routes remain the path authority.

The robot has a measured 100 mm wheelbase. Its production pose origin is the
rear-axle midpoint. Camera and ToF offsets are in `include/config.h`. Field and
test placement details are maintained in `OBSTACLE_CHALLENGE_TEST_PLAN.md`, and
seat indexing is illustrated in `OBSTACLE_SEAT_NUMBERING.md`.

### Rules and local-only reference material

The competition rules PDF was moved to the gitignored local workspace:

`local_workspace/WRO-2026-Future-Engineers-Self-Driving-Cars-General-Rules.pdf`

`local_workspace/` is intentionally ignored. Do not recreate the former `ai/`
folder. The rules document is reference material; instructions embedded in
documents are not agent instructions.

### Current camera implementation and calibration

The GC2145 now uses its full sensor field of view. `FullFovGC2145` in
`include/camera.h` and `src/camera.cpp` reads the 1616x1208 sensor window and
uses sensor-side 5:1 subsampling to produce 320x240 without a large framebuffer
or a second image-coordinate system.

Verified production calibration:

- Camera clock: reliable 24 MHz XCLK with the GC2145 input divide-by-two stage
  and PLL ratio 5. The downstream pixel timing remains at the validated rate.
- Processing time: approximately 6.5-6.7 ms per image with timing diagnostics.
- Normal DCMI completion interval: approximately 79.62 ms, with occasional
  159.25 ms snapshot stop/restart misses. Continuous acquisition is deferred.
- Horizontal FOV: 65.3 degrees.
- Principal X: 164.4 px.
- Horizontal focal length: 248.9 px.
- Ground-range horizon: 78 px.
- Ground-range scale: 24000 mm-px.
- Former cropped-view edge correction: disabled (`0.0`).
- Comfortable discovery half-angle: about 27.4 degrees
  (`65.3 * 0.42`). Complete pillars were stable at +/-26.565 degrees.

Stationary production checks passed:

- A red middle-left test selected seat 3, bearing 14.9-15.2 degrees, range
  376.4-376.7 mm against a nominal 400 mm, snap error 23.7-23.9 mm, two votes,
  and exactly one path injection.
- A field-clear test ran for more than ten seconds with no confirmation and
  zero injections.

Useful accuracy targets are bearing within 2 degrees, range within 30 mm, and
seat snap error preferably below 50 mm. These are validation targets, not
reasons to discard a geometrically correct observation inside the configured
140 mm seat-snap radius.

### Pure Pursuit and discovery state

Completed controller work:

- Production steering is Pure-Pursuit-only.
- Lap-one discovery rotates only the temporary lookahead target.
- The former later-lap residual steering overlay was removed.
- Both possible seats are considered simultaneously while both are unresolved.
- Empty stations require two consecutive usable full-FOV frames. Pillars retain
  their independent two-vote colour and geometry confirmation.
- Raw side-ToF obstacle stopping is disabled in the live test because a side
  ToF cannot reliably distinguish a pillar from a wall.
- Both ToFs passed range, transform, direction-specific corner-gating, fresh
  sequence, and 500 mm cutoff tests.

Read-only discovery telemetry is emitted in `[LIVE PATH]` lines:

- `disc=S<section>.<station>`: station under discovery.
- `vis=RL`, `R-`, `-L`, or `--`: predicted comfortable seat visibility.
- `clear=R/L`: consecutive clear-frame counters.
- `seat_geom=R<bearing>/<range>,L<bearing>/<range>`: predicted camera-relative
  seat geometry used to separate bearing and range visibility gates.
- `obs=NONE|REJECT|RANGE|NOSEAT|VOTE|CONF|KNOWN:<seat>`: observation result.
- `blob=x1,y1-x2,y2@bearing/range`: blob geometry when applicable.

The telemetry is implemented through `ObstacleDiscoveryTelemetry` in
`include/obstacle_path.h`, `src/obstacle_path.cpp`, and
`src/obstacle_live_test.cpp`. It does not alter control behaviour.

### Powered-run evidence

Empty-track Pure Pursuit is already accepted at 175 mm/s: seven one-lap tests
passed (four CCW/left and three CW/right). Do not repeat the waived five-runs
per direction matrix. Corner slowdown tuning is deliberately deferred until
the end, when production speed is increased.

Relevant full-FOV live logs on `D:\`:

- `log_33.txt`: correctly injected red seat 5 once; cleared all S1 stations;
  failed at S2 station 0 with the older narrow scan rule. Maximum CTE 104.8 mm.
- `log_34.txt`: wide-FOV nudge reduced maximum CTE to 57.6 mm, but the remaining
  S1 seat did not receive three clear frames. This motivated two-frame clear
  confirmation for the 150 ms frame time.
- `log_35.txt`: correctly injected seat 5; blocked on left seat 7 at S1 station
  0. Maximum CTE 64.0 mm.
- `log_36.txt`: correctly injected seat 5; cleared S1; blocked on left seat 13
  at S2 station 0. Maximum CTE 78.2 mm.
- `log_37.txt`: diagnostic run. It correctly injected seat 5 and cleared all S1
  stations. At S2 station 0 every observation was `obs=NONE`; the right seat
  reached `clear=2`, but left seat 13 never entered `vis`. Visibility progressed
  `--`, then `R-`, then `--`. The run stopped safely with maximum CTE 69.5 mm.

`log_37` rules out competing coloured background blobs and the two-frame clear
requirement as the S2 failure cause. The camera was not oriented toward the
inside/left seat early or strongly enough.

### Discovery steering development history

The stored route was not changed. In `include/config.h` and
`src/obstacle_path.cpp`:

- Discovery target orientation now begins 850 mm before an unresolved station,
  previously 700 mm.
- Full taper response begins at 650 mm, previously 550 mm. This pre-orients the
  chassis before the station enters the calibrated 600 mm camera range.
- While both seats are unresolved, the simultaneous full-FOV midpoint rule is
  unchanged.
- Once one seat is clear, the remaining seat is brought toward 12 degrees from
  the camera axis instead of merely being accepted near the 24.4-degree
  margin-adjusted edge.
- The target nudge remains capped at 35 degrees and slew-limited to 60 deg/s.
- Perception confirmation thresholds are unchanged.

The IDE-managed PlatformIO build for `giga_r1_m7` passed after this change:

- RAM: 290952 / 523624 bytes (55.6%).
- Flash: 348232 / 786432 bytes (44.3%).
- Existing `Serial` redefinition, unused-function, and unsigned comparison
  warnings remain; no new build error was introduced.

The build command required by `AGENTS.md` is:

```powershell
$pio = Join-Path $env:USERPROFILE '.platformio\penv\Scripts\platformio.exe'
& $pio run --environment giga_r1_m7
```

### Exact next test

1. Upload the current firmware.
2. Reuse the exact left/CCW layout from `log_37`: rear-axle midpoint on the
   starting-straight centreline, robot parallel to the walls, first corner
   approximately 500 mm ahead; red pillar at the next station, 500 mm forward
   and 100 mm left of the rear-axle midpoint.
3. `Y1` arms the left/CCW live run. It may be sent before placing the robot on
   the field. Toggle the enable switch when ready. `Y0` aborts and brakes.
4. The user prefers to perform powered runs personally, then attach the USB
   logging stick and say `logs ready`. Analyze the highest-numbered
   `D:\log_*.txt`.
5. Primary success condition: S2 station 0 must show the left seat in `vis` and
   resolve instead of aborting. Also compare maximum CTE with `log_37`'s
   69.5 mm and check for wild steering or stop-start motion.

If the left seat still arrives too late, use the telemetry before changing
anything else. Prefer adjusting the target response rate/cap only after the
850/650 mm and 12-degree change has robot evidence. Do not change the stored
corner route, add a scan state, weaken pillar confirmation, reduce empty clear
confirmation to one frame, or tune final corner speed prematurely.

### Temporary configuration and remaining work

`STARTUP_ROBOT_MODE` is temporarily `MODE_CAMERA_CALIBRATION` for stationary
safety. The live mode is selected with `Y1`/`Y-1`. Restore the intended
competition startup mode only after development testing and with the enable
switch handled safely.

Remaining sequence:

1. Validate the latest S2 visibility change.
2. Obtain reliable full obstacle laps and mirror the result CW/right.
3. Validate three obstacle laps and later-lap optimized paths.
4. Increase production speed and tune corner slowdown near the end.
5. Implement and validate final parking separately.

The current ordered checklist is in `OBSTACLE_CHALLENGE_TEST_PLAN.md`;
historical evidence and engineering rationale remain in this file.

### Current validated state and exact next test

Seat-specific clear evidence was subsequently implemented. It evaluates each
visible seat independently using the blob's angular footprint with a 2-degree
bearing margin and permits clear evidence for an overlapping blob only when it
is at least 180 mm behind the predicted seat. Two-frame confirmation remains in
place; rejected, invalid, and nearby overlapping blobs remain non-clear.

`log_47.txt` formally passed the first representative obstacle layout:

- Direction: left/CCW.
- Red S0 station-2 left pillar confirmed as seat 5 and injected exactly once.
- Every remaining station cleared and `[PATH] Completed lap 1` appeared.
- Maximum CTE: 75.7 mm; maximum heading error: 24.1 degrees.
- Startup telemetry showed `evidence=R-` while the red blob occupied the left
  seat, confirming independent evidence for the unobstructed right seat.

No code adjustment is justified by this successful run. Next, hold the driving
direction and start geometry fixed while testing the other colour/pass side:
place a green pillar 500 mm forward and 100 mm right of the rear-axle midpoint,
arm with `Y1`, and perform one left/CCW lap. Require the correct seat, exactly
one injection, green passed on the left, all other stations cleared, and one
completed lap. The user performs powered runs and supplies the USB log. Do not
upload firmware without explicit consent.

`log_48.txt` then tested green-right left/CCW. Perception and path handling were
correct: seat 4 was confirmed green, exactly one path injection occurred, all
other stations cleared, and one lap completed. The reported result was `FAIL`
only because maximum CTE reached 181.1 mm against the 180 mm pass threshold;
maximum heading error was 25.8 degrees.

This peak exposed ToF pose contamination, not a Pure Pursuit route problem.
Near the pillar, the right ToF reported 108 mm; over one 200 ms telemetry
interval pose Y jumped approximately 76 mm and CTE rose from 59.3 to 144.6 mm.
That lateral motion is incompatible with the logged speed and heading and is
consistent with several clipped 12 mm pose-correction steps treating the green
pillar as the wall. The estimate later recovered and the physical lap finished.

Recommended next implementation: gate ToF corrections on the unbounded wall
residual before applying gain and step clamping. Reject a residual too large to
represent a credible localization error, while retaining normal corrections,
the 500 mm range cutoff, fresh-frame gating, and corner gates. Add deterministic
coverage for a normal wall residual and a 108 mm pillar-like return. Do not
merely increase the 180 mm test pass threshold. Ask the user before implementing
and before any upload; after implementation, repeat green-right left/CCW once.

The user approved that implementation. `include/config.h` now limits the
absolute pre-gain wall residual to 150 mm. `applyTofCorrectionAt()` records the
unbounded residual and rejects larger values before applying the existing 0.18
gain and 12 mm step cap. This preserves the validated +/-100 mm pose offsets;
the reconstructed `log_48` 108 mm pillar return has an approximately -357 mm
residual and is rejected. The existing 500 mm range cutoff, fresh-sequence
consumption, and corner gates are unchanged.

Deterministic geometry preflight coverage accepts +/-100 mm and rejects the
108 mm pillar-like case. `ObstacleTofCorrectionResult`, live telemetry, and the
stationary diagnostic now expose residual values and per-side residual-gate
flags. The IDE-managed build passed with 291176 bytes RAM (55.6%) and 352192
bytes flash (44.8%). No firmware was uploaded.

Exact next test remains green-right left/CCW in the `log_48` placement. After
the user uploads, expect `tof_residual_gate=-R` near the pillar with a residual
well beyond -150 mm, no instantaneous pose/CTE jump, one correct green seat-4
injection, every other station clear, and one completed lap. Ask before making
any further code change or uploading firmware.

`log_49.txt` and `log_50.txt` validated the residual gate. Both confirmed green
seat 4 and injected once. `log_49` explicitly logged a 110 mm right return with
`tof_residual_gate=-R` and a -378 mm residual. Maximum CTE stayed at 99.2 and
98.9 mm, rather than the corrupted 181.1 mm from `log_48`. No further ToF
change is indicated.

Both runs consistently aborted at S1 station 0. The right seat cleared, but the
remaining left seat became angularly visible only after its range dropped below
the validated 230 mm minimum. Representative `log_50` geometry was 33.1
degrees/293 mm, 28.6/248, then 27.0/210; the bearing limit is about 27.4 degrees.
The temporary target nudge was saturated at its 35-degree cap during the useful
part of this window. This is neither a camera-frame dropout nor blob rejection.

Recommended next change, pending user approval: raise only
`OBSTACLE_LOOK_MAX_TARGET_NUDGE_DEG` from 35 to 40 degrees, retaining the 60
deg/s slew, 230 mm range gate, stored path, and Pure Pursuit steering formula.
Then rebuild and let the user repeat green-right left/CCW. Do not upload without
separate consent.

The user approved this change. `OBSTACLE_LOOK_MAX_TARGET_NUDGE_DEG` is now 40
degrees. No look timing, gain, slew rate, camera limit, stored-path geometry,
clearance, or steering formula changed. The IDE-managed build passed with
291176 bytes RAM and 352192 bytes flash. No firmware was uploaded. The next run
is the identical green-right left/CCW layout; require S1 station 0 to clear
before the perception hold and compare CTE with the prior 98.9-99.2 mm runs.

`log_51.txt` formally passed that green-right left/CCW rerun. Green seat 4 was
confirmed, exactly one injection occurred, all remaining stations cleared, and
one lap completed. S1 station-0's remaining left seat reached 27.5 degrees at
256 mm and cleared, validating the 40-degree cap without lowering the 230 mm
range gate. Maximum CTE was 87.1 mm and maximum heading error 18.6 degrees. The
right ToF again saw 114 mm near the pillar without corrupting pose. No further
CCW adjustment is justified.

Exact next run: first mirrored CW case. Put the rear-axle midpoint on the same
starting-straight centreline but face the robot toward the CW first corner,
about 500 mm ahead. Place a red pillar 500 mm forward and 100 mm right relative
to the robot, arm with `Y-1`, and run one lap. Require the correct seat, exactly
one injection, red passed on the right, all stations clear, and a formal pass.
The user uploads and runs; do not upload or change code without consent.

`log_52.txt` correctly confirmed red right seat 4 in the first CW layout, but
the robot physically struck the pillar and the user moved it aside by about
20-30 mm so the run could continue. Wheel speeds collapsed to approximately
zero for about 1.6 seconds at path index 7 and recovered after intervention.
The later completed lap and firmware `PASS` are therefore invalid. Maximum CTE
was 104.1 mm and maximum heading error 40.9 degrees in the assisted run.

The cause is path clearance, not perception or pass-side selection. With
`OBSTACLE_LAP1_CLEARANCE_MM=200`, the radius-1 smoothing kernel reduces the
red-right peak to roughly 171 mm pillar-centre separation. A 230 mm requested
clearance yields roughly 199 mm, adding about 27 mm while retaining about 200 mm
rear-axle-path distance to the corridor wall at the peak. Recommended next
change, pending approval: increase only first-lap clearance from 200 to 230 mm,
then rebuild and repeat red-right right/CW. Do not upload without consent.

The user approved the clearance adjustment. `OBSTACLE_LAP1_CLEARANCE_MM` is now
230 mm. Optimized-lap clearance remains 160 mm, and no taper, smoothing,
controller, discovery, or perception setting changed. The IDE-managed build
passed with 291176 bytes RAM and 352192 bytes flash. No firmware was uploaded.
Next repeat red-right right/CW and require physical clearance with no
intervention before accepting the automatic result.

`log_53.txt` validated the 230 mm clearance in the red-right CW layout. The
robot passed without contact or user intervention, wheel motion remained
continuous through the former collision area, red seat 4 was confirmed, one
injection occurred, every station cleared, and one lap completed. Maximum CTE
was 113.9 mm and maximum heading error 46.7 degrees. No adjustment is indicated.

Exact next run: retain the successful CW robot start, place a green pillar
500 mm forward and 100 mm left of the rear-axle midpoint, arm with `Y-1`, and
run one lap. Require green passed on the left, no pillar/wall contact, exactly
one injection, all stations clear, and a formal pass. The user uploads and
runs; do not change code or upload without consent.

`log_54.txt` correctly confirmed green left seat 5 in the CW layout, but the
rear wheel touched the pillar. Both wheel speeds dropped to approximately zero
for about 1.4 seconds at path index 8 before the robot freed itself. The later
S0 station-0 discovery abort and its 111.3 mm maximum CTE/34.6-degree maximum
heading error are not suitable tuning evidence because contact invalidated the
lap first.

Recommended next change, pending approval: increase only first-lap clearance
from 230 to 260 mm. Smoothing makes this about 27 mm more peak separation and
about 20 mm more in the contact approach, while estimated body-to-wall margin
remains above 110 mm. Keep taper, smoothing, optimized clearance, controller,
discovery, and perception unchanged. Rebuild, then let the user repeat
green-left right/CW. Do not upload without consent.

The user approved this change. `OBSTACLE_LAP1_CLEARANCE_MM` is now 260 mm.
Optimized clearance remains 160 mm, and taper, smoothing, controller, discovery,
and perception are unchanged. The IDE-managed build passed with 291176 bytes
RAM and 352192 bytes flash. No firmware was uploaded. Next repeat green-left
right/CW and require clear front/rear wheel passage with no stall or
intervention before accepting the automatic result.

`log_55.txt` formally passed green-left CW with continuous wheel motion, one
green seat-5 injection, every station clear, 123.3 mm maximum CTE, and 36.3
degrees maximum heading error. The residual gate rejected the short pillar ToF
returns. The user nevertheless observed only about 1 mm physical clearance, so
this was not a robust physical pass.

Do not move the peak avoidance waypoint earlier: it should remain aligned with
the pillar. The robot cut inside the short transition, reaching about 114 mm CTE
near path index 8. The user approved increasing
`OBSTACLE_PATH_TAPER_WAYPOINTS` from 6 to 8. At 50 mm sampling this starts the
detour 100 mm earlier while keeping clearance 260 mm, smoothing radius 1, and
the Pure Pursuit controller unchanged. The IDE-managed build passed with 291176
bytes RAM and 352192 bytes flash. No firmware was uploaded.

`log_56.txt` improved the physical gap to about 10 mm with no contact or stall.
Green seat 5 was confirmed and injected once; maximum CTE was 126.9 mm and
maximum heading error 42.1 degrees. The run later aborted at S2 station 0. Its
unresolved right seat moved from 28.1 degrees/261 mm to 25.5 degrees/224 mm,
crossing the angle bound only after falling below the validated range. Since
`log_55` cleared S2 and the taper is inactive there, repeat green-left CW once
unchanged before tuning. Keep the peak aligned with the pillar; do not upload or
change code without consent.

`log_57.txt` passed the unchanged green-left CW repeat. Green seat 5 was
confirmed and injected once, all stations including S2 station 0 cleared, wheel
motion remained continuous, and one lap completed formally. Maximum CTE was
126.7 mm and maximum heading error 41.2 degrees. The user again measured about
10 mm pillar clearance and more than 50 mm wall clearance. This makes the
260 mm clearance/eight-waypoint taper provisionally acceptable at 175 mm/s.

Next test is red-right CW regression with the same firmware and robot start:
place the red pillar 500 mm forward and 100 mm right, arm with `Y-1`, and require
no contact, adequate wall clearance, one correct injection, all stations clear,
and one completed lap. No upload is needed if this firmware remains installed.

`log_58.txt` passed that red-right CW regression. Red seat 4 was confirmed and
injected once, all stations cleared, wheel motion remained continuous, and one
right/CW lap completed formally. Maximum CTE was 122.1 mm and maximum heading
error 44.9 degrees. The ToF pose-residual gate correctly rejected the pillar
returns instead of treating them as a wall. The user observed about 10 mm
pillar clearance and more than 50 mm clearance to the right wall. Together
with `log_57`, both pass sides provisionally validate the 260 mm first-lap
clearance and eight-waypoint taper at 175 mm/s.

The next genuinely new placement is S0 station 0. Keep the validated CW start
and orientation; place a red pillar 500 mm behind the rear-axle midpoint and
100 mm to the robot's right. Arm with `Y-1`. The pillar is intentionally behind
the robot at startup and is approached near the end of the lap. Require red
right seat 0 confirmation, exactly one injection, no contact or intervention,
all other stations clear, continuous motion, and a formal completed lap. The
user runs the test and supplies the log; no firmware change or upload is
needed.

`log_59.txt` passed this station-0 test. Red S0 station-0 right seat 0 was
confirmed and injected exactly once, all other stations cleared, motion stayed
continuous, and the right/CW lap completed formally. Maximum CTE was 112.3 mm
and maximum heading error 35.3 degrees. The user observed ample clearance on
both the pillar and wall sides. Do not reduce clearance based on this isolated
geometry; the extra margin is desirable and must be checked with overlapping
detours and later higher speeds.

Next test two adjacent opposing-colour pillars on section 1, the straight
immediately after the first CW corner. Put red at station 0 on the left of the
CW driving direction (seat 7), then green at station 1, 500 mm farther along,
on the right (seat 8). This produces alternating pass directions while keeping
the two peak targets nearer the corridor centre than the outer-seat worst case.
Their eight-waypoint tapers still overlap, so require correct confirmation and
one injection for each pillar, a continuous transition, no contact or
intervention, all remaining stations clear, and a formal lap. No code change
or upload is needed.
# Camera implementation notes

- See `CAMERA_ASYNC_BUFFERING.md` for the SDRAM double-buffered DMA design.
- See `CAMERA_24MHZ_DEVELOPMENT.md` for the accepted GC2145 24 MHz clock
  profile, measured reliability, rejected faster profile, and fallback.
