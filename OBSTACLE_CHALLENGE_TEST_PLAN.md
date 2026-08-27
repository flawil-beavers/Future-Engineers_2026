# Obstacle Challenge checklist

This is the ordered checklist for current work. Complete the unchecked items
from top to bottom. Detailed history and decisions are in
`AGENT_DOCUMENTATION.md`; seat numbering and clearance telemetry are described
in `OBSTACLE_SEAT_NUMBERING.md` and `OBSTACLE_CLEARANCE_LOGGING.md`.

For a normal successful run, record only the log number, result, and next step
in two or three sentences. Add detail only for a failure, material code change,
new safety limit, or reusable engineering finding.

## Safety for every powered run

- [ ] Charge and secure the drive battery.
- [ ] Keep the disable switch reachable and cables clear of the robot.
- [ ] Clear people and fragile objects from the field.
- [ ] Place the robot accurately at the validated start pose.
- [ ] Record direction, layout, physical contact/clearance, and USB log number.
- [ ] Treat physical contact or intervention as failure even if firmware says
      `PASS`.
- [ ] Upload firmware only with explicit user consent.

Commands: `Y1` starts left/CCW, `Y-1` starts right/CW, and `Y0` aborts and
brakes. Arm with the physical enable switch LOW and toggle it HIGH only when
the field is ready.

## Completed foundations

- [x] Calibrate the camera distance model and validate the full camera FOV.
- [x] Validate both side ToF sensors and pose-aware ToF diagnostics.
- [x] Make Pure Pursuit the sole path-steering controller.
- [x] Remove point-by-point stopping and obtain continuous path following.
- [x] Implement live pillar discovery, seat assignment, two-frame voting, and
      bounded unresolved-seat holds.
- [x] Implement per-pillar planned, odometry, ToF, wall, and inner-corner
      clearance logging.
- [x] Validate isolated red and green passes on both sides at 175 mm/s.
- [x] Validate near, middle, and far stations in representative layouts.
- [x] Validate the moderate adjacent opposing-colour layout in both directions.
- [x] Protect the rare extreme adjacent reversal by deferring the second path
      until the first pillar is clear.
- [x] Validate lap-1 discovery paths and optimized lap-2/3 paths.
- [x] Complete representative three-pillar, three-lap runs in CW and CCW.
- [x] Validate controlled stopping after exactly three laps.
- [x] Fix the coincident lap-seam curvature defect; `log_110` confirmed
      `target=175..175` with no former post-start near-stop.
- [x] Selectively integrate the accepted loop/camera optimizations.
- [x] Build the integrated firmware: 361128 bytes RAM and 364568 bytes flash.

## Next: validate the integrated loop/camera firmware

- [x] Upload the already-built firmware after obtaining user consent.
- [x] Keep the robot stationary and unable to drive during the camera checks.
- [x] Confirm an official red pillar in the image centre with stable repeated
      production-valid detections and plausible distance/bearing.
- [x] Confirm an official green pillar in the image centre with the same
      criteria.
- [x] Repeat red at the validated left and right edge views.
- [x] Repeat green at the validated left and right edge views.
- [x] Stop and diagnose any colour error, unstable detection, or edge-only miss
      before a powered lap.
- [x] Run one previously safe 175 mm/s obstacle lap.
- [x] Require correct seats/colours and exactly one injection per pillar.
- [x] Require fresh ToF passage reports, no contact/intervention, acceptable
      CTE/heading error, and formal lap completion.
- [x] Compare timing and detection reliability with the accepted pre-port run;
      proceed only if no regression appears.

## Next: repair the driving-stall safety watchdog

- [x] Do not run another powered route test until the stall watchdog is fixed.
- [x] Preserve the existing fast maximum-duty overload response, but add a
      sustained commanded-motion/no-progress timeout that does not require
      99% motor duty.
- [x] Ensure planned stops and perception holds, where target speed is zero,
      cannot trigger the driving-stall watchdog.
- [x] Log the target/profile speed, duty, elapsed time, and measured progress
      when the watchdog pauses the mode.
- [x] Add deterministic logic coverage and build with the IDE-managed
      PlatformIO installation.
- [x] Because the drivetrain cannot be disconnected safely, do not induce a
      physical stall. Accept the deterministic trigger preflight plus the
      lifted free-wheel no-false-trigger check; never use a hard field wall.
- [x] Continue with remaining work after the watchdog preflight and lifted test
      pass; retain the first incidental real stall as trigger confirmation.

## Unparking, localization, and parking-marker robustness

- [x] Limit the current implementation to starts completely inside the
      parking lot, with the front axle pointing in the official driving
      direction. Middle-zone starts are intentionally deferred as an optional
      end-stage bonus.
- [x] Prepare an isolated parking-exit build that stops, locks the drive motor
      off, and saves start/end ToF, travel, and heading diagnostics instead of
      joining the obstacle lap. An unset final length is labelled
      `prototype_only` in the log.
- [x] Characterize the existing exit in the proportional prototype gap. It was
      rejected after the front-right wheel contacted the forward magenta
      marker after only 34 mm of its first forward arc.
- [x] Replace the forward-first two-arc exit with a length-aware multi-point
      manoeuvre for the official `1.5 * robot length` gap. Check the complete
      swept robot envelope against both magenta blocks and the outer wall
      offline before the next powered test; do not assume the larger gap in
      which the old manoeuvre worked.
- [x] Confirm the current prototype envelope inputs: 165 mm length, 125 mm
      front and 40 mm rear overhang from the rear axle, conservative symmetric
      135 mm full-steering width, and approximately +/-5 mm placement error.
- [ ] Finalize the robot length before fixing parking-lot geometry. The
      competition parking-space length is `1.5 * robot length`, so tests and
      route constants must not assume a final marker separation yet.
- [ ] Define and document robust initial positions and tolerances within the
      parking lot for both CW and CCW driving directions after the robot length
      is fixed.
- [x] Upload the staged multi-point diagnostic only with consent. Place the
      current robot with 45 mm rear clearance, its inner edge at the 200 mm
      parking opening, and a 247.5 mm inside-face marker gap. Run only segment
      1: reverse 20 mm at full steering toward the wall, then stop and save.
- [x] Accept segment 1 only if it has no contact, the observed clearance stays
      positive, and logged actual travel is close enough to 20 mm to preserve
      the modeled tolerance. `log_117` measured 22.3 mm and 10.4 degrees with
      no observed contact.
- [x] Increase the staged segment limit incrementally and validate the complete
      seven-segment path. The user skipped some individual one-segment
      increments, but `log_120` completed all seven without contact or
      intervention.
- [x] Replace former segments 5-7 with one continuous full-lock alignment arc
      after segment 4. The revised model keeps the parking pieces at their exact
      200 mm length, retains 5 mm on their broad faces and the outer wall, and
      passes all 16 gap/placement/heading tolerance combinations.
- [x] Build the five-segment isolated firmware with the IDE-managed PlatformIO
      installation; no firmware was uploaded.
- [x] Test the five-segment isolated exit. Segment 5 may stop after 120 mm once
      gyro heading error is within 2 degrees and is bounded at 180 mm. Require
      `aligned=yes`, no contact/intervention, and positive clearance. `log_122`
      passed after 162.6 mm with 0.9 degrees final error.
- [x] Move the nominal start 5 mm forward: use 50 mm from the rearmost robot
      point to the rear marker and 32.5 mm nominal front clearance. The updated
      model still passes all 16 tolerance combinations.
- [x] Add diagnostic-only post-exit logging for the outer-wall-side ToF when it
      faces the exact 200 mm end of a magenta piece. Log filtered/raw range,
      signal, sigma, inferred rear-axle distance beyond the parking end, and
      remaining distance to the field centreline; do not apply a pose reset yet.
- [x] Build the revised firmware without uploading.
- [x] Fix the post-alignment drift exposed by `log_123`: centre the steering
      immediately at the gyro trigger before engaging the encoder-position
      hold. The ToF sample itself was strong, but 2.3 degrees stopped heading
      made the diagnostic correctly report `usable=no`.
- [x] Build the steering-centering fix without uploading.
- [x] Repeat the complete isolated exit in the same direction from the new
      50/32.5 mm placement. Require no contact, final error within 2 degrees,
      and a usable `[PARK EXIT TOF REF]` result consistent with the observed
      magenta-piece end. `log_125` passed with 0.4 degrees stopped error and a
      63 mm right-ToF reference estimating 98.3 mm beyond the parking end.
- [x] Mirror the complete isolated exit in the opposite official direction
      from the same proportional gap and placement tolerances. Require no
      contact and final heading error within 2 degrees before connecting either
      exit to field localization. `log_126` passed with 0.9 degrees final error
      and a usable 44 mm left-ToF parking-end reference.
- [ ] Repeat and accept the isolated exit with the finalized robot length and
      official marker separation before allowing it to join the lap route.
- [ ] Implement and physically validate the safe multi-point unpark path at
      low speed without touching either magenta piece or a field wall.
- [ ] Establish the robot's field pose after unparking rather than assuming the
      rear axle reached one exact position.
- [x] Replace the incorrect midpoint assumption with Rules Figure 4 geometry:
      the right magenta piece is fixed immediately left of the right dotted
      section boundary and the left piece moves according to robot length.
- [x] Initialize the canonical parked pose from that fixed boundary, integrate
      encoder/gyro odometry through the exit, and apply only the validated ToF
      correction normal to the exact 200 mm parking-piece end.
- [x] Gate that ToF correction by the odometry-derived sensor-beam position:
      accept it only over the expected 20 mm parking-limit piece, with 5 mm
      placement tolerance, so an unrelated short return cannot move the pose.
- [ ] Run the unchanged isolated exit once in each direction and verify
      `[PARK FIELD START]`, `[PARK EXIT TOF REF] beam_over_piece=yes`
      plus `usable=yes apply_y=yes`, and
      `[PARK EXIT FIELD POSE]` against the physical placement before building a
      Pure Pursuit connector to the normal field route. `log_127` and
      `log_128` completed and aligned without physical contact, but both
      correctly left
      `apply_y=no`: the current gate tests only the centre ray, whereas the
      measured returns came from the finite ToF field of view.
- [x] Replace the centre-ray check with the documented 22-degree near-range
      ToF detection footprint and require that footprint to intersect only the
      expected piece. Replaying logs 127/128 predicts intersections in both
      directions; the firmware builds successfully without an upload.
- [x] Repeat the unchanged isolated exit once in each direction. Require no
      contact, `beam_over_piece=yes usable=yes apply_y=yes`, and plausible
      corrected field poses before implementing the Pure Pursuit connector.
      `log_129` (CCW) and `log_130` (CW) passed all firmware criteria, and the
      user confirmed no physical contact in either run.
- [ ] Remove the exact parked-pose assumption. At minimum initialize lateral
      `y` from the near outer-wall ToF. Select and validate one longitudinal
      strategy: a visually indicated safe start band plus post-exit magenta-edge
      localization (preferred), close-range camera localization, or added
      front/rear ranging hardware.
- [x] Implement the ruler-assisted version of the preferred strategy: retain
      the validated 50 mm rear placement, initialize start `y` from the near
      wall ToF, and add a test-only 60 mm/s straight edge search after the
      five-segment exit. Reject x/y corrections larger than 25 mm.
- [x] Build the isolated edge-localization firmware without uploading.
- [ ] Test the new post-exit creep in one direction only. Require no contact,
      `transition=yes`, both
      corrections applied within 25 mm, a plausible final pose, and the usual
      test-only motor lock. Logs 131/132 exposed premature acceptance of
      195/210 mm edge returns; repeat after the corrected wall-consistency gate.
- [x] Preserve only genuinely short magenta samples, ignore intermediate
      oblique returns, require two wall readings within 25 mm of the
      pose-predicted wall distance, and increase the hard creep bound to 60 mm
      so the full ToF cone can clear the magenta piece. Build without upload.
- [x] Use the newest sample in the confirmed wall-frame sequence for the y
      correction. `log_133` showed that using the first qualifying 239 mm
      transition sample discarded a later stable 259 mm wall return.
- [x] Replace the forward edge search with the user's proposed bounded straight
      reverse. Reference the opposite edge of the same magenta piece and keep
      the lap lockout enabled. The swept model passes all 16 existing
      gap/placement/heading tolerance cases through the full 60 mm reverse.
- [ ] Physically validate the reverse edge search in CCW and CW with no
      pillars. Require no contact, `direction=reverse`, `transition=yes`, a
      pose-consistent wall sequence, bounded x/y corrections, and the test-only
      motor lock before implementing the starting-section discovery connector.
- [ ] Before implementing that join, handle the legal signs in the parking
      section: the rules move them to the seats nearer the inner wall rather
      than removing them. Resolve the first relevant inner seat before
      committing to its red/green passing side.
- [ ] Compare two low-speed discovery connectors in the swept-envelope model:
      a cautious outer-side turn that brings the first inner seat into view,
      and a straight reverse that retains the forward camera view. Do not
      physically reverse after the exit until the complete robot envelope has
      positive clearance from both open magenta ends. An outer-wall ToF range
      alone does not prove this clearance.
- [ ] If using the current five-segment path, keep initial longitudinal
      placement inside its modeled safe band. The current model passes all 16
      tolerance cases at nominal 45 and 50 mm rear clearance, but not across
      the complete possible parking-gap range.
- [ ] Quantify post-unpark position and heading error for every supported start.
- [ ] Require localization accuracy sufficient to join the normal Pure Pursuit
      lap without an abrupt steering correction or wall approach.
- [ ] Run normal obstacle laps with both magenta parking pieces installed in
      their competition positions.
- [ ] Record camera and both raw ToF observations while passing the parking
      pieces and determine exactly when they can be confused with field walls.
- [ ] Add localization gating so a ToF return consistent with a known parking
      piece cannot produce a false wall correction or false field position.
      Keep the raw range available for collision safety; do not blindly disable
      the ToF sensor in that area.
- [ ] Validate the gating from both directions and from plausible localization
      error bounds, including partial/edge views of a parking piece.
- [ ] Complete unpark, localization, and three laps with the parking pieces on
      the field without contact or a false localization jump.

## Remaining route coverage

- [ ] Decide whether the unlikely 500 mm outer-extreme opposing pair
      (seat 6 followed by seat 9) needs competition acceptance testing; ask the
      user before spending more tuning time on it.
- [ ] Test the maximum pillar count allowed by the rules in CW for three laps.
- [ ] Test the maximum pillar count allowed by the rules in CCW for three laps.
- [ ] Cover every supported unparking orientation and resulting lap entry.
- [ ] Ensure the reliability set represents stations 0, 1, and 2 on both sides.
- [ ] Exercise bright, dim, shadowed, and edge-clipped pillar views.
- [ ] Confirm normal operation at a moderately discharged battery level; do not
      begin a multi-lap acceptance run with a nearly empty battery.

## Run timing telemetry

- [ ] Save elapsed active run time in the USB log, measured from the accepted
      start/enable event until controlled completion or abort; exclude setup
      waiting and the later USB file-save interval.
- [ ] Save lap-1, lap-2, and lap-3 split times as well as total run time.
- [ ] Include completion/abort status with the saved time so failed and
      manually interrupted runs cannot be mistaken for valid timing results.
- [ ] Use the same timing definition when comparing route and speed changes.

## Clearance telemetry follow-up

- [ ] Add the planned-path/odometry/ToF clearance comparison to the automated
      post-run analysis TODO without using it as a contact sensor.
- [ ] Continue prioritizing physical observation and fresh side-ToF evidence
      over conservative odometry capsule estimates when they disagree.
- [ ] Preserve practical safety margin to both pillar and wall; do not tighten
      accepted paths merely because one run had excess space.

## Route and speed optimization

Start only after the integrated firmware, field localization, parking-marker
handling, and representative routes work reliably at 175 mm/s.

- [ ] Define hard constraints for route optimization: robot envelope, pillar
      movement circles, field walls/corners, tracking-error allowance, minimum
      clearance, steering limit, and maximum curvature/curvature change.
- [ ] Evaluate and choose an algorithm for laps 2-3 that uses the complete
      lap-1 seat map to minimize path length and curvature while respecting all
      hard constraints. Compare at least constrained spline/Bezier smoothing
      and sampled candidate-path optimization before implementing one.
- [ ] Make the optimizer deterministic and bounded in memory/runtime, with a
      validated fallback to the current safe optimized route if it cannot find
      a feasible path.
- [ ] Add offline/preflight geometry checks for collision clearance, path
      continuity, curvature, steering feasibility, and lap-seam continuity.
- [ ] Use the optimized short/smooth route only on laps 2 and 3, after lap 1
      has resolved the complete pillar layout.
- [ ] Generate a curvature-based speed profile along every route, with smooth
      acceleration/deceleration limits instead of speed steps.
- [ ] On lap 1, distinguish discovery zones from already-resolved or
      camera-irrelevant path sections.
- [ ] Slow lap 1 early enough for reliable two-frame seat detection and retain
      the bounded unresolved-seat hold.
- [ ] Increase lap-1 speed only where the camera is not needed for upcoming
      seat decisions and enough distance remains to slow for the next station.
- [ ] Permit higher lap-2/3 speed than lap 1 because the seat map is known,
      while still reducing speed according to curvature and clearance margin.
- [ ] Increase straight and lap-2/3 speed caps in small steps; do not tune all
      caps, curvature gain, and route geometry in the same test.
- [ ] At each accepted step, repeat representative three-lap CW and CCW layouts
      and compare lap times, maximum curvature, tracking error, and minimum
      physical/ToF clearance.
- [ ] Verify smooth corner deceleration and acceleration without stopping and
      no missed seat decisions on lap 1.
- [ ] Stop increasing speed when camera reliability, tracking accuracy, motor
      control, or pillar/wall safety margin degrades.

## Final parking

Start after unparking/localization and the three-lap obstacle route are reliable.

- [ ] Cover all four possible physical starting-section/parking-lot rotations.
      Canonical rotation may share geometry, but test both real direction cases:
      parking ahead after the current CCW wrap and behind after the CW wrap.
- [ ] Keep obeying red/green pass sides through the official third lap. Once
      the complete vehicle has left the last corner, parking-route signs may be
      passed on either side but must still not be touched or moved.
- [ ] Measure the final front/rear projection, straight-wheel width, full-lock
      swept outline, and robot length. Recompute the `1.5 * length` gap and the
      centred rear-axle target from those measurements.
- [ ] Before any mechanical extension, obtain organizer confirmation of its
      treatment in robot-length measurement. Compare the unchanged 165 mm body
      with a rigid 35 mm rear extension (200 mm total); do not lengthen the
      already collision-critical front as the first candidate.
- [ ] Extend the parking swept-envelope model from the fully contained target
      outward. Do not reverse the current exit unchanged: its parked pose has
      zero nominal margin at the open boundary, and a centred target failed all
      16 current-path tolerance cases.
- [ ] Require positive wall/marker clearance and strict final containment for
      every gap/placement/heading tolerance case in both mirrored directions.
- [ ] Track localization uncertainty through all three laps, then approach the
      outer scan line using the known start-section pillar map.
- [ ] Directly scan both magenta pieces with fresh raw side-ToF frames, recover
      the fixed field edge and outer-wall reference, and verify the measured
      inside-face gap against `1.5 * measured robot length`. Do not depend on
      odometry alone and do not enter the bay if either piece is unresolved.
- [ ] Implement the calculated path behind an isolated test-only segment gate,
      with the existing steer-settle, bounded-distance, brake, gyro-health, and
      motor-lock safety pattern.
- [ ] Validate one parking segment at a time without pillars, then repeat with
      every legal starting-section pillar placement without moving a sign.
- [ ] Finish with steering centred, complete projection strictly inside the
      detected rectangle, heading error at most 2 degrees, and permanent motor
      hold. This is stricter than the rule's 20 mm wheel-distance difference.
- [ ] Validate complete final parking after both CW and CCW three-lap runs.

## Final end-to-end reliability

- [ ] Run the complete sequence: parked start, unpark, localize, three obstacle
      laps with optimized lap-2/3 routes and speeds, then park.
- [ ] Complete ten consecutive full runs without wrong-side passing, missed or
      moved pillars, false parking-piece wall corrections, wall/pillar/parking
      contact, intervention, localization jumps, or lap-count errors.
- [ ] Cover CW and CCW, supported start configurations, representative legal
      pillar layouts, and realistic lighting/battery variation in the final set.
- [ ] Confirm every accepted run stops fully inside the parking area after
      exactly three laps.

## Optional end-stage bonus

Start only after the parking-lot-start end-to-end sequence is accepted.

- [ ] Add support for the alternative legal start in the middle zone above
      the parking lot.
- [ ] Validate the middle-zone start in CW and CCW without regressing the
      parking-lot start.

## Build-system maintenance

- [ ] Split the shared `include/config.h` into focused configuration headers
      (such as hardware, motor, sensors, camera, parking exit, obstacle
      navigation, and modes), and make each translation unit include only the
      configuration it consumes so routine tuning changes trigger smaller
      incremental rebuilds.
- [ ] Remove configuration includes from public headers where practical; in
      particular, move `FullFovGC2145::getClockFrequency()` out of
      `include/camera.h` so camera configuration does not propagate through
      that header.
- [ ] Verify with the IDE-managed PlatformIO Core that changing the parking
      exit segment limit rebuilds only its actual consumers while the complete
      `giga_r1_m7` firmware still compiles successfully.
