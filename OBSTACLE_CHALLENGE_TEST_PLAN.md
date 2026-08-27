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

- [ ] Define and document every supported initial parking orientation and
      position allowed by the rules.
- [ ] Implement a safe unpark path that leaves the parking area without
      touching the magenta parking pieces or field walls.
- [ ] Establish the robot's field pose after unparking rather than assuming the
      rear axle reached one exact position.
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

- [ ] Determine the parking-place pose from field localization plus direct
      detection of both magenta parking pieces; do not depend on odometry alone.
- [ ] Track localization uncertainty through all three laps and decide when a
      final wall/parking-feature correction is required before parking.
- [ ] Select and approach the correct parking gap after completing lap 3.
- [ ] Reverse without touching either magenta boundary.
- [ ] Finish fully inside the parking rectangle.
- [ ] Keep the difference between the two side distances at or below 20 mm.
- [ ] Validate parking after both CW and CCW obstacle runs.

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
