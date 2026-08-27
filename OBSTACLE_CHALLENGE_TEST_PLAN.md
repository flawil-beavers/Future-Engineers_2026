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

- [ ] Do not run another powered route test until the stall watchdog is fixed.
- [ ] Preserve the existing fast maximum-duty overload response, but add a
      sustained commanded-motion/no-progress timeout that does not require
      99% motor duty.
- [ ] Ensure planned stops and perception holds, where target speed is zero,
      cannot trigger the driving-stall watchdog.
- [ ] Log the target/profile speed, duty, elapsed time, and measured progress
      when the watchdog pauses the mode.
- [ ] Add deterministic logic coverage and build with the IDE-managed
      PlatformIO installation.
- [ ] Agree on a safe physical validation method before deliberately inducing
      a stall; do not use a hard competition-field wall as the test fixture.
- [ ] After the watchdog passes, continue with remaining route coverage.

## Remaining route coverage

- [ ] Decide whether the unlikely 500 mm outer-extreme opposing pair
      (seat 6 followed by seat 9) needs competition acceptance testing; ask the
      user before spending more tuning time on it.
- [ ] Test the maximum pillar count allowed by the rules in CW for three laps.
- [ ] Test the maximum pillar count allowed by the rules in CCW for three laps.
- [ ] Cover remaining supported starting and parking-exit configurations.
- [ ] Ensure the reliability set represents stations 0, 1, and 2 on both sides.
- [ ] Exercise bright, dim, shadowed, and edge-clipped pillar views.
- [ ] Confirm normal operation at a moderately discharged battery level; do not
      begin a multi-lap acceptance run with a nearly empty battery.
- [ ] Complete ten consecutive full runs without wrong-side passing, moved
      pillars, wall/pillar contact, intervention, or lap-count errors.
- [ ] Confirm every accepted run stops under control after exactly three laps.

## Clearance telemetry follow-up

- [ ] Add the planned-path/odometry/ToF clearance comparison to the automated
      post-run analysis TODO without using it as a contact sensor.
- [ ] Continue prioritizing physical observation and fresh side-ToF evidence
      over conservative odometry capsule estimates when they disagree.
- [ ] Preserve practical safety margin to both pillar and wall; do not tighten
      accepted paths merely because one run had excess space.

## Final speed optimization

Start only after the reliability checklist passes at 175 mm/s.

- [ ] Increase the speed cap in small steps.
- [ ] At each step, repeat one representative three-lap CW layout.
- [ ] At each step, repeat one representative three-lap CCW layout.
- [ ] Move corner slowdown/tuning here; tune curvature speed only after the
      straight-line cap is selected.
- [ ] Verify smooth corner deceleration and acceleration without stopping.
- [ ] Retain camera reliability, tracking accuracy, and pillar/wall clearance.
- [ ] Stop increasing speed when any safety or reliability margin degrades.

## Final parking

Start after the three-lap obstacle run is reliable at the selected speed.

- [ ] Implement correct parking-gap detection and approach.
- [ ] Reverse without touching either magenta boundary.
- [ ] Finish fully inside the parking rectangle.
- [ ] Keep the difference between the two side distances at or below 20 mm.
- [ ] Validate parking after both CW and CCW obstacle runs.
