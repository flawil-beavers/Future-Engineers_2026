# Obstacle Challenge checklist

This is the ordered checklist for current work. Complete the unchecked items
from top to bottom. Detailed history and decisions are in
`AGENT_DOCUMENTATION.md`; seat numbering and clearance telemetry are described
in `OBSTACLE_SEAT_NUMBERING.md` and `OBSTACLE_CLEARANCE_LOGGING.md`.

For a normal successful run, record only the log number, result, and next step
in two or three sentences. Add detail only for a failure, material code change,
new safety limit, or reusable engineering finding.

## Safety for every powered run

- [ ] Confirm the selected drive battery is within its safe operating-voltage
      range, secure it, and record pack identity and resting voltage.
- [ ] Keep the disable switch reachable and cables clear of the robot.
- [ ] Clear people and fragile objects from the field.
- [ ] Place the robot accurately at the validated start pose.
- [ ] Record direction, layout, physical contact/clearance, and USB log number.
- [ ] Treat physical contact or intervention as failure even if firmware says
      `PASS`.
- [ ] Upload firmware only with explicit user consent.

Command `O` selects the complete Obstacle Challenge, including parking exit;
its direction is inferred from the nearer parking-wall ToF. Arm with the
physical enable switch LOW, send `O`, verify that the terminal says
`Pending mode: OBSTACLE_CHALLENGE`, and only then toggle the switch HIGH.
`Y1`/`Y-1` are live-path tests that bypass parking and must never be started
with the robot inside the parking lot. `Y0` stops an active live-path test.

## Next: validate low-speed pulse-density drive

- [x] Upload branch `test/low-speed-drive` only with explicit user consent.
- [x] Use a clear, level floor with at least 1.5 m of travel, keep the enable
      switch reachable, and start with the fully charged pack.
- [x] With the enable switch LOW, send `d60`, verify manual mode is
      pending and `Manual speed armed for enable: 60 mm/s` appears, send `f`
      for 200 ms debug telemetry, then enable the robot. Require `Armed manual
      speed applied: 60 mm/s` and `Resumed mode: MANUAL` before evaluating
      motion.
- [x] Let it drive straight for about 500 mm, disable it, then send `z`. Require
      immediate stopping, correct direction, no stall, and increasing
      `PDM slots/on` counts whenever requested PWM is below the 120-PWM
      low-speed carrier. Stop immediately if shaking is worse than log 196.
- [x] Before testing curved driving, verify the repaired manual steering path:
      with ample clearance and manual mode enabled, send `s40`, then `s-40`,
      and confirm that the wheels physically move both ways. A changing
      `Steer:` field alone is not sufficient. Disable immediately if either
      physical movement is absent.
- [x] Repeat speed/steering measurements as isolated runs: disable and re-arm
      between each condition, hold only one speed and steering command for at
      least five seconds on the same floor area, and disable before lifting the
      robot. This prevents retained controller state and free-wheel samples
      from contaminating comparisons.
- [x] Repeat at `d-60`; reject unexpected direction or delayed stop.
- [ ] Optionally extend the isolated matrix with forward 100 mm/s on each pack.
      Record visible/audible cycling, mean measured speed, requested/applied
      PWM, and the USB log number. This does not block acceptance of the
      low-speed unparking fix validated by log 227.
- [x] Accept the isolated controller only if neither pack shows a slow
      multi-second power cycle, direction and stopping remain correct, mean
      speed is within 10 mm/s of target after startup, and no unpowered gap is
      visibly long enough to produce chassis surging.
- [x] After straight tests pass, test 60 mm/s forward and reverse at steering
      -50 and +50 in a clear area; stop immediately for a stall or tight-space
      clearance risk.
- [x] Only after both packs pass isolated tests, run one normal parked `O` test
      with the full pack. Require safe rear positioning, five exit segments,
      localization, reverse entry, immediate final lock, and no contact.
- [x] Repeat representative complete parking validation with the low pack.
      `log_227` passed at a reported 7.16 V with the revised 62 mm correction
      approach and unchanged 65 +/-5 mm final verification gate.
- [x] Repeat the complete parked flow in the mirrored direction before merging
      the branch. `log_228` and `log_229` passed consecutively in CW with no
      contact, intervention, abnormal motion, watchdog, stall, or abort.

## Next: validate the parking-entry-to-lap connector

- [x] Implement the user-selected observation-track architecture. After the
      primary parking station resolves, preflight and traverse another 55 mm
      full-lock reverse arc at 60 mm/s, use the normal lap-1 voting/CLEAR and
      route-injection pipeline on the preceding station, then retrace the arc
      forward before building the connector from the returned measured pose.
      M7 builds at 366688 bytes RAM and 436592 bytes flash; no upload occurred.
- [ ] After explicit upload permission, run CW/red with the green station-0
      pillar restored. Require `[PARK ENTRY SCOUT] Preflight PASS`, a settled
      reverse scan, about 55 mm outbound travel, station 0 GREEN confirmation
      and live-route injection, about 55 mm forward return, and no wall/pillar
      contact. An unresolved scout must stop safely and is not a pass.
- [ ] Require the post-return connector to preflight against the updated
      red-plus-green live route, start forward without an immediate inward
      turn, complete within the unchanged endpoint/travel gates, and continue
      through the normal green then red avoidances without contact or stall.
- [ ] Only after the complete CW/red-plus-green transition is physically
      contact-free, repeat the mirrored CCW/green-plus-red case.

- [x] Analyze logs 358--361. Log 358's unguarded CW/red connector stalled after
      104 mm while turning immediately inward; log 360 safely rejected the same
      hidden-seat-0 conflict at -21.2 mm front clearance. Logs 359/361 CLEAR
      reached the final connector segment but checked the 500 mm abort before
      accepting the existing endpoint pose gate.
- [x] Restore pose-based handoff semantics for the finite connector: accept the
      unchanged 60 mm/15 degree endpoint gate before testing the unchanged
      500 mm abort, without requiring progress to equal the exact last sampled
      waypoint. Include endpoint error in any later travel-limit log. M7 builds
      at 366664 bytes RAM and 434248 bytes flash; no upload occurred.
- [x] Reject the attempted single guard-aware quintic after bounded offline
      replay: from the log-360 pose it cannot satisfy hidden-pillar clearance,
      the 42-degree steering gate, and the 500 mm/60 mm/15 degree handoff
      envelope together. The experimental curve is not retained.
- [ ] Choose and implement the next pillar-present architecture: either resolve
      the hidden preceding pillar with a separate safe observation/avoidance
      maneuver before connector motion, or obtain explicit authorization for a
      newly justified travel envelope. Do not widen the current limit silently
      and do not restore the disproven 150 mm leg.
- [ ] After that architecture is implemented and explicitly authorized for
      upload, test CW/red with the green pillar present before any CLEAR or CCW
      validation. Physical contact remains a hard failure.

- [x] Record the user's physical report for the two-test round: the first run
      contacted the green pillar and is a hard failure; the second stopped.
      `log_357` contains only the stopped second run and no forward connector
      motion, so it is not evidence about the collision.
- [x] The user clarified that the robot was probably not reset between the
      collision and stopped runs, so no separate collision log is available.
      Do not infer contact-free behavior from log 357.
- [x] Guard the inner legal pillar position at the station immediately before
      the parking-scan target, even while it is unobserved. Check every front
      and rear connector pose against this hidden-pillar guard, log its seat and
      clearance, and reject before motion on any conflict. M7 builds at 366664
      bytes RAM and 434120 bytes flash; no upload occurred.
- [ ] On the next explicitly authorized CW/red test, restore the green pillar
      that was contacted. Require `hidden_guard_seat` in a connector PASS or a
      safe `Preflight FAIL hidden_guard` stop, plus the user's confirmation of
      no physical contact. A safe guard rejection is not a completed join.

- [x] Analyze log 357. CW exit, localization, red seat-2 injection, and scan
      completed. The connector safely rejected its first target at 43.2 mm
      forward because 47.4 degrees exceeded the unchanged 42-degree bound; its
      selected merge was only 260 mm forward/136 mm lateral and therefore too
      short for the required transition.
- [x] Require at least 350 mm spatially forward and use a gradual S-transition
      with start/end tangent scales 1.25/1.50, bounded at 800/1000 mm. Retain
      the clearance, steering, before-pillar, and travel safety gates. M7 builds
      at 366664 bytes RAM and 433712 bytes flash; no upload occurred.
- [ ] After explicit upload permission, repeat CW/red with both pillars.
      Require preflight PASS at a merge at least 350 mm forward, initial
      forward motion, no turn-around/contact/stall, connector completion at the
      saved merge index, and normal red avoidance after the merge.
- [ ] Do not proceed to CCW/green until the user physically confirms CW/red is
      contact-free. Preflight and telemetry alone are insufficient.

- [x] Analyze logs 353--356 at the reported 6.95 V. CCW green/CLEAR completed
      the combined 70.2/70.1 mm localization continuation and entered the scan
      arc without the former extra centered reverse. All four connectors then
      stopped safely at preflight: their minimum-lookahead targets required
      -62.6, +49.0, +56.2, and -38.1 degrees steering, with the CW CLEAR target
      also 40.6 mm behind its nominal pose.
- [x] Correct the Hermite derivative chord term from the reversed
      `6*t^2 - 6*t` to `6*t - 6*t^2`, so generated waypoint headings and
      preflight frames match the connector curve. Retain the 0.75 start tangent
      but use a shorter 0.25 end tangent, capped at 100 mm, to preserve forward
      departure before aligning with the lap route. M7 builds at 366664 bytes
      RAM and 433712 bytes flash; no upload occurred.
- [ ] After explicit upload permission, test CW/red first with both pillars
      present. Require preflight PASS, a forward first target, no turn-around,
      stall, wall contact, or pillar contact, connector completion at its saved
      merge index, and normal confirmed red avoidance after the merge.
- [ ] Only if CW/red is physically contact-free, test CCW/green with both
      pillars and require the same transition criteria plus one continuous
      localization reverse and no separate centered-reverse entry phase.
- [ ] After both pillar-present directions pass, test the CLEAR layout in each
      direction. Confirm the chosen merge is the forward heading-ray/path
      intersection and does not select a behind-route target.

- [x] Analyze logs 349--352. CCW localization stopped after 316--327 mm and a
      separate entry state then reversed another 68--72 mm before the scan arc;
      CW already had zero separate straight distance. All four direct connectors
      safely rejected because their first minimum-lookahead targets required
      42.6--79.3 degrees steering.
- [x] Combine the CCW centered entry distance with localization: after latching
      the second-marker transition, continue the same gyro-held reverse for 70
      mm, brake once at the existing arc start, skip the redundant straight
      entry states, settle full-lock steering, and run the 55 mm scan arc. Raise
      direct-merge minimum forward distance to 250 mm and tangent scale to 0.75.
      M7 builds at 366664 bytes RAM and 433696 bytes flash; no upload occurred.
- [ ] In the next CCW run, require one continuous centered localization reverse,
      `[PARK LOCALIZE] CCW continuation complete` near 70 mm, corrected pose near
      x=60 mm, immediate arc preload with no separate reverse-straight segment,
      and the same valid green/CLEAR camera scan behavior.
- [x] Analyze logs 347--348. Even at 25 mm lookahead, the staged connector's
      first curved target was only 7.9--14.5 mm forward and required about 80
      degrees steering. The endpoint, not lookahead length, was unsuitable.
- [x] At the user's direction, remove the 150 mm leg and cyclic merge target.
      Build one direct cubic to an exact modified-route point selected in the
      measured pose frame: 50--800 mm physically forward and, with a pillar,
      retaining 150--500 mm of its normal approach; with CLEAR, closest to the
      current heading-ray intersection. M7 builds at 366656 bytes RAM and
      433080 bytes flash; no upload occurred.
- [ ] Test CW/red first with both parking-section pillars present. Require
      preflight PASS with positive `merge_forward`, a plausible small
      `merge_lateral`, 150--500 mm `before_pillar`, a reachable selected
      lookahead, forward motion without turn-around/contact/stall, connector
      completion, and normal red avoidance after the merge.
- [ ] Only after CW/red is physically contact-free, repeat CCW/green with both
      pillars. Then test CLEAR in each direction and verify that the selected
      merge point lies near the measured heading-ray/path intersection.
- [x] Analyze logs 345--346 from the forward-envelope build. CCW/green safely
      rejected a -48.1 degree target at 82.5 mm lookahead; CCW/CLEAR safely
      rejected -49.6 degrees at 150 mm. These stops are correct but not valid
      completed transitions.
- [x] Make the 150 mm leg a real controller phase: prevent progress/lookahead
      from entering the curve before 150 mm encoder travel. Afterward, select
      and store the largest connector-specific lookahead, in 25 mm steps, that
      passes the forward-target and 42-degree steering preflight at every curve
      waypoint. The IDE-managed M7 build passes at 366656 bytes RAM and 433456
      bytes flash; no upload occurred.
- [x] Superseded at the user's direction: do not physically validate the staged
      150 mm leg. The replacement is the direct spatial merge above.
- [x] Reject a finite connector before motion when its simulated Pure Pursuit
      lookahead is not ahead of the nominal pose or requires steering beyond
      the configured physical limit. Repeat the same check from the measured
      pose at runtime, and do not apply cyclic corner gates to connector-local
      distance. The IDE-managed M7 build passes at 366648 bytes RAM and 433200
      bytes flash; no firmware upload occurred.
- [ ] On the next powered test, a geometrically unsuitable connector must log
      `Preflight FAIL tracking_sample/target` and remain stopped; it must never
      turn around. A safe rejection is not a completed transition or a physical
      pass.
- [x] Analyze logs 338--341. The pillar-present runs passed preflight but did
      not complete: CCW/green stalled after 158 mm total connector travel, and
      CW/red was manually disabled after 1615 mm. Both CLEAR runs rejected
      because connector construction incorrectly required a confirmed pillar.
      Physical contact status for the two pillar-present runs remains pending
      the user's report.
- [x] Restore the validated 0.55 green parking-entry lookahead scale for the
      finite connector, allow CLEAR to use the inner-seat route phase with
      wall-only preflight, log straight-leg completion and total path length,
      and enforce the unchanged 500 mm recovery limit. The IDE-managed M7 build
      passes at 366648 bytes RAM and 432104 bytes flash; no upload occurred.
- [x] Logs 334-337 physically collided with the opposite parking-adjacent
      pillar: the single cubic turned inward within 67-113 mm even when the
      merge phase was corrected. The proposed straight clearance leg was later
      removed at the user's direction and replaced by spatial merge selection.
- [ ] If another pillar confirms while the connector is active, require a
      stopped connector rebuild against the updated live path before motion
      resumes. A stale connector after injection is a failure.

- [ ] Logs 334-335 invalidate the generic 800-1400 mm merge window: CCW green
      rejected preflight and CW red merged at index 16 after its confirmed
      avoidance. Validate the replacement target at the injected taper start,
      500 mm cyclically before the confirmed parking-section pillar.
- [ ] Run CW/red first. Require the preflight line to report merge distance
      approximately 500 mm before seat 2, connector completion before the red
      pass, and the normal displaced live route to clear the red pillar. Stop
      immediately on contact; do not accept confirmation alone as a pass.
- [ ] Then run CCW/green. Require preflight PASS near the green seat-5 taper
      start. If it rejects, preserve the stop and use the new sample plus
      front/rear wall/pillar clearance telemetry to correct geometry.

- [x] Upload the connector M7 build only with explicit user consent. M4 is
      unchanged and must not be rebuilt or uploaded.
- [x] First run one CCW parked `O` test with the previously accepted parking
      placement and pillar layout. Keep the disable switch reachable. Require
      a resolved `[PARK ENTRY RESULT]`, `[PARK ENTRY JOIN] Armed`, and
      `[PARK ENTRY JOIN] Complete` with no contact or abrupt steering. Log 230
      passed; the user confirmed no contact.
- [x] For this first powered connector test, stop manually after the join has
      settled onto the south-straight lap path; do not spend a complete lap on
      unvalidated connector geometry. Log 230 continued through three complete
      laps without contact, superseding the planned bounded stop with stronger
      CCW coverage; retain the bounded stop for the first CW test.
- [x] Accept only if join speed stays at or below 60 mm/s, cross-track and
      heading converge to at most 60 mm and 10 degrees, no parking-piece ToF
      correction occurs during the join, and no watchdog, travel-limit, or
      perception abort occurs. Log 230 completed at 30.6 mm and 9.4 degrees
      after 564.9 mm of 60 mm/s-target join travel.
- [x] After the CCW join passes, repeat the same bounded connector test in CW.
      `log_231` invalidly contacted the inner green pillar after false clear;
      `log_232` passed only with that pillar removed. Upload the rejected-blob
      clear-veto fix with consent, restore green in the exact log-231 position,
      and require confirmation/injection before the pillar plus no contact. A
      safe ambiguity hold/abort is not a pass but is preferable to false clear.
      `log_233` passed the repaired case: green seat 2 confirmed/injected and
      the join completed at 37.4 mm / 10.0 degrees without reported contact.
- [ ] Only after both bounded joins pass, run one complete obstacle lap from a
      parked start in each direction before enabling three-lap validation.
      CCW already completed three laps in `log_230`; next run one complete CW
      lap with green restored at seat 2 and stop manually after lap 1. Logs
      234/235 did not complete lap 1 because the rejected-blob veto caused an
      empty S3 station-0 hold expiry; the veto is now limited to parking/join.
      Log 238 completed three CCW laps with green seat 5 but is physically
      invalid because the user reported a marginal pillar touch.
- [x] Build the measured-pose connector with the IDE-managed M7 PlatformIO
      Core. Logs 330--331 showed a separate leftover
      `OBSTACLE_PARKING_EXIT_REVERSE_STRAIGHT_TEST_ONLY=true` stop after valid
      localization; it is now false and the build passes. Do not upload without
      explicit consent.
- [ ] After explicit upload consent, run CW/red first from the validated parked
      pose. Require `[PARK ENTRY CONNECTOR] Armed`, `preflight PASS`, Pure
      Pursuit steering from the first forward command, no nominal nearest-route
      jump, no old 450/500 mm join/recovery failure, and
      `[PARK ENTRY CONNECTOR] Complete` at the saved merge index. Stop on any
      contact or intervention and record the user's physical report. Confirm
      the logged merge index is in the forward entry window, not near the lap
      seam; a pending mode or no-motion result is not a valid test.
- [ ] For the revised connector-time perception behavior, keep both established
      red and green pillars on the field. In CW/red, require the red pillar to
      receive confirmation/injection while the connector or before its pass;
      a later unresolved-station failure is not acceptable. Record the exact
      first visible/confirmed line and the user's physical contact report.
- [ ] Only after CW/red passes, run CCW/green with the same connector criteria.
      Require the confirmed green avoidance to remain active through the
      connector, no premature lap boundary or discovery hold, and normal
      discovery to begin from the saved cyclic merge phase.
- [x] Establish whether log 238's marginal touch occurred on the 260 mm lap-1
      route or the 210 mm later-lap route. The user confirmed lap 1 only; laps
      2-3 were clear. Keep both displacement values and add a one-waypoint
      approach/exit plateau only to CCW lap-1 green seat 5.
- [ ] After the staged-speed CW checks pass, repeat CCW green seat 5 for one
      lap. Require clearly positive physical front- and rear-wheel clearance;
      firmware completion alone does not accept the route.
- [ ] Only after that one-lap clearance test passes, repeat the same CCW layout
      for three laps to confirm the unchanged 210 mm optimized route.

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
- [x] Integrate the M4 rear ToF as a configurable longitudinal reference. Its
      origin is currently 25 mm behind the rear axle; the position and target
      body clearance are compile-time variables. Measure the initial clearance,
      move straight toward the validated 50 mm rear clearance, confirm three
      settled frames within 2 mm, and fail closed on stale/impossible data or
      more than 65 mm correction travel.
- [ ] Validate rear-ToF positioning with the exit locked out. First complete
      the stationary rear-sensor checks in `REAR_TOF_M4_SETUP.md`. Then place
      the parallel robot approximately 20 mm behind the parking-gap middle and
      require a forward correction to 50+/-2 mm body clearance, no contact,
      `[PARK REAR RESULT]`, and the positioning-only motor lock.
- [ ] Reject and replace the first powered implementation. `log_142` drove to
      within about 10 mm of the front limit after rear range changed only
      31->54 mm during 65.1 mm encoder travel. `log_143` jumped
      87->39->97 mm, reversed repeatedly, and touched the front limit. Do not
      power it again until stationary measurements against the actual magenta
      piece are credible and the travel guard counts cumulative motion.
- [ ] Confirm rear-ToF startup reacquisition after log 182. That run remained
      safely stationary but terminated after only one second of `9999` rear
      frames while inside the parking lot. The stopped acquisition window is
      now three seconds, and terminal failures log sequence, filtered/raw
      range, signal, and sigma. Require normal `[PARK REAR RESULT]`; if it still
      fails, use `[PARK REAR DIAG]` to fix the sensor-side cause rather than
      bypassing rear positioning.
- [x] Verify stationary rear-ToF repeatability against the actual magenta
      limit. `log_144` held 42--43 mm at the approximate 40 mm ruler placement
      and 63--65 mm at the approximate 60 mm placement. The optical beam is
      unobstructed. Clarify the ruler reference point before calibrating the
      target.
- [x] Replace continuous bidirectional range chasing with one settled rear
      measurement followed by one encoder-bounded correction. Do not permit an
      automatic reversal in the same attempt; stop and report the final rear
      measurement for review. The implementation averages three frames within
      a 4 mm span, uses cumulative travel, limits the one-shot correction to
      55 mm, and requires final range/motion agreement before acceptance.
- [x] Repeat the positioning-only test approximately 20 mm ahead of the
      parking-gap middle. Require a reverse correction to the same clearance,
      no contact, and correction travel consistent with the measured start.
      Log 150 passed.
- [x] Repeat one substantial forward and reverse correction after the delayed
      final-verification change. Logs 145--148 were all physically safe and
      converged near 50 mm body clearance, but only 147/148 passed telemetry;
      145/146 sampled about 13 mm short/long. Require both repeats to report
      `accepted=yes` after the 1000 ms wait and three discarded fresh frames.
      Logs 149/150 passed.
- [x] Accept rear positioning in both directions. Logs 149 and 150 passed
      delayed stationary verification after substantial forward and reverse
      corrections, and the user reported no contact.
- [x] Run one CCW combined rear-positioning plus five-segment exit test. Stop
      immediately after exit telemetry; do not run edge localization or entry
      discovery yet. Require `accepted=yes`, 5/5 segments, aligned final
      heading, no contact, and the rear-positioned-exit motor lock. Log 151
      passed.
- [x] Accept the CCW combined rear-positioning plus exit. Log 151 passed rear
      verification, 5/5 segments, final alignment, usable parking-end ToF, and
      the user reported no contact.
- [x] Validate the general low-speed load compensation after log 158 moved
      only 8.9 mm in one second during full-lock segment 5. Require no contact
      or stall, continued segment-5 motion, and review `[MOTOR LOAD COMP]` for
      a bounded response no greater than 30 PWM. This is a general M7 speed-
      controller change; there is no segment-specific boost and the watchdog
      remains unchanged. Logs 160/161 are invalid for this test: `Y1` selected
      `OBSTACLE_LIVE_TEST`, printed `Parking exit: BYPASSED`, and drove the
      normal 175 mm/s lap path from the parking pose. Use `O` and require
      `Pending mode: OBSTACLE_CHALLENGE` before enabling. Log 162 then passed
      without contact or stall; segment 5 needed only 1.1 PWM compensation and
      completed normally.
- [ ] After both positioning-only tests pass, disable
      `OBSTACLE_PARKING_REAR_TOF_POSITIONING_TEST_ONLY` and repeat the complete
      five-segment exit in both directions before restoring entry discovery.
- [ ] Later, estimate/correct non-parallel parking placement. The current
      collision model accepts approximately +/-1 degree, but one rear range
      alone cannot measure heading; arbitrary angled starts are unsupported.
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
- [x] Repeat after extending only the hard reverse-search bound to 70 mm.
      Log 163 stopped contact-free at 59.7 mm and reported a credible 259 mm
      wall return immediately afterward. The 70 mm swept model passes 16/16;
      require `transition=yes`, x/y corrections within 25 mm, and no contact.
      Log 164 was also contact-free but rejected localization at 71.7 mm. Its
      parking-piece reference incorrectly applied an unbounded 98.2 mm y
      correction, and the stable wall frames were not processed during the
      brake hold. The correction is now limited to 25 mm and fresh ToF frames
      remain eligible during braking; the 70 mm motion bound is unchanged.
      Log 166 passed contact-free at 56.3 mm with both corrections accepted.
- [x] Use the newest sample in the confirmed wall-frame sequence for the y
      correction. `log_133` showed that using the first qualifying 239 mm
      transition sample discarded a later stable 259 mm wall return.
- [x] Replace the forward edge search with the user's proposed bounded straight
      reverse. Reference the opposite edge of the same magenta piece and keep
      the lap lockout enabled. The swept model passes all 16 existing
      gap/placement/heading tolerance cases through the full 60 mm reverse.
- [x] Physically validate the reverse edge search in both directions with no
      contact. `log_136` passed CCW and `log_137` passed CW. `log_135` was an
      invalid initial attempt with no usable wall-side start range and a stall;
      it is not one of the two accepted runs.
- [x] Implement the test-only direction-specific reverse Pure Pursuit scan.
      Pre-mark only the parking section's rule-guaranteed outer seats clear,
      preserve the measured field pose, and require camera resolution of the
      first upcoming inner seat. Both direction envelopes pass 16/16 modeled
      tolerance cases.
- [ ] Physically validate the entry scan with the field clear in one direction.
      Require no contact, the expected target (`CCW: S0 station 2`, `CW: S0
      station 1`), a reached scan pose, `result=CLEAR`, and the test-only motor
      lock. `log_138` was physically contact-free but failed this criterion:
      nearest-waypoint progress skipped the short scan arc, leaving 42.4
      degrees target bearing and `result=UNKNOWN`. Repeat after the
      encoder-distance progress and final-heading fix. `log_139` was also
      contact-free, but still ended in `Scan path overrun`: travel 404.4/384.3
      mm and heading error 20.1 degrees. The user did not see a distinct arc;
      `log_153` reproduced the same nearly straight overrun after valid edge
      localization, proving the attempted sign correction was itself wrong.
      Log 162 passed the complete exit and localization, but the opposite
      pursuit sign still requested only +2.3 degrees initially and ended with
      20.8 degrees heading error. The controller now stops at the arc start,
      settles at direction-specific full lock, and reverses through the exact
      modeled 40 mm/109 mm-radius arc. Require `Arc steering settle`, CCW
      `[PARK ENTRY CONTROL] phase=REVERSE_ARC ... steering=50`, a visible arc,
      and heading error <=5 degrees. Log 166 passed: final position error was
      1.8 mm, heading error was exactly 5.0 degrees, S0 station 2 was `CLEAR`,
      and the test-only motor lock engaged.
- [x] Repeat the accepted scan with one official pillar in that same inner
      seat. Require the correct seat and colour after two votes, no contact,
      and no `UNKNOWN` timeout. Test the opposite colour separately only if the
      first result and geometry are credible. Log 167 completed the motion
      contact-free but incorrectly resolved `CLEAR`: two approach frames
      cleared the target before the red pillar became fully visible at the
      left edge. Target-seat clear evidence is now deferred until the robot
      has stopped at the scan pose; obstacle votes remain active while
      approaching. Log 168 passed contact-free and resolved the official red
      pillar as `RED seat=5` after reaching the scan pose.
- [ ] Repeat once with the same official pillar position at S0 station-2
      inner/left seat 5, changing only its colour to green. Require
      `GREEN seat=5`, no premature `CLEAR`, no `UNKNOWN`, and no contact.
      Log 169 stopped before reverse localization because its single exit
      sample was an intermediate 217 mm edge return; log 170 used 76 mm and
      completed motion but resolved `CLEAR`. The localization gate now always
      enters the bounded search for a geometrically valid 1..400 mm seed,
      collects fresh piece samples during settle and reverse, and forbids x
      correction unless a true <=180 mm piece sample was seen. Repeat green
      only after uploading this M7 build. Log 171 passed the normal path with
      `localization_seed=yes piece_seen=yes`, but its initial 79 mm return did
      not exercise the new >180 mm intermediate-seed branch. It again resolved
      green as `CLEAR`; the only late green return was clipped at the far-left
      edge and below the validated area threshold. Repeat unchanged to collect
      both localization-seed reliability and green visibility evidence; do not
      lower blob thresholds from this partial return.
- [ ] Validate the revised CW reverse-straight gyro heading hold before joining
      the normal lap route. Logs 186/187 both reached the arc without contact
      or abort, but the user observed strong servo oscillation. The permanent
      10-degree bias and 5-degree minimum correction are replaced by an
      8-degree preload that fades to zero over the first 40 mm, smooth gain 4
      feedback capped at 12 degrees, and a 200 ms initial settle. Require no
      visible oscillation, no abort/contact, and a logged straight maximum
      heading error below 2 degrees.
      Earlier evidence: validate the CW reverse-straight gyro heading hold
      before any further CW
      perception acceptance. Logs 175 and 176 both made rear-barrier contact
      during the long reverse approach and were manually disabled before the
      arc. The old controller commanded steering zero and drifted from about
      179.5 degrees to 176.7--176.9 degrees. The new reverse gyro hold uses
      bounded +/-12 degree steering toward the localized start heading and
      locks off if absolute heading error exceeds 2.0 degrees. Require no
      contact, no heading abort, visible correction telemetry, and arrival at
      the arc settle with heading error remaining below 2 degrees. Log 177
      safely aborted at -2.0 degrees but proved the first feedback sign was
      wrong: a -0.5 degree error commanded +1 steering and grew. The measured
      sign interpretation was itself rejected by logs 180/181/183: -1 steering
      also let negative error grow to the abort, while full-lock reverse
      telemetry proves negative steering produces negative yaw. Restore
      positive correction for negative error, raise proportional gain to 8,
      and enforce a 5-degree minimum effective correction from 0.3 degrees
      error. Logs 184/185 still aborted because correction began only after
      reverse motion while the servo was centred. Preload direction-mirrored
      10-degree steering, settle stationary for 400 ms, then trim with gyro up
      to 20 degrees. Logs 186/187 rejected that final tuning because it
      oscillated visibly despite completing the path safely.
      Logs 188/189 then held the raw heading within 0.9/1.1 degrees and the
      user reported only slight residual oscillation. The correction now uses
      a 0.20 low-pass filtered heading error, gain 3, and a 0.4-degree engage
      threshold; the raw 2-degree abort remains immediate. Repeat one empty
      CW run and require calm steering plus the same sub-2-degree bound.
- [x] Validate the 55 mm settled scan arc in both directions. CW log 186
      resolved the empty target `CLEAR`; log 187 correctly resolved the
      official inner green pillar as `GREEN seat=2`. Repeat CW once with the
      calmer heading controller, then retain CCW green as the remaining
      mirrored regression. CCW log 172
      contained a production-sized green pillar blob (25x49 pixels, area 616)
      but its centre was near image x=11, outside the calibrated x>=30
      acquisition region; logs 171/173 similarly left green clipped or absent.
      The former 40 mm arc is extended by 15 mm, rotating the camera about 7.9
      degrees farther at the measured 109 mm radius. Keep all colour/blob and
      seat-snap gates unchanged. Require `GREEN seat=5` in CCW and the mirrored
      correct green seat in CW, with no contact or heading abort. Logs 188/189
      passed the final mirrored test: CW resolved `GREEN seat=2` and CCW
      resolved `GREEN seat=5`, with maximum reverse-straight heading errors of
      0.9 and 1.1 degrees respectively.
      The damped-controller regression then produced `GREEN seat=2` in CW log
      191, but CCW log 190 falsely resolved `CLEAR` with the pillar present.
      Its green regions were invalid low floor fragments and the upright
      pillar did not segment before the two-frame clear decision. Parking-only
      stationary clear confirmation is now five frames; repeat CCW green seat
      5 and require `GREEN seat=5`, not `CLEAR`.
- [ ] Validate increased clearance from the pink parking-limit pieces. The
      unrelated red/green obstacle-route clearance change was reverted. Exit
      segment 4 is extended from 75 to 85 mm and the modeled gyro-aligned final
      arc from 140 to 150 mm, shifting the parallel exit pose about 19.4 mm
      farther from the pink pieces in the swept model. Require this additional
      physical margin in both directions, with no contact or heading abort.
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

### Immediate parking-entry validation after logs 239-243

- [ ] Upload the current M7 firmware only after explicit user consent.
- [ ] Before the pillar tests, repeat one CW parking exit after the `log_244`
      pink-wall contact. Require final alignment to be captured at or after
      120 mm, centered-wheel continuation to at least 150 mm, and successful
      reverse-parallel motion with no contact or stall. Stop testing if this
      bounded regression fails.
- [ ] Run CW with the same green pillar at parking-entry seat 2. Require a
      valid green injection and clean passage; if the view remains ambiguous,
      the robot must hold/abort instead of declaring the station clear. Confirm
      the forward join starts at 100 mm/s and changes to 60 mm/s after 200 mm.
- [ ] Run CCW with the green pillar at seat 5. Require the edge-localization
      correction to be accepted, no physical contact, and join completion
      before the new 900 mm travel limit.
- [ ] If both bounded runs pass, run one complete lap in either direction to
      confirm that the 80 mm/s reverse straight and faster initial join do not
      regress detection, localization, or continuation.
- [ ] Keep the stationary full-lock steering settle before the scan arc. Test a
      moving steering transition separately before considering its removal.

### Immediate validation after logs 245-249

- [ ] First run one bounded CW unparking/localization test without relying on a
      pillar result. Require unconditional gyro-guided reverse, recognition
      that the first marker is present or already passed, two-frame acquisition
      of the second marker, a 120-to-60 mm/s change, its far-edge transition
      within 380 mm, less than 3 degrees heading error, valid corrections, and
      no contact or stall.
- [ ] Repeat the same bounded localization test CCW. Do not proceed if either
      direction misses the second marker or reaches the travel/heading limit.
- [ ] After both directions pass, repeat the layouts that failed at S3 station
      0 in log 245 and S1 station 0 in log 247. Confirm the robot slows to
      100 mm/s before the trusted seat-view window and resolves both seats
      before the 340 mm hold line.
- [ ] Confirm the forward parking-entry join runs with the 175 mm/s lap-1 cap
      throughout and still meets the 60 mm cross-track/10 degree heading gate
      before 900 mm.

### Immediate validation after logs 250-251

- [ ] Reset with the enable switch LOW and no serial connection required.
      Require the onboard LED to turn blue only after camera/vision setup is
      complete. Toggle the switch and require blue to turn off immediately
      before motion begins.
- [ ] Repeat CCW second-marker localization. Require the correction to remain
      within the new 35 mm gate and be applied, with no 5-degree heading abort,
      380 mm travel limit, contact, or stall.
- [ ] Repeat the same bounded check CW and require acquisition and passage of
      the second marker, not merely passage of the first marker.

### Immediate CW validation after logs 252-254

- [ ] Run one bounded CW unparking test. Require final segment steering -45,
      heading capture before the 220 mm fallback, and visibly more clearance
      than the roughly 23 mm wheel-side estimate from log 254.
- [ ] If the initial side return is the distant outer wall, require any marker
      reacquired within the first 100 mm to be labeled as the first marker.
      Require a later, separate second-marker acquisition and edge transition.
- [ ] Require maximum reverse heading error below 5 degrees with the 4.0 Kp / 15
      degree steering correction, accepted X/Y corrections, and no contact,
      stall, travel-limit stop, or intervention.
- [ ] Treat CCW as currently accepted from log 252; do not spend another run on
      it unless a shared localization change later affects both directions.

### Symmetric final-arc validation

- [ ] This section supersedes the direction-specific instruction immediately
      above: run one bounded CW and one bounded CCW exit because both final arcs
      now use the shared logical 45-degree magnitude.
- [ ] Require segment 5 steering to be -45 in CW and +45 in CCW, with heading
      capture before the symmetric 220 mm fallback and no pink-limit contact.
- [ ] If `SERVO_CENTER` is recalibrated later, do not retune these signs or
      magnitudes merely to compensate for zero; confirm that the steering API
      still produces mirrored physical angles around the new center.

### Immediate validation after logs 255-258

- [ ] Run CW first. Require the between-marker phase to use 100 mm/s, maximum
      heading error below the unchanged 5-degree abort, both markers and the
      second edge to resolve, and no contact, stall, or travel-limit stop.
- [ ] Require no seat-distance perception hold while `[PARK ENTRY JOIN]` is
      active. The join must first meet its 60 mm cross-track and 10-degree
      heading gates; normal unresolved-seat slowdown/hold resumes afterwards.
- [ ] Observe the handoff after any initial rear-position correction. Confirm
      that the 200 ms post-move settle plus one discarded frame feels shorter,
      while `[PARK REAR VERIFY]` still reports `accepted=yes` before segment 1.
- [ ] If CW passes, repeat once CCW because marker-crossing speed and rear-ToF
      handoff timing are shared, even though log 257 accepted its geometry.

### Immediate validation after logs 259-261

- [ ] Run CW first from a placement needing a short rear correction (less than
      20 mm if practical). Require `[PARK REAR] Correction stop reason=tof_target`
      and `[PARK REAR VERIFY] ... accepted=yes`; `travel_bound` is a failure to
      investigate, not a reason to widen the bound.
- [ ] Require the CW reverse localization to pass both markers and complete the
      second edge with maximum heading below 5 degrees. The controller remains
      Kp 4 but can now use the full requested 20-degree steering correction.
- [ ] If CW succeeds, run one CCW regression because rear closed-loop stopping
      and the 20-degree reverse steering cap are symmetric shared changes.
- [ ] Report any physical contact explicitly. Otherwise, stop after these two
      bounded runs; do not spend a three-lap attempt until unparking passes.

### Immediate validation after logs 262-267

- [ ] Run one bounded CW and one bounded CCW test. If the initial confirmed
      rear range is 60-70 mm, require the arcs to begin without an unnecessary
      rear correction. This entire interval is inside the final +/-5 mm gate.
- [ ] For a starting range outside that interval, require a raw-ToF moving stop
      at approximately 62 mm when driving forward or 68 mm when reversing,
      followed by `[PARK REAR VERIFY] ... accepted=yes`.
- [ ] Both runs must complete the first-marker, second-marker, and second-edge
      phases without a heading abort, travel-bound stop, intervention, or wall
      contact. The user confirmed no wall contact in logs 262-267.

### Immediate validation after logs 268-271

- [ ] Run CW first. After at most one rear correction, accept either
      `mode=target` or `mode=measured_pose`; both must proceed directly to the
      exit arcs using the confirmed stationary rear range as the exact pose.
- [ ] Require CW second-edge localization to complete below the unchanged
      5-degree abort with Kp 5 and at most 25 degrees logical steering.
- [ ] If CW succeeds, run one CCW regression. Log 271 is the current CCW
      reference with 1.5 degrees maximum error and a completed entry join.
- [ ] Do not tune the M4 software-I2C clock from these logs. Its 100 kHz bus is
      slower than hardware I2C, but the 30 ms sensor integration and mechanical
      take-up dominate this positioning behavior; the new policy avoids live
      sensor chasing instead.

### Safety stop after the post-log-271 batch

- [ ] The 48-80 mm measured-pose window is rejected: the user reports that the
      last run touched the wall and stalled. Analyze the new logs and tighten
      the range before another physical run.
- [ ] Do not run broad reliability or three-lap tests until one bounded CW and
      one bounded CCW unparking complete without contact, stall, or intervention
      under the corrected conservative positioning gate.
- [ ] Log 275 identifies the unsafe case: 76.3 mm was accepted, then CCW
      segment 2 contacted the wall and stalled. Require 60-70 mm before segment
      1; no `measured_pose` outside that range is acceptable.
- [ ] Validate one CW run first from a placement already near 65 mm. Confirm the
      bounded PI gyro controller completes the second edge below 5 degrees.
- [ ] Only after CW succeeds, run one CCW regression. Both runs must complete
      without contact, stall, or intervention before testing extreme initial
      rear offsets again.

### Current baseline after logs 288-297

- [ ] Treat 400 ms steering settle and 300 ms brake hold as the tested exit
      baseline. The batch did not validate the temporary 200/150 ms workspace
      values.
- [ ] Preserve the rear-position improvement (9/10 accepted starts), but repeat
      the micro-correction case near 55 mm because log 290 overshot to 73.3 mm.
- [ ] Repair localization before more broad runs. Logs 289, 291, 292, 294, and
      296 completed all five arcs but failed because the exit pose had no
      parking-piece seed and the old localizer accepted an immediate wall
      transition at zero creep.
- [ ] Do not solve those five failures only by widening the X correction gate.
      Require confirmed marker/wall phase ordering and nonzero bounded reverse
      travel before accepting a longitudinal reference.

### Validate restored marker phases with requested shorter timings

- [ ] Treat 200 ms steering settle and 150 ms brake hold as a new requested
      test configuration; logs 288-297 used 400/300 and do not validate it.
- [ ] Run one bounded CW and one bounded CCW attempt. Require ordered first
      marker, first-marker passed, second-marker acquired, and confirmed second
      edge messages with nonzero reverse travel.
- [ ] Require accepted X/Y correction, completed localization, and no heading
      abort, 380 mm travel limit, stall, wall contact, or intervention.
- [ ] If both pass, repeat each direction once before resuming broader obstacle
      layouts. If either fails, stop and inspect that log rather than widening
      the localization correction gates.

### Remove the rear-position second shot after evidence collection

- [ ] Keep the micro-correction temporarily while collecting one-shot model
      data. For every attempt retain initial stationary range, direction,
      requested first travel, encoder travel, signed stationary ToF change,
      final range/error, and whether a micro-correction was required.
- [ ] Collect at least 10 valid samples per direction and cover short (<=15
      mm), medium (15-35 mm), and long (>35 mm) requested first corrections
      where the physical starting placement permits it.
- [ ] Fit a bounded direction- and distance-dependent compensation that maps
      desired physical range change to first-shot encoder travel. Use stationary
      ToF displacement as ground truth; do not fit against moving ToF values.
- [ ] Validate the candidate model offline on held-out logs. Reject it if it
      predicts a final range outside the conservative 60-70 mm window or needs
      an unsafe travel allowance.
- [ ] Implement the accepted one-shot model and remove/disable the second
      micro-correction. Then require 10 consecutive one-shot positioning results
      inside 60-70 mm across both directions before calling it reliable.
- [ ] Optimize its time only after reliability: retain one stationary verified
      measurement, but avoid any second movement or second settle cycle.

### Green-block contact containment after log 301

- [ ] Record log 301 as physical green-block contact. Require parking-entry
      join completion below the new 450 mm travel limit; never widen the limit
      based only on odometry or a late successful gate.
- [ ] Run one bounded CW and one CCW test. A join-limit stop is an acceptable
      safe diagnostic failure; contact, stall, or continuing beyond 450 mm is
      not.
- [ ] Logs 302-303 isolate the contact to green seat 5; red seat 2 was physically
      clear. Test green/CCW first with its new plateau, 100 mm/s join cap, and
      0.55 lookahead scale. Require no green contact.
- [ ] After green passes, repeat red/CW once. Its avoidance geometry and 175
      mm/s join cap are intentionally unchanged; confirm it remains clear.
- [x] Logs 304-305: green/CCW and red/CW were both physically contact-free.
      Preserve their current clearance geometry and color-specific join speed.
- [ ] Both runs safely stopped at the 450 mm join limit despite cross-track
      reaching 34.2/30.9 mm; heading remained 60.1/47.9 degrees. Fix connector
      heading convergence without widening the travel limit.
- [ ] Replace or constrain lap-seam nearest-path joining with an explicit
      forward connector that aligns to the cyclic route before normal progress
      tracking. Revalidate green/CCW first, then red/CW, below 450 mm.
- [ ] Logs 306-308 were contact-free, but red/CW and green/CCW again stopped at
      450 mm with cross-track already inside 60 mm and heading still 48-63
      degrees. Treat four consecutive occurrences as sufficient evidence to
      implement a dedicated low-speed heading-alignment phase.
- [ ] Keep total join travel capped at 450 mm. The alignment phase must begin
      inside a conservative cross-track capture band and complete both the
      existing 60 mm / 10 degree gates without changing pillar clearance.
- [ ] Repeat CCW marker localization once during connector validation. Log 307
      acquired the second marker but missed the confirmed far edge at its
      distance bound; do not loosen thresholds from one miss.
- [ ] Validate the implemented heading phase: it must start only after
      cross-track <=60 mm, run at 80 mm/s with <=35 degrees logical steering,
      and complete heading <=10 degrees within its 300 mm bound.
- [ ] Confirm the extended 430 mm localization bound fixes the isolated log-307
      second-edge miss without heading abort, contact, or marker-order changes.
- [x] Logs 309-311 confirmed ordered localization within 341 mm; retain the 430
      mm bound without further widening.
- [ ] Reject direct heading steering: log 311 hit the red pillar and log 309
      corrected heading while worsening cross-track from 60 to 114 mm.
- [ ] Validate bounded Pure Pursuit recovery instead. After lateral capture,
      retain obstacle-aware path steering at <=80 mm/s and require completion
      inside a separate 500 mm recovery bound. Test red/CW first, then green/CCW.
- [ ] Validate the stationary observation increase from 1200 to 1600 ms against
      a weak/late color view like log 310 without weakening confirmation.
- [x] Reject immediate handoff into the old nearest-route join: log 317 armed
      after about 12 mm of scan but stopped at 88.5 mm / 55.6 degrees. Keep the
      complete bounded scan temporarily until the measured-pose connector is
      implemented and preflighted.
- [ ] Confirm the 15-degree join gate prevents log-314-style false stops while
      normal Pure Pursuit continues reducing heading error after completion.
- [ ] Confirm earlier 750 mm lap-1 slowdown plus 800 ms bounded hold resolves
      the log-315 S2 station-0 camera timeout without weakening two-frame votes.
- [ ] Revalidate S2 station-0 and station-1 at the 100 mm/s discovery cap with
      camera-target nudging suppressed through each injected avoidance. Require
      the driven clearance path to match the normal-speed behavior and retain
      at least 150 mm past-pillar protection before aiming at the next station.
- [ ] Replace nominal-route nearest-point parking joining with a finite
      measured-pose connector. Its first waypoint must be the localized pose;
      its terminal position and tangent must match a selected forward waypoint
      on the already-displaced fixed-field route.
- [ ] Run swept-envelope preflight for that connector against both field walls,
      the confirmed parking-section pillar, and the robot footprint. Reject an
      unsafe connector without moving; do not translate the complete lap or its
      fixed seat/wall geometry to the measured start pose.
- [ ] Follow the connector with the standard obstacle-aware Pure Pursuit
      controller from the first forward command. Transfer directly to the saved
      cyclic merge index without a global nearest-path search, heading-only
      controller, or separate recovery steering mode.
- [ ] Validate the connector first CW/red and then CCW/green. Require continuous
      Pure Pursuit control, no contact, no 450/500 mm join-limit stop, and a
      bounded monotonic reduction in connector error before removing the old
      join/recovery implementation.

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
