# Connector diagnostic run and acceptance sequence

## Current next run: diagnose the first-curve perception hold

Logs 390/391 physically confirm CW exit and connector completion without contact,
but stop at unresolved S1 station 0 (global station 3), before the next section's
reported later red pillar. The first lap remains incomplete.

New M7 firmware adds `DISCOVERY_TRACE` using cached frame pose and observations;
motion, thresholds, 800 ms hold and one-lap stop remain unchanged. At most 32
periodic trace records plus eight reserved hold records per run, 200 ms minimum
periodic interval. Each seat field contains bearing, range, comfortably-visible,
rejected-raw-blob overlap, observation-allows-clear, clear-frame count, stored
clear state. Frame timestamp exposes observation age. Observation status is the
numeric enum in `include/obstacle_path.h`. Motor command precedes trace formatting.

Upload M7, keep the same setup and parked CW pose, wait BLUE and start with the
switch. One repeat suffices. If it holds in the first curve, leave it stopped for
about two seconds, disable normally and let USB save finish. Return the complete
log and contact/stop report. Do not move pillars, widen thresholds or force it
through the hold. Read per-seat trace to distinguish unavailable view, rejected
colour overlap and insufficient clear frames before designing a correction.

## Previous build: corrected tangent and one full lap (tested in logs 390--391)

The newest build joins the outgoing XY direction of the displaced live route,
instead of its retained baseline heading metadata. For CW logs 386/387 that
direction is 132.57 degrees rather than -180 degrees. Before motor enable,
lookahead candidates must also pass a 2 mm kinematic rollout up to the existing
500 mm travel limit, checking front/rear capsules against walls and confirmed/
guard pillars with 10 mm model clearance margin. The 42-degree steering and
60 mm / 15-degree handoff limits remain unchanged. This margin and ideal rollout
do not establish real pose/actuator accuracy.

`OBSTACLE_FIRST_LAP_TEST_ENABLED=true` requests one normal discovery lap and
stops/saves before final parking. Obstacle remains the automatic startup mode.
The previous diagnostic-only instructions below describe the earlier build.

1. User uploads the new M7 firmware; M4 remains unchanged. No upload by the agent.
2. Keep the photographed setup unchanged for the first CW regression, photograph
   the parked start and wait for BLUE readiness before enabling the start switch.
3. Let normal exit, scan/scout, connector and first-lap discovery proceed. Serial
   O is unnecessary. Stay ready to disable on contact or uncontrolled motion.
4. Successful completion prints `[OC] First-lap test complete - stopped; final
   parking skipped` and saves automatically. Wait until the USB save is complete;
   do not disable power during writing. A safety hold is a failed test, not a
   reason to force continuation with O.
5. Return the complete log, direction, contact/stall report and whether the
   first-lap stop occurred. After a clean first CW run repeat CW once; review
   these before CCW. A repeated hold needs one complete log before more tests.

Offline verification: both recorded CW start geometries converge under the
existing gates; 324 assumed cases (two starts, reflection, +/-10 mm XY,
/-2-degree heading, yaw gain 0.85/1/1.15) pass. Modeled minima are wall 148.5 mm
and pillar 88.2 mm. Reflections are synthetic, not actual CCW geometry replays;
the failed CCW logs do not contain complete path geometry. The model covers
connector motion, not the full first lap, camera discovery or physical contact.

Reproduce with IDE-managed Python:
`simulation/connector_transition_sim.py simulation/evidence/parking_exit_diagnostics/20260926_log_386_cw.txt simulation/evidence/parking_exit_diagnostics/20260926_log_387_cw.txt`.
Output stays in `local_workspace/connector-transition/rollout.json`.

## Historical diagnostic build (superseded for the next run)

Read-only geometry/tail telemetry is added; connector targeting, speed,
42-degree steering limit, 60 mm / 15-degree handoff, obstacle guards and travel
limit are unchanged. No firmware upload has been performed by the agent.

`CONNECTOR_CONFIG` identifies this source build. Successful preflight emits
connector points and at most 32 following route points while stopped. Final
four waypoint positions emit at most 32 periodic records at 100 ms intervals,
plus the rejection record regardless of the periodic budget. Motor stop occurs
before rejection formatting. Records use cached pose/geometry only.

These records are outside the parking-exit diagnostic budget. Worst-case
geometry/tail snapshot is roughly 20 KiB (64 connector points, 32 route points,
33 tracking records); replans can emit further snapshots. Check raw-log overflow
and physical timing in the returned run; no measured loop-time bound exists.

## Historical diagnostic procedure (completed in logs 386--389)

1. Photograph the current setup from above, including parking lot, inner wall,
   pillars and CW direction. Keep positions unchanged for this diagnostic repeat.
   The user's two-consecutive-pillar setup is a recognition/transition regression,
   not accepted competition-layout proof.
2. Upload the prepared M7 firmware with the IDE; retain existing M4. Do not run
   until upload succeeds. Normal O retains practice false, discovery true,
   discovery test-only false, five exit segments and the 85 mm scout. Final
   parking entry remains unarmed.
3. Start CW from the same parked pose using normal O, as in logs 383--385.
   Be ready to disable on contact, trapping or uncontrolled motion. Do not guide
   the moving robot.
4. Observe exit, scan, scout/retrace and connector, especially the last turn.
   If it holds again, do not restart O within the run. If it completes, observe
   stable route following and the next pillar pass before disabling.
5. Disable as usual and finish the normal USB save before removing the stick.
   Preserve the complete original. Report log number, contact/no contact,
   automatic hold/manual stop and whether the next pillar was passed; battery
   voltage if available. One run suffices if the same rejection repeats.

## Offline decision

Run `simulation/analyze_connector_tracking.py <repository-relative-log>` with
the IDE-managed Python. Output is ignored
`local_workspace/connector-tracking/report.json`.

The tool checks finite target/steering replay against rounded printed values,
separates endpoint distance and heading gates and compares continued-route
targets at recorded poses. An assumed +/-10 mm XY, +/-2-degree heading grid
contains 27 cases per sample; these are not measured bounds or probabilities.
Old logs 383--385 explicitly report missing geometry/tail pose.

Before implementing route continuation, require shared runtime/preflight
selection, route coverage, swept vehicle/wall/pillar checks, closed-loop replay
and mirrored cases. Candidate steering feasibility alone is insufficient.
Preserve the steering and endpoint gates; do not force route activation.

## Physical acceptance after a justified correction

1. Three nominal CW repeats on an identified official starting-section variant.
2. CW repeats with modest parked-position variation inside the parking lot,
   varying and documenting one factor at a time.
3. Single red, single green, clear station and permitted two-pillar variants,
   following official layout cards and parking-section relocation.
4. Mirror accepted cases in CCW after contact-free CW connector completion.

Require connector completion, stable route following, correct pillar side and
no contact. Unresolved scenes must retain controlled hold. This expands coverage;
it does not prove success for every physical disturbance from a single batch.
