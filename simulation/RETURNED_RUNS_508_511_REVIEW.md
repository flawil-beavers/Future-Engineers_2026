# Returned runs 508--511, received 2026-10-26

Four complete original logs are archived unchanged in
`simulation/evidence/parking_exit_diagnostics/`; hashes, byte counts and physical
reports are in its README. Directions match the user's order: CW, CCW, CW, CW.
USB filesystem dates say 2098 and are unusable; filenames use receipt date.
All four have a `RUN_END` with `truncated=0`, with no detected log overflow or
parking-diagnostic ordering errors. The last exit is physically unsuccessful
and has no exit-completion record; its source file is nevertheless complete.

Parking build marker: `Oct_18_2026_20_08_55`. Connector build marker in 508/510:
`Oct 18 2026_20:08:41`. These agree with the last documented candidate build
markers, but no installed binary/hash was read back. The source reviewed is
checkout `94b2e0d`; this review changes no firmware or calibration.

| Log | User's physical report | Recorded outcome |
| --- | --- | --- |
| 508 CW | Stops after start | Start connector rejects required steering -42.030 degrees against the 42-degree limit. |
| 509 CCW | Also stops after start | Full scan/scout footprint preflight rejects the localized start pose. No scan movement follows. |
| 510 CW | Driving works; final parking fails | Three laps complete; final scan aborts `dual_marker_scan_incomplete`, before capture/entry. |
| 511 CW | Left front wheel contacts the front parking barrier | No-progress watchdog calls `mode_pause()` in segment 5. Onboard-pose footprint replay also exposes left-front reserve loss during segment 4. |

## 508: real tracking leaves the nominal connector envelope

The connector is planned from approximately (304.06, -1220.05, 147.66 degrees)
with 150 mm pursuit and an outgoing route continuation. Planning passes. At
t=138187 ms, progress 10, the onboard estimate is
(139.91, -1062.61, 123.66 degrees). The target is (0, -1000), still 129.67 mm
forward, with 81.74 mm lateral offset. Required steering is -42.030 degrees;
the runtime guard stops the motor. It is not a target-behind failure.

The actual CAD inverse replays this command as -42.0337 degrees from rounded
logged coordinates. All available connector-target commands in this batch
agree with CAD replay within 0.009 degrees. The old
`analyze_connector_tracking.py` uses the earlier bicycle wheel-angle model and
therefore raises a replay-disagreement error on these new commands; that is an
analyzer model limitation, not evidence of corrupted logs.

At progress 10, the corresponding stored waypoint is approximately
(144.50, -1075.86, 141.61 degrees). The robot is about 14 mm from this waypoint
and about 18 degrees below its tangent. The logged endpoint heading error
41.40 degrees refers to the future endpoint and is not the current tangent error.

Replaying the logged connector from its planned start through current production
preflight passes. Simple CAD-curvature multipliers 0.8/0.9/1.0/1.1/1.2 all pass
too, as does the corresponding 510 connector. Thus these simple nominal models
do not reproduce the physical failure. The sample does not identify a unique
servo-lag, backlash, slip, gyro or estimator cause. One gyro report timeout and
restart occurs before the connector progresses; causality for the later
steering rejection is not established.

Keep the 42-degree gate. A correction needs a connector/target strategy checked
against the measured pose/heading deviations and actual update/servo timing;
raising the limit by the observed 0.03 degrees would hide the missing reserve.

## 509: a localized pose falls just outside the scan reserve

Unparking and first-edge localization complete. Before the final localization
correction the estimate is approximately (394.8, -1221.5, 358.42 degrees).
The existing edge/wall correction applies (-38.6, -11.1) mm, producing
(356.2, -1232.6, 358.4 degrees). The robot then stays held; no start arc is driven.

Production-source replay reproduces rejection at the last simulated point:
92 mm travel = 84 mm scan plus 8 mm braking allowance, with the +1-degree
heading-budget branch. The failing gate is the parking footprint with its
5 mm body reserve, at approximately (274.36, -1268.32, 47.76 degrees).
Changing only the supplied Y by +1 mm passes this particular rounded-pose
replay. This illustrates a boundary sensitivity, not a validated 1 mm fix or
proof that the physical pose is accurately known to that precision.

This stop is explained by geometry/localization, not by a demonstrated camera
miss. Revisit the wall/edge registration and the accepted start-reference
envelope; retain swept-body and braking checks instead of bypassing preflight.

## 510: three laps succeed; the scan misses its required sequence

The physical report says the driving portion worked. The inferred map is the
existing B occupancy: GREEN 0/7/15/22 and RED 4/10/19. Raw images and independent
ground-truth object coordinates are absent, so successful driving alone does
not establish a recognition success rate.

| Stage | Time from `RUN_LAP` |
| --- | ---: |
| Start, unparking and lap 1 | 58.080 s |
| Lap 2 | 20.403 s |
| Lap 3 | 20.047 s |
| Start through all three laps | 98.530 s |
| Parking handover through scan abort | 35.244 s |
| Start through abort | 133.774 s |

Later-lap speed factor 2.00 / ceiling 520 mm/s is logged. Actual laps in this
run take about 20 seconds, rather than the earlier 16.5--18 second target-speed
integrals. These are measurements of this failed-parking run, not completion
times for a fully successful challenge or an exhaustive timing comparison.

The decisive scan sequence is visible in the sampled trace:

1. Approach reaches (797.4, -1207.0, 179.5 degrees), meeting the nominal line
   gate. Initial ToF wall registration then applies **Y=-25.4 mm**, moving the
   estimate to Y=-1232.4 before the scan starts. The scan target is Y=-1205.4.
2. The robot scans west while recovering that approximately 27 mm lateral
   difference. At estimated X=741..705, selected ranges drop to 166/110/81/75 mm,
   compatible with marker returns. Heading is approximately 172..168.6 degrees;
   the nominal line gate requires Y within 5 mm and heading within 1 degree.
   The scan state remains phase 0.
3. After line convergence, many nominal wall returns are about 293--313 mm
   versus predicted 257--269 mm. Several exceed the separate 25 mm wall-class
   residual gate. Simply being farther than the 180 mm marker threshold does
   not make a frame acceptable wall evidence.
4. Phase 1 first appears near X=460.9, phase 2 near X=442.9 with raw=142 mm,
   and phase 3 near X=396.6. Phase 3 means first marker completed, waiting for
   the second. It remains there down to X=-132.6, then aborts. No phase 4/5,
   scan localization, capture or seven-segment parking entry is reached.

Inference: the first physical marker is likely passed during line acquisition,
and the later marker is treated as the first. The traces establish late phase
progression and incomplete classification, but not the physical identity of
each return: accumulated X drift, cone edges and mixed/reflected objects are
not independently resolved. They also do not expose every sensor frame or
the exact internal line-latch instant.

Five gyro report timeouts/restarts occur during parking, followed by roughly
1.6 s poll gaps. The source stops driving while unhealthy and resumes; these
interruptions add delay. The poll-gap message immediately after a blocking
restart is not proof that scheduling starvation caused the original sensor
timeout. SPI/BNO stream health remains unresolved.

A correction should establish and verify the scan line **after** wall
registration, while preserving enough corridor to see both markers. Test
the observed wall residuals and partial marker sequences; do not simply accept
one marker or widen the thresholds without a validated registration model.

## 511: reported left-front contact is consistent with depleted swept reserve

User clarified the contact: **left front wheel, front barrier**. Exact contact
instant was not identified. There is no gyro restart in this exit.

At the end of segment 4, the onboard heading is about 114.62 degrees versus
nominal 98.00 degrees, with about 20.2 mm XY disagreement. These are estimator
versus nominal differences, not independently measured chassis pose errors.

Using the production body/wheel polygons at the logged onboard poses predicts
left-front-wheel overlap with the movable/front barrier at t=20929 and
21132 ms, both during segment 4, for a 242.5 mm gap. At the nominal 247.5 mm
gap the physical polygons do not overlap, but the 5 mm barrier-face reserve
is consumed at those same samples. At 252.5 mm, that sampled overlap is absent.
The body-mask identifies only the left front wheel. No equivalent sampled
physical overlap is found in exits 508/509/510. Actual gap, placement and pose
errors are unknown; the replay supports the reported contact but does not
independently prove its timestamp or penetration depth.

In segment 5 the car stalls. At t=25489 ms the watchdog records only 9.6 mm
directional progress in 1001 ms, target/profile=80/80 mm/s, measured=12.9 mm/s,
requested DC=144.6. It calls `mode_pause()`. Consequently the later
`Manual disable -> IDLE` text is **not proof of a manual button press**.
Exit completion and final localization are not reached.

The exit needs more physical left-front reserve across measured heading/path
variation. Do not increase torque, relax stall detection or infer a servo-zero
change from these full-lock movements. Stable straight-steering coverage is
insufficient for a new neutral estimate.

## Reproduction, scope and next work

Run with the IDE-managed Python from the repository root:

```
python simulation/analyze_parking_exit_batch.py
python simulation/review_returned_runs_508_511.py
```

The first refreshes archived pose, brake, reversal and existing ToF sensing-fan
assessments. The second extracts current CAD/footprint/preflight functions and
replays this batch. Reports and generated C++ remain under ignored
`local_workspace/parking-exit-analysis-all/` and
`local_workspace/batch_20261026_508_511/`. No firmware build/upload is necessary
for this analysis-only change, and no new raw camera frames exist in the batch.

Correction priority:

1. Restore robust front-left physical clearance during normal unparking;
   validate against 511 and preserve successful 508/509/510 exit cases.
2. Resolve post-registration scan-line acquisition and two-marker classification
   against 510; separately investigate gyro stream restarts without assuming a
   single cause or accepting invalid sensor evidence.
3. Add the real 508 connector deviation and 509 localized start to the normal
   approach regressions, then correct their route/reference handling while
   retaining steering, footprint, travel and terminal safety gates.

The user requested evaluation; no motion-control change is included here.
Subsequent physical validation should use normal runs and normal saving, with
actual contact/stops and direction recorded. No dedicated ToF sweep, extra
movement, pause or arbitrary-placement grid is requested.
