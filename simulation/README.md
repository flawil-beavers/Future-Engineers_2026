# Drive Simulation Folder

This folder contains the offline driving-geometry tools, their generated
visualizations, curated robot-log fixtures, and the physical reference images
used to define conservative robot envelopes. The programs support design and
verification; they do not run on the Arduino and cannot replace physical
contact testing.

## Files

| File | Description |
|------|-------------|
| `check_open_wall_following_20261028.py` | Compares 11200 wall-following commands in 560 scenarios with the pre-change Open controller and executes actual ToF filtering for first-corner timing; unresolved findings documented in `OPEN_CHALLENGE_REVIEW_20261027.md`. |
| `review_open_challenge_20261027.py` | Source-backed Open navigation audit: CW/CCW mission, frame freshness, servo resume and explicit unresolved corner/finish/mode-state findings; see `OPEN_CHALLENGE_REVIEW_20261027.md`. |
| `visualize_robot_run.py` | Whole-run SVG/PNG images with logged pillar maps, field context and parking/merge detail; usage in `ROBOT_RUN_VISUALIZATION.md`. |
| `corner_extra_view_check.py` | Additional90 mm reverse arc: both log398 scan poses,486 perturbed geometric cases, coast reserve, full-seat clearance and ideal reciprocal return; no camera or controller acceptance. |
| `parking_exit_swept_search.py` | Standard-library swept-footprint search and validation for the parking-exit manoeuvre. |
| `parking_exit_path.svg` | Generated top-down visualization of the selected parking-exit and localization path. |
| `PARKING_EXIT_PATH_SIMULATION.md` | Parking-exit coordinate system, footprint, search, selected path, limitations, and physical-validation history. |
| `analyze_parking_exit_pose.py` | Validates and analyzes automatic parking-exit pose, braking, reversal and servo-neutral diagnostics. |
| `analyze_parking_exit_batch.py` | Regenerates pose plots and reports from all archived complete parking-exit logs, with a source-hash manifest. |
| `analyze_connector_tracking.py` | Replays recorded connector tail targets and compares an offline route-continuation candidate; physical run procedure is in `../CONNECTOR_NEXT_TEST.md`. |
| `replay_green_middle_connector.py` | Replays logs 430/432 from connector start and recorded stop pose with continued route lookahead, fixed-field capsule and modeled pink-rail checks, plus an assumed pose/yaw grid. Run with `python simulation/replay_green_middle_connector.py`; it cannot prove physical clearance. |
| `connector_transition_sim.py` | Ideal closed-loop connector rollout using actual outgoing route tangents, capsule clearances and assumed sensitivity cases; mirrored cases are synthetic. |
| `analyze_discovery_trace.py` | Per-seat visibility and clear-block reasons from bounded cached first-lap discovery records. |
| `corner_reverse_view_search.py` | Offline straight-reverse view candidates from recorded holds; all 24 legal seats, front/rear capsules, 40 mm margin and 20 mm braking reserve. Dynamic yaw, images and real retrace remain unmodeled. |
| `corner_forward_view_search.py` | Forward-only first-corner arc candidates from recorded CW approach poses; all 24 legal seats, conservative front/rear envelope and assumed pose/yaw/braking deviations. No firmware control or route-join acceptance. |
| `PARKING_EXIT_DIAGNOSTIC_LOGGING.md` | Firmware switch, schema, analysis workflow, physical handoff and limitations. |
| `analyze_parking_exit_tof.py` | Laptop-only angle/target assessment automatically included in the batch command; preserves route outputs and robot driving. |
| `parking_scan_search.py` | Historical bounded search for a camera scan pose after parking exit. |
| `parking_entry_scout_sim.py` | Current log-driven replay of the preceding-station scout and its camera/wall/pillar geometry. |
| `PARKING_ENTRY_GEOMETRY_TOOLS.md` | Usage, inputs, results, limitations, and maintenance for both parking-entry tools. |
| `fixtures/parking_entry_scout/` | Tracked logs 362--369 and metadata used by the default scout replay. |
| `evidence/parking_exit/` | Straight and full-lock top-down robot photographs used to define the swept footprint. |
| `evidence/parking_exit_diagnostics/` | Immutable returned parking-exit logs and their run metadata. |

## Parking-exit model

The parking-exit search uses the Ackermann measurements maintained in `CAD/`,
but the vehicle motion model, collision search, generated path, and physical
evidence live here. See
[`PARKING_EXIT_PATH_SIMULATION.md`](PARKING_EXIT_PATH_SIMULATION.md).

Run the model from the repository root:

```powershell
python simulation/parking_exit_swept_search.py
```

The script regenerates `parking_exit_path.svg` beside itself.
That SVG is an idealized footprint model. To generate the estimated paths from
recorded robot runs instead, use:

```powershell
python simulation/analyze_parking_exit_batch.py
```

The resulting plots and source manifest are written under ignored
`local_workspace/parking-exit-analysis-all/`. See
[`PARKING_EXIT_DIAGNOSTIC_LOGGING.md`](PARKING_EXIT_DIAGNOSTIC_LOGGING.md) for
the evidence and interpretation rules.

## Parking-entry models

The current scout replay uses the tracked log fixtures by default:

```powershell
python simulation/parking_entry_scout_sim.py
```

The earlier exploratory scan search remains reproducible for design history:

```powershell
python simulation/parking_scan_search.py
```

See [`PARKING_ENTRY_GEOMETRY_TOOLS.md`](PARKING_ENTRY_GEOMETRY_TOOLS.md) before
changing constants or interpreting a PASS.

## Evidence and fixtures

Inputs are kept separate from generated outputs:

- `fixtures/` contains immutable text logs selected for a specific replay.
- `evidence/` contains physical measurements, photographs and immutable returned
  validation logs used to define or assess a model.
- Generated diagrams stay beside the script that produces them.

Every fixture/evidence directory has its own README describing provenance and
limitations. Do not silently replace an input file; add the new evidence and
update its metadata.

## Safety and maintenance

A simulation PASS means only that the modeled poses satisfy the modeled
constraints. It does not include servo lag, tire slip, camera segmentation,
localization error, battery effects, construction tolerances not represented
by the model, or the user's physical contact observation.

Whenever robot dimensions, steering geometry, firmware constants, field
coordinates, or validation logs change:

1. Update the model and its documentation together.
2. Preserve failed fixtures as regression cases.
3. Re-run the relevant scripts and affected PlatformIO environment.
4. Record the modeled minima as predictions, not measurements.
5. Upload firmware only with explicit permission.
6. Accept a manoeuvre only after physical testing confirms no prohibited
   contact.


## Full-run telemetry

See [RUN_TELEMETRY.md](RUN_TELEMETRY.md) for the bounded M7 telemetry format,
accepted route revisions, elapsed/lap timing, offline checks and normal CW/CCW
validation. `check_run_telemetry.py` compiles the actual telemetry writer against
a cached pose and bounded logger; generated fixtures stay in local_workspace.

Recorded-map inner-route refinement (2026-10-27): see
[LATER_LAP_INNER_ROUTE_REVIEW_20261027.md](LATER_LAP_INNER_ROUTE_REVIEW_20261027.md)
for the same-speed comparison, force/deviation assumptions and physical limits.
`check_later_lap_inner_routes.py` covers targeted transitions, exhaustive-clearance
equivalence and optional-planning budget fallback; generated work stays ignored.

Subsequent [full firmware review](FULL_CODE_REVIEW_20261027.md) records remaining
actuator/time-gap/logger/I2C defects and **six new dynamic reserve failures** in
the optional inner search, plus one pre-existing case. Reproduce runtime edges
with `review_runtime_edges_20261027.py`; after refreshing the normal later-lap
fixtures, `review_later_lap_matrix_20261027.py` compares all 2191 uncapped cases
with/without only that search. Its initial nonzero result documented a real
finding, not a test-infrastructure error. No firmware changes were made by the review.
The subsequent [runtime/route corrections](RUNTIME_AND_ROUTE_FIXES_20261027.md)
resolve these reproduced findings; the expanded dynamic matrix now passes.
`check_later_lap_dynamics.py --full-matrix` covers all 2191 uncapped layouts at
10/20/50ms, `check_drive_profile_timing.py` preserves the historical normal ramp,
and `check_bounded_tof_transport.py` exercises both generated vendor wrappers.

Returned pre-fix runs 531–540 and the additional parking-edge correction:
[RETURNED_RUNS_531_540_REVIEW_20261027.md](RETURNED_RUNS_531_540_REVIEW_20261027.md).
`check_parking_mixed_edge_ranges.py` exercises mixed wall/marker returns against
independent planar sensor geometry and keeps the old calculation as a negative
control. Raw evidence is preserved unchanged in the parking diagnostic archive.

Current CW layout-B failures 541–543 and the quick stationary camera check:
[CW_CORNER_541_543_FIX_20261027.md](CW_CORNER_541_543_FIX_20261027.md).
`check_cw_corner_541_543.py` covers the archived association geometries,
mirrored joins and equivalent recovery clearance decisions; runtime-edge tests
also cover consecutive slow-loop profile progress.

CCW 544–546, the user-repaired sensor contact, late corner recognition and
USB saving: [CCW_544_546_REVIEW_20261027.md](CCW_544_546_REVIEW_20261027.md).
`check_corner_red_544.py` reproduces the relevance rejection and tests its bounded
correction; `check_logger_directory_save.py` checks complete-file saving/retries.

CW/CCW547--555: [CW_CCW_547_555_REVIEW_20261027.md](CW_CCW_547_555_REVIEW_20261027.md).
`check_cw_ccw_547_555.py` covers edge aiming, scan/connector launch, distant-middle
conflict, measured exit tails, watchdog progress and parking equivalence.

CW556/558/559 and CCW560--562:
[GREEN_556_562_REVIEW_20261027.md](GREEN_556_562_REVIEW_20261027.md).
`check_cw_556_558.py` covers row resolution, middle aliases, false distant RED,
retained far acquisition and early CW route geometry, including negative controls.

CW563--566 / CCW567--569:
[CW_CCW_563_569_REVIEW_20261027.md](CW_CCW_563_569_REVIEW_20261027.md).
`check_green_563_566.py` couples first-corner acquisition to its unknown map and
tests third-GREEN viewing/capture alignment. `check_ccw_567_569.py` covers early
returning RED and independently sensed parking drift. CW565's unknown station6
hold remains specifically pending physical revalidation.

Follow-up scope correction for that hold:
[CORNER_HOLD_565_FIX_20261027.md](CORNER_HOLD_565_FIX_20261027.md).
`check_corner_hold_565.py` exercises actual RGB565 local silhouette detection,
the coverage gates, the old-veto negative control and the unchanged safe join.

Final parking in both directions after the third lap:
[FINAL_PARKING_CAPTURE_REVIEW_20261027.md](FINAL_PARKING_CAPTURE_REVIEW_20261027.md).
`check_parking_direct_capture.py` compares the checked direct capture with the
previous alignment, retains the short CW recovery and tests the new states'
health gates. Full sequence, robustness and braking checks cover completion
and bounded rejection; physical validation remains pending.

Complete approaches from recorded CW564/CCW567 arrivals, including modeled
brake coast and ambiguous early wall references:
[PARKING_ARRIVAL_AUDIT_20261028.md](PARKING_ARRIVAL_AUDIT_20261028.md).
Run `check_parking_arrival_564_567.py` after regenerating the later-lap fixture.

CW570--571 / CCW572--575 acquired views, CAD recovery and stopped parking wall:
[CW_CCW_570_575_REVIEW_20261029.md](CW_CCW_570_575_REVIEW_20261029.md).
Run `check_cw_570_571.py` and `check_ccw_572_575.py`; originals and SHA-256
metadata are in the parking-exit evidence directory. Physical acceptance of
the new revision remains pending normal runs.

CW576 false middle GREEN and skipped start scout:
[CW_576_START_REGRESSION_20261029.md](CW_576_START_REGRESSION_20261029.md).
`check_cw_576.py` tests the actual scout transition with a removed-guard negative
control; `check_green_seat_visibility.py` checks the missing mapped-foot gate,
archived-image comparisons and a second negative control.

CCW577--581 successful course runs versus pre-scan holds:
[CCW_577_581_REVIEW_20261029.md](CCW_577_581_REVIEW_20261029.md).
`check_ccw_580_581.py` covers the measured refused poses, shorter safe arc
selection, subsequent views/joins and the old-search-floor negative control.
Final parking is deferred for this batch.

CW582--586 and returned CCW/CW587--593 distant start evidence:
[RETURNED_RUNS_582_593_REVIEW_20261030.md](RETURNED_RUNS_582_593_REVIEW_20261030.md).
`check_green_563_566.py` now includes the disputed 582/583 row views;
`check_parking_seat_foot.py` preserves the false471/576 controls while admitting
the returned end-GREEN residual; `check_green_occupancy_hold_589.py` verifies
that one matched upright candidate cannot become CLEAR 192 ms later. Colour
thresholds, blob dimensions, speeds, parking and later-lap geometry are unchanged.

First-lap unknown-station recovery:
[LAP1_ACTIVE_SEARCH_REVIEW_20261030.md](LAP1_ACTIVE_SEARCH_REVIEW_20261030.md).
`check_active_lap1_search.py` verifies that a missed station alternates the two
swept-safe reverse parallax views and continues fresh observation instead of
entering an evidence-only permanent lock. Motion-safety stops remain active.

Final parking after three laps, direct staging and pose-based continuation:
[FINAL_PARKING_POSE_REPLAN_REVIEW_20261030.md](FINAL_PARKING_POSE_REPLAN_REVIEW_20261030.md).
`check_parking_final_arrivals_20261030.py` compares six archived arrivals with
the learned bypass and exercises the measured wall residuals.
`check_parking_adaptive_completion.py` uses recorded entry anchors, both turns
and a plant radius different from the planning model, including final wheel
centering and recovery health checks. Physical acceptance remains pending.

Camera observations during the unchanged parking exit:
[PARKING_EXIT_CAMERA_REVIEW_20261030.md](PARKING_EXIT_CAMERA_REVIEW_20261030.md).
`check_parking_exit_observer.py` verifies historical-pose interpolation, later
XY corrections, conservative multi-frame seat import and fallback. It also
reports possible view windows from the tracked exit logs; synthetic projections
are not proof of detection in real camera images.

Tight final parking completion and 10 mm end-clearance review:
[FINAL_PARKING_TIGHT_COMPLETION_REVIEW_20261030.md](FINAL_PARKING_TIGHT_COMPLETION_REVIEW_20261030.md).
`check_parking_tight_completion.py` reproduces the neutral-wheel search rejection
and containment/verification loop, including separate negative controls.
`check_parking_adaptive_completion.py` now covers five measured gap sizes.

Returned CW594--597 / CCW598--601:
[RETURNED_594_601_REVIEW_20261031.md](RETURNED_594_601_REVIEW_20261031.md).
`check_returned_594_601.py` exercises the logged end-place aliases, next-colour
selection, confirmed CCW refinement and unknown CW scout retry.
`check_parking_returned_594_601.py` covers stopped wall registration and a
checked neutral scan alternative. Adaptive entry additionally supports
`--extended-radius`; tight completion includes a sub-degree final correction.

Returned603 and direct entry practice:
[review](RETURNED_603_AND_PARKING_PRACTICE_20261031.md),
[Aufstellung und Bedienung](PARKING_ENTRY_PRACTICE_DE.md).
`check_returned_603_and_parking_practice.py` checks clipped-depth association,
wall/rail shortcut equivalence, actual entry-only startup and603 replanning.
Use `--without-clipped-depth-gate` for the wrong-end negative control.

CCW604--606: [retreat and rear-space review](RETURNED_604_606_REVIEW_20261031.md).
`check_returned_604_606.py` checks the14mm-reserve escape and the angled rear-ToF
reference, including rejected stale/ambiguous input and front-crowded parking.

CW607--609 completed entry practice; small rearward adjustment and restored
full-course startup: [handoff](RETURNED_607_609_FULL_RUN_HANDOFF_20261031.md).

Later-lap direct connections, clearance policy and lap/parking transitions:
[review](LATER_LAP_DIRECT_LANES_20261102.md).
`check_later_lap_direct_lanes.py` compares the actual route builder with and
without the new connections, including exhausted optional-search budgets.
Use `check_final_parking_approach.py --approach-only` for the route handoff and
`check_final_parking_sequence.py` for the current adaptive entry planner.

Lap 1 -> 2 pursuit/speed handoff: [review](LAP_HANDOFF_20261102.md).
`check_lap_handoff.py` checks same-pose target continuity and slow discovery
exits. `check_later_lap_dynamics.py --outgoing-speed 135` additionally checks
the real motor ramp from a slow lap-1 exit; the default is 260 mm/s.

Parking duration from the registered capture pose:
[audit](PARKING_DURATION_AUDIT_20261102.md), `audit_parking_duration.py`.
Separates motor-ramp travel and programmed settling from unmeasured MCU planning
time; `--nominal` emits the small nominal comparison and detailed short-plan log.

Shorter entry pauses with observed standstill and reused servo-command age:
[review](PARKING_SETTLE_REUSE_20261102.md), `check_parking_steering_wait.py`.
Duration sensitivity: `audit_parking_duration.py --filtered-speed`; append
`--nominal --legacy-waits` for the previous waits with the same plant.
