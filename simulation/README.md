# Driving simulations and test evidence

We use these offline checks to test sign recognition, measure route clearance
in the model and compare parking paths with recorded runs. The models use
measurements and photographs of the robot to estimate the space taken up by
the chassis and wheels. We check for actual contact during physical runs.

## Start here

| What you want to do | Where to start |
| --- | --- |
| Check camera recognition | [Sign recognition](#sign-recognition-check) |
| Check later-lap routes | [Route clearance](#route-clearance-check) |
| Replay the parking manoeuvres | [Parking exit](#parking-exit-model) · [Parking entry](#parking-entry-models) |
| Inspect a physical run | [Original logs and hashes](evidence/parking_exit_diagnostics/README.md) · [Run inspector](../tools/robot-run-inspector/README.md) |
| Find a dated engineering report | [Report index](reports/README.md) |

## Files

Submission-video rendering, sensor graphics and run633 footage synchronization
are documented in [VIDEO_VISUALIZATION.md](VIDEO_VISUALIZATION.md).

| File | Description |
|------|-------------|
| `check_open_wall_following_20261028.py` | Compares 11200 wall-following commands in 560 scenarios with the pre-change Open controller and executes actual ToF filtering for first-corner timing; unresolved findings documented in [the Open Challenge review](reports/OPEN_CHALLENGE_REVIEW_20261027.md). |
| `review_open_challenge_20261027.py` | Source-backed Open navigation audit: CW/CCW mission, frame freshness, servo resume and explicit unresolved corner/finish/mode-state findings; see [the Open Challenge review](reports/OPEN_CHALLENGE_REVIEW_20261027.md). |
| `visualize_robot_run.py` | Whole-run SVG/PNG images with logged pillar maps, field context and parking/merge detail; usage in `ROBOT_RUN_VISUALIZATION.md`. |
| `corner_extra_view_check.py` | Checks an additional 90 mm reverse arc from both log 398 scan poses across 486 geometric cases. Covers coast reserve, sign-position clearance and ideal return geometry. Camera and controller acceptance require separate checks. |
| `parking_exit_swept_search.py` | Standard-library swept-footprint search and validation for the parking-exit manoeuvre. |
| `parking_exit_path.svg` | Generated top-down visualization of the selected parking-exit and localization path. |
| `PARKING_EXIT_PATH_SIMULATION.md` | Parking-exit coordinate system, footprint, search, selected path, limitations, and physical-validation history. |
| `analyze_parking_exit_pose.py` | Validates and analyzes automatic parking-exit pose, braking, reversal and servo-neutral diagnostics. |
| `analyze_parking_exit_batch.py` | Regenerates pose plots and reports from all archived complete parking-exit logs, with a source-hash manifest. |
| `analyze_connector_tracking.py` | Replays recorded connector tail targets and compares an offline route-continuation candidate. See the [physical run procedure](../docs/validation/CONNECTOR_NEXT_TEST.md). |
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

## Sign recognition check

Run from the repository root with Python and a C++17-capable `g++` on your `PATH`.

```powershell
python simulation/check_surprise_rule.py
```

This checks recognition against approved camera images and exercises the
passing rules for modified signs. On November 22, the check passed 45 approved
camera frames, 648 rendered cases, 48 assignments, 32 policy checks and
16 frame sequences. [Camera validation report](reports/SURPRISE_RULE_MEASURED_VALIDATION_20261119.md#verification)

## Route clearance check

Use the same tools from the repository root. The first command generates the
later-lap test cases. The second measures the clearance between the modelled
chassis and wheels and the signs and inner wall.

```powershell
python simulation/check_later_laps.py
python simulation/check_later_lap_physical_clearance.py
```

On November 22, the planner passed all 2,811 known-map cases. The second
check also tests additional corner layouts and uses a 50 mm engineering
target. Seven of its 2,827 cases fell below this target. The smallest
sampled gaps were 43.8 mm to a pillar and 51.1 mm to the inner wall.
Both checks assume a correct sign map and ideal pose tracking. The
[November 20 clearance study](reports/LAP23_CLEARANCE_REVIEW_20261120.md#validation)
documents the earlier comparison and model assumptions.

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

Test the current four-move entry and its approach from the repository root
with Python and a C++17-capable `g++` on your `PATH`.

```powershell
python simulation/check_final_parking_sequence.py
```

On November 22, all 396 modelled approaches reached the entry position.
Separate cases executed all four moves in both directions, tested the
bounded slip allowance and checked registration against the bay markers.
The model assumes known field geometry. Compare with the [physical tests](evidence/parking_exit_diagnostics/README.md#2026-11-22-log-48),
or follow the [isolated practice guide](../docs/guides/PARKING_ENTRY_PRACTICE.md) to test
the manoeuvre on a robot. The [November 19 stress test](reports/CW_LOGS_3_6_REVIEW_20261119.md#verification)
covers the earlier adaptive parking controller.

The earlier scout replay uses the tracked log fixtures by default:

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

See the [dated reports and earlier reproduction notes](reports/README.md) for the history of these checks.
