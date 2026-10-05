# Drive Simulation Folder

This folder contains the offline driving-geometry tools, their generated
visualizations, curated robot-log fixtures, and the physical reference images
used to define conservative robot envelopes. The programs support design and
verification; they do not run on the Arduino and cannot replace physical
contact testing.

## Files

| File | Description |
|------|-------------|
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

