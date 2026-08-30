# Drive Simulation Folder

This folder contains the offline driving-geometry tools, their generated
visualizations, curated robot-log fixtures, and the physical reference images
used to define conservative robot envelopes. The programs support design and
verification; they do not run on the Arduino and cannot replace physical
contact testing.

## Files

| File | Description |
|------|-------------|
| `parking_exit_swept_search.py` | Standard-library swept-footprint search and validation for the parking-exit manoeuvre. |
| `parking_exit_path.svg` | Generated top-down visualization of the selected parking-exit and localization path. |
| `PARKING_EXIT_PATH_SIMULATION.md` | Parking-exit coordinate system, footprint, search, selected path, limitations, and physical-validation history. |
| `parking_scan_search.py` | Historical bounded search for a camera scan pose after parking exit. |
| `parking_entry_scout_sim.py` | Current log-driven replay of the preceding-station scout and its camera/wall/pillar geometry. |
| `PARKING_ENTRY_GEOMETRY_TOOLS.md` | Usage, inputs, results, limitations, and maintenance for both parking-entry tools. |
| `fixtures/parking_entry_scout/` | Tracked logs 362--369 and metadata used by the default scout replay. |
| `evidence/parking_exit/` | Straight and full-lock top-down robot photographs used to define the swept footprint. |

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
- `evidence/` contains physical measurements or photographs used to define a
  model.
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

