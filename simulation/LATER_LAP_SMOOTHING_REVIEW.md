# Conservative route refinement for laps 2 and 3

Implemented and checked on 2026-10-08. This refines the complete learned map
once at the lap-1 handover. Laps 2 and 3 use the same stored refined route.

## Shape and acceptance

- Four symmetric five-point refinement passes reduce bends at transitions.
  Smooth saturation limits total displacement from the checked original to
  **less than 6 mm per point**. Existing path indices and baseline distances
  remain intact.
- The full 400 mm lap-seam protection/blend region stays unchanged, with a
  gradual transition outside it. Confirmed pillar passing plateaus within
  150 mm longitudinal distance also stay unchanged; refinement fades in
  over the next 150 mm.
- An optional candidate must be no longer than the prior route (0.1 mm
  numerical tolerance), reduce discrete bending by at least 2%, and introduce
  no larger peak curvature (1e-7 numerical tolerance).
- The complete vehicle route must pass the existing wall, pillar and pink-bay
  footprint checks. Candidate minimum wall/pillar geometry reserves must be
  at least 30 mm and lose no more than 1 mm against their prior minima.
- Rejecting this optional refinement restores the already checked route;
  it adds no new stopping state. The official passing policy remains active.
  The check-all surprise mode does not use this optional refinement.
- No speed limits, acceleration settings, discovery manoeuvres or parking
  entry segments were changed. The existing route-dependent speed profile is
  recalculated after shape selection.

## Calculations

`check_later_laps.py` extracts the production planner/controller into its CAD
fixture. `measure_later_lap_smoothing.py --baseline` disables only this optional
refinement in that same extracted planner. A normal measurement enables it.
Results and generated fixtures remain in ignored `local_workspace/later-laps/`.

The bending measure is sum(turn-angle squared / mean adjacent segment length).
It measures route sharpness, **not measured servo jerk or lap time**.

| Existing physical layout | Route shortening per lap | Bending reduction |
| --- | ---: | ---: |
| A CW | Prior route retained | 0% |
| A CCW | 22.25 mm | 3.27% |
| B CW | 19.11 mm | 7.68% |
| B CCW | Prior route retained | 0% |
| C CW | Prior route retained | 0% |
| C CCW | 22.20 mm | 5.84% |

Across 2,807 input cases, 1,406 receive a useful refinement. In those cases,
mean shortening is 20.21 mm/lap and mean bending reduction 5.90%; maxima are
47.83 mm and 17.34%. Peak curvature increases in none. Counts include repeated
speed-label cases; they are not 2,807 unique complete fields.

## Verification and limitations

- Later-lap state/geometry/controller replay: 2,807 cases, zero failures.
  Correct full-body pillar passing is checked in both later laps. Lowest
  simulated wall reserve is 80.281 mm; lowest pillar reserve 36.306 mm.
- Parking approach regression on the refined map: 2,970 cases pass. All
  nominal entry preflights pass; six tight perturbed configurations are safely
  rejected among 162 entry checks.
- Complete parking state-machine regression: 396 cases all park fully
  contained, with no modeled collision or false completion. Five marker
  registration acceptance/rejection checks also pass.
- Final IDE-managed M7 build succeeds, RAM 432880/523624 and flash
  502680/786432 bytes. Firmware SHA-256:
  `30b47894838f0e0a425d87a2e81214b781bacde8bd2bcb4b7fe56d54b344ccb4`.
  M4 unchanged; no upload or agent commit.

CAD tracking assumes correct map/pose and calibrated steering, without wheel
slip, sensor errors, servo lag or brake dynamics. It does not establish the
physical clearances or eliminate the pre-scan localization and gyro failures
recorded in the previous investigation. The benefit is deliberately modest;
there is no basis to promise a large lap-time improvement.

Next: user uploads M7, drives B CW twice and C CCW twice through all three laps
and parking, observing corner transitions and pillar clearance in laps 2/3.
Save normal complete logs; archive originals before analysis. Compare the
`[LAPS] Smooth route` or `Smoothing rejected` record with the physical report.
