# Later-lap path recovery after CW log 7

## Evidence

The new original was copied from USB and SHA-256 verified before reading history or interpreting it. The user confirmed lap 1 drove well and reported widespread pillar contact in lap 2. The original is 92,663 bytes with SHA-256 `87ee5ae283a27d90e53e19cb04244c5b89b0c5dc685e6b8a24a0ed51b03aa177`. The intake verified six earlier originals as well. [CW log 7](../evidence/parking_exit_diagnostics/20261120_log_7_cw.txt), [intake manifest](../evidence/log_intake/20261120_lap23_regression/manifest.json)

The installed firmware reports `20261120_lap1_visibility` and 120 mm/s discovery pace. This matches the revision of the preceding prepared build with SHA-256 `4eb1a305d1f4b338310b7fc12f77d2533045bef73dd885affff10c7efa8c9158`. The installed flash hash was unavailable. The complete log reports `truncated=0`. Lap 1 took 58.248 s. The user stopped 12.281 s into lap 2. The preceding stall message reports only 1.5 mm per 1001 ms at a 305 mm/s target. There is no lap 3 or parking outcome in this run.

The learned effective map is seats 0 RED, 4 GREEN, 7 RED, 10 RED, 14 RED, 21 GREEN. Seat 10 was recorded once as a visible 50 mm GREEN with the original RED passing direction. The previous planner logged smoothing from 6,946.2 to 6,858.2 mm and an accepted direct connector down to 6,706.3 mm. It spent 58.8 ms on connectors and 203 ms on the full handoff build. There is no accepted optional inner-search record. The estimated poses, occasional slip messages and ToF correction counts can guide comparisons. They do not independently identify the contact points or isolate a single physical cause.

## Route change

Remove the direct cubic replacement of intermediate route spans. Restore the older 0.01 rad limit on extra cumulative turn for the optional inner search. Keep already checked same-colour connections, local smoothing, the existing shorter CW end bypass and the official start-middle clearance check. Increase the route construction offset 190 to 210 mm for an isolated moderate-position middle sign that needs the extreme lane, and 200 to 215 mm for a far inside GREEN on the CW start section when it has no special adjacent-pair policy. These are path offsets referenced to a pillar centre, not physical edge gaps. The resulting route must pass the existing full-footprint preflight.

The 520 mm/s ceiling, speed scheduler, turn and braking limits, protected start seam, 400 mm handoff blend, first-lap recognition, effective surprise-sign colour and parking control retain their previous behavior. The speed schedule is recalculated on the selected geometry. The new route reports `clearance_policy=20261120_lap23_passing_reserve`.

## Exact-map comparison

The source-derived controller comparison runs two laps on the layout above with the optional inner-search budget already exhausted. Nominal CW capsule clearance to the closest pillar goes from 48.3 to 56.9 mm. The separate model using measured wheel and body polygons gives 62.6 to 69.4 mm to the closest pillar and 92.2 to 97.3 mm to the inner wall. In the adverse tested 20 mm lateral and 3° heading bias combination, capsule clearance goes from 16.8 to 23.5 mm. All seven nominal and biased CW trials complete the two simulated laps on the revised route. The preceding route fails the full parking-piece guard in the +35 mm/+3° trial. Both versions complete the equivalent mirrored trials. The most adverse mirrored case has slightly lower clearance with the revised route, 20.5 versus 20.9 mm.

The revised exact-map CW route is 6,898.9 mm. The estimated time integrated over its speed targets is 16.28 s versus 15.00 s on the shortcut. That calculation excludes physical disturbances, planner latency and the approach to parking. It measures the cost of restoring clearance, without a new speed restriction. The controller model assumes the given effective map and CAD steering. Error-injection tests do not reproduce actual tyre slip, actuator lag or individual impacts in log 7.

## Validation

- Route and correct passing side verified in 2,809 known-map cases, including the logged CW map and its CCW reversal. The measured-body polygon model passes all 2,809 cases. Across that nominal set the minima are 54.0 mm to a pillar and 66.9 mm to the inner wall.
- Motor-ramp and steering simulation passes all 6,579 runs across 10, 20 and 50 ms control intervals. Peak modeled lateral acceleration was 0.665 m/s² against the existing 0.800 m/s² budget.
- Lap 1 to 2 handoff 18/18 and lap 3 parking handoff 24/24, including 12 rolling approaches.
- IDE-managed M7 build succeeded, 600,520-byte firmware with SHA-256 `53d3c85dc52cca2beba98a3f7e75ced5192356045336a2e86ecfe998c13efc51`.

The next ordinary run should use the log 7 CW layout and verify front and rear wheel clearance at the far start GREEN and the opposite middle RED. Compare both lap transitions and the later-lap policy marker before tuning the new route further.
