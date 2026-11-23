# Review of runs 531–540, 27 October 2026

## Run evidence

These ten runs preceded the M7 and M4 corrections under review. We copied each original before analysis and compared its SHA-256 hash with the USB copy. The archive records the receipt date because the USB filesystem reported timestamps in 2098. All ten logs contain `RUN_START`, `RUN_END` and `truncated=0`. Build timestamps differ by module, so the logs do not identify a single firmware hash. [Original logs, sizes and hashes](evidence/parking_exit_diagnostics/README.md)

Runs 531–534 were clockwise (CW) and match the order and outcomes of the earlier set of four. Runs 535–540 were counter-clockwise (CCW). We could not assign individual reports of CCW contacts or interventions to particular logs.

## What the logs show

| Log | Observation |
| --- | --- |
| 531 CW | Exited the bay and completed three laps. The parking scan found both markers and rejected a **289.3 mm** gap with `dual_marker_geometry`. |
| 532 CW | S3, station 2 remained unresolved. The recovery requested **−70 mm/s** while servo writes were disabled. The car moved about **2.2 mm**, reached the time limit and held. |
| 533 CW | Exited the bay and reached final parking. The completed scan rejected a **285.6 mm** gap. |
| 534 CW | Exited the bay and reached final parking. The completed scan rejected a **284.5 mm** gap. Several gyro restarts extended the run. |
| 535 CCW | Exited and joined the route. Confirmed GREEN 5, GREEN 7 and RED 9, then stopped near **(638 mm, −541 mm, 41°)** with a target of **252 mm/s**. |
| 536 CCW | Similarly stopped after joining the route, near **(637 mm, −544 mm, 42°)** with a target of **252 mm/s**. |
| 537 CCW | Joined the route and confirmed RED 10. Later stopped near **(1,231 mm, 450 mm, 64°)** with a target of **260 mm/s**. |
| 538 CCW | We manually disabled the drive after **2.356 s** during rear positioning. The log ends before the parking exit or an autonomous abort. |
| 539 CCW | Joined the route and later stopped near **(860 mm, −703 mm, 28°)** with a target of **100 mm/s**. The camera saw a large unassigned green region near the bottom of the image. |
| 540 CCW | Reached the return side and confirmed several sign colours. Stopped near **(−1,115 mm, −490 mm, 275°)** with a target of **256 mm/s**. |

In runs 535, 536, 537, 539 and 540, the motor controller recorded only **5.9–9.1 mm** of progress over **1,000 ms**. Drive and servo writes were still enabled, which rules out a deliberate stop to collect camera evidence. The logs lack full images and independent position measurements. Contact, obstructions, pose error and drivetrain problems remain possible, so we left the HSV thresholds and drive effort unchanged.

Run 532 shows the restart fault already reproduced in the earlier review. The correction restores steering writes for accepted reverse and return commands and pauses the phase timer during a gyro hold. We kept the existing manoeuvre geometry. The old logs also show about **1.6 s** of interruption per gyro restart. New runs were needed to measure the frequency after the fix.

## Projecting the parking marker edges

The original code used the last short range at the fixed marker and the first short range at the movable marker as the length of the ToF sensing cone. At either edge, the beam can include reflections from the marker and the wall. In run 533, the range at the end of the second marker was **171 mm**, compared with about **62–80 mm** over the marker and **255–280 mm** at the wall. With an **11°** cone half-angle, using **170 mm** instead of **60 mm** increases the combined edge correction by about **42 mm**. That is consistent with the **37–42 mm** overestimate in the rejected gaps.

We introduced `markerEdgeFootprint()` to intersect both edges of the sensing cone with the marker-tip plane at **outer wall Y + parking width**. It uses the sensor position and the heading. The range readings still confirm the order of wall and marker observations. The edge projection uses the geometry of the tip plane. The function rejects non-finite poses and rays pointing the wrong way. Two `EDGE` messages and `geometry=tip_plane_v1` identify it in a normal log.

We kept the **±15 mm** gap tolerance, position correction limit, braking, capture, seven entry segments, speeds and collision checks. The earlier radial calculation remained in the independent host fixture for comparison.

The old parking traces contain samples about every **200 ms**, so the exact measurements used at the transitions are missing. The reproduced geometry error fits the log pattern. The available logs cannot establish whether the correction would have let all three entries finish. Physical clearance still depends on the sensor cone, reflections, marker position and tyre slip.

## Verification and build

| Check | Result |
| --- | --- |
| `check_parking_mixed_edge_ranges.py` | **864** cases passed. The test independently builds the tip plane and covers both directions, **±3°** heading, scan distances, **±180 mm** position offsets and mixed returns of **0, 150, 170 and 180 mm**. Gap error stayed below **2 mm**, and true **280 mm** gaps were rejected. The frozen old calculation rejected valid gaps in **162** of the matching **170 mm** cases. Invalid poses and ray directions were also tested. |
| `check_final_parking_robustness.py` | **1,540** scenarios had no unsafe model result. All **990** known approaches parked. Another **46** boundary and **5** measurement-quality cases passed. |
| `check_final_parking_sequence.py` | **396** full sequences and **5** registration cases passed, including sensor holds. |
| `check_parking_segment_sensor_resume.py` | All **42** cases and the negative control passed. The models omit measured hardware delay. |
| Exit pose and ToF analysis | All ten originals were analysed under `local_workspace/batch_20261027_531_540/pose/`. Nine exits were complete and untruncated. Run 538 remains in the record as interrupted. The rear encoder/ToF difference had a median of **+1.51 mm** and a maximum of **9.35 mm**. All nine were within the existing uncertainty, so we kept the backlash and steering centre settings. |

The additional geometry fix affected M7. The earlier M4 update still applied if it had not been installed. The IDE-managed M7 build passed with **442,408 / 523,624 bytes** of RAM and **524,712 / 786,432 bytes** of flash. We did not rebuild the unchanged M4 or upload either image during the review.

```text
giga_r1_m7 d3f65f7fad458849edd2c0b192b6b265b7b5d33ee868695945b81a1d00a32c30
giga_r1_m4 fbc3da65845172d9be1405ec56b626da1b716999ce2fd31e591a4c3a14057215
```

The runtime checks passed again for all four recovery releases, the gyro hold timer, the drive profile reset, USB coordination and I²C error reporting. The planned follow-up was to archive and hash the next normal CW and CCW runs, then compare edge values, actual parking position and behaviour at the earlier stops.
