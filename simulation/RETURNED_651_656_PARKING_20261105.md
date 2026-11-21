# Parking runs 651–656 and the measured steering footprint

We used six physical runs to review final parking and update the footprint
used by the planner. The original files are preserved with byte counts and
SHA-256 hashes. Both USB batches were copied and verified before analysis.
[Original logs](evidence/parking_exit_diagnostics/README.md)

Run `python simulation/analyze_returned_651_656.py` from the repository root
to regenerate the summaries under `local_workspace/returned-651-656/`.
The exit analyser stores its reports in
`local_workspace/parking-exit-analysis/20261105_651_656/`.

## What the returned runs establish

All six logs report build `Nov5_2026_20_16_44` with an 80 mm/s parking exit
and 100 mm/s long arc. Completed navigation runs report path revision
`20261105_physical_lap_speeds` and parking revision
`20261105_efficient_alignment`.

| Run | Logged result and physical observation |
| --- | --- |
| 651 CW | Completed three laps and reported parking complete at local X/Y 86.8/116.4 mm. The quick search took 1,364 ms and the later broad search took 3,595 ms. The team reported poor or incomplete physical parking. |
| 652 | The team interrupted the run. |
| 653 | The team reported a slippery surface. |
| 654 CW | Completed three laps. The first steering arc had a much larger observed radius, clamped to 1.5 times the model radius. Replanning stopped with `entry_no_collision_free_plan` at local X/Y 87.4/206.8 mm and heading 60.7°. |
| 655 CCW | Reported parking complete in a measured 247.3 mm gap. The quick search took 1,309 ms and the controller updated the radius and remaining path several times. The team reported good use of space and a shallow final position. |
| 656 CCW | Reported parking complete in a measured 247.4 mm gap. The quick search took 1,321 ms with several radius and remaining-path updates. The team also reported good use of space and a shallow final position. |

Wall registration changed the estimated lateral position by **48.1 mm** in
run 655 and **62.4 mm** in run 656. We use the measured marker and wall
positions to refine the pose before planning the entry. The parking handoff
took **202–243 ms** in this batch. Its effect on the physical transition
remained open at this revision.

## Measured geometry and implementation

| Measurement | Distance |
| --- | ---: |
| Physical wheelbase | 98 mm |
| Width across straight wheels | 123 mm |
| Rear overhang from the rear axle | 35 mm |
| Front overhang from the front axle, straight / full lock | 25 / 37 mm |
| Front extent from the rear axle, straight / full lock | 123 / 135 mm |
| Total length, straight / full lock | 158 / 170 mm |

The steering-radius model uses a **127 mm effective wheelbase**.
[Steering calibration](../CAD/ACKERMANN_KINEMATICS_DOCUMENTATION.md)

**Checking the footprint.** The previous base polygon reached about 125 mm
ahead of the rear axle, and its optional 16 mm nose allowance disappeared in
the tight 1 mm recovery mode. The revised model includes the measured
135 mm steering contour at every margin, with a further 4 mm preferred
allowance. We use the wheel angles from the lookup table and conservatively
enclose the measured reach and rear tyre sweep. The default exit and lap
polygons matched their previous versions bit for bit.

A car at an angle occupies a different length and width along the bay. The
planner projects the chassis and wheels against the measured rails and wall
at each point along the path, including steering changes while stationary.
Full steering increases front reach and changes the width occupied by the tyres.

**Scanning and finishing.** We moved the scan and capture axle line from
294.6 to 271.5 mm from the wall. This leaves 10 mm between a straight wheel
and the 200 mm rail-tip plane. The preferred final axle position is
128.5 mm from the wall, leaving 10 mm to the open edge
(`200 − 128.5 − 123/2` mm). Deeper checked recoveries remain available.
The controller stops once the pose is parallel and contained. The stationary
pose and planned endpoint require 1 mm beyond the 5 mm boundary margin.
The moving early stop and the longitudinal completion check each use an 8 mm
reserve.

**Entry planning.** We check the full path after each candidate 20, 32 or
40 mm straight retreat. Normal registration planning adds 1.5 mm per
polygon or piece, even in tight mode, and fades that margin in over 20 mm.
This lets a current pose move out of a tight spot. Short and analytic
sequences are tried first. If they fail, a stopped multi-point search tries
both directions, steering extremes, straight and angle-alignment moves,
and travel available at both ends. It uses 512 static nodes, up to 512
expansions and 65,000 safety samples, and up to 16 compressed segments.
Each candidate receives the normal 1 mm swept-path check with radius
variation. A reused path keeps its planned endpoint. The search uses
20,480 bytes of scratch space. Recovery is limited to the cases the search
can find and verify.

**Speed at this revision.** The approach increased from 100 to 120 mm/s and
alignment from 80 to 100 mm/s. Marker sampling stayed at 60 mm/s. Normal
entry used 60 mm/s, and tight entry used 40 mm/s. The scan start moved from
680 to 650 mm. Early outward moves still require checks for the full
footprint and lateral uncertainty. The brake, servo readiness and sensor
health checks still apply.

## Validation and delivery

| Host check using production code | Recorded result |
| --- | --- |
| Complete parking and registration | 396 of 396 sequences and five registration cases with independent simulated and estimated poses. |
| Learned-route approaches | 2,970 of 2,970 passed. |
| Adaptive braking and health checks | 84 of 84 brake-overrun cases and eight gyro or servo health cases passed. |
| Measured footprint | All 24 cases passed in both directions using 248.9 and 249.6 mm gaps, initial yaw of ±1° and actual radius factors of 1, 1.15 and 1.5. The model checked for overlap along the path. |
| Parallel stop | 16 cases passed, including servo-age and brake verification checks. |

The geometry checks confirmed the full 135 mm nose at a 1 mm margin and
preserved the default polygons. Generated outputs are stored in
`local_workspace/`.

At radius factors of 1 and 1.15, the modelled entry took about 11.4–12.5 s
over 307–334 mm. At 1.5, recovery took up to approximately 21.3 s over 495 mm.
These estimates include configured movement, braking and servo wait times.
Physical time and clearance still depend on controller computation, the tyre
contour and grip under load.

The IDE-managed M7 build succeeded, using 480,696 of 523,624 bytes of RAM
and 576,320 of 786,432 bytes of flash. The 579,872-byte binary for
`20261105_measured_parking` had SHA-256
`5dbbf880f26137561d8108501900e6063d57fd9466e6f591f6a4a8dbd98f2844`.
This was a build and host review. M4 and robot upload were outside this
revision.

The physical follow-up for this revision was to compare approach time,
scan registration, requested and measured curvature, and any recovery
in normal CW and CCW runs. The checks included front clearance and the
actual final position. [Current engineering handoff](../AGENT_DOCUMENTATION.md)
