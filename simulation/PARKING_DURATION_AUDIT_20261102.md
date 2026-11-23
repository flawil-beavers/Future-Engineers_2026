# Parking duration and clearance, 2 November 2026

We compared the adaptive parking controller's planned and executed movements in the host model. The reviewed revisions were `20261102_lap_handoff` for the route and `20261101_efficient_entry` for parking. We made no firmware changes or uploads and received no new physical runs or USB logs. The [current parking sequence](../README.md#obstacle-challenge-strategy) uses a later design.

## Modelled parking duration

The tested CW and CCW cases completed without a collision in the model. The table shows elapsed time from the registered stop at the bay (`FP_CAPTURE_BRAKE`) to confirmed parking. It includes acceleration, braking and steering waits. It excludes MCU computation, USB saving and the approach and marker scan.

| Bay gap | Nominal time | Executed segments / arcs | Range across variants |
| --- | ---: | ---: | ---: |
| 232.5 mm | 15.08 s | 6 / 3 | 14.04–21.44 s |
| 245.2 mm, recorded starting pose | 15.34 s | 7 / 3 | 12.48–16.54 s |
| 247.5 mm | 12.24 s | 6 / 4 | 12.12–16.40 s |
| 251.7 mm | 12.20 s | 6 / 4 | 12.06–15.98 s |
| 256.6 mm, recorded starting pose | 12.20 s | 6 / 4 | 12.00–16.28 s |
| 262.5 mm | 12.20 s | 6 / 4 | 10.94–15.98 s |

The 216 cases combined six bay gaps, two driving directions, actual radius factors of **1, 1.136 and 1.235**, initial heading offsets of **−1.5°, 0° and +1.5°**, and added braking travel of **0 or 1.5 mm**. Mirrored CW and CCW inputs gave the same results. The planner starts with radius factor 1 and estimates the radius from movement. The initial heading offset is visible to the pose estimator. Unknown gyro errors and tyre slip fall outside this model, and 21.44 s is the largest result in this matrix.

## Where the time goes

In the 251.7 mm example, the car travels about **333.8 mm**. At a steady 60 mm/s that distance would take 5.56 s. The model includes acceleration and stops between short movements.

| Phase | Modelled time |
| --- | ---: |
| Driving, including acceleration and the control interval | 7.26 s |
| Six steering waits | 2.40 s |
| Braking, including the initial stop | 2.12 s |
| Final pose check and centring the steering | 0.42 s |
| **Total, excluding computation** | **12.20 s** |

The normal entry runs at **60 mm/s** and the tight-clearance fallback at **40 mm/s**. Each new steering angle receives **400 ms** to settle. At the end of each segment the controller waits at least **300 ms** and checks measured speed. We kept those waits pending actuator measurements.

At **245.2 mm**, the standalone planner produced five segments with two arcs and one direction change. In the full model, the controller selected the faster search routine with a **1 mm** modelled margin and drove at **40 mm/s**. Its clearance check requested an `entry_motion_clearance` correction before the second arc. The car then executed seven segments with three arc starts. The full model took **15.34 s**. The standalone plan gives the planned sequence only.

## How the footprint limits movement

The model uses **123 mm** overall width, **30 mm** behind the rear axle and **125 mm** ahead of it. It also checks the individual steered wheels, both parking markers and the outer wall. In the normal margin mode it prefers **16 mm** of space at the front while steering. The earlier additional **10 mm** rear allowance remains disabled. The local endpoint remains **60 / 127 mm**.

At heading angle $a$, the rectangular body alone occupies $155\cos(a) + 123|\sin(a)|$ mm along the bay. At **45°** it spans about **196.6 mm**, leaving about **50.9 mm** in a **247.5 mm** bay before accounting for wheel sweep, reserve and stopping distance. We check clearance along the whole move at millimetre intervals and calculate the allowable forward or reverse travel from the current pose.

## Verification and limits

| Host check | Result |
| --- | --- |
| `audit_parking_duration.py` | **216/216** completed with no overlap in the checked vehicle outlines. Used the production `DriveProfile` ramp at a 20 ms step. |
| `check_final_parking_sequence.py` | **396/396** entries completed on this revision and five marker registration checks passed. |
| `check_parking_efficient_entry.py` | **14** clearance and progress checks passed. **36/36** full sequences completed with radius, heading and braking variations. |

The test adapter reads production source and uses one encoder adapter, which keeps the route fixture in sync with the controller. Run `python simulation/audit_parking_duration.py --nominal` from the repository root for the nominal timing breakdown. The full run writes `report.json` and `output.txt` under `local_workspace/parking-duration/`.

The cost function weights travel, segment count and direction changes. Its **10 mm** penalty per segment corresponds to **0.167 s** at 60 mm/s, compared with about **0.7 s** for braking and steering. These figures identify a parameter to investigate using complete trajectories. The fallback search limits the number of geometry checks and has no guaranteed MCU-time bound. The pauses of up to about **10 s** in logs 633 and 634 came from an earlier controller. The planned physical follow-up for this revision was to compare `search_ms`, executed segments, clearance and total parking time during normal runs.
