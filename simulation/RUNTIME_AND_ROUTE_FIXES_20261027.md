# Runtime and route fixes, 27 October 2026

We addressed the specific faults reproduced in the [full code review](FULL_CODE_REVIEW_20261027.md). The results below apply to the reviewed October firmware. We received no new physical runs or original logs during this review. The four previously reported CW runs still needed to be matched to their evidence.

## What we changed

| Area | Change |
| --- | --- |
| Steering after a gyro hold | Restore steering writes when the controller accepts a straight or curved reverse viewpoint manoeuvre, including extra viewpoints. The four moving Open states also restore steering. Geometry and sensor-health checks still reject unsuitable commands. |
| Viewpoint timing | Exclude a real gyro hold once from the phase clock. On a transport restart, the hold starts before blocking sensor initialisation. |
| Motor profile after a pause | Restart from zero without turning time already elapsed into a jump in target distance. After an active loop gap above **100 ms**, reset to measured motion and the latest encoder position. This adds no lasting drive lockout. |
| USB saving and a new start | Queue a start until the old save finishes. A LOW switch or explicit stop cancels the request. During speed-controlled motion the logger performs no file operations. |
| USB filename search and RAM | Check at most **four** names per update. After a failed save, keep the snapshot in RAM even if the next run calls `clear()`. |
| M4 software I²C | Report data-bit and ACK clock failures as transaction errors. Return only a complete, valid register read. |
| VL53L4CX transport on both cores | Limit register addressing to **two attempts** so one transient NACK can recover. Report empty or short reads and failed write chunks through the existing sensor error path. |
| Open corner confirmation | Count only new, valid ToF sequences from each side. Use the side that met the confirmation threshold to choose the turn. |
| Optional inner routes on laps 2 and 3 | Evaluate candidates over two laps with the drive controller's acceleration ramp. Keep the existing route if a candidate fails a check or exceeds the planning budget. |

The maximum requested speed stayed at **520 mm/s**, with the same acceleration and jerk settings. We increased the later-lap braking preview from **0.35 s** to **0.40 s**. With the existing **10%** speed allowance at that speed, the change adds up to **28.6 mm** of preview and covers the previously failing dynamics case 154. Straight-line speed limits stayed the same.

The optional route check keeps nominal and perturbed starts (**±10 mm**, **±1°** and **10%** extra speed in the perturbed cases), a **40 mm** modelled wall reserve and the existing body clearance checks. New candidates require **32 mm** of obstacle clearance, compared with the general **30 mm** acceptance limit, to account for the time-step resolution. The general lateral acceleration limit stays at **0.8 m/s²**. The search keeps the cooperative **30 ms** budget and the controller can continue on the existing route.

We kept the existing colour thresholds, sign-slot association, first-lap geometry, parking and exit paths and their speeds. We kept unused legacy code outside this functional change. The review left potential DMA and gyro timestamp issues open for physical validation.

## Reproducible checks

Run the scripts with the IDE-managed Python from the repository root. Generated C++, executables and unprocessed reports stay under `local_workspace/`.

| Check | Result |
| --- | --- |
| `review_runtime_edges_20261027.py` | Passed pauses of **215, 819 and 1,200 ms** without a target-distance jump, the measured-motion reset and next normal ramp, all four accepted recovery moves and rejection of an invalid move. Also passed the queued start, cancellation on LOW and single start, **16** data-bit and **2** ACK timeouts and recovery, **534** files in **134** updates with **535** name probes, RAM retention after a failed save, fresh sequences and timer rollover. |
| `check_drive_profile_timing.py` | **40,000** comparisons against `drive_loop()` in `861ed8e` at **1, 10, 20 and 50 ms**. Alternating positive and negative requests with acceleration limits of **0, 60, 120, 500 and 1,000** gave **0 mm/s** speed difference. Arduino `constrain` behaviour for calibration limits matched. |
| `check_bounded_tof_transport.py` | The generated wrappers for both installed drivers passed **96-byte** reads and writes, persistent and transient address faults, empty chunks, missing bytes, partial writes, failures in intermediate chunks and recovery. |
| `check_runtime_safety.py` | Gyro and ToF faults and recovery, start switch, parser, pose, vision, dispatch and parking pause checks passed. |
| `check_later_laps.py` | **2,807/2,807** geometry cases passed. |
| `review_later_lap_matrix_20261027.py` | **2,191/2,191** at **20 ms** passed, with and without the optional inner search. The earlier variants had seven and one violations. Both revised variants had zero. |
| `check_later_lap_dynamics.py --full-matrix` | **6,573/6,573** dynamic cases passed. The same **2,191** uncapped maps ran at **10, 20 and 50 ms**. |
| `check_later_lap_inner_routes.py` | Passed **336** comparisons of the fast and full geometry checks, **4** budget and rollover cases and **8** transitions per comparison variant. The targeted inner route saved **118.92 mm** and **0.562 s** of calculated target time per lap in CW and the mirrored case. |
| `check_first_lap_geometry.py` | **1,736/1,736** passed. Minimum modelled obstacle clearance was **47.657 mm** CW and **47.6197 mm** CCW. Camera behaviour requires its own validation. |
| `check_final_parking_robustness.py` | **1,540** scenarios produced no unsafe result in the model. All **990** known regular approaches parked. The controller rejected some or all phases of unsuitable starts. Another **46** boundary and **5** measurement-quality checks passed. |
| `check_parking_segment_sensor_resume.py` | **42** checks across seven segments in both directions passed. The negative control reproduced the old failure. |
| `check_gyro_path_servo_resume.py` and `check_connector_servo_resume.py` | Passed, including negative controls with the steering restoration removed. |

| Control step | Cases | Maximum lateral acceleration (m/s²) | Minimum obstacle clearance (mm) |
| --- | ---: | ---: | ---: |
| 10 ms | 2,191 | 0.760805 | 32.1480 |
| 20 ms | 2,191 | 0.768285 | 32.2451 |
| 50 ms | 2,191 | 0.796430 | 33.4597 |

The matrix includes all seven exact sign layouts recorded in the original code review. The route logic treats them like any other layout.

## How the build applies the ToF change

[PlatformIO](../platformio.ini) runs [`bounded_tof_transport.py`](../scripts/bounded_tof_transport.py) for both cores. The middleware replaces two VL53L4CX I/O function bodies with calls to [`bounded_tof_transport.h`](../include/bounded_tof_transport.h). The remaining vendor implementation and licence stay in place. Generated sources go to `local_workspace/generated_tof/<environment>/` and objects go to `.pio/build/<environment>/`. The installed library files, package cache and dependency versions stay in place. An unexpected vendor source change produces a build error.

We bounded the number of bus calls made by our wrappers. The test does not measure the time taken by an individual hardware `Wire` call. The revised drive profile remains comparable with the separate reference constants in the existing tool. With the same braking preview applied to both sides of the seven-layout comparison, CW example A saved **405.27 mm** per lap (**16.9407 to 15.4049 s** of calculated target time). Mirrored CCW example C saved **193.27 mm** (**16.5581 to 14.9777 s**). These are model results.

## Build and physical validation

Both IDE-managed PlatformIO builds compiled and linked with the existing cache. Existing macro and library warnings remained.

| Environment | RAM (bytes) | Flash (bytes) |
| --- | ---: | ---: |
| `giga_r1_m7` | 442,408 / 523,624 | 524,296 / 786,432 |
| `giga_r1_m4` | 59,768 / 294,248 | 155,328 / 1,048,576 |

SHA-256 of `.pio/build/<environment>/firmware.bin`:

```text
giga_r1_m7 5542896c58ca0c36b2c389c0be2d63471b2a0dbeeb27a1cbedee4b44ab7b22d8
giga_r1_m4 fbc3da65845172d9be1405ec56b626da1b716999ce2fd31e591a4c3a14057215
```

M7 changed for control and planning, M4 for software I²C, and both for ToF transport. The RPC protocol and layout remained compatible. We did not upload during this review. Normal CW and CCW runs on the identified firmware were the planned validation for steering response, traction, lighting, mounting and camera timing. The plan was to preserve and hash complete originals before investigating the green-sign and parking problems. Normal runs were sufficient for this check.
