# Shorter waits between parking moves, 2 November 2026

After [auditing the parking duration](PARKING_DURATION_AUDIT_20261102.md), we reduced the wait between moves when the steering had already settled or the car had come to a measured stop. The reviewed revisions were `20261102_reuse_steer_settle` for parking and `20261102_lap_handoff` for the route. The results below come from the host model.

## When the next move can start

1. **Reuse steering settle time.** We record the angle and time of the servo command when the write actually goes out. A queued or blocked write leaves the record unchanged. If the next move uses exactly the same angle, it can reuse the time already spent settling. Every new angle still gets **400 ms**. The servo keeps receiving PWM when `stop()` blocks new writes. The controller observes elapsed time because the servo has no position feedback.
2. **Start the timer at the servo write.** When the controller enters the settle phase, it sends the checked angle immediately and starts timing from that write. Clearance checks stay active before and during steering adjustment.
3. **Verify an early stop.** At stops after approach, a segment or replanning, the parking controller can move on after at least **150 ms** of braking and **120 ms** of observed stillness. The encoder may change by at most **0.5 mm** and the heading by at most **0.25°**. The controller also needs two fresh speed readings and a current speed at or below **5 mm/s**. Movement restarts the observation window. A sensor hold or a reset clears it. Without that evidence the existing **300 ms** path still checks speed and enforces its timeout.

The route, geometry, target pose, clearance allowances, search and exit diagnostics stayed the same in this revision. Entry speeds remained **60 or 40 mm/s**. Braking during the other approach and scan phases retained its existing timing.

## Timing results

We compared identical starting poses, geometry and sensor inputs with the same acceleration profile and speed filter. The `--legacy-waits` option restores the earlier waits in the model. CW and CCW times match for mirrored inputs.

| Bay gap and starting pose | Earlier waits | Revised waits | Time saved |
| --- | ---: | ---: | ---: |
| 232.5 mm | 15.08 s | 13.84 s | 1.24 s |
| 245.2 mm, recorded starting pose | 15.34 s | 13.90 s | 1.44 s |
| 247.5 mm | 12.24 s | 11.42 s | 0.82 s |
| 251.7 mm | 12.20 s | 11.42 s | 0.78 s |
| 256.6 mm, recorded starting pose | 12.20 s | 11.48 s | 0.72 s |
| 262.5 mm | 12.20 s | 11.42 s | 0.78 s |

The filtered matrix ranged from **10.40 to 19.28 s**. The timings exclude MCU search time, approach and marker measurements, USB saves, unknown pose errors, tyre slip and actuator variation. The change shortens the waits between moves. Geometric replanning can still add computation time.

## Reproduction and build

`check_parking_steering_wait.py` checks the production servo and parking logic. It covers unknown angles, blocked and queued writes, the **399/400 ms** boundary, repeated commands and timer rollover. It also checks CW and CCW starts and finishes, minimum braking time, missing samples, encoder and heading movement, excessive or invalid speeds and sensor holds. All checks passed.

| Additional check | Result |
| --- | --- |
| Full parking and registration | **396** complete entries and **5** registration cases passed. |
| Braking and sensor health | **84** braking overshoot variants up to **3 mm** and **8** health checks passed. |
| Clearance and progress | **14** checks and **36** full sequences with radius and heading changes passed. |
| Timing matrix | **216/216** completed without modelled overlap, repeated successfully with the production encoder speed filter (**50 ms** minimum window and **0.60** alpha). |

From the repository root, run:

```powershell
python simulation/audit_parking_duration.py --filtered-speed
python simulation/audit_parking_duration.py --nominal --filtered-speed --legacy-waits
```

The output directories are `local_workspace/parking-duration-filtered/` and `local_workspace/parking-duration-nominal-filtered-legacy/`.

The IDE-managed `giga_r1_m7` build passed. RAM usage was **459,256 / 523,624 bytes**, flash usage **572,912 / 786,432 bytes** and binary size **576,464 bytes**. SHA-256:

```text
03fadef7953736ca4b8fb3e220300dfadc5e6c60457c31344f3713cb78870d2b
```

M4 was unchanged. This review produced no upload or physical run. Normal runs with this revision were the planned check for actual segment and braking times, `search_ms`, braking overshoot, contact and clearance. The team's [current parking approach](../../README.md#obstacle-challenge-strategy) reflects subsequent revisions.
