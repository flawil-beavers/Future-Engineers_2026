# Shorter, faster recorded-map laps (2026-10-18)

This supersedes the 390 mm/s speed setting and the 6 mm-only refinement in
`LATER_LAP_SMOOTHING_REVIEW.md`. Changes apply to recorded-map laps 2/3.
The default configuration continues through final parking; neither lap-test
switch nor the parking-entry arm was changed.

## Route selection

Once lap 1 resolves the map, try symmetric five-point smoothing with displacement
limits 40/20/10/6 mm and 24/12/6/4 passes, largest first. Each trial starts from
the same checked route. Accept the first shorter, smoother candidate satisfying
the existing full-vehicle wall, pillar and pink-piece preflight. Otherwise retain
the checked prior route. This is bounded local optimization, not a proof of the
globally shortest collision-free path.

The 400 mm seam region, 150 mm pillar passing plateaus, passing side, waypoint
indices and baseline station distances remain protected. Candidates must reduce
discrete bending by at least 2%, increase neither length (0.1 mm numerical
tolerance) nor peak curvature, keep at least 30 mm modeled wall/pillar reserve,
and lose at most 1 mm against the preceding route's minimum reserves. These
reserves are beyond the existing body/wheel and pillar-movement envelopes;
they are not a measured pose-error bound. The comparison below uses the earlier
6 mm refinement as its baseline, so some small bending changes relative to
that different accepted candidate can still occur.

## Force and speed calculation

All firmware quantities use mm and seconds. In SI units:

```
a_lateral = v^2 / R
F_lateral = mass * a_lateral
v_command <= sqrt(0.8 * R / 1.15) / 1.10       [R in metres]
v_command <= 0.520 m/s
```

The engineering lateral budget is **0.8 m/s^2**, about **0.082 g** or
**0.8 N per kilogram**. The 1.10 speed tolerance and 1.15 curvature tolerance
are simultaneous multiplicative allowances, not measured estimator accuracy.
Mass cancels from the friction-limited speed calculation; an absolute force
cannot be given without weighing the car.

| Turn radius | Maximum command after both allowances |
| --- | ---: |
| 150 mm | 293.7 mm/s |
| 180 mm | 321.7 mm/s |
| 200 mm | 339.1 mm/s |
| 300 mm | 415.3 mm/s |
| 500 mm | 520 mm/s (straight-speed ceiling applies) |

As a conditional plausibility check, assuming tyre/mat friction coefficient
mu >= 0.20 and using at most half of mu*g gives 0.981 m/s^2 available combined
acceleration. The selected 0.8 lateral budget and existing 0.5 longitudinal
motor limit combine to sqrt(0.8^2+0.5^2)=0.943 m/s^2. **The friction assumption
has not been measured.** No mass, centre-of-gravity height or support-polygon
measurement is available, so this is not a verified grip or tipping limit.
The static tipping condition is a_lateral < g*support_width/(2*CG_height);
the outside-wheel envelope must not be substituted for support width.

Speed planning uses circumcircle curvature over 150 mm of route on each side,
matching the spatial smoothing of the existing 182–280 mm pursuit horizon.
Treating every 50 mm polyline vertex as an instantaneous steering angle caused
unnecessary braking and slower nominal laps in an intermediate calculation;
that intermediate profile is not the delivered version. Every actual control
update additionally limits speed using both requested pursuit curvature and
the quantized CAD steering command. Invalid numeric requests command zero.

The existing linear curvature profile, multiplied by 2.00 rather than 1.50,
provides an additional ceiling. Limits are cached separately; route speed
metadata used by discovery/parking keeps its original meaning.

## Braking and acceleration

For every future curve, entry speed is limited by
`v_entry^2 <= v_curve^2 + 2*180*usable_distance`. Distances are the actual
shortened-route segment lengths. Before counting useful braking, reserve
`520*1.10*0.35 + maximum_actual_waypoint_interval`: 350 ms response at the
upper tolerated speed, plus one complete actual waypoint interval (at least
50 mm, giving at least 250.2 mm total reserve). Two cyclic forward sweeps limit planned acceleration
to 180 mm/s^2. Both calculations cross the lap seam. The existing motor jerk
and acceleration controller remains in charge of actual motion.

For example, slowing 520 to 339 mm/s needs approximately 432 mm of useful
braking at 180 mm/s^2; including response/waypoint reserve gives at least 682 mm.
Actual pursuit corrections can request a tighter turn than predicted. The
runtime curvature cap lowers the command immediately, but cannot guarantee
instantaneous physical deceleration. Motor-ramp checks below cover that lag
for the known layouts; disturbances are still a physical-test limitation.

## Comparison with the preceding firmware

Reference: checkout `e2d7027`, 390 mm/s maximum and the 6 mm refinement.
Times below integrate the target speed along the planned path. They exclude
sensor corrections, actual servo response, unplanned holds and parking.

| Existing layout | Extra shortening/lap | Previous target time | New target time | Reduction |
| --- | ---: | ---: | ---: | ---: |
| A CW | 0 mm | 19.48 s | 16.85 s | 13.5% |
| A CCW, legal mirror | 58.67 mm | 19.61 s | 18.07 s | 7.9% |
| A CCW, same physical field | 58.84 mm | 19.61 s | 17.92 s | 8.6% |
| B CW | 23.80 mm | 20.28 s | 17.89 s | 11.8% |
| B CCW | 0 mm | 19.20 s | 16.71 s | 13.0% |
| C CW | 0 mm | 19.90 s | 17.49 s | 12.1% |
| C CCW | 63.50 mm | 19.12 s | 16.46 s | 13.9% |

Across 2,191 uncapped cases, mean additional shortening is 18.51 mm/lap,
maximum 142.74 mm; mean target-time reduction 15.17%. Range is -3.43% to
+25.02%: some tight layouts deliberately run slightly slower. The full 2,807
case set also contains repeated 140 mm/s cap labels and is not an exhaustive
set of unique whole fields. This is not a measured lap-time improvement.

## Verification and reproduction

- `check_later_laps.py`: 2,807 state/geometry/CAD-tracking cases pass, using
  11.44 mm steps (520 mm/s, 20 ms, plus 10% speed tolerance). Minimum modeled
  wall/pillar reserves: 80.281 / 35.301 mm. Correct full-body passing is checked
  in both later laps. The prior 8 mm baseline also passed 2,807 cases.
- `check_later_lap_dynamics.py`: analytic radius/sign/nonfinite/cap/CAD/seam
  and duplicate-point checks pass. Seven A/B/C maps at 10/20/50 ms, 21 two-lap
  rollouts, all pass with the actual extracted motor `drive_loop` speed ramp.
  Maximum modeled lateral acceleration 0.6177 m/s^2, maximum speed 523.9 mm/s,
  minimum pillar reserve 35.578 mm. This assumes ideal tracking of the motor
  ramp and calibrated CAD steering; tyre slip/servo lag are not simulated.
- Final-parking approach: 2,970 cases pass; 156/162 perturbed entry preflights
  pass, with the same six conservative rejections and all nominal entries clear.
- Final-parking sequence: five registration checks and 396 complete sequences
  pass, no modeled collision or false completion.

Run scripts with the IDE-managed Python and host g++ on PATH. All generated
fixtures, executables, reports and comparison data remain in ignored
`local_workspace/`. For an exact before/after comparison, save the old generated
`check.cpp` and old `config.h` before changing the checkout, then run:

```
python simulation/check_later_laps.py
python simulation/check_later_lap_dynamics.py
python simulation/measure_later_lap_dynamics.py --reference-fixture local_workspace/later-lap-optimization/before.cpp --reference-config local_workspace/later-lap-optimization/config_before.h
```

No firmware upload or new physical evidence is part of this change. Next use
normal cable-free three-lap B CW / C CCW runs through parking and normal save.
Observe passing clearance, smooth transitions and lap timings. Archive unchanged
complete logs, hashes, firmware identity and physical reports before analysis.
No dedicated ToF sweep, added movements or pauses are needed. Actual grip,
clearances and speed improvement remain to be confirmed on the robot.
