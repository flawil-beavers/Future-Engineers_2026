# Obstacle pillar and wall clearance logging

This document explains the clearance reports emitted by the live Pure Pursuit
test. The reports answer three different questions:

| Report | Question answered |
| --- | --- |
| `[CLEARANCE PLAN]` | How much clearance did the injected Pure Pursuit route request? |
| `[CLEARANCE ODOM]` | How much clearance did the estimated robot trajectory achieve? |
| `[PILLAR TOF]` | What minimum side range did the physical ToF beam observe? |

The values must be compared, not averaged. Physical contact always invalidates
a run, even if the firmware reports `PASS`.

## Passage window

Each candidate pillar seat has an independent passage accumulator. Samples are
collected from 300 mm before to 300 mm after the seat's baseline path distance.
Adjacent stations are 500 mm apart, so their windows intentionally overlap.
Only confirmed pillars are printed.

For a `Y3` or `Y-3` test, each confirmed seat prints a separate passage block
for `lap=1`, `lap=2`, and `lap=3`. Its ToF and ODOM minima are reset after each
complete passage, so the three blocks can be compared directly. Lap 1 reports
the dynamically injected discovery route. Laps 2 and 3 report the validated
later-lap route built after lap 1.

`complete=yes` means the robot travelled beyond the complete +300 mm side of
the window. `complete=no` means the report was flushed because the test stopped
or aborted first.

## Robot and field geometry

PLAN and ODOM use the same conservative complete-robot safety envelope. It is
a capsule with:

- a 70 mm radius, based on the maximum steered-wheel half-width;
- its axis starting at the rear-axle midpoint;
- its axis ending 60 mm forward of the rear axle;
- a resulting conservative rear reach of 70 mm;
- a resulting front reach of 130 mm, matching the measured front plane.

The pillar is represented by the official 85 mm movement circle, with radius
42.5 mm. The geometry therefore calculates:

```text
pillar clearance = distance(pillar centre, robot capsule axis)
                   - 70 mm robot radius
                   - 42.5 mm movement-circle radius
```

The field model contains four outer wall segments at `x/y = +/-1500 mm` and
four inner wall segments between the inner corners at `x/y = +/-500 mm`.
Wall clearance is calculated as:

```text
wall clearance = distance(robot capsule axis, wall segment)
                 - 70 mm robot radius
```

A positive result is separated geometry. Zero is contact in the model. A
negative result means the safety envelopes overlap.

## `[CLEARANCE PLAN]`

Example:

```text
[CLEARANCE PLAN] seat=6 snapshot=route-activation
pillar_min_mm=50.3 pillar_pose=-780,-633@67.3
wall_min_mm=138.1 wall_feature=inner_west_face
wall_point=-500,-484 wall_pose=-730,-540@68.1
inner_corners_SW_SE_NE_NW_mm=138.7/1138.2/1470.1/711.2
envelope=capsule_r70_rear-axle_to_front-plane
```

On lap 1, the route is snapshotted immediately after that seat's avoidance
geometry is actually injected. A deferred pillar is snapshotted when its route
becomes active, not when the camera first confirms it. When the optimized
later-lap route is activated at the end of lap 1, its PLAN snapshots replace
the lap-1 snapshots so lap-2 and lap-3 reports describe the route actually in
use. Lap 2 and lap 3 share that later-lap route.

The calculation evaluates the Pure Pursuit path points inside the passage
window. Path points are spaced approximately 50 mm apart. Heading at each
point is derived from its neighbouring points before applying the robot
capsule.

- `seat`: canonical seat number.
- `snapshot=route-activation`: confirms that PLAN belongs to the route active
  for that passage: the injected lap-1 route or the optimized later-lap route.
- `pillar_min_mm`: smallest planned pillar clearance in the window.
- `pillar_pose=x,y@heading`: planned rear-axle pose at that pillar minimum.
- `wall_min_mm`: smallest planned clearance to any modelled wall.
- `wall_feature`: wall associated with the minimum. Values include
  `outer_south`, `inner_west_face`, and `inner_corner_SW`.
- `wall_point=x,y`: closest point on that wall or corner.
- `wall_pose=x,y@heading`: planned robot pose at the wall minimum. It does not
  normally equal `pillar_pose` because the two minima are independent.
- `inner_corners_SW_SE_NE_NW_mm`: independent minimum clearance to each of the
  four inner corners over the passage. These four minima can occur at different
  path points.

PLAN describes intended geometry. It cannot show path-tracking error, initial
placement error, wheel slip, odometry error, or the pillar's exact position
inside its legal movement circle. Because it checks discrete path samples, it
is a conservative diagnostic target but not a mathematical continuous swept-
path proof.

## `[CLEARANCE ODOM]`

Example:

```text
[CLEARANCE ODOM] seat=6 samples=3396
pillar_min_mm=-7.7 pillar_pose=-802,-462@112.4
wall_min_mm=210.4 wall_feature=inner_corner_SW
wall_point=-500,-500 wall_pose=-788,-582@82.1
inner_corners_SW_SE_NE_NW_mm=210.4/1209.6/1534.9/824.0
envelope=capsule_r70_rear-axle_to_front-plane
```

Every control-loop pose inside the passage window is evaluated using the same
capsule, pillar movement circle, walls, and inner corners as PLAN.

`samples` is the number of control-loop geometry samples, not the number of
camera or ToF frames. The pose comes from the position estimator, which uses
encoder/gyro motion and any accepted field-wall ToF corrections.

ODOM estimates the driven trajectory relative to the canonical seat and field.
It is deliberately conservative. A negative ODOM value can coexist with a
safe physical pass if pose error, pillar placement, or the rounded capsule
model overstates overlap. Physical observation and ToF evidence are needed to
interpret it.

## `[PILLAR TOF]`

Example:

```text
[PILLAR TOF] seat=6 color=RED sensor=L
window_mm=-300..+300 complete=yes
fresh=139 valid_raw=24 valid_filtered=24 inset_mm=35.0
raw_min_mm=50.0 raw_clearance_est_mm=15.0
filtered_min_mm=50.0 filtered_clearance_est_mm=15.0
wall_sensor=R wall_fresh=139
wall_raw_min_mm=267.0 wall_raw_clearance_est_mm=232.0
wall_filtered_min_mm=267.0 wall_filtered_clearance_est_mm=232.0
estimate=range-minus-max-steered-wheel-inset
```

Both ToFs are accumulated independently. After legal avoidance:

- a red pillar is on the robot's left, so the left ToF is the pillar sensor;
- a green pillar is on the robot's right, so the right ToF is the pillar
  sensor;
- the opposite sensor is reported as the expected wall-side sensor.

Each ToF aperture is 35 mm laterally from the robot centre. The conservative
wheel envelope reaches 70 mm, leaving a 35 mm aperture-to-envelope inset:

```text
ToF clearance estimate = measured surface range - 35 mm
```

The ToF measures the physical pillar surface, so the 42.5 mm movement-circle
radius is not subtracted again.

- `fresh`: unique sensor sequences seen in the passage window.
- `valid_raw`: quality-accepted raw samples at or below the 600 mm reliable
  range.
- `valid_filtered`: valid production-filtered samples.
- `raw_min_mm`: minimum accepted direct range.
- `filtered_min_mm`: minimum filtered range. It is steadier but can lag because
  production filtering limits rapid changes.
- `raw_clearance_est_mm` and `filtered_clearance_est_mm`: range minus the
  35 mm wheel-envelope inset.
- `wall_*`: equivalent minima for the opposite sensor.

ToF is the most direct physical evidence, but it is still a narrow beam at a
fixed longitudinal position. It can miss the closest wheel/body point, see an
opening at a corner, or occasionally see another object rather than the
expected wall. Agreement between raw and filtered minima, continuous motion,
and visual clearance is strongest.

## Interpreting the three reports together

| Evidence | Likely interpretation |
| --- | --- |
| PLAN is small or negative | The commanded route geometry itself is unsafe. |
| PLAN is safe, ODOM and ToF are smaller | The robot cut inside the planned route, or placement/localization differed. |
| PLAN and ODOM agree, ToF is smaller | The physical pillar may be displaced toward the robot, or the beam saw a closer surface. |
| ODOM is negative, but ToF and visual evidence are safe | The capsule or pose estimate is conservative/inaccurate for that instant. |
| ToF is near zero or negative and motion stalls | Strong physical collision or entrapment evidence. |
| Pillar margin is small while wall margin is large | The path can potentially move away from the pillar. |
| Both pillar and wall margins are small | Do not increase displacement without redesigning the transition. |

For current 175 mm/s development, approximately 30 mm or more repeatable
physical/ToF pillar clearance is the practical robustness target. Values from
0 to 20 mm are marginal even if one run succeeds. This is an engineering test
target, not a competition-rule threshold.

## Worked result: `log_85`

The CW extreme-adjacent test used red seat 6 followed by green seat 9:

- Red injected at 200 mm.
- Green confirmed as deferred and activated at 210 mm after the red release.
- The lap completed without contact, intervention, or a hidden speed stall.
- Maximum CTE was 100.0 mm and maximum heading error was 54.4 degrees.

Red reported 50.3 mm PLAN clearance, -7.7 mm ODOM capsule clearance, and
15 mm ToF wheel-clearance estimate. The user observed approximately 20 mm of
physical space. PLAN therefore requested a useful margin, but the actual robot
cut roughly 35 mm inside that intended pillar clearance. The opposite ToF still
estimated 232 mm wall clearance.

Green reported 31.2 mm PLAN clearance, -33.4 mm ODOM capsule clearance, 57 mm
ToF pillar clearance, and 153 mm ToF wall clearance. Its physical pass was
successful. This is a concrete example of why the conservative ODOM capsule
must not override direct physical and ToF evidence by itself.

## Second result: `log_86`

The unchanged repeat also completed without contact or intervention. Red
reported 50.3 mm PLAN, 38.2 mm ODOM, and 73 mm ToF clearance. Green looked close
to the user and reported 31.2 mm PLAN and 28 mm ToF clearance, so the visual
observation and physical beam agreed closely with the intended path. Its wall-
side ToF still estimated 171 mm clearance. Green ODOM was -30.5 mm despite the
safe pass, again illustrating the conservative-model limitation.

The maximum heading error reached 68.0 degrees. That does not indicate contact,
but it shows that the adjacent 610 mm lateral reversal is aggressive. More
pillar displacement is not automatically safer if the resulting transition
becomes harder for Pure Pursuit to track.
