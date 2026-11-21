# Later-lap clearance, 19–20 November 2026

For laps 2 and 3, we aimed for about 50 mm between the nearest point on the
chassis or wheels and the inner wall or a 50 × 50 mm sign. The planner uses
50 mm as a route target. The existing runtime stop checks still apply.

## Evidence and finding

We verified the six archived USB originals against the source before analysis.
The source contained the same logs. The results below come from replay and
simulation. [Verification manifest](evidence/log_intake/20261119_batch_3_6/later_lap_clearance_source_verified.json) · [Original logs](evidence/parking_exit_diagnostics/README.md)

The original later-lap route passed the collision and passing-side checks in
all 2,807 replay cases. The minimum distance from the diagnostic capsule to a
sign was 32.126 mm, and 164 cases fell below 50 mm. The wall minimum was
73.300 mm. This check uses a capsule with a 70 mm half-width and a 42.5 mm
sign movement circle. The separate polygon check uses the chassis and wheels
against the nominal 50 mm square sign.

The independent polygon check found a minimum modelled clearance of 43.143 mm.
The tightest cases involved a single middle sign in the start section on the
return approach in both directions. We tried increasing the ordinary-section
middle offset from 190 to 205 mm and shortening the pursuit horizon. Both
experiments left the start-section shortfall unresolved.

## Retained change

After selecting the later-lap route, we add a local shift on the required
passing side around a confirmed solitary sign in the middle of the start
section. At cyclic distance $d$ from the sign, the shift is
$20\sin^2(\pi d/400)$ mm for $d < 400$ mm. It peaks at 20 mm, 200 mm before
and after the sign, and falls smoothly to zero at the sign and the 400 mm
boundary. The sign is at the lap seam with the current 500 mm first-corner
reference. The mapped effective colour determines the passing side.

The planner applies the shift once after optional shortening and connectors.
It checks the walls, signs and wheel clearance at the magenta markers. It
requires at least 60 mm of sampled wall clearance, allows up to 20 mm of
reduction from the previous wall clearance, and allows at most 1 mm of
reduction in the previous sampled sign clearance. If a check fails, it
restores the already checked route. It then recalculates the route-dependent
dynamics. The change applies only to the later-lap route.

The transition keeps the measured-distance blend over 400 mm. We checked
18 transitions covering the previous layout B, both sign colours in the
start-section middle position, both directions and incoming speeds of
135, 180 and 260 mm/s. The steering target began without a jump in all
18 model cases. Actual servo motion was outside this comparison.

## Validation

| Check | Recorded result |
| --- | --- |
| `check_later_laps.py` | All 2,807 route and controller cases passed, including passing sides on both laps. The capsule minima were 73.300 mm at the wall and 41.550 mm at a sign. The cases include repeated speed labels and ordered neighbouring sections. |
| `check_later_lap_physical_clearance.py` | All 2,807 cases met the 50 mm model target. The minimum polygon clearances were **53.969 mm at a sign** and **66.897 mm at the inner wall**. |
| `check_later_lap_physical_clearance.py --without-reserve` | With only the new shift disabled, 19 of 24 focused cases fell below 50 mm. The minimum was 43.143 mm. |
| `check_lap_handoff.py` | All 18 cases passed with zero initial target jump. |
| `check_direct_parking_handoff_20261119.py` | 24 maps, 20 certified preparations and 12 CCW rolling approaches, with zero failures. The stopped CW fallback remained available. |
| `check_final_parking_approach.py --approach-only` | All 2,970 approaches passed on the changed map planner. |

The polygon audit uses the measured 123 mm wheel width, 35 mm rear
overhang, 98 mm wheelbase, and front extents from the rear axle of
123 mm straight and 135 mm steered. It checks the chassis, centre extension
and all four wheels with straight steering and at both steering extremes.
It measures clearance to the nominal sign squares and the full inner-wall
square, including corners. It uses exact edge and overlap checks for nearby
objects and skips distant objects only beyond a conservative bound.

The highest-speed sample step is 20 ms, or about 11.44 mm of travel. The audit
assumes precise poses, sign positions and calibrated steering angles. Physical
clearance depends on pose error, tyre slip, sign placement, steering response
and motion between samples. A new layout may retain the earlier checked
route if the shift fails its checks. [Reproduce the clearance check](README.md#route-clearance-check)

## Build and follow-up

The M7 path revision was `20261119_lap23_reserve`, and the discovery speed
remained 120 mm/s. The IDE-managed build succeeded using 452,176 of
523,624 bytes of RAM and 601,816 of 786,432 bytes of flash. The 605,368-byte
binary had SHA-256
`560961a45da4816ac08f689b70f4521119b34ef15da47a6a640445e053ef3f19`.
Existing Serial macro and unused-helper warnings remained. This review
covered the M7 build and host checks.

The remaining physical checks for this revision were actual wheel clearance
and the transition from lap 1 to lap 2 in ordinary runs in both directions.
[Current engineering handoff](../AGENT_DOCUMENTATION.md)
