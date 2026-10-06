# Optimized laps 2 and 3 (2026-10-06)

## Current run selection

The default Obstacle run is now **exit, discovery lap 1, recorded-map laps 2
and 3, final stop and USB log save**. No upload has been performed.

| Switch in `include/config.h` | Current value | Effect |
| --- | --- | --- |
| `OBSTACLE_FIRST_LAP_TEST_ENABLED` | `false` | Allow laps 2 and 3 |
| `OBSTACLE_THREE_LAP_TEST_ENABLED` | `true` | Stop after three laps, before final parking |
| `OBSTACLE_STARTUP_CHECK_ALL_STATIONS` | `false` | Official section rules; serial `O` selects this too |
| `OBSTACLE_FINAL_PARKING_ENTRY_ARMED` | `false` | Bay entry remains a separate validation step |

For isolated CW/CCW start tests set the first-lap flag back to `true`; it has
priority. Both lap-test flags must be `false` to hand over to the existing
final-parking state machine. Its own test and entry locks still apply.

## Behavior

1. Lap 1 retains the camera acquisition, empty-place inference, corner-view
   recovery and CW/CCW stored starting signs. A seat behind the initial travel
   direction remains recorded until its normal approach, as before.
2. At the first wrap, require a resolved and internally consistent map. The
   official mode accepts at most two signs per section and a solitary middle
   sign. Check-all mode accepts three different longitudinal stations. Neither
   mode accepts simultaneous inner and outer signs at one station.
3. Build one repeated route from every confirmed sign, including stored start
   signs. Preserve the learned route for 200 mm on both sides of the lap seam
   and blend over the next 200 mm. This avoids changing the middle sign's
   avoidance directly under the car at the handover.
4. Adjoining end/first signs with the same passing colour receive a continuous
   corner around the existing corner centre. Endpoint radii come from the
   actual displaced route; a smooth radius blend replaces the superimposed
   straight tapers, which otherwise can fold back on themselves.
5. Official signs requiring the outer lane receive a 150 mm plateau on each
   side. Middle signs are solitary and end pairs are 1000 mm apart. The
   validated clearance values are retained. Check-all retains its shorter
   plateau/adjacent-pair policy and does not assume empty neighbours.
6. Check route tangents against field walls, all confirmed signs and the actual
   body/wheel polygons at the pink parking pieces. Reject a failed candidate;
   use the learned route only if it passes the same gate. If both fail, hold
   with an explicit log rather than follow an unchecked path.
7. Select the repeated route in the same control update as the wrap. Freeze
   acquisition and manual observation clearing during laps 2/3. Existing
   camera-search holds/reversals run only on lap 1; wall-based position
   correction continues. Travel direction does not change.
8. Use a speed-independent 330 mm pursuit distance, with the existing 0.65
   corner multiplier, only on laps 2/3. The normal curvature speed profile,
   configured 260 mm/s maximum and 42-degree steering limit remain active.
   There is no planned stop at the 1-to-2 or 2-to-3 boundary for a valid map.
9. After the third wrap, follow 150 mm farther through the start section so the
   rear can complete a middle sign's passage. Do not count this as a fourth
   lap. Cap this final runout at 180 mm/s, brake once for 300 ms, then either
   save/stop in the current test mode or hand over to final parking.

A held first wrap remains pending: newly resolved camera evidence can finish
the map and resume once, without losing the boundary or counting it twice.
An inconsistent map or failed geometry remains a real hold; continuous driving
is conditional on valid first-lap evidence.

## Logs

- `[LAPS] Boundary held...`: map unresolved or contradictory.
- `[PATH] Later-lap avoidance seat=...`: colour and chosen clearance.
- `[LAPS] Route preflight...` / `Parking-piece footprint...`: rejected geometry.
- `[LAPS] ...using checked learned route`: geometric fallback.
- `[LAPS] stage=LAP_2_RECORDED_MAP` / `LAP_3_RECORDED_MAP`: accepted handover.
- `[LAPS] stage=FINAL_RUNOUT` / `stage=FINISH`: third wrap and final release.
- `[OC] Three-lap test complete...`: autonomous test stop and USB save.

## Reproducible verification

Run `simulation/check_later_laps.py` with the IDE-managed Python and a host
C++ compiler on PATH. It extracts the current production geometry, clearance,
route builder, map gate, wrap logic and pursuit functions. Hardware/logging are
stubbed and layouts are supplied explicitly. Generated C++/executable/output
and `report.json` remain ignored under `local_workspace/later-laps/`.

The matrix contains 2,800 two-lap continuations: 1,232 combinations of both
directions, unrestricted/140 mm/s caps, all ten legal inner start layouts plus
an empty diagnostic start, and 28 normal layouts repeated on the other three
straights; another 1,568 cases cover every ordered pair of the 28 normal
layouts, alternating through the other straights in both directions.

Each trace uses ideal bicycle kinematics at 5 mm steps, the production
42-degree limit and integer steering commands, actual capsule clearances,
pink-piece polygon checks, and correct-side full-body crossing witnesses for
at least two passes of every sign. Six physical body/wheel polygons at both
steering extremes are used instead of a bounding box with nonexistent corners.
Separate state cases check incomplete-map retry, contradictory maps, O3's
three stations, no duplicate wraps, isolated one-lap tests and final runout.

This is not exhaustive over all whole-field combinations or placement errors.
Images, ToF acceptance, wheel slip, braking overshoot, update timing and servo
dynamics are not simulated. Geometric preflight in firmware samples tangents;
the offline traces additionally check pursuit tracking between those samples.
It does not guarantee a contact-free physical run. Final numbers and firmware
identity are recorded in `AGENT_DOCUMENTATION.md`.

Final model result: **2,800/2,800 pass**, including integer steering and
two complete passing witnesses per sign. Smallest capsule reserves in the
matrix are **78.4 mm to a field wall** and **15.8 mm to a pillar**. The latter
is still a tight case; placement/pose error can consume it. Keep the configured
speed ceiling for initial trials and verify measured reserves before increasing
speed. Shared parking polygons/storage checks pass 2,400 comparisons against
the independent Python geometry; these do not replace driving tests.

## Next physical test

After the user uploads **M7 only**, keep a previously successful marked official
layout fixed. Start CW with the usual parking bay, cable removed from the field.
Run one complete three-lap trial, then the same in CCW. Observe the uninterrupted
handover, both end/first-sign corners, the start sign on both returns, and the
autonomous final stop. Record contact/clearance, manual intervention and lap
count. Stop and evaluate a failed run before further repeats or speed changes.
Archive each complete unchanged USB log under the evidence directory and add
its direction, firmware identity, SHA and physical report. Repeat successful
cases three times before widening the layout matrix. Start layouts still need
their separate physical acceptance; use the one-lap flag for that work.

## Rules

The canonical January 15, 2026 PDF and official Q&A were rechecked on
2026-10-06. Rules 9.16/9.19 require one travel direction and RED-right/GREEN-left;
there is no mandatory turn-around after lap 2. The normal Obstacle task is
three-lap return (9.22); stopping in the finish section or parking lot ends
the Obstacle attempt (9.24.4). The project additionally targets parking points.
The current stop stages validation before that parking implementation. See the
official links in `WRO_2026_RULES.md`.
