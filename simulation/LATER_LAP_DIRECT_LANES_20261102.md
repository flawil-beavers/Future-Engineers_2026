# Direct later-lap connections and handoffs

Completed 2026-11-02. Firmware path identity `20261101_direct_lanes`;
parking remains `20261101_efficient_entry`. M7 only.

## Finding and scope

The existing builder superimposes individual pillar bypasses on the discovery
centreline. Its optional elastic inner-route search has a 30 ms MCU budget;
even the initial geometric preflight can exhaust that budget before a useful
candidate is evaluated. Logs 632/634 show 7870.1 -> 7807.2 mm from ordinary
smoothing but no accepted inner-search message. This is consistent with that
early exit, not a hardware timing measurement proving the exact exit point.
No new USB/source batch was supplied during this change. Previously archived
629-634 remain the latest evidence; they do not validate this firmware.

Implemented a bounded three-candidate cubic connection between protected
passing plateaus, after the existing optional search. Endpoint tangents come
from the accepted route. The new construction does not depend on completing
the iterative search. It considers the space between known signs rather than
introducing an intermediate return to the centreline. It may retain the prior
route when no shorter, faster, sufficiently clear connection fits.

Only official normal-section CW GREEN end positions with a moderate outward
bypass now request 230 instead of 260 mm centre clearance. First-lap policy,
start signs, solitary middles, extreme bypasses, adjacent pairs and CCW
clearances retain their existing policies. A broader CCW reduction was tested
and discarded because it worsened one other layout's calculated route/time.
The CW outer-GREEN -> inner-RED 180 mm policy remains: its recorded wall-room
regression still passes (tracked RED-region wall minimum 138.407 mm).

GREEN must still be passed on its required side. In particular, a CW GREEN
on the robot's left can require an outward bypass before a shorter inward
connection is possible. A blanket smallest-radius route would violate this
or consume wheel/front-overhang clearance.

## Acceptance and transitions

- Protect each confirmed sign's +/-150 mm longitudinal passing region.
- Preserve the first 400 mm and the complete last corner, beginning 280 mm
  before that corner, exactly as they enter the new connector stage. Existing
  lap-seam preservation and the full-body last-corner finish gate remain.
- Require at least 5 mm route shortening and 0.01 s target-time improvement.
  Total turn must not increase by more than 0.01 rad; bending may increase at
  most 1%; peak curvature cannot exceed max(previous peak, 1/250 mm).
- Full route wall/pillar/parking-piece preflight must pass. A new connector
  needs at least 80 mm modeled wall and 50 mm modeled pillar reserve. These
  are envelope reserves, not axle-to-object distances or promised real gaps.
- The existing runtime pursuit, speed, wall and collision guards remain.
  No new stopping state. The new candidates use geometric acceptance online;
  pursuit/motor behavior is checked by the offline matrix below. The previous
  optional inner-search rollout gate is unchanged.
- Three finite analytic candidates replace no safety check. Planning is still
  synchronous at the existing route build; it is not a hard real-time task.
  `Direct connectors ... planning_us=` records acceptance or fallback and
  elapsed time for the next normal run. MCU latency remains unmeasured.

## Calculated results

Comparison uses actual extracted C++ and the previous construction (new
connectors disabled and old end-clearance restored), with unchanged dynamics.

| Layout | Shorter per lap | Target-time reduction per lap |
| --- | ---: | ---: |
| B CW | 93.04 mm | 0.729 s |
| B CCW, same physical map | 17.13 mm | 0.170 s |
| C CW | 102.81 mm | 0.270 s |
| CW outer end -> next outer middle, optional search timed out | 622.85 mm | 2.413 s |
| CCW mirror of that example, optional search timed out | 405.91 mm | 0.686 s |

The B result persists with unlimited and deliberately exhausted optional-search
budgets. Where the earlier unlimited search already found a better inner
connection, it stays unchanged. There is no claim of a globally shortest path
or of these times being measured laps. B's modeled tracked wall/pillar minima
are 102.564/70.3701 mm CW and 94.3475/66.593 mm CCW.

Speed ceilings remain unchanged. The existing speed bound is
`v <= sqrt(800 * R / 1.15) / 1.10` in mm/s with radius R in mm, capped at
520 mm/s. Thus R=250/350/500 mm gives approximately 379/449/520 mm/s.
The 10% speed and 15% curvature allowances are engineering tolerances, not
measured tyre-friction or slip guarantees. Motor acceleration/braking and
response allowance remain active.

## Verification

- `check_later_laps.py`: 2807/2807, modeled wall/pillar minima 80.281/32.1933 mm.
- `check_later_lap_dynamics.py --full-matrix`: 2191 cases each at 10/20/50 ms,
  6573/6573. Peak lateral acceleration 0.760805/0.768285/0.796430 m/s^2;
  minimum pillar reserve 31.9874/32.0376/33.4638 mm, wall minimum 80.281 mm.
  Final generated dynamics fixture matches changed production functions.
- `check_later_lap_direct_lanes.py`: 16 before/after comparisons, each with
  full two-lap passing, unchanged learned route, seam and protected final
  corner checks; explicit optional-budget exhaustion included.
- `check_cw_inner_red_623.py`: preserved RED passage regression passes.
- `check_final_parking_approach.py --approach-only`: 2970/2970.
- `check_final_parking_sequence.py`: 396/396 complete, no modeled collision;
  five marker-registration checks pass.

The old approach test's additional fixed-entry diagnostic rejects four nominal
poses (72/162 fixed entries accepted). Those tests assume the historical fixed
recipe succeeds alone, whereas production first selects the adaptive planner
and only then tries the fixed fallback. The new explicit `--approach-only`
option isolates route handoff; the full adaptive sequence is verified separately.
The original diagnostic remains available and its rejection is not represented
as a successful fixed entry.

Reproducible reports are in `local_workspace/later-laps/`,
`local_workspace/later-lap-dynamics/`, `local_workspace/later-lap-direct-lanes/`
and `local_workspace/final-parking-approach/`. The latter route comparison
directory also contains an illustrative `routenvergleich.png` (rear-axle paths,
optional-search budget exhausted, not physical telemetry).

M7 IDE build SUCCESS: RAM 459200/523624, flash 571720/786432;
binary 575272 bytes, SHA-256
`711c067090cb0a1464bdba05b9db87b69d3e9a0c2129827f1508d90a081fbf11`.
No M4 change/build, upload, robot movement, commit or push. Normal obstacle mode,
compact logging, camera/discovery, unparking diagnostics and parking maneuvers
remain unchanged.

Next: user uploads M7 and performs normal CW/CCW runs and normal saving.
Secure complete originals and hashes FIRST, then compare route-selection and
planning_us records, physical inner/outer wall and pillar reserves, lap times,
1->2 handoff and final corner->parking. No dedicated ToF sweep is needed.
