# Complete parking arrival audit: CW564 and CCW567 anchors

The user requests verification of the whole approach after lap3, particularly
apparent wrong-side travel and stopping beside the pink rails. This review uses
existing originals only; there is no new returned batch or physical validation.

## Recorded starting evidence

Paths are below `simulation/evidence/parking_exit_diagnostics/`:

| Original | SHA-256 | First parking pose X/Y/heading |
| --- | --- | --- |
| `20261027_log_564_cw.txt` | `72fb479284dc4f81e2ea222b8aa5c7ae8386b063a9a2ad1752c82a92a75b2df4` | 152.4/-1018.4/165.7 |
| `20261027_log_567_ccw.txt` | `e1cbb0c6089e114677a8987e7912ea9926aa9aa8b54ba428dfeb637b61563915` | -150.4/-1072.1/30.8 |

These are estimated stationary/initial parking poses, not external position
measurements. Confirmed maps are extracted from their original live-injection
records. In particular, CCW567 has inner start GREEN5; the learned approach
passes on its prescribed side before shifting towards the outer wall. Its
large inward bend alone is not evidence of reversed parking direction.

Both approaches travel towards field +X: CW in reverse, CCW forwards. The
outward shift is checked against pillars and pink rails. The body then becomes
parallel to the outer wall before scanning towards field -X. For the nominal
bay the rear-axle scan line is294.6 mm from the wall; straight half-width62.5 mm
leaves32.1 mm beyond the200 mm rail tips. Seeing the wall alone is insufficient:
the accepted two-rail scan fixes the actual bay and capture coordinates.
Nominal capture X is209.74 CW and502.76 CCW, with heading180/0 degrees.

## Additional correction

CW564 applies an early Y correction of-46.6 mm near X480 while still following
the approach lane. The old initial reference accepted two consistent residuals
within80 mm without checking whether the pink rails could lie in the sensing
fan. A stable mixed wall/rail range can therefore shift the estimated pose.
This mechanism is reproduced by injected accepted returns. The physical source
of the original564 measurements remains unproven without scene/target evidence.

During `FP_APPROACH_LANE` only, early wall registration now requires that the
modeled sensing fan miss the possible rails across their full depth, including
the existing250 mm longitudinal registration uncertainty. An ambiguous early
reference resets its confirmation count and applies no correction; the normal
approach keeps driving. The later stationary wall registration, bounded CCW
drift recovery, marker scan and existing capture alignment remain available.
No new stopping state, relaxed clearance, altered route/speed or exit change.

`markerEdgeFootprint` now accepts an optional intersection-plane Y to inspect
both tips and wall ends. Existing marker-edge calls retain the same tip plane;
the864-case independent mixed-edge regression confirms unchanged scan behavior.

## Recalculation

`check_parking_arrival_564_567.py` starts from the two recorded poses/maps,
perturbs X/Y by+/-10 mm, heading by+/-2 degrees and adds0/10/25 mm straight
forward coast. It then runs the actual final-parking state machine through
approach, scan, capture, all seven entry segments and containment. Subsequent
sensor returns/plant are ideal models, not the original temporal log replay.

- 162/162 variants park and remain within the modeled footprint checks.
- Minimum wall/pillar reserves45.55/65.16 mm in this grid. Pink rails are
 separately checked using the steered body/wheel polygons and existing margins.
- Largest capture error1.498 mm longitudinal,0.780 mm lateral,0.169 degrees.
- 12 reference checks: ambiguous early returns never move the pose; unobstructed
 early references remain usable. Removing only the new guard reproduces a
 false pose correction, without needing a physical test or sensor sweep.

Further checks:

- Existing approach geometry2970 cases passes;162 entry perturbations include
 156 accepted and6 previously rejected combinations at the narrow gap/extreme
 position-heading tolerance. Nominal cases pass; those margins were not relaxed.
- Full sequence396/396 parks, with independent initial lateral/scan-X errors.
- Broad robustness990/990 known arrivals parks;132 unknown-pose cases88 parked/
 44 rejected;360 arbitrary starts112 parked/248 rejected;40 other-section,
 16 faulty-sensor and2 wrong-gap cases rejected. No modeled unsafe case.
- Boundary46, quality5 and mixed-edge864 checks pass.

Hardware slip, steering lag, target reflections and unbounded odometry error
are not established by these calculations. Existing abnormal-pose refusals
remain; do not claim universal physical completion from these results.

## Next normal run

Current revision: `20261028_parking_wall_reference`. Upload M7 only. Follow
normal CW/CCW runs/save, copy originals FIRST and notify immediately. Verify
the early reference near rails no longer moves the estimate, later stable wall
registration succeeds, both marker edges/gap are accepted, capture and all seven
segments finish, and physical containment/contact agrees with the logged result.
No agent upload, robot motion or dedicated sensing sweep was performed.

IDE-managed M7 build SUCCESS: RAM442416/523624, flash532888/786432 bytes.
Binary SHA-256:
`ca500e0d2b6127929fa0ac99cf98164cd8e510be535fc7c708b3b12425a0f4d9`.
M4 unchanged/not built; existing `Serial` redefinition warning remains.
