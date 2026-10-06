# Start-case review, 2026-10-06

Reproduce with the IDE-managed Python using
`simulation/check_start_geometry.py` and
`simulation/check_connector_servo_resume.py`.
Generated reports and host fixtures stay under `local_workspace/`.

## Steering failure: concrete software cause

The active connector update calls `stop(false)` before checking a newly
confirmed pillar. This sets `servo_disabled=true`. Motor restart through
`set_speed()` does not restore servo writes, and `steer()` ignores commands
while disabled. Successful connector retention/replanning now restores the
flag; unsuccessful planning keeps the robot stopped.

The regression compiles the actual extracted source functions and pending
update branch. Both retained and replanned success write the new steering;
rejected planning stays disabled. Removing the restoration reproduces the
old failure. This explains how right commands in 455/456 could coexist with
continued left rotation. The installed binaries were not read back and the
complete logs are still unavailable, so physical validation remains required.

## Isolated inner last GREEN

The nominal westward route lies 100 mm outward of the pillar centre. The
old 260 mm setting therefore adds 160 mm outerward displacement; 200 mm
adds 100 mm. Radius-1 smoothing reduces the peak displacement slightly.

| Setting | Path Y at seat (mm) | Sampled capsule/pillar margin (mm) |
|---|---:|---:|
| 260 mm | -1147.3 | 121.5 |
| 200 mm | -1091.8 | 78.8 |

The narrower path moves inward by 55.6 mm at the seat. These estimates use
the existing 70 mm capsule radius and 42.5 mm pillar movement radius, with
front/rear capsule positions. They describe a nominal sampled route, not
closed-loop tracking or measured pose uncertainty. Other pillar clearances
are unchanged. This path refinement alone did not address the servo bug.

## RED first/right: reject a misleading apparent solution

Log436's connector start is reconstructed approximately as
(626.2, -1209.0, 153.3 degrees) from rounded scan/preflight values. A shorter
Hermite end tangent (1.0 instead of 1.5), continued 125 mm lookahead and an
offline 700 mm travel allowance reach the endpoint in 616 mm, with about
68 mm modeled pillar clearance. However, at the first crossing of the
pillar's X=500 mm line the axle is at Y=-1130.3 mm, outside the RED centre
at Y=-900 mm. Westward travel requires passing RED on its north/right side.
This candidate passes on the wrong side and is rejected.

Increasing the 500 mm firmware travel limit is therefore not a solution.
The firmware limit and tangents were not changed. A valid approach needs
explicit correct-side checks for the guard pillar in addition to clearance,
steering and handoff checks. A different observation/approach pose is a
design option, but no such complete route is validated here.

## Rosa parking pieces: replay of reverse-localization samples

| Original CW log | Samples | Minimum outline above 200 mm open end | Samples with modeled collision |
|---|---:|---:|---:|
| 439 | 19 | +10.3 mm | 0 |
| 440 | 20 | +3.8 mm | 0 |
| 441 | 11 | -5.1 mm | 6 |
| 442 | 23 | +16.9 mm | 0 |

This supports insufficient clearance as a plausible cause of the 441 stall.
The logged estimates do not independently identify actual contact or exact
marker placement. Footprints use the existing approximate chassis/wheel model
and conservative full-lock wheel envelopes for nonzero steering. Checks cover
three modeled parking gaps, 242.5/247.5/252.5 mm, at logged sample poses.

Translating the entire 441 sample set 10 mm inward leaves +4.9 mm, and 20 mm
leaves +14.9 mm. These are hypothetical translations, not feasible generated
motion or firmware changes. A revised exit must prove how it achieves this
shift while retaining heading, rear clearance and localization. Sampled
collision checks also do not establish clearance between telemetry samples.

## Evidence and next steps

The report includes original-log SHA-256 values for 439--442. Complete logs
455/456 must still be archived when the stick returns; current `_excerpt`
files contain only selected rows and cannot establish complete run coverage.
Use the sequence in `CONNECTOR_NEXT_TEST.md`: validate the restored steering
on one inner-last GREEN run first, review its complete log, then expand tests.
RED first/right and additional parking reserve remain separate design work.
