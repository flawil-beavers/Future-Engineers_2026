# CW logs3--6: first-lap association and parking, 2026-11-19

Complete USB originals were copied before history or analysis, with source-before/after and copy SHA256 verification; user immediately released the stick. Resume verification found no changed or additional files. Complete source file does not imply completed mission. Logs1/2 in the intake repeat previously secured evidence.

| Original | Bytes | SHA256 |
| --- | ---: | --- |
| [20261119_log_3_cw.txt](evidence/parking_exit_diagnostics/20261119_log_3_cw.txt) | 98495 | `07fd4ec064b140b6c1bd722f5429dcb475e50836c259e4eb9bacaa748ec3d835` |
| [20261119_log_4_cw.txt](evidence/parking_exit_diagnostics/20261119_log_4_cw.txt) | 62327 | `0379446f767a296bc02aa8de55c0f92498e71a37a48b9156dde2fc44bd76a0d5` |
| [20261119_log_5_cw.txt](evidence/parking_exit_diagnostics/20261119_log_5_cw.txt) | 140019 | `89c36184a46474dcecfb90201e122476902960fa5ad05c5c564420351470e795` |
| [20261119_log_6_cw.txt](evidence/parking_exit_diagnostics/20261119_log_6_cw.txt) | 145310 | `ecb427fb7f67a77efe49eb314f32195f4050aff38e1a11ae283544ecc4d55a0b` |

## Physical reports and installed identity

User reports: first CW run missed RED in the third/final non-start straight, passed on its left, then contacted it on lap2; second run hit the farther startGREEN; third had difficulties in the third section and stopped before parking. Latest run drove the obstacle part well and recognized the small RED, then stopped at parking. Logs3/4 are user-paused;5/6 autonomously abort during parking. Both logs reaching parking report `20261111_recovery_handoff`, predating the prepared scan-line/speed correction. All4 report measured_height20261119; exact installed binary not read back. Log6 changes the modified seat15 to50mm visibleRED/effectiveGREEN; logs3/5 store150mm visibleGREEN/effectiveRED. No hard-coded test map added.

## Findings and changes

1. **Visible RED rejected as too far around the corner (log3).** RED18 is already projected near(900,500) at ranges934--1255mm before S3. The corner-relevance extension incorrectly required range<=600mm, the EMPTY observation window, while colour acquisition supports the calibrated longer range. Later, two oblique dropout views mark its station CLEAR; the frozen later-lap map omits the RED. Use the colour calibration maximum1600mm for this existing extension, preserving6deg ray agreement, seat snap, side disambiguation and consecutive votes. A next-loop remote seat remains rejected. This explains a concrete missing-pillar map, not every camera dropout.

2. **Oblique EMPTY evidence (log3).** Its first corner station is cleared with section heading error~34--46deg. Require <=25deg section alignment for ordinary first-station absence within the existing450mm range. Confirmed colour can still be acquired farther away. Existing targeted corner observation of that same station and parking scouts remain eligible, preserving the previously checked565 recovery. No new motion/hold state is introduced; unresolved stations retain their checked re-observation path.

3. **Different colour at start (log4), unresolved.** No startGREEN is confirmed; a close RED component is ultimately associated with middle seat2, whose official solitary-middle inference removes the expected end seats. Raw camera summaries show RED and absentGREEN, unlike the other runs. No approved original images/per-object truth from this run establish why. Do not call it a proved HSV, byte-order, hardware or distance defect. Preserve colour thresholds and geometry gates; append cached camera errorBits/badPhaseCount to the existing bounded DRIVE_CAM line for future normal-run diagnosis. No extra images, captures or dense logging.

4. **Scan path clearance (log5).** At(638.3,-1228.7,181.7deg), even the neutral15mm scan sweep fails the existing inflated wheel/rail check. Moving the scan/capture line10mm inward (already prepared from log2) makes that exact sweep pass. Maintain20mm nominal tip reserve, current final parked target,150/130mm/s free approach/alignment maxima and terminal60mm/s blend. Scan speed stays60mm/s, entry speed unchanged.

5. **Invalid dual-rail measurement (log6).** Fixed556.1/moving333.6 yields222.4mm gap, outside the247.5+/-15mm gate. It previously aborted immediately. With a fresh stopped wall reference and a fully checked return/alignment corridor, permit one repeated scan of both rails. Clear invalid measured-gap state before planning that return; reset the ordinary scan phases on restart. Do not widen the accepted gap or enter using invalid geometry. A repeated bad measurement or unsafe retry path still rejects entry. Model replay uses the new safe lane and recorded heading/X, not an independently measured physical pose.

6. **Known232.5mm accepted-gap planner hole.** Current capture281.5mm high still fails all24 narrow-gap perturbations before motion with the prior bounded search. Add a fallback only after its ordinary families fail, for near-parallel captureX240..280mm outside the tips and measured gap<nominal-10mm. Try a40/60mm straight staging retreat, and accept only a complete freshly swept tail plus combined path. The child startsX<=240mm, preventing recursive staging. Existing margins and first-arc radius-envelope checks remain. This improves18/24 narrow cases;6 with +15% or +50% curve-radius deviation still safely abort after motion. Do not claim universal narrow-gap recovery.

Gyro polling during approach preflight remains; also service queued gyro reports during scan-alignment preflight. No first-lap speed reduction, HSV change, M4 change, robot upload, reset, motion or additional commit.

## Verification

- New archived-numeric corner regression2 passes; reverting only the old600mm gate reproduces rejection. Distant next-loop seat remains rejected.
- GREEN occupancy589, nearer-view621 and oblique-clear3 controls pass. Targeted565 empty/occupied evidence10 and checked rejoin27 pass, including the old-veto negative control.
- Far acquisition52 and UNKNOWN/recovery dispatch25 pass.
- Approved images45/45; captured sequences16; rendered648, association48, rule policy24 pass. No unapproved/rejected image used.
- Full parking quick3/3 and registration5 pass; existing recovery/braking handoff6 pass.
- Measured248.9/249.6mm parking24/24 pass, both directions, radius1/1.15/1.5.
- New log5/6 scan regression passes: old sweep rejected/new lane accepted; one bad scan recovers to250.3mm modeled gap/capture without modeled collision; repeat-invalid remains rejected.
- Accepted-span stress120:114 park,6 narrow-gap cases abort, no modeled collision. Before the staging fallback96 park/24 narrow aborts. CAD, ideal ToF and synthetic colour tests do not establish physical acceptance or fix log4's colour source.

Next: upload onlyM7 normally and perform normal CW/CCW runs. Secure originals first. Check early RED18 recognition, GREEN start passage, parking scan geometry/retry and physical clearance/final containment. Do not revive historical special sweeps or use invalid wrapped images. If wrong visible colour recurs, obtain a short user-approved stationary camera sample before changing colour thresholds.

Final IDE-managed M7 build SUCCESS: RAM452136/523624B, flash599200/786432B; firmware.bin602752B SHA256`332b21f986d6f48efa276998d4bca92c9f6c17d371662256b54b50402fb11948`. M4 unchanged/not built. Existing Serial macro and unused-helper warnings remain. No robot upload/movement or extra commit.

Rejected follow-up experiment: preferring an80mm staging retreat before60/40mm increases the narrow-gap stress failures from6 to12/120 (no modeled collision), including new5--10% radius deviations. It was applied only to a generated host fixture, never to production. Keep the tested60/40mm ordering; do not infer that a larger retreat always helps. Reproducible with the accepted-range-stress fixture after replacing only its retreat candidate order.
