# Final parking: direct staging and measured-pose continuation

Firmware parking revision: `20261030_pose_replan`. M7 only.
Physical acceptance of this revision is pending normal powered CW/CCW runs.

## Evidence

All21 archived logs containing a final-parking start were inspected. The
newest available source is log593. The20 newest USB originals,574--593, were
copied again unchanged and source/copy SHA-256 verified before continuing.
Every copy matches its already tracked portable original and exact hash in
[the evidence index](../evidence/parking_exit_diagnostics/README.md); no duplicate
evidence rows are needed. Generated analysis remains in `local_workspace/`.

The decisive complete originals are:

| Log | Direction | Recorded failure |
| --- | --- | --- |
|567|CCW|`initial_outer_wall_reference`; approach ends around x1052mm|
|577|CCW|Same failure; stable wall about387mm versus258mm predicted|
|579|CCW|+107.9mm wall correction, then `scan_alignment_limit`|
|582|CW|Segment2 endpoint13.3mm/6.5deg from nominal;6deg gate aborts|
|590|CW|Registered capture x210.9/y-1204.0/heading178.6;1deg heading gate aborts|
|593|CW|Passes capture; segment3 endpoint18.3mm/4.6deg from nominal; segment4 stops at x405.2/y-1357.0/heading141.8 before its remaining sweep|

For593, segment4 starts at local rear-axle105.0/177.6/58.5deg versus the
nominal116.85/163.62/63.08deg. The18.3mm position and4.6deg heading difference
already matter at the pink pieces. Its clearance abort was legitimate for
the old fixed remaining trajectory; widening that gate alone would not fix it.
Both582 and593 also turn approximately57--58deg during the120mm first
reverse arc, versus about63deg modeled: an effective radius near119--121mm
instead of about109mm. Encoder play/steering lag can contribute; these records
do not establish a globally calibrated tire-slip coefficient.

## Implemented behavior

After the existing whole-vehicle three-lap handover, a fully swept direct
staging line is tried before the learned pillar bypass. It has no passing-side
detour, but retains every confirmed pillar as a physical obstacle. Its complete
preflight includes +/-40mm lateral uncertainty; if it does not fit, the learned
bypass remains available. The outward shift still starts at x500/550/600/650
and the dual-marker scan remains required. An earlier outward shift can cross
a pink piece before lateral drift is registered, so it was not accepted.

This parking-only shortcut is consistent with the post-three-lap parking
exception in AppendixA/page41 of the [canonical rules PDF](https://wro-association.org/wp-content/uploads/WRO-2026-Future-Engineers-Self-Driving-Cars-General-Rules.pdf).
The January15,2026/55-page version and [official Q&A](https://wro-association.org/competition/questions-answers/)
were rechecked on2026-10-30. The existing lap-completion handover is unchanged.

At x>=900, a stable initial wall residual up to150mm can now be considered;
larger corrections still need a complete safe alignment preflight. The
alignment corridor is x800--1080, with the existing outer-wall/pillar/pink
checks, at most6 reversals and24s. Fresh stopped wall registration after the
dual-marker scan remains mandatory.

Entry is calculated from the registered current pose and measured gap. The
bounded analytic families solve a high-angle reverse straight from final Y,
then test reverse/forward arcs and final longitudinal centering. Adjacent
opposite movements with the same steering cancel rather than creating an
out-and-back loop. A short alignment is considered first when already inside.
If needed, a10--80mm retreat/arc is accepted only together with a complete
checked subsequent entry. The old nominal sequence remains a fallback seed.

Ordinary planning inflates body and pink pieces by6mm each after the first
20mm of recovery, including3mm additional allowance on each for localization
and endpoint error. Runtime checks retain3mm inflation on each. Normal
families and their alternatives are considered before the last tight mode:
1mm inflation on body and pieces,40mm/s, and the same full-path collision and
final-containment checks. Tight mode never authorizes a modeled intersection.
The target remains fully inside the measured bay, with5mm final boundary
margin and at most2deg heading error.

Arcs end from measured heading, straights from encoder travel. A settled full
or interrupted arc with >15mm travel and >0.08rad yaw supplies a run-local
effective-radius estimate, limited to0.85--1.20 of CAD. It affects only final
parking prediction; global Ackermann calibration and unparking do not change.
Prediction uses +/-15% radius initially, then +/-5% around a measured radius,
and stops each prediction at that arc's actual target heading. Every wheel
steering change is checked at the stationary pose too.

Pose drift or a refused motion brakes first, then searches a continuation.
Completion is checked after each settled segment. Before accepting the final
125mm straight-wheel footprint, the firmware actually enables the servo,
centers it and waits400ms; previously `stop(false)` disabled these servo writes.
Invalid pose/gyro, actual clearance, unsettled braking and no checked path
remain explicit stops. Replanning is bounded to8 attempts.

Search memory is fixed. Each reserve level has a60000 geometric-sample budget,
shared with escape children; gyro service continues during calculation.
10mm samples only reject candidates early; every accepted path receives a
complete1mm sweep. Hardware calculation latency still needs normal-run logs.

At60mm/s and109mm radius, lateral acceleration is about0.033m/s^2 (0.0034g).
The dominant tolerances here are geometry, localization and steering behavior,
rather than centrifugal loading. Course/approach speeds remain unchanged.

## Validation

The tests compile the production parking state machine with a CAD plant,
independent physical/estimated poses and synthetic ToF. They are not replays
of camera pixels or physical actuator/surface measurements.

* `check_parking_final_arrivals_20261030.py`:6 logged arrival anchors,
  both directions; no modeled contact. CCW approach saves611.8--627.2mm
  versus the learned bypass; CW saves14.0--16.8mm.8 additional wall-alignment
  cases, including107.9 and129mm residuals, reach scan readiness.
* `check_final_parking_sequence.py`:396/396 complete sequences park, including
  both directions,11 layouts,30/100ms sensor cadence, +/-40mm initial lateral
  bias and +/-180mm longitudinal registration perturbations.
* `check_final_parking_approach.py`:2970 approaches pass.156/162 perturbed
  legacy entry sweeps fit;6 deliberately narrow nominal variations reject.
* `check_final_parking_braking.py`:102 checks pass, including recovery braking.
* `check_parking_segment_sensor_resume.py`:42 checks and missing-servo-write
  negative control pass.
* `check_parking_mixed_edge_ranges.py`:864 cases pass;162 legacy edge failures
  remain reproduced by the negative control.
* `check_parking_adaptive_completion.py`:70/70 parked without modeled contact,
  including the recorded late partial-entry pose and independent plant radius
  factors0.95/1.00/1.05/1.10/1.15. Maximum4 replans,387.2mm total maneuver
  travel;8 recovery/final-verification health checks pass. Final wheels are
  neutral before completion.
* `check_parking_direct_capture.py`:72 cases,49 parked,23 retained safe
  rejections;21 direct alignments shorter, maximum169.2mm/4.44s saved;
 12 health checks and missing-health-guard negative control pass.

IDE-managed M7 build: SUCCESS; RAM442784/523624, flash545832/786432.
`firmware.bin` has549480 bytes, SHA-256
`30879c1228ded8ab9c5444c79ed27e25386141ec23e01e1cc5bf0cd433d303a3`.
M4 is unaffected and was not built. No upload or powered movement was performed.

## Next physical validation

Tester uploads M7 only and performs normal unchanged CW and CCW complete
three-lap runs, allowing automatic parking. Save normally; secure originals
first before any analysis. Verify the parking revision header, direct staging,
fresh two-marker registration, PLAN/CURVATURE/REPLAN records, actual neutral
wheels and complete contact-free containment. Record any physical contact,
manual stop or obstruction with the log number. No dedicated ToF sweep,
extra sensor movements or measurement pauses are required.
