# Parking motion review: returned629?634

## Preserved evidence

Complete unchanged originals and SHA-256 are indexed in
[evidence README](evidence/parking_exit_diagnostics/README.md).
629?631 were secured first from removable media;630?634 later supplied as
local source files were secured first again with source-before/source-after/copy
hash verification.630/631 match the already indexed originals exactly.
629 uses parking revision20261031_rear30_123mm;631?634 use
20261031_rear30_balanced. All report path20261031_far_green_rear30 and diagnostics
moduleOct_31_2026_17_20_17. No installed binary was read back.
630 CCW stops before parking; do not attribute a parking result to it.

| Log | Direction | Footer outcome | Relevant parking result |
| --- | --- | --- | --- |
|629|CW|completed,172369ms|Older extra10mm-preference version; superseded.|
|630|CCW|mode_pause,16188ms|No final-parking test.|
|631|CW|mode_pause,208216ms|Repeated roughly10.6?11.0s general searches.|
|632|CW|aborted,228575ms|entry_replan_limit after8 accepted corrections.|
|633|CCW|completed,190338ms|7 corrections including rear-marker localization.|
|634|CW|completed,198688ms|4 corrections;10.151s and8.570s searches.|

All supplied footers report truncated=0 and sample_bytes=0. Completed is an
estimator/state-machine outcome, not evidence that no physical contact occurred.
User reports available room, little progress and excessive stationary periods;
no additional per-run contact report was supplied for632?634. All three new
runs reach lap3 and the final-parking phase; no camera/route tuning is justified
by parking completion alone. Discovery still includes delayed confirmations and
recovery movements; these logs do not establish perfect perception.

## Reproduced failure and change

631 gap251.7mm: after the first arc, the settled field pose343.8/-1262.6/119.8
maps to local136.2/237.4/60.2 degrees, with measured radius scale1.159.
The former fast tail only changed the straight distance while retaining its
old intermediate headings. It rejected this pose in both mirrored tests,
then fell back to the expensive general search. Later short corrections were
rejected by entry_motion_clearance.632 repeats this pattern until the limit.
Its trace shows a1.1mm forward prefix followed by an11.2mm reverse arc, then
further2?3mm prefixes and renewed clearance rejection. This supports a planning
versus settled-execution mismatch; the exact later631 pose is unavailable
because detailed trace output ends earlier.

The bounded tail solver now keeps the original continuation first and tries
alternative intermediate headings, including continuing the initial reverse
arc to70/80 degrees when useful. It solves each straight from the actual pose
and measured radius, then checks the complete swept footprint at1mm intervals.
There are at most12000 geometry steps per margin level. Only early partial
entries (localY220?250mm with a measured radius) also use this solver before
the general fallback. Checked forward/reverse prefixes remain ordinary
alternatives; there is no preferred extra10mm rearward displacement.

For a steering transition following a straight shorter than5mm, preflight now
also checks the initial arc from up to2mm short of the nominal endpoint.
This targets the small-prefix trap without imposing a global minimum segment
length. The runtime collision forecast, brake settling, gyro health gates,
rear-reference corrections and checked general fallback remain active.
The successful bounded search logs search_ms for measurement on the next run.
Full search can still occur outside its feasible family; host timing is not
MCU timing, so no fixed new parking duration is claimed.

Actual geometry remains width123mm, rear overhang30mm, front125mm. The good
preferred final axle goal60/127mm remains; contained alternative depths already
permitted by the recovery planner remain available. Front steering reserve and
final containment checks remain. No route, camera, speed, exit-ToF or dense
telemetry changes were made in this parking correction.

## Validation and next run

Negative control before the change:631 first-arc fast adaptation0/2.
With the correction, both mirrored631 poses park at actual radius scales1.159,
1.20 and1.235:6/6, no modeled collision, at most2 further replans; nominal1.159
needs none. The first accepted tail uses3497 of12000 geometry steps.
The632?634 regression covers4 recorded anchors, both directions, yaw?1.5/0
and settled overrun0/1.5mm:48/48 completed without modeled contact.
Broader results:396/396 full sequences,350/350 adaptive,84/84 brake-overrun,
36/36 tight,12/12 balanced628 anchors,6/6 prior tail adaptations, health and
registration checks. These are independent kinematic plant/production-code
checks; tyre slip, actuator backlash, unknown pose bias and sensor errors can
still differ on the robot. The631 narrow regression uses the production motion
integrator;632?634 uses a separate plant.

Next: upload only the new M7 environment, run normal CW/CCW parking, save
normally and secure originals first. Compare bounded search_ms, correction
count, actual front/rear room and contact observations. Keep final goal and30mm
rear geometry; do not reinstate the cancelled extra10mm preference. No dedicated
ToF sweep or additional diagnostic driving is required.

Final IDE-managed M7 build: SUCCESS. Parking identity20261101_bounded_tail;
path20261031_far_green_rear30. RAM453816/523624, flash563912/786432;
firmware.bin567464bytes, SHA-256
`e9e762a1cf2660039b574b717083b4e1a7e5669c401a99a60738da52fe221acb`.
620/621 recorded12+forced-history12, normal partial goal and brake/rebase checks
also pass. M4 unchanged/not built. No upload, robot motion, commit or push.
