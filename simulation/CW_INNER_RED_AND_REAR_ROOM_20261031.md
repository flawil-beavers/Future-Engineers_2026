# CW inner RED reserve and normal partial-entry rear room

User identifies the straight after the start corner, GREEN outside then RED
inside, mainly laps2/3: too near inner wall beside RED. Reports about120mm room
to RED. Also reports front pink contact during parking with about40mm room
behind; wants extra10mm available during normal manoeuvring, same final pose.
No new removable drive was present at initial inventory. Existing complete623
is used; its SHA-256 and firmware identity are indexed in
evidence/parking_exit_diagnostics/README.md. New physical parking report is not
unambiguously assigned to a log number. Do not invent a new measured pose for it.

## CW passage

623 confirms seat7 GREEN(-1100,-500), seat10 RED(-900,500). Later-lap old
RED clearance210mm implies west-leg axle x approximately-690mm. Logged lap2
estimator reaches-626.8 at y427.4; lap3 stops around-627.9,358.3,62deg. Inner
wall x=-500mm. Axle-only wall range~127mm does not include the angled front
overhang and steered wheels. Actual120mm RED clearance is a user estimate;
independent physical XY/yaw are unavailable, so this cannot establish drift.

Only official CW later-lap RED at the inner row with an earlier confirmed
outer GREEN in the same section uses180mm instead of210mm centre clearance.
This shifts the passing line30mm outward, toward RED, away from the inner
wall. Other arrangements, CCW and first-lap clearance retain existing values.
No speed ceiling, colour thresholds, wall correction or logging changes.
All route, physical footprint, pillar and tracked-path checks remain active.

Actual production-code CAD replay of layoutB: at the RED station lane
x -689.167 -> -719.167mm; modeled tracked wall minimum in this passage
109.250 ->138.946mm, global pillar minimum70.370 ->67.859mm. Geometry-only
improvement is not a guarantee of the physical clearance under gyro/servo error.
Old/new and control regression: simulation/check_cw_inner_red_623.py.
All2807 later-lap configurations pass.

## Parking

Removed the two-failed-replans and clearance-reason conditions on the existing
checked10mm rearward alternative. Every ordinary partial-entry plan (localY
below250mm) now tries it first, including the first segment_pose replan after
the initial arc. First approach/arc remain unchanged. A complete subsequent
entry must fit before the candidate can be selected. Both directions mirror
the same logic. Actual123mm wheel geometry, rear overhang40mm, original end
targetX60/preferredY127 and end containment remain unchanged.

10mm is an available planning prefix, not a compulsory physical movement:
appendEntry merges successive equal-steering segments, including opposite
directions, so the net first straight may differ. On recorded partial anchors
at localY247.4/119mm, both directions select the alternative at zero replans;
net first straights are121.912/3.078mm after merging with their continuations.
This removes the artificial retry restriction; it does not prove every pose
now uses a full additional10mm or independently establish the reported rear gap.
Requested the physical rear-axle-centre to rearmost-part distance (model40mm)
to check possible overestimated rear geometry. Awaiting that measurement.

PASS: zero-failure-history rear-room selection4/4; recorded anchors12/12;
forced-clearance-history12/12;396 full parking sequences;350 adaptive radius;
84 braking-overrun;6 remaining-recipe transitions. Continuous CAD/encoder/gyro
fixtures exclude original pixels, unknown pose bias and mechanical compliance.
No real test of these edits yet. The broader unvalidated initial reverse
extension from the earlier attempt remains discarded.

M7 IDE build SUCCESS RAM453816/523624,flash563256/786432,binary566808B,
SHA-256da2616ba0ad2c53f05b81c36888e8fd7695d697221fe3bb719661bb22a7871a8.
Path identity20261031_cw_inner_red, parking20261031_rear10_normal.
Dense capture stays disabled; full obstacle startup remains. M4 unchanged/not
built. No upload/motion/commit/push by agent.

Next: upload M7, normal CW run/save; secure original FIRST. Compare RED passage
front-wheel/inner-wall reserve and physical clearance to RED. For parking,
check whether early rear-room planning reduces front pink contact, record which
arc and rear gap, and use the requested measured rear overhang before changing
physical footprint dimensions. Keep exit ToF/brake/reversal diagnostics; no sweep.
