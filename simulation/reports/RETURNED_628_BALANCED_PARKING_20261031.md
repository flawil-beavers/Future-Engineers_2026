#628: cancel extra10mm rearward preference after correcting rear geometry

User: front space became excessive while the robot moved rearward repeatedly.
Use the full available manoeuvring space; the old extra10mm request is explicitly
withdrawn because measured rear overhang is30mm instead of the model's old40mm.

Complete unchanged original evidence/parking_exit_diagnostics/20261031_log_628_cw.txt,
171194B,SHA-256c62f3f7995e30f4ec8e33d17571c63f65c2e5f27caad52fb0b9cfb8b8c641379.
Source-before/source-after/copy matched; immediate unplug notice; portable index
updated. Reported diagnosticmoduleOct31 17:20:17,pathfar_green_rear30,
parkingrear30_123mm,turn=-1. Installedbinary not read back. Footer firmwarecompleted
189835ms,truncated0,normal logging. User's physical report takes precedence over
any implication that completion means correct/contact-free parking.

628 measured gap253.3mm,capture(210.9,-1204.2,180.3deg). Initial quick recipe
3mm margin and remaining-recipe adaptation both accepted. All six subsequent
replans selected the preferred checked10mm rearward branch, including while
the measured radius changed through1.178/1.162/1.218/1.134. Some concatenated
straight prefixes cancelled/merged into short forward moves; others commanded
20.3,10.6,7.7 or10mm reverse. This identifies repeated rearward candidate bias,
not a measurement that each replan physically added10mm. Rear-reference final
correction-17.15mm is an estimator correction, not a driven movement.

Removed that entire preferred-prefix branch, with no replacement positional
offset. Normal quick recipes and bidirectional general/escape search remain,
including full swept footprint, steering-radius envelope and final containment
checks. Actual rear30mm, width123mm, established final axleX60/preferredY127 and
front reserve16mm remain. Safe reverse movements are still allowed if the
complete candidate needs them; neither direction gets the removed special
priority. Normal planner is a bounded search, not proof of global path optimality.
No camera, obstacle route, speed, exit movement, rear-ToF fan or logging change.

Validation:628 capture/partial-entry anchors mirrored over both directions,
measured gap253.3 and radius scales1.134/1.178/1.218 park safely12/12. Returned
620/621 anchors12/12 and forced-history12/12; no automatic-prefix message.
4 ordinary partial-entry selections confirm no removed preference and unchanged
endgoal.396 full sequences and6 remaining-recipe transitions PASS. Remaining
adaptive/tight/overrun checks and build identity recorded in history below.
Fixtures use estimated recorded anchors and a CAD plant, not independent
physical reconstruction of628; slip/registration/reflections can still differ.

Next: user uploads M7, normal park/full run/save. Secure originals FIRST; compare
forward/reverse segments, replan count and actual front/rear reserves against628.
Do not reintroduce the cancelled10mm request from older documentation/history.
Keep measured30mm geometry and good final position. No upload/motion/commit/push.

Final adaptive350/350,84overrun,36tight PASS. M7 IDE build SUCCESS:
RAM453816/523624,flash563104/786432,binary566656B,
SHA-25633e96b5b3ec08474a9f0a255c74474a5023d44f4f7f838ba1f93cd08cc161dd0.
Parking20261031_rear30_balanced; M4 unchanged/not built; dense capture off.
