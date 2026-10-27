# CCW first/second runs:523--524; third run unavailable

User reports first and third did not get out/held; second exited but operator
stood in the way and explicitly excludes it from driving acceptance.523/524
are the only new files on receipt,matching first/second in order and behavior.
User confirms third may not have been saved and robot has since restarted or
another run begun. No third-run diagnosis can be established from these files.

Both originals were copied FIRST,without modifications,and SHA-256 matched to
USB. `evidence/parking_exit_diagnostics/README.md` records sizes and hashes.
523 is the entire38605-byte available original but has no RUN_END/manual-stop
tail; `_excerpt` makes that incompleteness explicit.524 is55647bytes,complete
RUN_END/truncated0,no overflow. Both CCWturn1,parking-object compilation time
Oct_26_2026_21_02_12. Correction after525--527:this timestamp is per translation
unit,not a reliable whole-firmware identity in incremental builds. Exact image
identity/upload of6a07252e is unverified;installed flash has not been read back.

## First run: exit completes, then start perception locks the motor

- Parking-exit diagnostics finish at26519ms,75samples,untruncated. All five
  exit segments and localization complete; there is no recorded exit-state
  timeout,footprint rejection or motor stall before this finish.
- CCW reverse observation reaches its planned view within9.6mm/2degrees,
  measured reverse93.2mm vs84mm planned,including braking. Estimated stopped
  pose is261.1,-1210.5,41.8degrees. These are onboard estimates,not surveyed pose.
- Target is inner start seat5/station2. Modeled bearing/range15.5degrees/269.9mm.
  Both the primary stationary observation and the same-pose retry return UNKNOWN.
  Final explicit action: `Discovery unresolved after retry - drive motor locked off`.

Thus the logged reason for the first hold is failed start obstacle association,
after physical exit has completed. This is not evidence that the exit arcs
themselves failed or that GREEN pixels were absent.

## Why the standing retry does not resolve it

Source pinhole projection predicts
`expectedX = 164.4 - 248.9*tan(15.5deg) = 95.37px`.
Actual logged GREEN centroids while stopped are17then35--36px,foot188--192,
width43--71,height109--113. The local fallback searches candidate centers only
within18px of the predicted center,about77--113px. It repeatedly reports
`no_silhouette` at that expected position. The approximately60px center mismatch
persists during both stationary observation windows. Repeating the view from
the same pose does not correct that mismatch.

Primary recognition also fails to confirm the seat. This log does NOT include
the exact rejected primary blob bounds/status or a saved full image,so it does
not uniquely identify the primary rejection gate or cause of the optical/map
disagreement. The first centroid17is outside the primary30--290pixel window;
later35--36centroids are inside it,so that window alone cannot explain all
failures. Segmentation,estimated pose,camera phase and range/ambiguity gating
remain alternatives,not established hardware defects.

One source-supported conditional failure chain deserves a targeted check:
if the component touches imagex<=2,range code substitutes forward170mm rather
than the measured ground foot. At pose261.1,-1210.5,41.8 and centroid35:

| Range branch | Ray range | Projected XY | Distance to seat5 | Pair-midline lateral |
| --- | --- | --- | --- | --- |
| If edge-clipped:forward170 |191.60mm|422.11,-947.99|91.49mm|52.01mm,inside70mm ambiguity band|
| If unclipped:foot190 with52/33600 model |274.42mm|451.42,-870.53|56.82mm|129.47mm,outside ambiguity band|

Both snap distances are below140mm,but the clipped branch would be rejected
by the separate rail-ambiguity rule. Actual minX/maxX are missing,so the clipped
condition is NOT proven by centroid/width alone. Do not silently replace this
conditional calculation with a root-cause claim or remove the ambiguity guard.
Calculations saved in ignored `local_workspace/ccw_523_525/optical_projection.json`.

## Second run: exits; excluded driving acceptance

Exit diagnostics complete60575ms,75samples,untruncated. Similar stopped view,
roughly263.5,-1200.9,41.6degrees,is followed by a confirmed GREEN seat5 and an
accepted connector. Some trial connector shapes reject tracking before another
passes; these are planner alternatives,not separate driven failures. The driven
connector completes at merge9,error18.8mm/heading14.8degrees. Later stall/manual
pause occurs at74328ms; user reports physically standing in the way. Do not use
that stall as a free-running route failure or count the run as successful race
acceptance. It does show that this start can succeed with different image evidence.

## Archive, limitations and next action

Existing parking-exit analyzer processed both originals below
`local_workspace/ccw_523_525/pose/`,retaining rear positioning,braking,reversal,
localization and pose reports. Both exit diagnostic streams complete without
overflow/ordering/duplicate-ToF errors;523whole-run ending is still missing.
Rear-marker total-motion residuals3.05--3.51mm fit existing uncertainty; this
does not identify backlash or justify a neutral/pose correction. No dedicated
ToF readings,sweep,movement or pause was requested.

Read-only investigation: no firmware edits,build,upload or motion commands.
The third RAM log cannot be assumed recoverable after the reported reset/new
run,and no corresponding source archive file was found. Preserve the user report
as an unverified failure,without assigning it523's cause.

Next implementation investigation: CCW start observation mismatch. Log the actual
primary bounds/rejection and acquire a geometrically distinct checked view when
stationary evidence remains UNKNOWN; retain valid-seat/footprint checks. Check
edge-clipped ground-foot ranging against full-image evidence before changing
calibration or rail ambiguity. The6a07252e changes address normal-route recovery;
they do not claim to fix this separate parking-entry observation timeout. Use
normal saves,archive originals FIRST,and record actual installed firmware on
the next acceptance run. No new dedicated ToF test requested.
