# CW576: false middle confirmation skips the near-start observation

Original: `evidence/parking_exit_diagnostics/20261029_log_576_cw.txt`,83786bytes,
SHA-256 `9bde5b821ded2d659f3c1f84257e7788f2749475b3691d6bf2e084d5910c0a5a`.
Copied FIRST, source/copy SHA verified, safe-removal notice sent immediately.
Revision `20261029_start_view_park_reference`; installed binary hash unknown.
Complete saved file and exit diagnostics; RUN_END stopped/mode_pause,truncated0.
User reports wrong CW start, no proper green inspection, red missed, poor route.

## Evidence and cause

Exit completes, then first65mm scan stops308.1/-1209.7/148.5. At248028ms the
local green silhouette fallback confirms middle GREEN2 from three frames:
foot208px, range232mm, snap86mm,26 green samples,47 brightness contrast. Geometry
for that seat is317mm/-19.1deg, expected ground-foot about164px. The44px error
exceeds the existing12px mapped-foot tolerance. The function set
`silhouetteFound` only when that tolerance passed, but still returned a green
candidate when it failed. The caller's broader140mm range allowance accepted it.
These are recorded candidate metrics; run RGB565 images were not recorded, so
the source of those pixels/reflections remains unknown.

The primary-result transition clears observing/active before calling the scout.
That briefly enables normal official-layout inference. Confirmed middle2
implies empty end stations0/2; `stationResolved(0)` returns true. Log explicitly
says Station already resolved station0, then launches the connector. Thus the
new extra CW reverse arc never executes at all in576. Normal known-layout start
planner tests could not expose an incorrectly learned middle plus this flag gap.
RED4 obtains one accepted primary vote at252211ms but no confirmation. Inference
removes the unresolved-place hold after joining; no explicit RED4 conflict
rejection is logged. Later explicit official_middle_conflict records concern
GREEN20 in section3, not RED4. No claim that this single cause explains every
later physical deviation, camera synchronization issue or apparent stall.

## Correction

* Enforce the existing12px foot agreement before a local silhouette may supply
  GREEN colour votes. Keep HSV, sample counts, contrast, ordinary colour blobs
  and their acquisition independent. A real close green remains valid at its
  own projected foot. This is a missing geometric condition, not a new colour
  threshold or a relaxation of driving clearance.
* Add `stationDirectlyResolved`: independently observed clear or confirmed seat.
  The initial CW preceding scout checks that evidence rather than layout
  inference. Directly observed clear/occupied places still skip redundant motion.
  Normal lap layout inference and successful routes remain available.
* Retain the previously checked bounded CW view extension, CCW correction, CAD
  recovery and fresh stationary parking reference. No exit geometry/speed change.

## Validation and limits

`check_green_seat_visibility.py` compiles real Vision. Synthetic576 geometry
foot208/expected164 rejects; the same green with expected208 accepts. Removing
only the new gate reproduces acceptance with silhouetteFound=false. Archived
green and red comparison images and empty-front/distant-green cases pass, along
with dark/bright green, gray, wall strip and floor-line scenes. These controlled
images reproduce the missing predicate, not the unavailable576 scene.
Ten archived Oct26 green400mm images queried atx160/foot136 produce identical
old/new results:9 accepted, the same1 already rejected. This is a fixed-query
comparison, not a new distance calibration or full-course recognition rate.

`check_cw_576.py` executes the actual scout function with middle confirmed and
inferred near-empty. It now arms the scout; independently clear/confirmed near
places still join directly. Restoring only the old inferred predicate reproduces
the skipped scout. Geometry decisions are stubbed here and checked separately
by existing planner/view suites. Existing later-lap fixture includes the new
direct-evidence helper so source extraction stays current.
The regenerated later-lap suite passes2807 scenarios. CCW view/state/connector
and recovery checks81/162/27 also pass with this source revision.
Existing CW563/566 row ambiguity8, third-green view/route54, coupled first-green
unknown-to-confirmed27 and registered parking18 pass, including their negative
controls. CW570 view27 and CW571 stopped-reference/parking54 pass as well.

Build, complete validation results and binary SHA are in the dated
`AGENT_DOCUMENTATION.md` entry. Offline normal-exit/fan analysis stays ignored in
`local_workspace/cw-576/exit/`:174 unique observations,27 eligible under both fan
models,3 classification differences; no independent position accuracy inferred.
No dedicated sensor movements or special test pauses requested.

Next: tester uploads only M7, normal CW/CCW runs/save. Secure originals FIRST,
notify, archive/hash/index; verify no middle confirmation with incompatible
foot, explicit near scout when no direct evidence, actual GREEN0 and RED4
confirmation, normal route and final parking. Physical success of this revision
remains unvalidated. No agent upload/drive/commit/push.
