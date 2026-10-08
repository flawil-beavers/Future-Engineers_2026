# Layout B CW final-parking review, 2026-10-06

## Received evidence

Complete original logs 500 and 501 are in
`simulation/evidence/parking_exit_diagnostics/`, with individual hashes and
firmware header identity in its README. Captured unchanged before analysis.
Both reach LAP_2, LAP_3, FINAL_RUNOUT and FINISH. The user reports both
three-lap drives perfect, with no contact. This accepts this B CW lap batch,
not final parking or every other layout.

- 500: approach preflight passes, outward shift, scan starts/arms, then
  `dual_marker_scan_incomplete`. No capture or entry segments. User sees a
  straight overshoot and stop. The entry is armed; scan acquisition failed.
- 501: approach preflight passes and scan starts, but no scan-armed/completion
  or entry record; eventual Manual disable. User reports grazing pink during
  return and getting stuck on subsequent forward movement. Do not infer entry
  execution from this description: telemetry contains no entry segment.

These logs lack final-parking pose/raw-ToF periodic samples. Legacy navigation
Target heading is unrelated to current parking control. Exact collision pose
(and whether a scan phase missed wall or pink) is not recoverable.

## Rule check and shorter route

The canonical January 15, 2026 PDF and official Q&A were checked again.
[Scoring 1.2, printed page 21](https://wro-association.org/wp-content/uploads/WRO-2026-Future-Engineers-Self-Driving-Cars-General-Rules.pdf)
counts three laps when the complete vehicle leaves the last corner. The
whole starting straight need not be crossed again. Rules 9.22/9.24.4 require
return to the finish/start section and govern final stop/parking.

The existing middle-seam lap counter stays conservative. Its extra runout
is shortened 150 to 90 mm, with an actual-position projection as well as path
progress, avoiding an early finish due to nearest-waypoint rounding. This
retains clearance past a middle start pillar. The scan alignment lower bound
is shortened x=900 to 680 mm; actual alignment and swept clearance can still
require further travel. This is not a claim to the shortest possible route.

## Pink clearance and localization changes

Old nominal straight scan body/marker-tip reserve was 274.6-200-62.5=12.1 mm.
The entire seven-segment capture/entry trajectory is translated 20 mm inward:
scan/capture local y=294.6, field y=-1205.4; final centre local y=120.
Nominal straight scan reserve becomes 32.1 mm. At the final parallel pose the
complete 125 mm width ends at y=182.5 in a 200 mm deep bay: 17.5 mm reserve
at its open edge, still entirely inside. Swept entry checks use actual wheels,
measured gap, pillars, both pink pieces and outer wall.

The normal sensor pipeline deliberately selects the furthest accepted object.
That can prefer the wall over a simultaneously visible pink piece. Parking
now selects the nearest hardware-valid, filter-accepted cached object; general
navigation is unchanged. This is a demonstrated code issue, not proven as the
specific cause of 500 (raw object frames are absent).

Before scanning, two consistent fresh wall returns establish lateral pose,
with bounded correction and re-alignment outside the pieces if necessary.
During scan/capture only fresh wall-consistent returns update y, at most 5 mm
per frame; marker mixtures never apply wall corrections. Scan classification
also rejects stale frames. Scan/capture get a 15 mm actual-steering collision
forecast. Invalid sensing/geometry still aborts instead of driving blind.

Bounded FINAL_PARK_TRACE records include state/phase, actual estimated pose,
steering, speed/encoder, selected and nearest raw ToF, age, expected wall and
last wall correction. Vision console chatter is suppressed only while final
parking is active, preserving processing and space for parking evidence.

## Verification and next steps

Production-source CAD checks: 2807 later-lap cases pass; 2970 parking-approach
cases pass. Of 162 capture/gap perturbations, 156 accepted and six conservative
rejects; all nominal entry poses pass. Whole final-parking state machine is
checked for both directions, all 11 legal start occupancy cards, 30/100 ms ToF,
independent physical/estimated poses with -40/0/+40 mm lateral bias and mixed
wall/marker returns. All 132 sequences pass, including physical collision and final containment.
Final results and binary identity are recorded in AGENT_DOCUMENTATION.md.
Models omit real slip, servo delay, braking coast and optical reflections.

Next: user uploads M7 only, removes drive USB, unchanged B twice CW. Require
three laps plus scan completion, all seven entry segments and contained/stopped
FINAL PARK RESULT without pink contact. Then two CCW if CW parking passes.
Archive logs first. Inspect scan phase, both raw selections, lateral corrections
and actual clearance before making further parking changes. No agent commit.
