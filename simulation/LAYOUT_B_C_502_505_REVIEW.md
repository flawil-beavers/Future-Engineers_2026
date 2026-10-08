# B CCW discovery stops and B/C CW, logs 502-505

## Evidence and firmware

Complete original logs are archived in
`simulation/evidence/parking_exit_diagnostics/`; README contains individual
bytes/SHA-256, firmware markers and physical reports. Copied and checked
against removable originals before analysis. Directions are verified from
turn headers; layouts follow the user's reported order. User confirms no
contact at either CCW halt. Source removed after capture; no re-request needed.

All four have parking Oct__6_2026_22_37_23 and connector/corner
Oct6_2026_22:40:15. Final target y100, no FINAL_PARK_TRACE: these runs predate
the latest parking revision. Build headers do not establish binary readback.

## Stops in B CCW

- 502 reaches S3, acquires outer RED18, then stops on return to S0
  (fourth corner). Straight reverse90 mm, stopped RED height127-133 px;
  acquisition allows at most120. Extra90mm parallax produces RED width99-101
  px; acquisition allows at most80. Thus neither stopped view provides valid
  new RED evidence. The final straight return locks at heading deviation
  -3.01 degrees with cross-track0.1mm and encoder still30.5mm behind origin.
  A later unresolved-station hold would remain even without this angle abort.
- 503 stops at S3 entry (third corner). Reverse80mm; stopped RED75x123px,
  maxY202. Height123 exceeds120, so no valid acquisition despite visible RED.
  Returns to origin and locks unresolved, with heading -0.74 degrees and
  cross-track -0.7mm. This is a perception hold, not a collision or absence
  of a geometrically drivable track. Neither run completes lap1.

No RGB frames: bounding boxes demonstrate the rejection gates but cannot
prove whether optical merging, calibration, pitch or actual object distance
caused the oversized shape. Keep those alternatives explicit.

## Implemented corrections

Retain strict colour/shape, seat snap, voting and CLEAR evidence. A close
upright rejected RED/GREEN silhouette requests minimum300mm modelled seat
view range instead of230mm, retaining the existing 220mm reverse cap and
full all-potential-occupancy/wall swept preflight. Settling preserves the
request across frame dropouts. If the first stopped view reveals the close
silhouette later, extend that same straight reverse once within its checked
budget. This precedes optional extra parallax. No unresolved seat is skipped.
Logged nominal poses select160mm for502 and150mm for503; ordinary B495
recovery remains60mm. Modelled seat range is not an independent distance
measurement and optical acceptance still needs physical validation.

Main straight reverse/return now controls heading and cross-track relative
to the stored origin, steering capped at8. The existing3-degree/15mm abort
gates and20mm return-pose gate remain. Each steering command gets a15mm
CAD forecast with the same40mm unknown-pillar/wall margins. The original
constant-zero steering let drift accumulate to502's3.01-degree abort.

Two bounded CORNER VIEW OBS records per attempt preserve minimum view range,
resolution, nearest raw silhouette colour/size/foot and acquisition validity
even when the general discovery trace budget is exhausted. No new camera reads.

## Successful CW and parking limitations

504 B CW and505 C CW both complete all three laps; user describes drives as
successful. Old parking504 aborts dual_marker_scan_incomplete;505 aborts
approach_outer_gate before scan/entry. Neither records an entry segment.
Latest parking shorter approach,20mm offset, nearest valid multi-object ToF,
wall correction and bounded pose traces remain in the candidate; these older
logs do not accept or disprove it. Exact505 parking pose is not recorded, so
no speculative new parking detour is introduced.

## Verification and next

Production-function check_layout_c_and_join.py:173 checks pass, including
both logged recovery poses and54 +/-5mm/+/-2degree perturbations, close-vs-
ordinary view selection, bounded correct-direction steering and safe15mm
forecasts, plus previous C/B join, inference and ambiguity regressions.
Final M7 build/hash are in newest AGENT_DOCUMENTATION.md. Models omit image
formation/voting timing, slip, mechanical backlash and servo/braking lag.

Next user uploadM7, unplug driveUSB; unchanged B twoCCW. Verify both formerly
blocked corners resolve, all three laps complete and automatic parking ends
contained/stopped without contact. Then oneB CW and oneC CW on same firmware,
including parking. Archive originals first; inspect CORNER VIEW OBS and
FINAL_PARK_TRACE before changing gates or trajectory again. No agent commit.
