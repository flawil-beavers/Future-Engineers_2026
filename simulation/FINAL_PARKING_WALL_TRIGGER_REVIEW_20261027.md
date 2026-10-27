# Final parking: wall detection versus entry permission

This is a source and host-model review, not an analysis of the four new CW
runs. Their original logs remain unavailable. Firmware was neither modified
nor uploaded, and the installed firmware identity is unverified.

In the current working source, TEST_ONLY=false and ENTRY_ARMED=true. There is
no configured test stop before entry. The outer side ToF, rather than the
camera's colour detector, measures the bay. A wall return alone deliberately
does not start the entry manoeuvre.

The scan requires two fresh classification frames for each transition:
wall baseline, fixed pink piece, wall in the gap, movable pink piece, wall
after that piece. Only hardware-valid, filter-accepted object ranges count.
The nearer accepted object takes precedence over the wall in a mixed frame.
Marker maximum range is 180 mm; wall range is 190--400 mm and must agree with
the predicted wall within 25 mm. Scan frames must be at most 250 ms old and
have a new sequence number. A wall return during the wrong scan phase cannot
substitute for a missing pink piece.

After the scan, the measured gap must agree with nominal within 15 mm and
position registration must pass. The robot then moves to the computed capture
point, brakes to measured standstill, and checks X within 5 mm, Y within 5 mm
and heading within 1 degree. Finally a complete seven-segment swept check of
body and wheels against the pieces, walls and confirmed pillars must pass.
The rear wheel appearing beside a pink piece is not a complete pose check.

Host results against the current source:

- `check_final_parking_sequence.py`: 396 complete modeled sequences parked;
  five registration acceptance/rejection checks passed. Includes both travel
  directions, ideal cone ToF, independent physical/estimated poses and drift.
- `check_final_parking_approach.py`: 2,970 approach cases passed. Of 162 entry
  tolerance samples, 156 passed the swept entry check and six were rejected.
  Nominal poses all passed. The three CW rejections combine a 242.5 mm gap,
  X offset -5 mm and heading offset -1 degree, at Y offsets -5/0/+5 mm. CCW
  mirrors this with X +5 mm and heading +1 degree. Thus being inside the
  capture tolerances does not guarantee sufficient clearance along the entire
  entry; the stricter sweep is authoritative. Do not weaken it to force entry.

These tests exclude real sensor reflection/latency, tyre slip and servo lag.
The sequence's gyro-pause injection is at the first straight entry segment;
it does not validate actuator recovery in every curved segment. Source review
also notes that FP_SEGMENT_DRIVE does not explicitly restore servo_disabled
after a main-loop sensor hold, whereas driveFieldLine does. The previous servo
angle normally persists; this is not evidence for the reported initial entry
refusal. Do not conflate an actuator enable flag with measured wheel angle.

For each returned CW original, distinguish the final event:
`dual_marker_scan_incomplete`, `dual_marker_geometry`, `capture_pose_gate`,
`entry_swept_preflight`, or `FINAL PARK SENSOR HOLD`. Inspect the preceding
FINAL_PARK_TRACE state/phase, raw versus expected range, pose, and scan/capture
records. Archive originals unchanged FIRST, then compare the three parking
failures. Do not infer a camera fault, remove a safety check, or change entry
geometry solely from the physical report. No extra sensing sweep is needed.
