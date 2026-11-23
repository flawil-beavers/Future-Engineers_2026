# Returned runs 582--593: distant start acquisition and row assignment

## Evidence and scope

Original logs were copied before analysis and verified against their sources by
SHA-256. Portable paths and full hashes are recorded in
`evidence/parking_exit_diagnostics/README.md`. Logs 582--586 are five CW runs on
`20261029_start_foot_scout_evidence`. Logs 587--589 are CCW and 590--593 are CW
on `20261029_ccw_scan_fit`; the diagnostic build timestamp alone is not a full
firmware identity. Installed binary hashes were not read back.

The three supplied phone photographs document obstacle positions and viewing
geometry, but they are not robot RGB565 frames and are not used to change colour
thresholds. The user reports that recognition is generally weaker at distance,
while a later different arrangement completed the course. Exact physical
outcome-to-log mapping is incomplete, so conclusions below use only explicit log
events. Parking outcomes are indexed but parking remains outside this correction.

## Measured findings

- 582 completes all three laps. At the first post-start GREEN, two early views
  favour the nearer row but remain `row_bearing_pending`; the later oblique view
  correctly confirms outer seat 7.
- 583 confirms nearer seat 6 from an early still-ambiguous view. A later view at
  pose -572/-733/173.9 degrees points to seat 7 with 76 mm snap error but is
  rejected as `opposite_confirmed`. This is a deterministic row-commit defect.
- 584--586 receive usable GREEN observations for seat 7, but votes are separated
  by more than the 400 ms ordinary vote window or the close blob becomes broad.
  Their safe holds/recoveries remain unresolved. No image exists to justify a
  general colour or width relaxation.
- 587 and 589 repeatedly detect a production-valid GREEN at the CCW start end
  station, but `start_foot_mismatch` rejects feet around 184--190 px. The mapped
  residual is about 22--28 px. The previous false close RED471 and false middle
  GREEN576 require the original tighter guard.
- 589 later obtains one geometrically matched upright GREEN candidate at
  23757 ms. At 23949 ms the station is nevertheless declared CLEAR after a
  dropout, an ordinary connector without avoidance starts, and progress stalls.
  A recent occupied candidate therefore did not survive the five-frame clear
  decision.
- 588 confirms GREEN5, GREEN7 and RED10, then recovers RED19. On the later start
  return it sees a broad RED region (115--117 px width) which remains invalid and
  locks after the bounded recovery. Without the robot image, accepting this blob
  would also accept unknown connected foreground; its width/colour gates remain.
- 590 and 593 confirm the complete first-lap maps and finish three laps. 591
  completes lap 1 and later reports zero encoder progress while commanded on lap
  2; the log alone cannot distinguish physical obstruction/contact from a drive
  fault. 592 is a 1.99 s interrupted start, not a course run.
- Offline exit analysis finds three completed/untruncated CCW first-edge exits
  (587--589), three completed/untruncated CW short exits (590, 591, 593), and one
  incomplete legacy/unidentified start (592). Exit motion is not the common
  cause of the recognition failures.

## Scoped correction

Revision `20261030_far_start_row_evidence` changes only first-lap evidence
handling:

1. A GREEN at either end station of the parking/start section may use a 30 px
   mapped-foot tolerance. RED and the middle station retain 18 px. Thus the
   returned 22--28 px true GREEN band is admitted while RED471 and middle
   GREEN576 remain rejected.
2. A geometrically matched upright GREEN candidate blocks CLEAR for 600 ms.
   This does not confirm a colour or seat; it only prevents the 192 ms
   occupied-to-clear transition from 589. Normal five-frame clear evidence works
   after expiry.
3. When both rows remain plausible, CW GREEN row selection now stays pending
   even if the bearing winner equals the depth-nearest row. A later decisive view
   can confirm seat 7 instead of being rejected by an early seat 6 commitment.

No HSV thresholds, blob width/height limits, ordinary two-vote confirmation,
speed, parking controller, exit motion or later-lap route changed.

## Verification and limits

Production-source tests pass: 14 archived disputed-row observations including
582/583 with the wrong-row negative control; 54 third-GREEN routes; 27 coupled
unknown-to-confirmed routes; 48 far acquisitions; RED return and wall-drift
checks; archived/synthetic vision comparisons; false RED471 and middle GREEN576
foot controls; the 589 occupied-to-clear timing test; and 2807 later-lap cases.
The existing 580/581 fitted scan cases also pass.

IDE-managed M7 build succeeds: RAM 442552/523624, flash 538712/786432. Binary
SHA-256 is `195b01565b7b7589d7317d8f30dcaa003b4a1642c41462690b3c5aa241b9fc4f`.
M4 is unaffected and was not built. No upload, robot movement, commit or push was
performed.

Physical acceptance still requires normal CW and CCW runs. In particular, the
broad connected RED view from 588 remains deliberately unresolved until a robot
camera frame or a successful earlier acquisition demonstrates a safe rule.
