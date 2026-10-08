# CW final parking: logs 506--507

## Evidence and firmware

Complete originals and individual hashes are recorded in
`simulation/evidence/parking_exit_diagnostics/README.md`. Both runs are CW,
connector identity `Oct 6 2026_23:10:00`, with final-parking traces and the
120 mm local entry target. Layout assignment C then B follows the tester's
report. Both completed three laps; neither reached a parking-entry segment.

## Findings

- **507, missed first pink piece:** short returns (61/66 mm) occur at estimated
  rear-axle x709/693 before the old scan-arm x560. The subsequent piece near
  x433 is therefore counted as the first, and the scan waits for a nonexistent
  second piece until it aborts at x120. This directly explains straight travel
  past the bay without an entry command.
- Marker locations imply approximately 170--180 mm longitudinal odometry
  offset from nominal geometry. This is an inference from sensor/geometry,
  not independent physical position measurement. A 25 mm registration bound
  cannot accommodate this run even after fixing the missed-piece transition.
- Mixed returns near piece edges can enter the old wall feedback's 40 mm
  residual window. Repeated 5 mm corrections can shift the estimated lateral
  position and induce large steering changes. Traces support this mechanism
  but do not record every acquisition.
- **506, independent gyro interruptions:** sensor-report timeouts, stream
  restarts and approximately 1.6 s polling gaps occur during approach. It
  remains in state 1 and ends near x449/y-999, followed by manual disable.
  There is no marker-search or entry abort. A parking-geometry change cannot
  establish that these gyro interruptions are resolved.

## Changes

1. After lap 3, hand over once the complete body and both steered-wheel
   envelopes have left the last corner. The conservative start-straight
   rectangle includes 10 mm body padding and 250 mm longitudinal position
   reserve. Laps 1 and 2 keep their existing handover. Rule basis: scoring 1.2
   and official Q&A, linked in `WRO_2026_RULES.md`.
2. Scan immediately from the established wall baseline. Require the complete
   wall/piece/gap/piece/wall sequence, with two fresh confirming frames per
   transition. No nominal x560 delay.
3. Allow bounded longitudinal registration up to 250 mm only after both
   pieces, measured gap (within 15 mm) and wall reference agree. The lateral
   registration bound remains 25 mm. Search endpoint includes the same
   longitudinal allowance; distance/time limits remain active.
4. During scanning, accept wall feedback only within 10 mm residual;
   apply gain 0.2 and at most 1 mm per accepted frame. Initial acquisition
   still requires two consistent fresh frames. Register lateral drift already
   on the straight approach (wall range up to 700 mm), before the outward
   bend. The marker's 200 mm depth difference is outside the 80 mm initial
   residual allowance.
   If registration changes the scan-line position, settle it while returning
   west through the free corridor, not by continuing toward the east wall.
   The marker state machine is gated until scan-line Y and heading are ready.
5. Keep the seven entry segments, measured capture and final containment
   checks. Add at most six explicit `FINAL PARK SENSOR HOLD` records if the
   main gyro-health gate stops parking. No bypass of the gyro gate.
6. Choose the earliest swept-safe outward shift at x500/550/600/650.
   Beyond x500 continue the free straight lane instead of
   following the learned next-corner bend. Bound approach x at 1100 to retain
   physical wall reserve even before longitudinal registration. Wider,
   later shifts were rejected by the model: they consumed the room needed
   to settle the scan heading and correct lateral drift.

The learned bypass is retained while still inside the start section:
simply holding the initial lane there failed legal-card preflights. Also,
introducing an arbitrary +/-180 mm error before the entire approach exposed
collisions in the ideal model. The marker registration allowance therefore
does **not** establish safety under that much unknown error before the scan.
The focused registration cases inject X error only at scan start; this
separates the measured-marker failure from the unverified approach-pose
problem. Independent start/approach position accuracy remains a limitation.

## Validation and next runs

Completed on 2026-10-08:

- Later-lap state/geometry: 2,807 cases pass, including new final-corner
  handover positives in both directions, corner-not-clear negatives and
  exclusion of lap 2 / diagnostic modes.
- Final approach: 2,970 cases pass with neighbouring signs and +/-5 mm,
  +/-2 degree initial perturbations. Entry preflight accepts all nominal
  cases; six tight off-nominal cases are rejected among 162 configurations.
- Five registration cases pass: valid paired geometry accepted, one piece,
  wrong gap, excessive X correction and excessive Y correction rejected.
- Full production state machine: 396 cases, 394 physically contained
  parking completions, two safe aborts at nonzero scan-stage X error, zero
  collisions or false completions. All 132 cases without injected X error
  park successfully, with +/-40 mm initial lateral bias, both directions,
  all 11 starting cards and 30/100 ms sensor periods. Controller step 20 ms.
- IDE M7 build succeeds: RAM 432880/523624, flash 500920/786432 bytes.
  Firmware SHA-256:
  `70b1197ce1e8e0fa161e4ead2d0825395f20fe7e84d7a603fceaf88b6957cabd`.
  No M4 change, upload or agent commit.

Ideal CAD/sensor tests exclude wheel
slip, brake/servo delays, specular returns and hardware gyro failures; they do
not replace a physical parking test. Next: upload M7, remove the driving USB
cable, two B CW runs through all three laps and final parking. Then C CW and
CCW on the same firmware. Keep normal complete log saving; no extra ToF sweep,
movements or pauses are required. Do not change a successful exit route based
on these two parking failures.
