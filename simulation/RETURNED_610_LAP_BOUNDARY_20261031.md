# Returned log610: false cached middle and lap-boundary hold

Complete unchanged evidence: [log610](evidence/parking_exit_diagnostics/20261031_log_610_cw.txt),
96172 bytes, SHA-256 `f4630b160912c74fc01850b23f64d31dc59133ea78988b4dc05f4a07404ef7bc`.
Copied before analysis; source-before/source-after/copy matched, unplug notice sent.
Archive starts with CW (turn=-1), build Oct 31 2026 13:37:20,
path `20261031_retreat_escape`, exit `20261031_frame_bracket`.
Installed binary not read back. User reports stopping in the middle after one lap.

At exit import, three historical GREEN frames confirmed seat2 (start-section
middle), snap25.6mm, corrected historical pose390.1/-1366.3/150.0deg,
bearing-25.7deg/range517.7mm. That immediately resolved the normal middle scan.
Fresh views subsequently confirmed start seats0 GREEN and4 RED. All other
sections acquired legal layouts: S1 end pair, S2 middle GREEN, S3 end pair.
At the seam the start section contained an impossible middle plus two ends.
The exact recorded stop is `[LAPS] Boundary held: map incomplete or
contradictory; no blind lap 2`. Manual pause ended the first run after120.505s.
This was normal obstacle mode, not practice mode or the one-lap test.

The archive also contains a later turn=1 restart. Its rear positioning failed
stationary verification: planned18.2mm forward, filtered rear range47.3->40.3mm
instead of increasing; failed after5.898s and later manually paused. Whether
this restart was placed in the normal start lot, touched or repositioned is
unknown; do not attribute it to the lap-boundary stop or bypass its sensor check.

## Narrow correction

Historical exit observations now seed at most one unconfirmed pending vote.
They cannot inject a path, resolve a station or infer empty seats by themselves.
A fresh normal observation must match the seat and colour and pass existing
association rules before confirmation; hints expire after the existing400ms
window or clear on a different accepted seat/colour. Re-importing cannot confirm.
The original normal scan and exit movements remain. This conservatively trades
some cached-only scan skipping for protection against a falsely committed seat.
Normal far detection, speed, parking geometry and lap safety checks unchanged.
Path revision `20261031_exit_vote_hint`; startup remains normal obstacle mode.

Regression: observer1090 checks plus12 handover, including both directions,
repeat import, fresh confirmation, stale hints, conflicting fresh colour and
cached-middle/fresh-end-pair replay; far acquisition52 checks PASS. Later-lap
state and route regression recorded in the engineering handoff.
M7-only IDE-managed build SUCCESS; unchanged M4 not built. Host tests establish
state/geometry behaviour, not physical camera reliability. No upload or motion.

Next: upload M7, start from the normal lot, run/save normally. Secure originals
first. Check that exit reports pending hints, normal current views confirm actual
start seats and `LAP_2_RECORDED_MAP` follows the first seam; then verify third
lap and final parking. Preserve sensor failure diagnostics if a new normal-lot
rear-positioning failure recurs.

Later-lap state/route regression completed:2807 cases,0 failures; invalid-map
negative controls preserved.
