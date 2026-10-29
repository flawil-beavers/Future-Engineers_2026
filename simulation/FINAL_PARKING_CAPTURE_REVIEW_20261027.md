# Final parking after three laps: CW and CCW

## Evidence and scope

Rechecked the third-lap handover in `src/obstacle.cpp`, learned-route approach,
wall/dual-marker localization, capture, seven entry segments and final
containment in `src/final_parking.cpp`. Normal final parking is enabled, entry
is armed and both lap-test bypasses are disabled. No new robot runs or logs
arrived during this review; no firmware was uploaded.

Reused unchanged archived evidence:

- `evidence/parking_exit_diagnostics/20261027_log_564_cw.txt`, SHA-256
  `72fb479284dc4f81e2ea222b8aa5c7ae8386b063a9a2ad1752c82a92a75b2df4`:
  registered pose X226.6/Y-1194.8/heading182, only16.9 mm to capture,
  ultimately `capture_pose_gate`. Existing out-and-back alignment fixes the
  insufficient distance; it must remain available.
- `evidence/parking_exit_diagnostics/20261027_log_567_ccw.txt`, SHA-256
  `e1cbb0c6089e114677a8987e7912ea9926aa9aa8b54ba428dfeb637b61563915`:
  `initial_outer_wall_reference` with roughly102 mm lateral disagreement.
  Existing bounded stable-wall registration and cleared-corridor alignment
  remain in place. This is not independently measured absolute pose error.

These originals predate the current corrections. See
[the batch review](CW_CCW_563_569_REVIEW_20261027.md) for physical reports and
the previous fixes; neither log demonstrates current physical success.

## Changes

Previously any registered capture pose outside the5 mm/1 degree line gate
selected the extra out-and-back alignment. This also affected CCW poses with
hundreds of millimetres already available on the ordinary route to capture.

New `captureDirectPreflight` first checks that ordinary route with the actual
line controller,1 mm CAD steps, measured bay gap, known pillars and existing
body/wheel/wall margins. It requires the existing endpoint tolerances and
the complete subsequent seven-segment entry sweep. Only then does it omit
the extra alignment. Otherwise the previous checked alignment remains.
The runtime motion guards, final capture gate and entry preflight still apply.

The appended capture-alignment drive/brake states now also enter the module's
central pose/gyro/clearance health gate. The main loop already had a global
gyro hold; this closes the local omission and protects invalid poses too.
It is not established as the cause of an archived physical failure.

No speed increase, entry-arc tuning, departure change or relaxed sensor,
braking, footprint or final-containment threshold is included.

## Validation

All tests below use current source and the regenerated `check_later_laps.py`
host fixture. Generated programs/results remain under `local_workspace/`.

| Check | Result |
| --- | --- |
| Later-lap fixture/routes | 2807 passed; no navigation change beyond revision text |
| Full final-parking state machine | 396 parked,0 failed; CW/CCW,11 layouts,30/100 ms ToF, independent lateral and scan-X biases |
| Broad parking robustness | 990 known-pose arrivals all parked; varied gaps, headings and arrival offsets |
| Other robustness cases | 132 unknown-pose cases:88 parked/44 safely rejected;360 arbitrary starts:112 parked/248 rejected;40 other-section starts,16 sensor faults and2 wrong gaps rejected; no modeled unsafe case |
| Boundary/ToF quality | 46 boundary and5 quality checks passed |
| New capture comparison | 72 registered synthetic poses:64 parked,8 existing clearance rejections retained;36 direct cases shorter, no previously successful case lost |
| Appended-state health | 12 checks passed; removing the new local guard reproduces an unhealthy-gyro drive command in the isolated module test |
| Braking | 90 checks passed, now including capture-alignment braking |
| Prior CCW567 wall correction | 18 perturbed/mirrored sensed-drift alignments passed; old80 mm limit negative control fails as expected |

In the36 shortened capture cases, the largest saving is approximately162 mm
and4.32 s at unchanged speed. This is a model comparison for the affected
alignment cases, not a promised reduction for every parking run. The short
CW564 reconstruction still takes its necessary alignment and completes entry.
Eight synthetic capture perturbations already fail clearance/preflight in the
previous version; the test retains those refusals rather than reducing margins.

The CAD/ideal-ToF models exclude real steering delay, tyre slip and sensor
reflections. Estimated and true poses are separate where stated, but unknown
heading errors can still lead to a safe final gate rejection. These results
do not establish reliable parking from every possible physical position.

## Firmware and next validation

Revision: `20261027_parking_direct_capture`.
IDE-managed M7 build SUCCESS: RAM442416/523624, flash532600/786432 bytes.
Binary SHA-256:
`7ccf911449dd8d50a19c4b959fd40d6ca1d1e80f200653ba84e958678b029c94`.
M4 unchanged/not built. Existing `Serial` macro warning remains.

Next use the normal full CW and CCW runs after the user uploads M7. Save
normally; copy originals FIRST, verify SHA and immediately notify removal is
safe. Confirm revision, third-lap handover, measured marker gap, direct-capture
message only where feasible, retained CW alignment, all seven segments and
`FINAL PARK RESULT contained=yes stopped=yes`, alongside the physical contact
and containment report. No special ToF sweep or extra motion requested.
