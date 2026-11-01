# Temporary single-run position capture (2026-10-31)

## Disabled after returned622/623

User requested normal driving/logging again. The temporary
`-DRUN_TELEMETRY_POSE_ONCE=1` build flag has been removed from platformio.ini.
Both telemetry defaults are0; no dense pose/ToF sampling occurs, even after
reboot. Ordinary driving/error/parking-exit diagnostics remain. Full obstacle
challenge startup remains selected. The instructions below describe historical
capture builds and must not be used to claim the current binary captures densely.

## Current revision after619 (supersedes the historical notes below)

619 exposed phase-triggered duplicate sampling and has no usable full-run
trajectory. Fixed the producers/rate limit. See
[617-619 review](RETURNED_617_619_REVIEW_20261031.md) for evidence and changes.

For the next filmed run, upload current M7 and reboot once. The first run logs:

- RUN_POSE: estimated XY/heading, timestamp t in ms, lap/reference frame,
  local/field flag and cached fresh left/right/rear ToF mm. Invalid/stale=-1;
  capped/out-of-range=9999. These are sensor-ray ranges, not proven wall identity.
- RUN_OBSTACLE: confirmed seat/colour, nominal seat XY, estimated robot pose,
  absolute and elapsed timestamps. This marks confirmation, not the first pixel.
- RUN_SYNC: first drive request with timestamp for video alignment; physical
  wheel motion may start later. Film before enabling motion.

Relative pose time = RUN_POSE.t minus RUN_START.t. Sampling is200ms initially,
500ms after72KiB and1000ms after80KiB, within96KiB pose/16KiB event budgets.
300s plus55KiB other diagnostics tested with >850 poses/ToF and intact footer.
No new sensor acquisition or deliberate driving pause. Estimated positions
can drift; frame corrections must not be drawn as real motion. Reboot re-arms
capture; another run without reboot has ordinary logging. Save normally.

The previously pending front reserve is now16mm in ordinary planning with
the checked tight-space fallback retained.617 later-lap inward divergence
has a targeted speed response. See review for limits and validation.

## Historical initial capture build

Logs617/618 secured unchanged before any history/analysis, source-before/source-
after/copy SHA-256 matched, unplug notice delivered immediately. Originals:
- evidence/parking_exit_diagnostics/20261031_log_617_cw.txt,94809bytes,
  SHA6204b033ca6d5feff0aace85252b714288f3e44753a93137a71669e2ccc4c460.
- evidence/parking_exit_diagnostics/20261031_log_618_cw.txt,169762bytes,
  SHAcbdc8b2d771cb462e62330636e0e997b528dfccfbe8a501cb232520142291995.

User report only: first run too far between GREEN/RED on second section of
laps2/3; second CW perfect. No driving analysis of these logs performed.
Another8mm front manoeuvre reserve/use less rear room is a pending request,
not implemented now. Route, speed, parking geometry/control unchanged.

M7 platformio build flag RUN_TELEMETRY_POSE_ONCE=1 temporarily enables capture
for the first run started after each reboot. Subsequent runs without reboot
use normal logging. Remove this one flag and rebuild M7 after obtaining the
requested test; rebooting the temporary build re-arms its first-run capture.
No persistent flash/EEPROM setting, serial test command or extra sensor read.

Compact records:
`[RUN_POSE] v=2 t=<absolute_ms> lap=<0..3> frame=<reference_epoch> space=<L|F> pose=<x_mm>,<y_mm>,<heading_deg>`

Cached encoder/gyro position estimates, not independent ground truth.200ms
nominal spacing,500ms after72KiB pose records to preserve coverage on long runs.
Pose sample budget96KiB,events16KiB,total112KiB; existing192KiB logger and footer
reserve unchanged. Small continuous estimator corrections stay in sampled
positions. Rebases and larger corrections get RUN_EVENT markers/reference
changes. Stationary periods are sampled too. After the96KiB sample budget,
RUN_TRUNCATED explicitly flags incomplete capture; footer remains protected.

Repeated projection/approach/later-track/camera and periodic final-parking
chatter is reduced ONLY while capture is active. Important confirmations,
run/lap/phase events, faults, final results and existing parking-exit ToF,
braking/reversal/correction diagnostics remain. Detailed route dumping stays
off; this task asks for denser measured trajectories, not new route analysis.
Existing visualizer accepts v2 field poses and separates correction/lap/local
frames instead of inventing motion across reference jumps.

Validation: actual telemetry writer's detailed/default suites PASS; one-shot
300second run plus55KiB diagnostics has >1000 poses, intact footer, no truncation,
and a second run has no capture/extra pose reads. Visualizer synthetic frame/lap
segmentation PASS. Firmware does not move the robot during checks.

Next: upload M7 only, reboot once, normal full CW run, save normally, secure
original immediately on return. Analyze/visualize that run later with617/618.
Do not change route or add another8mm before the requested evidence review.

M7 capture build SUCCESS RAM453816/523624 flash560616/786432 binary564168B SHA256 31b28a3bcc0f386644a2b0f6b8494a7c07d2ed6a7101dfb39b98fc835eb08ee7. M4 unchanged/not built; no upload/motion/commit/push.
