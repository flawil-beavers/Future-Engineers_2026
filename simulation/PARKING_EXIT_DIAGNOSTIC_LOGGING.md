# Parking-exit diagnostic logging

Parking-exit diagnostics run automatically during the existing `O` unparking
sequence. They observe rear-ToF positioning, the five exit segments, braking,
direction changes and edge localization. They use cached sensor values and do
not add motion, delays, sensor reads, control changes or a separate save step.
Save the run through the normal robot procedure.

## Firmware switch and buffer

`PARKING_EXIT_DIAGNOSTICS_ENABLED` in
`include/parking_exit_diagnostics_config.h` controls the complete feature. Its
default value is `1`. Setting it to `0` compiles out the diagnostic calls and
restores the original 128 KiB log buffer. Enabled builds use 192 KiB. Each run
is limited to 150 periodic records and a 64 KiB diagnostic contribution;
state, correction and finish records remain higher priority. Periodic records
have a 57,000-byte sub-budget; events have an 8 KiB sub-budget, including
reserved space for a truncation marker and the final event. Both budgets count
CRLF. The formatting buffer holds 512 bytes, and an incomplete formatted sample
is discarded with an explicit truncation marker rather than emitted as data.

## Schema 2

Every run starts with `[PARK_DIAG_CONFIG]`. It records the schema, build time,
exit direction, configured servo centre, encoder conversion, limits, buffer
size and relevant manoeuvre constants.

`[PARK_DIAG]` records contain `v=2` and one of these `type` values:

- `state`, `direction` and `rebase` identify control-state boundaries;
- `sample` contains time in milliseconds, segment, commands, encoder count and
  millimetres, measured speed in mm/s, gyro heading and two poses;
- `correction` records the pose before and after an existing localization
  correction;
- `truncated` identifies a sample-count, sample-byte, event-byte or formatting
  limit; state events and the reserved final event can continue;
- `finish` identifies normal completion or the abort/hold reason.

The `pose=x,y,heading` value is the firmware estimator. `nominal=x,y,heading`
is a diagnostic-only Ackermann integration of encoder distance and commanded
steering. ToF fields `s0`, `s1` and `s2` are left, right and rear. A fresh value
is `sequence,age_ms,raw_mm,filtered_mm,signal_mcps,sigma_mm,valid,accepted`.
`same` means the cached sequence has already been recorded, and `none` means no
snapshot was available. `ref` names a physical reference only where the current
state identifies one.

The existing logger's `LOG BUFFER OVERFLOW` marker still indicates whole-log
overflow. It is separate from diagnostic truncation.

The analyzer rejects samples with missing/partial ToF fields or exceeded
declared sample/byte limits. A new invalid ToF observation invalidates older
evidence; only `same` may reuse a previous valid observation within its age
limit. Rear-ToF age is measured from receipt on M7, not from acquisition on M4:
the current protocol contains no acquisition timestamp. This limits conclusions
about short reversal transients; changing the protocol is outside this test.

## Analysis

From the repository root, run:

```powershell
python simulation/analyze_parking_exit_pose.py path/to/log.txt
```

Results go to `local_workspace/parking-exit-analysis/`. The Markdown report and
CSV files and one SVG trace per log include input hashes, schema/build identity,
completion, ordering, overflow, duplicate sensor data, segment errors, braking
travel, repeatability and reversals.
A reversal is assigned an effective lost-motion estimate only when valid ToF
samples observe the same unambiguous rear marker on both sides. Other cases are
reported as unobservable rather than guessed.

The servo-centre candidate comes from the zero-curvature intercept of gyro
heading change per signed encoder distance versus logical steering. Only moving
rear-positioning or localization intervals with unchanged steering are used.
The report separates estimates by travel direction, exit direction and steering
approach when enough samples exist. Treat disagreement as backlash or hysteresis
range; never copy one candidate into `SERVO_CENTER` automatically.

Encoder position is measured upstream of wheel/gear play. Differences between
encoder and ToF motion can also come from tire slip, compliance, imperfect wall
geometry or sensor noise. Robot sensors provide relative evidence, not external
ground truth.

## Physical test handoff

The tester runs the unchanged normal `O` procedure and saves the log normally.
For each run, record only the log number, firmware identity and physical result
(exit direction, visible contact, unexpected stop or unusual motion). Repeated
runs in both directions are needed before changing steering calibration or
adding backlash compensation.

When a batch is returned, the receiving agent must copy every complete raw log
unchanged to `simulation/evidence/parking_exit_diagnostics/` using
`YYYYMMDD_log_NNN_cw.txt` or `YYYYMMDD_log_NNN_ccw.txt`. It must calculate the
SHA-256 hash, add the run and physical report to that directory's `README.md`,
run the analyzer from the tracked copy, and summarize the result in
`AGENT_DOCUMENTATION.md`. If only part of a log is available, save the exact
part with `_excerpt` in its name and clearly mark the run incomplete. Generated
reports stay below ignored `local_workspace/`; the committed raw logs are their
reproducible inputs.
