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

To regenerate analysis and pose SVGs for every archived complete log in one step,
run from the repository root:

```powershell
python simulation/analyze_parking_exit_batch.py
```

This selects files matching `YYYYMMDD_log_NNN_cw.txt` or `_ccw.txt` in the
evidence directory, excludes `_excerpt` files, and skips byte-identical complete
sources. It expands multiple sessions within one source and writes the report,
plots, `parking_exit_sources.csv` (source hashes, build IDs and plot names), and
`parking_exit_batch.md` to ignored `local_workspace/parking-exit-analysis-all/`.
The batch index links separate reports under `by-build/`, grouped by the
diagnostic build timestamp and all logged configuration values. Compare these
reports when exit revisions change; the top-level report pools historical runs.
The source manifest also links each session's build report. The diagnostic
object's timestamp is not a complete firmware identity: an incremental build
can retain it after other code changes. Check actual segment targets and the
evidence README before treating a group as one motion revision.
Explicit `[CW START] Short scan from initial ToF seed + exit odometry` and
`[CCW START] Second-edge reference reached; no extra reverse` records select
separate procedure groups even when that diagnostic header is unchanged. The
manifest records `observed_procedure`; absent markers mean legacy or unidentified,
not proof of a specific uploaded mode. Official CW short starts omit the long
second-edge reverse, while official CCW omits its additional 70 mm continuation;
O3/CHECK_ALL retains the legacy selection. Preserve the original log markers.
Re-running refreshes the reports; use the manifest to identify current plots if
older SVG files remain in that directory. The command never copies logs from a
USB drive, changes robot control, or edits committed evidence. Archive each new
original log and its physical report first, as described below.

For one log or a custom selection, run from the repository root:

```powershell
python simulation/analyze_parking_exit_pose.py path/to/log.txt
```

Results go to `local_workspace/parking-exit-analysis/`. A source file containing
multiple `[PARK_DIAG_CONFIG]` headers is expanded into separately identified
sessions without editing the committed evidence. The Markdown report, CSV files
and SVG traces include source hashes/line mappings, schema/build
identity, completion, ordering, overflow, duplicate sensor data, segment errors,
braking travel, repeatability and reversals. `parking_exit_rear_motion.csv`
compares settled encoder and rear-ToF motion during rear positioning;
`parking_exit_corrections.csv` records the existing field-pose corrections.
Each field-pose session gets an `_exit_pose.svg` of the five exit segments and a
`_pose.svg` that also shows the later reverse edge localization. A rear-only
abort gets one `_pose.svg` in its local frame. Both axes use the same millimetre
scale; the field-pose reset is not drawn as robot travel. Purple marks an
estimator correction, which is also not physical motion. The SVGs show the rear
axle only; the separate `simulation/parking_exit_path.svg` is an idealized
parking-local footprint model, not a trace from a robot run.
A combined report summarizes only sessions with an untruncated
`unparking_complete` record; aborted/incomplete sessions still appear in the
per-session report and CSVs. Physical setup and outcome must be checked against
the evidence README. For newer firmware headers the analyzer also verifies the
separate sample and event byte budgets, counting CRLF as the firmware does.
A reversal is assigned an effective lost-motion estimate only when valid ToF
samples observe the same unambiguous rear marker on both sides. Other cases are
reported as unobservable rather than guessed.

The servo-centre candidate comes from the zero-curvature intercept of gyro
heading change per signed encoder distance versus logical steering. Only moving
rear-positioning or localization intervals with unchanged steering are used.
The report separates estimates by travel direction, exit direction and steering
approach when enough samples exist. Treat disagreement as backlash or hysteresis
range; never copy one candidate into `SERVO_CENTER` automatically.

The report also gives steering-approach-specific candidates and their midpoint.
Their span may show servo/linkage hysteresis or feedback lag, but it is not a
direct wheel-angle measurement. A centre change still requires a controlled
physical comparison.

Encoder position is measured upstream of wheel/gear play. Differences between
encoder and ToF motion can also come from tire slip, compliance, imperfect wall
geometry or sensor noise. Robot sensors provide relative evidence, not external
ground truth.

## Physical test handoff

Upload current firmware, perform normal runs and save normally. The offline
ToF assessment changes no driving or sensor scheduling. Do not launch, implement
or request a sensing-fan sweep or dedicated ToF test. If contact occurs, report
its phase and surface when known; do not invent an identification.

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

## Offline angle and target assessment

The unchanged batch command also produces `parking_exit_tof_assessment.md`,
`parking_exit_tof_observations.csv` and `tof-plots/`, both in the batch directory
and in each configuration/procedure report directory. Existing route, segment,
braking, reversal, correction and servo-neutral outputs remain unchanged.
The ToF module uses current `include/config.h` nominal geometry and mounting
offsets; this assumption must be checked before interpreting historical logs.

Each new sequence counts once per sensor/session. The centre ray and sampled
22/25-degree horizontal fans classify the known south wall and pink faces as
single-surface, mixed-surface, edge-sensitive or unidentified. Movable-rail gap
extremes are +/-5 mm; this is not a full measured geometry uncertainty model.
Continuous-angle plots separate build/configuration, observed procedure,
direction, sensor, phase and predicted-distance range. Residuals use only fresh
(<=50 ms), accepted, in-range observations that remain single-surface under
both fans. Other observations retain explicit exclusion reasons in the CSV.
Rear/local-frame pose-seeding data and readings crossing a field rebase or pose
correction are excluded from field residuals. Geometry uses logged pose without
acquisition-time interpolation, so sensor age still contributes motion error.

Raw and filtered residuals are separate. Raw/filtered differences and large
unexpected steps identify filter-lag/target-switch candidates, not proven causes.
The tuple lacks the full multi-object history, and periodic sampling misses many
sensor frames; invalid frequencies describe logged observations only. Sigma does
not cover mounting bias, target association, surface geometry or pose error.
Agreement with odometry is not absolute accuracy or guaranteed clearance. Keep
new ToF pose/servo corrections disabled until a separate plan is validated.

Coverage audit: periodic nominal/estimated route records cover rear positioning,
all five arcs and drive/brake/settle transitions, plus edge localization when
performed. Official CW finishes diagnostics before the short scan; official CCW
includes shortened edge localization and finishes before its scan. Subsequent
scan/connector text logs are not equivalent periodic route coverage. Completion
and pose seeding need not coincide with a sample. The logged nominal path uses
command curvature integrated over actual encoder travel; it is not an independent
time-based planned trajectory. No firmware changes extend coverage in this work.
