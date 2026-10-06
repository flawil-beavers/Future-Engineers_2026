# Parking-exit pose and backlash implementation plan

## Scope

Implement automatic diagnostics for the complete existing unparking pipeline:
initial rear-ToF positioning, all parking-exit segments and their braking/direction
changes, then wall/marker localization. Whenever normal `O` unparking runs,
diagnostics start and stop with this pipeline. They add no movement, pause, sensor
acquisition, control change or save action. The complete challenge and USB saving
continue exactly as they do now.

The natural exit already contains braking and forward/reverse transitions. Its
encoder, gyro and cached ToF data will be used to estimate:

- deviation from the nominal exit pose and heading;
- travel and heading change during braking;
- effective motion lost around direction changes;
- repeatability across segments, runs and mirrored exits.

The encoder is upstream of gearbox/wheel play. Therefore encoder motion without
corresponding ToF-observed chassis motion may indicate effective drivetrain
backlash, but slip, compliance and sensor geometry must remain alternative causes.

## Firmware implementation

- [x] Add bounded parking-exit diagnostic records in `src/obstacle.cpp`.
- [x] Move diagnostic state/formatting into a focused parking-exit diagnostics
      module. Guard its implementation and call sites with one compile-time
      `PARKING_EXIT_DIAGNOSTICS_ENABLED` switch. Disabled production builds must
      compile out the records and retain the original buffer size; removing the
      module later should require deleting it and its small guarded call sites.
- [x] Make logging activate automatically on parking-exit start and finish at the
      end of localization, without requiring a command or test mode.
- [x] Add a versioned run header containing firmware/build identity, direction,
      steering centre, distance conversion and relevant exit constants.
- [x] At every exit-state transition record timestamp, state, segment, commanded
      direction/steering/speed, raw encoder count, converted distance, measured
      speed, gyro heading, estimated pose and nominal segment pose.
- [x] Mark motor-command removal, brake-hold start/end, settled endpoint and every
      direction change. Preserve encoder and heading baselines across each event.
- [x] Add bounded periodic samples during movement and braking using existing
      cached values. Do not trigger sensor reads or block the control loop.
- [x] Record each new cached ToF sequence once with sensor, acquisition age,
      raw/filtered range, validity, signal and sigma. Record the expected wall or
      marker only where current state and geometry identify it unambiguously.
- [x] Log existing position corrections before and after application, including
      source, residual and accepted components. Do not add new corrections.
- [x] Cover rear positioning and localization with the same event model. Use an
      observation for error analysis only when freshness, validity and expected
      feature geometry make the reference reliable; otherwise retain it as an
      explicitly rejected diagnostic sample.
- [x] Add diagnostic-complete, abort and truncation records, and recognize the
      logger's existing overflow marker. State transitions take priority over
      periodic samples.
- [x] Consolidate periodic samples and enforce a 64 KiB maximum diagnostic
      contribution: at most 150 records within a 57,000-byte sample sub-budget,
      plus at most 8 KiB of headers/events, counting CRLF. The formatting buffer
      is 512 bytes; incomplete records are rejected explicitly. State changes
      cannot bypass the sample count, and events reserve truncation/finish space.
- [x] When diagnostics are enabled, increase the RAM log buffer from 128 KiB to
      **192 KiB**; keep 128 KiB when disabled. Historical logs have a 74,048-byte
      maximum among the reviewed complete parking runs and a 107,628-byte maximum
      among all non-overflow logs. Adding the 64 KiB diagnostic budget requires
      about 170 KiB, so 192 KiB provides roughly 23 KiB margin in the worst
      observed non-overflow case. Existing 128 KiB logs have overflowed twice.
      Do not use 256 KiB: it would leave only about 25.9 KiB RAM. Verified M7
      builds after the 2026-09-26 budget correction use 432,448 of 523,624
      bytes (82.6%) with diagnostics and 366,816 bytes (70.1%) with them
      disabled. The enabled build leaves 91,176 bytes.
- [ ] Verify by code inspection and timing telemetry that diagnostics do not change
      exit state timing, motor/steering commands, sensor scheduling or normal saving.
      Cached-only source inspection is complete (2026-09-26); physical timing
      telemetry and returned-log coverage remain required.

## Offline analysis

- [x] Add `simulation/analyze_parking_exit_pose.py` and validate the log schema,
      firmware identity, completeness, ordering and duplicated sensor sequences.
- [x] Reconstruct rear positioning, each nominal exit segment and localization;
      plot nominal versus encoder/gyro pose through drive, brake and settle.
- [x] Report per segment: driven distance, braking distance, heading change,
      settled pose error and repeatability.
- [x] At each direction change, compare encoder displacement with change in a
      reliably identified ToF reference. Estimate effective lost motion and its
      uncertainty only where the reference geometry is valid; otherwise report it
      as unobservable.
- [x] Compare forward/reverse and CW/CCW results. Separate systematic bias from
      spread and do not label every discrepancy as backlash.
- [x] Estimate the servo neutral from naturally occurring near-straight intervals.
      Fit measured gyro curvature (heading change per signed encoder distance)
      against logical steering command during rear positioning and localization;
      exclude stopped/braking samples, unhealthy gyro data, low motion and fixed
      turning arcs, and exclude the settling interval after steering changes. The
      zero-curvature intercept gives a candidate offset from the configured
      `SERVO_CENTER`.
- [x] Report neutral estimates separately by forward/reverse travel, CW/CCW exit
      and steering approach direction. If the groups disagree beyond uncertainty,
      report hysteresis/range instead of one optimum. Otherwise report the robust
      combined centre, confidence/spread and sample coverage. Never change
      `SERVO_CENTER` automatically from a log.
- [x] Write Markdown and CSV reports below
      `local_workspace/parking-exit-analysis/`, including input names, SHA-256
      hashes, firmware/schema identity and excluded evidence.
- [x] Regenerate the archived batch automatically, with separate reports by
      diagnostic build/configuration so revised exits can be compared separately.
- [x] Add a tracked synthetic fixture and tests for completion, braking,
      reversal, duplicate ToF data, missing configuration, abort/overflow and
      diagnostic truncation flags, and a known servo-neutral offset.

## Documentation and validation

- [x] Add `simulation/PARKING_EXIT_DIAGNOSTIC_LOGGING.md` describing schema, units,
      calculations, valid physical references and limitations; link it from
      `simulation/README.md`.
- [x] Document that the friend uses the unchanged normal `O` procedure and normal
      save process. Only firmware identity, log number and physical outcome need
      to accompany each run.
- [x] Run analyzer tests and build only `giga_r1_m7` with the IDE-managed
      PlatformIO Core in enabled and disabled configurations. Confirm disabled
      behavior/buffer size and enabled RAM use rather than relying on estimates.
- [x] Verify returned-log coverage and diagnostic budgets from normal runs.
      The archived revised exit has 28 completed, untruncated CW sessions.
      Control-loop timing comparison and revised-exit CCW runs remain pending.
- [x] Establish the original-log archive and metadata/handoff procedure. Repeat
      it for every returned batch; keep generated reports in `local_workspace/`.

Do not implement backlash compensation or new pose control until repeated logs
show a material, consistent error. Any later correction requires its own plan,
offline safety validation and powered testing.

## Current evidence and next steps (2026-10-06)

### Laptop ToF assessment and unchanged physical runs

The batch command now adds continuous-angle ToF quality/residual plots and a
unique-observation CSV alongside unchanged route/braking/reversal reports. It
models nominal wall/rail intersections and 22/25-degree sensing fans offline;
no dedicated ToF program, sweep, extra movement, pause or save step is required.
The tester uploads current firmware and runs/saves normally, reporting firmware,
log number, direction and physical outcome/contact phase/surface when known.

Coverage inspection confirms all rear positioning and five-arc diagnostics.
Official CW now ends before the short scan; CCW includes shortened edge
localization and ends before its scan. Subsequent scan/connector text logs are
not periodic nominal/estimated route coverage. Preserve this limitation instead
of claiming a complete post-exit trajectory. Nominal diagnostic pose follows
command curvature using measured encoder travel, not independent timed targets.
New logs remain archived unchanged with hashes and physical reports. Analyse
current procedures separately; retain gyro/encoder integration and defer active
ToF correction because residual agreement does not establish absolute accuracy.

The archive contains 72 complete sources / 73 sessions, with 62 completed,
untruncated exits. Compare revisions through `parking_exit_batch.md` under
`local_workspace/parking-exit-analysis-all/`; physical setup/outcome and exact
source hashes remain in the evidence README.

The archived 90/155 mm exit targets replace 85/150 mm; the last arc still ends by
gyro alignment. The `Oct__5_2026_21_30_53` diagnostic group contains 30 CW
sessions (425-454), 28 completed. Completed final arcs span 139.2-151.4 mm.
Logs 446-448 also physically completed the GREEN-middle connector/lap without
reported contact or a visibly narrow pink/green gap. This covers those layouts,
not all placements or CCW. The diagnostic build timestamp may survive later
firmware changes; cross-check the recorded segment targets and evidence metadata.

In those 28 completed exits, 30 rear-marker moves have encoder-minus-ToF median
+0.47 mm, spread 2.92 mm, maximum absolute 6.16 mm, all within conservative
uncertainty. Only one reverses, so motor backlash remains unidentified. The servo
fit gives centre 81.11 degrees, approach candidates 81.53/80.16 (midpoint
80.85); retain `SERVO_CENTER=80` because the 1.37-degree span remains unresolved.
Edge correction mean dx/dy is +22.6/-5.6 mm (dx spread 5.6), comparing onboard
references rather than external pose truth.

Two revised runs failed before completion: 429 failed rear stationary
verification; 441 aligned after only 134.1 mm and stalled about 68 mm into
reverse localization, with user-reported wall contact/hanging. Review 441's
estimated body/wheel clearance against pink rails and wall, using the actual
gyro-ended pose rather than the ideal 155 mm endpoint. Then obtain cable-free
CW/CCW observations identifying the contact phase/surface. Keep the existing
motion and safety gates while investigating; defer centre/backlash compensation.

The subsequent geometry audit gives sampled reverse pink gaps 11.8-38.8 mm in
completed runs and 6.3 mm in 441; its south-wall gap stays above 200 mm. Selected
unobstructed reverse wall readings agree laterally with odometry to a few mm,
but do not establish full XY accuracy or a guaranteed clearance. Effective arc
radii also differ by steering side/travel direction. Next software investigation:
shadow-check pose and full-body clearance at existing stops, then evaluate bounded
remaining-path changes with sensor/geometry uncertainty. Active servo/path control
needs a separate reviewed implementation and powered CW/CCW validation.

Incoming `ff9be2e` changes the pipeline after the same five arcs: official CW
uses the initial ToF seed and exit odometry directly for its short scan, omitting
second-edge reverse/correction. Official CCW keeps the edge correction but omits
the extra 70 mm continuation. O3/CHECK_ALL keeps its legacy procedure. No complete
short-start physical logs are archived yet; 455/456 remain excerpts of preceding
connector trials. Batch reports now separate explicit short-start markers even
with unchanged diagnostic headers. Validate CW exit-pose/scan clearance without
the former edge correction, and the shortened CCW reference, before expanding
live pose control. The incoming connector servo-resume fix is a software cause
of the later steering failures; it does not measure parking linkage backlash.
