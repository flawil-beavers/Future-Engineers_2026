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
- [ ] After explicit upload authorization, use the first normal run to verify log
      coverage, control-loop timing and buffer capacity. Then compare repeated
      exits in both directions.
- [ ] For each returned batch, copy every complete original log unchanged to
      `simulation/evidence/parking_exit_diagnostics/`, update that directory's
      metadata table and append concise hashes, physical reports, findings,
      limitations and exact next steps to `AGENT_DOCUMENTATION.md`. Commit these
      tracked sources; keep reproducible analyzer output in `local_workspace/`.

Do not implement backlash compensation or new pose control until repeated logs
show a material, consistent error. Any later correction requires its own plan,
offline safety validation and powered testing.

## Current evidence result (2026-09-30)

The 37 committed complete source files contain 38 diagnostic sessions; the
separate `000` excerpt duplicates complete log 399. Thirty-one sessions have an
untruncated `unparking_complete` record. Seven aborted or incomplete sessions
remain available per run but are excluded from the combined motion summaries.
Review the evidence README for physical setup and outcome before treating any
completed run as a valid performance trial.

Across 32 observable rear-marker moves in completed sessions, median
encoder-minus-ToF motion is +0.45 mm, spread 3.57 mm and maximum absolute
difference 13.27 mm. All fall within conservative sensor/settling uncertainty;
only one is a reverse move. Log 414 failed rear stationary verification after
the range rose while the encoder remained stationary. The interrupted log 406
has a reverse movement whose 7.39 mm disagreement exceeds its 5.84 mm
uncertainty, but its exit never completed. These observations warrant rear-ToF
investigation and do not identify drivetrain backlash on their own.

The exploratory completed-run curvature fit gives raw centre 80.97 degrees.
Increasing/decreasing steering approaches give 81.67/80.08 degrees, midpoint
80.87 degrees. The 1.59-degree span still prevents a precise centre claim;
retain `SERVO_CENTER=80` and treat 81 as a controlled physical comparison only.

Existing edge localization applies CW mean dx +22.5 mm (4.3 mm spread,
29 completed runs) and CCW mean dx -22.8 mm (0.8 mm spread, only two runs).
The 14 newly completed CW runs alone average +21.7 mm (4.7 mm spread). This
supports a repeatable sensor-versus-model offset, not external ground truth.
Keep the existing correction; obtain more CCW exits and investigate rear-ToF
settling before changing geometry, steering control or backlash compensation.
