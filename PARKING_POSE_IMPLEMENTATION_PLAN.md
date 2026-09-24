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
- [ ] Move diagnostic state/formatting into a focused parking-exit diagnostics
      module. Guard its implementation and call sites with one compile-time
      `PARKING_EXIT_DIAGNOSTICS_ENABLED` switch. Disabled production builds must
      compile out the records and retain the original buffer size; removing the
      module later should require deleting it and its small guarded call sites.
- [ ] Make logging activate automatically on parking-exit start and finish at the
      end of localization, without requiring a command or test mode.
- [ ] Add a versioned run header containing firmware/build identity, direction,
      steering centre, distance conversion and relevant exit constants.
- [ ] At every exit-state transition record timestamp, state, segment, commanded
      direction/steering/speed, raw encoder count, converted distance, measured
      speed, gyro heading, estimated pose and nominal segment pose.
- [ ] Mark motor-command removal, brake-hold start/end, settled endpoint and every
      direction change. Preserve encoder and heading baselines across each event.
- [ ] Add bounded periodic samples during movement and braking using existing
      cached values. Do not trigger sensor reads or block the control loop.
- [ ] Record each new cached ToF sequence once with sensor, acquisition age,
      raw/filtered range, validity, signal and sigma. Record the expected wall or
      marker only where current state and geometry identify it unambiguously.
- [ ] Log existing position corrections before and after application, including
      source, residual and accepted components. Do not add new corrections.
- [ ] Cover rear positioning and localization with the same event model. Use an
      observation for error analysis only when freshness, validity and expected
      feature geometry make the reference reliable; otherwise retain it as an
      explicitly rejected diagnostic sample.
- [ ] Add diagnostic-complete, abort, truncation and logger-overflow records.
      State transitions take priority over periodic samples.
- [ ] Consolidate each periodic sample into a compact bounded record and enforce
      a 64 KiB maximum diagnostic contribution: at most 150 periodic records of
      at most 380 bytes (57,000 bytes), plus at most 8 KiB of headers/events.
- [ ] When diagnostics are enabled, increase the RAM log buffer from 128 KiB to
      **192 KiB**; keep 128 KiB when disabled. Historical logs have a 74,048-byte
      maximum among the reviewed complete parking runs and a 107,628-byte maximum
      among all non-overflow logs. Adding the 64 KiB diagnostic budget requires
      about 170 KiB, so 192 KiB provides roughly 23 KiB margin in the worst
      observed non-overflow case. Existing 128 KiB logs have overflowed twice.
      Do not use 256 KiB: from the last 366,624-byte M7 build it would leave only
      about 25,928 bytes RAM. The proposed 192 KiB build is estimated at 432,160
      of 523,624 bytes (82.5%), leaving about 91,464 bytes; confirm by building.
- [ ] Verify by code inspection and timing telemetry that diagnostics do not change
      exit state timing, motor/steering commands, sensor scheduling or normal saving.

## Offline analysis

- [ ] Add `simulation/analyze_parking_exit_pose.py` and validate the log schema,
      firmware identity, completeness, ordering and duplicated sensor sequences.
- [ ] Reconstruct rear positioning, each nominal exit segment and localization;
      plot nominal versus encoder/gyro pose through drive, brake and settle.
- [ ] Report per segment: driven distance, braking distance, heading change,
      settled pose error and repeatability.
- [ ] At each direction change, compare encoder displacement with change in a
      reliably identified ToF reference. Estimate effective lost motion and its
      uncertainty only where the reference geometry is valid; otherwise report it
      as unobservable.
- [ ] Compare forward/reverse and CW/CCW results. Separate systematic bias from
      spread and do not label every discrepancy as backlash.
- [ ] Estimate the servo neutral from naturally occurring near-straight intervals.
      Fit measured gyro curvature (heading change per signed encoder distance)
      against logical steering command during rear positioning and localization;
      exclude stopped/braking samples, unhealthy gyro data, low motion and fixed
      turning arcs, and exclude the settling interval after steering changes. The
      zero-curvature intercept gives a candidate offset from the configured
      `SERVO_CENTER`.
- [ ] Report neutral estimates separately by forward/reverse travel, CW/CCW exit
      and steering approach direction. If the groups disagree beyond uncertainty,
      report hysteresis/range instead of one optimum. Otherwise report the robust
      combined centre, confidence/spread and sample coverage. Never change
      `SERVO_CENTER` automatically from a log.
- [ ] Write Markdown and CSV reports below
      `local_workspace/parking-exit-analysis/`, including input names, SHA-256
      hashes, firmware/schema identity and excluded evidence.
- [ ] Add tracked fixtures and tests for normal completion, braking, reversal,
      invalid/duplicate ToF data, missing records, abort, overflow, diagnostic
      byte-budget enforcement and known synthetic servo-neutral offsets.

## Documentation and validation

- [ ] Add `simulation/PARKING_EXIT_DIAGNOSTIC_LOGGING.md` describing schema, units,
      calculations, valid physical references and limitations; link it from
      `simulation/README.md`.
- [ ] Document that the friend uses the unchanged normal `O` procedure and normal
      save process. Only firmware identity, log number and physical outcome need
      to accompany each run.
- [ ] Run analyzer tests and build only `giga_r1_m7` with the IDE-managed
      PlatformIO Core in enabled and disabled configurations. Confirm disabled
      behavior/buffer size and enabled RAM use rather than relying on estimates.
- [ ] After explicit upload authorization, use the first normal run to verify log
      coverage, control-loop timing and buffer capacity. Then compare repeated
      exits in both directions.
- [ ] Record each returned batch concisely in `AGENT_DOCUMENTATION.md`, with hashes,
      curated evidence, physical report, findings, limitations and exact next step.

Do not implement backlash compensation or new pose control until repeated logs
show a material, consistent error. Any later correction requires its own plan,
offline safety validation and powered testing.
