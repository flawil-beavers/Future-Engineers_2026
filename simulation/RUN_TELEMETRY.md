# Full-run telemetry v1

M7 records compact challenge timing from `[RUN_START]` until `[RUN_END]`.
Production defaults to `RUN_TELEMETRY_DETAILED=0`: start, lap boundaries and
finish only. There are no new periodic poses, route dumps, phase/motor events
or estimator reads in this default. Existing parking/connector/later tracking
diagnostics remain unchanged. The paragraphs below about pose/route/events
apply only to an explicitly enabled detailed diagnostic build (`=1`).
The writer uses only `get_position_struct()` (cached dead reckoning); it adds
no sensor acquisitions, movement, delays or changes to control decisions.
In detailed builds, regular `[RUN_POSE]` records occur every 250 ms, with extra cached samples on
phase/motor transitions and final stop. Records carry `t` (unsigned millis),
`elapsed_ms`, phase, lap, active route version, pose frame and `space`.
Local coordinates remain separate until the existing field rebase. Correction
events contain before/after poses and increment the frame; they are not travel.

`[RUN_EVENT]` records phase changes, motor drive/brake/stop/hold commands and
pose corrections/rebases. Motor events describe commands, not an independent
measurement of zero speed. Phase suffixes use the existing state enum values.

## Accepted plans

`[RUN_ROUTE]` starts an activated waypoint snapshot, followed by
`[RUN_ROUTE_POINT]` and `[RUN_ROUTE_END]`. Each revision has a monotonic ID,
kind, count and closed flag. `base=0` means a complete snapshot; otherwise
only changed points are emitted relative to the referenced revision. Geometry
is hashed in place; unchanged plans do not create another version. Accepted
connector replans, scan paths, discovery and learned lap paths are included.
Existing rejected-candidate diagnostics remain separate and are never treated
as an activated plan. The limit matches the planner's 192 waypoint capacity.

Scout, corner-view and final-parking primitive controllers do not expose a
waypoint array. Their zero-point control versions clear the previous overlay;
the existing segment/control diagnostics remain available. The visualization
does not invent a waypoint path for these feedback-controlled movements.

## Timing and outcomes

Elapsed time subtracts the start millis using unsigned arithmetic, including
all in-run holds, braking, settling and sensor pauses. `[RUN_LAP]` carries
elapsed time and duration since the previous lap boundary; the first includes
the parking exit and connector. `[RUN_END]` distinguishes completed, aborted,
failed and stopped outcomes. Successful timing ends after the existing final
stop/parking verification, before automatic USB saving. Finishes are idempotent.
Operator mode pause cancels the existing challenge state; it records stopped.
The existing resume action starts a new run rather than resuming that state.

Inspector shows a completion duration only with start and completed end records
in a file not labelled as an excerpt. Other imports show unknown completion
and the observed elapsed span; legacy logs use their timestamp span. The start
marker alone never proves success. Software completion does not prove physical
contact-free driving.

## Bounded output

The existing logger size is unchanged (192 KiB with parking diagnostics,
128 KiB without). Compact event output is capped at2KiB, plus the bounded truncation marker and
completion footer. A normal start+three-laps+finish host test stays below1KiB.
Detailed pose/route/event sub-budgets are64/24/8KiB, additionally sharing a hard
24KiB combined ceiling; these are not additive96KiB allowances. Samples
and route transactions stop before the last 8 KiB of free logger capacity.
A 768-byte tail reserve prevents ordinary diagnostics from consuming the
completion footer. A bounded `[RUN_TRUNCATED]` marker and the end record's
`truncated=1` report exhaustion. Existing ordinary diagnostics can still fill
their available space and trigger the logger's existing overflow warning.

Routes are preflighted as whole transactions. If a dump cannot fit, subsequent
samples still name the actual active version; Inspector does not substitute
an older plan for a missing version. Capacity exhaustion can shorten pose
coverage on long/noisy runs; 250 ms is a target cadence, not a hard real-time
guarantee when the existing main loop blocks. No missing segment is fabricated.

## Offline checks and normal-run validation

Run from the repository root:

```text
python simulation/check_run_telemetry.py
python simulation/check_connector_servo_resume.py
python simulation/check_later_laps.py
python simulation/check_final_parking_sequence.py
node tools/robot-run-inspector/test-inspector.cjs
node tools/robot-run-inspector/test-app.cjs
```

The writer regression compiles actual telemetry C++ against a bounded logger
and cached pose, testing cadence, route deltas, command events, holds, lap/end
timing, limits, transaction completeness, footer protection and clock rollover.
Inspector checks historical originals and synthetic new-schema edge cases.
Build only `giga_r1_m7` with the IDE-managed PlatformIO installation.

After the owner uploads this firmware, validate through normal CW/CCW runs:
check original logs for start/end outcomes, lap/end durations and truncation
warnings. Route bases/end markers and250ms samples apply only to detailed
builds. Detailed route bursts and physical timing still need measured validation;
do not enable them by default on the competition robot.
Compare software results with the physical report. Preserve each complete
original under `simulation/evidence/parking_exit_diagnostics/`, add metadata
and hashes, and append findings to `AGENT_DOCUMENTATION.md`. No dedicated
sensor sweep, extra movement or pause is required.
