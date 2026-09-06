# Future Engineers parking-entry handoff

Continue development in:

`C:\Users\philk\Documents\GitHub\Future-Engineers_2026`

Read first, in order:

1. `AGENTS.md`
2. the newest 2026-08-30 section at the top of `AGENT_DOCUMENTATION.md`
3. the top ordered connector section in `OBSTACLE_CHALLENGE_TEST_PLAN.md`
4. `OBSTACLE_SEAT_NUMBERING.md`
5. `OBSTACLE_CLEARANCE_LOGGING.md`

Preserve all staged/unstaged changes. Do not reset, checkout, clean, or upload
firmware without explicit user permission. Document every durable finding in
`AGENT_DOCUMENTATION.md` and keep the test plan ordered. Build only M7 with the
IDE-managed PlatformIO executable.

Current branch is `pure-pursuit`. The current documentation and repository
organization changes are:

- `AGENT_DOCUMENTATION.md`
- `OBSTACLE_CHALLENGE_TEST_PLAN.md`
- `PARKING_ENTRY_HANDOFF.md`
- `CAD/README.md` plus removal of the old drive-simulation copies from `CAD/`
- `simulation/`, containing the relocated tools, generated SVG, tracked log
  fixtures, parking-exit photographs, and documentation
- `WRO_2026_RULES.md`

Newest reviewed logs are tracked in
`simulation/fixtures/parking_entry_scout/log_362.txt` through `log_369.txt`.
Logs 364/365/369 repeatedly stopped because the 55 mm CCW scout
could not resolve preceding station 1. Log 362 timed out the initial CW primary
station with no retry. Log 363 exposed a premature scout CLEAR followed by a
green confirmation during return; its old connector incorrectly logged no
hidden guard. Logs 367/368 completed three CW laps in telemetry, but the user
did not supply physical-contact reports, so they are not physical acceptance.

Implemented fixes:

- scout arc is 85 mm at 60 mm/s;
- the robot remains stopped at the scout pose for at least 400 ms;
- an initial primary UNKNOWN performs the scout/retrace, then one stationary
  primary retry if still unresolved;
- a second primary timeout or unresolved scout locks the drive off;
- connector arming requires both the primary and preceding stations resolved;
- confirmed hidden-seat guard priority over an earlier CLEAR is retained;
- retry state resets between runs.

Offline replay is in `simulation/parking_entry_scout_sim.py`. It passes
all eight log-362--369 scan poses at 85 mm: intended preceding-seat bearing
6.5--20.9 degrees, range 244--309 mm, minimum modeled wall clearance 139.5 mm,
and minimum modeled legal-pillar clearance 166.1 mm. This is ideal fixed-field
geometry only and cannot prove physical safety or camera reliability.

Verification already completed:

- IDE-managed `giga_r1_m7` build: SUCCESS
- RAM: 366688 bytes
- Flash: 437568 bytes
- `git diff --check`: clean except LF-to-CRLF warnings
- M4 was not built
- no firmware was uploaded

Do not change geometry without new evidence. The next action requires explicit
upload permission. Then validate CW/red first with the opposite green station-0
pillar present. Require 85 mm scout outbound/return, >=400 ms stopped scout
observation, green confirmation/injection, small return error, both connector
prerequisites resolved, confirmed guard, connector PASS/completion, normal
green then red avoidance, and the user's physical no-contact/no-stall report.
Only after that passes, run CCW/green with the opposite red pillar and require
station 1 to resolve at the 85 mm scout pose. Never accept simulation or
telemetry as proof of no contact.

Tool usage, inputs, and limitations are documented in
`simulation/PARKING_ENTRY_GEOMETRY_TOOLS.md`.
