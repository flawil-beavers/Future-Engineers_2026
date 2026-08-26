# Obstacle Challenge: tests to run next

This file is the ordered go/no-go checklist for remaining robot tests. Durable
engineering history, calibration, completed-test evidence, and tuning rationale
are in `AGENT_DOCUMENTATION.md`.

Do the tests in order. A failed test blocks later tests. Diagnose its evidence
before changing code, and change only one cause at a time. The user performs
powered runs and supplies the USB log; do not upload firmware or alter tuning
without explicit consent.

## Safety before every powered test

- Secure and charge the battery.
- Keep the emergency disable switch reachable.
- Keep USB cables clear of the wheels and steering.
- Clear people and fragile objects from the track.
- Start at the validated 175 mm/s cap after a material control change.
- Record the firmware commit, direction, layout, physical result, and serial
  log. Physical contact is a failure even if firmware reports `PASS`.

Arm with the physical enable switch LOW. Use `Y1` for left/CCW, `Y-1` for
right/CW, and `Y0` to abort and brake. Toggle the enable switch HIGH only when
placement and safety checks are complete.

## 1. Validate the eight-waypoint taper: green-left CW

Status: passed provisionally at 175 mm/s. `log_57` formally completed, and two
runs repeated about 10 mm pillar clearance with more than 50 mm wall clearance.

Use the successful CW starting position: rear-axle midpoint on the
starting-straight centreline, robot parallel to the walls and facing the first
corner about 500 mm ahead. Place a green pillar 500 mm forward and 100 mm left
of the rear-axle midpoint. Arm with `Y-1` and run one lap.

Pass only if all of the following hold:

- A practical, repeatable visible gap remains between the complete robot and
  both the pillar movement circle and walls; the prior approximately 1 mm gap
  is not sufficient.
- Wheel motion remains continuous with no contact, stall, or intervention.
- Green S0 station-2 left seat 5 is confirmed and injected exactly once.
- Green is passed on the left, every other station clears, and one lap
  completes formally.
- Steering enters and leaves the wider detour smoothly.

If it fails, preserve the log and physical observation. Do not move the peak
waypoint or change clearance, smoothing, lookahead, or perception until the
failure has been diagnosed.

`log_56` confirmed green seat 5 and injected once. Maximum CTE was 126.9 mm and
maximum heading error 42.1 degrees. At S2 station 0, the unresolved right seat
was 28.1 degrees/261 mm and then 25.5 degrees/224 mm: it crossed the angle limit
only after crossing below the validated 230 mm range. `log_55` cleared the same
station, and the wider taper is no longer active there, so do not tune this
single miss. Repeat once and record the pillar and wall gaps again. Do not move
the peak waypoint earlier; it should remain aligned with the pillar.

`log_57` passed the unchanged repeat. Green seat 5 was confirmed and injected
once, every station including S2 station 0 cleared, and one right/CW lap
completed formally. Wheel motion remained continuous. Maximum CTE was 126.7 mm
and maximum heading error 41.2 degrees. The user again observed about 10 mm
pillar clearance and more than 50 mm clearance to the left wall. Accept the
eight-waypoint/260 mm geometry provisionally at the current 175 mm/s test speed;
later placement and multi-run regression must preserve this margin.

## 2. Regress red-right CW

Status: passed provisionally at 175 mm/s in `log_58`.

Keep the same CW start, place a red pillar
500 mm forward and 100 mm right of the rear-axle midpoint, and arm with `Y-1`.
This regression is required because the wider taper affects both pass sides.

Require no pillar/wall contact, no stall or intervention, correct red S0
station-2 right seat 4 confirmation, exactly one injection, red passed on the
right, all remaining stations clear, smooth detour entry/exit, and one formal
lap completion.

`log_58` met these requirements. Red seat 4 was confirmed and injected once,
all stations cleared, wheel motion remained continuous, and one right/CW lap
completed formally. Maximum CTE was 122.1 mm and maximum heading error was
44.9 degrees. The user observed about 10 mm pillar clearance and more than
50 mm clearance to the right wall. Together with the green-left result, this
provisionally accepts the 260 mm clearance/eight-waypoint taper for both pass
sides at 175 mm/s.

## 3. Expand one-lap placement coverage

Run only after both representative CW layouts pass.

- [x] Test a pillar at station 0. Red-right CW passed in `log_59` with ample
      visible clearance on both sides.
- [x] Test a pillar at station 2. The green-left and red-right CW layouts
      passed both lateral sides in `log_57` and `log_58`.
- [x] Test two adjacent pillars with opposing colours. `log_60` confirmed both.
- [x] Confirm each pillar selects the correct seat and is injected once.
- [x] Confirm the first overlapping eight-waypoint layout remains smooth and
      preserves pillar and wall clearance. Mirror and worst-case layouts remain.
- [x] Mirror the moderate adjacent layout in CCW (`log_61`). The larger
      alternating layout still fails at 500 mm spacing in both directions.

`log_59` confirmed red S0 station-0 right seat 0 and injected it exactly once.
Every other station cleared, wheel motion remained continuous, and the lap
passed formally. Maximum CTE was 112.3 mm and maximum heading error was 35.3
degrees. The user observed ample clearance on both sides. Keep this extra
margin; do not tighten an isolated route before broader placement and speed
testing.

For the adjacent-pillar test, retain the validated CW start. Use section 1,
the straight immediately after the first corner:

- Put a red pillar at station 0 on the left side of the CW driving direction
  (seat 7).
- Put a green pillar at station 1, 500 mm farther along that straight, on the
  right side of the CW driving direction (seat 8).

These seat choices keep the initial alternating avoidance peaks nearer the
corridor centre while still making their eight-waypoint tapers overlap. Arm
with `Y-1` and run one lap. Require two correct confirmations and exactly two
injections, red passed on the right, green passed on the left, no contact,
stall, or intervention, every other station clear, continuous motion through
the transition, and a formal lap completion. Record the closest pillar and
wall gaps for each pass. Do not upload or change firmware for this test.

`log_60` confirmed red seat 7 and green seat 8, with exactly one injection for
each. The lap completed without a stall or intervention, maximum CTE was 159.8
mm (below the configured 180 mm pass threshold), and maximum heading error was
26.5 degrees. The displayed `FAIL` was a test-harness defect: its result
predicate then required `obstacle_path_injection_count() == 1`, so a correct
two-pillar run could not report `PASS`. The user observed ample clearance on both
sides. During the physical pass, the minimum logged filtered ToF ranges were
about 151 mm from the left sensor to the red pillar surface and 80 mm from the
right sensor to the green pillar surface. These sparse telemetry samples are
sensor-to-surface ranges, not guaranteed whole-robot minimum clearances.

### Engineering TODOs from `log_60`

- [x] Track every fresh ToF sample from 300 mm before through 300 mm after each
      seat and, for confirmed pillars, log raw and filtered minima from the
      facing sensor. Per-seat accumulators preserve the 100 mm overlap between
      adjacent 500 mm station windows. Powered validation passed in `log_71`
      and `log_73` with five independent pillar reports per lap.
- [x] Derive the conservative lateral inset from each ToF aperture to the
      maximum-steered outer-wheel envelope and report estimated clearance.
      The calculation and limitations are documented below.
- [x] Remove the generic live-test assumption that a passing run must contain
      exactly one injection. Keep expected seat IDs and counts in the
      layout-specific acceptance check.
- [ ] Correct the 500 mm seat-6 red to seat-9 green transition. Both directions
      reached the green pillar with a prolonged near-zero-speed interval even
      though detection and injection occurred before the avoidance taper.
- [x] Base adaptive Pure Pursuit lookahead on the speed after live-test/runtime
      caps rather than the uncapped nominal path speed. Multiple powered runs
      retained more than 30 mm visually observed pillar clearance, but the
      exact seat-6 -> seat-9 regression remains outstanding.

### Remaining one-lap tests, in order

1. [passed: `log_61`] Mirror the successful moderate adjacent layout in CCW. Start in the
   validated CCW orientation and use the straight immediately after the first
   corner. Put red at station 0 on the left of the CCW driving direction (seat
   7), and green at station 1 on the right (seat 8). Arm with `Y1`.
2. [failed: `log_62`] Test the larger alternating displacement in CW. On section 1 put red at
   station 0 on the right (seat 6), and green at station 1 on the left (seat
   9). Arm with `Y-1`. These positions request a transition from the right-side
   pass peak to the left-side pass peak, so keep the disable switch reachable.
3. [failed: `log_64`] Mirror that seat-6/seat-9 layout in CCW and arm with
   `Y1`. This was run despite test 2 failing and reproduced the station-1
   near-zero-speed problem.

For every run require correct colours and seats, exactly two injections, no
contact, stall, or intervention, continuous steering through the overlap, all
other stations clear, and one completed lap. The generic result predicate no
longer checks injection count; always judge expected seat IDs and counts as
layout-specific acceptance evidence and preserve the log.

Do not run another powered layout yet. `log_62` remained at essentially zero
speed for about 1.8 seconds and then aborted at the unresolved next station;
maximum CTE was 191.9 mm and heading error 50.1 degrees. `log_64` remained near
zero for about 1.2 seconds beside the green pillar before recovering; maximum
CTE was 131.5 mm and heading error 64.3 degrees. In both runs green seat 9 was
confirmed about 350-430 mm before its station, so late perception is not the
primary cause. The additive path asks for a 720 mm lateral reversal between
pillar centres only 500 mm apart, and the robot does not follow it safely at
175 mm/s.

The user confirmed that the robot physically struck the green pillar in both
`log_62` and `log_64` and required manual assistance to continue. Therefore
both runs are unambiguous physical failures; their recovery or later telemetry
must not be counted as successful controller behaviour.

Moving green from station 1 to station 2 doubled the centre spacing to 1000 mm.
Those variants completed continuously in both CW (`log_63`, seats 6 and 11,
CTE 126.4 mm) and CCW (`log_65`, seats 6 and 11, CTE 107.6 mm). Their printed
`FAIL` results are only the known two-injection reporting defect.

Sparse filtered ToF minima near the intended pillars were:

| Log | Layout | Red-facing left ToF | Green-facing right ToF | Motion |
| --- | --- | ---: | ---: | --- |
| `log_61` | seats 7 -> 8, 500 mm | 115 mm | 89 mm | continuous |
| `log_62` | seats 6 -> 9, 500 mm | 66 mm | no reliable side hit | stalled/aborted |
| `log_63` | seats 6 -> 11, 1000 mm | 34 mm | 155 mm | continuous |
| `log_64` | seats 6 -> 9, 500 mm | 92 mm | 32 mm | stalled/recovered |
| `log_65` | seats 6 -> 11, 1000 mm | 76 mm | 81 mm | continuous |

These historical values remain sparse sensor-to-surface evidence; the new
fresh-sample logger cannot retroactively improve them. The user could not
recall whether the 34 mm red reading in `log_63` corresponded to a visibly
close pass, so do not use that run to establish a red clearance margin.

### ToF passage-window and clearance calculation

The rear-axle midpoint is the robot origin, with +X forward and +Y left. The
wheel diameter is 43.2 mm, so the radius is 21.6 mm. With the physical 100 mm
axle spacing, the rear-wheel longitudinal extent is approximately
`-21.6..+21.6 mm` and the front-wheel extent is `78.4..121.6 mm`. These
longitudinal values describe the robot geometry but are not subtracted from a
side-facing range.

The measured straight outside-wheel width is 125 mm, giving a straight lateral
envelope of +/-62.5 mm. Steering adds approximately 7.5 mm on each side, so the
conservative maximum-steered envelope is +/-70 mm. The ToF aperture origins
are configured at `(40,+35) mm` left and `(40,-35) mm` right. Therefore both
aperture-to-outer-wheel insets are 35 mm:

`125 / 2 + 7.5 - 35 = 35 mm`.

The reported estimate is consequently `facing ToF range - 35 mm`. It is a
conservative lateral, geometry-based outer-wheel estimate, not an exact swept
body clearance: the ToF beam at X=40 mm and the widest steered wheel point do
not necessarily share a longitudinal coordinate, the beam can miss the exact
closest instant, and the pillar surface/beam geometry matters.

Each seat accumulates samples from path offset -300 through +300 mm. At the
175 mm/s validation cap, this 600 mm window lasts about 3.4 seconds and spans
roughly 100 nominal 30 ms ranging frames. Both the quality-accepted selected
raw range and the production filtered range are logged. The filtered value is
comparable with existing telemetry but can lag because it is limited to a
100 mm change per frame; the raw value is more immediate but more sensitive to
an isolated low measurement. Treat agreement between both minima, sample
counts, physical observation, and motion telemetry as the strongest evidence.

Code analysis identifies the next single-variable controller correction. The
Pure Pursuit lookahead is currently derived from `progress.speedMmS` before the
175 mm/s live-test/runtime cap is applied. A nominal 260 mm/s straight path can
therefore select the maximum 330 mm lookahead while the robot is actually
commanded at 175 mm/s. Use the same capped speed for both lookahead selection
and the motor command. At 175 mm/s this gives about 208 mm lookahead. Do not add
a special steering controller or change the obstacle geometry at the same
time. After rebuilding, first regress one previously safe outer single-pillar
layout, then repeat seat 6 red -> seat 9 green in CW. Do not upload without
explicit consent.

This correction is now implemented. `cappedPathSpeed()` is evaluated before
target selection and feeds both `adaptiveLookahead()` and the normal speed
command. Deterministic preflight checks cover the minimum, 175 mm/s test, and
maximum lookahead points. The IDE-managed `giga_r1_m7` build passed using
291176 bytes RAM and 352192 bytes flash. No firmware was uploaded. The change
does not yet close the adjacent-transition TODO; physical regression must do
that.

### Post-lookahead powered evidence: `log_66` through `log_73`

The user ran at least eight starts and reported more than 30 mm physical pillar
clearance in the detected cases. `log_71` and `log_73` each completed a formal
five-pillar CCW lap with seats 5, 9, 13, 16, and 23 confirmed exactly once.
Maximum CTE was 120.0/119.5 mm. The new ToF logger reported estimated
range-minus-wheel-envelope clearances as follows:

| Log | Seat 5 | Seat 9 | Seat 13 | Seat 16 | Seat 23 |
| --- | ---: | ---: | ---: | ---: | ---: |
| `log_71` | 144 mm | 188 mm | 90 mm | 189 mm | 119 mm |
| `log_73` | 142 mm | 193 mm | 99 mm | 182 mm | 131 mm |

The user-observed pauses were not controller stalls in the logs. No run after
startup contains a telemetry window with measured speed remaining near zero.
Instead, `log_66`, `log_67`, `log_69`, and `log_70` intentionally aborted and
braked because empty S2 station 0 remained unresolved. `log_68` and `log_72`
did the same at S1 station 0. The two complete five-pillar runs did not stop.
This makes empty-station perception the next reliability problem, separate
from Pure Pursuit tracking.

None of these logs contains both red seat 6 and green seat 9, so they do not
yet prove that the formerly colliding 500 mm pair is fixed. The user reported
one green detection miss, but the logs alone do not identify which physical
layout was present in the zero-injection `log_68` and `log_72` attempts. Record
that mapping before changing perception thresholds.

The user clarified that the missed pillar was green at S1 station-0 right seat
6 in a CW run, and that the exact red-seat-6/green-seat-9 adjacent regression
was not performed. The run number is not known, but both zero-injection logs
show the same marginal geometry: within the validated 230-600 mm discovery
range, seat 6 came no closer than -29.8 degrees in `log_68` and -27.8 degrees
in `log_72`, while the accepted half-angle is about 27.4 degrees. Do not widen
the camera acceptance or discovery nudge from one physical miss. Repeat the
single green seat-6 placement twice unchanged. Both runs must confirm
`seat=6 color=GREEN`, pass it on the left without contact, and continue past
S1 station 0. If either repeat misses, stop and tune the viewing geometry
before attempting the adjacent collision regression.

The later continuous-DCMI safe single-pillar regression passed physically in
`log_76`: the user observed continuous motion, no contact, and ample clearance
at 175 mm/s. Its USB log overflowed because the robot remained in stationary
camera mode for 4645 frames before driving, so the file ends before the seat
injection and lap result. This physical gate is sufficient to proceed; do not
repeat it merely for formal telemetry.

The next test is the exact formerly colliding adjacent pair in CW. Use section
1, the straight immediately after the first corner: red at station 0 on the
right (seat 6), then green at station 1 on the left (seat 9), 500 mm farther
along. Arm with `Y-1`. Power-cycle immediately before the run and start promptly
so stationary camera telemetry does not fill the 128 KiB logger. Require both
correct confirmations, exactly two injections, red passed on the right, green
passed on the left, continuous motion through the reversal, no contact or
intervention, and adequate pillar/wall gaps. Preserve the log even if a later
empty station causes the already known perception abort.

The isolated green-seat-6 repeat remains a later edge-view perception
regression. It is no longer a prerequisite for this controller test because
the adjacent layout uses the already demonstrated red at seat 6 and green at
seat 9.

`log_77` invalidated that assumption. The red seat-6 pillar was also missed:
there were zero injections, seat 6 never entered `vis`, and the robot followed
the baseline path into the red pillar. The best usable seat-6 geometry was
about -29.9 degrees/289 mm, outside the accepted approximately +/-27.4-degree
window. The farther green pillar produced only one seat-9 vote.

Do not run another powered seat-6 test yet. The unresolved hold configured at
170 mm latched at only 135 mm because progress is sampled on the 50 mm path;
that is inside the combined 130 mm robot-front offset and 42.5 mm pillar
movement-circle radius. The hold has now been raised to a safe 340 mm rear-axle
forward distance and the firmware builds successfully. The isolated-seat target
gain has also been raised from 1.0 to 1.35, while retaining the 40-degree cap
and the validated camera acceptance window. This isolated validation passed in
`log_78`: red seat 6 entered view at about -26.4 degrees/307 mm, confirmed and
injected exactly once, the hold did not fire, and one lap completed.

Next, restore the exact adjacent CW regression: retain red at S1 station 0
right/seat 6 and add green at S1 station 1 left/seat 9. Arm with `Y-1`. Require
both correct confirmations, exactly two injections, continuous motion through
the right-to-left transition, no contact/intervention, and one completed lap.
If perception remains unresolved, the robot must stop safely at the 340 mm
hold; do not immediately repeat a failed run. Any upload still requires
explicit user approval.

`log_79` failed this regression physically. Both pillars were correctly
confirmed and injected, with green seat 9 confirmed at about 410 mm range, but
heading error reached 61.1 degrees and the robot stalled against the green
pillar. The user observed about 20 mm insufficient clearance. This repeats the
`log_62`/`log_64` result: the 260 mm additive peaks demand a 720 mm lateral
reversal over only 500 mm. Do not repeat this layout unchanged.

This correction is now implemented. Lap 1 rebuilds from the baseline after each
new confirmation, and only an adjacent opposing outer pair uses the existing
160 mm optimized clearance. The seat-6/seat-9 reversal is therefore 520 mm with
a nominal 47.5 mm outer-wheel margin; isolated, moderate, and nonadjacent routes
remain at 260 mm. Deterministic checks cover the special-case selection and
margin. The IDE-managed build passed; no upload was performed.

After the user uploads, repeat this exact layout once with the disable switch
reachable. The red injection should log `clearance_mm=260`; after green confirms,
the rebuilt pair should log `clearance_mm=160`. Require no contact or near-zero
speed, materially less than 61.1 degrees maximum heading error, two injections,
and a completed lap. Stop after one failure and inspect the log.

`log_80` failed because that correction activated too late. Red seat 6 still
printed `clearance_mm=260`; the robot had already driven to that outer peak when
green seat 9 activated 160 mm. Rebuilding red points behind the robot could not
improve the remaining transition. Heading error reached 63.1 degrees and the
robot physically stalled against green until the user moved it. The following
station's discovery nudge also reached about -34 degrees during the green pass.

The next version prospectively assigns 160 mm to an outer-going pillar whenever
the next station in the same section is unresolved, and suppresses following-
station target nudging until 100 mm after a confirmed extreme pair. Pure Pursuit
remains the only steering calculation. The build passed; no upload occurred.
Repeat the same layout once. Both injections must print `clearance_mm=160`, the
nudge must return toward zero after green confirms, and the robot must pass with
continuous motion and no contact. Stop after one failure and inspect the log.

`log_81` passed both pillars physically with the expected two 160 mm injections
and continuous motion. The later abort at empty S0 station 0 was caused by the
extreme-pair nudge-release condition becoming active again cyclically near the
lap end. It is now a one-shot latch that permanently clears 100 mm after green.
The build passed.

The old ToF summary also selected the fixed seat-side sensor, which is the wall
side after a legal pass. The logger now collects both sensors and reports the
actual pillar side (left for red, right for green) plus the opposite wall side.
Approximate `log_81` telemetry estimates were 27/29 mm wheel clearance to the
red/green pillars and 222/189 mm to their outer walls; the pillar values are
sparse rather than exact minima.

Upload and repeat the same layout once. Require both passes, S0 station 0 to
clear, a completed lap, and dual-sided ToF reports. On success, close this
unlikely worst-case adjacent regression and return to normal placement tests.

`log_82` caught the green rear wheel again. The corrected dual-ToF logger made
the margin failure explicit: red had 47 mm estimated wheel clearance and 208 mm
to its wall, while green reached -16 mm estimated wheel clearance and still had
207 mm to its wall. Both pillars were detected early, both used 160 mm, and the
next-station nudge stayed at zero; placement tolerance at the second peak is the
remaining cause. The later S1 station-2 abort followed the physical stall and
does not constitute a separate valid-run failure.

Only the second member of the extreme adjacent pair is now raised to 210 mm;
the first remains 160 mm. This gives green 50 mm more requested rear-wheel
margin and leaves an estimated 157 mm wall margin, while keeping the reversal
at 570 mm rather than the original 720 mm. The build passed. Upload and repeat
once: require red `clearance_mm=160`, green `clearance_mm=210`, positive dual-ToF
pillar clearance, no contact/stall, and one completed lap.

`log_83` is not a valid pass even though the firmware completed the lap. Green
passed very well, but the robot struck the red pillar and remained nearly
stationary until the user moved it. The full passage reports estimated red
pillar/wall wheel clearances of -27/245 mm and green pillar/wall clearances of
57/155 mm. Green was confirmed while the robot was still beside red. Rebuilding
the complete overlapping 210 mm green taper at that instant pulled upcoming
Pure Pursuit points toward the red side before the rear wheel had cleared it.

The next version keeps the green confirmation but defers activating its path
geometry until the rear axle is 100 mm past the red seat. The red-only 160 mm
route therefore remains unchanged during the complete red pass; about 400 mm
of approach remains for the 210 mm green route. This is still one Pure Pursuit
path and adds no steering override.

Every confirmed passage now emits three source-specific clearance reports:

- `[CLEARANCE PLAN]` calculates minima over the current planned route.
- `[CLEARANCE ODOM]` calculates minima from every sampled odometry pose.
- `[PILLAR TOF]` retains the physical side-sensor minima and wheel-inset
  estimates.

The geometry reports use a conservative 70 mm-radius complete-robot capsule
from the rear axle to the measured 130 mm front plane. They print minimum
clearance to the 85 mm pillar movement circle, minimum clearance to the nearest
outer wall/inner wall face/inner wall corner, the robot and wall coordinates at
the minima, and clearance to all four inner corners (SW/SE/NE/NW). Keep the
three sources separate: plan is commanded geometry, odometry is estimated
motion, and ToF is a physical beam measurement. A discrepancy is diagnostic,
not a reason to average the numbers.

The IDE-managed build passes with 294008 bytes RAM and 361136 bytes flash. No
firmware was uploaded. After uploading, repeat only the same CW section-1
red-seat-6 then green-seat-9 layout. Expected sequence: red injects at 160 mm;
green confirms with `injection=DEFERRED`; after the robot is 100 mm past red,
green injects at 210 mm with `delayed_until_first_clear=yes`. Require no red or
green contact, no intervention or near-zero-speed stall, positive odometry and
ToF pillar minima consistent with the visible gaps, and adequate wall/corner
clearance. Stop after one physical failure and inspect the new reports.

`log_84` failed that repeat at red. The defer mechanism worked: green remained
confirmed but uninjected throughout the red contact, then injected only after
the user removed red and the robot travelled past it. Therefore the green
taper did not cause this collision. The red-only 160 mm route reached -32 mm
odometry/capsule clearance and -6 mm ToF wheel-clearance estimate, while about
232-242 mm remained to the inner wall/corner. Green subsequently passed with
57 mm ToF pillar clearance and 155 mm wall clearance.

The planned-clearance snapshot and 200 mm first-member change are implemented.
`[CLEARANCE PLAN]` now includes `snapshot=injection`; later path rebuilds cannot
rewrite it. The second remains 210 mm with its 100 mm deferred activation. The
IDE-managed build passes; no firmware was uploaded.

After uploading, repeat the exact layout once. Require red to inject at 200 mm,
green to confirm as deferred and later inject at 210 mm, no contact or manual
intervention, and one completed lap. Compare PLAN, ODOM, and ToF pillar and wall
minima for both passes. Because the requested reversal increased from 570 to
610 mm, stop after one failure rather than repeating it unchanged.

`log_85` passed that run without contact or intervention. Red had approximately
20 mm visually observed clearance and 15 mm ToF wheel-clearance estimate;
green had 57 mm ToF clearance. Their opposite wall estimates were 232 and
153 mm. The immutable PLAN snapshots reported 50.3 mm red and 31.2 mm green
pillar margin. The lap completed with 100.0 mm maximum CTE and 54.4 degrees
maximum heading error. See `OBSTACLE_CLEARANCE_LOGGING.md` for the complete
field definitions and worked interpretation.

The red margin remains below the preferred approximately 30 mm robustness
target and prior runs showed placement sensitivity. Repeat this exact layout
once unchanged before changing code. Require another no-contact lap, no stall,
the correct 200/210 mm injection sequence, and record the physical red gap. A
contact or materially smaller margin blocks later tests; a comparable safe
repeat permits an explicit decision whether this unlikely adjacency is
provisionally sufficient.

`log_86` passed the unchanged repeat without contact or a stall. Red was
comfortable at 73 mm ToF clearance. Green looked close and measured 28 mm ToF
clearance against its 31.2 mm PLAN snapshot, with 171 mm remaining to the wall.
Maximum CTE was 76.2 mm and maximum heading error was 68.0 degrees. Together
with `log_85`, this provisionally accepts the unlikely directly adjacent
extreme pair at 175 mm/s. Do not enlarge the reversal from one 28 mm pass while
heading error is already high.

Optimized-path construction now retains the validated layout-dependent values
on later laps: 260 mm for ordinary or isolated pillars, and 200/210 mm for the
unlikely extreme adjacent pair. It no longer falls back to the unsafe blanket
160 mm value. The three-lap live-test commands are `Y3` for CCW and `Y-3` for
CW; `Y1` and `Y-1` remain one-lap tests. Three-lap status telemetry is reduced
from 200 ms to 600 ms so the complete run is likely to fit in the 128 KiB USB
log, while camera, clearance, and event reports are not throttled. No firmware
has been uploaded yet. The IDE-managed `giga_r1_m7` build passed with 295576
bytes RAM and 363472 bytes flash.

## 4. Validate three laps and optimized paths

- [ ] First run: one red pillar at CW seat 0 (section 0, station 0, right),
      using `Y-3`. This isolates lap wrapping before repeating a complex
      layout. While the next station is still unresolved, lap 1 provisionally
      uses 200 mm for this outer-extreme seat; after the empty adjacent station
      is known, the validated later-lap route uses 260 mm.
- [ ] Confirm `[PILLAR PASS] seat=0 lap=1`, `lap=2`, and `lap=3`; compare the
      PLAN, ODOM, and ToF minima for repeatability.
- [ ] Lap 1 resolves occupied and clear stations without stopping early.
- [ ] `[PATH] Optimized laps 2-3 path built clearance_policy=validated-layout`
      appears exactly once after lap 1.
- [ ] The later-lap build reports
      `[PATH] Later-lap avoidance seat=0 color=RED clearance_mm=260`.
- [ ] Only confirmed occupied seats alter the optimized path.
- [ ] Pure Pursuit remains the only steering controller on laps 2 and 3.
- [ ] Optimized clearance is safe at near, middle, and far stations.
- [ ] Lap progress wraps once per physical lap and stops after lap 3.
- [ ] Pass one complete layout in each direction before increasing speed.

`log_87` exercised the three-lap command and lap wrapping, but it is not a
valid obstacle pass. The camera reported `obs=NONE`, marked S0 station 0 clear,
and completed all three laps with zero injections. The robot physically pushed
the red pillar. Right-ToF status samples near the pillar fell to 61 mm on lap
1, 95 mm on lap 2, and 111 mm on lap 3, consistent with a nearby object that
was moved during the first pass. The firmware's formal `PASS` is therefore
overridden by the physical collision. The optimized occupied-pillar path was
not tested because no pillar was recorded.

Before another moving test, run the stationary seat diagnostic with this same
red pillar and lighting. Start `S-1`, then send
`seat expect 0 0 R 400`. Place the robot facing CW with the pillar centre about
512 mm ahead of the rear-axle midpoint and 100 mm to the robot's right. Keep
the drive wheels off the ground or otherwise unable to move, although this
mode also locks the motor. Require repeated `VOTE`, `CONFIRMED_INJECTED`, or
`ALREADY_CONFIRMED` red observations. A `REJECTED_NO_BLOB` result proves a
camera/colour-acquisition failure; a reliable confirmation instead points to
the moving discovery/viewing geometry or the unsafe two-frame clear decision.
Do not repeat `Y-3` until this distinction is resolved.

`log_88` resolves the distinction in favour of moving view geometry. In the
stationary 400 mm test, red was production-valid continuously, voted on the
first frame, confirmed on the second, and remained detected at about 405.8 mm,
-9.7 degrees, and 34 mm seat-snap error. During the hand-moved portion it
remained production-valid over approximately 333-433 mm and +17 to -22
degrees. `WRONG_SEAT` events during that portion are not a detector failure:
manual rotation changes gyro heading while the stationary test's synthetic
position stays fixed, so the projected sighting can snap to the neighbouring
seat.

Do not change HSV, area, or height thresholds. The failed powered view reached
about -28.8 degrees/299 mm, beyond what the hand-moved test exercised and at
the edge of the complete-pillar acquisition window. The proposed minimal fix
is to require a more central view before `NO_BLOB` may count as clear, and to
give the existing Pure Pursuit discovery-target nudge enough angular authority
to bring that remaining seat centrally into view. Ask before implementing;
then build without uploading. The next validation should be one powered lap,
not three, in the same seat-0 layout. No additional centred stationary test is
needed because `log_88` already validated acquisition.

## 5. Full-field reliability regression

- [ ] Cover clockwise and counterclockwise three-lap runs.
- [ ] Cover every supported starting/parking-exit configuration.
- [ ] Represent near, middle, and far stations on both lateral sides.
- [ ] Test nearby opposing colours and the maximum allowed pillar count.
- [ ] Exercise bright, dim, shadowed, and edge-clipped observations.
- [ ] Exercise low and high battery conditions.
- [ ] Complete ten consecutive full runs without wrong-side passing, moved
      pillars, wall contact, or lap-count errors.
- [ ] Stop under control after exactly three laps.

## 6. Final speed optimization

Begin only after representative three-lap layouts are reliable at 175 mm/s.

- [ ] Increase the test cap in small steps and repeat one representative
      three-lap layout in each direction at every step.
- [ ] Tune curvature speed gain so corners are deliberately slower than
      straights.
- [ ] Verify smooth deceleration before corners and acceleration after them.
- [ ] At every speed, retain path accuracy, wall and pillar clearance, and
      reliable camera confirmation.
- [ ] Stop increasing speed when accuracy or perception margin degrades.

## 7. Final parking

Parking is not implemented. Develop and test it independently after the
three-lap obstacle run is reliable.

- [ ] Detect and approach the correct gap.
- [ ] Reverse without touching either magenta boundary.
- [ ] Finish fully inside the parking rectangle.
- [ ] Keep the difference between the two side distances at or below 20 mm.
