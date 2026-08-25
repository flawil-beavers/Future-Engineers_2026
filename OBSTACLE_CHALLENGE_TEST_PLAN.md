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
- [ ] Test two adjacent pillars with opposing colours. This is the next test.
- [ ] Confirm each pillar selects the correct seat and is injected once.
- [ ] Confirm overlapping eight-waypoint detours remain smooth and preserve
      pillar-circle and wall clearance.
- [ ] Mirror direction or lateral side where needed to cover a genuinely new
      geometry rather than a rotational duplicate.

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

## 4. Validate three laps and optimized paths

- [ ] Lap 1 resolves occupied and clear stations without stopping early.
- [ ] `[PATH] Optimized laps 2-3 path built` appears exactly once.
- [ ] Only confirmed occupied seats alter the optimized path.
- [ ] Pure Pursuit remains the only steering controller on laps 2 and 3.
- [ ] Optimized clearance is safe at near, middle, and far stations.
- [ ] Lap progress wraps once per physical lap and stops after lap 3.
- [ ] Pass one complete layout in each direction before increasing speed.

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
