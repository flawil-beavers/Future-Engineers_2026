# Final Parking Design Basis

**Project:** WRO Future Engineers 2026 robot  
**Date:** August 2026  
**Status:** Model-validated entry and staged firmware implemented; physical validation pending

## 1. Scope and authoritative rules

This document defines the engineering basis for final parallel parking after
three obstacle laps. It is deliberately a design gate, not an assertion that a
safe trajectory already exists.

The applicable international sources are the official WRO 2026 Future
Engineers rules and the official 2026 Q&A:

- <https://wro-association.org/wp-content/uploads/WRO-2026-Future-Engineers-Self-Driving-Cars-General-Rules.pdf>
- <https://wro-association.org/competition/questions-answers/>

The WRO Switzerland task page currently points teams to the international
WRO Association rules, and the Swiss FAQ contains no final-parking override:

- <https://wro.swiss/en/future-engineers-2026-tasks/>
- <https://wro.swiss/faq-de/>

The sources establish these requirements:

1. Two coin tosses select one of all four straight sections as the starting
   section, and the parking lot is always in that selected straight.
2. It is 200 mm deep and `1.5 * robot length` long.
3. Its two magenta limits are each `200 x 20 x 100 mm`.
4. After three laps, signs no longer need to be passed on their prescribed
   side, but they still must not be moved.
5. Full parking points require the complete vehicle projection to lie inside
   the rectangle between the limits and the vehicle to be parallel to the
   outer wall.
6. Parallel means that the two relevant wheel-to-wall distances differ by no
   more than 20 mm.
7. Touching either parking limit ends the round and scores no parking points.
8. The official Q&A permits parking in the opposite driving direction, but
   also forbids changing the robot size to simplify parking.

Within the selected straight, Figure 4 does not define two arbitrary parking
offsets. The right magenta piece is fixed next to the right dotted section
boundary and the left piece moves to establish the calculated gap. The four
physical field locations are rotations of this same geometry. The firmware's
canonical-south frame may therefore rotate all four into one coordinate frame,
but it must still handle both driving-direction mirrors.

The rules also define exactly when the last signs stop imposing a pass side.
The third lap is complete once the complete vehicle has driven out of its last
corner section. From that point, Appendix A allows every sign on the subsequent
route to the parking lot to be passed on either side. No sign may be moved. The
current path code counts a lap later, when its progress index wraps near the
middle of the canonical starting straight; that is conservative, but it
creates two distinct parking approaches:

- CCW/east at the current wrap pose: the parking lot at positive canonical x
  is still ahead.
- CW/west at the current wrap pose: the parking lot is behind the vehicle and
  must be approached in reverse or after a legal change of direction.

The final controller must explicitly test both cases. It must not assume that
the parking lot always lies ahead merely because every physical field location
was rotated into the canonical south frame.

## 2. Known prototype geometry and calculated target

The current measured/modelled straight-wheel geometry is:

| Quantity | Value |
|---|---:|
| Rear axle to front | 125 mm |
| Rear axle to rear | 40 mm |
| Overall length, `L` | 165 mm |
| Straight-wheel outside width, `W` | 125 mm |
| Full-steering swept width | about 135 mm |
| Wheelbase | 100 mm |
| Full-lock rear-axle radius | about 109 mm |
| Parking length, `G = 1.5L` | 247.5 mm |
| Parking depth, `D` | 200 mm |

For a straight, centred final pose, the available longitudinal clearance is

`G - L = 247.5 - 165 = 82.5 mm`,

or `41.25 mm` at each end. Measured from the rear marker's inside face in the
parking direction, the rear-axle target is therefore

`40 + 41.25 = 81.25 mm`.

The straight-wheel lateral clearance is

`(D - W) / 2 = (200 - 125) / 2 = 37.5 mm`.

The preferred parking-local target is consequently

`(rear axle x, y, heading) = (81.25, 100.0, 0 degrees)`.

In the canonical south-section field frame this mirrors to:

| End orientation | Canonical rear-axle target |
|---|---|
| East / CCW orientation | `(313.75, -1400.0, 0 degrees)` |
| West / CW orientation | `(398.75, -1400.0, 180 degrees)` |

These coordinates use the confirmed 165 mm robot projection.

The 20 mm rule limit is loose compared with the existing exit heading gate.
For the 100 mm wheelbase, a 2 degree heading error produces only
`100 * sin(2 degrees) = 3.49 mm` wheel-distance difference. Retaining the
existing 2 degree gyro gate therefore provides substantial parallelism margin.

## 3. Why the current exit cannot simply be reversed

Ackermann motion is kinematically reversible: reversing the order of a proven
exit's segments and negating each drive direction retraces the same geometric
rear-axle curve while retaining each steering command. The current exit is
therefore a useful seed for a parking search.

It is not, however, a rules-safe final trajectory. Its parked rear-axle
position is `(90.0, 137.5, 0 degrees)` in parking-local coordinates. With the
125 mm straight-wheel width, its open-side projection is
`137.5 + 62.5 = 200.0 mm`: exactly on the open boundary, with zero nominal
placement or tracking margin.

The current five controls were also checked with the same multi-part footprint,
109 mm radius, 1 mm swept sampling, `+/-5 mm` placement error, `+/-1 degree`
heading error, and two checked gaps used by the exit model:

| Parked local y | Nominal open-side margin | Passing cases |
|---:|---:|---:|
| 100.0 mm | 37.5 mm | 0 / 16 |
| 132.5 mm | 5.0 mm | 13 / 16 |
| 137.5 mm | 0.0 mm | 16 / 16 |

The deeper targets fail because the existing path was specifically selected
for the near-opening departure placement. A blind reversal could reproduce a
contact-free start pose yet still lose full-parking points, or collide when
asked to finish farther inside. It must not be installed as final parking.

## 4. Recommended final-parking sequence

### Phase A - controlled handoff after lap three

Do not mark the obstacle run complete at the current path wrap. Reduce speed
and hand control to a dedicated parking state machine while the field pose,
gyro heading, complete lap-one seat map, and measured localization uncertainty
are still available.

For initial implementation, retaining the current conservative wrap point near
canonical `x=0` is acceptable. The direction-dependent handoff must be explicit:

- CCW: continue forward toward positive x on the mapped outer approach.
- CW: reverse toward positive x on the mapped outer approach while retaining
  the west-facing heading, or use a separately modelled legal turnaround.

Do not transition earlier at the corner boundary until firmware can prove that
the complete robot projection, including uncertainty, has left the last corner.

### Phase B - approach outside the parking pieces

Use a separate low-speed connector in the starting straight. It should reuse
the known seat map to avoid the inner-row signs. Rules move every sign in the
parking section toward the inner wall, so the route may use the outer side,
but it still has to check the stored sign envelopes and must not assume the
lane is empty.

The connector ends parallel to the outer wall on a scan line outside the
200 mm magenta ends. Its exact path and speed are model outputs, not new fixed
constants. Pure Pursuit can follow the forward connector, but the close-range
multi-point parking manoeuvre should retain the exit controller's explicit
steer-settle, bounded-distance, brake, and stop states.

### Phase C - reacquire both magenta pieces

Do not park from three-lap odometry alone. At low speed, scan along the
starting straight with the outer-wall-side ToF sensor while gyro holds the
vehicle parallel. The scan must cross both 20 mm pieces and the gap between
them.

Reuse the already implemented raw-frame and geometry logic:

- inspect only fresh ToF sequence numbers;
- distinguish the short magenta return from the outer-wall return;
- require two consecutive pose-consistent wall frames;
- transform the documented 22 degree ToF cone into field coordinates;
- use the known fixed piece to establish absolute field `x`;
- derive field `y` from the outer wall;
- identify both inside faces and compare their measured separation with
  `1.5 * measured robot length`;
- reject incomplete, reversed, or geometrically inconsistent edge sequences.

At 60 mm/s, a 20 mm piece remains under the sensor for about 333 ms. With the
configured 30 ms ToF timing budget this permits roughly eleven measurement
windows in ideal timing, so the existing two-fresh-frame requirement is
feasible without inventing a new faster sensor mode. Firmware still needs a
hard travel limit and must stop outside the bay if both pieces are not found.

### Phase D - compute the bay frame and target

Let `xRearFace` and `xFrontFace` be the detected inside faces expressed in the
chosen parking direction. Then compute, rather than hard-code:

- `gap = xFrontFace - xRearFace`;
- `longitudinalSlack = gap - robotLength`;
- `rearAxleTarget = xRearFace + rearOverhang + longitudinalSlack / 2`;
- `lateralTarget = outerWall + parkingDepth / 2` in field coordinates;
- target heading equal to the starting-straight axis.

The complete projected footprint plus its declared localization/tracking
uncertainty must fit inside the measured rectangle before motion is armed.

### Phase E - generate a new swept entry path

Extend `parking_exit_swept_search.py` into a parking-entry search. Search from
the fully contained target outward, then reverse the resulting controls for
entry. This preserves the useful mathematical reversibility without inheriting
the old zero-margin target.

Keep the proven model ingredients:

- the six-part robot footprint with independently steered front wheels;
- measured Ackermann wheel angles and approximately 109 mm full-lock radius;
- explicit outer-wall and both-marker collision tests;
- 1 mm swept sampling;
- at least 20 mm per unchanged control segment;
- both nominal and minimum/maximum plausible gaps;
- `+/-5 mm` placement and `+/-1 degree` initial-heading cases;
- mirrored CW and CCW validation.

Add final-containment and uncertainty checks. The search is acceptable only if
every tolerance scenario has positive wall/marker clearance and the final
straight footprint remains strictly inside all four parking boundaries. The
number of segments and their distances must come from that search; they must
not be copied from the exit table merely for symmetry.

The dedicated search now produces this entry from capture pose
`(270.26, 274.60, 0 degrees)` to the centred target
`(81.25, 100.00, 0 degrees)`:

| Segment | Drive | Steering | Distance |
|---:|---|---:|---:|
| 1 | reverse | 0 | 20 mm |
| 2 | reverse | +50 | 120 mm |
| 3 | reverse | 0 | 80 mm |
| 4 | reverse | -50 | 65 mm |
| 5 | forward | +50 | 20 mm |
| 6 | reverse | -50 | 35 mm |
| 7 | forward | 0 | 25 mm |

For CW, steering signs mirror while drive directions and distances remain the
same. `parking_entry_swept_search.py` checks all 16 combinations of gap
`242.5/252.5 mm`, capture translation `+/-5 mm`, and heading `+/-1 degree`;
all 16 pass the swept collision and final-containment gates.

### Phase F - execute and verify

A suitable firmware state machine is:

1. `PARK_APPROACH`
2. `PARK_SCAN_ALIGN`
3. `PARK_SCAN_MARKERS`
4. `PARK_CAPTURE_POSE`
5. repeated `PARK_SEGMENT_SETTLE`, `PARK_SEGMENT_DRIVE`, and
   `PARK_SEGMENT_BRAKE`
6. `PARK_FINAL_ALIGN`
7. `PARK_VERIFY`
8. `PARK_HOLD` or `PARK_ABORT`

Every state needs a distance/time bound, fresh gyro health, and an immediate
motor stop on invalid geometry. Final acceptance requires all of the following:

- both marker inside faces were directly observed;
- their gap agrees with the measured robot length and rule factor;
- gyro heading error is at most 2 degrees;
- the calculated complete footprint, including uncertainty, is inside the
  detected rectangle;
- measured and commanded speed are below the existing soft-stop threshold;
- steering is centred and the drive remains locked off.

If a gate fails, stop outside the bay. Do not continue on nominal odometry and
do not use contact with a marker or wall as localization.

## 5. Required implementation and test order

1. Directly measure final front overhang, rear overhang, straight-wheel
   projection width, and full-lock swept outline.
2. Finish and physically validate reverse exit localization in both directions.
3. Complete the missing start-section discovery connector and validate normal
   three-lap runs with the parking pieces installed.
4. Extend the swept search for the fully contained target and require every
   tolerance case to pass in both mirrors.
5. Simulate all four combinations of driving direction and parking approach:
   CCW-forward, CCW-opposite, CW-reverse, and CW-after-turnaround. Select by
   worst-case clearance and reliability, not by one convenient field setup.
6. Implement only the dual-marker scan and verify its logged edge order, gap,
   field pose, and uncertainty without entering the bay.
7. Add the generated parking segments behind a test-only segment limit.
8. Validate one segment at a time, first with no pillars and then with every
   legal starting-section pillar placement.
9. Accept final parking only after repeated full-inside, no-contact results in
   both CW and CCW runs.

For a fully reproducible calculation, provide a scaled top-down outline at
steering `-50`, `0`, and `+50`; exact front/rear axle offsets; measured
left/right forward and reverse radii;
braking overshoot at the intended parking speeds; ToF raw logs over both
magenta pieces; and end-of-third-lap logs for CW and CCW with the parking pieces
installed. With those inputs, search robot lengths in fixed increments and
rank them by minimum swept clearance, final containment margin, segment count,
path length, and measured stopping uncertainty.

The firmware exists behind `OBSTACLE_FINAL_PARKING_ENTRY_ARMED=false`. Do not
arm it before the connector, dual-marker scan, and capture pose pass physically
in both directions. Then raise the test segment limit one reviewed segment at a
time; do not jump directly to unrestricted entry.
