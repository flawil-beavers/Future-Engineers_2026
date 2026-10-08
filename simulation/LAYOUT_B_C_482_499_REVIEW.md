# Layouts A, B and C: logs482-499

## Evidence and physical reports

All18 complete originals are in `evidence/parking_exit_diagnostics/`, with
individual byte counts, SHA-256, identities and physical reports in its README.
No raw image was saved in the failing perception scenes; logs are not proof
of optical accuracy or an independent measurement of clearance.

A482-485: two CW and two CCW three-lap runs, all contact-free per user.
C487/488: second outer GREEN contact in S1. C489-492: several automatic CCW
holds. C starts with far-inner **RED**, as corrected by the user. Log486 is an
additional exit-only interruption; association with the one reported short
exit is not certain. Do not alter the proven five exit arcs from this alone.

These A/C logs contain parking20:56:49 / connector21:04:51 build markers.
B493-499 contain parking21:24:58 / connector21:24:45, consistent with the
preceding candidate binary. Binary identity was not read from the robot.
B493/494/496: contact-free CW3laps;495 automatic corner hold before S2.
CCW497-499: GREEN passed, no contact, then automatic start-connector hold.

## Causes and corrections

- C488 already confirms second GREEN11 at -1325,-320.6,64.32deg, range717mm.
  Separate displacement tapers create an inward valley between two GREEN ends.
  Production now joins the two known same-colour straight ends continuously,
  both live and optimized. O3 and an unconfirmed second sign retain direct
  discovery. No second pillar is assumed before confirmation.
- Official Figure8c occupancy cards exclude the middle after **either** end
  is confirmed; old inference needed both ends and caused C490's unnecessary
  middle hold. Either end leaves the other end UNKNOWN; contradictory layouts
  infer nothing. CHECK_ALL still requires direct empty-place observations.
- C489 crosses its finite connector endpoint and aligns with the outgoing
  route. Handover now also accepts a tangent-aligned projection onto the first
 250mm of that outgoing route, with unchanged60mm/15deg gates and swept checks.
- C492 and B497-499 rebuild from an advanced pose after an unrelated next-section
  confirmation; a fresh minimum350mm approach no longer fits. An unchanged
  merge is retained only after the full remaining rollout is rechecked against
  every confirmed sign, unresolved start guard, walls and the changed route.
- B495 has already cleared one first-place side. At hold the other is205mm
  away; the old reverse preflight insists on seeing both sides. Recovery now
  checks only unresolved sides and preserves independently obtained CLEAR.
  Logged pose -722.7,861.1,58.19deg permits a60mm reverse, including20mm braking
  reserve, under the existing40mm wall/potential-pillar gates. Confirmed signs
  always override earlier CLEAR. Two unresolved sides keep the old170mm minimum.
- Broad rejected GREEN regions behind a near target's ground projection can
  stop blocking that target's empty evidence. FOV/range, fresh frame count and
  local upright/unknown-occupancy veto remain. Valid colour observations cannot
  be bypassed by this background exception. No blanket green ban/top crop.
- B498/499 project distant RED to inner11 rather than physical outer10.
  B497's correct10 projection is also close to the rails' midpoint. At ranges
  over700mm, projections within70mm of that midpoint now remain UNKNOWN until
  a clearer observation distinguishes the rails. Both colours/directions use
  this guard; telemetry records `distant_side_ambiguous`. This guards uncertainty,
  not a measured camera calibration; close physical validation remains required.

Official PDF January15 edition and Q&A checked2026-10-06; rendered Figure8c
inspected. Canonical links remain in `WRO_2026_RULES.md`.

## Automatic final parking

`config.h` now selects exit -> three laps -> final parking, with entry armed
and isolated/test holds disabled. The learned start bypass is followed towards
canonical+X (CW reversing, CCW forward). A full swept preflight chooses an
outward shift at x500/550/600/650, accounting for start signs, next-corner signs,
walls and both pink pieces. A blanket650 shift failed81 modeled next-pillar
cases and was replaced by this checked selection.

Then: align scan line -> scan both boundaries with fresh ToF -> apply bounded
localization -> capture pose -> seven existing entry segments -> full stopped
containment/parallel verification -> save. The actual measured gap is used for
entry geometry. Preflight and runtime predicted clearance gates remain active.
The scan travel limit covers an approach that aligns beyond x900. Servo output
is explicitly restored after finish braking; the completion line precedes save.

## Verification and limitations

- `check_cw_start_planner.py`:8943 cases pass.
- `check_ccw_start_planner.py`:4158 starts,33 returns,135 prefixes pass.
- `check_later_laps.py`:2807 CAD cases pass, including exact A/B/C directions.
- `check_layout_c_and_join.py`:107 checks, including C488 pose perturbations,
  B497 late detections, C489 endpoint, B49560mm recovery, far-side ambiguity,
  background negatives and all64 partial occupancy masks against legal cards.
- `check_green_seat_visibility.py`: archived GREEN/RED/empty and synthetic
  dark, bright, unknown occupancy, wall and floor controls pass.
- `check_connector_servo_resume.py`: actual stop/servo transition checks pass.
- `check_final_parking_approach.py`:2970 CAD start/next-pillar/pose cases pass.
 162 measured-gap/capture perturbations:144 allowed,18 conservatively rejected;
  every nominal capture passes. Rejections are not claimed as successful parking.
- `check_final_parking_sequence.py`: actual state machine44 cases passes,
  both directions/all11 start cards at30/100ms fresh ToF intervals; ideal cone
  classification and instantaneous motor/steering response only.

Generated fixtures, outputs, analyzer reports and downloaded rules stay under
ignored `local_workspace/`. First-lap motion replay uses the existing ideal
bicycle; repeated/final motion uses CAD. Sensor noise, camera voting in the
actual scenes, brake overshoot, slip and servo delay require hardware tests.

## Exact next tests

User uploads M7 only, then disconnects the robot USB cable. Keep B unchanged:
two CCW runs first, then two CW runs, each including automatic parking. Inspect
start connector, RED outer-end classification, S2 recovery and pink clearance.
If these pass, repeat C two CW then two CCW; watch continuous two-GREEN passage.
Immediately archive each complete original before further experiments. Check
new firmware markers, connector-retain/handover, ambiguity, corner-recovery,
FINAL PARK preflight/scan/segment/result records and physical contacts.
Final parking is software-verified and not yet physically accepted. A's four
successful older runs do not validate the new perception or final-parking code.
No agent upload or commit.
