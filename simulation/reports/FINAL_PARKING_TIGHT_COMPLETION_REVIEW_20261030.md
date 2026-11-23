# Tight parking completion review, 2026-10-30

User requests reliable CW/CCW parking in the small bay; 10 mm to the pink
pieces is sufficient. No new physical run was supplied. USB logs 574--593
were copied first and SHA-256 verified again, then the unplug notice was sent.
All 20 match the already tracked originals and hashes in
[the evidence index](../evidence/parking_exit_diagnostics/README.md); no new batch
or additional physical confirmation is claimed.

## Clearance calculation

There is no blanket 30 mm pink-piece clearance in the current final entry.
The `3.0f` entry margin is millimetres, applied to both the individual body/
wheel polygons and the parking pieces. Normal runtime separation is therefore
at least about 6 mm, with additional orientation-dependent conservatism.
The ordinary planner inflates each side by 6 mm after its initial 20 mm ramp,
providing at least about 12 mm nominal separation. The previously authorized
last-resort mode retains 1 mm inflation per side (about 2 mm separation),
40 mm/s, and a fully checked path. It is not a guaranteed 10 mm physical
minimum. These existing margins and speeds have not been changed in this review.

The final whole-footprint containment margin is 5 mm and heading tolerance
2 degrees. With the nominal 247.5 mm gap and 165 mm body length, a centred
straight robot has 41.25 mm at each end. In the accepted measured gap range
232.5--262.5 mm, the fixed local target leaves at least 26.25 mm at either
end in the ideal straight pose. At the 120 mm lateral target and 125 mm
width, wall/open-edge reserves are 57.5/17.5 mm. The turn, rather than the
straight final pose, is the tight part. Full chassis and steered wheels remain
part of every accepted entry sweep; the final containment rectangle remains.

## Two reproduced defects and scoped correction

1. A robot inside the final bounding rectangle could fail the stricter neutral
   wheel clearance. `FP_VERIFY` requested replanning, but `FP_REPLAN_BRAKE`
   returned directly to verification on containment alone. The loop did not
   consume the replan limit and never searched for a centring movement.
   At local x=45.5 or 117 mm, y=120 mm, heading=0, this repeats indefinitely
   in both directions. Replanning now bypasses search only when containment
   **and** the neutral-wheel clearance pass.
2. The planner rejected its entire search if neutral wheels did not fit at
   the starting pose, even though no neutral command was requested there.
   At local (110,120,35 degrees), the currently steered robot fits and a
   checked continuation exists. The uncommanded neutral prerequisite is
   removed. Each candidate still checks its first commanded steering,
   complete sweep and final pose, including escape-prefix alternatives.

These are code/host-model reproductions, not an assertion that they explain
the historical physical aborts. Earlier archived failures and fixes are in
[the pose/replan review](FINAL_PARKING_POSE_REPLAN_REVIEW_20261030.md).
Approach, scan, camera observer, exit movements, global geometry, motion
health checks and M4 are unchanged. New parking revision:
`20261030_tight_completion`.

## Validation

- `check_parking_tight_completion.py`: 30/30 complete without modeled contact,
  in both directions with physical radius factors 0.95/1.00/1.15. Includes
  the three defect poses and exactly 10 mm straight-body end clearance.
  Unmodified firmware passed only 12/30. Restoring just the old neutral
  prerequisite fails 6/30; restoring just the old finish loop fails 12/30.
  Both negative controls return nonzero as expected.
- `check_parking_adaptive_completion.py`: expanded from 70 to 350 cases over
  five gaps (232.5/242.5/247.5/252.5/262.5 mm), both directions, seven entry
  anchors and five physical radius factors (0.95--1.15). 350/350 complete;
  maximum four replans and 387.201 mm travel; eight health checks pass.
- `check_final_parking_sequence.py`: 396/396 complete, five registration
  checks pass. Includes full approach, marker scan and entry.
- Segment sensor-resume: 42 pass plus missing-servo negative control.
  Braking: 102 pass.
- IDE-managed incremental M7 build SUCCESS; RAM 453688/523624 bytes,
  flash 548704/786432. Existing Serial macro redefinition warning remains.
  `firmware.bin`: 552244 bytes, SHA-256
  `7718361204f8903bc3bba499f22e0d7dacd84f4178d839178c941ee87af7a618`.

No flash or movement was performed. The host plant reports its actual yaw
and position; unknown localization bias, steering transients, slip and physical
piece-placement error are not validated by these checks. The collision checks
reuse the production footprint geometry. These results do not prove universal
or contact-free physical parking. Actual unsafe geometry, invalid sensing,
unsettled braking or exhausted collision-free search can still stop the robot.

Next: tester uploads M7 only, runs normal CW/CCW three-lap courses and saves
normally. Secure originals first, match the new revision, inspect PLAN/REPLAN/
RESULT and final neutral wheels/whole-body containment. Record contact or
manual intervention by log number. No dedicated ToF sweep or extra test motion.
