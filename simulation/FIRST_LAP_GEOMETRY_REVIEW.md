# First-lap steering and approach correction, 2026-10-18

The first lap and parking-start connector now use the existing CAD/Ackermann
servo linkage, as laps 2/3 already did. Physical axle spacing, CAD calibration,
first-lap speeds and the 42-degree runtime steering limit are unchanged.
`pursuitServoForCurvature` is shared by the controller and feasibility checks.

Previously, first-lap pursuit sent the equivalent bicycle wheel angle directly
to the servo. For example, a requested 500 mm radius produced an integer
11-degree command and approximately 666 mm radius in the existing CAD model.
This correction removes that inconsistent servo/wheel assumption; it is not a
new physical calibration.

Connector preflight now propagates the actual integer servo command through
`Ackermann::getTurnRadius`, in 2 mm steps. It retains forward-target and finite
checks, the 42-degree CW / 40-degree CCW planning limits, travel/terminal gates,
all confirmed/hidden-guard pillars, 10 mm capsule reserve and 5 mm parking-piece
body reserve. Candidate lookahead/shape/merge retries remain bounded.

The duplicate steering gate at hypothetical exact Hermite waypoints was
removed. It rejected some tracks that the calibrated controller can safely
follow without occupying those exact poses. Every candidate still checks the
nominal curve's footprints and must pass the complete calibrated tracked-pose
rollout before it is accepted. Runtime steering rejection remains active.

Normal first-lap pursuit is capped at a 280 mm horizon, retaining the existing
corner multiplier and shorter low-speed horizons. The start connector keeps
its independently validated lookahead. This reduces corner cutting without
changing later-lap control or slowing the first-lap speed settings.

## Reproduction and results

Run these scripts with the IDE-managed Python from the repository root.
All generated C++, executables, inputs and reports stay under ignored
`local_workspace/`. No firmware upload or new physical test batch was received.

- `simulation/check_first_lap_geometry.py`: 1,736 known-map routes, 868 per
  direction. All 28 legal normal-section arrangements, repeated at uncapped /
  140 / 175 mm/s, plus every ordered neighbouring arrangement pair. The start
  section is empty to isolate normal-route geometry. It extracts the actual
  runtime steering/lookahead expressions and production route construction,
  uses integer steering and the CAD plant, and checks complete body/wheel
  passing sides as well as capsule clearance. Minimum modeled wall reserve
  65.3267 mm CW / 65.3270 mm CCW; pillar 47.6570 / 47.6197 mm. The same original
  clearance audit with the old controller found about 2.05 mm pillar reserve.
- `simulation/check_cw_start_planner.py`: 8,943 cases pass, including solitary
  middle RED and its perturbed approaches that previously passed ideal
  preflight but could exceed the runtime steering gate in CAD replay.
- `simulation/check_ccw_start_planner.py`: 4,158 cases, 33 stored-sign returns
  and 135 localization-prefix checks pass with calibrated continuation.
- `simulation/check_layout_b_measured_ccw.py`: 81 archived-start perturbations
  pass, including independent +/-3 mm / +/-1 degree actual starts against each
  retained connector. These are synthetic replays, not 81 new measurements.
- `simulation/check_layout_c_and_join.py`: 173 checks pass. The C passing test
  now witnesses the whole body/wheels crossing on GREEN's left using the
  runtime horizon and CAD plant; its old fixed 150 mm ideal-bicycle test used
  an x-only condition 150 mm beyond the sign instead of witnessing passage.
- `simulation/check_start_bay_return.py`: 88 calibrated first-lap returns pass.
- `simulation/check_later_laps.py`: 2,807 calibrated later-lap cases pass;
  the shared conversion preserves their existing numeric steering behaviour.
  Current-source discovery/state replay also passes (modeled corner reserve
  67.5203 mm unrounded / 109.675 mm rounded).
- Servo retain/replan regression, rectangular-wall reference, and 2,400
  C++/Python start-footprint comparisons pass.
- Final parking integration: 396/396 park; 5 registration cases and 66 brake
  checks pass. Final-parking production logic was not changed by this correction.

IDE-managed incremental `giga_r1_m7` build succeeds: RAM 433800/523624,
flash 507048/786432 bytes. Firmware SHA-256:
`ac94f696300f307b560d0dd4023ec0f75e04c0cf37666c38f78a76128c97e31c`.
M4 is unaffected and was not built. Existing compiler warnings remain.

## Limits and next step

The full-route calculation supplies a frozen, already known map. It does not
replay camera acquisition, discovery holds/nudges, ToF corrections, controller
timing, braking, servo lag or slip. The 2 mm sampled reserves are engineering
model results, not measured physical gaps or guaranteed error bounds. The
matrices are not an exhaustive combination of every complete field and pose.

Upload M7 and observe normal CW/CCW runs, especially middle-RED start, GREEN
joins and same-colour corners. Use normal saving and record actual contact,
clearance and any guarded stop. Archive complete original logs and firmware
identity according to AGENTS.md before analysing a new physical batch. No
dedicated sweep, arbitrary-placement grid or additional movement is required.
