# Layout B: return past parking and CCW entry

## Evidence

Complete unchanged logs477-481 are archived in
`simulation/evidence/parking_exit_diagnostics/`; its README lists hashes,
header identities and each physical report. Logs477/478 are CW,479-481 CCW.
All five contain the older Oct6 19:54 firmware headers; the preceding candidate
is not shown installed by this batch. No binary readback, new raw camera images
or independent physical pose measurements are available.

CW confirmed map: GREEN0, RED4, GREEN7, RED10, GREEN15, RED19, GREEN22.
CCW tests stopped before surveying the rest of the course. The simulated CCW B
map reverses section order, station order and left/right of this same CW map.

## Findings

- Both CW returns activate stored start GREEN0 and round corner3 between
  GREEN22 and GREEN0, then stall:6.0/7.7mm movement in1000ms. User identifies
  physical pink-boundary contact. The nominal rounded path itself clears the
  bay; logs do not record a continuous pose/ToF trace at contact.
- Discovery lap still used radial-offset wall geometry. A perfect-pose host
  counterexample produces139.5mm fictitious accepted residual. This is a real
  localization defect, but not proof that it alone caused these contacts.
  Pink pieces could also intercept a beam otherwise assigned to the outer wall.
- CCW480/481 accept a connector and reject its first actual command at
  -42.077/-42.037deg vs42deg. This is insufficient planning margin, not failure
  to recognise GREEN; GREEN5 is confirmed in both.
- CCW479 has an accepted75mm-lookahead connector,15704ms gyro polling gap
  during stopped candidate search and subsequent Manual disable. There is no
  explicit automatic tracking rejection. Its physical stop report is retained;
  attributing it to the same rejection would overstate the evidence.
- The nominal CCW connector is about528mm; the500mm rollout cap rejects larger
  lookaheads before their terminal heading settles, favouring sharp alternatives.

## Changes

1. Physical rectangular-wall ray reference now applies to normal driving in
   all laps, with the existing freshness/lag/residual limits. A15deg conservative
   beam cone excludes both pink boundary positions and measured gap extremes.
   Initial parking localization and scan/connector motion are unchanged.
2. Official CW inner GREEN clearances use200mm for all start seats0/2/4,
   including stored GREEN0 on return. Same-colour corner rounding is retained.
   CCW, other sections and check-all/O3 clearance policies remain independent.
3. CCW connector planning retains2deg steering reserve (40deg planning,42deg
   runtime). Its bounded simulated and encoder travel caps both become650mm.
   The three measured poses now choose150mm lookahead instead of75/50/75mm.
   Initial blanket40deg planning failed14 CW cases; it was restricted toCCW.
4. During ordinary return driving, the physical body and both extreme wheel
   poses are checked for the next50mm at10mm intervals using CAD curvature,
  5mm body inflation and existing5mm piece-face reserve. A rejection stops and
   records `[PARK RETURN]`; it never authorizes driving through the bay.
5. Existing bounded `[LATER_TRACK]` also records lap1 return. Its200-record
   budget is shared with laps2/3; no driving USB connection is needed.

## Verification and limitations

- `check_layout_b_measured_ccw.py`:81 independently perturbed measured starts
  (3logs, +/-3mmXY and +/-1deg); each accepted path additionally checked with
 27 actual-start perturbations against the retained path and42deg runtime gate.
- `check_ccw_start_planner.py`:4158 start cases,33 stored returns,135 localization
  prefix checks pass with new CCW planning/cap.
- `check_start_bay_return.py`:88 first-lap bay-return cases, all11 official start
  layouts including empty, both preceding end sides/colours, both directions.
  Actual source route and runtime footprint guard, ideal bicycle plant. Run
  `check_later_laps.py` first to regenerate its source fixture.
- `check_cw_start_planner.py`:8943 scan/connector cases pass after limiting
  the new planning reserve toCCW.
- `check_later_laps.py`:2805 passing layouts, including exact heterogeneous B both
  directions, calibrated CAD plant, actual parking runtime guard.
- `check_later_lap_wall_reference.py`:physical residual signs, vertex/grazing/
  pillar rejection and pink-boundary cone controls.

Geometry assumes known correct colours/positions. Camera errors, ToF closed-loop
trajectory, servo lag, slip and physical bay-placement variations need fresh
robot validation. These checks do not establish every full-field combination
physically reliable. Generated results remain ignored in `local_workspace/`.

## Next physical test

Upload only the completed M7 candidate, retain this exact B layout, run two CW
and two CCW full three-lap trials without a driving USB cable. Watch CW return
at the pink bay and CCW first GREEN bypass. Save each complete original log
before changing layouts. Check firmware headers, connector lookahead/hold,
`PARK RETURN`, lap1-return and later tracking, all passing sides and contacts.

Final IDE M7-only build: RAM432776/523624, flash486160/786432;
firmware SHA-256 `0efe2debc1941a47d02c1e5446b5c9fae5dc833259ff339ae544cbd94251ed2a`. Build successful; existing Serial/unused-function
warnings, no new compile errors. M4 consumers/interfaces unchanged.
