# Practical50mm later-lap clearance, 2026-11-19/20

Interpretation: aim for about50mm clear space between the nearest wheel/body point and an inner wall or50x50mm pillar during laps2/3. This is a route target, not a new instantaneous50mm stop rule. No change to lap1 speed/recognition, parking entry, recovery, map colours or Surprise Rule identity.

## Evidence and finding

The six originals still on the connected stick were recopied and SHA256 verified against source before/after and copy, before this investigation. No changed/new logs. User immediately released stick. [Verification manifest](evidence/log_intake/20261119_batch_3_6/later_lap_clearance_source_verified.json); originals and exact hashes remain in the [portable parking evidence index](evidence/parking_exit_diagnostics/README.md). These old runs do not physically validate the new route.

The existing2807-case later-lap replay passed collision/passing checks, but its minimum diagnostic capsule-to-pillar reserve was32.126mm;164 cases were below50mm. Minimum capsule wall reserve was73.300mm, with none below50mm. Existing optional route gates permitted smaller reserves; they were not a50mm contract. Capsule distances use a conservative70mm half-width and42.5mm sign movement circle, so they are not identical to nearest wheel/50mm-square sign clearance.

Independent polygon checks of the tight cases found43.143mm actual modeled wheel/body-to-sign clearance. The main shortfall was a solitary middle sign in the start section, on the repeated return approach, in both directions. Raising the ordinary-section middle offset190->205mm did not fix that protected start geometry. A smaller pursuit horizon also did not solve the physical wheel clearance. Both were local host experiments only and are not retained.

## Retained change

After selecting the existing later-lap shape, add a small passing-side correction around a confirmed solitary start-middle sign. Its magnitude is `20*sin(pi*d/400)^2` mm for cyclic distance `d<400mm` to that sign, zero elsewhere. It is zero with zero slope at the sign and400mm boundary, peaking20mm at200mm before/after. With the current500mm first-corner reference, the sign is also at the lap seam. Both colours use their stored effective passing side; no hard-coded field layout or visible-colour override.

The correction is applied once, after optional shortening/connectors. Use the existing scratch backup and complete wall/pillar/pink-wheel preflight. Retain at least60mm sampled wall reserve, lose no more than20mm of preceding wall reserve, and do not reduce the preceding sampled pillar reserve by more than1mm. Rejected optional correction restores the already checked route; it introduces no new stop. All2807 tested maps retain completion. Existing route-dependent dynamics are recalculated on the selected shape. Speeds are not reduced globally.

The existing measured-distance400mm handoff blend remains. Its18 controls now cover the prior B layout and both colours of a solitary start-middle sign, both directions, and135/180/260mm/s incoming speeds. First target jump is0mm in all18. This checks target continuity, not measured servo jerk.

## Validation

- `check_later_laps.py`:2807/2807 maps/controller cases pass, including passing side and both recorded laps. Minimum capsule wall73.300mm; capsule pillar41.550mm. Repeated speed labels and ordered neighbouring sections are included; these are not exhaustive arbitrary full fields.
- `check_later_lap_physical_clearance.py`:2807/2807 pass a50mm floor with123mm outside wheel width,35mm rear overhang,98mm wheelbase,123mm straight front and135mm steered front measured from the rear axle. Check chassis, centre extension and all four wheels at straight/both steering extremes against50x50mm signs and the complete inner square, including corners. **Minimum polygon pillar53.969mm; inner wall66.897mm.** The fixture bounds polygon excess over the diagnostic capsule; near objects receive exact edge/overlap checks and far-object skipping is permitted only beyond that conservative bound.
- `check_later_lap_physical_clearance.py --without-reserve`:24 focused controls reproduce19 cases below50mm, minimum43.143mm. This disables only the new correction in the extracted planner and expects the old shortfall.
- `check_lap_handoff.py`:18/18 pass, no initial target jump.
- `check_direct_parking_handoff_20261119.py`:24 maps,20 certified preparations,12 CCW rolling approaches,0 failures; stopped CW fallback retained.
- `check_final_parking_approach.py --approach-only`:2970/2970 pass on the changed map planner.

The body audit samples the controller trajectory at the existing20ms maximum-speed step, about11.44mm, and assumes exact map/pose and calibrated CAD steering. It is a nominal model check, not a measured physical50mm lower bound. Pose error, slip, sign displacement, actuator lag and between-sample extrema remain limitations. The preceding capsule movement-circle checks remain; the independent polygon test measures the nominal sign surface requested here. A new untested layout may reject the optional correction and retain its prior checked path. Do not claim universal5cm separation or weaken genuine fault stops.

Path identity `20261119_lap23_reserve`; discovery header still reports120mm/s. M7 only. Next: user normal M7 upload and ordinary CW/CCW runs; secure original logs first, then compare wheel clearance and actual1->2 smoothness. No agent upload, reset, movement, commit or new special sensor test.

Final IDE-managed M7 build SUCCESS: RAM452176/523624B, flash601816/786432B; firmware.bin605368B SHA256`560961a45da4816ac08f689b70f4521119b34ef15da47a6a640445e053ef3f19`. M4 unchanged/not built. Existing Serial macro and unused-helper warnings remain. No robot upload/movement or extra commit.
