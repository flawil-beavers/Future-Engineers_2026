# Bounded later-lap efficiency review, 2026-11-20

Scope: choose a faster safe known-map route for laps2/3, preserving the recent approximate50mm wheel/body clearance target, smooth1->2 handoff and3->parking transfer. No change to camera/rule storage, discovery pace, M4 or parking entry. Current path identity `20261120_lap23_efficiency`.

## Evidence and comparison

The six unchanged USB originals were copied first and SHA256 verified against source-before/source-after/copy; user immediately released the stick. No new run. [Verification manifest](../evidence/log_intake/20261119_batch_3_6/later_lap_efficiency_source_verified.json), [original evidence index](../evidence/parking_exit_diagnostics/README.md). These older logs are not physical acceptance of this revision.

Baseline is the preceding605368B binary, SHA256`560961a45da4816ac08f689b70f4521119b34ef15da47a6a640445e053ef3f19`, path`20261119_lap23_reserve`. Original reference source/fixtures, candidate outputs and reproducible analyzer results remain below ignored `local_workspace/later-lap-efficiency-20261120/`.

Several alternatives were tested locally before selecting the small change:

- Raising planned acceleration/deceleration180->220mm/s^2 or acceleration alone to260 produced small mean gains but also slower examples. These changes are rejected. Keep180/180, the520mm/s ceiling,800mm/s^2 lateral budget,10% speed tolerance,15% curvature tolerance and0.40s response budget.
- Changing the route-cost speed calculation alone did not improve the example set; two resulting final routes were slower. Not retained.
- Allowing0.20rad additional total turn in every inner-search candidate improved22 matrix cases but slowed6, by up to0.4134s/lap. Changing early mixed bypasses could prevent a better subsequent connector. This unrestricted variant is rejected.

## Retained change

The existing optional inner-route search previously allowed only0.01rad extra absolute turn over the entire loop. A short smooth connection from the protected start line can require more total turn even when it removes travel and stays within all curvature/steering limits. Permit up to0.20rad(~11.5deg) extra **only when every confirmed sign in the three normal sections permits an inner pass**. The protected start section may still have another passing direction. Passing policy uses stored effective colours, including a modified sign's original colour.

Mixed inside/outside normal sections retain0.01rad. Existing candidate rules remain: shorter path, lower modeled target time, bounded peak curvature/bending, complete wall/pillar/pink-footprint preflight, CAD/motor rollout with perturbations and force limits. The20mm start-middle clearance correction remains applied after optional route selection. This is a bounded local search, not a globally shortest/fastest-path proof or permission to cut a pillar.

No extra planner pass, larger time budget or runtime stop is introduced. Keep the30ms optional-search budget and restore the checked shape if a candidate cannot complete. Old logs show overall handoff planning around297--425ms and direct connectors around145--152ms; the host's unlimited optional-search clock does not establish that a particular candidate finishes on the MCU. Actual target-time gains and handoff latency require normal-run acceptance. No claim that every mixed field becomes faster.

## Verification

- `check_later_lap_efficiency.py`:2809 comparisons (2807 existing matrix cases plus2 empty-field geometry controls);8 faster,0 slower. Six improved matrix labels represent3 uncapped configurations plus capped repeats. Normal uncapped gains0.4902--0.9752s/lap, path reductions131.85--248.27mm/lap. Empty controls save284.91mm and0.4742s/lap. Maximum1.7734s/lap belongs to a140mm/s capped label and must not be advertised as a normal-run gain. The majority of already checked routes remain selected.
- Frozen CAD/controller/passing matrix:2807/2807 pass.
- Measured wheel/body polygon audit:2807/2807 pass; minimum53.969mm pillar/66.897mm inner wall, unchanged from the preceding accepted revision.
- Full motor-profile dynamics:2191 uncapped labels at each10/20/50ms,6573/6573 pass. Maximum modeled lateral acceleration0.658896/0.660617/0.678063m/s^2, below the0.8 budget; minimum diagnostic capsule-to-pillar41.030/41.875/41.335mm. These capsule distances differ from the separate measured-body polygon audit.
- Optional-budget controls4 and geometry-equivalence336 pass in each before/after run; targeted inner routes8 pass;1->2 handoff18/18 passes. The historical0.011rad assertion was updated only for the newly eligible continuous-inner cases; mixed cases keep it.
- Parking rolling handoff24 maps (20 preparations/12 rolling),0 failures;2970/2970 approach controls pass.

A separate20ms motor-ramp comparison of the3 improved uncapped configurations saves1.7000/0.8801/1.2201s across laps2+3; empty controls save0.84/0.86s. These are model times, excluding measured MCU planning latency.

These are explicit known-map simulations, with repeated layouts/speed labels and sampled trajectories. They assume correct pose/map and calibrated CAD steering; traction, servo lag and real clearance remain unmeasured. The preceding wrong-visible-colour and narrow-gap parking limitations remain in the [current handoff](../../AGENT_DOCUMENTATION.md). Do not claim universal50mm physical clearance or guaranteed completion.

Next: tester normal M7 upload and ordinary CW/CCW runs. Copy/hash/release originals first, compare later-lap times and the optional inner-route/retained-shape record, check physical clearances and both handoffs. No dedicated sensor sweep or additional robot movement is requested.

Final IDE-managed M7 build SUCCESS: RAM452176/523624B, flash601872/786432B; firmware.bin605424B SHA256`bd1d75611c2e50cb15df5a63f126d29d20fefdd43ab0856b5e90c406e7f9d597`. M4 unchanged/not built. Existing Serial macro and unused-helper warnings remain. No robot upload/movement or extra commit.
