# Current robot engineering handoff

Updated2026-11-20 after bounded later-lap efficiency and clearance; mandatory instructions are in [AGENTS.md](AGENTS.md). This is the current summary. Earlier build identities, rejected approaches and original reports remain in the [history index](docs/history/README.md), including the [exact pre-batch handoff](docs/history/AGENT_DOCUMENTATION_20261119_BEFORE_CW_BATCH.md). Historical test procedures are not active orders.

## Current source and validation

Normal O mode exits parking, drives3laps and attempts final parking. Practice/first-lap/three-lap-only flags remain off; Surprise Rule enabled by default and must match pre-run declaration. Current source revisions: path20261120_lap23_efficiency, parking20261119_direct_handoff, height20261119_measured_height. OnlyM7 affected; M4 unchanged. Final prepared binary/build identity is recorded below; no agent upload, reset, motion or extra commit.

- [Later-lap efficiency](simulation/LAP23_EFFICIENCY_REVIEW_20261120.md): only continuous inner normal sections may use0.20rad extra total connection turn; mixed bypasses retain0.01rad and all speed/force/time budgets.2809 cost comparisons:8 faster/0 slower (3 normal configurations, capped repeats,2 empty controls).6573 dynamics,2807 geometry/body audits,18 handoffs and parking controls pass; same53.969/66.897mm nominal minima. Real gain depends on MCU30ms optional budget; normal runs still required. Six unchanged USB originals reverified/released; no new evidence run.
- [Later-lap clearance](simulation/LAP23_CLEARANCE_REVIEW_20261120.md): optional smooth20mm start-middle passing correction on laps2/3; lap1 unchanged. Known-map2807 cases pass; independent measured-body audit minimum53.969mm pillar/66.897mm inner wall. Negative control reproduces43.143mm before. No new50mm runtime stop or universal physical guarantee. Handoff18/18, parking maps24 and approach2970 pass. Six unchanged USB originals reverified/released; no new runs.
- [Lap1 pace](simulation/LAP1_TEMPO_20261119.md): moving unresolved-station cap100->120mm/s; existing340mm hold/800ms grace, camera votes, geometry and recovery unchanged. Speed/recovery33, view policy270/archived286, frozen CAD1736 plus100/120 comparison112 pass. Actual recognition at higher speed still needs ordinary run acceptance. No new USB logs; six originals reverified/released.
- [Direct parking handoff](simulation/DIRECT_PARKING_HANDOFF_20261119.md): prepare sampled inner/outer arrival routes during laps2/3; conditional rolling CCW handoff, measured CW reversal brake; overlap safe servo settling with braking, preserving400ms servo age and60mm/s entry. Tests24 learned maps/12 rolling approaches,24 measured entries,102 braking cases pass. Fixed two-arc recipe is not universally clear with the measured footprint; checked fallback remains. No new physical acceptance run.
- [Current CW batch review](simulation/CW_LOGS_3_6_REVIEW_20261119.md): first-lap colour-horizon correction, targeted EMPTY guard, checked marker rescan and narrow-gap staging fallback; exact tests, evidence and unresolved cases.
- [CCW log2 review](simulation/PARKING_APPROACH_LOG2_REVIEW_20261119.md):10mm inward scan/capture shift,150/130mm/s approach/alignment maxima, terminal60mm/s blend and15mm CCW startGREEN-pair later-lap reserve. Parked target and entry speed unchanged. Physical contact was not reproduced by logged poses; pose/model/time discrepancy remains.
- [Measured camera/rule validation](simulation/SURPRISE_RULE_MEASURED_VALIDATION_20261119.md) and [approval registry](simulation/evidence/camera_diagnostics/validation_status.json):45 approved50/100/150mm reference frames pass;3 excluded. Never use wrapped/rejected images or an unapproved new reference.
- [USB heap mitigation](simulation/USB_SAVE_HEAP_REVIEW_20261119.md): unchanged colour LUT packed to4bits,32768B freed. Full new logs2--6 now saved and hash-verified without truncation, beyond the earlier930B stationary save. Runtime peak44484 exceeded old38352B heap; new heap71112B showed zero allocation failures. Old missing runs remain unrecovered.

## Received CW batch and evidence

Complete original logs3--6 are in [portable evidence index](simulation/evidence/parking_exit_diagnostics/README.md); bytes/SHA256 and source identity are in the [batch review](simulation/CW_LOGS_3_6_REVIEW_20261119.md). Immediate [intake manifest](simulation/evidence/log_intake/20261119_batch_3_6/manifest.json) preserves originals; resumed and direct-approach source verification found no changed/new files. User immediately released stick on each intake. Copies unchanged; no source deletion. Evidence/index/handoff are staged/tracked, not a claim of a new commit or remote sync.

Physical report: earlier CW missedRED and wrong GREEN passage; latest obstacle part good with smallRED, parking stops. Logs3/4 end user mode_pause;5 aborts scan_motion_clearance,6 aborts dual_marker_geometry at222.4mm. Both reaching parking have old20261111_recovery_handoff, before the prepared wider scan line. Source binary not read back. Log3 omitsRED18 after not_upcoming rejection then falseCLEAR. Log4 confirmsRED middle2 while no expected startGREEN is seen; exact camera/colour cause remains unestablished. Log6 correctly stores50mm visibleRED15 as effectiveGREEN; earlier3/5 store150mm visibleGREEN15 as effectiveRED. No hard-coded layout.

## Results and remaining limits

Current tests: speed/recovery33; far52; recovery dispatch33; approved frames45 and captured sequences16; rendered648/association48/policy24; corner3 regression2 plus negative control; occupancy/EMPTY controls;565 evidence10/rejoin27; parking quick3/registration5; braking handoff6; measured entry24; new scan retry/safety controls pass. Accepted-span parking stress114/120 park;6 cases at232.5mm with15--50% curve-radius change still safely abort. All24 nominal measured-gap cases pass. Model results exclude unmeasured traction, actuator lag and real reflections; do not claim all parking or wrong-colour problems solved.

The first-lap horizon now permits checked colour acquisition through a corner out to its1600mm calibration limit; EMPTY retains its own nearer/heading gates and active same-station searches. Invalid gap permits one fully checked rescan, not relaxed geometry. Narrow-gap fallback checks the entire straight staging prefix plus tail before moving. True sensor/pose/brake faults and an unsafe or twice-invalid scan remain stop conditions.

## Exact next steps

1. User normal upload ofM7 only. No auto-upload or motion authorized by this review. Use [current normal-run checklist](OBSTACLE_CHALLENGE_TEST_PLAN.md), not archived sweeps.
2. Normal CW/CCW runs; immediately copy full original logs, SHA256-verify, release stick and update portable evidence before analysis. Preserve [parking pose diagnostics plan](PARKING_POSE_IMPLEMENTATION_PLAN.md).
3. Compare later-lap timings and accepted/retained inner-route record; verify real later-lap wheel/body clearances and smooth start-middle passage; verify lap1 early recognition at120mm/s, earlyRED18 association, both startGREEN passages, once-only special identity/effective colour across3laps, parked pose/parallel completion, real wheel clearance, conditional CCW rolling handoff and actual planning pauses. Keep the232.5mm/large-radius limitation visible.
4. If log4-like wrong colour recurs, capture a stationary image only with driveOFF and obtain user image approval before analysis. The new cached error-bit field helps distinguish capture errors; it does not prove hardware damage.

[Official rules](WRO_2026_RULES.md) remain canonical; consult official clarifications before new rule-dependent changes. Video artifacts and reconstruction assumptions remain in [video guide](simulation/VIDEO_VISUALIZATION.md) and history; log poses are anchors, not independent physical truth.

Final IDE-managed M7 build SUCCESS: RAM452176/523624B, flash601872/786432B; firmware.bin605424B SHA256`bd1d75611c2e50cb15df5a63f126d29d20fefdd43ab0856b5e90c406e7f9d597`. M4 unchanged/not built. Existing Serial macro and unused-helper warnings remain. No robot upload/movement or extra commit.

## 2026-11-18: README source review

Updated README operation for the current full Obstacle run, rear-ToF ownership
and pins, fused IMU heading, measured versus nominal geometry, final-parking
recovery and current validation limits. Added the supplied challenge videos,
moved BOM before assembly and corrected the listed cost sum to CHF 304.65.
No firmware, physical run or evidence log changed. The team confirms unchanged
dimensions after adding rear ToF; mass needs remeasurement, photos will be
added later, and CAD/print exports and wiring still need updating. Print
settings remain 0.2 mm without supports. Next replace those assets and reconcile
the final specification via DOCUMENTATION_TODO.md. README.pdf remains unchanged
at the user's explicit request.

## 2026-11-20: CAD and print documentation

Linked supplied `CAD/Car_v74.f3z` and moved its unchanged binary G-code into
`CAD/printing/`. Original slicer screenshots are in `img/assembly/`
as `Car_v74_prusaslicer_models.jpg` and `Car_v74_prusaslicer_gcode.jpg`.
At the user's direction, earlier CAD versions are retained through Git history.
Updated README assembly downloads, screenshots and print settings,
the CAD guide and documentation checklist; fixed the supplied team-photo link.
SHA-256 checks verified every moved asset unchanged. Job metadata confirms
PrusaSlicer 2.9.6, MK4IS, PLA, 0.4 mm nozzle, 0.2 mm layers, 15% infill,
67.67 g and estimated 3 h 1 min 6 s. Screenshot uses support enforcers only.
Next confirm version 74 matches the frozen robot, replace robot photographs and
update wiring before final documentation/release checks. README.pdf remains
deferred. No firmware or physical validation changed.

The subsequently supplied `CAD/Car_v74.3mf` is linked in both READMEs as the
printing entry point. Archive integrity and saved configuration were checked:
17 part meshes, plate arrangement, MK4IS, PLA, 0.4 mm nozzle, 0.2 mm layers,
15% infill, support enforcers only and no brim. Instructions explain opening
it as a PrusaSlicer project and selecting the intended printer/material profile.

## 2026-11-20: World Final journal rewrite

Audited the official rubric, General Rules and Q&A. The team confirms v74
includes the current rear mount, identifies the Arducam GC2145 and replacement
power components, and confirms the public GitHub URL was submitted. Keep
firmware on current `main`; record the actual competition commit and settings
when known. The README now links tracked evidence and identifies historical
test conditions. Local logs 76/111, 122, 204–209 and 227 corroborate the
camera, exit, crawl-speed and rear-reference examples. No new runs or firmware
changes. Next verify signed-out videos, replace robot photos,
record physical measurements, then complete the PDF/hardcopy checks in
[DOCUMENTATION_TODO.md](DOCUMENTATION_TODO.md).

The supplied [revision 1 wiring PNG](img/diagrams/Wiring_Diagram_FE2026_v1.png)
replaces the June drawing. The updated export shows motor controls and outputs
on M2, the corrected voltmeter polarity and the current component labels.
Rear I²C remains SDA on A3 and SCL on A4, matching M4 source. The team confirmed
the power wiring on the robot, including the Mini-360 output and loads, ground
return, module terminals, voltmeter tap and motor supply.

The team confirmed that each ToF uses a half STEMMA QT/Qwiic lead with
jumpers directly to the GIGA. Separate ToF buses avoid startup address
conflicts and soldered XSHUT leads. The shared IMU/ToF I²C trial gave
unreliable heading readings, so the BNO085 uses SPI over separate jumpers.
The [README wiring section](README.md#wiring-and-signal-interfaces) records
the rationale, and the [rear setup guide](REAR_TOF_M4_SETUP.md#wiring) records
the connection method. No new firmware or physical test.

The team supplied six current robot photos. The [README photo gallery](README.md#robot-photos) now shows the front, rear, left, right, top and bottom, including the rear ToF and both battery holders. Next record final measurements and include the new photos in the final PDF export.
