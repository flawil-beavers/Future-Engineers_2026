# CCW log2: parking approach and start GREEN review, 2026-11-19

Original: [20261119_log_2_ccw.txt](../evidence/parking_exit_diagnostics/20261119_log_2_ccw.txt), 138372 bytes, SHA256 `ca40714175cfadcdad327c866291551c2e70f34470409c31170d484be7499a05`. Secured unchanged first, before analysis; source-before/after and destination hashes agreed. User immediately released stick. No log was deleted from removable media.

User confirms the reported light start-GREEN contact belongs to this CCW run. Other physical report: alignment too close to the parking wall, approach too slow and abrupt transition. Source revision: `20261119_measured_height`, parking `20261111_recovery_handoff`; prepared prior M7 binary SHA256 `69afd090f0d0347d930125a1b52f9c2734b339a9f7997ef720822f9ffd9d8882` is not a flash readback.

## What the original actually establishes

- All three laps counted: 59.716 / 19.588 / 18.165 seconds, 97.469 seconds total before parking. Normal-size run saved; `truncated=0`.
- Six confirmed pillars: GREEN seats1,5,6; RED11,14,21. The150mm visibleGREEN at14 locks once as effectiveRED and is retained for subsequent laps. No recognition rejection/hold in the running lap after connector completion is logged. This does not establish pixel-level or passage correctness independently.
- Initial connector rejects several unsafe candidates and eventually accepts a624mm route. Candidate rejections are planning results, not executed impacts. Twenty slip diagnostic messages occur over the run; low-distance/noise, compliance and physical slip cannot be separated by these messages alone.
- After lap3, the stopped approach search leaves the gyro unpolled for1230ms and produces a sensor hold. Approach then proceeds; after wall registration it adds a second alignment from approximatelyx697 tox890. Reverse scan begins att585804. User pause ends the file att593735, while still scanning; no autonomous parking failure, final completion or actual entry manoeuvre is in this file.
- Recorded rear-axle poses sampled roughly250ms apart imply minimum model clearance~102.7mm at startGREEN seat1 on lap2, inconsistent with reported physical touch. This is a pose/model/temporal-evidence limitation, not proof the user contact did not occur. Do not claim the contact root cause or its elimination from these poses.

## Narrow changes

- Scan/capture line moves10mm toward the mat interior: straight123mm wheel contour now has20mm nominal clearance from pink tips rather than10mm. Parked target and longitudinal rearward shift stay the same. Entry remains swept-checked from the changed capture.
- Free approach maximum120->150mm/s, alignment maximum100->130mm/s. Blend toward60mm/s near line acquisition and longitudinal stopping gate, retaining the normal motor profile and measured stop checks before reversal. Scan remains60mm/s to preserve marker sampling; entry speed unchanged.
- Service queued gyro data every10ms throughout the approach preflight, as the entry planner already does. This avoids starving sensing during computation; it does not eliminate search CPU time or authorize driving during an unvalidated search.
- For confirmed CCW startGREEN extreme pairs only, add15mm to later-lap clearance. Keep the continuous pair geometry, wall/pillar validation, first-lap recognition, speed and other sections unchanged. This is a conservative reserve pending physical recheck, not compensation for an identified103mm pose discrepancy.

## Verification and limits

- Later laps:2807 cases pass, including wall and minimum pillar reserve checks.
- Approach:2970 layout/pose perturbation cases pass in both directions.
- Complete parking state machine quick suite:3/3 park, registration5/5 pass.
- Measured body/gap/radius entry suite:24/24 pass for both directions, gaps248.9/249.6mm and radius multipliers1/1.15/1.5.
- New `check_parking_log2_acquisition.py`: production approach from this recorded handoff, with independent true Y bias-15/0/+15mm.3/3 reach reverse scan without modeled collision, tip clearance21.08/16.74/20.40mm; modeled motion8.44--11.96s. Ideal ToF/CAD and zero hardware lag; not a physical duration promise.
- Narrow gaps, unmodeled slip, sensor reflections and erroneous global pose can still prevent successful entry. No safety gate removed, no upload or robot movement performed. Next upload M7 only, perform normal CW/CCW runs, preserve originals immediately, compare physical wall clearance, scan edge detection, final parking and startGREEN passage.

Final IDE M7 build: SUCCESS, RAM452136/523624, flash598272/786432, binary601824B SHA256 `e306cfe314ca9d0181fb13a585897d36d722fa741a50d68ce8484412e4e5da5a`. M4 unaffected/not built; no upload. Existing Serial macro and unused-helper warnings remain.
