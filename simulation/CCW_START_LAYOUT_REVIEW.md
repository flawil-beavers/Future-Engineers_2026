# CCW start review — 2026-10-06

## Verdict and scope

The official CCW short start is now **implemented and ready for physical
validation**, based on CW commit `f56b610`. Its model checks pass across all ten
official layouts and an empty diagnostic. Physical reliability is not yet
accepted. The original investigation below explains the faults being replaced.

## Implemented behaviour

- `O` / OFFICIAL with `OBSTACLE_PARKING_CCW_SHORT_START_ENABLED=true` retains
  the second-edge ToF reference but removes its extra 70 mm reverse.
- The 55 mm scan starts directly from the settled measured reference, with
  no additional centred reverse to a nominal X. The earlier proposed X=130
  reset was deliberately not implemented: it would add a small new movement.
- Only front station 2 is an initial prerequisite. No middle scout/retrace.
  Two stored-seat bits preserve any confirmed back/left and back/middle signs;
  their geometry is injected on normal return within 800 mm, separately for
  both colours. Unknown behind signs are checked by normal lap discovery.
- The bounded short connector search/shapes and outgoing lookahead are shared
  with CW for RED/GREEN/clear, with existing capsule, steering, travel and
  handoff gates. CW-named connector constants are retained and labelled shared.
- Physical pink/body checks run before the entire scan including 8 mm braking
  and +/-1 degree, and during scan/connector. CCW keeps the body ahead of X=0;
  CW keeps its existing X<500 guard. Clear/reset removes stored bits too.
- `[CCW START]` logs identify the shorter reference, skipped behind scout,
  stored seat/colour and later activation. `O3` keeps the legacy 70 mm reverse,
  55 mm scan and middle scout. No firmware upload or agent commit.

### Verification of implemented code

`check_ccw_start_planner.py` extracts actual production scan, clearance,
connector and lookahead functions. **2970/2970** scan/connector cases pass;
front occupied cases also witness correct-side full-body passage in ideal
normal-route continuation. Five archived corrected reference poses translated
70 mm east are used, with +/-5 mm XY, +/-1 degree reference heading and
separate +/-5 mm XY / +/-3 degree settled-scan deviations.

**33/33** ideal returned-section calculations pass across eleven layouts and
speed caps unrestricted/140/260 mm/s, activating each behind bypass at 800 mm.
**135/135** sampled shorter localization-prefix cases avoid pink/body-middle
conflicts using archived exit/reference poses, +/-5 mm XY / +/-1 degree and
8 mm overrun. The unchanged unparking segments were not physically retested.
`check_cw_start_state.py` verifies actual two-place store/release/prerequisite
functions plus **2400** C++/Python polygon comparisons; CW checker still passes
**7319/7319**. New generated reports live under
`local_workspace/ccw-start-planner/` and `local_workspace/cw-start-state/`.

Known colours and ideal tracking replace perception/motor dynamics in these
checks. The new ToF transition timing/corrections are estimated, not measured.
Images, full first-lap discovery/lap-boundary behaviour and physical braking
still require logs from cable-free trials. In the one-lap diagnostic, the stop
at phase zero can occur before a middle sign's complete final body crossing;
the returned-path calculation continues beyond that boundary to check its
correct side. Do not mistake model return checks for a completed robot run.

IDE-managed M7-only build passes: RAM 432752/523624 bytes,
flash 473568/786432 bytes. Firmware binary SHA-256:
`84e4f200df84061a8ff7e780498ac15919535da30639e672c5edd0611de5eac6`.
Existing `Serial` macro redefinition warnings remain. No M4 changes/build.

In the southern start section, CCW travels east. Inner sign coordinates are
X=-500 (back/left), X=0 (back/middle), X=500 (front/right), all Y=-900 mm.
Only the right place lies ahead after unparking. The two behind places may be
recorded initially; their bypass must be activated before the later return.
Official layouts comprise six single signs and four end pairs. A middle sign
excludes the ends. Empty is an additional diagnostic case.

Rules: `WRO_2026_RULES.md`, January 15, 2026 PDF, 9.19 / Appendix A.5 and
figure 8e. Official Q&A checked October 6; no general correct-side exemption.

## Reproduction and evidence

Run from the repository root, with a host C++ compiler on PATH:

```powershell
$pythonExe = Join-Path $env:USERPROFILE '.platformio\penv\Scripts\python.exe'
& $pythonExe simulation/review_ccw_start.py
& $pythonExe simulation/study_ccw_start_candidate.py
& $pythonExe simulation/check_ccw_start_planner.py
& $pythonExe simulation/check_cw_start_state.py
```

Actual production C++ planner functions are extracted through the existing CW
checker. Inputs are five archived primary-observation poses: complete CCW
388/389 in `simulation/evidence/parking_exit_diagnostics/` and 364/365/369 in
`simulation/fixtures/parking_entry_scout/`. Generated
`local_workspace/ccw-start-review/report.json` records paths and original SHA-256
hashes. These older runs are geometric inputs, not physical tests of HEAD.
Their camera images and physical acceptance are not established by this audit.

Colours are explicit fixture inputs. Each pose has 27 perturbations: +/-5 mm
XY and +/-3 degrees heading. Total 1485 cases; this is a sensitivity grid,
not a measured pose-error distribution. Outputs remain ignored working files.

## Previous production results (before CCW short-start implementation)

| Back / middle / front | Nominal connector | Grid connector | Further issue |
| --- | ---: | ---: | --- |
| Green / empty / empty | 5/5 | 135/135 | Later passing not evaluated |
| Red / empty / empty | 5/5 | 135/135 | Later passing not evaluated |
| Empty / green / empty | 5/5 | 135/135 | All 135 cross middle on wrong side |
| Empty / red / empty | 5/5 | 135/135 | Later route not evaluated |
| Empty / empty / green | 5/5 | 135/135 | Full front passing not evaluated |
| Empty / empty / red | 1/5 | 42/135 | No eligible nominal merge in four poses |
| Green / empty / green | 5/5 | 135/135 | Later passing not evaluated |
| Green / empty / red | 1/5 | 42/135 | Same front-red gate |
| Red / empty / green | 5/5 | 135/135 | Later passing not evaluated |
| Red / empty / red | 1/5 | 42/135 | Same front-red gate |
| Empty / empty / empty | 5/5 | 135/135 | Diagnostic only |

### Front red: merge gate

Current CCW requires at least 350 mm forward projection and a join 150–500 mm
before the sign. Eligible maxima for the five poses are 343.4, 345.4, 353.0,
347.4 and 348.2 mm. Four therefore have no candidate even with correct RED
supplied. Colour thresholds and faster reverse cannot fix this geometry.
Current GREEN-only outgoing-route lookahead is also asymmetric.

### Middle green: wrong-side passage

The 85 mm middle scout moves the body behind X=0 (sampled minimum X=-110 to
-120 mm). The connector then crosses fully eastward at rear-axle Y=-1173 to
-1187 mm, on the outer/south side of the inner green sign Y=-900: wrong for
CCW green. Collision-free does not mean rule-compliant. Current entry/scout
requires primary and preceding-middle resolution; rule-based empty inference
does not remove this prerequisite during the entry phases.

### Camera view

Old primary poses see the front sign at 7.8–11.9 degrees, 446–455 mm range.
The middle scout sees the middle at 19.0–24.3 degrees, 242–251 mm: near the
clear-view limits of +/-26.4 degrees and minimum range 230 mm. The back/left
sign is 83–87 degrees off-axis, outside that view. No HSV conclusion follows
from this geometry; initial observation of both behind places is unnecessary.

## Simpler candidate — calculation only

1. Stop localization reverse 70 mm earlier: remove its additional 70 mm and
   change arc-start X from 60 to 130 mm.
2. Retain the 55 mm observation arc, check the front place and omit the initial
   85 mm middle scout/retrace.
3. Store confirmed behind signs without initial route displacement. CCW needs
   **two** deferred places; copying the CW single-place state is insufficient.
4. Reuse bounded CW connector search/shapes and outgoing-route lookahead for
   both colours, retaining wall/pillar, steering, travel and handoff limits.
   Add a CCW body guard X>0; the CW X<500 guard cannot be reused directly.

The isolated host candidate approximates step 1 by translating old primary
poses 70 mm east. It obtains **1485/1485 connector successes**. A separate
polygon audit at 2 mm rollout steps, wheel angles +/-50 degrees and pink gaps
242.5/252.5 mm finds zero pink conflicts and zero body crossings behind X=0;
minimum body X is **16.3 mm**. Front view becomes 13.5–17.7 degrees, 390–398 mm.
Removing 70+2*85 mm saves about four seconds at commanded 60 mm/s, excluding
observation/stops. These are model results, not a runtime guarantee.

The original design-only calculation disabled the CW-only footprint guard and
used a separate CCW polygon audit. The replay script now uses the implemented
CCW guard. This translated-primary-pose study is historical; use the complete
scan check above to validate implemented geometry. Recognition and physical
uncertainty remain unverified.

## Next steps

After review and M7 upload, perform cable-free official CCW trials: front red
first, front green, middle red/green, back red/green and four end-colour pairs.
Start with one per layout, then repeat successful layouts for three runs.
Keep other sections fixed; require correct-side passing, no contacts/holds,
valid entry/scanning poses, measured braking and complete immutable logs.
Verify no initial crossing behind middle and both stored-place releases on
the later approach, including new detections during/after the connector.

Opposite-side surprise parking is recorded separately in
`SURPRISE_CHALLENGE_PREPARATION.md`.
