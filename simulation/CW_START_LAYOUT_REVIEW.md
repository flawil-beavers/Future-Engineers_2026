# CW starting-section review, 2026-10-06

## Rules and driving phase

The freshly downloaded official January15_2026 PDF matches the local source
SHA-256 recorded in `WRO_2026_RULES.md`. Official Q&A was rechecked. Figure8e
moves parking-section signs to the inner row. After this relocation, Figure8c
has ten distinct layouts: six single signs (three stations, two colours) and
four colour combinations at the two end stations. An empty section is an
additional diagnostic case, not an official card.

For CW, the right/near station is X=+500, the middle X=0 and the left/last
X=-500, all inner seats at Y=-900. The right station is initially behind the
parked vehicle. It should be observed/stored for its later encounter, without
creating an unnecessary initial crossing. Rule9.19 and AppendixA.5 still
apply if a manoeuvre moves the vehicle beyond the sign and subsequently
fully crosses its line in the official direction. There is no blanket
starting-section exception in the checked international PDF/Q&A.

The existing reverse edge localization ends around X=579 and its scan around
X=620. This moves beyond the right sign and makes the ensuing connector
cross that sign again. Simply suppressing RED avoidance would pass on the
wrong side. The previous assumption that a colour-dependent connector alone
should fix every start missed this unnecessary preliminary crossing.

## GREEN at the right start station

Archived 427/428 both confirm GREEN at seat0 and finish their first lap.
Their primary scan sees that sign at about -68/-68 degrees and 271/272 mm,
outside the +/-26.426-degree comfortable view. The 85 mm scout puts it at
+2.3/+2.6 degrees and 289/292 mm. Thus missing it in the primary scan is a
viewpoint issue; those runs do not show fundamentally failed green thresholds.
426 instead times out at the middle station and later confirms an erroneous
RED there, triggering a failed replan. No raw driving frame identifies that
object, so blaming the parking piece or changing HSV values is not justified.

New M7 records `[GREEN START CHECK]` for the three inner CW start seats:
phase, reason, bearing/range, green/red votes and candidate shape values.
Reasons distinguish outside view, absent silhouette, range mismatch, invalid
range, wrong seat snap and red conflict. At most 12 records per station,
36 total, no extra capture or motion. This exposes why a future missed GREEN
fails instead of inferring it from a large colour blob alone.

The official parking-section snap now rejects impossible outer-row positions,
without remapping them to the inner row. Both red and green use that snap;
CHECK_ALL and nonparking tests retain their existing position choices.
`simulation/check_parking_seat_snap.py` compiles the actual C++ snap function
and verifies all three inner seats, outer-seat rejection, other sections,
CHECK_ALL and nonparking operation.

## Historical start evidence before the short CW implementation

| CW start placement | Existing evidence | Remaining requirement |
|---|---|---|
| GREEN right/near | 427/428 confirmed correctly and completed | Check new diagnostic/servo build; remove needless initial crossing in revised start |
| RED right/near | 436/437 confirmed correctly, connector rejected | Revised start that keeps this sign behind initially; current start unresolved |
| GREEN middle | 446--448 physically passed without contact | Regression after the new shared servo/snap changes |
| RED middle | 439/440/442 completed | Regression; 441's parking-rail stall is separate |
| GREEN left/last | 455/456 completed connector then drifted left | Validate corrected servo resume and narrower path physically |
| RED left/last | 443--445 physically completed | Regression |
| Two end signs: GG, GR, RG, RR | No complete validation of all four variants | Each must retain/store the near sign and correctly handle the last sign and subsequent encounter |

Passing some singleton cases does not validate all ten layouts or normal
placement variation. No claim of complete CW reliability is made.

## Historical design study (superseded by the implementation below)

`simulation/review_cw_start_matrix.py` reproduces evidence/visibility and a
replacement design. From each archived prelocalization exit pose, an ideal
65 mm reverse arc produces a middle-seat scan; another 130 mm reverse arc
views the near/right seat. Retrace that 130 mm to the primary scan, then join
the route. Sampled pink checks and view gates pass for all 19 logged exit
poses, with the whole modeled body staying before X=500. An isolated scout
study at (300,-1220,151.1 degrees), with assumed +/-10 mm XY and +/-2-degree
heading, passes 27/27 samples. The two studies use different primary headings
and must not be conflated as one full sensitivity validation.

This replaces the long reverse localization; adding it after the existing
localization would preserve the wrong initial crossing. The old longitudinal
marker correction is therefore unavailable in this design. The initial
rear/side-ToF anchor and subsequent odometry need an explicit error budget
and complete swept verification. No firmware movement change is made from
these isolated geometry results.

Exact next implementation work:

1. Validate the complete exit-to-scan-to-scout-to-return motion, including
   localization accuracy, realistic error bounds, braking and both pink rails.
2. Build and replay the shorter connectors for clear/RED/GREEN middle stations,
   with the live planner's selection and preflight gates. A geometric scout
   pass does not establish connector convergence.
3. Store the near sign without initial route displacement; release its normal
   avoidance when approached later. Check map/lap boundary and all four pairs,
   including the full directional sign-crossing condition.
4. Build M7 and physically accept the ten layouts one case at a time with
   contact-free passage and complete logs. Current GREEN followup remains
   the first validation of the already-built servo correction.

The same tool also finds a nominal shorter connector for each of the ten
layouts from the candidate primary scan pose. It keeps the near sign as a
collision guard without its initial colour-dependent route displacement.
The 500 mm rollout, 42-degree steering and 60 mm/15-degree handoff limits
remain in these simulations. This establishes candidate geometry, not
firmware readiness: all colours are assumed known, the candidate search is
broader than the current four-attempt planner, and its shorter forward gate
and correct map/injection timing still need implementation and verification.

The latest M7 adds only rule-aware snap and bounded GREEN diagnostics to the
preceding servo/path candidate. Build passed RAM432736/523624,
flash468472/786432; binary SHA-256
`db17da3d1716c83733e2eeb423979abce20a166dba6c9d9516e54e0c8ce0c83a`.
No upload or commit. Generated matrix reports remain in
`local_workspace/cw-start-matrix/`.

## Implemented CW start candidate (2026-10-06, ready for review)

This section supersedes the earlier unimplemented-design status. No commit or
firmware upload was performed. All ten official parking-section layouts now
have a complete code path; physical acceptance remains outstanding.

- `O` / OFFICIAL, CW only: retain the initial rear/outer-wall ToF field seed
  plus exit odometry. Skip the long reverse search for the second pink edge.
  Do not reset the measured exit pose to a nominal point.
- Reverse 65 mm at the measured R109 full-lock radius to observe the middle.
  If needed, reverse another 130 mm to observe the right/near inner place,
  then retrace the measured reverse travel, including settled braking travel.
  A confirmed middle excludes the ends under the official layout rules.
- Right RED and right GREEN are both confirmed in the map, without initial
  route injection. Keep the full body before X=500 during observation/join.
  Inject that stored seat once the normal route approaches it within 800 mm.
  Optimized laps include every confirmed seat, including the stored one.
- The short connector searches at most ten merge points and five Hermite
  shapes. Its search gates permit 100 mm forward / 50 mm before a pillar;
  these gates are not safety acceptance. Every accepted shape still passes
  the 2 mm swept rollout, 500 mm travel, 42 degree steering, wall/pillar
  reserve and 60 mm / 15 degree handoff limits. Preflight and runtime both
  use outgoing-route lookahead. A late left-place detection either retains
  a newly checked unchanged join or replans from the actual current pose.
- Physical chassis, protrusions and wheel polygons additionally check both
  pink pieces, 242.5 and 252.5 mm gap endpoints, and the near-sign line.
  Initial arc preflight includes 5 mm body reserve, +/-1 degree heading and
  8 mm braking allowance; runtime rejects a measured footprint conflict.
  The assumed error budget is a model input, not a measured localization
  guarantee. Larger return offsets were exploratory stress cases and are
  not accepted by this candidate's stated budget.
- Existing CCW and `O3` / CHECK_ALL localization/scan settings remain selected
  through direction/mode checks. Servo writes are explicitly restored after
  scout stops and validated connector updates.

### Reproducible checks

Run with the IDE-managed Python and a host C++ compiler on PATH:

1. `simulation/check_cw_start_planner.py`: **7319/7319 pass**, extracting actual
   production baseline/displacement, scan, clearance, connector and late-
   detection functions. Ten official layouts plus empty diagnostic; 19
   archived exit poses; 27 initial-pose perturbations; +/-5 mm XY and +/-1
   degree scout-return perturbations; 1170 late left-place retain/replan
   checks. Colours are supplied explicitly, so this is not image validation.
2. `simulation/check_cw_start_state.py`: actual stored-seat functions release
   RED and GREEN once on later approach, never on the initial connector;
   1200 C++/Python polygon checks agree, including negative collision cases.
3. Existing actual-C++ connector/servo negative control and rule-aware seat
   snap regressions pass. M7 build passes, RAM432744/523624,
   flash472480/786432. Binary SHA-256:
   `f585d0ee5c2d35cdf4106c09cfda4d53d79311ad56ad10c0384045c94941c130`.

Reports and extracted fixtures remain under ignored `local_workspace/`.
The original archived logs and their portable SHA metadata remain tracked.

### Exact next physical acceptance

After review/commit and M7 upload, use `O` / official mode and the existing
one-lap gate, without a driving USB cable. Keep other sections unchanged.
Test GREEN right, RED right, GREEN middle, RED middle, GREEN left, RED left,
then the four right/left pairs GG, GR, RG and RR. Start with one run per
layout; repeat successful cases for three runs before accepting them.
For right-place cases require a stored-seat log initially and activation on
later approach. For middle/left require the correct passing side, no pink
contact and no autonomous hold. Copy each complete original log into the
tracked evidence directory before evaluating. Confirm real scout return
error and braking travel against the assumed budget. These offline checks
prepare all cases; they do not establish physical reliability or colour
recognition in every room.
