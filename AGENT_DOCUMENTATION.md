# Agent documentation and engineering handoffs

Current handoff consolidated on 2026-11-17. Read this first, then the linked
report and relevant evidence before continuing an investigation. Mandatory
workspace instructions are in [AGENTS.md](AGENTS.md).

## History and evidence

The [history index](docs/history/README.md) leads to the preserved engineering
history and superseded test procedures. Historical instructions describe their
own builds; they are not the active test sequence. The archive retains exact
original text, firmware hashes, physical reports and rejected approaches.

- [Original logs and metadata](simulation/evidence/parking_exit_diagnostics/README.md).
- [Camera evidence](simulation/evidence/camera_diagnostics/README.md) and
  [image approval registry](simulation/evidence/camera_diagnostics/validation_status.json).
- [Current test checklist](OBSTACLE_CHALLENGE_TEST_PLAN.md).
- [Parking diagnostics plan](PARKING_POSE_IMPLEMENTATION_PLAN.md).
- [Official rules reference](WRO_2026_RULES.md).

## Current firmware and validation status

Source checked on 2026-11-17. Normal `O` runs exit parking, drive three laps
and attempt final parking. `OBSTACLE_FIRST_LAP_TEST_ENABLED`,
`OBSTACLE_THREE_LAP_TEST_ENABLED` and `OBSTACLE_FINAL_PARKING_TEST_ONLY` are false.
Surprise Rule participation defaults to enabled through
`OBSTACLE_SURPRISE_RULE_ENABLED=1`; it must match the pre-attempt declaration.

Before the November19 correction below, the latest recorded firmware change was the checked CCW first-lap view correction
from commit `484a23e`. Later commits `284b8a2` and `8383d9b` add visualization
tools and documentation. Logged control revisions are
`20261111_checked_ccw_view`, `20261111_recovery_handoff` and
`20261111_height_rule` for path, final parking and Surprise Rule respectively.

The latest recorded IDE-managed M7 build succeeded with RAM 484,896 / 523,624
bytes and flash 597,064 / 786,432 bytes. Its 600,616-byte binary has SHA-256
`fb49725aadef397cbcce99110e5239eb5fd919883c496da24b275de948ba3765`.
This is a historical build identity, not a fresh build or an installed-firmware
readback. M4 was unchanged. The latest archived runs predate these November 11
changes, so they do not establish physical acceptance of current firmware.

### First-lap perception

The [checked CCW view report](simulation/LAP1_CHECKED_CCW_VIEW_20261111.md)
records the bounded extra target rotation after the existing protected bypass.
CW behavior and the protected bypass remain unchanged. Model checks show an
earlier visibility window in the targeted CCW case, not a real detection rate.

The [retry and parking recheck](simulation/LAP1_PARKING_RECHECK_20261111.md)
documents braking and re-observation after bounded view drift or rejected
view paths. New motion still needs full preflight. Invalid pose, substantial
deviation and genuine sensor or motion faults remain stop conditions.

### Surprise Rule and camera approval

The [implementation report](simulation/SURPRISE_RULE_IMPLEMENTATION_20261111.md)
describes height recognition and the stored original passing colour. Approved November19 stationary50/100/150mm references now pass current-source replay; normal dynamic runs remain unvalidated.
The approved `20261027_green_400mm_01.png` is a normal-height reference only.
The rejected or invalid October 26 references cannot support new acceptance.
Consult the approval registry and obtain the required image approval before
using additional reference images. Synthetic tests do not replace this check.

### Final parking and exit analysis

The latest [parking recheck](simulation/LAP1_PARKING_RECHECK_20261111.md)
documents a remaining planner failure at a 232.5 mm gap within the accepted
measurement range. The current approach matrix passes 96 of 120 model cases;
all 24 aborts occur at that gap. Reported 248.9 mm and 249.6 mm gaps pass their
separate modeled cases. Universal parking reliability is not established.

The [measured-envelope report](simulation/RETURNED_651_656_PARKING_20261105.md)
records physical dimensions and outcomes. Runs 652 and 653 have user-reported
interruption and slippery-floor exclusions respectively. Preserve those reports.
The handoff planning latency remains a separate physical validation concern.

Parking-exit diagnostics remain passive analysis of normal runs. Retain servo
centre 80 and existing motion, braking, reversal and pose-correction diagnostics.
Encoder/ToF disagreement alone does not identify backlash or absolute pose
accuracy. Follow the diagnostics plan before continuing that investigation.

## Exact next steps for robot validation

1. Use the current checklist for normal CW and CCW runs with unchanged hardware
   and normal saving. Do not revive archived batch-specific test instructions.
2. Secure complete original logs first, verify hashes and immediately release
   the USB stick. Record firmware identity and physical outcome in the evidence
   index before analysis, following AGENTS.md.
3. Compare fresh first-lap frames, votes and recovery with the linked reports.
   Check that extra CCW aiming stays outside the protected bypass.
4. Check measured parking registration, first-command progress, final physical
   containment, parallel stopping and actual clearance. Retain the narrow-gap
   planner limitation until separately resolved and validated.
5. For Surprise Rule acceptance, review approved real height evidence and the
   stored seat identity, visible/original colour and passing side on each lap.
   Keep unvalidated height detection explicit.

## Video documentation

Use [the reproduction guide](simulation/VIDEO_VISUALIZATION.md). The latest
run 633 map export is
`local_workspace/video-documentation/sensor-usage/run-633-drone/map-only/run_633_map_video_synced_large_text_1x.mp4`.
It is 1080 square at 30 fps, lasts 196.9 seconds and uses 42-pixel labels.
Align its start to DJI0066 at 4:39.788. The earlier log-timed export ends early
relative to the footage; use the video-synchronized export in Kdenlive.

The calibrated clock maps one logged second to about 1.03447 filmed seconds.
Logged poses are anchors; footage fills gaps. The filmed top green pillar is
drawn at seat 14, with the original assignment retained in the manifest.
Displayed ToF distances intersect modeled surfaces; untimed closing rear
references have estimated display windows. These are reconstruction assumptions.
Original footage, transform caches and the one-time stabilization utility remain
local inputs. Full revision history is in the archive.

## 2026-11-17: documentation consolidation

Archived the complete prior handoff, obstacle checklist, connector guide and
rules reference under docs/history/. Replaced competing current/next procedures
with one source-checked checklist, consolidated rule metadata and kept active
limitations visible. No firmware, original evidence or physical test changed.
Next maintain concise findings here and follow the archive links for historical
investigations instead of copying old procedures into the current handoff.


## 2026-11-19 measured-height implementation (current)

Latest changes correct the mounted-camera ground/height model using the user-approved November19 stationary series. All48 original image bundles are secured;45 approved,3 excluded. Evidence, metadata and hashes are in simulation/evidence/camera_diagnostics/. All192 original files were CRC/payload/hash/index verified. The previous incorrect classification and exact capture history are preserved in docs/history/HEIGHT_CAPTURE_INTAKE_20261119.md and simulation/CAMERA_HEIGHT_CAPTURE_REVIEW_20261119.md.

Current-source replay now recognizes all45 approved images with correct visible colour and50/100/150mm class. Background-connected stems are narrowed for pillar association; original components remain available for occupancy/debugging. HSV thresholds, route geometry, speeds and parking control are unchanged. rememberModifiedSign() stores one seat/height, keeps visibleRed separate from effective original passing colour seat.red, and later laps retain that map. Rule participation remains enabled by default and must match the pre-run declaration. Current Surprise log revision20261119_measured_height.

Four origin/main commits through caa2709 were downloaded and fast-forwarded (video tools/history consolidation); only AGENT_DOCUMENTATION.md conflicted when restoring local work. Preserved the remote concise handoff/history layout and archived the complete local November19 intake tail, with this updated current section.

Additional stick originals637..644 were secured unchanged in simulation/evidence/parking_exit_diagnostics/ with README byte counts and SHA256; directionCW, installed firmware and physical outcomes unestablished. Earlier25-file intake and failed empty capture attempts remain documented. User told stick may be unplugged immediately after source/copy verification. No Surprise physical-run acceptance inferred from these older logs.

Validation/build identity and exact next steps: see simulation/SURPRISE_RULE_MEASURED_VALIDATION_20261119.md (updated at session completion). Only M7 is affected; do not rebuild/upload M4 for this change. No robot upload or motion performed this session.

Final validation:45/45 real frames,16 captured sequences,648 rendered/48 association/24 policy,52 far-acquisition,2807 later-lap,6 handoff and11 camera unit cases pass; current/legacy foot-gate and green occupancy checks pass. IDE M7 SUCCESS RAM484896/523624 flash597432/786432, bin600984B SHA256059f6fb03f35efc3f11c10cfcb8c17e32dd08d22234a14d4dfacdcef51bfc92a. M4 unchanged, not built; no upload. Next normal M7 CW/CCW physical acceptance with one special sign, secure complete logs first and verify one stored identity/original side on all3laps.
