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

The latest recorded firmware change is the checked CCW first-lap view correction
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
describes height recognition and the stored original passing colour. Real
50 mm and 150 mm pillars have not been accepted with the mounted camera.
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
