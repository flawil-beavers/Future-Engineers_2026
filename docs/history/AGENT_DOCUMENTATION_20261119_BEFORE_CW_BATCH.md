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

## 2026-11-19: user-reported CW special-sign test layout

Reference for the next physical runs, in CW section order from the parking straight:

| Section | User-reported visible arrangement | Effective passing colour |
| --- | --- | --- |
| Parking/start | GREEN at positions1 and3; row not repeated explicitly | GREEN / GREEN |
| Next straight | RED middle, inner row | RED |
| Opposite/start+2 | Tall GREEN middle, outer row;150mm from current test series | RED (one modified sign, pass right) |
| Last straight | RED position1 inner, GREEN position3 outer | RED / GREEN |

Positions1/2/3 retain the user's longitudinal labels; no new seat-ID mapping or measured pose inferred from this description. This is physical ground truth for subsequent log comparison only, not a hard-coded firmware map. Expect the tall visible GREEN to retain its effective RED/right passage on all3laps after three fresh matched height observations. Other GREEN signs pass left, RED right. First encountered sign protection remains based on actual encounter order; a parked start may have a sign behind it. Firmware remains74f3d7d; no build/upload, motion or further commit. Next secure fresh complete logs first and compare recognition/seat/height/visible-original identity against this arrangement.

## 2026-11-19: two reported runs not saved to observed stick

User reports red blinking only when saving after two new runs. Immediately secured highest-number and newest-mtime originals with source/copySHA256, then released stick. All656 source logs unchanged versus prior evidence/working archive; no20261119_measured_height marker. No new run originals available; highest655/656 are already archived November5 CCW runs, not these new CW tests. Portable check manifest/source hashes/limitations: simulation/evidence/log_intake/20261119_save_check/. Robot installed firmware unverified. No code change, repair/format, reset, upload or motion. Logger source provides steadyRED while saving and GREEN success; no repeatedRED blink pattern. Await user confirmation of exact pattern and USBconnected/driveOFF/no-reset before reading serial passively; preserve possibleRAM log. Do not call this a confirmed defective stick or claim either new run saved.

Save-check follow-up: user confirms four-short/four-longRED beginning only on save, USBconnected/driveOFF/no reset. Official Arduino GIGA support identifies that pattern as MbedOS crash (link in portable save-check README). Passive enumeration finds zero Arduino ports; no serial port opened, command sent, reset or readback. New-run data absent on stick; exact crash/root cause and installed version unknown. Do not diagnose defective stick, reformat it or change unrelated recognition based on this alone. Next capture error with a working serial session before a subsequent normal stationary save or use crash/debug context; any reset can discard the unsaved RAM log.

## 2026-11-19: stick-versus-robot storage check

User confirms original16GB FAT32 stick was intentionally emptied. No new logs to secure. Windows read-only CHKDSK reports no filesystem errors;15.7GB free.32x256KiB random-file flush/close/reopen/SHA256 passes,8MiB total, zero errors; own checked temporary file/directory removed. Portable results: simulation/evidence/log_intake/20261119_save_check/stick_write_read_check.json and README. No robot reset, motion, upload or command; no repair/format. Earlier RAM-access request stopped by user: no serial or DFU device/debug probe available, no RAM read.
User blink report establishes Mbed crash during save per official Arduino support, but exact crash/root cause/installed build unknown. PC check points away from simple stick/filesystem failure; robot USB host, power/compatibility or software remain candidates. Limited heap and dynamic USB-host construction visible in current source/ELF are suspects only, not measuredOOM. Next capture actual crash output/runtime heap around stationary USB connect/mount/write, with user-confirmed driveOFF and working serial; do not change vision, routes or reformat media based on this alone. No additional commit authorized/performed.

## 2026-11-19: USB save heap mitigation and phase diagnostics, not physically validated

Exact crash cause remains unknown (no current-run log/error code or serial device). PC stick tests pass. Existing ELF leaves38352B total heap span; USB host is instantiated on first connect (3644B object+3072B threadstack+other dynamic/FAT costs). Main32KiB threadstack is already .bss, not double-counted. Risk mitigation: pack unchanged65536-entry vision LUT to4bits, frees32768B;192KiB log capacity and HSV/height/routes/speeds unchanged. Add terminal-only USBLogger phase/heap diagnostics while stopped; preserve save verification/recovery/RAM retention.
Tests:131072 exhaustive LUT comparisons,45 approved image frames/16 captured sequences/648 rendered/48 association/24 rule cases,52 far acquisition, green background/occupancy negative controls and546-file logger/short-write/directory-failure/phase-order checks all pass. Report simulation/USB_SAVE_HEAP_REVIEW_20261119.md. Host tests mock USB and do not prove physical save.
IDE M7 SUCCESS: RAM452136/523624 flash598048/786432, bin601600B SHA25669afd090f0d0347d930125a1b52f9c2734b339a9f7997ef720822f9ffd9d8882; heap span71112B (net+32760B). M4 unchanged/not built. No robot upload/reset/movement, no additional commit. Next user normal M7 upload, driveOFF and serial115200 already open before normal stationary save; capture last [LOGGER USB] stage and raw heap_used/peak/reserved/fails plus any crash text. Then secure complete new source logs first. Do not claim out-of-memory or resolved hardware save until runtime evidence.

## 2026-11-19: stationary new-heap USB save observed, media check pending

After user corrected failed upload and reported ready, COM7 available. First DTRfalse passive capture empty; framework source confirms terminal-connected requiresDTR, 1200bps alone arms bootloader touch. At115200/DTRtrue sent p only (pause/disable/save); raw serial shows all USB phases through finalize_verify, zero allocation failures. Used heap35616 before connect,42432 at mount, peak44484B, reserved71112B; old total38352B cannot fit observed requirement. Strong causal memory-shortage evidence for earlier crash, not old stack/readback. Read-only t query after save returned POS, proving application still responds. No drive, camera, reset or agent upload. Original captures/hashes/phase values: simulation/evidence/log_intake/20261119_save_check/stationary_save_observation.json. Await user moving stick to laptop; secure original file FIRST+hash and release stick, then verify payload. Until then media durability unconfirmed.

## 2026-11-19: new stationary USB save physically confirmed

Source log_1.txt930B now exists on previously emptied stick. Copied immediately unchanged, source-before/source-after/copySHA256 verified; user released stick. Portable original simulation/evidence/log_intake/20261119_stationary_usb_save/20261119_log_1.txt, SHA2567dd527741778dcf321e7238faad9fc04f05d23858fbcebd35059ba58e370a91b, manifest/README adjacent. Contains startup/camera initialized/driveOFF and final SYSTEM DISABLED matching p snapshot; no full run took place. Live serial traversed finalize_verify and later t returned POS. New save is confirmed; this does not recover the two earlier missing runs.
Observed heap42432B at mount/44484B peak exceeds old total38352B; strong support for previous allocation failure. New71112B heap reports zero failures. No old crashstack or installed-flash hash readback. Source report/buildidentity remains simulation/USB_SAVE_HEAP_REVIEW_20261119.md; new serial/evidence matches that prepared M7 version. Next normal CW/CCW run and full-size save; secure new original FIRST. No further firmware edits, build, upload, motion or commit this test.

## 2026-11-19: full-size CCW log2 saved; parking approach reserve and handover

Secured newest complete source FIRST, SHA256 source-before/after/copy agree; user immediately released stick. Portable original simulation/evidence/parking_exit_diagnostics/20261119_log_2_ccw.txt138372B SHA256ca40714175cfadcdad327c866291551c2e70f34470409c31170d484be7499a05, indexed README. Repeated930B stationarylog1 is explicitly not a new run. Source revisions measured_height20261119/recovery_handoff20261111; prior prepared RAM-mitigated M7 identity69afd090f0d0347d930125a1b52f9c2734b339a9f7997ef720822f9ffd9d8882, no flashreadback. Real full-size save now confirmed, no truncation.
User confirms CCW light touch at far startGREEN lap2, otherwise good; parking alignment too near wall/slow/abrupt. All3laps97.469s. Initial connector rejects unexecuted candidates then passes; special150mm visibleGREEN14 locks effectiveRED. Parking planning gyro gap1230ms induces hold; extra alignment then reverse scan. User mode_pause ends file before entry, not autonomous park abort. Estimated lap2 startGREEN minimum clearance102.7mm contradicts physical contact: pose/time/model discrepancy unresolved. Twenty slip diagnostics are not independent ground truth.
Changes onlyM7: scan/capture10mm farther inward (20mm tip reserve), free approach150/alignment130mm/s with60mm/s terminal blend includingXgate, gyro serviced during swept approach planning, CCW confirmed startGREEN-pair later-lap offset+15mm. Same parked target/entryspeed, sensor/recognition gates intact. Report simulation/PARKING_APPROACH_LOG2_REVIEW_20261119.md. Tests laterlaps2807, approach2970, registration5, completequick3, measuredentry24 and new recorded-handoff acquisition3 all pass. Latest speed-gate helper validated again before finish; final build identity recorded below. Next normal user M7 upload and CW/CCW physical validation; immediately preserve sources, check startGREEN and wall/marker alignment. No upload/motion/additional commit.
Final M7 IDE build SUCCESS: RAM452136/523624, flash598272/786432, firmware.bin601824B SHA256e306cfe314ca9d0181fb13a585897d36d722fa741a50d68ce8484412e4e5da5a. M4 unaffected/not built. Existing Serial macro/unused helper warnings remain; no new compile failure. Final speed helper includes longitudinal gate; quick full-state3 and measured acquisition3 rerun pass. Evidence SHA256 rechecked unchanged.
