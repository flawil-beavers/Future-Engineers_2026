# Returned CW518--520 and stationary camera investigation

2026-11-11 acceptance restriction: `20261026_green_new_pose_400mm_01` is
user-rejected cyclically wrapped image evidence; `green_final_400mm_01` was
also declined for use. The original historical investigation below is
retained, but neither capture may validate or calibrate current detection,
geometry or sign height. Consult
`simulation/evidence/camera_diagnostics/validation_status.json` and obtain
individual user image approval before new acceptance use.

User reports: CW1 exited, then stopped before RED in the start section; CW2
looked out of the bay and stopped; CW3 missed RED/GREEN and stopped. Mapping to
518/519/520 follows the substantive source-run order. Additional515--517 have
zero recorded travel, no diagnostic build/direction and are not substituted
for those three runs. All complete originals are archived in
`simulation/evidence/parking_exit_diagnostics/`; the README records hashes,
bytes, firmware and limitations. Filename CW for515--517 denotes batch context
only; their direction is unverified.

Installed build markers518--520: `Oct_26_2026_17_45_26`, turn=-1,
30/36/26/84mm exit targets. The binary was not read back.

## What the logs establish

518 has123 RED projection records, all `outside_snap`. The final stationary
view still has valid RED near imagex85/foot146, range371mm, projected field
(-575,-1033). Nearest geometric seat5 has error101mm but is the outer row in
S0, excluded by official parking-section inner-seat mode. Therefore the stop
is not evidence that the sensor never sees RED. Incorrect actual placement,
pose/bearing/range calibration and rule-layout selection must be separated;
do not remove the legal-seat gate or invent a new seat from these logs alone.

519 completes exit diagnostics, then scout reports `Discovery unresolved -
drive motor locked off`. This is a perception-resolution stop, not an
established camera hardware failure. Some RED blobs are visible but project
far from a legal seat. No full image of the scout viewpoint is available.

520 contains RED detections even though user describes it as unseen. Near the
corner GREEN becomes production-valid but is rejected before association:
t37732 range312.2mm, projection(-1025,-402), nearest7/error124; t38536 range
301.7mm, projection(-1032,-412), nearest7/error111. Perception hold starts at
S1station0, forward335mm, and expires. The new close-unmapped diagnostic reason
overwrites `hold_expired` before the later code tests that string to trigger
the existing checked corner-view recovery. It consequently suppresses recovery.
This is a concrete regression introduced in the previous session, independent
of whether camera calibration is also wrong.

## Minimal source correction

Keep the existing hold_start/hold_expired reason when the close-unmapped hold
also applies; only set unmapped_foreground_hold if no previous reason exists.
Zero speed, colour/range/ambiguity thresholds, clearance checks, route lengths
and the existing recovery manoeuvre remain unchanged. No extra driving gate.
Three source-extracted cases cover existing hold_start,hold_expired and a new
unmapped-only reason; the previous unconditional assignment fails the first
two. `check_batch_512_514_fixes.py`:6camera+15perception+2304exit+10settling pass.
Firmware build identity is recorded in AGENT_DOCUMENTATION.md once completed;
this source fix has not been uploaded during the camera comparison.

## Stationary camera comparison and observed image-order defect

User confirms USB connected and drive enable OFF. Existing stationary commands
c0,c400,c600,camshot,camseat only. Initial baseline firmware unchanged; subsequent
M7 uploads listed below. No driving command or hardware rewiring was issued.
Curated originals,CRC,byte counts,hashes and setup reports are in
`simulation/evidence/camera_diagnostics/README.md`. Work/replay outputs stay in
`local_workspace/camera-test-20261026/`. Current-room baseline was not curated
as field evidence or used for calibration.

The user noticed that the left part appeared appended to the right. Quantitative
cyclic image comparisons on rows85..225 confirm:

| Comparison | Cyclic X displacement | RGB mean absolute difference before/after |
| --- | --- | --- |
| Initial RED01 vs02 |112px|41.1251 /4.2733|
| New-pose GREEN01 vs02 |64px|35.9192 /2.9654|
| Read-only sync-diagnostic GREEN01 vs02 |16px|13.9393 /2.1040|

All source payloads pass CRC and byte count. This establishes integrity of the
transmitted bytes,not correct spatial ordering before export. Initial RED01/02
alignment history is uncertain. After the user's physical reset,three RED
pictures are stablex151/foot134. The later new-pose GREEN sequence explicitly
follows user moving/rotating the robot; it is not a same-pose colour pair.

Read-only phase diagnostic records DMA remainder at the existing VSYNC callback.
With the16px shifted image, remainder38392 vs38400words per frame is8words;
RGB565 has2pixels per32-bit word,matching16px. After the stopped export's
existing stream restart,remainder38400 and picture ordering agree. This is
strong evidence of frame/buffer phase involvement. It does not establish why
phase was lost,whether electrical glitches occurred,or universal correctness of
an IRQ-time sample under all interrupt loads. A permanently wrong connector or
camera hardware defect has not been demonstrated.

ST's [DCMI synchronization description](https://www.st.com/content/st_com/en/technical-documents/an5020-introduction-to-digital-camera-interface-dcmi-for-stm32-mcus-stmicroelectronics_asset/chunk-004-topic-an5020-dcmi-description.html)
and [capture modes](https://www.st.com/content/st_com/en/technical-documents/an5020-introduction-to-digital-camera-interface-dcmi-for-stm32-mcus-stmicroelectronics_asset/chunk-005-topic-an5020-capture-modes.html)
are the primary reference for hardware frame/line synchronization. Do not infer
a missing interrupt merely from project code: HAL initialization already enables
VSYNC/error interrupts,as the [vendor HAL implementation](https://raw.githubusercontent.com/STMicroelectronics/stm32h7xx-hal-driver/master/Src/stm32h7xx_hal_dcmi.c)
shows. No extra interrupt source was enabled by the diagnostic patch.

## Delivered camera synchronization change

Cache VSYNC count,DMA remainder,min/max,current target,status,error bits and
consecutive good/bad phase observations. Publish a buffer only after two clean
frame boundaries. Three consecutive nonmatching phase observations request a
normal end-of-frame stop. The main loop polls completion while servicing the
rest of the robot; only after CAPTURE is clear does it finalize/restart DMA.
No image rotation/coordinate patch,no vehicle halt,no extra motion,gyro reset
or change to continuous/snapshot selection. Single transient observations do
not trigger a restart. CameraSystem recovery count/last fault/max service survive stopped exports;
per-stream VSYNC/DMA phase statistics reset when capture restarts.
Existing camera exports add optional metadata while retaining v1 payload/CRC.

This is recovery in image acquisition,not a new route guard. Continuous camera
operation must still be accepted under real driving load. A phase observation
can be influenced by interrupt latency; the consecutive-frame rule limits
transients but does not prove that false restarts are impossible. Source tests
model actual register/state paths. Initial autosync bench pictures stayed clean
without a restart. Final calibrated firmware then recorded actual live recovery:
capture header count1,last bad remainder38392,clean phase38400,max main-call
service7us. Subsequent stationary telemetry records count2,last38395,clean38400,
max service8us. Frame intervals remain76.49ms with zero reported capture errors.
This demonstrates two live stationary recoveries,not acceptance under driving
load. The final picture includes the operator; exposure settles1080->720->360,
so it is not a controlled exposure-only comparison.

## GREEN recognition and measured range calibration

| Reported geometry | Measured foot | Original source ray estimate | Revised source ray estimate |
| --- | --- | --- | --- |
| RED400mm,stable after reset |134px|429mm|410mm|
| GREEN400mm,paired first scene |132px|445mm|420mm|
| RED600mm,paired scene |108px|801mm|601mm|
| GREEN600mm,same scene |108px|801mm|601mm|
| GREEN400mm,new robot view,aligned02 |142px|375mm|374mm|

Distances are the user's reported lens-to-front-face references; they were not
independently surveyed. Five representative conditions,not every repeated frame,
fit footY=horizon+scale/depth. Final52px/33600mm*px replaces78px/24000mm*px.
It is a fit to400/600mm observations in two robot views,not a held-out accuracy
proof or a calibration for an arbitrary camera mount. Maximum error on those
references is26.328mm vs200.698mm previously. Current source replay with a40mm
reference-error budget and the failing legacy negative control is reproducible
using `simulation/review_camera_capture_20261026.py`.

GREEN primary shape varies with room background. Across56new frames:53/56valid
at first400mm,40/56at600mm. At the later new view/cold-start sequence,only5/113
primary-valid frames are counted; large raw widths95..117px exceed the unchanged
shape gate. Existing local dark-silhouette/colour fallback detects10/10fresh
frames in the first400mm scene and10/10in the later view,including10/10rejected
primary frames. It still needs the correct geometric seat projection. No HSV,
shape-limit or blanket CLEAR loosening was applied in this session.

## Firmware and verification

- Minimal recovery-reason-only build: M7 SHA-256
  `1f6cf395feb8d40fc9c9122513c66568ba97f7a109fbfd57757140835bce580a`;
  built,not separately uploaded.
- Read-only VSYNC/DMA diagnostic: M7 SHA-256
  `c755856649b925430ee707ce0608760a03c887eebfbf8ca059da5cf0671ecb25`;
  successful IDE-managed M7 DFU upload; two syncdiag images.
- Automatic resync with original distance model: M7 SHA-256
  `0ef8961d33b0489200e3d2f7159c6513efaf8f8e0ca669010c7b88cba51354f7`;
  successful M7 upload; autosync bench series and local GREEN check.
- Final calibrated build: M7 SHA-256
  `53a7bedc5e7b65849c267c568231a645db5a153d4e62b526793dcb1be1f77328`;
  RAM433920/523624,flash511184/786432. IDE-managed M7 build and DFU upload
  both SUCCESS; final stationary GREEN candidates10/10,primary8/10,
  ROI6.666..6.723ms,estimated373.4..373.5mm at reported400mm. Complete original
  images/telemetry and hashes are archived in camera_diagnostics/README.md. M4 consumers/protocol unchanged,not built/uploaded.

`check_camera_frame_sync.py`:17production-source state/register cases pass,
including withheld partial boundaries,delayed stop without blocking,one restart,
IRQ-mask preservation and transient recovery without restart.
`test-camera-image.py`:5payload/CRC/header/legacy compatibility checks pass.
`check_batch_512_514_fixes.py`:6camera+15perception+2304exit+10settling pass;
archived/synthetic GREEN/RED visibility checks pass. These remain bounded tests.
Prior full-route checks are documented in the previous batch; unchanged route
geometry is not presented as a newly driven physical success here.

## Next normal-run acceptance

Final M7 is installed and the stationary comparison completed. Perform normal CW and
CCW complete runs and save normally. Observe early GREEN/RED association,
absence of unnecessary holds,exit wheel/pink clearance and final parking.
Archive complete originals with physical observations; inspect camera recovery
counters when available. If phase recurrence or false resync occurs under
load,review the acquisition timing before adding navigation constraints. No
new dedicated ToF sweep/extra test movements. Full race reliability and actual
on-track recovery remain open until normal-run evidence exists.
