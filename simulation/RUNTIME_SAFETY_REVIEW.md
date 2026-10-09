# Runtime safety and Inspector review — 2026-10-09

## Authorized scope

User-selected items 1–4, 9, 11, 14, 15 and 17. No competition-profile removal,
route changes, larger logger, dedicated ToF sweep, upload or agent commit.
Previously modified Inspector files and history were retained.

## Implemented behavior and consequences

1. Enable switch: read once per loop. LOW accepts a falling edge immediately;
   HIGH must remain stable for the existing 100 ms debounce. A rejected edge is
   never stored as accepted. Initial HIGH startup and serial/manual controls stay
   available. Interrupt-disabled/blocking intervals remain a latency limitation.
2. Gyro: health requires a received valid quaternion within 200 ms. Timeout
   applies to HIGH, stuck LOW, missing and unrelated events. Unit-norm plausibility
   admits broad normalization tolerance and rejects zero/nonfinite/corrupt values.
   Long polling gaps defer transport recovery without renewing measurement age.
   Following the user correction, transport recovery retains the active mode,
   controller, route and continuous angle. The first valid restarted yaw seeds
   a new raw baseline without adding a changed quaternion zero to the angle.
   The existing motor pause remains; fresh valid reports automatically resume
   the same mode. Final-parking health failures hold rather than abort, and the
   parking state timeout excludes the gyro pause (even after log cap exhaustion).
   Rotation during the measurement gap is unobserved: continuity prevents an
   artificial jump but cannot reconstruct real unmeasured chassis rotation.
3. Serial: empty/whitespace messages do nothing. An overlong line is discarded
   completely through LF, never interpreted from its tail. Checked integers,
   finite floats, trailing-data checks and supported value limits precede actions.
   Valid legacy/PID/calibration/test commands remain. Seat indices are checked
   before narrowing to uint8_t. Negative gains and out-of-range commands fail
   explicitly rather than silently changing controllers.
4. Completed automatic parking enters the existing MODE_HOLD without reinitializing
   its verified hold target or resetting parking completion. Explicit stop/pause/
   mode changes release it normally. Failure cleanup of other modes is unchanged.
9. ToF scalar accessors return -1 for stale/bus-failed data, not 9999 (free gap).
   Valid fresh no-return frames still produce 9999. Age limit is max(250 ms,
   actual measurement budget +100 ms), supporting 300 ms long-range discovery.
   Raw/signal/sigma selections are replaced on empty frames, not carried forward.
   Diagnostic snapshots keep sequence/time for caller age checks and remain
   unavailable during transport failure. Reconfiguration invalidates old objects.
   Both side and rear slew filters restart from the first actual fresh value after
   a gap/error. Healthy-stream selection/limits remain. M4 read errors survive a
   successful measurement restart; RPC layout/version is unchanged.
11. Encoder ISR fields are volatile; snapshot reads preserve the prior interrupt
    mask and obtain count/direction together. Distance and diagnostic consumers
    use the snapshot API. Conversion scale and ISR direction logic are unchanged.
14. Invalid/null/too-short/oversized/odd camera frames clear prior detections and
    cannot overrun the fixed map. Valid supported even dimensions remain. Sparse
    brightness/dark/clipped ROI diagnostics reuse 200 already-read pixels at
    320x240, with no extra image pass. Values are printed only in the existing
    stationary CAM CAL record. HSV limits, silhouette/color votes, frame rate,
    gain and exposure are unchanged. White surroundings can produce clipped
    samples without a camera defect; these are measurements, not rejection gates.
15. Sub-threshold distance/yaw changes retain their integration baseline and
    accumulate. Nonfinite readings are rejected. Applying an XY correction does
    not automatically subtract its magnitude from heuristic uncertainty. This
    field is explicitly not an absolute accuracy guarantee or a covariance model.
17. Inspector 1.2 counts accepted scan/initial-wall corrections and breaks the
    affected final-parking track. Compact cs= identities in existing bounded
    LATER_TRACK/FINAL_PARK_TRACE records distinguish real corrections from cached
    values, including equal and rounded-to-zero new corrections. Legacy logs use
    value transitions and an explicit ambiguity warning. Rejected scans do not
    count as applied motion. No new periodic records or sensor acquisitions.

The cs field costs at most 14 bytes per existing record: 6,160 bytes at the
combined 200+240 limits (usually less). Existing buffer size and all prior
unparking/braking/reversal diagnostics are preserved. Camera diagnostics add no
normal-driving log output. Hardware timing of the small extra computation has
not been measured on the robot. A conservative projection adding all ten-digit
cs IDs to existing archived matching records gives a largest non-overflow log
of 165,959 bytes (507), leaving 30,649 bytes within the 192 KiB buffer. This is
an estimate for observed logs, not a proof for arbitrarily verbose future runs.

## Verification

- IDE-managed incremental PlatformIO builds: M7 and M4 SUCCESS. Normal cache and
  build directories used; sandbox cache write restriction required approved
  execution outside the sandbox. No clean build requested/performed.
- M7: RAM 432,912/523,624 (+32 bytes from preceding candidate), flash
  504,464/786,432; firmware SHA-256
  491c0f44e9f84b3a32a4e21fdd886a1d3b2f5aaf6e58548a48c892d46d77aadb.
- M4: RAM 59,768/294,248, flash 155,224/1,048,576; firmware SHA-256
  d85930eef24188e1f28e61cfa3edef32d1f090292d89e9efcb42e29d2e79a21e.
- Twelve source-backed host regression groups in check_runtime_safety.py cover
  switch bounce/short-off/timer wrap; strict inputs; actual gyro failure/recovery;
  actual serial framing; mode completion; tiny-step odometry; invalid camera
  frames/quality values; reset state retention/yaw continuity; encoder snapshots; ToF accessors;
  M4 status/recovery; side-ToF empty/error/stale/slew behavior; and seat parsing.
  The first group combines switch, integer, quaternion and freshness checks.
- Actual Vision: four archived images plus eight synthetic visibility/background
  cases PASS. Actual path/state: 2,807 cases PASS. Final approach: 2,970 PASS.
  Entry: 162 cases, 156 accepted and six existing conservative off-nominal rejects;
  nominal cases pass. Five registration checks and 396 actual-module final-parking
  sequences PASS. These geometry tests include ideal sensor/motion adapters.
- Inspector: 21 parser checks, all 125 complete archived logs parse/render and
  DOM-adapter import/search/notes/export tests PASS. Tests run with the available
  Node runtime VM; no physical browser visual acceptance is claimed.
- Generated test/build outputs are under local_workspace/runtime-safety/ and
  existing ignored simulation working directories. git diff --check PASS.

## Physical acceptance still required

No new robot logs or measured physical outcome. Firmware binary hashes identify
local builds, not installed images. Gyro wiring/power faults, actual servo lag,
braking/slip, optics, reflections and pose drift before the parking scan remain
unresolved. Camera diagnostics cannot adjust physical focus/mounting. Pose
uncertainty remains heuristic. USB/logger blocking itself was outside this scope.

Next: upload both changed cores; perform the existing normal cable-free B CW
and C CCW three-lap-plus-parking validation, normal saving only. Confirm physical
clearance and final stop/hold, archive each original immediately with identity
and physical report per AGENTS.md. If a gyro reset occurs, preserve the log and
report whether automatic continuation succeeded and whether motion deviated. No dedicated ToF sweep or extra
movements/pauses. Inspect camera quality in existing calibration only if needed;
keep optical diagnosis separate from changing runtime color thresholds.

## Gyro continuation correction (same day)

User rejected the permanent active-mode cancellation. It has been removed.
M7-only IDE incremental build SUCCESS: RAM432912/523624, flash504456/786432;
SHA-256 `8d10fcddcc46ea615ad6820c436c66da1291d57acafccf85f7375858b8b63b14`. M4 is unchanged by this correction.
Thirteen runtime host groups pass, including restarted yaw offsets, repeated
recovery, state retention on failed transport initialization, parking pause
timeout accounting, exhausted log cap and timer wrap. Geometry regression now
injects a 1500ms gyro pause at first entry drive in each accepted sequence.
Short motor pauses remain from the pre-existing health gate; no uninterrupted
driving with failed gyro is claimed. Unmeasured rotation during braking or a
real power fault can still invalidate heading; physical recovery is unverified.

Full updated parking simulation PASS: five registration checks and396/396
contained sequences with injected1500ms first-entry gyro pause, zero failures.
