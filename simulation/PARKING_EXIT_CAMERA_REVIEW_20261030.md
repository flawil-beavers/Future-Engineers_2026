# Camera observations during the normal parking exit

User request: use the diagonal views already present during CW/CCW unparking,
retain the normal motion, and carry reliable observations into the first lap.
Hardware acceptance of this revision is pending normal powered runs.

## Position and timing

The initial rear/outer-wall measurements establish a provisional field pose
before the exit arcs. Two later XY corrections can move that reference:
`parking_end_y` and `edge_localization`. Assigning an early blob using the final
robot pose would confuse the motion between observations with a coordinate
correction.

The new M7 observer records raw red/green pillar blobs and a historical pose.
A 32-sample pose history interpolates the middle of the measured camera frame
interval, using the existing DMA completion timestamp. This is an exposure-time
approximation, not a new camera calibration. Every applied XY correction shifts
the existing observations and history by its actual delta. New frames thereafter
already use the corrected coordinate frame. No seat is assigned while exiting.

The buffer has 160 entries and occupies 10,904 bytes in the host layout. It
stores no images, allocates no heap memory, adds no camera capture and changes
no steering, speed, reversal, brake or exit target. Capture is active after the
initial pose seed and before the first path handover. Invalid/old frames,
repeated sequences, missing pose brackets, long frame intervals and excessive
within-frame movement are rejected. A sequence restart discards the old epoch.

## Evidence and handover

After the ordinary path and start-section seats exist, observations are mapped
using their corrected historical poses. Existing blob, range, mapped-foot and
row-ambiguity checks remain. Cached acquisition additionally requires:

- A start-section seat, 120--900 mm range, at most 28 degrees off axis;
  nominal seat/range residual at most 100 mm and bearing residual at most 6 degrees.
- The same seat under all 27 combinations of +/-20 mm XY and +/-3 degrees;
  the opposite row must remain at least 40 mm farther from the projected point.
- Three distinct frames spanning at least 120 ms, gaps at most 400 ms,
  consistent colour, and projected-position spread from the burst anchor at
  most 60 mm. Evidence older than 20 seconds is ignored.
- No conflicting colour/row at that station and no contradictory official
  middle/end placement in the batch. Pending samples never become live votes.

Confirmed observations enter the existing `finalizeConfirmedSeat` path. Thus
CW behind-seat and CCW behind-seat storage, later release, normal route
injection and first-lap prerequisites remain owned by the existing code.
The ordinary short scan geometry remains; confirmed places are already known
when its result/scout logic runs. This change does not promise to skip the
whole scan movement. Insufficient cached evidence leaves the normal scan and
retry behavior in charge. No cached absence is recorded as CLEAR.
The last image before the exit-to-path handover is not reused as a live vote
with the end pose. The next genuinely fresh frame resumes normal processing.
This also prevents a duplicate vote from the same image at that transition.

## Evidence and verification

No new robot runs were available. Complete originals 574--593 were copied
first and SHA-256 verified again, then matched against their already tracked
portable originals and README hashes in `evidence/parking_exit_diagnostics/`.
User was told the stick could be unplugged before analysis. Representative
files/hashes remain the 577 CCW and 593 CW entries in that evidence README.

`check_parking_exit_observer.py` executes the production buffer and production
association/import functions: 630 checks passed, including both
directions, three start stations, red/green, 350/500/700 mm, off-axis views,
accumulated corrections, conflicts, insufficient evidence, stale/reset frames,
time/heading wrap, bounded storage and immutable live pose. Map/route dispatch
is stubbed in this test; the separate production planner/state tests cover it.
Another 12 checks execute the production handover branch, covering both turns,
one-time import, rejection of the handover image and normal fresh-frame resume.

The same test projects ideal synthetic pillar blobs from archived corrected
exit poses. For CCW front, CW middle and CW front, 9 of 10 direction-specific
logs each contain at least three geometrically eligible sampled poses; the
remaining logs are interrupted starts. CW rear has no such window. These are
possible views, not measured colour detections or proof against occlusion.
For 577 CCW the front window is in exit segments 4/5; for 593 CW the middle
window spans 3/4/5 and the front window is in segment 5. The observer runs
throughout the exit, so it does not assume the useful angle occurs after
exactly one or two steering changes.

Regressions passed: 8,943 CW and 4,158 CCW scan/connector/planned passing cases;
33 CCW returns and 135 localization-prefix cases; stored-seat lifecycle and
2,400 footprint comparisons; 48 far-acquisition checks; connector servo-resume
positive/negative controls; archived 471/576 false-foot and 589 occupancy
controls; 42 parking segment-resume checks.

Final IDE-managed M7 build: SUCCESS, RAM 453688/523624 bytes (86.6%), flash
548728/786432 bytes (69.8%). Binary: 552268 bytes, SHA-256
`b61b493418be465eb449168d5c7e0cd21e400b858349e2741b4c078a575847d4`.
M4 and shared protocols are unaffected. No upload or robot motion was performed.

## Next normal runs

Upload M7 only. Perform normal CW/CCW runs and save normally; no extra sensor
sweep, pause or movement is required. Secure originals first. Check
`[EXIT VIEW] revision=20261030_corrected_pose`, saved/rejected counts and
confirmed seats against the physical arrangement, then verify scan-to-first-lap
transition and ordinary obstacle passing. Motion-time association, actual
occlusion, processing latency and colour reliability still need these runs.
