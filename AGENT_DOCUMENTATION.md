# Agent documentation and engineering handoffs

This is the repository's durable handoff log for coding agents. It records
verified project state, important decisions, test evidence, operational
constraints, and the next concrete action. It complements `AGENTS.md`, which
  contains mandatory workspace instructions. Focused test plans contain only
  the current ordered tests and their go/no-go criteria; engineering history,
  calibration, rationale, and completed-test evidence belong here.

## How to maintain this file

- Add a dated entry for a substantial investigation or development session.
- Put the newest entry first.
- Separate measured facts from hypotheses and proposed changes.
- Include source files, log names, build results, and reproducible next steps.
- Ask the user when an unresolved question could materially affect safety,
  architecture, hardware placement, calibration, the validity of a test, or
  how a change should be implemented. If the intended behaviour, design choice,
  or tradeoff is unclear, ask before implementing it.
- If work can safely continue with a reasonable assumption, state the
  assumption to the user and record it in the relevant dated entry. Never
  present an assumption as a measured or verified fact.
- Preserve still-relevant earlier entries; consolidate them only when their
  conclusions have been superseded and state what replaced them.
- Never store credentials, personal access tokens, or large raw logs here.
- Keep routine successful-run documentation brief: normally two or three
  sentences stating the test, outcome, and next step. Add detailed analysis
  only when a failure, code change, new measured limit, safety issue, or
  reusable engineering conclusion materially changes the project state.
- Keep `OBSTACLE_CHALLENGE_TEST_PLAN.md` as an ordered checkbox list, not a log
  narrative. Put durable reasoning and important evidence here, and leave raw
  measurements in the USB logs or focused technical documents.

---

## 2026-08-27 - Pure-pursuit integrated into main and reviewed

Local `main` now contains the exact source tree from `pure-pursuit` commit
`57612568c64348654329ed6e366275f3e6af47aa` via merge commit `dcc3d70`; the
`pure-pursuit` branch itself was not changed. The IDE-managed `giga_r1_m7`
build passed with 361224 bytes RAM and 371488 bytes flash. Static review found
no new compile or control-flow blocker, but the parking exit intentionally
remains in test-only mode: reverse edge localization is not physically tested,
the start-section discovery connector and final parking are still missing, and
the final robot-length correction remains unset. Do not disable the test-only
lockout before those items are resolved. The Python swept-path model could not
be rerun on this workstation because no Python launcher is installed; its
latest checked-in handoff reports all 16 scenarios passing. Next, physically
validate the reverse localization without pillars in both directions, then
implement and test the direction-specific low-speed discovery connector.

---

## 2026-08-27 - Reverse parking-edge localization prepared

`D:\log_134.txt` physically passed the mirrored CW forward-edge-search run
without contact. The five exit segments aligned within 1.9 degrees and the
pink-end reference was usable. Edge search transitioned after 29.2 mm, but its
newest accepted wall sample was 219 mm against a 236.9 mm prediction. The
17.9 mm residual passed the current gate, although it still looks like a
transitional return rather than a settled black-wall measurement. This
reinforces the user's proposal to cross the other magenta edge instead of
continuing forward.

The isolated localization movement now reverses straight at 60 mm/s for no
more than 60 mm after the unchanged five-segment exit. CCW uses the fixed
piece's opposite `x=480` edge and the ToF footprint maximum; CW uses the moving
prototype piece's opposite `x=232.5` edge and the footprint minimum. The
existing two-frame wall consistency gate, newest-sample correction, 25 mm
correction bounds, and test-only motor lock remain active. The offline swept
model now includes this reverse and passes all 16 existing
gap/placement/heading tolerance scenarios.

The IDE-managed `giga_r1_m7` build passed with 361224 bytes RAM and 371496
bytes flash; existing warnings are unchanged.

This short reverse improves parking-edge localization and leaves somewhat more
approach distance, but it does not by itself make every legal starting-section
inner seat visible. The nearest relevant seat can remain over 40 degrees off
the camera axis. Do not enable the lap immediately after the reverse. First
physically validate the reverse in both directions with no pillars; then add a
direction-specific low-speed discovery connector which resolves the upcoming
inner-row seat before choosing the red/green side. The other sections already
use the normal full six-seat discovery logic. No firmware was uploaded.

---

## 2026-08-27 - Corrected CCW edge search passes gates; start-section sign needs discovery

`D:\log_133.txt` completed the corrected 60 mm parking-edge search in CCW
without a reported contact and reached the test-only motor lock. It preserved
the last genuine 67 mm magenta return, rejected intermediate returns, and
accepted two pose-consistent wall frames after 29.0 mm. The result applied
`x=-5.9 mm` and `y=-21.1 mm`, producing pose
`(497.0,-1225.0,358.2 degrees)`. The accepted 239 mm wall return was only
21.2 mm from the 260.2 mm prediction and therefore passed the present 25 mm
gate, but the stationary raw return later became 259 mm. Treat the y
correction as technically gated but not yet a precise wall measurement; before
the pose feeds the lap, retain the newest confirmed wall sample rather than
saving the first qualifying frame. This was implemented without changing the
two-frame confirmation count, residual gate, motion, or correction bounds.

The rules do not remove signs from the parking section. After placing the
parking lot, every sign in that section is shifted to the seat nearer the inner
wall. With the current canonical CCW geometry, the parking-end station's legal
inner seat is approximately `(500,-900)`, while log 133 ended around
`(497,-1225)` facing east. A sign there is about 325 mm laterally left of the
robot and outside the forward camera view. The normal lap connector must not
assume that the starting section is clear or commit to a red/green passing side
before this station is resolved.

Straight reversing while retaining the forward camera could eventually make
that seat visible, but it needs roughly 0.5--0.65 m to bring the whole pillar
inside the currently trusted/full camera angle. The present post-exit body
pose has only a small modeled margin past the open ends of the magenta pieces;
an outer-wall ToF measurement does not prove rear-corner clearance from those
ends. Do not add or physically test that reverse until its complete swept body
envelope is checked. Compare it against a cautious outer-side discovery
connector that turns enough to view the inner seat while preserving a route
that remains recoverable for either pillar colour. Ask the user before choosing
between these materially different manoeuvres.

Exact next powered validation remains the corrected edge search in the mirrored
CW direction, with no pillars and no lap join. No firmware was uploaded during
this investigation. The IDE-managed `giga_r1_m7` build passed with 361224
bytes RAM and 371464 bytes flash; existing warnings are unchanged.

---

## 2026-08-27 - Edge localization rejects transitional returns correctly

`log_131.txt` (CW) and `log_132.txt` (CCW) both completed the added creep and
test lock, with no contact reported. They did not validate black-wall
localization. CW accepted a 195 mm transitional return after 19.5 mm, although
the accepted pose predicted roughly 239 mm to the wall; it applied an
unreliable -15.3 mm x correction and correctly rejected a -43.5 mm y
correction. CCW accepted 210 mm after 27.5 mm while the pose predicted roughly
268 mm; its +3.4 mm x correction was applied, and the -57.9 mm y correction
was rejected. The existing 25 mm y bound therefore prevented bad wall fixes,
but the fixed 190 mm threshold and classification of every <=180 mm sample as
magenta were insufficient.

The next firmware keeps only ranges up to 50 mm beyond the initially accepted
magenta-end range as marker observations. Longer intermediate/oblique returns
cannot move the marker edge. A wall candidate must be in 190..400 mm and agree
within 25 mm with the central-ray wall distance predicted from current pose;
two fresh frames remain mandatory. X is now calculated at the last genuine
magenta pose rather than the midpoint to a premature wall observation. The
creep hard limit is 60 mm because at a 240-270 mm wall distance the 22-degree
cone is roughly 47-52 mm wide and can continue overlapping a 20 mm pink piece
after its centre has passed it.

The IDE-managed `giga_r1_m7` build passed with 361224 bytes RAM and 371464
bytes flash; existing warnings are unchanged. No firmware was uploaded. Next,
repeat CW only with the same ruler placement and no pillars. Require two
pose-consistent wall frames, both bounded corrections, no contact, and a final
test lock before mirroring or joining Pure Pursuit.

---

## 2026-08-27 - Ruler-assisted active parking-edge localization implemented

The user will retain ruler placement for the current prototype, so departure
safety still uses the validated nominal 50 mm rear clearance. Firmware no
longer assumes the nominal lateral coordinate: once the near ToF identifies
the outer-wall side, `initializeParkingFieldPose()` calculates rear-axle y as
canonical outer-wall y plus measured range plus the 35 mm sensor offset. The
chosen range and `start_y_source` are logged.

After the unchanged five-segment exit aligns and accepts its magenta-end
reference, a new isolated state-machine phase centres steering, settles for
200 ms, and creeps straight at 60 mm/s. It watches fresh ToF frame sequence
numbers rather than counting the same sample repeatedly. The last <=180 mm
magenta sample and first two raw wall samples in `190..400 mm` bracket the
transition; motion stops at confirmation or a hard 30 mm travel limit. The
known fixed edge is x=500 for CCW and the known moving-piece outer edge is
x=212.5 for the current CW prototype. The cone edge at the transition produces
an x correction, while the wall range produces y. Each correction is applied
only if its magnitude is <=25 mm. Logs are `[PARK LOCALIZE]`,
`[PARK LOCALIZE RESULT]`, and `[PARK LOCALIZE POSE]`.

This is deliberately still guarded by `OBSTACLE_PARKING_EXIT_TEST_ONLY=true`:
it cannot start the lap after the new movement. The IDE-managed `giga_r1_m7`
build passed with 361216 bytes RAM and 370944 bytes flash; existing warnings
are unchanged. No firmware was uploaded. Next, perform one same-placement
isolated test in one direction. Require no contact, a short transition before
30 mm, corrections within their bounds, and final motor lock. Review that log
before running the mirrored direction or implementing a Pure Pursuit join.

---

## 2026-08-27 - Cone-gated parking pose passes; start-x remains unobservable

`log_129.txt` passed the CCW firmware criteria: final heading error was 1.8
degrees, the right-ToF footprint `x=496.3..518.4` intersected the fixed
`480..500` piece, and the 58 mm return applied a +7.6 mm field-y correction.
The resulting pose was `(470.2,-1205.8,358.2)`. `log_130.txt` passed the CW
criteria: final error 0.2 degrees, left-ToF footprint `198.4..216.3`
intersected the variable `212.5..232.5` piece, and the 47 mm return applied a
+4.8 mm y correction. Its pose was `(247.1,-1217.8,180.1)`. Both logged
`beam_over_piece=yes usable=yes apply_y=yes`. The user confirmed that both
specific runs completed without physical contact. The cone-gated post-exit
field-y reference is therefore accepted in both directions.

The parked pose is not fully observable from the current stationary sensors.
The near side ToF identifies direction and can calculate rear-axle `y` from
the known outer wall, but neither side-facing ToF measures longitudinal `x`
between the two magenta pieces. The camera faces the forward piece, but its
aperture is at the current foremost coordinate and the nominal face is only
about 32.5 mm away, outside the existing pillar-distance calibration and
requiring a dedicated close-range magenta test.

A direct sweep of the tracked footprint model shows why software localization
alone cannot permit arbitrary placement with the current fixed manoeuvre. The
five-segment route passes all 16 existing gap/pose/heading tolerance cases at
nominal rear clearances of 45 and 50 mm. It passes only 8/16 at 40 mm, 12/16 at
55 mm, and progressively fewer outside that region. Therefore the simplest
recommended approach is: put visible marks on the robot for a broad accepted
45-50 mm rear-clearance band (no ruler needed); initialize `y` from the outer
wall ToF; execute the validated exit; then determine `x` from the known
magenta-piece edge as the side-ToF footprint leaves it, and refresh `y` from
the outer-wall return. Gyro supplies heading, with consecutive wall ranges
available as an optional heading-consistency check. This avoids a general
particle filter and does not require exact operator measurement, while still
respecting the route's collision envelope.

Other viable choices are a dedicated close-range magenta camera observation
before moving, front/rear ranging hardware for arbitrary placement, or a small
post-exit map-matching search over possible start-x hypotheses. Camera and
map-matching still need a safe initial placement band unless the unpark route
itself becomes adaptive. Contact/stall homing against a magenta piece is not a
safe competition strategy. Await the user's choice before implementing the
localization architecture.

---

## 2026-08-27 - Parking-end ToF beam gate ready for physical validation

`log_127.txt` and `log_128.txt` completed the CW and CCW exit state machines
with final heading errors of 1.5 and 0.1 degrees. The user confirmed that both
exits made no contact with either magenta piece or the wall. Their strong,
short ToF returns
were intentionally rejected by the new centre-ray gate. CW reported the left
sensor at `x=201.8` against the variable piece at `212.5..232.5`; CCW reported
the right sensor at `x=507.5` against the fixed piece at `480..500`. Both
therefore logged `beam_over_piece=no usable=no apply_y=no`; no erroneous pose
correction was applied.

This does not show that the returns came from another object. The VL53L4CX has
a finite optical field of view, while the implemented gate treated it as an
infinitely thin centre ray. At the logged 57 and 68 mm ranges, the piece edges
were only 10.7 and 7.5 mm from the respective centre rays, consistent with an
edge return inside the sensor cone plus the measured placement tolerance. The
implementation now transforms both horizontal edges of the documented
22-degree near-range detection cone into field coordinates and requires that
footprint to intersect the expected 20 mm piece. It does not merely increase a
fixed centre-ray tolerance. Replaying the two end poses predicts CW footprint
`x=192..214` intersecting `212.5..232.5`, and CCW footprint `x=495..521`
intersecting `480..500`. Logs now include both the centre endpoint as
`beam_x_mm` and the interval as `beam_footprint_x_mm`. The IDE-managed
`giga_r1_m7` build passed with 361168 bytes RAM and 368856 bytes flash;
existing warnings are unchanged and no firmware was uploaded. Repeat both
isolated directions before enabling a Pure Pursuit connector.

The visible stop between forward segments 4 and 5 is intentional in the
current validated manoeuvre. Every segment uses a 300 ms encoder-position hold
followed by a 400 ms steering-settle period. Segments 4 and 5 are both forward,
but steering changes directly from full lock away from the wall to full lock
toward it; moving during that servo sweep would create a path not covered by
the swept-footprint validation. This pause may be optimized later only after a
continuous steering-transition path is modeled and physically validated.

The rules-aware field-pose correction requires more than a short ToF range and
an aligned robot. For CCW the right-sensor footprint must intersect the fixed
`x=480..500` piece; for CW the left-sensor footprint must intersect the
variable `x=212.5..232.5` piece. Both intervals allow the user's measured
+/-5 mm placement tolerance. `usable=yes` and `apply_y=yes` are impossible
unless this geometry gate passes. The exit path and test-only lockout are
unchanged. No firmware was uploaded. Next, after user-controlled upload,
repeat the isolated exit in both directions. A
CCW run should start at `(322.5,-1362.5,0)` and report a beam over
`480..500`; a CW run should start at `(390.0,-1362.5,180)` and report a beam
over `212.5..232.5`. Require no contact, final heading error <=2 degrees,
`beam_over_piece=yes usable=yes apply_y=yes`, and a physically plausible
`[PARK EXIT FIELD POSE]`. Do not join the lap until both pass.

---

## 2026-08-27 - Prototype footprint photos and staged multi-point unpark

The user supplied three square-ish top-down photographs in the gitignored
`local_workspace/`: `PXL_20260827_113122609.MP.jpg` shows straight steering,
and the other two show manually pushed near-left and near-right lock. They are
useful temporary engineering evidence because they show that the robot does
not occupy the corners of its 165-by-135 mm bounding box and that the front
wheel envelope caused the failed forward-first exit. They are not final
calibration records: the steering was not driven to exact commanded lock, the
rear-axle midpoint is not marked, and the chassis may still change. Keep the
originals in `local_workspace`; after the mechanics are final, retake matched
overhead images at commanded `-50`, `0`, and `+50` with a fixed scale, rear
axle mark, and unambiguous robot-left/right labels. Only selected annotated
final images should later be committed under the project's documentation.

The refined swept-footprint search models the current chassis, wheel regions,
Ackermann front-wheel angles, 109 mm full-lock rear-axle radius, the minimum
242.5 mm marker gap, approximately +/-5 mm placement tolerance, and about
5 mm additional modeled clearance. A robust mirrored candidate starts with
45 mm between the robot's rearmost point and the rear marker inside face, with
the robot's inner edge at the parking lot's 200 mm open boundary. Its control
segments are: reverse/toward-wall 20 mm; forward/away 25 mm;
reverse/toward-wall 20 mm; forward/away 75 mm; forward/toward-wall 60 mm;
forward/straight 25 mm; forward/toward-wall 80 mm. Steering lock is 50 and
the low test speed is 80 mm/s. Mirroring means inverting every nonzero steering
command. The result passed the modeled minimum/maximum gap and +/-1 degree
initial-heading cases, but the residual physical margin is small and the
manually positioned photos do not justify executing it all at once.

The obsolete forward-first two-arc firmware has therefore been replaced by a
small data-driven multi-point state machine. It settles the servo before each
segment, uses a brief encoder-position hold instead of allowing an uncontrolled
coast at a direction change, logs each segment's target/actual travel and
heading, and mirrors itself from the nearer-wall ToF result. The development
gate `OBSTACLE_PARKING_EXIT_TEST_SEGMENT_LIMIT` is currently `1`, so the next
firmware runs only the initial 20 mm reverse/toward-wall segment, releases and
locks off the drive, saves the log, and cannot execute the remaining route or
start the lap. The IDE-managed build passed at 361160 bytes RAM and 366776
bytes flash; existing warnings are unchanged. No firmware was uploaded.

Exact next test: use the current 165 mm prototype and set the parking-marker
inside-face gap to 247.5 mm. Place the robot straight with 45 mm from its
rearmost point to the rear marker's inside face, leaving nominally 37.5 mm from
its foremost point to the front marker. Place the side of the straight robot
at the 200 mm open boundary, as far from the black wall as possible while fully
inside the parking lot. Repeat the same direction as the failed front-right
wheel test. After user-controlled upload, start the isolated obstacle mode and
keep the disable switch reachable. It must reverse only about 20 mm, turn the
nose outward, stop, and save. Reject contact or intervention. Before enabling
segment 2, compare logged `target_mm=20.0` with `actual_mm`; unexpected travel
or an observed clearance below roughly 15 mm requires stopping-distance or
model correction.

`log_117.txt` accepts this first staged test. It identified the right wall at
108 mm, selected away steering `-50`, then executed reverse steering `+50`.
The 20.0 mm target stopped at 22.3 mm with a 10.4-degree heading change; the
right ToF changed from 108 to 119 mm, and the user observed no contact. The
2.3 mm excess is acceptable for advancing one stage, but it must remain in the
physical-path evidence rather than being assumed away.

The exact offline program is now tracked as
`CAD/parking_exit_swept_search.py`, with its coordinate system, multi-polygon
footprint, Ackermann primitives, separating-axis collision checks, Hybrid-A*
search, tolerances, selected path, limitations, and physical-validation
workflow documented in `CAD/PARKING_EXIT_PATH_SIMULATION.md`. The original
working copy was moved out of `local_workspace`; the three approximate-lock
photographs remain there as temporary source evidence.

Next, change only `OBSTACLE_PARKING_EXIT_TEST_SEGMENT_LIMIT` from 1 to 2 and
build without uploading. After separate upload consent, repeat the identical
247.5 mm gap and initial pose. It should reproduce segment 1, then drive
forward 25 mm at full lock away from the wall and stop. Require no contact,
positive observed clearance, and plausible actual travel/heading for both
segments before enabling the second reverse movement.

The user subsequently enabled the route incrementally but did not test every
possible segment-count increase separately. `log_120.txt` ran the current
`7/7` configuration and physically exited the parking lot without touching a
marker or wall. Actual segment travels were 24.0, 28.6, 20.5, 78.1, 59.5,
25.0, and 80.2 mm. The clearance geometry is therefore accepted provisionally
for this start direction and prototype placement.

Do not connect this result to the lap yet. Final heading error was 7.7 degrees;
the last 80.2 mm toward-wall arc changed heading by only about 37.4 degrees,
leaving the robot visibly outside but not parallel to its start heading. The
next code change should retain the proven 80 mm final arc as a minimum, then
continue the same curvature until gyro heading error is at most 2 degrees,
with a conservative maximum around 110 mm to prevent an unbounded turn. After
one same-direction isolated regression passes, run the fully mirrored exit.
Only then proceed to post-unpark field localization. The logged `travel_mm=226`
is net encoder displacement because the manoeuvre reverses; the sum of segment
travel magnitudes is approximately 316 mm.

The user requested simplifying the route because the robot is sufficiently
outside after segment 4 to align in one continuous fifth movement. The swept
model previously applied its general footprint margin around the 200 mm open
ends of the magenta pieces, effectively making them longer. That is now
corrected: the black wall and broad marker faces retain 5 mm clearance, while
the marker open ends remain at exactly 200 mm. Placement is checked explicitly
at +/-5 mm instead of being represented by extra marker length.

The revised route retains the proven first four controls and replaces former
segments 5-7 with one forward full-lock arc toward the wall. Its geometric
parallel distance is 140 mm. All 16 combinations of 242.5/252.5 mm gap,
+/-5 mm x/y placement, and +/-1 degree heading pass collision checking. The
firmware now has five segments. The last segment ignores fixed-distance
completion: after 120 mm it stops when gyro error is at most 2 degrees, with a
hard 180 mm bound. The route and physical footprints are drawn in
`CAD/parking_exit_path.svg` and embedded in
`CAD/PARKING_EXIT_PATH_SIMULATION.md`.

Next, build without uploading. After explicit upload consent, repeat the same
247.5 mm-gap isolated test and require five segment reports, final
`aligned=yes`, no contact/intervention, and final heading error at most 2
degrees. Do not connect to the lap before this same-direction test and the
mirrored-direction test pass.

The IDE-managed `giga_r1_m7` build passed with 361160 bytes RAM and 366992
bytes flash; existing warnings are unchanged. The generated SVG parses as
valid XML. No firmware was uploaded. The next action is now the user-controlled
upload and same-direction isolated five-segment test described above.

`log_122.txt` accepts that five-segment same-direction exit physically. The
first four actual travels were 21.8, 28.2, 22.0, and 79.5 mm. The continuous
fifth arc started from about 75.4 degrees, reached `aligned=yes` after 162.6 mm,
and settled at 0.9 degrees final heading error. The robot cleared both pieces
and the wall without contact or intervention. The final right ToF was 69 mm,
compared with 109 mm at the parked start.

The user observed that the robot should begin slightly farther forward than
the middle to give its first reverse move more room. The nominal placement is
now shifted 5 mm forward: 50 mm from the robot's rearmost point to the rear
magenta inside face and 32.5 mm from its foremost point to the front face in
the 247.5 mm gap. This is deliberately a small change because placement
accuracy itself is about +/-5 mm. The updated five-segment path still passes
all 16 modeled tolerance cases, and the SVG was regenerated from that pose.

The user's ToF observation is geometrically useful. At the aligned exit the
outer-wall-side sensor lies over the adjacent magenta piece and faces its exact
200 mm open end. Firmware now logs a diagnostic-only `[PARK EXIT TOF REF]`
record containing filtered/raw range, signal, sigma, and a heading-compensated
estimate of how far the rear axle is beyond that end. It also reports remaining
distance from that estimate to the 500 mm corridor centreline. The calculation
uses the measured sensor origin at local x=40 mm and lateral offset=35 mm. A
range above 180 mm or final heading error above 2 degrees is marked unusable.
No position reset is applied yet: the beam-to-piece assumption must first be
confirmed in both mirrored directions.

The IDE-managed build passed at 361160 bytes RAM and 367656 bytes flash; no
firmware was uploaded. Next, perform one same-direction isolated exit from the
new 50/32.5 mm placement. Require safe clearance, `aligned=yes`, and a usable
ToF reference whose inferred geometry agrees with the physical pose. If it
passes, repeat mirrored; only then apply this observation to the production
field pose and join the Pure Pursuit lap.

`log_123.txt` again completed the five-segment exit without a reported motion
failure. It triggered final alignment at 163.1 mm and 1.6 degrees, but travelled
about 2 mm farther while the wheels remained at full lock and settled at -2.3
degrees. The ToF evidence itself was high quality: right filtered/raw 77/77 mm,
16.00 Mcps signal, and 1.2 mm sigma. It implied the rear axle was 113.5 mm
beyond the parking-piece end and 186.5 mm short of the centreline. The
diagnostic correctly returned `usable=no` solely because stopped heading error
exceeded 2 degrees; it did not apply a pose correction.

The final-segment transition now commands steering zero directly before
calling `stop(true)`. This removes full-lock curvature while the encoder hold
absorbs the final settling motion; intermediate segment behaviour is unchanged.
The IDE-managed build passed with 361160 bytes RAM and 367672 bytes flash.
No firmware was uploaded. Repeat the same isolated 50/32.5 mm placement and
require stopped heading <=2 degrees plus `[PARK EXIT TOF REF] ... usable=yes`.

`log_125.txt` accepts the steering-centering correction and the same-direction
ToF reference. The fifth segment triggered at 164.8 mm/1.9 degrees and settled
after only another 0.3 mm at -0.4 degrees, eliminating the full-lock drift from
`log_123`. The right ToF returned filtered/raw 63/63 mm, 18.02 Mcps signal, and
1.2 mm sigma. It estimated the rear axle 98.3 mm beyond the exact parking end
and 201.7 mm short of the centreline, reported `usable=yes`, and still applied
no localization correction. No firmware change or upload is required next.

Next, mirror the isolated test physically: point the robot in the opposite
official driving direction, keep the 247.5 mm gap, use 50 mm behind the robot
and 32.5 mm in front relative to that new direction, and keep its inner side at
the 200 mm opening. The same firmware should identify the left wall, invert
every steering command, finish within 2 degrees, and report a usable left-ToF
parking-end reference. Require no contact or intervention before allowing the
reference to influence production field localization.

`log_126.txt` accepts the fully mirrored five-segment exit and left-ToF
reference. It correctly identified the left wall, inverted every steering
command, aligned after 143.6 mm, and settled at 0.9 degrees final error. The
left ToF returned filtered/raw 44/44 mm, 17.24 Mcps signal, and 1.1 mm sigma;
it inferred 79.6 mm beyond the parking end and 220.4 mm remaining to the
centreline with `usable=yes`. No contact or intervention was reported. The
shorter final arc than the right-wall case is consistent with the measured
left/right full-lock asymmetry; gyro termination handled it correctly.

Both directions now validate the side-ToF method for the coordinate normal to
the parking-piece end. The right-wall exit estimated 201.7 mm to centreline and
the mirrored left-wall exit 220.4 mm, so production joining should use the
measured value rather than one fixed post-exit distance. Do not yet infer the
coordinate along the straight from the ToF range alone: the range fixes
distance normal to the 200 mm end face, while the beam merely constrains its
along-straight coordinate to the 20 mm magenta-piece width. Confirm the field
coordinate of that piece (including whether the parking gap is centred along
the 1000 mm straight) before applying a complete field pose.

The rules resolve that coordinate. Figures 3 and 4 do not centre the parking
gap: the right magenta limitation is fixed beside the right-hand dotted
boundary of the selected straight, and the left limitation moves to make
`1.5 * robot length`. Figure 8d randomizes which of the four sections is the
starting/parking section, not the gap's position within that section. Because
the field is rotationally symmetric, firmware continues to treat the selected
section as canonical south. The right dotted boundary is canonical x=+500; as
drawn, the 20 mm fixed piece occupies x=480..500, and the piece ends are at
y=-1300 relative to the outer wall at y=-1500.

The old `nominalFieldStartPose()` midpoint reset must not be used after the
parking exit. The isolated firmware now initializes the real parked rear-axle
pose when the nearer wall establishes direction. For the 247.5 mm gap, 50 mm
rear clearance, 40 mm rear overhang, and 125 mm straight wheel width, the
canonical starts are `(322.5,-1362.5,0)` for CCW/east and
`(390.0,-1362.5,180)` for CW/west. Encoder/gyro odometry then carries x and
heading through the exit. A usable parking-end ToF measurement corrects only y
to `-1300 + rear_beyond_end`; it cannot observe exact along-face x. New logs
are `[PARK FIELD START]`, `[PARK EXIT TOF REF] ... apply_y=yes`, and
`[PARK EXIT FIELD POSE]`.

This pose correction is currently safe to test because
`OBSTACLE_PARKING_EXIT_TEST_ONLY` remains true: it changes localization and
logging but cannot start the lap or alter the validated exit controls. The
IDE-managed build passed with 361168 bytes RAM and 368224 bytes flash; no
firmware was uploaded. Next, repeat one isolated exit in each direction and
validate the logged field poses. Only then replace the production midpoint
reset and construct a Pure Pursuit connector from the measured exit pose to the
correct point on the canonical lap route.

---

## 2026-08-27 - Parking-start scope and temporary-file convention

Current obstacle development supports only the higher-scoring legal start
from inside the parking lot. The alternative middle-zone start is deferred to
an optional end-stage bonus after the complete parking-lot-start sequence is
reliable in both directions. Do not generalize the parking-exit implementation
for both start classes during the current phase.

The robot's final length is still undecided. Because the rules define the
parking-space length as `1.5 * robot length`, do not freeze marker separation,
longitudinal start coordinates, exit distances, or final-parking geometry from
the present prototype dimensions. Keep those values parameterized and perform
the exact placement/exit characterization after the mechanical length is
finalized. Both official driving directions remain required: the front axle
must point toward the next corner in the chosen direction.

Repository-task temporary/generated working files belong under the gitignored
`local_workspace/` directory. `AGENTS.md` now records this as a mandatory
workspace convention; do not create a separate repository `tmp/` tree.

Exact next step while the length remains open: inspect the existing parking
exit for geometry that can be expressed relative to robot dimensions and
parking-marker observations, then implement an isolated, non-lap exit test
without baking in a final parking length. Ask the user before choosing any
length-dependent clearance or longitudinal placement.

That preparation is now implemented. `OBSTACLE_PARKING_EXIT_TEST_ONLY` is
enabled, so command `O` plus the physical enable switch executes only the
existing two-arc exit, ramps to a stop, locks the drive motor off, and writes
the log; it cannot silently continue into the obstacle lap. Configuration now
records the official 200 mm width and 1.5 length factor while leaving
`OBSTACLE_FINAL_ROBOT_LENGTH_MM` at zero. Startup therefore prints the length
and parking length as `UNSET` and labels the geometry `prototype_only=yes`.
No arbitrary chassis length or final marker separation was selected.

The result line now records total encoder travel, final heading error, and
left/right ToF at the start and end. The existing steering, first-arc,
counter-arc, speed, and brake calibrations are unchanged. They can be
characterized with the current prototype by physically setting the marker
inside-face separation to 1.5 times its measured current projection length,
but clearance acceptance must be repeated after the mechanical length is
final. For the calibrated lateral start, centre the robot longitudinally and
keep its inner side just inside the parking lot's 200 mm open boundary rather
than centring it across the 200 mm width.

The IDE-managed `giga_r1_m7` build passed with 361160 bytes RAM and 366200
bytes flash. Existing warnings are unchanged. No firmware was uploaded. Next,
obtain upload consent and run one prototype-only exit in a chosen direction;
require no marker/wall contact and retain the saved `[PARK EXIT RESULT]` line.
Then repeat in the opposite direction before making any trajectory change.

`log_115` rejects the existing forward-first exit in the official proportional
gap. With the current robot approximately 155 mm long, the inside-face marker
spacing was approximately 232.5 mm, leaving only 77.5 mm total longitudinal
slack, or about 38.75 mm at each end when centred. The robot started CCW with
right-side wall range 108 mm, travelled only 34 mm into the `-40` first arc,
reached 9.4 degrees, and touched the front magenta block. The commanded-motion
watchdog then correctly stopped it after measuring only 3.3 mm progress in one
second at a 120 mm/s target. The counter-arc and result logger were never
reached.

The local `main` branch does not contain a different initial unparking path.
Its `IDLE`, forward first arc, counter-arc, distances (239/250 mm), steering
(40), and speeds are the same as the rejected path. Its only extra parking
states occur after the S-shaped exit and reverse 200 mm for start-section
visibility; they cannot prevent the initial front-block collision. Therefore
the historical success on `main` came from its larger physical parking gap,
not from reusable close-gap logic. Do not merge or copy those post-exit states
as a fix for `log_115`.

No further powered attempt should use the forward-first path in a 1.5-times
gap. The next engineering step is an offline swept-envelope design for a
low-speed multi-point parallel-parking exit, parameterized by robot length,
front/rear overhang, width, marker size, and calibrated steering radii. Only
after both magenta blocks and the outer wall have positive modeled clearance
should the isolated firmware be changed and tested. The approximate 155 mm
length is prototype evidence, not a finalized mechanical constant.

The user subsequently measured 125 mm from the rear-axle centre to the
foremost point, 40 mm to the rearmost point, and 135 mm maximum width at full
steering lock; the steering envelope is slightly asymmetric. Those overhangs
sum to 165 mm, which conflicts with the earlier approximate 155 mm overall
length and must be resolved by one direct foremost-to-rearmost measurement
before geometry is implemented. The failed test used a 230 mm inside-face
marker gap and the contacted object was the forward magenta parking block. If
165 mm is correct, the official 1.5-times gap is 247.5 mm, not 230 or 232.5 mm.
Even centred, that supplies only 41.25 mm at each end and does not make the
239/250 mm forward-first arc safe.

Do not lengthen the front merely to obtain a larger official parking gap. An
added front length `d` increases a centred end gap by only `0.25*d` while the
front swept envelope itself grows by approximately `d`, normally worsening
the forward-block conflict. A short front addition may still be justified for
camera protection or packaging, but its purpose and final dimensions must be
decided mechanically before recalculating the unpark path. Preserve the
already validated camera yaw, roll, and pitch unless a specific visibility or
occlusion problem requires a change; moving or pitching it requires geometry
updates and camera recalibration.

The prototype geometry is now confirmed as 165 mm overall length, measured
from the same rear-axle reference used for the 125 mm front and 40 mm rear
overhangs. Use a conservative symmetric 135 mm full steering width for the
first swept-envelope model; the user rechecked the Ackermann asymmetry and
found both sides extend approximately equally. Robot placement and both
magenta-piece positions are repeatable to approximately +/-5 mm. The official
inside-face parking gap for this prototype is therefore 247.5 mm nominal and
must be checked over 242.5-252.5 mm in the model.

One start-pose detail remains intentionally unresolved: the user said the
robot will be placed at the "outermost position", which could mean closest to
the outer field wall or farthest outward toward the parking lot's 200 mm open
boundary. These yield materially different wall and marker swept clearances.
Ask which boundary the robot is placed against before selecting or testing a
parking-exit trajectory; do not infer it from the word "outermost".

The user confirmed the second interpretation: place the robot as far from the
outer black wall as possible while its complete projection remains inside the
parking lot. Model the nominal inner edge at the 200 mm open boundary and the
worst-case start 5 mm farther toward the black wall. Longitudinal position is
also selectable and should be optimized rather than forced to the centre.

An initial Hybrid-A* feasibility search was added only under the gitignored
`local_workspace/`. With the robot conservatively treated as a solid
165-by-135 mm rectangle, 5 mm inflated safety envelope, minimum 242.5 mm gap,
109 mm full-steering rear-axle radius, and the confirmed outer/open-boundary
start, it found no exit for rear clearances from 5 through 75 mm. Even a
nominal zero-margin 247.5 mm-gap search found no path. This is not yet proof
that the real robot cannot exit: the top photo shows that the camera nose,
chassis, and wheel regions do not fill the bounding-box corners, and those
empty corners matter in a close parallel-parking manoeuvre. Do not port a
trajectory to firmware from the rectangle search.

The next required geometry is a conservative top-down footprint outline, not
another overall width. Record the outer outline relative to the rear-axle
midpoint for straight, full-left, and full-right steering, or obtain a square
top-down photograph over graph paper with a scale. Also record which physical
part contacted the forward magenta block in `log_115`. Use that outline in the
swept collision model before deciding whether the current chassis can exit a
1.5-times gap or needs a mechanical change.

---

## 2026-08-27 - Loop/camera optimization integration gate

The former obstacle test plan accumulated completed log narratives, tuning
rationale, and calculations alongside its remaining actions. It has been
replaced by a compact ordered checklist that retains the meaningful completed
milestones and current pending gates. Durable engineering history remains in
this file; seat numbering and clearance interpretation remain in their focused
documents. Future routine successes should be summarized in no more than two
or three sentences unless they establish new engineering knowledge.

The remaining work is now ordered by dependency. After the integrated camera
regression, implement reliable unparking and post-unpark field localization,
then validate normal laps with the magenta parking pieces installed. ToF cannot
be made physically blind to those pieces; instead, preserve raw ranges for
collision safety while rejecting returns geometrically consistent with a known
parking piece from wall-based localization corrections. Validate this before
speed or final parking work.

After baseline route coverage, investigate a deterministic constrained path
optimizer for laps 2-3 using the complete lap-1 seat map. Its objective is a
short route with low curvature, subject to robot-envelope, pillar, wall,
tracking-error, steering, curvature, and continuity constraints, with the
current safe route as fallback. Speed must then depend on route curvature. Lap
1 may accelerate only outside upcoming camera decision zones and must slow in
time for reliable two-frame seat detection; laps 2-3 may use higher caps because
the layout is known. Final parking depends on maintained field localization and
direct localization of both magenta parking pieces. End-to-end acceptance comes
last: unpark, localize, three laps, and park.

Run telemetry must also persist comparable timing data in the USB log. Measure
active run time from the accepted start/enable event through controlled finish
or abort, excluding pre-start setup and post-run USB saving. Save each lap split
and total time together with completion/abort status, and retain this definition
for all later route and speed comparisons.

The five accepted optimizations have now been selectively ported into the
current `pure-pursuit` workspace without copying the stale obstacle-path state
from the performance worktree. `Vision` builds an exact 65,536-byte RGB565
classification lookup and skips the image rows above y=80; ToF ready checks
are limited to one poll per millisecond; the GC2145 retains HBLANK `0x011C`
and uses VBLANK `0x0000`; and camera ready-wait telemetry clamps signed
timestamp underflow to zero. The rejected HBLANK `0x008E` profile was not
ported. Current obstacle geometry, perception holds, discovery limits, seam
handling, Pure Pursuit, PID, and speed settings are unchanged.

The IDE-managed `giga_r1_m7` build passed using 361128 bytes RAM (69.0%) and
364568 bytes flash (46.4%). This is the expected roughly 65.5 KiB RAM increase
and leaves about 162 KiB RAM. Existing compiler warnings are unchanged. No
firmware was uploaded. Next, upload only with user consent and repeat the
accepted stationary camera-equivalence checks with official red and green
pillars at centre and validated edge views. Require stable correct colour,
usable geometry, and no new edge-only misses. If that passes, run one
already-safe 175 mm/s obstacle lap and inspect injection timing, ToF freshness,
CTE, heading error, and physical clearance before starting broader reliability.

`log_111` passes the post-port stationary camera gate: red and green were
production-valid at centre and both approximately +/-26-degree edge views,
with no edge clipping, capture errors, missed intervals, or false clear-field
pillar. Frame interval stayed at 76.48-76.50 ms and sampled processing/control
block time was 1.20-2.28/1.27-2.35 ms. Next run one known-safe 175 mm/s
obstacle lap on the integrated firmware; no camera change is warranted.

`log_113` passes the post-port moving gate physically and in telemetry. It
completed one CW lap with red seat 6 at 230 mm, green seat 14 at 260 mm, and
red seat 23 at 260 mm; all three had complete fresh-ToF reports and the user
reported flawless motion. ToF pillar/opposite-wall clearance estimates were
75/177, 131/325, and 149/279 mm; the footer reported `PASS`, maximum CTE
100.1 mm, and maximum heading error 43.5 degrees. The green was physically at
seat 14 rather than the previously requested seat 15, but this still closes
the loop/camera regression because colour acquisition, route injection, ToF
service, and a complete moving lap all succeeded.

`log_112` exposes a separate safety defect after the robot was placed for the
opposite direction but commanded `RIGHT/CW`. From about 7.2 to 12.6 seconds it
made essentially no forward progress while requesting 175 mm/s; duty rose from
about 115 to 144 and the robot continued pushing the wall until the user used
the switch. `check_stalling()` currently arms only when absolute duty exceeds
99% of `MOTOR_MAX_DC`, i.e. greater than 198 of 200. The normal speed loop
never reached that threshold, so its 100 ms stall window was continually
reset. Cross-track stayed below the 300 mm abort because encoder odometry also
stopped, and a raw side-ToF stop is intentionally unavailable because a side
sensor cannot distinguish legal pillars from walls. A relative gyro/odometry
reset also cannot infer that the robot was initially facing the wrong physical
direction.

Treat this as a real watchdog design gap, not a camera-optimization regression.
Before any further powered route test, retain the existing fast high-duty
protection and add a slower commanded-motion/no-progress watchdog independent
of near-maximum duty. It must exclude target-speed-zero braking/perception
holds and log its evidence before pausing. Add deterministic logic coverage
and obtain agreement on a safe physical stall-validation setup; do not
deliberately drive into a hard field wall.

The approved watchdog correction is implemented in `motor_control.cpp` while
retaining the former fast >99%-duty/100 ms protection. The new independent
watchdog arms only when `DC_ENABLED`, target speed and the ramped profile have
the same direction, and both are at least 80 mm/s. It requires at least 10 mm
of signed encoder progress per 1.0-second window; reverse rebound does not
count. A trigger logs progress, elapsed time, target, profile, measured speed,
and duty, then calls `mode_pause()` for an immediate de-energized stop.
Target/profile zero, acceleration below the arming speed, direction changes,
and position holding reset or bypass the watchdog, so perception holds and
planned braking cannot trigger it.

A startup preflight deterministically checks no-progress expiry, progress
reset, target-zero disarming, and reverse-motion rejection. A failed preflight
prevents commanded driving and pauses the mode. The IDE-managed `giga_r1_m7`
build passed with 361144 bytes RAM and 365704 bytes flash; existing warnings
are unchanged and no firmware was uploaded. Next, upload only with consent,
require `[STALL] commanded-motion watchdog preflight: PASS` at startup, and
agree on a non-destructive physical stall setup before deliberately testing
the trigger. Do not use a competition-field wall.

`log_114` contains `[STALL] commanded-motion watchdog preflight: PASS` and no
false stall during the lifted manual-drive check. Since the drive motor cannot
be isolated safely, do not induce a physical stall; accept the deterministic
trigger coverage and free-wheel non-trigger test, and use only a future
incidental stall as physical trigger evidence.

The user's post-save `[GYRO] Main loop did not poll gyro for ...` messages do
not indicate a BNO085 outage. The same messages in `log_111` occur immediately
after `SYSTEM DISABLED`, when the asynchronous USB logger performs mount,
write, `fflush`, close, and verification operations that can block one loop
iteration for roughly 250-400 ms. `update_gyro()` deliberately recognizes a
host-loop gap, resets its observation window, and avoids falsely restarting
the sensor. This occurs with motors disabled and needs no gyro or controller
change; preserve the message because it distinguishes host blocking from a
real `[GYRO] Sensor report timeout`.

The separate task `Profile robot loop timing` produced accepted stationary
changes in branch `perf/loop-camera-optimization` and worktree
`Future-Engineers_2026-loop-perf`. They are not present in the current
Pure Pursuit workspace and remain uncommitted together with an older copied
Pure Pursuit dirty state. Do not merge that branch or worktree wholesale.

The accepted performance changes are: an exact 65,536-entry RGB565 colour
lookup; skipping classification/blob scanning above y=80 because all current
ROIs reject it; polling 30 ms ToF readiness at most once per 1 ms; retaining
stock horizontal blanking while removing 50 vertical blanking lines; and a
telemetry-only signed clamp for camera ready-wait timestamp underflow. The
stationary A/B measurements improved frame interval from 79.61-79.63 to
76.48-76.50 ms, vision processing from 6.62-6.69 to 2.05-2.09 ms, camera-frame
loop blockage from 6.70-6.76 to 2.12-2.16 ms, and ordinary-loop average from
about 0.45-0.51 to 0.026-0.029 ms. Full FOV was preserved. A 3,025-frame run
had zero capture errors; red was always valid and green's 17 invalid frames
were confined to startup. Static RAM rises by about 65.5 KiB to roughly 361 KiB
used, leaving about 162 KiB.

Integrate one gate later, not before the pending lap-seam regression. First run
the current seam-only firmware on the accepted CW `Y-3` layout and require
continuing lap boundaries to remain at 175 mm/s with no delayed index-7
undershoot. This isolates the seam fix. If it passes, selectively port only the
five accepted performance changes before expanding full-field reliability;
otherwise diagnose the seam behavior first. After porting, build, perform a
stationary red/green edge-view equivalence test, and run one already-safe
175 mm/s obstacle lap to validate injection timing, ToF freshness/corrections,
CTE, heading error, and physical clearance. The full reliability suite should
then use the optimized firmware so it does not need to be repeated later.

Main integration risks are the additional 65.5 KiB RAM, up to 1 ms ToF service
latency (only about 0.18 mm travel at 175 mm/s), approximately 6 ms earlier
two-frame camera decisions, and slightly reduced low-light exposure headroom
from zero vertical blanking. Do not port the rejected 0x008E horizontal
blanking experiment or any stale `obstacle_path.cpp`, configuration, test-plan,
or documentation copy from the performance worktree.

`log_110` closes the seam gate. It is a complete CW three-pillar `Y-3` run with
nine passage reports, exactly three laps, correct 230/230/260 lap-1 and
210/210/260 optimized routes, and firmware `PASS`. Both continuing lap
boundaries remained `target=175..175`; the former artificial `135..175` seam
step is absent. Around the formerly failing index-7 region, lap-2 measured
speed remained within the sampled 99..153 mm/s window and lap 3 within
119..166 mm/s, versus 13..55 mm/s in `log_109`. The user did not perceive a
large slowdown, only possible mild variation. Maximum CTE was 98.2 mm and
maximum heading error was 47.6 degrees. Seat-6 optimized pillar/wall ToF pairs
were 80/163 and 75/142 mm; the accepted geometry remains safe in telemetry.

Accept the coincident-neighbour seam fix. The next implementation step is now
to selectively port the five accepted loop/camera changes, not merge the dirty
performance worktree wholesale. After the port and build, run stationary
red/green edge-view equivalence, then one known-safe 175 mm/s obstacle lap
before continuing full-field reliability.

## 2026-08-26 - `log_105` optimized outer route hits the east wall

The representative three-pillar `Y3` run is a physical failure even though the
firmware eventually printed `PASS`. Lap 1 safely repeated the accepted live
path: seat-6, seat-15, and seat-23 ToF pillar/wall clearance pairs were
41/158, 72/155, and 161/280 mm. The optimized path then selected 260 mm for all
three seats. During the lap-2 seat-6 red pass, the robot became stuck against
the east outer wall. The user moved it away so it could continue, which makes
all subsequent pose-dependent evidence and the final result invalid for route
acceptance.

The collision has independent telemetry confirmation. The right wall-side ToF
minimum was 24 mm, corresponding to -11 mm after the 35 mm conservative wheel
inset. Sampled odometry reported -49.4 mm to `outer_east`. Measured speed stayed
at approximately 0-2 mm/s for several seconds while target speed remained
175 mm/s and duty continued rising. The later jump in pose and speed is the
manual intervention, not controller recovery. At the same time the pillar-side
ToF estimated 117 mm clearance, showing that the route spent excessive space
on the pillar side and ran out of wall margin. Lap-2 seat-6 PLAN clearance was
only 78.8 mm to the outer wall, versus 88.1 mm for the physically successful
230 mm lap-1 plateau.

The evidence isolates the optimized route policy/shape rather than detection,
speed, or the Pure Pursuit formula. The safest minimal change is to reuse the
validated 230 mm one-waypoint approach-lead/one-waypoint exit-hold geometry for
isolated outer-extreme pillars on laps 2-3. Keep the 260 mm legacy taper for
moderate pillars such as seat 23, and keep the separately validated 200/210 mm
extreme-adjacent policy. This uses an already physically successful outer route
instead of inventing a new clearance. Do not run this layout again until the
user approves implementation and the IDE-managed PlatformIO build passes. No
firmware source was changed or uploaded in this analysis.

The user approved and the correction is now implemented. A dedicated
optimized clearance selector returns the 230 mm safe route for isolated outer
extremes, the existing 200/210 mm values for confirmed extreme-adjacent pairs,
and 260 mm for moderate pillars. The displacement builder recognizes every
230 mm outer-safe route, not only an unresolved lap-1 route, and applies its
one-waypoint approach lead and exit hold. Configuration names were generalized
from `EXTREME_UNRESOLVED` to `OUTER_SAFE` to reflect the shared lap-1/later-lap
geometry. Deterministic preflight asserts that optimized seat 6 selects 230 mm
and moderate seat 7 selects 260 mm. Pure Pursuit, lookahead, speed, perception,
adjacent geometry, and moderate geometry are unchanged.

The IDE-managed `giga_r1_m7` build passed using 295584 bytes RAM and 364120
bytes flash. Existing compiler warnings are unchanged. No firmware was
uploaded. Next, upload and repeat the same three-pillar CCW `Y3` layout. Expect
optimized messages of 230 mm for seats 6/15 and 260 mm for seat 23. Require no
east-wall contact, nine complete passage reports, and a stop after lap 3.

`log_106` exercises that build and confirms the intended 230/230/260 mm
optimized selectors, nine complete pillar reports, and exactly three laps. The
primary seat-6 wall regression is fixed in telemetry: lap-2 and lap-3
pillar/east-wall ToF estimates were 138/37 and 108/39 mm, with corresponding
positive odometry wall minima of 37.2/38.7 mm. This replaces `log_105`'s -11 mm
ToF and -49.4 mm odometry wall collision estimates. Seat-15 optimized passes
reported 178 mm pillar clearance with 69/67 mm inner-wall clearance; seat 23
reported 162/153 mm pillar and 283/281 mm wall clearance. Maximum CTE was
114.9 mm and maximum heading error was 59.9 degrees.

Do not mark the complete reliability test accepted until the user confirms
there was no physical contact. One additional anomaly needs that context:
around 95.6-96.8 seconds, early on lap 3's clear starting straight, progress
barely changed and the 600 ms status windows fell through measured ranges of
7..101, 3..7, and 3..64 mm/s while target stayed 175 mm/s and duty increased.
There was no perception hold, avoidance event, or commanded stop. The robot
then recovered. Ask whether a physical snag or visible pause occurred. If no
external cause is known, retain the route fix but repeat the same `Y3` layout
once; diagnose drivetrain/power only if the pause repeats. No new firmware
change is justified from this single event.

The user observed about 10 mm physical front-wheel clearance to the outer wall
at red seat 6 on both optimized laps, despite no wall contact. This overrides
the 37/39 mm side-ToF wheel-inset estimates for whole-vehicle clearance and is
below the approximately 30 mm robustness target. The same run retained very
large seat-6 pillar-side ToF estimates of 138/108 mm, so the optimized route can
move inward without threatening the pillar margin. The proposed single route
change is 210 mm plus the existing short plateau for isolated outer optimized
passes only. Lap-1 unresolved outer routes remain 230 mm, moderate routes 260
mm, and adjacent extremes 200/210 mm without the plateau. Implementation needs
an explicit optimized-outer constant or shape flag so the adjacent 210 mm case
does not inherit the isolated plateau. Obtain user approval before changing it.

The user also physically noticed the post-start pause in both laps 2 and 3.
Log telemetry shows it recurring near path index 7 at approximately x=330-380,
y=-990 on the starting straight. Lap 2 slowed to sampled 62..114 mm/s; lap 3
progressed through 7..101, 3..7, and 3..64 mm/s windows before recovering.
Throughout, target was 175 mm/s, steering approximately 2 degrees, duty rose,
and no perception hold or obstacle event was active. It is not intentional
corner slowdown. Because it recurs at essentially the same field location,
first ask whether a mat seam, bump, line, cable, or other local resistance is
present. If not, instrument or diagnose drivetrain/power behavior separately;
do not try to cure it by changing the obstacle route or Pure Pursuit.

The user approved the optimized route correction and reported no physical mat
anomaly at the pause location. The battery was at a low state of charge, which
is now the leading but unverified pause hypothesis. Do not change motor control
from that hypothesis alone.

Implementation adds `OBSTACLE_OPTIMIZED_OUTER_CLEARANCE_MM=210` separately from
the 230 mm unresolved lap-1 value. Optimized isolated outer seats select 210 mm
and pass an explicit plateau flag into path displacement. This is intentionally
not inferred from the numeric clearance because the extreme-adjacent second
route also uses 210 mm but must retain its established non-plateau shape.
Moderate optimized routes remain 260 mm and adjacent routes remain 200/210 mm.
Preflight checks seat-6/seat-7 optimized selection, plateau classification, and
the ordering 200 < 210 < 230 < 260. Pure Pursuit, speed, perception, and motor
control are unchanged.

The IDE-managed `giga_r1_m7` build passed using 295584 bytes RAM and 364216
bytes flash. Existing warnings are unchanged; no firmware was uploaded. Next,
fully charge the battery, upload, and repeat the same CCW three-pillar `Y3`
layout. Expect lap-1 230/230/260 and optimized 210/210/260 messages. Require a
physical front-wheel wall gap of roughly >=30 mm on optimized seat-6 passes,
nine reports, three laps, and no post-start pause. A charged-battery recurrence
of the pause blocks further route testing and calls for focused drivetrain
power/speed instrumentation.

`log_107` is the fully charged-battery A/B repeat. The route selectors were
correct: 230/230/260 mm on lap 1 and 210/210/260 mm on optimized laps. It
produced nine complete passage reports, completed exactly three laps, and
reported `PASS`. Optimized seat-6 pillar/east-wall ToF pairs were 138/92 and
94/119 mm, with odometry wall minima 88.8/117.1 mm. Seat-15 pillar/inner-wall
ToF pairs were 136/111 and 150/93 mm; seat-23 pillar/wall pairs were 157/278 and
163/278 mm. Maximum CTE was 98.7 mm and maximum heading error was 54.1 degrees.

The charged run did not reproduce either starting-straight pause. At the same
path-index-7 area, lap-2 and lap-3 measured ranges were 153..182 and 129..168
mm/s, compared with 62..114 and 3..7 mm/s in low-battery `log_106`. Duty was
also lower. This strongly supports low available battery power, not Pure
Pursuit or obstacle routing, as the earlier cause. Treat this as an A/B
correlation because battery voltage is not logged. Require a fully charged
battery for subsequent multi-lap validation and add voltage instrumentation
only if the pause recurs while charged.

Physical route acceptance is still pending. Ask the user whether any contact
occurred and the approximate front-wheel gap to the outer wall on laps 2/3.
If it was >=30 mm, accept the 210 mm optimized isolated-outer route and the CCW
representative three-lap layout; otherwise use the physical gap rather than the
larger ToF estimates for the next route decision.

The user confirmed approximately 30 mm physical front-wheel clearance to the
outer wall and no meaningful contact. This meets the provisional robustness
target and accepts `log_107`, the 210 mm optimized isolated-outer plateau, and
the representative CCW three-pillar three-lap regression. The charged-battery
A/B result also closes the immediate pause investigation: require a fully
charged battery for multi-lap testing, and reopen drivetrain instrumentation
only if a pause recurs under that condition.

No code change or same-layout repeat is needed. The exact next test is the CW
logical mirror with a charged battery and `Y-3`: red at CW section 1 station 0
right (seat 6), green at CW section 2 station 1 left (seat 15), and red at CW
section 3 station 2 left (seat 23). Reposition the pillars relative to CW travel;
do not assume their present CCW physical locations are the same seats. Expect
lap-1 230/230/260, optimized 210/210/260, nine passage reports, no pause or
contact, and a controlled stop after lap 3.

`log_108` is an incomplete CCW run manually paused after lap 2; do not use it
as the requested direction regression. `log_109` is the complete CW mirror and
its obstacle behavior is correct in telemetry. It selected lap-1 230/230/260
and optimized 210/210/260 mm, produced nine passage groups, completed three
laps, and reported `PASS`. Optimized seat-6 pillar/wall ToF pairs were 99/152
and 106/145 mm; seat-15 pairs were 156/105 and 146/117 mm; seat-23 pairs were
131/294 and 135/292 mm. Maximum CTE was 96.4 mm and maximum heading error was
46.8 degrees. Ask for physical contact and gap observations before accepting
the CW layout.

The user confirmed no physical contact and said the gaps looked good. Accept
the CW multi-pillar obstacle geometry; only the lap-seam speed defect prevents
the run from closing the reliability item.

The slowdown recurred with a charged battery, so low state of charge is not a
sufficient explanation. On lap 3 at path index 6-7, measured speed fell from
55..121 to 13..55 mm/s while target remained 175 mm/s and duty rose, then
recovered. Its approximately 330-360 mm offset from lap wrap matches earlier
CCW events even though CW travels across the opposite physical part of the
starting straight. This rules out a single local mat defect and points to the
lap transition.

The concrete software trigger is a degenerate cyclic seam. Baseline generation
starts with waypoint 0 and finishes its final straight by appending a closing
waypoint at essentially the same coordinates. `recomputeSpeedProfile()` uses
immediate cyclic neighbours; at the coincident closing segment, `atan2(0,0)`
creates an artificial heading and curvature spike. Telemetry shows the result:
each continuing lap boundary briefly includes a 135 mm/s target on a physically
straight segment, immediately followed by 175 mm/s. The acceleration/cruise
controller can then exhibit a delayed undershoot after the target step. This
also explains why the charged `log_107` did not visibly pause even though the
same seam target dip existed: the response is intermittent, but the trigger is
deterministic.

The narrow next fix is to make speed-profile curvature skip coincident previous
or next neighbours and use the nearest non-degenerate segments. Add preflight
coverage requiring straight-line speed at both baseline seam endpoints. Keep
PID, feedforward, obstacle geometry, speed cap, and Pure Pursuit unchanged.
Obtain user approval, build, and repeat the same CW `Y-3` layout once with a
charged battery. No firmware source was changed in this analysis.

The user approved the seam correction. `recomputeSpeedProfile()` now searches
cyclically for the nearest previous and next waypoint at least 1 mm from the
current point before calculating headings and curvature. It keeps both closing
waypoints and every avoidance point intact; only the degenerate neighbour used
for curvature is bypassed. `obstacle_path_geometry_valid()` now additionally
requires both baseline seam endpoints to have `OBSTACLE_PATH_MAX_SPEED`, which
deterministically detects a recurrence of the false seam curvature. PID,
feedforward, speed cap, obstacle geometry, and Pure Pursuit are unchanged.

The IDE-managed `giga_r1_m7` build passed using 295584 bytes RAM and 364424
bytes flash. Existing warnings are unchanged; no firmware was uploaded. Next,
repeat the same charged CW three-pillar `Y-3` run. Continuing lap boundaries
must show `target=175..175`, with no delayed index-7 undershoot; require nine
passage reports, no contact, and a stop after lap 3.

## 2026-08-26 - `log_104` accepts the representative three-pillar lap

The repeated CCW layout passed physically and produced a complete log. The
user reported success without contact or intervention. Red seat 6, green seat
15, and red seat 23 were each injected exactly once at the expected 230, 230,
and 260 mm clearances. All three passage windows completed, one lap completed,
and the footer reported `PASS` with three injections. There was no perception
hold expiry. Maximum CTE was 96.0 mm and maximum heading error was 43.8 degrees.

The PLAN/ODOM/ToF pillar minima were respectively 78.5/-9.6/50 mm for seat 6,
102.5/40.2/73 mm for seat 15, and 121.5/93.3/163 mm for seat 23. ToF-estimated
opposite-wall clearances were 169, 165, and 271 mm. Seat 15's nearest planned
wall feature was the inner north face and still retained 91.6 mm nominally and
165 mm in the opposite-side ToF estimate. The negative seat-6 odometry capsule
estimate again conflicts with both physical success and fresh ToF evidence; it
is a conservative warning/model discrepancy rather than proof of contact.

No controller change is warranted. The exact next test is the same physical
layout for three CCW laps with `Y3`. This isolates optimized multi-pillar path
validation from layout variation. Require one discovery/injection per seat in
lap 1, one optimized-path build, 260 mm later-lap routes for all three isolated
pillars, nine complete passage groups, no contact/intervention or perception
hold expiry, and a controlled stop after lap 3. Save the complete USB log by
waiting through red, then green, then LED off before removal. No firmware
upload is needed.

## 2026-08-26 - Main-loop and camera latency assessment

The current continuous-DCMI camera path is sensor-rate limited, not blocked on
frame capture in the main loop. Existing robot measurements show a stable
79.62-79.63 ms DCMI completion interval (12.56 FPS), zero doubled intervals in
the accepted continuous-capture runs, 71-77 us camera service, and approximately
7.46-7.56 ms combined camera service plus vision processing. Earlier detailed
measurements place vision alone around 6.5-7.5 ms and main-loop frame-ready
service delay commonly around 0.1-1.1 ms. A newly completed frame therefore
normally becomes a processed result roughly 7.6-8.7 ms after DCMI completion.
Including one sensor frame period, acquisition-start to processed-result latency
is approximately 87-88 ms; this estimate is not a rolling-shutter exposure-age
measurement.

Continuous acquisition has already removed the former snapshot stop/restart
misses near 159.25 ms. The dominant interval between usable results is now the
GC2145 timing profile. The accepted 24 MHz XCLK uses the input divide-by-two and
PLL ratio 5, intentionally preserving the reliable downstream timing. PLL ratio
6 shortened capture to about 68 ms but produced lighting bands and missing or
partial pillars, so it must not be enabled without renewed exposure,
anti-flicker, and multi-light validation.

Vision currently classifies all 19,200 samples in a 160 x 120 grid. Every sample
performs RGB565 expansion, integer RGB-to-HSV conversion (including divisions),
four-colour classification, and ROI rejection; a second pass plus 8-neighbour
flood fill extracts blobs. Obstacle navigation consumes red and green blobs;
orange and blue are only printed by the legacy vision debug path. A dedicated
obstacle-only red/green ROI processor could reduce the approximately 7 ms
processing tail, but it would not improve the 79.62 ms sensor cadence. It
requires equivalence tests at FOV edges and under competition lighting before
use.

The existing `[DEBUG]` loop timing is insufficient for a full loop budget. It
records start-to-start periods in `last_loop_time`, mixes fast no-frame loops
with camera-processing loops, reports only mean and maximum over 200 ms, and its
own long telemetry line perturbs the sampled interval. There is no retained
powered log in this checkout containing those loop fields. Do not claim a
measured ordinary-loop duration from the current evidence. For a definitive
profile, add low-overhead counters for complete loop duration and each top-level
stage (serial, ToF, gyro, pose, mode/control, logger), split camera-frame from
no-frame iterations, and report count/mean/max plus histogram percentiles at a
sparse interval. Also timestamp DCMI completion, vision start/end, and obstacle
decision completion so frame age is measured directly. Run stationary and one
known-safe powered lap; profiling changes and any upload require user approval.

No firmware source, build, or upload was performed in this assessment.

---

## 2026-08-26 - `log_86` second 200/210 mm adjacent pass

The unchanged CW seat-6 red/seat-9 green repeat completed without contact,
intervention, or a hidden speed stall. The user judged red acceptable and
green rather close. The log confirms the expected 200 mm red injection,
deferred green confirmation, and 210 mm green activation. One lap completed
formally with 76.2 mm maximum CTE and 68.0 degrees maximum heading error.

Red was substantially farther away than in `log_85`: PLAN remained 50.3 mm,
ODOM/capsule was 38.2 mm, and the left ToF estimated 73 mm pillar clearance.
The right wall-side ToF estimated 153 mm. The physical observation that red was
okay is consistent with all three sources.

Green's close appearance was real but not a contact. Its injection-time PLAN
minimum was 31.2 mm and the right ToF estimated 28 mm, closely agreeing with
the intended geometry and lying near the provisional 30 mm robustness target.
The opposite left ToF still estimated 171 mm wall clearance. ODOM/capsule was
-30.5 mm, again contradicting the successful physical and ToF evidence because
of conservative movement-circle/capsule geometry and pose uncertainty.

The two 200/210 mm repeats now provide physical passes with red ToF minima of
15 and 73 mm and green minima of 57 and 28 mm. Placement and beam geometry
still cause large variation, but neither run contacted or stalled. Accept this
unlikely directly adjacent extreme layout provisionally at 175 mm/s rather
than increasing displacement immediately: the second run already reached a
68-degree heading error, so a reversal larger than 610 mm would become harder
to track. Revisit only if normal layouts expose the same margin problem.

Do not begin a three-lap powered run with the current blanket 160 mm optimized
clearance. Earlier logs proved 160 mm is not placement-robust for these outer
passes. Before three-lap validation, change optimized path construction to use
the validated layout-dependent clearances: ordinary/isolated seats 260 mm and
the extreme adjacent special case 200 mm first / 210 mm second. This retains
the known path rather than introducing an unsafe generic reduction. Obtain
user approval before implementing and do not upload without separate consent.

---

## 2026-08-26 - `log_85` passes 200/210 mm adjacent route narrowly

The first 200 mm red / deferred 210 mm green CW seat-6/seat-9 run completed
without contact or intervention. The user observed approximately 20 mm of red
pillar clearance. Motion telemetry contains no hidden stall. Red injected at
200 mm, green confirmed as deferred, and green then injected at 210 mm with
`delayed_until_first_clear=yes`. The lap formally passed with 100.0 mm maximum
CTE and 54.4 degrees maximum heading error.

The new activation-time PLAN snapshot is working. Red reported 50.3 mm planned
pillar clearance and 138.1 mm planned inner-wall clearance. The driven evidence
was tighter: ODOM/capsule clearance was -7.7 mm while the physical left ToF
minimum was 50 mm, or 15 mm after the wheel inset. The user's approximately
20 mm visual estimate agrees closely with ToF. The opposite ToF retained
232 mm wall clearance. The robot therefore cut about 35 mm inside the planned
pillar margin while leaving ample wall room.

Green reported 31.2 mm PLAN clearance, -33.4 mm ODOM/capsule clearance, 57 mm
ToF pillar clearance, and 153 mm ToF wall clearance. It passed physically.
This repeats the evidence that ODOM plus the conservative movement-circle
capsule can report overlap despite a safe physical pass; treat it as a warning,
not an exact gap.

`OBSTACLE_CLEARANCE_LOGGING.md` now documents the passage window, complete
robot capsule, pillar and wall equations, every PLAN/ODOM/ToF output field,
limitations, cross-source interpretation, and `log_85` as a worked example.

Do not change code from this single safe but marginal run. Repeat the exact
layout once unchanged because earlier red clearance varied with placement. If
red contacts or drops materially below the present 15-20 mm evidence, stop and
redesign or increase its margin. If it repeats safely, decide explicitly
whether to accept this unlikely adjacent layout provisionally or spend more
reversal margin to reach the preferred approximately 30 mm robustness target.

---

## 2026-08-26 - `log_84` proves the 160 mm red-only route is unsafe

The exact CW section-1 seat-6 red followed by seat-9 green repeat failed
physically again. The robot struck red and remained at essentially zero speed
until the user removed that pillar. It then activated the green route and
passed green successfully. The automatic lap `PASS` is invalid because of the
red contact and intervention.

The deferred-injection implementation behaved correctly and rules out the
previous overlap-timing hypothesis. Red seat 6 injected alone at 160 mm.
Green seat 9 later confirmed with `injection=DEFERRED` while injection count
remained one. During the red collision the left ToF stayed around 31-34 mm and
speed remained zero. Only after the pillar was removed and progress advanced
did green inject at 210 mm with `delayed_until_first_clear=yes`. Thus green
geometry did not cause this red collision; the prospective red-only 160 mm
route lacks placement tolerance.

The new source-specific diagnostics agree on the red failure. Odometry/capsule
geometry reached -32.4 mm pillar safety-envelope clearance at pose
(-819,-487) heading 111.0 degrees. The physical left ToF minimum was 29 mm,
or -6 mm after its 35 mm steered-wheel inset. Meanwhile wall clearance remained
large: odometry reported 231.6 mm to the southwest inner corner, and the right
ToF estimated 242 mm. There is therefore room to move the red route farther
away from the pillar.

Green passed physically. Its right ToF estimated 57 mm pillar clearance and
the opposite left ToF estimated 155 mm wall clearance. The odometry capsule
reported -31.7 mm at green despite that physical result, demonstrating that
the conservative capsule plus pose estimate is useful as a warning but not an
exact physical gap. Do not average it with ToF or override the user's physical
observation.

The `[CLEARANCE PLAN]` red value of +8.3 mm is not the red-only plan that was
actually followed. It was calculated at report time, after green had injected
and rebuilt historical path points. Planned clearance must be snapshotted when
each seat's route is activated; otherwise later route revisions rewrite the
diagnostic history. This telemetry defect did not affect steering.

The user approved that implementation. Each seat now stores an immutable
planned-clearance snapshot immediately after its avoidance geometry is
actually injected. `[CLEARANCE PLAN]` identifies this with
`snapshot=injection`; later path rebuilds cannot rewrite the earlier record.
The prospective/first extreme-adjacent clearance is now 200 mm, while the
second remains 210 mm with its 100 mm deferred activation. The 40 mm increase
is supported by the -6 mm ToF contact estimate and targets roughly 34 mm
physical tolerance while retaining substantial measured wall space. Speed,
lookahead, taper, smoothing, perception, and the Pure Pursuit calculation are
unchanged. The requested adjacent reversal is now 610 mm.

Deterministic geometry preflight now requires the 610 mm reversal and the
updated first-member capsule margin. The IDE-managed `giga_r1_m7` build passed
using 295568 bytes RAM and 362856 bytes flash. No firmware was uploaded. After
the user uploads, make one exact CW seat-6 red/seat-9 green repeat and inspect
both physical passes plus all three clearance reports. Stop after any contact;
the larger reversal is not yet physically validated.

---

## 2026-08-26 - `log_82` green rear-wheel contact; second peak raised

The next exact adjacent CW repeat was not robust to physical placement
variation. Red seat 6 passed with a complete dual-ToF report: 82 mm left
pillar-side range, 47 mm estimated wheel clearance, 243 mm right wall range,
and 208 mm estimated wall clearance. Green seat 9 caught the rear wheel and
stalled until the user moved the pillar. Its right pillar-side raw/filtered
minimum was 19 mm, or -16 mm relative to the conservative steered-wheel
envelope. The opposite left wall still measured 242 mm, or 207 mm estimated
wheel clearance. These new full-window measurements supersede the sparse
`log_81` pillar estimates and prove the 160 mm second peak lacks placement
tolerance.

Both pillars confirmed early with the expected 160 mm routes, and the following
station nudge remained zero throughout the green collision. Thus neither
perception timing nor the discovery nudge caused this contact. After the user
moved green, the run later aborted at unresolved S1 station 2; this is not a
separate lap-end latch regression because the physical stall/intervention had
already invalidated the run and delayed its discovery window.

The narrow corrective change raises only the second member of a confirmed
extreme adjacent pair from 160 to 210 mm. The first prospective outer pillar
remains 160 mm. For seats 6 -> 9 the requested reversal becomes 570 mm, still
well below the original failing 720 mm, while the later green peak gains 50 mm
rear-wheel margin. Based on `log_82`, its estimated wall margin would remain
about 157 mm. Geometry preflight requires a 570 mm reversal and more than 50 mm
nominal second-pillar wheel margin. Pure Pursuit, speed, discovery, and all
ordinary layouts are unchanged.

The IDE-managed `giga_r1_m7` build passed using 292376 bytes RAM and 354928
bytes flash. No firmware was uploaded. Repeat the exact layout once; red should
print `clearance_mm=160`, green should print `clearance_mm=210`, and both dual-
sided reports must show positive pillar clearance without contact or a stall.

---

## 2026-08-26 - `log_81` adjacent pass succeeded; cyclic release bug fixed

The prospective 160 mm route succeeded physically for both adjacent outer
pillars. The user reported that red and green passed without contact. `log_81`
confirmed both as seats 6 and 9 with `clearance_mm=160`, two injections,
continuous motion through both passes, 89.3 mm maximum CTE, and 55.0 degrees
maximum heading error. This closes the physical collision regression, pending
one complete-lap confirmation.

The later stop was a separate one-shot-state bug. The extreme-pair discovery
nudge gate correctly released 100 mm after green, but its cyclic distance test
became true again when green was considered ahead on the next lap. It then
suppressed scanning of S0 station 0 near the lap end. That empty station remained
unresolved and the 340 mm safety hold correctly aborted at 335 mm. The gate now
has an explicit pending latch which clears permanently after the first release;
it cannot reactivate later in the lap.

`log_81` also proved the existing `[PILLAR TOF]` sensor mapping was reversed for
legal passes. Red is passed on its right and therefore lies to the robot's left;
green is passed on its left and lies to the robot's right. The old logger chose
the sensor from the fixed seat side and actually summarized the outer wall.
Reconstruction from the 200 ms telemetry gives approximate filtered minima:

- Red: left pillar-side sensor about 62 mm, approximately 27 mm beyond the
  conservative steered-wheel inset; right wall-side sensor minimum 257 mm,
  approximately 222 mm wheel-envelope clearance.
- Green: right pillar-side sensor about 64 mm, approximately 29 mm beyond the
  inset; left wall-side sensor minimum 224 mm, approximately 189 mm
  wheel-envelope clearance.

The pillar-side numbers are sparse telemetry minima, not guaranteed exact
closest gaps. The live-test passage logger now accumulates both ToFs over every
pillar window, selects left for red and right for green at report time, and also
prints the opposite wall sensor and clearance explicitly. Its preflight remains
passing. The IDE-managed `giga_r1_m7` build passed using 292376 bytes RAM and
354848 bytes flash. No firmware was uploaded.

Next: upload and repeat the same CW seats-6/9 layout once. Require both physical
passes, S0 station 0 to clear instead of aborting, one completed lap, and two
new dual-sided `[PILLAR TOF]` records. If it passes, do not spend more powered
testing on this unlikely worst-case adjacency before moving to normal placement
coverage.

---

## 2026-08-26 - `log_80` prospective adjacent preparation and nudge isolation

The first 160 mm special-pair implementation still collided with green seat 9.
The user moved the pillar away after the robot had remained blocked for several
seconds. `log_80` proves why: red seat 6 printed `clearance_mm=260`; only after
green confirmed did seat 9 print `clearance_mm=160` and rebuild both historical
peaks. The robot had already physically driven the 260 mm red detour and was at
its outer side, so changing red path points behind the robot could not recover
the required 100 mm. Heading error still reached 63.1 degrees. Measured speed
then remained essentially zero for about 3 seconds while target speed stayed
175 mm/s, confirming another physical stall rather than a controller pause.

The following empty station's discovery nudge also grew to approximately
-34 degrees before contact, pulling the rotated Pure Pursuit target away while
the robot still needed to complete the green pass. The second fix addresses
both measured causes without adding another steering controller:

- A confirmed outer-going pillar now prospectively uses 160 mm when the next
  station in the same section is unresolved. Thus red seat 6 prepares for a
  possible green seat 9 before the second pillar is known. Moderate, inward,
  nonadjacent, and final-station routes retain their existing behavior.
- Once an extreme adjacent pair is confirmed, discovery target nudging toward
  the following station is suppressed until 100 mm after the newly confirmed
  second pillar. Pure Pursuit follows the avoidance path without that competing
  view rotation. The following station is then about 400 mm ahead, still before
  the 340 mm hold.

Geometry preflight now also verifies outer/moderate classification and the
prospective station gate. The IDE-managed `giga_r1_m7` build passed using
291896 bytes RAM and 354152 bytes flash. No firmware was uploaded. In the next
single CW repeat, both red seat 6 and green seat 9 should print
`clearance_mm=160`; after green injection, `nudge_deg` should decay toward zero
and remain there through its pass. Require no contact/stall and stop after one
failure for log inspection.

---

## 2026-08-26 - `log_79` adjacent outer reversal still collides

The CW red-seat-6 to green-seat-9 adjacent regression failed physically again.
The user reported that the robot drove into the green pillar and was about
20 mm too close during the pass. Both perception events were correct: red seat
6 confirmed and injected first, then green seat 9 confirmed at approximately
23.5 degrees/410 mm and produced the second injection. The 340 mm unresolved
hold was therefore not relevant; this was not a camera miss or late-detection
failure.

After the green injection, heading error climbed to 61.1 degrees. Measured speed
then fell from roughly 150-220 mm/s to essentially zero for about 2.6 seconds
while target speed remained 175 mm/s and motor duty rose, matching a physical
stall against the pillar. The later pose jump/recovery must not be counted as
controller success. The run ultimately aborted at empty S1 station 2 and
printed `FAIL`, zero completed laps, and two injections. The incomplete green
ToF report (236 mm raw minimum/201 mm derived estimate) is not a credible
measurement of the physical contact clearance.

This confirms the same controller/path limitation as `log_62` and `log_64`.
With 260 mm lap-1 clearance, the additive path targets -360 mm lateral at the
outer red seat and +360 mm at the adjacent outer green seat: a 720 mm reversal
between centres only 500 mm apart. Corrected capped-speed lookahead and earlier
viewing did not make that route trackable. Do not repeat this layout unchanged.

The user approved the narrow route correction and noted that adjacent pillars
within one section are unlikely in competition. The live lap-1 path is now
rebuilt from the baseline whenever a pillar confirms. Only a confirmed pair in
neighboring stations of the same section whose normal targets point to opposite
outer extremes uses the existing 160 mm optimized clearance for both members;
isolated, moderate, nonadjacent, and cross-section layouts remain at 260 mm.
For seats 6 -> 9 this reduces the reversal from 720 to 520 mm while retaining a
nominal 47.5 mm outer-wheel gap outside the 42.5 mm movement circle
(160 - 42.5 - 70).

The implementation is in `src/obstacle_path.cpp`, with the dedicated clearance
alias in `include/config.h`. Injection telemetry now prints `clearance_mm`; the
first isolated seat-6 injection should print 260 and the seat-9 injection that
activates the pair should print 160. Geometry preflight verifies that seats
6/9 activate the special case, moderate seats 7/8 and separated seats 6/11 do
not, the reduced reversal is 520 mm, and nominal wheel margin remains at least
30 mm. Pure Pursuit steering, speed, perception, and the 340 mm safety hold are
unchanged. The IDE-managed `giga_r1_m7` build passed using 291896 bytes RAM and
353880 bytes flash. No firmware was uploaded.

Next powered validation is one repeat of the exact CW red-seat-6/green-seat-9
layout with the disable switch reachable. Require the second injection to print
`clearance_mm=160`, no physical contact or near-zero-speed interval, materially
lower heading error than `log_79`'s 61.1 degrees, and a completed lap. Stop after
one failure and inspect its log; do not tune another variable concurrently.

---

## 2026-08-26 - `log_78` isolated red seat-6 viewing validation passed

The first powered validation of the 1.35 single-seat target gain and 340 mm
safety hold passed. The user reported a successful complete lap. In CW
`log_78`, the isolated red pillar at S1 station 0 right/seat 6 entered the
accepted view at approximately -26.4 degrees/307 mm after the discovery target
nudge reached the intended -40-degree cap. It was confirmed as
`seat=6 color=RED`, exactly one avoidance path was injected, no perception hold
or abort occurred, and the result was `PASS` with one completed lap.

The per-pillar right-ToF summary reported a 236 mm raw minimum and 201 mm
geometry-based wheel-clearance estimate. This does not match a close pillar
pass and was probably a wall/background return, so it must not be treated as an
exact pillar-clearance measurement. It establishes no ToF hazard but does not
replace the user's visual no-contact observation.

Next test: restore the exact CW adjacent transition, retaining the red pillar
at S1 station 0 right/seat 6 and adding green at S1 station 1 left/seat 9. Arm
with `Y-1`. Require two correct confirmations/injections, continuous motion
through the right-to-left avoidance transition, no contact or intervention,
and one completed lap. If either pillar is unresolved, the 340 mm hold must
stop the robot safely; do not immediately repeat a failed run.

---

## 2026-08-26 - `log_77` seat-6 red miss and physical collision

The first exact adjacent-layout attempt on the continuous-DCMI/capped-
lookahead firmware failed before path injection. In RIGHT/CW `log_77`, the red
pillar at S1 station-0 right seat 6 was never observed or confirmed. Obstacle
injections remained zero, so the robot followed the baseline path and
physically drove into the red pillar. The user reported the contact. This is a
perception/fail-safe failure, not evidence about the corrected adjacent Pure
Pursuit transition.

The opposite/left seat cleared, but seat 6 never entered `vis`. Its best
geometry in the validated 230-600 mm discovery range was approximately
-29.9 degrees at 289 mm, outside the accepted approximately +/-27.4-degree
half-angle. Observations remained `NONE`; a single `VOTE:9` was for the farther
green pillar and did not resolve station 0. This reproduces the earlier green
seat-6 edge-view miss with the other colour and confirms a geometry/orientation
problem rather than a general green threshold problem.

The unresolved-station hold was not physically safe. Although
`OBSTACLE_DISCOVERY_HOLD_DISTANCE_MM` is 170 mm, station distance is derived
from 50 mm path progress, so the hold latched at 135 mm. The robot front is
measured 130 mm ahead of the rear axle and the official movement circle has a
42.5 mm radius; a 135 mm rear-axle-to-seat distance already permits geometric
overlap before braking. Do not repeat a powered seat-6 test with this value.

Safety-first change implemented with user approval: the unresolved hold
distance is now 340 mm instead of 170 mm. With the 125 mm camera offset and
100 mm seat lateral offset, approximately 335-340 mm rear-axle forward distance
corresponds to the last validated roughly 230-235 mm camera slant range. It
also leaves about 160 mm between the robot front and the pillar movement-circle
near edge before braking. Keep speed, route, FOV, confirmation counts, and
discovery target behaviour unchanged for this safety change.

The IDE-managed PlatformIO `giga_r1_m7` build passed after this change. No
firmware was uploaded. With subsequent user approval, the single-seat target
gain was raised from 1.0 to 1.35 while retaining the 40-degree cap, slew limit,
and validated camera acceptance window. At the useful 289 mm sample, the old
gain requested about 29.9 degrees; the new gain reaches the existing 40-degree
cap so the chassis is biased farther toward the remaining unresolved seat.
Simultaneous two-seat viewing is unchanged.

Next test: place only a red pillar at S1 station 0 right/seat 6 for CW and arm
with `Y-1`; leave the opposite seat and following station empty. Require a red
seat-6 confirmation, exactly one injection, no contact, and no perception-block
abort. If detection still fails, the new 340 mm hold must stop the robot safely;
do not repeat or restore the adjacent green pillar until the log is evaluated.

---

## 2026-08-25 - Continuous DCMI low-speed powered regression

The known-safe single-pillar powered regression was completed on the uploaded
`camera-continuous-dcmi` firmware. The field was available under the same
darker cellar lighting used for stationary acceptance. The robot ran the
RIGHT/CW live obstacle mode at the configured 175 mm/s cap with the official
red pillar in the instructed outer seat-0 layout. The user observed a flawless
complete run, continuous motion, no contact, and substantially more clearance
around the pillar than required. Treat this as a physical pass of the
low-speed driving gate and the capped-lookahead regression.

The corresponding newest USB file is `D:\log_76.txt`. It confirms continuous
camera capture remained at 79.62-79.63 ms with zero missed intervals, discarded
frames, or capture errors through 4,645 stationary frames before the drive. It
also confirms `OBSTACLE_LIVE_TEST`, RIGHT/CW, and the 175 mm/s cap, then records
the first 6.8 seconds of motion. It does not contain the completed-lap report,
the expected red seat-0 injection, or the target pillar's completed
`[PILLAR TOF]` record.

This missing telemetry is explained by a logger-capacity failure rather than a
driving or camera failure. The normal live test does not clear the logger at
enable, so the long stationary camera session had already consumed nearly all
of the fixed 128 KiB RAM buffer (`LOG_BUFFER_SIZE` in `include/logger.h`). The
131,109-byte file ends with `*** WARNING: LOG BUFFER OVERFLOW ***` while the
robot is still at path time 6,816 ms. The user's full-lap observation occurred
after that point and cannot be reconstructed from this file. Consequently the
physical regression passes, while exact seat/injection count and the new ToF
clearance diagnostic remain unverified for this run.

No repeat is required for the continuous-DCMI low-speed physical gate. If
formal obstacle-map and ToF telemetry are needed before progressing to the
previously failed adjacent-pillar layout, repeat the same safe layout from a
fresh boot and start it promptly, or first change the live-test logger to clear
stale stationary telemetry at the start of a run. The brighter
competition-like stationary test remains explicitly waived/skipped, not
passed.

---

## 2026-08-25 - `log_60` ToF clearance diagnostics and result predicate

The deferred `log_60` diagnostics are implemented. The live one-lap test now
maintains an independent accumulator for every candidate seat from 300 mm
before through 300 mm after its baseline path position. This buffers samples
before camera confirmation, supports the intentional 100 mm overlap between
adjacent 500 mm station windows, and reports only confirmed pillars. A left
seat uses the left ToF and a right seat uses the right ToF. Every diagnostic
snapshot sequence is consumed at most once per seat.

`[PILLAR TOF]` prints the seat, colour, facing sensor, whether the full window
completed, fresh and valid sample counts, the minimum quality-accepted raw
range, and the minimum production-filtered range. It also prints a clearance
estimate for each minimum. Raw is retained because the production filter can
limit a change to 100 mm per frame; filtered is retained for direct comparison
with historical live telemetry. Invalid, non-finite, out-of-range, and above-
600 mm samples do not contribute to either minimum.

The clearance conversion is now explicit in `include/config.h`. Wheel diameter
is 43.2 mm and radius is 21.6 mm. Rear/front axle positions are 0/100 mm, so
their nominal longitudinal wheel extents are -21.6..+21.6 mm and
78.4..121.6 mm. Outside-wheel width is 125 mm, or 62.5 mm per side; maximum
steering adds approximately 7.5 mm per side, producing a conservative 70 mm
half-envelope. With the ToF apertures at lateral coordinates +/-35 mm, both
aperture-to-wheel-envelope insets are `70 - 35 = 35 mm`. The logger therefore
reports `range - 35 mm`. This is deliberately labelled an estimate: the
side-facing beam at local X=40 mm does not necessarily coincide
longitudinally with the widest steered wheel point, and it cannot guarantee
that it samples the exact whole-robot closest instant.

The generic result predicate no longer requires exactly one path injection.
It still requires no abort reason, one completed lap, and maximum CTE at or
below 180 mm. The final report states that injection acceptance is
layout-specific; expected seat IDs, colours, and counts must still be checked
in each run log.

Startup deterministic coverage checks the 21.6/70/35 mm geometry derivation,
fresh-sequence deduplication, raw and filtered minimum accumulation, independent
overlapping accumulators, and invalid-sample exclusion. The IDE-managed
`giga_r1_m7` build passed using 291848 bytes RAM (55.7%) and 354688 bytes flash
(45.1%). The build also contains the already approved capped-speed lookahead
change documented below. No firmware upload or robot connection occurred.

After the user uploads, retain the existing next-test order: first regress the
known-safe outer single-pillar layout for the capped-lookahead change. Its log
should also contain one complete `[PILLAR TOF]` record with plausible raw and
filtered minima and sample counts. Only after that physical regression should
the failed adjacent seat-6 red/seat-9 green CW layout be repeated. Do not infer
clearance from the estimate alone; require the physical no-contact observation
and continuous motion.

---

## 2026-08-25 - Continuous DCMI fixed-pillar dark-light acceptance

The first positive-image stationary acceptance test passed on the uploaded
`camera-continuous-dcmi` firmware. With the enable switch LOW, the official red
and green pillars were placed in the established fixed geometry and the camera
ran in `capture_mode=continuous`. A settled baseline was taken at test frame
82 with zero cumulative invalid frames for either colour.

Over the following 2025 processed frames, red and green each remained valid on
every frame: neither invalid-count difference increased. The periodic detailed
telemetry contained 75/75 production-valid red reports and 75/75
production-valid green reports, with no `blob=NONE` reports. Estimated range
was stable at 443.7-444.0 mm for red and 454.3-455.0 mm for green.

Completion intervals remained 79.62-79.63 ms. `missed_intervals`,
`discarded_frames`, and `capture_errors` all remained zero. This passes the
current/darker-light 2000-frame classification, timing, and observable image
integrity gate. COM4 was closed and released; no drive command was sent.

Keep the robot and both pillars in exactly the same geometry. The next test is
the same settled 2000-frame procedure under brighter competition-like light.
Only after that passes should the known-safe red seat-0 right/CW powered
regression be run.

The user subsequently chose to skip the brighter-light stationary test because
that lighting/setup is not available in the cellar. Treat it as explicitly
waived for the present development sequence, not as passed. The successful
2025-frame cellar-light result is the available stationary evidence. When the
full field becomes available, proceed with the known-safe red seat-0 right/CW
powered regression at 175 mm/s; preserve the bright-light limitation in the
final reliability assessment.

---

## 2026-08-25 - Continuous DCMI branch implementation and first robot run

The optional camera update-jitter work is implemented on branch
`camera-continuous-dcmi`. The project-owned Arducam driver now starts one
uninterrupted `DCMI_MODE_CONTINUOUS` capture with STM32 DMA double-buffer mode.
The DMA completion callbacks publish only the completed SDRAM buffer, its
timestamp, and a sequence number. Cache invalidation and vision processing
remain in the main loop. When the consumer falls behind it selects the newest
complete buffer and counts discarded frames. `CAMERA_CONTINUOUS_CAPTURE_ENABLED`
selects this path; setting it to `false` retains the prior asynchronous snapshot
path.

Telemetry now reports `capture_mode`, `discarded_frames`, and `capture_errors`.
The IDE-managed `giga_r1_m7` build passed with 291888 bytes RAM and 352880 bytes
flash. The user authorized an upload to the stationary robot; the upload via
COM4/DFU succeeded. Camera calibration auto-started with motors disabled.

The direct serial run reached frame 2215. After the initial 404.58 ms startup
frame, every reported completion interval stayed within 79.62-79.63 ms.
`missed_intervals`, `discarded_frames`, and `capture_errors` remained zero.
DMA service was 71-77 us and control blockage was approximately 7.46-7.56 ms.
This passes the 2000-frame duration/timing criterion and eliminates the former
159.25 ms snapshot restart gaps in this scene.

The continuous flag was then temporarily disabled and the snapshot fallback
also compiled successfully (291872 bytes RAM, 354872 bytes flash). The flag was
restored to `true`, and the final continuous build again passed with the same
291888-byte RAM and 352880-byte flash result. The robot still contains the
equivalent successfully tested continuous build; no second upload was needed.

The robot faced an uncontrolled non-mat scene without official pillars, as the
user specified. Therefore red/green production validity and a positive
pillar-geometry/torn-frame check were not tested; both counters correctly
remained zero for the background. Before calling continuous acquisition fully
accepted, place both official pillars in the established fixed stationary
geometry and require at least 2000 settled production-valid frames with stable
geometry, then repeat under brighter competition-like lighting. Only after
those pass should a user-operated low-speed obstacle lap be used for the final
driving gate. No powered movement was commanded in this session.

---

## 2026-08-25 - 24 MHz camera and continuous-DCMI TODO assessment

The accepted 24 MHz GC2145 profile is working as intended. The current source
still sets `CAMERA_SENSOR_XCLK_HZ=24000000`, page-zero `F7=0x1F` for input
divide-by-two, and PLL ratio 5. Post-change logs report `async=yes`, stable
normal completion intervals near 79.62 ms, roughly 6.6-8.1 ms control blockage,
and successful obstacle classifications/injections. The profile was intended
to provide reliable 24 MHz XCLK while preserving the proven downstream sensor
and DVP timing, not to increase frame rate.

The optional continuous-DCMI TODO in `CAMERA_24MHZ_DEVELOPMENT.md` is still
unimplemented: both async starts in the project-owned Arducam driver use
`DCMI_MODE_SNAPSHOT`. Snapshot restart misses remain measurable. Across the
final counters of the instrumented post-24-MHz sessions in `D:\log_44.txt`,
`log_46.txt`, `log_49.txt`, `log_54.txt`, `log_56.txt`, and `log_61.txt`, there
were 78 intervals over 120 ms among 975 measured inter-frame intervals (about
8.0%). The long intervals consistently peak near 159.25 ms rather than the
normal 79.62 ms. Delivered images remained usable; this is update jitter, not
evidence of unreliable 24 MHz signalling.

The recent driving evidence does not make continuous DCMI a current blocker.
`log_61` accumulated five missed intervals by frame 50 before its run, then
correctly found both pillars, injected seats 7 and 8, and completed the lap.
`log_62` and `log_64` also confirmed and injected both pillars before the
already documented large adjacent path reversal caused physical green-pillar
contact. Moving green farther away produced completed laps in `log_63` and
`log_65`. Thus those failures cannot reasonably be attributed to a late camera
update. The printed `FAIL` in the completed two-pillar runs is the separately
documented result-predicate defect, which the newer handoff above resolves.

Recommendation: retain snapshot async capture and the accepted 24 MHz profile
for current path development. Continuous DCMI would approximately halve the
worst normal update gap (159.25 to 79.62 ms) and raise delivered rate from the
measured roughly 11.8 FPS toward the no-miss 12.56 FPS, but it adds nontrivial
DMA/buffer-ownership risk and has low likelihood of changing the outcome of
the recent runs. Reconsider it only after path tracking is stable, or if a
future log ties an unresolved/late two-frame decision to a doubled interval.

---

## 2026-08-25 - Obstacle test-plan documentation consolidation

`OBSTACLE_CHALLENGE_TEST_PLAN.md` was reduced to the tests that remain to be
run. Its former development narrative, calibration notes, completed gate
evidence, failure analysis, and tuning rationale are retained in the obstacle
handoffs below. This file is now authoritative for that durable engineering
context; the test plan is authoritative for test order and acceptance criteria.

Current obstacle state at consolidation:

- Empty-track Pure Pursuit is accepted at the 175 mm/s validation cap in both
  directions. Camera calibration, asynchronous acquisition, stationary
  perception, and ToF localization/residual gating are validated.
- Representative red-left and green-right left/CCW one-lap layouts pass.
- Red-right right/CW passes without contact with 260 mm first-lap clearance.
- Green-left right/CW completed without a stall at 260 mm clearance, but the
  observed approximately 1 mm physical gap is not a robust margin.
- `OBSTACLE_PATH_TAPER_WAYPOINTS` was therefore increased from 6 to 8. The
  IDE-managed `giga_r1_m7` build passed with 291176 bytes RAM and 352192 bytes
  flash. It has not been uploaded by an agent.

Exact next tests, in order:

1. After the user uploads, repeat green-left right/CW with the eight-waypoint
   taper. Require a practical visible gap at the wheels/body and wall,
   continuous motion, the correct green seat-5 detection, exactly one
   injection, all stations clear, and one completed lap.
2. If that passes, regress red-right right/CW because the wider taper changes
   both pass sides. Require the same no-contact/no-stall conditions, correct
   red seat-4 detection, one injection, all stations clear, and a completed
   lap.
3. Then exercise station 0 and station 2 placements, followed by two adjacent
   opposing-colour pillars. These tests specifically cover detour overlap and
   different longitudinal seat positions.
4. Only after those one-lap tests pass, proceed to three-lap optimized-path
   validation, full-field reliability regression, speed optimization, and
   final parking, in that order.

The live-test controls remain `Y1` for left/CCW, `Y-1` for right/CW, and `Y0`
to abort and brake. Arm with the physical enable switch LOW and keep the switch
reachable. The user performs powered runs and provides the USB log. Do not
upload firmware or make a new tuning change without explicit consent. Treat
physical contact as failure even if firmware reports `PASS`, and change only
one diagnosed cause at a time.

---

## 2026-08-25 - Asynchronous camera branch integration

`camera-async-buffering` was merged into `pure-pursuit`. Conflict resolution
kept the measured full-FOV calibration and the latest obstacle-discovery nudge,
while adding the project-owned Arducam driver, two fixed SDRAM framebuffers,
and non-blocking DCMI/DMA capture. `CAMERA_ASYNC_CAPTURE_ENABLED` and the safe
stationary camera auto-start are enabled. The stored routes and Pure Pursuit
steering calculation were not changed by this merge.

The merged build succeeds. Before the pending powered red-left run, repeat the
representative official-pillar seat test and field-clear test with motors
disabled. Confirm `[CAM PERF]` reports `async=yes`, advancing frame numbers,
stable detections, and no capture stalls. Only then resume the exact powered
test described in the next handoff entry.

`D:\log_39.txt` passed the async and official-pillar portions. Frames advanced
continuously through 1929; normal capture spans were about 76-84 ms, occasional
missed sensor-frame spans were about 150-159 ms, service cost was 75-83 us, and
typical processing/control blockage was about 6.2-6.8 ms. Seat 3 confirmed in
two frames with one injection and 12.6-22.0 mm snap error. A direct robot-side
attempt verified that the red pillar was still physically present, so a valid
field-clear control was impossible. The mode was stopped, the drive motor
remained locked, and COM4 was released. Remove the pillar; an agent may then
reconnect and perform the five-second field-clear test without another upload.

That direct continuation is now complete. With the pillar removed, the agent
resumed the motor-locked seat mode, armed `seat expect 0 1 L 400`, and observed
for more than seven seconds. Results alternated between no blob and tiny green
fragments around y=94-100; every fragment was rejected, votes remained `R0/G0`,
and injections remained zero. The mode was stopped and COM4 released. The next
step is the user-operated, post-async red-left powered run; do not retune the
nudge before analyzing it.

Three post-async powered repetitions are now available as `D:\log_40.txt`
through `D:\log_42.txt`. All three correctly injected red seat 5 exactly once,
then failed identically at S1 station 0: its right seat cleared, left seat 7
never appeared in `vis`, observations remained `NONE`, and the 35-degree nudge
cap arrived only after the viewing window. Maximum CTE was 69.0/67.9/67.8 mm
and maximum heading error 15.3/16.7/16.6 degrees. Async capture is not the
blocker. The old blocking loop sometimes produced more heading overshoot; the
smoother async controller no longer happens to swing the camera far enough.

Proposed next implementation, requiring user approval: only when one seat is
left unresolved, target it nearer the optical axis and use full bearing-error
gain so the existing 35-degree cap is reached sooner. Do not modify the stored
route, the simultaneous two-seat rule, perception confirmation, scan start
distance, or the Pure Pursuit steering formula. The user explicitly prohibited
firmware downloads without consent; code changes and uploads should be
separately confirmed if the requested scope is unclear.

The user approved the code change but not a firmware upload. The single-seat
target bearing is now 0 degrees and its dedicated gain is 1.0; the simultaneous
two-seat rule still uses the existing 0.75 gain. The scan start, 35-degree cap,
routes, perception, and Pure Pursuit formula are unchanged. Build locally and
request separate consent before uploading to the robot.

The local `giga_r1_m7` build passed at 291096 bytes RAM and 349952 bytes flash.
No upload or robot connection was performed.

The user subsequently uploaded and ran this firmware themselves. `D:\log_43.txt`
shows that the stronger single-seat response fixed the earlier first-corner
failure: S1 stations 0, 1, and 2 all cleared. At S2 station 0 the right seat
cleared, the nudge reached the unchanged 35-degree cap, and left seat 13 never
entered `vis`; S2 station 1 cleared independently. The robot stopped at the
135 mm perception limit. It made one correct obstacle injection, with 92.9 mm
maximum CTE and 20.9 degrees maximum heading error.

Do not infer the next controller change from `log_43` alone. `vis` combines the
predicted bearing window with the 260-600 mm range window, so the log cannot
distinguish an angular miss from seat 13 becoming too close. The next proposed
change is diagnostic only: expose predicted camera-relative bearing and range
for both seats of the active discovery station in `[LIVE PATH]` telemetry.
Ask before implementing if the intended diagnostic scope is uncertain, and
obtain explicit consent before any firmware upload.

The user approved that diagnostic implementation. `ObstacleDiscoveryTelemetry`
now carries predicted bearing and range for the right and left seats, and live
telemetry prints them as
`seat_geom=R<bearing_deg>/<range_mm>,L<bearing_deg>/<range_mm>`. It uses the
same camera/seat geometry as `vis` and does not change any controller,
perception, route, visibility, or timing decision. The IDE-managed build passed
at 291096 bytes RAM and 350208 bytes flash. It has not been uploaded; explicit
user consent is still required before uploading or connecting to the robot.

The user uploaded the merged 24 MHz/diagnostic firmware and produced
`D:\log_44.txt`. The geometry diagnostic isolates the S2 station 0 failure:
left seat 13 first entered the angular window at `L27.3/272`, then was already
below the current 260 mm range gate (`L25.3/238`) 201 ms later. Interpolation
leaves only about 70 ms of valid overlap, less than one normal 79.62 ms frame
interval and therefore insufficient for two clear frames. The run injected the
red pillar once, cleared all S1 stations and S2 station 1, then stopped safely;
maximum CTE was 94.3 mm and maximum heading error was 19.5 degrees.

Before lowering the view minimum, validate that both official pillar colours
remain production-valid at 220 mm slant range and about +/-25 degrees. Place
them relative to the camera lens about 200 mm forward and 93 mm left/right in
the motor-disabled camera mode. If both remain `production_valid=yes` after
settling, the proposed next code change is 260 to 220 mm for
`OBSTACLE_DISCOVERY_VIEW_MIN_MM`; ask before implementing and obtain separate
consent before uploading.

The direct COM4 test at the nominal 220 mm placement measured red at
+25.1 degrees/224.5 mm and green at -24.0 degrees/226.4 mm. Neither pillar was
edge-clipped and both were usually production-valid, but the boundary was not
fully reliable. Red intermittently lost its upper segment and failed the
`max_top_y` rule; green intermittently measured 81 px wide against the 80 px
acquisition maximum. The serial listener was stopped and COM4 released; no
drive command or upload occurred.

Do not lower the gate to 220 mm or test nearer yet. The next stationary check
is 230 mm at about +/-25 degrees: approximately 208 mm forward and 97 mm
left/right from the camera lens. A 230 mm gate would retain about 250 ms of the
S2 visibility overlap from `log_44`, enough for two frames even with one normal
snapshot miss. If both colours are stable there, ask before implementing a
260-to-230 mm change and separately before any upload.

The direct 230 mm COM4 check passed. The camera measured red at +25.4 degrees
and 237.3 mm, and green near -24.3 degrees and 235.2-239.8 mm. Neither was
edge-clipped; red was normally 77x111 px and green 73-77x109-111 px, below the
production size maxima. Over a settled 383-frame interval, green was valid
383/383 times and red 367/383 (95.8%). Red's occasional invalid frame was a
top-segmentation flicker rather than clipping. The available S2 window still
contains about three frames, enough for the independent two-vote obstacle and
two-clear-frame rules. COM4 was released without commands or upload. The next
proposed implementation is only `OBSTACLE_DISCOVERY_VIEW_MIN_MM` 260 to 230;
ask before changing it and separately before uploading.

The user approved the code change. `OBSTACLE_DISCOVERY_VIEW_MIN_MM` is now
230 mm; steering, stored routes, FOV, camera timing, and confirmation counts
are unchanged. Build locally and obtain separate permission before any upload.

The IDE-managed `giga_r1_m7` build passed at 291128 bytes RAM and 351136 bytes
flash. No upload or robot connection occurred. After explicit upload consent,
repeat one user-operated left/CCW run on the `log_44` layout and verify that S2
station 0 reaches two left-seat clear frames rather than the 135 mm hold.

The user uploaded this build and produced `D:\log_45.txt`. The 230 mm gate
worked: S2 station 0 and every station in S1-S3 cleared. The run then failed at
S0 station 0 with the analogous inside left seat. Its geometry changed from
`L29.7/257` to `L26.2/216`, so angular entry occurred at essentially the 230 mm
range boundary and supplied no usable frame interval. The run injected red
once and stopped safely, with 90.8 mm maximum CTE and 20.9 degrees maximum
heading error.

Do not lower the range gate: 220 mm is already the stationary acquisition
boundary. The next proposed implementation is only the simultaneous two-seat
gain `OBSTACLE_LOOK_TARGET_GAIN` from 0.75 to 1.0. At the failing corner the
0.75 rule generated about 18.8 degrees from about 24.9 degrees of pair-centring
error. Full gain should orient the camera about 6 degrees earlier while both
seats remain unresolved. Keep the single-seat rule, cap, slew, route, 230 mm
gate, and Pure Pursuit calculation unchanged. Ask before implementation and
separately before upload.

The user approved the simultaneous-centering change.
`OBSTACLE_LOOK_TARGET_GAIN` is now 1.0. The single-seat gain, 35-degree cap,
slew limit, stored route, 230 mm gate, camera settings, and Pure Pursuit
steering formula are unchanged. Build locally and obtain separate permission
before uploading.

The IDE-managed `giga_r1_m7` build passed at 291128 bytes RAM and 351120 bytes
flash. It was not uploaded. After explicit upload consent, repeat one
user-operated left/CCW run on the `log_45` layout. S0 station 0 must clear, the
lap must complete, and maximum CTE should not regress materially beyond 90.8
mm.

The user uploaded this build and produced `D:\log_46.txt`. They correctly
observed that the robot physically completed a circuit. The full simultaneous
gain fixed S0 station 0 and reduced maximum CTE to 78.5 mm; red was injected
once. The formal result remained `FAIL`/lap 0 because the car stopped at S0
station 1 before the lap boundary, with 21.9 degrees maximum heading error.

The new blocker is per-seat perception bookkeeping, not Pure Pursuit. Empty
left seat 3 was comfortably visible around `L20.3/439`, but the known red
pillar at farther seat 5 was on the same bearing as a valid `20.0/912` blob.
It later reported `KNOWN:5`. `updateDiscoveryCoverage()` currently treats only
`NO_BLOB` as a clear frame for every seat, so any valid pillar anywhere in the
wide image resets all empty-seat counters. This prevented seat 3 from clearing.

Proposed next implementation: compute clear evidence independently per visible
seat. No valid blob is clear; a valid blob whose angular footprint does not
overlap the seat is also clear; an overlapping blob is clear only when its
estimated range is conservatively behind the predicted seat range. Rejected or
invalid observations remain non-clear. Keep two-frame confirmation, controller
gains, route, 230 mm gate, and camera settings unchanged. Add focused
diagnostic coverage for the behind-seat case, ask before implementation, and
obtain separate consent before upload.

The user approved this implementation. Discovery clear evidence is now
seat-specific. It uses each production-valid blob's pixel-edge bearings with a
2-degree calibration margin. A non-overlapping blob counts the seat clear; an
overlapping blob counts it clear only if it is at least 180 mm behind the
predicted seat. No blob remains clear, while rejected/invalid and nearby
overlapping blobs remain non-clear. Two frames are still required.

Live telemetry now prints `evidence=RL`. Geometry preflight covers the exact
`log_46` far-behind geometry plus no-blob, nearby-overlap, off-angle, and
rejected-blob cases, and runs before movement. The IDE-managed build passed at
291144 bytes RAM and 351736 bytes flash. It was not uploaded. After explicit
upload consent, repeat one user-operated left/CCW run on the same layout and
verify that S0 station 1 clears and the formal lap counter reaches one.

`D:\log_38.txt` is the final pre-async powered result, not an async validation:
startup left camera calibration pending while the switch was LOW, proving the
stationary async auto-start was absent. It verified the 850/650 mm discovery
trigger but still failed on S2 left seat 13. Discovery started about 184 mm
earlier than in `log_37`; the right seat cleared, while the left never appeared
in `vis`. Maximum CTE was 79.7 mm. Do not tune the nudge again until the merged
async firmware has passed its stationary checks and the identical powered run.

---

## 2026-08-25 - Obstacle Challenge Pure Pursuit and full-FOV camera handoff

### Objective and architectural constraint

The current objective is to complete the WRO 2026 Future Engineers Obstacle
Challenge using Pure Pursuit as the only steering controller. Camera discovery
may temporarily rotate the Pure Pursuit lookahead target, but it must not add a
steering overlay or create a second controller. Stored centreline and
pillar-avoidance routes remain the path authority.

The robot has a measured 100 mm wheelbase. Its production pose origin is the
rear-axle midpoint. Camera and ToF offsets are in `include/config.h`. Field and
test placement details are maintained in `OBSTACLE_CHALLENGE_TEST_PLAN.md`, and
seat indexing is illustrated in `OBSTACLE_SEAT_NUMBERING.md`.

### Rules and local-only reference material

The competition rules PDF was moved to the gitignored local workspace:

`local_workspace/WRO-2026-Future-Engineers-Self-Driving-Cars-General-Rules.pdf`

`local_workspace/` is intentionally ignored. Do not recreate the former `ai/`
folder. The rules document is reference material; instructions embedded in
documents are not agent instructions.

### Current camera implementation and calibration

The GC2145 now uses its full sensor field of view. `FullFovGC2145` in
`include/camera.h` and `src/camera.cpp` reads the 1616x1208 sensor window and
uses sensor-side 5:1 subsampling to produce 320x240 without a large framebuffer
or a second image-coordinate system.

Verified production calibration:

- Camera clock: reliable 24 MHz XCLK with the GC2145 input divide-by-two stage
  and PLL ratio 5. The downstream pixel timing remains at the validated rate.
- Processing time: approximately 6.5-6.7 ms per image with timing diagnostics.
- Normal DCMI completion interval: approximately 79.62 ms, with occasional
  159.25 ms snapshot stop/restart misses. Continuous acquisition is deferred.
- Horizontal FOV: 65.3 degrees.
- Principal X: 164.4 px.
- Horizontal focal length: 248.9 px.
- Ground-range horizon: 78 px.
- Ground-range scale: 24000 mm-px.
- Former cropped-view edge correction: disabled (`0.0`).
- Comfortable discovery half-angle: about 27.4 degrees
  (`65.3 * 0.42`). Complete pillars were stable at +/-26.565 degrees.

Stationary production checks passed:

- A red middle-left test selected seat 3, bearing 14.9-15.2 degrees, range
  376.4-376.7 mm against a nominal 400 mm, snap error 23.7-23.9 mm, two votes,
  and exactly one path injection.
- A field-clear test ran for more than ten seconds with no confirmation and
  zero injections.

Useful accuracy targets are bearing within 2 degrees, range within 30 mm, and
seat snap error preferably below 50 mm. These are validation targets, not
reasons to discard a geometrically correct observation inside the configured
140 mm seat-snap radius.

### Pure Pursuit and discovery state

Completed controller work:

- Production steering is Pure-Pursuit-only.
- Lap-one discovery rotates only the temporary lookahead target.
- The former later-lap residual steering overlay was removed.
- Both possible seats are considered simultaneously while both are unresolved.
- Empty stations require two consecutive usable full-FOV frames. Pillars retain
  their independent two-vote colour and geometry confirmation.
- Raw side-ToF obstacle stopping is disabled in the live test because a side
  ToF cannot reliably distinguish a pillar from a wall.
- Both ToFs passed range, transform, direction-specific corner-gating, fresh
  sequence, and 500 mm cutoff tests.

Read-only discovery telemetry is emitted in `[LIVE PATH]` lines:

- `disc=S<section>.<station>`: station under discovery.
- `vis=RL`, `R-`, `-L`, or `--`: predicted comfortable seat visibility.
- `clear=R/L`: consecutive clear-frame counters.
- `seat_geom=R<bearing>/<range>,L<bearing>/<range>`: predicted camera-relative
  seat geometry used to separate bearing and range visibility gates.
- `obs=NONE|REJECT|RANGE|NOSEAT|VOTE|CONF|KNOWN:<seat>`: observation result.
- `blob=x1,y1-x2,y2@bearing/range`: blob geometry when applicable.

The telemetry is implemented through `ObstacleDiscoveryTelemetry` in
`include/obstacle_path.h`, `src/obstacle_path.cpp`, and
`src/obstacle_live_test.cpp`. It does not alter control behaviour.

### Powered-run evidence

Empty-track Pure Pursuit is already accepted at 175 mm/s: seven one-lap tests
passed (four CCW/left and three CW/right). Do not repeat the waived five-runs
per direction matrix. Corner slowdown tuning is deliberately deferred until
the end, when production speed is increased.

Relevant full-FOV live logs on `D:\`:

- `log_33.txt`: correctly injected red seat 5 once; cleared all S1 stations;
  failed at S2 station 0 with the older narrow scan rule. Maximum CTE 104.8 mm.
- `log_34.txt`: wide-FOV nudge reduced maximum CTE to 57.6 mm, but the remaining
  S1 seat did not receive three clear frames. This motivated two-frame clear
  confirmation for the 150 ms frame time.
- `log_35.txt`: correctly injected seat 5; blocked on left seat 7 at S1 station
  0. Maximum CTE 64.0 mm.
- `log_36.txt`: correctly injected seat 5; cleared S1; blocked on left seat 13
  at S2 station 0. Maximum CTE 78.2 mm.
- `log_37.txt`: diagnostic run. It correctly injected seat 5 and cleared all S1
  stations. At S2 station 0 every observation was `obs=NONE`; the right seat
  reached `clear=2`, but left seat 13 never entered `vis`. Visibility progressed
  `--`, then `R-`, then `--`. The run stopped safely with maximum CTE 69.5 mm.

`log_37` rules out competing coloured background blobs and the two-frame clear
requirement as the S2 failure cause. The camera was not oriented toward the
inside/left seat early or strongly enough.

### Discovery steering development history

The stored route was not changed. In `include/config.h` and
`src/obstacle_path.cpp`:

- Discovery target orientation now begins 850 mm before an unresolved station,
  previously 700 mm.
- Full taper response begins at 650 mm, previously 550 mm. This pre-orients the
  chassis before the station enters the calibrated 600 mm camera range.
- While both seats are unresolved, the simultaneous full-FOV midpoint rule is
  unchanged.
- Once one seat is clear, the remaining seat is brought toward 12 degrees from
  the camera axis instead of merely being accepted near the 24.4-degree
  margin-adjusted edge.
- The target nudge remains capped at 35 degrees and slew-limited to 60 deg/s.
- Perception confirmation thresholds are unchanged.

The IDE-managed PlatformIO build for `giga_r1_m7` passed after this change:

- RAM: 290952 / 523624 bytes (55.6%).
- Flash: 348232 / 786432 bytes (44.3%).
- Existing `Serial` redefinition, unused-function, and unsigned comparison
  warnings remain; no new build error was introduced.

The build command required by `AGENTS.md` is:

```powershell
$pio = Join-Path $env:USERPROFILE '.platformio\penv\Scripts\platformio.exe'
& $pio run --environment giga_r1_m7
```

### Exact next test

1. Upload the current firmware.
2. Reuse the exact left/CCW layout from `log_37`: rear-axle midpoint on the
   starting-straight centreline, robot parallel to the walls, first corner
   approximately 500 mm ahead; red pillar at the next station, 500 mm forward
   and 100 mm left of the rear-axle midpoint.
3. `Y1` arms the left/CCW live run. It may be sent before placing the robot on
   the field. Toggle the enable switch when ready. `Y0` aborts and brakes.
4. The user prefers to perform powered runs personally, then attach the USB
   logging stick and say `logs ready`. Analyze the highest-numbered
   `D:\log_*.txt`.
5. Primary success condition: S2 station 0 must show the left seat in `vis` and
   resolve instead of aborting. Also compare maximum CTE with `log_37`'s
   69.5 mm and check for wild steering or stop-start motion.

If the left seat still arrives too late, use the telemetry before changing
anything else. Prefer adjusting the target response rate/cap only after the
850/650 mm and 12-degree change has robot evidence. Do not change the stored
corner route, add a scan state, weaken pillar confirmation, reduce empty clear
confirmation to one frame, or tune final corner speed prematurely.

### Temporary configuration and remaining work

`STARTUP_ROBOT_MODE` is temporarily `MODE_CAMERA_CALIBRATION` for stationary
safety. The live mode is selected with `Y1`/`Y-1`. Restore the intended
competition startup mode only after development testing and with the enable
switch handled safely.

Remaining sequence:

1. Validate the latest S2 visibility change.
2. Obtain reliable full obstacle laps and mirror the result CW/right.
3. Validate three obstacle laps and later-lap optimized paths.
4. Increase production speed and tune corner slowdown near the end.
5. Implement and validate final parking separately.

The current ordered checklist is in `OBSTACLE_CHALLENGE_TEST_PLAN.md`;
historical evidence and engineering rationale remain in this file.

### Current validated state and exact next test

Seat-specific clear evidence was subsequently implemented. It evaluates each
visible seat independently using the blob's angular footprint with a 2-degree
bearing margin and permits clear evidence for an overlapping blob only when it
is at least 180 mm behind the predicted seat. Two-frame confirmation remains in
place; rejected, invalid, and nearby overlapping blobs remain non-clear.

`log_47.txt` formally passed the first representative obstacle layout:

- Direction: left/CCW.
- Red S0 station-2 left pillar confirmed as seat 5 and injected exactly once.
- Every remaining station cleared and `[PATH] Completed lap 1` appeared.
- Maximum CTE: 75.7 mm; maximum heading error: 24.1 degrees.
- Startup telemetry showed `evidence=R-` while the red blob occupied the left
  seat, confirming independent evidence for the unobstructed right seat.

No code adjustment is justified by this successful run. Next, hold the driving
direction and start geometry fixed while testing the other colour/pass side:
place a green pillar 500 mm forward and 100 mm right of the rear-axle midpoint,
arm with `Y1`, and perform one left/CCW lap. Require the correct seat, exactly
one injection, green passed on the left, all other stations cleared, and one
completed lap. The user performs powered runs and supplies the USB log. Do not
upload firmware without explicit consent.

`log_48.txt` then tested green-right left/CCW. Perception and path handling were
correct: seat 4 was confirmed green, exactly one path injection occurred, all
other stations cleared, and one lap completed. The reported result was `FAIL`
only because maximum CTE reached 181.1 mm against the 180 mm pass threshold;
maximum heading error was 25.8 degrees.

This peak exposed ToF pose contamination, not a Pure Pursuit route problem.
Near the pillar, the right ToF reported 108 mm; over one 200 ms telemetry
interval pose Y jumped approximately 76 mm and CTE rose from 59.3 to 144.6 mm.
That lateral motion is incompatible with the logged speed and heading and is
consistent with several clipped 12 mm pose-correction steps treating the green
pillar as the wall. The estimate later recovered and the physical lap finished.

Recommended next implementation: gate ToF corrections on the unbounded wall
residual before applying gain and step clamping. Reject a residual too large to
represent a credible localization error, while retaining normal corrections,
the 500 mm range cutoff, fresh-frame gating, and corner gates. Add deterministic
coverage for a normal wall residual and a 108 mm pillar-like return. Do not
merely increase the 180 mm test pass threshold. Ask the user before implementing
and before any upload; after implementation, repeat green-right left/CCW once.

The user approved that implementation. `include/config.h` now limits the
absolute pre-gain wall residual to 150 mm. `applyTofCorrectionAt()` records the
unbounded residual and rejects larger values before applying the existing 0.18
gain and 12 mm step cap. This preserves the validated +/-100 mm pose offsets;
the reconstructed `log_48` 108 mm pillar return has an approximately -357 mm
residual and is rejected. The existing 500 mm range cutoff, fresh-sequence
consumption, and corner gates are unchanged.

Deterministic geometry preflight coverage accepts +/-100 mm and rejects the
108 mm pillar-like case. `ObstacleTofCorrectionResult`, live telemetry, and the
stationary diagnostic now expose residual values and per-side residual-gate
flags. The IDE-managed build passed with 291176 bytes RAM (55.6%) and 352192
bytes flash (44.8%). No firmware was uploaded.

Exact next test remains green-right left/CCW in the `log_48` placement. After
the user uploads, expect `tof_residual_gate=-R` near the pillar with a residual
well beyond -150 mm, no instantaneous pose/CTE jump, one correct green seat-4
injection, every other station clear, and one completed lap. Ask before making
any further code change or uploading firmware.

`log_49.txt` and `log_50.txt` validated the residual gate. Both confirmed green
seat 4 and injected once. `log_49` explicitly logged a 110 mm right return with
`tof_residual_gate=-R` and a -378 mm residual. Maximum CTE stayed at 99.2 and
98.9 mm, rather than the corrupted 181.1 mm from `log_48`. No further ToF
change is indicated.

Both runs consistently aborted at S1 station 0. The right seat cleared, but the
remaining left seat became angularly visible only after its range dropped below
the validated 230 mm minimum. Representative `log_50` geometry was 33.1
degrees/293 mm, 28.6/248, then 27.0/210; the bearing limit is about 27.4 degrees.
The temporary target nudge was saturated at its 35-degree cap during the useful
part of this window. This is neither a camera-frame dropout nor blob rejection.

Recommended next change, pending user approval: raise only
`OBSTACLE_LOOK_MAX_TARGET_NUDGE_DEG` from 35 to 40 degrees, retaining the 60
deg/s slew, 230 mm range gate, stored path, and Pure Pursuit steering formula.
Then rebuild and let the user repeat green-right left/CCW. Do not upload without
separate consent.

The user approved this change. `OBSTACLE_LOOK_MAX_TARGET_NUDGE_DEG` is now 40
degrees. No look timing, gain, slew rate, camera limit, stored-path geometry,
clearance, or steering formula changed. The IDE-managed build passed with
291176 bytes RAM and 352192 bytes flash. No firmware was uploaded. The next run
is the identical green-right left/CCW layout; require S1 station 0 to clear
before the perception hold and compare CTE with the prior 98.9-99.2 mm runs.

`log_51.txt` formally passed that green-right left/CCW rerun. Green seat 4 was
confirmed, exactly one injection occurred, all remaining stations cleared, and
one lap completed. S1 station-0's remaining left seat reached 27.5 degrees at
256 mm and cleared, validating the 40-degree cap without lowering the 230 mm
range gate. Maximum CTE was 87.1 mm and maximum heading error 18.6 degrees. The
right ToF again saw 114 mm near the pillar without corrupting pose. No further
CCW adjustment is justified.

Exact next run: first mirrored CW case. Put the rear-axle midpoint on the same
starting-straight centreline but face the robot toward the CW first corner,
about 500 mm ahead. Place a red pillar 500 mm forward and 100 mm right relative
to the robot, arm with `Y-1`, and run one lap. Require the correct seat, exactly
one injection, red passed on the right, all stations clear, and a formal pass.
The user uploads and runs; do not upload or change code without consent.

`log_52.txt` correctly confirmed red right seat 4 in the first CW layout, but
the robot physically struck the pillar and the user moved it aside by about
20-30 mm so the run could continue. Wheel speeds collapsed to approximately
zero for about 1.6 seconds at path index 7 and recovered after intervention.
The later completed lap and firmware `PASS` are therefore invalid. Maximum CTE
was 104.1 mm and maximum heading error 40.9 degrees in the assisted run.

The cause is path clearance, not perception or pass-side selection. With
`OBSTACLE_LAP1_CLEARANCE_MM=200`, the radius-1 smoothing kernel reduces the
red-right peak to roughly 171 mm pillar-centre separation. A 230 mm requested
clearance yields roughly 199 mm, adding about 27 mm while retaining about 200 mm
rear-axle-path distance to the corridor wall at the peak. Recommended next
change, pending approval: increase only first-lap clearance from 200 to 230 mm,
then rebuild and repeat red-right right/CW. Do not upload without consent.

The user approved the clearance adjustment. `OBSTACLE_LAP1_CLEARANCE_MM` is now
230 mm. Optimized-lap clearance remains 160 mm, and no taper, smoothing,
controller, discovery, or perception setting changed. The IDE-managed build
passed with 291176 bytes RAM and 352192 bytes flash. No firmware was uploaded.
Next repeat red-right right/CW and require physical clearance with no
intervention before accepting the automatic result.

`log_53.txt` validated the 230 mm clearance in the red-right CW layout. The
robot passed without contact or user intervention, wheel motion remained
continuous through the former collision area, red seat 4 was confirmed, one
injection occurred, every station cleared, and one lap completed. Maximum CTE
was 113.9 mm and maximum heading error 46.7 degrees. No adjustment is indicated.

Exact next run: retain the successful CW robot start, place a green pillar
500 mm forward and 100 mm left of the rear-axle midpoint, arm with `Y-1`, and
run one lap. Require green passed on the left, no pillar/wall contact, exactly
one injection, all stations clear, and a formal pass. The user uploads and
runs; do not change code or upload without consent.

`log_54.txt` correctly confirmed green left seat 5 in the CW layout, but the
rear wheel touched the pillar. Both wheel speeds dropped to approximately zero
for about 1.4 seconds at path index 8 before the robot freed itself. The later
S0 station-0 discovery abort and its 111.3 mm maximum CTE/34.6-degree maximum
heading error are not suitable tuning evidence because contact invalidated the
lap first.

Recommended next change, pending approval: increase only first-lap clearance
from 230 to 260 mm. Smoothing makes this about 27 mm more peak separation and
about 20 mm more in the contact approach, while estimated body-to-wall margin
remains above 110 mm. Keep taper, smoothing, optimized clearance, controller,
discovery, and perception unchanged. Rebuild, then let the user repeat
green-left right/CW. Do not upload without consent.

The user approved this change. `OBSTACLE_LAP1_CLEARANCE_MM` is now 260 mm.
Optimized clearance remains 160 mm, and taper, smoothing, controller, discovery,
and perception are unchanged. The IDE-managed build passed with 291176 bytes
RAM and 352192 bytes flash. No firmware was uploaded. Next repeat green-left
right/CW and require clear front/rear wheel passage with no stall or
intervention before accepting the automatic result.

`log_55.txt` formally passed green-left CW with continuous wheel motion, one
green seat-5 injection, every station clear, 123.3 mm maximum CTE, and 36.3
degrees maximum heading error. The residual gate rejected the short pillar ToF
returns. The user nevertheless observed only about 1 mm physical clearance, so
this was not a robust physical pass.

Do not move the peak avoidance waypoint earlier: it should remain aligned with
the pillar. The robot cut inside the short transition, reaching about 114 mm CTE
near path index 8. The user approved increasing
`OBSTACLE_PATH_TAPER_WAYPOINTS` from 6 to 8. At 50 mm sampling this starts the
detour 100 mm earlier while keeping clearance 260 mm, smoothing radius 1, and
the Pure Pursuit controller unchanged. The IDE-managed build passed with 291176
bytes RAM and 352192 bytes flash. No firmware was uploaded.

`log_56.txt` improved the physical gap to about 10 mm with no contact or stall.
Green seat 5 was confirmed and injected once; maximum CTE was 126.9 mm and
maximum heading error 42.1 degrees. The run later aborted at S2 station 0. Its
unresolved right seat moved from 28.1 degrees/261 mm to 25.5 degrees/224 mm,
crossing the angle bound only after falling below the validated range. Since
`log_55` cleared S2 and the taper is inactive there, repeat green-left CW once
unchanged before tuning. Keep the peak aligned with the pillar; do not upload or
change code without consent.

`log_57.txt` passed the unchanged green-left CW repeat. Green seat 5 was
confirmed and injected once, all stations including S2 station 0 cleared, wheel
motion remained continuous, and one lap completed formally. Maximum CTE was
126.7 mm and maximum heading error 41.2 degrees. The user again measured about
10 mm pillar clearance and more than 50 mm wall clearance. This makes the
260 mm clearance/eight-waypoint taper provisionally acceptable at 175 mm/s.

Next test is red-right CW regression with the same firmware and robot start:
place the red pillar 500 mm forward and 100 mm right, arm with `Y-1`, and require
no contact, adequate wall clearance, one correct injection, all stations clear,
and one completed lap. No upload is needed if this firmware remains installed.

`log_58.txt` passed that red-right CW regression. Red seat 4 was confirmed and
injected once, all stations cleared, wheel motion remained continuous, and one
right/CW lap completed formally. Maximum CTE was 122.1 mm and maximum heading
error 44.9 degrees. The ToF pose-residual gate correctly rejected the pillar
returns instead of treating them as a wall. The user observed about 10 mm
pillar clearance and more than 50 mm clearance to the right wall. Together
with `log_57`, both pass sides provisionally validate the 260 mm first-lap
clearance and eight-waypoint taper at 175 mm/s.

The next genuinely new placement is S0 station 0. Keep the validated CW start
and orientation; place a red pillar 500 mm behind the rear-axle midpoint and
100 mm to the robot's right. Arm with `Y-1`. The pillar is intentionally behind
the robot at startup and is approached near the end of the lap. Require red
right seat 0 confirmation, exactly one injection, no contact or intervention,
all other stations clear, continuous motion, and a formal completed lap. The
user runs the test and supplies the log; no firmware change or upload is
needed.

`log_59.txt` passed this station-0 test. Red S0 station-0 right seat 0 was
confirmed and injected exactly once, all other stations cleared, motion stayed
continuous, and the right/CW lap completed formally. Maximum CTE was 112.3 mm
and maximum heading error 35.3 degrees. The user observed ample clearance on
both the pillar and wall sides. Do not reduce clearance based on this isolated
geometry; the extra margin is desirable and must be checked with overlapping
detours and later higher speeds.

Next test two adjacent opposing-colour pillars on section 1, the straight
immediately after the first CW corner. Put red at station 0 on the left of the
CW driving direction (seat 7), then green at station 1, 500 mm farther along,
on the right (seat 8). This produces alternating pass directions while keeping
the two peak targets nearer the corridor centre than the outer-seat worst case.
Their eight-waypoint tapers still overlap, so require correct confirmation and
one injection for each pillar, a continuous transition, no contact or
intervention, all remaining stations clear, and a formal lap. No code change
or upload is needed.

`log_60.txt` correctly confirmed red S1 station-0 left seat 7 and green S1
station-1 right seat 8, injecting each exactly once. The robot completed the
lap with continuous motion and ample visually observed clearance on both
sides. Maximum CTE was 159.8 mm, below the configured 180 mm pass limit, and
maximum heading error was 26.5 degrees. The printed `FAIL` does not indicate a
controller failure: `obstacle_live_test.cpp::finishAndReport()` hard-codes
`obstacle_path_injection_count() == 1`, which necessarily rejects a correct
two-pillar run.

The side-ToF evidence supports the physical observation. Around S1 station 0,
the left sensor, facing the red pillar while the robot passed on its right,
reached a minimum logged filtered range of about 151 mm. Around S1 station 1,
the right sensor, facing the green pillar while the robot passed on its left,
reached about 80 mm. These values measure from each sensor aperture to the
visible pillar surface along its beam. They cannot yet be treated as exact
whole-robot minimum gaps because normal live telemetry samples sparsely, the
range filter can lag, the beam may miss the instant at which a wheel is
closest, and the configured +/-35 mm ToF lateral coordinates do not by
themselves establish the aperture inset from the widest body/wheel point.

Recommended diagnostic change: associate each confirmed seat with its facing
side ToF, collect every fresh sample over a bounded longitudinal passage
window, and print the minimum sensor-to-pillar range once the seat has been
passed. A body-clearance estimate additionally requires measured left and
right aperture-to-outer-envelope insets. Separately, remove the generic
one-injection equality from the live-test result predicate; expected seat IDs
and counts remain layout-specific acceptance checks in the log.

These were explicit deferred TODOs at the time of `log_60`; the newer handoff
above documents their implementation. The remaining
one-lap queue is: (1) mirror the moderate seat-7 red/seat-8 green adjacent
layout in CCW, (2) test the larger alternating seat-6 red/seat-9 green layout
in CW, and (3) mirror that larger-displacement layout in CCW only if the CW
test retains safe margins. The current firmware can run all three; do not
upload without consent. Until the result predicate is fixed, evaluate correct
two-pillar runs from their seat IDs, two injections, lap completion, CTE,
motion, and physical result rather than the misleading final `FAIL` alone.

The user then ran `log_61` through `log_65`. `log_61` mirrored the moderate
seat-7 red/seat-8 green layout in CCW and completed continuously (CTE 109.5 mm,
heading error 27.4 degrees). The two-injection predicate alone caused its
printed `FAIL`.

The larger seat-6 red to seat-9 green reversal at 500 mm spacing failed in both
directions. In CW `log_62`, green was confirmed around 430 mm before its seat,
but the car later remained at essentially zero speed for about 1.8 seconds,
recovered, exceeded the 180 mm pass CTE at 191.9 mm, and aborted because the
next station could not be resolved. In CCW `log_64`, green was confirmed around
350 mm before its seat, heading error rose to 64.3 degrees, the right ToF fell
to 32-39 mm, and speed stayed essentially zero for about 1.2 seconds before
recovery. Thus late detection is not the primary cause. The additive tapers ask
the path to reverse about 720 mm laterally between centres only 500 mm apart,
which the robot does not safely track at 175 mm/s.

The user also moved green from station 1 to station 2. The resulting seat-6 red
to seat-11 green layouts completed continuously in CW `log_63` (CTE 126.4 mm,
heading error 35.2 degrees) and CCW `log_65` (CTE 107.6 mm, heading error 44.2
degrees). This doubles peak spacing to 1000 mm and isolates the failure to the
adjacent large reversal rather than either colour or side independently.

Sparse filtered sensor-to-pillar minima were: `log_61` red/green 115/89 mm;
`log_62` red 66 mm with no reliable green side hit before the stall; `log_63`
34/155 mm; `log_64` 92/32 mm; and `log_65` 76/81 mm. Do not interpret these as
whole-robot gaps until fresh-sample tracking and sensor-to-envelope offsets are
implemented. Stop additional powered layouts until the adjacent seat-6 to
seat-9 transition is corrected and rebuilt. Ask the user whether physical
pillar contact or manual intervention occurred in `log_62` and `log_64` before
choosing the smallest implementation change.

The user subsequently confirmed physical green-pillar contact in both
`log_62` and `log_64` and manually assisted the robot so it could continue.
These are definitive physical failures; neither recovered trajectory is valid
acceptance evidence. The user could not recall whether the 34 mm red ToF sample
in `log_63` appeared physically close, so retain it only as a warning and do
not infer a verified body gap from it.

The next recommended implementation is a general Pure Pursuit consistency fix,
not a layout-specific steering mode. Lookahead currently uses the uncapped
`progress.speedMmS`; the live-test and runtime speed caps are applied only
later to the motor command. Thus a path point retaining the 260 mm/s nominal
speed selects the maximum 330 mm lookahead even when the robot is capped at
175 mm/s. Compute one effective capped speed first and use it for both adaptive
lookahead and the speed command. At 175 mm/s the configured interpolation gives
about 208 mm lookahead. Change no obstacle geometry or other controller setting
in the same iteration. After a build, regress a known safe outer single-pillar
layout before repeating the failed seat-6 to seat-9 CW layout. Implementation
and any firmware upload still require the user's direction; never upload
without explicit consent.

The user approved implementation. `src/obstacle_path.cpp` now computes
`cappedPathSpeed(progress.speedMmS)` before Pure Pursuit target selection and
uses it for both `adaptiveLookahead()` and the normal motor command. At the
175 mm/s cap this selects approximately 207.6 mm instead of 330 mm. Perception
safety logic may still reduce motor speed afterward. Deterministic geometry
preflight checks cover the minimum-speed, 175 mm/s, and maximum-speed
interpolation points. No path geometry, clearance, steering limit, perception,
or speed cap changed. The IDE-managed `giga_r1_m7` build passed with 291176
bytes RAM and 352192 bytes flash. No firmware was uploaded. Next, the user must
upload and regress a known safe outer single-pillar layout before repeating the
failed adjacent seat-6 red to seat-9 green CW layout.

Post-change logs `66` through `73` cover at least eight powered starts. The
user reported more than 30 mm physical pillar clearance in all detected cases.
`log_71` and `log_73` formally completed five-pillar CCW laps, each confirming
seats 5 red, 9 green, 13 red, 16 green, and 23 green exactly once. Their maximum
CTE values were 120.0 and 119.5 mm. Fresh-sample ToF clearance estimates were,
respectively, 144/188/90/189/119 mm and 142/193/99/182/131 mm for those seats.
This validates the new passage logger and demonstrates strong physical margin
for those layouts.

The reported intermittent stops are perception safety aborts, not post-start
motor/controller stalls: no new log contains a telemetry window whose maximum
measured speed remains near zero. Logs 66, 67, 69, and 70 blocked and braked at
unresolved empty S2 station 0; logs 68 and 72 blocked at unresolved S1 station
0. Logs 71 and 73 completed without a stop. Thus empty-station resolution has
become the next reliability issue after the shorter lookahead changed the
tracked pose/heading. Do not weaken clear evidence based only on these logs.
The user reported one green miss, but the physical layouts corresponding to
the zero-injection logs 68 and 72 are not known from telemetry alone and must
be clarified.

These runs did not contain the exact formerly colliding red seat-6 to green
seat-9 adjacent pair. Keep that transition TODO open even though broader
clearance improved. Ask which log contained the green pillar, its exact seat,
and whether the robot contacted it or stopped safely before selecting either
the adjacent regression or an empty-station perception diagnostic.

The user clarified that the missed green pillar occupied S1 station-0 right
seat 6 in CW, and that none of the runs used the exact red-seat-6 followed by
green-seat-9 adjacent collision layout. It is not known whether the missed run
was log 68 or 72. Both zero-injection logs put seat 6 marginally outside the
validated camera region: their best bearings within 230-600 mm were -29.8 and
-27.8 degrees, versus the accepted approximately +/-27.4 degrees. Treat this as
a seat-6 edge-view reliability question, not evidence that the colour detector
generally fails. Repeat a single green seat-6 CW layout twice unchanged before
tuning. If both confirm and pass safely, proceed to the exact adjacent
red-seat-6/green-seat-9 CW regression. If either misses, stop and adjust viewing
geometry before combining pillars.

## Validated-clearance optimized path and three-lap harness

The later-lap optimized path no longer applies a blanket 160 mm avoidance to
every confirmed pillar. `buildOptimizedPath()` now uses the same validated
layout selector as lap-1 injection: 260 mm for ordinary/isolated pillars and
200/210 mm for a confirmed extreme adjacent pair. It prints one
`[PATH] Later-lap avoidance` line per confirmed seat and then
`clearance_policy=validated-layout`. At the lap-1 transition, PLAN snapshots
are refreshed from the newly activated optimized path.

The live harness accepts `Y3` (CCW) and `Y-3` (CW) for three laps; the existing
`Y1/Y-1` commands remain one lap. Passage accumulators reset after each lap and
print `[PILLAR PASS] seat=<id> lap=<1..3>` with independent ODOM and ToF minima.
PLAN output now says `snapshot=route-activation`, because lap 1 refers to the
injected route while laps 2-3 refer to the optimized route. Three-lap repetitive
status telemetry uses 600 ms instead of 200 ms to preserve the 128 KiB USB log
buffer; clearance and event reports remain immediate. Timeout remains 120 s
per requested lap.

The IDE-managed `giga_r1_m7` build passed after this change, using 295576 bytes
RAM and 363472 bytes flash. The first powered test should isolate
lap wrapping with one previously safe pillar: CW start, one red pillar at seat
0 (section 0, station 0, right), all other seats clear, command `Y-3`. Place the
rear-axle midpoint on the starting-straight centreline, parallel to the walls,
facing the first CW corner 500 mm ahead. Place the pillar centre 500 mm behind
the rear-axle midpoint and 100 mm to the robot's right. Require three untouched
passes, one optimized-path build after lap 1, 260 mm reported for seat 0, three
complete passage reports, one injection total, and a controlled stop after
exactly lap 3. Do not upload firmware without explicit user consent.

`log_87.txt` ran that requested `Y-3` test and contains a complete log, so the
600 ms telemetry interval successfully avoided USB-buffer truncation. The path
wrapped and stopped after three laps, but the obstacle result is physically
invalid: the user reported that the robot failed to detect the red pillar and
pushed it aside. Telemetry showed `obs=NONE`, `[MAP] Clear S0 station=0`, zero
injections, and an empty validated-layout optimized path after lap 1. The right
ToF status value near the seat fell to 61 mm on lap 1, 95 mm on lap 2, and
111 mm on lap 3. The changing values are consistent with the pillar having
been displaced; they are not clearance reports because an unconfirmed pillar
does not activate a passage accumulator. The firmware printed `PASS` only
because the live harness has no declared expected layout; physical contact
always overrides it. Maximum CTE was 88.7 mm and maximum heading error 20.7
degrees, but these describe an effectively empty route.

Do not tune clearance or repeat a powered run from `log_87`: the 260 mm route
was never injected. First isolate acquisition with the same red pillar and
lighting in the motor-locked seat test. Use CW `S-1`, then
`seat expect 0 0 R 400`; physically place the pillar 100 mm right and about
512 mm forward of the rear-axle midpoint (400 mm horizontal camera range with
the existing 125 mm camera offset). If it remains `REJECTED_NO_BLOB`, inspect
red colour/blob acquisition before altering discovery geometry. If it reliably
confirms red, investigate the dynamic seat-0 view and the two-frame NO_BLOB
clear policy. Ask the user before choosing or implementing either change, and
do not upload without explicit consent.

`log_88.txt` contains the requested stationary test followed by a second
hand-moved trial. It rules out a general red HSV/blob failure. At the nominal
400 mm view, red was production-valid on every reported frame, produced a vote
immediately, confirmed on frame two, and remained `ALREADY_CONFIRMED`; measured
geometry was about -9.7 degrees/405.8 mm with 34 mm snap error. In the moved
trial, red stayed production-valid over about 333-433 mm and bearings from +17
to -22 degrees. `WRONG_SEAT` events while the chassis was manually rotated are
expected because gyro heading changes while the stationary harness retains its
synthetic x/y pose; do not tune seat snapping from those events.

The powered `log_87` miss occurred at the more extreme predicted seat-0 view
of about -28.8 degrees/299 mm. This lies near/beyond the complete-pillar
acquisition boundary and was not reached in `log_88`. Therefore red hue,
saturation, area, height, and range thresholds remain unchanged. The user
approved the small viewing-geometry fix: `seatComfortablyVisible()` now applies
the existing 3 degree look-FOV margin, so a NO_BLOB frame can prove a seat
clear only within about +/-24.4 degrees rather than +/-27.4. Coloured-blob
acquisition still uses the full configured camera window. The discovery
lookahead-target nudge cap is now 45 degrees rather than 40, enough to move the
logged -28.8 degree view inside the central gate. The 340 mm hold, nudge slew,
speed, path geometry, colour thresholds, and Pure Pursuit steering calculation
are unchanged. This is target shaping, not a steering overlay.

The IDE-managed `giga_r1_m7` build passed with 295576 bytes RAM and 363472
bytes flash. No firmware was uploaded. Next, upload only with consent and run
one CW lap with the same red seat-0 layout using `Y-1`. Expect one provisional
200 mm lap-1 injection because the following station is unresolved at that
time. Require confirmation, no contact/intervention, one complete passage
report, and one completed lap. If confirmation still fails, the stricter clear
gate must leave the station unresolved and the existing 340 mm hold must stop
the robot before contact.

`log_89.txt` did not reach the red seat-0 pillar, so it provides no obstacle
clearance or acquisition result. It stopped safely at an empty S3 station 0
with zero injections. The new target authority was exercised up to -43.5
degrees. The remaining right seat then reached -24.8 degrees/264 mm, only 0.4
degrees outside the +/-24.4 clear gate; at the next 200 ms status sample it was
-25.1 degrees/221 mm, below the validated 230 mm range. Thus no two consecutive
camera frames could prove it clear. The perception hold fired at 335 mm against
the configured 340 mm threshold. Maximum CTE was 78.0 mm and maximum heading
error was 20.2 degrees.

The user approved separating the empty-clear margin from the 3 degree
target-aiming margin. `OBSTACLE_DISCOVERY_CLEAR_FOV_MARGIN_DEG` is now 2.0,
so `seatComfortablyVisible()` uses about +/-25.4 degrees; target aiming retains
`OBSTACLE_LOOK_FOV_MARGIN_DEG=3.0` and the 45 degree cap. Camera acquisition is
unchanged. At 175 mm/s, the logged 264-to-230 mm interval is about 194 ms or
2.4 normal 79.6 ms frames, enough for the existing two-frame clear requirement,
while the failed log_87 view at -28.8 degrees remains excluded. The
IDE-managed `giga_r1_m7` build passed with 295576 bytes RAM and 363472 bytes
flash. No firmware was uploaded. Repeat the same one-lap red seat-0 CW layout
with `Y-1`; require confirmation and a safe pass, or a no-contact hold if it
still cannot resolve.

`log_90.txt` passed the firmware-side one-lap regression. The empty S3 station
0 that blocked `log_89` now cleared. Red seat 0 confirmed at the edge-view
approach, injected exactly once at the expected provisional 200 mm, and the
robot completed one CW lap with continuous measured motion. The passage report
gave PLAN pillar/wall minima 50.3/138.1 mm, ODOM pillar/wall minima -10.1/211.7
mm, and side-ToF pillar/opposite-wall clearance estimates 20.0/221.0 mm. The
negative ODOM value conflicts with the positive fresh ToF estimate and should
not be interpreted as contact by itself. Maximum CTE was 76.5 mm and maximum
heading error 32.3 degrees. The user subsequently confirmed about 20 mm
physical clearance and no contact, matching the ToF estimate, so `log_90` is
accepted.

`log_91.txt` is the three-lap follow-up. It confirmed red seat 0 once at the
edge-view approach, used the provisional 200 mm lap-1 path, built the optimized
path exactly once with seat 0 at 260 mm, and stopped after lap 3. All passage
windows completed and there was no measured-speed stall signature. Per-lap
PLAN/ODOM/ToF pillar clearances were 50.3/-10.6/7.0 mm on lap 1,
90.5/62.0/125.0 mm on lap 2, and 90.5/68.9/91.0 mm on lap 3. Corresponding ToF
opposite-wall clearance estimates were 227, 115, and 118 mm. Maximum CTE was
92.7 mm and maximum heading error 54.2 degrees. This validates later-lap route
selection and per-lap logging, subject to the user's physical observation:
lap 1's 7 mm ToF estimate is marginal, so ask whether `log_91` touched before
accepting it or choosing the next layout. The user confirmed no contact and
about 10 mm physical clearance on lap 1. This accepts the three-lap mechanism
and optimized route, but the provisional 200 mm lap-1 path remains below the
preferred roughly 30 mm robustness margin.

Lap 1 retained 227 mm ToF clearance to the opposite wall. The user approved a
new `OBSTACLE_EXTREME_UNRESOLVED_CLEARANCE_MM=220`, used only while an
outer-extreme seat's following station is unresolved. Confirmed extreme
adjacent pairs remain at the physically validated 200/210 mm after the first
member is clear, and isolated later laps remain 260 mm. Because the second
member's geometry is deferred until 100 mm past the first, the initial 220 mm
route protects the first pass and then rebuilds to 200/210 if the rare adjacent
layout is actually confirmed. Camera perception, speed, steering, and all
other geometry are unchanged. Geometry preflight checks selector output and
the strict ordering 200 < 220 < 260.

The IDE-managed `giga_r1_m7` build passed with 295576 bytes RAM and 363504
bytes flash. No firmware was uploaded. Next, repeat one CW lap—not three—in the
same red seat-0 layout with `Y-1`. Require a 220 mm injection, no contact, and
preferably about 20-30 mm or more physical/ToF pillar clearance. The unchanged
260 mm later-lap route does not need another immediate repetition.

`log_92.txt` completed the requested one-lap CW seat-0 test with one red
confirmation, one 220 mm injection, and no contact, but the user observed only
about 10 mm at the rear wheels after most of the robot had passed. The ToF
passage report corroborates this: left raw minimum 46 mm minus the 35 mm wheel
inset gives 11 mm pillar clearance. The route snapshot predicted 64.5 mm,
sampled odometry predicted 2.5 mm, and the opposite-wall ToF estimate retained
245 mm. Maximum CTE was 79.0 mm. Treat the physical and ToF agreement as the
tuning evidence; the route prediction does not include tracking error and the
odometry estimate is too uncertain to override a fresh side range.

The implemented next change keeps the provisional 220 mm route at peak
displacement for one additional 50 mm waypoint after the pillar, then uses the
existing exit taper. This specifically protects the rear-wheel phase rather
than increasing the complete detour. It applies only when an outer-extreme
pillar uses the unresolved-next-station 220 mm policy. Confirmed adjacent-pair
200/210 mm geometry, isolated later-lap 260 mm geometry, the approach taper,
speed, perception, and Pure Pursuit steering are unchanged. Geometry preflight
requires exactly one exit-hold waypoint and keeps it shorter than the taper.

The IDE-managed `giga_r1_m7` build passed with 295576 bytes RAM and 363616
bytes flash. No firmware was uploaded.

Next, upload only with user consent and repeat the same one-lap CW red seat-0
`Y-1` layout. Expect `clearance_mm=220`; require no contact and preferably
20-30 mm or more physical rear-wheel/ToF clearance. Do not spend a three-lap
run on this change because later-lap geometry is unchanged.

The user ran this build twice. `log_93.txt` missed confirmation but identifies
a precise timing race: the first production-valid red observation appeared at
-26.8 degrees/190 mm and recorded `VOTE:0`, then the unresolved-station limit
immediately aborted in the same control update. No second frame was allowed,
although normal confirmation requires two votes. `log_94.txt` acquired its
first vote earlier at -25.7 degrees/333 mm, confirmed on the following frame,
injected the 220 mm route, and completed the lap. The user described that pass
as perfect. Its PLAN/ODOM/ToF pillar minima were 64.5/14.1/44.0 mm and ToF
opposite-wall clearance was 199 mm. This accepts the one-waypoint route exit
hold; the remaining problem was perception-hold timing, not clearance.

The implemented fix introduces a 400 ms stationary grace at the existing
340 mm unresolved-station hold line. Entering it commands zero speed but does
not expose `obstacle_path_perception_blocked()` until the grace expires, so
normal camera processing can collect several ~80 ms frames. If a pillar
confirms or the station clears, the hold state resets and motion resumes with
the normal path. If it remains unresolved for 400 ms, the existing hard block
and live-test abort occur. This preserves the two-frame vote, camera/FOV/blob
thresholds, and safety distance; it never drives forward during the grace.
Logs distinguish `Perception hold`, `Perception hold resolved`, and
`Perception hold expired`.

The IDE-managed `giga_r1_m7` build passed with 295584 bytes RAM and 363992
bytes flash. No firmware was uploaded. Next, upload only with user consent and
run the same one-lap CW red seat-0 layout twice. Both must confirm and pass. A
late case should hold and resolve rather than expire; retain the already
accepted roughly >=20-30 mm physical/ToF rear-wheel margin.

`log_95.txt`, `log_96.txt`, and `log_97.txt` are three consecutive clean
one-lap repetitions of that CW red seat-0 layout. All confirmed and injected
seat 0 exactly once at 220 mm, completed the lap, and looked perfect to the
user. ToF pillar/wall clearance pairs were 51/201, 52/198, and 46/202 mm.
ODOM pillar minima were 13.8, 13.3, and 12.7 mm; PLAN remained 64.5 mm. Maximum
CTE was 87.7-87.9 mm and maximum heading error 33.6-37.1 degrees. No run entered
the perception hold because confirmation completed before the 340 mm line.
Accept the provisional exit shape and normal acquisition. Keep the 400 ms
fallback because these runs do not reproduce `log_93`'s late first vote.

No code change is warranted from logs 95-97. The next test should add distinct
coverage: mirror the single red outer-extreme case in CCW and run three laps
with `Y3`. Place red at CCW seat 0. Require one 220 mm lap-1 injection, the
validated-layout 260 mm route on laps 2-3, three complete passage reports, no
contact/intervention, and a controlled stop after exactly lap 3. This closes
the remaining one-complete-layout-per-direction prerequisite before broader
multi-pillar reliability work.

`log_98.txt` passed that three-lap CCW test. It confirmed and injected red seat
0 once, built the 260 mm optimized route after lap 1, emitted three complete
passage reports, and stopped after exactly three laps. The user judged it
acceptable but saw a close initial approach and a long/wide avoidance,
especially on later laps. Telemetry quantifies the tradeoff. Lap 1 at 220 mm
reported PLAN/ODOM/ToF pillar minima 48.1/-30.8/19 mm and 196 mm ToF wall
clearance. Laps 2/3 at 260 mm reported ToF pillar/wall pairs 96/94 and 109/92
mm. Maximum CTE was 100.8 mm and maximum heading error 58.8 degrees, without a
stall or abort.

The 19 mm first-pass result is slightly below the preferred 20-30 mm robustness
margin and has ample opposite-wall room. Therefore the provisional unresolved
outer-extreme clearance is now 230 mm rather than 220 mm. Its 50 mm exit hold
is unchanged. Do not shorten the 260 mm later-lap route from this one red test:
it is also the established protection for green rear-wheel clearance, and its
roughly 92-94 mm CCW wall margin remains safe. Revisit its duration during final
speed optimization after representative multi-pillar layouts pass.

The IDE-managed `giga_r1_m7` build passed with 295584 bytes RAM and 363992
bytes flash. No firmware was uploaded.

Next, upload only with user consent and repeat only one CCW lap with the same
red seat-0 placement using `Y1`. Expect a 230 mm
injection, no contact, and preferably >=25-30 mm physical/ToF clearance. Do not
repeat three laps because the 260 mm optimized route was not changed.

`log_99.txt` passed one CCW lap with the requested 230 mm injection, but the
user still saw only about 10 mm at the initial approach and the side-ToF
minimum remained 54 mm raw/19 mm wheel-clearance estimate. The opposite-wall
ToF estimate was 189 mm. PLAN predicted 53.0 mm and ODOM -39.3 mm. Red was
confirmed while the robot was still before the taper, so late detection does
not explain the unchanged physical minimum; increasing 220 to 230 alone did
not solve front-envelope tracking.

The implemented adjustment extends only the provisional outer-extreme approach
by one 50 mm waypoint and reaches peak displacement one waypoint earlier. In
combination with the already accepted one-waypoint exit hold, this makes a
short full-displacement plateau spanning the front-to-rear pass. It retains
230 mm nominal clearance. Confirmed 200/210 mm adjacent-pair and 260 mm later-
lap shapes, camera logic, speed, and Pure Pursuit steering remain unchanged.
Geometry preflight requires one approach-lead and one exit-hold waypoint, both
shorter than the eight-waypoint taper.

`log_99` also exercises the 400 ms perception fallback for the first time. The
next empty station reached `Perception hold at ... forward_mm=300`, resolved
before expiry, and the robot resumed and completed the lap. Accept the bounded
hold behavior; it preserved two-frame resolution without causing an abort.

The IDE-managed `giga_r1_m7` build passed with 295584 bytes RAM and 363992
bytes flash. No firmware was uploaded.

Next, upload only with user consent and repeat the same one-lap CCW red seat-0
`Y1` test. Require a 230 mm injection and at least about
25-30 mm physical/ToF clearance during both the initial approach and rear-wheel
exit. Laps 2-3 are unchanged and need not be repeated.

The user made two attempts. `log_100.txt` stopped before red at empty S1
station 0. The right seat reached 2 clear frames, but the left seat only briefly
entered the trusted view at 25.3 degrees/275 mm and stayed at zero frames in
the sampled telemetry. It then moved to 26.3 degrees/235 mm and outside the
current +/-25.4-degree clear gate. The 400 ms hold correctly stopped forward
motion but expired because stationary steering cannot rotate the camera and
the remaining seat was already below the validated 230 mm range. Do not extend
the grace timeout for this geometry failure.

`log_101.txt` cleared the same empty station and completed the one-lap red test.
The approach-lead change is successful: the user said clearance looked much
better, and ToF estimated 64 mm to the pillar with 163 mm to the opposite wall,
versus 19/189 mm in `log_99`. PLAN/ODOM pillar minima were 78.5/-14.8 mm. Accept
the 230 mm short-plateau route.

The implemented perception change reduces only
`OBSTACLE_DISCOVERY_CLEAR_FOV_MARGIN_DEG` from 2.0 to 1.0, widening trusted
empty-seat geometry from about +/-25.4 to +/-26.4 degrees. This remains just
inside the camera's validated complete-pillar view of approximately +/-26.6
degrees and excludes the known failed -28.8-degree view. Actual coloured-blob
acquisition stays unchanged, as do both two-frame requirements, nudge control,
speed, and Pure Pursuit. Geometry preflight enforces a positive clear margin
smaller than the separate 3-degree target-aiming margin.

The IDE-managed `giga_r1_m7` build passed with 295584 bytes RAM and 363992
bytes flash. No firmware was uploaded.

Next, upload only with user consent and run the same one-lap CCW red seat-0
`Y1` layout once. Require empty S1 station 0 to resolve
without hold expiry, one 230 mm red injection, and safe clearance. On success,
move to representative multi-pillar testing rather than repeat this layout.

`log_102.txt` passes the requested regression. Empty S1 station 0 cleared
without entering the perception hold. Red seat 0 confirmed and injected once
at 230 mm, all other stations cleared, and one CCW lap completed. PLAN/ODOM/ToF
pillar minima were 78.5/-6.1/69 mm; opposite-wall ToF clearance was 164 mm.
Maximum CTE was 93.6 mm and maximum heading error 51.6 degrees. Accept the
1-degree empty-clear margin and the complete provisional red route. No code
change or upload is needed for the next test.

Exact next test: retain the validated CCW start and use `Y1` for one lap. Place
red at S1 station 0 right (seat 6), green at S2 station 1 left (seat 15), and
red at S3 station 2 left (seat 23). Leave every other station empty. These
seats are in different sections and avoid adjacent-taper interaction while
covering station 0/1/2, red/green, right/left, outer-extreme, and moderate
routes. Expect exactly three injections: 230 mm at seats 6 and 15 while their
following stations remain unresolved, and 260 mm at moderate seat 23. Require
three complete passage reports, no contact/intervention or hold expiry, and a
completed lap. Analyze each pillar/wall ToF pair independently.

`log_103.txt` is truncated at about 17.4 seconds during the approach to S2
station 0 because the user likely removed the USB drive before the asynchronous
save completed. The user reports that the physical three-pillar run succeeded,
but the file contains only seat 6. That red pillar confirmed and injected at
230 mm; PLAN/ODOM/ToF pillar minima were 78.5/-15.6/68 mm and the ToF opposite-
wall clearance was 159 mm. There is no seat-15 or seat-23 confirmation/passage
report and no lap/result footer. Count this as a promising physical trial, not
a complete telemetry acceptance. Do not tune code from the truncation.

Next, repeat the exact same one-lap CCW seats 6/15/23 `Y1` layout without a
firmware upload. After inserting the USB drive for saving, keep it connected
while the RGB LED is red. Green is the successful-save indication; wait for
green to appear and then turn off before removing the drive. Require all three
injections, all three PLAN/ODOM/ToF passage groups, and the final completed-lap
`PASS` footer before advancing.

One policy detail exposed by the stationary log: seat 0 injected at 200 mm,
not 260 mm, because it targets an outer extreme while its following adjacent
station is unresolved. That provisional value preserves the possible adjacent
reversal. Once the adjacent station is known empty, the later-lap optimized
selector uses the isolated 260 mm value. Documentation must not describe the
lap-1 seat-0 route as 260 mm.
# Camera implementation notes

- See `CAMERA_ASYNC_BUFFERING.md` for the SDRAM double-buffered DMA design.
- See `CAMERA_24MHZ_DEVELOPMENT.md` for the accepted GC2145 24 MHz clock
  profile, measured reliability, rejected faster profile, and fallback.

## Adjacent extreme-pair log 83 and clearance diagnostics

`log_83` formally completed but is physically invalid. The user reported that
the robot drove into the red pillar and could continue only after the pillar
was moved; green avoidance worked very well. Red seat 6 used 160 mm and green
seat 9 used 210 mm. The full side-ToF reports estimated red pillar/wall wheel
clearances of -27/245 mm (left raw minimum 8 mm, right wall minimum 280 mm) and
green pillar/wall clearances of 57/155 mm (right minimum 92 mm, left wall
minimum 190 mm). During red contact, the left ToF stayed around 27-28 mm and
measured speed remained essentially zero while target speed and duty rose.

The key ordering is that green was confirmed and the full route rebuilt while
the robot was still passing red. The stronger green pre-taper overlaps the red
region and pulls upcoming Pure Pursuit points toward the red pillar. This
explains why increasing the second clearance fixed green but regressed red;
the values cannot be tuned independently when both additive tapers become
active immediately.

The implemented fix records green normally but defers only its geometry
injection until odometry progress is 100 mm past the earlier red seat. Until
then `livePath` contains the red-only 160 mm avoidance. It then rebuilds with
green at 210 mm, leaving roughly 400 mm before green. Expected logs include
`injection=DEFERRED` at confirmation and
`delayed_until_first_clear=yes` at activation. Pure Pursuit remains the sole
steering controller; there is no contact response or steering override.

Clearance telemetry was expanded for every confirmed pillar passage. Planned
route and sampled odometry minima use a conservative robot capsule with 70 mm
radius, axis from the rear axle to 60 mm forward, hence a 70 mm conservative
rear reach and the measured 130 mm front reach. Pillar clearance is to the
official 42.5 mm-radius movement circle. Wall geometry contains the four outer
wall segments and four inner square wall segments in the canonical field
frame. Reports identify the nearest outer face, inner face, or inner corner,
include the pose and nearest wall point, and print minima to every inner corner
in SW/SE/NE/NW order. The existing dual side-ToF report remains separate and
uses range minus the 35 mm sensor-to-steered-wheel inset. Never average plan,
odometry, and ToF numbers: each diagnoses a different error source.

The IDE-managed `giga_r1_m7` build passed with 294008 bytes RAM and 361136
bytes flash. No firmware was uploaded. Next, upload only with user consent and
repeat the exact CW section-1 red seat 6 followed by green seat 9 layout once.
Physical contact invalidates the run even if it later reports `PASS`. On a
safe run, compare `[CLEARANCE PLAN]`, `[CLEARANCE ODOM]`, and `[PILLAR TOF]`
for each seat, including the reported nearest wall feature and inner-corner
values, before selecting any further geometry change.
