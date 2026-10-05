# Connector diagnostic run and acceptance sequence

## Prepared next test: GREEN at the far/left start seat (2026-10-05)

Logs 433–435 show the first two start stations CLEAR, then GREEN at S0
station2 inner seat4 confirmed while the already preflighted parking connector
is active. The old code discarded that connector and demanded a fresh merge
from the advanced pose; all three runs stopped with `No safe merge candidate`.
The prepared M7 keeps the existing connector only for this exact CW case,
when its outgoing route prefix is unchanged and a fresh 2 mm swept rollout
from the measured pose clears the newly confirmed GREEN. The existing
steering, wall/pillar, 60 mm/15 degree handoff and total 500 mm encoder travel
limits remain in force. If a check fails, the existing replan/hold remains.

`simulation/replay_green_far_connector.py` replays the archived connector
geometry from logs 433–435 at four saved route points with assumed +/-10 mm
position and +/-2 degree heading variation. All 108 cases per log reached the
handoff without modeled wall, seat4 pillar, pink-rail, steering or remaining
500 mm travel failure. Log 434's maximum modeled remaining travel was 488 mm,
so its total live encoder limit needs particular attention. The logs do not
contain the exact pose at GREEN confirmation; this replay is an ideal model,
not proof that every physical approach works. Real pink placement and later
path following also require the drive test.

Next: upload the prepared M7 firmware, keep the single GREEN pillar at the
far/left start station and the remaining layout as in tests 433–435. Start CW
from the usual parking pose, with the laptop USB cable disconnected for the
drive. Watch the pink rails, the connector and the GREEN bypass. The log
should show `Retained after far GREEN seat=4`, `Complete` and the first-lap
stop; it must not show `No safe merge candidate`, tracking rejection or travel
limit. Stop manually if there is contact or an unsafe gap. Save the complete
log and report the location of any halt. After a safe first run, repeat three
times with normal small placement variation. This test does not validate the
separate RED near/right case.

## Prepared next test: GREEN at the middle start seat (2026-10-05)

Logs 430 and 432 correctly confirmed GREEN at S0/1 right seat2, then stopped
because live steering reached -42.02 degrees immediately before connector
handoff while the allowed magnitude is 42 degrees. The connector target was
pinned to its final waypoint. M7 now lets only a connector built for a
confirmed GREEN target station continue its fixed lookahead along the outgoing
route. The preflight rollout and live controller use the same target function;
the existing steering, footprint and 500 mm travel gates remain active. This
does not address the different false-RED/replan failure in log 431, nor GREEN
at the far start seat or RED at the near seat.

`python simulation/replay_green_middle_connector.py` replays both archived
connector geometries. At the actual last poses, the old target immediately
reproduces a steering rejection; continued lookahead reaches the handoff in
8/18 mm of ideal travel. A grid of +/-10 mm x/y, +/-2 degrees heading and
0.85/1.00/1.15 yaw response passes 81/81 cases per run from connector start
and 81/81 per run from the logged stop pose. Minimum modeled wall/pillar
clearances in the connector-start grid are 167.5/157.9 mm. These are model
assumptions, not measured physical margins. The replay also checks both
modeled parking-rail gap extremes; real pink placement still needs observation.
The M7-only PlatformIO build
succeeded; no firmware upload or robot test occurred for this revision.

Next powered test after the owner uploads the prepared M7: leave the other
pillars as in the single-start-pillar matrix, put only GREEN at the middle
start seat, use the normal CW parking start and drive without a connected
laptop cable. Watch the pink bay during exit/scout and the green bypass.
The log should show `route_lookahead=yes` on the connector preflight, then
`[PARK ENTRY CONNECTOR] Complete` and the expected first-lap stop. Stop
manually for physical contact or an unsafe gap; preserve the full USB log and
report the exact stop/contact location. A single success checks feasibility;
repeat at least three times with modest normal placement variation before
calling the case reliable. Do not use this revision to claim the other start
seat arrangements are repaired.

## Paused at owner's request (2026-09-28)

Owner corrected that green had been moved: the 20260927 green distance check
cannot serve as a600 mm reference or be paired with red. Red stationary capture
and diagnostics are archived in `simulation/evidence/camera_diagnostics/`.
All colour measurements are paused pending the owner's next command. No
distance/HSV/route changes from the invalid pairing; if resumed, repeat green
with measured camera-to-foot placement before comparison.

## Immediate next step: stationary camera check before more motion

Completed red and green A/B/A checks: red five fresh diagnostics valid; green
without neutral background five invalid width179, with Pappe four of five valid
width51..57, then without Pappe five invalid width179 again. Exposure changes
135->338->136 lines, so colour connectivity and automatic exposure effects must
be separated. Evidence in `simulation/evidence/camera_diagnostics/` with hashes.
Defer further powered testing. Next inspect an actual onboard image/colour mask
at this failing view; no verified image-export command currently exists. Use it
to choose a small image-processing correction, not weaker acquisition gates or
more recovery motions. Far-red CLEAR veto remains separately unresolved.

Update: stationary camshot export implemented, authorized M7 upload completed,
two consecutive CRC-valid images received and normal camera service resumes.
Actual image/mask confirms green pillar connects to background at ROI y80.
Current installed prepared binary73124b... (full hash in agent history), no
firmware readback. Archives/PNGs in camera_diagnostics evidence directory.
Next is a small segmentation correction validated on near/far/partial red and
green plus empty backgrounds; preserve raw rejected evidence for CLEAR. One
diagnostic ROI100 crop is not a safe global fix. Extra-arc driving remains deferred.

Offline while owner away: column-support candidate isolates green on all three
saved frames, unchanged production gates and selected-component foot. See
`simulation/CAMERA_PILLAR_SUPPORT_ANALYSIS.md`; six mask/geometry checks pass,
but a vertical background patch remains an explicit false-positive ambiguity.
No firmware change. First next physical samples: stationary green~600 mm,
red~600 mm and background with pillar removed; partial/edge/lighting samples
still needed before deployment. Filtered evidence must never replace raw CLEAR
veto. Installed firmware and current paused driving plan remain unchanged.

Owner prioritizes simpler/faster discovery; defer the extra-arc powered trial
below until a short camera check. Physical drive enable off, lowercase `c0`
starts existing stationary camera diagnostics. Same lighting, red then green
about400 mm from camera, about10 seconds each, pillar centred/fully in frame.
Record CAM CAL HSV/shape/production_valid and CAM PERF exposure/errors; centre
HSV samples one pixel, not an average of the detected blob. Then at the blocked
scan viewpoint collect10 seconds with unchanged setup and10 seconds without
only the far red, chassis unchanged. Restore setup. The fragment disappearing
would support a far-pillar explanation; logs alone do not prove onboard image
coverage. Colour thresholds remain unchanged pending measured evidence.
Extra arc adds3 seconds of travel plus settling/observation and is a fallback,
not the desired normal section-entry path. Seek one useful entry viewpoint and
two near-seat CLEAR decisions, then normal middle-station discovery.

## Current firmware/test: one additional reverse arc at section entry

Complete log398 contains two CW sessions. Both straight peeks and retraces pass,
but a rejected tiny red fragment overlaps the inner first-seat ray and vetoes
CLEAR. User confirms S1/0 and S1/1 empty, red only S1/2. A later red is a plausible
cause, not independently established from images; do not remove this veto.

M7 adds one optional90 mm reverse arc at60 mm/s and20-degree steering only when
the original section-entry peek leaves exactly one seat camera-CLEAR and the
other blocked by a rejected blob. It preflights all24 seats, walls, both capsules
and20 mm braking reserve with40 mm margin, physically settles steering300 ms,
observes stopped with existing evidence gates, retraces the measured arc, centres
and then completes the original straight return. Arc tracking limits15 mm/5 deg,
maximum heading change35 deg; arc return20 mm/3 deg; original straight3 deg and
15 mm limits remain. One extra attempt only, first discovery lap/section entry.
Unsafe motion locks immediately; unresolved evidence returns safely then holds.

Offline check `simulation/corner_extra_view_check.py` uses both recorded scan
poses and486 perturbed cases including0/10/20 mm coast. Minimum modeled wall
margin296.1 mm, pillar111.96 mm; no target-visibility failures. Near/far ray
separation at least3.25 degrees in this reported setup. These are model results,
not image proof, physical tracking, CCW or full-lap acceptance. The initial110 mm
candidate had48 visibility failures with coast and was shortened to90 mm.

Next: upload only the latest M7, leave this setup unchanged, run one CW trial.
Watch the initial straight reverse, optional short curved reverse, curved return,
straight return, red bypass and continuation after middle green. Stop manually
if unsafe. Record contacts, actual stop location and complete log. Expected new
markers: `Extra parallax scan`, `extra_origin`, `extra_scan`, `extra_returned`.
First-lap test mode remains enabled and should stop/save after one discovery lap.
If it holds, use its exact lock reason before changing any limits. A changed
layout, CCW and fully official start-section arrangement follow after this trial.

## Previous firmware/test: official section-layout inference

Logs396/397 pass both corner reverse/observe/return manoeuvres, then hold at
S2/2 despite confirmed middle greenS2/1. Figure8c of the official2026 rules
(page15,36 cards) excludes end pillars when the middle is occupied. M7 now
uses this inference on the normal lap route, gated by
`OBSTACLE_USE_OFFICIAL_SECTION_LAYOUT=true`. Two confirmed end pillars likewise
exclude the middle. Inference is dynamically recomputed from confirmed seats,
separate from camera-observed CLEAR; conflicting confirmations disable it.
Parking-exit/scout/connector discovery remains explicit and unchanged.
`[PATH LAYOUT]` prints the three-bit inferred-empty station mask.

Upload the latest M7, repeat one same-layout CW run. At confirmed middle green
expect inferred mask5 (station0/2), normal avoidance and continuation through
the final station rather than its old hold. Stop on unsafe behavior; preserve
full log and physical outcome. All normal unknown-position holds and the
section-entry-only reverse fallback remain. No new end-station reverse added.
The existing start section confirms green at station0 and red at station1,
which contradicts official layouts; its inferred mask must therefore be0.
For final full-lap acceptance use a wholly rule-valid layout (two signs at the
ends, or one sign at a chosen station), and revalidate the changed arrangement.
Disable the layout option for arbitrary practice or locally different rules.

## Previous firmware/test: bounded straight reverse corner view

Logs394/395 both abort after reverse starts with `heading/cross-track limit`.
Code inspection found `stop(true)` disables servo writes, so the original stopped
settle only set a logical zero without physically commanding it. The correction
now re-enables and calls `steer(0)` AFTER stop at arming and every stationary
phase; the original300 ms settle and all movement limits remain. New
`[CORNER VIEW ABORT]` records heading error, cross-track and encoder displacement.
Exact abort displacement/heading are absent from394/395; residual steering is
a code-supported explanation, not independently measured wheel alignment.
Next test is one same-layout CW repeat with this corrected M7 build.

User explicitly selected the reverse fallback. M7 now attempts one straight
peek per unresolved first station of a section during discovery lap1. At the
normal hold it first waits the existing800 ms stationary observation grace.
Only if still unresolved at section entry does it brake/centre steering for300 ms,
then select170..220 mm only when
the modeled endpoint sees both seats, and preflights the entire reverse sweep
plus20 mm braking reserve against walls and all24 legal seats, front/rear
capsules, margin40 mm. Speed is60 mm/s. Heading drift3 deg, cross-track15 mm,
wrong encoder direction5 mm, overshoot20 mm and motion timeout6 s lock motion.
After200 ms braking it collects fresh frame events stopped, observes at least
400 ms (up to1600 ms), and returns by measured reverse travel even if unresolved.
Return sweep is rechecked; after braking, XY return error must be<=20 mm and
heading must remain within3 deg. Only resolved evidence plus>=2 stopped frames
permits route resume. Any motion/preflight violation holds at the current pose;
unresolved after a safe return also holds. No repeated automatic attempts.
Progress/lap/ToF path correction is suspended during the manoeuvre. Existing
camera/color/empty gates remain; actual camera geometry is still unverified.

Next physical test:
1. Upload M7 from the prepared source; do not change M4 or the current layout.
2. Use the same parked CW start, automatic obstacle mode, wait BLUE, then enable.
3. Expect exit, first straight, corner hold, short straight reverse, stationary
   observation, forward retrace, then continuation only if the station resolves.
4. Be ready to disable if contact or unexpected motion occurs. If locked, leave
   stopped about2 seconds, disable normally and wait for USB saving to finish.
   Do not use serial O to force continuation or move the robot while enabled.
5. Return full log, log number, physical contacts/interventions, whether reverse
   and return occurred, and whether the first lap completed. Completion still
   stops/saves before final parking. No CCW acceptance is claimed.

`[CORNER VIEW]` logs build, transitions and lock reason; `[CORNER VIEW POSE]`
records origin/scan/return. Keep originals under the existing evidence policy.

## Previous software step: earlier corner approach alternatives

Priority correction after the user's camera-visibility observation: the trace
visibility flag is a projection from estimated pose and mounting constants,
not an image measurement. Before further trajectory changes, compare actual
onboard approach/hold frames with the predicted seat pixels and independent
pose/mounting measurements. All candidate searches share this projection;
their failures do not independently establish actual camera invisibility.

The user requested prioritizing an earlier forward viewpoint and considering
deferred coverage. `simulation/corner_forward_view_search.py` now checks sampled
constant-steering arcs from the recorded CW S1/0 approach poses in logs 392/393.
All 24 legal pillar seats remain occupied for collision checks, with conservative
front/rear capsules and 25 mm model margin. Assumed XY +/-10 mm, heading +/-2 deg,
yaw gain .85/1/1.15 and coast 0/10/20 mm produce no robust simultaneous-view
candidate. Best sampled candidates fail visibility in 72/243 and 61/243 cases,
despite positive swept clearances. Separate fixed stops on the same arc also
produce no robust candidate (up to 40 mm cumulative coast). Ideal gyro-heading
stops with +/-2 deg stop error and instant steering centring likewise fail.

This rejects these sampled simple arcs, not every forward route. Firmware remains
unchanged; there is no new build or upload and no new physical test requested.
Next design step: a route with independently planned position/heading observation
poses, checked through its subsequent merge into the live path. If deferring a
seat, preserve UNKNOWN and check the entire intervening sweep against both
possible pillar locations; do not simply suppress the 340 mm hold. Then compare
against a fully checked reverse/observe/retrace fallback. Preserve camera gates.
Reproduce with `corner_forward_view_search.py` on the two evidence logs; generated
reports stay under `local_workspace/corner-forward-view/`.

## Previous candidate: reverse first-corner view manoeuvre

Logs 392/393 prove the first unresolved corner seat never entered the calibrated
clear window during the recorded approach; at hold it is also closer than 230 mm.
Waiting longer cannot fix this viewpoint. The second seat is clear in 392 and
blocked by rejected green overlap in 393. Do not run further unchanged repeats.

`corner_reverse_view_search.py` finds 170 mm straight reverse as the shortest
common sampled view candidate passing all 54 assumed +/-10 mm XY / +/-2-degree
heading cases. It checks only field walls and the two corner seats with the
existing capsules. It is not robot firmware and does not prove usable images.

Before a new physical test, implement/validate the complete manoeuvre: check all
known/possible obstacles along reverse and return, settle steering, bound yaw,
travel/time/braking, observe stopped with the existing clear/colour gates, return
by measured travel and verify the pose. Retain a hold if resolution fails. Keep
the user's empty layout as test evidence, not hardcoded controller knowledge.

## Completed diagnostic step: first-curve perception hold (logs 392--393)

Logs 390/391 physically confirm CW exit and connector completion without contact,
but stop at unresolved S1 station 0 (global station 3), before the next section's
reported later red pillar. The first lap remains incomplete.

New M7 firmware adds `DISCOVERY_TRACE` using cached frame pose and observations;
motion, thresholds, 800 ms hold and one-lap stop remain unchanged. At most 32
periodic trace records plus eight reserved hold records per run, 200 ms minimum
periodic interval. Each seat field contains bearing, range, comfortably-visible,
rejected-raw-blob overlap, observation-allows-clear, clear-frame count, stored
clear state. Frame timestamp exposes observation age. Observation status is the
numeric enum in `include/obstacle_path.h`. Motor command precedes trace formatting.

Upload M7, keep the same setup and parked CW pose, wait BLUE and start with the
switch. One repeat suffices. If it holds in the first curve, leave it stopped for
about two seconds, disable normally and let USB save finish. Return the complete
log and contact/stop report. Do not move pillars, widen thresholds or force it
through the hold. Read per-seat trace to distinguish unavailable view, rejected
colour overlap and insufficient clear frames before designing a correction.

## Previous build: corrected tangent and one full lap (tested in logs 390--391)

The newest build joins the outgoing XY direction of the displaced live route,
instead of its retained baseline heading metadata. For CW logs 386/387 that
direction is 132.57 degrees rather than -180 degrees. Before motor enable,
lookahead candidates must also pass a 2 mm kinematic rollout up to the existing
500 mm travel limit, checking front/rear capsules against walls and confirmed/
guard pillars with 10 mm model clearance margin. The 42-degree steering and
60 mm / 15-degree handoff limits remain unchanged. This margin and ideal rollout
do not establish real pose/actuator accuracy.

`OBSTACLE_FIRST_LAP_TEST_ENABLED=true` requests one normal discovery lap and
stops/saves before final parking. Obstacle remains the automatic startup mode.
The previous diagnostic-only instructions below describe the earlier build.

1. User uploads the new M7 firmware; M4 remains unchanged. No upload by the agent.
2. Keep the photographed setup unchanged for the first CW regression, photograph
   the parked start and wait for BLUE readiness before enabling the start switch.
3. Let normal exit, scan/scout, connector and first-lap discovery proceed. Serial
   O is unnecessary. Stay ready to disable on contact or uncontrolled motion.
4. Successful completion prints `[OC] First-lap test complete - stopped; final
   parking skipped` and saves automatically. Wait until the USB save is complete;
   do not disable power during writing. A safety hold is a failed test, not a
   reason to force continuation with O.
5. Return the complete log, direction, contact/stall report and whether the
   first-lap stop occurred. After a clean first CW run repeat CW once; review
   these before CCW. A repeated hold needs one complete log before more tests.

Offline verification: both recorded CW start geometries converge under the
existing gates; 324 assumed cases (two starts, reflection, +/-10 mm XY,
/-2-degree heading, yaw gain 0.85/1/1.15) pass. Modeled minima are wall 148.5 mm
and pillar 88.2 mm. Reflections are synthetic, not actual CCW geometry replays;
the failed CCW logs do not contain complete path geometry. The model covers
connector motion, not the full first lap, camera discovery or physical contact.

Reproduce with IDE-managed Python:
`simulation/connector_transition_sim.py simulation/evidence/parking_exit_diagnostics/20260926_log_386_cw.txt simulation/evidence/parking_exit_diagnostics/20260926_log_387_cw.txt`.
Output stays in `local_workspace/connector-transition/rollout.json`.

## Historical diagnostic build (superseded for the next run)

Read-only geometry/tail telemetry is added; connector targeting, speed,
42-degree steering limit, 60 mm / 15-degree handoff, obstacle guards and travel
limit are unchanged. No firmware upload has been performed by the agent.

`CONNECTOR_CONFIG` identifies this source build. Successful preflight emits
connector points and at most 32 following route points while stopped. Final
four waypoint positions emit at most 32 periodic records at 100 ms intervals,
plus the rejection record regardless of the periodic budget. Motor stop occurs
before rejection formatting. Records use cached pose/geometry only.

These records are outside the parking-exit diagnostic budget. Worst-case
geometry/tail snapshot is roughly 20 KiB (64 connector points, 32 route points,
33 tracking records); replans can emit further snapshots. Check raw-log overflow
and physical timing in the returned run; no measured loop-time bound exists.

## Historical diagnostic procedure (completed in logs 386--389)

1. Photograph the current setup from above, including parking lot, inner wall,
   pillars and CW direction. Keep positions unchanged for this diagnostic repeat.
   The user's two-consecutive-pillar setup is a recognition/transition regression,
   not accepted competition-layout proof.
2. Upload the prepared M7 firmware with the IDE; retain existing M4. Do not run
   until upload succeeds. Normal O retains practice false, discovery true,
   discovery test-only false, five exit segments and the 85 mm scout. Final
   parking entry remains unarmed.
3. Start CW from the same parked pose using normal O, as in logs 383--385.
   Be ready to disable on contact, trapping or uncontrolled motion. Do not guide
   the moving robot.
4. Observe exit, scan, scout/retrace and connector, especially the last turn.
   If it holds again, do not restart O within the run. If it completes, observe
   stable route following and the next pillar pass before disabling.
5. Disable as usual and finish the normal USB save before removing the stick.
   Preserve the complete original. Report log number, contact/no contact,
   automatic hold/manual stop and whether the next pillar was passed; battery
   voltage if available. One run suffices if the same rejection repeats.

## Offline decision

Run `simulation/analyze_connector_tracking.py <repository-relative-log>` with
the IDE-managed Python. Output is ignored
`local_workspace/connector-tracking/report.json`.

The tool checks finite target/steering replay against rounded printed values,
separates endpoint distance and heading gates and compares continued-route
targets at recorded poses. An assumed +/-10 mm XY, +/-2-degree heading grid
contains 27 cases per sample; these are not measured bounds or probabilities.
Old logs 383--385 explicitly report missing geometry/tail pose.

Before implementing route continuation, require shared runtime/preflight
selection, route coverage, swept vehicle/wall/pillar checks, closed-loop replay
and mirrored cases. Candidate steering feasibility alone is insufficient.
Preserve the steering and endpoint gates; do not force route activation.

## Physical acceptance after a justified correction

1. Three nominal CW repeats on an identified official starting-section variant.
2. CW repeats with modest parked-position variation inside the parking lot,
   varying and documenting one factor at a time.
3. Single red, single green, clear station and permitted two-pillar variants,
   following official layout cards and parking-section relocation.
4. Mirror accepted cases in CCW after contact-free CW connector completion.

Require connector completion, stable route following, correct pillar side and
no contact. Unresolved scenes must retain controlled hold. This expands coverage;
it does not prove success for every physical disturbance from a single batch.
