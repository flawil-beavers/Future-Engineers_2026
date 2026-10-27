# Committed first-lap / exit audit, 2026-10-27

## Corrected scope

The user clarified that deterioration already occurred before the current
uncommitted work. This audit therefore compares committed history only:
`689b3c3` (2026-10-08 later-lap refinement) through `94b2e0d` (2026-10-18 HEAD).
The immediately preceding `718d310` (2026-10-06) was also checked for exit
changes through that baseline. Current staged/working firmware changes are
excluded. No source was restored, edited, built for hardware or uploaded.

The exact binary of the user's last successful physical test is not identified
by a Git commit or installed-flash readback. This is a source-history audit,
not proof of which individual change caused a particular physical failure.

## Findings

| Area | Committed change | First-lap / exit effect |
| --- | --- | --- |
| Five exit segments | No geometry or speed change in the comparison | Remain 20/25/20/90 mm plus gyro-terminated final alignment; model155 mm, speed80. `updateParkingExit` matches after removal of one detailed-telemetry prefix. |
| Initial observation path | `buildParkingEntryPath` unchanged | Base observation geometry retained, but subsequent connector acceptance/tracking changed. |
| Steering conversion | `94b2e0d` replaces first-lap bicycle-angle command with existing CAD/Ackermann inverse, including connector rollout | Changes physical steering commands in lap1 and the post-exit join. It is not restricted to laps2/3. |
| First-lap lookahead | `94b2e0d` adds cap280 mm instead of the previous upper330 mm | Changes normal first-lap tracking; corner multiplier remains0.65. |
| Connector preflight | `94b2e0d` uses integer CAD steering propagation and removes the duplicate exact-Hermite-pose steering gate | Accepted connector/horizon/merge selection can differ even though the base observation path builder is unchanged. Runtime steering limits remain. |
| Gyro and ToF | `e2d7027` changes freshness, transport validity and recovery semantics | Applies while exiting and in lap1 as well as later laps; stale gyro reports now prohibit driving. |
| Motor/position/switch | `e2d7027` adds atomic encoder snapshot, changes enable debounce and retains sub-threshold odometry accumulation | Shared runtime behaviour changes; not isolated later-lap optimization. |
| Telemetry | Inspector-related commits add bounded runtime logging/hooks | Additional execution work exists; its timing effect is not established by this audit. Detailed telemetry defaults off. |
| Later-lap route/speed optimization | Larger optional smoothing and factor2.0 instead of1.5 in `94b2e0d` | Path selected after first-lap completion; speed and dynamic limits gated by completedLaps>0. No direct activation during initial exit/normal lap1 was found. |

Exit steering/distance implementation also remains unchanged from `718d310`
through `689b3c3`; the intervening obstacle-module edit suppresses vision debug
during final parking. Camera ground calibration remains78/24000 through HEAD;
the current pending52/33600 change is explicitly outside this audit.

## Quantified steering change

Laptop calculation uses the unchanged CAD lookup and old100 mm bicycle formula.
Runtime truncates servo commands to integer degrees.

| Requested radius | Old servo command | New servo command | Old CAD radius with integer command | New CAD radius with integer command |
| --- | ---: | ---: | ---: | ---: |
| 500 mm | 11.31 deg | 14.52 deg | 666.43 mm | 519.56 mm |
| 300 mm | 18.43 deg | 23.35 deg | 398.68 mm | 305.18 mm |
| 200 mm | 26.57 deg | 32.93 deg | 265.60 mm | 207.31 mm |

The new conversion is internally consistent with the CAD model, but it clearly
changes the first-lap vehicle response. Previous model checks did not validate
real servo response, grip, pose bias or acquisition timing on the current field.
Do not label it physically validated merely because frozen-map replay passes.

## Reproduced committed recovery gap

A laptop actuator-state replay extracts `stop`, `set_speed`, `set_steering`,
`steer` and the main gyro-health branch from HEAD via `git show`, using the
committed host actuator fixture. It exercises a normal tracking commit, with
no special mode transition or lap-boundary restoration:

1. Unhealthy gyro calls `stop(false)`, disabling motor and servo writes.
2. Healthy gyro permits the route again. Its normal `set_steering` / `set_speed`
   commit can enable the motor without restoring servo writes.
3. Observed host state: motor_enabled1, servo_disabled1, target_speed260,
   actual servo80 (initial centre), expected110 for requested+30.

This gap pre-exists the new freshness check; `e2d7027` adds another condition
that can enter the gyro hold, increasing its relevance. The conditional fault
is reproduced; its occurrence in any specific unreturned run is not proven.
The currently pending restoration fix was not applied to the audited source.
Generated C++, executable and outputs remain under ignored
`local_workspace/committed_first_lap_audit_20261027/`.

## Engineering conclusion / next steps

The committed interval was **not** confined to laps2/3. Initial exit segment
geometry stayed the same, but first-lap steering, post-exit connector tracking
and shared sensor/motor recovery changed. These are stronger investigation
targets for an early failure than the correctly gated later-lap speed factor.

Keep current pending work intact. Identify the actual pre-pending test firmware
from archived logs/build evidence, then separate gyro-hold/servo-resume events
from first-lap tracking-limit and discovery stops. Treat the already prepared
recovery correction and the CAD first-lap conversion as separate hypotheses.
No wholesale rollback of sensor freshness/safety or later-lap geometry is
justified by the available evidence. No new physical test or firmware upload
was performed in this audit.

## Why these measures were introduced (follow-up, 2026-10-27)

Checked against the original committed reviews and implementation diffs, not
inferred solely from the later audit summary:

| Measure | Recorded engineering reason | What that evidence does not establish |
| --- | --- | --- |
| CAD/Ackermann conversion in lap1 and connector | The prior controller sent a bicycle wheel angle as a servo command despite the linkage ratio. At requested R500, integer command11 models R666; the corrected command14 models R520. Sharing the existing inverse removes that model inconsistency. | Real steering response on the field and camera recognition during the changed approach were not measured. Agreement between controller and the same CAD plant is conditional on the plant being accurate. |
| Lap1 lookahead cap330 to280 mm | The calibrated frozen-map replay cut obstacle bypasses too closely through corners; the cap retained a shorter target horizon and improved modeled clearance. | This also changes heading corrections, camera bearing, station visibility and available observation time. The cited full-route matrix explicitly excludes acquisition/discovery timing. |
| Connector acceptance based on integer-servo rollout | The previous ideal-curvature propagation did not reproduce actual integer commands. A duplicate gate on exact Hermite poses could reject a feasible tracked trajectory. Nominal footprint checks and full tracked rollout remain. | Acceptance, merge selection and computation can change. A collision-free ideal rollout is not proof of timely execution or successful physical joining. |
| Gyro/ToF freshness and transport handling | Stuck interrupt, unrelated/invalid gyro events and stale/bus-failed ranges must not look like current heading or a clear gap. Valid gyro reports are required within200ms; ToF distinguishes failure from a valid no-return frame. | The new semantics can expose long loop gaps and cause holds. The already reproduced motor-resume/servo-write gap was not covered by successful nominal geometry tests. |
| Atomic encoder and accumulated odometry | Avoid torn ISR snapshots and loss of individually small motion increments. Scale and wheel-direction interpretation remain unchanged. | Slight pose changes can affect projection/association. No individual recognition failure is attributed to this without corresponding evidence. |
| Camera validation/quality diagnostics | Clear old detections on invalid frames and record brightness from sparse pixels already read. | Additional computation was not physically timed. These quality numbers are not new runtime colour-rejection thresholds. |

Original rationale and limitations are in FIRST_LAP_GEOMETRY_REVIEW.md and
RUNTIME_SAFETY_REVIEW.md as committed in94b2e0d/e2d7027. The geometry correction
was separately authorized after the first-lap audit, according to the handoff;
it was not solely a side effect of the lap2/3 speed request.

In this committed interval, no change to HSV thresholds, silhouette/colour
votes, supported camera frame rate, exposure/gain or ground model78/24000 was
found. Invalid input now clears old results instead of retaining detections;
valid supported320x240 images continue through the existing classification.
The separate pending camera calibration/fallback/capture changes are outside
this comparison and must not be used as its explanation.

Conclusion: the motivations address real source/model inconsistencies, but
physical perception acceptance after the changed steering/lookahead and runtime
recovery was missing. Worse driving-time recognition is plausible even with
unchanged colour thresholds: changed heading/position changes which seats are
visible and where observations project; holds/recovery and long computation
can alter observation opportunities. This is a mechanism to investigate, not
proof of the cause of the four unavailable CW logs. The prepared servo-resume
fix addresses a reproduced source defect; the CAD/horizon and visibility issue
remains a separate hypothesis. No wholesale rollback, threshold change, firmware
edit, new build or upload was made for this explanatory follow-up.
