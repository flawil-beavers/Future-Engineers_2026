# Final-parking scope and robustness audit (2026-10-18)

The controller supports the normal post-lap handover in both CW and CCW.
It does not establish successful parking from every field position and heading.
An accepted modeled approach, a complete marker registration and a checked
entry pose are required. A safe rejection is different from a successful park.

## Supported handover and restrictions

- The complete vehicle leaves the last corner after lap 3; `finalCornerVehicleClear`
  checks all body/wheel polygons inside the start straight. The challenge applies
  its 300 ms finish brake before handing over to final parking.
- Inside the start straight, parking follows the recorded pillar bypass toward
  canonical +X. CCW drives forward; CW reverses with its west-facing heading.
  The passed turn sign selects these two orientations. It does not infer an
  arbitrary vehicle heading or build a route across the entire field.
- Before x500, the recorded route target must be within 150 mm, with candidate
  route points restricted to the south/start-section region. Later, the controller
  extends the cleared straight instead of entering the next corner. The outward
  shift at x500/550/600/650 is selected by a complete swept preflight. An invalid
  starting pose or failed preflight stops the attempt.
- Fresh, consistent outside-wall evidence establishes lateral reference.
  The scan requires wall / first piece / gap / second piece / wall, including
  two confirming fresh frames per transition. Both measured faces, gap and
  wall reference must agree before bounded registration can be applied.
- Capture requires +/-5 mm X/Y and +/-1 degree estimated heading. The seven
  parking segments use the measured gap, a complete swept entry preflight,
  runtime clearance checks, bounded travel/time and endpoint gates. Completion
  additionally requires containment, estimated heading within 2 degrees and
  low commanded-profile/measured speed.
- The isolated practice command explicitly resets estimated position to a
  configured starting pose. Its physical placement must match that pose; it is
  not a general-purpose park-from-here command.

The scan's +/-250 mm X-registration allowance is a late measurement allowance.
It is not a guarantee that an approach starting with an unknown 250 mm position
error is safe. Correct obstacle mapping and sufficient initial pose accuracy
remain prerequisites, as already documented for logs 506/507.

## Corrected protection defects

1. `finalFootprintContained()` previously checked boundary inequalities without
   first rejecting non-finite X/Y. With NaN coordinates these inequalities can
   all be false while a valid heading still passes. A negative control reproduced
   acceptance of NaN X. All pose components are now checked explicitly. Initial
   preflight and final verification also use the existing motion-health gate,
   including gyro health, before proceeding.
2. When no cached ToF object passed hardware/filter acceptance, `parkingOuterRange`
   previously fell back to `selected_raw_distance_mm`. A negative control with
   a positive diagnostic fallback and no accepted object reproduced acceptance.
   Such a frame now returns -1 and cannot establish a wall or marker. Accepted
   mixed wall/marker frames continue selecting the nearest accepted object.
   The normal sensor producer usually selects only accepted raw objects itself;
   the fix makes that quality requirement explicit in this consumer too.
3. Initial handover and the approach/scan/capture/segment brake phases previously
   relied on a fixed dwell alone. A state-machine negative control reproduced
   continuation, use of a moving wall reference or an unrelated abort when
   measured speed remained +/-20 mm/s or was NaN after that dwell. They now
   require finite measured speed <=5 mm/s before proceeding. The existing
   300 ms brake dwell remains; an additional wait is bounded at 2,500 ms and
   aborts with `brake_not_settled` if stopping cannot be established. The initial
   handover retains the challenge's existing hold target and adds a wait only
   when measured motion persists. Gyro-pause time is excluded from this wait.
4. Motor position HOLD preserves `current_speed` from the completed lap;
   `set_speed` changes the target without resetting that ramp. Even after the
   chassis has stopped, starting parking directly could inherit a 520 mm/s
   profile, particularly problematic when CW parking initially reverses.
   The handover now clears the drive profile with `stop(false)` after measured
   standstill, before starting the checked low-speed approach. A negative
   control without that reset fails the two direction-specific handover tests.

These corrections retain entry segment distances, steering, speed, discovery,
unparking, sensor scheduling and log budgets. Brake transitions may wait longer
when measured motion persists. The main-loop gyro gate remains
active. No automatic pose/backlash compensation was introduced.

## Reproducible audit

`simulation/check_final_parking_robustness.py` imports the reusable production
fixture from `check_final_parking_sequence.py`. It supplies independent actual
and estimated poses and actual barrier coordinates, including true bay gaps
242.5/247.5/252.5 mm. Physical collision checks use the actual gap rather than
the controller's currently assumed/measured gap, with 1 mm motion substeps.
Field/pillar checks use the conservative existing capsule; pink-piece checks
use body/wheel polygons. Sensor returns are an ideal binary cone model.

The matrix includes known arrival offsets, arbitrary absolute arrival headings,
other field sections, unknown estimator errors, missing/stale/repeated/empty
or rejected sensing, one missing piece and inconsistent bay geometry. It counts
successful contained parking, stopped rejection and unsafe outcomes separately.
Full generated cases, reasons and source fixtures remain ignored under
`local_workspace/final-parking-robustness/`.
The runner uses four independent host processes by default, with disjoint cases
and persisted per-process input/output files. `--workers 1` selects a serial
run; slow machines may need more wall time. The final four-process run completed
the entire matrix successfully after a serial repetition exceeded its timeout.

Run with the IDE-managed Python and host g++ on PATH after refreshing the shared
fixture using `simulation/check_later_laps.py`:

```
python simulation/check_final_parking_sequence.py
python simulation/check_final_parking_approach.py
python simulation/check_final_parking_robustness.py
python simulation/check_final_parking_braking.py
```

The original regressions still pass: five registration tests, 396/396 complete
sequences and 2,970 approaches. Of 162 perturbed entry preflights, 156 accept and
the same six tight cases reject; all nominal entries accept. These are model
results, not proof of physical parking reliability.

The final IDE-managed `giga_r1_m7` incremental build succeeds: RAM
433800/523624 bytes, flash 507792/786432 bytes. Firmware SHA-256:
`d72f96b8fbd60376143ba7937fb51de854c8eaa34796b3624297d58ce202f551`.
M4 is unaffected and was not rebuilt. Existing compiler warnings remain.
No upload, physical test batch, commit or push was performed.

The expanded matrix has 1,540 cases, with all counted as collision-free,
correctly contained completion or stopped rejection under the stated ideal model:

| Case group | Cases | Parked | Stopped rejection |
| --- | ---: | ---: | ---: |
| Known arrival: 11 start configurations, both directions, three true gaps, five +/-40 mm offsets, three headings (-10/0/+10 degrees) | 990 | 990 | 0 |
| Unknown initial estimator error: +/-40 mm X, +/-20 mm Y, or +/-3 degrees heading, one error at a time | 132 | 88 | 44 |
| Arbitrary absolute south/start-region pose: five X, three Y and twelve headings, both turn signs | 360 | 112 | 248 |
| Other field sections, four headings and both turn signs | 40 | 0 | 40 |
| Missing, stale, repeated, empty or rejected ToF; either piece absent | 16 | 0 | 16 |
| Inconsistent true gap, 280 mm | 2 | 0 | 2 |

The empty start configuration is diagnostic; the other ten are official inner-row
arrangements. The 990 arrival cases change the actual pose and give that same pose
to the estimator. They do not represent 40 mm of unknown localization error.
Unknown-error cases keep actual and estimated poses separate; not every one can
park. None of the modeled perturbation sets is an established physical error bound.

Another 46 checks reject non-finite pose components and stop/preserve the state
on gyro loss at each of the 14 phases in both directions, including initial
handover and final verification. Five ToF checks cover rejection and nearest
accepted-object selection. A further 66 braking checks verify held phases,
bounded timeout, finite low-speed acceptance and recovery after rolling stops.
They also verify clearing the stale lap-speed ramp in both directions. Their
motor adapter retains the profile in HOLD and changes only the target on
`set_speed`, matching the relevant real motor semantics.
The earlier time-only controller fails all 60 corresponding rolling/invalid-speed
negative-control checks. Full-sequence models stop instantaneously in their
hardware stub; the separate braking tests supply nonzero measured speed explicitly.

Actual tyre slip, braking coast, servo latency, optical reflections, obstacle
map errors and independently unknown pose bias are not validated by ideal
models. A successful estimator-based containment check is not an independent
measurement of the real chassis position. Normal current-firmware CW/CCW runs
through all three laps and parking, with normal saving and physical reports,
remain required. No dedicated ToF sweep, extra motion or pause is requested.
