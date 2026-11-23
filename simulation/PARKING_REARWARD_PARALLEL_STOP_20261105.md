# Rearward entry and stop without centring

User request: shift parking10mm toward the robot's rear, then clarification:
stop as soon as fully inside and parallel, without driving back to the middle.
Revision `20261105_rearward_10mm`, M7 only.

Capture local X270.26->260.26mm and planning reference60->50mm. Local+X
follows the parked heading, so CW/CCW mirror automatically. Legacy segment
endpoint references also shift10mm. Real marker coordinates, measured gap,
scan registration, lateral depth, speed and unparking remain unchanged.
The adaptive planner checks each new trajectory against the actual barriers;
the executed path may differ from a rigid translation when space is tight.
For the nominal247.5mm gap, capture is fieldX492.76mm CCW /219.74mm CW.
Practice setup: axle1.276cm beyond the nearby marker's inside face, rather
than2.276cm. See [setup](../docs/guides/PARKING_ENTRY_PRACTICE.md).

The reference-X is not a mandatory final position. During an entry movement,
a fully contained pose within the existing2deg alignment tolerance initiates
braking immediately, including before the current planned segment ends.
Straighten only after checking the neutral-wheel footprint. Keep the measured
standstill gate and actual servo-command settling before declaring completion.
An already acceptable stopped pose skips the next segment too. Remaining
segments toward the reference-X are not executed.

The former10mm rear/30mm front preference no longer forces repositioning of
a valid stationary robot. Longitudinal completion retains8mm model room at
both ends (5mm boundary+3mm registration allowance), plus neutral-wheel safety.
The early stop during motion additionally requires3mm beyond the ordinary
5mm containment margin on all sides. This avoids premature success at the
open edge with small scan/pose errors; it is not a centred target. Ordinary
stationary containment and candidate terminal checks retain their existing
5mm margin. Unsafe or not-yet-parallel poses still require correction.

Validation from actual production-source extraction:

- `check_final_parking_sequence.py`:396/396 parked,0 collisions or false
  containment claims;5/5 registration checks. Includes separate physical and
  estimated poses, scan timing and injected registration errors.
- `check_parking_adaptive_completion.py --brake-overrun`:84/84 parked,
  8 health checks; independent turn radii and0/1.5/3mm brake overrun.
  Maximum modeled travel358.599mm, maximum4 replans.
- `check_parking_parallel_stop.py`:16 cases plus mirrored capture checks.
  Accepts stationary positions at localX45/70/100mm without centring;
  rejects alignment alone outside the bay and a5deg yaw.
- `check_parking_steering_wait.py`: actual servo-age/settling, brake gates
  and CW/CCW verification regression.

Generated outputs remain under `local_workspace/`. Model checks exclude
unknown slip, gyro bias and real servo/transmission lag; physical validation
is pending. No upload or robot movement performed.
