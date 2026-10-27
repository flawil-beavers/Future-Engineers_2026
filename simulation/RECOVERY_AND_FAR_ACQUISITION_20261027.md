# Recovery correction and distant acquisition, 2026-10-27

User authorizes correction of remaining weaknesses and asks whether a block
may still be recognized and mapped from farther away. No new physical logs
are available. Preserve existing pending source work and separate the proven
actuator-state defect from unproven steering/visibility hypotheses.

## Delivered source correction

The prepared normal-route steering restoration already re-enables servo
writes at an accepted tracking commit after a gyro hold. Its source-backed
negative-control test still passes. The connector restoration remains intact.

`FP_SEGMENT_DRIVE` in `src/final_parking.cpp` now does the same after healthy
pose and complete short swept-motion checks have accepted the drive. A main
gyro hold calls `stop(false)`, which disables servo writes as well as driving;
the segment previously resumed the motor without explicitly clearing that
flag. The previous physical servo angle can persist, so this flag defect is
not proof of a physical steering error or of the reported initial refusal to
enter. Restoration does not bypass a rejected pose, clearance or gyro gate.
Approach/scan/capture already restore steering through `driveFieldLine`.

`check_parking_segment_sensor_resume.py` executes the actual parking state
machine at every entry segment, both directions: 42 checks pass for unhealthy
hold, healthy resume and invalid-pose rejection. Removing only the new servo
restoration reproduces disabled steering at the resumed motor command.
This uses ideal stopped actuators; no measured servo motion is claimed.

## Farther observations remain permitted

The 230--600 mm comfortable-view window governs clear/empty decisions and
the additional mapped-seat GREEN search. It is not a general range cap in
`obstacle_path_observe`. Normal primary RED/GREEN observations can still
confirm and dispatch map/route injection beyond 600 mm if image shape,
projected seat, rail-side ambiguity, upcoming-station relevance and consecutive
colour votes pass. Do not declare a distant station empty just because no
block was found there.

`check_far_obstacle_acquisition.py` extracts the actual primary observation,
camera projection, shape gate, nearest-seat and vote code. With synthetic
upright blobs at modeled forward depths about646 and700 mm, both colours and
both rows receive a first vote, then confirmation and injection dispatch.
Rail-midpoint projections and unrelated future stations remain rejected;
the map remains frozen after lap1. All48 checks pass. Final map/route mutation
is stubbed after dispatch: this is not a whole-route or real-image range test.

There is no arbitrary-distance promise. The existing minimum image foot100px
and ground model52/33600 imply approximately700 mm forward at the centred
image boundary (ray distance grows off axis); area/height/FOV and upcoming
route gates impose further limits. Farther raw colour may be visible while
being unsuitable for reliable mapping. No pixel/HSV/ambiguity/snap threshold
was loosened and the600 mm absence window was not widened. Physical camera
calibration has direct400/600 mm references, not held-out long-range proof.

## Scope and outstanding acceptance

Retain the CAD conversion, lap1 horizon and sensor freshness while investigating
their real perception interaction separately. The current evidence does not
justify replacing them with a new untested steering strategy or removing stop
guards. Previously prepared rollout optimizations and normal gyro/servo resume
fixes remain part of the working candidate. No new ToF sweep or extra physical
movement is requested; the user uploads personally.

The four missing CW originals are still required FIRST for the specific GREEN
stop and final parking refusal. This change does not claim to diagnose or fix
those unreturned events.

Verification: check_gyro_path_servo_resume.py PASS with negative control;
check_runtime_safety.py all groups PASS; check_green_seat_visibility.py archived
and synthetic foreground/background checks PASS; final-parking sequence396/396
and5registration checks PASS after the correction. IDE-managed incremental
M7-only build SUCCESS, RAM433928/523624,flash514616/786432, firmware SHA-256
5313b1d0ec4d288909a0a3431326eaf9cc9e55b18d10b04383efd4b9ff56454e. M4 unaffected and not built. No upload, commit or push.
Existing Serial macro warning remains; no new build failure.
