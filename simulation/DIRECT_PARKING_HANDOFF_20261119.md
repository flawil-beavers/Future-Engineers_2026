# Direct final-lap parking handoff, 19 November 2026

We use the learned inner or outer arrival route to prepare for parking before
the final corner. This revision reduces time spent handing over to the parking
controller and waiting for the steering to settle. The firmware reports the
parking identity as `20261119_direct_handoff`.

## Evidence and route

The connected source still contained the same six originals, with no new runs. They were copied/reverified before analysis; source-before/source-after/copy hashes agree in [verification manifest](evidence/log_intake/20261119_batch_3_6/direct_approach_source_verified.json). The stick was immediately released. Full canonical originals and hashes remain in the [parking evidence index](evidence/parking_exit_diagnostics/README.md). This is source/model validation, not a new physical acceptance run.

We derive the handoff point from the selected route. It is the first point
after the final corner where the whole car clears it. During laps 2 and 3, the
controller checks up to 12 samples per update and checks a 1,200 µs time budget
before each sample. That budget governs when the next sample can start. We
have yet to measure the time on the GIGA. The checks run in the background
while the lap controller continues to drive.

Each accepted candidate checks27 starting poses across X/Y +/-40mm and heading +/-5deg, using the existing complete approach collision checks, including both wheel-angle extremes, the longer steering nose, walls, pink rails and learned pillars. Existing direct-lane +/-40mm lateral checks remain. Earlier outward-shift candidates(-100,100,200mm) precede the previous350--650mm candidates; the learned bypass retains its500mm minimum. Only a fully checked candidate is retained. This improves route choice; it is not a proof of the globally shortest path.

Near the final corner of lap3 only, the route speed blends toward150mm/s using the existing180mm/s^2 planning deceleration and80mm reserve. Lap1 and lap2 speed commands remain unchanged. Final runout also retains the150mm/s cap.

CCW can transfer drive ownership in the same update without an added finish stop, provided actual arrival fits the prepared envelope, gyro is healthy, drive is enabled, commanded/profile and measured speed are forward and <=190mm/s, and the actual first30mm command sweep is clear. The active parking controller keeps ownership on subsequent updates; the ordinary finish brake cannot interrupt it. CW requires reversing relative to the completed lap and retains the measured brake before reversing. An unfinished, failed or mismatched preparation uses the existing stopped checked planning path.

The known rail scan and registration remain necessary before tight entry. Their650mm scan-start reference is unchanged. In the obstructed logged log2 layout, approach-to-scan time stays8.44s in the ideal model; do not claim every approach is shorter or completely without stops.

## Entry and waiting

Retain60mm/s normal entry. An80mm/s experiment with endpoint taper increased nominal model time11.46s to13.25s and mean14.265s to16.542s across24 cases. Another80mm/s variant failed4/24; an adjusted variant passed but remained slower. None is retained.

Instead, overlap a known next steering command with the already required brake. Before writing it, check all intermediate steering angles in <=5deg steps, radius scale +/-15%, and possible coast of at least10mm or measured brake travel+3mm. Skip overlap when coast exceeds20mm or any checked contour is unsafe. Motor stays in position hold. The next drive still waits for measured standstill and the full400ms since the actual servo write. Existing motion-clearance prediction, travel/time limits and recovery remain.

This overlap reduces the24-case mean entry time14.265s to13.648s; nominal11.46s to10.93s; worst20.20s to19.14s. Reproduce the comparison with `check_parking_measured_651_656.py` and its `--without-brake-servo-overlap` negative control. These synthetic times exclude real CPU planning cost, actuator lag and traction effects. Maximum modeled entry travel remains475.197mm. The final fully contained parallel pose still ends parking without an unnecessary longitudinal recentering move.

The requested fixed two-reverse-arc/straight recipe was also checked against the actual123mm width,98mm wheelbase,35mm rear and135mm steered front projection. Tested two-arc/diagonal-capture families did not clear the rail/wheel contour across the measured-gap cases. Keep the existing checked short-family search and complete adaptive fallback rather than force that recipe. The historical `check_parking_efficient_entry.py` still rejects its old <=2-arc assumption; it is not a passing acceptance check for the current footprint.

## Validation and limits

- `check_final_parking_approach.py --approach-only`:2970/2970 approaches pass.
- `check_direct_parking_handoff_20261119.py`:24 learned-map cases;20 certified preparations,12/12 CCW rolling handoffs. All12 execute the real outer dispatcher repeatedly through approach-to-scan without modeled collision or injected finish brake. Four CW middle-RED variants cannot certify the broad cache envelope and retain stopped planning. Cache mismatch/overspeed controls reject rolling. This fixture uses the real finish stop guard but a synthetic path-completion trigger, not a complete driven lap3 simulation.
- `check_parking_brake_servo_overlap.py`:400ms servo age retained;150ms brake overlap demonstrated; unsafe next steering rejected without moving the motor.
- `check_parking_measured_651_656.py`:24/24 park, both directions, gaps248.9/249.6mm, radius factors1/1.15/1.5 and yaw +/-1deg, no modeled collision.
- `check_final_parking_braking.py`:102/102 pass. `check_parking_steering_wait.py`:servo-age and measured-brake gates pass.
- `check_final_parking_sequence.py --quick`:3/3 park; registration5/5 pass. `check_parking_recovery_handoff.py`:6/6 pass.
- Discovery recovery25/25 and lap1-to-lap2 handoff6/6 pass after the dispatcher change.

Previous limitations remain: six232.5mm narrow-gap/large-radius stress cases safely abort; log4's wrong visible colour has no established image-level cause. A corrupt/missing sensor reference, repeated invalid rail measurement or genuinely unsafe route can still stop parking. Do not claim universal completion or disable those checks.

Next: normal M7 upload by the tester, then ordinary CW/CCW runs. Save/copy/hash full originals first. Check physical rear/front wheel clearance, rolling CCW transition, actual planning pauses and parallel completion. Preserve passive parking-exit diagnostics; no dedicated sensing sweep or added robot movement is requested.

Final IDE-managed M7 build SUCCESS: RAM452176/523624B; flash601184/786432B; firmware.bin604736B, SHA256`72a62c4160e38c9d6b87d32497b11fb68fec8b9b2fabfc1386806b2442060099`. M4 unchanged/not built. Existing Serial macro and unused-helper warnings remain. No upload, reset, movement or extra commit.
