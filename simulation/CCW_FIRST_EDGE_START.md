# CCW first-edge start candidate (2026-10-06)

## Received evidence

Complete originals461-465 and SHA256 rows are under
`simulation/evidence/parking_exit_diagnostics/`. User confirmed layout A was
unchanged:461/462 autonomous start holds;463/464 right-wheel GREEN contact at
S3 entry prevented lap1 completion;465 light contact only in lap1, then normal
laps2/3. Binary was not read back. Build markers in the evidence README.

461 rejected measured settled scan59.7/-1209.8/29.9 against parking/hidden-seat
geometry;462 rejected the full scan sweep from130.5/-1221.0/0.1. The old
330-338mm reverse finds the second pink piece's edge;0mm extra continuation
does not remove that search. Colour recognition is not the cause of those holds.

## New normal O sequence

1. Unpark unchanged. Use the initialized rear/outer-wall seed and exit odometry.
2. Confirm the first pink marker, then fresh wall frames. Check first-piece
   exit-edge footprint against field x480 with the existing50mm correction bound.
3. Freeze that observed edge/wall data. Continue gyro-held reverse only until
   corrected rear-axle x360; apply the accepted existing x/y corrections at stop.
4. Explicitly identify this reference to the path module. Its84mm camera arc
   uses the existing physical scan radius; sweep, hidden-seat, capsule, servo
   rollout and connector guards stay active. Observe only the front station.
5. Defer the two behind-start stations to their ordinary later approach.

If the first edge is absent/ambiguous, keep the established second-edge search
and55mm arc. O3/CHECK_ALL and reverse-localization-only tests retain that path.
No reference guess or bypass of collision checks is introduced.

Do not tune x and arc independently: x240/55mm collides with the parking model
at lower-y poses; x360/55mm lacks enough yaw for front GREEN. x360/82mm still
fails9 perturbed GREEN cases. The chosen360/84 pair passes4158 combined cases
(2970 legacy+1188 first-edge candidates), all11 official/empty start layouts,
actual batch y/heading spread with -8/+5mm x, +/-5mm y and +/-1deg heading.
Legacy return33 and reverse-prefix135 checks also pass. First-edge state test
executes production ToF selection, target, freshness, frozen reference and
O3/invalid-geometry fallback. These are ideal calculations, not motor/vision
or measured first-edge pose-error validation.

## GREEN/GREEN discovery corner

Once both adjoining signs are injected, reuse the existing radial corner join
in lap1 instead of additive straight tapers. Stored observations remain excluded.
Nearby measured pre-confirmation pose463 gives modeled pillar reserve63.9mm
before and108.4mm after; the missing exact injection pose prevents claiming
an exact reconstruction. New logs include injection pose and rounded-corner ID.
Later-lap geometry and1.50 recorded speed factor retain their previous values.

## Physical acceptance

After final M7 upload: unchanged A,2-3fresh CCW full-three-lap trials, cable-free.
Verify first-edge acceptance, final corrected/reference pose and connector
completion. Watch S3-entry GREEN right-wheel clearance in lap1. Compare later
laps separately. No contact required for acceptance. If a guard holds, inspect
its measured pose/reason rather than removing the guard. Archive/copy all raw
logs before further analysis so removable media can be reused.


2026-10-06 follow-up466-470: later laps additionally use calibrated CAD servo
conversion, rectangular-wall ToF references and280mm pursuit horizon.
See `LATER_LAPS_IMPLEMENTATION.md` for findings/limits. First-edge360/84mm
start and injected-pair discovery geometry are unchanged. Physical next batch
is unchanged A CW first, then CCW,2-3runs each; no claim of hardware acceptance.
