# Efficient parking entry - 2026-11-01

Parking revision:20261101_efficient_entry. M7 only; the normal obstacle mode,
route, approach/exit, camera and compact logging remain unchanged.

## Implemented behavior

- Try a fully swept two-reverse-arc entry before the multi-point recipe.
  This shortcut uses the ordinary reserve, including the existing nose reserve
  and planning uncertainty allowance. It never lowers the margin just to fit
  two arcs. Otherwise retain the checked multi-point families.
- Rank quick recipe improvements by travel, segment count and direction changes
  within at most1500 further geometry steps after the first safe recipe.
  The first safe recipe is retained if no better one fits that bounded budget.
- Recompute remaining arc distances, the first diagonal straight and final
  centring from the settled pose/measured radius. A fresh complete1mm sweep is
  mandatory. This often avoids restarting the expensive general search.
- Calculate usable forward and rearward neutral-wheel travel from the actual
  pose, measured gap and complete body/rail geometry at1mm intervals. Subtract
  at least2mm, or the measured brake lead when larger. Include the safe room
  limits and half-room positions in fallback families in addition to prior
  candidates. Do not assume6cm strokes or a fixed10mm usable gap. Physical
  steering changes and the whole following path are still checked separately.
- General search budget is20000 rather than60000 geometry steps per margin
  level, divided between ordinary analytic families and checked escape search.
  This bounds expensive work more tightly; it is not a hard MCU time deadline.
- The former eight-replans lifetime limit is now a consecutive-no-progress
  limit. Net improvement over3mm in distance-to-goal plus equivalent turning
  arc length resets that counter. An improving entry can make a ninth correction.
  Unproductive repeated poses and absence of a complete safe plan still stop.

The good final goal and actual geometry remain60/127mm, rear30mm, front125mm,
straight width123mm. No extra rearward10mm preference. Brake300ms and steering
settle400ms are retained because no measured actuator data justifies reducing
those physical waits. Most delay reduction targets computation and needless
correction cycles. No blind drive or unvalidated short prefix is introduced.

## Evidence and tests

Original629-634 logs remain unchanged and indexed in
[evidence README](evidence/parking_exit_diagnostics/README.md).
They predate this change; no current hardware run has been received.

At local269.2/295.9/-0.5deg and measured gap245.2mm, comparison with the prior
bounded-tail source gives:

| Nominal plan | Segments | Arcs | Direction changes | Travel |
| --- | ---: | ---: | ---: | ---: |
| Prior |7|4|3|338.396mm|
| New |5|2|1|332.487mm|

Gap251.7 retains its7/4/3 recipe; gap262.5 retains5/4/2 at the same probe poses.
Fewer bends are not forced into a pose where the safe shortcut does not fit.
A trial that selected the two-arc entry with reduced clearance failed the
independent pose-error tests and was rejected. Released shortcut retains the
ordinary reserve. Final full-sequence checks:396/396, no modeled collisions.

Other passing checks:350adaptive,84brake-overrun,48returned632-634,6returned631,
36tight,12balanced628,6prior-tail,24returned620/621, registration/gyro health.
The631 cases now complete with0 general replans at all three tested actual
radii1.159/1.20/1.235, mirrored both directions. New regression has14 free-travel,
recipe-count and progress-limit checks plus36 complete reduced-arc runs with
actual radius1/1.136/1.235 (planner initially1.0, unmeasured),
yaw0/+1.5/-1.5 and overrun0/1.5mm. No modeled contact.
Host kinematics do not establish physical contact-free operation or MCU latency.

## Build and next step

IDE-managed giga_r1_m7 buildSUCCESS. RAM453824/523624;
flash566400/786432. firmware.bin569952bytes, SHA-256
`0ea50490d45dd1fc03dd71cbc36de350db5dc0319718e63f72e8fcb0218801ed`.
M4 unchanged/not built; no upload, robot motion, commit or push performed.

Upload M7, run normal CW/CCW and save normally. Secure original logs first.
Compare planner search_ms, number of arcs/direction changes, refresh messages,
actual front/rear room and any contact. Broad search can remain necessary;
no fixed maximum physical parking duration or universal completion is claimed.
Preserve parking-exit ToF/brake/reversal/pose diagnostics; no dedicated sweep.
