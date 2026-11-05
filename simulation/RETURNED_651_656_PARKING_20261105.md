# Returned651-656: measured parking envelope and shorter entry

Originals are tracked unchanged in [evidence](evidence/parking_exit_diagnostics/README.md),
with per-file byte counts and SHA-256. Both USB batches were copied first,
source-before/source-after/copy verified, and unplug permission reported immediately.
`analyze_returned_651_656.py` regenerates summaries under ignored
`local_workspace/returned-651-656/`; the existing exit analyzer writes its outputs
under `local_workspace/parking-exit-analysis/20261105_651_656/`.

## What the returned runs establish

All six files identify buildNov5_2026_20_16_44, exit80/long_arc100.
Completed navigation runs identify path20261105_physical_lap_speeds and
parking20261105_efficient_alignment.652 is user-interrupted;653 was reported
slippery. Neither is evidence of a normal software failure.655/656 are reported
nearly perfect CCW, including good space use and shallow physical final pose.
The user explicitly permits further CCW improvements.

| Run | Finding |
| --- | --- |
| 651 CW | Three laps; estimated parking complete. Initial quick search1364ms, later broad search3595ms. Final estimated localX/Y86.8/116.4mm. Physical parking reported poor/incomplete. |
| 654 CW | Three laps; first steering arc has substantially larger observed radius, clamped1.5. Partial pose local87.4/206.8/60.7deg. Replanning ends entry_no_collision_free_plan. |
| 655 CCW | Parking complete; measured gap247.3mm, quick search1309ms, multiple radius/tail updates. |
| 656 CCW | Parking complete; gap247.4mm, quick search1321ms, multiple radius/tail updates. |

Wall registration in655/656 changes lateral pose by48.1/62.4mm. Keep actual
marker/wall registration: knowing the nominal final route position is insufficient.
Handoff planning remains202-243ms in this batch; its physical jerk/timing issue
is separate from this parking change and remains open. No camera colour or
first-lap acquisition changes were made in this session.

## Measured geometry and implementation

New user measurements: physical axle spacing98mm, rear35mm, front25mm straight
or37mm at full steering, straight total width123mm. Rear-axle front extent is
therefore123mm straight and135mm at full steering; total longitudinal extent
158/170mm. The calibrated Ackermann wheelbase127mm remains unchanged.

The prior base polygon reached only about125mm at the front. Its additional
16mm nose preference disappeared in tight1mm recovery. The new135mm steering
contour is part of the physical model at every margin. The remaining4mm front
preference is additional. Wheels use their actual LUT angles; the measured
front reach is conservatively enclosed, including the original rear tyre sweep.
Default polygons for exit/lap callers are unchanged and compared bitwise.

A diagonal car does not have gap minus straight length available to move.
The calculation projects body and wheel polygons at each pose against both
measured rails and the wall, including stationary steering changes and complete
paths. Full steering widens the tyre envelope as well as increasing front reach.
No fixed centimetre allowance replaces these checks.

Capture/scan axle line moves294.6->271.5mm from the wall:10mm between a straight
wheel and the200mm rail-tip plane. Preferred final axleY128.5mm gives
200-128.5-123/2=10mm inside the open longitudinal edge. Deeper checked recovery
alternatives remain available; a valid contained parallel pose stops without
longitudinal centring. Final stationary/candidate containment now uses a matching
extra1mm beyond the5mm boundary margin; moving early stop retains8mm total.
The existing8mm longitudinal completion reserve remains.

Initial straight alternatives include rearward20/32/40mm, accepted only with
the complete subsequent path checked. Normal registration planning includes an
extra1.5mm per polygon/piece even in tight mode, fading in over20mm so a current
pose can escape. Partial recovery retains the measured physical contour.

Normal short/analytic sequences run first. Only after they fail, a stopped,
bounded multi-point search tries both driving directions, steering extremes,
straight/angle-alignment strokes and actual free travel at both ends. It uses
512 static nodes, at most512 expansions/65000 charged safety samples and16
compressed segments. Candidates receive the normal1mm full-path/radius-envelope
preflight. Reused remaining paths retain their own finalX, not a new centring
request. The extra scratch consumes20480B; no nested search stack allocation.
This is a finite fallback, not a guarantee of recovery from every physical pose.

Approach120 vs100mm/s; alignment/capture100 vs80mm/s. Marker sampling60,
entry60/tight40, brake/servo/health gates unchanged. Scan start650 vs680mm;
early outward candidates still need the full footprint and lateral-uncertainty
preflight. Exit and laps retain their previously successful behaviour.

## Validation and delivery

Production-source host checks:

- 396/396 complete sequences plus5 registration cases, independent real/estimated poses.
- 2970/2970 learned-route approaches.
- 84/84 adaptive brake-overrun cases plus8 gyro/servo health cases.
- 24/24 new measured-envelope cases, mirrored directions, measured gaps248.9/249.6mm,
  initial yaw+/-1deg and independent actual radius1/1.15/1.5; no physical-envelope collision.
- 16 parallel-stop cases; actual servo-age/brake verification regressions.
- New envelope assertions include full135mm nose even at1mm margin and unchanged
  default geometry. Generated artifacts remain under ignored local_workspace/.

The new normal-entry plant takes approximately11.4-12.5s at radius1/1.15 and
uses about307-334mm travel. Radius1.5 recovery requires more manoeuvres, up to
about21.3s/495mm in this matrix. These are model durations including configured
movement/brake/servo time, excluding MCU planning computation and real loaded-drive
behaviour; they are not promised physical run times. The new MCU search duration,
actual tyre contour, traction and final clearances require normal returned runs.

IDE M7 buildSUCCESS, RAM480696/523624B (42928B remaining), flash576320/786432B.
Binary579872B, SHA256
`5dbbf880f26137561d8108501900e6063d57fd9466e6f591f6a4a8dbd98f2844`.
Firmware parking revision20261105_measured_parking. M4 unchanged/not built.
No firmware upload, reset, serial connection or robot motion performed.

Next: user uploads M7 and performs normal CW/CCW runs, saves normally. Secure
originals first. Compare approach duration, scan registration, requested/actual
curvature and any bounded-recovery message; confirm front clearance and shallow
physical finish. Preserve current exit/lap behaviour. No dedicated sensor sweep.
