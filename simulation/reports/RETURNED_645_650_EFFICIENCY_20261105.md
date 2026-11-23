# Returned645-650: later-lap speed and parking approach

Only the latest six supplied files are assessed. Originals and SHA-256 are
indexed in [evidence](../evidence/parking_exit_diagnostics/README.md).
All source/copy hashes verified before analysis; unplug notice sent immediately.
650 ends mid-line at36864bytes and is explicitly archived as an excerpt.

## Evidence

| Log | Direction | Observed result |
| --- | --- | --- |
| 645 | CW | Manual pause during lap1; no complete parking assessment |
| 646 | CW | Three laps and reported parking completion |
| 647 | CW | Three laps and reported parking completion |
| 648 | CCW | Three laps, then `scan_alignment_limit` |
| 649 | CCW | Manual pause during the last exit arc |
| 650 | CCW | Incomplete file during the last exit arc |

All diagnostic headers identify Nov2_2026_17_43_41,servo center80,exit80.
646-648 identify path20261102_lap_handoff and parking20261102_reuse_steer_settle.
They precede the rearward10mm/parallel-stop changes; no binary readback exists.
Parking completion is an onboard estimate, not proof of no physical contact.

Bounded LATER_TRACK command samples:646 lap2 median465.4mm/s,lap3 median452.6;
647446.85/452.1;648433.0/417.65. These are neither measured wheel speeds nor
complete time-weighted coverage. Maximum request remains520mm/s.

Handoff calculation actually blocks233.240/242.398/299.733ms. The controller
does not run inside this synchronous build; the reported jerk has a credible
timing mechanism, although these files do not independently measure the jerk.

CW parking starts approximately54s/90s before reported completion. Its initial
wall correction is+6.8/-27.8mm, followed by an out-and-back wall alignment and
long scan. Final scan registers longitudinal corrections-58.2/-66.2mm. Thus the
stored post-lap position cannot safely replace actual marker registration.

648's initial wall correction is+79.4mm. The controller then saturates at42deg
while requesting60mm/s; position changes only a few mm/s in parts of this
phase.649 likewise measures about3-14mm/s against an80mm/s last-arc request.
This is a real drive/traction/load discrepancy, not just a software speed cap.
Logs alone cannot distinguish friction, supply, slip or actuator effects.

## Changes

- Later laps use the physical curvature/braking speed profile without also
  inheriting the lap1 pointwise discovery-speed heuristic. The520mm/s ceiling,
  .8m/s2 lateral budget, speed/curvature tolerances, response distance, finite
  acceleration/deceleration and runtime inward-wall/curvature caps remain.
  Candidate route ranking keeps its former profile so geometry is preserved.
- Cache immutable seat/seam smoothing weights once per build, rather than
  recalculating them in each of up to46 relaxation passes.22 historical/new
  comparisons are bitwise identical. This removes redundant computation;
  total MCU handoff latency and the physical jerk still need a returned log.
  The planner remains synchronous, with768bytes static scratch storage to avoid increasing nested stack use.
- Direct-lane approach also considers x350/400/450 outward shifts, accepting
  only the complete checked route including+/-40mm Y uncertainty. Learned
  pillar-bypass approaches retain x500 or later. Nose-reserve checking is now
  consistent with entry, so the earlier turn cannot spend that steering room.
- Keep the validated oblique lane/corner acquisition. On the cleared scan/
  capture line, use measured Ackermann pursuit with a horizon that grows with
  lateral error, reducing sustained full lock. Geometry and command preflight
  use the same controller. Actual marker sampling remains60mm/s to retain
  slow-ToF/narrow-marker coverage.
- Approach70->100mm/s; wall alignment60->80; registered capture60->80.
  Entry arcs remain60 (tight mode40), including prior10mm rearward shift and
  stop when contained/parallel without returning to a reference-X.
- Exit strokes1-3 remain80mm/s; longer arcs4/5 now100mm/s. All distances,
  gyro gates, footprint checks, braking/reversal/settling and observer fallback
  remain. The header adds `long_arc_speed=100` to identify actual requests.

These request increases do not prove that a loaded wheel will attain them.
If measured motion remains near3mm/s, the drive discrepancy still needs work;
the robot must not substitute encoder travel for a verified safe pose.

## Validation

-6573 dynamic cases (2191 at10/20/50ms):all PASS; maximum lateral acceleration
 .678063m/s2, minimum modeled pillar clearance31.9203mm.
-2807 route/passing/state cases PASS;2970 parking-approach variants PASS.
-396 full parking sequences+5 registration checks PASS, including independent
 physical/estimated position and30/100ms ToF timing.
-84 adaptive radius/brake-overrun cases+8 health cases PASS.
-54 logged alignment-anchor variants PASS (646/647/648, lateral/yaw changes,
 independent radius1/1.15). Longest ideal alignment7.96s,max steering34deg.
 These exclude the real load/supply/slip effect; no physical time guarantee.
-2304 existing exit-geometry variations+10 brake/settling checks PASS.
 Geometry tests are speed-independent;10 extracted-command cases separately
 verify first3 strokes80 and last2 arcs100. Powered braking remains pending.
-6 mirrored handoff/speed cases PASS;22 cache cases bitwise identical.

M7 IDE build SUCCESS:RAM460024/523624,flash573576/786432,binary577128bytes,
SHA2567f2228b54ca4c734c9d90f9f9fbeb62deceb413b5f8f8ddb68136cf013076d4c.
Path20261105_physical_lap_speeds,parking20261105_efficient_alignment.
M4 unchanged/not built; no upload or motion performed.

Reproduce summaries with `analyze_returned_645_650.py`. The generic exit analyzer
can process645-649; its parser rejects650's cut final sample, so650 must not be
passed as a complete input. Generated reports remain in `local_workspace/`.
Next normal CW/CCW runs: secure originals first, verify build/revisions, compare
actual exit speed/settled overrun, approach X, wall correction/alignment duration,
later-lap command/physical response and handoff `planning_us`. No dedicated
ToF sweep or additional test movement is requested.
