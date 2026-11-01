# Returned603 and isolated entry practice

## Evidence

Complete saved originals602/603 copied FIRST from USB, source-before/source-after
and copy SHA-256 checked; immediate unplug notice sent before history/analysis.
Portable files and full hashes are indexed in
[evidence README](evidence/parking_exit_diagnostics/README.md).
602 is mixed and ends with OPEN_CHALLENGE completion; excluded as requested.
603 is the one available CW obstacle run. User reports two runs, one saved;
the missing run cannot be reconstructed. Physical report: single outer-middle
GREEN opposite the start was assigned first outer seat; parking stopped.

603 records path20261031_view_recovery, exit20261031_frame_bracket and
parking20261031_brake_completion. PARK_DIAG_CONFIG Oct30_20:04:33 is a
translation-unit build, not a complete binary identity. Whole installed binary
not read back. Exit observer93frames/1rejected/0overwritten, import0: improved
timestamp bracketing captured frames, but no physical early-seat import proven.
Normal observation fallback remained in use.

## GREEN association

At65781--66073ms the logged GREEN projection uses about188--191mm range despite
foot144--146; it commits outer end13 at65858ms. Official layout then infers
middle station empty. The exact camera pixels/bounds are not archived, but
these ranges match the170mm forward placeholder used when a component touches
image X<=2 or X>=317, divided by bearing cosine. This placeholder was allowed
to establish a map seat. It is an avoidance estimate, not measured depth.

Reconstructed edge-clipped components at logged rounded pose(-708,958,36),
centres32/34/36, foot144 reproduce wrong end13 confirmation in production
acquisition without the fix. This is a matching mechanism, not original-pixel
proof or independent pose truth.

Primary acquisition now leaves clipped-depth association unknown, logs
edge_clipped_depth_pending, and keeps the component available to existing
occupancy/close-obstacle/view recovery. Targeted GREEN and buffered exit import
also exclude clipped placeholder depth from seat votes. HSV, measured-distance
fit, avoidance placeholder, camera speed and ordinary uncropped far acquisition
are unchanged. Uncropped middle-seat observations still confirm. Far acquisition
at646/700mm retains all52 checks. This does not guarantee every later erroneous
end assignment or middle inference is eliminated.

## Parking stop

603 registers gap247.7mm, capture(211.1,-1201.5,180.5), first entry arc105.4mm.
It ends near(326.2,-1257.2,122.8), measured radius factor1.213 and settled brake
lead0.12mm. Pose deviation triggers replan. A five-segment tight-margin plan is
accepted, followed by manual pause at190287ms. There is no final parking abort
or completed record. Last periodic trace168383ms precedes that search; duration
is not explicitly logged, so the approximately22-second silent interval cannot
be treated as an independently timed MCU planner measurement. Initial planning
also produces a logged3008.8ms main-loop interval. Expensive stopped search is
consistent with the visible wait; the unsaved second run remains unknown.

Keep the same search families, budgets, braking, radius adaptation, positive
margins, gyro polling and final gates. Reduce repeated work with exact sufficient
south-strip capsule wall bounds; fall back to the existing full wall test
outside that strip. Pillar threshold uses squared distance; every confirmed
pillar remains checked. Skip rail SAT only when an already tested separating
X axis proves no overlap. Differential host checks compare previous full
wall/rail tests against these shortcuts. Log search_ms on accepted plans so
normal returned runs can measure actual MCU duration. No physical speed-up
or resolution of every parking stall is claimed yet.

## Standalone practice

[German setup sheet](PARKING_ENTRY_PRACTICE_DE.md) defines the surveyed capture
pose:247.5mm free rail gap, rear axle294.6mm from outer wall and22.76mm beyond
the near inner face in vehicle-front direction, parallel heading. The separate
MODE_FINAL_PARKING_PRACTICE starts directly in the existing capture-brake/entry
pipeline, clears old course observations, and uses the same planner/completion.
No camera initialization, laps, exit, long approach or dual-marker scan.
The tester must set the bay/pose; this mode does not measure their placement.

Current config boots to entry practice, default CW. Serial park cw/park ccw
selects direction only while drive disabled. Pause cancels the practice origin;
reposition and explicitly restart/reset, rather than rebase a partial entry.
Successful parking retains HOLD and normal one-time save. Serial O selects the
ordinary challenge until reset; restore MODE_OBSTACLE_CHALLENGE in config.h for
permanent course startup. New enum value appended; M4 interface unchanged.

## Checks

- New603 regression12checks; without clipped-depth gate confirms wrong end13.
- Wall shortcut1440 and rail shortcut12168 differential poses pass.
- Entry-only practice4complete CW/CCW cases at radius1.00/1.15, no modeled contact;
  the two mirrored rounded603 post-brake anchors also produce a safe replan.
- Existing396full parking +5registration,84brake-overrun +8health,36tight,
  102braking,42sensor-resume +negative control pass.
- Existing18returned association/start,52far,636observer +12handover,
  14disputed GREEN rows/54third views/27first transitions/18capture variations
  and associated negative controls pass.

Host geometry shares production polygons and ideal sensor/actuator adapters.
No independent wheel sweep survey, actuator lag, full unknown pose drift,
original camera replay or physical parking confirmation. Current source and
binary identity/build result are recorded in AGENT_DOCUMENTATION.md.

IDE-managed M7 SUCCESS:RAM453704/523624,flash552736/786432,binary556288bytes,
SHA256 `ddca8118fd4796a5a78be7f6d01b8016cf652da9f061253d677502a19db982dd`.
Only existing Serial-macro warnings. Path20261031_clipped_depth,
parking20261031_entry_practice; exit frame_bracket unchanged. M4 not built;
no upload, powered movement, commit or push.

Next: upload M7 only, run the defined CW/CCW entry practice and save normally.
Secure originals FIRST; record direction/start placement, contact/manual pause,
search_ms, settled_lead_mm, measured radius, replan progression and actual
whole-body containment with neutral wheels. Normal course confirmation remains
separate. No dedicated ToF fan sweep or extra sensor motions requested.
