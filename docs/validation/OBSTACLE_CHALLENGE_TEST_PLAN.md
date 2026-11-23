# Obstacle Challenge validation checklist

Current guidance consolidated on 2026-11-17 from the source and latest
[engineering handoff](../../AGENT_DOCUMENTATION.md). This checklist replaces the
batch-specific procedures in the [historical plan](../history/OBSTACLE_CHALLENGE_TEST_PLAN.md).
Read the relevant linked review before investigating a failure.

## Before a normal run

- Establish the exact firmware being uploaded and retain its identity.
  Source currently enables the full exit, three-lap and final-parking sequence.
  The first-lap-only, three-lap-only and final-parking-only flags are false.
- Confirm Surprise Rule participation before the attempt. Its build flag
  currently defaults to enabled. Follow [the rules reference](../competition/WRO_2026_RULES.md)
  and [height implementation report](../../simulation/reports/SURPRISE_RULE_IMPLEMENTATION_20261111.md).
- Use the actual field layout and measured parking geometry. Record changes
  to hardware, lighting, placement or surface that affect comparisons.
- Use the established disable procedure, keep the stop control reachable and
  remove the drive USB cable for normal powered runs. Stop on contact or unsafe
  approach. Follow the IDE-managed, affected-core build instructions in AGENTS.md.

## Active validation sequence

The next evidence is normal CW and CCW runs with unchanged hardware and normal
save behavior. No dedicated ToF sweep, extra motion, scan pause or special save
step is required. A software/model pass does not establish physical acceptance.

| Phase | Check in the returned evidence and physical report |
| --- | --- |
| Parking exit | Rear reference, segment/braking diagnostics, localization result and actual wall/rail clearance. Keep exit failures separate from later connector failures. |
| First-lap acquisition | Fresh images and votes, correct seat assignment, re-observation after bounded drift, and a checked rejoin. Unknown height must not consume a useful fallback vote in the same frame. |
| CCW view correction | Earlier usable view at the targeted unresolved seat, no extra aiming during the protected bypass, and no new contact. Preserve existing motion/clearance gates. |
| Three laps | Correct passing side at every confirmed sign, correct lap counting and continuous learned-route behavior. Recorded poses are estimates, not independent clearance measurements. |
| Final parking | Actual marker/wall registration, first-command progress, bounded recovery, physical front/rail clearance, complete containment, parallel pose and verified stop. |
| Surprise Rule, when declared | One stored modified-seat identity, accepted unusual height and original-colour passing on all laps. Do not treat synthetic height tests as physical acceptance. |

## Open limitations and supporting reports

- [First-lap retry and parking recheck](../../simulation/reports/LAP1_PARKING_RECHECK_20261111.md).
  The bounded planner still fails at the 232.5 mm gap in the current approach
  matrix. Model results at larger gaps do not remove this limitation.
- [Checked CCW aiming](../../simulation/reports/LAP1_CHECKED_CCW_VIEW_20261111.md).
  Modeled visibility improvements require real camera and driving evidence.
- [Measured parking envelope](../../simulation/reports/RETURNED_651_656_PARKING_20261105.md).
  Keep physical reports and known user/surface exclusions attached to each run.
- [Surprise Rule implementation](../../simulation/reports/SURPRISE_RULE_IMPLEMENTATION_20261111.md).
  Real 50 mm and 150 mm height acceptance remains open. New reference images
  need approval under the camera registry workflow.
- [Parking diagnostics plan](PARKING_POSE_IMPLEMENTATION_PLAN.md).
  Backlash and absolute pose accuracy remain unestablished. Keep passive sensing
  analysis separate from new control or compensation proposals.

## Receiving and reviewing results

1. Copy complete original logs immediately, unchanged, and verify SHA-256 before
   analysis. Tell the user the test USB stick can be unplugged immediately after
   verification, as required by AGENTS.md.
2. Archive portable evidence with firmware identity, completeness, direction and
   physical outcome in the [evidence index](../../simulation/evidence/parking_exit_diagnostics/README.md).
   An excerpt must remain explicitly labeled. Keep generated analysis local.
3. Compare each failure with the phase and firmware that produced it. Preserve
   physical reports even when the estimated completion status disagrees.
4. Record concise durable findings and exact next steps in AGENT_DOCUMENTATION.md.
   A normal success needs only the log, result and next step. Detailed failures
   and reusable analysis belong in their linked review reports.

Use [CONNECTOR_NEXT_TEST.md](CONNECTOR_NEXT_TEST.md) for the connector-specific
review focus. Historical run counts, test flags and build hashes remain in the
archive and their individual reports, rather than becoming active instructions.
