# Connector and first-lap handoff

Current focus consolidated on 2026-11-17. The active physical sequence is the
[Obstacle Challenge checklist](OBSTACLE_CHALLENGE_TEST_PLAN.md). Earlier
batch-specific procedures are preserved in the
[historical connector guide](../history/CONNECTOR_NEXT_TEST.md).

## Current behavior

Normal `O` runs exit parking, complete three laps and attempt final parking.
The source disables first-lap-only, three-lap-only and final-parking-only test
flags. Older instructions to stop after a lap or before parking describe
historical builds. Confirm the installed firmware rather than inferring it
from a diagnostic timestamp alone.

The latest path revision is `20261111_checked_ccw_view`. Its limited extra
CCW first-lap aiming applies after the existing protected bypass and requires
the checked quantized steering/body envelope. It changes neither CW aiming
nor the protected bypass release. The
[view report](../../simulation/LAP1_CHECKED_CCW_VIEW_20261111.md) records its model
coverage and limitations.

The [retry recheck](../../simulation/LAP1_PARKING_RECHECK_20261111.md) describes
braking and renewed observation after bounded drift or rejected view paths.
New connections require current-pose preflight. An unknown object or a
rejected projection is not evidence that a seat is empty.

## Review focus in the next normal returned runs

- Separate parking-exit completion from scan, connector, first-lap acquisition
  and later-lap outcomes. The exit diagnostics do not provide a complete
  independent trajectory through all later phases.
- Compare raw observation, assigned seat, vote history, clearance checks,
  recovery reason and accepted rejoin. Use physical reports for contact.
- Check fresh frame availability and any pause around handoff planning.
  Planning latency and physical jerk remain separate from geometric feasibility.
- For CCW, inspect whether the targeted seat gains useful early frames while
  protected passing behavior remains intact. Modeled visibility alone does
  not establish successful colour/height recognition.
- Preserve whole-vehicle passing side, possible-pillar clearance and the
  existing safety gates. Do not revive old speed increases or disable guards
  to reproduce an archived test sequence.

Use normal CW/CCW driving and saving. Preserve the passive exit/ToF diagnostics;
no dedicated sensor sweep or extra maneuver is requested. Follow AGENTS.md for
immediate original-log copying, hash verification and evidence indexing.
The [current handoff](../../AGENT_DOCUMENTATION.md) lists active limitations and
the latest recorded build identity.
