# Later surprise-challenge preparation

## Open task: starting parking bay on the other side

User request recorded 2026-10-06: examine a starting parking bay on the other
side and prepare that variant later. This is **not implemented** or enabled in
official `O`. `O3` changes obstacle-position checking; it does not already
provide an alternate parking location.

Before implementation:

- Establish the exact other location allowed by the surprise task.
- Derive coordinates for both pink pieces, gap, side/rear ToF rays and pose seed.
- Reassign ahead/behind signs separately for CW/CCW, including deferred bypass.
- Calculate complete exit, observation, connector, braking and final parking
  sweeps with the physical body/wheels and pose tolerances.
- Check camera coverage, lap counting and final parking for both directions.
- Add a clearly labelled independent selection after validation.

Current southern-bay coordinates and direction-specific body guards must be
transformed together; flipping the turn sign alone is insufficient. Continue
standard-task validation first. See `simulation/CCW_START_LAYOUT_REVIEW.md`.
