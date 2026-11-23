# Lap-1 active unresolved-station search

## Problem

During the first mapping lap, an unresolved station already triggered a slow
approach, an 800 ms stationary evidence window, a swept straight reverse and
one swept 90 mm parallax reverse. If the station was still neither confirmed
empty nor assigned a colour, the recovery returned and entered a permanent
motor lock. This made a recoverable camera-view problem end the run even though
the robot still had checked space for another viewing angle.

The returned 582--593 evidence remains the basis for perception handling. No
newer USB log was present for this change. The user's supplied photographs are
geometry context only; no colour or blob threshold is changed here.

## Correction

Revision `20261030_active_lap1_search` keeps the existing successful path and
evidence flow unchanged. The new behavior starts only after the normal lap-1
hold and stationary confirmation window have failed:

1. The existing straight reverse is selected only by the complete swept
   wall/all-possible-pillar preflight.
2. If the straight view remains unresolved, the two existing 90 mm reverse
   parallax arcs at -20 and +20 degrees are evaluated independently. The
   selector chooses a swept-safe arc that keeps at least one unresolved legal
   seat comfortably visible.
3. Both safe angles are alternated. After both have been tried, a new checked
   cycle starts and fresh camera frames continue to be processed. Evidence
   failure alone no longer returns to a permanent lock.
4. Clear evidence already obtained for one of the two seats is retained. The
   search targets only the still unresolved seat.
5. If no movement currently passes the swept preflight, the controller keeps
   processing frames and retries the preflight rather than driving blindly.
6. Invalid pose/encoder data, travel in the wrong direction, excess heading or
   cross-track error, unsafe clearance, or an unsafe route rejoin still stops
   the robot. Those are motion-safety failures rather than perception misses.

Each parallax attempt still reverses at 70 mm/s, observes while stopped,
retraces its measured path at 60 mm/s, checks return pose error and then selects
the next view. There is no forward motion toward an unknown station.

## Verification

`check_active_lap1_search.py` executes the production viewpoint selector and
verifies alternating arcs, a new search cycle, preservation of prior clear
evidence, rejection of unavailable movement and absence of an evidence-only
terminal lock. Existing source-backed checks pass for all 12 discovery-station
dispatches, 3,557 recovery/rejoin cases, corner evidence, CW/CCW recovery
geometry, far acquisition, the log589 occupancy hold and runtime motor/servo
commit behavior. The later-lap model remains `2807 FAILED 0`.

IDE-managed M7 build succeeds: RAM 442560/523624, flash 539064/786432.
`firmware.bin` is 542712 bytes with SHA-256
`e3e805a4a6353d28c1cb8aaa2d475af875b5ad224e2630d2ad32ff3c416ee924`.
M4 is unaffected and was not built. No firmware upload or robot movement was
performed.

## Physical acceptance

Upload the M7 build and run the normal unchanged CW and CCW course. A station
that is seen normally must produce no additional search movement. For a missed
station, the log should contain `Active parallax` records until the station is
directly observed empty or a colour/seat is confirmed, followed by
`Resolved; resume`. Any `Locked` record now identifies a
motion/geometry/sensor-state safety rejection and must be reviewed separately.
