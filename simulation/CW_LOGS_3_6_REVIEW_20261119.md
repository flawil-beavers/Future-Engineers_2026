# Clockwise runs 3–6, 19 November 2026

We reviewed four runs to understand missed signs on lap 1 and stops during
final parking. We copied the original USB files before analysis and verified
the SHA-256 hashes against the source. A later check found the same files.
Logs 1 and 2 in the intake duplicate evidence already archived. A complete
file can record an interrupted or failed attempt.

| Original | Bytes | SHA256 |
| --- | ---: | --- |
| [20261119_log_3_cw.txt](evidence/parking_exit_diagnostics/20261119_log_3_cw.txt) | 98495 | `07fd4ec064b140b6c1bd722f5429dcb475e50836c259e4eb9bacaa748ec3d835` |
| [20261119_log_4_cw.txt](evidence/parking_exit_diagnostics/20261119_log_4_cw.txt) | 62327 | `0379446f767a296bc02aa8de55c0f92498e71a37a48b9156dde2fc44bd76a0d5` |
| [20261119_log_5_cw.txt](evidence/parking_exit_diagnostics/20261119_log_5_cw.txt) | 140019 | `89c36184a46474dcecfb90201e122476902960fa5ad05c5c564420351470e795` |
| [20261119_log_6_cw.txt](evidence/parking_exit_diagnostics/20261119_log_6_cw.txt) | 145310 | `ecb427fb7f67a77efe49eb314f32195f4050aff38e1a11ae283544ecc4d55a0b` |

## Physical reports and installed identity

The team reported that the first CW run missed a red sign on the third
non-start straight, passed to its left and contacted it on lap 2. The next
run hit the farther green sign on the starting straight. The third had
difficulty in the third section and stopped before parking. In the latest
run, the team reported that the obstacle section went well, including
recognition of the small red sign, and that the car stopped during parking.

Logs 3 and 4 record a user pause. Logs 5 and 6 record autonomous parking
aborts. Both parking logs report `20261111_recovery_handoff`, which predates
the revised scan line and speed profile. All four report the
`20261119_measured_height` camera revision. The source identity comes from
the logs. The installed binary was unavailable for readback.

Log 6 records a 50 mm modified sign at seat 15 with visible red and original
green. Logs 3 and 5 record a 150 mm sign with visible green and original red.

## Findings and changes

1. **A red sign was discarded around the corner in log 3.** Before S3, the
   camera projected red seat 18 near (900, 500) mm at ranges of 934–1,255 mm.
   The acquisition check used the 600 mm empty-view limit. Two later
   oblique views marked the station clear, so the sign was missing from the
   later-lap map. We extended the colour check to its calibrated 1,600 mm
   limit. It retains the 6° ray agreement, position and side checks, and
   consecutive votes. A remote seat on the next loop is still rejected.
2. **An oblique view counted as empty in log 3.** The first corner station
   was cleared from views roughly 34–46° away from the section heading.
   Ordinary first-station empty votes now require heading alignment within
   25° and retain the existing 450 mm range. Targeted corner observations
   and parking scouts can still check the station. Unresolved stations
   continue through the existing observation and recovery states.
3. **Log 4 records a different colour at the start.** The camera summaries
   show red and no confirmed green. The map associates the red component
   with middle seat 2, which excludes the expected end positions under the
   solitary-middle rule. No approved original image establishes the cause.
   We kept the colour thresholds and position checks and added cached
   `errorBits` and `badPhaseCount` to the existing bounded `DRIVE_CAM` line.
4. **The scan path failed its clearance check in log 5.** At
   (638.3, −1,228.7 mm, 181.7°), even the 15 mm sweep with neutral steering
   failed the inflated wheel and rail check. The scan line prepared after
   log 2 moves the car 10 mm inward and passes the same model check. That
   change retains 20 mm of nominal clearance from the rail tips, approach
   and alignment limits of 150 and 130 mm/s, and the blend down to 60 mm/s.
   The scan itself runs at 60 mm/s.
5. **The marker scan returned an invalid gap in log 6.** The fixed and movable
   marker positions were reported as 556.1 and 333.6 mm. The estimated gap
   was 222.4 mm, outside the accepted 247.5 ± 15 mm range. The
   controller now clears the invalid result and can scan both markers once
   more if it has a fresh stopped wall reference and the entire return path
   is clear. An unsafe return path or a second invalid gap still stops
   parking. We replayed the logged heading and X position on the revised
   scan line. Those estimates come from the robot's telemetry.
6. **The previous search stalled in narrow 232.5 mm gaps.** With the
   281.5 mm capture line, it failed all 24 perturbations before motion.
   If the normal search fails at a near-parallel capture X of 240–280 mm
   and the measured gap is more than 10 mm below nominal, the fallback
   checks a 40 or 60 mm straight retreat and the entire path that follows.
   The next search starts at X ≤ 240 mm, so the retreat search does not
   repeat recursively. The existing margins and first-arc radius checks
   still apply. The change recovered 18 of 24 model cases. The six
   remaining cases safely stopped after motion with radius deviations of
   +15% or +50%.

During scan-alignment preflight, the controller now services queued gyro
reports, as it already did during the approach preflight.

## Verification

| Check | Recorded result |
| --- | --- |
| Archived corner association | Two cases passed. Restoring the old 600 mm gate reproduced the rejection, and the distant seat on the next loop remained rejected. |
| Empty and occupied positions | Green occupancy case 589, nearer-view case 621 and three oblique-clear cases passed. The case 565 check passed ten observations and 27 rejoins, including the old-veto control. |
| Far acquisition and recovery | 52 far-acquisition and 25 unresolved-position/recovery cases passed. |
| Modified signs | All 45 approved images, 16 captured sequences, 648 rendered cases, 48 association cases and 24 rule cases passed. |
| Parking sequence | Three quick sequences, five registrations and six recovery or braking handoffs passed. |
| Measured-gap parking | All 24 cases passed in both directions with 248.9 and 249.6 mm gaps and radius factors of 1, 1.15 and 1.5. |
| Scan replays from logs 5 and 6 | The old scan sweep failed and the revised line passed. One invalid scan recovered to a modelled 250.3 mm gap without a modelled collision. A second invalid scan was rejected. |
| Accepted-gap stress test | **114 of 120** model cases parked and six stopped safely in narrow gaps. The previous search parked in 96 cases and stopped in 24. Neither model produced a collision in this test. |

The replay uses modelled geometry, idealised ToF measurements and synthetic
colour checks where applicable. Physical clearance and the source of the
colour observation in log 4 still require run evidence.

We tested an 80 mm retreat first in a host fixture. It increased the
stress-test failures from six to 12 of 120, including new failures at
5–10% radius deviation. The production search kept the tested order.
Reproduce the comparison by changing only the retreat candidate order in
the accepted-range stress fixture.

## Build and follow-up

The IDE-managed M7 build succeeded. It used 452,136 of 523,624 bytes of RAM
and 599,200 of 786,432 bytes of flash. The 602,752-byte binary had SHA-256
`332b21f986d6f48efa276998d4bca92c9f6c17d371662256b54b50402fb11948`.
Existing Serial macro and unused-helper warnings remained. This revision
covered the M7 source and host checks.

The physical follow-up for this revision was to check early recognition of
red seat 18, green sign passage at the start, marker scan and retry, actual
wheel clearance and final containment during ordinary CW and CCW runs. If
the wrong visible colour recurs, the next diagnostic is a user-approved
stationary camera sample before changing thresholds.
[Current engineering handoff](../AGENT_DOCUMENTATION.md)
