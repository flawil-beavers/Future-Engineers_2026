# Obstacle seat numbering

The field model contains four straight sections. Each section has three
longitudinal stations, and every station has a possible seat on the right and
left of the robot's driving direction.

Seat IDs are calculated as:

```text
seat_id = section * 6 + station * 2 + side
side 0 = RIGHT
side 1 = LEFT
```

Right and left are always viewed in the selected driving direction. Therefore
the physical interpretation mirrors when changing between CCW and CW.

| Section | Station | Right seat | Left seat |
| ---: | ---: | ---: | ---: |
| 0 | 0 | 0 | 1 |
| 0 | 1 | 2 | 3 |
| 0 | 2 | 4 | 5 |
| 1 | 0 | 6 | 7 |
| 1 | 1 | 8 | 9 |
| 1 | 2 | 10 | 11 |
| 2 | 0 | 12 | 13 |
| 2 | 1 | 14 | 15 |
| 2 | 2 | 16 | 17 |
| 3 | 0 | 18 | 19 |
| 3 | 1 | 20 | 21 |
| 3 | 2 | 22 | 23 |

For sections after the starting section, station 0 is just after entering the
straight, station 1 is in its middle, and station 2 is just before the next
corner. The starting pose is in the middle of section 0, so its encounter order
wraps: station 2 is ahead at the first corner, while stations 0 and 1 are
revisited near the end of the lap.

Example: `S1 station=0` is the first station after the first corner. Its right
seat is ID 6 and its left seat is ID 7. Thus `scan_seat=7` in `log_31` means the
camera was trying to resolve the left seat of that station.

Discovery telemetry uses two additional values:

- `scan_seat=-1`: no discovery scan is active.
- `scan_seat=-2`: both seats of the current station are targeted together.
