# Isolated reverse-parking practice

Use this test to isolate the parking entry from the rest of the Obstacle
Challenge. Normal Obstacle mode and the practice commands use the same
four moves. Counter-clockwise (CCW) mirrors the steering directions.

<a id="cw-eckentest-mit-fester-zugfolge"></a>

## Clockwise corner approach and fixed entry

Start at the corner on the parking straight, facing the bay with the adjacent
outer wall on the left. With the wheels straight and the car parallel to that
wall, set the rear edge about **20 mm** ahead of the transverse wall. Put the
centre of the rear axle **291.5 mm** from the side wall, measured perpendicular
to it. This gives a nominal **30 mm** gap from the wall-side wheel edge to the
line between the marker tips. Keep the route from the corner to the bay clear.

1. Upload `giga_r1_m7`. With the start toggle LOW, send `park cw fixed` in
   the serial monitor.
2. Unplug the upload cable and set the start toggle HIGH. The car approaches
   the bay, measures the markers and brakes at the entry position. It starts
   the reverse entry after stopping when within **10°** of parallel.
3. The car reverses on full lock to **47°**, reverses straight for **125 mm**,
   reverses on opposite lock to **22.5°**, then changes lock and drives
   forward until parallel.
4. The car centres the steering, checks the estimated parked outline and
   saves the log. `final_parking_stop` records a park accepted by the firmware.
   Check actual clearance and containment on the mat. Reposition the car
   and select the command again to repeat the test.

The four moves use IMU heading and encoder travel. A fresh, valid rear ToF
reading of **20 mm or less** aborts a reversing move. Braking, progress,
travel and time checks remain active. The car holds after the final check.

## Entry-only setup

Select `park cw` or `park ccw` with the start toggle LOW. The car assumes
the surveyed starting pose and nominal gap in this mode. It omits the
approach scan and camera setup.

| Measurement | Nominal setup |
| --- | ---: |
| Gap between the inside faces of the markers | **247.5 mm** |
| Marker length from outer wall into the field | **200 mm** |
| Rear-axle centre to outer wall, measured perpendicular to it | **291.5 mm** |
| Rear-axle centre to the line joining the marker tips, on the driving side | **91.5 mm** |
| Rear-axle centre along the wall from the inside face of the entry marker | **35 mm** towards the front of the car, outside the bay |
| Car orientation | Parallel to the outer wall with the front wheels straight |

Use two marks for the rear wheel centres to set the axle reference. The
35 mm offset from the entry marker corresponds to the rear overhang. Leave
the immediate manoeuvring area clear because this test has no learned sign
map. Aim to place the car within about 2 mm of the marks and carefully
align it. This is a setup target. The entry-only test uses the nominal
247.5 mm gap above, while the team's full-run practice target is 237.5 mm
and the normal approach measures the gap.

## Start, repeat and return to normal driving

1. Upload M7, keep the start toggle LOW and place the car at the surveyed
   entry pose. Select `park cw` or `park ccw` over serial.
2. Once the mode shows blue, unplug the upload cable and set the start
   toggle HIGH. The car waits for braking and steering to settle, then
   begins the four moves.
3. Use `park stop` to leave practice. Reposition the car and select the
   mode again for every new attempt. Record the log number, direction and
   any contact or manual stop.

A reset with the default Obstacle configuration, or the serial command
`O` with the start toggle LOW, restores the full course. Keep
`OBSTACLE_FINAL_PARKING_PRACTICE_ENABLED` off for normal runs. That flag
selects a separate practice path.

Entry-only practice checks the manoeuvre from a surveyed position. Use a
full course run to validate the automatic approach and bay measurement.
[Park commands](../src/serial_handler.cpp#L488-L509) ·
[Four-move sequence](../src/final_parking.cpp#L1796-L1805) ·
[Earlier engineering work](../docs/history/README.md)
