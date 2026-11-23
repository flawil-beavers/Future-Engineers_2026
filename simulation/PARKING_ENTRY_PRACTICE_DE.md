# Isolated reverse-parking practice

We use this test to check the four-move entry separately from the rest of the Obstacle Challenge. The normal run and the practice modes use the same moves, with mirrored steering in clockwise (CW) and counter-clockwise (CCW) runs. [Parking sequence and diagram](../README.md#obstacle-challenge-strategy)

<a id="cw-eckentest-mit-fester-zugfolge"></a>

## Clockwise corner approach and fixed entry

Place the car on the parking straight, facing the bay, with the outer wall on the left. Centre the steering and line the car up parallel to that wall. Leave about **20 mm** from the rear edge to the wall behind the car. Set the rear-axle midpoint **291.5 mm** from the side wall, measured at a right angle. That gives the wall-side wheel edge a nominal **30 mm** gap to the line between the marker tips. Keep the approach clear.

1. Upload `giga_r1_m7`. With the start toggle (SW1) LOW, send `park cw fixed` in the serial monitor.
2. Unplug the upload cable and set SW1 HIGH. The car approaches the bay, measures the markers and brakes at the entry position. It starts reversing from a stop when its heading is within **10°** of parallel.
3. Watch the sequence. It reverses on full lock to **47°**, reverses straight **125 mm**, reverses on opposite lock to **22.5°**, then switches back and drives forward until parallel.
4. The car centres the steering, checks the estimated outline and saves the log. The `final_parking_stop` event means the firmware accepted the pose. On the mat, check for contact and confirm that the entire car is inside the bay.

The IMU tracks the heading and the encoder tracks travel. A fresh, valid rear ToF reading of **20 mm or less** stops a reversing move. The controller also checks braking, progress, travel and time, then holds the car after the final check.

## Entry-only setup

The `park cw` and `park ccw` commands start from a measured pose. This mode assumes the nominal gap below and skips the approach scan and camera setup.

| Measurement | Nominal setup |
| --- | ---: |
| Gap between the inside faces of the markers | **247.5 mm** |
| Marker length from the outer wall into the field | **200 mm** |
| Rear-axle midpoint to the outer wall, measured at a right angle | **291.5 mm** |
| Rear-axle midpoint to the line between the marker tips, on the driving side | **91.5 mm** |
| Rear-axle midpoint along the wall from the entry marker's inside face | **35 mm** towards the front of the car, outside the bay |
| Heading | Parallel to the outer wall with the front wheels straight |

Mark the centres of the rear wheels on the mat to set the axle reference. The **35 mm** offset accounts for the rear overhang. We aim to place the car within about **2 mm** of the marks. Keep the manoeuvring area clear because the practice mode has no learned sign map. In full runs we use a **237.5 mm** practice target and the car measures the actual gap on approach.

1. With M7 uploaded and SW1 LOW, place the car at the measured entry pose. Send `park cw` or `park ccw` over serial.
2. Wait for the blue status light, unplug the upload cable and set SW1 HIGH. The car waits for braking and steering to settle, then begins the four moves.
3. Use `park stop` to leave practice. Before every new attempt, reposition the car and select the mode again. Record the log number, direction and any contact or manual stop.

## Return to a full run

Reset with the default Obstacle configuration or send uppercase `O` while SW1 is LOW. Keep `OBSTACLE_FINAL_PARKING_PRACTICE_ENABLED` off for normal runs. The flag selects a separate practice path. Use a full course run to check the automatic approach and bay measurement. [Park commands](../src/serial_handler.cpp#L488-L509) · [Four-move sequence](../src/final_parking.cpp#L1796-L1805) · [Engineering history](../docs/history/README.md)
