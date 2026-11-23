# Rear ToF on the GIGA M4 core

We use the rear Adafruit VL53L4CX to measure the gap behind the car. The GIGA's
M4 core reads it over a dedicated software I²C bus, filters the measurements
and sends fixed-size status frames to the M7 over OpenAMP/RPC. The M7 reads the
result through `get_tof_distance(TOF_REAR)`.

## Wiring

Use one half of a cut four-pin STEMMA QT/Qwiic cable. Plug the connector into
the rear ToF sensor and use jumper wires from the cut end directly to the GIGA.
Connect the signal wires by pin name. [Sensor wiring rationale](../../README.md#wiring-and-signal-interfaces)

| VL53L4CX | GIGA R1 |
| --- | --- |
| VIN | 3.3 V |
| GND | GND |
| SDA | A3 |
| SCL | A4 |

Do not connect A3 or A4 to another peripheral. Verify the signal names rather
than relying only on cable colours.

## Build and upload

Use the IDE-managed PlatformIO Core. For a complete installation, upload **M4
first, then M7**. [Build and upload instructions](../../README.md#build-and-run-the-firmware)

```powershell
$pio = Join-Path $env:USERPROFILE '.platformio\penv\Scripts\platformio.exe'
& $pio run --environment giga_r1_m4 --target upload
& $pio run --environment giga_r1_m7 --target upload
```

Both images must be present. The M7 starts the M4 when RPC initializes.

## Behaviour

- The software I²C bus runs at 100 kHz on A3 and A4.
- The rear sensor starts in medium-distance mode with a 30 ms timing budget.
- This rear module is the previously damaged sensor. Its measured black-wall
  reliability limit is approximately 370 mm, so longer filtered distances are
  deliberately reported as `TOF_OUT_OF_RANGE_MM` (`9999.0f`). Raw distance,
  signal and sigma remain available for diagnostics.
- The same hardware-status, signal, sigma and slew filters used by the side
  sensors are otherwise applied on M4.
- M4 publishes measurements asynchronously; the M7 control loop never waits
  for individual sensor transactions.
- The rear distance, raw distance, signal and sigma getters return `-1.0f`
  when RPC data is missing, the sensor reports an initialization or bus error,
  or the measurement is older than 250 ms.
- The serial `v` command and general telemetry print left, right and rear ToF.

The existing side-only navigation and diagnostic loops intentionally remain
limited to `TOF_SIDE_COUNT`; adding `TOF_REAR` does not make them interpret the
rear range as a lateral wall measurement.

## Parking-position configuration

Before the five-segment parking exit, we use the rear range to check and
correct the car's position along the bay. The settings are in
[`include/config.h`](../../include/config.h#L451-L501).

| Setting | Meaning |
| --- | --- |
| `OBSTACLE_REAR_TOF_BEHIND_AXLE_MM` | The sensor origin is 25 mm behind the rear-axle midpoint. |
| `OBSTACLE_PARKING_REAR_TOF_TARGET_CLEARANCE_MM` | The target clearance from the exit model's rear extent to the rear magenta marker is 50 mm. |
| `OBSTACLE_PARKING_REAR_TOF_POSITIONING_TEST_ONLY` | Stops and saves the log after positioning, before the exit. Off for normal runs. |
| `OBSTACLE_PARKING_REAR_TOF_EXIT_TEST_ONLY` | Runs the exit, then stops before post-exit localisation and discovery. Off for normal runs. |

The exit model places its rear extent 40 mm behind the axle, 15 mm behind
the sensor origin. A 50 mm clearance in that model therefore corresponds to
a **65 mm sensor reading**. The log records the measured range and the
derived clearance. If the sensor moves, update its position in the
configuration so the firmware recalculates the target range.
[Measured contour used for final parking](../../simulation/RETURNED_651_656_PARKING_20261105.md#measured-geometry-and-implementation)

Start with the car fully inside the bay, approximately parallel to its limits
and within about ±20 mm of its longitudinal midpoint. The swept model includes
about ±1° of placement error. The positioning routine corrects the longitudinal
gap only.

1. With the steering centred and the car stationary, the controller averages
   three fresh rear readings that span at most **4 mm**.
2. If the gap needs correction, the controller moves at **60 mm/s** and uses
   cumulative encoder travel to track the move. The initial correction is
   limited to **55 mm**.
3. After braking, it waits **500 ms**, discards the first two fresh frames and
   averages three stationary readings. The final range must be within **5 mm**
   of target, and the measured range change must agree with encoder travel
   within **8 mm**.
4. If verification fails and the remaining range error is at most **15 mm**,
   the controller can make one small correction in the direction of the target
   and verify again. If the retry fails, the car holds in place.

[Positioning settings](../../include/config.h#L451-L501) · [Verification and retry](../../src/obstacle.cpp#L1515-L1581)

## Stationary setup check

When commissioning or troubleshooting the rear sensor, keep the motor enable
switch off and check the serial output.

1. Startup prints `M4 rear ToF RPC connected.`
2. Serial command `v` reports plausible `REAR` distances at several known
   black-wall placements through 370 mm while left and right readings remain
   normal. A placement clearly beyond 370 mm must report `9999.0`.
3. Disconnecting the rear sensor changes `REAR` to `-1.0` without affecting
   the M7 loop, gyro, camera, or side sensors.

For normal driving validation, follow the
[current Obstacle Challenge checklist](../validation/OBSTACLE_CHALLENGE_TEST_PLAN.md#active-validation-sequence).
