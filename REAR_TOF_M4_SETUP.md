# Rear ToF on the GIGA M4 core

The rear Adafruit VL53L4CX uses a software I2C bus owned exclusively by the
GIGA's M4 core. The M4 filters measurements and sends fixed-size status frames
to the M7 over OpenAMP/RPC. The M7 exposes the result through the normal sensor
API as `get_tof_distance(TOF_REAR)`.

## Wiring

Connect a Qwiic-to-pin cable as follows:

| VL53L4CX | GIGA R1 |
| --- | --- |
| VIN | 3.3 V |
| GND | GND |
| SDA | A3 |
| SCL | A4 |

Do not connect A3 or A4 to another peripheral. Verify the signal names rather
than relying only on cable colours.

## Build and upload

Use the IDE-managed PlatformIO executable described in `AGENTS.md`. Build or
upload the M4 image first, followed by the M7 image:

```powershell
$pio = Join-Path $env:USERPROFILE '.platformio\penv\Scripts\platformio.exe'
& $pio run --environment giga_r1_m4 --target upload
& $pio run --environment giga_r1_m7 --target upload
```

Both images must be present. The M7 starts the M4 when RPC initializes.

## Behaviour

- Software I2C runs at 100 kHz on A3/A4.
- The rear sensor starts in medium-distance mode with a 30 ms timing budget.
- This rear module is the previously damaged sensor. Its measured black-wall
  reliability limit is approximately 370 mm, so longer filtered distances are
  deliberately reported as `TOF_OUT_OF_RANGE_MM` (`9999.0f`). Raw distance,
  signal and sigma remain available for diagnostics.
- The same hardware-status, signal, sigma and slew filters used by the side
  sensors are otherwise applied on M4.
- M4 publishes measurements asynchronously; the M7 control loop never waits
  for individual sensor transactions.
- Missing RPC data, an initialization/bus error, or no fresh measurement for
  250 ms makes all rear getters return `-1.0f`.
- The serial `v` command and general telemetry print left, right and rear ToF.

The existing side-only navigation and diagnostic loops intentionally remain
limited to `TOF_SIDE_COUNT`; adding `TOF_REAR` does not make them interpret the
rear range as a lateral wall measurement.

## First physical validation

Keep the motor enable switch disabled. After uploading both images, require:

1. Startup prints `M4 rear ToF RPC connected.`
2. Serial command `v` reports plausible `REAR` distances at several known
   black-wall placements through 370 mm while left and right readings remain
   normal. A placement clearly beyond 370 mm must report `9999.0`.
3. Disconnecting the rear sensor changes `REAR` to `-1.0` without affecting
   the M7 loop, gyro, camera, or side sensors.
