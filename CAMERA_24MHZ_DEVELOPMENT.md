# GC2145 24 MHz camera development

## Result

The Arducam B0462 / GC2145 now receives a 24 MHz XCLK reliably. Register
`P0:F7` is changed from `0x1D` to `0x1F`, enabling the GC2145's documented
input divide-by-two stage. This keeps the established sensor, ISP, and DVP
pixel-clock timing while allowing the GIGA to drive the camera module at
24 MHz.

The accepted compile-time profile is in `include/config.h`:

```cpp
#define CAMERA_SENSOR_XCLK_HZ 24000000UL
#define CAMERA_GC2145_PLL_MODE1 0x1F
#define CAMERA_GC2145_PLL_DIVX4 0x05
```

`FullFovGC2145::setResolution()` applies the clock registers after the stock
sensor initialization and full-FOV configuration. It also programs the AEC
anti-flicker step from the PLL ratio. With the accepted ratio of 5, this is the
datasheet default `0x0168`.

## Why the earlier 24 MHz attempt failed

Changing only XCLK from 12 to 24 MHz left `F7=0x1D`, so the complete internal
clock chain and DVP output were doubled. The receiver then sampled a pixel
stream outside the already proven timing margin, producing displaced and
fragmented blobs. The reliable fix is not a slower external edge: it is using
the sensor's input divider so the downstream clock remains controlled.

## Measurements on 2026-08-25

Stationary setup, switch LOW, motors disabled, darker-than-competition indoor
lighting:

- Red pillar: 400 mm forward and 150 mm left of camera.
- Green pillar: 400 mm forward and 150 mm right of camera.
- 12 MHz baseline: typical capture 79–82 ms, service 76–81 us, processing
  about 6.55 ms. Both pillars were stable and production-valid.
- 24 MHz XCLK, input `/2`, PLL ratio 5: typical capture 76–82 ms, service
  76–81 us, processing 6.54–6.70 ms. Long run: red 2065/2065 valid and green
  2065/2065 valid.
- 24 MHz XCLK, input `/2`, PLL ratio 6: typical capture about 68 ms (roughly
  14% faster), but periodic lighting bands caused partial or missing pillars.
  Scaling the AEC anti-flicker step from `0x0168` to `0x01B0` did not cure it.
  This profile was rejected.

The accepted 24 MHz profile deliberately gives essentially no frame-rate gain:
it proves reliable 24 MHz module communication without overclocking the DVP
pixel stream. The speed benefit still comes from asynchronous DMA buffering:
the main loop blocks for only about 6.6 ms per processed image instead of the
old synchronous capture plus processing time of roughly 149 ms.

## Diagnostics

Camera-calibration telemetry now includes `test_frames`, `red_valid`, and
`green_valid`. These counters cover every processed frame, while the detailed
line is printed only every 500 ms. A reliability run passes only when both
valid counters equal `test_frames` for the fixed two-pillar scene.

## Safety and fallback

Stationary camera auto-start remains enabled and the camera calibration mode
never enables the motors. To return to the original electrical and internal
timing, set `CAMERA_SENSOR_XCLK_HZ` to `12000000UL`,
`CAMERA_GC2145_PLL_MODE1` to `0x1D`, and keep the PLL ratio at `0x05`.

Further frame-rate increases need exposure/anti-flicker work across several
lighting conditions. Do not enable ratio 6 merely because it is faster; it
failed the required dark-light reliability test.
