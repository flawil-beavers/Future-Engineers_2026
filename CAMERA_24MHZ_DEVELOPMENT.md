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

## DMA timing follow-up

The original `capture_ms` ended when the main loop noticed `frameReady()`, so
it mixed hardware capture with service delay. Commit follow-up instrumentation
timestamps the DCMI frame interrupt and reports:

- `capture_ms`: capture start to DCMI frame-complete interrupt;
- `ready_wait_ms`: interrupt to main-loop service;
- `frame_interval_ms`: consecutive DCMI completion timestamps;
- aggregate minimum/maximum interval and `missed_intervals` over 120 ms;
- current page-zero exposure in line periods.

In the stationary dark-light test the normal DCMI completion interval was
79.61–79.63 ms. Normal hardware capture was about 76–79.4 ms, service wait was
usually 0.1–1.1 ms, and vision processing was about 6.5–6.7 ms. A 2,145-frame
run confirmed stable pillar geometry; its first three green classifications
were startup settling, followed by 2,142 consecutive valid green frames and
2,145/2,145 valid red frames.

True 159.25 ms intervals also occur. An aggregate follow-up observed 20 such
intervals in the first 303 measured intervals while all 304 delivered frames
remained valid. Exposure stayed fixed at 1,080 lines, below the normal
1,258-line frame period, proving these exact doubled intervals are snapshot
stop/restart misses rather than AEC extending exposure in low light. Do not cap
exposure to address them: it would reduce dark-scene image quality without
removing the cause.

The reliable delivered-image rate in this diagnostic-heavy run is therefore
about 11.8 FPS on average, with 12.56 FPS whenever no VSYNC is missed. Normal
driving should print much less camera telemetry. Eliminating every missed VSYNC
would require uninterrupted continuous DCMI acquisition with DMA buffer
handoff, a materially more complex change than the current snapshot ping-pong
API; undertake that separately only if driving logs show the update jitter
affects obstacle decisions.
