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
line is printed only every 2,000 ms. A reliability run passes only when both
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

## Continuous DCMI acquisition

This optional update-jitter optimization is implemented on the
`camera-continuous-dcmi` branch. `CAMERA_CONTINUOUS_CAPTURE_ENABLED` selects it,
while setting that flag to `false` retains the established asynchronous
snapshot fallback. It does not change the accepted 24 MHz sensor-clock profile.

The first uploaded stationary run used the robot facing an uncontrolled
non-mat scene with no official pillars. Through frame 2,215 it measured
79.62-79.63 ms completion intervals, zero intervals over 120 ms, zero discarded
frames, zero DMA/DCMI errors, 71-77 us service time, and approximately
7.46-7.56 ms control blockage. This passes the timing and duration portions of
the stationary acceptance criteria and removes the former 159.25 ms snapshot
restart gaps in this run. Because the required red and green pillars were not
in the scene, production classification and explicit torn-image checks remain
pending, as do brighter-light and low-speed driving validation.

The subsequent fixed official-pillar test passed under the current darker
indoor light. After a settled baseline, both colours were production-valid for
2025/2025 frames. All 75 detailed reports per colour were valid, with stable
red range of 443.7-444.0 mm and green range of 454.3-455.0 mm. Completion
intervals stayed at 79.62-79.63 ms and the miss, discard, and error counters
remained zero. Brighter competition-like lighting and the low-speed driving
lap remain pending.

The user explicitly waived the brighter-light stationary repetition for the
current development sequence because it is unavailable in the cellar. It is
skipped, not passed. The next available gate is the known-safe low-speed field
lap; final competition-light reliability remains unverified until suitable
lighting is available.

That low-speed field gate subsequently passed physically under the darker
cellar lighting. In the known-safe RIGHT/CW single-red-pillar layout at the
175 mm/s cap, the user observed a flawless complete run, continuous motion,
no contact, and generous extra clearance around the pillar. The continuous
capture implementation is therefore accepted for stationary operation and
low-speed driving in the available lighting. Brighter competition-like light
remains untested.

The run's USB file, `D:\log_76.txt`, is incomplete because 4,645 stationary
camera frames had nearly filled the logger's fixed 128 KiB RAM buffer before
driving began. It confirms 79.62-79.63 ms frame intervals and zero missed,
discarded, or errored captures before the drive, plus the correct RIGHT/CW
175 mm/s live-test start, but ends at path time 6,816 ms with a log-buffer
overflow warning. This is not evidence of a camera or driving failure. It does
mean the final lap result, exact obstacle injection, and pillar ToF clearance
record are unavailable from USB for this run.

The implementation is:

1. Leave the GC2145 running with the accepted 24 MHz XCLK, input `/2`, and PLL
   ratio 5. Do not combine this work with another sensor-clock experiment.
2. Replace repeated `DCMI_MODE_SNAPSHOT` stop/start operations with uninterrupted
   `DCMI_MODE_CONTINUOUS` acquisition.
3. Use two SDRAM frame buffers. DMA writes only to the active buffer; the frame
   interrupt atomically publishes the completed buffer and switches DMA to the
   other one.
4. Keep cache invalidation and RGB565 processing outside the interrupt. The ISR
   should only timestamp completion, update buffer ownership, and arm the next
   DMA destination.
5. If processing ever falls behind, discard the older completed frame and
   process the newest complete frame. Never process a buffer while DMA owns it.
6. Keep the current snapshot implementation behind a compile-time fallback
   until the continuous path has passed stationary and driving tests.

Acceptance criteria:

- at least 2,000 stationary frames with both fixed pillars production-valid
  after initial camera settling;
- completion intervals remain near 79.62 ms with no approximately 159.25 ms
  intervals;
- no torn frames, partial pillars, buffer-ownership errors, or cache artifacts;
- camera service and vision processing do not materially increase the control
  loop's approximately 6.6 ms blocking time;
- repeat under the present dark lighting and brighter competition-like light;
- complete one low-speed obstacle lap before removing the snapshot fallback.

Do not shorten exposure or enable PLL ratio 6 as part of this TODO. Those are
separate image-quality/timing experiments and would make failures harder to
attribute.
