# Full-FOV asynchronous camera buffering

## Branch and safety state

- Branch: `camera-async-buffering`, created from `pure-pursuit` commit
  `a5ea882` in a separate Git worktree.
- Development worktree: `local_workspace/async_camera_worktree` in the original
  checkout. The directory is already ignored there, so the other Codex session
  can keep using the original checkout without a branch switch.
- The development firmware boots only into `MODE_CAMERA_CALIBRATION` and can
  auto-start that stationary mode with the physical switch LOW. It does not
  enable the motor system or bypass the switch for a driving mode.
- `CAMERA_ASYNC_STATIONARY_AUTOSTART` must be disabled and the production
  startup mode restored only after camera recalibration and moving tests are
  ready.

## Why a project-owned camera library exists

Arducam DVP 1.0.0 exposes only blocking `Camera::grabFrame()`. Its DCMI and DMA
handles are private to the library source, so a reliable non-blocking wrapper
cannot be implemented solely in application code. This branch vendors the four
required Arducam/GC2145 files under `lib/arducam_dvp_async` and adds:

- `startFrame()`: starts snapshot DMA and returns immediately.
- `frameReady()`: polls the hardware capture-complete state without waiting.
- `finishFrame()`: stops/finalizes DCMI and invalidates the M7 data cache.

The original synchronous `grabFrame()` remains available for A/B diagnostics.
`CAMERA_ASYNC_CAPTURE_ENABLED` selects the implementation at compile time.

## Full FOV

`FullFovGC2145` reads the complete 1616 x 1208 sensor window and configures the
sensor to subsample it 5:1 into a normal 320 x 240 RGB565 DCMI frame. This keeps
the full optical view without a 960 kB 800 x 600 transfer. The sensor remains
at its driver-supported 12 MHz XCLK because 18 and 24 MHz were not reliable
with the stock PLL/blanking configuration.

The async branch originally carried provisional full-FOV constants. During the
merge into `pure-pursuit`, those were replaced by the later measured production
calibration: 65.3 degree horizontal FOV, principal X 164.4 px, focal X 248.9 px,
horizon 78 px, ground scale 24000 mm-px, and zero edge-foot correction.

## Buffer ownership and RAM

Each 320 x 240 RGB565 frame is 153,600 bytes. The two fixed, 32-byte-aligned
buffers are:

```text
A: 0x60000000 .. 0x600257ff
B: 0x60025800 .. 0x6004afff
```

Both are in the GIGA's 8 MB external SDRAM. `SDRAM.begin(0)` initializes SDRAM
without adding it to the general allocator, so no other heap user can overlap
these addresses.

The build reports 291,024 / 523,624 bytes of static internal RAM. The former
default `FrameBuffer` additionally allocated 153,600 bytes from the internal
heap at runtime, leaving only about 79 kB before stacks and other allocations.
The asynchronous version allocates neither camera buffer internally, leaving
about 232 kB before normal runtime stack/heap use. The two SDRAM buffers consume
307,200 bytes, about 3.7% of external SDRAM.

At runtime DMA owns one buffer while vision owns the other:

```text
DCMI/DMA -> buffer A       M7 vision reads buffer B
DCMI/DMA -> buffer B       M7 vision reads buffer A
```

After a snapshot completes, `finishFrame()` publishes the DMA writes, the
buffers swap roles, and the next DMA snapshot starts before vision processes
the completed buffer. Vision takes about 6.6-6.8 ms, much less than the shortest
observed frame interval, so DMA cannot catch the buffer still being processed.

## Measured result on the connected robot

All measurements used the full-FOV 12 MHz mode with motors disabled.

| Metric | Blocking, SDRAM A/B control | Asynchronous SDRAM |
| --- | ---: | ---: |
| Capture call / DMA service | 141.5-143.0 ms | 76-79 us |
| Vision processing | 6.5-6.7 ms | 6.6-6.8 ms |
| M7 control-loop blockage per result | 148-150 ms | about 6.8 ms |
| Observed result throughput | about 7 FPS | about 13 FPS |

The control-loop blockage fell by about 95.4%. Result throughput approximately
doubled because the next capture begins before processing and logging the prior
frame. Individual asynchronous capture spans were normally about 77-83 ms and
occasionally about 154 ms when a snapshot missed the next sensor frame boundary.

The synchronous-SDRAM A/B run produced the same stable scene features as the
asynchronous run, which verifies SDRAM addressing, cache publication and frame
layout. More than 1,000 asynchronous frames completed without a capture stall.
The official centered pillar used in the earlier full-FOV baseline was no
longer visible during these A/B tests, so a final official-pillar detection
comparison and physical recalibration remain required.

## 24 MHz status

Changing only XCLK from 12 to 24 MHz reduced capture to about 62 ms, but the
stationary pillar fragmented or disappeared. At 18 MHz it also jumped in
position and estimated range. The Arducam GC2145 driver programs a fixed
12 MHz PLL, blanking and exposure profile, and its `setFrameRate()` is a no-op.

A reliable 24 MHz mode is still plausible, but requires a separate sensor
timing profile: read back and tune registers `0xF7`-`0xFA`, horizontal/vertical
blanking, exposure limits and possibly DCMI sampling polarity. Validate raw
frames at increasing internal clock rates; do not enable 24 MHz merely by
overriding `getClockFrequency()`.

## Validation after merging into `pure-pursuit`

1. Completed: the representative official-pillar production seat check and
   field-clear background check passed with asynchronous capture. The measured
   camera constants remain valid.
2. Completed: telemetry reported `async=yes`, stable frame increments, roughly
   76-84 ms normal capture spans, roughly 6-7 ms processing, and no stall
   through frame 1929.
3. Next: repeat the pending red-left Pure Pursuit run at 175 mm/s. This run
   validates both the async integration and the new earlier single-seat nudge.
4. Later: restore the intended competition startup mode after development
   validation.
