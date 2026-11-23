# Open Challenge code and host review, 27–28 October 2026

We reviewed the Open Challenge at `cc0bc59232359435018cc12281b488242767f96a`. The host tests reproduced a delay at the first corner and an older finish condition that could keep the car moving. On 28 October we also verified that the inner-wall steering commands matched the older controller. This is a record of those revisions. See the [README](../README.md#open-challenge-strategy) for the current strategy.

## First-corner timing

While the car searches for a wall to follow, `state_following()` requests a **300 ms** ToF measurement budget. At `cc0bc59`, it requires three fresh ToF frames from the same side before confirming a corner. The earlier revision `861ed8e` counted three loop iterations, so it could confirm on the same measurement after two more **1 ms** iterations. [Navigation source](../src/navigation_controller.cpp)

With one new frame every **300 ms**, both CW and CCW tests entered `NAV_TURNING` **600 ms** after the first positive frame. At the **300 mm/s** startup speed this is another **180 mm** of travel. Time until the first frame and physical sensor and transport latency would add to that distance estimate. After the controller selects a side, it returns to a **30 ms** budget and the two additional frames take about **60 ms**.

The three-frame check fixes the repeated-frame error. The follow-up was to tune the search budget and confirmation together, then measure where the car turns during normal CW and CCW runs.

## Finish condition

On the reviewed revision, `state_following()` enters `NAV_STOPPED` after twelve turns and more than **500 mm** on the new straight only if `0 < current_wall_distance < nav_wall_margin`. The corner check runs first. The host gave the controller twelve turns, three counted laps, **501 mm** since the last turn and three fresh gap frames. It attempted a **13th turn** in both directions. With missing or stale side readings, it still requested the normal driving speed after **1,000 mm**. The gap case also occurs at `861ed8e`.

The follow-up was to evaluate lap count and finish distance independently of wall correction, with the finish taking priority over another turn. The normal soft stop and single save request still needed to work with fresh gaps, invalid frames and stale frames. These tests reproduce controller states. The review recorded no physical overrun.

## Other findings

| Area | What we observed | Follow-up at the time |
| --- | --- | --- |
| Speed when changing modes | Obstacle setup sets `nav_normal_speed` to **220 mm/s**. Switching to Open calls `navigation_enable()` and leaves that value in place. The Open cold-start value was **300 mm/s**. | Define per-mode defaults or explicitly share an operator setting. |
| Turn completion | `state_turning()` finishes when the heading crosses the gyro target. With a fixed heading marked healthy and continuing encoder movement, the controller was still turning after a simulated **10 s** and **3,000 mm**. The stall watchdog observes encoder progress. | Add a plausibility or abort limit for a turn that makes no heading progress. |
| Range budget after a stop | Disabling navigation during the initial wall search left the **300 ms** budget and `nav_long_range_active` set. | Reset the sensor state when ending the mode and when starting the next one. |

The heading test deliberately supplies an unchanged angle marked healthy. It demonstrates how the controller responds to that input. The frequency of such a fault on the robot was unknown.

## Behaviour verified on the host

The fixture compiled the full navigation controller with the production stop, speed, steering and ToF snapshot functions. It simulated sensor frames, travel, angles, electrical outputs and USB saving.

- With valid walls, both directions completed twelve turns, reached the expected gyro targets, braked normally and requested one save.
- Repeated ToF sequences could not confirm a corner. Stale frames and transport errors cleared the confirmation, and fresh frames restored it.
- The confirmed side selected the first turn direction and restored the normal measurement budget.
- The moving states checked in this test restored drive and steering after `stop(false)`.
- Open mode did not invoke camera processing or the Obstacle planner. The build default was `MODE_OBSTACLE_CHALLENGE`. Lowercase `l` selected Open until reset. Starting directly in Open required the corresponding M7 configuration.

## Recheck on 28 October

We used `16db81b8bc55833ef755009f4ea0d6d51ce9c671` with the parking edits already in the worktree. We changed the test tools and documentation during this recheck. The corner confirmation originated in the earlier runtime change at `cc0bc59`.

**Inner-wall steering.** We compared **11,200** commands across **560** scenarios with `861ed8e`. We covered both walls, distances of **200, 250, 300, 350 and 550 mm**, seven heading errors from **−12° to +12°**, control intervals of **1, 10, 30 and 50 ms**, and wall reacquisition on and off. Steering, simulated servo output, speed target and reacquisition matched in every compared step. The controller steered away when too close and towards the wall when too far. The **300 mm** target, PD gains, gyro filter and turn command of **±25** also matched.

The recent parking changes in `final_parking.cpp` and the diagnostic revision string in `obstacle_path.cpp` did not run through normal Open operation. Shared motor and gyro code passed the runtime suite and **40,000** drive-profile comparisons, including the newer handling of repeated slow control iterations.

**First-corner delay.** The second fixture runs `read_single_tof()` as well as navigation. It supplies one hardware frame every **300 ms** and runs the controller between frames. Both directions gave the same results.

| Input after a 300 mm wall reading | Reference `861ed8e` | Reviewed revision |
| --- | --- | --- |
| No detected object | First new frame | Third new frame |
| Detected object with signal below the threshold | First new frame | Third new frame |
| Valid 2,000 mm range | Thirteenth new frame | Fifteenth new frame |

**Range filtering.** In the initial search, an accepted reading can rise by at most **100 mm** per frame. Starting at **300 mm**, the strict **1,500 mm** threshold is first crossed on the thirteenth frame. The newer confirmation then needs two more frames. This synthetic case took about **4.2 s** from the first long-range frame, compared with about **3.6 s** in the older code. Empty or out-of-range frames bypass this step limit. A frame with a rejected low-quality return still counts as a gap in this sensor path. The test did not measure whether black walls produce sustained valid returns of this kind.

The recheck also reproduced the wall-dependent finish condition. The three-lap tests supply the angles and distances needed to exercise state transitions. Real corner clearance, sensor latency and finish position remained to be measured.

## Reproduction and build

Run from the repository root with IDE-managed PlatformIO Python and host `g++` or `clang++` on the path.

```powershell
$py = Join-Path $env:USERPROFILE '.platformio/penv/Scripts/python.exe'
& $py simulation/review_open_challenge_20261027.py
& $py simulation/check_runtime_safety.py
& $py simulation/check_drive_profile_timing.py
& $py simulation/review_runtime_edges_20261027.py
& $py simulation/check_open_wall_following_20261028.py
```

The commands passed on the recorded revisions. `FINDING` means a known problem was reproduced. The drive-profile test recorded **0 mm/s** maximum speed difference in its **40,000** normal ramp comparisons. The runtime tests covered gyro recovery, data freshness, USB coordination and earlier control-loop timing fixes. Generated files stay under `local_workspace/open_challenge_review_20261027/` and `local_workspace/open_wall_following_20261028/`. The wall-following check generates its own baseline fixtures.

| IDE-managed M7 build | RAM (bytes) | Flash (bytes) |
| --- | ---: | ---: |
| 27 October | 442,408 / 523,624 | 524,296 / 786,432 |
| 28 October | 442,416 / 523,624 | 532,888 / 786,432 |

```text
2026-10-27 5542896c58ca0c36b2c389c0be2d63471b2a0dbeeb27a1cbedee4b44ab7b22d8
2026-10-28 ca500e0d2b6127929fa0ac99cf98164cd8e510be535fc7c708b3b12425a0f4d9
```

We built M7 incrementally with the existing cache and startup configuration. M4 was unchanged. Neither review uploaded firmware or ran the robot. The engineering follow-up was to measure first-corner timing and long accepted returns, and to separate the lap finish from wall correction. The inner-wall steering comparison did not call for a new planner. Any shared ToF filter change would also need validation in Obstacle.
