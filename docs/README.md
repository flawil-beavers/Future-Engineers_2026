# Repository guide

The [engineering journal](../README.md) explains how we designed, built and tested the car. The [print edition](../README.pdf) covers the same material and includes QR codes for the repository and both challenge videos.

## Quick build and first run

Follow the [quick build and run guide](guides/QUICK_BUILD_AND_FIRST_RUN.md) for the steps from the parts list to the first autonomous run.

## Current firmware

We publish the competition firmware on `main`. Build both cores from the same checkout and record `git rev-parse HEAD` and any changes to `include/config.h`. [Competition firmware on `main`](https://github.com/flawil-beavers/Future-Engineers_2026/tree/main) · [Build and upload](../README.md#build-and-run-the-firmware)

- `giga_r1_m7` runs the challenges, processes the camera and controls the drive and steering. [M7 entry point](../src/main.cpp)
- `giga_r1_m4` reads the rear ToF sensor. [M4 entry point](../src/m4/rear_tof_m4.cpp)
- PlatformIO defines both environments and their dependencies. [Build configuration](../platformio.ini)

## Reproduce the tests

| What you want to check | Where to start |
| --- | --- |
| How the car recognises signs, plans routes and parks | [Simulation commands and model assumptions](../simulation/README.md) · [Parking practice](guides/PARKING_ENTRY_PRACTICE.md) |
| What happened on the field | [Original run logs and SHA-256 hashes](../simulation/evidence/parking_exit_diagnostics/README.md) · [Run inspector](../tools/robot-run-inspector/README.md) |
| How we interpret obstacle data | [Sign positions](guides/OBSTACLE_SEAT_NUMBERING.md) · [Clearance logging](guides/OBSTACLE_CLEARANCE_LOGGING.md) |
| How we plan a practical test | [Obstacle Challenge checklist](validation/OBSTACLE_CHALLENGE_TEST_PLAN.md) · [Connector and first-lap checks](validation/CONNECTOR_NEXT_TEST.md) · [Parking-exit measurements](validation/PARKING_POSE_IMPLEMENTATION_PLAN.md) |

## Rules and project history

- [WRO 2026 rules and rubric](competition/WRO_2026_RULES.md)
- [Submission checklist](competition/DOCUMENTATION_TODO.md) and [proofreading checklist](competition/PROOFREADING_CHECKLIST.md)
- [Engineering history](history/README.md) and [current engineering handoff](../AGENT_DOCUMENTATION.md)
- [How we generate the print edition](pdf/README.md)
