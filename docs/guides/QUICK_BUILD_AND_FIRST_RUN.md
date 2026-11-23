# Quick build and first run

[Repository guide](../README.md)

1. **Gather the materials.** Check materials and quantities against the [bill of materials](../../README.md#bill-of-materials-bom).
2. **Print the parts.** Open the v74 PrusaSlicer project. Keep the saved orientations and slice in PLA without supports. [Print files and settings](../../CAD/README.md#current-design-and-printing)
3. **Assemble the chassis.** Build the lower stage first. Fit the upper-stage electronics and route the cables before joining the stages. Check wheelbase, straight-ahead width and steering centre. Make sure the wheels turn freely and clear the chassis and cables at full lock. [Assembly photos](../../README.md#mechanical-assembly) · [Reference dimensions](../../README.md#robot-specifications) · [Steering calibration](../../CAD/README.md#workflow)
4. **Wire and check the electronics.** Label both ends of each cable using the [revision 1 schematic and pin table](../../README.md#wiring-and-signal-interfaces).

   - **Power and switches.** Battery holders, main rocker (SW2), power distribution and voltmeter leads, plus the start toggle (SW1).
   - **Drive and steering.** Motor leads to driver channel M2, driver control jumpers, encoder leads, and the servo lead with its control signal routed through the level shifter.
   - **Sensors.** Three separate STEMMA QT/Qwiic pigtails with jumpers for the ToFs, SPI jumpers for the IMU, and the 20-pin camera connection.

   With the cells removed, check continuity, wiring polarity and for shorts between supply and return. With SW1 LOW, check the pack feed (**7.4 V nominal, 8.4 V fully charged**), the Mini-360 output (**5 V**, before connecting the servo), and the logic and sensor supply (**3.3 V**). Switch SW2 off before changing connections. [Power rails](../../README.md#power-budget)

5. **Build and upload the firmware.** Update your local `main` and record its commit with `git rev-parse HEAD`. Connect the GIGA with a USB data cable and use PlatformIO in VS Code. Keep SW1 LOW and upload M4 first, then M7 from the same checkout. [Competition firmware on `main`](https://github.com/flawil-beavers/Future-Engineers_2026/tree/main) · [Commands and mode settings](../../README.md#build-and-run-the-firmware)
6. **Check the car on the bench.** Lift the drive wheels clear, leave SW1 LOW and turn on SW2. Confirm the motor stays still and the M4 rear ToF connects. At **115200 baud**, use `n`, `g` and `v` to check encoder travel, heading and ToF. [Serial commands](../../src/serial_handler.cpp#L324-L368) · [Rear ToF check](REAR_TOF_M4_SETUP.md#stationary-setup-check)
7. **Check motion.** On a clear straight, run a short low-speed test. Confirm forward motion, increasing encoder travel and a stop when SW1 goes LOW. Check for binding, stalling or steering pull. [Drive test and tuning](PID_SETUP.md#test-and-adjust-the-gains)
8. **Validate the mounted sensors.** Compare ToF readings with known distances and IMU heading with a known turn. Check red and green signs at measured distances in field lighting. [Sensor placement](../../README.md#sensor-placement) · [Camera calibration](../../simulation/reports/SURPRISE_RULE_MEASURED_VALIDATION_20261119.md#image-measurements) · [Stationary camera capture tool](../../scripts/capture-camera-image.py)
9. **Make the first autonomous run.** Select Open Challenge to check wall following, turns and stopping. Unplug the programming cable, insert a FAT32 USB drive, and start with SW1 once the car is ready. Check the parking start and Surprise Rule setting before running Obstacle. Record the commit, direction, outcome and any contact or intervention. [Start and logging](../../README.md#build-upload-and-reproduction) · [Obstacle checklist](../validation/OBSTACLE_CHALLENGE_TEST_PLAN.md#before-a-normal-run)
