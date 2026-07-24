# Implementation Plan - USB Logging for WRO Robot

Implement non-blocking logging of Serial outputs to a USB flash drive connected to the Arduino GIGA R1's USB-A port.

## Proposed Design & Architecture

To satisfy the requirements and ensure the robot's control loop speed is not halted during execution, we propose a buffered asynchronous logging scheme:

1. **RAM Buffering during execution**:
   - We will allocate a `128 KB` buffer in the Arduino GIGA R1's SRAM (since it has 1MB, this is very safe).
   - A custom `USBLogger` class (inheriting from Arduino's `Print` class) will intercepts all `Serial.print` and `Serial.println` calls.
   - When printing, data is written directly to the RAM buffer and forwarded to the USB Serial CDC interface (the laptop connection) *only if* it is active. No USB disk writes occur during normal loop execution.

2. **USB write on stopping events**:
   - The RAM buffer will be flushed to the USB flash drive when:
     - The robot is paused (switch flicked to `LOW`), which triggers `system_disable()`.
     - An error is raised (such as stall detection), which also triggers `system_disable()`.
     - The robot completes the course (state transitions to `GF_STOPPED`), triggering `state_stopped()`.
   - Halting the CPU for a second to perform USB file operations is safe here since the robot has already stopped or is being disabled.
   - **Visual Shutdown/Removal Indicator**: The GIGA R1's onboard RGB LED will light up solid **RED** during the active USB connection/mount/write/unmount sequence (takes < 2 seconds). It will turn off (or turn **GREEN**) when done, signaling that it is safe to power off the robot or unplug the USB stick.

3. **Non-blocking USB initialization**:
   - The USB-A port is enabled via pin `PA_15` set to `HIGH`.
   - The `Arduino_USBHostMbed5` library is used to mount the USB FAT filesystem.
   - We do NOT block startup in `setup()` if the USB drive is missing. Instead, initialization and mounting are attempted dynamically when flushing the log. If a USB drive is not plugged in, the RED LED will turn off quickly, and a warning is printed to Serial.


4. **Sequential File Naming**:
   - Logs are saved as `/usb/log_1.txt`, `/usb/log_2.txt`, etc., by finding the first available integer, preventing older runs from being overwritten.
   - The RAM buffer is cleared at the start of each run (when `system_enable()` is called) to start fresh.

---

## User Review Required

> [!IMPORTANT]
> The USB flash drive must be formatted as **FAT32** with the **MBR (Master Boot Record)** partition table. The `FATFileSystem` class from Mbed cannot mount GPT or exFAT/NTFS volumes.

---

## Open Questions

None at this time. The requirements are clear, and the proposed design covers all criteria including loop speed protection and event-driven logging.

---

## Proposed Changes

### [Core Components]

#### [NEW] [logger.h](file:///c:/Users/philk/Documents/GitHub/Future-Engineers_2026/include/logger.h)
- Declare `USBLogger` class extending `Print`.
- Declare methods for writing, initializing USB, flushing to USB, and clearing the buffer.

#### [NEW] [logger.cpp](file:///c:/Users/philk/Documents/GitHub/Future-Engineers_2026/src/logger.cpp)
- Implement `USBLogger` buffer buffering.
- Implement sequential file finding and file writing using `fopen`/`fprintf`.
- Implement dynamic USB-A power enable, connection with a short timeout, and file mounting.

#### [MODIFY] [config.h](file:///c:/Users/philk/Documents/GitHub/Future-Engineers_2026/include/config.h)
- Include `logger.h` and `#define Serial robot_logger` to redirect all serial prints.

#### [MODIFY] [platformio.ini](file:///c:/Users/philk/Documents/GitHub/Future-Engineers_2026/platformio.ini)
- Add `arduino-libraries/Arduino_USBHostMbed5` to `lib_deps`.

#### [MODIFY] [motor_control.cpp](file:///c:/Users/philk/Documents/GitHub/Future-Engineers_2026/src/motor_control.cpp)
- Call `robot_logger.clear()` in `system_enable()`.
- Call `robot_logger.write_to_usb()` in `system_disable()`.

#### [MODIFY] [wall_follower.cpp](file:///c:/Users/philk/Documents/GitHub/Future-Engineers_2026/src/wall_follower.cpp)
- Call `robot_logger.write_to_usb()` in `state_stopped()`.

---

## Verification Plan

### Automated Tests
- Build the project using `pio run` to verify clean compilation with the new `Arduino_USBHostMbed5` library and memory allocations.

### Manual Verification
- Deploy to the robot.
- Insert a FAT32 MBR USB flash drive into the USB-A port.
- Perform a manual drive, then flick the switch to pause: verify a new log file `/usb/log_X.txt` is created containing the serial log.
- Run the autonomous wall follower: verify that it finishes 3 laps, stops, and saves the log.
- Unplug the USB drive and run the robot: verify it does not block or crash.
