# Dual-Core Logging Analysis: M7 & M4 Cores on Arduino GIGA R1

Comparing the single-core RAM-buffered logging approach against a dual-core architecture where the M4 core continuously logs to the USB drive.

---

## 1. How Dual-Core USB Logging Works

The Arduino GIGA R1 houses an **STM32H747** microcontroller with two cores:
* **Cortex-M7 (Main Core)**: Runs at 480 MHz. Handles navigation, sensor updates, PID loop, and motor/servo controls.
* **Cortex-M4 (Co-processor)**: Runs at 240 MHz. Can be booted from the M7 core to run its own sketch.

In a dual-core logging configuration:
1. The **M7 Core** redirects all serial output to a virtual inter-core port using the **RPC (Remote Procedure Call)** library (specifically `RPC1`, which establishes a high-speed shared-memory buffer between the two cores).
2. The **M4 Core** runs a separate sketch, initializes the `Arduino_USBHostMbed5` library, and continuously polls the `RPC1` stream.
3. When `RPC1` receives log data, the **M4 Core** writes it to the USB drive.

---

## 2. Comparison Table

| Feature / Criteria | Single-Core RAM-Buffered (Proposed) | Dual-Core Continuous (Alternative) |
| :--- | :--- | :--- |
| **Control Loop Lag** | **Zero**. RAM writes take nanoseconds. USB writes only occur when stopped/paused. | **Zero**. RPC writes are non-blocking shared-memory operations. USB writes are offloaded to M4. |
| **Real-time Logging** | No. Logs are flushed to disk only at the end of the run or on pause/error. | **Yes**. Logs are saved to the USB drive in real-time as the robot drives. |
| **Log Size Limit** | **Limited by RAM**. Max ~128-256 KB buffer size. | **Virtually unlimited**. Only limited by the USB drive's capacity. |
| **Crash Protection** | Logs may be lost if the board loses power completely during a run. | Logs up to the point of power failure are saved. |
| **Development Complexity** | **Low**. Single code base, standard PlatformIO build and deployment. | **High**. Requires writing, compiling, and flashing two separate sketches (M7 and M4). |
| **File System Corruption Risk** | **Extremely Low**. Disk writes only happen when motors are off (stable battery voltage/no noise). | **Medium**. Unplugging the battery during active runs can corrupt the FAT partition table. |
| **Power Consumption** | **Low**. Cortex-M4 core remains in a low-power sleep state. | **Higher**. Both cores are active, drawing more battery current. |

---

## 3. Implementation Complexity & PlatformIO Challenges

Deploying a dual-core project on PlatformIO requires configuring two independent environments in `platformio.ini`:
1. **M7 Core Environment**: Compiles the main robot control code and exports the M4 binary as a header to load it.
2. **M4 Core Environment**: Compiles the logging code.

### Inter-core RPC Overhead
While RPC is fast, the M4 core still has to keep up with the data rate. If the M7 core floods `RPC1` with debug messages faster than the M4 core can write to the USB drive (typically 50-60 KB/s), the M4 core will either drop messages or its input buffer will overflow, blocking RPC on the M7 core and stalling the robot anyway.

### USB Host Interrupt Sharing
The USB OTG hardware is shared between the cores. Mbed OS maps the USB interrupts to the M7 core by default. Reconfiguring the framework to hand over USB peripheral control to the M4 core can lead to unstable compilation flags and runtime core lockups.

---

## 4. Conclusion & Recommendation

* **Single-Core RAM-Buffered (Recommended)**: Best for reliability, simplicity, and safety. A 128KB buffer is more than enough for a 2-minute WRO run. Since the USB drive is only written to when the robot is already stationary and disabled, we avoid all risk of control loop stalls and electrical noise-induced disk corruption.
* **Dual-Core Continuous**: Only necessary if you need to capture long-duration runs (hours) or if the robot is prone to complete hardware crashes where RAM contents are lost. The overhead and complexity of dual-core debugging in PlatformIO are major trade-offs.

---

## 5. USB Shutdown Safety & Preventing Corruption (Field Testing)

When testing on the field, the main concern is **corrupting the USB drive** if the main power switch is flipped while the drive is active.

### Safety Analysis of the RAM-Buffered Design

1. **During Active Runs (Driving)**:
   - **100% Safe**. The USB port is powered down (`PA_15` is `LOW`), the filesystem is unmounted, and there is zero USB communication. You can cut the robot's power at any point during active driving with **zero risk** of USB stick corruption.
2. **On Pause / Completion / Error (USB Writing Window)**:
   - When the robot stops, it powers the USB port, mounts it, writes the buffered log file, and unmounts it.
   - This active write window lasts **less than 2 seconds**.
   - If power is cut *during* these 2 seconds, the FAT filesystem could be corrupted.

### Design Recommendation for Safe Shutdown

To ensure you can shut off the robot without thinking, we will implement **Visual LED Status Feedback** using the GIGA R1's onboard RGB LED:

* **LED off/green**: USB is not in use. You can safely power off the robot or unplug the USB stick.
* **LED solid RED (or BLUE)**: USB is actively writing. **Do not cut power**.

#### Lifecycle of a Write Operation:
```
[Event: Pause / Finish]
        │
        ▼
Turn On RGB RED LED  <-- Signals "Writing in Progress, DO NOT Power Off"
        │
Power USB Port (PA_15 = HIGH)
        │
Mount Filesystem & Write Log File
        │
Unmount Filesystem & Power Down USB Port (PA_15 = LOW)
        │
        ▼
Turn Off RGB RED LED <-- Signals "Safe to Power Off / Remove USB"
```

This simple visual cue ensures that you can confidently flip the power switch at any time, just waiting for the LED to go out if you've recently paused the robot.

