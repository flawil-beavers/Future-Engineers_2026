# WRO Future Engineers 2026 – Flawil Beavers 🇨🇭

## Team Introduction

We are the **Flawil Beavers**, a team from Switzerland competing in the WRO Future Engineers 2026 category.

This documentation presents our autonomous vehicle, including its mechanical design, electronics and software. Our goal was to build a reliable and efficient robot capable of successfully completing the competition challenges.

---

## Team Members

<table>
<tr>
<td valign="top">

<b>Team Members</b>

Philipp Kündig <br>
Damian Hardegger

</td>
<td valign="top">

<b>Coach</b>

Stefan Gemperli

</td>
</tr>
</table>

<br>

![Team Photo](Photos/Team-photo.JPG)

---

## Table of Contents

- [Team Introduction](#team-introduction)
- [Team Members](#team-members)
- [Assembly](#assembly)
- [Mechanical Design](#mechanical-design)
- [Mobility System](#mobility-system)
- [Steering System](#steering-system)
- [Power Budget](#power-budget)
- [Electronics Architecture](#electronics-architecture)
- [Sensor Architecture](#sensor-architecture)
- [Distance Sensors](#distance-sensors)
- [Camera System](#camera-system)
- [Wiring Diagram](#wiring-diagram)
- [Bill of Materials (BOM)](#bill-of-materials-bom)
- [Software Architecture](#software-architecture)
- [Navigation Strategy](#navigation-strategy)
- [State Machine](#state-machine)
- [Video](#video)
- [Robot Photos](#robot-photos)
- [Failure Mode Analysis](#failure-mode-analysis)
- [Conclusion](#conclusion)

---

<!-- Todo: move Bill of materials up here -->

# Assembly

Gather all required materials according to the [**Bill of Materials (BOM)**](#bill-of-materials) and print all necessary **[3D‑Printed‑Parts](./CAD/Car_v71.3mf)**. Just use 0.2 mm layer height and no support.  
Use **M3 screws** and **M3nS/M3n nuts** for all mechanical components, and **M2 and M 2.5 screws** with **M2n and M2.5n nuts** for electronic parts.

Start by building the base and then working your way up to the second stage. Always wire all components you add. See the following images for a better understanding:

| Building Instruction | | |
|---|---|---|
| ![General Pieces](Photos/build-instructions/PXL_20260531_062612987.MP.jpg) Perpare all the printed parts | ![Servo linkage](Photos/build-instructions/PXL_20260531_062619468.MP.jpg) Screw the servo linkage onto the servo | ![Steering Rod](Photos/build-instructions/PXL_20260531_062629739.MP.jpg) Connect the tie rod to both of the steering arms using M3n nylon lock nuts |
| ![DC Motor](Photos/build-instructions/PXL_20260531_062952891.MP.jpg) Connect the DC Motor Shaft to the DC Motor| ![Base Plate](Photos/build-instructions/PXL_20260531_063232503.MP.jpg) Fix the DC motor onto the first stage using the DC Motor Cover and screw the servo in place| ![Battery Holder assembly](Photos/build-instructions/PXL_20260531_063344414.MP.jpg) Perpare the Battery holder (2x) |
| ![Battery Holder Connection](Photos/build-instructions/PXL_20260531_065603142.MP.jpg) Screw the battery holders onto the first stage | ![Battery Connector Wiring](Photos/build-instructions/PXL_20260531_065845130.MP.jpg) Connect protruding screws to the wires and screw nut on | ![Wiring](Photos/build-instructions/PXL_20260531_071506514.MP.jpg) Use M2 screws to fix the motor driver in place and wire everything up (use Wagos for GND and VCC) |
| ![Lego Build Step](Photos/build-instructions/PXL_20260531_072809927.MP.jpg) Connect the lego gears, axles and wheels | ![Second Stage Sensor montage](Photos/build-instructions/PXL_20260531_074835064.MP.jpg) Now fasten the sensors to the Second Stage using M2.5 screws and nuts | ![M2.5 Nuts location](Photos/build-instructions/PXL_20260531_074841767.MP.jpg) The ToF sensors are mounted via the ToF Mounts that are fastened with M3 screws |
| ![Arduino Connection](Photos/build-instructions/PXL_20260531_075033876.MP.jpg) Place the Arduino Giga on top of the Arduino Spacer and fasten with M3 Screws | ![Cable Management](Photos/build-instructions/PXL_20260601_121901176.MP.jpg) Wire the DC-DC-Converter and the Logic Level Shifter according to the schematics | ![Cables moving up](Photos/build-instructions/PXL_20260601_121915804.MP.jpg) Thread the wires to the arduino through the Second stage |
| ![Connecting both stages](Photos/build-instructions/PXL_20260601_123524626.MP.jpg) Screw the second stage onto the first one | | |

---

# Mechanical Design

## Chassis

The chassis was designed to provide:

- High rigidity: We used screws and nuts to connect all 3D printed parts
- Low centre of gravity: Special attention was given to the placement of heavy components (e.g. Batteries) to maximise stability
- Easy maintenance: No glue and tape (just for electric isolation), electrical connections with jumper wires
- Efficient component placement: Used up almost all space available in our robot
- Easy 3D printability: All printed parts are printable without any supports necessary

### Steering

We used Ackermann steering to enable the robot to turn without the steering wheels slipping. As visible in the following image, the steering was created by drawing a line from the turning point of the Steering Arm to the middle of the back axis.
![Ackermann Steering Construction](Photos/Ackermann-construction.png)

By constructing our own steering mechanism, we could focus on creating a very tight turning radius, so to improve future parking and unparking.

One major difficulty in designing the steering was calculating the exact spacing between servo and base plate. We had to make sure that all the connections between 3D printed parts were parallel so to minimise friction when steering.

---

## Battery System

### Custom Battery Holders

The initial design used a commercial battery holder. However, during the mechanical integration process, it became clear that mounting a standard dual battery holder underneath the vehicle was difficult and inefficient in terms of space usage.

To solve this problem, we designed and manufactured our own battery holders. The custom design allowed us to mount one holder on each side of the underside of the vehicle, making much better use of the available space.

This arrangement also improved the vehicle's weight distribution by placing the batteries lower and more symmetrically, resulting in a lower center of gravity. In addition, the custom holders make battery replacement faster and more convenient during testing and maintenance.

Overall, the self-designed battery holders provided a more **compact**, **practical** and **competition-ready solution** than the original commercial holders.

---

# Mobility System

## Drive Motor

Two different drive motors were tested to compare their performance and driving characteristics. After evaluating speed, torque and controllability, the motor that provided the best overall performance for our vehicle was selected.

### Option 1

DFRobot Micro Metal Geared Motor

- 310 RPM
- 50:1 Gear Ratio
- 0.35 kg·cm Torque

### Option 2

DFRobot Micro Metal Geared Motor

- 155 RPM
- 100:1 Gear Ratio
- 0.70 kg·cm Torque

---

## Testing Results

The faster motor initially appeared advantageous because of its higher theoretical speed.

However practical testing revealed:

- Lower torque and the robot had difficulty driving with the faster motor.

The slower motor consistently produced better results.

---

## Final Decision

The 155 RPM motor was selected.

Reasons:

- Higher torque

Although the maximum speed is lower, the overall challenge performance is better and we have no problems with him suddenly getting stuck.

This decision demonstrates a trade-off between speed and reliability.

---

# Steering System

## Initial Prototype

The first steering design used a very small micro servo because we wanted to be as space-saving as possible.

Problems observed:

- Insufficient steering force
- Inconsistent wheel positioning

## Final Solution

### TowerPro MG90S Metal Gear Servo

The MG90S was selected because it provides:

- Higher torque
- Metal gears
- Better durability
- More accurate steering

The more powerful servo drive significantly improved navigation accuracy but requires a bit more space. However, it's worth it.

---

# Power Budget

## Power System Overview

The robot is powered by four 14500 Li-Ion cells and distributes power to all electronic subsystems.

### Estimated Current Consumption

| Component | Typical Current |
|------------|------------|
| Arduino GIGA R1 WiFi | 200 mA |
| BNO085 IMU | 15 mA |
| VL53L4CX ToF Sensor #1 | 40 mA |
| VL53L4CX ToF Sensor #2 | 40 mA |
| Arducam BO462 Camera | 120 mA |
| MG90S Servo (average) | 250 mA |
| Drive Motor 155 RPM | 300 mA |
| MC33926 Motor Driver | 10 mA |
| LM2596S-5 DC-DC Converter | 5 mA |
| TXS0104E Logic Level Converter | 2 mA |
| Voltage Display | 20 mA |
| **Total** | **1002 mA** |

### Peak Current Consumption

During acceleration, steering corrections and obstacle avoidance, the motor and servo can draw significantly more current.

| Component | Peak Current |
|------------|------------|
| Drive Motor | 800 mA |
| MG90S Servo | 700 mA |
| Remaining Electronics | 472 mA |
| **Estimated peak current** | **≈ 2.0 A** |

---

# Electronics Architecture

<!-- Todo: Explain each electronic component in more detail:
            - Add section about why logic level convert is used
            - Add section about why we use voltage display -->

## Main Controller

### Arduino GIGA R1 WiFi

The Arduino GIGA R1 WiFi acts as the central controller.

Reasons for selection:

- High processing power
- Large memory capacity
- Multiple communication interfaces
- Excellent expandability

The controller manages:

- Sensor communication
- Navigation algorithms
- Steering control
- Motor control

---

# Sensor Architecture

## BNO085 IMU

### Purpose

The BNO085 was selected because of its high accuracy, low drift and reliable sensor fusion capabilities. During development, several orientation measurement methods were evaluated to determine which provided the most consistent results for autonomous navigation.

### Usage

In our vehicle, the gyroscope is the primary function used from the IMU. It is used to verify and control the robot's turning angles, providing accurate and repeatable rotation measurements. This approach has proven to be highly reliable during testing.

The magnetometer was also evaluated but was ultimately not used. We observed significant heading drift and instability, most likely caused by magnetic interference from the motor, wiring and other electronic components inside the vehicle.

The accelerometer is not used for position or speed estimation. Instead, we rely on the motor encoder for movement feedback, which provides more consistent and predictable results for our application.

By focusing on the gyroscope data, we achieved accurate turn control while avoiding the limitations of the other sensors in our specific robot design.

---

# Distance Sensors

## Initial Design

The first version of the vehicle used VL53L3CX Time-of-Flight sensors. While these sensors performed well, they required direct soldering of wires to the module, making integration and maintenance more difficult.

During development, one of the sensors was damaged during the soldering process. This highlighted the need for a more robust and serviceable solution.

## Final Design

### Adafruit VL53L4CX

The team replaced the original sensors with Adafruit VL53L4CX modules. In addition to offering improved range and signal performance, these modules support Qwiic/STEMMA QT connectors.

Advantages of the new design:

- Improved sensing range
- Stronger and more reliable signal quality
- No direct soldering required
- Fast sensor replacement
- Cleaner cable management
- More flexible integration

Using connector-based modules significantly improved reliability and maintainability. Sensors can now be connected or replaced within seconds, reducing the risk of damage and making the overall system more robust for competition use.

---

# Camera System

## Arducam BO462

The Arducam BO462 was selected as the camera for our vehicle because it is specifically designed for use with the Arduino GIGA R1 WiFi. In addition, extensive documentation, examples and community resources are available online, making development and troubleshooting easier.

### Purpose

The camera is intended to be used for object and color detection during the competition. Its main task is to identify relevant objects and determine their colors rather than capturing highly detailed images.

### Advantages

- Designed for the Arduino GIGA R1 WiFi
- Well documented with many online examples
- Easy integration into our system
- Low image resolution reduces processing requirements
- Suitable for real-time object and color detection

The relatively low resolution is an advantage for our application because it reduces the amount of data that must be processed by the microcontroller, allowing for faster image analysis.

### Challenges

One limitation of the camera is its relatively narrow field of view and limited detection range. Depending on the final challenge setup, this may require additional software optimization or careful positioning of the camera.

At the current stage of development, the camera has been successfully connected and tested. However, it has not yet been fully integrated into the autonomous driving system, and further development is planned.

---

# Wiring Diagram

![Wiring Diagram](Electronics/Wiring_Diagram_FE2026.png)
---

# Bill of Materials (BOM)

## Electronics

| Component                           | Quantity | Purpose                                 |
| ----------------------------------- | -------- | --------------------------------------- |
| Arduino GIGA R1 WiFi                | 1        | Main robot controller                   |
| Adafruit BNO085 9-DOF IMU           | 1        | Gyroscope-based orientation measurement |
| Arducam BO462 Camera                | 1        | Object and color detection              |
| Adafruit VL53L4CX ToF Sensor        | 2        | Distance measurement and wall tracking  |
| SparkFun Flexible Qwiic Cable 100mm | 2        | Sensor communication                    |
| Pololu Dual MC33926 Motor Driver    | 1        | Drive motor control                     |
| LM2596S-5 DC-DC Converter           | 1        | Voltage regulation for electronics      |
| TXS0104E Logic Level Converter      | 1        | 3.3V ↔ 5V signal level translation      |
| SVM330-Y Digital Voltmeter          | 1        | Battery voltage monitoring              |
| MTS-102 ON-ON Toggle Switch         | 2        | Power and subsystem switching           |

---

## Drive System

| Component                                        | Quantity | Purpose                 |
| ------------------------------------------------ | -------- | ----------------------- |
| DFRobot Micro Metal Geared Motor 155 RPM (100:1) | 1        | Main drive motor        |
| Rear Drive Wheels (43.2 mm diameter)             | 2        | Vehicle propulsion      |
| Axles and Bearings                               | Various  | Mechanical transmission |

### Alternative Motor Tested

| Component                                       | Quantity | Result                                                            |
| ----------------------------------------------- | -------- | ----------------------------------------------------------------- |
| DFRobot Micro Metal Geared Motor 310 RPM (50:1) | 1        | Rejected due to insufficient torque and lower driving consistency |

---

## Steering System

| Component                       | Quantity | Purpose              |
| ------------------------------- | -------- | -------------------- |
| TowerPro MG90S Metal Gear Servo | 1        | Front wheel steering |

### Alternative Servo Tested

| Component                         | Quantity | Result                                       |
| --------------------------------- | -------- | -------------------------------------------- |
| PDI-1102HB 2g Digital Micro Servo | 1        | Rejected due to insufficient steering torque |

---

## Power System

| Component                           | Quantity | Purpose                 |
| ----------------------------------- | -------- | ----------------------- |
| 14500 Li-Ion Battery (3.7V, 750mAh) | 2        | Main power source       |
| Custom 3D Printed Battery Holder    | 2        | Secure battery mounting |
| Battery Wiring Harness              | 1        | Power distribution      |

### Previous Design

| Component                      | Quantity | Result                                                         |
| ------------------------------ | -------- | -------------------------------------------------------------- |
| Commercial 2xAA Battery Holder | 1        | Replaced due to inefficient mounting and reduced accessibility |

---

## Mechanical Components

| Component                 | Quantity | Purpose               |
| ------------------------- | -------- | --------------------- |
| Custom 3D Printed Chassis | 1        | Main robot structure  |
| Custom Sensor Mounts      | Multiple | Sensor positioning    |
| Custom Battery Holders    | 2        | Battery retention     |
| Screws, Nuts and Spacers  | Various  | Assembly and mounting |

---

## Robot Specifications

| Parameter              | Value                |
| ---------------------- | -------------------- |
| Total Weight           | 670 g                |
| Wheelbase              | 100 mm               |
| Track Width            | 100 mm               |
| Wheel Diameter         | 43.2 mm              |
| Drive Configuration    | Rear-Wheel Drive     |
| Steering Configuration | Front-Wheel Steering |

---

## Cost Overview

| Component                      | Approximate Cost (CHF) |
| ------------------------------ | ---------------------- |
| Arduino GIGA R1 WiFi           | 60.90                  |
| BNO085 IMU                     | 24.95                  |
| VL53L4CX Sensors (2x)          | 39.80                  |
| Qwiic Cables                   | 15.40                  |
| MG90S Servo                    | 7.90                   |
| DFRobot 155 RPM Motor          | 10.90                  |
| 14500 Batteries (2x)           | 15.80                  |
| Voltmeter                      | ~5.00                  |
| Switches                       | 7.60                   |
| Pololu MC33926 Driver          | ~25.00                 |
| LM2596S-5 DC-DC Converter      | ~2.00                  |
| TXS0104E Logic Level Converter | ~3.00                  |
| Arducam BO462 Camera           | ~30.00                 |
| 3D Printed Parts               | ~10.00                 |

### Estimated Total Cost

**≈ 255–285 CHF**

(The exact total depends on manufacturing costs, spare parts and shipping fees.)

# Software Architecture

The software is built on a modular, non-blocking architecture designed for real-time responsiveness on the Arduino GIGA R1.

```text
Main Controller
System Entry (main.cpp)
│
├── Sensors (BNO085 Gyro via SPI, Dual VL53L4CX ToF via I2C)
├── Motor Control (DC Motor PID, Servo Mapping)
├── Wall Follower (Hybrid Navigation State Machine)
└── Serial Handler (Telemetry and Command Parsing)
```

### Key Implementation Details

- **Asynchronous Polling:** Sensors are polled based on hardware readiness flags (using the `BNO085_INT` pin) to prevent CPU stalling.
- **Slew Rate Limiting:** Distance readings use a delta-limit filter (`TOF_MAX_DELTA_MM`) to reject outlier "ghost" readings.
- **Dynamic Timing Budgets:** The ToF sensors automatically switch to a high-budget "Discovery Mode" (300 ms) when searching for walls, then return to high-speed mode (30 ms) for active following.

---

# Navigation Strategy

To solve the WRO Open Challenge (3 laps), we implement a **Hybrid Gyro-Stabilized Wall Following** strategy. This decouples heading stability from lateral distance maintenance.

### 1. Geometric Correction (Incidence Compensation)

Unlike basic robots that use raw ToF data, our system calculates the **perpendicular distance** to the wall. Using the gyro heading relative to the target, we apply a cosine correction:

$$
d_{perp} = d_{raw} \cdot \cos(\theta_{error})
$$

This prevents "phantom distance" increases when the robot is tilted relative to the wall, which is critical for stable PD control.

### 2. Combined Control Law

The steering output is the sum of two PD (Proportional-Derivative) controllers:

- **Primary Control:** Gyroscope-based PD maintains a global grid heading (0°, 90°, 180°, 270°).
- **Secondary Control:** ToF-based PD nudges the robot to maintain exactly 300 mm from the followed wall.

---

# State Machine

Our navigation logic is managed by a robust state machine (`NavigationState`):

```text
NAV_IDLE      -> System waiting for start switch or serial command.
NAV_FOLLOWING -> Combined Gyro + ToF PD control. Tracks turns and laps.
NAV_TURNING   -> Executes 90° pivot turns based on gyro integration.
NAV_STOPPED   -> Triggered after 12 turns (3 laps) are completed.
```

### Corner Detection Logic

The robot detects a corner when the followed wall's distance suddenly exceeds the `nav_wall_margin`. It then increments the `nav_turn_count` and transitions to `NAV_TURNING`.

After 12 successful turns, the mission is automatically flagged as complete, and the robot enters `NAV_STOPPED`.

---

# Video

Opening Challenge

[![Watch the Opening Challenge](https://img.youtube.com/vi/7w7cAxLPb28/maxresdefault.jpg)](https://youtu.be/7w7cAxLPb28)

# Robot Photos

| ![Left](Photos/robot/PXL_20260617_185159148.MP.jpg) | ![Back](Photos/robot/PXL_20260617_185207747.MP.jpg) | ![Right](Photos/robot/PXL_20260617_185218350.MP.jpg) |
|---|---|---|
| ![Front](Photos/robot/PXL_20260617_185228987.MP.jpg) | ![Top](Photos/robot/PXL_20260617_185256705.MP.jpg) | ![Bottom](Photos/robot/PXL_20260617_185318976.MP.jpg) |

---

# Failure Mode Analysis

## Failure Mode 1 – Insufficient Steering Servo Torque

### Problem

The first steering prototype used a very small micro servo. While it was lightweight and compact, it struggled to control the steering mechanism reliably during driving.

### Observed Issues

- Inconsistent steering angles
- Reduced steering precision
- Steering position changed under load

### Root Cause

The servo did not provide enough torque for the steering system, especially during quick direction changes and when friction forces acted on the wheels.

### Solution

Several servos were evaluated and compared. The original servo was replaced with a TowerPro MG90S metal gear servo, which provided significantly higher torque and a more robust gear train.

### Result

- More accurate steering
- Improved repeatability
- Better control during turns
- Increased mechanical durability

---

## Failure Mode 2 – Battery Holder Integration

### Problem

The original design used a commercial battery holder. Integrating it into the chassis proved difficult because it occupied valuable space underneath the vehicle and limited the placement options.

### Observed Issues

- Inefficient use of space
- Less favourable weight distribution
- Batteries were more difficult to access

### Root Cause

The commercial holder was not designed for the geometry and packaging requirements of the robot.

### Solution

The team designed and manufactured two custom 3D-printed battery holders. This allowed one battery to be mounted on each side of the underside of the chassis.

### Result

- Better use of available space
- Improved weight distribution
- Lower centre of gravity
- Faster battery replacement
- Cleaner chassis integration

---

## Failure Mode 3 – ToF Sensor Damage During Assembly

### Problem

The initial design used VL53L3CX Time-of-Flight sensors that required direct soldering.

### Observed Issues

- One sensor was damaged during assembly
- Sensor replacement was time-consuming
- Maintenance became more difficult

### Root Cause

The small sensor boards required delicate soldering work, increasing the risk of accidental damage.

### Solution

The sensors were replaced with Adafruit VL53L4CX modules. These offered improved range, stronger signal performance and support for Qwiic/STEMMA QT connectors.

### Result

- No direct soldering required
- Faster replacement process
- Improved reliability
- Cleaner wiring
- Easier maintenance

---

## Failure Mode 4 – Drive Motor Selection

### Problem

Two different drive motors were evaluated during development.

### Observed Issues

The 310 RPM motor provided higher speed but showed less consistent driving behaviour.

### Root Cause

The motor prioritised speed over torque, making precise speed control and obstacle avoidance more difficult.

### Solution

Both motors were tested on the vehicle under realistic conditions. After comparing acceleration, controllability and consistency, the 155 RPM motor with a 100:1 gearbox was selected.

### Result

- Higher available torque
- Improved acceleration
- Better speed control
- More reliable autonomous driving

---

## Failure Mode 5 – Magnetometer Drift

### Problem

The BNO085 magnetometer was initially considered for heading estimation.

### Observed Issues

- Unstable heading measurements
- Significant drift over time
- Inconsistent orientation readings

### Root Cause

Magnetic interference from the motor, wiring and surrounding electronics affected the magnetometer measurements.

### Solution

The magnetometer was removed from the navigation system. Instead, the robot relies primarily on the gyroscope to measure and verify turning angles.

### Result

- More stable orientation measurements
- Improved turning accuracy
- Greater repeatability

---

## Failure Mode 6 – Sensor Connection Reliability

### Problem

Poor electrical connections, damaged cables or weak solder joints can cause intermittent sensor failures.

### Observed Issues

- Sporadic communication errors
- Missing sensor readings
- Difficult troubleshooting

### Root Cause

Connection problems often appear similar to software bugs. As a result, debugging can take a long time because the software is usually investigated before a hardware fault is discovered.

### Solution

The team reduced the number of soldered connections wherever possible and adopted Qwiic/STEMMA QT connectors for sensor integration.

### Result

- Improved reliability
- Faster troubleshooting
- Easier maintenance
- Reduced risk of wiring faults

---

## Failure Mode 7 – Reduced ToF Performance on Black Surfaces

### Problem

The competition field contains black walls and markings that absorb a large portion of the infrared light emitted by the ToF sensors.

### Observed Issues

- Reduced signal strength
- Less reliable measurements at longer distances
- Occasional invalid readings

### Root Cause

The VL53L4CX measures distance using reflected infrared light. Black surfaces reflect significantly less infrared light than bright surfaces, meaning fewer photons return to the sensor.

### Solution

The measurement timing budget was increased, reducing the sampling rate and allowing the sensor to collect more reflected photons per measurement. Additional software filtering was implemented to reject measurements with poor signal quality.

### Result

- Improved measurement stability
- Better performance on dark surfaces
- Reliable wall detection during normal operation
- Consistent navigation close to walls

---

## Failure Mode 8 – Battery Voltage Drop

### Problem

Battery voltage decreases during operation as the batteries discharge.

### Observed Issues

- Reduced motor performance
- Changes in vehicle speed
- Potential loss of repeatability

### Root Cause

A lower battery voltage reduces the power available to the drive motor.

### Solution

A closed-loop PID speed controller continuously adjusts the motor output using encoder feedback.

### Result

- Constant vehicle speed
- Improved repeatability
- Consistent behaviour throughout a run

## Conclusion

The project showed that the best results come from consistent testing, evaluation, and adjustment. Hardware and software issues were identified and resolved step by step, making the vehicle more stable, reliable, and easier to operate.

As a result, the team was able to develop a ready-to-drive solution that responds better to changing conditions with controlled motor management and a robust sensor setup. This work not only improved performance, but also increased repeatability and competition readiness of the system

---

![Team funny](Photos/Funny-team-photo.JPG)
