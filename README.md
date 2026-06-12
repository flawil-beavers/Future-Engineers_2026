# WRO Future Engineers 2026 – Flawil Beavers 🇨🇭

## Team Introduction

We are the **Flawil Beavers**, a team from Switzerland participating in the **World Robot Olympiad (WRO) Future Engineers 2026** category.

Our goal is to design, build and program a fully autonomous vehicle capable of completing all Future Engineers challenges in a reliable, repeatable and efficient manner.

Throughout the development process we focused on engineering principles, iterative improvement, system reliability and maintainability. Every subsystem was tested individually and as part of the complete robot to ensure consistent performance under competition conditions.

---

# Team Members

| Name | Role |
|--------|--------|
| Team Member 1 | Hardware & CAD |
| Team Member 2 | Software Development |
| Team Member 3 | Electronics & Testing |

Coach:

- Coach Name

---

# Project Overview

The robot combines computer vision, inertial navigation, distance sensing and closed-loop control to autonomously navigate through the WRO Future Engineers competition field.

The system was developed according to the following engineering goals:

- Reliability
- Repeatability
- Maintainability
- Robust autonomous navigation
- Easy troubleshooting
- Modular architecture

The final robot consists of:

- Arduino GIGA R1 WiFi
- Arducam BO462 Camera
- BNO085 IMU
- VL53L4CX Time-of-Flight Sensors
- MG90S Steering Servo
- DFRobot Geared Drive Motor
- Custom 3D-Printed Battery Holder

---

# Engineering Process

The development followed a structured engineering workflow.

## Phase 1 – Requirements Analysis

The WRO Future Engineers regulations were analysed to identify all functional requirements.

Main requirements:

- Fully autonomous operation
- Obstacle avoidance
- Reliable navigation
- Consistent turning behaviour
- Accurate parking
- Robust performance under changing conditions

## Phase 2 – Concept Development

Several possible architectures were evaluated.

The team compared:

- Different drive motors
- Different steering servos
- Various sensor combinations
- Multiple power system concepts

## Phase 3 – Prototype Development

Initial prototypes were built and tested.

Weaknesses and failures were documented and used to improve the design.

## Phase 4 – Final Integration

After multiple iterations all subsystems were integrated into the final robot.

---

# Mechanical Design

## Chassis

The chassis was designed to provide:

- High rigidity
- Low centre of gravity
- Easy maintenance
- Efficient component placement

Special attention was given to the placement of heavy components to maximise stability.

---

## Battery System

### Initial Design

The original design used a commercial battery holder.

During testing several problems were discovered:

- Batteries could move slightly
- Electrical contacts were not always reliable
- Maintenance was difficult

### Final Design

The team designed and manufactured a custom 3D-printed battery holder.

Advantages:

- Secure battery fixation
- Improved electrical reliability
- Better use of available space
- Easier maintenance

This modification significantly improved overall system robustness.

---

# Mobility System

## Drive Motor

Two different drive motors were evaluated.

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

- Lower torque
- Reduced acceleration
- More inconsistent driving behaviour
- Reduced obstacle avoidance reliability

The slower motor consistently produced better results.

---

## Final Decision

The 155 RPM motor was selected.

Reasons:

- Higher torque
- Better acceleration under load
- More reliable cornering
- Better repeatability

Although the maximum speed is lower, the overall challenge performance is better.

This decision demonstrates a trade-off between speed and reliability.

---

# Steering System

## Initial Prototype

The first steering design used a very small micro servo.

Problems observed:

- Insufficient steering force
- Reduced steering precision
- Inconsistent wheel positioning

---

## Final Solution

### TowerPro MG90S Metal Gear Servo

The MG90S was selected because it provides:

- Higher torque
- Metal gears
- Better durability
- More accurate steering

The stronger servo significantly improved navigation consistency.

---

# Electronics Architecture

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

The BNO085 provides:

- Heading information
- Rotation measurements
- Orientation estimation

### Why an IMU?

Wheel encoders alone are insufficient because:

- Wheels can slip
- Surface friction varies
- Small errors accumulate

The IMU allows the robot to determine its orientation independently of wheel movement.

This improves turning precision and repeatability.

---

# Distance Sensors

## Initial Design

The original design used VL53L3CX Time-of-Flight sensors.

Unfortunately several sensors were damaged during soldering.

This revealed an important engineering problem.

### Problems

- Delicate solder joints
- Difficult replacement process
- Increased maintenance effort

---

## Final Design

### Adafruit VL53L4CX

The team switched to connector-based VL53L4CX modules using Qwiic/STEMMA QT connectors.

Advantages:

- No direct soldering
- Fast replacement
- Improved reliability
- Easier maintenance

This design significantly reduced the risk of sensor failure.

---

# Camera System

## Arducam BO462

The camera is the primary perception sensor.

Functions:

- Wall detection
- Obstacle detection
- Direction determination
- Parking detection

### Why a Camera?

Alternative solutions were considered.

| Technology | Advantages | Disadvantages |
|------------|------------|------------|
| Ultrasonic Sensors | Simple | Limited information |
| ToF Sensors Only | Accurate distance | Limited environmental awareness |
| Camera | Rich information | Higher processing requirements |

The camera provides the greatest amount of information and therefore became the primary perception system.

---

# Software Architecture

The software is divided into independent modules.

```text
Main Controller
│
├── Camera Processing
├── Obstacle Detection
├── Wall Detection
├── Distance Sensor Processing
├── IMU Processing
├── Navigation
├── Steering Control
├── Parking Logic
└── Communication
```

This modular structure simplifies development and debugging.

---

# Navigation Strategy

The robot continuously performs the following loop:

1. Read sensors
2. Process camera image
3. Detect obstacles
4. Estimate position
5. Calculate steering correction
6. Update motor commands
7. Repeat

This process runs continuously throughout the challenge.

---

# State Machine

```text
INITIALIZATION
      ↓
DIRECTION DETECTION
      ↓
WALL FOLLOWING
      ↓
OBSTACLE DETECTION
      ↓
OBSTACLE AVOIDANCE
      ↓
LAP COUNTING
      ↓
PARKING
      ↓
STOP
```

The state machine ensures predictable and structured robot behaviour.

---

# Obstacle Detection Strategy

The robot combines camera data and distance sensor measurements.

Processing pipeline:

1. Capture image
2. Colour filtering
3. Contour extraction
4. Object classification
5. Position estimation
6. Steering calculation

This approach allows reliable obstacle detection under varying conditions.

---

# Engineering Trade-Offs

## Speed vs Reliability

A faster motor could theoretically improve lap times.

Testing showed:

Advantages:

- Higher speed

Disadvantages:

- Lower torque
- Less stable driving
- Worse obstacle avoidance

Final decision:

Use the slower but stronger motor.

---

## Small Servo vs MG90S

Advantages of small servo:

- Lower weight

Disadvantages:

- Insufficient steering force

Final decision:

Use MG90S.

Reliability was prioritised over minimal weight savings.

---

## Soldered Sensors vs Connector-Based Sensors

Advantages of soldered sensors:

- Lower cost

Disadvantages:

- Difficult replacement
- Higher risk during assembly

Final decision:

Connector-based VL53L4CX modules.

Maintenance and reliability improved significantly.

---

# Failure Analysis

## Sensor Failure

Potential cause:

- Damaged sensor

Mitigation:

- Modular connectors
- Fast replacement

---

## Wheel Slip

Potential cause:

- Surface variation

Mitigation:

- IMU feedback
- Heading correction

---

## Low Battery Voltage

Potential cause:

- Battery discharge

Mitigation:

- Voltage monitoring
- Regular battery replacement

---

## Camera Lighting Issues

Potential cause:

- Reflections
- Changing illumination

Mitigation:

- Colour filtering
- Threshold optimisation

---

# Testing and Validation

## Mechanical Testing

Tested:

- Steering precision
- Wheel alignment
- Stability

---

## Sensor Testing

Tested:

- IMU accuracy
- Distance sensor repeatability
- Camera robustness

---

## Navigation Testing

Tested:

- Turn consistency
- Obstacle avoidance
- Parking accuracy
- Full challenge runs

---

# Iterative Development

## Version 1

Features:

- Original battery holder
- Small steering servo
- Initial sensor setup

Problems:

- Steering weakness
- Battery instability

---

## Version 2

Improvements:

- New battery holder
- Improved steering

Problems:

- Sensor reliability

---

## Version 3

Improvements:

- VL53L4CX integration
- Connector-based wiring

Result:

- Increased reliability
- Easier maintenance

---

## Final Version

The final robot combines:

- Reliable steering
- Stable power system
- Robust sensor integration
- Accurate navigation

---

# Bill of Materials

| Component | Quantity |
|------------|------------|
| Arduino GIGA R1 WiFi | 1 |
| Adafruit BNO085 IMU | 1 |
| Arducam BO462 Camera | 1 |
| VL53L4CX ToF Sensor | 2 |
| MG90S Servo | 1 |
| DFRobot 155RPM Geared Motor | 1 |
| 14500 Li-Ion Batteries | 4 |
| Custom 3D Printed Battery Holder | 1 |
| Qwiic/STEMMA QT Cables | 4 |
| Power Switches | Multiple |

---

# Repository Structure

```text
Future-Engineers_2026
│
├── README.md
├── CAD
│   ├── Fusion360
│   ├── STL
│   └── Technical Drawings
│
├── Electronics
│   ├── Wiring Diagram
│   ├── Schematics
│   └── BOM
│
├── Software
│   ├── Arduino
│   ├── Vision
│   └── Documentation
│
├── Testing
│   ├── Mechanical Tests
│   ├── Sensor Tests
│   └── Challenge Results
│
├── Photos
│
├── Videos
│
└── Engineering Journal
```

---

# Future Improvements

Potential future developments:

- Sensor fusion enhancements
- Improved path planning
- Advanced obstacle classification
- Additional autonomous behaviours

---

# Conclusion

The final robot is the result of a continuous engineering process involving testing, evaluation and iterative improvement.

Throughout development, reliability was prioritised over theoretical performance. This resulted in a robust and maintainable robot capable of consistently completing the Future Engineers challenges.

The project demonstrates the application of engineering principles, autonomous systems design, software development and iterative problem solving in a real-world robotics competition environment.