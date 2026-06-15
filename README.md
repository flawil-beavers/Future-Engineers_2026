# WRO Future Engineers 2026 – Flawil Beavers 🇨🇭

## Team Introduction

We are the **Flawil Beavers**, a team from Switzerland participating in the **World Robot Olympiad (WRO) Future Engineers 2026** category.

Our goal is to design, build and program a fully autonomous vehicle capable of completing all Future Engineers challenges in a reliable, repeatable and efficient manner.

Throughout the development process we focused on engineering principles, iterative improvement, system reliability and maintainability. Every subsystem was tested individually and as part of the complete robot to ensure consistent performance under competition conditions.

---

# Team Members

| Name | Occupation |
|--------|--------|
| Philipp Kündig | Student ETH |
| Damian Hardegger | Electrician |


Coach: 
- Stefan Gemperli
<br>
<br>   

![Team Photo](Photos/Team-photo.JPG)

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
| Voltage Display | 20 mA |

### Total Average Consumption

Average current consumption:

```text
200 + 15 + 40 + 40 + 120 + 250 + 300 + 20

≈ 985 mA
≈ 1.0 A
```

### Peak Current Consumption

During acceleration, steering corrections and obstacle avoidance, the motor and servo can draw significantly more current.

| Component | Peak Current |
|------------|------------|
| Drive Motor | 800 mA |
| MG90S Servo | 700 mA |
| Remaining Electronics | 435 mA |

Estimated peak current:

```text
≈ 1.9 A
```

### Engineering Decision

The power system was designed with sufficient reserve capacity to maintain stable operation under all competition conditions.

During development, a commercial battery holder was initially used. Testing revealed occasional mechanical movement of the batteries, which could potentially affect electrical reliability. To solve this issue, a custom 3D-printed battery holder was designed and manufactured.

Advantages of the custom battery holder:

- Improved battery fixation
- Increased electrical reliability
- Better use of available chassis space
- Easier maintenance and battery replacement

This modification significantly increased the overall robustness of the robot.
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
# Bill of Materials (BOM)

## Electronics

| Component | Quantity | Purpose |
|------------|------------|------------|
| Arduino GIGA R1 WiFi | 1 | Main robot controller |
| Adafruit BNO085 9-DOF IMU | 1 | Orientation and heading estimation |
| Arducam BO462 Camera | 1 | Wall, obstacle and parking detection |
| Adafruit VL53L4CX ToF Sensor | 2 | Distance measurement and wall tracking |
| SparkFun Flexible Qwiic Cable 100mm | 2 | Sensor communication |
| Pololu Dual MC33926 Motor Driver | 1 | Drive motor control |
| SVM330-Y Digital Voltmeter | 1 | Battery voltage monitoring |
| MTS-102 ON-ON Toggle Switch | 4 | Power and subsystem switching |

---

## Drive System

| Component | Quantity | Purpose |
|------------|------------|------------|
| DFRobot Micro Metal Geared Motor 155 RPM (100:1) | 1 | Main drive motor |
| Rear Drive Wheels (43.2 mm diameter) | 2 | Vehicle propulsion |
| Axles and Bearings | Various | Mechanical transmission |

### Alternative Motor Tested

| Component | Quantity | Result |
|------------|------------|------------|
| DFRobot Micro Metal Geared Motor 310 RPM (50:1) | 1 | Rejected due to insufficient torque and lower driving consistency |

---

## Steering System

| Component | Quantity | Purpose |
|------------|------------|------------|
| TowerPro MG90S Metal Gear Servo | 1 | Front wheel steering |

### Alternative Servo Tested

| Component | Quantity | Result |
|------------|------------|------------|
| PDI-1102HB 2g Digital Micro Servo | 1 | Rejected due to insufficient steering torque |

---

## Power System

| Component | Quantity | Purpose |
|------------|------------|------------|
| 14500 Li-Ion Battery (3.7V, 750mAh) | 2 | Main power source |
| Custom 3D Printed Battery Holder | 1 | Secure battery mounting |
| Battery Wiring Harness | 1 | Power distribution |

### Previous Design

| Component | Quantity | Result |
|------------|------------|------------|
| Commercial 2xAA Battery Holder | 2 | Replaced due to poor mechanical reliability |

---

## Mechanical Components

| Component | Quantity | Purpose |
|------------|------------|------------|
| Custom 3D Printed Chassis | 1 | Main robot structure |
| Custom Sensor Mounts | Multiple | Sensor positioning |
| Custom Battery Holder | 1 | Battery retention |
| Screws, Nuts and Spacers | Various | Assembly and mounting |

---

## Robot Specifications

| Parameter | Value |
|------------|------------|
| Total Weight | 670 g |
| Wheelbase | 130 mm |
| Track Width | 100 mm |
| Wheel Diameter | 43.2 mm |
| Drive Configuration | Rear-Wheel Drive |
| Steering Configuration | Front-Wheel Steering |

---

## Cost Overview

| Component | Approximate Cost (CHF) |
|------------|------------|
| Arduino GIGA R1 WiFi | 60.90 |
| BNO085 IMU | 24.95 |
| VL53L4CX Sensors (2x) | 39.80 |
| Qwiic Cables | 15.40 |
| MG90S Servo | 7.90 |
| DFRobot 155 RPM Motor | 10.90 |
| 14500 Batteries (4x) | 31.60 |
| Voltmeter | ~5.00 |
| Switches | 7.60 |
| Pololu MC33926 Driver | ~25.00 |
| Arducam BO462 Camera | ~30.00 |
| 3D Printed Parts | ~10.00 |

### Estimated Total Cost

**≈ 270–300 CHF**

(The exact total depends on manufacturing costs, spare parts and shipping fees.)

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
# Failure Mode Analysis

## Failure Mode 1 – Steering Servo Too Weak

### Problem

The initial prototype used a very small micro servo for steering.

### Observed Issues

- Inconsistent steering angles
- Reduced steering precision
- Difficulty maintaining wheel position under load

### Root Cause

The servo did not provide enough torque to reliably control the steering mechanism during dynamic driving situations.

### Solution

The servo was replaced with a TowerPro MG90S metal gear servo.

### Result

- Increased steering torque
- Improved steering precision
- Better repeatability
- Higher mechanical durability

---

## Failure Mode 2 – Battery Holder Instability

### Problem

The original commercial battery holder allowed slight battery movement during operation.

### Observed Issues

- Reduced mechanical stability
- Potential intermittent electrical contact
- Difficult maintenance

### Root Cause

The battery holder was not specifically designed for the robot's vibration and acceleration loads.

### Solution

The team designed and manufactured a custom 3D-printed battery holder.

### Result

- Secure battery mounting
- Improved electrical reliability
- Better chassis integration
- Easier battery replacement

---

## Failure Mode 3 – Damaged ToF Sensors During Assembly

### Problem

The original VL53L3CX sensors were damaged during soldering.

### Observed Issues

- Sensor failures
- Increased replacement costs
- Additional development delays

### Root Cause

The small sensor boards required delicate soldering work, increasing the risk of accidental damage.

### Solution

The design was changed to Adafruit VL53L4CX modules using Qwiic/STEMMA QT connectors.

### Result

- No soldering required
- Fast sensor replacement
- Reduced assembly risk
- Improved maintainability

---

## Failure Mode 4 – Drive Motor Torque Too Low

### Problem

A 310 RPM geared motor was initially tested.

### Observed Issues

- Reduced acceleration
- Poor performance under load
- Less reliable obstacle avoidance
- Inconsistent driving behaviour

### Root Cause

The motor prioritised speed over torque and was unable to provide sufficient force for reliable autonomous navigation.

### Solution

The 155 RPM motor with a 100:1 gearbox was selected.

### Result

- Twice the available torque
- Improved acceleration
- More consistent driving behaviour
- Better obstacle avoidance performance

---

## Failure Mode 5 – Wheel Slip

### Problem

Wheel encoders alone can accumulate navigation errors.

### Observed Issues

- Position drift
- Inaccurate turns
- Reduced repeatability

### Root Cause

Wheel slip occurs due to changing surface conditions and mechanical tolerances.

### Solution

A BNO085 IMU was integrated into the navigation system.

### Result

- Improved heading estimation
- More accurate turns
- Reduced accumulated navigation error
- Better lap consistency

---

## Failure Mode 6 – Changing Lighting Conditions

### Problem

Competition environments may contain varying lighting conditions.

### Observed Issues

- Reduced colour recognition reliability
- Potential object detection errors

### Root Cause

Camera-based systems are sensitive to illumination changes and reflections.

### Solution

The vision system uses:

- HSV colour space processing
- Adjustable thresholds
- Extensive testing under different lighting conditions

### Result

- Improved robustness
- More reliable obstacle detection
- Better overall navigation performance

---

## Failure Mode 7 – Sensor Cable Failure

### Problem

Loose or damaged sensor connections can lead to communication errors.

### Potential Impact

- Loss of distance measurements
- Reduced navigation performance

### Solution

The robot uses Qwiic/STEMMA QT connector systems and flexible cables designed for repeated use.

### Result

- Improved connection reliability
- Easier maintenance
- Faster component replacement

---

## Failure Mode 8 – Battery Voltage Drop

### Problem

Battery voltage decreases during operation.

### Potential Impact

- Reduced motor speed
- Inconsistent robot behaviour

### Solution

A closed-loop PID speed controller continuously adjusts motor power based on encoder feedback.

### Result

- Constant vehicle speed
- Improved repeatability
- Consistent performance throughout a run

---

## Engineering Conclusion

The development process revealed several hardware and software weaknesses that were systematically addressed through testing and iteration.

Each identified failure mode resulted in a design improvement that increased:

- Reliability
- Repeatability
- Maintainability
- Competition readiness

This iterative engineering process was a key factor in the development of the final robot.   



The national menu of Switzerland is fondue. While usually fondue is made from cheese, the team is enjoying a robot fondue 😎🤪
---
![Team funny](Photos/Funny-team-photo.JPG)