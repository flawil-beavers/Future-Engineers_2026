# WRO Future Engineers 2026 – Flawil Beavers

## Team Introduction

We are the **Flawil Beavers** from Switzerland participating in the **World Robot Olympiad (WRO) Future Engineers 2026** category.

Our goal is to design, build and program a fully autonomous vehicle capable of solving the Future Engineers challenges reliably and consistently.

---

# Robot Overview

The robot is based on a modular architecture consisting of:

- Raspberry Pi for high-level processing
- Arduino for low-level control
- Camera for environment perception
- IMU for heading estimation
- Servo steering system
- Rear-wheel drive system

The design focuses on:

- Reliability
- Repeatability
- Easy maintenance
- Fast repairs during competition
- Robust obstacle detection

---

# Engineering Design Process

The development process followed several engineering stages:

1. Requirement analysis
2. Mechanical design
3. Electronics integration
4. Software development
5. Testing and validation
6. Iterative improvements

Each subsystem was developed and tested separately before being integrated into the final robot.

---

# Mechanical Design

## Chassis

The chassis was designed to provide:

- High rigidity
- Low centre of gravity
- Easy access to components
- Efficient weight distribution

The battery is mounted low in the chassis to improve stability during turns.

## Drive System

The vehicle uses a rear-wheel drive system optimized for:

- Reliable acceleration
- Consistent speed control
- Smooth cornering

## Steering System

A servo-based steering mechanism controls the front wheels.

Advantages:

- High precision
- Fast response
- Simple control architecture

---

# Electronics Architecture

## Main Components

| Component | Function |
|------------|------------|
| Raspberry Pi | Vision and navigation |
| Arduino | Motor and servo control |
| Camera | Object detection |
| IMU | Heading estimation |
| Motor Driver | Power control |
| Servo | Steering |
| Battery | Power supply |

## Communication

The Raspberry Pi communicates with the Arduino via serial communication.

Responsibilities:

### Raspberry Pi

- Image processing
- Obstacle detection
- Navigation decisions
- Direction determination

### Arduino

- Motor control
- Steering control
- Sensor acquisition
- Real-time execution

---

# Software Architecture

The software is divided into independent modules.

```text
Main Controller
│
├── Camera Processing
├── Obstacle Detection
├── Wall Detection
├── Navigation
├── IMU Processing
├── Steering Control
└── Communication
```

This structure simplifies maintenance and debugging.

---

# Navigation Strategy

The robot continuously performs the following cycle:

1. Capture camera image
2. Detect walls and obstacles
3. Determine robot position
4. Calculate steering correction
5. Update motor commands
6. Repeat

---

# Obstacle Detection

The camera is used to identify:

- Green obstacles
- Red obstacles
- Walls
- Parking area

The image is processed using colour filtering and contour detection.

This approach allows reliable detection while maintaining real-time performance.

---

# Heading Estimation

The IMU is used to:

- Measure rotation
- Track orientation
- Improve turn consistency

Combining camera information with IMU data increases overall navigation accuracy.

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

The state machine guarantees predictable robot behaviour.

---

# Testing

Extensive testing was performed during development.

## Mechanical Tests

- Steering accuracy
- Wheel alignment
- Chassis stability

## Vision Tests

- Obstacle detection
- Wall detection
- Lighting robustness

## Navigation Tests

- Lap consistency
- Turning accuracy
- Parking precision

Results from these tests were used to improve both hardware and software.

---

# Engineering Decisions

## Camera vs Distance Sensors

A camera was selected because it provides significantly more information than simple distance sensors.

Advantages:

- Multiple object detection
- Flexible software processing
- Future expandability

## Raspberry Pi + Arduino

This architecture separates high-level and low-level tasks.

Benefits:

- Improved reliability
- Better real-time behaviour
- Easier debugging

## Low Centre of Gravity

Keeping heavy components low improves stability and reduces the risk of tipping during aggressive manoeuvres.

---

# Repository Structure

```text
Future-Engineers_2026
│
├── README.md
├── CAD
├── Electronics
├── Software
├── Photos
├── Videos
├── Testing
└── Documentation
```

---

# Media

## Robot Photos

Add:

- Front view
- Rear view
- Left side
- Right side
- Top view

## Videos

Required WRO videos:

- Open Challenge
- Obstacle Challenge

---

# Future Improvements

Potential future developments include:

- Sensor fusion improvements
- Advanced path planning
- Improved obstacle classification
- Enhanced autonomous parking

---

# Conclusion

The Flawil Beavers Future Engineers robot combines computer vision, inertial sensing and modular software architecture to achieve reliable autonomous navigation.

The design prioritizes robustness, repeatability and maintainability while fulfilling the requirements of the WRO Future Engineers competition.Don't forget to download the Arduino IDE when cloning this repository and uploading the code using PlatformIO. Then download the Arduino Mbed OS GIGA Board Package.
