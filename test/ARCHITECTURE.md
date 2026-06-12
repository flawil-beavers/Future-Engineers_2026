# Architecture Overview

## Before vs After

### BEFORE: Monolithic Structure (~1000 lines in main.cpp)
```
main.cpp
├── 100+ Global Variables
├── Pin Definitions
├── Motor Control
├── Steering Control
├── PID Algorithm
├── Sensor Reading (Gyro)
├── Sensor Reading (ToF x2)
├── Encoder Management
├── Serial Parsing
├── Ring Buffer Management
└── Main loop
```

**Problems:**
- ❌ Difficult to find code
- ❌ Hard to modify without breaking things
- ❌ Duplicate code patterns
- ❌ All variables global and intermingled
- ❌ Steep learning curve for new developers
- ❌ Difficult to test individual components

---

### AFTER: Modular Structure

```
┌─────────────────────────────────────────────────────────────────┐
│                        main.cpp (40 lines)                      │
│              Coordinates all subsystems in loop()                │
└────────────────────────────┬────────────────────────────────────┘
                             │
        ┌────────────────────┼────────────────────┐
        │                    │                    │
        ▼                    ▼                    ▼
   ┌─────────────┐    ┌──────────────┐    ┌──────────────┐
   │   MOTOR     │    │   SENSORS    │    │   SERIAL     │
   │  CONTROL    │    │  MANAGEMENT  │    │  HANDLER     │
   │             │    │              │    │              │
   │ motor_-     │    │ sensors.-    │    │ serial_-     │
   │ control     │    │ h/cpp        │    │ handler      │
   │ h/cpp       │    │              │    │ h/cpp        │
   └──────┬──────┘    └──────┬───────┘    └──────┬───────┘
          │                  │                   │
          │                  │                   │
   ┌──────▼──────────────────▼───────────────────▼──────────────┐
   │                     config.h                               │
   │  All Pins • Constants • Tuning Parameters • Timing values  │
   └────────────────────────────────────────────────────────────┘
```

**Benefits:**
- ✅ Easy to locate code
- ✅ Simple to modify subsystems
- ✅ Minimal code duplication
- ✅ Organized global variables
- ✅ New developers learn one module at a time
- ✅ Easy to test individual modules
- ✅ Scales to larger projects
- ✅ Professional structure

---

## Data Flow

```
                      Main Loop
                         │
        ┌────────────────┼────────────────┐
        │                │                │
        ▼                ▼                ▼
   Time Update      Serial Input       Safety Check
        │                │                │
        │         Process Commands        │
        │                │                │
        └────────────────┼────────────────┘
                         │
                    ▼ ▼ ▼ ▼ ▼
            ┌──────────────────────┐
            │   Motor Control      │
            │ - Steering angle     │
            │ - PID controller     │
            │ - Speed ramp-up      │
            └──────────────────────┘
                    │
         ┌──────────┴──────────┐
         ▼                     ▼
    ┌─────────────┐       ┌─────────────┐
    │   Sensors   │       │   Actuators │
    │ - Gyro (IMU)│       │ - DC Motor  │
    │ - ToF Left  │       │ - Servo     │
    │ - ToF Right │       └─────────────┘
    │ - Encoder   │
    └─────────────┘
```

---

## Module Responsibilities

### **config.h**
- Pin definitions for all hardware
- Tuning constants (PID, acceleration, etc.)
- Timing parameters
- Sensor update rates
- Buffer sizes
- Error thresholds

**Philosophy:** Single source of truth for configuration

---

### **motor_control.h/cpp**
**Responsibility:** Drive the robot

**Manages:**
- DC motor PWM control
- Motor direction (forward/backward)
- Speed ramping/acceleration
- Servo steering angle
- PID speed controller
- Encoder position tracking
- Stall detection

**Functions exported:**
- `set_speed()` - Queue a new speed command
- `set_steering()` - Queue a steering angle
- `drive_loop()` - Main control loop (call every cycle)
- `pid_config_print()` - Debug output

**Does NOT deal with:**
- Reading sensors
- Serial communication

---

### **sensors.h/cpp**
**Responsibility:** Provide sensor data

**Manages:**
- BNO085 Gyroscope (heading angle)
- VL53L4CX ToF Left Sensor (distance)
- VL53L4CX ToF Right Sensor (distance)
- I2C and SPI initialization
- Sensor reset/recovery
- Best measurement selection (lowest sigma)

**Functions exported:**
- `update_gyro()` - Read latest heading
- `update_lasers()` - Read latest distances
- `sensors_setup()` - Initialize all sensors

**Does NOT deal with:**
- Motor control
- Serial communication

---

### **serial_handler.h/cpp**
**Responsibility:** Communicate with operator/controller

**Manages:**
- Ring buffer for incoming serial data
- Command parsing
- Command execution routing
- Debug output formatting
- Parameter tuning via serial

**Functions exported:**
- `check_serial_available()` - Check for new data
- `parseMessage()` - Execute command
- `pid_config_print()` - Send debug telemetry

**Does NOT deal with:**
- Actual motor control (only calls control functions)
- Sensor reading directly (just reports sensor variables)

---

### **main.cpp**
**Responsibility:** Orchestrate the system

**Only contains:**
- `setup()` - Initialize all modules in order
- `loop()` - Call each module's update functions
- Minimal state tracking

**Philosophy:** Dumb coordinator, smart modules

---

## Communication Between Modules

```
motor_control.cpp
├── Reads: encoder_pos (from interrupts)
├── Reads: config.h constants
├── Writes: current_speed, current_dc, etc.
└── Uses: servo.write(), digitalWrite(), analogWrite()

sensors.cpp
├── Reads: I2C, SPI buses
├── Reads: config.h constants
├── Writes: current_degree, distances
└── Uses: Wire, Wire2, SPI1

serial_handler.cpp
├── Reads: motor_control variables (speed, dc, etc.)
├── Reads: sensors variables (degree, distances)
├── Reads: config.h constants
├── Calls: motor_control functions (set_speed, set_steering, etc.)
└── Writes: Serial output
```

**All data is accessed via:**
1. `extern` declarations in header files
2. Function calls between modules
3. Configuration constants from config.h

---

## Adding New Features

### Example: Add a "Limp Home" Mode

1. **Add configuration** in `config.h`:
   ```cpp
   #define LIMP_MODE_DC 100      // Reduced motor power
   ```

2. **Add function** in `motor_control.cpp`:
   ```cpp
   void limp_home_mode() {
     disable_servo = true;
     target_speed = 50; // Slow speed
     // ...
   }
   ```

3. **Add serial command** in `serial_handler.cpp`:
   ```cpp
   case 'l':
     limp_home_mode();
     break;
   ```

4. **Call from main.cpp** if needed:
   ```cpp
   // Nothing needed - serial handler calls it
   ```

**Result:** 1 new constant, 1 new function, 1 new command - isolated change!

---

## File Size Comparison

| File | Before | After | Change |
|------|--------|-------|--------|
| main.cpp | 990 lines | 40 lines | **-96%** ✨ |
| motor_control.cpp | (in main.cpp) | 300 lines | Extracted |
| sensors.cpp | (in main.cpp) | 350 lines | Extracted |
| serial_handler.cpp | (in main.cpp) | 200 lines | Extracted |
| config.h | (in main.cpp) | 110 lines | Extracted |
| **Total .cpp** | 990 lines | 890 lines | Same functionality, better organized |

---

## Team Development Impact

**Before (Monolithic):**
```
Developer A editing motor control
         ↓
         Conflicts with Developer B's serial changes?
         ↓
         Everyone blocks each other
```

**After (Modular):**
```
Developer A → motor_control.cpp
Developer B → serial_handler.cpp
Developer C → sensors.cpp
         ↓
         No conflicts! Can work in parallel
```

---

## Scalability

As the project grows:
- ✅ Easy to add new features (new module)
- ✅ Easy to support new sensors (extend sensors.cpp)
- ✅ Easy to add autonomous features (new module)
- ✅ Easy to implement vision processing (integrate camera)
- ✅ Easy to add path planning (new module)

**Foundation is solid for growth!**

