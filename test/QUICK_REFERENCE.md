# Quick Reference Guide

## Main Loop Flow
Every iteration of `loop()` calls these subsystems in order:
```
1. loop_updater()           → Update timing and calculate distances
2. check_serial_available() → Check for and process serial commands  
3. check_stalling()         → Safety check: stop if motor stalls
4. drive_loop()             → Execute motor/steering control logic
5. update_lasers()          → Read distance from ToF sensors
6. update_gyro()            → Read heading from gyroscope
```

## Where to Find Things

| Need | File | Function/Variable |
|------|------|-------------------|
| Change a pin | `include/config.h` | `#define *_PIN` |
| Adjust PID gains | `include/config.h` | `PID_Kp/Ki/Kd` |
| Modify motor behavior | `src/motor_control.cpp` | `drive_loop()`, `set_dc()` |
| Add sensor | `src/sensors.cpp` | Add function, call in loop |
| Add serial command | `src/serial_handler.cpp` | `parseMessage()` switch statement |
| Change servo limits | `include/config.h` | `SERVO_MAX_ANGLE`, `SERVO_MIN_ANGLE` |
| Debug output interval | `include/config.h` | `STATUS_PRINT_INTERVAL_US` |

## Common Tasks

### Add a new serial command
**Example: Add 'b' command to print battery voltage**

1. Add in `serial_handler.cpp`, inside `parseMessage()`:
```cpp
case 'b':
  Serial.print("Battery: ");
  Serial.println(analogRead(BATTERY_PIN));
  break;
```

2. Update documentation in `serial_handler.h` file header

### Tune motor responsiveness
Edit `include/config.h`:
```cpp
#define DEFAULT_ACCELERATION 700    // Higher = faster response (mm/s^2)
#define MOTOR_MAX_DC 200            // Lower = less power, more gentle
```

### Increase sensor update rate
Edit `include/config.h`:
```cpp
#define GYRO_UPDATE_INTERVAL_MS 20  // Lower = faster updates (but uses more CPU)
```

### Adjust motor minimum power
Edit `include/config.h`:
```cpp
#define MOTOR_MIN_DC (0.32 * 255)   // Minimum PWM to overcome friction
```

### Change steering range
Edit `include/config.h`:
```cpp
#define SERVO_CENTER 81             // Center position
#define SERVO_MAX_ANGLE (SERVO_CENTER + 60)  // +60° = full right
#define SERVO_MIN_ANGLE (SERVO_CENTER - 60)  // -60° = full left
```

## Serial Commands Reference

| Command | Format | Example | Effect |
|---------|--------|---------|--------|
| Drive | `d<speed>` | `d100` | Set speed to 100 mm/s |
| Steer | `s<angle>` | `s45` | Turn right 45° |
| Pause | `p` | `p` | Stop immediately |
| Hold | `h` | `h` | Hold position (don't move) |
| Resume | `r` | `r` | Resume with last speed |
| Get distance | `n` | `n` | Print encoder distance |
| Get heading | `g` | `g` | Print gyro angle |
| Get ToF | `v` | `v` | Print distance sensor readings |
| Set Kp | `q<value>` | `q9` | Kp = 0.9 (divide by 10) |
| Set Ki | `w<value>` | `w10` | Ki = 0.1 (divide by 100) |
| Set Kd | `e<value>` | `e5` | Kd = 0.5 (divide by 10) |
| Set accel | `a<accel>` | `a800` | Acceleration = 800 mm/s² |
| Master enable | `m` | `m` | Enable motors and servo |
| Steering timing | `x` | `x` | Print servo timing info |
| **Wall Follow START** | `l` | `l` | Start autonomous wall following |
| **Wall Follow STOP** | `z` | `z` | Stop wall following |
| **Set wall distance** | `u<mm>` | `u300` | Set target wall distance (300mm) |
| **Debug output ON** | `i` | `i` | Enable wall follower debug output |
| **Debug output OFF** | `o` | `o` | Disable wall follower debug output |

## Module Exports

### motor_control.h
**Global Variables** (extern in header):
- `encoder_pos` - Current encoder position
- `current_speed` - Current speed (mm/s)
- `target_speed` - Desired speed
- `current_dc` - Current duty cycle
- `Kp`, `Ki`, `Kd` - PID tuning

**Functions** (always available):
- `set_speed(int speed)` - Command new speed
- `set_steering(int angle)` - Command steering angle
- `drive_loop()` - Main control loop
- `motor_control_setup()` - Initialize subsystem

### sensors.h
**Global Variables**:
- `current_degree` - Heading in degrees (-180 to +180)
- `current_distance_left_m` - Left ToF in meters
- `current_distance_right_m` - Right ToF in meters

**Functions**:
- `update_gyro()` - Read gyro (call every loop)
- `update_lasers()` - Read sensors (call every loop)
- `sensors_setup()` - Initialize subsystem

### serial_handler.h
**Functions**:
- `check_serial_available()` - Check for data (call every loop)
- `parseMessage(char *msg)` - Parse command string
- `pid_config_print()` - Print debug telemetry

## Troubleshooting

| Problem | Check |
|---------|-------|
| Motor doesn't respond | `MOTOR_MIN_DC` threshold too high in config.h |
| Servo doesn't move | `disable_servo` is true (send `m` command) |
| Slow steering response | `SERVO_CENTER` value (default 81 might be off) |
| Gyro readings erratic | `GYRO_UPDATE_INTERVAL_MS` too short |
| ToF reads -1.0m | Sensors not initialized or out of range |
| Commands not working | Check serial baud rate (115200 default) |
| Motor overshoots target | Increase `PID_Kd` to add damping |
| Motor hunts around target | Decrease `PID_Ki` integral gain || Wall follower not starting | Check sensors working with `v` command first |
| Robot oscillates | Decrease Kp gain in navigation_controller |
| Robot doesn't detect gaps | May need to reduce `wf_wall_margin` |

## Wall Follower Quick Start

```
i              Enable debug output
u300           Set wall distance to 300mm
w              Start wall following (follows right wall initially)
              Robot autonomously follows perimeter for 3 rounds
v              Check sensor readings during mission
z              Stop wall follower anytime
o              Disable debug output
```

**For detailed wall follower guide:** See [WALL_FOLLOWER_GUIDE.md](WALL_FOLLOWER_GUIDE.md)
## Compilation Notes

Ensure `platformio.ini` has these libs:
```ini
[env:...]
lib_deps =
    Adafruit BNO08x
    VL53L4CX  
    Servo
```

All modules automatically included via `#include` statements in main.cpp.

## Key Design Patterns

### 1. **Configuration Centralization**
All tunable values in `config.h` for easy experimentation

### 2. **Function Interface**
Each module exports functions, no direct variable access outside module

### 3. **Modular Timing**
Each subsystem can have its own update rate via timing checks

### 4. **Command Routing**
Serial handler routes commands to correct subsystem

### 5. **Safe Defaults**
All critical values have sensible defaults that should work

---

**For detailed architecture, see:** `ARCHITECTURE.md`
**For restructuring details, see:** `RESTRUCTURING.md`
