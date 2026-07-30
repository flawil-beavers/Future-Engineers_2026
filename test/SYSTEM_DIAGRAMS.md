# System Architecture - Wall Following

## Data Flow Diagram

```
┌──────────────────────────────────────────────────────────────────┐
│                         Serial Input                              │
│                    (USB Terminal @ 115200)                        │
└────────────────────────────┬─────────────────────────────────────┘
                             │
                    ┌────────▼──────────┐
                    │ serial_handler.cpp│
                    │  parseMessage()   │
                    └────────┬──────────┘
                             │
                  ┌──────────┴──────────┐
                  │                     │
          (Motor Command)      (Wall Follower Command)
                  │                     │
       ┌──────────▼─────────┐ ┌────────▼────────────┐
       │ motor_control.cpp  │ │ navigation_controller.cpp   │
       │   (manual mode)    │ │   (autonomous mode) │
       └──────────┬─────────┘ └────────┬────────────┘
                  │                     │
                  └─────────┬───────────┘
                            │
                   ┌────────▼──────────┐
                   │  Motor Outputs:   │
                   │ - set_speed()     │
                   │ - set_steering()  │
                   │ - stop()          │
                   └────────┬──────────┘
                            │
        ┌───────────────────┼───────────────────┐
        │                   │                   │
        ▼                   ▼                   ▼
    ┌────────┐          ┌────────┐         ┌────────┐
    │ DC Motor│         │ Servo  │         │Encoder │
    │ (Speed) │         │(Steer) │         │(Speed) │
    └────────┘          └────────┘         └────────┘
        │                   │                   │
        │                   │ Feedback          │
        └───────────┬───────┴─────────┬────────┘
                    │                 │
              ┌─────▼────────────────▼─────┐
              │    Sensor Readouts:        │
              │ - current_distance_left_m  │
              │ - current_distance_right_m │
              │ - current_degree (gyro)    │
              │ - encoder_pos              │
              └─────┬──────────────────────┘
                    │
        ┌───────────┴─────────────┐
        │                         │
        ▼                         ▼
   ┌──────────┐           ┌────────────┐
   │ sensors. │           │navigation_controller│
   │cpp       │           │.cpp        │
   └─────────┬┘           └─────┬──────┘
             │                  │
             └──────────┬───────┘
                        │
                   Main Loop
```

## Wall Following State Machine - Detailed

```
┌─────────────────────────────────────────────────────────────────────┐
│ IDLE State                                                          │
│ - Robot stopped, motors disabled                                   │
│ - Waiting for enable signal                                        │
│ - No sensor reading                                                │
│ - No output to motors                                              │
└────────────────┬────────────────────────────────────────────────────┘
                 │
                 │ Serial Command: 'w'
                 │ navigation_enable()
                 │
                 ▼
┌─────────────────────────────────────────────────────────────────────┐
│ FOLLOWING State                                                     │
│ - Robot moving forward at 200 mm/s                                 │
│ - PD controller maintains distance from wall                       │
│ - Steering angle = Kp × error + Kd × d(error)/dt                 │
│ - Continuous sensor reading                                        │
│                                                                     │
│ Logic:                                                              │
│   1. Read distance from followed wall (left or right)             │
│   2. Calculate error = target_distance - current_distance        │
│   3. Steering = 0.5 × error + 0.1 × d(error)                    │
│   4. Check opposite wall distance                                 │
│   5. If opposite_distance > 1.0m → Turn!                        │
│                                                                     │
│ Output:                                                             │
│   set_speed(200)        ← Keep moving                             │
│   set_steering(angle)   ← Maintain distance                       │
└────────┬──────────────────────────────────────────────────────────┘
         │
         │ Opposite wall distance > 1.0m
         │ wf_turn_count++
         │ if (wf_turn_count % 4 == 0) wf_completed_rounds++
         │ if (wf_completed_rounds >= 3) → STOPPED
         │
         ▼
┌─────────────────────────────────────────────────────────────────────┐
│ TURNING State                                                       │
│ - Robot executing 90° pivot turn                                   │
│ - Duration: 1500ms (tunable)                                      │
│ - Speed reduced to 150 mm/s for stability                         │
│ - Steering angle: full right (+60°) or left (-60°)               │
│                                                                     │
│ Logic:                                                              │
│   1. Timer: elapsed = millis() - turn_start_time                 │
│   2. If elapsed < 1500: Keep turning                             │
│   3. If elapsed >= 1500: Turn complete, switch wall             │
│                                                                     │
│ Output:                                                             │
│   set_speed(150)            ← Slower during turn                 │
│   set_steering(±60)         ← Full steering angle                │
│   wf_following_left_wall = !wf_following_left_wall  ← Switch    │
└────────┬─────────────────────────────────────────────────────────┘
         │
         │ Turn duration elapsed (1500ms)
         │ wall switched
         │ steering reset
         │
         ▼
         (Back to FOLLOWING with new wall)
         
         After 3 complete rounds (wf_completed_rounds >= 3):
         │
         ▼
┌─────────────────────────────────────────────────────────────────────┐
│ STOPPED State                                                       │
│ - Robot halted                                                      │
│ - Motors disabled                                                   │
│ - Servo centered                                                    │
│ - Mission complete message sent                                    │
│ - Reports total turns and rounds                                   │
│                                                                     │
│ Output:                                                             │
│   set_speed(0)      ← Stop                                        │
│   set_steering(0)   ← Center servo                                │
│   stop()            ← Disable motors                              │
└────────┬────────────────────────────────────────────────────────────┘
         │
         │ Serial Command: 'z' (or 'w' to restart)
         │ navigation_disable()
         │
         ▼
         (Back to IDLE)
```

## PD Controller Operation

```
Target Distance = 300mm (0.3m)
Current Distance = Left sensor reading

Error = 300 - Current
Steering = 0.5 × Error + 0.1 × (Current_Error - Last_Error)

Example 1: Too far (Error = +50mm)
  Steering = 0.5 × 50 = 25° (turn toward wall)
  Result: Robot turns slightly left to close gap

Example 2: Too close (Error = -50mm)
  Steering = 0.5 × (-50) = -25° (turn away from wall)
  Result: Robot turns slightly right to increase distance

Example 3: Oscillating (Error changing rapidly)
  dError/dt = large
  Damping = 0.1 × dError (slows changes)
  Result: Smooth motion, no hunting
```

## Turn Detection Logic

```
Current State: FOLLOWING

Check Loop:
└─→ Is opposite_wall_distance > 1.0m?
    │
    ├─ YES → Gap detected!
    │   ├─ Increment turn_count
    │   ├─ Check if (turn_count % 4 == 0)
    │   │   ├─ YES → Round complete! (wf_completed_rounds++)
    │   │   │        Check if (wf_completed_rounds >= 3)
    │   │   │        ├─ YES → Set state = STOPPED
    │   │   │        └─ NO → Continue to TURNING
    │   │   └─ NO → Continue to TURNING
    │   │
    │   ├─ Set wf_turn_angle (±90° based on wall side)
    │   ├─ Switch wall: wf_following_left_wall = !wf_following_left_wall
    │   ├─ Set state = TURNING
    │   └─ Start timer
    │
    └─ NO → Continue FOLLOWING
        (maintain distance, no turn)
```

## Round Counting

```
Start: wf_turn_count = 0, wf_completed_rounds = 0

Turn 1 executed: wf_turn_count = 1 (1 % 4 = 1) → Round still 0
Turn 2 executed: wf_turn_count = 2 (2 % 4 = 2) → Round still 0
Turn 3 executed: wf_turn_count = 3 (3 % 4 = 3) → Round still 0
Turn 4 executed: wf_turn_count = 4 (4 % 4 = 0) → Round becomes 1! ✓

Turn 5 executed: wf_turn_count = 5 (5 % 4 = 1) → Round still 1
Turn 6 executed: wf_turn_count = 6 (6 % 4 = 2) → Round still 1
Turn 7 executed: wf_turn_count = 7 (7 % 4 = 3) → Round still 1
Turn 8 executed: wf_turn_count = 8 (8 % 4 = 0) → Round becomes 2! ✓

Turn 9 executed:  wf_turn_count = 9  (9 % 4 = 1) → Round still 2
Turn 10 executed: wf_turn_count = 10 (10 % 4 = 2) → Round still 2
Turn 11 executed: wf_turn_count = 11 (11 % 4 = 3) → Round still 2
Turn 12 executed: wf_turn_count = 12 (12 % 4 = 0) → Round becomes 3! ✓

Next gap detected at Turn 13:
  └─ wf_completed_rounds == 3 → STOP! ✓
```

## Heading vs. Turn Counting

```
Robot starts facing North (heading = 0°)
Target: Follow perimeter of rectangular room

After Turn 1 (90° right):  Heading ≈ 90°, wf_turn_count = 1
After Turn 2 (90° right):  Heading ≈ 180°, wf_turn_count = 2
After Turn 3 (90° right):  Heading ≈ 270°, wf_turn_count = 3
After Turn 4 (90° right):  Heading ≈ 360° (= 0°), wf_turn_count = 4
                          └─ 1 complete round! wf_completed_rounds = 1

After Turn 5 (90° right):  Heading ≈ 90°, wf_turn_count = 5
... (repeats 3 times total)
After Turn 12 (90° right): Heading ≈ 360°, wf_turn_count = 12
                          └─ 3 complete rounds! wf_completed_rounds = 3
                          └─ STOP!
```

## Main Loop Execution Order

```
┌──────────────────────────────────────────────────────────┐
│ void loop()                                              │
└──────────────────────────────────────────────────────────┘
         │
         ├─ loop_updater()
         │  └─ Update timing, calculate encoder distance
         │
         ├─ check_serial_available()
         │  └─ Read serial commands, call navigation_controller functions
         │
         ├─ check_stalling()
         │  └─ Safety check, stop if stalled
         │
         ├─ update_lasers() ◄─── Sensors updated BEFORE navigation_controller
         │  └─ Read ToF distances (needed by navigation_controller)
         │
         ├─ update_gyro() ◄───── Sensors updated BEFORE navigation_controller
         │  └─ Read gyro heading (needed for round counting)
         │
         ├─ navigation_update() ◄─── Main autonomous logic
         │  ├─ Read sensor globals (current_distance_left_m, etc)
         │  ├─ Execute state machine logic
         │  └─ Call motor control functions
         │
         └─ drive_loop()
            └─ Motor PID (may be overridden by navigation_controller)
```

## Memory Map - Global Variables

```
motor_control module:
├─ encoder_pos (long)
├─ current_speed (float)
├─ target_speed (int)
├─ current_dc (float)
├─ Kp, Ki, Kd (float)
├─ current_degree (from sensors module!)
└─ ... and 30+ more state variables

sensors module:
├─ current_degree (float) ◄── Used by navigation_controller
├─ current_distance_left_m (float) ◄── Used by navigation_controller
├─ current_distance_right_m (float) ◄── Used by navigation_controller
└─ ... and 5+ more sensor variables

navigation_controller module:
├─ wf_state (WallFollowerState)
├─ wf_turn_count (int)
├─ wf_completed_rounds (int)
├─ wf_target_distance (float)
├─ wf_pd_kp, wf_pd_kd (float)
└─ ... and 10+ more control variables

serial_handler module:
├─ ringBuffer[] (char)
├─ head, tail (int)
└─ ... and 2+ buffer management variables
```

---

**For implementation details:** See [IMPLEMENTATION_SUMMARY.md](IMPLEMENTATION_SUMMARY.md)
**For user guide:** See [WALL_FOLLOWER_GUIDE.md](WALL_FOLLOWER_GUIDE.md)  
**For testing:** See [TESTING_GUIDE.md](TESTING_GUIDE.md)
