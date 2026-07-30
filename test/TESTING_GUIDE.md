# Wall Follower Testing Guide

## Pre-Flight Checklist

Before testing wall-following behavior, ensure:

- [ ] Both ToF sensors are working (`v` shows valid distances, not -1.0m)
- [ ] Gyro is initialized (message shows "BNO085 Streaming reports enabled")
- [ ] Motor responds to manual commands (`d100` makes robot move)
- [ ] Servo responds to manual commands (`s30` steers robot)
- [ ] Serial connection at 115200 baud
- [ ] Robot placed in test environment (ideally a rectangular room or hallway)

## Test Phase 1: Manual Verification

### Step 1: Check Sensor Calibration

```
Send: v
Expected Output: LEFT_m: 0.XX RIGHT_m: 0.YY
```

**Acceptable ranges:**
- Left sensor: 0.2m - 2.0m ✅
- Right sensor: 0.2m - 2.0m ✅
- Either showing -1.0m ❌ (sensor malfunction)

### Step 2: Check Motor Response

```
Send: d200
Expected: Robot moves forward at 200 mm/s
         Stops after a few seconds (no speed maintained in manual mode)
```

### Step 3: Check Servo Response

```
Send: s30
Expected: Front of robot turns right slightly
Send: s-30
Expected: Front of robot turns left slightly
Send: s0
Expected: Front of robot centers
```

## Test Phase 2: Wall Follower Enable

### Step 1: Position Robot

Place robot:
- In an empty space with a wall nearby (300mm away if possible)
- Or in a hallway (narrow space)
- Or in a rectangular room

### Step 2: Set Configuration

```
Send: u300
Expected Output: Wall target distance set to: 300 mm

Send: i
Expected Output: Wall follower debug ON
```

### Step 3: Enable Wall Following

```
Send: w
Expected Output:
  ===== WALL FOLLOWING STARTED =====
  Start heading: 45.2°
  Following RIGHT wall initially
```

Robot should:
- Start moving forward
- Begin steering to maintain ~300mm distance from nearest wall
- Show debug output every 500ms

## Test Phase 3: Debug Monitoring

After enabling (`w`), you should see continuous output:

```
[WF] State: FOLLOWING | Wall: RIGHT | Dist: 0.30m | Opp: 2.50m | 
Turns: 0 | Rounds: 0 | Heading: 45.2° | Speed: 200 mm/s
```

**Check these values:**

| Value | Expected | Issue |
|-------|----------|-------|
| State | FOLLOWING | If "IDLE", enable didn't work |
| Wall | RIGHT or LEFT | Should alternate after turns |
| Dist | ~0.30m | Distance from followed wall |
| Opp | > 1.0m (normal) | If < 0.5m, robot too close to opposite wall |
| Turns | Increments | Should increase as robot turns |
| Rounds | 0→1→2→3 | Should increment every 4 turns |
| Heading | Varies smoothly | Should change by ~90° with each turn |
| Speed | 200 (following) or 150 (turning) | Speed during different states |

## Test Phase 4: Corner Navigation

### Expected Behavior

When robot reaches corner:

```
Before corner:
Dist: 0.32m, Opp: 2.51m → FOLLOWING right wall

Approaching corner:
Dist: 0.32m, Opp: 1.05m → Still FOLLOWING

At corner:
Dist: 0.32m, Opp: 1.52m → Gap detected!
State: TURNING | Turn angle: 90° → TURNING left
Wall switches: RIGHT → LEFT

After turn:
Dist: 0.28m, Opp: 2.45m → FOLLOWING left wall
Turns: 1 | Rounds: 0
```

### What to Look For

✅ **Good Turn:**
- Clean 90° rotation
- Robot aligns with new wall
- Turns counting increments
- No oscillations before/after

❌ **Bad Turn:**
- Robot overshoots or undershoots turn
- Crashes into wall
- Multiple rapid turns in sequence
- Turns counter stalls

## Test Phase 5: Round Completion

### Track Progress

```
After Turn 1: Rounds: 0 (1/4 complete)
After Turn 2: Rounds: 0 (2/4 complete)
After Turn 3: Rounds: 0 (3/4 complete)
After Turn 4: Rounds: 1 ← First round complete!

After Turn 8:  Rounds: 2 ← Second round complete
After Turn 12: Rounds: 3 ← Third round complete

Turn 13: State: STOPPED ← Mission complete
         Robot stops, no more movement
```

### Success Criteria

✅ Robot completes 3 full rounds
✅ Automatic stop after 12 turns
✅ Heading approximately same as start (±30°)
✅ No crashes or stalls

## Test Phase 6: Error Conditions

### Test: Obstacle in Path

```
Setup: Place obstacle in path while robot is moving
Expected: Robot will stall detect and stop
Output: "Stall detected, stopping robot"
```

### Test: Sensor Failure

```
Setup: Cover one ToF sensor while running
Expected: Robot may behave erratically or stop
Output: Debug shows distance -1.0m for blocked sensor
```

### Test: Manual Stop

```
Setup: Wall follower running
Send: z
Expected: Robot stops immediately
Output: Wall follower disabled
        Completed: X turns, Y rounds
```

## Test Phase 7: Configuration Tuning

### Test 1: Change Target Distance

```
Current: Following at 300mm
Send: u250
Expected: Robot moves closer to wall

Send: u400
Expected: Robot moves farther from wall
```

### Test 2: Enable/Disable Debug

```
Verbose Output ON:
Send: i
Result: See [WF] telemetry every 500ms

Verbose Output OFF:
Send: o
Result: Only state changes print
```

## Troubleshooting

### Problem: Robot doesn't start moving after `w`

**Possible Causes:**
1. Sensors not initialized
2. Motor disabled
3. Servo disabled

**Solution:**
```
v              → Check sensor readings
m              → Master enable motors/servo
w              → Try enabling again
```

### Problem: Robot moves but doesn't follow wall

**Possible Causes:**
1. Target distance incorrect
2. PD gains too low
3. Distance sensor not working

**Solution:**
```
v              → Verify distances are changing
u400           → Try very far target (400mm)
i              → Enable debug to see distance error
```

### Problem: Robot oscillates left-right wildly

**Possible Causes:**
1. PD Kp gain too high
2. Distance sensor noisy
3. Servo has dead band

**Solution:**
Edit `navigation_controller.cpp`:
```cpp
float wf_pd_kp = 0.3;  // Reduce from 0.5
float wf_pd_kd = 0.2;  // Increase from 0.1
```

### Problem: Robot doesn't turn at corners

**Possible Causes:**
1. Wall margin threshold too high (1.0m)
2. Turn duration too short
3. Robot moving too fast to detect gap

**Solution:**
Edit `navigation_controller.cpp`:
```cpp
float wf_wall_margin = 0.8;        // Reduce threshold
unsigned long wf_turn_duration_ms = 2000;  // Increase turn time
```

### Problem: Robot completes only 1-2 rounds instead of 3

**Possible Causes:**
1. Robot stuck or crashed
2. Sensor reading becomes invalid
3. Round counting logic issue

**Solution:**
```
z              → Stop robot
i              → Enable debug
w              → Try again, observe debug output
              → Look for error messages or invalid distances
```

## Performance Validation

### Measure Distance Tracking Accuracy

While robot is running:
```
Observe: [WF] ... Dist: X.XXm ...

Record 10 consecutive readings:
1. 0.32m
2. 0.31m
3. 0.33m
4. 0.30m
5. 0.32m
...

Acceptable: ±50mm variation (0.30m - 0.35m when targeting 0.30m)
```

### Measure Turn Accuracy

During corner:
```
Before turn: Heading: 0.0°
After turn:  Heading: ≈90.0° (or -90.0°)

Acceptable: Within 10° of perfect 90° (80° to 100°)
```

### Measure Round Completion Time

```
Time from 'w' to 'Rounds: 1' = Time for first round
Expected: 60-180 seconds depending on perimeter size
```

## Success Indicators

✅ Wall follower starts with `w` command
✅ Debug telemetry shows [WF] state correctly
✅ Robot maintains ~300mm distance from wall
✅ Corners are detected and turns executed
✅ Rounds increment every 4 turns
✅ Robot stops automatically after 3 rounds
✅ Manual stop with `z` works
✅ No crashes or sensor errors

## Passing Criteria

All of the following must be true:

1. ✅ Robot completes at least 1 full perimeter lap (4 turns)
2. ✅ Distance tracking within ±50mm of target
3. ✅ Turn accuracy within ±10° of 90°
4. ✅ Stops automatically at "Rounds: 3"
5. ✅ Debug telemetry is stable (no NaN, Inf, or -1.0m values)
6. ✅ No software crashes or hang-ups
7. ✅ Serial commands respond correctly
8. ✅ Manual override works (can stop with `z`)

---

**If tests pass:** Wall follower is ready for deployment! 🎉
**If tests fail:** See troubleshooting section and review debug output carefully.

For questions about behavior, check [WALL_FOLLOWER_GUIDE.md](WALL_FOLLOWER_GUIDE.md)
