# Wall Follower Control System

## Overview

The robot can now autonomously follow walls using ToF distance sensors and a PD controller. The system:

1. **Detects enable signal** - Start via serial command
2. **Follows wall at target distance** - Uses PD controller to maintain distance from detected wall
3. **Detects wall gaps** - When distance > 1.0m, executes 90° turn toward the gap
4. **Counts perimeter rounds** - Each 4 turns (360°) = 1 complete round
5. **Stops after 3 rounds** - Stops in middle of a straight section (13th turn/stop)

## How It Works

### States

```
IDLE (Waiting)
    ↓
    'w' command received
    ↓
FOLLOWING (Moving straight, maintaining distance)
    ↓
    Distance > 1.0m detected on opposite side
    ↓
TURNING (90° pivot turn)
    ↓
    Turn complete / Switch wall
    ↓
FOLLOWING (continue with new wall)
    ↓
    ... after 3 complete rounds (12 turns + stop) ...
    ↓
STOPPED (mission complete)
```

### Distance Control

The robot uses a **PD controller** to maintain distance from the wall:

```
Error = (Target Distance) - (Current Distance)
Steering = Kp × Error + Kd × (Change in Error)
```

- **Positive error** = too far from wall → steer toward wall
- **Negative error** = too close to wall → steer away
- **Kp** = Proportional gain (responsive)
- **Kd** = Derivative gain (smooth/damping)

### Turn Detection

When the **opposite wall distance > 1.0m**:
- Robot is too close to current wall, or parallel to wall
- Take 90° turn toward the gap
- Switch to following the opposite wall

### Round Counting

- **Turn 1-4** = Round 1 (perimeter cycle 1)
- **Turn 5-8** = Round 2 (perimeter cycle 2)  
- **Turn 9-12** = Round 3 (perimeter cycle 3)
- **Turn 13** = Stop (mid-straight)

## Serial Commands

### Basic Control

```
w          Start wall following
z          Stop wall following
```

### Configuration

```
u<distance>   Set target wall distance (mm)
              Example: u300 = 300mm target distance

i          Enable debug output (verbose telemetry)
o          Disable debug output
```

### Example Session

```
Serial Input:
u300          → Set target distance to 300mm
w             → Start wall following
              → Robot begins following right wall at 300mm distance
              
(Observe output)
i             → Enable detailed debug output
              → See real-time: state, distances, turns, heading

(After 3 rounds)
z             → Stop (can stop anytime)
o             → Disable debug output
```

## Debug Output Format

When enabled with `i` command:

```
[WF] State: FOLLOWING | Wall: RIGHT | Dist: 0.34m | Opp: 1.52m | 
Turns: 3 | Rounds: 0 | Heading: 45.2° | Speed: 200 mm/s
```

Explanation:
- `State: FOLLOWING` - Currently following wall
- `Wall: RIGHT` - Following right wall (left is opposite)
- `Dist: 0.34m` - Current wall distance
- `Opp: 1.52m` - Opposite wall distance (> 1.0m triggers turn)
- `Turns: 3` - Total turns executed
- `Rounds: 0` - Complete rounds (0-3)
- `Heading: 45.2°` - Current gyro heading
- `Speed: 200 mm/s` - Movement speed

## Configuration Constants

Edit these in `src/wall_follower.cpp` or use runtime commands:

```cpp
float wf_target_distance = 300.0;     // Target distance from wall (mm)
float wf_wall_margin = 1.0;           // Gap detection threshold (m)
float wf_pd_kp = 0.5;                 // Proportional gain
float wf_pd_kd = 0.1;                 // Derivative gain
unsigned long wf_turn_duration_ms = 1500;  // 90° turn time (ms)
```

## Tuning Guide

### If robot oscillates side-to-side:
**Problem:** Controller is too responsive
**Solution:** Decrease `Kp` (reduce gain)
```cpp
// In wall_follower.cpp:
float wf_pd_kp = 0.3;  // was 0.5, reduce responsiveness
```

### If robot doesn't turn quickly enough:
**Problem:** Turn duration is too short
**Solution:** Increase `wf_turn_duration_ms`
```cpp
unsigned long wf_turn_duration_ms = 2000;  // was 1500, more time for turn
```

### If robot misses wall gap and crashes:
**Problem:** Wall margin threshold too high or detection too slow
**Solution:** Decrease wall margin or increase update rate
```cpp
float wf_wall_margin = 0.8;  // was 1.0, detect gaps earlier
```

### If robot is too close/far from wall:
**Problem:** Target distance needs adjustment
**Solution:** Use serial command:
```
u250    // 250mm (closer)
u350    // 350mm (farther)
```

## Behavior Examples

### Scenario 1: Following a rectangular room

```
Start position: Corner of room, facing right wall
Robot behavior:

1. Enable with 'w' → Start following right wall (300mm distance)
2. Move forward → Maintain distance using PD controller
3. Reach next corner → Right wall opens up (distance > 1.0m)
4. Turn 1 (RIGHT 90°) → Switch to top wall, follow left
5. Move forward → Maintain distance
6. Reach next corner → Left wall opens up
7. Turn 2 (LEFT 90°) → Switch to back wall, follow right
... (repeat for complete perimeter)
8. After turn 12 → 3 complete rounds
9. Turn 13 → Stop in middle of straight section
```

### Scenario 2: Following an L-shaped area

```
Same logic applies:
- Detects wall gap (distance > 1.0m)
- Takes appropriate 90° turn
- Continues following adjacent wall
```

## Motor Speed During Walls Following

- **Normal following:** 200 mm/s
- **During turn:** 150 mm/s (slower for accuracy)
- **After stop:** 0 mm/s (stationary)

These are set in `wall_follower.cpp` and can be customized.

## Safety Features

✅ **Stall detection** - Stops if motor stalls while moving
✅ **Distance validation** - Ignores invalid sensor readings (< 0)
✅ **Heading tracking** - Uses gyro to count complete rotations
✅ **State safety** - Can stop anytime with 'z' command
✅ **Speed limits** - Motor respects max duty cycle

## Integration with Manual Control

While wall following is active (`w` command), the system is autonomous. You can:

- **Monitor with `i`** - See what robot is doing
- **Stop anytime with `z`** - Return to manual control
- **Use `m` to re-enable** - Resume manual motor/servo control after stop

After stopping wall follower, you can use manual commands:
```
d100   → Set speed manually
s45    → Steer manually
```

## Troubleshooting

| Issue | Check |
|-------|-------|
| Robot doesn't start | Verify sensors work: `v` shows valid distances |
| Robot drifts left/right | PD gains need tuning (adjust Kp/Kd) |
| Robot spins in circles | Target distance wrong, or sensors misaligned |
| Robot doesn't detect walls | Verify ToF sensors calibrated and detecting |
| Turning is erratic | Increase `wf_turn_duration_ms` |
| Stops before 3 rounds | Check round counting logic or sensor reliability |

## Future Enhancements

Possible improvements:
- Adaptive speed based on wall distance variation
- Machine learning to optimize PD gains
- Multi-wall tracking (follow complex maze)
- Path logging and replay
- Obstacle avoidance (use heading changes)
- Perimeter measurement

---

**For implementation details:** See [wall_follower.cpp](../src/wall_follower.cpp)
**For architecture:** See [ARCHITECTURE.md](ARCHITECTURE.md)
