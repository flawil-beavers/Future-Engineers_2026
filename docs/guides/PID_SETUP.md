# PID and drive motor tuning

We tune the drive motor from the serial monitor. Use `pid help` to list the commands, `pid show` to see the active values, and `pid export` to print settings for [`include/config.h`](../../include/config.h).

## How the controller works

We use proportional–integral (PI) feedback to control speed. Each of the three speed bands has its own gains. The controller uses the LOW gains up to the LOW boundary, interpolates from LOW to MID and MID to HIGH between the boundaries, and uses the HIGH gains above the HIGH boundary.

| Setting | What it controls |
| --- | --- |
| Cruise Kp | Responds immediately to the speed error. A low value gives a weak response. A high value can cause rapid oscillation. |
| Cruise Ki | Corrects persistent speed error under load. A high value can cause slow surging and overshoot. |
| Static feed-forward | Adds a base PWM command to overcome friction. |
| Kv | Adds PWM in proportion to the target speed in mm/s. |
| Ka | Adds PWM as the controller requests acceleration. |
| Acceleration Kp and Ki | Correct the difference between requested and measured acceleration. The same gains apply across all speed bands. |

[Drive and low-speed control](../../README.md#drive-selection-and-low-speed-control)

## Test and adjust the gains

1. Clear a straight stretch and start with the drive-enable switch (SW1) LOW.
2. Send `pid test 150` to select a **150 mm/s** test. Use SW1 to enable the drive when ready. Send uppercase `P` during the test to inspect target speed, measured speed, PWM and active gains.
3. Send `z` or set SW1 LOW to stop. Change a setting, then repeat the test.

For example, these commands change selected gains, a speed boundary and the static feed-forward setting.

```text
pid set low kp 0.035
pid set low ki 0.020
pid set mid kp 0.080
pid set high ki 0.040
pid set bound low 120
pid set ff static 86
pid show
```

Keep the speed boundaries in ascending order, `LOW < MID < HIGH`. Changes take effect immediately in RAM and reset at restart. When the car drives as intended, send `pid export`, copy the printed values into `include/config.h`, and rebuild and upload M7 to make them permanent. [PID commands](../../src/serial_handler.cpp#L97-L113)

## Use the relay autotuner

```text
pid tune <speed_mm_s> <baseline_pwm> <relay_step>
pid tune 150 103 15
```

- **Target speed** selects the LOW, MID or HIGH band to tune.
- **Baseline PWM** should drive the motor at approximately that speed. Use a manual test to choose a starting value.
- **Relay step** sets how far the PWM moves above and below the baseline.

The tuner measures several oscillation cycles, discards asymmetric or inconsistent samples, and calculates Kp and Ki. It applies a valid result to the selected band in RAM. Tune each band separately and run `pid test` after each result. Adjust feed-forward or acceleration gains when the same issue appears across the bands. Use `pid export` to save the final values.

The drive-enable switch and `z` stop a test. The autotuner also stops when it reaches its time or travel limit.
