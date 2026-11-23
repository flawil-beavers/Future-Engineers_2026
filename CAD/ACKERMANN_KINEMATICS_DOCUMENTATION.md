# Ackermann steering calibration

We used a Fusion 360 model to find how the front wheel angles change as the
steering servo turns. We then compared the predicted turning radii with
full-circle measurements from the robot. The firmware uses the CAD-derived
wheel angles to estimate curvature and check how much space the wheels need
during a manoeuvre.

## Steering linkage

The central servo drives the front steering linkage through a pin-slot joint.
The slot lets the pin slide as the steering moves. The steering knuckles
rotate about their kingpins, and the inner wheel turns more sharply than the
outer wheel to reduce tyre scrub. In ideal Ackermann geometry, the difference
between the cotangents of the outer and inner steering angles equals the
kingpin track width divided by the wheelbase. Our linkage approximates that
relationship. [Chassis and steering](../README.md#chassis-and-steering)

The measured axle spacing is **98 mm**. The firmware uses that measurement for
the parking footprint and **127 mm** as the effective wheelbase in its
steering-radius model. [Measured parking geometry](../simulation/reports/RETURNED_651_656_PARKING_20261105.md#measured-geometry-and-implementation) · [Firmware model](../include/ackermann_kinematics.h)

## From Fusion 360 to the wheel-angle table

1. Run the exporter in Fusion 360. It moves the servo joint in **1° steps** and
   records the servo position, pin-slot travel and both wheel angles. The
   checked-in sweep spans **−50° to +50°**.
2. Run the analysis script. It wraps the wheel angles into the **[−180°, 180°)**
   range so a value such as 322.5° is treated as −37.5°. It plots the data,
   fits a cubic to each wheel's angles and exports the coefficients and
   lookup tables.
3. Inspect the wheel-angle curves and pin travel. The firmware interpolates
   between adjacent table samples when it needs a wheel angle.

[CAD workflow](README.md#workflow) · [Exported sweep](ackermann_data.csv) · [Analysis plot](ackermann_analysis.png) · [Generated header](ackermann_lut.h)

## Turning Radius Comparison

We used encoder travel and the change in BNO085 heading to estimate the
turning radius during a full circle. The radius is the distance travelled
divided by the turn angle in radians. Servo commands are offsets from neutral.
Negative commands turn left and positive commands turn right.

The recorded radii below come from the original test log. The last column
recalculates the current firmware model from the saved CAD wheel angles.
That model divides 127 mm by the tangent of the mean absolute wheel angle and
multiplies by the configured **1.02** tyre-slip factor.
[Original test log](log_47.txt) · [Radius calculation](../include/ackermann_kinematics.h)

| Servo command | Measured radius (mm) | CAD radius reported in August (mm) | Current model radius (mm) |
| ---: | ---: | ---: | ---: |
| −40° | 154.6 | 155.9 | 154.4 |
| −35° | 186.7 | — | 185.0 |
| −30° | 226.4 | 222.5 | 224.4 |
| −25° | 277.4 | — | 277.8 |
| −20° | 356.8 | 350.3 | 355.9 |
| −15° | 506.5 | 475.0 | 483.4 |
| +10° | 681.2 | 720.9 | 734.7 |
| +15° | 463.8 | 475.0 | 483.4 |
| +20° | 337.2 | 350.3 | 355.9 |
| +25° | 271.2 | — | 277.8 |
| +30° | 225.8 | 222.5 | 224.4 |
| +35° | 187.0 | — | 185.0 |
| +40° | 158.8 | 155.9 | 154.4 |

The historical CAD column preserves the values recorded in the August 2026
report. Its eight entries between 15° and 40° in either direction give an
average absolute difference of **2.56%** relative to the listed CAD radii.
The −10° test timed out before a complete circle, so the log has no radius
at that command. The results describe this calibration run.

The August report also records a **+0.6°** servo-neutral offset and a trial
change of the centre command from **81° to 82°**. The linked log contains the
initial +10° measurement. The report attributes a reduction from **5.50% to
0.06%** at +10° to the adjustment. The current firmware sets the servo centre
to **80°**. [Current servo configuration](../include/config.h#L163)

## How the firmware uses the model

The test log compares two fits of the measured radii. The cubic fit gave
root mean square errors of **2.97 mm** on the left and **4.88 mm** on the right.
The fit based on servo command and a single scale factor gave **7.49 mm**
and **16.15 mm**, respectively.

The current firmware uses the CAD wheel-angle tables in its radius model.
It can convert a requested radius into a servo command, and the parking
planner uses the individual wheel angles to check the swept footprint.
The physical radius can vary with grip and steering load, so parking also
checks how far the robot actually turns. [Radius-to-servo conversion](../src/motor_control.cpp) · [Parking controller](../src/final_parking.cpp)
