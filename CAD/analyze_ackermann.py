import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os

# 1. Flexible CSV Path Check
primary_path = "C:/Users/Public/ackermann_data.csv"
local_path = "ackermann_data.csv"

if os.path.exists(primary_path):
    csv_path = primary_path
    print(f"Found CSV at primary location: {csv_path}")
elif os.path.exists(local_path):
    csv_path = local_path
    print(f"Found CSV in current directory: {csv_path}")
else:
    raise FileNotFoundError(
        f"Could not find 'ackermann_data.csv' in '{primary_path}' or in the current directory ('{os.getcwd()}')."
    )

# Load CSV Data
df = pd.read_csv(csv_path)

servo_deg = df['Servo_Angle_Deg'].values
slide_mm  = df['Servo_Slide_mm'].values

# FIX: Normalize angles from [0, 360] into standard [-180, +180] relative range
left_deg  = (df['Left_Wheel_Deg'].values + 180) % 360 - 180
right_deg = (df['Right_Wheel_Deg'].values + 180) % 360 - 180

# 2. Fit 3rd-Order Polynomials: Angle_Wheel = a*x^3 + b*x^2 + c*x + d
p_left  = np.polyfit(servo_deg, left_deg, 3)
p_right = np.polyfit(servo_deg, right_deg, 3)

print("\n--- Corrected Polynomial Coefficients ---")
print(f"Left Wheel:  {p_left}")
print(f"Right Wheel: {p_right}")

# 3. Plot Results for Verification
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

# Plot Steering Angles
ax1.plot(servo_deg, left_deg, 'b.', label='Data Left')
ax1.plot(servo_deg, np.polyval(p_left, servo_deg), 'b--', label='Fit Left')
ax1.plot(servo_deg, right_deg, 'r.', label='Data Right')
ax1.plot(servo_deg, np.polyval(p_right, servo_deg), 'r--', label='Fit Right')
ax1.set_title("Servo Angle vs. Wheel Angles (Normalized)")
ax1.set_xlabel("Servo Angle (°)")
ax1.set_ylabel("Wheel Angle (°)")
ax1.grid(True)
ax1.legend()

# Plot Pin-Slot Slide Distance
ax2.plot(servo_deg, slide_mm, 'g-o', label='Pin-Slot Slide')
ax2.set_title("Servo Angle vs. Pin-Slot Slide")
ax2.set_xlabel("Servo Angle (°)")
ax2.set_ylabel("Slide Distance (mm)")
ax2.grid(True)
ax2.legend()

plt.tight_layout()

# Save the figure in the same folder as this script
script_dir = os.path.dirname(os.path.abspath(__file__))
plt.savefig(os.path.join(script_dir, "ackermann_analysis.png"))
plt.show()

# 4. Generate C++ Header File for PlatformIO
header_content = f"""#ifndef ACKERMANN_LUT_H
#define ACKERMANN_LUT_H

#include <Arduino.h>

namespace Ackermann {{

    // Range metadata from Fusion 360 sweep
    constexpr float MIN_SERVO_DEG = {float(min(servo_deg)):.1f}f;
    constexpr float MAX_SERVO_DEG = {float(max(servo_deg)):.1f}f;
    constexpr float SERVO_STEP_DEG = {float(abs(servo_deg[1] - servo_deg[0])):.1f}f;
    constexpr size_t TABLE_SIZE = {len(servo_deg)};

    // Fitted Polynomial Coefficients: f(x) = A*x^3 + B*x^2 + C*x + D
    constexpr float LEFT_A  = {p_left[0]:.8f}f;
    constexpr float LEFT_B  = {p_left[1]:.8f}f;
    constexpr float LEFT_C  = {p_left[2]:.8f}f;
    constexpr float LEFT_D  = {p_left[3]:.8f}f;

    constexpr float RIGHT_A = {p_right[0]:.8f}f;
    constexpr float RIGHT_B = {p_right[1]:.8f}f;
    constexpr float RIGHT_C = {p_right[2]:.8f}f;
    constexpr float RIGHT_D = {p_right[3]:.8f}f;

    // Fast polynomial evaluation function for C++
    inline float getLeftWheelAngle(float servoAngle) {{
        float x = constrain(servoAngle, MIN_SERVO_DEG, MAX_SERVO_DEG);
        return ((LEFT_A * x + LEFT_B) * x + LEFT_C) * x + LEFT_D; // Horner's method
    }}

    inline float getRightWheelAngle(float servoAngle) {{
        float x = constrain(servoAngle, MIN_SERVO_DEG, MAX_SERVO_DEG);
        return ((RIGHT_A * x + RIGHT_B) * x + RIGHT_C) * x + RIGHT_D;
    }}

    // High-performance Lookup Table (LUT) with Linear Interpolation
    const float LUT_LEFT[{len(servo_deg)}] = {{
        {", ".join([f"{v:.3f}f" for v in left_deg])}
    }};

    const float LUT_RIGHT[{len(servo_deg)}] = {{
        {", ".join([f"{v:.3f}f" for v in right_deg])}
    }};

    inline float getLeftWheelAngleLUT(float servoAngle) {{
        float clamped = constrain(servoAngle, MIN_SERVO_DEG, MAX_SERVO_DEG);
        float index_f = (clamped - MIN_SERVO_DEG) / SERVO_STEP_DEG;
        int idx = (int)index_f;
        if (idx >= (int)TABLE_SIZE - 1) return LUT_LEFT[TABLE_SIZE - 1];
        
        float fraction = index_f - idx;
        return LUT_LEFT[idx] + fraction * (LUT_LEFT[idx + 1] - LUT_LEFT[idx]);
    }}

}} // namespace Ackermann

#endif // ACKERMANN_LUT_H
"""

output_header_path = "ackermann_lut.h"
with open(output_header_path, "w") as f:
    f.write(header_content)

print(f"\nHeader file successfully generated: {output_header_path}")