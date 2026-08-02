#ifndef ACKERMANN_LUT_H
#define ACKERMANN_LUT_H

#include <Arduino.h>

namespace Ackermann {

    // Range metadata from Fusion 360 sweep
    constexpr float MIN_SERVO_DEG = -50f;
    constexpr float MAX_SERVO_DEG = 50f;
    constexpr float SERVO_STEP_DEG = 1f;
    constexpr size_t TABLE_SIZE = 101;

    // Fitted Polynomial Coefficients: f(x) = A*x^3 + B*x^2 + C*x + D
    constexpr float LEFT_A  = 0.00000000f;
    constexpr float LEFT_B  = 0.00452465f;
    constexpr float LEFT_C  = 1.00000000f;
    constexpr float LEFT_D  = -0.48296623f;

    constexpr float RIGHT_A = -0.00000000f;
    constexpr float RIGHT_B = -0.00452465f;
    constexpr float RIGHT_C = 1.00000000f;
    constexpr float RIGHT_D = 0.48296623f;

    // Fast polynomial evaluation function for C++
    inline float getLeftWheelAngle(float servoAngle) {
        float x = constrain(servoAngle, MIN_SERVO_DEG, MAX_SERVO_DEG);
        return ((LEFT_A * x + LEFT_B) * x + LEFT_C) * x + LEFT_D; // Horner's method
    }

    inline float getRightWheelAngle(float servoAngle) {
        float x = constrain(servoAngle, MIN_SERVO_DEG, MAX_SERVO_DEG);
        return ((RIGHT_A * x + RIGHT_B) * x + RIGHT_C) * x + RIGHT_D;
    }

    // High-performance Lookup Table (LUT) with Linear Interpolation
    const float LUT_LEFT[101] = {
        -37.545f, -37.397f, -37.180f, -36.903f, -36.572f, -36.193f, -35.771f, -35.310f, -34.813f, -34.285f, -33.727f, -33.143f, -32.533f, -31.901f, -31.247f, -30.573f, -29.882f, -29.172f, -28.447f, -27.706f, -26.951f, -26.183f, -25.402f, -24.609f, -23.804f, -22.988f, -22.162f, -21.326f, -20.481f, -19.626f, -18.763f, -17.891f, -17.011f, -16.123f, -15.227f, -14.324f, -13.414f, -12.497f, -11.573f, -10.643f, -9.706f, -8.762f, -7.813f, -6.857f, -5.895f, -4.927f, -3.954f, -2.974f, -1.988f, -0.997f, 0.000f, 1.003f, 2.012f, 3.026f, 4.046f, 5.073f, 6.105f, 7.143f, 8.187f, 9.238f, 10.294f, 11.357f, 12.427f, 13.503f, 14.586f, 15.676f, 16.773f, 17.877f, 18.989f, 20.109f, 21.238f, 22.374f, 23.519f, 24.674f, 25.838f, 27.012f, 28.196f, 29.391f, 30.598f, 31.817f, 33.049f, 34.294f, 35.553f, 36.828f, 38.118f, 39.427f, 40.753f, 42.099f, 43.467f, 44.857f, 46.273f, 47.715f, 49.187f, 50.690f, 52.230f, 53.807f, 55.428f, 57.097f, 58.820f, 60.603f, 62.455f
    };

    const float LUT_RIGHT[101] = {
        -62.455f, -60.603f, -58.820f, -57.097f, -55.428f, -53.807f, -52.229f, -50.690f, -49.187f, -47.715f, -46.273f, -44.857f, -43.467f, -42.099f, -40.753f, -39.427f, -38.118f, -36.828f, -35.553f, -34.294f, -33.049f, -31.817f, -30.598f, -29.391f, -28.196f, -27.012f, -25.838f, -24.674f, -23.519f, -22.374f, -21.237f, -20.109f, -18.989f, -17.877f, -16.773f, -15.676f, -14.586f, -13.503f, -12.427f, -11.357f, -10.294f, -9.238f, -8.187f, -7.143f, -6.105f, -5.073f, -4.046f, -3.026f, -2.012f, -1.003f, 0.000f, 0.997f, 1.988f, 2.974f, 3.954f, 4.927f, 5.895f, 6.857f, 7.813f, 8.762f, 9.706f, 10.643f, 11.573f, 12.497f, 13.414f, 14.324f, 15.227f, 16.123f, 17.011f, 17.891f, 18.762f, 19.626f, 20.481f, 21.326f, 22.162f, 22.988f, 23.804f, 24.609f, 25.402f, 26.183f, 26.951f, 27.706f, 28.447f, 29.172f, 29.881f, 30.573f, 31.247f, 31.901f, 32.533f, 33.143f, 33.727f, 34.285f, 34.813f, 35.310f, 35.770f, 36.193f, 36.572f, 36.903f, 37.180f, 37.397f, 37.545f
    };

    inline float getLeftWheelAngleLUT(float servoAngle) {
        float clamped = constrain(servoAngle, MIN_SERVO_DEG, MAX_SERVO_DEG);
        float index_f = (clamped - MIN_SERVO_DEG) / SERVO_STEP_DEG;
        int idx = (int)index_f;
        if (idx >= (int)TABLE_SIZE - 1) return LUT_LEFT[TABLE_SIZE - 1];
        
        float fraction = index_f - idx;
        return LUT_LEFT[idx] + fraction * (LUT_LEFT[idx + 1] - LUT_LEFT[idx]);
    }

} // namespace Ackermann

#endif // ACKERMANN_LUT_H
