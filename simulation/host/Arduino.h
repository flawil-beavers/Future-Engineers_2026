#pragma once

// Minimal Arduino surface for the source-extracted geometry fixtures only.
// Firmware builds continue to use the board framework's Arduino.h.
#include <cstddef>
#include <cmath>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
template<class T> T constrain(T value, T low, T high)
{
    return value < low ? low : (value > high ? high : value);
}
