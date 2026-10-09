#pragma once

#include <stdint.h>
#include <math.h>
#include <ctype.h>
#include <errno.h>
#include <limits.h>
#include <stdlib.h>

namespace RuntimeSafety {
// LOW is accepted immediately. HIGH must remain stable for the full debounce.
// An accepted HIGH stays accepted after a software pause, preserving manual resume.
struct EnableSwitchFilter {
    bool accepted = false;
    bool rising = false;
    uint32_t highSinceUs = 0;
    void reset(bool initial, uint32_t nowUs) {
        accepted = initial; rising = false; highSinceUs = nowUs;
    }
    int update(bool high, uint32_t nowUs, uint32_t debounceUs) {
        if (!high) {
            rising = false;
            if (accepted) { accepted = false; return -1; }
            return 0;
        }
        if (accepted) return 0;
        if (!rising) { rising = true; highSinceUs = nowUs; }
        if (uint32_t(nowUs - highSinceUs) < debounceUs) return 0;
        rising = false; accepted = true; return 1;
    }
};

inline bool quaternionYaw(float r, float i, float j, float k, float &yawDeg) {
    if (!isfinite(r) || !isfinite(i) || !isfinite(j) || !isfinite(k)) return false;
    const double norm = double(r)*r + double(i)*i + double(j)*j + double(k)*k;
    // SH2 rotation reports describe a unit quaternion. This deliberately broad
    // tolerance admits quantization/normalization error, but rejects finite
    // corrupt packets instead of turning them into a plausible heading.
    if (norm < 0.5 || norm > 1.5) return false;
    // The homogeneous formula accepts small normalization error without changing yaw.
    const double yaw = atan2(2.0*(double(i)*j + double(r)*k),
                             double(r)*r + double(i)*i - double(j)*j - double(k)*k);
    yawDeg = float(yaw * 57.29577951308232);
    return isfinite(yawDeg);
}

inline bool sampleFresh(uint32_t nowMs, uint32_t sampledMs, bool received,
                        uint32_t timingBudgetUs, uint32_t minimumAgeLimitMs) {
    const uint32_t intervalLimit = timingBudgetUs / 1000U + 100U;
    const uint32_t limit = intervalLimit > minimumAgeLimitMs
        ? intervalLimit : minimumAgeLimitMs;
    return received && uint32_t(nowMs - sampledMs) <= limit;
}

inline bool integerToken(const char *&argument, int &value) {
    while (isspace(static_cast<unsigned char>(*argument))) ++argument;
    char *end = nullptr; errno = 0;
    const long parsed = strtol(argument, &end, 10);
    if (end == argument || errno == ERANGE || parsed < INT_MIN || parsed > INT_MAX ||
        (*end && !isspace(static_cast<unsigned char>(*end)))) return false;
    value = static_cast<int>(parsed); argument = end; return true;
}
inline bool argumentEnd(const char *argument) {
    while (isspace(static_cast<unsigned char>(*argument))) ++argument;
    return *argument == '\0';
}

// Parse the legacy single-letter interface without atoi truncation/overflow.
// Bare commands retain their historic zero value; callers decide allowed ranges.
inline bool legacyCommand(char *message, char &command, int &value, char *&argument) {
    if (!message) return false;
    while (isspace(static_cast<unsigned char>(*message))) ++message;
    if (!*message) return false;
    command = *message++;
    while (isspace(static_cast<unsigned char>(*message))) ++message;
    argument = message; value = 0;
    if (!*message) return true;
    char *end = nullptr;
    errno = 0;
    const long parsed = strtol(message, &end, 10);
    if (end == message || errno == ERANGE || parsed < INT_MIN || parsed > INT_MAX)
        return false;
    while (isspace(static_cast<unsigned char>(*end))) ++end;
    if (*end) return false;
    value = static_cast<int>(parsed);
    return true;
}
} // namespace RuntimeSafety
