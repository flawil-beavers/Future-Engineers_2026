#pragma once

#include <math.h>
#include "config.h"

// Shared by the real motor controller and recorded-map candidate rollouts.
// No hardware, global motor state or side effects beyond the referenced state.
namespace DriveProfile {
inline float clamp(float value, float low, float high) {
    // Match Arduino constrain's branch order, including legacy calibration
    // settings with a maximum below the configured minimum acceleration.
    return value<low ? low : (value>high ? high : value);
}

inline int approachSpeed(float remaining, float cruise, float terminal,
                         float reserve, float taperDecel) {
    const float usable = fmaxf(0.0f, remaining - reserve);
    return static_cast<int>(fminf(cruise,
        sqrtf(terminal * terminal + 2.0f * taperDecel * usable)));
}

inline float advance(float &speed, float &acceleration, float &limit,
                     int target, float dt, float maximumAcceleration) {
    if (!(dt>0.0f) || !isfinite(dt)) return 0.0f;
    if ((target>0 && speed<=0 && acceleration<0) ||
        (target<0 && speed>=0 && acceleration>0)) acceleration=0;
    limit=target==0 ? fminf(SOFT_STOP_DECELERATION_MMSS,maximumAcceleration)
        : clamp(fabsf(float(target))*PROFILE_ACCEL_PER_TARGET_SPEED,
                MIN_PROFILE_ACCELERATION_MMSS,maximumAcceleration);
    const float error=target-speed;
    const float allowed=sqrtf(2.0f*DRIVE_ACCEL_RELEASE_JERK_MMSSS*fabsf(error));
    const float desired=fabsf(error)>0.5f
        ? copysignf(fminf(limit,allowed),error) : 0.0f;
    const bool releasing=acceleration*desired<0 || fabsf(desired)<fabsf(acceleration);
    const float jerk=releasing ? DRIVE_ACCEL_RELEASE_JERK_MMSSS : DRIVE_JERK_LIMIT_MMSSS;
    acceleration+=clamp(desired-acceleration,-jerk*dt,jerk*dt);
    float next=speed+acceleration*dt;
    if ((target>0 && next<0) || (target<0 && next>0) ||
        (target==0 && ((speed>0 && next<0) || (speed<0 && next>0)))) next=0;
    if ((error>0 && next>=target) || (error<0 && next<=target) || fabsf(error)<=0.5f) {
        speed=target; acceleration=0;
    } else speed=next;
    return speed*dt;
}
} // namespace DriveProfile
