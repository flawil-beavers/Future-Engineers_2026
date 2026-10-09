#pragma once
#include <stddef.h>
#include <stdint.h>

#ifndef RUN_TELEMETRY_DETAILED
#define RUN_TELEMETRY_DETAILED 0
#endif

#ifdef ARDUINO
void run_telemetry_start();
void run_telemetry_tick();
void run_telemetry_phase(const char *phase, unsigned lap = 0);
void run_telemetry_finish(const char *outcome, const char *reason);
void run_telemetry_lap(unsigned lap);
void run_telemetry_motion(const char *state);
void run_telemetry_pose_change(const char *kind, float bx, float by, float bh,
                               float ax, float ay, float ah);
// Points begin with float x, y, heading; stride permits existing PathPoint arrays.
void run_telemetry_route(const char *kind, const void *points, unsigned count,
                         size_t stride, bool closed);
#else
// Existing motor-independent host simulations do not instantiate the logger.
inline void run_telemetry_start() {}
inline void run_telemetry_tick() {}
inline void run_telemetry_phase(const char *, unsigned = 0) {}
inline void run_telemetry_finish(const char *, const char *) {}
inline void run_telemetry_lap(unsigned) {}
inline void run_telemetry_motion(const char *) {}
inline void run_telemetry_pose_change(const char *, float,float,float,float,float,float) {}
inline void run_telemetry_route(const char *, const void *, unsigned, size_t, bool) {}
#endif
