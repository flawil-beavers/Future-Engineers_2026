#pragma once

#include "parking_exit_diagnostics_config.h"
#include "position_estimator.h"
#include <Arduino.h>

#if PARKING_EXIT_DIAGNOSTICS_ENABLED
void parking_exit_diagnostics_reset();
void parking_exit_diagnostics_update(const char *state,
                                     uint8_t segment,
                                     int8_t turn_sign,
                                     float segment_target_mm,
                                     const char *expected_reference);
void parking_exit_diagnostics_rebase(const char *reason);
void parking_exit_diagnostics_correction(const char *source,
                                         const PositionEstimate &before,
                                         const PositionEstimate &after);
void parking_exit_diagnostics_finish(const char *result);
#else
inline void parking_exit_diagnostics_reset() {}
inline void parking_exit_diagnostics_update(const char *, uint8_t, int8_t,
                                            float, const char *) {}
inline void parking_exit_diagnostics_rebase(const char *) {}
inline void parking_exit_diagnostics_correction(const char *,
                                                const PositionEstimate &,
                                                const PositionEstimate &) {}
inline void parking_exit_diagnostics_finish(const char *) {}
#endif

