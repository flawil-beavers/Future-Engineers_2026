#pragma once

// Override with -DPARKING_EXIT_DIAGNOSTICS_ENABLED=0 for a production build
// that compiles out parking-exit diagnostics and retains the 128 KiB logger.
#ifndef PARKING_EXIT_DIAGNOSTICS_ENABLED
#define PARKING_EXIT_DIAGNOSTICS_ENABLED 1
#endif

#if PARKING_EXIT_DIAGNOSTICS_ENABLED
#define ROBOT_LOG_BUFFER_SIZE (192 * 1024)
#else
#define ROBOT_LOG_BUFFER_SIZE (128 * 1024)
#endif
