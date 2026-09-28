#pragma once

// OBSTACLE CHALLENGE logging switch, separate from the drive/mode selectors in
// config.h. 1 records parking-exit diagnostics with a 192 KiB logger; 0
// compiles them out and keeps the 128 KiB logger. It changes no motion.
// Override with -DPARKING_EXIT_DIAGNOSTICS_ENABLED=0 for a production build.
#ifndef PARKING_EXIT_DIAGNOSTICS_ENABLED
#define PARKING_EXIT_DIAGNOSTICS_ENABLED 1
#endif

#if PARKING_EXIT_DIAGNOSTICS_ENABLED
#define ROBOT_LOG_BUFFER_SIZE (192 * 1024)
#else
#define ROBOT_LOG_BUFFER_SIZE (128 * 1024)
#endif
