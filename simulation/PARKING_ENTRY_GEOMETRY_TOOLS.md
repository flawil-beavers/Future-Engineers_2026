# Parking-entry geometry tools

This document covers the two offline parking-entry geometry programs in this
folder. They support firmware design and log analysis; neither program proves
that a powered robot will avoid contact.

## Current scout replay

`parking_entry_scout_sim.py` replays the measured parking-entry scan poses
recorded in robot logs. For each log it:

1. Reads the selected CW or CCW direction and the primary scan pose.
2. Applies the ideal full-lock reverse-arc model for the configured scout
   distance.
3. Calculates camera bearing and range to the preceding inner legal seat.
4. Samples the complete robot capsule against the fixed inner/outer walls and
   the legal guarded-pillar position.
5. Fails if the target is outside the calibrated clear-evidence window or any
   modeled clearance is non-positive.

The script also checks the intended high-level outcomes of the connector
prerequisites: both stations resolved permits connector construction, an
initial unresolved primary permits one retry after a resolved scout, and an
unresolved scout or retry remains stopped. These are small consistency checks,
not an execution of the C++ state machine.

The default tracked input is `fixtures/parking_entry_scout/log_362.txt`
through `log_369.txt`. A different directory or log range can be supplied:

```powershell
python simulation/parking_entry_scout_sim.py
python simulation/parking_entry_scout_sim.py `
  --log-dir C:\path\to\logs --first-log 362 --last-log 369
```

The replay uses only the Python standard library. A successful exit status
requires every requested log to exist, contain the expected parking-entry
telemetry, and pass the geometry checks.

The fixture README records the firmware context and the limits of the physical
evidence. Keep it synchronized whenever logs are added or replaced.

### Model inputs

The values mirror the production configuration and clearance model:

| Input | Current value | Firmware source |
| --- | ---: | --- |
| Scout arc | 85 mm | `OBSTACLE_PARKING_ENTRY_SCOUT_ARC_MM` |
| Full-lock rear-axle radius | 109 mm | `OBSTACLE_PARKING_ENTRY_SCAN_RADIUS_MM` |
| Camera position ahead of rear axle | 125 mm | `OBSTACLE_CAMERA_LOCAL_X_MM` |
| Trusted camera range | 230--600 mm | `OBSTACLE_DISCOVERY_VIEW_MIN_MM` / `MAX_MM` |
| Clear-evidence bearing limit | 26.4 degrees | camera FOV, fraction, and margin constants |
| Robot capsule radius/front axis | 70/60 mm | obstacle clearance constants |
| Pillar movement-circle radius | 42.5 mm | `OBSTACLE_PILLAR_MOVEMENT_RADIUS_MM` |

Keep the script synchronized when any of these firmware values or fixed-field
coordinates change. The script intentionally does not read C++ at runtime, so
a configuration review remains explicit in code review.

### Result behind the 85 mm scout

Replaying logs 362--369 rejected the previously prepared 75 mm distance: the
three CCW poses left the preceding seat at 27.7--29.6 degrees, outside the
validated 26.4-degree clear-evidence window. At 85 mm, all eight cases model at
6.5--20.9 degrees and 244--309 mm. The minimum sampled wall and legal-pillar
clearances are 139.5 mm and 166.1 mm respectively.

These results assume exact field localization, exact steering radius, ideal
encoder travel, and an exact same-arc retrace. The model does not include
servo settling, tire slip, braking overshoot, camera segmentation, pillar
placement within its movement circle, localization error, battery effects, or
the user's physical contact observation. The firmware preflight and physical
test sequence therefore remain mandatory.

## Historical scan-pose search

`parking_scan_search.py` is the earlier bounded search that explored two extra
motion segments after the parking exit/localization sequence. It imports
`parking_exit_swept_search.py`, searches forward/reverse and full-lock control
pairs, checks the parking-piece swept footprint across placement tolerances,
and reports candidates whose target lies inside a 25-degree, 230--600 mm
camera window.

Run it from the repository root:

```powershell
python simulation/parking_scan_search.py
```

This tool records design history and may help if the physical robot geometry
changes. It does not describe the current production transition by itself: the
firmware now uses localized direction-specific primary scan arcs, an 85 mm
preceding-station scout, exact retrace, and an obstacle-aware connector.

## Maintenance and acceptance

When changing parking-entry geometry:

1. Preserve the fixed field and legal seat coordinates.
2. Update the firmware and matching model inputs together.
3. Add the new logs to the replay range and retain failed cases.
4. Run `git diff --check`, the replay, and only the affected PlatformIO build.
5. Record modeled minima as predictions, not physical measurements.
6. Upload only with explicit permission and accept a route only after the user
   reports no wall or pillar contact.
