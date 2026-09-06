# Parking-entry scout log fixtures

These eight robot logs are the regression inputs for
`simulation/parking_entry_scout_sim.py`. They were copied unchanged from the
ignored working log archive.

## Firmware context

The telemetry identifies the tested firmware as the initial 55 mm
preceding-station scout implementation. The exact Git commit was not recorded
inside the files. The last documented build for that implementation used
366688 bytes RAM and 436592 bytes flash on `giga_r1_m7`.

Battery voltage and physical-contact reports were not supplied for this log
set. Therefore, connector or lap completion in these files is telemetry only,
not physical acceptance.

## Cases

| Log | Direction | Relevant outcome |
| --- | --- | --- |
| `log_362.txt` | CW | Primary station 1 timed out UNKNOWN; old firmware stopped without a scout/retry. |
| `log_363.txt` | CW | Scout station 0 marked CLEAR, then green seat 0 confirmed during retrace; connector completed. |
| `log_364.txt` | CCW | Primary green seat 5 confirmed; 55 mm scout station 1 remained unresolved and stopped safely. |
| `log_365.txt` | CCW | Primary station 2 CLEAR; 55 mm scout station 1 remained unresolved and stopped safely. |
| `log_366.txt` | CW | Primary/scout CLEAR and connector completed; run was manually disabled later. |
| `log_367.txt` | CW | Connector completed and three laps completed in telemetry. |
| `log_368.txt` | CW | Connector completed, three laps completed, and controlled stop completed in telemetry. |
| `log_369.txt` | CCW | Primary CLEAR; unrelated red seat 7 confirmed; intended 55 mm scout station 1 remained unresolved. |

## Regression use

Run from the repository root:

```powershell
python simulation/parking_entry_scout_sim.py
```

The current 85 mm replay must keep all eight preceding-seat views inside the
calibrated clear-evidence camera window and retain positive modeled wall and
legal-pillar clearance. When adding logs, extend the requested range only when
every number in that range is present, then update this table with direction,
layout, firmware revision, battery voltage, and the user's physical report.

Do not edit raw telemetry to make a regression pass. If a source file is
corrupt or incomplete, retain it as evidence and document why the replay
excludes it.

