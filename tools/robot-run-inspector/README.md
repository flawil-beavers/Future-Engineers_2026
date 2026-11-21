# Robot Run Inspector

Open a saved robot log in your browser to review the run, compare the estimated
path with the recorded plan and save your physical observations. The inspector
works offline in a current version of Edge, Chrome or Firefox.

## Open and review a run

1. Double-click `Start Robot Run Inspector.cmd` or open
   `Robot Run Inspector.html` in your browser. The HTML file is self-contained.
2. Choose the original TXT file from the USB drive or drag it into the window.
   If the file contains multiple `[OC] New obstacle run` markers, choose the
   run under **Laufabschnitt**.
3. Review the findings, sensor summary and flagged events. Click a line button
   to see the original log around that event. You can also search the log or
   enter a line number directly.
4. Use the full-run map to see where events occurred. Use **Spur** to inspect
   an individual trace and compare plan and estimated motion, where both are
   recorded.
5. Under **Physische Beobachtung**, record the layout, battery, any contact or
   intervention, the actual finish position and the video filename.

You can copy the folder to the competition laptop and check it with a real log
while Wi-Fi is off. Opening a log requires no connection to the robot.

## Read the map and timings

The full-run view shows the field and a detail of the parking exit, route
connection and final parking. A thick solid line shows the estimated drive.
A thin dark dashed line shows the selected accepted plan. Purple dotted lines
show logged before-and-after pose corrections in field coordinates. Grey
points show isolated discovery positions. Gaps over two seconds, timestamp
resets, changes of coordinate frame and explicit pose corrections break the
trace. Local positions from the initial rear-gap adjustment are available in
the individual trace.

- **Runde** selects all laps and parking, one lap, or **Parken / ohne Runde**.
  The connector belongs to lap 1. Parking stays in the parking selection even
  when a record still carries the last lap number. Corrections follow their
  logged phase.
- **Spuren** selects the plan, the estimated drive or both.
- **FE-Spielfeld** toggles the official mat artwork. The nominal walls,
  confirmed signs and known parking boundaries remain visible.
- **Planversion** appears when several route revisions are available. **Letzter
  Plan in Auswahl** uses the route active at the last field-position record
  in the selection. You can pick an earlier logged revision to compare it
  with the same drive. The dashed trace shows a single selected revision.

Sign markers come from confirmed map, avoidance or stored start-position
records in the selected run. Labels show the colour and position index, for
example R or G followed by a number. Icons mark nominal centres using
100 mm lateral offsets and 500 mm station spacing. They can be drawn when the
travel direction is known. The map reflects what the robot recorded, and the
icons use a fixed size. Magenta parking boundaries require both logged
`fixed_line_x` and `gap_mm` values.

The inspector reads run and lap times from `RUN_START`, `RUN_LAP` and
`RUN_END`. Total time includes observation holds, sensor pauses, braking and
final parking. If a complete run lacks a start or a confirmed finish, the
inspector shows the observed time span and leaves the completion time
unknown. An interrupted run or excerpt cannot establish completion.
[Run telemetry](../../simulation/RUN_TELEMETRY.md)

## Compare an individual trace

The individual view keeps each source, lap and coordinate frame separate.
Purple shows a recorded reference, green shows the estimated path, and yellow
shows steering targets ahead of the robot. Click a point to open its log
context. Both axes use the same scale.

| Log data | What the comparison measures |
| --- | --- |
| `PARK_DIAG` v2 | The nominal trace integrates requested curvature using the actual encoder travel. The inspector compares it with the estimated pose in the same record and reports mean, RMS, maximum, X/Y difference and wrapped heading difference. It separates the local start from each field rebase. |
| `CONNECTOR_POINT` and `CONNECTOR_TRACK` | The inspector matches each recorded position to the nearest eligible segment of the associated plan snapshot. It uses projections that fall within a recorded segment and respects missing indices and plan changes. The nearest segment on a loop may belong to a different point in time. |
| `LATER_TRACK` and other target records | The target is the point ahead used for steering. Its distance from the robot is a lookahead distance. A full path comparison requires a recorded plan or comparable reference. |

Red links show comparison distances. The table lists the 20 largest.
**Abweichungen als CSV speichern** exports every comparable point in the
selected trace. The chart draws up to 400 clickable points and 120 distance
markers. The statistics and CSV use the complete comparable set.

Accepted final-parking scans and explicitly applied initial wall corrections
are counted and break the estimated trace. When present, the `cs=` sequence
number identifies each applied nonzero wall correction, including repeated
corrections of the same size. In older records the viewer uses changes in the
reported value, which leaves identical repeated corrections ambiguous. A
rejected scan is shown as an event. A correction in a local or unknown frame
stays out of the field view. When both formats describe a field correction,
the viewer deduplicates events within 10 ms and 0.15 mm of coordinate rounding.

## Save and reopen your review

| Control | What it saves |
| --- | --- |
| **HTML-Bericht speichern** | The selected run, its timings, all its individual traces and the current full-run map selection. |
| **JSON speichern** | All run sections, original lines and your observations so you can restore the review later. |
| **Drucken / PDF** | The browser print view. Select your printer or save as PDF. |
| **SVG speichern** | The selected field view with your current lap, trace, plan and artwork settings. |

The browser saves files in its configured download location. Observations
stay in memory until you export JSON. To recover them, first open the same
original log and then choose **Beobachtungen aus JSON wieder laden** under
**Physische Beobachtung**. The original's SHA-256, byte count and run section
bounds must match. The browser needs local Web Crypto support to calculate the
hash. The inspector recalculates its findings from the original on every import.

For work in this repository, keep generated reports under
`local_workspace/robot-run-inspector/`. Archive the original text separately,
byte for byte. [Run evidence index](../../simulation/evidence/parking_exit_diagnostics/README.md) · [Evidence instructions](../../AGENTS.md)

## What the log can establish

The inspector reads explicit direction and completed-lap markers and
`build=` identifiers reported by individual modules. It flags gyro timeouts,
stream restarts, main-loop pauses, test and safety stops, parking outcomes,
corner aborts, suspected slip, observation timeouts, rejected connections
and logged pose corrections. It reports total-log overflow separately from
truncated or overflowing parking diagnostics. It displays ToF measurements
from `PARK_DIAG` v2 and side ranges from `FINAL_PARK_TRACE`.

The path is the robot's estimate. Use physical observations or video to
judge actual contact, movement and parking containment. A missing marker
leaves the associated result unknown. Single-session logs without distinct
start markers can contain multiple parking configurations that cannot be
split reliably into runs. The inspector flags filenames containing
`excerpt`. Completeness depends on the provenance of the original file.

The detailed firmware setting records `RUN_POSE` and route revisions within
the logging budget. Normal firmware records compact start, lap and finish
markers plus any available `PARK_DIAG`, `CONNECTOR_TRACK`, `LATER_TRACK` and
`FINAL_PARK_TRACE` traces. An incomplete route block or missing base revision
leaves a gap. [Run telemetry and detail settings](../../simulation/RUN_TELEMETRY.md)

The importer accepts UTF-8 or ASCII text up to 20 MB. The page treats the log
as text, embeds its resources and uses a Content Security Policy that blocks
network connections.

## Playfield artwork and development checks

The official playfield artwork is embedded in both field views and their
exports. The source PDF's 3,210 mm MediaBox includes a 5 mm bleed on each
side. Its 3,200 × 3,200 mm TrimBox spans −1,600 to +1,600 mm on each axis.
The field origin is at the centre, with +X right and +Y up. The starting
straight is at the bottom in both directions. The outer and inner wall
overlays are nominally at ±1,500 mm and ±500 mm. The 24 printed sign positions
match the nominal coordinates to within 0.6 mm in the artwork.
[Official game mat](https://wro-association.org/wp-content/uploads/WRO-2026_FutureEngineers_Playfield.pdf) · [Source and asset metadata](assets/fe-2026-mat.json)

Run these checks from the repository root with Python and Node.js.

```powershell
python tools/robot-run-inspector/check-field-asset.py
node tools/robot-run-inspector/test-inspector.cjs
node tools/robot-run-inspector/test-app.cjs
```

The parser tests use the delivered HTML and cover historical logs and synthetic
edge cases. The application test uses a minimal DOM adapter to check import,
run selection, search, line jumps and exports.

To regenerate the artwork, install Poppler's `pdftocairo` and run
`python tools/robot-run-inspector/build-field-asset.py`. The script checks the
source PDF hash, updates the SVG and metadata, and replaces the marked
embedded asset in the HTML. It stores the input PDF and raw conversion in
`local_workspace/` and stops for review if the source changes. SVG bytes can
vary by Poppler version. [Embedded SVG source](assets/fe-2026-mat.svg)
