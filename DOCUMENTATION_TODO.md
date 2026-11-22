# World Final documentation checklist

Use this checklist to compare the published documentation with the finished
robot and competition firmware. Confirm items that require physical validation
on the robot and record the results.

Once the hardware and competition software are final, clear resolved TODOs
from the published documentation, regenerate `README.pdf` and review the
official rules, Documentation Rubric, Questions & Answers and any announced
surprise rule before the repository deadline.

The canonical official links used for this checklist are recorded in the [WRO 2026 Rules Reference](WRO_2026_RULES.md).

- [x] Replace the June robot photographs with the supplied front, rear, left, right, top and bottom views of the current robot (November 20).
- [x] Link the supplied 2026 team photo in the README (November 20). Public GitHub rendering remains part of the final repository link check below.
- [x] Record the final mass of 680 g with the rear ToF installed (team confirmation, November 20).
- [x] Record the final robot height of 115 mm (team confirmation, November 20).
- [x] Record the target parking gap of 237.5 mm between the inside faces of the markers (team confirmation, November 20). The firmware's initial estimate is 247.5 mm.
- [x] Add and link the supplied version 74 Fusion archive, printable `.3mf` project, MK4 Input Shaper binary G-code and PrusaSlicer preparation/toolpath screenshots; document how to open the project and adapt printer settings, plus printer, nozzle, material, layer height, infill, supports and estimates. Earlier CAD versions remain available in Git history (November 20).
- [x] Confirm version 74 is the current CAD model, including the rear-ToF mount (team confirmation, November 20).
- [x] Confirm that all 17 version 74 parts print without supports in their saved orientations and document the setting in the CAD guide (team confirmation, November 20).
- [ ] Check the CAD and print assets against any subsequent physical changes before submitting the final documentation.
- [x] Replace the June schematic with the supplied [revision 1 wiring diagram](img/diagrams/Wiring_Diagram_FE2026_v1.png), which shows the 2S pack and all three ToF sensors.
- [x] Verify that the drawing's rear-ToF pin labels match the M4 firmware (A3 SDA, A4 SCL).
- [x] Document the internal M4-to-M7 OpenAMP/RPC link in the [README data-flow diagram](README.md#control-ownership-and-data-flow).
- [x] Identify the Arducam GC2145, Mini-360 adjustable step-down module, generic 3.3 V to 5 V logic-level shifter and BerryBase SVM330-Y display. Retain the team's BOM quantities and project cost estimates.
- [x] Check the updated wiring drawing for consistent M2 motor connections, voltmeter polarity, and labels for the switches, camera, ToFs, Mini-360 and generic level shifter (November 20).
- [x] Verify the power wiring against the robot, including the Mini-360 output and loads, ground return, module terminals, voltmeter tap and motor supply (team confirmation, November 20).
- [ ] Recheck power budget by rail against the finished schematic and measure typical and peak battery and regulator currents if available. Existing current figures are planning estimates based on datasheet values and engineering allowances.
- [x] Move the Bill of Materials before Assembly and update the README navigation (November 18).
- [x] Replace the historical Open Challenge video and add the supplied Obstacle Challenge link and matching previews (November 18): Open `koMCLdpg0-I`, Obstacle `5dCMkFn-6TI`.
- [x] Confirm both challenge videos are available (team confirmation, November 20).
- [ ] Verify the titles, signed-out playback from `main` and at least 30 seconds of autonomous driving in each video during the final link check.
- [ ] Revalidate the current 30:1 drivetrain at low speeds in forward and reverse, straight and full-lock, with charged and lower-voltage packs. Record mean speed, instantaneous speed ripple, shaking/noise, stalls, stopping behaviour and current consumption. Update the low-speed discussion and test results in `README.md` with the findings and any further fixes; earlier mean-speed passes do not establish that the currently reported problems are resolved.
- [ ] Validate camera colour and distance calibration under competition-like lighting and add the resulting detection and reliability metrics.
- [ ] Confirm the final camera clock and timing configuration and reconcile the camera description and engineering testing table with its measured frame intervals. If reverting from 24 MHz with input divide-by-two to 12 MHz without that divider, compare the current full-FOV configuration under varied lighting and motion before claiming equivalent reliability. Do not attribute a performance improvement to 24 MHz itself; distinguish continuous-capture and vertical-blanking improvements from the external clock setting.
- [ ] Validate ToF wall-edge and perpendicular-corner detection on competition-like black walls in both directions, at the intended wall distances and speeds. Measure the actual wall-loss position and any perpendicular-wall returns; compare raw and filtered ranges with encoder travel and heading to distinguish detection-cone, measurement-timing and clamping delays. Record the results in `README.md` rather than assuming the sensor centreline crossing is the detection point.
- [ ] Disable all development-only, test-only and practice flags. Build and upload the final `giga_r1_m4` image first and the final `giga_r1_m7` image second, then verify both complete challenge modes from the physical start switch.
- [ ] Run the final full-field validation matrix in both directions, including difficult adjacent pillars, parked starts, three laps, final parking and lower-voltage operation. Record run counts, completion rate, time, minimum measured clearance and every intervention/contact.
- [ ] Use the latest `main` for firmware. Record the evaluated and actual competition commit IDs and any local M7/M4 configuration, CAD and calibration changes in a version note when known.
- [x] Submit the public GitHub URL (team confirmation, November 20).
- [x] Confirm the organizer's November 24 evaluation cutoff (team confirmation, November 20).
- [ ] Review the General Rules §7 code-volume requirement with the organizers if needed. Git history begins March 15 (`25eeb32`) with camera and main-loop code. The second and third commits are dated May 18 and 21. The first commit's one-fifth share of the final code remains unverified.
- [x] Review the General Rules, Documentation Rubric and Future Engineers Q&A for the November 22 documentation audit. Recheck the official sources before the evaluation cutoff.
- [ ] Push the updated documentation to `main`. Confirm the submitted URL shows the revised README, all links work, all important code, CAD and wiring files are visible, and the repository will remain public for at least 12 months after the event.
- [x] Review the [README’s software architecture and challenge logic](README.md#software-architecture-and-challenge-strategy) against November 22 firmware (`4033ef0`), including the fixed parking entry, modified-sign voting, recovery viewpoints and USB save verification. Check later firmware changes again before publication.
- [ ] Review the judge-facing `README.md` against the final robot and recorded test evidence. Use stable links to public reports or run IDs for earlier observations and keep unresolved documentation tasks in this checklist.
- [x] Regenerate `README.pdf` from the updated `README.md`, inspect all 23 rendered pages and decode the three QR codes. Each challenge preview and its QR code share a page (November 22).
- [ ] Print the required hardcopy for the World Final.
- [x] Verify that the repository and both challenge-video QR codes scan correctly (team confirmation, November 21).

November 18 README update: the team confirms the rear-ToF addition, unchanged dimensions and unchanged print settings (0.2 mm, no supports). The README reflects current full-run flags, measured parking geometry, modified-sign implementation and recorded validation limits.

November 20 documentation update: the team confirmed v74 includes the rear-ToF mount. The linked `.3mf` contains 17 meshes with the saved plate and print settings. The README links the current CAD, team photo, six current robot views, challenge videos, revision 1 wiring diagram and engineering evidence. The regenerated journal includes the confirmed mass, height and target bay gap.

November 22 documentation update: the current known-map and fixed-parking host checks passed. Seven of 2,827 cases in the later-lap footprint check fell below the 50 mm modelling target. The README and simulation guide report the 43.8 mm minimum sign clearance. Repeat the affected layouts physically and record observed clearance and contact before claiming a higher margin.
