# World Final Documentation TODO

This checklist tracks the documentation work that remains before the WRO Future Engineers World Final. Keep it synchronized with the physical robot and competition software. Do not mark an item complete using simulation or telemetry alone when physical validation is required.

Complete the checklist after the hardware and competition software are frozen. Before the repository deadline, remove resolved TODO wording from the published documentation, regenerate `README.pdf`, and review the current official Future Engineers rules, Documentation Rubric, Questions & Answers and any announced surprise rule.

The canonical official links used for this checklist are recorded in the [WRO 2026 Rules Reference](WRO_2026_RULES.md).

- [ ] Photograph the final robot from the front, rear, left, right, top and bottom. Replace any image that does not show the rear ToF, final wiring, final camera mount or final chassis.
- [ ] Take the final team photo and confirm all image links render in the public GitHub repository.
- [ ] Remeasure the finished robot's length, height, wheelbase, track width, straight-wheel width, maximum steering envelope and mass. Reconcile the measurements with `include/config.h`, the parking model and the Robot Specifications table.
- [ ] Freeze the final Fusion 360 model, export the current printable package, identify obsolete CAD versions clearly and update the assembly link from `Car_v71.3mf` if necessary.
- [ ] Update the wiring diagram to show the 2S battery pack, both side ToF sensors, the rear ToF on M4 pins A3/A4, the M4-to-M7 RPC relationship and every final power connection.
- [ ] Explain the exact final purpose and connections of the logic-level converter, DC-DC converter and voltage display after the wiring is frozen.
- [ ] Recheck the BOM, quantities, spare parts, component prices, regulator rating and calculated typical/peak current against the finished robot.
- [ ] Consider moving the Bill of Materials closer to the Assembly section so builders can find the required parts before starting; update the README navigation if its location changes.
- [ ] Record new Open and Obstacle Challenge videos with clear titles. Each must contain at least 30 seconds of autonomous driving and be public or accessible by link. Replace the historical Open Challenge video in `README.md` and add an Obstacle Challenge video subsection with its final link and preview.
- [ ] Revalidate the current 30:1 drivetrain at low speeds in forward and reverse, straight and full-lock, with charged and lower-voltage packs. Record mean speed, instantaneous speed ripple, shaking/noise, stalls, stopping behaviour and current consumption. Update the low-speed discussion and test results in `README.md` with the findings and any further fixes; earlier mean-speed passes do not establish that the currently reported problems are resolved.
- [ ] Validate camera colour and distance calibration under competition-like lighting and add the resulting detection and reliability metrics.
- [ ] Confirm the final camera clock and timing configuration and reconcile the camera description and engineering testing table with its measured frame intervals. If reverting from 24 MHz with input divide-by-two to 12 MHz without that divider, compare the current full-FOV configuration under varied lighting and motion before claiming equivalent reliability. Do not attribute a performance improvement to 24 MHz itself; distinguish continuous-capture and vertical-blanking improvements from the external clock setting.
- [ ] Validate ToF wall-edge and perpendicular-corner detection on competition-like black walls in both directions, at the intended wall distances and speeds. Measure the actual wall-loss position and any perpendicular-wall returns; compare raw and filtered ranges with encoder travel and heading to distinguish detection-cone, measurement-timing and clamping delays. Record the results in `README.md` rather than assuming the sensor centreline crossing is the detection point.
- [ ] Disable all development-only, test-only and practice flags. Build and upload the final `giga_r1_m4` image first and the final `giga_r1_m7` image second, then verify both complete challenge modes from the physical start switch.
- [ ] Run the final full-field validation matrix in both directions, including difficult adjacent pillars, parked starts, three laps, final parking and lower-voltage operation. Record run counts, completion rate, time, minimum measured clearance and every intervention/contact.
- [ ] Create a named competition release or version note that identifies the exact firmware, CAD, calibration and documentation used at the World Final.
- [ ] Check that the required meaningful commits exist before the WRO deadlines and that all important material is available by the evaluation cutoff.
- [ ] Review the latest official Future Engineers rules, Documentation Rubric, Questions & Answers and any announced surprise rule. Apply any newer clarification before submission.
- [ ] Confirm the GitHub repository is public, all links work, all important code, CAD and wiring files are visible, and the repository will remain public for at least 12 months after the event.
- [ ] Review the judge-facing `README.md`, including the conclusion, against the final robot and recorded test evidence. Update provisional performance descriptions after validation, preserve relevant development history and limitations, and keep outstanding maintenance work in this checklist or hidden comments rather than visible future-work wording.
- [ ] Regenerate `README.pdf` from the final `README.md`, inspect every rendered page and print the required hardcopy for the World Final.
- [ ] Scan the repository and both challenge-video QR codes from the final printed PDF. Verify the canonical repository URL, final video destinations and public access; confirm both video QR codes appear in their respective challenge subsections.
