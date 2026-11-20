# World Final proofreading checklist

## Main documents and rules

- [x] [Main README](README.md) — Check the BOM, assembly, wiring, challenge strategy and test claims against the current robot. Review the [AI use declaration](README.md#ai-use-declaration) for accuracy.
- [ ] [Documentation checklist](DOCUMENTATION_TODO.md) — Confirm the remaining measurements, assets and submission checks.
- [x] [WRO rules reference](WRO_2026_RULES.md), [General Rules](https://wro-association.org/wp-content/uploads/WRO-2026-Future-Engineers-Self-Driving-Cars-General-Rules.pdf) and [documentation rubric](https://wro-association.org/wp-content/uploads/WRO-2026-Future-Engineers-Documentation-Rubric.pdf) — Check requirements and scoring.
- [x] [Official Q&A](https://wro-association.org/competition/questions-answers/) and [Surprise Rule](https://wro-association.org/wp-content/uploads/WRO-2026-Future-Engineers-Extra-Task.pdf) — Check parking and modified-sign wording.

## Hardware and assembly references

- [x] [Wiring diagram](img/diagrams/Wiring_Diagram_FE2026_v1.png) — Checked pins and component labels against the BOM. The team confirmed the power wiring on the robot.
- [ ] [CAD guide](CAD/README.md) — Check the [printing project](CAD/Car_v74.3mf), [Fusion model](CAD/Car_v74.f3z), [prepared print job](CAD/printing/Car_v74_0.4n_0.2mm_PLA_MK4IS_3h1m.bgcode) and [steering calibration workflow](CAD/README.md#workflow).
- [ ] [Ackermann calibration](CAD/ACKERMANN_KINEMATICS_DOCUMENTATION.md#turning-radius-comparison) and [measured parking contour](simulation/RETURNED_651_656_PARKING_20261105.md#measured-geometry-and-implementation) — Check dimensions and turning-radius figures.
- [ ] Component references — [Motor](https://www.bastelgarage.ch/6v-530rpm-30-1-micro-gear-motor-with-encoder), [regulator](https://sph.ethz.ch/materials/product/10.2.4), [voltmeter](https://www.berrybase.ch/detail/0191f2bebc0d71c7afa5fd3d0c6cde11) and [ToF pinout](https://learn.adafruit.com/adafruit-vl53l4cx-time-of-flight-distance-sensor/pinouts).
- [ ] Manufacturer references — Check the [50:1](https://www.dfrobot.com/product-1432.html), [100:1](https://www.dfrobot.com/product-1433.html) and [30:1](https://www.dfrobot.com/product-1431.html) motor specifications, the [VL53L3CX](https://www.st.com/en/imaging-and-photonics-solutions/vl53l3cx.html) and [VL53L4CX](https://www.st.com/en/imaging-and-photonics-solutions/VL53L4CX) descriptions, the [ToF field-of-view data](https://www.st.com/resource/en/datasheet/vl53l4cx.pdf#page=26) and the [Arducam reference](https://docs.arducam.com/DVP-Camera-Module/Arduino-GIGA/Arduino-GIGA/Introduction/).
- [ ] [Rear ToF setup](REAR_TOF_M4_SETUP.md#parking-position-configuration) — Check the sensor offset, parking target and firmware settings against the README.

## Test evidence and reproduction

- [ ] [Parking exit study](simulation/PARKING_EXIT_PATH_SIMULATION.md), [physical parking runs](simulation/RETURNED_651_656_PARKING_20261105.md#what-the-returned-runs-establish) and [direct parking handoff](simulation/DIRECT_PARKING_HANDOFF_20261119.md).
- [ ] Modified-sign evidence — [Camera calibration result](simulation/SURPRISE_RULE_MEASURED_VALIDATION_20261119.md#ergebnis-und-aenderung), [image measurements](simulation/SURPRISE_RULE_MEASURED_VALIDATION_20261119.md#aktuelle-bildmessungen) and [CW run verification](simulation/CW_LOGS_3_6_REVIEW_20261119.md#verification).
- [ ] [Later-lap clearance study](simulation/LAP23_CLEARANCE_REVIEW_20261120.md#validation) — Check that the README describes the clearance figures as model results.
- [ ] Simulation workflow — [Sign recognition](simulation/README.md#sign-recognition-check), [route clearance](simulation/README.md#route-clearance-check) and [parking exit](simulation/README.md#parking-exit-model).
- [ ] [USB logging procedure](README.md#reproduce-the-evidence), [run evidence index](simulation/evidence/parking_exit_diagnostics/README.md) and [run inspector guide](tools/robot-run-inspector/README.md) — Check the save triggers, LED signals and steps for finding and reviewing a log.

## Firmware references

Spot-check the claims and links in the README against these files.

- [ ] Build and control — [PlatformIO](platformio.ini), [configuration](include/config.h), [main loop](src/main.cpp), [modes](src/mode_manager.cpp) and [motor control](src/motor_control.cpp).
- [ ] Sensors and pose — [M7 sensors](src/sensors.cpp), [M4 rear ToF](src/m4/rear_tof_m4.cpp), [RPC receiver](src/rear_tof_rpc.cpp), [RPC protocol](include/rear_tof_rpc_protocol.h) and [position estimator](src/position_estimator.cpp).
- [ ] Camera and vision — [capture driver](lib/arducam_dvp_async/src/arducam_dvp.cpp), [GC2145 setup](lib/arducam_dvp_async/src/GC2145/gc2145.cpp), [vision](src/vision.cpp) and [sign height](include/obstacle_sign_height.h).
- [ ] Challenge logic and telemetry — [Open navigation](src/navigation_controller.cpp), [Obstacle start](src/obstacle.cpp), [routes](src/obstacle_path.cpp), [final parking](src/final_parking.cpp), [telemetry](src/run_telemetry.cpp) and [logger](src/logger.cpp).

## Final media and print checks

- [x] [Team, robot and assembly photos](img/) — Check the captions and every image in the rendered README.
- [x] [Open video](https://www.youtube.com/watch?v=koMCLdpg0-I) and [Obstacle video](https://www.youtube.com/watch?v=5dCMkFn-6TI) — Check previews, signed-out playback and at least 30 seconds of autonomous driving in each.
- [x] [Public GitHub repository](https://github.com/flawil-beavers/Future-Engineers_2026/tree/main) — Check the rendered README and its links on GitHub.
- [ ] [README.pdf](README.pdf) — Review after regeneration. Check page breaks and diagram legibility, then scan the repository and both video QR codes from the printout.
