# World Final proofreading checklist

## Main documents and rules

- [x] [Main README](../../README.md) — Check the BOM, assembly, wiring, challenge strategy and test claims against the current robot. Review the [AI use declaration](../../README.md#ai-use-declaration) for accuracy.
- [ ] [Documentation checklist](DOCUMENTATION_TODO.md) — Confirm the remaining measurements, assets and submission checks.
- [x] [WRO rules reference](WRO_2026_RULES.md), [General Rules](https://wro-association.org/wp-content/uploads/WRO-2026-Future-Engineers-Self-Driving-Cars-General-Rules.pdf) and [documentation rubric](https://wro-association.org/wp-content/uploads/WRO-2026-Future-Engineers-Documentation-Rubric.pdf) — Check requirements and scoring.
- [x] [Official Q&A](https://wro-association.org/competition/questions-answers/) and [Surprise Rule](https://wro-association.org/wp-content/uploads/WRO-2026-Future-Engineers-Extra-Task.pdf) — Check parking and modified-sign wording.

## Hardware and assembly references

- [x] [Wiring diagram](../../img/diagrams/Wiring_Diagram_FE2026_v1.png) — Checked pins and component labels against the BOM. The team confirmed the power wiring on the robot.
- [ ] [CAD guide](../../CAD/README.md) — Check the [printing project](../../CAD/Car_v74.3mf), [Fusion model](../../CAD/Car_v74.f3z), [prepared print job](../../CAD/printing/Car_v74_0.4n_0.2mm_PLA_MK4IS_3h1m.bgcode) and [steering calibration workflow](../../CAD/README.md#workflow).
- [ ] [Ackermann calibration](../../CAD/ACKERMANN_KINEMATICS_DOCUMENTATION.md#turning-radius-comparison) and [measured parking contour](../../simulation/reports/RETURNED_651_656_PARKING_20261105.md#measured-geometry-and-implementation) — Check dimensions and turning-radius figures.
- [ ] Component references — [Motor](https://www.bastelgarage.ch/6v-530rpm-30-1-micro-gear-motor-with-encoder), [regulator](https://sph.ethz.ch/materials/product/10.2.4), [voltmeter](https://www.berrybase.ch/detail/0191f2bebc0d71c7afa5fd3d0c6cde11) and [ToF pinout](https://learn.adafruit.com/adafruit-vl53l4cx-time-of-flight-distance-sensor/pinouts).
- [ ] Manufacturer references — Check the [50:1](https://www.dfrobot.com/product-1432.html), [100:1](https://www.dfrobot.com/product-1433.html) and [30:1](https://www.dfrobot.com/product-1431.html) motor specifications, the [VL53L3CX](https://www.st.com/en/imaging-and-photonics-solutions/vl53l3cx.html) and [VL53L4CX](https://www.st.com/en/imaging-and-photonics-solutions/VL53L4CX) descriptions, the [ToF field-of-view data](https://www.st.com/resource/en/datasheet/vl53l4cx.pdf#page=26) and the [Arducam reference](https://docs.arducam.com/DVP-Camera-Module/Arduino-GIGA/Arduino-GIGA/Introduction/).
- [ ] [Rear ToF setup](../guides/REAR_TOF_M4_SETUP.md#parking-position-configuration) — Check the sensor offset, parking target and firmware settings against the README.

## Test evidence and reproduction

- [ ] [Parking exit study](../../simulation/PARKING_EXIT_PATH_SIMULATION.md), [physical parking runs](../../simulation/reports/RETURNED_651_656_PARKING_20261105.md#what-the-returned-runs-establish) and [direct parking handoff](../../simulation/reports/DIRECT_PARKING_HANDOFF_20261119.md).
- [ ] Modified-sign evidence — [Camera calibration result](../../simulation/reports/SURPRISE_RULE_MEASURED_VALIDATION_20261119.md#results-and-changes), [image measurements](../../simulation/reports/SURPRISE_RULE_MEASURED_VALIDATION_20261119.md#image-measurements) and [CW run verification](../../simulation/reports/CW_LOGS_3_6_REVIEW_20261119.md#verification).
- [ ] [Later-lap clearance study](../../simulation/reports/LAP23_CLEARANCE_REVIEW_20261120.md#validation) — Check that the README describes the clearance figures as model results.
- [ ] Simulation workflow — [Sign recognition](../../simulation/README.md#sign-recognition-check), [route clearance](../../simulation/README.md#route-clearance-check) and [parking exit](../../simulation/README.md#parking-exit-model).
- [ ] [USB logging procedure](../../README.md#reproduce-the-evidence), [run evidence index](../../simulation/evidence/parking_exit_diagnostics/README.md) and [run inspector guide](../../tools/robot-run-inspector/README.md) — Check the save triggers, LED signals and steps for finding and reviewing a log.

## Firmware references

Spot-check the claims and links in the README against these files.

- [ ] Build and control — [PlatformIO](../../platformio.ini), [configuration](../../include/config.h), [main loop](../../src/main.cpp), [modes](../../src/mode_manager.cpp) and [motor control](../../src/motor_control.cpp).
- [ ] Sensors and pose — [M7 sensors](../../src/sensors.cpp), [M4 rear ToF](../../src/m4/rear_tof_m4.cpp), [RPC receiver](../../src/rear_tof_rpc.cpp), [RPC protocol](../../include/rear_tof_rpc_protocol.h) and [position estimator](../../src/position_estimator.cpp).
- [ ] Camera and vision — [capture driver](../../lib/arducam_dvp_async/src/arducam_dvp.cpp), [GC2145 setup](../../lib/arducam_dvp_async/src/GC2145/gc2145.cpp), [vision](../../src/vision.cpp) and [sign height](../../include/obstacle_sign_height.h).
- [ ] Challenge logic and telemetry — [Open navigation](../../src/navigation_controller.cpp), [Obstacle start](../../src/obstacle.cpp), [routes](../../src/obstacle_path.cpp), [final parking](../../src/final_parking.cpp), [telemetry](../../src/run_telemetry.cpp) and [logger](../../src/logger.cpp).

## Final media and print checks

- [x] [Team, robot and assembly photos](../../img/) — Check the captions and every image in the rendered README.
- [x] [Open video](https://www.youtube.com/watch?v=koMCLdpg0-I) and [Obstacle video](https://www.youtube.com/watch?v=5dCMkFn-6TI) — Check previews, signed-out playback and at least 30 seconds of autonomous driving in each.
- [x] [Public GitHub repository](https://github.com/flawil-beavers/Future-Engineers_2026/tree/main) — Check the rendered README and its links on GitHub.
- [x] [README.pdf](../../README.pdf) — Reviewed all 22 rendered pages and decoded the repository and both video QR codes from the PDF.
- [x] Verify that the repository and both video QR codes scan correctly (team confirmation, November 21).

## Additional linked documents

These documents are reachable from the main README through its supporting guides and reports. Check that each document explains when its evidence, results or instructions apply.

### Workspace and validation guides

- [ ] [Workspace instructions](../../AGENTS.md)
- [ ] [Current engineering handoff](../../AGENT_DOCUMENTATION.md)
- [ ] [Connector and first-lap handoff](../validation/CONNECTOR_NEXT_TEST.md)
- [ ] [Obstacle Challenge validation checklist](../validation/OBSTACLE_CHALLENGE_TEST_PLAN.md)
- [ ] [Parking-exit pose and backlash plan](../validation/PARKING_POSE_IMPLEMENTATION_PLAN.md)
- [ ] [Full-run telemetry](../../simulation/RUN_TELEMETRY.md)
- [ ] [Parking-exit diagnostic logging](../../simulation/PARKING_EXIT_DIAGNOSTIC_LOGGING.md)
- [ ] [Parking-entry geometry tools](../../simulation/PARKING_ENTRY_GEOMETRY_TOOLS.md)
- [ ] [Video visualisation tools](../../simulation/VIDEO_VISUALIZATION.md)

### Camera, navigation and parking reports

- [ ] [Final parking design](../../CAD/PARKING_ENTRY_DESIGN.md)
- [ ] [Stationary sign-height measurements](../../simulation/reports/CAMERA_HEIGHT_CAPTURE_REVIEW_20261119.md)
- [ ] [Modified-sign detection and passing-side implementation](../../simulation/reports/SURPRISE_RULE_IMPLEMENTATION_20261111.md)
- [ ] [Camera observations during parking exit](../../simulation/reports/PARKING_EXIT_CAMERA_REVIEW_20261030.md)
- [ ] [USB save and memory review](../../simulation/reports/USB_SAVE_HEAP_REVIEW_20261119.md)
- [ ] [Firmware code review](../../simulation/reports/FULL_CODE_REVIEW_20261027.md)
- [ ] [Runtime and route fixes](../../simulation/reports/RUNTIME_AND_ROUTE_FIXES_20261027.md)
- [ ] [First-lap search at unresolved sign positions](../../simulation/reports/LAP1_ACTIVE_SEARCH_REVIEW_20261030.md)
- [ ] [First-lap camera angles in the last 20 archived files](../../simulation/reports/LAP1_APPROACH_LAST20_REVIEW_20261111.md)
- [ ] [Anticlockwise first-lap camera view](../../simulation/reports/LAP1_CHECKED_CCW_VIEW_20261111.md)
- [ ] [First-lap and parking recheck](../../simulation/reports/LAP1_PARKING_RECHECK_20261111.md)
- [ ] [First-lap speed change](../../simulation/reports/LAP1_TEMPO_20261119.md)
- [ ] [Transition from lap 1 to lap 2](../../simulation/reports/LAP_HANDOFF_20261102.md)
- [ ] [Later-lap route efficiency](../../simulation/reports/LAP23_EFFICIENCY_REVIEW_20261120.md)
- [ ] [Direct connections and handoffs on later laps](../../simulation/reports/LATER_LAP_DIRECT_LANES_20261102.md)
- [ ] [Shorter inner routes on laps 2 and 3](../../simulation/reports/LATER_LAP_INNER_ROUTE_REVIEW_20261027.md)
- [ ] [Final parking in both directions](../../simulation/reports/FINAL_PARKING_CAPTURE_REVIEW_20261027.md)
- [ ] [Parking approach from recorded arrival poses](../../simulation/reports/PARKING_ARRIVAL_AUDIT_20261028.md)
- [ ] [Parking replanning from the measured pose](../../simulation/reports/FINAL_PARKING_POSE_REPLAN_REVIEW_20261030.md)
- [ ] [Final parking in tight gaps](../../simulation/reports/FINAL_PARKING_TIGHT_COMPLETION_REVIEW_20261030.md)
- [ ] [Parking entry efficiency](../../simulation/reports/EFFICIENT_PARKING_ENTRY_20261101.md)
- [ ] [Parking duration and space requirements](../../simulation/reports/PARKING_DURATION_AUDIT_20261102.md)
- [ ] [Steering settling and standstill checks](../../simulation/reports/PARKING_SETTLE_REUSE_20261102.md)
- [ ] [Rearward entry and parallel stop](../../simulation/reports/PARKING_REARWARD_PARALLEL_STOP_20261105.md)
- [ ] [Anticlockwise run 2 and its parking approach](../../simulation/reports/PARKING_APPROACH_LOG2_REVIEW_20261119.md)

### Run reviews and test procedures

- [ ] [Runs 531–540](../../simulation/reports/RETURNED_RUNS_531_540_REVIEW_20261027.md)
- [ ] [Clockwise runs 541–543 and green sign association](../../simulation/reports/CW_CORNER_541_543_FIX_20261027.md)
- [ ] [Anticlockwise runs 544–546 and USB saving](../../simulation/reports/CCW_544_546_REVIEW_20261027.md)
- [ ] [Runs 547–555](../../simulation/reports/CW_CCW_547_555_REVIEW_20261027.md)
- [ ] [Green sign association in runs 556–562](../../simulation/reports/GREEN_556_562_REVIEW_20261027.md)
- [ ] [Runs 563–569 and parking](../../simulation/reports/CW_CCW_563_569_REVIEW_20261027.md)
- [ ] [Corner observation hold in run 565](../../simulation/reports/CORNER_HOLD_565_FIX_20261027.md)
- [ ] [Runs 570–575 and parking registration](../../simulation/reports/CW_CCW_570_575_REVIEW_20261029.md)
- [ ] [False sign confirmation in run 576](../../simulation/reports/CW_576_START_REGRESSION_20261029.md)
- [ ] [Anticlockwise runs 577–581](../../simulation/reports/CCW_577_581_REVIEW_20261029.md)
- [ ] [Runs 582–593 and starting-straight sign detection](../../simulation/reports/RETURNED_RUNS_582_593_REVIEW_20261030.md)
- [ ] [Runs 594–601](../../simulation/reports/RETURNED_594_601_REVIEW_20261031.md)
- [ ] [Run 603 and isolated parking practice](../../simulation/reports/RETURNED_603_AND_PARKING_PRACTICE_20261031.md)
- [ ] [Runs 604–606 and rear-space parking](../../simulation/reports/RETURNED_604_606_REVIEW_20261031.md)
- [ ] [Runs 607–609 and return to full runs](../../simulation/reports/RETURNED_607_609_FULL_RUN_HANDOFF_20261031.md)
- [ ] [Run 610 and the lap-boundary hold](../../simulation/reports/RETURNED_610_LAP_BOUNDARY_20261031.md)
- [ ] [Runs 611–615 and final parking](../../simulation/reports/RETURNED_611_615_REVIEW_20261031.md)
- [ ] [Run 616 and parking clearance](../../simulation/reports/RETURNED_616_CLEARANCE_REVIEW_20261031.md)
- [ ] [Runs 617–619 and filmed-run capture](../../simulation/reports/RETURNED_617_619_REVIEW_20261031.md)
- [ ] [Runs 620–621 and the first clockwise corner](../../simulation/reports/RETURNED_620_621_REVIEW_20261031.md)
- [ ] [Return to normal logging after runs 622–623](../../simulation/reports/NORMAL_DRIVING_HANDOFF_20261031.md)
- [ ] [Runs 624–627](../../simulation/reports/RETURNED_624_627_REVIEW_20261031.md)
- [ ] [Run 628 and parking position](../../simulation/reports/RETURNED_628_BALANCED_PARKING_20261031.md)
- [ ] [Runs 629–634 and parking motion](../../simulation/reports/RETURNED_629_634_PARKING_MOTION_20261101.md)
- [ ] [Runs 645–650 and later-lap speed](../../simulation/reports/RETURNED_645_650_EFFICIENCY_20261105.md)
- [ ] [Isolated reverse-parking practice](../guides/PARKING_ENTRY_PRACTICE.md)
- [ ] [Single-run position capture](../../simulation/reports/POSITION_CAPTURE_ONCE_20261031.md)

### Evidence indexes

- [ ] [Stationary camera evidence](../../simulation/evidence/camera_diagnostics/README.md)
- [ ] [Parking-exit footprint photographs](../../simulation/evidence/parking_exit/README.md)
- [ ] [19 November USB intake](../../simulation/evidence/log_intake/20261119_stick/README.md)
- [ ] [19 November USB save check](../../simulation/evidence/log_intake/20261119_save_check/README.md)
- [ ] [19 November stationary USB save verification](../../simulation/evidence/log_intake/20261119_stationary_usb_save/README.md)

### Engineering history

- [ ] [Engineering history index](../history/README.md)
- [ ] [Archived engineering handoff](../history/AGENT_DOCUMENTATION.md)
- [ ] [Archived handoff before CW runs 3–6](../history/AGENT_DOCUMENTATION_20261119_BEFORE_CW_BATCH.md)
- [ ] [Archived connector test plan](../history/CONNECTOR_NEXT_TEST.md)
- [ ] [Archived sign-height image intake](../history/HEIGHT_CAPTURE_INTAKE_20261119.md)
- [ ] [Archived Obstacle Challenge test plan](../history/OBSTACLE_CHALLENGE_TEST_PLAN.md)
- [ ] [Archived WRO rules notes](../history/WRO_2026_RULES.md)
