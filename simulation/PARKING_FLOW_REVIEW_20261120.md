# Final-lap parking flow review, 2026-11-20

The handoff still starts at the first checked pose after the full vehicle clears the final corner. CCW can continue rolling when the prepared route, pose, speed and gyro agree. CW brakes to reverse. The planner uses the learned line and checks earlier outward shifts when the vehicle can clear the pillars. The subsequent rail scan establishes a fresh reference before the tight bay entry.

The registered approach now allows up to 175 mm/s after the existing two-frame wall registration, with a steering command of at most 20 degrees in either direction and a clear 36 mm swept preview. Otherwise the standard approach profile applies. The faster preview covers about 206 ms at 175 mm/s, compared with 200 ms for 30 mm at 150 mm/s. Both profiles taper toward 60 mm/s near the scan reversal. The scan and entry use their established speeds and geometry.

After the scan, the move to the entry pose now counts the actual time since the servo was commanded straight. It still requires 400 ms at that angle. It can proceed immediately if the servo already settled while braking and checking the stationary wall reference. A newly written command still receives the full wait. Trying the same shortcut before the rail scan changed ToF sampling phase and introduced three modeled entry overlaps. Moving wall registration into the outward bend also caused additional alignment failures. Both experiments were discarded.

The six original logs on the stick were recopied and SHA-256 verified immediately. They are the previous batch. The [verification manifest](evidence/log_intake/20261119_batch_3_6/parking_flow_source_verified.json) and [parking evidence index](evidence/parking_exit_diagnostics/README.md) preserve their identity.

## Model comparison

The complete sequence matrix used the same 396 cases before and after, with both directions, 30/100 ms sensor frames, lateral offsets through ±40 mm and longitudinal offsets through ±180 mm. The host loop now services the real steering write helper each tick to exercise servo command age. The preceding stand was path `20261120_lap23_efficiency` and parking `20261119_direct_handoff`. Current parking identity is `20261120_approach_flow`.

| Outcome | Before | After |
| --- | ---: | ---: |
| Completed with modeled clearance and containment | 374 | 377 |
| Safe rejection of perturbed pose | 7 | 7 |
| Other failures, including modeled overlap | 15 | 12 |

All 374 previously successful cases still complete. Mean improvement is 0.444 s and median is 0.38 s, with a range of 0–2.82 s among those paired cases. With zero injected position bias, paired successful means excluding the deliberate 1.5 s gyro pause are 30.06 → 29.68 s CW and 33.23 → 32.72 s CCW. These timings assume immediate motor command tracking, ideal ToF and the modeled wheel geometry. They cover the parking controller from its initial handoff pose and exclude the outer CW dispatcher brake. They do not measure real actuation time or prove a globally shortest route.

Ten remaining cases with pose perturbations and 100 ms sampling predict contact during tight CCW entry. Two other baseline failures persist. The result is an approach and wait improvement. Physical clearance and time still require ordinary CW and CCW runs. Preserve the full original logs first and compare the approach, wall registration, capture and entry timestamps against the observed motion.

The focused verification covers 3 complete sequences, 5 reference registration checks, 24 learned-map handoffs including 12 CCW rolling handoffs, 3 log2 acquisitions, 102 brake cases, 24 measured-gap entries, and the log5/log6 checked rescan. The neutral steering test confirms the 399/400 ms boundary in both directions.

Final IDE-managed M7 build succeeded. RAM 452176/523624 B, flash 601880/786432 B, firmware.bin 605432 B, SHA-256 `a9f2df37fbf5cbddc94b11f8160ffece78932d059556ab48f79f2e1df99a8c53`.
