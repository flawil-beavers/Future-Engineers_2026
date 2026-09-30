# Parking-exit diagnostic evidence

This directory stores the immutable robot logs used to measure parking-exit
pose error, braking, direction-change error and servo neutral. Generated reports
belong in ignored `local_workspace/parking-exit-analysis/`, not here.

## Batch ingestion

For every returned test batch:

1. Copy each complete source log here without editing its telemetry.
2. Name it `YYYYMMDD_log_NNN_cw.txt` or `YYYYMMDD_log_NNN_ccw.txt` using the test
   date, original log number and exit direction.
3. If only part is available, copy the exact excerpt as
   `YYYYMMDD_log_NNN_cw_excerpt.txt` (or `ccw`) and mark it incomplete below.
4. Calculate SHA-256 from the tracked copy and add one row to the table below.
5. Run `simulation/analyze_parking_exit_pose.py` against the tracked file.
6. Add concise findings, limitations and the next test to
   `AGENT_DOCUMENTATION.md`, referring to this repository-relative path and hash.
7. Before finishing, use `git status` to verify the log, this README and
   `AGENT_DOCUMENTATION.md` are tracked changes. Never rely on the USB copy or an
   ignored `local_workspace/` file as synchronized evidence.

Do not rename an existing evidence file, normalize line endings, remove unrelated
telemetry or replace it with analyzer output. Add a new file when new evidence
arrives. Full raw logs are preferred; excerpts are only a fallback when the
complete source cannot be obtained.

## Received runs

| Evidence file | SHA-256 | Firmware/build | Direction | Complete | Physical report | Analysis/limitations |
| --- | --- | --- | --- | --- | --- | --- |
| `20260929_log_417_cw.txt` | `95ec8d1c7af3e74b4177b79d98f1d897968ec622baec19aa3006a313a4992940` | M7 retry/seat-projection revision; parking header Sep_28_2026_22_58_57, installed binary not read back | CW | Yes, 153,731 bytes | First of three new user-reported CW runs: pulled inward after apparent extra green, then stopped | Parking exit and connector completed. Empty S1/0 first marked clear, then green seat7 confirmed by fallback; empty S1/1 red seat9 confirmed. Neither matches the reported S1 layout (green only at last outer seat11). S1/2 remained unresolved; 800 ms perception hold expired and stopped. Later manual disable is secondary. |
| `20260929_log_418_cw.txt` | `04c08a2326e6e3b89d4dd7411a073d574b732726b4b7f041b6337d671ba2235c` | same M7 revision | CW | Yes, 78,166 bytes | Second reported run: first green passed correctly, first red wrong side, second red correctly, last green wrong side, start green passed | Start green seat0 and S1 green seat11 confirmed. S2 middle red repeatedly projected 162-254 mm from seat14 and rejected; S3 first red mapped to seat19, not reported inner seat18; last green confirmed as seat22, not reported outer seat23. Onboard lap 1 completed; physical route was incorrect. |
| `20260929_log_419_cw.txt` | `959932dec130dcf1b32b90b1d090cd26b20400615513adb4efc4efcaa948ebbd` | same M7 revision | CW | Yes, 100,978 bytes | Third reported run: green first section passed, skewed before first red, struck second red, missed subsequent green, narrow start-green pass | S1 last station was confirmed RED seat11 though physically GREEN was reported there; S2 red was assigned seat15 rather than inner seat14, S3 first red missed, S3 last green was confirmed RED seat23, start green assigned seat2. Onboard lap 1 completed despite reported contact. Log cannot independently prove which red blob belonged to which physical pillar. |
| `20260929_log_413_cw.txt` | `8214a7419be28995cf45a1085d82cb379c7b462ae298fde925bc6f6b6e1dc288` | M7 connector diagnostics, parking header Sep_28_2026_22_58_57; installed binary not read back | CW | Yes, 22,119 bytes | Individual physical outcome not assigned | Parking localization heading abort at 8.4 deg versus 8.0 deg limit; no first-lap result. |
| `20260929_log_414_cw.txt` | `2a960c1e90f62b3b28b7e78f9bc9cb01a0063c757ca5c49761b206bc281c2695` | same M7 parking header | CW | Yes, 5,486 bytes | Individual physical outcome not assigned | Rear parking positioning failed stationary verification; motor locked. |
| `20260929_log_415_cw.txt` | `b29b2be0f43ef7fc9e22170e59ad4c7441181cc2ae1dc04ed7f9c57a47444bc5` | M7 pre-retry connector logic, same parking header | CW | Yes, 49,737 bytes | Consistent with earlier reported autonomous stop after start, but run ID unconfirmed | Exit/scout completed; all six connector rollouts rejected by 500 mm maximum travel, endpoint error 112-113 mm; motor locked. No red-pillar passage. |
| `20260929_log_416_cw.txt` | `1d07c854088e830d49ccb7c2e848ca6450dd527aca510268c6e311cb20a5a78f` | uploaded M7 retry/seat-projection revision, binary SHA-256 `eb7270f834d767e8f1ba024bf11019396beb37e5ec71cb00a3f7b65c08bc7f66`; parking header unchanged | CW | Yes, 79,443 bytes | Matches reported wrong first red pass and later green/start collision; exact run number unconfirmed | Connector alternative at phase 300 passed. S2 red middle projected about 199-233 mm from seat14 and rejected; S3 first red projected onto seat19, opposite expected seat18; S3 last green marked clear despite visible green blobs. Ends with stall 9.3 mm/1 s after last corner. One of two reported full runs is not identifiable in this batch. |
| `20260928_log_409_cw.txt` | `71ba0b97535e75947216f84da8a5d7fed9e1a4eb6c35db809539ce927a375680` | M7 diagnostic build Sep_28_2026_21_57_15; installed binary not read back | CW | Yes, 74,527 bytes | User reports both red pillars passed on the wrong side and green at start grazed in legal new CW layout | Exit and connector completed; expected S2 inner-middle red seat 14 never confirmed despite visible red blobs; S3 inner red seat 18 was mapped to outer seat 19; expected S3 outer green seat 23 was marked clear. Only true green seats 0/11 and wrong red 19 confirmed. Lap-1 counter completed but physical run failed. |
| `20260928_log_410_cw.txt` | `5ad0490511d5d104d614141f30cc320d8a12594ae74f045877e1caa910193a8e` | same M7 diagnostic build | CW | Yes, 66,019 bytes | User reports exit and observation, then no further movement | Start green seat 0 confirmed; after 85 mm scout/retrace, connector rollout rejected at all six lookaheads (150..25 mm), `before_pillar=50 mm`; motor locked by preflight. Later generic manual-disable label does not identify the original stop cause. |
| `20260928_log_411_cw.txt` | `9cbed8e595361a95ee5e07d6ecc21dc99d26f24a9c4516e8db142aa324cca9a3` | same M7 diagnostic build | CW | Yes, 46,647 bytes | Same physical stop after observation | Start green seat 0 confirmed; connector rollout rejected at all six lookaheads, `before_pillar=100 mm`; motor locked by preflight. Later generic manual-disable label does not identify the original stop cause. |
| `20260928_log_412_cw.txt` | `d1705778151079b4b7c10cf2a63a48fa12ac3a2aa1dbec73f816385e5d6d8d81` | same M7 diagnostic build | CW | Yes, 101,674 bytes | User reports after first curve a left deviation around empty positions, then autonomous standstill; confirms S1/0 physically empty | Start green S0/0 was marked clear during scout. Green fallback falsely injected seat 7 at empty S1/0 from three frames (range 187 mm, snap 109 mm), after that station had been marked clear. S1/1 perception hold expired at 300 mm and blocked drive; later generic manual-disable label. One diagnostic event-order error; no overflow/truncation. |
| `20260928_log_406_cw.txt` | `f0e911b1bf2b7a7d69c09cd69404b6643fd76482de18a13ba8f808d2741cf2c6` | M7 diagnostic build Sep_28_2026_21_57_15; installed binary not read back | CW | Yes, 23,940 bytes | User says robot exited, surveyed and then stopped; explicitly denies switching it off | Stored log ends during exit segment 5 at about 77/150 mm, before localization, camera scan or lap. `Manual disable` is a generic pause label, not proof of human action. No preceding stall/overload message. Physical/log chronology remains unexplained. |
| `20260928_log_407_cw.txt` | `1206834f8464e137b45b65916f072a96fea3256de4c609e78dd107dc23a9c3e9` | same M7 diagnostic build | CW | Yes, 84,522 bytes | User reports complete first lap without any contact | Exit 5/5, localization +21.7/-2.1 mm, one 170 mm corner-view reverse at S1/0 (172.0 mm measured); return pose about 0.3 mm/0.2 deg from origin; five expected pillar seats confirmed, contradictory opposite-seat projections ignored, lap 1 complete and stop gate fired. |
| `20260928_log_408_cw.txt` | `4586c4e9ce0656208de91e312d593daa2e6c1e7284dce10412c209aa97ed50a2` | same M7 diagnostic build | CW | Yes, 91,936 bytes | User reports complete first lap without any contact | Exit 5/5, localization +21.1/-5.9 mm, 170 mm corner-view reverses at S1/0 and S2/0 (170.1/172.1 mm measured); return poses within about 0.9 mm/0.4 deg of origins; five expected pillar seats confirmed, contradictory red projection ignored, lap 1 complete and stop gate fired. |
| `20260928_log_399_cw.txt` | `aad53186d018236f5838172d5f7cb7bc02aea8d6a3f7806aa0612fcaf724f3da` | uploaded M7 `f87224f6a6458faa724a8f815045b3a3eaf39911228f47be1e8a32a3719c2081`; Sep_28_2026_21_09_08 | CW | Yes, 34,701 bytes | Laptop cable interfered | Original of the earlier `000` excerpt; exit localization X correction +55.8 mm exceeds 50 mm gate. Do not count as obstacle run. |
| `20260928_log_400_cw.txt` | `2dd5eb7c2eda30ccaf828a2652171a99fd280cedaa3d4fbdd1710b170508a66f` | same uploaded M7 | CW | Yes, 90,029 bytes | Valid photographed-layout attempt; user said it looked good, no individual contact report | Exit and lap 1 completed. Two 170 mm corner-view reverses; green S0/0 was never confirmed because red S0/1 triggered official empty-end inference. Invalid layout means lap completion does not prove green avoidance. |
| `20260928_log_401_cw.txt` | `b8ee61a55b91286417427237509924d5bf27fbaf7723ba6dfa5e1413326bd10d` | same uploaded M7 | CW | Yes, 28,591 bytes | Parking barriers misplaced; exclude from performance evaluation | Original preserved; parking-exit diagnostic completion missing. |
| `20260928_log_402_cw.txt` | `227534df9ace4eae5e90cf6ccda54fb741304fc194f64bfad2b90c6c6db62bd8` | same uploaded M7 | CW | Yes, 29,398 bytes | Parking barriers misplaced; exclude from performance evaluation | Original preserved; parking-exit diagnostic completion missing. |
| `20260928_log_403_cw.txt` | `f9fac11a03a16144f95b295e1a3205b67ce09c50bed84b9ba8551454fcde40d1` | same uploaded M7 | CW | Yes, 31,193 bytes | Parking barriers misplaced; exclude from performance evaluation | Original preserved; parking-exit diagnostic completion missing. |
| `20260928_log_404_cw.txt` | `1efd5b5170ea96cd773ba92d3b155637986366f2ad627670fdd9f2b65499cb57` | same uploaded M7 | CW | Yes, 78,865 bytes | Passed green, drove between green and red, then into wall; manually stopped | Exit valid, no corner-view reverse. Green S0/0 confirmed late (snap 131/140 mm); contradictory red S0/1 LEFT injected after red RIGHT at parking entry. Stall 5.8 mm/1 s. Collision location is user report; log does not measure contact point. |
| `20260928_log_405_cw.txt` | `d6330f305f1809afcc4954ac66355c2649564de053a749e8f4f84b538a3ac7fd` | same uploaded M7 | CW | Yes, 96,243 bytes | Passed too close left of green near lap end and stuck against it | Exit valid, two corner-view reverses. Green S0/0 never confirmed after official inference, opposite red S0/1 LEFT injected after red RIGHT, then stall 8.4 mm/1 s at 257 mm/s target. |
| `20260928_log_000_cw_excerpt.txt` | `7195c639a06c17f4226d3317d0263bd557d5515c320114a882d828b991a2b478` | uploaded M7 binary `f87224f6a6458faa724a8f815045b3a3eaf39911228f47be1e8a32a3719c2081`; diagnostic build Sep_28_2026_21_09_08 | CW | No; exact laptop serial capture, now matched to complete original log 399 | User reports laptop cable interfered with robot; obstacle contact not established | Exit 5/5, reverse localization transition at285.9 mm; computed X correction+55.8 mm exceeded 50 mm gate, Y correction-13.3 mm accepted, motor locked before camera entry/pillars. Cable makes this attempt unsuitable to evaluate route or colour. |
| `20260927_log_398_cw.txt` | `5ed231efb66a9996999717cecd6d3234b7b8f47624b2f2cac71e918c2f855628` | corner/connector Sep26_2026_18:33:21; prepared M7 `c711bf8d70ba209b9c246d5b17a112e3cba5b5fb8b13f4f1bbd4a700d8cb6f87`, installed binary not read back | CW | Yes,140,882 bytes; two sessions in one unchanged source file | Good exit, first curve reverse/forward then stop; setup unchanged, S1/0 and S1/1 empty, red only S1/2; contact absence not explicitly reported for this batch | Both straight peeks/retraces pass but inner seat remains unresolved, rejected red area164..180 blocks CLEAR; derived session slices74/77 diagnostic samples, no integrity errors; aggregate151 exceeds per-run analyzer cap, not robot overflow |
| `20260926_log_396_cw.txt` | `8f8b731af089e290d10e18c67782c0994b0ae4a7046f1a33fd7ae3039eda6eaa` | corner/connector Sep26_2026_18:15:21; prepared M7 `b24c1fba47d5f88e903d56973ad59c75f00c336fdc59f5edad383d9c72256e91`, installed binary not read back | CW | Yes,89,874 bytes | User reports good corner reverse, red bypass, second reverse, left bypass of middle green then stop; not individually assigned to run; no explicit contact report | Both corner peeks resolve/retrace; stopS2/2 at300 mm; inner seat not stored clear despite confirmed middle green; exit78 samples,no integrity errors |
| `20260926_log_397_cw.txt` | `366515c1fc55ea56e0bf7278ace7319751b252251dc258e893d27d895008ae0c` | same corner/connector build and prepared M7; installed binary not read back | CW | Yes,93,794 bytes | Same batch-level report, individual run outcome/contact not confirmed | Both peeks resolve/retrace; sameS2/2 hold after confirmed middle green; exit87 samples,one diagnostic ordering inversion,no overflow/truncation |
| `20260926_log_394_cw.txt` | `ef104e7fb872490a34f0b791352e2ea91b8c674e7efad13ceaf42d29fe358ae1` | corner/connector Sep 26 2026_18:03:25; prepared M7 hash `ea01a855cdc566f2efd2dd8bc9ccb505ff2fc2298a6594fc57d802eba3cb3fcc`, installed binary not read back | CW | Yes, 72,194 bytes | User reports reverse began, about 2 cm then stop; contact not reported | Connector completes60.0 mm/0.3 deg; reverse170 mm preflight passes, locks heading/cross-track; no observation or return phase |
| `20260926_log_395_cw.txt` | `b8df76c179dbe85f9445b932398536f32c4e11bbfbef0b0e3c04f2bd4a249240` | corner/connector Sep 26 2026_18:03:25; same prepared build, installed binary not read back | CW | Yes, 82,219 bytes | Same reported about2 cm reverse then stop; contact not reported | Connector completes59.9 mm/0.5 deg; reverse170 mm preflight passes, locks heading/cross-track; no observation or return phase |
| `20260926_log_392_cw.txt` | `b2ced7074114324a068a7085abf9437a803ab7d2b24afcaf6b123a9ff4801c6e` | connector Sep 26 2026_17:09:24; discovery trace v1 | CW | Yes, 82,608 bytes | User reports another stop; no new contact statement | Connector completes 60.0 mm / 1.4 deg; inner corner seat never visible in recorded approach, too near at hold; outer seat stored clear |
| `20260926_log_393_cw.txt` | `48571b315690476a4676352ab00b2d5183183590f9f03abc568c568aebcd73ac` | connector Sep 26 2026_17:09:24; discovery trace v1 | CW | Yes, 80,455 bytes | User reports another stop; no new contact statement | Connector completes 60.0 mm / 2.3 deg; same inner visibility failure; rejected green overlap also blocks outer clear |
| `20260926_log_390_cw.txt` | `1e500f44c20a79bfac6b0a189eb00ed7d1bf90c14223b39a4ac06023ad48eb56` | exit Sep_26_2026_16_38_05; connector Sep 26 2026_16:38:04 | CW | Yes, 65,187 bytes | Exit and first straight contact-free; began first curve then stopped | Connector complete at 59.8 mm / 0.7 deg; perception hold S1/0 at 335 mm expires after configured 800 ms |
| `20260926_log_391_cw.txt` | `c879866174b624fe080a048dd92af9c723027ccab7d7ddb854c13604d89469b9` | exit Sep_26_2026_16_38_05; connector Sep 26 2026_16:38:04 | CW | Yes, 57,647 bytes | Exit and first straight contact-free; began first curve then stopped | Connector complete at 59.5 mm / 0.0 deg; same S1/0 perception hold; no complete lap |
| `20260926_log_383_cw.txt` | `541ac039b0ccf9d29b461475621f15b3a65ecef7baeb30392542c07bc5a2e8ca` | schema 2, Sep_26_2026_12_06_07 | CW (`turn=-1`) | Yes, 56,689 bytes | Similar to both other tests; no obstacle contact reported | 87 samples; unparking complete; scout return 0.4 mm / 3.1 deg estimated; connector stopped at 16/17, steering -42.0 deg rounded |
| `20260926_log_384_cw.txt` | `f03911028f24c4300c5536ecb68f78fafe8f92c3c04cf40d5e82e1cb71463179` | schema 2, Sep_26_2026_12_06_07 | CW (`turn=-1`) | Yes, 47,813 bytes | Similar to both other tests; no obstacle contact reported | 82 samples; unparking complete; scout return 2.2 mm / 1.0 deg estimated; connector stopped at 16/17, steering -42.2 deg |
| `20260926_log_385_cw.txt` | `29332777168cf4fb004a40173adf4a9dc4476d8127dcd8d18ca28a2dcb8f3f86` | schema 2, Sep_26_2026_12_06_07 | CW (`turn=-1`) | Yes, 50,366 bytes | Similar to both other tests; no obstacle contact reported | 87 samples; unparking complete; scout return 3.9 mm / 2.2 deg estimated; connector stopped at 16/17, steering -42.1 deg |
| `20260926_log_386_cw.txt` | `11d5d56337b49fdc0ad57fedce7b8a54f33b95ce301a9377e1b86afcebcb661d` | exit schema 2; connector v1 Sep 26 2026_16:00:28 | CW | Yes, 64,877 bytes | First of two CW runs; no further motion reported; user confirms no obstacle contact; stall not separately reported | 8 tail records replay; rejection at 35.39 mm / 23.80 deg endpoint error and -42.026 deg steering |
| `20260926_log_387_cw.txt` | `a0b579346f0a1a50c03d6bc5435ffca3380545970db64f46ca78577e9585afa4` | exit schema 2; connector v1 Sep 26 2026_16:00:28 | CW | Yes, 51,671 bytes | Second CW run; user confirms no obstacle contact; stall not separately reported | 8 tail records replay; rejection at 30.92 mm / 22.22 deg endpoint error and -42.165 deg steering |
| `20260926_log_388_ccw.txt` | `dd3831acb6c0e5d8683b60948ee97b5308387a46a577d39b681206eacb33c5ec` | exit schema 2 Sep_26_2026_12_06_07; connector build not emitted after failed preflight | CCW | Yes, 51,102 bytes | First CCW run; user confirms no obstacle contact; stall not separately reported | Exit/scout complete; connector preflight tracking fails at 43.2 deg; no connector motion/tail |
| `20260926_log_389_ccw.txt` | `8a40df45cdfe819f6ff4f65fdd6e62f3cc42340c2942e047f5cf9e557220c74e` | exit schema 2 Sep_26_2026_12_06_07; connector build not emitted after failed preflight | CCW | Yes, 49,228 bytes | Second CCW run; user confirms no obstacle contact; stall not separately reported | Exit/scout complete; connector preflight fails at 42.9 deg; one diagnostic timestamp inversion of 1 ms |

### Third batch on 2026-09-26

Complete originals match source/copy SHA-256. User reports both CW and no contact
through the first straight and curve entry. Logs show automatic perception hold,
later manual disable; not a complete first-lap stop. Headers match the corrected
tangent build documented with SHA-256
`55a9dc3621f1a17e85bf991b804ad412747a97fb17485bf04cc02accd55c68f7`;
installed binary not independently read back. No overflow/truncation, duplicate
ToF or diagnostic timestamp ordering errors. Exit samples 80/78; scout estimated
retrace 5.4/4.4 mm and 2.2/0.9 degrees. Raw red areas 248--268 during hold remain
below the 300 acquisition gate; rejected broad green regions also occur. No
coverage-frame pose, per-seat visibility, clear-block reason or observation age
is logged, so the unresolved station's exact cause is not identifiable. Preserve
thresholds and acquire bounded discovery trace before changing motion/recognition.
### Second batch on 2026-09-26

All four original source/copy SHA-256 values match. Direction agrees with the
user's ordering and diagnostic turn signs. Logs end with manual disable after
automatic hold; no connector completion is present. No logger overflow or
diagnostic truncation. Only CW successful preflight emits connector-build identity,
consistent with the prepared binary hash documented in the agent handoff; no
installed binary was read back. The user supplied a setup photo in conversation,
not yet a repository image artifact; dimensions and exact seat coordinates are
not established by the perspective view. Do not infer no contact from logs.

CW replay agrees with rounded recorded targets/steering. Endpoint heading,
not the 60 mm distance gate, blocks handoff at rejection. Continued-route
counterfactual requires +18.58/+21.07 deg versus actual -42.026/-42.165 deg,
reversing turn direction at the measured poses. All 432 assumed perturbation
checks pass the candidate steering/forward guard, but neither heading convergence
nor swept collision safety is established. CCW preflight lacks geometry output,
so it cannot be replayed from these originals. Log 389 has event t=12489 then
sample t=12488; original order is preserved and analyzer flags it. Servo-neutral
fits outside the sampled command range are now rejected as unidentifiable.

### 2026-09-26 batch limitations

All three complete originals were copied unchanged and SHA-256 verified against
the source archive. The portable date is the session/build date: removable-media
timestamps incorrectly showed 2097/2098. Assignment of test 1/2/3 to ascending
log numbers is inferred, not individually confirmed by the user.

The header matches the prepared diagnostic build, whose local binary SHA-256 is
`66655013d709c983b12642488fc0a7852076ed5613b14a1011824f7e0a6582ff`;
the installed robot binary hash was not independently read back. All logs have
automatic connector rejection followed later by manual disable. No connector
completion or complete lap is evidenced. The user confirmed no obstacle contact;
stall and manual-stop timing were not separately reported.

No logger overflow, diagnostic truncation, duplicate ToF snapshots or diagnostic
ordering errors occurred. Diagnostics finish after localization, before the scout.
Main-loop timing and exact scout observation duration are not recorded. Pose
errors/clearances are onboard estimates, not external measurements. Rear-ToF age
is M7 receipt age; all 15 reversal-loss estimates are unobservable. Servo-neutral
fits vary by run/subgroup and do not justify changing the steering centre.

### Cross-batch unparking reanalysis on 2026-09-27

The analyzer now expands multiple diagnostic sessions directly from one unchanged
source, so both sessions in log 398 are reproducible without unsynchronized slice
files. All 16 sources produce 17 sessions. Nineteen rear-positioning moves have
a fixed rear-marker comparison: encoder-minus-ToF median +1.23 mm, spread 2.53
mm, maximum absolute 5.50 mm, all within conservative ToF/settling uncertainty.
Only one is a reverse move, so drivetrain backlash remains unidentified.

The combined exploratory servo fit is 80.62 degrees. Increasing/decreasing
steering-approach groups give 81.51/80.06 degrees (midpoint 80.78); retain centre
80 because this span can include linkage hysteresis and controller/sensor lag.
Edge-localization X corrections are mirrored and repeatable: CW +23.3 mm mean
(3.8 mm spread, 15 runs), CCW -22.8 mm mean (0.8 mm spread, two runs). This is
evidence of a systematic model/reference offset, not independent absolute pose.
Generated tables and plots remain in `local_workspace/parking-exit-analysis-all/`.

### Reanalysis including 2026-09-28/29 runs (2026-09-30)

The 37 complete source files expand to 38 sessions; the `000` excerpt repeats
part of complete log 399 and is not counted. Thirty-one sessions have an
untruncated `unparking_complete` record. Combined motion summaries now exclude
the seven aborted/incomplete sessions, while retaining them in per-run output.
Completion does not establish a valid physical setup or obstacle result; use the
run metadata above for those distinctions.

Among completed exits, 32 settled rear-marker moves have encoder-minus-ToF
median +0.45 mm, spread 3.57 mm and maximum absolute disagreement 13.27 mm,
all within conservative sensor/settling uncertainty. Only one reverses. The
interrupted log 406 has reverse disagreement -7.39 mm beyond 5.84 mm uncertainty;
log 414 fails stationary rear-ToF verification with range rising after the
encoder stops. These isolated failures do not quantify drivetrain backlash.

Completed-run exploratory servo centre is 80.97 degrees; increasing/decreasing
approach estimates are 81.67/80.08 degrees. Keep configured centre 80 pending a
controlled physical comparison. Fourteen new completed CW exits have mean edge
X correction +21.7 mm (4.7 mm spread); all 29 completed CW exits average +22.5
mm (4.3 mm spread). The only CCW evidence remains two older exits averaging
-22.8 mm. This correction compares two onboard pose references, not ground truth.
The regenerated report is `local_workspace/parking-exit-analysis-all/parking_exit_analysis.md`.
