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
