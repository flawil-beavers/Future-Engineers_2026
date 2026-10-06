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
5. Run `python simulation/analyze_parking_exit_batch.py` from the repository root
   to refresh all archived pose plots and reports under ignored
   `local_workspace/parking-exit-analysis-all/`. The command selects complete
   filenames and skips excerpts and byte-identical originals; it does not copy
   logs from USB or update this metadata table.
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

### Revised-exit reanalysis (2026-10-06)

All 75 archived-file metadata hashes (72 complete sources plus three excerpts)
match their repository copies after correcting a transcription error in log 452's
hash. Batch analysis expands 73 sessions; 62 completed,
untruncated exits contribute to the historical aggregate. The batch index now
links separate diagnostic-build/configuration reports under `by-build/`.

The archived `Oct__5_2026_21_30_53` group (425-454, recorded 90/155 mm targets)
has 28 completed CW exits of 30 runs. Its 30 observable rear-marker moves have
encoder-minus-ToF median +0.47 mm, spread 2.92 mm and maximum absolute 6.16 mm,
all within conservative uncertainty; only one reverses. Exploratory centre is
81.11 degrees, with increasing/decreasing approach candidates 81.53/80.16;
keep centre 80. Mean edge correction dx/dy is +22.6/-5.6 mm, dx spread 5.6.
These are onboard-reference comparisons, not physical clearance or backlash.
Completed final arcs span 139.2-151.4 mm; failed log 441 aligned at 134.1 mm
before its reverse stall and reported contact/hanging. Review that actual pose
and clearance separately; shorter travel alone does not establish causation.
New official CW/CCW short-start implementations have no complete physical
validation logs here yet. Explicit procedure markers now separate their future
batch reports from these legacy reverse-localization results. Historical excerpts455/456 remain excluded from automatic batch analysis.
Complete originals455/456 were recovered on2026-10-06 and are now the canonical
sources, with new short-start originals457-460 archived below. The paragraph
above describes the preceding historical aggregate, not the new batch.

| Evidence file | SHA-256 | Firmware/build | Direction | Complete | Physical report | Analysis/limitations |
| --- | --- | --- | --- | --- | --- | --- |
| `20261006_log_466_cw.txt` | `ef4699b91c9e2bcbe42ad4af5c6b437e8718fb05f3b4c31f95458f2beccbb5b2` | Diagnostic/connector build strings in raw original; installed binary not read back | CW | Yes, 105,772 bytes | CW trial1: all three laps good, no problems; discovery occasionally hesitant; same layout A, sequence confirmed by user | See LATER_LAPS_IMPLEMENTATION: exact later tracking absent; calibrated steering and rectangular-wall candidate prepared, not physically accepted; no measured clearance |
| `20261006_log_467_cw.txt` | `33c39936ccc2da326ad90403947dd26b30ffb86a9596fc397f399b02dc6136ce` | Diagnostic/connector build strings in raw original; installed binary not read back | CW | Yes, 93,096 bytes | CW trial2: lap1 good; lap2 inner-wall collision avoiding middle-inner RED in S3; other unfavorable trajectories; same layout A, sequence confirmed by user | See LATER_LAPS_IMPLEMENTATION: exact later tracking absent; calibrated steering and rectangular-wall candidate prepared, not physically accepted; no measured clearance |
| `20261006_log_468_cw.txt` | `d9c84f63ea2a57eac460d806b2838f660c986a50a860ae08ef89612f7f4edc04` | Diagnostic/connector build strings in raw original; installed binary not read back | CW | Yes, 92,948 bytes | CW trial3: lap1 good; lap2 inner-wall collision avoiding middle-inner RED in S3; other unfavorable trajectories; same layout A, sequence confirmed by user | See LATER_LAPS_IMPLEMENTATION: exact later tracking absent; calibrated steering and rectangular-wall candidate prepared, not physically accepted; no measured clearance |
| `20261006_log_469_cw.txt` | `cd916412d146554da0eab5bd036cd106ce4ae59104b0fac4a8c10c36a0926518` | Diagnostic/connector build strings in raw original; installed binary not read back | CW | Yes, 98,215 bytes | CW trial4: lap1 good; lap2 inner-wall collision avoiding middle-inner RED in S3; other unfavorable trajectories; same layout A, sequence confirmed by user | See LATER_LAPS_IMPLEMENTATION: exact later tracking absent; calibrated steering and rectangular-wall candidate prepared, not physically accepted; no measured clearance |
| `20261006_log_470_cw.txt` | `36afca7c0b98409f12f43161a774c053c710d04956509bc4b1c4899b2777725a` | Diagnostic/connector build strings in raw original; installed binary not read back | CW | Yes, 99,224 bytes | CW trial5: lap1 good; lap2 inner-wall collision avoiding middle-inner RED in S3; other unfavorable trajectories; same layout A, sequence confirmed by user | See LATER_LAPS_IMPLEMENTATION: exact later tracking absent; calibrated steering and rectangular-wall candidate prepared, not physically accepted; no measured clearance |
| `20261006_log_461_ccw.txt` | `889871ba3f06b5cdeb51c77bad32b220257388376d31c4c5ddba5cb00319533f` | PARK diagnostic Oct6 18:19:03; connector not emitted; binary not read back | CCW | Yes, 90,942 bytes | CCW trial1: stopped autonomously after exit; user confirms unchanged layout A; contacts only lap1 | Measured scan endpoint59.7/-1209.8/29.9; no safe connector, parking-piece or hidden middle-seat guard rejects candidates. |
| `20261006_log_462_ccw.txt` | `d0ac715f8f32c9a6d70f875e4a1ef727fe432125549c2a22a985f40da8d41793` | PARK diagnostic Oct6 18:19:03; connector not emitted; binary not read back | CCW | Yes, 54,000 bytes | CCW trial2: stopped autonomously after exit; user confirms unchanged layout A; contacts only lap1 | Measured localized pose130.5/-1221.0/0.1; scan/scout footprint rejects before motion. |
| `20261006_log_463_ccw.txt` | `9a2330df165eed93ae3593cfa502b18171e09035d2af283cefb8d6ea5a295f8a` | PARK diagnostic Oct6 18:19:03; connector Oct  6 2026_18:21:00; binary not read back | CCW | Yes, 74,008 bytes | CCW trial3: right wheel hits GREEN at S3 entry; user confirms unchanged layout A; contacts only lap1 | Investigating dynamic discovery GREEN/GREEN corner; first-lap geometry differs from optimized later laps. |
| `20261006_log_464_ccw.txt` | `aaa6feaadde984988af0442dfb0523ca761c8437bc76e5b48f9e036d089821fe` | PARK diagnostic Oct6 18:19:03; connector Oct  6 2026_18:21:00; binary not read back | CCW | Yes, 75,867 bytes | CCW trial4: right wheel hits GREEN at S3 entry; user confirms unchanged layout A; contacts only lap1 | GREEN seat18 confirmed; stall after contact. Investigating preceding GREEN seat17 and additive corner tapers. |
| `20261006_log_465_ccw.txt` | `f99aa27dc71b36bb93caca8ed3885eef1282e0694b9fd92c56142d31261e1d66` | PARK diagnostic Oct6 18:19:03; connector Oct  6 2026_18:21:00; binary not read back | CCW | Yes, 107,408 bytes | CCW trial5: light GREEN contact at S3 entry; continues, remainder looks good; user confirms unchanged layout A; contacts only lap1 | GREEN seat18 confirmed after clear observation; completes3laps/runout. Physical contact means not an accepted run. |
| `20261006_log_455_cw.txt` | `a6a80bcd30e4596f5d4dc2bdc4ff47d35672803ec1d3ba9ae188cb1aae7754b4` | M7 `Oct 5 2026 21:30:53`; installed binary not read back | CW | Yes, 59,883 bytes | Earlier outer-wall approach; autonomous stop, no contact reported; not this batch | Complete original recovered; supersedes excerpt as canonical evidence. Heading guard stop; historical case. |
| `20261006_log_456_cw.txt` | `2193bf989b91dfb20e59a0dccb0b3521faedb1182af925414bbf2e4dfa6d44c1` | M7 `Oct 5 2026 21:30:53`; installed binary not read back | CW | Yes, 67,372 bytes | Earlier run; physical report not individually assigned | Complete original recovered; supersedes excerpt as canonical evidence. Heading guard stop; historical case. |
| `20261006_log_457_cw.txt` | `88cecd33d8f7bb316ec80380db2e54f805071ed5979e69fc372df347d5589d7a` | M7 `Oct 6 2026 17:25:06`; installed binary not read back | CW | Yes, 108,768 bytes | User confirms different setup; not the current three trials | Completed three laps plus final runout; separate layout. Does not validate current layout. |
| `20261006_log_458_cw.txt` | `4c9424bc7be68dd6daa91a66254cc3f8c653b5f9672aa2381a3412105aa4cacc` | M7 `Oct 6 2026 18:19:03`; installed binary not read back | CW | Yes, 43,500 bytes | Current CW trial 1, middle inner GREEN; good left pass then autonomous hold; no contact | Connector merge142 -> initial phase-zero crossing incorrectly counted as lap1; incomplete-map boundary gate holds. Manual disable follows the autonomous hold. |
| `20261006_log_459_cw.txt` | `c5849df95f441fd86ce6d3c3e0ef31922ba6af4b7480de66105835d62268b401` | M7 `Oct 6 2026 18:19:03`; installed binary not read back | CW | Yes, 55,983 bytes | Current CW trial 2, middle inner GREEN; good left pass then autonomous hold; no contact | Same false initial lap crossing after merge142; not a colour rejection or lap2 speed failure. |
| `20261006_log_460_cw.txt` | `9405721ecf272c0469b91d5fa6697b894611c5b504d636c1f24880a0103a6a6a` | M7 `Oct 6 2026 18:19:03`; installed binary not read back | CW | Yes, 24,052 bytes | Current CW trial 3; autonomous hold beside parking bay; no contact | Initialized field pose 240.9/-1220.6/182.2; settled heading error2.1 exceeds old2.0 gate, misleading missing-reference hold. No driving beyond exit. |
| `20261006_log_455_cw_excerpt.txt` | `a31feffda832d2f16eea905c3c51e5fb162d1612f046d395a1b24dd4d0323368` | M7 connector revision after logs 453/454; installed binary not read back | CW | No, 1,078-byte selected telemetry excerpt; full 59,883-byte source was queried but removable stick disconnected before unchanged archive copy, so the rest is missing | User: left toward outer wall, stopped autonomously, front wheel about 150 mm away, no contact | GREEN S0/2 inner seat4, connector complete. At followup 605.6 mm: commanded +26 deg right but heading 234.7 deg left, path heading 199.6 deg, modeled wall/pillar 187.4/208.6 mm; true 35.1 deg heading guard stopped motor. Replace with full original when stick returns. |
| `20261006_log_456_cw_excerpt.txt` | `7f8e247c6bb437e92d155d2734dbb6ed127cb72fcb7a24c24c5afe00866143a3` | same M7 revision | CW | No, 537-byte selected telemetry excerpt; full 67,372-byte source was queried but removable stick disconnected before unchanged archive copy, so the rest is missing | Physical outcome not individually assigned; user reports latest leftward wall approach and autonomous stop | Similar GREEN detection and true 35.1 deg guard stop at followup 630.2 mm; commanded +25.3 deg right, modeled wall/pillar 192.2/214.3 mm. Replace with full original when stick returns. |
| `20261006_log_453_cw.txt` | `b28b59182c0903054b556ad9ac631925b3955779be32bbbc1359bcb8d6246fda` | M7 connector build `Oct 6 2026_00:03:11`; consistent with live-path heading-guard revision, installed binary not read back | CW | Yes, 57,384 bytes | User: autonomous stop near middle of start section, trial 1/2 | GREEN S0/2 inner seat4 confirmed. Old route-prefix gate rejected retention because GREEN taper changed later points; unnecessary replan to merge143 then steering -42.067 deg exceeded 42 deg and motor locked. |
| `20261006_log_454_cw.txt` | `917c9c0d1dd8021f138b7139f599dc63877fa5109fe601c6307c2f5e0a3b3506` | same M7 connector build | CW | Yes, 54,919 bytes | User: autonomous stop near middle of start section, trial 2/2 | Same false route-prefix rejection; unnecessary replan to merge144 then finite endpoint steering -42.027 deg exceeded 42 deg and motor locked. |
| `20261006_log_452_cw.txt` | `03a22c34b6c3ae7f453dcce506f8159bbc25325ebb65ce4184fa7f8a903162eb` | M7 connector build `Oct 5 2026_23:49:40`; consistent with far-GREEN followup revision, installed binary not read back | CW | Yes, 53,632 bytes | User: started well, saw sole inner GREEN, steered left and stopped while angled; outer wall visibly not close, roughly consistent with modeled 340 mm reserve | GREEN confirmed S0/2 inner seat4; connector retained and completed at 59.9 mm/13.7 deg. Followup stopped at travel 355.1 mm: cross-track 43.4 mm, modeled wall/pillar 339.7/305.2 mm, baseline heading error 35.1 deg. Outgoing displaced route tangent from adjacent logged targets is about 199.6 deg, giving about 15.5 deg relative error; false heading guard. End-of-run left ToF 125 mm did not correspond to the user's visible outer-wall distance; target feature unknown. No physical contact reported. |
| `20261005_log_449_cw.txt` | `7e6c2348e7ca1c4d1465cbe4dc3fca56e9edd76a0f92fec14d1ee7aabc6951a9` | M7 connector build `Oct 5 2026_23:19:15`; consistent with far-GREEN revision, installed binary not read back | CW | Yes, 51,604 bytes | Far/left GREEN, trial 1/3; user reports autonomous stop | S0/2 GREEN seat4 confirmed; retained connector, then live steering -42.287 deg exceeded 42 deg at 60.56 mm/14.26 deg endpoint error; motor locked. |
| `20261005_log_450_cw.txt` | `5ed788dd6887fc11aeb1d4b59baf039677e813ee19ed1f25351b810341a54776` | same M7 connector build | CW | Yes, 49,318 bytes | Far/left GREEN, trial 2/3; long left curve into outer wall before the green pillar | S0/2 GREEN seat4 confirmed; retained connector completed at 60.0 mm/11.4 deg. Later unresolved lap boundary, slip reports and stall. No post-handoff pose/steering trace, so exact steering cause is unmeasured. |
| `20261005_log_451_cw.txt` | `46b1b4c7a6cc6a635354a78779162495311067d101365bc8a094ce261ba9515a` | same M7 connector build | CW | Yes, 50,486 bytes | Far/left GREEN, trial 3/3; user reports autonomous stop | S0/2 GREEN seat4 confirmed; retained connector, then live steering -42.396 deg exceeded 42 deg at 65.18 mm/15.87 deg endpoint error; motor locked. |
| `20261005_log_446_cw.txt` | `2aa752b5d7d605baeb5c7e3ef0cfd6b0b19259d1c68579f6d3c9c1683b37e385` | M7 connector build `Oct 5 2026_22:51:38`, `route_lookahead=yes`; consistent with prepared revision, installed binary not read back | CW | Yes, 72,960 bytes | GREEN middle start, trial 1/3; user reports full success, no contact or visibly narrow gap | GREEN S0/1 seat2 confirmed; connector handoff 60.0 mm/12.4 deg; onboard lap 1 complete. |
| `20261005_log_447_cw.txt` | `d1f2609fbba83eac6e149a01fc0e3d2b2a461adafe5fc889b6adde0c4715c34a` | same M7 connector build | CW | Yes, 78,663 bytes | GREEN middle start, trial 2/3; user reports full success, no contact or visibly narrow gap | GREEN S0/1 seat2 confirmed; connector handoff 60.0 mm/11.4 deg; lap 1 complete. A later contradictory opposite seat3 was ignored. |
| `20261005_log_448_cw.txt` | `d18bb1b05dc9e82557b25bcfadd5413aa1ab656a3d1a2567f25aff5981784e1d` | same M7 connector build | CW | Yes, 79,342 bytes | GREEN middle start, trial 3/3; user reports full success, no contact or visibly narrow gap | GREEN S0/1 seat2 confirmed; connector handoff 49.2 mm/14.9 deg; lap 1 complete. A later contradictory opposite seat3 was ignored. Heading gate is 15 deg, so this handoff had only 0.1 deg logged margin; monitor under placement variation. |
| `20261005_log_429_cw.txt` | `fe0065270a694e477caaba2b56f9e4fb5b539a433a3dd543c832267bda74279e` | M7 Oct_5_2026_21_30_53 header; installed binary not read back | CW | Yes, 6,008 bytes; short run | Extra start before the 16 assigned trials; physical outcome not assigned | Rear-position stationary verification failed after 20.4 mm correction; no exit or lap. |
| `20261005_log_430_cw.txt` | `9a5111cc46b7e74a8d1a570358d0c6f5184ad318fcfa971bc700efbe8789176a` | same M7 header, 90/155 mm exit | CW | Yes, 46,425 bytes | Green at middle start seat, trial 1/3; stopped | GREEN confirmed S0/1 right seat2; connector preflight passed, then forward tracking rejected at 59.8 mm and motor locked. |
| `20261005_log_431_cw.txt` | `054bfd6c51fb09c71d97d4ffc7679b399a70ac158c57737f1f2a0ec3964f9e1e` | same M7 header | CW | Yes, 64,494 bytes | Green at middle start seat, trial 2/3; stopped | S0/1 and S0/0 first marked clear; late RED assigned to S0/1 right seat2 despite reported green. Connector replan found no safe merge. Raw image unavailable. |
| `20261005_log_432_cw.txt` | `2d83f4b212e9d6818e8e75930e750995a83088f0b750e871d39a3d9e14e3e04d` | same M7 header | CW | Yes, 74,757 bytes | Green at middle start seat, trial 3/3; stopped | GREEN confirmed S0/1 right seat2; connector preflight passed, then forward tracking rejected at 64.4 mm and motor locked. |
| `20261005_log_433_cw.txt` | `e377c97825b6e4c83f0a7049d8830e5e547370219f4f09a78cbbd5678106fdf8` | same M7 header | CW | Yes, 54,979 bytes | Green at far/left start seat, trial 1/3; stopped | S0/0 and S0/1 cleared, initial connector armed; late GREEN confirmed S0/2 right seat4, replan found no safe merge. |
| `20261005_log_434_cw.txt` | `a6b1f10f78223e46d8baf2b8c07f004c05d38ec8adf0973efb8c53e4bbd27849` | same M7 header | CW | Yes, 49,853 bytes | Green at far/left start seat, trial 2/3; stopped | Same late GREEN S0/2 right seat4 and rejected connector replan. |
| `20261005_log_435_cw.txt` | `eadac6b0e5127d64bfd81f8a2534592e1f5daef3726d5a1fed45ab6671adb00a` | same M7 header | CW | Yes, 46,954 bytes | Green at far/left start seat, trial 3/3; stopped | Same late GREEN S0/2 right seat4 and rejected connector replan. |
| `20261005_log_436_cw.txt` | `184a87791f43db6aac8274da26a39e6b20d4698dddac5db698eea1cc7ccc8694` | same M7 header | CW | Yes, 45,479 bytes | Red at near/right start seat, trial 1/3; stopped | RED confirmed S0/0 right seat0; connector preflight rejected every candidate: tracking/max travel or hidden pillar clearance. |
| `20261005_log_437_cw.txt` | `fb51ec4e27cd2722bf3218c6a423833cdc0e10c71af9933a2c70c00b52188474` | same M7 header | CW | Yes, 43,750 bytes | Red at near/right start seat, trial 2/3; stopped | RED confirmed S0/0 right seat0; connector preflight again rejected all candidates for tracking/hidden pillar clearance. |
| `20261005_log_438_cw.txt` | `743cde84d3ddf522ff1ccac95ac4870eef66d00aca75aed32aaa41c186cf932b` | same M7 header | CW | Yes, 73,516 bytes | Red at near/right start seat, trial 3/3; stopped | Initially both near stations clear; late RED mapped to S0/1 right seat2, not the reported near seat0. Replan found no safe merge. Raw image unavailable. |
| `20261005_log_439_cw.txt` | `ed4473b48ddb74d0fbc2b7b3e5669c7303e93f0705ee04991dffd4b5aeed11ee` | same M7 header | CW | Yes, 86,933 bytes | Red at middle start seat, trial 1/4; physically successful | RED confirmed S0/1 right seat2; connector and onboard lap 1 completed. |
| `20261005_log_440_cw.txt` | `17af9e8fe42fca1ec546c5330ed854c512585ec700f5b46d0f41f2fb797147f8` | same M7 header | CW | Yes, 91,515 bytes | Red at middle start seat, trial 2/4; physically successful | RED confirmed S0/1 right seat2; connector and onboard lap 1 completed. |
| `20261005_log_441_cw.txt` | `6bc1dc20a965e3d73d233e4c7c9b4dcf28443fc0e1b56cc4a675a0abb0664fea` | same M7 header | CW | Yes, 31,497 bytes; run stopped | Red at middle start seat, trial 3/4; user reports robot hung at a wall while exiting | All five exit segments completed. During reverse edge localization, reverse command stayed at -100 mm/s but progress fell to 8.4 mm/1004 ms after about 68 mm reverse; stall guard fired before manual disable. Logs cannot identify the contacted wall. |
| `20261005_log_442_cw.txt` | `1b8df66148985b0df413ecd3fed7c67e5646f6cf7b2e496f4cb3a8ffbea4d49e` | same M7 header | CW | Yes, 90,135 bytes | Red at middle start seat, trial 4/4; physically successful | RED confirmed S0/1 right seat2; connector and onboard lap 1 completed. |
| `20261005_log_443_cw.txt` | `ae4dc499d29c9ab5e45d2a1ea6fae325d5f86ebc08e3b11db60b66523d085e1f` | same M7 header | CW | Yes, 94,890 bytes | Red at far/left start seat, trial 1/3; physically successful | RED confirmed S0/2 right seat4; connector and onboard lap 1 completed. |
| `20261005_log_444_cw.txt` | `3462f0fe002b4cffb87edeffbc5753e31ac6463dcfe1d5f94cf528d5bf9043c8` | same M7 header | CW | Yes, 94,106 bytes | Red at far/left start seat, trial 2/3; physically successful | RED confirmed S0/2 right seat4; connector and onboard lap 1 completed. |
| `20261005_log_445_cw.txt` | `7b19cf9063d4382cb38936efd0a4bb57c3fbc79efe34400b74ccc53530435306` | same M7 header | CW | Yes, 103,066 bytes | Red at far/left start seat, trial 3/3; physically successful | RED confirmed S0/2 right seat4; connector and onboard lap 1 completed. |
| `20261005_log_425_cw.txt` | `9ae31292d0657db7f46d6da35e2d13ccbe5ebf3ef21a6560412e7b5c5843833a` | M7 Oct_5_2026_21_30_53 parking header, 90/155 mm exit; installed binary not read back | CW | Yes, 85,454 bytes | First of four current same-layout runs: user reports perfect, no contact | Exit/localization/connector and lap 1 completed. Start S0/0 GREEN and S0/1 CLEAR. A late S0/1 RED seat3 confirmation after the lap approach conflicts with the earlier CLEAR and merits later review despite the physical success. |
| `20261005_log_426_cw.txt` | `f4d62993364b9b6f2b9ff6511895b5e2f0034cc711aacf2f81c81e2630eb7110` | same Oct 5 M7 header and exit | CW | Yes, 60,148 bytes | Second current run: user saw a pause after start green, then travel and autonomous stop around the middle of the start section | Exit and localization completed. Primary S0/1 timed out UNKNOWN; scout marked S0/0 CLEAR. During the active connector a RED candidate was confirmed as S0/1 right seat2 after prior S0/1 CLEAR, triggering avoidance injection. Connector replanning found no safe merge and locked the motor. This seat assignment conflicts with the reported unchanged layout, but the raw image/object identity is unavailable. Manual disable came afterward. |
| `20261005_log_427_cw.txt` | `60a8c24993c431c51500d7aac383d7dd4bb39e88bd5c72121979b74591887f19` | same Oct 5 M7 header and exit | CW | Yes, 87,667 bytes | Third current run: user reports perfect, no contact | Exit/localization/connector and lap 1 completed. Start S0/0 GREEN and S0/1 CLEAR; later section confirmations match the logged route. |
| `20261005_log_428_cw.txt` | `80b374c95a341791c123661354bad147404cc974270b7882231fb794281ac687` | same Oct 5 M7 header and exit | CW | Yes, 93,717 bytes | Fourth current run: user reports perfect, no contact | Exit/localization/connector and lap 1 completed. Rear-ToF needed one bounded micro-correction. Start S0/0 GREEN and S0/1 CLEAR. |
| `20261005_log_420_cw.txt` | `3ecec2de4d50625372b6d95e4908e911cc5b2ab5984302b2a27486aaa78c0965` | older M7 Sep_28_2026_22_58_57 parking header, 85/150 mm exit | CW | Yes, 81,864 bytes | Earlier run on stick; individual physical outcome not assigned | Exit/localization/connector and onboard lap 1 completed. Do not count toward four current Oct 5 revision trials. |
| `20261005_log_421_cw.txt` | `0e7ab297951df5682b98a6e7d454c3281846c76e710913f4703ce7683d4bb9b5` | same older M7 header | CW | Yes, 27,515 bytes; run interrupted | Earlier run on stick; individual physical outcome not assigned | Manual disable after rear positioning and part of exit; no completed exit or lap. |
| `20261005_log_422_cw.txt` | `52f4cf386c15aa5db4b00f8714beb451ffa3b509813f2469b8ed1f933c7d0d13` | same older M7 header | CW | Yes, 47,064 bytes | Earlier run on stick; individual physical outcome not assigned | Exit/localization completed. S0/1 GREEN seat2 was confirmed and the connector later locked due forward tracking rejection; no full lap. |
| `20261005_log_423_cw.txt` | `3b8571f69bb06d4362d26507b80c0a975f65854dee598bebcc80fce630cb5f63` | same older M7 header | CW | Yes, 35,217 bytes; run interrupted | Earlier run on stick; individual physical outcome not assigned | Manual disable during reverse edge localization after completed five-segment exit; no completed localization or lap. |
| `20261005_log_424_cw.txt` | `9cf72c8a12c5d23181de8e652d4c472e50eadb9c62fa6f019bc72d8f618d744b` | same older M7 header | CW | Yes, 92,379 bytes | Earlier run on stick; individual physical outcome not assigned | Exit/localization/connector and onboard lap 1 completed. Do not count toward four current Oct 5 revision trials. |
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
