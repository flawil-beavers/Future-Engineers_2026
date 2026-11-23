# Robot run evidence

We keep the original USB logs here so that each result can be traced back to
the recorded run. Find a test by date and original log number. CW means
clockwise and CCW means anticlockwise. Each batch gives the available build
identity, observations and SHA-256 hashes. A file marked as an excerpt
contains only the available portion.

Open an original log in the inspector to review the estimated route and its
events. Use the batch notes and physical observations to check what happened
on the field. [Open a run in the inspector](../../../tools/robot-run-inspector/README.md) · [Regenerate parking plots](../../README.md#parking-exit-model)

## Batch ingestion

For every returned test batch, follow the USB copy and verification order in
[the workspace instructions](../../../AGENTS.md).

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
| `20261006_log_471_cw.txt` | `188d6a1f0101d2c25d7f45b5a9a1e1ef5fa3096a2468c2f56c7a33c15fd8e044` | Parking `Oct__6_2026_19_54_43`; connector `Oct 6 2026_19:54:26`; binary not read back | CW | Yes, 42,312 bytes | CW trial1: GREEN start contact then halt; unchanged A per user | Reviewed in CW_CCW_471_476_REVIEW; start/corner candidate prepared, not physically accepted; onboard pose is not measured clearance |
| `20261006_log_472_cw.txt` | `da4ded31f93cb58e86eaa64639745f2ae80d31e2126cb4b633e3921464a7d222` | Parking `Oct__6_2026_19_54_43`; connector `Oct 6 2026_19:54:26`; binary not read back | CW | Yes, 23,407 bytes | CW trial2: exits bay then autonomous halt; unchanged A per user | Reviewed in CW_CCW_471_476_REVIEW; start/corner candidate prepared, not physically accepted; onboard pose is not measured clearance |
| `20261006_log_473_cw.txt` | `94a7104c3eb71d8b4b7803032d4c62324ce910bb0504073a96ac76a8aae3c922` | Parking `Oct__6_2026_19_54_43`; connector `Oct 6 2026_19:54:26`; binary not read back | CW | Yes, 126,406 bytes | CW trial3: all3laps contact-free; S3 RED wall clearance visually4-5cm; unchanged A per user | Reviewed in CW_CCW_471_476_REVIEW; start/corner candidate prepared, not physically accepted; onboard pose is not measured clearance |
| `20261006_log_474_cw.txt` | `2c02179aa3fbf6a088ccb83cb455f321d6584b87a846ad2a377e4bff02010bcb` | Parking `Oct__6_2026_19_54_43`; connector `Oct 6 2026_19:54:26`; binary not read back | CW | Yes, 124,519 bytes | CW trial4: all3laps contact-free; S3 RED wall clearance visually4-5cm; unchanged A per user | Reviewed in CW_CCW_471_476_REVIEW; start/corner candidate prepared, not physically accepted; onboard pose is not measured clearance |
| `20261006_log_475_ccw.txt` | `fae67bf609f302762b0a8ab337ef869e8416550f1e1c7a2aa5e15aef0079d4ef` | Parking `Oct__6_2026_19_54_43`; connector `Oct 6 2026_19:54:26`; binary not read back | CCW | Yes, 128,872 bytes | CCW returned trial1: all runs reported contact-free,3laps recorded; unchanged A per user | Reviewed in CW_CCW_471_476_REVIEW; start/corner candidate prepared, not physically accepted; onboard pose is not measured clearance |
| `20261006_log_476_ccw.txt` | `3afa8b91c000c15b9f44d5fd9c81ef090c6a028b31b14bf2dfe8a5596eebef2f` | Parking `Oct__6_2026_19_54_43`; connector `Oct 6 2026_19:54:26`; binary not read back | CCW | Yes, 135,196 bytes | CCW returned trial2: all runs reported contact-free,3laps recorded; unchanged A per user | Reviewed in CW_CCW_471_476_REVIEW; start/corner candidate prepared, not physically accepted; onboard pose is not measured clearance |
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

## 2026-10-06 layout B, logs477-481

Raw source and archive SHA-256 matched byte-for-byte before further analysis.
Direction is verified by PARK_DIAG_CONFIG turn. Header identity for all five:
parking `Oct__6_2026_19_54_43`, connector `Oct  6 2026_19:54:26`;
these remain older than the preceding candidate, installed binary not read back.
User reports fixed photo layout B, CW return collisions with pink bay pieces
in both runs, and three CCW stops after exiting the bay.

| File | SHA-256 | Complete bytes | Physical report / telemetry |
| --- | --- | --- | --- |
| `20261006_log_477_cw.txt` | `aad0d148dd76c78a65408810912da226b54da58f66898968c18e3833dc4f9e56` | Yes, 75380 | CW trial1: pink parking-boundary contact on return. Stored GREEN seat0 injected and corner3 rounded; stall6.0mm/1000ms. |
| `20261006_log_478_cw.txt` | `bcc366d4a1c2c158e4ab1cdafa5e3b825ca46a4229cb8dca04a6b1ed4c8b547d` | Yes, 80081 | CW trial2: same contact on return. Same confirmed B map; stall7.7mm/1000ms. |
| `20261006_log_479_ccw.txt` | `dee4a5afcef80d7e5c31551a3ae329c53a5a848c5d4d555fac1792bcc0dae479` | Yes, 57401 | CCW trial1: user reports stopped after exit. Front GREEN confirmed; connector75mm lookahead accepted after lengthy search; gyro poll gap15704ms, then Manual disable. No explicit automatic hold record; cause not conclusively established. |
| `20261006_log_480_ccw.txt` | `5e3bc2faa1348b770e60a77100fa93fc3363405302a6b09537621da79d0b2976` | Yes, 83359 | CCW trial2: front GREEN confirmed. Automatic connector steering rejection atprogress0, request-42.077deg exceeds42deg. |
| `20261006_log_481_ccw.txt` | `062533137a19d5d966c09d1d0684d8378bfb36deb7811d2b854f5267f6c8e399` | Yes, 90796 | CCW trial3: front GREEN confirmed. Automatic connector steering rejection atprogress0, request-42.037deg exceeds42deg. |

## 2026-10-06 A/C, logs 482-492

Parking Oct__6_2026_20_56_49; connector Oct 6 2026 21:04:51, older than prior candidate.
Complete unchanged originals archived before analysis; source/archive SHA-256 matched at capture.
User confirms C start far-inner RED, correcting the initial GREEN description. No exact physical attribution is claimed for extra log486.

| File | SHA-256 | Complete bytes | Physical report / telemetry |
| --- | --- | --- | --- |
| `20261006_log_482_cw.txt` | `ed92bdc4ee774f3f5466580e07e24db74d15396a95479cbdf3cd1763c1ed5dcf` | Yes, 118668 | A CW1: user reports three laps, no contact. |
| `20261006_log_483_cw.txt` | `c301f7a0397a52f22b8cf421d4eae2cfca27eb2c2a76ddd695ba6728b44d3638` | Yes, 121992 | A CW2: user reports three laps, no contact. |
| `20261006_log_484_ccw.txt` | `48119c70629c6932cf3643dcf55eb856715e61e546cdbe569ebdb71c49704971` | Yes, 132043 | A CCW1: user reports three laps, no contact. |
| `20261006_log_485_ccw.txt` | `b39693bff2f7c528255b42aa49c026deae7236dfaa57f5b163a3e7c81161ef85` | Yes, 142115 | A CCW2: user reports three laps, no contact. |
| `20261006_log_486_cw.txt` | `8273080c3563b1f070690aeae07c1d2250859a95fb0e04e08ae2f8a46fc0b7ef` | Yes, 35453 | Additional CW exit trial: manual disable after exit5/5. Exact association with the reported short exit is not confirmed. |
| `20261006_log_487_cw.txt` | `972458d8c309ec2ced5ea1b2d17e26c3baa8bee03557b3990b3b42a594701f78` | Yes, 75829 | C CW1: contact with second outer GREEN in S1 after returning inward between two GREEN ends. |
| `20261006_log_488_cw.txt` | `11241cf9fd727fe657f7017541699599e8d336bf22f756928c0194e4afbe5615` | Yes, 79364 | C CW2: same contact. Seat11 confirmed at pose -1325,-320.6,64.32deg, range717mm before collision. |
| `20261006_log_489_ccw.txt` | `7099254e026ad7991d94d3d36b41d88e731253dbee1e115e254b3e9c79c4610f` | Yes, 62480 | C CCW: automatic connector travel hold after passing its finite endpoint; outgoing tangent alignment was already reached. |
| `20261006_log_490_ccw.txt` | `49d94afbf38b835fc59cd76e86badf1e3a10cdfbf99e2854d5959bf1025dd3f7` | Yes, 66850 | C CCW: passes GREEN then stops at S1 middle. Previous inference required both end signs to exclude middle. |
| `20261006_log_491_ccw.txt` | `749341d7e1eac34141614355f5d55d7addbf6e5434dd911c6ba8f08efe94e433` | Yes, 46924 | C CCW: initial front seat unresolved; rejected broad GREEN component behind its ground projection. No raw RGB capture. |
| `20261006_log_492_ccw.txt` | `0ad232f03971e81601e60939c3996af53ba8938009ed5cc967815a01154916f8` | Yes, 62282 | C CCW: GREEN7 confirmed during connector; fresh replan rejected although original merge can remain valid. |

## 2026-10-06 B repeat, logs 493-499

Parking Oct__6_2026_21_24_58; connector Oct 6 2026 21:24:45, consistent with prior candidate build markers. Installed binary not read back.
Complete unchanged originals archived before analysis; source/archive SHA-256 matched at capture.
User reports unchanged B: CW1/2/4 contact-free; CW3 and all three CCW automatic stops without contact.

| File | SHA-256 | Complete bytes | Physical report / telemetry |
| --- | --- | --- | --- |
| `20261006_log_493_cw.txt` | `0dc51a9be38a26bb1c8f2fe4ac2869f27bb7f3206158b0a9608b65233914d202` | Yes, 139287 | B CW1: user reports all three laps excellent, no contact. |
| `20261006_log_494_cw.txt` | `8f404fc400309a5c90a34114efea790d044f85a4c9dea18d439fb75800e055a4` | Yes, 148289 | B CW2: same successful physical report. |
| `20261006_log_495_cw.txt` | `bbbd912e043e1b3ca8682333304be636f49ce9108c50589dd103bbc08d302bf8` | Yes, 84444 | B CW3: no contact; automatic hold before S2 solitary middle GREEN. One first-station side already CLEAR, other too near/out of view; broad GREEN blocks empty evidence. |
| `20261006_log_496_cw.txt` | `110f1602ef076a7b4ad9c9892359a8bb3873e20359ac923d46c457931bdda91a` | Yes, 146003 | B CW4: user reports all three laps excellent, no contact. |
| `20261006_log_497_ccw.txt` | `cabda2988c6357eda9eeee9e25837e5bc1c0bc889312138c78b08bf113738676` | Yes, 62834 | B CCW1: passes GREEN, no contact, stops. RED10 confirmation during start connector triggers rejected replan. |
| `20261006_log_498_ccw.txt` | `63263a299ca8645dac68e5abe65d09300717471ca372ecebc51b43e9cd07af65` | Yes, 65400 | B CCW2: same physical report; distant RED projected to seat11 (inner) rather than physical seat10 (outer), then rejected replan. |
| `20261006_log_499_ccw.txt` | `96b8e9eeab316c91cc396042d91bac0910f332d901a8a0fbe1fd79cb162a28e0` | Yes, 58490 | B CCW3: same physical report; distant RED11 confirmation then rejected replan. |


## 2026-10-06 B CW laps accepted, final parking fails (500-501)

Complete unchanged originals copied before analysis; capture SHA-256 verified against source.
Parking build Oct__6_2026_22_37_23; connector Oct 6 2026_22:40:15, consistent with prior candidate. No installed-binary readback.
User reports both CW three-lap drives perfect/contact-free; failures occur during final parking.

| File | SHA-256 | Complete bytes | Physical report / telemetry |
| --- | --- | --- | --- |
| `20261006_log_500_cw.txt` | `3846f82c283b7b2e5f1747270d9c9e4eb77ffcc72e2b7ab70b546462da86e5d9` | Yes, 154226 | CW1: overshoots bay and stops, no entry. Three laps complete; scan armed, then dual_marker_scan_incomplete; no entry segment reached. |
| `20261006_log_501_cw.txt` | `6e6204fe5521a3c7b0ffe98a967a11af510a49079d104481fe0ed173e70a4744` | Yes, 158223 | CW2: grazes pink during return, stuck during subsequent forward motion. Three laps complete; scan starts, no armed/completed scan or entry segment logged; later Manual disable. Physical contact not independently measurable from this log. |

Final-parking pose/raw-ToF periodic trace is absent in this firmware; exact scan path, wall residual and mixed-object selection are not recoverable. Legacy navigation Target heading is not the parking heading. See `simulation/LAYOUT_B_FINAL_PARKING_500_501_REVIEW.md`.


## 2026-10-06 B CCW discovery stops and B/C CW, logs 502-505

Complete originals captured unchanged before analysis; individual SHA-256 checked against source at capture. Source removed later; saved originals remain available.
Parking Oct__6_2026_22_37_23; connector/corner Oct 6 2026_22:40:15. Same older markers as 500/501; no FINAL_PARK_TRACE, final target y100. Latest parking revision not tested. No installed-binary readback.
Layouts assigned from user sequence; directions verified from turn headers. User reports no contact at either CCW halt.

| File | SHA-256 | Complete bytes | Physical report / telemetry |
| --- | --- | --- | --- |
| `20261006_log_502_ccw.txt` | `16c7f5d7ba0e3abcd9b69e1e4277048039e9c9dd860a0d134a0e2a309a8464f4` | Yes, 102547 | B CCW1: successful exit, halt at fourth corner, no contact. S3 RED18 acquired; S0 reverse90 then parallax; return locks at heading_error=-3.01, origin_encoder_delta=-30.5 before completing return. No completed lap. |
| `20261006_log_503_ccw.txt` | `0010d0b95b10ff78e1f9440cafa31ce188c79c9a27b28d7020d4a13d31f8da64` | Yes, 80878 | B CCW2: successful exit, halt at third corner, no contact. S3 reverse80, stopped RED remains123px tall (cap120); station unresolved on return, heading_error=-0.74/cross_track=-0.7. No completed lap. |
| `20261006_log_504_cw.txt` | `f9ec504f3dd860c4cb5fd034dc8236d9ffe6e974e9519a4adff7e6c5cea3230d` | Yes, 157322 | B CW: user reports perfect drive; all three laps complete. Older parking abort dual_marker_scan_incomplete before entry. |
| `20261006_log_505_cw.txt` | `2d4c6fceefd3d82884493d0679b4fcf138fd3cec3f35db578b711e2fd1eb84d4` | Yes, 153392 | C CW: user reports successful drive, straight/reverse/stop during parking. Three laps complete; older parking abort approach_outer_gate before scan or entry. |

See `simulation/LAYOUT_B_C_502_505_REVIEW.md`. Bounding-box vision telemetry is available; raw RGB and final-parking pose/ToF traces are absent.

## 2026-10-07: layouts C/B, CW final parking (506--507)

Both complete originals were copied unchanged before analysis and checked
byte-for-byte by SHA-256 against the source. Direction is CW in both logs;
layout names follow the user's reported order (the user was unsure of 506's
direction). Connector identity: `Oct 6 2026_23:10:00`. Both contain
`FINAL_PARK_TRACE` and the y=120 entry revision; installed binary was not read
back. User reports three successful laps in both, unsuccessful final parking,
and good lateral clearance in 507's return past the bay.

| File | SHA-256 | Complete bytes | Physical report / telemetry |
| --- | --- | --- | --- |
| `20261007_log_506_cw.txt` | `2bad3c0f62f139db45830406250f5ad0282a829b9129ef1e368bcd613d2f8446` | Yes, 150564 | C CW: three laps complete. Parking remains in approach state 1, ending near x449/y-999. Repeated gyro timeouts/restarts interrupt motion; manual disable follows. No marker scan or entry. |
| `20261007_log_507_cw.txt` | `e5b798e7e8a1b7327cc78064b2b607fcf286768bf268df673f219f51d4032fd1` | Yes, 161409 | B CW: three laps complete. First pink returns at x709/693 precede old x560 scan arm; later marker at x433 is counted as first. Aborts at x120 with dual_marker_scan_incomplete, never reaches entry. |

See `simulation/LAYOUT_B_C_FINAL_PARKING_506_507_REVIEW.md`. Final parking
diagnostics are sampled, not a full-rate sensor or independent ground-truth
pose recording. No new dedicated sensing test was requested.


## 2026-10-26: four returned runs (508--511)

Complete original source files copied unchanged before analysis; source/archive SHA-256 matched. Test date follows receipt date; USB filesystem dates are invalid (2098). Directions verified from diagnostic turn headers and match user order. Parking build `Oct_18_2026_20_08_55`; connector build `Oct 18 2026_20:08:41`. Installed binary was not read back. All four include RUN_END with truncated=0; this does not imply every sensor sample was logged.

| File | SHA-256 | Complete bytes | Physical report / telemetry |
| --- | --- | --- | --- |
| `20261026_log_508_cw.txt` | `5c46c575e32dcf127baa9bb9a4fabd739e22aedd2a20df263c9558d192b36232` | Yes, 53161 | CW1: user reports stop after start; connector tracking rejects steering beyond42deg. No contact reported. |
| `20261026_log_509_ccw.txt` | `14338878c0aaf21bf6408100500fddc8293af7225e3d559c9a528a22f8b735ed` | Yes, 32899 | CCW: user reports stop after start; scan/scout footprint preflight rejects. |
| `20261026_log_510_cw.txt` | `4bba445ecff125c1758a70aa3d817fb8f42ea2ab96b43d1cf7035a1f75341c71` | Yes, 164022 | CW2: user reports all driving successful except final parking; three laps complete, dual_marker_scan_incomplete abort. |
| `20261026_log_511_cw.txt` | `a063008e7ea19574bf60ffcbf0175af723bcbff9978cb63d29a86aa7fe88f78c` | Yes, 27499 | CW3: user confirms LEFT front wheel contacts FRONT parking barrier during exit. Exact contact instant not identified; no-progress watchdog triggers mode_pause in segment5. |


## 2026-10-26: three new CW failures (512--514)

Complete originals copied unchanged from the source archive before analysis;
SHA-256 and byte counts verified. Receipt date used because source filesystem
clock is invalid (2098). All diagnostic headers have turn=-1 and parking build
`Oct_26_2026_16_57_33`; installed binary was not read back. All RUN_END records
have truncated=0, which is not a full-rate sensor recording guarantee.

| File | SHA-256 | Complete bytes | Physical report / telemetry |
| --- | --- | --- | --- |
| `20261026_log_512_cw.txt` | `078f39fe83bb7a44d7c3e331c4eafd060207ea0e723c1a5a25f84e42a61d954b` | Yes, 84732 | First CW: user reports missed GREEN after three quarters of lap1. Late seat23 GREEN injection after a CLEAR; ends stopped near estimated x1210/y-415. Exact physical pillar seat/contact not independently measured. |
| `20261026_log_513_cw.txt` | `55335769760eb6a2fc9b0106cf4e2a5e43f31a39546af071072f8064727a2252` | Yes, 13729 | Second CW: user reports no exit. Current-pose footprint guard holds during reverse segment3; field-start estimate x393.3/y-1360, outside the earlier tested Y range. Encoder motion also occurs during the preceding steering settle. |
| `20261026_log_514_cw.txt` | `17cfea0b6c7199a6c8967b6bd5c026722f1316f04cfaf68554497eaa51a73ddd` | Yes, 70622 | Third CW: user reports missed GREEN in section after start, then RED contact. CLEAR at S1 station1 despite rejected low GREEN; RED wrongly latches seat11 before closer projections favor seat10 and are discarded; subsequent no-progress watchdog stops. |

Analysis and delivered candidate: `simulation/FIXES_512_514.md`. Camera frames
for the failed sections were not saved; colour/exposure or true field-pose
errors cannot be reconstructed from these logs alone. No dedicated ToF test.


## 2026-10-26: CW518--520 and preceding stationary attempts515--517

Complete source originals copied unchanged; bytes and SHA-256 verified.
Receipt date used because source clock reads2098. User describes three CW
runs;518--520 match the three substantive runs in sequence, each with turn=-1,
build `Oct_26_2026_17_45_26`, targets30/36/26/84mm. Installed binary not read back.
515--517 are complete additional stationary attempts found immediately before
these runs: no direction/build diagnostic header, no recorded movement; the
`cw` filename is batch context only, not independently confirmed direction.
They are not substituted for the user's three described runs. All six include
RUN_END,truncated=0; full camera frames were not logged.

| File | SHA-256 | Complete bytes | Physical report / telemetry |
| --- | --- | --- | --- |
| `20261026_log_515_cw.txt` | `afc8057b0b279099cb8c07f2078d83fd3b02133561ff50d7d66e60a1b65d8679` | Yes, 889 | Ancillary stationary attempt3.765s;0mm. Direction/build unverified. Not one of the three described moving runs. |
| `20261026_log_516_cw.txt` | `115698349db564e91b3a744d65b705f758d3877dfd57540fc1034c12f53089a1` | Yes, 888 | Ancillary stationary attempt6.310s;0mm. Direction/build unverified. |
| `20261026_log_517_cw.txt` | `b3d26a306fb01326513e543360811a25c99786e5916cda5a1c02135f0c5bd047` | Yes, 887 | Ancillary stationary attempt6.588s;0mm. Direction/build unverified. |
| `20261026_log_518_cw.txt` | `9c77a3161e674346997d01483f4650dca510e9669ba7f444ebdcb01d0a0f42ae` | Yes, 85526 | User CW1: exited, then held before RED in start section.123 RED projection records all outside_snap; late projected outer start seat5 is excluded in official inner-row mode. This is association rejection, not absent RED pixels. |
| `20261026_log_519_cw.txt` | `bd96a993dc37ed501052923a8608fe680a1ccad51926a811df0c5845875e7463` | Yes, 50289 | User CW2: looked out of bay, stopped. Exit diagnostic completes; start scout ends Discovery unresolved and locks drive. No hardware camera failure established. |
| `20261026_log_520_cw.txt` | `bacc04ed965a632e3395282aae2d68bd60aa892e6e564c55ddf1df721ce0c563` | Yes, 122889 | User CW3: reports missed RED/GREEN then stop. RED projection records exist; GREEN is production-valid in discovery traces but unassigned. Holds at S1station0; new unmapped_foreground_hold masks hold_expired trace reason, preventing the existing corner recovery trigger. Minimal subsequent source fix restores that existing trigger; no upload during camera comparison. |

See `simulation/RETURNED_RUNS_518_520_CAMERA_REVIEW.md`. The subsequent stationary
RED/GREEN comparison completed with drive enable OFF. Camera phase recovery and
range calibration were implemented and final M7 uploaded successfully; camera
originals and firmware hashes are in `../camera_diagnostics/README.md`. The
table describes the returned original runs, before these changes. No new
driving gates or HSV/shape widening; normal CW/CCW acceptance remains pending.

## 2026-10-26: returned CW521--522, after camera calibration

Both complete saved source files copied unchanged and SHA-256 checked against
the USB originals. Receipt-date filenames; source clock is unreliable.
Both turn=-1,build `Oct_26_2026_21_02_12`,targets30/36/26/84mm. This corresponds
to the last uploaded M7 SHA-256
`53a7bedc5e7b65849c267c568231a645db5a153d4e62b526793dcb1be1f77328`;
installed flash was not read back. Both include RUN_END,truncated=0, but522
has LOG BUFFER OVERFLOW: its saved file is complete,its telemetry is incomplete.

| File | SHA-256 | Source bytes | Physical report / measured findings |
| --- | --- | --- | --- |
| `20261026_log_521_cw.txt` | `f1c899ab429ca8655a866b2d4a009e9432e748312e164f8d64e9d1f2d9bc0791` | 78189 | User: odd first corner,reverse/return,then contact with outer-row GREEN after first corner. GREEN7 confirmed during reverse observation; returns128.2mm to old stop pose before resuming. Stall follows. CW,40712ms to manual pause. |
| `20261026_log_522_cw.txt` | `92c4297dab21f8c6021bdd8d5e86fdafe1576ad3431368155cc014fbcbaad115` | 196322 | User: repeated stops,finally held near GREEN in third section after start. Logged S3station2 UNKNOWN hold expires at100834ms; no middle/end recovery exists in returned firmware. GREEN at right edge,foot238;both expected seat ranges208/216mm are below230mm local view minimum. Paused at143680ms; buffer overflow prevents full late telemetry. |

521/522 were initially inspected before copying; source stick was briefly
removed. It was then reconnected and both full originals secured/verified.
No reconstructed excerpts are substituted. Camera resync counts were not in
these driving logs; do not attribute the failures to resync from missing data.

## 2026-10-26: returned CCW523--524; third reported run not yet saved

Original saved files secured FIRST,unchanged,and SHA-256 checked against USB.
Both turn=1,parking-object build `Oct_26_2026_21_02_12`,targets30/36/26/84mm.
Correction after525--527: this timestamp identifies one compilation unit and
does not uniquely identify the whole incremental firmware. Exact whole-build
identity/upload of `6a07252e...` is unverified; no installed-flash readback.
Sequence-to-physical-report mapping is
523:first,524:second based on receipt order and compatible observed outcomes;
no third file525 was present. User says third may not have been saved.

| Evidence file | Original source bytes | SHA-256 | Completeness / physical report |
| --- | --- | --- | --- |
| `20261026_log_523_ccw_excerpt.txt` | 38605 | `3250deee29e6d1757b10d777ddb2974eff8a858abcb301b6a21a9f7052ac2e1c` | Exact entire available source file log_523,not a complete run: RUN_START and completed exit diagnostics present,RUN_END/manual-stop tail missing. Excerpt suffix explicitly marks missing run ending. User:first CCW held while leaving. Logged physical exit completes26519ms; primary start observation and stationary retry both UNKNOWN; motor locked after retry. |
| `20261026_log_524_ccw.txt` | 55647 | `8b58668e1f2051e3bfd38c17e0a6095b1af2344dc81e6325e58333da90e3de21` | Complete saved run,RUN_END/truncated0,no overflow. User:second CCW exits but operator physically obstructs driving; INVALID driving acceptance. GREEN5 confirmed,checked connector completes; later stall must not be used as evidence of autonomous route failure. |

No third-run excerpt has been fabricated. User was asked whether normal save
could recover its RAM log before a reset/new run. No vehicle movement,upload
or firmware edits requested/performed during this read-only investigation.

## 2026-10-26: CW525--527 repeated GREEN contact

All three complete originals copied FIRST,unchanged,and USB/disk SHA checked.
User reports three identical CW approaches straight into GREEN after first
corner. All turn=-1,RUN_END/truncated0,no overflow. All confirm/inject GREEN7,
resume from reverse observation,index16,then stall at routeindex28.

| File | Bytes | SHA-256 | Report / evidence |
| --- | --- | --- | --- |
| `20261026_log_525_cw.txt` | 63707 | `37da5a65521b318fa03e301ae5c4b9403158f9ec4deea99aba7dc433ab960a22` | CW1: GREEN contact. After resolved view,gyro poll gap819ms; final stall260mm/s; RUN_END38963ms. |
| `20261026_log_526_cw.txt` | 64235 | `fd6f894d79c516036f43b353281819dc8f118345503bc96ccc63ed06c0213883` | CW2: same contact. Poll gap812ms after resolved view; stall260mm/s; RUN_END40318ms. |
| `20261026_log_527_cw.txt` | 68595 | `e261e80eaa1df8f0d198329cb8b6d37fcf2e882b1fdd67f5559cd9dc0bfbc5cf` | CW3: same contact. Poll gap814ms after resolved view; stall260mm/s; RUN_END42457ms. |

Firmware identity caveat:unchanged parking diagnostic object still prints
`Oct_26_2026_21_02_12`,while changed route prints `Oct 26 2026_21:34:48` and
new DRIVE_CAM/resume-from-view features are present. __DATE__/__TIME__ identify
translation-unit compilation,not a whole incremental build. This batch is
compatible with6a07252e candidate;installed flash was not read back. A single
old parking header must not be used to reject newer route identity.
No complete images or direct servo-enable telemetry in these originals. Gyro
gap correlation and source-level frozen-servo reproduction distinguish the
new blocking preflight/actuator-resume regression from absent GREEN pixels.

Delivered source correction and test limitations:`simulation/FIXES_525_527.md`.
M7-only build SUCCESS,SHA-256
`3c32252fb12c7f7afda474d717aa117a1377b1edaa2a2896ed3eb9991cda6559`;
not uploaded by agent,user's personal-upload preference retained. Existing
parking/pose/reversal/braking diagnostics preserved;no new sensor sweep.

## 2026-10-26: CCW528--530 start approach failures

All three saved originals secured FIRST,unchanged,and SHA-256 matched to USB.
User:first/third unusual driven arc;all3fail. User suggests earlier reverse
observation and notes front parking-section pillar can only occupy inner row.
All turn=1,parking-object timestamp21:02:12 is not a whole-firmware identity.
DRIVE_CAM lacks speed/steer/enabled fields added to3c32252e;exact returned image
hash unverified. Do not equate earlier object timestamps with full firmware.

| File | Source bytes | SHA-256 | Physical report / logged outcome |
| --- | --- | --- | --- |
| `20261026_log_528_ccw.txt` | 43794 | `69186f1022677f4d33c4eecd3163af2ff106c913914a89c7afd51a66f5931689` | CCW1 unusual arc. GREEN5 confirms,then connector turns to135.68deg before steering43.247deg tracking rejection at progress10/23. Gyro poll gaps372/314/216ms. RUN_END/truncated0,no overflow. |
| `20261026_log_529_ccw_excerpt.txt` | 35823 | `547721e6da8648d3e37ef5bad51df86ecf99de45905be5de54c23f6358dda4e7` | Exact whole available source log_529;run ending/RUN_END missing,so excerpt suffix. CCW2 UNKNOWN at primary/retry,same view263.6,-1205.2,43.5deg;no_silhouette,then motor locked. No full-run ending claimed. |
| `20261026_log_530_ccw.txt` | 44143 | `a1a7c351b0adbd21d4fae24e060e3d8a51dd89cbf9251659f64a224e63050bad` | CCW3 unusual arc. Local GREEN5 confirms3frames,x85/foot184,range267/snap23. Connector reaches144.33deg then steering42.007deg rejection10/24. Gyro poll gaps392/326/309/230ms. RUN_END/truncated0,no overflow. |

Green row is already restricted to inner seats in official parking-section
mapping. First/third associate GREEN correctly;do not force a new row inference
to hide their subsequent connector/control failure. No failed-view full images.


## 2026-10-27: newly received logs 531--540

All ten originals copied before analysis and SHA-256 verified against source.
Archive date is receipt date; removable-media filesystem dates are invalid
(year 2098), so actual run dates are not inferred from them. Direction comes
from PARK_DIAG_CONFIG: turn=-1 CW, turn=1 CCW. Each has RUN_START and RUN_END;
truncated=0. Object compilation timestamps are not whole-image identities.
531--534 connector object Oct 26 2026_21:54:57; 535--540 Oct 26 2026_22:15:28.
User confirms these are yesterday's previously described runs, BEFORE the latest
M7/M4 corrections, not a test of the latest images. Exact flashed hashes and
individual CCW contact/operator-intervention mapping remain unverified.
Earlier user report tentatively matches 531--534: four CW, second GREEN stop,
three failed parking entries; sequence assignment remains provisional.

| Evidence file | Source bytes | SHA-256 | Completeness / outcome |
| --- | --- | --- | --- |
| `20261027_log_531_cw.txt` | 149613 | `255f924381d05e28d10b5bf75f5967281eea22dc9b2ad590c50af4ad9152f264` | Complete original source, RUN_END/truncated=0; aborted reason=dual_marker_geometry. Physical outcome mapping pending. |
| `20261027_log_532_cw.txt` | 91607 | `89f73cd3449f791337c0ea16ea0a6282725e2f684fef7e4cc5644bc634daa8ae` | Complete original source, RUN_END/truncated=0; stopped reason=mode_pause. Physical outcome mapping pending. |
| `20261027_log_533_cw.txt` | 163618 | `9c494241effb7992a329fcbf6fe1f6f0946a471d82a8914083b6ebf68e704704` | Complete original source, RUN_END/truncated=0; aborted reason=dual_marker_geometry. Physical outcome mapping pending. |
| `20261027_log_534_cw.txt` | 176586 | `2f448b278b3f95527f541ab48c2a83d67b7c2e96584edb6a21662dd9f446a456` | Complete original source, RUN_END/truncated=0; aborted reason=dual_marker_geometry. Physical outcome mapping pending. |
| `20261027_log_535_ccw.txt` | 51584 | `9ca005f00ac24d3099ac1b8ed7c781a3a823c9447983d31f34e035a7d0e16a71` | Complete original source, RUN_END/truncated=0; stopped reason=mode_pause. Physical outcome mapping pending. |
| `20261027_log_536_ccw.txt` | 50629 | `f40894ad0b8924f9538376c96c8ccfb40a934705291e25caee490334ffd0a1ec` | Complete original source, RUN_END/truncated=0; stopped reason=mode_pause. Physical outcome mapping pending. |
| `20261027_log_537_ccw.txt` | 60027 | `87e1e4f6edb3ff7320b5d34b4aa30097db539812377d8570bfe5c60d0757a98b` | Complete original source, RUN_END/truncated=0; stopped reason=mode_pause. Physical outcome mapping pending. |
| `20261027_log_538_ccw.txt` | 6082 | `627a1a17798aa86381cb48e640c795742510ca30a907ed8084e730ba60164d3f` | Complete original source, RUN_END/truncated=0; stopped reason=mode_pause. Physical outcome mapping pending. |
| `20261027_log_539_ccw.txt` | 55361 | `b5e0f4040a36d44bd4b14fdc8ffe264b2b765839e9029153183ec0b47f84648c` | Complete original source, RUN_END/truncated=0; stopped reason=mode_pause. Physical outcome mapping pending. |
| `20261027_log_540_ccw.txt` | 74786 | `ede221da3169b3f14a9358a9df6f92602a0a9ac1ccc94377d37f330f8687bb25` | Complete original source, RUN_END/truncated=0; stopped reason=mode_pause. Physical outcome mapping pending. |

Batch assessment: `../../RETURNED_RUNS_531_540_REVIEW_20261027.md`. Nine exits
complete without diagnostic truncation; 538 ends during rear positioning.
No log overflow or missing run ending in this batch. No active ToF correction,
servo-centre change, extra sensing sweep, agent upload or drive command.


## 2026-10-27: CW541--543 layout B

Originals secured FIRST with SHA-256 verification before analysis; user
notified immediately that media could be removed. Layout B unchanged from
yesterday; all three repeat unwanted bend towards GREEN after start section,
recovery and permanent stop. Object build timestamps only, whole flashed M7/M4
hashes not read back. Run endings and direction verified below.

| Evidence file | Source bytes | SHA-256 | Completeness / physical report |
| --- | --- | --- | --- |
| `20261027_log_541_cw.txt` | 62370 | `3ffe2ef3f32ba24cecf560697992c32d9c7bcf2eacc660d38af8dba557c9c51f` | Complete RUN_START/RUN_END, truncated=0; user: layout B unchanged, all three stop before GREEN after start section after unwanted bend/recovery. |
| `20261027_log_542_cw.txt` | 64211 | `95791c994279aa29d7eb0dcd491113c62144d7f6bc3fe67737a3c5f20b70e121` | Complete RUN_START/RUN_END, truncated=0; user: layout B unchanged, all three stop before GREEN after start section after unwanted bend/recovery. |
| `20261027_log_543_cw.txt` | 64943 | `08661f62a3170cbf291ead87b4efded1f26d3958eaae3bc17619e67acae2bce9` | Complete RUN_START/RUN_END, truncated=0; user: layout B unchanged, all three stop before GREEN after start section after unwanted bend/recovery. |

Assessment and correction: `../../CW_CORNER_541_543_FIX_20261027.md`.
All three completed exits; GREEN early valid but rail mapping rejected.
541/542 reverse command with zero encoder delta; 543 view preflight rejection.
No gyro restart or long-poll warning. New M7 correction built, not uploaded;
source reproduction does not substitute for a new physical acceptance run.


## 2026-10-27: CCW544 and stalled start attempts 545--546

All originals copied FIRST, unchanged, SHA-256 checked; user notified immediately
that media could be removed. User confirms these runs occurred DURING the prior
CW correction work, BEFORE its new M7 image. Not a validation of455d7a77.
544 old control_revision/14:01:27 path object; exact flashed image hash unknown.
545/546 have no direction/config header: filenames reflect intended CCW from
user context, not a firmware-determined direction. No run tail missing.

| Evidence file | Bytes | SHA-256 | Completeness / physical report |
| --- | --- | --- | --- |
| `20261027_log_544_ccw.txt` | 75852 | `feeb368d78742f965b0c5ae93949b2de903ccc40621ce196e67cd5de23147852` | Complete RUN_START/RUN_END, truncated=0. CCW confirmed turn=1. User: first GREEN bypassed as RED, late RED hold. |
| `20261027_log_545_ccw.txt` | 889 | `ac8ae79862ea2ad7d0bde071ef50cea595abb175a6ebedd680db31441f3f35a2` | Complete RUN_START/RUN_END, truncated=0. CCW start attempt by user context; firmware never selected direction. No movement; right ToF unavailable. |
| `20261027_log_546_ccw.txt` | 890 | `ccafbd8fb1ced71fce9f3d8502baf2d0e596bcf9e60e3337071f23cca0fb68d1` | Complete RUN_START/RUN_END, truncated=0. CCW start attempt by user context; firmware never selected direction. No movement; right ToF unavailable. |

Stationary cached-value excerpts (drive release off, v/P only; no motion,
reset, firmware upload, or dedicated ToF mode). Before contact fix: RIGHT=-1
five times; after user fixes loose contact: RIGHT=161 then fresh9999 gaps,
not unavailable. User confirms start fault physically repaired. Automatic
sensor recovery experiment removed; original startup/sensing control retained.

| Diagnostic excerpt | Bytes | SHA-256 | Scope |
| --- | --- | --- | --- |
| `20261027_tof_contact_before_excerpt.txt` | 613 | `9c7113531839d5c86fb89f6bd76bcc127ab18d6a13193c4a99dd3163c36ae395` | Exact stationary serial capture; diagnostic excerpt, not a run. |
| `20261027_tof_contact_after_excerpt.txt` | 119 | `a49fd992f5c10a98e19b159c4650788702c43ee901de1dea4f20a195f176cbaf` | Exact stationary serial capture; diagnostic excerpt, not a run. |


## 2026-10-27: CW547--550 repeated third-section GREEN and parking difficulty

All four newest originals copied FIRST and verified against source SHA-256;
user immediately notified that media could be removed. User reports three CW
runs with improved first GREEN bypass, late third-section GREEN requiring
reverse, slow first RED approach, less visible exit offset and manual stop
during final parking. Four files exist; map the additional shorter attempt
from telemetry rather than assuming four complete laps. Full flashed hashes
not read back.

| Evidence file | Bytes | SHA-256 | Completeness / physical report |
| --- | --- | --- | --- |
| `20261027_log_547_cw.txt` | 164846 | `a27a023f43ebb2463eccc17d3a43bf47274ca718631799c484053449f183fae3` | Complete original saved file; RUN_START/RUN_END present, truncated=0. Intended CW; per-file outcome assessed below. |
| `20261027_log_548_cw.txt` | 30647 | `5f8208e81bd0c35a9110dcfd2d7507af22754278378f59991fa9bac330ad7a7d` | Complete original saved file; RUN_START/RUN_END present, truncated=0. Intended CW; per-file outcome assessed below. |
| `20261027_log_549_cw.txt` | 156826 | `157727ce47aefa43a462bfe39b808f0b3f108d0a9e20fcf6294d60108b228630` | Complete original saved file; RUN_START/RUN_END present, truncated=0. Intended CW; per-file outcome assessed below. |
| `20261027_log_550_cw.txt` | 180173 | `5f655e7a736956d7cb4da0e125daf16fee03c91f8d364ae943bf8ddde0cf6d93` | Complete original saved file; RUN_START/RUN_END present, truncated=0. Intended CW; per-file outcome assessed below. |


## 2026-10-27: CCW551--555 follow-up batch

Copied FIRST and SHA-256 verified; user immediately notified of safe removal.
User reports repeated stops during exit/GREEN view, penultimate run proceeds
but reverses before second GREEN and stops in start section; final run stops
after parking exit. Firmware changes being prepared were not uploaded by agent.

| Evidence file | Bytes | SHA-256 | Completeness / physical report |
| --- | --- | --- | --- |
| `20261027_log_551_ccw.txt` | 103799 | `a3dbb836bf546c1519ebda69f90468e2c83c2b38a3aee3d90a0817b995dda312` | Complete saved original; CCW reported by user; detailed outcome below. |
| `20261027_log_552_ccw.txt` | 15468 | `0ec519cc1981625d365e8d2ed6c7d2eea35c25bfe2c928fd2649610174cdacec` | Complete saved original; CCW reported by user; detailed outcome below. |
| `20261027_log_553_ccw.txt` | 97769 | `d186513abf4688b728624c1464f5fe1aa3a8b6fa867651140743c495bd2c76c4` | Complete saved original; CCW reported by user; detailed outcome below. |
| `20261027_log_554_ccw.txt` | 115074 | `afd707a06a57885f04170fadb541c17e040fb4bf9c828af0b82be29d2658d23e` | Complete saved original; CCW reported by user; detailed outcome below. |
| `20261027_log_555_ccw.txt` | 129822 | `da9e3d39fc820fedcab24c4b4d08945b7c5025627b731a1053c10979f307d804` | Complete saved original; CCW reported by user; detailed outcome below. |

All nine files: RUN_START/RUN_END, truncated=0, no overflow; directions confirmed
by turn=-1 (547--550) and turn=1 (551--555). Old path revision
20261027_ccw_corner_usb_save, parking build Oct_27_2026_18_52_33; exact flashed
hash unknown. 547/549/550 finish three laps then parking is manually stopped;
548 aborts fifth exit on swept geometry. 551/553/555 confirm GREEN5 but reject
connector at its initial parking-piece footprint. 552 watchdog pauses exit
segment2 despite 8.3mm progress in one second. 554 returns after one lap but
contradictory GREEN7/RED9/RED10 prevents lap2. Seven complete/untruncated exits;
552 diagnostic finish absent because watchdog pauses, not because file is partial.
Assessment: `../../CW_CCW_547_555_REVIEW_20261027.md`. Normal next-run physical
acceptance pending; no agent drive/upload or dedicated sensing sweep.


## 2026-10-27: CW556--558 repeated early GREEN failures

Originals copied FIRST and SHA-256 verified; user immediately notified of safe
removal. User reports first attempt wrong GREEN bypass after start corner and
late GREEN hold; subsequent attempts stop at early GREEN. File mapping verified
from telemetry below; 557 is a short exit-only attempt. New firmware revision
20261027_cw_ccw_scan_parking, parking build19:43:55; flashed image hash unknown.

| Evidence file | Bytes | SHA-256 | Completeness |
| --- | --- | --- | --- |
| `20261027_log_556_cw.txt` | 91933 | `f70d676aaedf77b59e9f128831bcde4198fe4873efab03e9692ec3c0e5b5ca1b` | Complete saved original, RUN_START/RUN_END, truncated=0; CW turn=-1. |
| `20261027_log_557_cw.txt` | 11857 | `f7f44f0b9556b1d12064e2cc6fcf84cf1a3c26f58cb989627bd20f5502642c34` | Complete saved original, RUN_START/RUN_END, truncated=0; CW turn=-1. |
| `20261027_log_558_cw.txt` | 82235 | `6b1d3f98f6cca9ceaf726b0aa4983b32995a15cd0e5b9ed288c947a5b10f57f9` | Complete saved original, RUN_START/RUN_END, truncated=0; CW turn=-1. |


## 2026-10-27: CW559 and CCW560--562 after successful departures

Originals copied FIRST and SHA-256 verified; immediate safe-removal notice.
User reports all departures succeed; third-section steering strongly left after
RED, late GREEN hold in first/third reported runs, contact with GREEN in second.
Four saved files exist; map individual attempts from telemetry. Firmware not
changed/uploaded by agent between these tests.

| Evidence file | Bytes | SHA-256 | Completeness |
| --- | --- | --- | --- |
| `20261027_log_559_cw.txt` | 93922 | `272fcc098e45beb1c5da89df0ae02581c331a2ec2cf1198dd1cbf8493d8e8a69` | Complete saved original; direction CW from telemetry; early GREEN recovery hold. |
| `20261027_log_560_ccw.txt` | 93160 | `974884d4123de61f1f381b8d1a86108bde01a7ca6b34ee89119f49298115a332` | Complete saved original; user CCW report; outcome below. |
| `20261027_log_561_ccw.txt` | 84083 | `92ab23565965df79cf624a0955fe605a92424e8d732aa088d1e3023b088a94e6` | Complete saved original; user CCW report; outcome below. |
| `20261027_log_562_ccw.txt` | 123213 | `f1963a24cce2c2e7be38b3cc7383d51a95e4a12985f20bbb5da54780db58bfa4` | Complete saved original; user CCW report; outcome below. |

556: early GREEN row rejection/wrong bypass; later phantom GREEN20 and hold.
557: interrupted exit segment2; no complete driven-lap outcome inferred.
558/559: early GREEN acquisition/recovery hold. 560/562: phantom GREEN20 after
RED19, late GREEN22 recovery hold. 561: phantom RED23 and opposite-GREEN conflict,
then low-progress watchdog stop, consistent with reported contact. All seven
files have RUN_START/RUN_END and truncated=0; six exits complete,557 interrupted.
All identify revision20261027_cw_ccw_scan_parking, parking build19:43:55;
flashed binary hash unknown. Full assessment and verification:
[GREEN_556_562_REVIEW_20261027.md](../../reports/GREEN_556_562_REVIEW_20261027.md).
Normal physical validation of the new revision remains pending.


## 2026-10-27: CW563--566 first/third GREEN and parking

Originals copied FIRST and SHA-256 verified before safe-removal notice.
User reports early GREEN contacts/near passes, late third GREEN recovery,
a corner stop and one parallel parking approach that did not enter.

| Evidence file | Bytes | SHA-256 | Completeness |
| --- | --- | --- | --- |
| `20261027_log_563_cw.txt` | 64347 | `78719cf9847ca4addb1c0c97df4651cbf44873d668822ad4fefbe5ad6c369651` | Complete saved original; analysis below. |
| `20261027_log_564_cw.txt` | 171447 | `72fb479284dc4f81e2ea222b8aa5c7ae8386b063a9a2ad1752c82a92a75b2df4` | Complete saved original; analysis below. |
| `20261027_log_565_cw.txt` | 112873 | `351d9338199391250d15f10653813d19e634b87d98d0a3edde92b743d65c0e36` | Complete saved original; analysis below. |
| `20261027_log_566_cw.txt` | 65293 | `62ffb6c44f1eeb549cf29a0d751ddf36c9c6a8a21e620ef53e8efa99789f839c` | Complete saved original; analysis below. |


## 2026-10-27: subsequent CCW567--569

Originals copied FIRST and SHA-256 verified; immediate safe-removal notice.
User reports third-section GREEN and following RED require reversals; late RED
holds in later attempts; first attempt did not approach the parking bay correctly.
Firmware same as CW batch; file/attempt mapping must follow telemetry.

| Evidence file | Bytes | SHA-256 | Completeness |
| --- | --- | --- | --- |
| `20261027_log_567_ccw.txt` | 161011 | `e1cbb0c6089e114677a8987e7912ea9926aa9aa8b54ba428dfeb637b61563915` | Complete saved original; mapping/outcome below. |
| `20261027_log_568_ccw.txt` | 1795 | `82db3ab1f3c5a457ce72a17278509266889989c16fb87c3ed0c32af1d1b0e59d` | Complete saved original; mapping/outcome below. |
| `20261027_log_569_ccw.txt` | 125562 | `65f64a08183672fd2ceac6cf77b40af09da599463dbc5aae076ff31bac9045ee` | Complete saved original; mapping/outcome below. |

563/566: wrong firstGREEN seat6, contact consistent with user report and later
stall.564: correct7, thirdGREEN reverse recovery, three-lap return reaches parking
but capture_pose_gate aborts after registration.565: firstGREEN recovered; then
unknown station6 remains unresolved after110mm reverse and locks.567: completes
route then initial_outer_wall_reference abort;569: GREEN22 recovery then lateRED1
confirmation, recovery join unsafe and locks.568:96ms interrupted rear-positioning
start, not a complete CCW lap. No mapping of three full CCW reports to these
three files is asserted. All have saved boundaries/truncated0; six exits complete.
Path revision20261027_green_row_bearing_layout, park header19:43:55, flashedSHA
unknown. Report: [CW_CCW_563_569_REVIEW_20261027.md](../../reports/CW_CCW_563_569_REVIEW_20261027.md).
Physical validation of the next revision and CW565 unknown-seat hold pending.

CW565 follow-up (same original/hash; no new run): scoped the lower-image fragment
veto to individual seat bearings while preserving upper-silhouette and local
occupancy guards. [Assessment](../../reports/CORNER_HOLD_565_FIX_20261027.md).
Software reproducer and27 safe joins pass; actual scene acceptance still pending.

## 2026-10-29: CW570--571 and CCW572--575

Complete original files copied before analysis; SHA-256 verified against the
source and the portable archive. Safe-removal notices sent immediately for
both batches. All six report path revision `20261028_parking_wall_reference`;
diagnostic build header `Oct_27_2026_19_43_55` alone is not firmware identity.
Installed binary hash is unknown. No robot upload or movement by the agent.

| Evidence file | Bytes | SHA-256 | Completeness / observed outcome |
| --- | ---: | --- | --- |
| `20261029_log_570_cw.txt` | 42220 | `deed70efa781e8faf0ce6965f1d6afc1302b668b3d62c9a5213132cda28370c9` | Complete saved original, full exit diagnostics; scout unresolved/locked; no RUN_END record. |
| `20261029_log_571_cw.txt` | 157372 | `2c1186ea7501a53271e848d071d47c1abdd18ac945d959771398c1996d306ca4` | Complete saved original, full exit; three laps then capture_alignment_preflight abort; RUN_END truncated=0. |
| `20261029_log_572_ccw.txt` | 105924 | `25d8d3d98d65232bf6c46a8c2ccc8acfd23c6c12a3671f452cfea661371be9ef` | Complete saved original; reaches S2 station1, extra-arc tracking lock; manually paused; RUN_END truncated=0. |
| `20261029_log_573_ccw.txt` | 46693 | `32080ebc1d85a3f59591577e6561efe9beb177159b4a5c64577253d627cc222c` | Complete saved original; front GREEN5 unresolved after stationary retry; manually paused; RUN_END truncated=0. |
| `20261029_log_574_ccw.txt` | 45737 | `b16742bf377977628ee8791b1df282f99a3314162a35f8e10f41da8c9a389740` | Complete saved original; front GREEN5 unresolved after stationary retry; manually paused; RUN_END truncated=0. |
| `20261029_log_575_ccw.txt` | 70218 | `deacf7d82f1d415882e95ea1e396dbd696320d572b7bea300a199cca911704bd` | Complete saved original; front GREEN5 unresolved after stationary retry; manually paused; RUN_END truncated=0. |

Physical reports: CW570 green not acquired and stopped, user requests a little
more reverse; CW571 remaining route perfect, reaches pink rails then stops.
CCW572 leaves well and reaches the single outer GREEN opposite the start,
reverses twice then stops. CCW573--575 stop after leaving because GREEN is not
confirmed. Complete saved file does not imply a completed challenge. Images of
these attempts were not recorded; segmentation causes cannot be fully replayed.
Review and next steps: [CW_CCW_570_575_REVIEW_20261029.md](../../reports/CW_CCW_570_575_REVIEW_20261029.md).

## 2026-10-29: CW576 false middle confirmation / skipped scout

Copied original before analysis, SHA-256 verified, immediate safe-removal notice
sent. Complete saved original `20261029_log_576_cw.txt`,83786bytes, SHA-256
`9bde5b821ded2d659f3c1f84257e7788f2749475b3691d6bf2e084d5910c0a5a`.
Revision `20261029_start_view_park_reference`, connector build
`Oct 29 2026_19:21:10`; installed binary hash unknown. Full exit diagnostics;
RUN_END manually paused, truncated0, elapsed70311ms, lap/index1/38 near end.
Physical report: CW start completely wrong, green not inspected, red missed,
poor driving overall. No exact contact location or image supplied.

At248028ms local green fallback confirms middle seat2, measured foot208px
versus roughly164px expected, range232mm versus317mm expected, snap86mm.
At the primary-result/scout transition the phase flags briefly allow layout
inference: the middle confirmation infers station0 empty and skips its scout.
RED4 gets one accepted vote at252211ms but no confirmation; the inferred layout
also removes the normal unresolved-place hold. No explicit RED4 conflict
rejection is logged. The new CW extension
never runs in this file. Original source of those green pixels is unrecorded.
Review: [CW_576_START_REGRESSION_20261029.md](../../reports/CW_576_START_REGRESSION_20261029.md).

## 2026-10-29: CCW577--581, two good routes and two pre-scan holds

Copied originals FIRST, SHA-256 compared source/copies, immediate safe-removal
notice sent. All are complete saved files with RUN_END truncated0. Installed
binary hash unknown.577/579/580/581 path revision is
`20261029_start_view_park_reference`: these runs predate the later576 foot/scout
correction.578 never reaches a path header; its exact path revision is unproved.

| Evidence file | Bytes | SHA-256 | Run outcome |
| --- | ---: | --- | --- |
| `20261029_log_577_ccw.txt` | 149113 | `d8dbb0c2d047eb860ac6f56899873c263507e99dc113d2c7d808e317f6bcf5fd` | Full exit, first GREEN confirmed, three laps; parking initial_outer_wall_reference abort. |
| `20261029_log_578_ccw.txt` | 4609 | `39e40351286551c60bca83eb51f1e9787cfc56ae853afe7329a19c96e5cd040f` | Manually paused1143ms after RUN_START at first exit drive; exit sequence incomplete. |
| `20261029_log_579_ccw.txt` | 174609 | `ccf9e3d8e6b1ed01d69599d3a7ecc3e852702ded8d1c87244ddb007e48cc1ed1` | Full exit, first GREEN confirmed, three laps; parking scan_alignment_limit abort. |
| `20261029_log_580_ccw.txt` | 45419 | `131e305fee9e590870cf67f467cacce22a21757a781182346c0df937492598d8` | Full exit; first observation arc preflight rejected; manually paused after hold. |
| `20261029_log_581_ccw.txt` | 44823 | `71c1d168eb48cd00ee1f40f1654f5b72a023f77c9cfe64b9474b87bb6963361e` | Full exit; first observation arc preflight rejected; manually paused after hold. |

Physical report: two excellent runs through course except parking; other runs
leave then stop. Parking investigation explicitly deferred by user. Log578 is
a complete saved original of an interrupted attempt, not a complete exit or
an excerpt.580/581 stop before camera scan, not because a scan failed to see
GREEN. Detailed comparison: [CCW_577_581_REVIEW_20261029.md](../../reports/CCW_577_581_REVIEW_20261029.md).

## 2026-10-29/30: CW582--586 and CCW/CW587--593

Originals copied before analysis, verified source-to-copy by SHA-256, and the
safe-removal notice was sent immediately for the newly returned batch. All
582--591 and 593 files have saved RUN_END/truncated0 boundaries. 592 is a complete
saved 1.99 s interrupted attempt, not a complete exit or course run. Installed
binary hashes are unknown.582--586 report `20261029_start_foot_scout_evidence`;
587--593 report `20261029_ccw_scan_fit` where a path header exists.

| Evidence file | Bytes | SHA-256 | Run outcome |
| --- | ---: | --- | --- |
| `20261029_log_582_cw.txt` | 176869 | `2b81ad9ad3814d9cda03ed753fbc6cbcf1736222606f67ba92eedb54360a456b` | Full exit and three laps; first post-start GREEN ultimately confirms outer seat7; parking segment_pose_gate abort. |
| `20261029_log_583_cw.txt` | 78860 | `7b7673bdc1a43b99cd7806676ef4042a3d9aac261976d52fa1fccad82f955ff1` | Full exit; ambiguous early view confirms inner seat6, later outer-seat7 view rejected; manually paused. |
| `20261029_log_584_cw.txt` | 91525 | `e7ae5fce1061614809f62bb6fcd5c71f00abe37855a33968bc989cf0fa8bf627` | Full exit; separated seat7 votes and broad close GREEN, recovery unresolved; manually paused. |
| `20261029_log_585_cw.txt` | 89680 | `35feb75f82dbde59c423e63843711f4143a2730d30affd06792af2c4779bb3db` | Full exit; separated seat7 votes, no safe recovery view; manually paused. |
| `20261029_log_586_cw.txt` | 111374 | `08e1a904e6c81096558a5904b1511e4bd90908401a8b1abea66df6c82cc21741` | Full exit; seat7 vote then broad close GREEN, no safe recovery view; manually paused. |
| `20261030_log_587_ccw.txt` | 42181 | `8912b07e387519aacb51aed9d8779e0a3f54a570b8cf80310d0d4612fb2a837c` | Completed/untruncated exit; true end GREEN repeatedly rejected by start-foot gate; stationary retry unresolved. |
| `20261030_log_588_ccw.txt` | 94651 | `87dcbf5d8314c3fe95cd930c6f324f81ed88139564ab9b2358932f822dbe3201` | Completed/untruncated exit; several correct confirmations, then broad RED at deferred start station remains unresolved after bounded recovery. |
| `20261030_log_589_ccw.txt` | 53279 | `001909634ef6c8aafe53e5d6bab642dfa484d05d1c4257a756eaeee22ffcd12b` | Completed/untruncated exit; one matched GREEN candidate followed by false CLEAR and unprotected connector; later stall/manual pause. |
| `20261030_log_590_cw.txt` | 174528 | `d77d85e26f84073e97f0a3a0296803a4eedeb108970f20509d0c646b2b05b237` | Completed/untruncated exit, complete first-lap map and three laps; parking capture_pose_gate abort. |
| `20261030_log_591_cw.txt` | 85807 | `d751049490d655adfbea8ead7947c4ada71e5fe77ca0bbe1eb6609fcbf57ef0b` | Completed/untruncated exit and lap1; later lap2 zero-progress stall, physical cause not identifiable from log alone. |
| `20261030_log_592_cw.txt` | 4759 | `799ddbf6eb0223c87158effad88ee98706ace5abbc354c41a4dd9e6659bf7cd8` | Complete saved interrupted attempt; manually paused after1.99s during rear settle, exit incomplete. |
| `20261030_log_593_cw.txt` | 186247 | `6521046a2309498e5c1bf07296d09a7ac7aa0709c25c97473e5c6ff6cb13b2bc` | Completed/untruncated exit, complete first-lap map and three laps; parking entry_motion_clearance abort. |

User reports mixed CW/CCW failures in the first arrangement and a successful
course in a later different arrangement; exact report-to-file mapping was not
provided. Phone photos are geometry context, not robot RGB565 evidence. Detailed
analysis and scoped correction:
[RETURNED_RUNS_582_593_REVIEW_20261030.md](../../reports/RETURNED_RUNS_582_593_REVIEW_20261030.md).

## 2026-10-31: CW594--597 and CCW598--601

Collection date is used in filenames; removable-media timestamps are unreliable.
Complete original files copied FIRST and verified by SHA-256 before analysis;
the user was immediately notified that the stick could be unplugged. All eight
saved originals have RUN_END/truncated=0, including interrupted attempts; this
does not mean the physical course or parking completed.
PARK_DIAG_CONFIG reports the translation-unit build Oct_30_2026_20_04_33;
CORNER_VIEW reports Oct30_20:09:49 where present. Installed binary not read back.
Revisions: path20261030_active_lap1_search, exit20261030_corrected_pose,
parking20261030_tight_completion. Physical report and scoped corrections:
[RETURNED_594_601_REVIEW_20261031.md](../../reports/RETURNED_594_601_REVIEW_20261031.md).

| Evidence file | Bytes | SHA-256 | Observed ending |
| --- | ---: | --- | --- |
| `20261031_log_594_cw.txt` | 148955 | `37205da6db23cc9881dc8fde6cfebfb07e8c5add40abbc23d442598270ed3ecb` | Three laps, manual pause after two parking replans; repeated gyro restarts during stopped search. |
| `20261031_log_595_cw.txt` | 49612 | `c128190df7610b7269f449823753532093933994c2611e93b0de1180fc9365f7` | CW scout unknown, motor locked; manually paused. |
| `20261031_log_596_cw.txt` | 193844 | `a352954edc3289b0ee85adf805201864dbb48e210a64f57aed729d488e172aef` | Three laps, parking replans then connector_clearance_gate abort. |
| `20261031_log_597_cw.txt` | 189159 | `e277bfd70343eab57ebf6ba8a5808d425de9fdf591192ba2395fec20f056800b` | Three laps, stopped_scan_wall_reference abort at stable residual about33mm. |
| `20261031_log_598_ccw.txt` | 54721 | `adaa67b3cade5d4c407995a658c3217726320542bf75cc21fefd895f33d58d85` | GREEN5 confirmed, then CCW front-view travel guard; manually paused. |
| `20261031_log_599_ccw.txt` | 86606 | `1406d52ec8ca69d407e4fc0d5e16b1bf8181d71056f75e297154d3bb17c0b859` | CCW primary unknown after stationary retry, motor locked; manually paused. |
| `20261031_log_600_ccw.txt` | 144320 | `2226de0c54a2fedb9b131178c8e21bab8e5409e386e2e78e303dcce8fcf8957f` | Three laps; end GREEN12 inferred middle empty; initial_outer_wall_reference abort at about120mm residual. |
| `20261031_log_601_ccw.txt` | 172650 | `659896556cf8b8474d0413b1a7f6f800716243ce8040525539ccff6c804a6a04` | Three laps; same end GREEN12 inference; scan_motion_clearance abort near first piece. |

## 2026-10-31: follow-up CW603; mixed/open602 excluded

Copied complete originals FIRST, before analysis, with source-before/source-after
and local SHA-256 verification; immediate unplug notice sent. Collection date
used. User reports two obstacle runs, only one saved; the unsaved run cannot be
reconstructed. Open laps are excluded from obstacle/parking conclusions.
603 records path20261031_view_recovery, exit20261031_frame_bracket and
parking20261031_brake_completion. PARK_DIAG_CONFIG translation-unit build is
Oct_30_2026_20_04_33, not a whole-binary identity; installed binary not read back.

| Evidence file | Bytes | SHA-256 | Observed ending |
| --- | ---: | --- | --- |
| `20261031_log_602_cw.txt` | 26385 | `1f4f0dcf8104799bd3cda190bc49f567fbfa3fd7c9416d9d6380fa89ef3c0326` | Mixed file ending OPEN_CHALLENGE completed; excluded per user instruction. Original preserved for provenance. |
| `20261031_log_603_cw.txt` | 171521 | `2d6d19d667c1400830f9638e054d10f573c839a49cd23067a4d2d5bf31c462c8` | Three laps; user reports outer-middle GREEN wrongly assigned first outer seat and parking stall. Saved RUN_END mode_pause/truncated0 after first entry arc and accepted replan; no successful parking record. |

## 2026-10-31: CCW604--606, course recovery and parking contact

Complete saved originals copied FIRST and source-before/source-after/local SHA-256
verified; user immediately told USB can be unplugged. Collection date used.
User:604 missed second GREEN on next straight, stationary unresolved recovery;
605 course otherwise good with reversals, front parking rail contact prevented
completion;606 course good, brushed rail but parked, final photo supplied.
Firmware records view_recovery/frame_bracket/brake_completion, predating the
clipped-depth/practice changes. Installed binary not read back.
605/606 contain logger overflow: these are complete original saved files, but
later detailed telemetry is missing. RUN_END retained separately does not make
their missing parking trace complete or prove contact-free success.

| Evidence file | Bytes | SHA-256 | Observed ending |
| --- | ---: | --- | --- |
| `20261031_log_604_ccw.txt` | 89075 | `58b9c20f1331f13365a08b519fbff81190e30762cb5d5bb3d9ab2f538beba554` | Repeated stationary unresolved corner view, manual pause; RUN_END truncated0. |
| `20261031_log_605_ccw.txt` | 196018 | `4bd9b95f798c632f3103c86fb6916c903fcaa5f244c9d5ff5f671e3489967192` | Three laps; retained RUN_END abort entry_replan_limit; detailed parking tail lost to overflow. User reports front rail contact. |
| `20261031_log_606_ccw.txt` | 196181 | `88dd03ed5ab885812d702933a1400a898c2ad656df062c376c1eb2e9441cbffe` | Three laps; retained RUN_END completed final_parking_stop; detailed parking tail lost to overflow. User reports brushing rail, then parked; photo of final pose. |

Findings, scoped corrections and limitations:
[CCW604--606 review](../../reports/RETURNED_604_606_REVIEW_20261031.md).

## 2026-10-31: successful isolated CW607--609

Complete originals copied FIRST, source-before/source-after/copy SHA-256 checked;
immediate unplug notice sent. Collection date used. User reports parking worked,
but prefers less rear and more front space for steering. Saved files all record
entry-only CW practice, rear_space revision, completed final_parking_stop;
no log overflow. Contact-free status was not individually specified per log.
Installed binary not read back; this is entry-only evidence, not full-course
approach localization or CCW validation.

| Evidence file | Bytes | SHA-256 | Observed ending |
| --- | ---: | --- | --- |
| `20261031_log_607_cw.txt` | 20810 | `ef30ce221d781961e9b6609b9b163b41c56a080c09ce98fbd5fca8c99f1690ab` | Completed CW practice,24.168s,5replans, rear X correction+20.76mm; maximum logged search7201ms. |
| `20261031_log_608_cw.txt` | 17237 | `0554b83b31eb184da68d521770a1856e9bb5a99736da59f566ca59eb70f7e7bf` | Completed CW practice,22.489s,4replans, rear X correction+11.19mm; search7122ms observed. |
| `20261031_log_609_cw.txt` | 18556 | `de8b7f8ccdb4b97397e8598e612f3e410e37dbaa1aba1c0569f8875a40a49a68` | Completed CW practice,20.481s,3replans, rear X corrections-3.42/+4.85mm; escape search5604ms observed. |

Review and return-to-course changes:
[CW607--609 handoff](../../reports/RETURNED_607_609_FULL_RUN_HANDOFF_20261031.md).

## Returned log610: normal-course halt (2026-10-31)

Complete unchanged source archive, 96172 bytes, SHA-256
`f4630b160912c74fc01850b23f64d31dc59133ea78988b4dc05f4a07404ef7bc`.
[Original log610](20261031_log_610_cw.txt). Filename reflects first CW exit
turn=-1; archive also contains later restart(s), including turn=1.
Build reports Oct 31 2026 13:37:20. User reports halt in middle after one lap;
segment attribution and cause under review. No independent installed-binary readback.
Source-before/source-after/copy hashes matched; unplug notice delivered immediately.
Analysis: [log610 boundary review](../../reports/RETURNED_610_LAP_BOUNDARY_20261031.md).
First CW run: cached middle GREEN plus freshly confirmed end pair caused
explicit lap-boundary map hold; manual pause120.505s. Later turn=1 restart
failed rear-positioning verification; placement not known. No evidence overflow.

## Returned full-course batch611--615 (2026-10-31)

Complete unchanged originals; source-before/source-after/copy hashes verified; unplug notice sent before analysis. Report: CW third-section GREEN recovery trouble; CW approach worsens alongside pink pieces; CCW obstacle runs good, one parking successful, last stopped before lot. Parking front tight, rear >=30mm reported; user requests shallower and faster parking. Path exit_vote_hint, parking front_swing_space; installed binary not independently read back.

| Evidence | Bytes | SHA-256 | Direction |
| --- | ---: | --- | --- |
| 20261031_log_611_cw.txt | 87192 | 7ed28a9f16601037c7b41e323dd09a10594e65d9746a364fcf7a4373dd426278 | cw |
| 20261031_log_612_cw.txt | 161033 | b61e60b907b8301e24b9873b994a32f04edb137b3e2916ef578bf0f05412faad | cw |
| 20261031_log_613_cw.txt | 171672 | 243d51f31bd9c3efdd9c2a6cc095361277955fa447770cc9216053481f271590 | cw |
| 20261031_log_614_ccw.txt | 171041 | 8a15b5c9b738eab76236b945c59b43112a032a8526a35d91be9d5eb0083c3485 | ccw |
| 20261031_log_615_ccw.txt | 167872 | d33f0d5353e8002adb32d5b5e2159100500f9fd8567a1d4952513d39328ab52f | ccw |

[611--615 review and validation](../../reports/RETURNED_611_615_REVIEW_20261031.md).
611: CW manual pause after search-arc tracking lock.612: CW manual pause during
parking recovery.613: CW completed,6parking replans,max8825ms search.
614: CCW completed,4parking replans,max8769ms search.615: CCW capture_distance
abort at old15s capture timer. No buffer overflow found in this batch.

## Returned616 (2026-10-31): clearance clarification and GREEN stop

Complete unchanged [log616](20261031_log_616_cw.txt),157693bytes,
SHA-256 `1c68941851e2bb7dfde788998edc1abeccc8e0565993816784f74c29a524df82`.
Source-before/source-after/copy matched; immediate unplug notice delivered.
User: latest parking depth looks better; only about5mm shallower requested.
Straight outside wheel width corrected to123mm. During steering allow about8mm
more front room, use available rear room; do not confuse this with parked-X target.
Abrupt GREEN stop reported; exact event under review.

[616 clearance/latency review](../../reports/RETURNED_616_CLEARANCE_REVIEW_20261031.md).
Recorded CW completion149.001s, no overflow; quick recipe100ms, later radius
replan8806ms. Abrupt approach pause coincides with gyro report timeout and
SPI/SH2 restart (1601ms post-restart poll gap); no corner-view lock recorded.
Firmware path view_reobserve/parking checked_recipe, buildOct31 14:40:53.

## Returned617--618 (2026-10-31), stored without driving analysis

User: first run too far in section2 between GREEN and RED on laps2/3; second CW run perfect. Request pending: another8mm front manoeuvre reserve/use less rear room. Do not investigate or change routes yet; next run is for dense portable pose capture. Source-before/source-after/copy hashes matched; immediate unplug notice delivered.

| Evidence | Bytes | SHA-256 |
| --- | ---: | --- |
| 20261031_log_617_cw.txt | 94809 | 6204b033ca6d5feff0aace85252b714288f3e44753a93137a71669e2ccc4c460 |
| 20261031_log_618_cw.txt | 169762 | cbdc8b2d771cb462e62330636e0e997b528dfccfbe8a501cb232520142291995 |

Header metadata (no route analysis): both logsCW turn=-1, buildOct31 15:03:51,
path view_reobserve;618 reports parking front8_123mm. Installed binary not readback.
Capture instructions: [single-run pose capture](../../reports/POSITION_CAPTURE_ONCE_20261031.md).

## Returned619 (2026-10-31), pending-change followup

Complete unchanged [log619](20261031_log_619_cw.txt),147241bytes,
SHA-256 `03fc9ab424631c2702a25921424422607481c1c9d18db02d5eea1da402a84577`.
Source-before/source-after/copy verified; unplug notice delivered.
No individual physical report yet. User now authorizes implementing previously
pending parking front8mm addition and investigating617 route issue against618.
Metadata: buildOct31 15:24:55,turn=-1,path20261031_view_reobserve,
parking20261031_front8_123mm; installed binary not read back.
Footer completed/final_parking_stop,182798ms,truncated=1.1041 poses but none
for laps2/3; maximum gap171364ms caused by phase-triggered duplicate capture.
Original complete; trajectory incomplete. No independent physical completion
report. Findings and current correction: [617-619 review](../../reports/RETURNED_617_619_REVIEW_20261031.md).

## Returned620-621 (2026-10-31)

| Complete unchanged original | Bytes | SHA-256 |
| --- | ---: | --- |
| [620 CW](20261031_log_620_cw.txt) | 80329 | 9826d6c908f06dc900e6939ca46876e7c73790adec7d3568c0b6cedf1e7f0c12 |
| [621 CW](20261031_log_621_cw.txt) | 148179 | 9df133d711c3b7349560a8776bbffac93887b15f535f69fafd7f786e88a94215 |

Source-before/source-after/copy SHA-256 verified FIRST; immediate unplug notice.
User reports CW first GREEN after first corner: first run approached then did
nothing, second did not pass there as expected. Parking unfinished; user confirms
rear physical space and authorizes using10mm more rearward during manoeuvring.
Firmware metadata and measured findings follow after analysis; installed binary
not independently read back.

620/621 metadata: bothCW turn=-1,path20261031_inward_response;
diagnosticmodule buildOct31 15:49:10,connectormodule15:57:18;
621parking20261031_front16_123mm.620 stopped/mode_pause52501ms,
621aborted/entry_replan_limit218739ms;bothtruncated=0.
[Findings and correction](../../reports/RETURNED_620_621_REVIEW_20261031.md).

## Returned622/623 (2026-10-31), normal-mode restoration

| Complete unchanged original | Bytes | SHA-256 |
| --- | ---: | --- |
| [622 CW](20261031_log_622_cw.txt) | 106929 | e25973171c3a02ad384ada3e5748851565fb274d20ebb89c2fbfdc5d41c8b3f3 |
| [623 CW](20261031_log_623_cw.txt) | 109544 | e7bae2554a70241993093ef0cb7baebfd5792addecf95f4c3dfa1956c86bfd2a |

Copied FIRST; source-before/source-after/copy SHA-256 verified, immediate unplug
notice. User: first run serious scanning difficulty, second later-lap curve wrong;
requests disabling dense capture and returning to normal safe driving. Metadata
and measured findings follow; installed binary not read back.

Both CW, diagmoduleOct31 15:49:10,path20261031_settled_view,
corner module16:25:32; no finalparking reached.622 stopped/mode_pause97286ms,
623 stopped/mode_pause104033ms,bothtruncated=0. Original pixels unavailable.
[Normal logging handoff and measured limits](../../reports/NORMAL_DRIVING_HANDOFF_20261031.md).

## Returned624-627 (2026-10-31), measured30mm rear overhang

| Complete unchanged original | Bytes | SHA-256 |
| --- | ---: | --- |
| [624 CW](20261031_log_624_cw.txt) | 171236 | 86d99692dca3bc6a430f57eace8e905a5ce19fc0a1679d94a07ba9a6a8dbec24 |
| [625 CW](20261031_log_625_cw.txt) | 92310 | d019d1b4ba0448c82ce06414da01e40d7429980b64b843774e78c420edb58c0c |
| [626 CW](20261031_log_626_cw.txt) | 88488 | 95e28acd1414d5c5e79d1d4459a251e29e5539b7ef88d6d7c651e3bec988e2ba |
| [627 CCW](20261031_log_627_ccw.txt) | 89409 | 1dbbbfcdf7b751410cbf2bc85e30150b3099b499a24ffc7291038a9384086a72 |

Source-before/source-after/copy SHA-256 verified; immediate unplug notice.
Header diagnosticmoduleOct31 16:38:46; pathsettled_view,624parkingrear10_escape.
No installedbinary readback.624 footer completed191942ms;625/626/627 manual
mode_pause82194/77290/107270ms; alltruncated=0,sample_bytes=0 (normal logging).
User measured rear axle centre to rearmost robot part including sensor:30mm.
Earlier physical frontpinkcontact/~40mm rearroom not assigned to an exact file.

User correction: NONE of624-627 was physically good despite624 firmwarecompleted.
Same layout throughout; secondGREEN afterRED in thirdsection passedwrongside.
625 explicitly confirms seat22 RED at617mm/foot112; actual GREEN per physical
report. See [corrected analysis](../../reports/RETURNED_624_627_REVIEW_20261031.md).

## Returned628 CW (2026-10-31), remove superseded extra10mm preference

Complete unchanged [628](20261031_log_628_cw.txt),171194B,
SHA-256c62f3f7995e30f4ec8e33d17571c63f65c2e5f27caad52fb0b9cfb8b8c641379.
Source-before/source-after/copy verified; immediate unplug notice. Header
Oct31 17:20:17,turn=-1,pathfar_green_rear30,parkingrear30_123mm. Installedbinary
not readback. Firmware footercompleted189835ms/truncated0,normal logging.
User: too much front room and repeated rearward movement while parking; shorter
30mm rear means the earlier extra10mm request no longer applies. Use available
room on both ends. Physical correctness/contact not established by footer.
[Analysis and cancellation of extra10mm preference](../../reports/RETURNED_628_BALANCED_PARKING_20261031.md).

## Returned629-631 (2026-10-31), parking stalls

| Complete unchanged original | Bytes | SHA-256 |
| --- | ---: | --- |
| [629 CW](20261031_log_629_cw.txt) | 171862 | 1a0c95af9b3ef10c760f02d230d75db4c1bbb7f695d48090ab2c728f08dcdc8b |
| [630 CCW](20261031_log_630_ccw.txt) | 38690 | 916d03f7dba09df9493e77f5458810d957a19c6ca30a9d6bb7867a7acce962dc |
| [631 CW](20261031_log_631_cw.txt) | 168822 | 7f511c356c18e3d988131a7f717033a95e2566ecfb488c488b2a0b5f53bbfa53 |

Source-before/source-after/copy SHA-256 verified; immediate unplug notice.
User reports latest parking barely progressed despite space, wants a clean
sequence without long stationary intervals. Firmware/source findings follow.
Do not assign this report to630: it stopped before final parking.

## Returned632-634 (2026-11-01), parking corrections

Complete unchanged originals, source-before/source-after/copy SHA-256 verified.
630/631 supplied again and byte-identical to existing indexed originals.

| Original | Bytes | SHA-256 |
| --- | ---: | --- |
| [632 CW](20261101_log_632_cw.txt) | 172083 | 3e2bd3e85a436c7ae1f9076c9cae9097f9d185bc9c497b083313fe3b943f225e |
| [633 CCW](20261101_log_633_ccw.txt) | 173125 | 069666c69da124d83fce97efa0b7e40654898ea14f111c25872731f67c1a1813 |
| [634 CW](20261101_log_634_cw.txt) | 171761 | ec36d786488a266067586dfe4c21de8df10905a8b8f3adec93fecfe245a3cd42 |

All three report parking revision20261031_rear30_balanced, pathfar_green_rear30.
632 CW aborted entry_replan_limit;633 CCW and634 CW report completed.
All footers truncated=0; completion is an estimator result, not physical contact proof.

[Combined629?634 analysis and validation](../../reports/RETURNED_629_634_PARKING_MOTION_20261101.md).

## Returned635-636 (2026-11-05): CW exit alignment rejected

Complete original files copied unchanged; source/copy SHA-256 verified before
analysis and immediate unplug notice sent. These are complete source files,
not excerpts; neither represents a completed obstacle run.

| Original | Bytes | SHA-256 |
| --- | ---: | --- |
| [635 CW](20261105_log_635_cw.txt) | 25618 | a5baa6465673c2f4387d98501cdb70152adb57b52c111adf23548b7845e6e257 |
| [636 CW](20261105_log_636_cw.txt) | 27716 | fb28ae5dd6a192393f559ac9d386875cce0c953e83647ac48c0b946638f70671 |

Diagnostic build Nov 2 2026 17:43:41, center80, exit speed80mm/s, schema2.
Both finish cw_short_start_reference_invalid; no diagnostic overflow/truncation,
ordering errors or duplicate ToF snapshots.635 has later manual-disable footer;
636 ends after reference rejection without RUN_END. All five segments executed,
but final stopped heading error26.29/17.66deg exceeded5deg start acceptance.
User reports no mechanical/setup changes and observed wheel slipping.
Prior634 aligned after185.2mm;635/636 reached240.4/240.0mm without alignment.
Slip is consistent with encoder/yaw discrepancy, not independently measured
chassis translation. Generated analysis: local_workspace/parking-exit-analysis/20261105/.
Firmware unchanged: proposed speed reduction explicitly rejected and reverted.

## Returned645-650 (2026-11-05), latest six files only

Source-before/source-after/copy SHA-256 verified before analysis; immediate
unplug notice. All saved source bytes preserved unchanged.650 is cut in the
middle of a diagnostic line at36864bytes; archived explicitly as an excerpt,
missing the end of exit and all subsequent phases. It is not a complete run.

| Original | Direction | Bytes | SHA-256 |
| --- | --- | ---: | --- |
| [645](20261105_log_645_cw.txt) | CW | 74642 | 98a2a08b72c19d8f67d161130c29565de5187d374ba6bf000bee2256879e8089 |
| [646](20261105_log_646_cw.txt) | CW | 171088 | a79e8e4979856d5fcfd6af2c434e39a9c0ce3fe71263b74d84b6096c38fd085d |
| [647](20261105_log_647_cw.txt) | CW | 169746 | b6032a55bcbb09962055334641cca6e4f96a8d60cf68ec9d166fff1c0ba118e4 |
| [648](20261105_log_648_ccw.txt) | CCW | 167055 | d36934a7239725d6391936b39f7b415001f93d923f20b1329871632dbf9099cc |
| [649](20261105_log_649_ccw.txt) | CCW | 45682 | 9895f5fcd7deaf217b902f2bcf4be8cac678efb11ddcaef889a9e6480167a5ec |
| [650 excerpt](20261105_log_650_ccw_excerpt.txt) | CCW | 36864 | ee970a9995c210d30ae2a19922d6a3e029ea511e5146dc2fbcab08d2c348597d |

All headers diagnostic build Nov2_2026_17_43_41,exit80mm/s,center80.646-648
path20261102_lap_handoff/parking20261102_reuse_steer_settle: these predate the
new rearward10mm/parallel-stop changes.646/647 report parking complete;
648 aborts scan_alignment_limit.645/649 manual pause;650 incomplete.
Physical report: unnecessary slow later-lap regions, long CW parking search/
alignment, CCW creeping corner then stop; user now authorizes faster unparking.

## Returned651-654 (2026-11-05), measured parking dimensions

All four unchanged complete source files copied immediately and source-before/
source-after/copy SHA-256 verified; immediate unplug notice. All CW.

| Original | Bytes | SHA-256 |
| --- | ---: | --- |
| [651](20261105_log_651_cw.txt) | 165926 | 51dd9c1f45fbaaf650b8e9a26086e9939432d7f2f7c9fa9997217dd8a8d40551 |
| [652](20261105_log_652_cw.txt) | 13988 | b88d85d42923fc29c8d4aa96802234eddc42c96ad6c18392266e1dee8cb144c0 |
| [653](20261105_log_653_cw.txt) | 28884 | c71457a3058a248a485d27dba681309cf563ca49420a0a293ca46a4775ae3610 |
| [654](20261105_log_654_cw.txt) | 147611 | 2bc27ae7e4ada744acf0fb2c71f1eb1c1fd729777975afb5b39efb06b4e6a46a |

Build Nov5_2026_20_16_44,long_arc_speed100;651/654 identify
path20261105_physical_lap_speeds/parking20261105_efficient_alignment.
User reports exit/laps improved;652 user fault and653 slippery floor excluded
from normal software-failure inference.651 reports parking complete but user
reports poor/incomplete physical parking;654 aborts entry_no_collision_free_plan.
User newly measures axle spacing98mm,rear overhang35mm,front25mm straight or
37mm at full lock; entire straight width remains123mm. Requests shallow finish
about10mm inside the open longitudinal bay edge and more front manoeuvring room.

Latest follow-up655/656: complete CCW originals, same firmware as651-654.
User reports both nearly perfect, good longitudinal space use and shallow
physical end position; requests shorter/faster approach, further improvement
allowed. Both contain final_parking_stop/completed with no truncation.

| File | Bytes | SHA-256 |
| --- | ---: | --- |
| [655](20261105_log_655_ccw.txt) | 167917 | 7eca2d6fe725481fb908508205e5992ce2bc8d64a63b146dbd2ef6288c8ac4d3 |
| [656](20261105_log_656_ccw.txt) | 167316 | 9f0815eb7417c59657a738b926b1197befbe70124586718258e3d575614366e1 |

Cross-batch lap1 view audit of the latest20 archived files (629–636,645–656):
[2026-11-11 review](../../reports/LAP1_APPROACH_LAST20_REVIEW_20261111.md).
Original bytes unchanged;650 remains an excerpt. No new physical test batch.

## Intake20261119: unassigned older parking/exit originals

16 complete source files secured unchanged from the connected stick. Their actual capture dates/firmware versions and physical outcomes have not been established. The source clock reports2098. All files end on a complete line; no run-completion claim is made. No SURPRISE revision marker is present. Source-before/source-after/copy SHA-256 matched. See the [25-file intake manifest](../log_intake/20261119_stick/README.md) for other serial logs.

| Original | Bytes | SHA-256 |
| --- | ---: | --- |
| [20261119_log_356_cw.txt](20261119_log_356_cw.txt) | 8499 | 8b2f028b62ae37118f392f01bcdc45174bc44f0a26ad749708264c71aa1a67c4 |
| [20261119_log_355_cw.txt](20261119_log_355_cw.txt) | 8138 | b55701a005be22772a6b3cfe32b02d4114e165a028b163582a5e3fdfdf0e45ce |
| [20261119_log_354_ccw.txt](20261119_log_354_ccw.txt) | 11011 | e672bca3ae1717acc2f6071cbbe3260c3252d5f0fe589ff768b06a3d15b9b89d |
| [20261119_log_353_ccw.txt](20261119_log_353_ccw.txt) | 9160 | 131aabc5dbe018bfea59c22b7ced5630c6e029d720fcaf5daaf883a4fa9df11e |
| [20261119_log_352_cw.txt](20261119_log_352_cw.txt) | 8786 | b0504bed158b07efd370de59a93780976deed48df61947443e8ad83c2a5f90e5 |
| [20261119_log_351_cw.txt](20261119_log_351_cw.txt) | 8350 | 5029e4f89879ba26381f5db9cb51e1f348817f1a333f652349f56e3d435e2163 |
| [20261119_log_350_ccw.txt](20261119_log_350_ccw.txt) | 9689 | 8707aa6d33f81abc629ea955e451a50f6b3d214edc164f8578aa801437744377 |
| [20261119_log_349_ccw.txt](20261119_log_349_ccw.txt) | 9592 | 27ff035605a234cb9b88ec52005b1fc06698bf4c552287b5c9b38559ca0e0906 |
| [20261119_log_348_ccw.txt](20261119_log_348_ccw.txt) | 13699 | f042a2c923e32736494e28f422b948da52a1715201c2a29c336b12e13691bc14 |
| [20261119_log_347_ccw.txt](20261119_log_347_ccw.txt) | 9410 | 2824f86656cd8c1df475a9e3a42affb1303fc64437a68577fffb3330ca01fa22 |
| [20261119_log_346_ccw.txt](20261119_log_346_ccw.txt) | 9081 | c65b8d11bfdefea6fd4597d81029167b87645e61ef8c9a0522d4557320434211 |
| [20261119_log_345_ccw.txt](20261119_log_345_ccw.txt) | 8831 | 3563a281be716f5420decc917399eb2c8b9b4e8e3d250a5ea13b4b721b5a21fc |
| [20261119_log_344_ccw.txt](20261119_log_344_ccw.txt) | 13548 | f0d2b4a61ddf930efba4accf773b58f0a25496ee4b2cea3bb6916e34b0f96e78 |
| [20261119_log_343_cw.txt](20261119_log_343_cw.txt) | 11956 | 4abeba4514abd9b87149c8fa87b178a1c5c69a7ca5fb82f38a7c66c01a8005ff |
| [20261119_log_342_cw.txt](20261119_log_342_cw.txt) | 12331 | c89ffda1832ce1e06abfeda0892c1ea2b46f46a3e9e4bcb2807a53f9f039edd9 |
| [20261119_log_341_cw.txt](20261119_log_341_cw.txt) | 8575 | 1a516ca9fd817b78ca6a3d56d984582fb6939685ac2c6533bd293554547a89ea |

## 2026-11-19 additional source intake (637-644)

Complete original files, unchanged; source-before/source-after/copy SHA-256 verified during immediate intake. Date is intake date; physical reports and firmware not established. No new Surprise Rule acceptance claim.

| File | Bytes | SHA-256 |
| --- | ---: | --- |
| [20261119_log_637_cw.txt](20261119_log_637_cw.txt) | 26597 | 1f2b826dbc8131b104c568128afd6ae372627ef1cad902ff5607046fe8d0dbf9 |
| [20261119_log_638_cw.txt](20261119_log_638_cw.txt) | 29696 | 8b71fbd25f2079de3469b1f074cb9ea95b98ce82bfa5ef3d176bf3452bfda424 |
| [20261119_log_639_cw.txt](20261119_log_639_cw.txt) | 99225 | b70aafae642b4aa81f825991a3e91dcae9850bab725903146909a5c4979dad0d |
| [20261119_log_640_cw.txt](20261119_log_640_cw.txt) | 171336 | 222ee9f5f9c235f2b9dce7efa4ded8c166bc4a48130b87d5bef6a40363bcd5e9 |
| [20261119_log_641_cw.txt](20261119_log_641_cw.txt) | 15763 | c24b673daec9faf3c19c91a64b4ec2d2c2eecc33f3efdfefa2c0b353ae1fef6e |
| [20261119_log_642_cw.txt](20261119_log_642_cw.txt) | 33220 | 61abe9e4537492c49952a13ea880d0898a2fc91b16d39188aaf6544680e2cfb4 |
| [20261119_log_643_cw.txt](20261119_log_643_cw.txt) | 29488 | 341da9f3f7ebe4e96e71cc1f97cd2cc5ec5d894ecc94a22f5873ea129b27d459 |
| [20261119_log_644_cw.txt](20261119_log_644_cw.txt) | 32275 | 3b2dcbb0f23ab58555a994dbb747a9e7ef77fec06d313e45d45d8d38fd606a94 |

## 2026-11-19 new full-file intake, log2CCW

User: aligning too close to wall, slow approach/abrupt parking handover; light contact with far start GREEN near end lap2; otherwise good. Installed RAM-mitigated M7 reported from prior test, exact flash hash not read back. Full source copied unchanged before analysis and source-before/after/copy SHA256 checked; user released stick. This is a complete source file, not a completed-run claim.

| Original | Bytes | SHA256 |
| --- | ---: | --- |
| [20261119_log_2_ccw.txt](20261119_log_2_ccw.txt) | 138372 | ca40714175cfadcdad327c866291551c2e70f34470409c31170d484be7499a05 |

## 2026-11-19 CW batch, logs 3–6

We verified all four original copies against the source before analysis.
All four report camera revision `20261119_measured_height`. Both runs that
reached parking report `20261111_recovery_handoff`. The first two were
paused by the tester and the last two stopped during final parking. The
latest run recognised the 50 mm red sign. The exact binary was unavailable
for readback. [Batch observations and checks](../../reports/CW_LOGS_3_6_REVIEW_20261119.md)

| Original | Bytes | SHA256 |
| --- | ---: | --- |
| [20261119_log_3_cw.txt](20261119_log_3_cw.txt) | 98495 | 07fd4ec064b140b6c1bd722f5429dcb475e50836c259e4eb9bacaa748ec3d835 |
| [20261119_log_4_cw.txt](20261119_log_4_cw.txt) | 62327 | 0379446f767a296bc02aa8de55c0f92498e71a37a48b9156dde2fc44bd76a0d5 |
| [20261119_log_5_cw.txt](20261119_log_5_cw.txt) | 140019 | 89c36184a46474dcecfb90201e122476902960fa5ad05c5c564420351470e795 |
| [20261119_log_6_cw.txt](20261119_log_6_cw.txt) | 145310 | ecb427fb7f67a77efe49eb314f32195f4050aff38e1a11ae283544ecc4d55a0b |

## 2026-11-20 CW log 7

Original secured before analysis and verified against the source by SHA-256. The user reported a good first lap and multiple pillar contacts in the second. Complete export with `truncated=0`. The run ended by manual pause in lap 2 and contains no parking attempt. Firmware log reports `20261120_lap1_visibility`.

| Original | Bytes | SHA256 |
| --- | ---: | --- |
| [20261120_log_7_cw.txt](20261120_log_7_cw.txt) | 92663 | 87ee5ae283a27d90e53e19cb04244c5b89b0c5dc685e6b8a24a0ed51b03aa177 |

## 2026-11-20 logs 8–10

Complete originals copied before analysis, checked against the source before and after copying, and released to the user. All three logs report `20261120_lap1_visibility`. The completed lap pairs report later-lap policy `20261120_lap23_passing_reserve` and parking revision `20261120_approach_flow`. Log 8 ends in lap 1, log 9 during CW capture alignment, and log 10 on the CCW capture alignment preflight. [Batch review](../../reports/RUNS_8_10_REVIEW_20261120.md)

| Original | Bytes | SHA-256 |
| --- | ---: | --- |
| [20261120_log_8_cw.txt](20261120_log_8_cw.txt) | 84595 | 27023f71e09f6146fb0ed071a2fefda074c144b681bfc3bc2efd8633aba77e43 |
| [20261120_log_9_cw.txt](20261120_log_9_cw.txt) | 151354 | 2011d582cb51d554d1be695c2005f467407d7a79bb1a3561bdab55bf1f3bea69 |
| [20261120_log_10_ccw.txt](20261120_log_10_ccw.txt) | 160793 | b2f005a2720c347c0ed8ff628db1d131f966e6178745a4be5165f5a3c31c3d9f |

## 2026-11-20 logs 11–18

Complete originals copied and verified against the source by SHA-256 before analysis. The user released the stick. The logs report lap 1 revision `20261120_lap1_pair_view` where the lap starts. Logs 12, 14 and 16 report `20261120_lap23_smooth_pursuit` and parking `20261120_capture_clearance`. Log 13 stopped in rear positioning, with CW selected in the diagnostic configuration. The user reports one field layout for all tests. [Intake manifest](../log_intake/20261120_discovery_parking_batch/manifest.json) · [Batch review](../../reports/RUNS_11_18_REVIEW_20261120.md)

| Original | Bytes | SHA-256 | Recorded outcome |
| --- | ---: | --- | --- |
| [20261120_log_11_cw.txt](20261120_log_11_cw.txt) | 90447 | fce454442f4e52d79d74e4370a7a822f1390cd93c693395f7e70e8a0818deee2 | Paused in lap 1 at 96.404 s |
| [20261120_log_12_cw.txt](20261120_log_12_cw.txt) | 163806 | df927261878b24028dc9d3ca865ca0b0a3402d021f0c00a273e6972187ad6242 | Three laps, entry planner abort after registration |
| [20261120_log_13_cw.txt](20261120_log_13_cw.txt) | 3228 | 21f9b215c0e9c4bedc19159deeb9f72856f74eb18f7e09c6266657a0cb1ec4f1 | Paused after 0.757 s, before initializing the field pose |
| [20261120_log_14_cw.txt](20261120_log_14_cw.txt) | 169478 | 17abd14f264240786a6edf5bb11cc612111b207e65009b68418937d4d03a9a25 | Three laps, entry planner abort after registration |
| [20261120_log_15_ccw.txt](20261120_log_15_ccw.txt) | 97776 | 133f4e80970545fb63e3c3b8b4e71363d8e4dd177260f2212d3a879e6066060b | Paused during the lap 1 station 3 search |
| [20261120_log_16_ccw.txt](20261120_log_16_ccw.txt) | 125106 | 0713f4b1c2ad8d40557fdf618db868fba8b2e1c9595562d221a03e2761253de6 | Three laps, paused during parking approach |
| [20261120_log_17_ccw.txt](20261120_log_17_ccw.txt) | 73306 | ce518755198abbb2fc1251c2790c21e6bce2aa040c18bbb32a97f0be809c70f2 | Paused after the 60 mm station 3 retreat |
| [20261120_log_18_ccw.txt](20261120_log_18_ccw.txt) | 138797 | a8b848dc3c0366bc7190182ff804872d98b21d87e927addde8af5a8828452882 | Paused during repeated searches at station 3 |

## 2026-11-20 logs 19–20

Both complete originals copied unchanged and hash-verified against USB before analysis. Both report `20261120_discovery_reposition`, `20261120_lap23_smooth_pursuit` and `20261120_parallel_four_move`. These are normal obstacle runs before the fixed CW practice command. Both finished three laps in telemetry before a mode pause during parking. [Intake manifest](../log_intake/20261120_followup_after_reposition/manifest.json)

| Original | Bytes | SHA-256 | Recorded outcome |
| --- | ---: | --- | --- |
| [20261120_log_19_cw.txt](20261120_log_19_cw.txt) | 157555 | 02a553dfe40d2784b40145ebca6c70fc3bd0be341ca67a12f8bd0ba8bd8eca78 | Recovery at station 9. Entry first arc turned about 70°, paused after replan. |
| [20261120_log_20_ccw.txt](20261120_log_20_ccw.txt) | 140900 | 845b82f19aa2429870f42a3a4292920d28027d3d8f7097fdb08555878e69cacb | Early EMPTY at middle slots, recovery at station 6, paused before entry. |

## 2026-11-20 logs 21–26

Original files copied and SHA-256 verified before review. Some USB exports contain multiple runs. The user reported one successful park and another run with a poor approach followed by a stop. Log 23 directly records that failure pattern in fixed CW practice. Log 26 is the only new normal run with a field pose. It reports `20261120_near_empty` and pauses in lap 1 before final parking. The reported run order cannot be mapped to an exact file beyond the matching failure. [Intake manifest](../log_intake/20261120_normal_parking_followup/manifest.json)

| Original | Bytes | SHA-256 | Export contents |
| --- | ---: | --- | --- |
| [20261120_log_21_cw.txt](20261120_log_21_cw.txt) | 37699 | 828396483bf2d69838f8022c02b5705cb13be6c3440c3194a2f0fa974b9a6e28 | Early mode pause and completed fixed practice. |
| [20261120_log_22_cw.txt](20261120_log_22_cw.txt) | 47522 | edd906b1e9570f98b3b4c6158d0d75727af985a644b85f95db178a745aa735b9 | Early mode pause and completed fixed practice. |
| [20261120_log_23_cw.txt](20261120_log_23_cw.txt) | 21464 | d056524612f0d8075b924dfd4ff537d60d2a11c69959179483637ec60f7814fd | Fixed practice aborted with `capture_alignment_clearance`. |
| [20261120_log_24_cw.txt](20261120_log_24_cw.txt) | 5500 | 413c1f38a986b401624cb0e262301e15189361c5dfaba5f0d8e816c0f91eab10 | Original ends with outer-wall start reference rejection. There is no run summary in the source file. |
| [20261120_log_25_cw.txt](20261120_log_25_cw.txt) | 62068 | 2099dd29584c04863ce5a4d52015953a27c25f4eff682062f917c66a01d5f45e | Two completed fixed practices, 26.416 s and 26.568 s. |
| [20261120_log_26_cw.txt](20261120_log_26_cw.txt) | 97014 | 248a7dbebbcac5326bd6c373830b14ff6c35363af52a9f0e0b02d1669024163d | Normal `20261120_near_empty` paused in lap 1, after an 180 mm view recovery and rejoin. |

In log 23 the stopped left ToF accepted 241.5 mm against a predicted 257.9 mm. Registration shifted Y by −16.4 mm, from −1206.8 mm to −1223.2 mm, while the heading was only 0.5° from parallel. The controller then backed about 24 mm into the second rail's vicinity and the swept clearance gate stopped it before entry. The parked-side ToF fan at that stop intersects the measured rail interval even when one ray can see the wall. This explains how a mixed return could bias lateral registration. The log confirms the accepted shift and the missed capture, and cannot independently establish a physical contact. The production replay uses the unobstructed 252 mm wall return in the gap and selects direct capture to the same edge target.

## 2026-11-20 logs 27–28

Both complete source files were copied unchanged and SHA-256 checked before USB release. [Intake manifest](../log_intake/20261120_ccw_after_abort/manifest.json)

| Original | Bytes | SHA-256 | Recorded outcome |
| --- | ---: | --- | --- |
| [20261120_log_27_unknown.txt](20261120_log_27_unknown.txt) | 1092 | bf05c3833e2f055024a1c1e618c5a0225e8fe32243fb9b03f146f025e9042815 | Mode pause after 1.004 s at the start wall-reference gate. No direction or lap build identifier was emitted. |
| [20261120_log_28_ccw.txt](20261120_log_28_ccw.txt) | 129607 | 5285e3d43fd16cf0d07035d1d07ec11d74123b50c86343c146a542b7f550c153 | Three laps with `20261120_near_empty` / `20261120_lap23_smooth_pursuit`. Parking `20261120_normal_fixed_entry` aborted at `initial_outer_wall_reference`. Footer is present with `truncated=0`. |

Log 28 confirms sole GREEN at S1 seat 9 and sole RED at S2 seat 14. Parking traces show +42 then −37 steering commands during the outward shift and a stop at about x=686 mm, y=−1210 mm, heading 359°. The nearest side ToF return remained about 173–179 mm against approximately 255 mm predicted. The data show a rejected measurement and establish no independent physical wall distance. Log 26 shows map reset and a separately started lap 1 before two corner-view holds, a 180 mm retreat at station 9 and manual pause. The reported failed USB write is not observable in the intact log files.

## 2026-11-20 logs 29–31

The device reported FAT32 with read/write access. Old files had newer filesystem timestamps. Originals 29–31 were identified by file number and verified byte for byte before analysis. [Intake manifest](../log_intake/20261120_fixed_followup/manifest.json)

| Original | Bytes | SHA-256 | Outcome |
| --- | ---: | --- | --- |
| [20261120_log_29_cw.txt](20261120_log_29_cw.txt) | 156092 | 26b27eff80e4caf2217cb375af1b9563026d69f88ff674df08716aeac8d8e9de | Three laps, fixed entry ran two strokes, old planner aborted with `entry_no_collision_free_plan`. |
| [20261120_log_30_cw.txt](20261120_log_30_cw.txt) | 149458 | 0af3c8aed8d1514266e978492870e1673b88c46bfac33a1cb214d92679a0eafb | Three laps, five effective GREEN seats, manual stop after the old replan at the start of stroke 3. |
| [20261120_log_31_unknown.txt](20261120_log_31_unknown.txt) | 1093 | aabc26340c29019d5d52f06d16bae518f2d484aceb4342bcab4fd93b32dca275 | Manual stop before direction lock and exit, 1.057 s. |

Logs 29 and 30 use `20261120_ccw_rail_approach` with `20261120_compact_lap1` and `20261120_lap23_ccw_single_corner`. Both reset the run map and report complete USB footers with `truncated=0`. Initial Y corrections after the approach were −21.5 mm and −29.5 mm, and measured gaps were 238.5 and 240.6 mm. After the nominal 120 mm reverse they recorded diagonal headings 49.8° and 49.7°. The old third-stroke gate rejected both. Both reversed 180 mm at unresolved station 9. Log 30 classified an oblique RED in seat 7 as 50 mm at about 560 mm, stored its effective colour as GREEN and used that map in rounds 2–3. The logs cannot independently establish its physical height, actual contact points or the specific cause of the reported LED indication.

## 2026-11-20 logs 32–33

Both complete USB files were copied and SHA-256 verified before release. The good first run and its parking are user observations. Neither file contains that completed run or its parking angles. [Intake manifest](../log_intake/20261120_cw_fixed_125/manifest.json)

| Original | Bytes | SHA-256 | Outcome |
| --- | ---: | --- | --- |
| [20261120_log_32_unknown.txt](20261120_log_32_unknown.txt) | 3597 | 294e2714a05d2562ec61d8cbed945a5317fb8d82d8a49366ac5afa2f1beb68d9 | Stopped after 0.994 s during rear positioning, before direction lock. |
| [20261120_log_33_ccw.txt](20261120_log_33_ccw.txt) | 91987 | 74286250f577d468e9f74ad38250050b7393d955c856d9cb811412cb92b92032 | CCW, lap-1 marker `20261120_compact_lap1`. Tall GREEN seat 19 saved as effective RED, repeated gyro restarts before seat 23 confirmed, then drive stall and run stop. |

The third position in S3 is seat 23 (RED, confirmation t=112672 ms). The short motions before it coincide with repeated SPI/SH2 recovery and about 1592–1602 ms without a gyro poll. The next GREEN at S0 seat 1 was already confirmed and stored at t=92050 ms, but the startup bypass deferral kept it out of the active line until the return corner at (−1146.7, −476.5 mm). The physical wall contact is user-reported, and the logs cannot identify the electrical or transport cause of the gyro faults.

## 2026-11-20 logs 34–35

Both source files were copied and SHA-256 checked before USB release. The stick mounted read/write as FAT32. The user reported a perfect first run, steady red during the save, and a manual reset. The perfect run is absent from this batch. [Intake manifest](../log_intake/20261120_usb_timeout/manifest.json)

| Original | Bytes | SHA-256 | Outcome |
| --- | ---: | --- | --- |
| [20261120_log_34_unknown.txt](20261120_log_34_unknown.txt) | 4384 | 0e6b3001df0b8375ca4aed9970bbe5e7f7f3108c59590a14a3581156e0c3d918 | Moved during parking start, manually stopped after 1.082 s before direction lock. |
| [20261120_log_35_ccw.txt](20261120_log_35_ccw.txt) | 156978 | e7a9c070c1f7f57fe9a7009160c807cdc59136570bca14668bb4432396e53d9a | New map, three laps, parking `capture_alignment_preflight` abort, footer present with `truncated=0`. |

Log 35 reports lap-1 marker `20261120_compact_lap1`, three-lap route marker `20261120_lap23_ccw_single_corner` and parking marker `20261120_fixed_entry_only`. The gyro polling gaps include 758 ms in lap 1 and 230 ms alongside the recorded 230543 µs lap handoff computation. The firmware records a fresh map reset and freshly stored signs, with no evidence of reuse across the save or reset. This successful large export does not identify the blocking call or physical USB condition during the missing run.

## 2026-11-20 logs 37–38

Both complete exports were copied unchanged and verified by SHA-256 before USB release. They report `20261120_compact_lap1`. [Intake manifest](../log_intake/20261120_ccw_position3/manifest.json)

| Original | Bytes | SHA-256 | Outcome |
| --- | ---: | --- | --- |
| [20261120_log_37_cw.txt](20261120_log_37_cw.txt) | 162216 | e5b6a2e3cd64884ff24449e203dbbe44ff32663d738d8a8ff131bfa18b86ea6d | Three laps, fixed parking aborted on `fixed_no_measured_progress`. Footer complete. |
| [20261120_log_38_ccw.txt](20261120_log_38_ccw.txt) | 91876 | 71dcddc3c49f342f915e15a85e2caf09f817ff2ea1b21fdbe6c7a39550d55f17 | Tall GREEN mapped as RED in slot 19. Eleven gyro timeouts/restarts. Next GREEN stored in slot 1 and activated on return. Stalled before completing lap 1, then manually disabled. Footer complete. |

The tester reported a diagonal view of the third positions, short motions and an outward departure towards the wall after RED in the second run. Log 38 confirms the slot-1 GREEN at `t=64797` and the slot-23 RED at `t=85214`. The slip flags and estimated poses cannot establish the physical wall clearance or contact.

## 2026-11-21 logs 39–42

Complete exports copied unchanged and verified by SHA-256 before USB release. They report `20261120_ccw_tall_return` and, when reached, parking `20261120_fixed_entry_only`. [Intake manifest](../log_intake/20261121_green_parking/manifest.json)

| Original | Bytes | SHA-256 | Outcome |
| --- | ---: | --- | --- |
| [20261121_log_39_cw.txt](20261121_log_39_cw.txt) | 157291 | 642d78ca746de88bb8f5f47b008b6f8486bd7eb692285ba8148b57faeec99cb4 | Tall GREEN correctly mapped as RED, three laps and four parking strokes, `fixed_final_pose` abort. |
| [20261121_log_40_ccw.txt](20261121_log_40_ccw.txt) | 176836 | 8f2bc5d045df998ae66cc5f198909facbc8eb5b6f14c2792cdca1709c81b6571 | Middle GREEN and later tall GREEN mapped, three laps, stopped wall correction −66.1 mm required realignment, `fixed_final_pose` abort. |
| [20261121_log_41_ccw.txt](20261121_log_41_ccw.txt) | 66909 | 5835ba76a69d76d7b8cd0cc13951b6e888e895dc98ff29eec31f2785285f4b53 | Middle GREEN position 9 marked CLEAR from 422.4 mm at 21.7°, stall followed by manual pause in lap 1. |
| [20261121_log_42_ccw.txt](20261121_log_42_ccw.txt) | 173516 | 66c5f5ef705ada90f0e4c3caa4662092cf363f2d33fc437ca75672a6790a3f20 | Ordinary middle GREEN incorrectly locked as the 50 mm surprise sign. Tester confirmed the actual modified sign was later in the course. Three laps and four parking strokes, `fixed_final_pose` abort. |

The three completed obstacle runs reported 238.8 / 238.8 / 241.9 mm registered gaps and estimates of final lateral containment outside the 5 mm margin. These are odometry estimates, not independent physical clearance measurements.

## 2026-11-21 logs 43–47

Complete originals copied unchanged and verified by SHA-256 before USB release. The batch reports lap-1 `20261121_ccw_middle_view`, later laps `20261121_lap23_cw_inner_red`, and parking `20261121_flush_capture` when reached. The user confirmed that runs 46/47 used the same layout and that the robot did not contact the pink rails. [Intake manifest](../log_intake/20261121_followup_43_47/manifest.json)

| Original | Bytes | SHA-256 | Outcome |
| --- | ---: | --- | --- |
| [20261121_log_43_cw.txt](20261121_log_43_cw.txt) | 162424 | 63930d9585bf2e2ca3910943c34ca7b6515462478613f88a74f0cab19bba3f37 | Seat 15 inner refinement timed out. Three laps and four parking strokes, `fixed_final_pose`. |
| [20261121_log_44_ccw.txt](20261121_log_44_ccw.txt) | 175744 | 2d683c030fff339d5c2fad1dee1df4dd0115ff976cc6cd3175dbedf634b81686 | S1 normal GREEN and later 150 mm surprise locked independently. Three laps and four parking strokes, `fixed_final_pose`. |
| [20261121_log_45_ccw.txt](20261121_log_45_ccw.txt) | 175182 | 4f8474dbad7a233fc97213f0212551dbd31625f8a710554428252ba7d2878dc2 | Alternate layout with 50 mm surprise, three laps and four parking strokes, `fixed_final_pose`. |
| [20261121_log_46_cw.txt](20261121_log_46_cw.txt) | 99475 | c5f9308ae86e4721c7a49081761e9dbb1d4c9627a9e840ebb0bc455cce6ff7a6 | CW start middle plus end contradiction. Lap boundary held, followed by manual pause. |
| [20261121_log_47_cw.txt](20261121_log_47_cw.txt) | 170127 | 2f04cd6316fe810610d070a9e89bbfcf81d5ec74f65403bfd9a81eb907e30c54 | Same layout, S0 middle directly clear, 50 mm surprise confirmed at seat 12. Three laps, first parking arc timed out while yaw and encoder still progressed. |

Log 43 recorded 92–93 mm on the parked outer fan with about 99.7 mm predicted. Logs 44/45 report parallel and stationary but the parked ToF samples are absent because the trace budget ended earlier. Log 47 ended at about 36° local yaw with 81 mm of reverse movement at the five-second first-stroke deadline.

## 2026-11-22 log 48

The complete original was copied and SHA-256 verified before releasing the stick. The stick timestamps do not establish run chronology.

| Original | Bytes | SHA-256 | Outcome |
| --- | ---: | --- | --- |
| [20261122_log_48_ccw.txt](20261122_log_48_ccw.txt) | 149495 | 162b0f3175a5179fff19d6d7e0b1a4e62526654005e26a307d0342fc8068bdd3 | CCW, rear-ToF start, three laps and four fixed parking strokes. Complete and contained according to the firmware, with `truncated=0`. |

`20261122_two_sided_wall_line` confirms the parking approach revision. The final exit arc aligned at 201.5 mm with 1.8° error, and the first parking arc finished after 109.8 mm in about two seconds. Neither reached the old limits. The reserve patch shares the same approach revision and prints reserve events only on activation, so this log cannot distinguish the two builds.

## 2026-11-22 follow-up logs 49–51

Complete originals copied byte for byte and SHA-256 verified before USB release. The tester confirmed the layout stayed the same and that the two reported wall runs were the CCW runs.

| Original | Bytes | SHA-256 | Outcome |
| --- | ---: | --- | --- |
| [20261122_log_49_cw.txt](20261122_log_49_cw.txt) | 157374 | 2caa400958a16dfbea4adfe71e31deb53b5b1f7985d2c1e469b5d6a116546b9 | CW. Red seat 8 confirmed as ordinary before it reached the image centre. Three laps. Left-side rail scan and four fixed strokes. Estimated final footprint rejected. |
| [20261122_log_50_ccw.txt](20261122_log_50_ccw.txt) | 91705 | 67b27b86462019db199b81b30dd4a7cbe565b5ba12d439969071ceeb12ba3cc5 | Short visible RED at seat 21 stored as effective GREEN. Start station 0 cleared, then a stall before lap 1 finished and manual disable. |
| [20261122_log_51_ccw.txt](20261122_log_51_ccw.txt) | 92426 | cda4b84208cd9eb611f5d4aec7b2b17337eb0bd2bc3ddc4757247a5ed3ab5f47 | Short visible RED at seat 21 stored as effective GREEN. Additional GREEN at seat 1 accepted with 86 mm snap error and activated at the start return, then a stall and manual disable. |

Log 49 and 48 report the two-sided wall approach revision when parking is entered. All three logs advertise the same cached parking diagnostic object build as 48; that stamp does not identify the complete binary after an incremental build. On 50 and 51, confirming the first GREEN in S2 took the 180 mm straight recovery plus a short extra move. The tester reported wall contact on the returns. Last side readings were LEFT 87 mm in 50 and RIGHT 62 mm in 51. The encoder stalls and late side ranges do not independently locate the contact.

## 2026-11-22 log 52

| Original | Bytes | SHA-256 | Outcome |
| --- | ---: | --- | --- |
| [20261122_log_52_cw.txt](20261122_log_52_cw.txt) | 69515 | c8fab298c9069ee5c64fd4ba9c76ab29dc010e65858fda9fe2a125a6169f33f8 | CW. Complete original. Normal exit, then first-lap S1 station 0 stop. The 230 mm retreat cleared seat 6. Seat 7 stayed unresolved. No further viewpoint started before manual pause at 53.988 s. |

## 2026-11-22 logs 53 and 54

| Original | Bytes | SHA-256 | Outcome |
| --- | ---: | --- | --- |
| [20261122_log_53_cw.txt](20261122_log_53_cw.txt) | 82648 | 0622ace83776ab075e639d4593f7be0a1c6ca45434480d50a6d21782db094c5c | CW. Full export. Same corner module timestamp as log 52. A 230 mm retreat cleared seat 6 and seat 7 remained unresolved before manual pause. |
| [20261122_log_54_ccw.txt](20261122_log_54_ccw.txt) | 154491 | ef010af79d88f910112de37eb8ba91eae1bc4c737a5f1c9c902774e0a11ee6ad | CCW. Full export. One 230 mm discovery retreat at S2. Three laps. Direct corner handoff hit the 16 ms budget; the fallback entered the bay. The tester reported it parked, firmware recorded `fixed_final_pose`. |

## 2026-11-22 logs 55 to 58

Complete originals verified by SHA-256. The tester confirmed the same physical layout throughout.

| Original | Bytes | SHA-256 | Outcome |
| --- | ---: | --- | --- |
| [20261122_log_55_cw.txt](20261122_log_55_cw.txt) | 138742 | 060e28cf3dc91af140fd500929a02886fa7fc2158d704f643bc904cc0511673f | CW. 180 mm observation retreat. Seat 8 was confirmed as ordinary RED from a distant edge view. Three laps. |
| [20261122_log_56_ccw.txt](20261122_log_56_ccw.txt) | 82584 | 224ba6b92a158136cd4b9d63f8e375291f857b267a14bec43561a32e5b2dc287 | CCW. Seat 16 confirmed ordinary RED. Gyro restarted later in S3; manually paused. |
| [20261122_log_57_ccw.txt](20261122_log_57_ccw.txt) | 3887 | 5594a2762727a05b34b9ee96a7058abe696bc4924b6d732d3bce6f9df4857a26 | CCW. Complete but stopped during exit after 1.537 seconds. |
| [20261122_log_58_ccw.txt](20261122_log_58_ccw.txt) | 82676 | 8b9864c0e7d4a215677fc0e02c955c90a898e33edc3e79195e4a02ff0c0ca553 | CCW. Seat 16 correctly stored as 50 mm visible RED, effective GREEN. The S3 observation retreat repeatedly paused for gyro recovery and locked at 179.1 of 180 mm. |
