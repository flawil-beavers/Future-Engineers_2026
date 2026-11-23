# Height-task intake history, 2026-11-19

Preserved pre-correction findings; the active handoff and November19 measured-height report supersede acceptance status below.

## 2026-11-19: height-task status and incidental USB intake

User asks whether new-task recognition is complete/problem-free. CurrentHEAD
484a23e retains enabled height rule, first-encounter protection, single special
identity and effective original passing colour. Existing model checks are
documented; physical50/150mm/current-mount acceptance still unestablished.
No firmware changed, new reference image used, build, upload or motion.

Connected stick discovered:25newest-by-filesystem-mtime original logfiles
copied unchanged and source-before/source-after/copySHA256 verified; immediate
unplug notice sent. Source clock reports2098, so actual run dates/firmware and
physical reports are unknown.16parking/exit originals341..356 are
in simulation/evidence/parking_exit_diagnostics/ with20261119 filenames and
direction from PARK FIELD START. Nineother serial originals and the complete
portable25-file/hash index are simulation/evidence/log_intake/20261119_stick/.
All saved originals end on a full line; no complete-run claim. No SURPRISE
revision marker in these25files: not new height-task acceptance evidence.
Intake only; no motion-error conclusions. Next validate real50/100/150mm under
current camera/light, require approval of each new reference image, then
normal CW/CCW and original-colour passage on all3laps; secure newlogs first.

## 2026-11-19: rapid stationary height-task camera intake, complete

User requests acquire all images quickly FIRST, compute after complete series. Confirmed USBconnected/driveOFF/stationary; fixed robot/mount/light, centred targets; distance is lens to near block face. Existing c0/camshot only.
48 complete CRC/footer-checked153600B images, four original files each; all hashes/setup facts in simulation/evidence/camera_diagnostics/README.md and validation_status.json.
Sets: green_100mm_center_500mm, green_100mm_center_700mm, green_150mm_center_350mm, green_150mm_center_500mm, green_150mm_center_700mm, green_50mm_center_350mm, green_50mm_center_500mm, green_50mm_center_700mm, red_100mm_center_500mm, red_100mm_center_700mm, red_150mm_center_350mm, red_150mm_center_500mm, red_150mm_center_700mm, red_50mm_center_350mm, red_50mm_center_500mm, red_50mm_center_700mm.
Pending image-quality approvals: none.
Excluded captures: 20261119_green_100mm_center_500mm_01, 20261119_green_50mm_center_500mm_01, 20261119_green_50mm_center_350mm_01.
Next: Capture phase complete: 48 images, 45 approved, 3 excluded. Verify all original/index hashes; then evaluate height/range on approved ground_truth-tagged images before any firmware tuning.
No colour/height computation, calibration, firmware change/upload or motion. Serial closed after each capture. USB25earlier originals unchanged. Do not use pending/invalid/wrapped images for acceptance. Before eventual replay, adapt old tests that assume all approved images are100mm to the explicit ground_truth metadata. Capture bookkeeping helper is ignored local_workspace/height-capture-20261119/intake.py.

## 2026-11-19: approved height-series offline replay

All48 complete captures secured;45 approved,3 excluded;192 original bundle/index hashes verified and48 CRC/footer/payload comparisons passed. User-reset recovered two prior empty RED150/700 capture timeouts; failed wire archives remain explicitly marked. No firmware upload or motion.
Current-source replay:648 synthetic/48 association/24 policy cases pass, but only11/45 approved real images return expected colour height class;7 have no expected-colour component. At500/700mm, clear RED ground range is about382/454mm. Tall150mm at500mm becomes normal100mm: physical Surprise Rule recognition is NOT validated. Report/table/source hashes: simulation/reports/CAMERA_HEIGHT_CAPTURE_REVIEW_20261119.md. Original evidence and SHA registry: simulation/evidence/camera_diagnostics/. Firmware identity on robot unknown; replay is working-tree code, not proof of flashed revision.
Updated simulation/check_surprise_rule.py to honor ground_truth colour/height, retain historical100mm checks, report all failures/ranges. Real-reference test deliberately fails (35 failure events including one extra background component); no vision thresholds or driving code changed. Next fit/check current ground-plane and height projection jointly, then offline small70cm/background-component review; keep45 approved references and do not silently widen class windows. Only after offline validation build affected M7 and request normal physical CW/CCW acceptance.

## 2026-11-19: additional existing USB originals secured

Connected stick scanned before analysis; all656 source files compared to existing evidence;365 previously unindexed originals copied and source/copy hashes verified in ignored working archive. Highest-numbered637..644 complete originals additionally curated unchanged under simulation/evidence/parking_exit_diagnostics/20261119_log_NNN_cw.txt; README records exact byte counts/SHA256. All eight identify CW via PARK FIELD START. Firmware and physical outcomes not established; no new special-height acceptance inference. Older unindexed sources remain working copies only. User told stick can be unplugged immediately after verification. This session changes recognition only, no parking motion tuning; next validate current height task with approved image series, then normal new-firmware runs.
