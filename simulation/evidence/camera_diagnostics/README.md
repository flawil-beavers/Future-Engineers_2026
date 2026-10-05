# Stationary camera evidence

## 2026-10-05: green and empty control in a different room

The user corrected the setup: this GREEN measurement was made in a different
room from the RED measurement below. The robot was USB-connected, stationary,
and drive-disabled. The near green pillar was on a marked seat and the raw
`20261005_green_middle_same_pose_01.serial.bin` passed CRC32 `39ad899f`
and footer validation (frame 36766, exposure 360 lines). The broad legacy
GREEN region x191, foot124, width89, height45, area1396 was production-invalid
because it joined background; the visible pillar itself is upright in the
decoded image. The separate mapped-seat diagnostic at expected x207/foot126
found a compact candidate x195-196/foot124 in 10/10 fresh frames, while the
same now-empty position at expected x196/foot124 yielded 0/10 candidate
frames. This supports the fallback for this one stationary view. It does not
validate moving-run colour or seat assignment. Do not pair its exposure or
range with the RED image from the other room.

| Evidence file | SHA-256 |
| --- | --- |
| `20261005_green_middle_same_pose_01.serial.bin` | `7155dafb5a9de8ebac92ad9fda9e6075ce174d07e81f5c2809ed1ac6cf1e9395` |
| `20261005_green_middle_same_pose_01.rgb565` | `5e6396851e27784bb9e40d1fd943ec869edeaa047a21d288f8f481eb0831b8ef` |
| `20261005_green_middle_same_pose_01.json` | `b82baac5f425b5bfb3be9207939814d2f847404a233a52e123f7a1551abf1e8c` |
| `20261005_green_middle_same_pose_01.png` | `7bce7af71486e5b7d41243db79e93c625f6a6a32afbfd23325e63191c80a177b` |
| `20261005_green_middle_same_pose_01.stationary.txt` | `37ff912baf58cb66d6f8ecd52f1f57acfe6112d8fd953d8cdbe1d5a670b2370e` |
| `20261005_green_middle_same_pose_01.camseat.txt` | `1541c8f2b7500728f972db1c960ca826ff849ced4333f466376ccda555dab1bd` |
| `20261005_green_room_empty_camseat_196_124.txt` | `e2ff93df5f6097f8067fbb8f805ab9a5112fa32930f2e2847490cd3de76c4a4c` |

After the empty control, the user placed RED at the same near seat in this
new room. Its complete `20261005_red_green_room_same_pose_01.serial.bin`
passed CRC/footer validation (CRC32 `7c438c51`, frame 3490, exposure 360
lines). The stopped red blob was production-valid at x197, foot124, area
about1120, bearing -7.5 deg and estimated range526 mm. Red-valid counter
increased by 28 across each of two 28-frame intervals; the green-valid
counter did not increase. At the same expected green seat x196/foot124,
the diagnostic found 0/10 green candidates. All three controls therefore
separate at this particular stationary pose and room light: green fallback
10/10, empty0/10, red0/10 green false candidates. The normal broad GREEN
blob itself was shape-invalid; neither moving colour nor field-seat mapping
is proven. The earlier RED capture was from a different room and must not
be treated as paired with these samples.

| Evidence file | SHA-256 |
| --- | --- |
| `20261005_red_green_room_same_pose_01.serial.bin` | `890a5f8e06e4cacb517da282d6d9a96e8939c59515db7a88354e984c132d574f` |
| `20261005_red_green_room_same_pose_01.rgb565` | `5c086b107a2908946649bef1d5007dd15b69a4b6f1891a43771954d40af33610` |
| `20261005_red_green_room_same_pose_01.json` | `359225923b9c06ba637a827fd240ce679da2ac6d5d09caef46a547814603ba7d` |
| `20261005_red_green_room_same_pose_01.png` | `3e3ad085d176b283afa00d7249bdf854e29057bb36d82deb6b06629fba0ad650` |
| `20261005_red_green_room_same_pose_01.stationary.txt` | `a2ba53846fbfb140d281701cb8781591aff595132e08b694b8a4b5178d5e0cfb` |
| `20261005_red_green_room_same_pose_01.camseat.txt` | `d0f254737ffbbe8dc8f415b7cd065d3661619339580039bca1427e548812a744` |

## 2026-10-05: middle red pillar after the preceding corner

The user positioned the stationary robot before the middle inner red pillar
in the section opposite the start, with USB attached and drive disabled.
The user subsequently provided an overhead photo showing the near red pillar
on the marked inner middle seat and estimated camera-to-pillar distance at
about 55 cm, not tape-measured. The robot was manually placed; its onboard
field pose from a moving run is unavailable. This capture therefore cannot
calibrate field position. The
original `20261005_red_middle_after_corner_01.serial.bin` is a complete
`camshot` transfer (153,600 RGB565 payload bytes, CRC32 `1eb94299`, frame
443, exposure 94 lines); receiver CRC and footer validation passed. A
derived `.rgb565`, `.json` and rotated `.png` accompany it. The independent
`.stationary.txt` c0 capture shows 29/29 production-valid RED frames, x=207,
foot y=126, area 1,036, bearing -9.7 degrees and estimated ray range
507.3 mm. The near red pillar is visibly complete; a smaller red pillar is
also visible farther back at the right edge. This supports colour/shape
recognition at this stopped view. The estimated 507 mm ray range is broadly
consistent with the user's approximate 550 mm physical estimate, but that
estimate is too uncertain for a camera calibration or a 20 cm pose-error
claim. This does not establish moving-run seat projection. No firmware or
drive setting changed.

| File suffix | SHA-256 |
| --- | --- |
| `.serial.bin` | `898818cfe5a326382f965989553cd5e1359ce729d469f3ec5af0ac4b3b85733a` |
| `.rgb565` | `2696da7a887e6cc15a8fe78f0818093889b48644fd0dff41779235c547d7e4bf` |
| `.json` | `85e605ed68495c019a2d5f32583275f771224ac72ba7a5eb1db0fd2a194d7787` |
| `.png` | `dedb672a64ee7cbf130a4c23234ed6802c4c8d717002844b7d6712c9cde014d5` |
| `.stationary.txt` | `3d61942c85c3daf97f9643138bf5b799ea7abbce6ed82b26252cec9223a00419` |

## 2026-09-29: same S1 front-empty view before diagnostic upload

The user reconnected the stopped robot before S1; front station empty, green
pillar at the rear outer station. Drive stayed disabled. Complete CRC-checked
`c0`/`camshot` transfer stem `20260929_s1_empty_front_recheck_01`:
`.serial.bin` SHA256
`0fef2444e074355778d70953427c9297e3e3e463eac1285acd814bab1ad77f66`;
`.rgb565` `1f3c11b984f19f55574f4758ec8797e6bc0af806aa6fc77e57c90d8e18cf57ab`;
`.json` `e44358d61ab1a086b47f22aa1e89752457691ba975dad0bb9e83fa814d817c30`;
derived `.png`
`920f055feb4f8858a7f1918ac27adc057b4378f6898ee6015cae9a6191755ecb`.
Header: frame 9054, exposure 97 lines, CRC32 4232037854. The text capture
`20260929_s1_empty_front_camseat_128_205.txt` SHA256
`5509e19505f86a93fbb1eb431c4461bf22290605aed6308dbae7fe0b1bce8e3b`
contains 10/10 candidate=0 frames 252..261 at the empty front seat.
This is still the pre-revision firmware and a stopped view, not a replay of
the moving log-412 false-positive image.

After explicit user consent, M7 firmware binary SHA256
`d2cacaa34301059a17ed9e6bb80b34874012a57119a3544948c74e00e1390862`
was uploaded successfully with PlatformIO/DFU while drive remained disabled.
The follow-up stopped receiver text
`20260929_s1_empty_front_post_upload_camseat_128_205.txt` SHA256
`4037641d7c12c047ac436645f1cee2e74ac9441c40e4a64dc658e251c81fe5a9`
again shows 0/10 mapped-green candidates at the empty front image point,
frames416..425. DFU success is not an independent binary readback. The
physical moving false-positive case remains unverified.

## 2026-09-28: CW empty S1 front, green at S1 rear

The user placed the stopped robot before the left section, with the legal
green pillar at the last outer station and the first station empty. Drive
enable was off. The complete `c0`/`camshot` USB transfer and CRC-checked
derivatives share stem `20260928_s1_empty_front_green_rear_stationary_01`.
Original `.serial.bin` SHA256:
`2ef03314dfea45134e1d04eeaf869f6aaf82cb89c67c46efd30e420d4e9792b9`;
`.rgb565`: `a47fe1ca52db01545b31af8e9e442dbab881af4ab933c2e91f9de667a2ab25a87`;
`.json`: `b7238e491c14dae0ada4e561743a565799af4fed6a7339555f24572acc37ed5f`;
derived `.png`: `93f48722d9cebddf5fe5e0f606ccbfea593759f6ca621722bec689e77976eb18`.
Header: frame 897, exposure 144 lines, CRC32 3880047042. The rear green is
small near the horizon; foreground printed lines are visible. A stopped
`camseat 128 205` check on ten subsequent fresh frames found 0/10 mapped-green
candidates. This standing pose does not reproduce the moving false green
confirmation in log 412, and no raw camera image from that exact drive pose
exists. The subsequent offline continuity check retained candidates in all
ten archived 40-cm true-green images. This is a limited image check, not
moving-route acceptance.

## 2026-09-28: uploaded mapped-seat candidate, stopped live-frame check

M7 firmware SHA256 `f87224f6a6458faa724a8f815045b3a3eaf39911228f47be1e8a32a3719c2081`
was uploaded successfully by DFU. With drive enable OFF, the user kept the
robot facing the green wall at the CCW corner/straight boundary and placed
green about 40 cm from the camera on the image right, then removed it, then
put red at the nominal same position. The robot and room lighting stayed
unchanged. `c0` and `camseat 278 138` read ten consecutive camera frames for
each case. The text captures are complete receiver sessions, including
surrounding stationary camera telemetry; `legacy` in the `camseat` rows means
legacy **green** validity only. The red camera-telemetry rows separately show
production-valid red. These are stationary tests, not a moving route test.

| Text capture | SHA256 | Consecutive frames | Green candidate | Per-seat ROI time |
| --- | --- | --- | --- | --- |
| `20260928_ccw_green_wall_camseat_green_01.txt` | `96185c7dbd52c33482cb6c25891bdd9445a4325e446b4c0b8b363bfecfcea20` | 450..459 | 10/10; x267..271, foot138, samples99..130 | 5699..5784 us |
| `20260928_ccw_green_wall_camseat_empty_01.txt` | `4ba26c6e6da85d30a191e1dfe38685e2c6cf4746273b2ec6ae9caf82220e3d27` | 981..990 | 0/10 | 431..436 us |
| `20260928_ccw_green_wall_camseat_red_01.txt` | `7f38ae0cf46961c022fe404b49a3783a09c1f53904df0e6760a68bcfc867caea` | 1544..1553 | 0/10; red production-valid in nearby camera telemetry | 428..446 us |

Camera interval was about 76.5 ms. The ten-frame diagnostic temporarily
blocks the main loop for about 0.9 s while drive is disabled; the gyro
timeout warning after each diagnostic is therefore expected. No `O` drive
has tested moving-camera confirmation or correct mapped-seat assignment yet.

## 2026-09-28: changed room, green wall, CCW view

User placed the stopped robot at the CCW corner/next-straight boundary, facing
across the straight and next corner toward a green wall. Drive stayed disabled;
`c0` and `camshot` were the only commands. Distances 60/40 cm were measured by
the user from the camera lens. Robot orientation and room lighting were held
fixed; the last side image adds a hand-held diffuse light aimed at the pillar.
Runtime firmware was not read back. The original `.serial.bin` files contain
the complete USB image transfers; PNGs are derived, CRC-checked logical images.

| Image stem (`.serial.bin` and `.png`) | Original SHA256 | PNG SHA256 | Observation |
| --- | --- | --- | --- |
| `20260928_ccw_green_wall_background_01` | `6bcd0ff25d243ac06d6350142fe4421ad205ee444946c3d45de691b4eba5539a` | `a0c3c9d39bed4384d6e79dfffbd376bcff926697594d38d8ef95b864ea4402fd` | No pillar. Green wall/horizon yielded an invalid ~113x13 green component. Exposure34 lines. |
| `20260928_ccw_green_wall_green_60cm_01` | `85bf6a94871c45721a924a303cb51b778857a185d6acbc32f5a923e1cf78c056` | `e61f325464cc24e506eb1c12e7bcada9860b1c4b6ee623bd86dbb98fe4862766` | Centre green, about60 cm. Wall joins the pillar near ROI top; selected blob invalid. Exposure34. |
| `20260928_ccw_green_wall_red_60cm_01` | `b304c07871458dbb40e17d8bb81ba1c4773f4c524bd766354e43165b7568a015` | `07df58a1ffd51a54fcad75c923e05daafe4769f535a538871bd4884ce7d52b07` | Red replacing green at the same marked 60-cm position. One fresh 28-frame interval gave 28 valid red; green background invalid. Exposure34. |
| `20260928_ccw_green_wall_green_60cm_02` | `1d5237e7e4543aa2e74755055f583cc6d3c018732a38c8d3bdb5ba8e751cbd67` | `ab9f43fe9da27d8b375838c86c9c93bf5188fb60aeb56962e351607dc83295d1` | Green restored to marked 60-cm position; wide joined blob again invalid. Exposure34. |
| `20260928_ccw_green_wall_green_40cm_01` | `c2997a9c2952714f192028e63cd94f1cc63682dcee7e8bf910a1642b8d475fc2` | `811f607ccfb6bd8bf71c5046a82cc45a3bb558f6555bcdd3c5646651b211167b` | Centre green moved to about40 cm; alternates valid/invalid because wall still sometimes joins. Exposure37. |
| `20260928_ccw_green_wall_green_40cm_side_01` | `fdba358fd53074296de51548ce02ecec8e8cc46f88d337b318dff28bcfeedfc9` | `b06bab1a603bc59f6193f7fcf3fd010b5774c4cadbe7d2736582f0a6e8adb045` | Green about40 cm, moved right in image (~x265). Front appears nearly black; green mask covers only small upper fragments. Exposure37. |
| `20260928_ccw_green_wall_green_40cm_side_lit_01` | `a49555c6c47ecfa9d49caa0b6e052acf815c36bc9595bb400a208523407c15f3` | `e79138ec0862ff0d65e786cb541c65a4234f3bacbda650f4e74e70ba03f4c8b6` | Same side pose with user-added diffuse light. Recorded pillar brightness changed little; selected background blob invalid. Exposure37. |
| `20260928_ccw_green_wall_red_40cm_side_01` | `b0a2c9a142df867ebbacb8fa64ec58028756d6cf853e7f26a64da00879f971cd` | `25807072ff262e1979c57b5406d53debfb33d7f23f1a03e0a41d8f50a1b34cd9` | Red replacing side green without added light. Red blob valid at x~244, estimated range420 mm, exposure43. Position was nominally the same but image centre shifted ~20 px and estimated range ~20 mm; do not treat this as pixel-aligned comparison. |
| `20260928_ccw_green_wall_green_40cm_left_01` | `bffbb2c35d944591ba2df50df71cf262164186bb28817a73a0b2aa2fb8d74fcc` | `7c62557ac9e3c7339ee8027817bb3a2f2bb707f62c18a756ac1e6c5a89f753eb` | Green moved to the left of the image, nominally 40 cm; body again near RGB16/16/16. It is very close to the left image edge, so this is a colour/contrast check rather than an acquisition-geometry acceptance. Exposure42. |

Ten further stopped green captures used the same robot and room pose, no added
light, and a user-placed approximately 40-cm green pillar on the image right.
Its measured image centre was about x236, inside both safe horizontal windows.
Each separate `c0`/`camshot` transfer passed payload length, CRC and footer
checks; the complete unchanged USB transfers and derived PNGs are archived.
These images were acquired seconds apart because each transfer opened a new
serial session; they do **not** represent ten consecutive 76-ms drive frames.
In a fixed x236, y100..140, 36-px-wide window, the existing HSV-green mask
produced 201..234 samples across all ten images; centre-versus-flank brightness
contrast at y104..132 was 147.7..149.9 value units. The candidate two-signal
rule (>=15 green samples and >=40 contrast) accepted all ten. This demonstrates
repeatability in this one stationary pose, not field-wide specificity.

| Image stem (`.serial.bin` and `.png`) | Original SHA256 | PNG SHA256 | Fixed-window green samples |
| --- | --- | --- | ---: |
| `20260928_ccw_green_wall_green_40cm_right_repeat_01` | `d2f369a355c711d4c3ef2169139f362cd9affb38d6af2e35c11ab7f944b1a418` | `5222c6a4114f34f77d6364ba684709ced3b67fa60d54f7c1c3c75efa051d5090` | 215 |
| `20260928_ccw_green_wall_green_40cm_right_repeat_02` | `fac60d2c03c1d55d807ee9b3e6ee3f1b30c5c5773253a4d0c7b639650f70901a` | `24659e3770b5bd3e4140944776b94cee27f97eb2a3355b8f21954f42ad09e611` | 208 |
| `20260928_ccw_green_wall_green_40cm_right_repeat_03` | `84a68d4e92acb2adaac84b3f017dd5af82121dc4af53cc3bf342cb3d488358fe` | `9ffdbf9abefe0dc9f7682e72be0999860b93e21a5a6054bc5f3899da23225af1` | 208 |
| `20260928_ccw_green_wall_green_40cm_right_repeat_04` | `4ba85c07b2cc5642b9b777fd6256fead7b385ce67336dcf48b7f4b297393b8a0` | `74fa7edeca82c9dcac978c59003aa04ada1ed5dd1d1f2f5ea6b3fb83160ce566` | 234 |
| `20260928_ccw_green_wall_green_40cm_right_repeat_05` | `bdd9a76c282ecd013d815d9d4509c881e873e9388a58b6501441a491806ba` | `2dbce955f3e7f23f0c177e43c475005087cb0dc9b9aa8a2d8b4e3dfc508520e5` | 216 |
| `20260928_ccw_green_wall_green_40cm_right_repeat_06` | `05d12e7869177bde6813ef9413483c553a9c59cf3c97fcd4de364390f6d740ea` | `0149e320ed9fa15355cf9e8a1c9554c71b97d5eb38c86cee266aeab3bf84e7f0` | 212 |
| `20260928_ccw_green_wall_green_40cm_right_repeat_07` | `5119e8667001924eceef32096eb480daecbf5e39b98bd79961ff382d5bf58438` | `6fac8ee777637e2a40267d0917b19ee03fd499ce50f6d7a30991c3edb0c4b109` | 202 |
| `20260928_ccw_green_wall_green_40cm_right_repeat_08` | `96b4f4d6a22aa9b5668f48a7790c7cd79806e2e7d1aaed9a733470fecdbea627` | `339d7bb3fbbe1cdfcfd10afa6fb3d7fd5c1bf5a945496c7e22ef40f1c4f8a799` | 210 |
| `20260928_ccw_green_wall_green_40cm_right_repeat_09` | `32404479fe0b76ef5ca18a9a85439e56bdc9ae6e18e52ea0482f3d66f7ed4998` | `ee9da69d8ab4c84f46646e6c0e723c0efec2958bed4adcbef5b24a40221d2166` | 201 |
| `20260928_ccw_green_wall_green_40cm_right_repeat_10` | `39327664ab3ee4bec41290030511943518e6d3bda204346a900449a5b265173d` | `1e031499156fa0e21593ee0c10ad3db23c938b489d4b75043cbb58efce4d645f` | 218 |

| Fresh stopped serial interval | SHA256 | Result |
| --- | --- | --- |
| `20260928_ccw_green_wall_green_40cm_right_repeat_stationary.txt` | `c645e62c17849296beeb8e9afe369a41b6713363d947fd8c8073bcd1ee265e8f` | Exposure42 lines. Settled counters: 93 green-valid in 112 frames, 76.50-ms intervals. Printed valid blobs x231..233, foot142, estimated range388..389 mm; an occasional green wall blob remained invalid. |

The user moved the same green pillar farther right without changing the room
light. Ten more stopped exports place it around x278, still fully inside the
image and inside the conservative empty-seat centre window (ends near x288).
The old production classifier was valid in only 2/112 fresh settled frames.
At fixed x278, the same 36x40 seat window counted 19..51 green samples; the
middle was 122.7..130.3 value units darker than its brighter visible flank.
Both diagnostic candidate conditions passed 10/10 separate images. The margin
above the 15-sample candidate threshold fell as low as four samples, so this
does not justify a firmware threshold or driving confirmation yet. These are
separate USB captures, not ten successive frames in one 0.8-second hold.
The `.json` and `.rgb565` sidecars alongside this set are derived by the
capture script; the unchanged complete serial transfers below remain the
source evidence.

| Image stem (`.serial.bin` and `.png`) | Original SHA256 | PNG SHA256 | Fixed-window green samples |
| --- | --- | --- | ---: |
| `20260928_ccw_green_wall_green_40cm_far_right_repeat_01` | `996e0b3742d8da5f1d9559a4325de12eca5e9854d5f27cb187545d02e3c1568f` | `506a58892b7893bf42dba0d04bfb01e68fc6acabffb0b88b8b60540677a37283` | 24 |
| `20260928_ccw_green_wall_green_40cm_far_right_repeat_02` | `182dff2ef0f5e5f31e28bbf5b2d03bee0a65087d7a7a8278c506882960081777` | `6126d44922344109be9927e7ada22b3034c00837bd7fb4e290f3a24556ee7a10` | 25 |
| `20260928_ccw_green_wall_green_40cm_far_right_repeat_03` | `14dbb1018b83e9b1bcb39ab7bb93ccd7c12421ffc030c288156a496d5a20bfaf` | `7bb0790995b520848bda88aca9f6bba981d0768a38a8aceab268d455db777d35` | 51 |
| `20260928_ccw_green_wall_green_40cm_far_right_repeat_04` | `34dfba2394df2d337ae0d19805ac7bc93209b407f136518bf486640fac42310b` | `7faceb4249e92c118267169ccc4e1f666d7b6f3d9aa9ab2797c89b047eea8db1` | 24 |
| `20260928_ccw_green_wall_green_40cm_far_right_repeat_05` | `bfc062c7e4e5e118df0b238e76748493aa2214e2bfe6ad30b29a452e0f7a7660` | `c58d27692e876ee45c7b75165f6205d4d733f8a4bc174f5a97dcd658355e7a1d` | 19 |
| `20260928_ccw_green_wall_green_40cm_far_right_repeat_06` | `0f938455c015e1336b469714f359f797db07e66b54fe7ce05dee4b5b365dc78a` | `fed71b32be7432f85379e4e6ba6711dacbd10a1d80da173003b39b034e8c3135` | 21 |
| `20260928_ccw_green_wall_green_40cm_far_right_repeat_07` | `cebe38d7e609a0d5fb0f3a61e1f88018cd4002c4aa03a2fcefc04b4bcbf4f7b1` | `8b40dc82fd146542c0ef164e6211db9bc09267db58c1ab653807c5f871030356` | 20 |
| `20260928_ccw_green_wall_green_40cm_far_right_repeat_08` | `e20a4b99fcfcf563ca0596fdc0e3c2dce3044b302321043a7a6430e71028ee46` | `f00b15f533728beeda8abc3cb773f953b7e76833e59a3179e73b90c6f67ed156` | 27 |
| `20260928_ccw_green_wall_green_40cm_far_right_repeat_09` | `c24de74e8f5e42979e52060adb9c58d9f2a2f20bb4b66d0d295ce8025d586f75` | `5b6cb42d2b39a6bc1975d0e602f16a2ac69ab3d3ceeee7f0a42d1523c5ea557a` | 27 |
| `20260928_ccw_green_wall_green_40cm_far_right_repeat_10` | `3764fe45a50f128b23d4b3b1284d9115045332e54e3ae57849e03a964acce921` | `16b81dc060fbc152d17292df55d9ce5ec4e101cd18e2d8debf79fa44aa51685d` | 27 |

| Fresh stopped serial interval | SHA256 | Result |
| --- | --- | --- |
| `20260928_ccw_green_wall_green_40cm_far_right_repeat_stationary.txt` | `30e664068ebb7958da39fb75309677567fea3c3c3404453aaa32d12e0a632451` | Exposure42 lines. Settled counters: 2 green-valid in 112 frames, 76.50-ms intervals. Most printed blobs selected the narrow green wall region instead of the pillar. |

Fresh stationary intervals are archived unchanged:

| File | SHA256 | Settled counter difference |
| --- | --- | --- |
| `20260928_ccw_green_wall_green_60cm_stationary.txt` | `ea1351508151cfb60590c602b6d86d81a278972433e521417d10c2024f419549` | 4 green-valid / 112 frames. |
| `20260928_ccw_green_wall_green_40cm_stationary.txt` | `74af0591c3f169561c9e0bc47f7e484f7778618789e8f0ef73b8f02c516eab5e` | 46 green-valid / 112 frames. |
| `20260928_ccw_green_wall_green_40cm_side_stationary.txt` | `48bf5aafb43081261e2e6a25218d73fbbab7d13343019f62248d36293637f946` | 0 green-valid / 112 frames. |
| `20260928_ccw_green_wall_green_40cm_side_lit_stationary.txt` | `abb38ab5aa3f79c38008678ddc5b6e878e2d592e2dc965f2fd7de788ab65e789` | 1 green-valid / 84 frames. |
| `20260928_ccw_green_wall_red_40cm_side_stationary.txt` | `a719bd4d6f3f90da356bd0f71ed1d6e97e6f3f691a8f1665286847aed554f549` | 112 red-valid / 112 frames. |
| `20260928_ccw_green_wall_green_40cm_left_stationary.txt` | `9f719ba3d66fc693262eef369f5c6235882d13fcc4c2f4ad194246e606c598da` | 5 green-valid / 112 frames; placement near the left image boundary limits direct comparison to the right-side test. |

Offline replays of these snapshots found that cropping green to y>=96 and
lowering minimum green area to 200 pixels would isolate the centred pillar in
the sampled frames and reject the empty background. **It fails for the side
image**, where the pillar's median HSV value is only about V16 and much of the
surface has no recoverable green hue. Thus this crop is diagnostic only, not
an accepted firmware fix. No camera thresholds or firmware were changed.
The side-red repeat confirms the side view is geometrically usable for a
stronger colour signal, while the side-green lower body is too dark for the
current global HSV mask. An offline sweep of 320 HSV settings with the
existing largest-blob and shape constraints found no single setting that
recovered more than two of five green snapshots without accepting either the
empty background or the red image. This does not establish that no more
structured colour/shape method can work; it rejects a simple global HSV tweak
on the available images.
The mirrored left-green capture likewise has a near-neutral RGB16/16/16 body,
with only 5/112 valid stationary frames. Both sides therefore have weaker
green colour than the middle in this lighting. A dark-silhouette replay from
y>=104 finds upright foreground regions on both sides, but the right region
joins the outer boundary and the left pillar is near the frame edge; this is
not a validated replacement detector. No firmware change was made.

Seat-window replay (working script `local_workspace/camera_green_roi_probe.py`):
the left pillar centre x~28 is outside the acquisition window x30..290 and
conservative empty-seat view x~41..288; the right-side x~267 is within both.
The existing HSV-green mask, summed rather than connected in a 36x40-pixel
window at y100..140, produced 52/70 samples for two centred 60-cm green
snapshots, 166 for centred 40-cm green, and 34 for right-side 40-cm green.
Current-room empty and red control windows gave zero. The older red image has
28 misleading green floor samples at x288, so a pixel-count threshold alone
would be unsafe. Requiring brightness contrast against a 40-px flank
distinguished those available snapshots (green >=60 value units, old red
floor 28). These are single-image retrospective results, not a measured
ten-frame reliability rate or accepted firmware threshold.

These are unchanged serial captures, not full robot driving logs. CDC reconnect
delivers buffered prior-view text before the command acknowledgement; only rows
after the fresh `c0` acknowledgement describe the requested stationary sample.
No independent images or measured reference distance were obtained. Placement
was approximately400 mm. Runtime firmware binary was not read back.

| Capture | Bytes | SHA256 | Report and findings |
| --- | --- | --- | --- |
| `20260927_red_stationary_01.txt` | 4496 | `de553e7249c4432a932d328bbd1f23334e2326c91b548227f0efe7d88730168a` | User confirms red ready. Five fresh RED diagnostics valid; area1148..1992,height49..65,width23..31. Three red centre-pixel HSV samples H349..357,S192..198,V82..90; two background samples excluded. Geometry varies; do not calibrate range from this. |
| `20260927_green_stationary_01.txt` | 3428 | `2e2946f68243a65827f4b765425b9655ccdd9026e30bd59b0671e8a43e34c33e` | User confirms green ready. Five fresh GREEN diagnostics invalid; area2184..2272,width179,height63..65,minX140,maxX318,minY80. Centre HSV H120..165,S63..141,V32..36 fits configured green bounds. Possible connected green background, not verified without neutral-background comparison. |
| `20260927_green_neutral_01.txt` | 3302 | `711d0dce2214ee5c399a75ebaac8639cfdca54e8b00a50e3ef18c16f1cd3b16f` | User adds neutral Pappe, robot/pillar requested unchanged. First fresh row width87 invalid, next four width51..57 valid; exposure338 lines, HSV H120..135,S137..176,V48..56. Between first/last PERF counters96/112 frames green-valid. Background and automatic exposure both change. |
| `20260927_green_return_01.txt` | 3113 | `19dc3852542c1010a79e30e2d7dff797926cc62219d5e947a8fcd14bd7c9c0d3` | User removes only Pappe. All five fresh rows width179,height63..65 invalid,maxX318; exposure136 lines. Green-valid count unchanged across112 fresh frames. Reproduces original green failure, supports background/exposure sensitivity; exact pixel connection not independently imaged. |

Fresh camera intervals approximately76.5 ms, exposure127 red/135 green;
camera error counter0. Historical timing maxima/counters are not new errors.

## Onboard images received

The original `.serial.bin` contains unchanged USB bytes including setup text,
versioned image header,153600-byte payload and footer. PNGs are derived with
RGB565 expansion and mounting rotation; `capture-camera-image.py --replay`
reproduces them offline and verifies CRC. No independent ruler distance/images
of the setup were obtained. Drive flag was off as enforced by the export gate.

| Original serial capture | Bytes | SHA256 | Firmware and image |
| --- | --- | --- | --- |
| `20260927_green_camshot_prototype.serial.bin` | 155055 | `cb6b0aecea8ffdb5771be1dff1c1f979593eae07ae28e0a31f12554d3ff2bc30` | Uploaded M7 `268c4a3936ce7422375b75401d71dc6a34528acf3058396d0ed40aea0a5a9a89`; CRC453795035,frame117,exposure1080. One valid image, subsequent fresh-frame timeout; prototype replaced. |
| `20260927_green_camshot_01.serial.bin` | 155109 | `b38bd68ea468966baeea7fedeb184c473e8075c5bad155c4ea41b738b9fd198c` | Corrected uploaded M7 `73124b456ebac8e9f911256ad7f0a3a426a2c9c9c3d2c6af0e80d6d606f759e8`; CRC147553963,frame297,exposure1080; valid repeated export. |
| `20260927_green_camshot_02.serial.bin` | 155827 | `cc3021a637bc53d6d72de2d9f1a9dae9cfda33f92667f9b6c508496eef1a5754` | Same corrected M7; CRC3011930585,frame130 after stream restart,exposure1080; second valid export. |
| `20260927_post_camshot_stationary.txt` | 1949 | `32d5e248ad5268c5251f6c80c54c6641f7104359c40ac309127f1d5a602f7400` | Fresh normal c0 diagnostics after second corrected export: frame169->197,76.49..76.50 ms,error0 since restart; green still shape-invalid. |
| `20260927_green_distance_check_01.serial.bin` | 154555 | `2fe34a9decc873a44ac2dff8859c5f98c579fcfe4d3a4cb173b1d71386850458` | Same prepared M7; CRC4171968638,frame46536,exposure155. User initially reported about600 mm from camera, then corrected that the green pillar had been moved; actual placement during capture is unknown. Raw173x57/area1800 invalid; column9 candidate27x57/area1424,foot136, valid geometry. Model output~414 mm is not a calibration error without a valid reference distance. Green/red paired comparison invalid; repeat later. |
| `20260927_red_distance_check_01.serial.bin` | 155612 | `6652eab13850f075b0438f51eb41e20328aede075522a020cbf523dc2a7b51f7` | User reported red ready at requested nominal position after moving green. CRC2677930304,frame1527,exposure149. Do not pair with displaced green or infer calibrated distance without measured reference. Follow-on stationary diagnostics below. |
| `20260927_red_distance_check_stationary.txt` | 2490 | `ba76775d672a43b97b175ae786acdf34466527bdfcf3e3e824954a4dc544c109` | After fresh c0, three RED rows valid: x163,width19,height41,area796..820,foot120,estimated571.4 mm,exposure149. Background GREEN thin y80..84 invalid. Buffered prefix before c0 acknowledgement excluded. |

Derived PNG SHA256: prototype
`b99708051071adda63e5540c5336bd9bc548b14a4e4aa7d8b342a5210ff8fd2e`,
01 `a85b5632c426a09499b612bf421a79e5d6c13d97bdc9e657c0ddaa1bc2f9d1bd`,
02 `7fc150e9849d95d9d13fb56348309d67bc1b42d7585b818a5bb5ae12a726f374`.
Distance-check derived PNG SHA256
`a89d073a86ac67ca17cdfabbf732d51f96ab91f21bd5b9232ff714fccc07fc16`.
Red distance-check PNG65560 bytes SHA256
`ec09e048b2fb51966a2325e9a50bd8cc8ef3b304e9adc3e14144c3813e25db02`.
DFU reports successful downloads, not independent firmware readback.
Green mask replay uses `simulation/analyze_camera_green.py`; generated reports
remain under local_workspace. Largest component in final02 is191x55 pixels,
area1832, joining background at ROI top y80 to pillar. Diagnostic crop at y100
isolates25x35 pixels,area612, but splits/removes image evidence; this is NOT an
accepted firmware change. Validate far/partial pillars before any segmentation
change. The original far-red CLEAR veto remains separately unresolved.

## Single image export

With M7 snapshot support installed, drive enable OFF, `c0` selects stationary
camera mode. `camshot` publishes a fresh320x240 RGB565 image, pauses camera DMA
for the stopped diagnostic, and transfers it directly over USB with count, CRC32, byte
order and mounting rotation in a versioned header. The receiver
`scripts/capture-camera-image.py <label> --port <port>` selects c0, receives
one image, rejects corrupt/truncated payloads or missing footer, and stores the
original serial bytes, derived RGB565 payload, metadata and rotated PNG under
`local_workspace/`. It issues no movement/upload command. Binary export is
stationary-only and bounded by a1-second acquisition and10-second USB timeout;
normal telemetry is paused during that command. Camera DMA restarts afterwards,
including failed/disconnected transfers; its stream sequence/error counter
restarts too. The original unpaused-copy prototype stalled after one image and
was replaced. This is diagnostic tooling,
not a change to perception thresholds or competition driving.
