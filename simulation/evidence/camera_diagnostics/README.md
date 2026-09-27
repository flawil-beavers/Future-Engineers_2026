# Stationary camera evidence

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
| `20260927_green_distance_check_01.serial.bin` | 154555 | `2fe34a9decc873a44ac2dff8859c5f98c579fcfe4d3a4cb173b1d71386850458` | Same prepared M7; CRC4171968638,frame46536,exposure155. User explicitly confirms green about600 mm measured from camera. Raw173x57/area1800 invalid; column9 candidate27x57/area1424,foot136, valid geometry. Candidate range model~414 mm disagrees with reported600 mm; red same-position comparison pending. |

Derived PNG SHA256: prototype
`b99708051071adda63e5540c5336bd9bc548b14a4e4aa7d8b342a5210ff8fd2e`,
01 `a85b5632c426a09499b612bf421a79e5d6c13d97bdc9e657c0ddaa1bc2f9d1bd`,
02 `7fc150e9849d95d9d13fb56348309d67bc1b42d7585b818a5bb5ae12a726f374`.
Distance-check derived PNG SHA256
`a89d073a86ac67ca17cdfabbf732d51f96ab91f21bd5b9232ff714fccc07fc16`.
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
