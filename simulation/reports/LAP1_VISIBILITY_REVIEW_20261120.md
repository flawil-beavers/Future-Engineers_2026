# First-lap block association and visibility

The normal camera path now retains a pending observation when a frame is assigned to an already confirmed seat. The mapped seat supplies no second vote. Another accepted, unconfirmed seat still resets the competing observation. Matching evidence must arrive within the existing 400 ms window and pass the existing projection, colour and height checks. The far test uses distinct frame sequences and moves the robot estimate 21 mm between the two views at roughly 700 mm range.

When two places in the next station remain unresolved and only one has a recent vote, the controller can bias its target toward that place by up to 15 degrees. It checks the resulting steering arc and limits how quickly the correction builds. The existing bypass guard retains control until it releases, and an unsafe aim leaves the ordinary view strategy in charge. A hint feeds steering rather than the block map.

A pending vote now blocks an EMPTY decision at its seat until it expires or is resolved. A known pillar also vetoes EMPTY when its 42.5 mm movement radius intersects the horizontal ray to the target. The rule is deliberately conservative for foot visibility and keeps the existing field-of-view and nearby corner requirements. Once the ray opens and the normal clear frames arrive, the station can resolve without a retreat. The earlier experiment that released steering while still in the protected bypass worsened later viewing angles and remains excluded.

## Checks

- Far acquisition 56 checks pass. The interleaved RED and GREEN tests confirm with two matching fresh frames and reject confirmation when their evidence is older than 400 ms. Log 3 still acquires its far corner RED and rejects a next-lap projection.
- The look-direction test passes 270 policy cases and 286 archived poses. Targeted cases check each row in CW and CCW and preserve the earlier protected interval. The existing CCW steering dispatch passes 362 controls.
- The EMPTY tests check a blocked ray, a fresh candidate and a clear ray after expiry. All 10 log 565 evidence checks, 27 rejoin cases and 33 recovery dispatch checks pass. The active same-station search test also passes.
- The special-rule fixtures pass 648 rendered detections, 48 associations, 24 policy cases, 45 individually approved camera images and 16 captured sequences.
- Three historical checks were also attempted. The 625 negative control, the 556/558 row case and the 547/550 edge-aim case already fail with the preceding production functions under the current fixture calibration. Their results do not establish physical recognition in those cases.

Path revision `20261120_lap1_visibility`. The six previously saved originals were reverified against the connected source before analysis. The [verification manifest](../evidence/log_intake/20261119_batch_3_6/lap1_visibility_source_verified.json) records the hashes. Normal CW/CCW runs must still verify earlier association, fewer unnecessary retreats, and wheel clearance after the first pass. The older wrong-visible-colour case lacks an approved image and remains unresolved.

Final IDE-managed M7 build succeeded. RAM 452176/523624 B, flash 602328/786432 B, firmware.bin 605880 B, SHA-256 `4eb1a305d1f4b338310b7fc12f77d2533045bef73dd885affff10c7efa8c9158`.
