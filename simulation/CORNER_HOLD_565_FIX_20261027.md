# CW565 corner hold: scope lower-fragment occupancy veto

## Evidence and cause

Reanalysis of the already archived, unchanged
[CW565 original](evidence/parking_exit_diagnostics/20261027_log_565_cw.txt),
SHA-256 `351d9338199391250d15f10653813d19e634b87d98d0a3edde92b743d65c0e36`.
No new physical run or USB batch was received.

At62569 ms the robot is(-718.3,860.6,53.83 deg), station6 has one seat CLEAR,
the other at camera range200.1 mm, below the230 mm view minimum. It reverses
110.3 mm to(-782.7,771.1,54.29 deg). The remaining outer position13 is then
visible at bearing-6.98 deg/range309.36 mm. The local search reports no occupied
silhouette, but an untrusted GREEN component remains: the stopped observation
has width19/height45/foot192, therefore top148. Nearby periodic camera records
put GREEN near image-x271--273. It returns and locks unresolved at70394 ms.

`unmappedForegroundBlocksClear()` applied its veto without comparing the
fragment's direction to the individual seat. Its low foot could veto every
unknown seat, despite no local silhouette. That made this corner's existing
reverse view ineffective. The global veto was originally added to protect
512/514's real upper/large GREEN silhouettes and must remain for those cases.

Original RGB images and exact stopped-frame horizontal bounds are unavailable.
The logged geometry and code reproduce the failure mechanism; they do not prove
that the actual scene contained an empty seat or identify the physical source
of the fragment. Synthetic bounds near the recorded centroid are hypotheses.

## Correction

Only a component starting below `OBSTACLE_MAX_TOP_Y` (100 px) may stop vetoing an
individual seat when that seat lies outside the component's full angular width
plus12 deg on either side. The margin is deliberately wider than the existing
6 deg acquisition bearing match. Upper/complete silhouettes retain the global
veto, including512/514. An overlapping low fragment still vetoes the seat.

This helper does not grant CLEAR itself. The existing visible230--600 mm seat
geometry, local silhouette veto, rejected-blob angular veto, observation gate
and repeated frame evidence all remain required. No colour thresholds, speeds,
snap/vote gates, recovery motion or route/join clearance limits changed.

Stopped-view diagnostics now retain raw bounds and weighted centroid so the
next normal log can verify the spatial distinction directly, rather than infer
it from periodic summary frames. Revision: `20261027_corner565_clear_scope`.

## Verification

`check_corner_hold_565.py` exercises production coverage gates and production
`Vision::findGreenSeatCandidate` using controlled RGB565 scenes:

- 10 checks preserve512/514, resolve the isolated lower fragment with a white
  local scene, and refuse CLEAR for an overlapping fragment, upper silhouette
  or an unknown-colour upright object at the predicted position.
- Removing only the scoped veto reproduces the unresolved station.
- 27 perturbed poses around the recorded reverse endpoint pass the existing
  production700 mm CAD pursuit/join preflight with the four actually confirmed
  signs and station6 legitimately CLEAR. No join threshold was relaxed.
- Prior logged perception12 cases PASS, including both disputed edgeGREEN seats
  in512 and the giant GREEN in514. First-GREEN/third-GREEN/capture regression
  also remains passing.

These are synthetic-scene/ideal-motion checks, not a replay of missing images.
They establish the code-level correction and safe model rejoin; actual
resolution of CW565's scene remains pending normal-run validation.

IDE-managed M7 build SUCCESS, RAM442416/523624 and flash532264/786432 bytes.
Binary SHA-256:
`68f093f0f6fd0c03a6e86217336b32b16d92e8d09429c16c3176f0c9f83b11c1`.
M4 is unaffected and was not built.

Next: user uploads M7, performs normal runs and saves normally. Copy originals
FIRST and announce safe removal immediately. Check the new revision, remaining
seat13 coverage after the existing reverse if needed, stopped raw bounds,
absence of false CLEAR on real pillars, then continued route/parking. No agent
upload/drive or special sensor movement was performed. M4 remains unchanged.
