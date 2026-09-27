# Green pillar/background separation: offline experiment

## Result

All three archived onboard images have the same nearby green pillar connected
to background colour near logical y80. Current raw connected components fail
unchanged production geometry limits. Counting colour samples by column inside
the selected connected component, removing columns with fewer than9 samples,
and recomputing connected components produces one plausible green pillar in
each image. This is an offline candidate, not installed perception firmware.

| Image | Raw width x height | Candidate width x height | Candidate area | Raw/candidate bottom |
| --- | --- | --- | --- | --- |
| Prototype | 201 x65 | 25 x65 | 1168 | 144/144 |
| Final01 | 169 x55 | 25 x55 | 1004 | 134/134 |
| Final02 | 191 x55 | 25 x55 | 968 | 134/134 |

Seven through11 colour samples per column recover a plausible pillar in all
three frames;9 produces stable centre x174. Counts must be calculated within
each connected component. Pooling disconnected fragments in one column could
create misleading support. Existing shape/area/top/bottom/centre limits apply.
An already-valid raw detection is returned unchanged. No new pixels are added.

## Comparisons and limitations

- A global ROI crop to y100 also reduces the broad component, but removes
  evidence from far/partial pillars. It is not proposed as a firmware fix.
- Requiring contiguous vertical runs cuts the detected foot in some frames;
  e.g. final02 gives bottom126 instead of134 for run5. This can bias range.
- Column counts retain the selected raw component's bottom in these frames.
  This does not prove the true physical foot is recovered: final02 also has a
  separate lower green fragment not in the largest raw component.
- A/B/A cardboard experiment changed exposure135->338->136 lines. Archived
  images after upload use1080 lines. Do not infer universal colour thresholds
  or calibrated range from these different exposure conditions.
- These are three frames of one nearby view, not three distinct placements.
- Thin/thick horizontal synthetic backgrounds stay rejected. However a
  synthetic background with a vertical green patch yields a valid candidate.
  A binary colour mask alone cannot establish whether that patch is a pillar.
- Do not use a filtered mask to declare a seat CLEAR. Preserve the original
  rejected colour evidence and its veto even if a recovered candidate is used
  for positive detection. Missing filtered pixels are not proof of empty space.
- The far-red CLEAR veto in log398 is separate and remains unresolved.

## Reproduction

Run `scripts/test-camera-pillar-support.py` with the IDE Python: six checks
cover actual images, unchanged valid/fragmented raw detections, background
rejection, two-pillar splitting, separate fragments and the vertical-patch
ambiguity. These are geometry/mask checks, not camera or firmware acceptance.

Run `simulation/camera_pillar_support.py` with the three PNGs from
`simulation/evidence/camera_diagnostics/`. It reads geometry thresholds from
`include/config.h`; green HSV values match the current `Vision::classifyColor`.
Reports and a three-panel comparison are generated under
`local_workspace/camera-green-analysis/` and remain ignored.

## Exact next step

No robot operation or firmware changes were made while the owner was away.
When available, first collect stationary camshot images of green at about600 mm,
red at about600 mm and the same background with the pillar removed. Keep lighting
and chassis pose fixed within each comparison and record actual placements.
These first additions check far acquisition and background rejection. Partial/
edge views and additional lighting still follow before perception deployment.
Only then implement a small positive-detection fallback in M7, retain raw
evidence for CLEAR decisions, measure processing time and retest stationary
acquisition before a powered run. No extra driving manoeuvres are proposed.
