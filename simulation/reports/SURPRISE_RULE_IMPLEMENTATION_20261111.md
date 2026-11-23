# Modified sign recognition and passing side, 11 November 2026

We added height checks to identify a modified sign and preserve its original passing side on all three laps. This report records the initial implementation and M7 build. A [same-day recheck](LAP1_PARKING_RECHECK_20261111.md) added a fallback from an unusable primary height to a valid alternative measurement from the same frame and expanded the policy checks from **21 to 24**. The recheck records its own build. See the [README](../../README.md#modified-traffic-sign) for the current implementation.

## How we confirm and pass a modified sign

Participation is selected before the run. For this revision, `OBSTACLE_SURPRISE_RULE_ENABLED` defaults to `1` in [`include/config.h`](../../include/config.h). To run without the surprise rule, set it to `0` before building or override it with a build definition. The value is fixed during the run.

The camera reports the visible colour. We confirm one modified sign when three distinct frames agree on its visible colour and unusual height at the same mapped position. We retain the visible colour for diagnostics and use the original colour to plan the route.

| Visible sign | Colour the planner uses | Passing side |
| --- | --- | --- |
| Green, 50 or 150 mm tall | Red | Right |
| Red, 50 or 150 mm tall | Green | Left |
| Normal sign, 100 mm tall | Visible colour | Usual side for that colour |

Keeping the sign's position in the map prevents a second colour reversal. The planner uses the original colour on the first pass and reuses the map on laps 2 and 3. The map is frozen after lap 1.

The first sign encountered in the driving direction is ineligible. A later sign on the starting straight can qualify. We determine encounter order from mapped positions along the route, confirmed earlier signs and positions already resolved as empty. A camera can see a farther sign first, so time of first sighting cannot establish encounter order. We handle positions behind the start separately. [Official surprise rule, 9 November 2026](https://wro-association.org/wp-content/uploads/WRO-2026-Future-Engineers-Extra-Task.pdf)

## Estimating height from the image

We use the sign's base to estimate distance. With the surprise rule enabled, a frame without a usable base cannot supply a distance from an assumed **100 mm** sign height. We kept the earlier ground calibration, HSV thresholds and lower image region. We also trace a narrow sign region upwards so a **150 mm** sign can extend above the previous crop. The detector checks base width, the image edge, the ground contact point and side boundaries. It guards against narrow attached background bands that could change the estimated height and against broad regions with ambiguous height.

In the pinhole model, `b` is the base row, `t` the top row, `h` the horizon row, `cy` the vertical principal point, `f` the vertical focal length and `S` the ground calibration scale.

```text
q = (cy - h) / f
camera_height = S / (f * (1 + q*q))
sign_height = camera_height * (b - t) / (b - h)
              * (1 + q*q) / (1 + q*(cy - t)/f)
```

We take the vertical focal length from the horizontal model and assume square pixels. That assumption still needed physical validation on this revision.

| Estimated height | Height class |
| --- | --- |
| 32–68 mm | Short |
| 80–120 mm | Normal |
| 132–178 mm | Tall |

Outside these ranges the height stays unknown. A cropped top can support a tall classification only when the visible part provides a strong enough lower bound. Changes in colour or height, expired votes and repeated calls on the same frame cannot make up three independent votes.

Until the modified sign is fixed in the map, a later sign with unclear height stays unresolved. The controller uses its existing visibility checks and can look again. An unclear object also blocks a vote for an empty position. The green fallback uses the same height decision, and images buffered while leaving the bay serve as preliminary evidence. This revision used the existing driving manoeuvres, speed limits and parking sequence.

## Camera evidence

| Image | Status and result |
| --- | --- |
| `20261027_green_400mm_01.png` | Approved for replay. The detector found a normal green sign and estimated **92.9173 mm** height, base row **138** and **390.7 mm** distance. The recorded set-up distance was **400 mm**. This sample covers one stationary normal-height sign. |
| `20261026_green_new_pose_400mm_01` | Invalid because parts of the image were cyclically shifted. |
| `20261026_green_final_400mm_01` | Excluded from validation. |

We preserved the original files. The [image validation manifest](../evidence/camera_diagnostics/validation_status.json) records approval status and SHA-256 for the four files associated with each sample. The height, green visibility and camera replay tests count approved and hash-verified images for new validation. Images outside the manifest still needed review. The older 26 October replay totals document the earlier detector.

## Checks and build

| Check | Result |
| --- | --- |
| [`check_surprise_rule.py`](../check_surprise_rule.py) | **648** independently projected RGB565 images covered both colours, **50, 100 and 150 mm** heights, **300–600 mm** ranges, three image positions, three base views and three camera heights. **48** CW and CCW association and confirmation cases and **21** rule and error cases passed. The fixture compiles the production image processing and association, then substitutes the downstream map update. |
| Approved stationary image | Detected the normal green sign and preserved its colour. |
| Standard acquisition and green fallback | **52** standard acquisition cases and **7** green fallback cases passed. |
| Corner recovery | Corner 565 passed **10** evidence cases and **27** route rejoins. The negative control reproduced the old failure. Corner 576 passed the direct view, the alternate viewpoint and the negative control. |
| Empty-position and occupancy checks | Runs 589 and 621, synthetic green, red and grey regions, floor lines and background bands passed. The fixture skipped rejected archival images. |
| Later laps | **2,807** modelled routes passed. These cases exercise planning and state transitions with idealised motion. |

The IDE-managed `giga_r1_m7` build passed, with **484,896 / 523,624 bytes** of RAM and **595,392 / 786,432 bytes** of flash. The binary was **598,944 bytes**. Existing serial and unused legacy warnings remained. M4 used the same interfaces and was not rebuilt. We did not upload or drive the robot during this review.

```text
1b4d87efb71b9b36f883b849c9e0bb5e795346526437358a85a54b0bd06f39ba
```

The planned physical check at the time was to use **50, 100 and 150 mm** signs with the installed camera and normal field lighting, then drive in both directions. The team planned to review and approve new images and preserve and hash the complete run logs before analysis.
