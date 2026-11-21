# Modified-sign camera calibration, 19 November 2026

## Results and changes

The revised calibration correctly identified the visible colour and height
class in all **45 approved images** from the current camera mount. It found the
target in every image, and the largest error in forward distance was **13.1 mm**.
The images show centred, stationary signs at **350, 500 and 700 mm**. We tested
50 mm and 150 mm signs at all three distances, and 100 mm signs at 500 and
700 mm. These results describe recognition on the recorded images. Recognition
in motion and at other viewing angles still needs physical validation.

The earlier calibration classified the height correctly in **11 of 45**
images. We checked the CRC and footer on all 48 complete raw transfers. The
three excluded images remain outside the validation set, and the 192 archived
files matched their SHA-256 records and the Git index.
[Initial review](CAMERA_HEIGHT_CAPTURE_REVIEW_20261119.md) · [Camera evidence and approvals](evidence/camera_diagnostics/README.md)

- **Distance and height.** We measured the sign's base at image rows 158, 140
  and 126 px for distances of 350, 500 and 700 mm. An equally weighted fit
  against inverse distance gave a horizon of about 94.6 px and a scale of
  22,430 mm·px. The firmware uses 94 px and 22,400 mm·px. The pinhole fit to the
  red signs gave a vertical focal length of about 259 px. The green signs
  provided an independent colour comparison. We retained the height windows
  and HSV thresholds.
- **Green signs against a green background.** In two images of the 150 mm green
  sign, the target joined a same-colour background strip. We use the measured
  width and centre of the lower part of the sign to constrain the candidate,
  then recount its pixels with the existing HSV classifier. We keep the full
  raw component for occupancy and diagnostics. Candidates still need plausible
  geometry and a valid height measurement.
- **Modified-sign map entry.** `rememberModifiedSign()` records the one accepted
  modified position and its height. The map retains the observed colour in
  `visibleRed` and uses the original colour in `seat.red` to choose the passing
  side. The first sign encountered remains protected. A later sign on the
  starting straight can be eligible. The planner builds the route from the
  stored original colour and reuses it on later laps.
- **Confirmation.** We require three fresh matching frames for a modified sign
  and normally two for a standard sign. The existing expiry, disagreement
  and later-lap map settings still apply. Two approved sequences of a 50 mm
  green sign contained only two frames and correctly remained unconfirmed.

## Image measurements

| Colour | Sign height (mm) | Measured distance (mm) | Correct images | Estimated height (mm) | Estimated distance (mm) |
| --- | ---: | ---: | ---: | --- | --- |
| GREEN | 50 | 350 | 2 | 51.4..51.4 | 350.0..350.0 |
| GREEN | 50 | 500 | 2 | 47.1..48.8 | 487.0..509.1 |
| GREEN | 50 | 700 | 3 | 59.1..59.1 | 700.0..700.0 |
| GREEN | 100 | 500 | 2 | 93.0..93.0 | 487.0..487.0 |
| GREEN | 100 | 700 | 3 | 90.9..90.9 | 700.0..700.0 |
| GREEN | 150 | 350 | 3 | 144.7..144.7 | 350.0..350.0 |
| GREEN | 150 | 500 | 3 | 143.5..153.7 | 487.0..509.1 |
| GREEN | 150 | 700 | 3 | 143.4..143.4 | 700.0..700.0 |
| RED | 50 | 350 | 3 | 51.4..51.4 | 350.0..350.0 |
| RED | 50 | 500 | 3 | 48.8..48.8 | 487.0..487.0 |
| RED | 50 | 700 | 3 | 53.8..53.8 | 700.0..700.0 |
| RED | 100 | 500 | 3 | 93.0..93.0 | 487.0..487.0 |
| RED | 100 | 700 | 3 | 90.9..90.9 | 700.0..700.0 |
| RED | 150 | 350 | 3 | 144.7..147.2 | 350.0..350.0 |
| RED | 150 | 500 | 3 | 143.5..143.5 | 487.0..487.0 |
| RED | 150 | 700 | 3 | 143.4..143.4 | 700.0..700.0 |

## Verification

| Check | Recorded result |
| --- | --- |
| `simulation/check_surprise_rule.py` | Passed 648 synthetic projections, 48 CW/CCW association cases, 24 rule and error cases, 45 hash-verified images and 16 recorded image sequences. The sequences check confirmation and reuse of the original colour through lap 3. The two-frame modified-sign sequences remained unconfirmed. |
| `simulation/check_far_obstacle_acquisition.py` | Passed 52 cases, including map freeze and selection of the passing manoeuvre. |
| `simulation/check_green_seat_visibility.py` | Passed synthetic occupancy, brightness, background and floor-line cases, base validation and the negative control. Its fixed archived query point covers a limited set of sign heights. |
| `simulation/check_parking_seat_foot.py` | Passed the historical pixel cases using their October calibration and the current geometry checks for the sign base. |
| `simulation/check_later_laps.py` | Passed 2,807 route and state cases. |
| `simulation/check_lap_handoff.py` | Passed all six direction and speed cases without an initial steering target jump. |
| `scripts/test-camera-image.py` and `scripts/test-camera-pillar-support.py` | Passed five and six tests, respectively. |

The IDE-managed PlatformIO build for `giga_r1_m7` succeeded. It used
484,896 of 523,624 bytes of RAM and 597,432 of 786,432 bytes of flash. The
600,984-byte binary had SHA-256
`059f6fb03f35efc3f11c10cfcb8c17e32dd08d22234a14d4dfacdcef51bfc92a`.
Existing Serial and legacy warnings remained. M4 and robot upload were
outside this build.

## Physical follow-up

The camera model affects every distance and sign-position estimate made from
the image. At this revision, the next validation step was to use the same
mount during ordinary clockwise and anticlockwise runs. The test plan called
for a standard first sign and one modified 50 or 150 mm sign, with the declared
participation matching the firmware setting. Check the expected original
colour and passing side on all three laps. Save the complete original logs and
look for `[SURPRISE] revision=20261119_measured_height` and exactly one
`locked seat/height/visible/original` record.

This camera test covers stationary recognition. The separate parking limit at
a 232.5 mm gap remains documented in the engineering handoff. Use normal
saved runs for follow-up. [Engineering handoff](../AGENT_DOCUMENTATION.md)

The original review checked the official season page, the 9 November surprise
rule and the Q&A on 19 November. It recorded no newer clarification at that
time. [WRO season page](https://wro-association.org/competition/2026-season/) · [Surprise Rule](https://wro-association.org/wp-content/uploads/WRO-2026-Future-Engineers-Extra-Task.pdf) · [WRO Q&A](https://wro-association.org/competition/questions-answers/)

## Revision record

The local branch fast-forwarded through `caa2709`, incorporating four remote
commits for visualisation and documentation history. The handoff conflict was
resolved by keeping the remote summary structure and archiving the original
local text. The camera revision retained the route, speed, steering and parking
settings.

| Source file | SHA-256 |
| --- | --- |
| `src/vision.cpp` | `89615bf7c65450de85e27e3d2c413c0116a3f75894cd6c374f3d4384be04d7cb` |
| `src/obstacle_path.cpp` | `39476fa7543f6a6e4c4c34e3649a4eea09a13b58a3826979a79f0923bac2778f` |
| `include/config.h` | `9bd289ba8a01aec77b0f2186bd6b1ac002b013087ac719a4080174378dfd71fb` |
| `include/vision.h` | `c112c3ee7596627cf4d6f4921f8f0dc878572b1b278568ed00503d9f25b5f180` |
| `include/obstacle_sign_height.h` | `e81121e081f852ee4baa0a1ccc7cc6f8686f02524b076846020fe9b0b71062f5` |
