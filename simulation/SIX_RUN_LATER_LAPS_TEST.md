# Six recorded-map speed trials (2026-10-06)

Upload the current M7 build first. Official `O`, first-lap flag false,
three-lap flag true. Lap 1 learns; laps 2/3 command the existing speed profile
times 1.50 (rounded up), maximum 390 mm/s. Motor acceleration, explicit test
caps and final runout/brake retain their limits. Actual measured speed and
lap time must be established from logs; ideal geometry does not validate slip
or servo dynamics at the new speed.

## Position convention

Inner = next to centre wall; outer = next to outside wall. In each ordinary
straight, P1 is the first marked station after its entry corner, P2 the middle,
P3 the last before the next corner, **in that trial's travel direction**.
S1/S2/S3 are the straights reached after the first/second/third corner from
the parking start. Thus S1 is the left straight for CW, right straight for CCW,
when viewed as in the earlier photos with the start closest to the viewer.
Start left/right below uses that same photo view, not the driver's view.
Use marked stations and centre each pillar on its square.

| Field section | A: CW, trials 1–3 | B: CCW, trials 4–6 |
| --- | --- | --- |
| Start | GREEN middle, inner; other start stations empty | RED left, inner + GREEN right, inner; middle empty |
| S1 | RED P1 inner + GREEN P3 outer | GREEN P1 outer + RED P3 inner |
| S2 | GREEN P1 inner + RED P3 outer | GREEN P2 outer only |
| S3 | RED P2 inner only | RED P1 outer + GREEN P3 inner |

Unlisted places are empty. Normal middle signs are solitary; pairs occupy
only P1/P3. The start signs are inner. A exercises the successful GREEN-middle
start, opposing end passes, a GREEN/GREEN corner and an inner RED singleton.
B exercises CCW's front GREEN and stored rear RED, a GREEN/GREEN first corner,
outer middle GREEN, other passing-side/seat combinations and opposite-colour
transitions. Together these deliberately include both directions and all three
station positions. They are a focused first batch, not exhaustive start layouts.

## Run sequence and physical report

Update after CW logs 458-460: their holds were the initial false lap wrap
and a 2.1-degree settled exit rejected by the old2-degree gate. Both software
causes are corrected; repeat A in CW first on the new M7 build. User confirms
these three runs had no contact; log457 was a different layout.

User's revised batch order: each fixed physical setup gets two or three CW
runs, then two or three CCW runs without moving the pillars. Only reinterpret
P1/P3 and S1/S3 for the opposite direction; do not mirror the setup. After
that change to setup B and repeat both directions. Save/copy originals before
analysis so the stick can be reused. The table above defines A in CW and B
in CCW; its station labels are not instructions to move signs when reversing.

1. Photograph A once, fit the usual CW parking start, reset for a fresh map.
   Remove the laptop cable from the field before enabling drive.
2. Three unchanged trials, each exit plus all three laps plus autonomous stop.
   Reset before each trial; leave the map/layout unchanged within a trial.
3. Photograph B once and use the established CCW parked orientation. Repeat
   three unchanged complete trials, each with a fresh reset.
4. If contact or a dangerous trajectory occurs, stop and report it before
   repeating that failed case. Do not silently relabel an interrupted run as
   successful; retain its log as evidence.
5. Record A1/A2/A3/B1/B2/B3, log number, last completed lap, contact/nearest
   place, autonomous vs manual stop, and any wrong-side passing. Note whether
   a failure was on discovery lap 1 or fast recorded-map lap 2/3.

Leave logs on the USB stick and provide both setup photos plus physical reports
with the six logs. Incoming originals will be copied unchanged and SHA-recorded
under `simulation/evidence/parking_exit_diagnostics/`; no new batch exists yet.
The final halt saves the log automatically. Parking entry is not part of this
batch. No agent upload or commit has been performed.


## CCW correction after461-465

User confirmed unchanged A, only lap1 GREEN contact at S3 entry. Repeat A CCW
on the next M7 candidate after the current prior-firmware CW tests are reported.
New CCW first-edge reference keeps the observation start at rear axle x360;
its84mm camera arc is selected by explicit reference identity, not direction
alone. Second-edge fallback remains55mm. Keep fresh-reset/cable-free procedure.
Known injected adjacent same-colour signs now get a continuous corner in lap1.
Watch right wheel at S3 entrance, and front-place scan clearance near the pink
limits. Two or three full trials; report contact, halt and lap before proceeding.
Model checks do not replace physical acceptance.


## Superseding next batch after CW466-470

Keep physical A unchanged. Upload the candidate described in
`LATER_LAPS_IMPLEMENTATION.md` (CAD steering, real wall references,280mm
later pursuit), then2-3CW trials followed by2-3CCW trials on the SAME setup.
Do not change to B yet. Lap1 and laps2/3 require separate contact-free acceptance;
watch CW S3 middle-inner RED in lap2 and CCW S3-entry GREEN in lap1.
The field layout is not mirrored for these robot trials. The additional host
CCW mirrored-A case is a legal symmetric software fixture, not this physical
CCW replay. Retain all interrupted logs and physical reports.
