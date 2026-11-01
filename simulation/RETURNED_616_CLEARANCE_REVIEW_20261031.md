# Returned616 and parking-clearance clarification

Complete unchanged [log616](evidence/parking_exit_diagnostics/20261031_log_616_cw.txt):
157693bytes, SHA-256
`1c68941851e2bb7dfde788998edc1abeccc8e0565993816784f74c29a524df82`.
Source-before/source-after/copy matched; unplug notice preceded analysis.
CW, path20261031_view_reobserve/parking20261031_checked_recipe,
build Oct31 14:40:53; installed binary not independently read back.
Completed full course and parking in149.001s, no buffer overflow found.

User corrected straight outside-to-outside wheel width from120 to123mm.
The5mm request referred to manoeuvre clearance, not parked-X displacement.
Final request: about8mm more front room during steering, use plentiful rear
space; keep the good longitudinal final position, only a few mm shallower.
User reports latest parking looks better; also an abrupt stop near GREEN.

## Recorded findings

- First parking recipe search100ms, six segments at3mm runtime margin.
- First arc measured radius factor1.136; segment_pose caused general replanning
  taking8806ms. Later short replan94ms. Final contained/stopped pose
 422.2/-1375.4/178.5deg. These timings are real logged MCU times.
- Near S3 GREEN approach, log records Sensor report timeout; restarting SPI/SH2,
  stream restarted, then main-loop poll gap1601ms. At72054ms profile speed0
  resumes from stopped pose922.0/239.9/287.49deg. GREEN later confirms seat22
  at75371ms and course continues. No corner-view lock is recorded in this run.
  The transport restart explicitly stops the motor before blocking BNO startup.
  This explains an abrupt pause; underlying report loss (wiring, power, sensor
  or transport) cannot be distinguished from this log. Do not attribute the
 1601ms post-restart warning itself to camera calculation, or disable stale-gyro
  drive protection. Gyro driver/timeouts unchanged in this correction.

## Final correction

- Restore preferred parked axleX60mm and original end-reserve acceptance
  rear>=10/front>=30mm; remove the mistaken new axleX<=60 completion restriction.
  The good prior longitudinal final pose remains acceptable.
- Straight parking geometry123mm, including wheels. The shared footprint accepts
  an explicit outside width for final parking; its unchanged125mm default keeps
  the pre-existing conservative navigation/exit geometry. Individual wheel
  positions and full-lock polygons follow the parking width; CAD steering-angle
  calibration and empirical turning-radius model remain unchanged.
- While steering, ordinary-reserve checks also inspect the footprint shifted
 8mm forward: preferred extra nose room without adding rear padding. Prefer a
  fully checked8mm straight reverse prefix before the first arc, solving the
  remaining straight and final alignment back to the original parked-X goal.
  No blind offset command. Tight1mm escape keeps actual physical sweeps even
  when the current pose cannot already supply the preferred extra8mm reserve.
- Depth preference only127mm lateral axle distance versus125mm previously:
 2mm shallower, nominal open-edge reserve200-123/2-127=11.5mm. A blanket20mm
  reduction remains unjustified. Fast recipes try127/125mm; full fallback may
  use deeper poses if required to fit a complete safe entry.
- After the first quick-recipe arc, solve its remaining straight and turns
  using the measured pose/radius. Full1mm swept preflight and existing tight
  fallback required before reuse. Six mirrored cases with radius factors
 1.05/1.10/1.136 all reuse safely; there is no exhaustive search in this branch.
  Otherwise normal checked planning remains. This targets616's8806ms pause.
- Check radius uncertainty at EVERY candidate arc start, including after a
  straight prefix. Previously only a first-segment arc got this preflight;
  partial-entry tests exposed repeated forward/back8mm replans hiding a blocked
  next arc. Reject such candidates before execution rather than oscillating.

## Validation

396 complete state-machine cases+5registration controls PASS;350 adaptive
radius/gap/recovery cases+8health PASS;84 brake-overrun+8health,36tight,
42segment-resume+negative controls PASS.123mm geometry6 checks verify measured
straight width, unchanged parked-X acceptance, and a backward move obtaining
extra nose room. Remaining-recipe6/6 mirrored radius cases PASS. Near-capture
quick recipes18/18 plus60 parking checks and20 tracking controls PASS.
Full polygon/reference equivalence1440wall/12168rail,4practice and2registered
replan cases PASS. No new physical validation of the123mm/front8mm version yet.
Original pixels and independent external pose measurements unavailable.

Next: upload M7, normal lot start/full runs/save; secure originals first.
Compare remaining-recipe log versus8806ms full replan, real search_ms/dwell,
front/rear contact during steering, final depth and finished pose. Preserve gyro
reports; repeated report-timeout stops require diagnosing that transport/sensor
fault, not bypassing its stop. No dedicated ToF sweep or extra sensor pause.

Final M7 IDE build SUCCESS: RAM453720/523624,flash558264/786432;
binary561816bytes SHA-256
`52abb5777036e7bb3cd869cbee3100f7cc7694a555ad71e5659e76a8ccec9408`.
Parking revision `20261031_front8_123mm`; unchanged path `view_reobserve`.
M4 not built (no consumed changes), no upload or motion performed.
