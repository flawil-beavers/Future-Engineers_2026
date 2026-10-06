# CW/CCW471-476 review and next test

## Complete evidence and physical outcome

Immutable originals: `evidence/parking_exit_diagnostics/20261006_log_NNN_cw.txt`
for471-474, `_ccw.txt` for475/476; source/archive SHA256 and lengths in that
archive README. Parking build `Oct__6_2026_19_54_43`, connector `Oct 6 2026_19:54:26`
are consistent with preceding candidate; installed binary was not read back.
User reports unchanged A. File numbering, not removable-media timestamps, orders
this batch. Only two new CCW source files are present.

| Log | Outcome | Measured finding |
| --- | --- | --- |
|471 CW1| GREEN-start contact then halt | S0 middle incorrectly CLEAR at pose300.3,-1221.6,147.7deg; subsequent false REDseat0; ordinary non-avoidance connector; stall5.3mm/1000ms |
|472 CW2| Holds after exit | Settled heading3.23deg rejected by3deg prerequisite, before camera/start scan |
|473 CW3|3laps, contact-free per user| S3 inner-wall capsule minimum57.5/56.5mm in sampled laps2/3; rear-axle x627.5/626.5mm |
|474 CW4|3laps, contact-free per user| S3 sampled wall minimum81.2/68.6mm; rear-axle x651.2/638.6mm |
|475 CCW1|3laps, contact-free per user| First-edge accepted, remaining reverse30.1mm; corrected scan startx357.7,84mm arc; sampled whole-field wall minimum82.8/78.9mm |
|476 CCW2|3laps, contact-free per user| First-edge accepted, remaining reverse41.9mm; corrected scan startx357.6,84mm arc; sampled wall minimum85.8/85.4mm |

Wall figures are capsule calculations on onboard poses at250ms intervals, not
independent measurements. They can miss the closest instant. CW human report
4-5cm is consistent with sampled modeled5.7cm. CW laps2/3 last0-based path indices
also show xabout1000mm at S3 entry and headingabout220deg on the subsequent inward
swing, followed by a strong opposite steering correction near the RED.

CW473/474 map still omits physically reported S2 first-inner GREENseat12.
CCW475/476 identifies the same physical sign (CCW seat17). A contact-free lap
is therefore not proof of full CW perception; keep this in the next review.
No RGB565 images of471 were saved, so optical cause of missing GREEN cannot
be reconstructed. The false RED is geometrically inconsistent with the known
close start seat: observed foot174/178 vs expectedabout152. Its physical image
source is unproven; parking boundaries are one plausible source.

## Candidate corrections

- CW prerequisite5deg (from3), with finite initialized pose and unchanged actual
  swept scan/connector preflights. The independent2deg ToF pose-adjustment gate
  stays separate. Production gate and8943 scan/connector cases include472 pose,
  perturbations and the5deg boundaries.
- Close mapped start seats require projected ground foot agreement18px, rejecting
 471 false close RED projections without changing distant/normal-section snaps.
- Mapped-seat GREEN fallback accepts brighter dominant-green pixels up toV200
  after upright silhouette validation. Full-frame thresholds/FOV unchanged.
  A matching upright silhouette without sufficient colour blocks empty evidence;
  it never confirms GREEN or injects a route. Existing retry/scout supplies another
  view; if colour remains unavailable, unresolved safety holds remain necessary.
- Initial observation now participates in bounded GREEN_START_CHECK logging.
  A CW middle-empty decision also saves one upright64x96 RGB565 window sampled
  every2px as START_ROI/48rows, <=8kB in the existing onboard log. Cached frame
  only, stopped observation only, no extra capture/save/motion. No471 source
  image is retroactively available; this sampled crop is for the next run.
- Official laps2/3: same-colour last sign -> solitary next middle sign can carry
  the existing inner lane radially through the corner, rather than return to centre.
  Minimum radius180mm; opposite-colour transitions, discovery and O3 excluded.
  Carry into the start section is excluded to protect its existing seam blend;
  an early unrestricted carry also reduced a start-related reserve to15.6mm.
  Full-footprint preflight may reject the shortcut and retain the prior route.
- Reduced avoidance only at solitary middles in NORMAL sections: moderate210mm,
  extreme190mm. Start-section and end-pair clearance targets retain previous values.
  A broader210/190 reduction was rejected: its matrix minimum pillar15.2mm was
  unnecessarily small. First-lap avoidance and1.50 later speed factor are retained.

Actual-source Vision replay confirms stored GREEN and rejects stored RED and
empty-front samples; synthetic bright/dark GREEN, gray occupancy, red, broad wall,
floor line and wall-plus-floor tests pass. Production projected-foot test rejects
471 projections, accepts plausible feet and preserves non-start compatibility.
This does not prove missing471 GREEN would now be classified; test physically.

Reports are reproducible under `local_workspace/later-tracking-471-476/`,
`local_workspace/green-seat-visibility/` and `local_workspace/later-laps/`.
Scripts: `analyze_later_lap_tracking.py`, `check_green_seat_visibility.py`,
`check_parking_seat_foot.py`, `check_cw_start_planner.py`, `check_later_laps.py`.
Do not use ideal CAD rollout as a physical speed/servo-lag guarantee.

## Next physical gate

User uploads M7 candidate, same A unchanged, drive cable removed. Three fresh
CW trials (including start); then two CCW trials. Require all three laps,
no contact, correct passing sides. Watch S0 GREEN, S3 RED/wall and the preceding
corner; preserve logs even on holds. Check GREEN_START_CHECK/start_foot_mismatch,
mapseat12/17, inner-lane candidate/fallback and LATER_TRACK. Save complete logs
before analysis. No automatic commit or upload was performed.


## Final verification and firmware

8943CW start cases,2400footprint/state comparisons, CCW first-edge state,
Vision/projected-foot/ROI regressions pass.2803CAD-route cases pass a30mm modeled
pillar-reserve floor: matrix minimum wall90.4/pillar36.1mm. Exact A CW minimum
wall121.8/pillar69.8mm; physical A CCW wall96.7/pillar37.6mm. These are ideal-model
minima, not predictions of exact real clearance. No servo-delay/ToF feedback model.
Parking analyzer94complete sources/95sessions. ROI logger bounded7130bytes.
Final M7 build RAM432776/523624, flash484344/786432, SHA256
`6521bc8ecc4caf1dd373231d9ee4e1d5d1b554759d2bed61aae6da5b72081c3a`. Existing Serial warnings. No M4 build/upload/agent commit.
