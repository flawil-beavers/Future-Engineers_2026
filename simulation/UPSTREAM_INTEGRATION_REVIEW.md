# Review and selective integration of upstream changes — 2026-10-09

Reviewed upstream commits ab58250, f2876c1, 31af6e6, 6ab362c,
and0d091f4, from local45e173d to origin/main0d091f4. The changes were fetched
before reviewing. Local runtime safety, gyro continuation and Inspector comparison
work were backed up under ignored local_workspace/integration-review/.

## Engineering judgement

Overall original submission:6/10. Offline Inspector/view/export work:8/10;
firmware telemetry as proposed:4/10. These are engineering judgement scores,
not measurements of robot success. Useful functionality and meaningful tests
exist, but default logging and integration consequences were insufficiently
bounded for this robot. Most added physical lines are official SVG artwork,
not controller code; raw added-line counts are not code-quality metrics.

## Accepted

- Whole-field and detail views; accepted recorded pillars, correct nominal seat
  rotations, local/field separation, round/layer filters, standalone SVG export.
- Offline embedded mat and reproducible source/asset identity/alignment checks.
- Route-version transactions/deltas, explicit missing-plan behavior, command vs
  measured motion qualifications, outcome/lap timing and idempotent completion.
- Optional laptop PNG/SVG batch output; procedure-specific grouping for accepted
  CCW first-edge starts; documentation distinguishing historical evidence.
- Existing regression checks and historical raw evidence remain unchanged.

## Changed before accepting

- Original new64/24/8KiB allowances total96KiB. An8KiB remaining-capacity gate
  protects only the tail, not future ordinary parking/fault diagnostics. Older
  logs already occupy about158KiB; adding96KiB can crowd out critical evidence.
  Production now defaults RUN_TELEMETRY_DETAILED=0: start/lap/end only, event
  allowance2KiB plus bounded marker/footer. Typical five-record timing stays
  below1KiB. No new periodic poses/routes or estimator reads. Optional detailed
  output additionally shares a24KiB total cap; saturation remains explicit.
- Detailed192-point route dumps are synchronous formatting bursts. Host tests
  cannot establish MCU timing; the feature remains opt-in, not competition default.
- The plan-version control was hidden according to session count. It now depends
  on the selected session's actual route revisions; a one-session file with
  multiple plans can select them, while empty multi-session logs cannot.
- Preserve local Soll/Ist comparison, CSV, hash-bound notes restore, strict
  numeric parsing, correction identities and geometry-gap breaks. The new whole
  view also honors accepted scan and sequence-based correction breaks.
- Preserve successful parking motor hold and automatic gyro continuation.
  No permanent gyro-mode cancellation has been reintroduced.
- Clamp logger tail reservation to prevent unsigned capacity underflow.
  Strided route points are copied into a real float array instead of indexing
  past a struct member through reinterpret_cast.

## Remaining limitations

Onboard estimated poses are not actual chassis ground truth. No complete plan
exists for primitive final-parking control; its control version intentionally
has zero points. Default compact timing does not create full RUN_POSE coverage;
existing sparse traces remain sparse. The optional Python historical overview
explicitly omits FINAL_PARK_TRACE, unlike the HTML Inspector. Future raw log
verbosity can still overflow the existing buffer; no larger RAM buffer was added.
Physical gyro recovery, sensor gaps, slip, optical conditions and timing still
need normal-run evidence. No firmware upload or push is authorized/performed.

## Validation

See the final dated AGENT_DOCUMENTATION.md entry for build identities/results.
Tests and generated reports stay under ignored local_workspace/.

Visual inspection of the Python476 PNG exposed another original defect:
connector replans were concatenated into a false chord. They now render as
separate versions, also splitting missing waypoint indices; regression added.

Final checks: both IDE firmware builds pass; M7RAM433024,flash505816;
M4RAM59768,flash155224.41Inspector checks/125original logs andDOM adapter,
13runtime groups, compact+detailed telemetry, mat alignment,6visualization and
7batch tests,2807path/2970approach/396parking cases pass. Native browser and
physical robot acceptance remain unverified. Full identities in project history.
