# Normal logging restored after622/623

User asked to disable dense visualization logs and return to normal safe driving.
Removed the temporary M7 RUN_TELEMETRY_POSE_ONCE=1 flag. Default pose-once and
detailed telemetry are both0. Existing compact start/end/fault logs and ordinary
obstacle, camera and parking-exit ToF/braking/reversal/correction diagnostics
remain. Startup is full OBSTACLE_CHALLENGE; parking practice stays disabled.
No speed, steering, route geometry, colour thresholds or obstacle mapping changes
in this restoration. Existing bounded mismatch brake/rebase and checked parking
rescue remain. This is restoration of logging configuration, not a claim that
all physical driving issues have been solved or an arbitrary old firmware reset.

The in-progress extension of10mm rearward preference to every initial park plan
failed its tail-adaptation regression. It was not delivered/uploaded; all its
unvalidated source edits were restored to the previously staged, tested version.
Current parking still has the checked10mm rescue after repeated clearance
replans. User's clarified request for using rear room during normal manoeuvring,
with the original final parked position, remains pending. Do not claim it done.

## Returned evidence

Both originals copied FIRST, source-before/source-after/copy SHA-256 checked;
immediate unplug notice. Portable files/index under evidence/parking_exit_diagnostics/.

| Original | Bytes | SHA-256 |
| --- | ---: | --- |
| 20261031_log_622_cw.txt | 106929 | e25973171c3a02ad384ada3e5748851565fb274d20ebb89c2fbfdc5d41c8b3f3 |
| 20261031_log_623_cw.txt | 109544 | e7bae2554a70241993093ef0cb7baebfd5792addecf95f4c3dfa1956c86bfd2a |

Both CW, diagnostic moduleOct31 15:49:10, path20261031_settled_view,
corner-view module16:25:32. Installed binary not read back; no final parking
revision printed because neither reached final parking.
User reports severe scanning trouble first run, wrong later-lap curve second.
622 footer stopped/mode_pause97286ms,truncated=0: S2 station0 unresolved,
checked parallax and measured reobserve_settled occur, then no safe viewpoint
found and repeated~388-390ms polling gaps during search. The earlier immediate
3deg lock does not recur, but recognition/geometry recovery remains unresolved.
623 footer stopped/mode_pause104033ms,truncated=0: S1 station0 confirmed GREEN
after110mm reverse, later laps built with seat7 occupied (unlike621 which cleared
it). Lap2 starts76942ms,lap3 starts98419ms. Gyro report timeout/restart appears
separately. These logs do not prove dense logging caused the wrong later curve;
no original pixels or independent physical pose trace supplied.

## Validation and next step

Production compact telemetry host test PASS: no pose reads/dumps, under1KiB;
detailed telemetry regression PASS; restored remaining-recipe6/6 PASS. M7-only
IDE build identity is recorded in AGENT_DOCUMENTATION.md after completion.
M4 code/interfaces unchanged; no M4 build, upload, robot motion, commit or push.

Next user uploads normal M7 and performs normal full run/save. Secure new logs
FIRST. Compare ordinary perception/route/tracking diagnostics with622/623;
investigate~390ms view-search polling gaps, camera visibility and the seat7
later-lap bypass. Resume normal-parking rear-room preference only with a passing
transition regression; retain123mm geometry and previous good final position.

Normal M7 build SUCCESS: RAM453816/523624,flash563192/786432,binary566744B,
SHA-25662d0f09351e6eadc65355e10b39fd4e16cf4cd734336271c3a95c3d9b3fcf359.
