# Parking-exit diagnostic evidence

This directory stores the immutable robot logs used to measure parking-exit
pose error, braking, direction-change error and servo neutral. Generated reports
belong in ignored `local_workspace/parking-exit-analysis/`, not here.

## Batch ingestion

For every returned test batch:

1. Copy each complete source log here without editing its telemetry.
2. Name it `YYYYMMDD_log_NNN_cw.txt` or `YYYYMMDD_log_NNN_ccw.txt` using the test
   date, original log number and exit direction.
3. If only part is available, copy the exact excerpt as
   `YYYYMMDD_log_NNN_cw_excerpt.txt` (or `ccw`) and mark it incomplete below.
4. Calculate SHA-256 from the tracked copy and add one row to the table below.
5. Run `simulation/analyze_parking_exit_pose.py` against the tracked file.
6. Add concise findings, limitations and the next test to
   `AGENT_DOCUMENTATION.md`, referring to this repository-relative path and hash.
7. Before finishing, use `git status` to verify the log, this README and
   `AGENT_DOCUMENTATION.md` are tracked changes. Never rely on the USB copy or an
   ignored `local_workspace/` file as synchronized evidence.

Do not rename an existing evidence file, normalize line endings, remove unrelated
telemetry or replace it with analyzer output. Add a new file when new evidence
arrives. Full raw logs are preferred; excerpts are only a fallback when the
complete source cannot be obtained.

## Received runs

| Evidence file | SHA-256 | Firmware/build | Direction | Complete | Physical report | Analysis/limitations |
| --- | --- | --- | --- | --- | --- | --- |
| _No robot diagnostic logs received yet._ | — | — | — | — | — | — |
