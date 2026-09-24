# Workspace Instructions

## Agent handoffs

- Read `AGENT_DOCUMENTATION.md` before continuing an existing investigation or
  robot-validation sequence.
- Append durable engineering findings and exact next steps there after a
  substantial session. Keep mandatory agent instructions in this file and
  project history in `AGENT_DOCUMENTATION.md`.
- For parking/unparking motion-error tests, also read
  `PARKING_POSE_IMPLEMENTATION_PLAN.md`. After each received test batch, append a
  concise entry to `AGENT_DOCUMENTATION.md` with firmware identity, committed
  evidence paths and hashes, physical reports, measured findings, limitations
  and exact next steps. Keep this history concise; do not paste full telemetry
  into `AGENT_DOCUMENTATION.md`.
- For every parking/unparking test batch, copy each complete original log,
  unchanged, from the USB/source archive into
  `simulation/evidence/parking_exit_diagnostics/`. Use the portable filename
  format `YYYYMMDD_log_NNN_cw.txt` or `YYYYMMDD_log_NNN_ccw.txt`, and add its
  metadata row to that directory's `README.md`. If a complete log cannot be
  obtained, copy the exact available excerpt there with `_excerpt` in its name
  and document the missing portion; never present an excerpt as a complete run.
- Treat files below `local_workspace/` as working copies only. Analyzer reports
  remain there because they are reproducible from the committed logs. Before
  finishing the batch, verify that the evidence logs, evidence README and
  `AGENT_DOCUMENTATION.md` are tracked by Git and contain no machine-specific
  absolute paths. Do not claim the batch is synchronized merely because it
  exists in ignored `local_workspace/` or on removable media.

## Temporary files

- Store repository-task temporary and generated working files under
  `local_workspace/`, which is intentionally gitignored. Do not create a
  separate `tmp/` working tree in the repository.
- Do not commit machine-specific absolute paths, including checkout locations or
  removable-drive letters. Use repository-relative paths, portable filenames or
  paths resolved at runtime.

## Competition rules

- Use the official links recorded in `WRO_2026_RULES.md` as the canonical rules
  reference.
- Agents are authorized to download the official rules PDF without additional
  permission when a task materially involves competition rules. If it is not
  already present, save it under `local_workspace/` so repeated searching and
  page inspection can use the faster local copy.
- Before relying on a local PDF, verify its source and version against
  `WRO_2026_RULES.md` and check the official Questions & Answers for newer
  clarifications. Keep the download gitignored and never commit it.

## PlatformIO builds

- Always build and compile this project with the PlatformIO Core installation managed by the IDE on the current machine.
- Build only the environments affected by a change:
  - M7-only changes: build `giga_r1_m7` only. Do not build `giga_r1_m4`.
  - M4-only changes: build `giga_r1_m4` only. Do not build `giga_r1_m7`.
  - Shared interfaces or cross-core changes: both environments.
- Do not compile both cores merely as a general verification step. Compile the
  unchanged core only when the change affects code, constants, or interfaces
  that it actually consumes, or otherwise changes cross-core compatibility.
- `include/config.h` is included by both cores, but an edit to that file does
  not automatically require both builds. Inspect which constants changed and
  where they are used:
  - M7-only parking, navigation, camera, motor, or obstacle constants require
    only `giga_r1_m7`.
  - M4 rear-ToF constants used by `src/m4/rear_tof_m4.cpp` require
    `giga_r1_m4`; also build M7 only if the same change affects its consumers.
  - Constants or protocol assumptions consumed by both cores require both.
- Resolve the executable without hard-coded usernames or home-directory paths:
  - Windows PowerShell: `$pio = Join-Path $env:USERPROFILE '.platformio\penv\Scripts\platformio.exe'; & $pio run --environment <required-environment>`
  - macOS/Linux: `$HOME/.platformio/penv/bin/platformio run --environment <required-environment>`
  - If the IDE-managed executable is not at the default location, use `pio` or `platformio` from `PATH` after confirming that `platformio system info` reports the IDE's current PlatformIO Core directory.
- Keep PlatformIO's default core directory so builds reuse the current account's existing package and tool cache (normally `~/.platformio`).
- Run builds from the repository root so PlatformIO reuses the project's incremental build directories under `.pio/build/`.
- Do not set `PLATFORMIO_CORE_DIR`, redirect the build directory, or create a separate PlatformIO package/cache directory.
- Do not run a clean build or delete `.pio` unless the user asks for it or a clean rebuild is necessary to resolve a demonstrated build issue.
- If sandbox permissions prevent PlatformIO from writing to the current account's cache, request permission to run the IDE-managed executable outside the sandbox. Do not fall back to a separate cache.
