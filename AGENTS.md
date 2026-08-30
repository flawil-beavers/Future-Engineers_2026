# Workspace Instructions

## Agent handoffs

- Read `AGENT_DOCUMENTATION.md` before continuing an existing investigation or
  robot-validation sequence.
- Append durable engineering findings and exact next steps there after a
  substantial session. Keep mandatory agent instructions in this file and
  project history in `AGENT_DOCUMENTATION.md`.

## Temporary files

- Store repository-task temporary and generated working files under
  `local_workspace/`, which is intentionally gitignored. Do not create a
  separate `tmp/` working tree in the repository.

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
