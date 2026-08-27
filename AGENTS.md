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

## PlatformIO builds

- Always build and compile this project with the PlatformIO Core installation managed by the IDE on the current machine.
- Resolve the executable without hard-coded usernames or home-directory paths:
  - Windows PowerShell: `$pio = Join-Path $env:USERPROFILE '.platformio\penv\Scripts\platformio.exe'; & $pio run --environment giga_r1_m7`
  - macOS/Linux: `$HOME/.platformio/penv/bin/platformio run --environment giga_r1_m7`
  - If the IDE-managed executable is not at the default location, use `pio` or `platformio` from `PATH` after confirming that `platformio system info` reports the IDE's current PlatformIO Core directory.
- Keep PlatformIO's default core directory so builds reuse the current account's existing package and tool cache (normally `~/.platformio`).
- Run builds from the repository root so PlatformIO reuses the project's incremental build directory at `.pio/build/giga_r1_m7`.
- Do not set `PLATFORMIO_CORE_DIR`, redirect the build directory, or create a separate PlatformIO package/cache directory.
- Do not run a clean build or delete `.pio` unless the user asks for it or a clean rebuild is necessary to resolve a demonstrated build issue.
- If sandbox permissions prevent PlatformIO from writing to the current account's cache, request permission to run the IDE-managed executable outside the sandbox. Do not fall back to a separate cache.
