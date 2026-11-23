# USB save failure and heap usage, 19 November 2026

## What we observed

The GIGA flashed four short and four long red pulses when we tried to save. This matches the documented Mbed OS crash pattern. We could not confirm a completed save. The 656 existing files were unchanged, and the USB drive contained no run from the new modified-sign firmware. We then cleared the drive.

The same **16 GB FAT32** drive passed the Windows filesystem check and **32** write, flush, close and SHA-256 readback tests totalling **8 MiB**. Those checks covered the drive on a PC. Power and USB compatibility on the robot were still open. [PC test evidence](evidence/log_intake/20261119_save_check/README.md)

Neither USB CDC nor DFU was visible after the crash, so we could not capture a RAM dump. The installed build, Mbed error code and failing stack frame were unknown. We did not reset, upload or move the robot during this review.

## What made the heap a concern

In the earlier ELF, `__end__=0x24076630` and `__HeapLimit=0x2407fc00` left **38,352 bytes** for the heap before runtime allocations and fragmentation. Static RAM usage was **484,896 / 523,624 bytes**. The ARM code shows that the USB host allocates **3,644 bytes** in `USBHost::getHostInst()` when `msd.connect()` runs. It also needs a **3,072-byte** USB thread stack and allocations for the filesystem and other bookkeeping. The main thread's **32 KiB** stack was already counted in `.bss`.

## What we changed

- **Free memory without changing vision.** We packed the six RGB565 colour classes into four-bit entries, reducing the lookup table from **65,536 to 32,768 bytes**. Initialisation and lookup still give identical classifications. We kept the HSV thresholds, detection windows and **192 KiB** log buffer, along with the existing speeds and route.
- **Record save progress.** With a USB serial terminal connected and the drive outside `DC_ENABLED`, `USBLogger::log_save_stage()` prints one line per stage. It reports raw Mbed heap counters as `[LOGGER USB] stage=... heap_used/peak/reserved/fails=...`. The stages are `power_settle`, `connect`, `mount`, `directory_open`, `directory_scan`, `file_open`, `write_flush` and `finalize_verify`. The diagnostics allocate no extra heap and produce no extra output while driving. They run in the save path to avoid USB writes after a fatal error. The USB worker runs in parallel, so the last stage gives context about the crash but cannot identify the failing stack frame.
- **Keep the existing recovery.** We kept write, flush and final verification, along with the full retry into a new file. We remove saved bytes from RAM only once the save succeeds.

The revised ELF had `__end__=0x2406e638` and the same heap limit, leaving **71,112 bytes**. The net gain was **32,760 bytes**, with **8 bytes** used by diagnostics or linker layout. Static RAM was **452,136 / 523,624 bytes** and flash **598,048 / 786,432 bytes**.

## Verification and build

| Check | Result |
| --- | --- |
| `check_packed_vision_lookup.py` | **131,072** exact comparisons passed. It checks all **65,536** RGB565 values twice after `begin()` against the existing RGB-to-HSV classification. We completed the Arduino types in the host fixture without changing firmware for the fixture. |
| `check_surprise_rule.py` | **45/45** approved images, **16** real image sequences, **648** rendered cases, **48** association cases and **24** rule and error cases passed. The effective sign colour stayed in the map through lap 3. |
| `check_green_seat_visibility.py` | Approved archival inputs and negative controls for background, occupancy and base checks passed. |
| `check_far_obstacle_acquisition.py` | All **52** cases passed. |
| `check_logger_directory_save.py` | Passed with **546** existing files, directory processing in batches of **16**, a complete snapshot, the expected stage order, no progress while driving, a retry into a new file after a short write and preservation of RAM after a directory error. The host mocks USB and Mbed. |

The IDE-managed M7 build passed with existing serial, USB macro and legacy warnings. M4 was unchanged and was not rebuilt. The M7 binary was **601,600 bytes**, SHA-256 `69afd090f0d0347d930125a1b52f9c2734b339a9f7997ef720822f9ffd9d8882`. We did not upload or make another commit during the review.

## Physical follow-up

For the stationary follow-up we used the revised M7 firmware, kept the drive disabled and opened the USB serial monitor at **115200 baud** before saving. The planned evidence was the last logger stage, heap counters, any Mbed output and the LED response, plus a copy and hash of the saved file.

The revised build reached every stage with **zero allocation failures**. We observed **42,432 bytes** of heap in use at mount and a **44,484-byte** peak during the write. Both exceed the earlier build's entire **38,352-byte** heap span. The application answered a read-only position query after saving, and the drive contained a complete **930-byte** `log_1.txt` with the expected initialisation and `SYSTEM-DISABLED` snapshot.

This is strong evidence of a heap constraint on the earlier build. The old crash stack is still missing, and the stationary save covered a short snapshot. [Original stationary save and SHA-256](evidence/log_intake/20261119_stationary_usb_save/README.md)
