# USB save check, 2026-11-19

User reports two runs and red blinking starting only during saving. All656 original log files compared bytewise bySHA256 against the earlier evidence/working archive: none new or changed, no current20261119_measured_height marker. Highest filenames remain655/656 (existing November5 CCW originals); newest by broken2098 source clock remain355/356 (already secured November19). Selected files were immediately copied and source-before/source-after/copy hashes checked before analysis; user told stick can be unplugged. Existing portable originals are linked in manifest.json; do not label them as the new two runs. No new-run log available, no inference about robot motion outcomes.

Firmware logger source uses steadyRED during saving, GREEN on success, RED+GREEN for2s on handled failure. It does not implement repeated RED blinking. Exact blink pattern and read-only robot serial error are pending. No repair, formatting, USB write, firmware upload, reset or motion issued.

User subsequently confirmed four-short/four-longRED and USBconnected/driveOFF/no reset. This pattern identifies an MbedOS crash per [Arduino primary support](https://support.arduino.cc/hc/en-us/articles/7991505987612-If-an-LED-on-GIGA-R1-WiFi-is-flashing-red). Passive port enumeration found no Arduino USB CDC port, so no port was opened, no command sent and no serial error obtained. Exact fault location, heap state, installed firmware and whether the stick triggers it remain unknown. No firmware fix or hardware-defect conclusion supported yet.

## Follow-up on the emptied original stick

User confirms original stick was deliberately emptied before this check. FAT32, roughly16GB, labelPHILIPP; no log files now. Read-only Windows CHKDSK reported no filesystem problems and about15.7GB free. Then32 cycles of256KiB random write, flush/fsync, close/reopen and SHA256 comparison passed:8,388,608B verified, zero errors. Only a uniquely created test directory/file was written; both removed after checked path containment. Portable result: stick_write_read_check.json. No formatting/repair, existing-file deletion, robot reset/upload/command or motion performed.

Conclusion: no evidence of ordinary PC-side media/filesystem/write failure; confirmed user-reported Mbed crash occurs in the robot save path. This points to robot USB-host/software/power/compatibility, not a proven defective flash drive. Exact fault needs runtime crash/heap context. Source review confirms dynamic USB-host instantiation/thread allocation at connection; current ELF leaves a limited heap window, but this does NOT establish out-of-memory or installed-firmware identity. No speculative code fix applied.

## Stationary new-firmware save observed

User confirmed robot connected/driveOFF and later ready after correcting a failed upload. First passive DTRfalse session yielded no bytes and was not used as evidence of failure. Active terminal at115200 with DTRtrue (not1200; no reset) then sent only `p` (pause/disable/save). Full phase/heap serial original and after-save `t` position reply are preserved with hashes in stationary_save_observation.json. No drive/camera/reset/upload.

Measured used/reserved: power35616/71112B, connect35644/71112B, mount42432/71112B, directory_open42944/71112B, scan43268 (peak43780), write/finalize44484B. Zero failed allocations reported. Previous entire heap window38352B is smaller than the new observed live requirement already at mount. This is strong evidence for the old save-time memory shortage, without an old crashstack/readback. New application answered POS after finalization. Await original file on stick before claiming durable save.
