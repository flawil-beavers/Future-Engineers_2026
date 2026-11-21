# 2026-11-20 USB save diagnosis

The user reported a full run ending in steady red during saving. The unavailable run and the subsequently saved 156978-byte log 35 are documented in the [original evidence index](../../parking_exit_diagnostics/README.md#2026-11-20-logs-3435).

The same FAT32 volume (`PHILIPP`, serial `70B5-3F9E`) had about 14.6 GiB free. Its log 35 matched the archived SHA-256. A fresh read-only Windows `chkdsk` completed file and folder verification and reported no filesystem problems. The previous PC check also passed 32 write/flush/readback cycles. [Prior volume test](../20261119_save_check/README.md)

With physical drive release off and the stick attached to the robot, the test sent only the normal pause/save command and a subsequent position query over the serial console. The installed code reported the earlier `finalize_verify` marker. It reached that stage with 44484 bytes used/peak out of 97328 bytes reserved and zero failed allocations, and the position query answered afterward. No reset, firmware upload or movement command was issued during the test.

| Evidence | Bytes | SHA-256 |
| --- | ---: | --- |
| [log 36, USB original](20261120_log_36_7dd527741778.txt) | 930 | 7dd527741778dcf321e7238faad9fc04f05d23858fbcebd35059ba58e370a91b |
| [Serial capture](stationary_com7_before_patch.serial.bin) | 832 | 6fef40788a6b4210ed7c920f5602bb6c218ed1916c6c7ab6e8eb1c7f8fdf1b3c |

Log 36 appeared under a new filename with bytes identical to the earlier 930-byte stationary save. It contains startup and disable messages without a run header or footer. It demonstrates the short save in this session, not reproduction of the large-log hang.

Source review of the installed Arduino_USBHostMbed5 0.3.1 found that a successful REQUEST SENSE masked an unsuccessful mass-storage command, TEST UNIT READY supplied a null pointer to code that inspects the opcode on error, a failed capacity read could leave the disk marked initialized, and a missing transfer descriptor returned without releasing its mutex. Generated M7 library sources correct these paths. The logger now distinguishes write close, verification open/read/close and unmount. It checks each written payload byte before discarding the RAM snapshot. The earlier timeout remains in effect. The specific fault from the missing complete run remains unobserved.
