# Independent A3.2 Linux driver (project C ABI v2)

This is an independent POSIX driver, not a Linux build of the vendor SDK. It
implements the exact subset used by `src/a32_driver/native.py` and exports the
same version-2 C ABI, including `a32_control_state` and `a32_send_targets`.
It uses Linux `termios` for USB CDC and POSIX TCP sockets, with no Windows
process, Wine, bridge service or vendor binary at runtime.

Build inside the isolated Linux project:

```bash
bash src/a32_driver/build_native_linux.sh
```

CMake, a C++17 compiler, pthreads and Python 3 are required. The build writes
`src/a32_driver/bin/liba32_native.so` and runs the offline TCP/PTY integration
suite. `A32_NATIVE_BUILD_DIR` can select another build directory.
`A32_NATIVE_LIBRARY` overrides the Python loader path; the older
`A32_NATIVE_DLL` override remains supported. Both overrides also apply on Windows.

## Evidence and verification status

The implementation was derived from the supplied SDK headers and **offline
calls to the unmodified Windows SDK static library**, using a mock ITransport.
The audit used no physical device. Evidence is under
`E:\WSL\migration\protocol-audit`: `protocol.md`, `fixtures.json`, `oracle.json`,
`compare_protocol.cpp` and `comparison.json`. The comparison runner links both
the original SDK and this `protocol.cpp`; its recorded result is **1,391,558
checks over 8,284 telemetry samples, zero failures**. It covers all self slots,
online masks, homing encoding, MIT centering/scales, control packets, CRC,
firmware/board payloads, split frames and resynchronization.

The delivered Linux shared library was built natively inside the isolated
Ubuntu 24.04 WSL2 distribution using GCC 13.3.0 and glibc 2.39. All seven
TCP/PTY integration cases passed on 2026-09-30, and the default Python loader,
C ABI version 2 and all 16 required exports were verified. Build provenance,
SHA256, dependency details and full test output are recorded under
`E:\WSL\migration\native-runtime` and `elf-verification.json`.

The earlier Windows/Zig cross build was superseded: its bundled glibc headers
used newer baud-rate constants incompatible with Ubuntu glibc 2.39, causing
PTY configuration to fail. That binary is archived only for diagnosis; build
against the target Linux system headers with the command above.

These tests use loopback TCP and pseudo terminals. Physical USB transport,
real device handshake, homing, motion and sustained 500 Hz behavior still
require separate hardware acceptance; no physical device was connected.

## Connection and command behavior

- Connect starts an RX thread, sends only the 16-byte firmware query and waits
  up to 1500 ms. Protocol major 1 and app major 1 are required. Connect does not
  enable motors or send position targets. Firmware failure closes the link.
- USB `auto` selects exactly one `/dev/ttyACM*` device whose sysfs USB parent has
  VID/PID `0483:5740`. Multiple matches require an explicit device path.
  Explicit paths also permit PTY integration tests. `TIOCEXCL` claims exclusive
  serial ownership and cleanup releases it using `TIOCNXCL`. No DTR/RTS pulse is issued.
- TCP uses nonblocking connect with a 1500 ms total connection deadline after
  `getaddrinfo`. OS hostname lookup is not covered by that deadline; use a
  numeric device IP where deterministic connection timing is needed.
- All writes loop until complete, handling EINTR/EAGAIN and partial writes.
  A write deadline is 250 ms. TCP uses `MSG_NOSIGNAL`. Receive polls at 100 ms,
  accepts fragments/coalesced frames, verifies CRC and discards malformed data.
- Explicit targets must be supplied after each connection before a position
  send is accepted. Targets are validated atomically: NaN/infinity rejects the
  whole request. Finite values clamp to SDK slot limits: linear 0..40 mm,
  rotary/MIT -180..180 degrees. MIT degrees are output-axis units; packing
  multiplies by 50, telemetry divides by 50. No extra factor is applied in Python.
- Enable, homing/reset and estop are explicit calls. Estop sends the vendor's
  all-off packet; it does not disconnect. Disconnect/destruction sends no
  command. A lost link cannot guarantee hardware stopping.
- RX does not close descriptors. Disconnect signals RX, joins its thread and
  only then closes the descriptor, avoiding file-descriptor reuse races. Lost
  links invalidate online flags and preserve an error/diagnostic record.
- The position packet sequence is uint8, starting at zero, and persists across
  reconnects on the same native object. `send_targets` enforces >=1 ms spacing
  between actual send boundaries and reports monotonic nanosecond timestamps.
  Linux scheduling and device throughput still determine achievable latency.

## Compatibility scope

State JSON keeps the project's existing keys and explicitly identifies this
implementation as `independent-linux-abi2/1.0`. `streaming` is always false:
the project drives individual sends from its existing Python control loop;
this driver never creates an automatic command stream. `rx_hz` and `tx_hz`
are measured from the most recent adjacent valid packets/sends. Touch, PID,
open-spin and other vendor APIs not exported by the project's C ABI are outside
this implementation. This driver targets the SDK 1.1.0 wire protocol documented
in the audit, not the unrelated older H723 V2 repository protocol.

Run just the offline integration suite after building:

```bash
python3 src/a32_driver/native_linux/tests/test_offline.py src/a32_driver/bin/liba32_native.so
```

The suite opens only loopback sockets and pseudo terminals. It checks exact
vendor query/power/home/reset packets, fragmented telemetry and MIT units,
invalid CRC/device/length recovery, compatible/incompatible/timeout handshakes,
NaN rejection, JSON capacity, send timing, disconnect and reconnect behavior.
