# A3.2 Linux adapter — SDK 1.1.15, C ABI v2

The adapter statically links the unmodified SDK sources in
`vendor/a3-cpp-sdk`. The user supplied `a3.2_cpp_sdk-main.zip` on 2026-10-12;
`vendor/a3-cpp-sdk/sdk-source.json` records its SHA-256 and imported file hashes.
The archive identifies version 1.1.15 and its GitHub ZIP comment records commit
`fd676e5a3296e5e3ada2734d290acc443206e000`.

The SDK provides packet encoding, CRC, telemetry decoding, actuator limits and
an offline joint-angle conversion oracle. The project's POSIX USB/TCP transport,
locking, send timing and local-stop behavior remain in this adapter. It does not
run SDK examples, a bridge service or a Qt application.

## Build

From the project root on native Linux:

```bash
bash src/a32_driver/build_native_linux.sh
```

CMake, a C++17 compiler, pthreads and Python 3 are required. The build runs the
SDK protocol/command tests and project offline TCP/PTY integration tests before
atomically replacing `src/a32_driver/bin/liba32_native.so`. It does not access
physical hardware. `A32_NATIVE_BUILD_DIR` selects a different build directory;
`A32_NATIVE_LIBRARY` selects the Python runtime library.

Stop control and restart each track/play service after updating: a running
process retains its old loaded library and Python modules. The two Python
environments remain separate. Both deployment bundles contain the source
needed to rebuild this library after relocation.

## Project lifecycle contract

- Connect starts RX and sends the SDK firmware query as optional metadata. It
  does not wait for firmware, reject its version, enable, home or send targets.
  This intentionally differs from the vendor `A3HandApi::connect*` handshake.
- USB auto-selection requires exactly one H7 VID/PID `0483:5740`. Explicit paths
  also permit PTY tests. `TIOCEXCL`/`TIOCNXCL` enforce serial ownership; no DTR/RTS
  pulse is issued. TCP uses the supplied host and port, without subnet scans.
- Writes handle partial writes and EINTR/EAGAIN with a 250 ms deadline. TCP uses
  `MSG_NOSIGNAL`; receive handles fragmented frames and checks device ID/CRC.
  TCP connect has a 1500 ms deadline after DNS resolution.
- A finite complete target batch is required after each connection or local
  stop. Invalid batches leave all targets unchanged. Self targets use SDK caps:
  slots 0/1/3/8/11/16: 0..12.44 mm; 4/9/12/17: 0..16.95 mm;
  2/5/10/13: 0..19.63 mm; rotary 6/7/14/15: -20..20 degrees.
  MIT targets use output-axis -180..180 degrees, with the SDK's 50:1 ratio.
- `a32_stop_sending`, compatibility `a32_estop`, and `a32_enable_all(false)`
  only discard host targets. They never call vendor `estop` or send disable.
  Disconnect/destruction is also silent. These actions do not confirm a
  physical stop or cancel a command already accepted by firmware.
- RX is joined before descriptors are closed. Link loss clears online state.
  Firmware `hand_side` is exposed as metadata (0 unset, 1 right, 2 left).
  MIT `valid` and `online` remain separate; position decoding retains the
  adapter's accurate int64-centered arithmetic near zero.
- The application owns its trajectory loop; `streaming` remains false. The
  additive timed `a32_send_targets` API keeps at least 1 ms between send
  boundaries and reports monotonic timestamps. This is not a hard realtime
  guarantee. `rx_hz`/`tx_hz` use adjacent packet/send timestamps.

State reports `sdk_version=1.1.15` and
`native_implementation=a3-sdk-protocol-linux-abi2/1.0`. Existing ABI-v2 functions
remain available. `a32_angles_to_actuators(const float*, float*)` is a stateless
addition that invokes upstream `anglesDegToActuator` for all 20 joints without
connecting; it is used to verify the pure Python command mapper.

SELF `iq` and `iq_ref` now arrive in amperes from the SDK decoder: linear
counts × 0.001374, rotary slots 6/7/14/15 counts / 819 (already zero-subtracted).
The adapter does not convert a second time. State identifies these units with
`telemetry_format=a32-native-sdk-state/v2`, `telemetry_units` and each row's
`current_unit=A`. Historical v1 recordings retain their native-unit meaning.
Homing current remains `IqSeekCounts`, not amperes. The SDK's OpenSpin classifier
now requires both byte0 in 1..4 and byte3 in 0..4; an ordinary electrical-angle
float with low byte 1..4 is no longer classified as OpenSpin solely for that byte.

## Offline checks

```bash
python3 src/a32_driver/native_linux/tests/test_offline.py src/a32_driver/bin/liba32_native.so
.venvs/track/bin/python -m pytest tests/test_a32_sdk_stroke.py tests/test_a32_driver.py
```

Coverage includes exact wire bytes and per-slot caps, all-or-nothing target
validation, MIT precision/validity, firmware hand side, fragmented/corrupt
telemetry, optional late/absent/incompatible firmware replies, reconnect,
serial ownership and stop/disconnect paths that emit no command.

SDK calibrated closed-loop conversion changes actual linear travel for saved angles.
Three actuator sizes share their respective calibrated curves; thumb MCP uses
the medium 16.95 mm curve. The Python mapper and feedback inverse use these curves; CAD
assets, calibrated directions, thumb origins and MIT ratios remain unchanged.
Real USB, physical motion, homing and sustained throughput require separate
hardware verification.
