# Changelog

## 1.1.15

- Linear angle to stroke uses the linkage formula. Four curves: four-finger MCP (0–19.627779 mm), index/middle/ring PIP and thumb MCP (0–16.939022 mm), DIP and thumb DIP (0–12.437477 mm), pinky PIP (0–11.761490 mm).
- 0° is 0 mm and 90° is that linkage's stroke. Angles outside 0–90° use the nearer endpoint. Feedback inverts the same formula.
- Forward linear stroke uses a uniform lookup table. The grid is sized from a bound on |x''|, so linear interpolation error is at most 1e-9 m.
- Stroke feedback also uses a uniform lookup table. Interpolation error is at most 1e-6°.

## 1.1.14

- Rotary slots #06, #07, #14 and #15 report `iq_meas` and `iq_ref` in amperes: (raw code − zero) / 819. The telemetry code is already zero-subtracted. Homing `IqSeekCounts` stays in counts.

## 1.1.13

- Linear-slot `iq_meas` and `iq_ref` are amperes: counts × 0.001374. Rotary self slots stay in counts. Homing `IqSeekCounts` is still sent as counts.

## 1.1.12

- Linear angle and stroke use the calibrated closed-loop model. The same motor size shares one curve: large (four-finger MCP, 0–19.63 mm), medium (index/middle/ring PIP and thumb MCP, 0–16.95 mm), small (DIP, thumb PIP, and pinky PIP, 0–12.44 mm).
- Thumb MCP slot #17 stroke clamp is 0–16.95 mm.
- Forward commands stay limited to 0.005–89.995°. Stroke feedback still inverts on the same curve, so an exact stroke cap reads back as 90°.

## 1.1.11

- Electrical-angle feedback whose float32 low byte is 1–4 is classified as theta, not open-loop (`OpenSpin` / 255). Open-loop still requires byte0 in 1–4 and byte3 still holding a homing code.

## 1.1.10

- `actuatorToAnglesDeg` converts actuator feedback back to joint angles. Float32 rounding within 0.00001 mm of a linear stroke cap, or 1e-6 rad of a rotary limit, is clamped and inverted. Values outside that tolerance are rejected.

## 1.1.9

- `setJointAnglesDeg` converts mechanism joint angles to actuator commands. Linear output follows the linkage, scaled so 0° is 0 mm and 90° is that joint's stroke cap.
- Linear angle commands are limited to 0.005–89.995°. Smaller values use 0.005°, larger values use 89.995°.
- Pinky PIP (slot #01) stroke clamp is 0–12.44 mm. Other PIP joints stay 0–16.95 mm. DIP is 0–12.44 mm, MCP is 0–19.63 mm.
- Self rotary slots #06, #07, #14 and #15 are clamped to ±20°.

## 1.1.8

- ETH subnet scan waits on one poll for the whole subnet, so the web bridge answers before the browser drops the request. The scan button reports the request error instead of asking for a rebuild.

## 1.1.7

- A SEEK already running does not satisfy a new home; the slot must leave SEEK or RELEASE first. `homeReset` keeps the previous wait if the reset frame is not sent, and ignores unchanged telemetry after a successful reset.
- `homeBusKind` sends reset to motors already commanded when a later slot fails.
- `startStream` keeps the stream flag set after joining an exiting sender.
- Successful `setPositions` / `setMit` clear `lastErr()`.
- Commands and disconnect from any thread are rejected while a user callback is running.

## 1.1.6

- Register homing before sending, ignore telemetry cached before that operation, and roll back failed sends.
- Clear link state on unexpected loss; telemetry validity expires after one second. Add atomic `telemetrySnapshot()`.
- Serialize lifecycle/command operations; waits do not block stop/disconnect and cannot cross connection sessions.
- Catch callback exceptions as `CallbackFailed`; reject commands/waits inside callbacks. Interrupt low-frequency stream sleeps on stop.
- Reject non-finite control values, invalid addresses, PID parameters and oversized touch commands. Target setters now return `bool`; invalid batches leave targets unchanged.
- Recover subsequent frames after bad CRC; validate frame header and CRC in `parseFrame`. Suppress Linux TCP SIGPIPE.
- Serialize Web commands and enforce strict JSON syntax, required fields, types and ranges. Return stream-start failures.
- Use generated version headers first, propagate C++17 to consumers and run tests before packaging. Homing example confirms each slot before motion.

## 1.1.5

- Linear stroke clamp: DIP 0–12.44 mm, PIP 0–16.95 mm, MCP 0–19.63 mm. Rotary slots stay ±180 deg.

## 1.1.4

- Linear stroke clamp is per joint: DIP 0–8 mm, PIP and MCP 0–14 mm. Rotary slots stay ±180 deg.

## 1.1.3

- Reconnect clears online mask and telemetry, so `waitOnline` does not succeed on the previous link.
- `waitHomeDone` after `home` / `homeSlot` waits for SEEK or RELEASE from that command before DONE or theta.
- ETH send/recv/close share one lock. A blocked TCP write gives up after 4 ms instead of spinning.
- Touch matrices read only the bytes present in the frame.
- The HTML bridge no longer sends `Access-Control-Allow-Origin: *`, and rejects bodies over 256 KB.

## 1.1.2

- H7 no longer has a static IP. `listEthHosts()` / `connectEthAuto()` / `--eth auto` scan the local subnet for TCP 5001.

## 1.1.1

- Linux (glibc) host: USB CDC on `/dev/ttyACM*` / `/dev/ttyUSB*`, ETH TCP 5001, and the HTML bridge.
- `listUsbPorts` marks H7 only for VID `0483` PID `5740` (ST-Link VCP is not `[H7]`).
- `build.sh` / `pack.sh` produce `dist/a3-cpp-sdk-linux/`. Udev rule: `linux/99-a3-h7-cdc.rules`.

## 1.1.0

Product API for customer integration.

- PIMPL `A3HandApi`; public headers are `A3.h` / `A3HandApi.h` / `Types.h`. Wire packing moved to `a3/detail/Protocol.h`.
- `Err` codes; `lastErr()` / `errMessage()`. Telemetry structs no longer carry display strings.
- Disconnect clears streaming; `startStream` fails if not connected; send failure is treated as link loss.
- `setPositions` / `setMit` clamp by slot; `connectUsbAuto()`; `IqSeekCounts` for homing (Iq counts, not amperes). MIT ×50 is internal to `setMit`.
- FUN `0x01` firmware handshake on `connectUsb` / `connectEth`.
- HTML bridge binds 127.0.0.1 (use `--lan` for all interfaces).
- `pack.bat` installs `dist/a3-cpp-sdk/` for customer delivery.

### Breaking

- `homeSlot` / `home` take `IqSeekCounts`, not `float`.
- `startStream` returns `bool`.
- `DisconnectCallback` is `(Err, const std::string &)`.
- `ComPort::st_cdc` renamed to `h7_cdc`.
- `slotKind()` returns `"linear"` / `"rotary"`.
- `connectUsb` / `connectEth` require H7 APP with FUN `0x01`.

## 1.0.0

Initial USB/ETH control library.
