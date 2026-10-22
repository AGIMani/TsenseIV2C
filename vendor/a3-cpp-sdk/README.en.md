# A3 C++ SDK 1.1.15

Host library for the H723 A3 dexterous hand over **USB CDC** or **ETH TCP 5001** (AA55 + CRC16-Modbus). The SDK does not speak CAN. Hosts: **Windows x64** and **Linux** (x64 or arm64, glibc). The hand has no static IPv4 address; it takes one from DHCP. `connectEthAuto()`, `a3_example --eth auto`, and the HTML Scan button search the PC subnet for TCP 5001 and use the first hand that answers FUN `0x01`. With several devices, pick from `listEthHosts()` or type the serial-log line `eth dhcp ip=...`. Scanning finds nothing when there is no DHCP server.

Ship customers the packed tree (headers, lib, examples). Do **not** send the firmware repository. Windows: `pack.bat` → `dist/a3-cpp-sdk/`. Linux: `pack.sh` → `dist/a3-cpp-sdk-linux/`.

This release has no motor IAP, H7 OTA, Python, or ROS bindings.

## Compatibility

| SDK | Host proto | H7 APP |
|-----|------------|--------|
| 1.1.x | proto 1.0 | 1.0.0.x with FUN `0x01` |

`connectUsb` / `connectEth` query firmware and fail on mismatch. See `COMPAT.md`.

## Slots

| Slots | Bus | Nodes | Joint | Clamp |
|-------|-----|-------|-------|-------|
| 0, 3 | CAN1 | 1, 4 | little / ring DIP | 0–12.437477 mm |
| 1 | CAN1 | 2 | little PIP | 0–11.761490 mm |
| 4 | CAN1 | 5 | ring PIP | 0–16.939022 mm |
| 2, 5 | CAN1 | 3, 6 | MCP | 0–19.627779 mm |
| 6–7 | CAN1 | 7–8 | little / ring rotary | ±20 deg |
| 8, 11 | CAN2 | 1, 4 | middle / index DIP | 0–12.437477 mm |
| 9 | CAN2 | 2 | middle PIP | 0–16.939022 mm |
| 12 | CAN2 | 5 | index PIP | 0–16.939022 mm |
| 10, 13 | CAN2 | 3, 6 | MCP | 0–19.627779 mm |
| 14–15 | CAN2 | 7–8 | middle / index rotary | ±20 deg |
| 16 | CAN3 | 1 | thumb DIP | 0–12.437477 mm |
| 17 | CAN3 | 2 | thumb MCP | 0–16.939022 mm |
| MIT 0/1 | CAN3 | — | thumb rotary; output-axis deg/rpm, SDK applies ×50 | ±180 deg |

Pass the mask for the motors you actually have to `waitOnline`. Do not use `kSelfOnlineMaskAll` unless all 18 slots are populated.

## Build

Visual Studio 2022 or 2026:

```bat
build.bat
pack.bat
```

HTML teach pendant binds **127.0.0.1** by default (no auth). `--lan` listens on all interfaces.

## Linux

CMake ≥ 3.15 and a C++17 compiler. Output is `build-linux/` so it does not collide with a Windows `build/` in the same tree.

```bash
bash build.sh
bash pack.sh
./build-linux/a3_example --list
./build-linux/a3_web_bridge
```

`--list` tags VID `0483` PID `5740` as `[H7]`. Do not open the ST-Link VCP. USB and ETH must not both be the control link.

```bash
sudo cp linux/99-a3-h7-cdc.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo usermod -aG dialout "$USER"   # log in again; Arch often uses group uucp
```

Pass `/dev/ttyACM*` to `--usb`. If sysfs has no VID/PID, the port is not tagged `[H7]` and `connectUsbAuto` will not pick it; pass the path from `--list` yourself.

## Embed

```cpp
#include "a3/A3.h"
#include <iostream>

int main() {
    a3::A3HandApi hand;
    std::string err;
    if (!hand.connectUsbAuto(&err)) {
        std::cerr << err << '\n';
        return 1;
    }
    float pos[a3::kSelfMotorNum]{};
    constexpr uint32_t kMyHand = 0xFFFFu; // slots 0-15, adjust to actual hardware
    if (!hand.setPositions(pos) || !hand.startStream(500, &err) ||
        !hand.waitOnline(kMyHand, 2000, &err) ||
        !hand.enable(a3::kBusAll, a3::kNodeAll, true, a3::kPwrDrvAll, &err) ||
        !hand.homeSlot(0, a3::IqSeekCounts{80.f}, &err) ||
        !hand.waitHomeDone(0, 15000, &err)) {
        std::cerr << hand.lastError() << '\n';
        hand.estop();
        hand.disconnect();
        return 1;
    }
    const auto snapshot = hand.telemetrySnapshot();
    const bool stopped = hand.estop(&err); // requires a live link
    if (!stopped) std::cerr << err << '\n';
    hand.disconnect();
    return stopped ? 0 : 1;
}
```

`enable` / `home` / PID succeeding means the frame was written, not that the motor acked. `estop()` cannot stop motors if the link is already dead. `setPositions` clamps to the slot table. `setJointAnglesDeg` converts mechanism angles to actuator commands. Linear joints use the linkage formula: one curve for the four-finger MCPs, one for index/middle/ring PIP and thumb MCP, one for DIP and thumb DIP, and one for pinky PIP. Forward stroke and feedback both use uniform lookup tables. Interpolation error is at most 1e-9 m for angle to stroke, and 1e-6° for stroke to angle. Angles outside 0–90° use the nearer endpoint. 0° is 0 mm and 90° is that linkage's stroke cap. Self rotary slots are clamped to ±20°. `actuatorToAnglesDeg` inverts actuator feedback. Float32 values within 0.00001 mm of a linear cap, or 1e-6 rad of a rotary limit, are clamped before the inverse.

Application code should use `A3HandApi`. Wire packing lives in `a3/detail/Protocol.h`.

Telemetry validity expires after 1000 ms. Use `telemetrySnapshot()` for a consistent copy of values, sequence, tick and freshness. Target setters return bool and reject invalid batches without changing targets. Commands and lifecycle operations are serialized; waits do not block stop/disconnect. Telemetry callbacks run on RX; disconnect callbacks may run on RX, TX or the sending caller. Callbacks may only read state, set targets or register callbacks; commands/waits are rejected and exceptions become CallbackFailed. Do not destroy the hand inside a callback.
