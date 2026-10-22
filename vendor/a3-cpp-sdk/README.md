# A3 C++ SDK 1.1.15

经 **USB CDC** 或 **ETH TCP 5001** 控制 H723 A3 灵巧手。线格式 AA55 + CRC16-Modbus，不直接发 CAN。主机支持 **Windows x64** 和 **Linux**（x64 / arm64，glibc）。

发给客户的是打包目录（头文件 + 库 + 示例），**不要**把整个 H7 固件仓库交出去。Windows 用 `pack.bat` 得到 `dist/a3-cpp-sdk/`；Linux 用 `pack.sh` 得到 `dist/a3-cpp-sdk-linux/`。

本版本不含电机 IAP / H7 OTA / Python / ROS 绑定。升级仍用既有工具。

## 兼容

| SDK | 主机协议 | H7 APP |
|-----|----------|--------|
| 1.1.x | proto 1.0 | 1.0.0.x（须支持 FUN `0x01`） |

`connectUsb` / `connectEth` 会查询固件版本，对不上会失败。旧 APP 请先升级。详见 `COMPAT.md`。

## 槽位

| 槽 | 总线 | 节点 | 关节 | 行程钳位 |
|---|---|---|---|---|
| 0、3 | CAN1 | 1、4 | 小拇指 / 无名指 DIP | 0–12.437477 mm |
| 1 | CAN1 | 2 | 小拇指 PIP | 0–11.761490 mm |
| 4 | CAN1 | 5 | 无名指 PIP | 0–16.939022 mm |
| 2、5 | CAN1 | 3、6 | MCP | 0–19.627779 mm |
| 6–7 | CAN1 | 7–8 | 小拇指 / 无名指旋转 | ±20 deg |
| 8、11 | CAN2 | 1、4 | 中指 / 食指 DIP | 0–12.437477 mm |
| 9 | CAN2 | 2 | 中指 PIP | 0–16.939022 mm |
| 12 | CAN2 | 5 | 食指 PIP | 0–16.939022 mm |
| 10、13 | CAN2 | 3、6 | MCP | 0–19.627779 mm |
| 14–15 | CAN2 | 7–8 | 中指 / 食指旋转 | ±20 deg |
| 16 | CAN3 | 1 | 大拇指 DIP | 0–12.437477 mm |
| 17 | CAN3 | 2 | 大拇指 MCP | 0–16.939022 mm |
| MIT 0/1 | CAN3 | — | 大拇指旋转，输出轴 deg / rpm，SDK 内部 ×50 | ±180 deg |

`waitOnline` 传入**本机实际槽位 mask**，不要对缺电机的手使用 `kSelfOnlineMaskAll`。

## 构建（开发本库）

Visual Studio 2022 或 2026：

```bat
cd a3-cpp-sdk
build.bat
```

或：

```bat
cmake --preset vs2022-x64
cmake --build --preset vs2022-x64-release
```

客户交付包：

```bat
pack.bat
```

得到 `dist/a3-cpp-sdk/`。

```bat
build\Release\a3_example.exe --list
build\Release\a3_example.exe --usb auto --hold
build\Release\a3_example.exe --eth auto --hold
```

`--list` 把 H7 CDC（VID `0483` PID `5740`）标成 `[H7]`。不要连 ST-Link 的 Virtual COM。USB 与 ETH 不要两路同时当控制口（H7 上行优先 ETH）。

H7 没有静态 IP，地址由 DHCP 分配。`connectEthAuto()` / `--eth auto` / HTML「扫描」都在本机同一网段查找 TCP 5001，连上第一台能完成 FUN `0x01` 的设备。多台时用 `listEthHosts()` 自己选，或手填串口日志里的 `eth dhcp ip=...`。没有 DHCP 服务器时扫不到。

## 构建（Linux）

需要 CMake ≥ 3.15、C++17 编译器（gcc 或 clang）。产物在 `build-linux/`，和 Windows 的 `build/` 分开，同一份目录可以两边各编一次。

```bash
cd a3-cpp-sdk
bash build.sh
bash pack.sh
```

或：

```bash
cmake --preset linux-release
cmake --build --preset linux-release
```

```bash
./build-linux/a3_example --list
./build-linux/a3_example --usb auto --hold
./build-linux/a3_web_bridge
```

打开 CDC 需要串口权限。Debian/Ubuntu 把用户加入 `dialout` 后重新登录。规则只匹配 H7（`0483:5740`），不会改 ST-Link：

```bash
sudo cp linux/99-a3-h7-cdc.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo usermod -aG dialout "$USER"
```

Arch 等发行版的串口组常是 `uucp`，改规则里的 `GROUP` 后再安装。端口名是 `/dev/ttyACM*`（USB 转串口芯片则是 `/dev/ttyUSB*`）。读不到 sysfs 的 VID/PID 时不会标成 `[H7]`，`connectUsbAuto` 会失败，这时把 `--list` 里的路径直接传给 `--usb`。

## HTML 示教器

```bat
build\Release\a3_web_bridge.exe
```

默认只绑 **127.0.0.1:8080**，无鉴权。`--lan` 才会听所有网卡（可远程使能电机，仅实验室使用）。

## 嵌入

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

`a3::sdkVersion()` 是本库版本。H7 版本见 `hand.firmware()`。

遥测回调在 RX 线程；断线回调可能在 RX、TX 或发送命令的调用线程。回调只允许读取状态、修改目标和设置回调；命令、连接启停和等待操作会被拒绝。回调异常会被捕获为 `Err::CallbackFailed`，请保持回调简短，不要在回调中销毁对象。

CMake：

```cmake
add_subdirectory(a3-cpp-sdk)
target_link_libraries(your_app PRIVATE A3::a3_sdk)
```

安装后：

```cmake
find_package(a3-cpp-sdk 1.1 CONFIG REQUIRED)
target_link_libraries(your_app PRIVATE A3::a3_sdk)
```

组包层（仿真/录包）用 `a3/detail/Protocol.h`。应用代码只用 `A3HandApi`。MIT 输出轴请走 `setMit()`，不要自己 ×50 再调 `packHandBatch`。

## 安全

- `enable` / `home` / PID 返回 true 只表示 USB/TCP 已写出，电机无 ACK。
- `estop()` 在掉线时不能保证电机停下。
- 进程崩溃后电机保持最后位置（无主机看门狗）。
- `setPositions` 按上表钳位，仍须在上层做轨迹规划。
- `setJointAnglesDeg` 把机构角换成执行器指令。直线关节用连杆公式：四指 MCP、食/中/无名指 PIP 与拇指 MCP、四指 DIP 与拇指 DIP、小拇指 PIP 各一条曲线。下发和反馈都用均匀查表。角度→行程的插值误差不超过 1e-9 m，行程→角度不超过 1e-6°。角度限制在 0°–90°，超出端点按端点计算。0° 对应 0 mm，90° 对应上表行程上限。侧摆按 ±20° 钳位。
- `actuatorToAnglesDeg` 把执行器反馈反算回关节角。直线限位附近 0.00001 mm、旋转限位附近 1e-6 rad 以内先钳回再反算，再往外视为越界。

## 错误

`lastErr()` 为 `a3::Err` 枚举；`lastError()` / `errMessage()` 为英文说明。

遥测超过 1000 ms 未更新时，在线/有效标志会失效。用 `telemetrySnapshot()` 一次读取数据、序号、时间和新鲜度。目标设置接口现在返回 bool，非法输入不会改变目标。并发命令会串行执行；等待接口不会阻塞停止或断开。
