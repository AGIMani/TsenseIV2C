# A3.2 原生 SDK 接入

## 数据链路

手势跟踪和 motion_play 共用 `src/a32_driver/A32Device`：

```text
完整 20 个 URDF 关节角（rad，带型号与手别）
  → 按机构/硬件交集限制关节目标
  → A32StrokeMapper：14 路直线 mm + 6 路旋转 deg
  → a32_hardware.json：18 个 self 槽位 + 2 个 MIT 通道
  → ctypes 加载 a32_native.dll（C ABI v2）
  → 静态链接的 a3_sdk.lib → a3::A3HandApi
  → USB CDC 或 ETH TCP 5001 → H7
```

运行时不启动 `a3_example.exe`、`a3_web_bridge.exe`、SDK Web 工作台、MATLAB 或其他 SDK 控制程序。无需 Bridge HTTP 端口。`motor_index` 是运动学模型编号，不能直接减一当成 SDK 槽号；本接入逐个关节名映射。self 17 使用 SDK bus 2、node 2，底层 CAN node 5 由 SDK/固件处理。

MIT 减速比已按用户 2026-09-29 的最新要求恢复为 M1 `rotaback=50`、M2 `rota_block=50`。与 SDK 1.1.0 / C ABI v2 的两路 50 换算一致，NativeHand 不再对 M2 下发或反馈做额外缩放。上层使用输出轴 deg/rpm，状态中 `mit_gear_ratios=[50,50]` 标明当前换算。两款 A3.2 模型、手势控制和 motion_play 共用此配置；无需修改厂商静态库或重编 DLL。

裸手 `TsenseI_v3_2` 和带 ID 的 `TsenseI_v3_2_withid` 共用电机映射。裸手四指 swing 轴向已与带 ID 对齐，模型范围 `[0,0.7]`、默认 `0`。从原始 CAD 转换时，裸手使用 `原始 SDK 角度+0.350`，带 ID 使用 `−原始 ID 角度+0.350`；当前模型角均限幅到 `[0.005,0.695]` 后从 rad 转为 SDK deg。反馈从 deg 直接换算回 rad。范围和模型变换见 [关节范围说明](a32_joint_ranges.md)。模型和手别在会话创建时确定。

## 使用的 SDK 接口

| 功能 | SDK 调用 | 说明 |
|---|---|---|
| USB | `connectUsb` / `connectUsbAuto` | 连接时 SDK 自动查询固件兼容性 |
| ETH | `connectEth` | 默认 TCP 5001；与 USB 二选一 |
| 全手目标 | `setPositions`、`setMitAll`、`sendOnce` | Python 锁串行执行，全部 20 路一次更新；MIT 速度/电流目标为 0 |
| 使能 | `enable(0xFF,0xFF,on)` | 18 self 与 2 MIT 全手操作；先写入起步姿态，反馈缺失时使用软件姿态 |
| 自研电机找零 | `homeSlot`、`IqSeekCounts` | 0..17；Iq 是码值；SDK 未提供 MIT 找零接口 |
| 复位找零状态 | `homeResetSlot` | 先停止，再复位指定 self 槽或全部 18 槽；不会标记已找零 |
| 停止/取消 | `estop` | SDK 停流并请求全手失能 |
| 断开 | `disconnect` | 上层先请求停止，再释放链路 |
| 状态 | `selfTelem`、`mitTelem`、`boardTelem`、`firmware`、`rxMask`、`telemSeq`、`tickMs`、`rxOk`、`rxHz`、`txHz`、`targetSelf`、`targetMit`、`linkName` | 仅导出普通 JSON 值 |
| 回调 | `setTelemCallback`、`setDisconnectCallback` | C++ 内更新时间和错误；不跨线程执行 Python，不在回调里断开/等待 |

`setTelemCallback` 在 SDK 解析 `0x02` 手遥测后触发。SDK `parseHandTelem` 同时解析 self、MIT、sequence 和 tick。就绪检查还逐路要求 self `online`、MIT `valid/online`，并检查 MIT fault；没有把板级温度或触觉帧当成手位置的新反馈。SDK 未公开逐个电机测量时间戳，单路新鲜程度以该手帧的在线标志为准。

未使用的接口包括 PID 写入、电流环调参、开环旋转、触觉配置、固件升级、SDK `startStream` 和 SDK 阻塞 `wait*`。轨迹节奏由两个平台自己的 Ruckig 控制循环提供，SDK 用 `sendOnce` 发送当前采样。未复用 A30/V2 USB 协议、槽位、电流监视布局或校准参数。

## 生命周期与保护

- 导入、构造、离线预览、读取未连接状态不加载 SDK 或连接硬件。原生 DLL 在首次连接时加载。
- 连接不使能、不回零、不发送姿态。右手仅支持预览；硬件映射未标定，连接明确失败。
- 驱动层连接后即可使能和下发，不要求收到遥测、20 路全部在线或先完成找零。手势平台自 2026-09-29 起按 A3.0 增加首包手部遥测和明确 MIT 故障检查，详见 [手势下发规则](a32_gesture_control_policy.md)。`enable_all(..., send_initial_pose=False)` 只发使能命令；motion_play 使用此选项，勾选后等待用户调整关节或启动动作。手势跟踪及显式找零保留默认的起步姿态写入流程。
- 遥测缺失、部分通道离线、MIT 反馈无效及遥测超过 500 ms 均不阻止普通姿态发送。500 ms 仅决定反馈是否适合用作起步参考。UI 的 `enabled` 是本次会话已请求使能的状态；SDK 没有全手使能 ACK，不能将其当成硬件确认。
- `joint_limits()` 返回机构与命令限位交集。手势跟踪在 Ruckig 规划前调用 `constrain_pose()`；motion_play 允许 swing 的模型端点 `0/0.7`，校验模型轨迹后在下发时限幅。`prepare_targets()` 的 `applied_pose` 保存实际应用的关节角，日志分别记录页面角度与限幅结果；需要严格硬件限位校验的调用者可使用 `clamp=False`。
- 平台使用 `starting_pose(fallback_pose)` 逐关节选取有效反馈；缺失部分依次使用上次成功写入姿态、调用方软件姿态、模型初始姿态。手势平台首次使用初始姿态，motion_play 首次使用页面姿态。`seed_missing` 列出使用软件姿态的关节，软件姿态不是实测位置。严格读取实测位置的 `feedback_pose()` 保留供诊断使用，控制链路不调用它。
- 直线反馈由 `a32_loop_kinematics` 从 mm 反解为 rad；旋转反馈从 deg 换算并恢复 URDF 方向。`ready` 保留完整反馈及找零就绪诊断，`command_ready` 表示连接且未处于找零/本次找零错误状态；下发不使用 `ready` 作为门槛。
- 找零是可选的异步操作，内部先设置起步姿态并使能，然后只对 self 调用 `homeSlot`。主动找零过程中禁止轨迹发送；完成后需再次使能来对齐新位置。
- 找零完成要求新序号遥测，不能用命令之前的 DONE 帧。独立监视线程每 50 ms 检查一次，错误、反馈失效或默认 30 秒超时会请求全手停止；浏览器不轮询也会执行。关闭会话释放监视线程与 SDK 对象。
- 停止后固件可能仍报告 SEEK/RELEASE，可显式调用 `reset_home(slot=None)`（界面的“复位找零状态”）：先请求全手停止，再调用 `homeResetSlot`，保留未使能。`home_reset_pending` 表示尚无新遥测确认复位，但不阻止用户重新使能和下发，也不声称找零已完成。过期的 SEEK/RELEASE 不作为普通下发门槛；本次主动找零仍受独立监视线程管理。
- 写入失败取消软件使能并尽力急停；正常关闭也先停止再断开。SDK 明确说明链路已断、进程崩溃时不能保证电机停下，SDK/固件没有主机看门狗。

## 构建和分发

从项目根目录在 PowerShell 运行：

```powershell
.\src\a32_driver\build_native.ps1 -SdkRoot D:\Qtworks\a3-cpp-sdk
```

脚本通过 `vswhere` 选择 Visual Studio 2022/2026 x64 工具链及其 CMake。也可用 `-Generator 'Visual Studio 17 2022'` 指定版本。这里只运行编译器/构建工具，不运行 SDK 示例或控制程序。

构建输出：`src/a32_driver/bin/a32_native.dll`。生产包需要本目录的 Python 文件、`a32_hardware.json`、DLL、`SDK_LICENSE.txt`，以及项目已有 `a32_kinematics`、NumPy、SciPy。不需要交付 SDK include/lib 或原始 `a3-cpp-sdk` 文件夹；重新编译时才需要。可用 `A32_NATIVE_DLL` 指定另一处已构建 DLL。

实测 DLL 仅依赖 Windows 的 WS2_32、SETUPAPI、KERNEL32、ADVAPI32、Universal CRT 和 Visual C++ 运行时（MSVCP140、VCRUNTIME140、VCRUNTIME140_1）。目标系统需安装 x64 Visual C++ Redistributable。C ABI 自带版本检查，不依赖特定 Python 小版本。

本次构建使用 MSVC 19.51 x64、C++17、Release `/MD`；已实际链接成功并用 Python 加载、读离线状态、析构。供应库：SDK 1.1.0，SHA-256：

```text
lib/a3_sdk.lib
613D585FAF14CBF52F47DC71AA095A1B22198C58C8A07D3E7C62E0E4505EBBAB
```

`src/a32_driver/source_snapshot.json` 记录库/配置/头文件快照哈希和包装器 ABI。SDK 的 MIT 许可保留为 `SDK_LICENSE.txt`。

## 已验证的范围

`tests/test_a32_driver.py` 覆盖完整映射、两款模型的正反解、四指 swing 方向、硬件限位、使能前反馈对齐、USB/ETH 生命周期、右手阻止、找零 WAIT/DONE、新序号检查、找零错误和超时、无页面轮询时的监视线程、取消后显式复位及重新找零、失联、MIT fault、非法输入、原生 DLL 离线加载。测试替身不打开 USB/TCP，也不向实物发运动。

实物连接、通道在线、固件兼容和实际运动尚需现场验收；离线编译和测试不等于硬件已验收。
