# A3.2 原生 SDK 接入

## SDK 1.1.15 更新（2026-10-12）

来源为用户提供的 `a3.2_cpp_sdk-main.zip`，仓库为
[AGIMani/a3.2_cpp_sdk](https://github.com/AGIMani/a3.2_cpp_sdk)。完整源码版本、压缩包
SHA-256 和逐文件校验见 `vendor/a3-cpp-sdk/sdk-source.json`。ZIP 注释记录的提交为
`fd676e5a3296e5e3ada2734d290acc443206e000`。
SDK 源码保留原样，Linux 适配库静态链接它的协议与角度转换实现。

网页采用新版 `Stroke.cpp` 的四类连杆闭环查表算法：四指 MCP 上限 19.627779 mm；
食指/中指/无名指 PIP 和拇指 MCP 上限 16.939021 mm；四指 DIP 和拇指 PIP
上限 12.437477 mm；小指 PIP 上限 11.761490 mm。同一旧动作角度会产生与旧版不同的行程。保存动作仍以模型 rad 表示，
读取后使用新版换算，不改写动作文件。食指/无名指 MCP 现在可下发到应用上限
1.565 rad，不再受旧解析解 1.5229 rad 上限约束。

纯 Python `sdk_stroke.py` 用同源连杆和均匀查表计算并对照实际 SDK C++ 输出测试；离线预览
不需要加载共享库。直线反馈先沿用原有负号校正，再反解同一新版闭环曲线；
反馈允许数学零点和 90° 行程端点，并采用 SDK 的浮点边界容差。
SDK 的线性关节输入范围是 0°–90°（超出端点按端点计算）；当前网页继续使用更窄的
0.005–1.565 **rad** 范围。`method=lut/exact` 保留兼容字段，两者均走新版解析
公式；`conversion=sdk-1.1.15` 标明实际换算。

SELF 实测/参考电流现在均为安培：直线原始码值乘 0.001374，旋转槽
6/7/14/15 的已减零点码值除以 819。换算在 SDK 协议解析中只做一次；
归零 `IqSeekCounts` 仍为 counts，默认 80 不改成 80 A。新遥测/记录使用
`a32-native-sdk-state/v2` 和明确单位；旧 v1 日志保留原始单位，不重算历史数值。

归零字段的 OpenSpin 分类新增第四字节检查：第一字节为 1..4 且第四字节为
0..4 才判 255，修复普通电角度浮点低字节恰好为 1..4 时的误分类。
这不改变 DONE/THETA 的机械归零含义，也不新增硬件停止确认或保护阈值。
本次接入及验证记录见 [1.1.15 更新记录](a32_sdk_1_1_15_update.md)。

## A3.2 with-ID 模型替换（2026-10-12）

`TsenseI_v3_2_withid` 已替换为用户提供的 20-output middle-DIP URDF。源 STL
按米制导入，左手保留源关节坐标；右手由 XZ 平面镜像生成（位置取 `M p`，旋转取
`M R M`，轴向按 `-M a`），因此左右手使用相同关节角时姿态互为镜像。碰撞网格、
MJCF、初始姿态和左右手关键点文件已同步更新，来源和校验记录在
`assets/hands/TsenseI_hand_v3_2_withid/source_mapping.yaml`。关键点是从旧零位自动
重投影的初始值，实机标定前仍标记为待复核；源 URDF 未提供质量/惯量，MJCF 使用仿真
代理惯量。

## 数据链路

手势跟踪和 motion_play 共用 `src/a32_driver/A32Device`：

```text
完整 20 个 URDF 关节角（rad，带型号与手别）
  → 按机构/硬件交集限制关节目标
  → A32StrokeMapper：14 路直线 mm + 6 路旋转 deg
  → a32_hardware.json：18 个 self 槽位 + 2 个 MIT 通道
  → ctypes 加载 liba32_native.so（C ABI v2）
  → Linux 原生 USB CDC / TCP 驱动
  → USB CDC 或 ETH TCP 5001 → H7
```

当前 Ubuntu 副本使用项目 POSIX 传输和 SDK 1.1.15 的协议实现，不运行厂商示例、SDK Web 工作台或转发服务。无需 Bridge HTTP 端口。`motor_index` 是运动学模型编号，不能直接减一当成 SDK 槽号；本接入逐个关节名映射。self 17 使用 SDK bus 2、node 2，底层 CAN node 5 由设备协议处理。

MIT 减速比已按用户 2026-09-29 的要求恢复为 M1 `rotaback=50`、M2 `rota_block=50`。与 SDK 1.1.15 / C ABI v2 的两路 50 换算一致，NativeHand 不再对 M2 下发或反馈做额外缩放。上层使用输出轴 deg/rpm，状态中 `mit_gear_ratios=[50,50]` 标明当前换算。两款 A3.2 模型、左右手、手势控制和 motion_play 共用此配置。

裸手 `TsenseI_v3_2` 和带 ID 的 `TsenseI_v3_2_withid` 共用电机映射与新版行程
换算。四指 swing 模型范围 `[-0.35,0.35]` rad，初值 0；下发先限制在
`[-0.34,0.34]` rad，再直接转换为 SDK deg，落在 SDK ±20° 以内。
当前 CAD 轴向、拇指零位、通道方向/偏移以及 MIT 50:1 换算保留。
范围和模型变换见 [关节范围说明](a32_joint_ranges.md)。模型和手别在会话创建时确定。

## 当前停止策略（2026-10-09）

A3.2 不发送任何电机失能指令，包括用户点击停止、关闭实时控制、停止播放、页面隐藏或退出、心跳超时、输入/轨迹/通信故障、取消或复位找零、断开和关闭会话。停止仍会取消软件使能和后续位置下发；恢复下发仍需用户手动操作。A3.0 / V2 的电源控制规则不变。

软件停止不保证电机停下，也不撤销设备已经接收的运动或找零命令。设备可能继续执行这些命令。`enabled=false` 只表示软件使能状态已清除，不表示电机断电。复位找零仍可发送指定的找零状态复位命令，但不会附带失能指令。

## 使用的兼容接口

保留项目 C ABI v2 和 Python `NativeHand` 接口；不直接使用厂商 `A3HandApi`
的连接/停止生命周期。传输由现有 Linux termios/POSIX socket 处理，数据包由
新版 `packHostInfo`、`packHandBatch`、`packSelfPwr`、`packSelfHoming` 和
`parseHandTelem` 等 SDK 函数处理。目标在原生层通过 SDK `clampSlotPos` /
`clampMitPos` 再限幅。`a32_angles_to_actuators` 是无设备的附加 C ABI，供新版
换算对照检查；不发送命令。

USB 连接仍发送固件查询作为可选元数据，不等待回复或拒绝固件版本。
ETH 为用户指定主机/TCP 5001，不自动扫描局域网。
状态新增 `firmware.hand_side`（0 未设置、1 右、2 左），不据此自动修改手别。
遥测区分 MIT `valid` 与 `online`，保留近零位置解码精度。
接收线程更新状态，不跨线程执行 Python 回调。

18 个 self 槽及 2 个 MIT 输出轴目标一次打包；MIT 速度/电流为 0。
轨迹节奏仍由跟踪和播放各自的控制循环提供，不调用 SDK `startStream`、`wait*`
或 `estop`，也不调用 PID、开环、触觉写入和固件升级接口。

## 生命周期与保护

- 导入、构造、离线预览、读取未连接状态不加载原生库或连接硬件。原生共享库在首次连接时加载。
- 连接不使能、不回零、不发送姿态。左右手均可连接；两侧共用 SDK 槽位映射，固件 `hand_side` 保留为状态诊断字段。
- 手势与动作播放每次开启控制均要求本次连接的新鲜、完整有效反馈，以实测位置重新初始化规划器。连接本身不发送姿态；无反馈时不使用旧命令或页面姿态起步。`enable_all(..., send_initial_pose=False)` 只发使能命令；motion_play 在验证起步反馈后使用此选项，勾选后等待用户调整关节或启动动作。详见 [手势下发规则](a32_gesture_control_policy.md)。
- 遥测缺失、部分通道离线、MIT 反馈无效及遥测超过 500 ms 均不阻止普通姿态发送。500 ms 仅决定反馈是否适合用作起步参考。UI 的 `enabled` 是本次会话已请求使能的状态；SDK 没有全手使能 ACK，不能将其当成硬件确认。
- `joint_limits()` 返回机构与命令限位交集。手势跟踪在 Ruckig 规划前调用 `constrain_pose()`；motion_play 允许 swing 的模型端点 `-0.35/0.35`，校验模型轨迹后在下发时限幅。`prepare_targets()` 的 `applied_pose` 保存实际应用的关节角，日志分别记录页面角度与限幅结果；需要严格硬件限位校验的调用者可使用 `clamp=False`。
- `starting_pose()` 只使用当前连接中不超过 500 ms 的完整反馈；兼容参数 `fallback_pose` 不再参与起步。直线端点噪声及应用限位的最小修正限制在 0.1 mm 内，旋转限制在 0.002 rad 内，避免零位小幅波动导致回退到旧姿态。`seed_missing` 列出阻止起步的缺失或异常关节。显式传入 `start_pose` 也会重新核对新鲜反馈和目标偏差。严格的 `feedback_pose()` 保留供诊断使用。
- 直线反馈由新版 SDK 闭环曲线从 mm 反解为 rad；旋转反馈从 deg 换算并恢复 URDF 方向。`ready` 保留完整反馈及找零就绪诊断，`command_ready` 表示连接且未处于找零/本次找零错误状态；下发不使用 `ready` 作为门槛。
- 找零是可选的异步操作，内部仅使能后对 self 调用 `homeSlot`，不预先发送旧姿态或猜测的位置。主动找零过程中禁止轨迹发送；完成后需再次使能来对齐新位置。
- 找零完成要求新序号遥测，不能用命令之前的 DONE 帧。独立监视线程每 50 ms 检查一次，错误、反馈失效或默认 30 秒超时会停止软件下发，不发送失能；浏览器不轮询也会执行。设备已接受的找零可能继续。关闭会话释放监视线程与原生对象。
- 软件停止后固件可能仍报告 SEEK/RELEASE，可显式调用 `reset_home(slot=None)`（界面的“复位找零状态”）：先停止软件下发，再调用 `homeResetSlot`，软件使能保持关闭，不发送失能。`home_reset_pending` 表示尚无新遥测确认复位，但不阻止用户重新使能和下发，也不声称找零已完成。过期的 SEEK/RELEASE 不作为普通下发门槛；本次主动找零仍受独立监视线程管理。
- 写入失败取消软件使能并停止后续下发；正常关闭也先停止软件下发再断开。这些路径均不发送失能指令，也不保证设备立即停止。

## 构建和分发

在本项目根目录的原生 Ubuntu 终端执行：

```bash
bash src/a32_driver/build_native_linux.sh
```

构建需要 CMake、C++17 编译器、pthreads 和 Python 3，输出 `src/a32_driver/bin/liba32_native.so`，并运行上游 SDK 测试及不连接实物的 TCP/PTY 集成测试。可用 `A32_NATIVE_LIBRARY` 指定其他已构建共享库。运行依赖与 track/play 两套隔离环境见 [Ubuntu 启动说明](../README-UBUNTU.txt)，原生接口细节见 [Linux 驱动说明](../src/a32_driver/native_linux/README.md)。本副本不依赖 Qt 或其他平台的运行时。构建通过后原子替换共享库；已运行的 track/play 服务需停止控制后分别重启，以同时加载新版 Python 换算和共享库。

## 已验证的范围

`tests/test_a32_driver.py` 覆盖完整映射、两款模型的正反解、四指 swing 方向、硬件限位、使能前反馈对齐、USB/ETH 生命周期、左右手连接、找零 WAIT/DONE、新序号检查、找零错误和超时、无页面轮询时的监视线程、取消后显式复位及重新找零、失联、MIT fault、非法输入、原生库离线加载。测试替身不打开实物 USB/TCP，也不向实物发运动。

实物连接、通道在线、固件兼容和实际运动尚需现场验收；离线编译和测试不等于硬件已验收。
