# A3.2 接入手势跟踪与 motion_play

更新日期：2026-09-28。

## 1. 接入结果与范围

两个在线平台都增加了以下两款模型：

| 页面名称 | 程序型号 | 模型来源 | 实物驱动 |
| --- | --- | --- | --- |
| A3.2 | `TsenseI_v3_2` | SDK `daxian_hand_v3_2`，无 ID 外壳 | A3 C++ SDK 1.1.0 |
| A3.2 with ID | `TsenseI_v3_2_withid` | `TsenseI_V3.2_hand_withid_fixed/urdf/TsenseI_left.urdf` | 同一 SDK，单独处理 URDF 方向 |

两款模型的显示、20 个关节、重定向配置、动作文件和手别都有明确身份。左手接入 USB CDC 和 ETH TCP 5001；右手可预览、编辑和离线播放，SDK 的右手硬件表尚未标定，因此禁止右手连接实物。

**运行链路不调用 SDK 的任何 exe。** 项目内的 `a32_native.dll` 静态链接供应商 `a3_sdk.lib`，Python 用 `ctypes` 在同一进程调用 DLL。已完成原生库构建、离线加载、模拟设备测试及网页验证；本次没有连接实物或下发运动，实机验收仍需完成。

对应代码入口是 `online_control/online_motion_track` 与 `online_control/online_motion_play`。Qt 主窗口旧的 V2 `UsbController` 保持原用途；A3.2 没有被接进 V2 的 21 槽协议。

## 2. 启动与使用

在项目根目录使用已有 `hand_dex` 环境：

```powershell
# 手势平台
& E:/conda/envs/hand_dex/python.exe online_control/online_motion_track/main.py --port 8766 --no-open

# 动作平台（另开终端）
& E:/conda/envs/hand_dex/python.exe online_control/online_motion_play/server.py --port 8765 --no-browser
```

- 手势入口：`http://127.0.0.1:8766/?model=TsenseI_v3_2_withid`；裸手将型号改为 `TsenseI_v3_2`。
- 动作入口：`http://127.0.0.1:8765`，在型号选择框切换 A3.2 / A3.2 with ID。
- USB 地址可填 `auto` 或具体 `COM5`；ETH 填设备 IP，默认端口 5001。
- 设备应连接在运行 Python 服务的 Windows x64 电脑。随包 SDK 库只有 Windows x64 版本；node0/Linux 可运行模型预览，实物控制需本地 Windows 服务。

手势操作顺序：选择型号和左手 → 选择碰撞停止或 Mink → 连接 SDK → 开始识别 → 开启实时下发。按 A3.0 的控制逻辑，首次开启需要一包有效手部遥测，无需手可见、反馈齐全或先找零。开启时逐关节参考有效反馈，缺失部分采用上次成功下发姿态或初始姿态。切换模型、停止识别、隐藏页面或结束会话会请求停止并断开；手势暂时消失会继续完成最后目标。全部开启及停发条件见 [手势下发规则](a32_gesture_control_policy.md)。

驱动层普通姿态发送要求已连接、已请求使能且未正在找零；遥测缺失、过期、通道不全或未找零均不阻止驱动调用。手势层另检查首包遥测和明确的 MIT 故障。motion_play 首次缺失反馈时从页面姿态起步，后续优先使用上次成功下发姿态。页面显示使用软件姿态的关节数量，不将其当作实测反馈。界面软件使能状态表示指令已请求，SDK 没有电机使能 ACK。

取消或找零错误后，如固件仍处在找零状态，可先按“复位找零状态”，再重新找零或使能，无需等待复位遥测。复位不代表机械找零完成。两个平台可以同时打开预览，同一只手应由其中一个平台连接控制。

首次构建或更换 SDK 后：

```powershell
.\src\a32_driver\build_native.ps1 -SdkRoot D:\Qtworks\a3-cpp-sdk
```

已提供本次构建的 `src/a32_driver/bin/a32_native.dll`。构建需要 MSVC x64/CMake，运行需要 x64 VC++ 运行时；构建工具不属于 SDK 控制程序。详见 [原生 SDK 说明](A32_NATIVE_SDK.md)。

## 3. 为什么不能直接复用 A30

| 项目 | A30 | A3.2 |
| --- | --- | --- |
| 机构输出 | 9 直线 + 11 旋转 | 14 直线 + 6 旋转 |
| 下发实现 | 原 A30 浏览器串口编码 | C++ SDK `A3HandApi` |
| 电机通道 | 原 A30 独立布局 | self 0–17 + MIT 0–1 |
| 连接所在电脑 | 网页浏览器本机 | Python SDK 服务本机 |
| 动作文件 | `motions_a30...json` | `motions_a32...json` |

A3.2 原有代码仅提供机构转换和手势预览。本次新增原生驱动、反馈反解、使能/找零/停止、轨迹下发和动作编辑会话；保留 A30 的既有协议和 UI 分支。

## 4. 模型、方向和资源

`src/hand_profiles.py`、`online_control/shared/hand-models.js` 注册两个独立型号，资源分别位于：

```text
assets/hands/TsenseI_hand_v3_2/
assets/hands/TsenseI_hand_v3_2_withid/
```

两款模型保留原始网格、连杆和局部关键点，带 ID 底座 STL 保留 `0.001` 缩放。2026-09-28 起，PIP/DIP 为 `[0.005,1.565]`、MCP 为 `[0.005,1.3]` rad；四指 swing 的模型范围为 `[0,0.7]`、默认 `0`，下发范围为 `[0.005,0.695]` rad。swing 同步调整关节原点与轴向，左右手 MJCF 一并更新。详见 [范围和坐标说明](a32_joint_ranges.md)。

裸手四指 swing 轴向已与带 ID 对齐，并重设 origin 保持原摆动区间和 `0.350` 中间姿态。从原始 CAD 转换时，裸手使用 `原始 SDK 角度+0.350`，带 ID 使用 `−原始 ID 角度+0.350`。两款型号都将当前模型角限幅到 `[0.005,0.695]` 后转成 SDK 的度。反馈从度直接换算回 rad；动作和显示保存当前所选 URDF 的弧度。

带 ID 拇指与裸手存在局部 CAD 坐标差异，此处复用的是同机构的关节语义和角度到行程关系，不能理解为两套网格几何完全一致。带 ID 的 21 点由几何生成，仍需人工标定确认；右手来自上传的临时镜像。碰撞模型是几何代理，覆盖检查不等于完整实物碰撞认证。

带 ID 每侧使用 10 个指节胶囊和 5 个完整末端外壳凸包。末端凸包从全部 STL 顶点生成，修复原上传代理只覆盖细小尖端、漏掉指腹的问题。已验证指腹/指尖表面接触及跨手指停止；该配置启用跨手指碰撞，外部环境和同指相邻机构接触不在这套自碰撞检查范围内。

## 5. SDK 通道映射

唯一硬件配置为 `src/a32_driver/a32_hardware.json`。槽位和单位来源于 SDK 配置；命令范围已按用户指定角度和现有机构模型更新。方向/零偏保留原值，模型 swing 坐标在机构转换层处理一次。

| 通道 | 关节 | 单位 | 当前软件命令范围（四舍五入） |
| --- | --- | --- | --- |
| self 0 / 1 / 2 | 小指 DIP / PIP / MCP | mm | 0.039782–8 / 0.047213–15.563614 / 0.063461–15.917403 |
| self 3 / 4 / 5 | 无名指 DIP / PIP / MCP | mm | 0.039889–8 / 0.048521–16.493970 / 0.052956–17.506244 |
| self 6 / 7 | 小指 / 无名指 swing | deg | 0.286479–39.820567 |
| self 8 / 9 / 10 | 中指 DIP / PIP / MCP | mm | 0.040020–8 / 0.048437–16.480593 / 0.063562–15.926229 |
| self 11 / 12 / 13 | 食指 DIP / PIP / MCP | mm | 0.039889–8 / 0.048521–16.493970 / 0.052956–17.506244 |
| self 14 / 15 | 中指 / 食指 swing | deg | 0.286479–39.820567 |
| self 16 / 17 | 拇指 PIP / MCP | mm | 0.040022–12.380970 / 0.049437–13.723953 |
| MIT 0 | `thumb_rotaback_joint2` | 输出轴 deg | 0–99 |
| MIT 1 | `thumb_rota_block_joint` | 输出轴 deg | 0–119 |

`motor_index` 是机构源模型 1–20 编号，不是 SDK 槽位。MIT 不当作 self 18/19；SDK 对两路 MIT 按减速比 50 换算下发和反馈，NativeHand 不做额外缩放。self 17 使用 SDK bus 2/node 2，底层 CAN node 5 的差异由 SDK/固件负责。

`A32Device.joint_limits()` 将机构可达范围、URDF 范围和上述实物限位求交。目标先在关节空间约束，再做 Ruckig；正式采样用 `clamp=False`，避免到发送时偷偷夹紧而导致模型、轨迹与硬件不一致。

2026-10-05 起，四指 DIP 的机构行程上限为 8 mm。14 路直线电机的 SDK 回传目标和
实际位置在解析时乘以 `-1`，统一到下发命令坐标；6 路旋转反馈和下发方向保持原值。
日志保留原始 SDK 字段，使用 `linear_feedback_sign: -1` 标记已转换的派生位置，避免重复取反。

## 6. 调用了哪些 SDK

只使用一套硬件 SDK：**A3 C++ SDK 1.1.0**。角度转换复用项目 `src/a32_kinematics` 的 A3.2 几何模型和 LUT，反馈用其解析机构反解。MediaPipe、NumPy 重定向、MuJoCo/Mink 和 Ruckig 是现有平台的视觉/规划依赖。

| SDK API | 用途 |
| --- | --- |
| `connectUsbAuto` / `connectUsb` / `connectEth` | 连接、固件协议兼容检查 |
| `setPositions` + `setMitAll` + `sendOnce` | 更新 18 self 和 2 MIT 并发送一帧 |
| `enable` | 全手使能/失能；使能前写入起步目标，允许缺失反馈 |
| `homeSlot`、`IqSeekCounts` | 18 self 全部或单槽找零 |
| `homeResetSlot` | 停止后显式复位找零状态，恢复取消/错误后的找零流程 |
| `selfTelem` / `mitTelem` / `boardTelem` / `firmware` | 反馈、温度/电压和版本 |
| 遥测/断线回调、状态与计数读取 | 反馈新鲜度、断线和诊断 |
| `estop` / `disconnect` | 停流、请求失能、关闭连接 |

不需要 SDK 的 `a3_web_bridge.exe`、`a3_example.exe`、`start_joint_ui.bat`、原 Joint UI 或 HTTP Bridge；不调用这些文件。也不需要 MATLAB、ROS、PID 写入、开环旋转、触觉设置、固件升级、A30/V2 串口编码。SDK `startStream` 和阻塞 `wait*` 没有使用，两个平台自己控制轨迹时钟并以 `sendOnce` 下发。

逐个 API、ABI、依赖 DLL、供应库哈希和限制见 [A32_NATIVE_SDK.md](A32_NATIVE_SDK.md)。供应商 MIT 许可随 `SDK_LICENSE.txt` 一起打包。

## 7. 两个平台的数据流

### 手势跟踪

```text
浏览器 MediaPipe 21 点
  → main.py / A32WebSession
  → 所选模型的重定向 + 目标碰撞检查
  → A32Control：可选反馈及软件姿态起步、Ruckig、采样轨迹/碰撞检查
  → A32Device：机构转换、限位、槽位映射
  → NativeHand → a32_native.dll → A3HandApi
```

`a32/control.py` 使用独立控制线程，目标周期 5 ms。侧摆速度 1 rad/s，其他关节 2 rad/s，加速度 8 rad/s²，jerk 80 rad/s³。实际周期使用单调时钟；采样超过 100 ms 不再单独导致停发，规划每次最多推进 5 ms，不补发积压帧。网页心跳超过 2 秒中断下发；会话到期也释放设备。碰撞和关节限位恢复与 A3.0 共用实现，按手指分组规划。

每次 SDK 成功写入后才提交显示姿态和命令样本。浏览器每 200 ms 获取硬件状态及未读命令样本，原有浏览器记录功能保留；其命令环形缓冲保留最近 800 帧，读取落后造成缺帧会记录 `a32_sdk_recording_gap`。

服务器连接时自动开始独立 JSONL 记录。手势原始检测、重定向输入/结果、碰撞后目标、启用起步姿态、转换及每次成功/失败写入通过 `frame_id` 关联。控制线程每约 20 ms 观察 SDK 状态，记录不依赖浏览器轮询，也不受上述 800 帧缓冲限制。SDK 遥测是缓存快照抽样，不代表全部 USB 原始帧。

### motion_play

`a32_session.py` 提供 20 关节编辑、保存/加载/删除、导入/导出、序列/循环、过渡/间隔、实时控制和动作同步。采用可选反馈及软件姿态起步，由独立 20 轴 Ruckig 规划；停止、关闭和控制权超时取消下发。

motion_play 后续修正：轨迹由独立线程运行；页面隐藏或控制页心跳超时只停止/失能，保留 SDK 连接，明确断开或释放控制才关闭连接。停止后可以直接勾选实时控制来重新使能。连接会自动开始 JSONL 记录，页面提供下载和数据查看器入口。详见 [连接诊断](a32_connection_diagnostics.md) 与 [数据记录](a32_motion_recording.md)。

动作文件按型号和手别分成四个文件，带 `_model_id`、`_hand_side`。导入时校验完整关节集合、数值、机械范围和时长；A30、V2、带 ID 与裸手文件不能混用。动作编辑平台没有加入手势碰撞模块。详见 [motion_play 接入说明](a32_motion_play.md)。

### 共用日志与数据查看器

两平台共用 `src/a32_driver/recording.py`，旧 `online_motion_play/a32_recording.py`
保留兼容导入。默认日志目录为 `E:\TsenseI_hand_logs\A32`，两个型号通过
`metadata.model_id` 区分。父目录配置在 `configs/control_log_parent.txt`；查看器
可修改父目录，下一次录制使用新目录，当前文件继续写到原位置。

格式沿用 `hand-control-session/v2`。每个事件包含单调相对秒数 `t`、主机采样
时间 `wall_time_unix_ms` 和带时区、精确到毫秒的 `wall_time`。设备 `tick_ms`
及浏览器 `timestamp_ms` / `capture_unix_ms` 单独保留；`h7.control_dt_s` 和
`control_period_ms` 表示实际控制周期。异步写盘不会替换原采样时间。

查看器支持实际时间、范围选择、平均/P95/最大控制周期、CSV 和恰好 20 个 A3.2
通道。离线、缺失或过期测量显示断线并导出空值，不用缓存零值冒充反馈。
两个平台提供 `/api/viewer/logs`、`/api/viewer/log-file`、`/api/viewer/log-parent`，
支持本机目录刷新、直接打开与时长汇总。日志覆盖等待和暂停时段，汇总时长
不是电机通电或运动时长。详细事件与接口见 [数据记录](a32_motion_recording.md)。

## 8. 核对范围与验证

本次按依赖链核对了模型注册、URDF/STL、镜像坐标、21 点配置、FK/重定向、MJCF、机构正反解、SDK 头文件/配置/静态库、控制会话、网页 API、权限/控制权、运动文件、显示/日志、停止/断线/找零、打包白名单和现有测试。旧代码中失效的裸手生成来源也改为 SDK 的实际来源；用户已有模型校准、动作数据及其他未提交修改没有被重置。

可复现检查：

```powershell
$env:NO_PROXY='127.0.0.1,localhost'
& E:/conda/envs/hand_dex/python.exe online_control/online_motion_track/main.py --check --no-open
& E:/conda/envs/hand_dex/python.exe -m pytest tests/test_a32_driver.py tests/test_a32_models_integration.py tests/test_a32_mapper.py tests/test_a32_tracking_control.py -q --basetemp .pytest-runtime/a32-check
& E:/conda/envs/hand_dex/python.exe -m pytest online_control/online_motion_play/test_a32.py -q --basetemp .pytest-runtime/a32-motion-check
```

有正在运行的本地服务时，还可运行 `online_motion_track/tests/a32-sdk-browser.mjs` 和 `online_motion_play/test_a32_browser.mjs`。它们使用真实 Chrome 加载网格、检查左右手和接口，不连接手或使能电机。

软件验证覆盖两种模型的正反解与符号、18+2 映射、硬件限位、反馈起步、失联/超时、找零、非法输入、动作文件隔离、网页模型加载和打包完整性，并回归现有 A30/V2。最终结果见 [验证记录](a32_integration_validation.md)。

尚待实机确认：固件握手、20 路在线状态、机构零位/方向/实际行程、找零电流、带 ID 标定、碰撞代理与外壳匹配，以及 USB/ETH 下真实连续运动。SDK 在链路已断或进程崩溃后无法保证电机停止，该底层能力不能用网页心跳替代。

## 9. 带 ID 的 952 g 参数与裸手资源目录（2026-09-30）

带 ID 的当前左右手 URDF 已按关节的子连杆匹配用户提供的
`TsenseI_V3.2_hand_mass_inertia_corrected_952g`。两侧各 22 个连杆、20 个活动关节，
每手总质量为 `0.952 kg`。质量和完整六项惯性张量直接采用已修正的源 URDF 数值；
质心、关节原点、轴向、范围、网格与比例保留当前项目数据。源惯性是按连杆质量比
`I_new = I_original × mass_new / mass_original` 缩放的估算。

持久参数位于 `assets/hands/TsenseI_hand_v3_2_withid/inertial_parameters.json`，
包含两侧逐关节/连杆映射、质心、惯性、原表格单元格和输入文件哈希。
`tools/apply_a32_withid_inertials.py` 验证连杆集合、关节父子关系、质心坐标、质量及惯性
后写入参数；重复应用不会再次缩放。带 ID 的 CAD 导入脚本也会重新应用该参数文件。
当前 MJCF 继续用于既有碰撞过滤，其代理质量及控制参数不随 URDF 参数导出而改变。

按请求新增的项目根目录 `TsenseI_A3.2_hand/` 提供裸手左右手 URDF 和 44 个完整 STL。
它是 `assets/hands/TsenseI_hand_v3_2` 的资源副本，含已校正的拇指机械零点，质量沿用裸手
原数据。应用模型入口继续使用 `assets/hands` 下的资源。

两份可独立解压的压缩包位于 `exports/`：

- `TsenseI_A3.2_hand_withID_952g_left_right_20260930.zip`：带 ID 左右手、网格、参数及逐关节映射报告。
- `TsenseI_A3.2_hand_left_right_20260930.zip`：上述新增裸手资源目录。

包内 `resource_manifest.json` 记录导出文件哈希。带 ID 包中的原始说明和质量/惯性报告
位于 `reference/source_reports`，Shadow/钢琴仿真设置位于 `reference/simulation_configs`，
作为参考保存；这些场景专用设置未写成手部硬件参数。

验证覆盖 44 个带 ID 连杆的源报告一致性、质量和、惯性正定/三角不等式、左右惯性及质心
镜像、重复应用、原 CAD 重新导入、全部网格路径及两份 ZIP 的 CRC/SHA256。
8 项现有模型加载、坐标和 FK 相关回归通过。本次未执行实物或动力学标定。
