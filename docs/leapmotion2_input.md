# Leap Motion Controller 2：骨架与相机输入

## 红外相机 → MediaPipe（2026-10-10）

工作台和动态标定页新增 **Leap Motion 2 相机 · MediaPipe · 120 fps**。
先停止当前输入，再选择此项并开始识别。也可打开
`https://127.0.0.1:8766/?source=leapmotion_camera`；相机检查页是
`https://127.0.0.1:8766/leap-camera-preview.html`，支持红外画面、21 点叠加和独立帧率统计。
设备接在运行服务的 Ubuntu 电脑上，仍使用 `bash migration/ubuntu/start-track.sh` 启动。

此入口通过 LeapC `eLeapPolicyFlag_Images` 读取相机 0（左路）的 8 位灰度图像，
复制原生缓冲区后送入网页现有 MediaPipe Worker。灰度扩展为 RGB，21 点与三维估计
均来自 MediaPipe；原有 **Leap Motion 2 · 红外追踪** 仍使用官方原生骨架。
接入依据见 [官方图像接口说明](https://docs.ultraleap.com/api-reference/tracking-api/leapc-guide/images.html)。
当前使用原始广角红外图，尚未做畸变校正或左右双目三角测量；Z 坐标是模型估计，
提高采集帧率不等于提高深度精度，边缘画面和遮挡下的效果需要单独测量。

相机连接在 HMD 模式下请求 `fast_hand_motion`、`high_hand_fidelity`，
通过 `LeapSetDeviceHints` 提交，以 `NULL` 结束数组。原有骨架入口保持只请求
`high_hand_fidelity`。这些 [Hints](https://docs.ultraleap.com/api-reference/tracking-api/group/group___hints.html)
是服务偏好，不是强制设置摄像头帧率的 API，也不能宣称已被采纳。

- 120 fps 采集需要 USB 3 数据线和接口，实际协商速度可用 `lsusb -t` 检查。
- 页面分别显示相机采集 FPS 和 MediaPipe 识别 FPS；后者取决于浏览器、GPU 与负载，不能保证 120 次推理/秒。
- 采集与推理并行，只保留最新图像。不会重复旧图来凑帧，也不受显示器 60 Hz 刷新率限流。
- 图像超过 350 ms、设备断开或停止会话时不再输出该图像的手部结果。最后一个相机会话停止后释放应用的 LeapC 连接。
- 采集统计按实际收到的 ImageEvent 计算，窗口约 2 秒，至少测量 1 秒才显示。
  低于目标 98% 会提示未达目标；小幅抖动仍显示实际测量值。
- 浏览器需要支持 MediaPipe 图像处理所需的 WebGL；GPU 不可用时会尝试现有 CPU 回退。
  若回退也失败，相机检查页保留红外预览并明确显示“MediaPipe 未运行”，不会把采集成功当成识别成功。

本机 USB 3 / 5 Gbps 的 10 秒 HTTP 实测：512×512，收到 1,196 幅独立图像；
稳定段传输 119.35 fps，末次采集 119.37 fps，重复帧 0，图像龄期中位数 9.11 ms。
数据保存在 `runtime/ubuntu/leap-camera-benchmark.json`。这是一次短时测量，不代表持续负载下每个时刻都能达到 120 fps。
本次 Codex 内嵌浏览器禁用 WebGL，已验证实拍画面，但网页 MediaPipe 端到端性能仍需在可用的 GPU 浏览器中验证。
另外，将用户放入视野的手掌采样 6 帧，使用工作台同一份 `hand_landmarker.task`
和临时目录中的 MediaPipe 1.0.1 CPU 运行时离线检测，6/6 均输出左手的完整 21 点，
单次推理 22.41–27.71 ms。这不是网页实时 FPS，也不是 120 fps 推理验证。
结果见 `runtime/ubuntu/leap-camera-mediapipe-check.json`；临时检测依赖未安装到 track/play 环境。

补充延迟测量：以 Leap 图像原生时间戳为起点，经过正式 HTTPS 图像接口、灰度转 RGB，
到本机 MediaPipe CPU 返回结果为终点，8 秒采样 486 帧。跳过前 5 帧后总延迟中位数
31.94 ms、P95 36.43 ms；图像收到时的龄期中位数 17.31 ms，推理中位数 13.43 ms。
此次实时采样未检出手，因此不能将该数字当成有手时的总延迟。
结合此前有手的 22–28 ms 推理耗时，有手全链路可粗估为 39–45 ms，但仍需同步实测。
不包含重定向求解、机械手下发或机械响应，也不是网页 Worker 的延迟。
完整统计见 `runtime/ubuntu/leap-camera-latency.json`。

接口：创建 `/api/session` 时传 `input_source: "mediapipe"`、
`camera_source: "leapmotion_camera"`；`POST /api/leap-camera/frame` 传
`session` 和上次 `after_revision`（初次为 -1）。二进制响应为 4 字节小端 JSON 长度、
UTF-8 元数据、`width × height` 个 gray8 像素；`has_frame: false` 时像素为空。
该接口复用同源/会话校验。推理结果仍走 `/api/frame`，附带 `source_metadata.camera`；
停止沿用 `/api/stop`，标定页冻结输入也会释放相机。

## 原生 Ubuntu Hint 配置（2026-10-09）

工作台使用头显模式 `TrackingMode.HMD`，通过同一条启用 `multi_device_aware`
的 LeapC 采集连接，在设备打开、订阅并设为 primary 后，先请求 HMD，再调用
`LeapSetDeviceHints`，只发送 `high_hand_fidelity`（`LEAP_HINT_HIGH_HAND_FIDELITY`）。
请求列表位于 `leapmotion_source.py` 的 `DEVICE_HINTS`；每次设备/服务重连后重新设置模式和 Hint。
绑定未导出该字符串宏，因此使用头文件对应的 ASCII 字符串；CFFI 为字符串自动补 NUL，
并以 `ffi.NULL` 结束 `const char*[]` 数组。对应的原生调用为：

```c
LeapSetTrackingMode(connection, eLeapTrackingMode_HMD);
const char *hints[] = { LEAP_HINT_HIGH_HAND_FIDELITY, NULL };
LeapSetDeviceHints(connection, device, hints);
```

设备应按头显方式朝向前方手部操作区域。Hint 是追踪偏好，服务决定实际使用的模型；
它不是 Z 轴校准参数，不能替代实际精度测量。
`source_metadata.device_hints` 返回 requested、status、error、device_id 和 effective：
`submitted_unverified` 仅表示 LeapC 调用成功，effective 保持 null；
`request_failed` 表示调用失败，错误会记录，但骨架采集继续。
只订阅第一台枚举到的设备，避免多个相机的时钟和手部身份混入同一条骨架流。

启动方式仍为 `bash migration/ubuntu/start-track.sh`。track 启动器和 binding 安装器设置
Linux LeapC 动态库搜索路径；仅在 track 环境安装 binding，不影响 play 环境。
这些 Hint 是连接期间的 API 请求，不是 `/etc/ultraleap/hand_tracker_config.json` 配置键。

本机实测（2026-10-09 18:53，原生 Ubuntu 24.04 / Linux 服务 6.2.0 / LMC2）：
`get_tracking_mode()` 返回 `TrackingMode.HMD`，服务日志记录
`HintResolver accepted hints: high_hand_fidelity`，选择 `ldat-436727.ldat`。
20 秒内收到 1,721 个追踪事件，包含手部检测，单条连接未中断。
随后通过实际工作台 HTTPS 接口进行两轮各 12 秒的启停采集，均收到持续递增的帧序号，
没有采集错误；每轮日志均再次确认 HMD 和高保真 Hint。该两轮未检测到所选右手。
当前链路协商为 USB 2.0 / 480 Mbps，服务设置为 `256x512x90`、90 fps；
服务也明确提示 USB 2.0 会降低追踪质量，应使用 USB 3 数据线和接口。
此前日志曾记录视频接收失败、缺失帧后重置设备；本次短时恢复不能证明长期稳定性或深度精度。
关闭诊断连接后，该连接的 Hint 不再参与选择；工作台下次开始识别时自动重提。
本次相关 Python 回归 273 项、网页输入 JavaScript 测试 19 项及 track 启动检查通过。

参考：[官方 Hint 列表](https://docs.ultraleap.com/api-reference/tracking-api/group/group___hints.html)、
[接口约束](https://docs.ultraleap.com/api-reference/tracking-api/group/group___functions.html)、
[模型适用范围](https://docs.ultraleap.com/hand-tracking/Hyperion/trackingmodels.html)。

网页手势映射工作台及 `/calibration` 动态标定页新增 **Leap Motion 2**，来源标识为 `leapmotion`，与 MediaPipe、Quest WebXR、Wuji 手套并列。设备连接到运行 Python 网页服务的 Linux 主机，浏览器读取该服务收到的骨架；浏览器所在电脑的 USB 设备不会自动传给服务器。

```text
Leap Motion Controller 2
    → Linux Ultraleap Tracking Service → LeapC / 官方 Python leap
    → 米制 21 点 → 现有输入验证与 retarget → 模型预览 / 原有控制流程
```

适配器面向原生 Ubuntu，当前使用头显模式（`TrackingMode.HMD`）。Python binding 是可选依赖，只在开始读取 Leap 输入时导入；普通服务启动和其他输入源不要求安装它。接入采集不等于开启机械手输出，实物连接与下发仍由工作台原有操作决定。

## 适用环境与当前验证范围

截至 2026-10-03，[Ultraleap Linux 文档](https://docs.ultraleap.com/linux/index.html)明确列出的支持环境是 **Ubuntu 22.04 LTS、x86_64 / amd64**。[LMC2 下载页](https://www.ultraleap.com/downloads/leap-motion-controller-2/)提供 Linux Hyperion 6.2.0，要求 AVX 指令支持，并要求 LMC2 使用 USB 3.0 端口。当前项目使用 `Hand-Ubuntu-24.04`；Ubuntu 24.04 和 WSL 不在该页面明确列出的支持范围内，需要设备实测，不能因 Python 代码能启动就认定驱动兼容。

2026-10-04 已在当前 WSL 安装 **Ultraleap Tracking Service 6.2.0.0**、LeapC，以及适配现有 Python 3.12 虚拟环境的官方 Python binding。服务以 `leap` 用户运行；`leapctl` 和项目真实输入适配器均已连接成功。2026-10-05 已通过 USB 直通连接真实 LMC2，项目适配器连续收到有效左手 21 点数据；未进行机械手联动验收。

### 已连接却没有手部数据

2026-10-05 现场发现设备红外画面可见手，但当前摆放方向与默认 `normal` 方向相反。通过官方配置旋转 180° 后，项目立即恢复左手识别，采样期间数据龄期约 16–38 ms（不是端到端机械控制延迟）。当前设备已保存为 `fixed-inverted`。在 WSL 内可查询或调整：

```bash
leapctl config orientation
leapctl config orientation inverted
# 如果以后将设备水平旋转 180°，按实际画面改回：
leapctl config orientation normal
```

上述方向设置取决于实际摆放位置，不应对所有设备固定使用 `inverted`。工作台选择与实际伸出的手一致的左右手；本次验证识别的是左手。页面已修复无手数据时误报“骨架已过期”的提示，旧页面按 Ctrl+F5 刷新。

本次也出现过视频帧中断及 USB 连接断开。若 `usbipd list` 显示 `Shared`，需要重新 `attach`；若 Windows 的 Connected 列表已无设备，先检查接线并重新插拔，再直通。仅有 `Attached` 或服务连接成功不足以证明正在输出有效手部骨架。

后续实测仍发现稳定性问题：诊断文件 `hand_diagnostic_20261005_100435_122000.jsonl` 中，10:04:42 收到有效重定向结果后失去骨架；同一时间服务日志出现 `error receiving video frames`，10:04:44 出现 `missing frame data` 并重启设备。随后即使 usbipd 显示 `Attached`，`leapctl devices` 仍无设备；解除直通后 Windows 一度报告“未知 USB 设备（设定地址失败）”。因此上述方向调整只验证了短时真实识别，不代表长时运行稳定性已通过；USB 数据链路、设备重置和 WSL 直通兼容性仍需排查。现有日志不足以单独判定线材、硬件或 USB/IP 哪一项是根因。

同日恢复记录：重新插拔后，三轮 25 秒采集及短暂停止后重连均收到有效左手数据；较长空闲后服务日志也重新记录到递增帧数。之后 WSL 重启，设备更换为 BUSID `2-2`；重新附加后，实际链路协商为 **USB 2.0 / 480 Mbps**，服务自动采用 `256x512x90`（原 USB 3 链路为 `512x512x120`）。完整 A3.2 应用采集与求解复测 45 秒，877 次读取中有 826 次有效骨架/求解结果，来源序号持续增长，没有服务断连或设备丢失事件；丢手期间仍持续接收无手帧。这是当前连接下的恢复验证，不代表原 USB 3 链路的问题已根治。后续更换端口或数据线恢复 USB 3 时应重新做持续采集验证。

##补充验证：单独原始采集 90 秒收到 10,654 帧，无设备丢失、SDK 错误或过期帧；随后通过真实 HTTP API 交替开启 A3.2 与 A3.0 WithID 共 8 轮，中间停止所有 Leap 会话空闲 180 秒，8 轮均收到递增追踪序号。测试期间没有检测到手，因此只验证连接/帧流，未完成本次真实 21 点识别与求解验收。

若要进一步判断 USB 故障的最初原因，需要在 Windows 管理员终端抓取 usbipd Trace 与 USB/IP 包头。本机已准备 E:\WSL\capture-hand-leap-usb.ps1，可用 powershell -ExecutionPolicy Bypass -File E:\WSL\capture-hand-leap-usb.ps1 运行。它识别唯一的 LMC2、拒绝在其他 USB 设备也直通时启动，临时停止生产 usbipd 服务，运行调试服务并采集 10 分钟，然后恢复生产服务和 Leap 直通。会短暂重连 Leap；采集限制为每包 128 字节，保存到 E:\WSL\leap-usb-trace-时间戳\。当前 Agent 进程没有管理员权限，尚未运行该捕获；这是下一步定位所缺的证据。

## 再次断流的证据与 SDK 修复（2026-10-05 11:42 起）

用户提供的 `hand_diagnostic_20261005_113729_975000.jsonl` 在 11:37:37–11:38:20 仍有有效 A3.2 左手结果；`hand_control_20261005_114201_470000.jsonl` 开始 A3.0 WithID 会话后，只收到一次新流序号 14860，随后报告设备丢失，新连接长期停在序号 -1。这不是换成 25 点或开启电脑性能模式能够解决的问题。

同一时间的原始日志给出了更低层的故障：11:42:02 Tracking Service 报视频接收失败，11:42:04 报缺失帧并重启设备；Windows 的 usbipd-win 随后记录 TCP 通信超时并释放设备。WSL 中 USB 查询和网页服务 accept 系统调用曾进入阻塞，内核记录 RCU stall。保存日志后重启 WSL、重新附加设备，恢复了 USB 3.0 原始采集。**这些证据确认故障涉及服务视频采集和 USB/IP 链路，但不能单独断定线材、设备固件、Linux 驱动或 WSL 内核哪一项是最初原因。**

还独立复现了官方 Python binding 的兼容缺陷：服务恢复时发送 `NewDeviceTransform (272)`，枚举存在，事件工厂与 Listener 分发表却没有对应项，导致 `KeyError`，SDK 采集线程退出而连接状态仍可显示已连接。本机虚拟环境已修复该通知的元数据包装与默认分发；它不生成骨架，不改动 21 点转换。安装脚本为：

```bash
/home/onkiall/.venvs/hand-track/bin/python tools/patch_leapmotion_sdk.py
```

脚本仅针对本次安装的 binding，修改前保存 `.before-hand-transform` 备份；其他新版本应先核对实现。项目采集器另增加 SDK 线程存活检查，线程退出时清除旧观察并重连；服务连接后超过 5 秒无追踪帧时，页面明确提示帧中断。正常的无手 TrackingEvent 不触发该错误。回归测试覆盖“变换通知后继续收到 TrackingEvent”和“采集线程退出后旧帧不能提交、重连后恢复”，46 项 Leap 相关测试通过。

为排查重新开流曾试验独立保活程序，但 12:04:25 在持续连接期间仍断流，并反复出现增益 0 的重新开流提示。因此保活不能视为修复，已停止该程序并恢复启动器，工作台不自动运行它。原始采集与工作台持续验证记录保存在 `~/.cache/hand-ultraleap/`；11:42 故障的服务、内核、阻塞任务日志保存在 `failure-20261005-1142/`。短时恢复到约 119 Hz 不能视为长时稳定性验收。

## USB 3.0 复测与速度定位（2026-10-05）

设备重新接到 BUSID `2-5` 后，`lsusb -t` 确认实际设备链路为 **5000M**，服务采用 `512x512x120`。不能仅凭接口颜色或根集线器的 `10000M` 判断设备速度，应读取设备本身的那一行。

- 原生 LeapC 连续采集 120 秒：14,205 帧、5,128 帧含手，稳定阶段约 119 Hz，最大回调间隔 18.54 ms；没有设备丢失、连接丢失或 SDK 错误。采集帧龄期中位数约 9–12 ms、每秒 p95 约 11–15 ms。这不是机械控制或显示端到端延迟。
- 三轮各 20 秒采集、轮间断开 SDK 客户端并空闲 45 秒：每轮约 2,380 帧，全部恢复，最大回调间隔小于 20 ms，没有过期帧或断连。
- 主动解除并重新附加 USB 直通后，项目采集线程收到 `device_lost`，换用新的流 ID 并自动恢复递增帧和真实右手数据；无需重启服务。解除直通时 Windows 需要重新枚举设备，立即连续执行 `detach` / `attach` 曾暂时报“没有该 BUSID”，等待 `usbipd list` 再次出现 Ultraleap 后附加成功。此人为断开的服务错误日志不能算作自发故障复现。
- 完整 A3.2 应用采集/求解连续运行 120 秒，设备帧序号持续递增，但初期手部识别间歇丢失。相机画面显示手在视野下缘、手指不完整；将张开的右手移到设备正上方后，20 秒原生测试的 2,367 帧中有 2,361 帧含手。居中后的完整应用测试 60 秒，3,189 次带手输入有 3,187 次成功求解。丢手与 USB 断连必须分别判断。
- 安静电源模式下，居中测试的原始采集约 119 Hz，带手求解约 51–54 Hz，求解耗时中位数约 13 ms。此时应用求解是额外瓶颈，USB 5 Gbps 并不等于每个机械手姿态都能以 120 Hz 完成求解。

上述完整应用测试使用 A3.2、右手、碰撞模式 `off`、不录制，不连接或下发机械手。其他模型、碰撞配置、录制和浏览器负载需要独立测量。后台求解读取最新帧，不排队补算全部相机帧；页面约 60 Hz 轮询不代表原始采集只有 60 Hz。

Windows 原电源方案为“安静模式”。为速度定位，临时切换到已有“野兽模式”，用完全相同的两组真实记录姿态做离线求解对照：360 次有效求解的完整处理耗时中位数由 11.15 ms 降为 8.46 ms，再次复测为 8.48 ms；p95 从 16.90 ms 降到约 10.17–10.35 ms。随后电脑自动切回“均衡模式”，用户确认没有手动切换，并说明目前无法使用野兽模式。因此最终保留**均衡模式**，不再强制切换。

均衡模式下，同组姿态的 360 次求解全部有效，完整处理耗时中位数 **8.95 ms**、p95 **12.18 ms**，纯求解中位数 **7.94 ms**；相对最初安静模式的完整处理耗时改善约 20%。当前模式下最后一次原生检查为 119.25 Hz、1,192 帧中 964 帧识别到左手，零跳号、过期、设备丢失或 SDK 错误。HTTP 左手输入也得到真实有效求解；手未放入视野期间，后台无手状态更新约 119 Hz，不能把这个状态更新频率当作带手求解频率。

上述离线对照只反映计算耗时，不是新的实时采集帧率或动作识别准确率。没有修改求解迭代数、标定或机械手参数。

原方案记录在 Windows `%LOCALAPPDATA%\hand-ultraleap\power-plan-before-20261005-1117.json`。按需手动恢复：

```powershell
powercfg /setactive 16edbccd-dee9-4ec4-ace5-2f0b5f2a8975
```

另以官方 `LeapSetDeviceHints` 测试 `high_hand_fidelity` 和 `fast_hand_motion`。必须先订阅设备事件再设置提示；API 返回成功不保证服务采纳。当前 Linux 6.2 服务日志只接受 `fast_hand_motion`，且明确记录所需模型已运行、无需切换，相机仍为约 120 Hz。因此没有把这一测试作为持久的“提速开关”，也没有依据 USB 描述符中的其他视频模式宣称已经得到 240 Hz 手部骨架。参见[官方性能模式](https://docs.ultraleap.com/hand-tracking/Hyperion/performancemodes.html)。

本轮稳定结果不能反推此前故障已经根治。此前证据定位到原生视频流中断、服务重置设备及 Windows USB 枚举失败；仍不足以唯一判定线材、端口供电、设备固件或 USB/IP 驱动。官方 Linux 支持范围仍是 Ubuntu 22.04；目前是 WSL 上的 Ubuntu 24.04。只有在原生 Ubuntu/Windows 与同一设备、线材、端口的对照中复现，才可能进一步排除某一层。

### 可重复的原生采集诊断

新增 `tools/leapmotion_diagnostic.py`，独立读取真实 LeapC，无需浏览器，不连接机械手：

```bash
cd /home/onkiall/projects/hand
~/.venvs/hand-track/bin/python tools/leapmotion_diagnostic.py \
  --duration 120 --require-hand \
  --output ~/.cache/hand-ultraleap/leap-live-$(date +%Y%m%d-%H%M%S).jsonl

# 验证停采、空闲、重新开始；此时关闭其他正在采集的 Leap 客户端
~/.venvs/hand-track/bin/python tools/leapmotion_diagnostic.py \
  --duration 20 --cycles 3 --idle 45 \
  --output ~/.cache/hand-ultraleap/leap-reconnect-$(date +%Y%m%d-%H%M%S).jsonl
```

每秒记录真实帧数、左右手帧数、接收频率、原始帧龄期、最大间隔、丢失事件及帧序号跳跃。没有帧、连接/设备错误、倒序、350 ms 以上卡顿或过期帧会返回失败；`--require-hand` 还要求每轮至少检测到一帧手。未传此选项时，“通过”只代表数据流健康，不保证手始终可见，更不代表动作准确率达标。输出文件不能覆盖已有记录。

基线与应用测量保存在 `~/.cache/hand-ultraleap/usb3-*.jsonl`。工作台 HTTP 日志里的 `Request timed out: ... read operation timed out` 来自 15 秒请求读取超时，单独出现不能证明 Leap 视频断流；应同时核对服务日志、真实帧序号及 USB 状态。

## 当前 WSL 的安装记录与管理

- 安装包来自 [Ultraleap 官方 Linux 6.2.0 下载](https://s3.eu-west-1.amazonaws.com/downloads.ultraleap.com/software/tracking-software/6.2.0/tracking-software-linux-x64-6.2.0.deb)。本次 APT 仓库 DNS 不可用，使用官方独立安装包完成安装，没有添加失效的 APT 源。
- 安装包及完整协议保存在 `~/.cache/hand-ultraleap/6.2.0/`；安装包 SHA256 为 `8911eee255138b8355f48066fe4d69d32b57cabd9de9722121ac631fb96d7fa5`。用户明确接受 SDK 协议及 EULA 后已执行接受步骤；重启服务后配置中的 `hyperion_eula_accepted` 仍为 `true`。
- 官方 Python binding 源码使用提交 `2341c6c3db5dc08236bb52890d8c43a224139694`，编译并安装到 `/home/onkiall/.venvs/hand-track`；源码和 wheel 保存在 `~/.cache/hand-ultraleap/`。普通解释器导入、原生服务连接、采集线程关闭和 `pip check` 均通过。
- 虚拟环境的 `site-packages/ultraleap_sdk_location.pth` 在未显式设置时，将 `LEAPSDK_INSTALL_LOCATION` 指向该环境实际安装的 binding 包根目录；现有启动器无需手动导出该变量。系统 `/etc/ld.so.conf.d/ultraleap.conf` 登记官方动态库目录 `/usr/lib/ultraleap-hand-tracking-service`，并已运行 `ldconfig`。

当前 WSL 没有启用 systemd，使用独立环境脚本管理服务。在 WSL 内执行：

```bash
/usr/local/sbin/hand-ultraleap-service status
leapctl devices
sudo /usr/local/sbin/hand-ultraleap-service restart
# 按需停止或启动：
sudo /usr/local/sbin/hand-ultraleap-service stop
sudo /usr/local/sbin/hand-ultraleap-service start
```

日志位于 `/var/log/ultraleap/service-console.log`。`/etc/wsl.conf` 的启动命令已改为 `/usr/local/sbin/hand-wsl-init`：先运行原有 `hand-usb-init`，再启动 Ultraleap。原配置备份为 `/etc/wsl.conf.before-ultraleap-20261004`。已验证启动脚本可重复执行且不会产生重复服务，并验证服务重启；未重启整个 WSL。迁移至启用 systemd 的原生 Ubuntu 后，使用安装包自带的 `ultraleap-hand-tracking-service.service`，无需这些 WSL 启动脚本。

以下为在其他环境重新安装的通用指引；当前开发环境无需重复执行。

## 安装 Linux 服务与可选 Python binding

以下 Bash 命令在目标 Linux 环境执行。Windows 下先打开项目对应的 WSL 终端：

```powershell
wsl.exe -d Hand-Ubuntu-24.04 --cd /home/onkiall/projects/hand
```

按照官方 Linux 文档添加 APT 源并安装服务；完整元包同时包含控制面板等组件：

```bash
wget -qO - https://repo.ultraleap.com/keys/apt/gpg | gpg --dearmor | sudo tee /etc/apt/trusted.gpg.d/ultraleap.gpg
echo 'deb [arch=amd64] https://repo.ultraleap.com/apt stable main' | sudo tee /etc/apt/sources.list.d/ultraleap.list
sudo apt update
sudo apt install ultraleap-hand-tracking
```

安装时的 EULA 由操作者阅读并决定是否接受；本项目不自动代为接受。若安装未提示或此前拒绝，官方提供 `leapctl eula` 重新打开相关流程。先用 `leapctl --help` 与 `ultraleap-hand-tracking-control-panel` 检查服务、设备和骨架，再排查项目应用。没有 systemd 的 WSL 不能直接假定安装后的服务已自动启动；当前环境的启动配置与验证结果见上节。启用 systemd 的原生 Ubuntu 使用软件包自带服务单元。

Python 必须安装到**启动本项目服务的同一个虚拟环境**。这里以现有 `~/.venvs/hand-track` 为例。使用[官方仓库](https://github.com/ultraleap/leapc-python-bindings)，不要用名称相似的任意 PyPI `leap` 包代替。官方 README 列出的 Linux x64 预编译支持为 Python 3.8；其他 Python 版本可按其流程自行编译，需要匹配该 Python 的开发头文件、C 编译器以及 LeapC 头文件/动态库。

```bash
source ~/.venvs/hand-track/bin/activate
git clone https://github.com/ultraleap/leapc-python-bindings.git ~/leapc-python-bindings
cd ~/leapc-python-bindings
python -m pip install build cffi setuptools
```

已有该检出目录时直接进入，不要重复克隆覆盖。构建前确认系统包实际安装位置：

```bash
dpkg -L ultraleap-hand-tracking-service
ls -l /usr/include/LeapC.h /usr/lib/x86_64-linux-gnu/libLeapC.so
```

官方 Linux 文档描述了 `/usr/share/doc/ultraleap-hand-tracking-service` 中的 SDK 以及系统头文件/库目录中的链接；不同发行包布局可能不同。官方 binding 的[构建脚本](https://github.com/ultraleap/leapc-python-bindings/blob/main/leapc-cffi/setup.py)支持以下两个变量，值均为**完整文件路径**，不是目录。只有上面的检查确认路径存在后才执行；否则按 `dpkg -L` 的实际结果替换。

```bash
export LEAPC_HEADER_OVERRIDE=/usr/include/LeapC.h
export LEAPC_LIB_OVERRIDE=/usr/lib/x86_64-linux-gnu/libLeapC.so
python -m build leapc-cffi
python -m pip install leapc-cffi/dist/leapc_cffi-0.0.1.tar.gz
python -m pip install -e leapc-python-api
```

变量名是 `LEAPC_HEADER_OVERRIDE` / `LEAPC_LIB_OVERRIDE`，不是 `LEAPC_HEADER_DIR`。`0.0.1` 是当前官方构建脚本的 binding 包版本，不是 Tracking Service 版本；若上游版本变化，以实际构建产物为准。头文件应来自当前安装的 Tracking Service，避免混用其他版本。

官方[运行时导入代码](https://github.com/ultraleap/leapc-python-bindings/blob/main/leapc-python-api/src/leap/__init__.py)还会检查 `LEAPSDK_INSTALL_LOCATION/leapc_cffi`，Linux 默认根目录是 `/usr/lib/ultraleap-hand-tracking-service`。这是运行时查找位置，与上述两个构建文件路径不同。若使用自己编译并装入虚拟环境的包，可以指向该环境实际存在的包根目录，然后验证：

```bash
export LEAPSDK_INSTALL_LOCATION="$(python -c 'import sysconfig; print(sysconfig.get_paths()["purelib"])')"
test -d "$LEAPSDK_INSTALL_LOCATION/leapc_cffi"
python -c 'import leap; print(leap.__file__)'
python examples/tracking_event_example.py
```

此设置使用已安装的 `leapc_cffi`，不创建假 SDK 目录或修改系统软链接。编译失败时先检查头文件、库、Python 开发头文件和 C 编译器；导入成功但示例没有手数据时，继续检查服务/设备/手别。运行示例会真实连接追踪服务；结束时按 Ctrl+C。

启动工作台也必须带上同一运行时环境变量。可以在上述已激活的终端中运行：

```bash
cd /home/onkiall/projects/hand
python -m online_control.online_motion_track.main --no-open
```

若用另一个终端或启动器，须确保其使用同一 `LEAPSDK_INSTALL_LOCATION`，不能假设当前终端的 `export` 会自动传到已有服务进程。当前 `hand-track` 虚拟环境已经通过上文记录的 `.pth` 完成配置，其他新环境需自行配置。该路径依赖具体虚拟环境，不宜硬编码进业务逻辑。

## WSL 的 USB 直通只属于环境配置

原生 Ubuntu 直接连接设备，跳过本节。当前 WSL 可按 [Microsoft USB/IP 指引](https://learn.microsoft.com/en-us/windows/wsl/connect-usb)由操作者使用 `usbipd-win`；以下 `LEAP_BUSID` 必须替换为 `usbipd list` 中实际 LMC2 的 BUSID，不能照抄其他摄像头或机械手的编号。

```powershell
usbipd list
# 管理员 PowerShell：将 LEAP_BUSID 替换为实际设备编号
usbipd bind --busid LEAP_BUSID
# 保持 Hand-Ubuntu-24.04 已启动，再执行
usbipd attach --wsl --busid LEAP_BUSID
```

进入目标发行版后运行 `lsusb` 和 `lsusb -t` 检查设备与 USB 拓扑，并在 Ultraleap 控制面板确认真实追踪。设备附加到 WSL 后，Windows 应用不能同时使用它。需要归还时由操作者执行 `usbipd detach --busid LEAP_BUSID`。

本次服务安装未执行 USB 设备绑定或直通，保留了原有 USB 初始化脚本；安装包已安装其设备规则。USB 直通、带宽及真实设备追踪仍需独立验证；仅看到 USB 设备不代表已经获得有效骨架。

## 工作台与标定页操作

1. 在手势映射工作台选择目标模型、采集手别，以及关键点来源 **Leap Motion 2**，开始会话。将所选手置于桌面设备上方的有效跟踪区。
2. 页面显示有效骨架后检查手别、手指顺序、腕点、屈伸方向及模型姿态。该路径不申请浏览器摄像头权限，也不经过 MediaPipe 推理。
3. 动态标定页 `/calibration` 同样可选 Leap Motion 2，点击“接入实时输入”。输入按现有标定页规则对齐到模型；冻结后可继续编辑。该页面用于标定求解预览，不下发实机。
4. 切换输入源前先停止/冻结；关闭最后一个 Leap 输入会话会停止该应用的采集连接。Tracking Service 是独立系统组件。

API 沿用会话制：`/api/session` 中选择 `input_source: "leapmotion"`，随后调用 `/api/leapmotion/create`。`/api/leapmotion/latest` 读取观察，`/api/leapmotion/frame` 读取/处理服务端最新帧；关闭使用已有 `/api/stop`。这些接口接受本项目会话标识，不接受浏览器伪造的 Leap 骨架作为采集来源。A3.2 沿用服务端后台求解，其他会话沿用各自已有求解路径。

## 21 点、单位与失效处理

映射保持现有 MediaPipe 21 点编号：

| 点编号 | 解剖位置 | LeapC 字段 |
|---|---|---|
| 0 | 腕 | `arm.next_joint` |
| 1–4 | 拇指 CMC、MCP、IP、TIP | 拇指 `proximal.prev_joint`、`proximal.next_joint`、`intermediate.next_joint`、`distal.next_joint` |
| 5–8 | 食指 MCP、PIP、DIP、TIP | 食指同上四个端点 |
| 9–12 | 中指 MCP、PIP、DIP、TIP | 中指同上四个端点 |
| 13–16 | 无名指 MCP、PIP、DIP、TIP | 无名指同上四个端点 |
| 17–20 | 小指 MCP、PIP、DIP、TIP | 小指同上四个端点 |

映射依据 [LeapC 骨骼定义](https://docs.ultraleap.com/api-reference/tracking-api/group/group___structs.html)：其拇指 `metacarpal` 为零长度占位骨，并不表示拇指真的没有掌骨。因此不能从五指都取五个端点后，误把拇指重复点作为额外有效关节。

根据[官方坐标说明](https://docs.ultraleap.com/api-reference/tracking-api/leapc-guide/leap-concepts.html)，Leap 设备坐标为右手系，x 沿相机基线、y 向上、z 朝用户，原始单位是毫米。适配器减去腕点并除以 1000，生成米制 21×3 坐标；保留原坐标轴手性，由既有重定向/标定处理人体与模型的对齐。兼容字段 `normalized` 不代表图像坐标，元数据明确标记 `normalized_is_image: false`。

采集帧复制后缓存，重复/倒序帧不会刷新有效期。龄期计入 SDK 当前时间与原始帧时间的差值，以及进入本项目后的等待；达到 350 ms、所选手丢失、服务断连或数据无效时不再作为有效姿态输入。标定页冻结/丢手时可保留最后显示用于编辑；实机相关路径仍遵循既有型号的失手处理。界面龄期不是已经测量出的端到端控制延迟。

Leap 输入在后台与同步求解路径中，均会在提交控制目标前再次检查来源标识和原始观察龄期。求解期间断连、换手或观察过期时，旧结果不会作为新的控制目标提交。

`raw_leapmotion` 保存设备毫米坐标、腕/前臂骨端点、五指各骨端点、手 ID、流 ID、帧序号和 SDK 微秒时间戳，便于之后重建不同点集实验。SDK 时间戳不是 Unix 时间。`confidence` 按 SDK 原值保留，本机 6.2 实测并非固定 1.0；它不是 21 个关节点各自的置信度，不能据此声称每点可靠或已通过质量检测。

## 25 点会不会更好

**不能只凭点数判断。** LeapC 提供骨、关节端点、手掌和前臂等结构，不是一个固定“原生 25 点数组”。如果这里的 25 点指 [WebXR 手部骨架](https://www.w3.org/TR/webxr-hand-input-1/#skeleton-joints)，它比当前 21 点定义多出食指、中指、无名指、小指的掌骨基部四点。

当前项目主要做 **21 点到机械手关节的重定向求解**，并非已经训练好的手势类别分类器。改成 25 点不仅是扩大数组，还要补齐机器人对应点的标定、求解约束、记录格式以及其他输入源的语义；相机端不能凭空获得四个可靠测量点。

工程上，额外的可靠掌骨点可能有助于区分掌弓、掌形或细微动作；重复点、冗余点以及遮挡时的模型估计不自动增加观测信息。建议继续把 21 点作为四种来源的公共接口，使用保留的原始 Leap 骨架做 21/25 点对照。相同动作、被试、采集条件和算法预算下，重定向应比较指尖/对指误差、抖动、丢手与恢复、求解耗时和端到端延迟；若以后训练手势分类器，再比较跨人测试集的 F1、漏检和误触发。未完成这些测量前，不能承诺 25 点识别更好。


### 管理员 USB 抓包实测（2026-10-05 12:23–12:33）

用户授权管理员操作后完成十分钟 usbipd Trace / USBMMAPPED 抓包，目录为 E:/WSL/leap-usb-trace-20261005-122335。原始包、解析结果 analysis.json、摘要 summary.json、Tracking Service 日志、内核日志和九分钟原始追踪统计 tracking-probe.jsonl 已保存在同一目录。

视频端点累计成功完成 1,218,197 次请求，实际载荷约 37.58 GB，均速约 62.54 MB/s；抓包期间主动收尾前没有非零视频完成状态。仅结束抓包、主动解除 USB 直通时出现 65 次 -32，不能把这些主动断开状态当成自发故障。九分钟原始采集 64,236 帧，无设备丢失、SDK 错误、倒序或过期帧，最大帧间隔 48.747 ms。抓包末尾因停止调试服务有不完整最后一个块，解析器明确跳过该尾块。

用户报告卡住的 12:25–12:26，原始帧仍约 119 Hz，页面选择左手而 SDK 几乎只识别到右手；用户确认实际放的是左手，因此手别误判原因尚不能认定是用户选择错误。红外画面还显示前景高亮遮挡、手较小或位于边缘。方向 normal/inverted 对比期间均收到视频，但没有识别手，保留原 inverted 配置；不能宣称旋转设置已修复本次识别。

切换 Windows usbipd 服务及收尾时复现了 USB/IP stub 设备 Code 21（移除挂起），目标设备在 Windows 列表消失。针对同一序列号执行 pnputil /restart-device 后恢复设备，再 attach 成功；未重启其他 USB 设备。抓包脚本已补充移除挂起的定向恢复，并在验证 Attached 后才报告正常结束。该恢复路径与原 11:42 自发视频丢失的最初原因仍不能画等号。

红外诊断运行在 http://127.0.0.1:8767/；当前可复用脚本为 tools/leapmotion_ir_preview.py，已替换最初缓存目录中的临时预览。它属于独立本机诊断，不是默认工作台启动流程；关闭后不影响通常的 21 点输入接口。

后续按实时红外画面调整位置，保持原 fixed-inverted 配置，已正确识别真实左手：30 秒原始采集 3,580 帧全部为左手，无设备丢失或 SDK 错误；独立 A3.2 左手工作台 25 秒共 697 次读取，其中 696 次有完整左手 21 点并返回有效跟踪求解结果。手别未做强制改标，机械手未连接。该结果证明本次摆位后的采集/重定向恢复；原始约 119 Hz 不等于机械手下发频率或端到端延迟，也不能据此宣称原 11:42 自发 USB 故障已根治。


### 红外画面叠加 21 点

在已安装 Leap SDK 的 Python 环境中运行：

```bash
cd /home/onkiall/projects/hand
/home/onkiall/.venvs/hand-track/bin/python tools/leapmotion_ir_preview.py --port 8767
```

打开 http://127.0.0.1:8767/。两个相机画面显示 0–20 点编号与骨架连线；4 为橙色拇指尖，8 为黄色食指尖，虚线连接两点。页面提供设备毫米坐标表、4–8 三维距离、SDK pinch_distance 和 pinch_strength，以及手别选择、编号/骨架开关和暂停按钮。暂停只冻结诊断显示。

该工具只连接 Ultraleap 服务，不连接机械手，不改变关键点、方向或追踪模式。当前投影验证条件为 Desktop / fixed-inverted，未验证 HMD、ScreenTop 或其他姿态模式。投影使用各相机外参和 LeapRectilinearToPixelByIndex 畸变映射；Desktop 追踪坐标与原始红外画面的横轴转换在相机基线变换之前完成，仅影响显示坐标。

图像与 TrackingEvent 按 info.frame_id 精确配对。不存在对应帧时只显示图像，不沿用上一帧骨架；图像超过 350 ms 则变暗、显示过期提示并清除关键点。图像复制与 PNG 编码限速，默认预览上限 20 FPS（可选 --fps 1..30），不更改服务追踪帧率。页面追踪 FPS 使用最近最多 240 帧计算，不代表预览刷新率或端到端延迟。

2026-10-05 本机验证：USB 3 采集恢复后约 119 FPS，两个相机同帧叠加可用；投影方向、21 点编号、坐标与距离一致性、暂停恢复已检查。此前停帧时服务重复报告 gain changed / reopened；重新接通并重启服务后恢复，自发故障根因尚未证实。

拇指测试共 30 秒、3578 帧，全部识别为左手；用户确认最后 10 秒实际对指接触，但该阶段原始 4–8 距离中位数约 101 mm、pinch_strength 最大约 0.06，表明那段追踪有明显偏差。后续恢复采集时也观察到 pinch_strength=1、4–8 距离约 20.4 mm、SDK pinch_distance 约 0.8 mm 的帧；这不是普遍接触阈值或完成精度标定。不能强行令 4=8 来掩盖误差。

官方字段定义见 [LEAP_HAND](https://docs.ultraleap.com/api-reference/tracking-api/struct/struct_l_e_a_p___h_a_n_d.html)：pinch_distance 是拇指与食指之间的距离，pinch_strength 是拇指与任意其他手指捏合的估计；它单独不能确定是食指。
