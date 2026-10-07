# TsenseI V2 / A3.0 / V3.2 网页配置、node0 部署与内网访问手册

## Quest 3 WebXR 输入

工作台「关键点来源」可选择 Quest 3，启动后生成配对链接，在 Quest Browser 的
`/quest.html` 进入 XR。25 关节转换复用 Qt，重定向、碰撞及设备控制沿用当前链路。
配对 API 与原 HTTPS 站点共用入口；Windows 本机可使用 `--host`、`--tls-cert`、
`--tls-key` 开启局域网 HTTPS。完整操作和验证范围见
[在线 Quest 接入说明](../../docs/online_webxr_quest3.md)（发布包内为 `docs/online_webxr_quest3.md`）。

## 本地更新：2026-09-23 完整数据捕获与分段导出

有效 retarget 求解结果独立于 `tracking` 保存；后续碰撞或控制拒绝使用独立事件记录。队列替换、过期、取消和请求失败均写入 `retarget_skipped`，诊断状态中增加完整记录器状态。浏览器完整控制记录取消 256 MiB 总量停录限制，改为最多 512 MiB 分段导出；结束后使用“记录分段 / 下载所选分段”逐段下载，每段可独立在查看器打开，时间沿用整次会话。诊断日志仍是低频抽样，不用于逐帧对账。

存储和 JSON 工作仍位于 Worker；磁盘配额不足或存储积压时封存并提示，不阻塞电机下发。完整说明和测试见 [捕获检查记录](../../docs/issues/20260923_capture_audit.md)。本次已做本地浏览器验证，未部署远端或连接真机；使用更新后的前端并刷新页面后生效。

## 本地更新：2026-09-23 抓帧调度与日志线程

最新图像若已在推理期间解码，立即处理而不再多等一个视频周期；角度记录使用独立模块 Worker 序列化、存储和导出，保留原 JSONL 格式、源事件时间和 seq。部署时务必一同带上 `static/worker-recorder.js` 与 `static/recording.worker.js`；现有打包脚本已自动包含 static 目录。NumPy 正运动学改为按树深度批量计算，运动约束保持原值。性能对比、长尾恢复测试和 Ruckig 速度说明见 [修复记录](../../docs/issues/20260923_a30_latency_fix.md)。需重启计算服务并刷新页面；本次仅本地模拟验证。

## 本地更新：2026-09-22 A3.0 按手指保持与轨迹自动恢复

A3.0 正常碰撞保护改为 `collision_limited`：保持碰撞相关手指的最后安全目标，其余手指继续运动，完整 20 轴报文继续下发；手势变化后重新检查并自动恢复。位置极值风险在发布前重新规划，浏览器使用服务器返回的实际 `motion_limits` 和服务器采样时间检查跳变。过期/可恢复异常样本丢弃后，从最后成功下发目标重新规划，不取消开启状态。记录存储失败仅停止记录；旧版 256 MB 总量限制已由上节分段导出取代。USB/电机明确故障、持续计算或链路超时仍停止。

详细定义、日志证据、参数同步原因、保护边界和验证见 [连续控制修复记录](../../docs/issues/20260922_a30_continuous_protection.md)。需同时更新前后端，重启服务并刷新页面。本地模拟验证完成，未连接实机、未部署 node0。下节记录为此前实现历史，当前碰撞行为以本节为准。

## 历史更新：2026-09-22 A3.0 碰撞暂停自动恢复

A3.0 的 Ruckig 采样碰撞现在进入 `collision_paused`：拒绝危险采样，规划器停留在最后一个通过检查的姿态，清空旧下发帧并重置轨迹速度/加速度；浏览器保持已开启状态，暂停 USB 位置写入。新手势到来后，从该安全姿态重新规划，只有通过限位、Ruckig 和采样碰撞检查的帧才能恢复下发，无需再次点击开启。

暂停期间 GTR1 流发送 `flags=4` 的 108 字节心跳，不含 USB 报文；正常位置帧仍为 `flags=1`，硬故障为 `flags=2`。暂停仅豁免“没有位置写入”的看门狗，数据流超时/过期、串口故障和外接电机故障仍会关闭下发。手动关闭、急停或切换关节电源后，新手势不会自动重新开启。服务器的采样超时、轨迹越界等硬故障也不能被下一帧手势清除，需重新开启并用反馈重置。

界面显示“碰撞保护暂停，手势安全后自动恢复”，控制日志和诊断日志记录 `control_pause` / `control_resume`。本次修复来自写入 4080 帧后停止的日志，未放宽碰撞阈值。前后端需一起更新，重启计算服务并刷新页面后生效；本地修改不代表已部署 node0。

验证：30 项 Python 控制测试（含日志第 685 帧姿态的真实 Ruckig / 碰撞几何回归）、52 项前端控制/串口测试通过；Chrome 完整网页流程在模拟相机和 USB 下验证了持续暂停、自动恢复、界面提示及两类日志事件。尚未连接实机验证。

## 本地更新：2026-09-22 A3.0 输入中断保持

A3.0 取消有效手势启动门槛和连续三帧确认。开始识别后，即使画面里没有手也能开启实时下发；首次串口连接仍需收到至少一包可解析的 444/484 字节手部遥测，开启时使用缓存反馈建立并截断起始姿态，不再要求该包在最近 500 ms 内更新。

运行中丢手、检测到其他手或手势超过 400 ms 未更新，Ruckig 继续平滑到达最后一个有效目标并保持下发；下一帧有效手势自动恢复跟随。遥测超过 500 ms 未更新仅提示并记录，不关闭实时下发。首次无手启动保持反馈姿态，不主动回零。已识别的故障不会被缺手帧清除。

保留串口断开/写入超时、计算轨迹流中断/过期、下位机重启、左右手配置不符、明确外接电机故障、关节限位、Ruckig 和碰撞保护。找零仍需要新鲜遥测来判断完成状态。此策略仅作用于 A3.0 手势网页；V2 和动作播放不变。下文历史发布中的 A3.0 手势/遥测超时停发描述由本节替代。本次为本地代码更新，尚未部署到 node0。

验证：38 项前端控制/串口测试、25 项 Python 控制测试，以及模拟相机与 USB 的完整网页流程通过（无手启动、过期遥测启动、丢手与遥测同时中断、自动恢复、本机记录）；未执行真机操作。

## 最近发布：2026-09-21 本地诊断日志

已通过 `deploy_node0.cmd --no-pause --target houjue@192.168.88.229` 发布 `20260921-152119`，上一版 `20260921-151013` 保留。诊断日志独立于角度记录自动启用，支持本机浏览器缓存/历史恢复、JSONL 下载和用户选择目录后每秒写入本机文件。已验证角度记录关闭时记录连接失败与堆栈、目录追加、刷新恢复、缓存失败降级；42 项串口/控制测试及含截断反馈的浏览器下发流程通过，日志包含下发动作、反馈、API 和停发原因。三个模型左右手与新旧前端文件一致性检查通过，未操作真机。

### 上一版本：A3.0 实际反馈范围截断

已使用 `deploy_node0.cmd --no-pause --target houjue@192.168.88.229` 发布 `20260921-151013`；`current` 指向该版本，上一个版本 `20260921-143243` 保留。A3.0 实际角度/行程反馈按模型范围截断，规划起点与启动保持位置使用同一结果，找零前保持帧也应用范围截断。42 项前端测试、23 项 Python 测试及负角度反馈的浏览器模拟流程通过；真实记录中的小指 DIP −0.1604° 能截断到 0° 并通过反馈姿态初始化。三个模型左右手接口、前端文件一致性和服务健康检查通过。更新后结束旧会话并 Ctrl+F5，再重新连接串口；未执行实机下发。

### 上一版本：遥测对齐与使能修复

已通过根目录 `deploy_node0.cmd --no-pause --target houjue@192.168.88.229` 发布 `20260921-143243`，node0 的 `current` 指向 `/home/houjue/tsensei-hand-web/releases/20260921-143243`；上一个版本 `20260921-134353` 保留。本次按 `A3_USB_Downlink.pyw` 修正外接电机反馈标识、补齐 444/484 字节手部及板载遥测，移除切换使能时等待遥测的要求，并显示实时下发按钮的具体禁用原因。40 项串口/控制测试、34 项 Python 反馈/下发测试及本地 444 字节、484 字节部分反馈浏览器流程通过；部署后的三个模型左右手接口、前端文件和 HTTPS 健康检查通过。原 CA 证书保持不变；服务器端 USB/录制仍禁用，串口与记录由访问者浏览器处理。本次未执行实机下发。若 `node0` 名称解析失败，使用上述 `--target` 内网 IP 参数。

更新时间：2026-09-18。本文合并原网页版说明、通用部署说明和 node0 内网说明，是网页版本唯一维护文档。按本地配置、打包上传、服务器运行、HTTPS、其他电脑访问、更新维护的顺序，记录已完成部署及可复现命令。

发布后已在 `https://192.168.88.229/` 使用模拟相机与模拟 Web Serial 验证：无遥测切换电源、484 字节反馈、不找零开启下发、反馈姿态重置、108 字节流式下发、丢手停发、板载温度/电压显示、本机 JSONL 导出和数据查看器均通过。模拟测试不连接真实设备。

2026-09-20 本地新增 A3.0：精简包与完整包均包含 A3.0 左右手模型、标定和专用接入代码。转换为 **9 路直线行程 + 11 路旋转角度**（四指 DIP、拇指 PIP 直出角度）。`--full` 包支持碰撞停止/Mink、Ruckig、浏览器本机串口、18 路自研电机找零和 H7 批量下发；两路外接电机使用实际反馈起步，源协议无独立找零/断电命令。A3.0 使用独立 108 字节位置帧、484 字节遥测，不使用 V2 协议。操作与限制见 [A3.0 接入与标定](../../docs/hand_v3_0_setup.md#浏览器连接-h7)。本次以模拟 H7 验证，此说明不代表 node0 已更新或已经实机验收。

## 关节电源控制与免找零下发（2026-09-21）

### 本地诊断日志

打开页面即自动开启诊断日志，不依赖“记录角度”、相机或串口连接成功。展开“本地诊断日志”，可下载当前/历史 `hand_diagnostic_*.jsonl`；缓存位于访问者浏览器独立的 IndexedDB，刷新后通过“读取历史日志”恢复。日志不会上传 node0。选择“日志目录 · 自动写入”后，每秒把 JSONL 追加并关闭写入到所选本机目录，便于直接读取文件；浏览器要求由用户选择目录，每次刷新需重新选择。可使用项目下 `logs/diagnostics` 目录。选择目录时带入本页内存保留的最近日志（最多 2000 条 / 4 MiB），更早记录仍从浏览器历史导出。

日志含带时间戳的按钮/设置操作、下发前置条件、控制动作开始/完成/失败、错误堆栈、API 错误及耗时、反馈截断详情、串口连接/断开、停发原因及每秒状态快照。高频遥测、关键点、规划样本和成功运动写入最多按每类 1 Hz 采样；操作失败和关键控制事件及时记录。原有角度与全链路数据记录保持原格式。浏览器缓存失败时保留有界内存日志供下载，文件写入失败则提示并保留浏览器日志，不改变真机控制状态。意外终止可能丢失尚未提交的最后一小段日志。

“关节电源控制”按五指分组，支持单关节和全部使能/断电。同一页面内按模型和左右手保留选择；刷新恢复默认选择，但不会仅因刷新自动上电。未连接时保存选择，连接时先落实断电选择，开启实时下发时再应用全部选择。
V2 使用原协议的直线 `0x11`（1 使能、2 断电）、旋转 `0x22`；A3.0 按 `A3_USB_Downlink.pyw` 使用 `0x21`，payload 为 `[bus,node,cmd,3]`（cmd=1 使能、2 断电，DRV1/2 全部）。A3.0 18 路自研电机支持电源控制，两路外接拇指电机没有对应指令，因此开关禁用并标注“不支持”，不能伪装成断电。A3.2 尚无真机协议，电源开关禁用。
失能不再把位置槽位写 0。位置报文保留完整布局与原目标，停用电机由电源指令关闭。切换电源会先暂停实时下发并取消排队轨迹，然后直接发送使能/断电指令；不等待遥测，也不额外发送保持位置报文。切换后手动重新开启实时下发，此时才根据反馈/备用姿态重置规划器。找零跳过已选择断电的电机，结果弹窗列出原因。界面“已发指令”只说明 USB 写入完成，不代表固件状态确认。
一键找零不再是下发前提：V2/A3.0 收到有效位置反馈和手势、开启碰撞与 Ruckig 后即可下发；每次开启均用遥测反解姿态重置规划器。原有限位、超时、碰撞和手势丢失保护保留。初始选择、电源指令、每帧目标和遥测均写入本机 JSONL。

A3.0 已取消“18+2 路全部反馈有效”前置条件。实时下发仍需收到 CRC 正确且持续更新的 444 或 484 字节手部遥测载荷（500 ms 内），但单路/多路缺失不再阻止开启或导致停发。444 字节固件只含 18 路自研电机，外接两路自动标记为缺失，不能把不存在的字节当成外接反馈。有有效反馈的轴按反馈起步；缺失轴采用上次成功下发目标，首次采用模型初始姿态，页面显示缺失数量，`feedback_seed.fallback_joints` 记录本次使用备用姿态的关节。这是估计的起点，并非缺失轴实际位置。找零跳过缺失反馈的轴并列入结果，不会把旧完成状态当作成功。在线外接电机明确报告故障、有限值/限位、碰撞、Ruckig 和整包超时检查保留。此放宽仅接入手势网页，`online_motion_play` 继续使用严格反馈模式。

“开启实时下发”下方显示每个禁用原因，以及串口收到的字节数、解析有效包数、接受更新包数、载荷长度和最近更新时间。收到字节但有效包为零时检查型号/协议/CRC；有有效包但更新数停滞时检查 seq/tick；有新鲜遥测但目标未就绪时查看手势或碰撞原因。序号跳过较多包、16 位回绕及下位机重启后能重新接收，重复包不会刷新遥测时间。找零状态和单路反馈缺失不再作为开启前置条件。

A3.0 遥测以根目录 `A3_USB_Downlink.pyw` 为准：设备 3、功能 2 是手部遥测，功能 3 是板载遥测。接受 CRC 正确的扩展载荷并按源程序截取前 444/484 字节。外接 M1/M2 标识为 `05 05` / `09 05`；在线位有效时即使标识不匹配也按源程序解析，只有标识匹配但在线位为零的缓存值仍视为过期，不用于规划起步。外接位置/速度除以 50 换算输出轴，并记录电流、电压、故障、运行/标定状态、模式和电机类型。18 路自研电机按 `<5f2h>` 解码；开环状态下末 4 字节是频率，不解释为重力电流和 MCU 温度。

板载遥测按 `<HhhH>` 解码，NTC 温度单位 0.01℃、VIN 单位 mV，`0x7FFF` / `0xFFFF` 表示无效。界面显示两路温度和输入电压，4 秒未更新标记超时；记录为独立 `board_telemetry` 事件。板载包不会刷新手部遥测时效，也不会替代规划起步所需的位置反馈。现有 `telemetry` 事件和数据查看器字段保持兼容。

A3.0 网页对在线且有限的实际位置反馈采用模型范围截断：旋转电机按关节角度上下限截断，直线电机按模型端点对应的行程范围截断后反解角度。例如小指 DIP 回传 −0.16°、模型下限 0°，规划起点取 0°。开启下发时的第一帧保持指令使用服务器返回的同一份截断后目标，不再复制原始越界值；找零前保持帧也按模型对应的执行器范围截断。原始遥测不覆盖，`feedback_seed.clipped_joints` 保存关节、原始值、采用值、单位及上下限。缺失反馈仍采用备用姿态，通信时效、明确电机故障、碰撞与 Ruckig 检查保留。共享反解函数的默认严格模式保留，只有 A3.0 手势网页明确开启 `clamp=True`。

### Mink 距离参数

`mink_guard.py` 采用 `collision_detection_distance=0.015 m`（15 mm）作为 QP 约束检测范围；普通碰撞对的最小间距为 `0.0005 m`（0.5 mm），拇指 `PIP_link_capsule` 与其他手指 `DIP_link_capsule` 为 `0.00005 m`（0.05 mm）。目标及中间路径满足最小间距时可直接通过，并非进入 15 mm 就停止；违反最小间距时修正/限制姿态。距离是 MJCF 碰撞代理表面的有符号距离，不是可视 STL 网格或真实外壳的间距。路径按最多约 0.025 rad 的关节增量离散检查，不是连续碰撞保证。

## 当前版本：本机 USB 串口与本机记录（2026-09-18）

网页支持 V2 / A3.0 / V3.2 和左右手切换，使用原版 URDF、当前标定。识别运行中也能切换模型：页面先停识别和下发、断开串口、封存本机记录，再加载新模型。切换后需重新开始识别，不会自动开启真机下发。碰撞策略需在结束会话后更改。

**node0 计算，访问者电脑连接下位机。** V2 采用 Chrome / Edge 的 Web Serial，直接访问打开网页的电脑上的 USB 串口；客户端无需 Python、Node 或本地代理。node0 的 Docker 不映射串口，公共模式仍禁止服务器 USB 操作和服务器记录。`recording_allowed:false` / `hardware_allowed:false` 指服务器权限；`browser_serial:true` / `browser_recording:true` 表示浏览器功能已提供。

公司内网入口保持 [https://192.168.88.229/](https://192.168.88.229/)。客户端先按第 2 节信任公司根证书；可信 HTTPS 或本机 localhost 才能使用相机和 Web Serial。使用当前桌面 Chrome / Edge，若公司浏览器策略禁用串口需由 IT 放行。浏览器第一次通过按钮弹出设备选择器；“已授权串口”只显示这个网站已获授权的本机设备。串口不能同时被 Qt 程序或其他标签页占用。

### 使用步骤

1. 将下位机插到**访问网页的电脑**，选择 V2、正确左右手和碰撞策略；按需开启“记录角度”。V3.2 继续提供模型预览和 Mink，尚无真机协议。
2. 点击“连接串口”，在浏览器弹窗选择 USB 设备。使用原 H7 协议，115200 波特、8N1、无流控。连接设置 V2 手型并发送已选择的断电指令，不发送运动姿态。
3. 一键找零可选。V2 按原 14 路直线和 6 路旋转流程执行，A3.0 按 18 路自研电机流程执行；已选择断电的关节跳过。默认 Iq=700，允许 0..5000。找零结果弹窗列出未成功/跳过关节及原因；小指固定旋转自由度和 A3.0 外接两路不参与找零。
4. 点击“开始识别”，取得有效手势后点击“开启实时下发”。需要遥测新鲜、碰撞与 Ruckig 开启，无需完成一键找零。每次开启都从实时反馈姿态重置规划器，先写入反馈位置，再按关节选择使能/断电。找零部分失败不会禁止后续下发，但反馈缺失、碰撞不满足等独立检查仍可能阻断。
5. “关闭实时下发”保留识别。“停止并断电”先关闭下发，再发送直线找零复位、直线驱动关闭、旋转驱动关闭命令；不会突然跳到零姿态。它是软件停止请求，写入 USB 不代表固件已经执行。
6. “停止识别”“结束会话并保存”或切换模型，会停发、断开串口并导出本机 JSONL。正常断开保持最后目标；正在找零时先发停止命令。断开后重新连接、等待有效反馈即可再次开启下发，找零仍可选。

### 控制链和频率

浏览器 MediaPipe GPU → HTTPS 关键点 → node0 的 One Euro / Wuji → TopoRetarget → V2 碰撞规划 → Ruckig → 每次采样碰撞复查 → UI/HW 角度与连杆行程转换 → 浏览器时效/限位/跳变检查 → 本机 Web Serial → H7。

node0 保留真实经过时间驱动的控制线程，目标 500 Hz。浏览器串口只发送收到的新鲜计算样本，目标上限 50 Hz，实际频率受网络和渲染影响，页面分别显示服务器采样频率和本机写入频率。只有一个状态请求和一个姿态写入在途，不累积位置队列，不声称在浏览器实现 500 Hz 硬实时控制。调试预览不会打开 node0 串口。

响应超过 100 ms、控制/写入间隔超过 150 ms、关键点超过 400 ms 或遥测超过 500 ms，以及错误手别、非法数值、关节跳变、页面隐藏，均会关闭实时下发；恢复后需手动重新开启。写入超过 250 ms 中止串口写入流并取消找零资格。丢手不会自动恢复运动。浏览器被强制终止或系统卡死时 JavaScript 无法保证执行停止命令，固件侧仍需保留通信超时保护，软件按钮不等同于硬件急停。

### 本机数据与原查看器

“记录角度”对本地服务和 node0 均可用。数据先写访问者浏览器的 IndexedDB；通过“选择日志根目录”授权 E:\TsenseI_hand_logs 后，结束时自动按 A30 / A32 / V2 子目录保存全部分段（刷新页面后需重新授权）。未授权时结束后下载 `hand_control_*.jsonl`；自动下载被拦截时点击“下载本次 JSONL”。超过单段 512 MiB 时使用 `_part001.jsonl` 等文件名，页面显示总段数并提供逐段下载；自动下载只下载当前段，请下载其余分段后再删除缓存。页面刷新后可通过“本机历史记录”重新下载；按域名/IP、浏览器和用户隔离。每 250 ms、100 条事件或约 1 MiB 待写入数据触发存储，异常强制关闭可能丢失尚未提交的数据。完整控制记录不再按 256 MiB 截断，但仍受浏览器磁盘配额约束；存储积压或失败会封存并提示，控制继续。导出失败时保留当前页面，可恢复尚在内存中的存储块。

兼容原 `hand-control-session/v2`：`session`、`mediapipe_raw`、`mediapipe`、`retarget`、`control_target`、`h7`、`conversion`、`downlink`、`usb_tx`、`telemetry`、`session_end`。额外记录找零事件和 `planner_sample`。实际写入保存 96 字节 H7 帧，遥测保存 CRC 有效的原始 payload 和解码字段。记录服务器计算结果中**浏览器收到的样本**，不冒充服务器每个 500 Hz tick；未连接设备不伪造发送、回传或电流。`sent` 仅表示浏览器串口写入成功，不是固件执行确认。

通过“数据查看器”（`/data-viewer`）选择本机 JSONL 即可看曲线；文件不会上传到 node0。V3.2 同样记录本机原始关键点、retarget 前后角度和碰撞诊断，统一使用 JSONL；旧 Python 客户端的 JSONL/CSV 保持其原格式与分析入口。

### 一键更新现有 node0

在项目根目录运行（已有 SSH 密钥、node0 Docker 和当前 CA）：

Windows 推荐直接双击根目录 `deploy_node0.cmd`，或在 CMD / PowerShell 中运行（不受当前工作目录影响）：

```cmd
.\deploy_node0.cmd
```

这是现有 `online_vision2control_node0.cmd` / `deploy_node0.py` 的简短入口，旧入口仍可用，所有入口共用同一部署流程。自动选择 `PYTHON`、已激活的 Conda、这台电脑现有的 `E:\conda\envs\hand_dex\python.exe`，再尝试 PATH 中的 Python 3.10+。

```powershell
.\deploy_node0.cmd --check-connection  # 只检查 SSH、Docker Compose 和 Docker 服务权限
.\deploy_node0.cmd --prepare-only      # 只准备本地完整包，不上传、不更新 node0
.\deploy_node0.cmd --no-pause          # 执行部署，结束不暂停；供终端/自动调用
```

默认通过现有 SSH 密钥连接 `houjue@node0`，更新 `/home/houjue/tsensei-hand-web`。脚本不存储密码，保留主机密钥验证。没有参数时结束后暂停，方便双击查看结果。部署打包的是当前工作目录里的代码和标定，不要求先 Git 提交。
部署后检查三个模型左右手、A3.0 9+11 转换与 Ruckig，并逐字节核对运行中的逐关节开关等前端文件；检查失败走现有自动回退流程。检查仅使用未使能的预览会话，不连接下位机。

Git Bash / Linux / WSL：

```bash
bash online_vision2control_node0.sh
```

Windows Git Bash 可先执行 `cd /d/Qtworks/hand`。脚本自动寻找 Python 3.10+；需要指定解释器时使用 `PYTHON=/e/conda/envs/hand_dex/python.exe bash online_vision2control_node0.sh`。WSL 使用 WSL 自己的 Python、SSH 密钥和网络配置。默认连接 `houjue@node0`，可用 `--target houjue@实际地址` 替换 SSH 地址（仍更新同一 node0 安装目录）。

Windows 下自动优先使用系统 OpenSSH，上传也固定使用同一个 SSH 程序，避免 Git Bash 的 `HOME` 不同导致找不到已有密钥或主机信任记录。保持主机密钥校验。仅检查连接、不部署可运行：

```bash
bash online_vision2control_node0.sh --check-connection
```

只生成本地部署包、不上传或重启 node0：

```bash
bash online_vision2control_node0.sh --prepare-only
```

PowerShell 也可使用同一个部署流程：

```powershell
conda activate hand_dex
powershell -ExecutionPolicy Bypass -File online_control/online_motion_track/deployment/node0/deploy-node0.ps1 -Python python
```

也可传入 `-Python E:\conda\envs\hand_dex\python.exe`。脚本自动准备前端、下载 Linux CPython 3.12 wheel、构建 `package.py --full` 白名单包、生成 SHA-256、通过 SSH/scp 上传、验证和构建新 release、保留原证书与 Compose 项目，健康检查通过后切换 `current`。构建失败保持旧版本运行，激活失败尝试恢复旧 release。上传不包含用户记录、截图、账号或密钥。首次空服务器安装仍按第 5 节执行。

脚本打包当前工作目录内的代码，包含尚未提交到 Git 的修改。正常部署会重建服务并中断已有网页会话，部署结束后刷新网页重新开始识别。保留旧 release 与原 CA；加速算法验证失败也会进入回滚流程。

完整版额外使用 MuJoCo 3.10.0、Mink 1.3.0、qpsolvers 4.13.0、Ruckig 0.19.4、pyserial 3.5；Windows 本地仍可使用 Ruckig 0.17.3。`verify_runtime.py` 在 Docker 构建期间验证两模型左右手、V2 预绑定查表、A3.2 独立 LUT、计时接口和服务器 USB 禁用，不需要服务器 Qt、摄像头或 GPU。容器启动后，`verify_acceleration.py` 通过实际 HTTP 接口再次核对左右手的 V2 查表输出及 A3.2 `method=lut`，验证通过才切换 `current`。

本次测试还覆盖：混合直线/旋转找零失败的弹窗、继续后续轴、反馈起点、失败后正常下发；断连/遥测超时仍终止流程。结果与逐轴失败原因写入本机 JSONL。

本次测试包含 Python 服务端门禁、JS/Python 逐字节协议比对、780/932 字节遥测对比、浏览器模拟 USB 完整找零/下发/丢手停发/模型切换、本机下载与刷新恢复、原数据查看器读取。未操作真实机械手，实际固件找零、停止命令和运动效果仍需硬件验收。

## 本地试验：Mink 自碰撞处理

2026-09-17 首次本地接入可选 Mink 避碰，2026-09-18 纳入 node0 完整运行包。
在项目根目录、`hand_dex` 环境运行：

```powershell
python -m pip install -r online_control/online_motion_track/requirements-mink.txt
python online_control/online_motion_track/main.py --port 8766 --collision-mode mink
```

打开 http://127.0.0.1:8766/，页面“自碰撞处理”默认选择 `Mink · 避碰`。
点击“开始识别”测试；要对比原始映射，先停止，再选“关闭 · 原始映射”并重新开始。
未安装 Mink 时该选项不可用；原来的最小依赖服务默认仍为 `off`，不会隐式引入 MuJoCo。

- 保留现有 TopoRetarget 和完整 URDF 显示，在输出端用真正的 Mink `CollisionAvoidanceLimit` + DAQP 修正关节目标，并把修正结果反馈到下一帧求解。
- 使用现有左右手 MJCF，共 100 对启用的碰撞代理。普通碰撞对留 0.5 mm；拇指末节与其他手指末节留 0.05 mm，允许接近接触。沿用模型的碰撞掩码和 Mink 相邻刚体排除规则，不代表检测了所有外壳组合。
- 不运行动力学 `mj_step`；目标及其直线路径通过距离检查时直接输出。存在碰撞时最多执行 12 次局部 QP，并做距离复查、回溯和帧间离散采样。求解失败时保持已验证姿态，不输出原始穿透目标。
- 模型下方显示最小间距和最大关节修正角度，鼠标悬停查看原始间距、避碰耗时和最近碰撞对。右侧 Retarget 时间包含避碰处理。
- 网页本机 JSONL 的 `retarget.joints` 保存修正前目标，`control_target.joints` 保存输出，`control_target.self_collision` 保存诊断。原 Python 客户端仍使用原字段和 CSV。

建议测试：无名指与小拇指相向摆动、食指/中指/无名指合拢、拇指逐指对捏、握拳后迅速伸出中指。距离按胶囊体/盒体代理计算，并非原版三角网格的精确距离；几何过粗可能提前阻挡动作，离散采样也不是连续碰撞的严格保证。局部求解可能停在接触边界，需要张开手后重新接近目标。

本机验证：31 项 Mink/网页/采集管线/记录测试通过；已有左手记录 903 帧回放，代理穿透帧由 81 降至 0，最小输出间距约 0.0499 mm；避碰耗时中位数 0.37 ms、P95 10.95 ms。该结果仅针对这段记录及当前碰撞代理，真人相机体验需要实际试用。最小服务器包不含 Mink；node0 现使用 `--full` 包，已经包含这项依赖和 MJCF。

## 1. 已部署结果

| 项目 | 当前配置 |
|---|---|
| SSH | `houjue@node0`，使用已有密钥 |
| 服务器 | Ubuntu 22.04.5 LTS / x86_64，系统 Python 3.10，已有 Docker Compose |
| 运行方式 | Docker 内独立 Python 3.12 + Caddy，不改系统 Python 和已有 Conda |
| 首次访问说明和证书下载 | [http://192.168.88.229/](http://192.168.88.229/) |
| 手势工作台 | [https://192.168.88.229/](https://192.168.88.229/) |
| 域名入口 | `https://node0.lan/`，需要公司 DNS 能解析 |
| 另一内网接口 | `http://10.90.0.153/`、`https://10.90.0.153/` |
| 运行目录 | `/home/houjue/tsensei-hand-web/current` |
| 当前部署 release | 以 `readlink -f /home/houjue/tsensei-hand-web/current` 为准；一键部署最后输出新旧路径 |
| 首次部署 release | `/home/houjue/tsensei-hand-web/releases/20260917-1320` |
| 会话上限 | node0 全局 8 个、每来源 IP 2 个；V2 失联 3 秒 / V3.2 空闲 120 秒或持续 15 分钟后回收 |
| 访客记录与真机控制 | 浏览器本机记录 / V2 Web Serial；服务器 USB 和服务器记录禁用 |

主 IP 和 `node0.lan` 已从开发电脑跨机器验证。`10.90.0.153` 已在服务器验证，但开发电脑所在网段无法直连。不同公司 VLAN 仍需路由和 TCP 80/443 放行，连接公司网络不代表所有网段必然互通。

## 2. 同事首次访问

1. 连接公司内网，用 Chrome / Edge 打开 **HTTP 首次访问说明页**。
2. Windows 下载“证书安装包”，全部解压到同一文件夹，运行 `install-certificate.cmd`，确认信任 **TsenseI node0 LAN** 根证书。只安装到当前 Windows 用户，无需管理员权限；受公司策略限制时由 IT 统一分发。
3. 完全退出并重新打开浏览器，打开 **HTTPS 工作台**，点击“开始识别”，允许使用当前电脑的摄像头。

每台电脑、每个用户首次信任一次，以后直接访问 HTTPS。客户端无需安装 Python、Node、本项目或专门的服务器 GPU 软件。macOS 可在“钥匙串访问”导入并信任根证书，Linux / Firefox 按浏览器证书管理器配置。

Windows ZIP 包含公开根证书、安装脚本和说明；脚本校验证书文件 SHA-256，再调用 `certutil -user -addstore Root`，不包含私钥。当前根证书 SHA-256 指纹：

```text
8A:E1:C7:C5:DF:1C:03:B6:5E:E4:56:26:B2:69:C6:BD:C3:1D:CF:89:E2:2E:DB:9C:E5:15:EB:3E:35:9F:31:D6
```

这是本服务创建的内网 CA，并非公司既有统一 CA。IT 可以统一下发此根证书，或替换为公司已有可信证书。相机需要受信任的 HTTPS，普通 HTTP 只提供说明页；localhost 是开发例外。参见 [getUserMedia 安全上下文要求](https://developer.mozilla.org/en-US/docs/Web/API/MediaDevices/getUserMedia) 和 [Caddy 本地 HTTPS](https://caddyserver.com/docs/automatic-https#local-https)。

## 3. 网页结构、模型与精简依赖

```text
访客摄像头 → 浏览器 MediaPipe Worker（GPU 优先，失败则 CPU 回退）
  → 21 个三维关键点、手别和时序信息，经 HTTPS 发送
  → node0：One Euro 平滑 → V3.2 坐标转换 → retarget
  → 返回 20 个 URDF 关节角
  → 访客浏览器 Three.js / WebGL 驱动原版左右手 URDF
```

图像留在访客电脑，服务器只接收关键点及必要元数据。每个会话有独立滤波器和求解状态。服务器不需要摄像头、GPU、Qt、OpenCV、PyTorch、Pinocchio 或 USB 设备；完整版使用 MuJoCo 做碰撞计算。

| 依赖 | 锁定版本 / 作用 |
|---|---|
| NumPy | 2.4.4，数组计算、解析 FK 和雅可比 |
| SciPy | 1.17.1，数值计算 |
| PyYAML | 6.0.3，读取手型与标定 |
| DAQP | 0.8.7，关节角优化 |
| MediaPipe Tasks Vision | 1.0.1，浏览器检测 |
| Three.js | 0.186.0，浏览器模型渲染 |

基础四依赖位于 `requirements-server.txt`，node0 完整版使用 `requirements-full.txt`；前端版本锁在 `package-lock.json`。JS、WASM、识别模型、STL、URDF 和标定都从本站提供，无外部 CDN。服务器运行包不需要 Node；Node 仅用于开发机准备前端资源和测试。

主界面包含相机、retarget 后机械手及可折叠三维骨架，窄屏上下排列。相机叠加 21 个关节点、0–20 编号及连线，按镜像画面和留黑区域对齐。输入 MediaPipe 的像素不镜像，沿用项目手别约定。停止或页面隐藏时释放相机与 Worker；目标手消失时保持模型姿态、清空骨架。重置视角不改变关节角。

MediaPipe 使用 VIDEO 模式、两手检测、OffscreenCanvas 和模块 Worker；ESM WASM 通过 `FilesetResolver.forVisionTasks(path, true)` 加载。每轮检测和求解完成后再取下一张新帧，最多一帧在途，不堆积请求；网页没有额外关节插值。识别和渲染利用访客 GPU，retarget 在服务器 CPU 上执行。

左右手分别使用 `assets/hands/TsenseI_hand_v3_2/hand_profile.yaml` 配置的原版 URDF 和当前标定，完整 STL 不减面，关节树、固定 RPY、轴和限位与 Python 一致。沿用中指响应修复参数，网页 NumPy 解析后端已与 Pinocchio 对照；桌面默认后端不变。V3.2 可选 Mink 避碰；V2 提供 Qt 碰撞 / Ruckig 和浏览器本机下发。

页面顶部使用用户提供的透明原图 `static/brand/tsensei-logo.png`（4844×1418），按原比例显示。网站小图标 `static/brand/tsensei-favicon.svg` 嵌入同一原图，仅裁出左侧蓝色图形，保留透明背景，不再使用旧版 SVG 重绘标志。许可见 `THIRD_PARTY_NOTICES.txt`。

## 4. 本地准备与文件位置

下列命令在 Windows 项目根目录执行。可用现有 `hand_dex` 环境；新开发环境采用 Python 3.11–3.13。

```powershell
cd D:\Qtworks\hand
conda activate hand_dex
python -m pip install --only-binary=:all: -r online_control/online_motion_track/requirements-server.txt
npm.cmd ci --prefix online_control/online_motion_track
python online_control/online_motion_track/main.py --check
python online_control/online_motion_track/main.py
```

本地地址为 `http://127.0.0.1:8766/`；`--no-open` 禁止自动打开浏览器，`--port 8899` 可换端口。8765 已有其他程序使用，没有复用。确认根目录 `hand_landmarker.task`、左右手模型和标定齐全，`--check` 通过后再打包。

以下文件相对 `online_control/online_motion_track/`：

| 文件 | 职责 |
|---|---|
| `main.py` / `server.py` | HTTP/API、独立会话、帧验证 / 可迁移运行入口 |
| `static/app.js` / `detector.worker.js` | 相机、帧调度、界面 / MediaPipe GPU 与 CPU 检测 |
| `static/overlay.js` | 相机关键点、编号和连线 |
| `static/kinematics.js` / `viewer.js` | 关节变换、STL 渲染和可折叠骨架 |
| `static/index.html` / `style.css` | 排版、窄屏适配及状态提示 |
| `package.py` | 白名单运行包与 SHA-256 清单 |
| `deployment/node0/Dockerfile` | 独立 Python 3.12、完整版依赖、构建时检查 |
| `deployment/node0/verify_runtime.py` | 两模型左右手、Mink、Ruckig 计时和公共模式门禁检查 |
| `deployment/node0/compose.yaml` | node0 服务、会话上限、健康检查和自动启动 |
| `deployment/node0/Caddyfile` | 内网 HTTPS、证书与代理 |
| `deployment/node0/activate.sh` | 构建启动、导出公开证书、生成 Windows 安装包 |
| `deployment/node0/index.html` | 首次访问说明页 |
| `deploy.sh` | 备选公网 systemd 部署，不用于当前 node0 |

## 5. 打包、上传与服务器启动全过程

以下可用于复现或更新。node0 已运行，应使用新 release 标识，不覆盖运行中的目录。`deployment/node0/` 和 Linux wheelhouse 单独作为补充包上传；当前 `package.py` 不会自动包含这两部分。

### 5.1 检查服务器

用现有密钥登录，不将密码写进脚本或文档：

```powershell
ssh -o BatchMode=yes houjue@node0
```

在 node0 检查：

```bash
hostname
cat /etc/os-release
python3 --version
ip -brief address
ss -ltn
docker compose version
docker ps --format '{{.Names}} {{.Image}} {{.Ports}}'
systemctl is-enabled docker
```

首次检查发现系统 Python 3.10 不满足本运行包要求，Docker Compose 已存在，80/443 空闲，8765、8443 等端口有其他服务。因此选择独立 Docker Python 3.12 和 Caddy，没有升级系统 Python、修改原 Conda 或替换既有 HTTPS 服务。Docker 已启用开机启动；其他服务器复现需先安装 Docker Engine/Compose 并配置部署用户权限。

### 5.2 生成运行包、校验值和 Linux wheelhouse

回到 Windows 项目根目录。以下使用同一个 PowerShell 会话保存变量；每次生成新包名，打包程序拒绝覆盖已有包。

```powershell
$releaseTag = Get-Date -Format 'yyyyMMdd-HHmmss'
$bundleName = "tsensei-hand-web-$releaseTag.tar.gz"
$bundlePath = "online_control/online_motion_track/dist/$bundleName"
$extrasPath = "logs/node0-extras-$releaseTag.tar"
python online_control/online_motion_track/package.py --full --output $bundlePath
$bundleHash = (Get-FileHash -LiteralPath $bundlePath -Algorithm SHA256).Hash.ToLowerInvariant()
[IO.File]::WriteAllText("$((Resolve-Path $bundlePath).Path).sha256", "$bundleHash  $bundleName`n", [Text.Encoding]::ASCII)
Write-Output "本次 release 标识：$releaseTag"
```

运行包只含必要程序、前端资源、两个手型的当前模型与标定、控制预览所需模块、部署入口及本文，排除录制数据、个人截图、虚拟环境和 Qt 应用。`manifest.json` 保存每个文件的 SHA-256。

在 Windows 上下载适用于 **Linux x86_64 / CPython 3.12** 的 wheel，不能用 Windows wheel 代替：

```powershell
python -m pip download --only-binary=:all: --platform manylinux_2_27_x86_64 --platform manylinux_2_28_x86_64 --platform manylinux2014_x86_64 --python-version 312 --implementation cp --abi cp312 --dest logs/web_linux_wheels -r online_control/online_motion_track/requirements-full.txt
```

将 Docker 配置与 wheelhouse 打成补充包：

```powershell
@'
from pathlib import Path
import sys, tarfile
source = Path('online_control/online_motion_track/deployment/node0')
with tarfile.open(sys.argv[1], 'w') as archive:
    for path in sorted(source.iterdir()):
        if path.is_file():
            archive.add(path, arcname=path.name)
    for path in sorted(Path('logs/web_linux_wheels').glob('*.whl')):
        archive.add(path, arcname='wheelhouse/' + path.name)
'@ | python - $extrasPath
```

### 5.3 上传文件

```powershell
ssh houjue@node0 'mkdir -p /home/houjue/tsensei-hand-web/incoming'
scp $bundlePath "$bundlePath.sha256" $extrasPath houjue@node0:/home/houjue/tsensei-hand-web/incoming/
```

首次实际部署使用 `tsensei-hand-web.tar.gz`、`node0-deploy-extras.tar`，release 为 `20260917-1320`。原运行包约 63.31 MiB、104 个文件，SHA-256 为：

```text
37d9e94eb9017586f93088c442e23be3bab1f4393a76048fe7472500db3443cb
```

这是首次快照的校验值。以后重新打包，包括只更新本文，也必须使用新计算的值。

### 5.4 校验、解压和固定镜像

登录 node0，在 Bash 中执行。提示时输入步骤 5.2 输出的 release 标识：

```bash
set -e
read -r -p 'Release tag from Windows: ' RELEASE_TAG
[[ "$RELEASE_TAG" =~ ^[0-9]{8}-[0-9]{6}$ ]]
BASE=/home/houjue/tsensei-hand-web
RELEASE_DIR="$BASE/releases/$RELEASE_TAG"
PREVIOUS=$(readlink -f "$BASE/current" 2>/dev/null || true)
cd "$BASE/incoming"
sha256sum -c "tsensei-hand-web-$RELEASE_TAG.tar.gz.sha256"
test ! -e "$RELEASE_DIR"
mkdir -p "$RELEASE_DIR"
cd "$RELEASE_DIR"
tar -xzf "$BASE/incoming/tsensei-hand-web-$RELEASE_TAG.tar.gz" --strip-components=1
python3 -c "import package; print('Verified files:', sum(1 for _ in package.files()))"
tar -xf "$BASE/incoming/node0-extras-$RELEASE_TAG.tar"
mkdir trust
cp index.html trust/index.html
```

系统 Python 3.10 可运行上述标准库校验程序，实际求解使用容器 3.12。首次部署用 `package.py --stage checked-runtime` 做过相同完整性校验，产生的额外暂存副本不是运行依赖。

首次已拉取并验证的官方镜像固定为以下 digest，避免后续无意改变运行环境：

```bash
docker pull python@sha256:782412e85d0f0984994c290652577d4018aff08145c85b262bb63dc0c7522254
docker pull caddy@sha256:5f5c8640aae01df9654968d946d8f1a56c497f1dd5c5cda4cf95ab7c14d58648
cat > .env <<EOF
PYTHON_IMAGE=python@sha256:782412e85d0f0984994c290652577d4018aff08145c85b262bb63dc0c7522254
CADDY_IMAGE=caddy@sha256:5f5c8640aae01df9654968d946d8f1a56c497f1dd5c5cda4cf95ab7c14d58648
RELEASE_TAG=node0-$RELEASE_TAG
EOF
```

镜像首次拉取需要访问镜像仓库，Python 四依赖从上传的 wheelhouse 离线安装。基础镜像升级后应重新验证并记录新 digest。

### 5.5 构建、启动并切换入口

继续在新 release 目录执行：

```bash
bash activate.sh
ln -sfnT "$RELEASE_DIR" "$BASE/current"
```

脚本依次校验 Compose、构建镜像、执行左右手 `server.py --check`、校验 Caddy、启动服务并等待健康检查，然后导出公开根证书、生成指纹与 Windows 安装 ZIP，最后用根证书验证两个内网 IP 的 HTTPS。运行镜像只保留必要依赖和资产，不携带构建阶段 wheelhouse。

成功后才切换 `current` 链接。两容器为 `restart: unless-stopped`，Docker 已配置开机启动；主动 stop 后需要再次 up 才恢复。

### 5.6 部署中解决的 HTTPS 问题

Caddy 用 `tls internal` 为 IP 和主机名自动签发、续期。HTTP 80 保留说明页，HTTPS 443 提供工作台。

早期 Docker 端口映射下，纯 IP URL 的无 SNI 请求报 `tlsv1 alert internal error`，带域名的 SNI 请求能通过。端口映射后的本地目标地址是容器 IP，影响按原内网 IP 选择证书。最终改为 **host 网络 + Caddy 显式绑定两个内网 IP**，保留真实目标地址，两个 IP 的证书链校验均通过。

最终边界：Caddy 只绑定 `192.168.88.229`、`10.90.0.153` 的 TCP 80/443，使用 h1/h2，拒绝非私有来源地址；Python 只监听 `127.0.0.1:8766`。Caddy 覆盖 `X-Real-IP`，Python 校验 Host 和 Origin。应用以非 root、只读文件系统运行，无设备映射，限额 8 CPU / 2 GiB；网关限额 384 MiB。默认程序为 4 会话，node0 Compose 明确覆盖为 8。

这也是当前 node0 使用专用 Docker 配置，而不是直接运行公网域名 `deploy.sh` 的原因。

## 6. 验证与实际结果

node0 中检查：

```bash
cd /home/houjue/tsensei-hand-web/current
docker compose ps
curl --noproxy '*' --fail --cacert trust/tsensei-node0-root.crt https://192.168.88.229/api/health
curl --noproxy '*' --fail --cacert trust/tsensei-node0-root.crt https://10.90.0.153/api/health
```

预期 `ok: true`、`public: true`、`recording_allowed: false`、`hardware_allowed: false`，`models` 包含 `TsenseI_v3_2` 和 `TsenseI_v2`，`mink_available: true`。`public` 表示多人共享、禁用服务器录制和服务器 USB，不代表已开放互联网。浏览器本机记录与串口仍可用，健康状态同时返回 `browser_serial:true`、`browser_recording:true`。另一台电脑按第 2 节信任证书后验证相机、左右手模型、骨架折叠及停止后释放相机。

| 检查 | 首次部署验证结果 |
|---|---|
| 精简环境 | 四依赖、pip 检查、包内 SHA-256、左右手解析求解通过 |
| 本地回归 | 22 项 Python、3 项 Node 测试通过，解析 FK/雅可比与原后端一致 |
| 原记录回放 | 149 帧新旧后端角度最大差约 3.4×10⁻¹³ rad；开发机求解中位数约 32 / 35 ms |
| 跨机器浏览器 | 主 IP、完整左右手模型、GPU 和测试相机、独立会话、禁止录制通过，无外部 CDN |
| 失败处理 | CPU 回退、相机权限拒绝后恢复已通过 |
| node0 动作 | 左右手各 36 帧合成握拳→中指伸展通过，求解中位数约 5.29 / 5.19 ms |
| 恢复 | 本服务重启后健康，既有 CloudXR 容器保持运行 |

测试摄像头和合成动作不等于真人精度验证，求解时间不含采集、浏览器推理与网络耗时。8 是会话上限，不保证 8 路满帧率。自动化浏览器只在测试进程中允许已通过证书链校验的服务端公钥指纹，没有关闭全局 TLS 校验或自动修改开发电脑证书库；日常访问按第 2 节安装证书。产物在开发机 `logs/node0_validation/`，不分发。

开发回归在项目根目录执行：

```powershell
python -m pytest tests/test_numpy_kinematics.py tests/test_web_v3_2.py tests/test_v3_2_latency.py tests/test_v3_2_recording.py -q --basetemp .pytest-runtime/web_v3_2
npm.cmd test --prefix online_control/online_motion_track
npm.cmd run test:browser --prefix online_control/online_motion_track
npm.cmd run test:failure --prefix online_control/online_motion_track
```

浏览器测试需先启动本地服务；默认使用已安装 Chrome，可设置 `CHROME_PATH`、`HAND_PYTHON`、`WEB_URL`。公共运行包验证入口是 `tests/deploy-check.mjs`；浏览器本机串口集成测试使用 `tests/local-serial-browser.mjs`，默认指向 8769 公共测试服务，所有设备用模拟串口，不连接真实机械手。

## 7. 日常维护、更新与回滚

```bash
ssh houjue@node0
cd /home/houjue/tsensei-hand-web/current
docker compose ps
docker compose logs --tail 100 app gateway
docker compose restart
```

停止与恢复只操作本项目：

```bash
docker compose stop
docker compose up -d --wait
```

更新代码、标定、模型或文档时，按第 5 节生成新的运行包、校验值和 release；保持 Compose 项目名 `tsensei-hand-web` 和证书卷名不变。记录旧 `current`，构建及启动检查通过后切换链接。更新可能短暂中断，原识别会话需重新开始。

新版失败时，在同一部署 Shell 用步骤 5.4 保存的 `PREVIOUS` 回滚：

```bash
if [[ -z "$PREVIOUS" || ! -d "$PREVIOUS" ]]; then
    echo 'No previous release to restore.' >&2
    exit 1
fi
cd "$PREVIOUS"
docker compose up -d --no-build --force-recreate --wait
curl --noproxy '*' --fail --cacert trust/tsensei-node0-root.crt https://192.168.88.229/api/health
ln -sfnT "$PREVIOUS" /home/houjue/tsensei-hand-web/current
```

第一次安装无旧版本时不能回滚，排查新服务日志即可。旧 release 保留用于恢复，不因整理文档删除。

证书和续期状态在 Docker 卷 `tsensei-hand-web_caddy_data`，配置状态在 `tsensei-hand-web_caddy_config`；说明页、公开根证书和安装包在 release 的 `trust/`。**不要执行 `docker compose down -v`**：删除证书卷会更换 CA，所有电脑需重新信任。备份迁移应保存 release 与证书卷；私钥只能保留在受控服务器备份，不能放进客户端 ZIP、Git 或下载目录。

IP 改变时同步更新 Caddy 绑定/站点、Compose 的 `--allowed-host`、首次页链接、activate.sh 的健康检查和 Windows 启动地址，再部署。只重启或由同 CA 续签叶证书不需重装根证书。

## 8. 常见问题

| 现象 | 检查与处理 |
|---|---|
| HTTP 说明页也打不开 | 公司内网连接、路由、TCP 80/443；不同 VLAN 可能需 IT 放行 |
| 证书警告或相机 API 不可用 | 用 HTTPS、信任正确根证书并完全重启浏览器；不依赖 HTTP 或仅跳过警告 |
| 相机拒绝 / 找不到 | 浏览器站点权限、系统权限、设备是否被其他程序占用 |
| GPU 初始化失败 | 检查浏览器图形加速，页面明确显示 CPU 回退；服务器不需 GPU 推理环境 |
| 服务繁忙 | 每 IP 2 个、node0 全局 8 个；NAT 共用出口会共享每 IP 限额 |
| 会话失效 | 空闲 120 秒或共享会话达到 15 分钟后回收，重新开始 |
| HTTP 可用但 HTTPS/模型失败 | 看 app/gateway 日志、健康状态和 Host/IP 配置 |
| IP URL 的 TLS internal error | 检查 host 网络及 Caddy 实际绑定，见 5.6 节 |
| Windows curl 撤销状态未知 | 私有 CA 无公共吊销端点时 Schannel 可能如此；用 node0 curl --cacert 或 Python 显式 CA 验证区分链错误，不全局忽略证书 |
| 本地标定改了远程没变 | 容器是独立快照，需重新打包、上传和构建 |
| Compose 缺少 buildx/Bake 提示 | 本次回退内建 builder 后成功，该提示本身不是构建失败 |

## 9. 本地角度记录

当前网页统一使用本文开头“本机数据与原查看器”的浏览器记录方式，node0 不落盘用户识别记录。需要旧 V3.2 Python JSONL/CSV 或中指离线分析时，继续使用 `online_control/online_motion_track/pipeline_v3_2/` 的原采集和分析入口，不将当前浏览器 JSONL 当成旧字段格式读取。桌面识别、模型替换和手工标定文档有独立用途，继续保留。

## 10. 备选：其他服务器的公网部署

这是原通用 `deploy.sh` 路线，**当前 node0 没有使用**。目标 Ubuntu 24.04 / Debian 12、13、systemd、Python 3.11–3.13；安装需访问发行版软件源、PyPI 和 Caddy 官方仓库。

准备真实域名，A/AAAA 指向服务器，开放 TCP 80/443；运行包解压后执行，替换示例域名：

```bash
cd tsensei-hand-web
sudo bash deploy.sh --domain hand.example.com
```

脚本安装隔离环境、检查左右手求解、配置 systemd 与自动 HTTPS。程序在 `/opt/tsensei-hand-web/current`、数据在 `/var/lib/tsensei-hand-web`，服务账号为 `tsensei-hand-web`。应用启动检查失败会尝试恢复旧版本。此路线通过 Bash 语法检查，尚未在其他目标服务器实机验证，不能把 node0 Docker 验证当作其验证。

已有 Nginx/Caddy 占用 80/443 时：

```bash
sudo bash deploy.sh --domain hand.example.com --no-caddy --port 8766
```

在既有 HTTPS 站点添加代理，证书使用该站点配置：

```nginx
location / {
    proxy_pass http://127.0.0.1:8766;
    proxy_set_header Host $http_host;
    proxy_set_header X-Real-IP $remote_addr;
    proxy_set_header X-Forwarded-Proto $scheme;
    client_max_body_size 128k;
    proxy_read_timeout 30s;
}
```

整站放域名根路径，不支持 `/hand/` 子路径。代理需与后端同机并覆盖真实来源 IP；经 CDN 时先配置可信来源，否则会话限制按 CDN 出口统计。原版 STL 首次加载大，支持 ETag 和网关压缩；并发按实际负载扩容。

```bash
sudo systemctl status tsensei-hand-web
sudo journalctl -u tsensei-hand-web -f
sudo systemctl restart tsensei-hand-web
sudo systemctl stop tsensei-hand-web
```

仅本地验证运行包、不安装系统服务：

```bash
python3 -m venv .venv
.venv/bin/python -m pip install --only-binary=:all: -r requirements-server.txt
.venv/bin/python server.py --check
.venv/bin/python server.py --no-open
```

Windows 对应 `.venv\Scripts\python.exe`，访问 localhost；共享服务使用 `--public --allowed-host 实际域名` 并配置受信任 HTTPS。

## 11. 参考与文档维护

- [MediaPipe 示例](https://google-ai-edge.github.io/mediapipe-samples-web/#/vision/hand_landmarker)、[Web 指南](https://developers.google.com/edge/mediapipe/solutions/vision/hand_landmarker/web_js)
- [官方 Hand Landmarker Worker](https://github.com/google-ai-edge/mediapipe-samples-web/blob/main/src/workers/hand-landmarker.worker.ts)、[Base Worker](https://github.com/google-ai-edge/mediapipe-samples-web/blob/main/src/workers/base-worker.ts)
- [Caddy 自动 HTTPS](https://caddyserver.com/docs/automatic-https)、[Docker Compose 网络](https://docs.docker.com/compose/how-tos/networking/)

只维护本文 `online_control/online_motion_track/DEPLOY.md`，后续运行包自动携带。原网页版及 node0 两份独立说明已合并删除，入口链接改指本文。

保留模型标定、桌面识别、实验分析、online_motion_play 等独立文档。历史压缩包、校验清单和 release 内的旧文档是部署快照，不作为源文档继续维护，也不因去重改写。
# 下发节拍与串口诊断（2026-09-23）

A3 手势流的位置包以 2 ms 为目标最小调用间隔；静止姿态和直接模式也不会将
同一个网络批次无间隔写入 USB。Ruckig 的运动限速仍可能要求更长间隔。
计时器从 MessageChannel 任务启动，避免嵌套 setTimeout 的 4 ms 下限。
`serial_io_stats.io.position_write_intervals` 提供最近 4096 个成功位置写入的
起始间隔、p50/p95/p99 和 1–3 ms 占比，包含暂停间隔，仅代表主机 API 调用时间。
浏览器/操作系统不是硬实时系统；H7 实际收包间隔仍需固件时间戳测量。
