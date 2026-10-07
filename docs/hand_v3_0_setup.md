# TsenseI A3.0 接入与标定

模型 ID 为 `TsenseI_v3_0`。源文件为 `TsenseI_A3.0_hand/urdf/V6 26.8.4.urdf`，源目录保留不动。
资源位于 `assets/hands/TsenseI_hand_v3_0/`，包含完整原版 STL、左右手 URDF、独立 profile 和 21 点标定文件。
网页模型下拉框选择 **A3.0**，左右手选择独立生效。原 V2、A3.2 配置不变。

## 21 点标定（在项目根目录，hand_dex 环境）

使用原版 URDF 网格的三维编辑器标定左手：

```powershell
conda activate hand_dex
cd D:\Qtworks\hand
python tools/edit_robot_keypoints_21.py --model TsenseI_v3_0 --hand-side left --view 3d
```

保存后，按相同几何镜像生成右手标定（会备份已有右手 YAML）：

```powershell
python tools/prepare_hand_v3_0.py --sync-right-keypoints
```

需要检查或单独微调右手时：

```powershell
python tools/edit_robot_keypoints_21.py --model TsenseI_v3_0 --hand-side right --view 3d
```

左手保存为 `assets/hands/TsenseI_hand_v3_0/robot_keypoints_21.yaml`，右手保存为同目录 `robot_keypoints_21_right.yaml`。
右手单独微调后，不要再运行同步命令，除非确实要用左手镜像覆盖它。
编辑器支持旋转、缩放、点选、局部坐标调整和 Shift 点击模型表面。坐标面板以 mm 显示，YAML 以 m 保存。
初始点来自 A3.0 自身的关节中心和网格端部，**是人工标定前的初稿**，不是 A3.2 标定结果。
修改保存后，停止网页识别再重新开始，让新会话读取标定文件；部署服务器时需重新打包带上新 YAML。

## 网页启动

```powershell
python online_control/online_motion_track/main.py --port 8766 --no-open
```

打开 `http://127.0.0.1:8766/?model=TsenseI_v3_0`。现有网页入口集中管理三个模型，A3.0 专用接入代码放在 `online_control/online_motion_track/pipeline_v3_0/`。
路径：浏览器 MediaPipe 21 点 → A3.0 独立标定/retarget → 20 个 URDF 关节角 → 网页模型。
支持原有本机 JSONL 记录，`retarget`/`control_target` 保存角度；`a30_conversion` 保存 9 路直线行程（mm）、11 路旋转角度（deg）、限幅明细及换算耗时。
完整依赖环境已接入独立碰撞模型、Ruckig 和浏览器本机 H7 串口控制。控制专用代码位于 `online_control/online_motion_track/a30/`；服务器只计算，不连接访问者的下位机。

## 浏览器连接 H7

实际控制链路：浏览器相机/MediaPipe → A3.0 retarget → 碰撞停止或 Mink → 机构有效角度范围 → 20 关节 Ruckig → 采样路径碰撞检查 → 9 路 mm + 11 路 deg → 浏览器组装 108 字节帧 → 本机 USB 串口 → H7。

1. 使用桌面 Chrome / Edge，打开本机 localhost 或可信 HTTPS 页面，选择 A3.0 和实际手别。连接前退出占用串口的 Qt 程序。
2. 选择“碰撞停止”或“Mink 避碰”。点击“连接串口”，在浏览器设备选择器中授权本机 USB；已授权串口列表可刷新。115200、8N1、无流控。连接不会自动使能或发送姿态。
3. 等待完整 18+2 路新鲜遥测，再点击“一键找零”。依次执行 18 路自研电机复位、使能、找零，并根据反馈 WAIT/SEEK/DONE/ERR 判断结果。结束后弹窗列出失败或未支持找零的关节。
4. 点击“开始识别”，有有效手势后点击“开启实时下发”。每次开启都从当前完整遥测反解实际关节角，重置 Ruckig，再确认连续有效手势。个别找零失败不会单独禁止下发；仍须满足反馈、关节范围、碰撞与跟踪条件。
5. “停止 / 自研断电”请求 18 路退出找零及断电。两路外接电机仅停止下发：源文件没有独立找零/断电命令，页面会明确提示，不套用 V2 的指令。外接减速比与 `online_motion_play` 一致：M1（ROTABACK）＝50、M2（ROTA_BLOCK）＝30，下发和反馈均使用此比例。
6. 结束会话导出本机 JSONL，再用网页数据查看器打开。记录包含 `model_id`、20 路 `slot_maps`、原始遥测、组帧字节、实际写入标记、Ruckig 样本及找零结果；保持原 `hand-control-session/v2` 格式。

服务器采样目标 500 Hz，使用实际经过时间推进；浏览器只写入新鲜样本，上限 50 Hz，不缓存追赶旧姿态。四指 swing 的速度/加速度/jerk 上限为 1.5/10/80，其余旋转为 2.5/20/120，屈伸为 4/30/200，单位依次为 rad/s、rad/s²、rad/s³。这些是关节空间约束，尚不是按实测电机能力标定的 mm/s 约束。

丢手、反馈不完整/过期、外接故障、页面隐藏、采样碰撞或链路超时都会停发，需要手动重新开启。普通停发维持最后目标，不等同断电；固件通信超时保护仍由 H7 实现。

后端接口为 `/api/a30/control`，只接受 `status`、`tracking_start`、`browser_rebase`；浏览器负责串口扫描、找零、使能和写入。profile 的 `hardware.enabled: false`、纯转换器的 `hardware_enabled/downlink_ready: false` 保持不变，表示不启用服务器端串口。Web 控制能力由 `/api/health` 的 `a30_control_available` 单独报告。

## 自碰撞检测与避碰

网页选择 A3.0 后可选：

| 模式 | 行为 |
|---|---|
| 关闭 | 直接预览 retarget 结果 |
| 仅检测 | MuJoCo 检测接触，显示接触数量，不修改姿态 |
| 碰撞停止 | 复用 V2 `SelfCollisionGuard` 的路径采样与二分截断，在首次碰撞前停止；不执行 V2 的全局绕行规划 |
| Mink 避碰 | 与 A3.2 相同的距离约束/QP 修正，含关节限位和相邻输出之间的路径采样检查 |

本地依赖齐全时默认 Mink；缺少 Mink 但具备停止模式时默认停止。缺少依赖的选项禁用，服务端也拒绝对应模式，不静默关闭避碰。
左右手分别使用 `mjcf/mjmodel.xml` 与 `mjcf/mjmodel_right.xml`，由 A3.0 原版 URDF 的关节树、轴、限位生成，右手按 Y 轴反射。
15 个手指胶囊体 + 1 个掌部盒体，共 100 对启用的碰撞对；同指接触排除，末两节检测与掌部的接触，拇指与其余四指的末节对仍启用。
Mink 普通对目标间距 0.5 mm，拇指末节与其他指末节为 0.05 mm，接近接触时可继续对捏。停止策略允许穿透设为 0 mm，路径采样步长不大于 0.025 rad（当前限位下）。

处理顺序：MediaPipe → retarget 原始角度 → 碰撞检测/修正 → Ruckig 与采样检查 → 模型显示与 9+11 执行器转换。网页本机记录 `control_target.self_collision` 保存诊断，`retarget.joints` 保留原始角度，`a30_conversion` 对应返回网页的控制姿态。关闭碰撞与仅检测仅供预览，不能开启实时下发。

碰撞代理可在 `assets/hands/TsenseI_hand_v3_0/collision_geometry.yaml` 调整，然后重建（保留原 YAML，不覆盖手工调整）：

```powershell
python tools/generate_profile_collision.py --model TsenseI_v3_0
python tools/render_profile_collision.py --model TsenseI_v3_0
```

检查图为同目录 `qa/collision_overlay.png`。胶囊/盒体近似并不精确包络全部外壳，路径离散采样也不是连续碰撞的严格证明；本轮不改变原版显示网格与 21 点标定。

## 镜像与关节命名

A3.0 的掌宽沿 Y 轴，关于 XZ 平面镜像，`M=diag(1,-1,1)`。
位置和 STL 顶点使用 M；旋转使用 `M R M`；旋转关节轴使用 `-M a`；惯性使用 `M I M`，STL 三角形翻转绕序。
左右手的同名关节在同一角度下形成镜像姿态，保留原 CAD 的轴向、限位和全分辨率网格。
小拇指按实际功能统一命名：源 `rotaback → MCP`、`MCP → swing`、`swing → PIP`、`PIP → DIP`，同时调整 link 名。
原链的第一关节是屈伸、第二关节是侧摆，因此链顺序是 `MCP → swing → PIP → DIP`，与其余三指一致。
映射记录在 `source_mapping.yaml`，原始 CAD 不改名。

重新生成资源：`python tools/prepare_hand_v3_0.py`。相同源保留已有标定；源发生变化时先归档旧资源与标定到 `assets/hands/_backups/`。

## 角度转行程与协议编码接口

按用户指定，从 A3.2 换算改编：保留拇指 MCP 和四指 MCP/PIP 的 9 路直线连杆参数，四指 DIP、拇指 PIP 改成旋转电机，直接 `degrees(URDF角度)` 输出。四指 swing、拇指两路旋转仍直接输出角度。
新增独立入口 `src/a30_kinematics/mapper.py`，复用 A3.2 的低延迟解析/查表核心，默认 `lut`、可选 `exact`。每帧只对 9 个直线关节执行连杆换算，四指 DIP 与拇指 PIP 不经过任何直线行程计算、查表或行程限位。

```python
from a30_kinematics import A30StrokeMapper  # 项目 src 在 Python 导入路径中

mapper = A30StrokeMapper("left")  # right 使用相同关节角符号
pose = {name: 0.0 for name in mapper.joint_names}
pose["left_hand_index_DIP_joint"] = 0.5
targets = mapper.convert(pose)
assert len(targets.linear_strokes_mm) == 9
assert len(targets.rotary_positions_deg) == 11
# 食指 DIP 为 28.6478897565 度，不出现在 linear_strokes_mm 中
preview = mapper.preview(pose)
```

严格 `convert()` 拒绝缺关节、错误手别、NaN/Inf 和超限值；网页 `preview()` 显式限幅并记录 `clamped_joints_rad`。旋转通道限位来自 A3.0 URDF，直线通道取 URDF 与复用机构有效范围的交集（食指/无名指 MCP 上限 1.5229 rad）。完整控制会话在 Ruckig 前限制目标范围，模型显示对应控制采样姿态。
复用参数的来源在输出 `linear_geometry_source` 中记录；这不是根据 A3.0 CAD 重新测量的连杆参数。

`online_control/online_motion_track/pipeline_v3_0/actuation.py` 定义：

- `A30Actuation.convert(joint_angles_rad, hand_side=...) -> A30MotorTargets`：已实现，输入完整 URDF 关节名/弧度；输出线性电机 mm 和旋转电机 deg。
- `DownlinkEncoder.encode(targets, hand_side=...) -> bytes`：把执行器目标编码为 A3.0 下位机协议帧。
- `A30Actuation.preview()`：已接到每帧响应的 `actuation` 字段，状态为 `conversion_ready`，并给出 `valid`。`encode()` 已接入确认后的 108 字节协议组帧，仅返回 bytes，不连接或发送串口。

固件通道及批量协议已按用户确认映射，浏览器传输、反馈保护、Ruckig 与找零已接入。实机零位/方向、速度约束及复用连杆参数仍需通过实机验证，本次验证使用模拟 H7。
转换结果按关节名索引；描述中的 `source_motor_index` 是 A3.2 源模型编号，不是 USB 通道。`firmware_channel` 给出 CAN、motor、field、slot、unit 和减速比；`encoder_ready=true`。纯转换接口只产生数据，实际传输由独立浏览器控制器执行。现有 V2 控制 API 仍拒绝 A3.0 会话。

## 打包

现有 `online_control/online_motion_track/package.py` 的精简包和 `--full` 包均包含 A3.0 资源及专用接口。
要在服务器使用碰撞与 H7 控制，使用 `--full` 包（含 MuJoCo、Mink、Ruckig、A3.0 控制会话及反馈反解）；精简包仅提供关闭碰撞的预览。
沿用已有部署脚本即可随版本发布；当前改动在本地完成，标定后再部署。

## 转换验证（2026-09-20）

后续碰撞接入验证：48 项 A3.0/转换/网页/采集管线测试通过；共享 V2/A3.2 与 A3.0 碰撞回归 31 项通过（两组有重叠）。浏览器验证左右手分别使用 Mink/停止模式，并核对本机记录；完整可移植包在隔离导入环境中通过左右手三种碰撞模式检查。相向侧摆用例从约 15.8 mm 代理穿透修正到普通碰撞对约 0.5 mm 间距。

20 项 A3.0 测试通过，覆盖左右手、exact/lut 对照、四指 DIP 独立输出、非法输入与限幅、工作台响应及打包资源。
浏览器使用测试关键点驱动真实求解器，验证左右手本机 JSONL 输出与 DIP 弧度到度换算；可移植包独立启动检查通过。
首次联合回归曾检出 A3.2 小指 MCP/swing 映射与当前 URDF 轴向不一致。后续已修复 `src/a32_kinematics/mapper.py`：MCP 保持同名，源 SWING 仅转为小写 swing。A3.2/A3.0 转换与网页回归共 35 项通过，包括左右手 MCP 1.0 rad 不误限幅、正负 swing 不串入直线通道。A3.0 转换逻辑未改变。

## 已确认的下发顺序（2026-09-20）

提取模块：`src/a30_kinematics/downlink.py`，来源 `A3_USB_Downlink.pyw`。不导入原 GUI，不包含 OTA/IAP，不打开 USB/TCP，不自动使能或找零。

| 总线 | M1 | M2 | M3 | M4 | M5 | M6 | M7 | M8 |
|---|---|---|---|---|---|---|---|---|
| CAN1，self_pos[0:8] | 小指 DIP ° | 小指 swing ° | 无名指 DIP ° | 无名指 swing ° | 小指 PIP mm | 小指 MCP mm | 无名指 PIP mm | 无名指 MCP mm |
| CAN2，self_pos[8:16] | 中指 DIP ° | 中指 swing ° | 食指 DIP ° | 食指 swing ° | 中指 PIP mm | 中指 MCP mm | 食指 PIP mm | 食指 MCP mm |
| CAN3，self_pos[16:18] | 拇指 PIP ° | 拇指 MCP mm | — | — | — | — | — | — |

外接 CAN3 M1 (`mit_pos_deg[0]`, `0x505`) 对应 `thumb_rotaback_joint2`，M2 (`mit_pos_deg[1]`, `0x509`) 对应 `thumb_rota_block_joint`。
外接输入为输出轴 deg/rpm，由 `encode_targets` 按 M1 ×50、M2 ×30 **各一次**变成电机端数值；前馈电流 A 不乘减速比。反馈按对应电机的减速比还原。
自研 18 槽使用 mm/deg，其中全部 9 路直线电机遵循 `online_motion_play` 的 `-abs(行程)` 规则（零行程下发正零），旋转电机角度保持原符号。直线反馈在进入几何换算前取反还原。协议内 bus 为 0/1/2，对应 CAN1/2/3，node 为数字 1..8，而不是 ASCII。

### 与 online_motion_play 的关节和范围同步（2026-09-21）

手势跟踪与动作播放共用 `A30StrokeMapper` 的 20 个关节及有效限位，直线关节取 URDF 与连杆可达范围的交集；手势目标在进入 Ruckig 前限幅。`src/a30_kinematics/policy.py` 提供共用的下发规则和实际 float32 下发范围，`A30Actuation.encode()` 与 `online_motion_play` 均调用该模块。转换器描述的每个 `channels` 条目包含 `limits_rad`、固件槽位和 `downlink` 范围（含角度两端对应的下发值）。

浏览器 A3.0 协议同步为 50/30 减速比及非正直线行程；反馈位置、速度、保持当前位置和缺失反馈回退采用相同换算。回归覆盖左右手全部关节各五个角度采样、正负行程输入的 400 组帧逐字节对照，并检查帧值位于动作播放的下发范围内。验证使用模拟数据，未向真实 H7 下发。

整包：`AA 55 | len=100(u16 LE) | DEV=3 | FUN=0x20 | payload | CRC16-Modbus`，共 108 字节。
载荷严格采用源代码实际布局：`<B18f2f2f2f3x`，即 seq、18 自研位置、2 外接位置、2 外接转速、2 外接电流前馈、3 保留字节。序号按 uint8 回绕。
左/右手只用于关节名字校验，原协议不含手别选择字段；镜像模型同值关节角不额外取负。

```python
from online_control.online_motion_track.pipeline_v3_0.actuation import A30Actuation
from hand_profiles import load_hand_profile

adapter = A30Actuation("left")
pose = load_hand_profile("TsenseI_v3_0").init_pose("left")
targets = adapter.convert(pose)  # 严格转换，9 mm + 11 deg
frame = adapter.encode(targets, seq=0,
                       mit_vel_rpm=(0.0, 0.0), mit_iq_ff_a=(0.0, 0.0))
assert len(frame) == 108  # 仅生成帧，没有发送
```

另提取纯组帧函数：`pack_self_pwr` (0x21)、`pack_self_homing` (0x22)、`pack_self_pos_pid` (0x23)、`pack_self_cur_pid` (0x24)、`pack_self_open_spin` (0x25)。找零 iq_seek 沿用原程序的码值单位，不是 A。低层 `pack_hand_batch` 接受已经按方向取反、乘减速比的协议端值，业务调用优先用 `adapter.encode` 防止重复换算。

确认后验证：58 项测试通过，包括 9+11 换算、拇指 PIP、20 电机槽位、外接减速比、float32 非法输入拒绝、源文件逐字节批量/单点组帧对照，以及 A3.2 与网页回归。未执行实机下发。

网页 H7 接入验证：后端联合回归 69 项通过，随后补充的 Mink 反馈重置和精简包能力检查也通过；A3.0 协议/模拟设备测试 5 项、原 V2 串口测试 10 项通过。浏览器完成模拟 H7 的授权串口连接、部分找零失败弹窗、实时下发、丢手停发、本地 JSONL 导出与 20 通道数据查看器验证；左右手预览及 V2 找零回归通过。完整部署包在隔离导入环境通过左右手 Stop/Mink + Ruckig 会话检查。没有向真实 H7 发送指令，也没有发布 node0。
