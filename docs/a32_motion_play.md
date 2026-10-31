# A3.2 接入 Motion Play

## 支持范围与启动

动作平台新增两个独立型号：`TsenseI_v3_2`（SDK 的 daxian_v32 裸手模型）和 `TsenseI_v3_2_withid`（用户提供的带 ID 模型）。两者各有左、右手模型、独立动作文件和 20 个关节控件。左右手均可通过原生 SDK 控制实机，并共用 A3.2 的 18 个 SELF、2 个 MIT 槽位映射。

```powershell
& E:/conda/envs/hand_dex/python.exe online_control/online_motion_play/server.py --no-browser
```

打开 `http://127.0.0.1:8765`，在“手部型号”选择对应的 A3.2 型号。SDK 动态库的构建、位置和槽位定义见 [A3.2 原生 SDK 模块](../src/a32_driver/)。

**A3.2 的 USB / ETH 连接发生在运行 Python 服务的电脑。** 实机操作仅允许服务电脑上的同源本机浏览器（loopback 请求），远程页面可离线查看和编辑。在 node0 部署时，需在服务主机上准备兼容的原生 SDK 库，并从该主机本地访问控制页面。当前随项目构建的动态库为 Windows x64；原来的 V2 / A3.0 浏览器串口功能保留。

## 页面操作

1. 选择 A3.2 裸手或 with ID，选择左手。USB 模式可填写 `auto` 或明确的 `COM3`；ETH 模式填写设备 IP 和 TCP 端口（默认 5001）。点击“连接本机 SDK”。连接不会使能电机或发送关节轨迹。
2. 可以点击“使能 18 self + 2 MIT”，也可以直接勾选“实时关节控制”或“动作同步下位机”来使能。勾选本身及随后的空闲周期不发姿态，等待调整关节或启动动作。不需要等待遥测；驱动优先参考各关节有效反馈，缺失部分采用上次成功下发姿态，首次使用页面姿态。
3. 如需找零，填写已标定的正整数 Iq 码值，选择全部 18 路 self 或单独一个 self 槽位，点击“自研电机找零”并在页面确认。找零通过 SDK `homeSlot` 实现，MIT 两路不参与。找零期间禁止姿态操作，完成后重新使能。SDK 反馈未达到要求时会显示具体错误。
4. 勾选“实时关节控制”后调整滑块，从上述起步姿态开始 Ruckig 轨迹。不要求 18 self + 2 MIT 的完整反馈或 self 找零完成。页面显示使用软件姿态起步的关节数量，这些姿态不是实测位置。
5. 保存、加载、删除、导入、导出、按选定动作开始播放及循环播放，与 A3.0 的操作方式一致。勾选“动作同步下位机”可将序列下发实机；它与实时关节控制互斥。不勾选时可离线预览。
6. “回到初始姿态”设置 PIP/DIP/MCP 为 `0.005`、四指 swing 和两路 MIT 为 `0`。swing 在实际下发前限幅到 `[0.005, 0.695]`。开启实时控制时通过轨迹移动，未开启时只修改预览。顶部“停止下发”和“停止 / 全部失能”停止播放、取消轨迹并调用 SDK 停止/失能。页面隐藏或退出时暂停下发，保留 SDK 连接；返回后重新勾选，调整关节或启动动作后再下发。控制页心跳超过 5 秒则停发并释放控制权，连接仍保留。明确断开或释放控制会断开 SDK。
7. 连接时自动开始保存控制日志，断开时结束，暂停期间继续记录状态。默认保存到 `E:\TsenseI_hand_logs\A32`，实际路径显示在按钮下方。可点击“下载控制记录”，也可在“数据查看器”中刷新目录并打开文件。查看器支持实际时间、时间范围、平均/P95/最大控制周期、20 通道曲线、CSV 和记录时长汇总；缺失或过期测量断线显示，CSV 留空。详见 [记录格式](a32_motion_recording.md) 和 [串口/遥测诊断](a32_connection_diagnostics.md)。

找零取消、超时或错误后，如果固件仍显示寻零状态，可点击“复位找零状态”。它先停止并失能，然后对所选 self 槽位（或全部 18 路）调用 SDK `homeResetSlot`，保持未使能。它不把状态伪造成找零完成，也不自动移动关节；根据新反馈重新找零和使能。

控制页面没有调用 SDK 的示例程序、Web Bridge 或任何 SDK `.exe`。本项目的 Python 解释器加载 DLL，由 DLL 直接调用 SDK。

## 代码路径与调用关系

```text
index.html
  → server.py 的 Bridge（校验控制权、选择型号、管理连接及超时）
  → a32_session.py 的 A32Session（动作文件、页面控件、Ruckig、播放）
  → a32_driver.A32Device（模型坐标、物理限位、反馈、SDK 状态）
  → a32_driver.NativeHand（ctypes 调用本进程动态库）
  → A3 C++ SDK（USB CDC / ETH TCP）
```

Motion Play 使用以下 `A32Device` 接口：

| 接口 | 用途 |
| --- | --- |
| `joint_limits()`、`constrain_pose()`、`prepare_targets()` | 页面可用关节范围、初始姿态和 SDK 下发范围预览；无需连接设备 |
| `connect_usb()`、`connect_eth()` | 建立 SDK 连接并检查固件协议 |
| `get_state()`、`starting_pose()` | 通道反馈、找零状态及允许缺失反馈的起步姿态 |
| `enable_all(start_pose=..., send_initial_pose=False)` | 只使能，不发送起步姿态，无需遥测 |
| `send_joint_targets(..., clamp=True)` | 先校验模型轨迹，再将 swing 两端限幅到实际命令范围后下发 |
| `home()` | 全部或单 self 槽位找零 |
| `reset_home()` | 停止后显式复位所选 self 找零状态，保留未使能 |
| `stop()`、`disconnect()`、`close()` | 取消控制、请求停止/失能、释放连接 |

SDK 对应使用 `connectUsb` / `connectEth`、位置及状态读取、`setPositions`、`setMitAll`、`sendOnce`、使能/失能、`homeSlot` 与停止/断开接口。MIT 下发为输出轴角度，减速比在 SDK 中处理一次。A3.2 的 14 个直线、4 个 self 旋转和 2 个 MIT 通道不使用 A3.0 的 9+11 编码、负直线规则、浏览器 Worker 串口或外接电机协议。

两款模型使用各自的连杆和网格，裸手四指 swing 轴向已与带 ID 对齐。裸手原点随轴向同步调整，保留原摆动区间；两款模型和页面范围 `[0, 0.7]`、默认 `0`。SDK swing 都等于限幅到 `[0.005, 0.695]` 后的当前模型角，再换算为度，详见 [关节范围说明](a32_joint_ranges.md)。已有正坐标动作保留原数值与 SDK 命令，无版本的原始有符号动作按各自 CAD 规则转换。

## 轨迹、范围和文件隔离

`A32Session` 使用独立线程驱动 20 轴 Ruckig，目标周期为 10 ms（100 Hz），与 Qt 页面/HTTP 任务分离，每次采样按实际经过的时间推进。实时控制采样间隔超过 100 ms 时仍停止，显示停止原因；只要保持连接且不在找零，复选框可手动重新使能恢复。关节速度上限为 1 rad/s，加速度 4 rad/s²，jerk 40 rad/s³。动作“过渡”是轨迹最短时长；当速度、加速度或 jerk 限制需要更长时间时，以轨迹结束为准，再计动作间隔。调度速度受系统实际负载影响。

页面范围是模型/机构范围和 SDK 实物标定范围的交集。每次下发前检查采样点及整段轨迹的位置极值，越界会停止；SDK 下发成功后才提交页面与下一步轨迹状态，下发失败则保留上次成功姿态。连接断开、失能或进入找零会取消播放及实时下发；反馈缺失、过期或部分通道离线不取消下发。该动作编辑器沿用手动控制范式，没有加入手势平台的碰撞检测。

数据目录默认是 `online_control/online_motion_play/runtime/`，可通过服务配置的 `data_dir` 更改：

| 模型 | 左手文件 | 右手文件 |
| --- | --- | --- |
| 裸手 A3.2 | `motions_a32_left.json` | `motions_a32_right.json` |
| 带 ID A3.2 | `motions_a32_withid_left.json` | `motions_a32_withid_right.json` |

文件头包含 `_model_id`、`_hand_side`，动作在 `motions` 下存储完整 `pose`、`transition`、`interval`。导入验证型号、手别、20 个关节完整性、有限数值、实际关节范围、时间 0–600 秒和最多 2000 个动作。验证通过后才原子替换文件。A3.0、V2、另一 A3.2 型号以及另一手别的动作不会混用。

控制日志独立于上述动作数据目录。两平台共用 `src/a32_driver/recording.py`；旧
`online_motion_play/a32_recording.py` 只保留兼容导入。日志父目录保存在
`configs/control_log_parent.txt`，查看器修改后对下一次录制生效；两款 A3.2 均归入
`A32`，由文件的 `metadata.model_id` 区分。每个事件记录单调相对秒数 `t`、
主机采样时间 `wall_time_unix_ms` 和带时区的 `wall_time`；设备 `tick_ms` 单独保存，
`h7` 记录实际控制周期 `control_dt_s` / `control_period_ms`。记录时长包含暂停，
不能当作电机通电或运动时长。

服务提供 `/api/viewer/logs`、`/api/viewer/log-file` 和 `/api/viewer/log-parent`
供查看器列目录、读取文件及设置父目录，本机日志接口只允许本机同源页面访问。

## 部署与验证

`online_motion_play/package.py` 包含两款 A3.2 的 URDF/STL/profile、机构 `.mat` 和源码、A32 session、硬件 JSON 配置及已构建 DLL；跳过构建中间目录和 SDK `.exe`。构建动态库仍需原 SDK 源码和 C++ 构建工具，动作页面运行时不需要 SDK 的 Joint UI、C++ Bridge、SDK 原示例页面、IAP/OTA、相机或 ROS。

离线检查：

```powershell
& E:/conda/envs/hand_dex/python.exe online_control/online_motion_play/server.py --check --no-browser
& E:/conda/envs/hand_dex/python.exe -m pytest online_control/online_motion_play/test_a32.py -q
```

测试覆盖四个型号/手别组合的动作隔离、导入失败保护、USB/ETH 分流、使能前不下发、反馈起步、离线与同步播放、停止、找零操作、控制权超时和打包资源。软件测试使用模拟设备，不代替真实手的接线、方向、零位、电流和连贯运动验收。

### Motion Play 全部姿态下发电机补偿（2026-10-27）

A3.2 和 A3.2 with ID 按左右手分别配置电机系数，同一手指的全部电机（含侧摆）使用相同系数。

| 手指 | 左手 | 右手 |
| --- | ---: | ---: |
| 拇指 | 1.0000 | 1.000000 |
| 食指 | 1.0560 | 1.09 |
| 中指 | 1.0808 | 1.10 |
| 无名指 | 1.1049 | 1.10 |
| 小指 | 1.0514 | 1.06 |

系数定义在 `online_control/online_motion_play/a32_session.py` 的 `MOTOR_FINGER_SCALES["left"]` 和 `MOTOR_FINGER_SCALES["right"]`。左手食指保留当前文件中的 1.0560。

“动作与序列”区域的“增加系数补偿”复选框默认不勾选：勾选后所有电机增加当前手别的补偿系数；取消勾选时使用原始下发值。开关控制所有姿态下发，包括滑块、数值输入、加载动作、序列播放和回初始姿态。切换本身不发送位置帧；正在下发时，从下一帧生效。页面的转换后行程曲线和范围同步更新。此开关属于当前会话，切换型号/手别或重启服务后恢复默认不勾选。

乘法位于 SDK 机构转换之后，作用于电机目标行程（mm）或旋转角度（deg），保持正负方向和零值，结果夹紧到原有 SDK/标定限位。模型弧度、保存的动作文件不乘系数，每次下发只乘一次。motion track 保持原转换。日志的 `self` / `mit_pos` 为实际下发值，`motor_scales` 和 `clamped_motor_targets` 记录生效系数及触发限位的通道。
