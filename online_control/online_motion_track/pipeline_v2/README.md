# Light Test — Wuji MediaPipe 21点查看器

这个小程序同时显示三条同源链路：

- 左侧显示相机画面和 MediaPipe 2D 关键点；
- 中间显示 `src/tracking/wuji_mediapipe.py` 输出的 `(21, 3)` 骨架；
- 右侧加载项目 TsenseI URDF，并用项目 `utils.retarget_runtime` / `toporetarget`
  输出的 21 个关节角实时驱动模型。

中间视图使用 Wuji 平滑后的 `(21, 3)` 数据，保留形状和尺度，只为显示建立一个
清晰的手掌局部坐标：wrist 为原点、wrist → middle MCP 为 `+Z`，所以伸直时四指
方向与 `+Z` 大致相同。`+X` 是掌面法线，`+Y` 是掌宽方向。送入右侧 TopoRetarget
之前，再复用主项目的 `apply_mediapipe_transformations` 转到 MediaPipe/MANO 手腕
坐标。Topo 自身还会建立掌面规范坐标；因为共享求解资产是左手，真实右手会在进入
求解器的唯一边界做一次 X 镜像。这样不会把相机坐标误当成求解器坐标，也不会把
拇指翻到掌面的错误一侧。

识别器已从旧的 `mp.solutions.hands.Hands` 迁移为项目同款 MediaPipe Tasks
`HandLandmarker`。普通 webcam 默认使用其 `hand_world_landmarks` 作为三维输入，
不再用归一化 `z × 2.5` 作为主路径；只有外部/测试结果缺少 world landmarks 时才
保留原 Wuji 公式兜底。world landmarks 给出的是单目模型估计的手部相对三维，
不是相机测得的绝对距离。CMC 不再向掌面融合，也没有任何单点深度覆盖。

world landmarks 进入后仍保留原有 Wuji 环节：One Euro 平滑、以
wrist→middle MCP 缩放到 `0.09 m`、固定指骨长度，再进入手腕坐标变换和
TopoRetarget。除此之外的链路差异没有在这次修改中调整。

目标手选择为右手时加载 `TsenseI__hand_right_v1.urdf`，选择左手时加载
`TsenseI__hand_left_v1.urdf`；TopoRetarget 内部仍使用项目统一的左手求解资产。
手腕/掌面变换都只做旋转、不会改变手性，因此真实右手在共享左手求解器入口做且
只做一次 X 镜像；`retarget_runtime` 随后负责右手关节名与 `thumb_swing` 的 UI
符号转换。

当前 webcam/Tasks 链路已用日志中的三维手性和主项目行为完成校准，直接使用
MediaPipe 标签：真实右手预期 `MediaPipe label=Right`，真实左手预期 `Left`。
识别器一次检测两只手后再按该规则选择，避免始终落入 fallback。
这个标签也是后续链路的硬门禁：只有 `detected_label == expected_label` 且不是
fallback 的新识别帧才可进入 TopoRetarget。若不一致，会立即撤销控制目标和实时
下发使能；Ruckig、角度/行程转换及 H7 打包都不会继续处理旧目标。连续确认 3 帧
正确手别并成功规划后才会恢复目标，实时下发仍需人工重新勾选。

## 运行

在项目根目录执行：

```powershell
conda activate hand_dex
python example/light_test/main.py --camera 0 --hand-side right
```

也可以选择左手或视频文件：

```powershell
python example/light_test/main.py --camera 1 --hand-side left
python example/light_test/main.py --camera D:/video/hand.mp4 --hand-side right
```

常用参数：

- `--width 1280 --height 720 --fps 30`：请求的相机参数；
- `--smoothing standard|strong|off`：One Euro 关键点时域平滑；
- `--collision-mode avoid|stop|off`：选择柔性绕开、碰撞停止或关闭（默认 `avoid`）；
- `--recording full|mediapipe|off`：完整链路、可回放关键点或不记录（默认 `full`）；
- `--no-collision`：兼容参数，等价于 `--collision-mode off`；
- `--no-hold`：丢检时清空骨架，默认保持上一有效帧；
- `--no-auto-start`：打开窗口后手动点击“开始识别”。

顶部“拖动窗口”区域可以移动整个窗口。3D 视图中左键旋转，右键或中键平移，
滚轮缩放，`R` 重置视角。“停止”只释放相机和识别线程，可以再次点击“开始识别”；
“关闭窗口”会安全停止线程后退出。

## 抖动处理

默认分别对 MediaPipe 归一化 21 点（用于左侧叠图）和 world 21 点（用于三维链路）
使用同参数 One Euro 自适应低通，再进行 Wuji 的尺度与指骨校正。静止时滤波更强，
快速运动时自动提高截止频率，以减少传统固定均值滤波产生的明显拖尾。“强”模式
更稳但延迟稍大；需要检查原始数据时可关闭。

## 丢手与重新识别的连续过渡

默认开启的 Ruckig 和避碰策略共同处理这一过程：

`正常跟随 → 丢手后受限减速 → 保持停稳姿态 → 连续确认 3 帧 → 重规划并平滑恢复`

- 未检测到手、`held_last_valid`（仅显示旧骨架）或关键点质量不合格，都立即取消
  追赶旧目标。从控制器当前的位置、速度、加速度开始，用 Ruckig 速度控制平滑减速
  到零；不会先冲向旧目标，也不会先退回零位。停稳后保持停止处的手型。
- 手很快回来时，仍先完成减速。只有连续 3 帧新识别都通过手别/几何校验，且已经
  停稳，才重新进入求解。清掉旧避碰路径，从最后通过安全检查的控制姿态重新规划
  到新手势；不会把丢手前尚未执行完的目标当成当前位置。
- 恢复初期速度上限为正常值的 35%，用 1 秒 smoothstep 逐渐恢复正常上限；加速度
  与 jerk 上限按相应时间尺度一起变化。这 1 秒是限速恢复时间，不是强制到达时间；
  绕行较远或目标持续移动时，完成动作会更久。
- 减速、保持、恢复的每个 500 Hz 采样都通过独立碰撞门禁；规划异常时不接受原始
  未检查目标。找不到安全绕行路径就停住；若平滑制动也将触碰边界，安全截停优先。
- 识别线程超过 0.30 秒没有新有效观测，控制线程也会独立触发减速，防止相机/求解
  卡顿后仍然追赶旧目标。过期求解结果带有代次校验，不能覆盖丢手后的制动状态。

普通丢手不会重新开启原本关闭的实时下发，也不会撤销已经由用户开启的下发，因而
能继续发送受检的减速/保持轨迹。识别到错误手别仍是独立硬门禁：立即解除使能，
恢复后需人工重新开启。关闭 Ruckig 只用于定位问题，不提供平滑恢复；关闭碰撞策略
不提供避碰保护。

窗口模型状态会显示“平滑减速中 / 保持停稳姿态 / 重新确认手势 / 平滑恢复”。
完整与仅 MediaPipe 两种日志均保存 `tracking_transition`：阶段、原因、丢手次数、
恢复次数、连续确认帧数和当前速度比例。离线回放重新执行同一状态机，并报告实际
采样最大关节步长、最大模型穿透量及各阶段帧数，不依赖旧日志中存储的控制输出。

## 日志与拇指诊断

顶部“记录”可以在启动前选择三档：

- `完整链路（推荐）`：记录下面列出的全部输入、中间量和下发结果；
- `仅MediaPipe/回放输入`：只记录原始/平滑关键点、Wuji 21点与必要的时间/手别信息，
  文件更小，但仍能从 TopoRetarget 起重跑完整后续链路；
- `关闭`：本次识别不创建日志。

启用记录后，每次点击“开始识别”都会在 `example/light_test/logs/` 新建一个
`session_YYYYMMDD_HHMMSS_mmm.jsonl`。文件首尾是 session 记录，中间每个相机帧
一行。完整模式包含：

- MediaPipe 原始/平滑后的归一化 21 点；
- MediaPipe 原始/平滑后的 world 21 点（米）和本帧 `depth_source`；
- 未修改 CMC、仅经 Wuji 尺度/骨长处理后的 21 点（米）；
- 进入 TopoRetarget 的手腕坐标系 21 点；
- TopoRetarget 原始关节角、自碰撞修正后的最终关节角（弧度）；
- 碰撞安全目标、Ruckig 实时采样角、硬件角、14 路直线行程(mm)与 7 路旋转角(deg)；
- CMC 到掌面的距离、wrist→CMC / CMC→MCP 长度、`thumb_rota` 是否触顶；
- 碰撞对、选择的避碰策略、目标/输出最大穿透量、安全路径比例与下发状态。

JSONL 按行写入并周期 flush，即使程序异常退出，已经完成的行仍可直接解析。
schema v2 明确把 `wuji_points_21_m` 定义为稳定回放入口，同时保留原始 world / normalized
关键点用于排查单目深度和平滑问题。旧的 schema v1 日志也能被回放工具读取。

不接相机、不连接下位机的完整离线回放：

```powershell
python example/light_test/replay.py example/light_test/logs/session_xxx.jsonl
python example/light_test/replay.py example/light_test/logs/session_xxx.jsonl --max-frames 200
python example/light_test/replay.py example/light_test/logs/session_xxx.jsonl --output replay_result.jsonl
```

回放会实际执行 `TopoRetarget → 自碰撞 → Ruckig 500 Hz → UI/HW角度转换 →
连杆行程 → H7 0x20帧打包与CRC校验`，但不会扫描、打开或写入任何串口。最后打印
求解帧数、避碰帧数、Ruckig tick、H7 校验数和固定关节违规数，便于直接把日志交给
测试人员复现。

## 自碰撞保护

顶部“碰撞策略”有三档：

- `全局规划+CBF柔性绕开`：参考
  [Omnihand MoveIt2 控制项目](https://github.com/chiway-luo/Omnihand_moveit_control)，把五指作为
  一个完整关节空间进行规划。先用 MuJoCo 胶囊模型检查目标和整段路径；目标本身碰撞时，
  选择与 MediaPipe 目标最接近的安全手型，并显式生成“先侧摆让位、再屈曲、最后回正”的
  同步路点。结构化路点不能通过时，使用有计算预算上限的确定性 RRT-Connect 搜索，并对
  结果做逐段碰撞检查和捷径平滑；
- 全局路径执行期间仍保留原有局部 CBF：用胶囊有符号表面距离构造
  `J_dist Δq >= -γ(d-d_safe)`，让 `swing/rota` 优先柔顺让位。保护在 `1.25 mm`
  开始介入，目标表面间隙为 `0.25 mm`，单帧关节增量不超过 `0.035 rad`，随后由
  Ruckig 限速度、加速度和 jerk；
- `碰撞即停止`：从上一安全姿态沿关节空间连续采样，一旦穿透超过 `0.20 mm`，用
  二分搜索停在碰撞边界前；
- `关闭`：只用于定位问题，不提供自碰撞保护。

两种保护都允许不超过 `0.20 mm` 的胶囊擦过；全局规划失败且 CBF 找不到安全局部解时
自动降级为“碰撞即停止”。此外还有一套独立胶囊检查器逐个验证 Ruckig 的 500 Hz 实际采样
姿态；若采样轨迹碰到边界，会停住该帧并把被接受姿态回灌给 Ruckig 重新规划，避免
平滑器内部轨迹越过安全层。

`pinky_rota` 在 light_test 中是固定机械自由度：它已从 TopoRetarget 的优化变量中移除，
求解器只优化其余 20 个关节；对 URDF、Ruckig、日志和 H7 仍补回对应关节名，但数值在
左右手链路中始终强制为 `0 rad`。安全投影也不会把它当作可绕行关节。

实现保留了 MoveIt 最关键的规划语义：整手 planning group、状态/边离散碰撞检查、全局
路径搜索、碰撞检查后的 shortcut，以及控制器逐路点执行；同时采用 MoveIt Hybrid
Planning 的全局/局部分层思路，让 CBF 处理快速局部变化。碰撞场景仍使用当前项目自己的
Wuji/TsenseI MuJoCo 模型，因此没有引入 ROS 2、MoveIt 或新的大型运行时依赖。

## Ruckig、数据下发与一键找零

light_test 复用主项目的完整控制边界，顺序固定为：

`TopoRetarget → 自碰撞策略 → Ruckig(500 Hz) → UI角度转硬件角度 → DownlinkGuard
→ 四指闭环连杆/拇指行程映射 → H7 0x20整手批量帧`

`light_test` 使用多自由度、相位同步的 Ruckig。所有关节沿同一个规划线段使用共同进度，
避免 21 个独立 1-DoF 轨迹因完成时间不同而偏离已经验证过的无碰撞路径；主项目原有的
Ruckig 默认行为不受影响。

所以右手符号、`thumb_rota` 增益、14 路直线电机行程、7 路旋转电机角度、CRC 与
上下位机帧格式都来自项目现有实现。窗口里的 URDF 也显示 Ruckig 当前采样姿态，
而不是跳变的原始 retarget 目标。

实机操作顺序：刷新串口 → 连接 → 确认目标手 → 开始识别 → 勾选“允许实时下发”。
实时下发默认关闭；停止识别、识别线程异常、切换手型和开始找零都会自动解除使能。
“急停”复用项目 `UsbController.emergency_stop()` 下发整手安全下限帧并关闭连续下发；
其中 `thumb_rota`（R04）遵守 H7 的 `0.5°` 下限，`thumb_swing`（R05）下发其 `0°` 下限。

“一键找零”严格复用 Form2 流程：先向 14 台直线电机发送 `LM_HOMING_START`，等待
2 秒，再按 `SPI2 M1 → SPI2 M0 → SPI1 M1 → SPI1 M0 → SPI3 M0 → SPI4 M0 →
SPI4 M1` 逐个使能并找零。旋转电机只有观察到 `home` 边沿或等价的 Iq/位移遥测
证据才进入下一台；无遥测不会误报成功，15 秒超时后停止。完成后实时下发仍保持
关闭，需要人工重新勾选。找零过程中点击“停止找零”会向仍在执行的直线电机发送
`LM_HOMING_RESET`，或给当前旋转电机断使能，不只是停止界面计时器。

## 分支归属

当前目录属于本地 `light_pipeline` 分支，并跟踪远程 `origin/light_pipeline`。
