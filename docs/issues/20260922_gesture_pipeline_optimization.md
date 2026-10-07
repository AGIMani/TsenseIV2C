# A3.0 手势帧链路与滤波优化

后续变更：本报告保留当时的 Ruckig 参数对照；之后已按用户要求对齐 Qt 默认约束，当前值见 [Ruckig 参数修改记录](20260922_a30_ruckig_qt_limits.md)。

本次修改 `online_control/online_motion_track`，完成浏览器帧链路解耦、A3.0 米制关键点滤波替换、部署包补齐与离线验证。没有重启正在使用的服务，没有开启相机/串口，没有部署 node0。Ruckig 约束保持本次修改开始时的值。

## 1. 为什么原来约 67 ms 才更新一次

相机配置是 640×480、30 fps，上限每 33.3 ms 一帧。原页面串行执行：视频帧 → MediaPipe Worker → `/api/frame` 求解/避碰 → 更新模型和日志 → 预约下一视频帧。已分析录像中，推理中位约 16 ms，推理结束到目标响应约 20.2 ms；两段串行超过 33.3 ms，常错过下一次视频回调，形成约 67 ms 的有效目标间隔。

改动：

- 推理结束后立即预约下一视频帧，下一帧识别与上一帧求解重叠。
- 同时最多一个识别任务、一个求解请求；求解忙时最多保留一个最新候选，覆盖旧候选。不会并发修改同一求解会话。
- 待提交帧超过 120 ms 时丢弃，不补发历史姿态。此限制用于提交前，不能截断已经开始的慢请求，也不是端到端延迟保证。
- 停止、切换模型后关闭队列，以会话代数拦截迟到响应；丢手帧仍可替换待处理的有手帧。
- world/normalized 坐标每帧只转换一次，识别记录与请求复用数组。
- 诊断记录增加队列计数；`control_target.pipeline_timing` 增加抓帧调用到响应、等待提交、请求响应三个耗时。抓帧时刻来自浏览器 `performance.now()`，不含曝光前的相机延迟。

相机配置仍是 30 fps。本次提高有效目标吞吐量，没有将硬件相机帧率强制提高到 60 fps。

### 调度模拟结果

使用生产 `LatestFrameQueue` 和虚拟时钟，每个场景运行模拟 10 秒；30 fps 相机、16 ms 推理。不是浏览器实拍或真机端到端测量。

| 求解及响应耗时 | 原串行目标间隔中位 | 并行目标间隔中位 | 并行同帧抓取至结果中位 |
| --- | ---: | ---: | ---: |
| 20 ms | 66.7 ms | 33.3 ms | 36.0 ms |
| 50 ms | 66.7 ms | 50.0 ms | 82.7 ms |
| 180 ms | 200.0 ms | 180.0 ms | 216.0 ms |

20 ms 场景把吞吐量约 15 Hz 提高至 30 Hz，单帧计算总耗时仍是 36 ms。慢请求场景存在一个候选帧的等待，单帧年龄可能比串行略大；最新帧覆盖机制避免等待随运行时间不断增长。浏览器负载、相机实际帧率、求解尾部耗时会影响实际结果，不能把 33.3 ms 当作保证。

## 2. Qt Form2 与当前 A3.0 网页 Ruckig 对照

按源码默认值比较，Qt 的 `RUCKIG_*` 环境变量可覆盖默认值；没有读取一个正在运行的 Qt 进程的实际参数。不同模型关节结构并不完全相同，按功能类别对照。

| 关节类别 | Qt 最大速度 rad/s | 网页最大速度 rad/s | Qt 最大加速度 rad/s² | 网页最大加速度 rad/s² | 加速度变化 | Qt jerk rad/s³ | 网页 jerk rad/s³ |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 手指弯曲 MCP/PIP/DIP | 4.0 | 2.5 | 30 | 15 | -50% | 200 | 120 |
| 四指侧摆 swing | 1.5 | 1.5 | 10 | 15 | +50% | 80 | 120 |
| 拇指旋转 rotation | 2.5 | 2.5 | 20 | 15 | -25% | 120 | 120 |
| Qt 拇指 swing | 2.0 | 无直接对应类别 | 15 | — | — | 100 | — |

Qt Form2 由 `src/ui/form2_window.py::_setup_form2_ruckig` 构造 `RuckigJointInterpolator`，每关节一个 1-DOF 实例；约束在 `src/utils/ruckig_interpolator.py::_DEFAULT_CONSTRAINTS`。另有 Qt 避碰路径使用 Phase 同步类，不能将“独立关节”概括到所有 Qt 路径。

当前网页 A3.0 在 `a30/control.py` 调用 `configure_ruckig(acceleration=15., jerk=120.)`；`online_control/shared/downlink.py` 设置侧摆速度 1.5、其余 2.5，以及整手 20 轴 Time 同步。Qt 和这个手势网页的默认规划周期都是 2 ms / 500 Hz。此前动作播放项目的 1 ms 优化不能代入这里。

弯曲的速度、加速度、jerk 均比 Qt 默认低，再加整手同步，会影响跟手响应。目标末速度/末加速度均为零；当前运动状态持续传递，不是每帧重新从静止起步。本次只做对照，没有提高这些约束。此前录像的目标到规划输出约 255–285 ms 相位滞后，不能靠本次帧链路和滤波修改保证消除。

## 3. 滤波选择与结果

比较了 11 个配置：原 One Euro、4 Hz 固定低通、恒速 α-β 预测、四档普通 One Euro beta、四档带静止噪声门限的向量 One Euro。选择后者 beta=80 的配置作为当前 A3.0 默认方案，不声称对所有相机、动作和噪声都最优。

参数及行为：

- world 坐标单位为米；最低截止 1.2 Hz、beta=80、速度估计低通 1 Hz。
- 每个关键点使用三维速度模长调节截止频率，三个坐标共用权重，避免坐标轴方向影响响应。
- 速度模长先减 0.01 m/s 噪声门限，再计算动态截止，最高 15 Hz。静止门限只控制截止频率，不冻结位置。
- 连续因果滤波，无未来样本、无外推；每步输出位于前次输出和本次测量之间，不产生外推型阶跃过冲。
- 丢手、换手、跟踪间断超过 300 ms 时复位；非有限坐标、尺寸错误和不递增时间戳被拒绝。
- 只替换 A3.0 的 world 分支，其他模型及 normalized 诊断支路保持原滤波。

One Euro 的方法是静止时低截止抑制抖动，快速运动时提高截止减少滞后，作者也明确要求按输入尺度调整参数。[作者说明及论文入口](https://gery.casiez.net/1euro/)

### 真实录像离线回放

| 数据 | 有效左手帧 | 旧滤波小指 PIP/DIP 相位滞后 | 新滤波小指 PIP/DIP 相位滞后 |
| --- | ---: | ---: | ---: |
| 20260922_125239_139000 | 403 | 65 / 60 ms | 10 / 5 ms |
| 独立验证：20260921_203818_978000 | 533 | 35 / 30 ms | 25 / 25 ms |

保持各录像原始时间戳，只重放输入滤波；输出弯曲角相对原始关键点弯曲角作相关性估计，搜索 0–250 ms、5 ms 网格。本报告排除了跨丢手/复位区间的插值，所以第一段旧滤波估计与之前未分段分析的 35–55 ms 不完全相同。5 ms 是搜索网格，不代表测量精度达到 5 ms。原始 MediaPipe 本身不是真实手部运动真值，录像没有独立静止真值，因此不据此宣称真实静止抖动下降。

滤波运算耗时中位约 16 → 20 微秒。此次“更快”是响应滞后减少，计算量略有增加；微秒级计算不是原来几十毫秒滞后的来源。

### 有真值的合成基准

30 Hz，0.5 mm 独立高斯噪声、25 mm 幅值 / 0.8 Hz 正弦运动，以及独立的干净 30 mm 阶跃。所有配置使用相同噪声序列。

| 算法 | 运动相位滞后 | 静止 RMSE | 运动 RMSE | 阶跃外推过冲 |
| --- | ---: | ---: | ---: | ---: |
| 原 One Euro | 90 ms | 0.184 mm | 4.547 mm | 0 |
| 固定 4 Hz | 40 ms | 0.271 mm | 1.967 mm | 0 |
| α-β 恒速预测 | 15 ms | 0.352 mm | 0.863 mm | 1.354 mm |
| 普通 One Euro beta=80 | 25 ms | 0.203 mm | 1.401 mm | 约 0 |
| **采用的向量自适应方案** | **30 ms** | **0.167 mm** | **1.736 mm** | **0** |

采用方案在该基准的静止 RMSE 比旧方案低约 9%，运动 RMSE 低约 62%；15/60 Hz 和另一个随机种子的回归测试也改善。预测方案虽滞后更小，静止噪声及过冲更大，因此未采用。阶跃无过冲不代表能拒绝 MediaPipe 的错误关键点；最终体验及碰撞行为仍需真实相机验证，原碰撞保护继续工作。

## 4. 验证与复现

已通过：Python 21 项（10 项新滤波/部署/会话测试，8 项原 V3.2 会话/HTTP 测试，3 项 A3.0 API 与运动约束回归）；Node 10 项（6 项最新帧队列，4 项实际页面帧处理函数的模拟相机/Worker 测试）；3 个调度模拟场景；修改后的 JS 语法检查。没有执行实机测试。

HTTP 测试初次被本机代理转发返回 502，测试子进程设置 `NO_PROXY=127.0.0.1,localhost` 后通过；系统代理配置未改。pytest 使用本项目 `logs` 下独立临时目录。

```powershell
$env:NO_PROXY='127.0.0.1,localhost'
& E:/conda/envs/hand_dex/python.exe -m pytest tests/test_responsive_landmark_filter.py tests/test_web_v3_2.py -q
& E:/conda/envs/hand_dex/python.exe -m pytest tests/test_a30_control.py -k 'public_a30_api_never_opens_server_usb or qt_aligned_gesture_trajectory_respects_derivative_limits' -q
& 'C:/Program Files/nodejs/node.exe' --test online_control/online_motion_track/tests/latest-frame-queue.test.mjs online_control/online_motion_track/tests/frame-processing.test.mjs
& 'C:/Program Files/nodejs/node.exe' online_control/online_motion_track/tests/benchmark-frame-pipeline.mjs
& E:/conda/envs/hand_dex/python.exe tools/benchmark_gesture_filters.py --input C:/Users/ONKIALL/Downloads/hand_control_20260922_125239_139000.jsonl --output logs/gesture_pipeline_20260922/filter_metrics.json
```

完整数据：`logs/gesture_pipeline_20260922/filter_metrics.json`、`filter_holdout.json`、`frame_pipeline_metrics.json`。

代码已就位；正在运行的 Python 服务不会自动加载新滤波。结束当前控制会话后，重启本地手势服务并刷新网页才能完整生效。下一次真实记录可用新增的时间和队列指标核对目标间隔、排队和滤波效果。本次没有更改分支、提交或推送其他会话的工作。
