# MediaPipe 手部模型与实时滤波检索

检索与本地校验日期：2026-09-23。范围：Google 官方模型文档、MediaPipe 源码、GitHub 实现、Hugging Face 模型卡及相关论文。约束：继续使用 MediaPipe Hand Landmarker 识别。

后续已按用户新增的 5% 延迟增幅上限完成保守调参与计算优化，见 [实现与验收记录](20260923_landmark_filter_latency_budget.md)。下文参数与建议保留调研时状态；新增状态锁定与骨架约束未纳入本次实现。

## 结论

1. 在本次检索范围内，未找到公开可下载、能直接替换当前 HandLandmarker `.task`、且有可信精度提升证据的更强手部模型。
2. 本项目根目录的 `hand_landmarker.task` 与本次实际下载的 Google 官方 `latest` 文件逐字节相同。更换为该下载文件不会改变模型。
3. 继续以 One Euro 为实时滤波基线；优先评测异常观测处理、静止/运动滞回、手掌相对姿态与骨架软约束。Kalman 作为对照，而非未经验证的升级。
4. 检索可以确认实现与算法性质，不能证明本机整条控制链已经稳定。需要同时验证静止噪声、微动保真、延迟、丢手恢复和自动恢复连续性。

## 模型实物校验

本地文件：`D:/Qtworks/hand/hand_landmarker.task`。

官方来源：[Google 官方 latest 下载](https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/latest/hand_landmarker.task)。

| 项目 | 本地与官方下载的结果 |
|---|---|
| 大小 | 7,819,105 字节 |
| SHA-256 | `fbc2a30080c3c557093b5ddfc334698132eb341044ccee322ccf8bcf3607cde1` |
| 逐字节比较 | 相同 |
| 官方响应 Last-Modified | 2023-04-26 03:33:25 GMT |
| 包内模型 | `hand_detector.tflite`、`hand_landmarks_detector.tflite` |

下载副本与来源校验记录保存在 `.work/mediapipe_research_20260923/`，其中 `sources.json` 记录下载 URL、大小和散列值。服务端路由 `online_control/online_motion_track/main.py` 将 `/hand_landmarker.task` 映射到项目根目录文件；本次未验证远端部署或浏览器缓存的字节内容。

Google 当前 [Hand Landmarker 模型表](https://developers.google.com/edge/mediapipe/solutions/vision/hand_landmarker) 列出 HandLandmarker (full)，手掌与关键点网络输入分别是 192×192 与 224×224，模型包为 float16。没有在该表找到 Hand Heavy。人体 Pose 的 Heavy 不能替代手部 21 点模型。

### 检索到但不构成升级的候选

- [Leo-TX/mediapipe-hand 模型卡](https://huggingface.co/Leo-TX/mediapipe-hand) 明确说明来源是 Google 官方同一个下载 URL，属于再分发。
- [Qualcomm MediaPipe-Hand-Detection](https://huggingface.co/qualcomm/MediaPipe-Hand-Detection) 提供面向设备部署的 ONNX、QNN、TFLite 导出；这不是已验证更精准、可直接交给当前浏览器 HandLandmarker 的 `.task` 包。更大的文件也不等于更好的精度。
- [MediaPipe Gesture Recognizer 定制教程](https://developers.google.com/edge/mediapipe/solutions/customization/gesture_recognizer) 先提取已有手关键点，再训练手势类别识别。因此自定义 `gesture_recognizer.task` 的分类准确率提升，不能当作 21 点定位网络精度提升。
- [旧版 Hands 文档](https://github.com/google-ai-edge/mediapipe/blob/master/docs/solutions/hands.md) 的 `model_complexity` 为 0/1；当前 Web Tasks 的 [选项表](https://developers.google.com/edge/mediapipe/solutions/vision/hand_landmarker/web_js) 没有该参数，也没有手部 `model_complexity=2` 的升级开关。

## 官方滤波源码核对

[LandmarksSmoothingCalculator 配置](https://github.com/google-ai-edge/mediapipe/blob/master/mediapipe/calculators/util/landmarks_smoothing_calculator.proto) 提供 NoFilter、VelocityFilter 和 OneEuroFilter。

本次查阅的 [HandLandmarkerGraph](https://github.com/google-ai-edge/mediapipe/blob/master/mediapipe/tasks/cc/vision/hand_landmarker/hand_landmarker_graph.cc) 与 [HandLandmarksDetectorGraph](https://github.com/google-ai-edge/mediapipe/blob/master/mediapipe/tasks/cc/vision/hand_landmarker/hand_landmarks_detector_graph.cc) 没有接入上述关键点平滑器。VIDEO 模式具有手部区域跟踪与重检测流程，不能因此推断输出的 21 点已做 One Euro 平滑。Web Tasks 选项也没有 `smooth_landmarks`。

这个判断针对所查官方源码与公开 API，不表示已反编译核对项目当前 WASM 的所有内部实现。

官方平滑组件的速度尺度可以按对象尺寸调整。项目 A3.0 使用米制坐标，并采用每点统一三维权重、速度噪声门槛与截止频率上限。因此，网上像素坐标或归一化坐标示例的 beta 不能直接搬到这里。

## 滤波方案比较

以下适用性为针对本项目的工程判断，不是已完成的算法排名实验。

| 方法 | 适用性 | 主要局限 |
|---|---|---|
| One Euro | 优先保留；单帧递推、使用真实时间间隔，速度变化时可调整平滑强度 | 不能单独区分真实快速动作与识别离群点；不会自动保持静止姿态 |
| Relative Velocity Filter | 官方可复现对照；由历史窗口速度调整低通权重 | 加大窗口会增加延迟，不保证比 One Euro 更适合精细手指控制 |
| Kalman / 自适应 Kalman / EKF | 可估计位置、速度与不确定度；适合与运动学约束结合评测 | 效果依赖状态模型和 Q/R；急停、换向、遮挡预测可能产生误差 |
| EMA / 移动平均 | 简单基线，易复现 | 固定平滑强度兼顾不了慢速抑抖和快速跟随；长窗口会增加延迟 |
| 因果中值 / Hampel 异常检测 | 适合辅助识别孤立跳点；只使用历史窗口 | 真实快速运动也可能被误判；不能把每帧都强制中值替换 |
| Butterworth / Savitzky–Golay | 离线分析有用；因果版本可作为基线 | 在线实现有相位/响应代价；常见双向或居中实现依赖后续数据 |
| SmoothNet 等学习式时序平滑 | 可作为进一步研究方向，识别前端仍可为 MediaPipe | 公开预训练主要面向人体姿态，窗口与手部数据适配需要单独验证 |

来源与证据：

- [One Euro 作者页面](https://gery.casiez.net/1euro/)：先在 beta=0 时调静止截止频率，再通过运动调 beta；降低截止频率会增加延迟。
- [官方 RelativeVelocityFilter 实现](https://github.com/google-ai-edge/mediapipe/blob/master/mediapipe/util/filtering/relative_velocity_filter.cc)：历史位移和时间决定速度与平滑权重。
- [OpenCV KalmanFilter 文档](https://docs.opencv.org/4.x/dd/d6a/classcv_1_1KalmanFilter.html)：预测/校正以及过程、测量噪声协方差。其存在不能证明某组默认参数适合本项目。
- [Hand-Landmark-Tracker](https://github.com/Jshulgach/Hand-Landmark-Tracker)：公开 MediaPipe + Kalman 实现示例；本次未执行该项目，也没有采用其稳定性宣传作为本项目实测结果。
- [SciPy filtfilt](https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.filtfilt.html)：前后双向滤波；离线零相位效果不能当作无延迟在线效果。
- [SciPy savgol_filter](https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.savgol_filter.html)：滑动窗口多项式滤波；使用居中窗口时不能忽略等待后续样本的代价。
- [SmoothNet 官方实现](https://github.com/cure-lab/SmoothNet)：提供 8/16/32/64 帧窗口，默认窗口 32 帧，任务为人体 2D/3D 姿态与 SMPL。窗口长度不等于固定在线延迟，是否使用未来帧必须按部署方式核验。
- [MediaPipe + ZED2 的 RGB-D 研究](https://www.mdpi.com/1424-8220/26/12/3730)：报告中 One Euro 的平滑指标更好，但动作路径衰减也更明显，作者选择了更折中的 EMA。该结果来自 RGB-D 离线序列，不能直接复制到当前单目手部控制，也不能据此宣称 EMA 普遍更好。此次网页正文打开受限，相关结论依据检索工具返回的该论文正文摘录。

## 本项目建议

当前 `ResponsiveLandmarkFilter` 参数为 min_cutoff=1.2 Hz、beta=80、derivative_cutoff=1 Hz、noise_speed=0.01 m/s、max_cutoff=15 Hz。noise_speed 仅从估计速度中扣除噪声底，不是位置死区或静止锁定。

建议按以下顺序做增量对照，避免一次叠加所有滤波导致无法定位效果来源：

1. 以 retarget 自动恢复保留状态的修复版为基线，使用真实帧时间戳，保持同一只手的状态连续。
2. 在速度估计之前增加异常观测处理。结合骨长突变、相对手掌的变化和时间连续性，避免孤立跳点让 One Euro 误判为快速动作。被拒绝的观测不能污染速度历史；连续拒绝也需要重新捕获策略，防止永久冻结。
3. 加入静止/运动状态和滞回。进入静止需要持续低运动证据，退出阈值与进入阈值不同；检测低速累计位移，避免吃掉缓慢的小动作。阈值从数据估计，而非直接抄光标控制中的像素死区。
4. 平滑状态切换与重新捕获，不把锁定输出瞬间跳到最新观测。丢手处理应保持有界，不用长期预测继续驱动机器人。
5. 单独评测手掌相对坐标、骨方向及骨长软约束；保留使用者的手部比例。手掌坐标系本身也需要稳定，退化几何不能直接参与归一化。
6. 若目标关节仍抖动，再评测 retarget 连续性正则或小幅关节静止保持，并将其置于最终碰撞检查与轨迹规划之前。

不采用“一律提高 confidence”“一律增加滤波窗口”“把 Z 轴缩小”作为精度修复。HandLandmarker 的 handedness 分数是左右手分类分数，不能当作每个关键点的定位置信度。

## 稳定性验收

必须同时报告以下结果，不能仅以曲线更平滑判断成功：

- 标记静止动作的各关键点与目标关节标准差、峰峰值；原日志自动选出的区间只作为疑似静止。
- 缓慢小幅弯曲、捏合的幅度保留与释放延迟，防止过大死区。
- 快速弯曲、换向、急停的响应时间、超调与单帧步长。
- 单点离群、低帧率、时间戳异常、遮挡、丢手重获、左右手切换。
- 自动轨迹恢复与不中断处理的同输入对照，保留已修复的连续性。
- 滤波计算成本和端到端 P95 延迟，检查轨迹流是否仍积压。
- 继续通过关节限位、速度/加速度与碰撞路径检查。已有异常遥测在修复解析前不能作为真机角度精度的证据。

本轮完成检索、官方源码核对和模型字节校验，没有新增在线滤波实现或真机稳定性实测。
