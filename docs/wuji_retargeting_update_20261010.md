# Wuji Retargeting 更新与 MediaPipe 关键点对比

核查日期：2026-10-10（Asia/Shanghai）。当前输入对比以本项目网页手势工作台为主，并单列 V2 和 `WujiMediaPipe21` 模块，避免把不同路径当成同一处理链。

## 更新结果

- `external/wuji-retargeting`：`2026.6.15` → `2026.8.17`。
- 上游 tag：`v2026.8.17`；提交：`531f6ed4250b475d2e9231f54e988fc9b1c5b4ea`，核查时与 `main` 相同。
- `wuji-description` 按该版本锁定到 `8271644a78d69ed9a4adcf9165d882c64ad33dfa`（`v2026.8.3`）；两个模拟器相关子模块也按锁定提交完整取回。
- 保持原迁移包的源码快照形式，不含 `.git`。完整来源、子模块提交、保留文件哈希见 [来源记录](../migration/ubuntu/wuji-retargeting-source.json)。
- 更新前逐文件对照旧 tag：上游文件只有换行符差异和已经展开的 Git LFS 数据，没有发现上游文件被本地修改。保留了本地额外的物理滤波示例，以及三个自定义左手 URDF/MJCF 文件。
- `example/data/avp1.pkl` 保留真实的 1,689,467 字节内容，并核对上游 LFS SHA-256，未将其替换成指针文本。
- 原目录完整备份：`runtime/ubuntu/backups/wuji-retargeting-v2026.6.15-20261010-115215`。

本次更新外部源码与模型，没有更换网页重定向算法或安装 Python 依赖。track、play 保持独立且均无 Qt。当前网页运行链未导入外部 `wuji_retargeting` 包，而是使用项目内的 MediaPipe 处理函数和 `TsenseI_retargeting.TopoRetargeter`。

官方 [发布页](https://github.com/wuji-technology/wuji-retargeting/releases/tag/v2026.8.17) 和 [更新日志](https://github.com/wuji-technology/wuji-retargeting/blob/v2026.8.17/CHANGELOG.md) 显示：此期间主要调整硬件命名、SDK 资源 API 兼容、手套旋转标定及 Hand 2 模型路径。旧版到新版的 `example/input_devices/`、`wuji_retargeting/mediapipe.py`、`wuji_retargeting/retarget.py` 没有代码变化；优化器中的变化只有名称注释。官方仓库在 2026-08-31 已归档。

## 核心区别

两边都采用相同的 MediaPipe 21 点编号：0 为腕，1–4 为拇指，5–8、9–12、13–16、17–20 分别为其余四指。区别主要发生在选用哪一组三维坐标，以及后续如何改变尺度、骨长和时间响应。

| 步骤 | 官方 Wuji 视频 / RealSense / ZED 输入 | 当前网页 A3.2 / A3.0 输入 |
| --- | --- | --- |
| 检测器 | Python `mp.solutions.hands.Hands`，最多 1 只手 | 浏览器 Tasks `HandLandmarker`，最多 2 只手，GPU 失败可回退 CPU |
| 三维数据 | `multi_hand_landmarks` 图像归一化点，包括相对深度 z | `worldLandmarks` 的米制三维估计；同时保留 `landmarks` 用于画面叠加、记录等 |
| 深度处理 | x 乘图像宽，y 乘图像高，z 乘图像宽再乘 `2.5` | 三维 world 点不乘图像尺寸，也不乘 `z_scale` |
| 全局尺度 | 腕 0 到中指 MCP 9 强制归一到 `0.09 m` | 关键点预处理保留尺度；TopoRetarget 内部再以 5–17 掌宽对齐所选机器人的标定掌宽 |
| 指骨长度 | 默认把每指三段骨长替换成固定 AVP 参考值，保留段方向 | 不执行 Wuji 固定骨长替换；整体掌宽缩放保留当前观测的骨长比例 |
| 腕部坐标 | 0、5、9 三点估计掌面，SVD + 左右手 MANO 矩阵 | 相同的坐标变换公式；前面增加有限值、退化骨段、掌面检查 |
| 额外旋转 | 视频 YAML 默认额外旋转 x=-5°、y=-5°、z=-15°；可选腕/拇指偏移 | A3 网页预处理没有应用这组 Wuji 视频补偿参数，使用各自模型及标定 |
| 滤波 | 相机适配器无显式 One Euro 点滤波；求解关节角后做 LPFilter，视频默认 alpha=0.15 | world 点先做按速度调整的 One Euro 变体，再交给 TopoRetarget；后续运动控制是独立阶段 |
| 左右手 | 默认交换检测标签；找不到目标手时使用第一只检测到的手 | 按原始 Tasks 标签选手；即使前端退回第一只手，后台仍检查标签，不让错误手进入求解 |
| 丢手 | 输入适配器复用上一次有效关键点 | 当前帧标记失去跟踪，保持上一姿态并重置输入滤波；恢复时重新处理有效帧 |
| 重定向 | 视频配置使用 `AdaptiveOptimizerAnalytical`，包括位置/方向/捏合目标及逐段缩放参数 | `TopoRetargeter` 使用机器人标定点、骨方向、拓扑及捏合约束；输出不同不能只归因于 MediaPipe |

上游 RealSense 适配器只启用 RGB 流，ZED 适配器显式关闭深度计算；这里的“深度处理”不是使用硬件深度图。Tasks 的 world landmarks 也是模型估计，不代表单目相机取得了深度传感器测量。[MediaPipe 输出坐标定义](https://developers.google.com/edge/mediapipe/solutions/vision/hand_landmarker/python#handle_and_display_results)

官方固定指骨长度如下，单位为毫米；这些参数用于输入骨段修正，优化器 YAML 中还有独立的 `segment_scaling`。

| 手指 | 第 1 段 | 第 2 段 | 第 3 段 |
| --- | ---: | ---: | ---: |
| 拇指（1→2→3→4） | 50.5 | 31.8 | 30.2 |
| 食指 | 41.8 | 24.3 | 22.3 |
| 中指 | 48.9 | 28.9 | 22.7 |
| 无名指 | 42.2 | 27.4 | 22.7 |
| 小指 | 34.3 | 19.5 | 20.1 |

### 本项目其他路径

- **V2 网页**：也使用浏览器 world landmarks，但随后调用 `preprocess_wuji_world_points`，仍有 9 cm 归一化和固定骨长修正，再进入 MANO 和 TopoRetarget。它与 A3.2/A3.0 在骨长处理上不同；world 路径同样不使用 `z_scale=2.5`。
- **`src/tracking/wuji_mediapipe.py` 的 `WujiMediaPipe21` 默认输出**：优先使用 Tasks world 点，先做 One Euro，再做 9 cm/固定骨长处理；仅在没有 world 点时使用归一化 z 回退。`frame.world_points` 是滤波后的 world 点，`frame.points` 则是经过 Wuji 尺度/骨长处理、尚未进入 MANO 的点，两者不可混淆。
- **标定页**：使用 world 点和掌框架对齐，按源/目标掌宽整体缩放，保留骨长比例；入口为 `align_world_landmarks`。
- 上游 `MediaPipeSmoother` 虽然定义了五帧加权平滑，但调用位置是 Vision Pro 适配器，不能据此说官方视频输入也使用该平滑器。

当前 A3.2 world 点滤波参数为 `min_cutoff=1.5`、`beta=82`、噪声速度门限 `0.02 m/s`、最大截止频率 `15 Hz`；A3.0 对应 `1.17 / 82 / 0.01 m/s / 15 Hz`。V2 和 `WujiMediaPipe21` 默认使用 `1.5 / 0.25 / derivative_cutoff=1.0` 的原始 One Euro 实现。不同链路的平滑响应也会造成“跟手程度”差别。

## 如何理解或衔接这些输出

以当前 A3.2/A3.0 网页为例，实际链路是：

```text
摄像头图像 → 浏览器 Tasks → worldLandmarks[21,3]（米）
→ 左右手/时间戳检查 → 自适应点滤波 → 腕部/MANO 坐标
→ 按机器人掌宽整体缩放 → TopoRetarget → 关节角与后续控制
```

官方视频链路是：

```text
视频图像 → Python Solutions → normalized landmarks[21,3]
→ 像素缩放与 z×2.5 → 腕归零 → 0–9 距离统一为 9 cm
→ 固定指骨长度 → 腕部/MANO 坐标 → YAML 旋转/偏移
→ Wuji 优化器 → 关节角低通
```

若以后把当前点送入 Wuji Retargeter，应明确输入阶段：`hand.world` 不应再执行像素/z 放大；网页返回的 `result.points` 已经过 MANO，不应再次作为原始点让 Retargeter 重做同样变换。官方视频配置按它自己的尺度与骨长处理调参，换成原始 world 点后需要重新验证参数。

此次代码检查及数值测试能证明处理公式是否相同，不能据此判断哪条链路的真实手势精度更高。需要同一批录制图像/关键点与目标姿态进行误差和延迟评估。

## 代码定位

- 上游像素/z、尺度和骨长：[video_mediapipe.py](../external/wuji-retargeting/example/input_devices/video_mediapipe.py)，`_landmarks_to_array` / `_process_landmarks` / `_correct_segment_lengths`。
- 上游腕变换：[mediapipe.py](../external/wuji-retargeting/wuji_retargeting/mediapipe.py)；附加旋转及角度滤波：[retarget.py](../external/wuji-retargeting/wuji_retargeting/retarget.py)；默认值：[视频 YAML](../external/wuji-retargeting/example/config/adaptive_analytical_video.yaml)。
- 网页检测与发送：[detector.worker.js](../online_control/online_motion_track/static/detector.worker.js)、[app.js](../online_control/online_motion_track/static/app.js) 的 `processFrame`。
- 网页接收与滤波：[main.py](../online_control/online_motion_track/main.py) 的 `WebSession.process`、[landmark_filter.py](../online_control/online_motion_track/landmark_filter.py)。
- A3 点处理：[pipeline.py](../online_control/online_motion_track/pipeline_v3_2/pipeline.py) 的 `prepare_points` / `process`；A3.0 继承此处理。
- V2 点处理：[session.py](../online_control/online_motion_track/v2/session.py)。
- 项目内 Wuji 点模块：[wuji_mediapipe.py](../src/tracking/wuji_mediapipe.py)；共用腕变换：[mediapipe_frame.py](../src/utils/mediapipe_frame.py)。
- 机器人掌宽归一：[toporetarget.py](../TsenseI_retargeting/toporetarget.py) 的 `_source_points_in_robot_scale`。

## 验证

更新后逐文件核对 477 个上游文件（包含递归子模块）：无缺失、无内容不符；另有 4 个有哈希记录的本地扩展。展开的 LFS 数据按对象哈希单独验证。

更新前后相同回归均通过：track 网页与输入相关 18 个 Python 测试文件为 **314 passed、4 skipped**；play 目录完整 Python 测试为 **365 passed**；track 与 shared 的 32 个 JavaScript 单元测试文件为 **274 passed**。合计 **953 passed、4 skipped**。跳过项均依赖可选的 Python 本地摄像头/OpenCV/MediaPipe，浏览器关键点链路已经覆盖。

测试使用独立的 track/play 解释器。Node 18 运行时需添加 `--experimental-default-type=module`，并通过 `HAND_PYTHON` 与 `PATH` 指向 track Python；初次缺少这些运行参数导致的调用错误，在补齐参数后全部通过，没有修改测试或生产代码。

日志、测试文件清单、命令、文件一致性结果及数值证据保存在 [本次验证目录](../runtime/ubuntu/validation/wuji-retargeting-20261010-115215/)。这是相关网页/输入及播放回归，不代表运行了整个仓库全部历史测试或相机硬件端到端测试。

数值对照直接执行上游三个几何方法的源码和项目函数：40 组非退化合成手形、3 种图像尺寸，共 120 组归一化输入结果完全一致；左右手各 40 组腕部/MANO 转换也完全一致，最大绝对差均为 0。

在已有独立 `.venvs/wuji-mediapipe` 环境中，Wuji Hand 与 Wuji Hand 2、左右手共 4 组新版本配置均能完成真实 IK 求解，返回有限且在关节限位内的 20 个角度。没有安装或修改环境依赖。

兼容边界：上游官方相机示例仍调用旧 `mp.solutions` API；本机独立环境的 MediaPipe 1.1.0 已不提供该属性，因此不能把上述 IK 冒烟测试理解为官方相机示例已可直接运行。当前网页使用浏览器 Tasks，不经过该旧 Python 相机入口。实际相机/机械手精度未在本次检查中测量。
