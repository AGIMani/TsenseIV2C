# D455f 深度融合 21 点输入

工作台和动态标定页新增 `D455f 深度融合 · 21 点`，来源 ID 为
`realsense_rgbd`。原有 `MediaPipe · 相机` 下选择 D455f RGB 的方式保留；
两者是独立入口。深度设备连接在运行 track 服务的 Ubuntu 主机上。

21 点沿用 MediaPipe 编号：0 为手腕，1–4 为拇指，5–8 为食指，
9–12 为中指，13–16 为无名指，17–20 为小指；各指按掌根到指尖排列。

## 使用

1. track 环境安装新依赖（新安装的 `requirements-track.txt` 已包含）：

   ```bash
   .venvs/track/bin/python -m pip install -r online_control/online_motion_track/requirements-realsense.txt
   ```

2. 原生 USB 访问需要 udev 权限。若此前没有 RealSense SDK 的设备规则：

   ```bash
   sudo install -m 0644 migration/ubuntu/70-hand-realsense.rules /etc/udev/rules.d/70-hand-realsense.rules
   sudo udevadm control --reload-rules
   ```

   重新插入相机。规则仅匹配 Intel RealSense USB 设备，给本机登录会话及
   `plugdev` 组读写权限。使用 USB 3 数据线；先停止其他占用这台相机的应用。

3. 启动 `bash migration/ubuntu/start-track.sh`，在输入来源中选择
   `D455f 深度融合 · 21 点` 后开始识别。也可打开
   `https://127.0.0.1:8766/?source=realsense_rgbd`。
   已运行的服务需停止识别后重启，以加载新增 Python 模块。
4. 查看相机采集/MediaPipe 识别帧率、深度有效点数，以及融合后的三维骨架。
   点数不足、相机断开或图像过期时显示不可用，不输出伪装成深度修正的单目结果。
5. 停止识别后，可切回 `MediaPipe · 相机`，在视频来源选择 D455f RGB。
   原有高帧率 RGB 能力查询和采集逻辑不变。动态标定页也支持独立选择新来源。

## 数据链路

`RealSense SDK RGB + Z16 → 同帧对齐 → 浏览器 MediaPipe → 后端深度拟合 → 21 点 → 原重定向`

- 后端用 `pyrealsense2` 原生采集；不依赖 Qt、OpenCV、Windows 或 WSL 桥接。
  只有显式读取新输入的图像时才打开设备，最后一个使用者退出后释放。
- 从 SDK 实际报告的 RGB8/Z16 模式中选取相同帧率的组合，优先 30 FPS，
  再选择接近 640×480 的分辨率。新 RGB-D 入口不承诺旧 RGB 入口的 90 FPS。
- SDK 将深度对齐到彩色帧。21 点反投影调用 SDK 自带函数，使用对齐后的内参，
  包括畸变处理；深度单位是米。
- `/api/realsense/frame` 返回 4 字节小端 JSON 长度、JSON 元数据、RGB8 图像。
  图像携带 `stream_id` 和 `frame_id`；浏览器把同一张图像交给已有 MediaPipe Worker。
- `/api/frame` 接收 MediaPipe 假设和图像身份。后端使用有界历史中对应的深度，
  不把“当前最新深度”套到之前识别的 RGB 图像上。同一会话不能重复消费同一帧。
- 深度来源重启、时间戳不一致、重复/过期图像均失效；有效期为 500 ms，
  融合和重定向结束后再次检查，过期结果不能提交控制目标。

## 21 点拟合方法和边界

`rgbd_fusion.py` 在每个二维关键点附近分层采样，排除明显背景和前景遮挡。
掌部有效深度用于估计手的距离与尺度，再沿各自相机射线联合优化 21 个关节深度。
目标函数包含观测表面区间、骨长、MediaPipe 相对姿态和腕部深度约束。
输出仍为 MediaPipe 顺序、腕部相对的 `(21, 3)` 米制坐标；XYZ 都处于同一相机轴向，
再由项目已有的腕部坐标转换送入 TopoRetarget。

深度观察的是皮肤表面，不能直接当成骨骼中心。当前使用 4–8 mm 的近似表面偏移、
±3 mm 区间，并将单关节深度修正限制在初始估计 ±45 mm。它们是启发式参数，
不是对特定用户手型的测量标定。当前至少要求 5 个可靠点，其中掌部至少 2 个；
支持的数值深度范围为 0.12–2.0 m，并不代表相机在整个范围内都有相同精度。

遮挡点依靠骨长与姿态先验补足，并在诊断中明确标为 `inferred`。因此“输出21点”
不意味着21点都由相机直接测得。严重侧视、握拳和相互遮挡仍可能丢失或估计错误；
该版本没有移植原仓库的完整手表面模型/关节角限制，也不能恢复所有隐藏关节。

## 提取的上游代码

来源是 `realsenseai/hand_tracking_samples`，固定版本
`75d5579aa258d97157d8a29b1c4da32f3d0a7608`。
`vendor/realsense-hand-tracking/` 保存原始 `deprojectz`、`PointCloud`、
`CloudConstraint(s)`、`FitPointCloud` 片段、文件哈希、LICENSE 和 COPYING。

反投影公式直接适配上游实现；21 关节有界求解器为当前项目新写，采用其
“深度观测 + 骨骼约束”的思路。没有使用旧 CNN 的8点输出，也没有复制旧的
右手模型或完整刚体求解器。开发说明与直接移植内容在同目录 README 中区分。

## 接口和记录

新帧来源是 `realsense_rgbd`，融合成功后 `depth_source=realsense_rgbd_fused`。
浏览器显示使用返回的 `fused_world`；后端重定向的 `points` 已转到腕部模型坐标，
两者不能混用。原 MediaPipe 归一化点仍用于 RGB 叠图。

`source_metadata` 保存设备/模式、内参、同帧身份、捕获时间、原始 MediaPipe
三维先验和图像点；`depth_fusion` 保存逐点有效性、推断标记、表面残差、
骨长误差、相对 Z 修正和耗时。记录足以比较融合前后骨架；当前不保存整幅
RGB/深度原始图像，不能仅凭现有骨架日志重新运行深度采样。

## 验证

自动测试覆盖合成深度的相对 Z 修正、左右手/尺度、骨长、空洞、背景异常、
交叉手指遮挡、时间戳/会话/帧身份、断线重连、停止期间在途帧、标定回滚、
HTTP 二进制传输、网页入口与普通相机回归。

```bash
.venvs/track/bin/python -m pytest tests/test_rgbd_fusion.py tests/test_rgbd_input.py tests/test_realsense_source.py tests/test_realsense_application.py tests/test_calibration_metric_commit.py -q
npm --prefix online_control/online_motion_track run test:realsense
npm --prefix online_control/online_motion_track run test:realsense-browser
```

浏览器测试需要项目的 Playwright Chromium，使用合成 RGB/深度和测试检测器，
实际经过网页、HTTP、融合和重定向。为避免无 GPU 环境下连续 CAD 绘制占用
主线程，测试仅在截图时绘制模型；生产渲染逻辑和 500 ms 有效期不变。

本次开发时 SDK 枚举到 0 台 RealSense，实机深度精度和整体帧率尚未验证。
合成数据通过不等同于 D455f 在实际手势/距离/光照下的精度保证。
