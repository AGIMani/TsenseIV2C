# V3.2 MediaPipe 实时手势

独立入口：`online_control/online_motion_track/pipeline_v3_2/main.py`。
网页入口为 `online_control/online_motion_track/main.py`，使用浏览器 GPU 识别和 WebGL 原版 URDF 渲染，详见 [网页配置、node0 部署与内网访问手册](../online_control/online_motion_track/DEPLOY.md)。
与原 `online_control/online_motion_track/pipeline_v2/` 并列，用目录后缀切换版本；原 V2 程序保持原有行为。
新增接入代码都在 `online_control/online_motion_track/pipeline_v3_2/`：

- `main.py`：相机画面、三维骨架、原版 URDF 显示与启动/停止。
- `worker.py`：MediaPipe 检测、相机线程和只保留最新帧的消息缓冲。
- `pipeline.py`：所选手验证、三维坐标处理、V3.2 求解与碰撞处理。
- `skeleton_angles.py`：直接由 21 点计算骨架几何角度。
- `recording.py`：逐帧 JSONL 骨架记录及 CSV 角度对照。
- `analyze_middle.py`：中指角度曲线、伸指响应统计、原参数复现和当前参数回放。

## 启动

```powershell
conda activate hand_dex
cd D:\Qtworks\hand
python online_control/online_motion_track/pipeline_v3_2/main.py --hand-side left --camera 0
```

右手：

```powershell
python online_control/online_motion_track/pipeline_v3_2/main.py --hand-side right --camera 0
```

窗口自动启动识别。停止后可更换左右手、相机编号和碰撞模式，再点“开始跟随”。
每次启动重新读取对应手别的标定文件，并创建独立求解器。
摄像头编号通常从 0 开始，也可使用 `--camera 1` 或本地视频文件路径。
`--no-autostart` 只打开窗口，由用户手动开始。

资产固定为 `assets/hands/TsenseI_hand_v3_2/`，模型使用完整原版 STL，无减面。
当前源文件为用户于 2026-09-17 确认的 `TsenseI_v3_2/urdf/A3.1V6 26.8.4-2.urdf`。
模型替换后已重建标定草稿，请先用三维标定工具确认点位，再评估实时跟随效果。
左手读取 `robot_keypoints_21.yaml`，右手读取 `robot_keypoints_21_right.yaml`。
修改左手标定后，如需右手继续采用镜像标定，执行：

```powershell
python tools/prepare_hand_v3_2.py --sync-right-keypoints
```

该命令会备份并覆盖右手标定；实时入口自身不修改任何标定文件。

## 数据路径

未镜像相机画面 → MediaPipe Tasks HandLandmarker → 检查所选手别 →
三维关键点平滑 → 腕坐标旋转 → `HandModelSession("TsenseI_v3_2", side).retarget()` →
20 个 URDF 弧度关节角 → 原版 URDF 显示。

默认使用项目现有相机路径的直接 Left/Right 标签，`swap_handedness=False`。
优先采用过滤后的 `world_points`；关闭 Wuji 固定骨长修正，掌宽尺度由 TopoRetarget 处理。
没有 world landmarks 时，沿用原检测器的归一化 Z 回退，状态栏明确提示其深度精度较低。
左右手使用各自模型求解，不经过 V2 的共享左手求解器或额外镜像。

没有检测、仅检测到另一只手、使用缓存检测、三维点无效或求解失败时保持最后模型姿态。
停止后保持最后姿态；重新启动创建新的跟踪和求解状态。
窗口显示当前标定文件、求解耗时和接触数。模型和骨架支持鼠标旋转、缩放。
“重置视角”会按模型实际包围盒重新取景。加载完成后自动按 V3.2 掌部方向居中。

为确保完整模型可见，共用的 ModernGL 显示模块同时修正了透视矩阵的两项转置
及高 DPI 视口像素尺寸；该修改只涉及显示，不改变 V2 控制或重定向逻辑。

## 碰撞

默认 `--collision monitor`：检测接触、按原始重定向角度显示；可勾选“显示胶囊”。

```powershell
python online_control/online_motion_track/pipeline_v3_2/main.py --hand-side left --collision filter
```

`filter` 使用 V3.2 MuJoCo 碰撞代理修正角度，可能改变跟随姿态。
它是简化几何的仿真修正，不保证所有完整网格都没有穿插。
`--collision off` 关闭碰撞计算与胶囊显示。

本入口不导入 V2 控制流水线，不连接 USB，不调用硬件下发，不读写动作角度文件。
真机仍通过 `HandModelSession` 中预留的 adapter 接口单独对接。

## 逐帧骨架与角度记录（默认开启）

原启动命令即可记录，无需额外参数。每次“开始跟随”创建一组新文件，停止/关闭时写入结束记录。
默认保存到 `online_control/online_motion_track/pipeline_v3_2/logs/`；完整路径显示在窗口和启动终端：

- `session_时间戳_标识.jsonl`：会话配置、标定文件内容及 SHA-256、逐帧数据和结束原因。
- 同名 `.angles.csv`：同一帧的人手骨架角、retarget 角、最终显示角，方便绘制角度曲线。

每帧包含 `frame_id`、采集时间 `capture_timestamp_unix_ns`、左右手标签、有效性/保持状态。
记录位于采集工作线程，界面跳过中间显示帧不会导致这些已处理帧丢失。
日志不保存相机图片。写入失败会停止本次跟随并显示错误，避免静默遗漏记录。
会话头的 `retarget_config` 保存实际求解参数与资产路径，方便复现不同调参版本。

| JSONL 字段 | 内容 |
|---|---|
| `raw_normalized_points_21` / `smoothed_normalized_points_21` | MediaPipe 原始/平滑后归一化点 |
| `raw_world_points_21_m` / `smoothed_world_points_21_m` | 原始/平滑后三维 world 点，米 |
| `source_points_21_m` | 几何角度使用的 21 点，优先平滑后的 world 点；否则使用归一化 Z 预处理结果 |
| `topo_input_points_21_m` | 腕坐标旋转后实际交给重定向的点 |
| `skeleton_joint_angles_rad` / `_deg` | 根据骨架几何直接计算的人手角度 |
| `retarget_joint_angles_rad` / `_deg` | 模型 20 个 URDF 关节角，限位后、碰撞处理前 |
| `output_joint_angles_rad` / `_deg` | 最终显示角度，可能经过碰撞修正，也可能保持上一帧 |

CSV 列名对应 `skeleton.角度名称.rad/deg`、`retarget.URDF关节名.rad/deg`、
`output.URDF关节名.rad/deg`。所有角度同时保存弧度和角度，不写入已有动作角度文件。
无检测/手别不符时，新的 retarget 角度为 JSON `null`、CSV 空格；保持的姿态只放在 output 中。
若仅碰撞修正失败，仍保留已经成功计算出的 retarget 角度，`tracking=false` 表示该帧未更新显示姿态。

### 人手骨架角度定义

四指各记录 `MCP_bend`、`PIP`、`DIP` 和 `swing`。弯曲角为相邻两条有向骨段的夹角，
伸直为 0°，范围 0～180°。MCP 使用腕→MCP 与 MCP→PIP 两段，因而是包含侧摆影响的几何估计。
`swing` 为 MCP→PIP 在掌平面内相对腕→MCP 的方位差，范围 -180～180°。
掌坐标 Y 沿腕→中指 MCP，X 沿小指 MCP→食指 MCP 并对 Y 正交化，正侧摆朝 X。

拇指记录 `CMC_bend_proxy`（腕→CMC→MCP）、`MCP`、`IP`、`CMC_spread_proxy`
（CMC→MCP 相对掌部 Y 轴的平面方位角）。拇指 CMC 的独立轴向旋转不能仅由这些骨段唯一确定，
所以 proxy 字段不能直接视为机械手 thumb rota/swing 的真实关节角。人手角度与模型角度不是逐项同义量。
骨段长度为零、掌平面退化或向量投影为零时，相应角度留空。
精确定义同时保存在每份 JSONL 的 `angle_definitions` 中。

修改保存位置或关闭记录：

```powershell
python online_control/online_motion_track/pipeline_v3_2/main.py --hand-side left --camera 0 --log-dir D:\Qtworks\hand\logs\v3_2_angles
python online_control/online_motion_track/pipeline_v3_2/main.py --hand-side left --camera 0 --no-recording
```

也可在窗口停止跟随后取消“记录骨架/角度”，再重新开始。

## 中指伸展响应与回放分析

V3.2 的 `assets/hands/TsenseI_hand_v3_2/hand_profile.yaml` 现在单独配置求解器：
初始化历史姿态权重 `lambda_smooth=0.1`、拓扑历史姿态权重 `lambda_reg=0.1`，
初始化/拓扑迭代上限为 2/3。原来通用的 2.5/2.5、1/2 配置会使快速伸指出现明显拖尾。
V2 的默认权重不变。关键点平滑、交互图稳定、URDF 限位继续生效。
重新启动程序即可读取配置；不需要重新标定。

```powershell
python online_control/online_motion_track/pipeline_v3_2/analyze_middle.py online_control/online_motion_track/pipeline_v3_2/logs/session_20260917_111142_248015_0539c5.jsonl --output-dir docs/analysis_v3_2/middle_latency_20260917
```

替换为自己的 JSONL 路径；省略 `--output-dir` 时，在日志旁创建 `_analysis_v3_2` 后缀目录。
输出四个中指关节的 PNG/PDF、伸指局部放大图、角度 CSV、配置与统计 JSON。
分析使用记录的同一组已平滑三维输入，先复现原参数，再回放当前配置。
当前 URDF/标定与日志 SHA 不一致或旧结果无法复现时会拒绝比较，防止把模型变化误当延迟优化。
旧日志没有求解参数时尝试原通用默认值，并严格核对回放结果。
统计为 PIP 持续三个有效样本低于 45° 的时间差，不是相机到屏幕的总延迟，也不表示 45° 已完全伸直。

本次实测分析见 [中指延迟报告](analysis_v3_2/middle_latency_20260917/report.md)。

## 验证

```powershell
python -m pytest tests/test_light_test_v3_2.py tests/test_v3_2_recording.py tests/test_v3_2_latency.py tests/test_hand_profile_v3_2.py -q --basetemp .pytest-runtime/v3_2_live
```

覆盖左右手相机坐标回算、world 优先与 Z 回退、错误手别/丢失/无效点保持姿态、
碰撞模式、工作线程资源释放、最新帧缓冲，以及与 V2 硬件模块的隔离。
模型回算测试不等同于真人识别精度验证；首次使用应检查张手、握拳和拇指捏合。
