# 拇指弯曲丢失与右手朝向偏差（2026-09-18）

状态：已修复代码和标注资产，已通过离线验证；尚未进行修复后的摄像头实测或 H7 实机验证。

## 现象与日志证据

主窗口使用固定右手、开启模型同步与碰撞检测。人手拇指弯曲并靠近中指，模型拇指却接近水平伸出。

最新诊断日志：`logs/widget_20260918_122025.log`。代表帧 `#4639`，时间 `12:23:37.225`（第 623–624 行）：

| 量 | 日志值 | 说明 |
|---|---:|---|
| MediaPipe 拇指 IP 弯曲 | 0.733 rad，42.0° | 由关键点 2–3–4 的骨向夹角计算 |
| 求解器 thumb_PIP | 0.053 rad，3.0° | 模型拇指末节实际命名为 PIP，对应人手 IP |
| 求解器 thumb_MCP | 0.946 rad，54.2° | MediaPipe 同帧 MCP 夹角为 0.453 rad |
| 求解器 thumb_swing | 1.937 rad | RetargetDiag 记录的是输出转换前的值 |
| 修复前右手 UI thumb_swing | 2−1.937=0.063 rad | 由旧代码重建，不是该帧屏幕读数 |

人手夹角与机器人关节角存在标定偏置，不能要求数值逐项完全相等。但这里末节大幅失配与下面的运动学错误一致，不能仅靠增大迭代次数解决。

本次最新会话没有完整 21 点全链路记录，不能精确重放截图对应帧。此前的完整会话用于下面的独立离线对照，不能冒充本次摄像头复测。

## 根因一：拇指标注点绑定到了错误的运动连杆

文件：`assets/hands/TsenseI_hand/robot_keypoints_21.yaml`。

修复前 ThumbMCP（2）、ThumbIP（3）、ThumbTip（4）全部绑定在 `thumb_PIP_link`。三点刚性地一起运动，导致 2–3–4 的骨向夹角在任意关节姿态下均约为 **1.091°**。求解器虽然还能通过末节整体方向调整关节，但无法用该点链表示真实 IP 弯曲，会把弯曲错误地分配给其他关节。

ThumbCMC（1）原先还绑定在 MCP 之后，导致 MCP 关节本身也无法正确改变对应骨向夹角。

修复时将标注点转换到上游连杆坐标系，保留原有全零 init 世界位置：

| 标注点 | 原连杆 | 新连杆 | 新 local_xyz（米） |
|---|---|---|---|
| 1 / ThumbCMC | thumb_MCP_link | thumb_swing_link | [0.0016260921, 0.0076427049, -0.0290505333] |
| 2 / ThumbMCP | thumb_PIP_link | thumb_MCP_link | [0.0007465015, 0.0046522036, 0.0025495972] |

完整精度保存在 YAML；3、4 点继续绑定末节。21 点全零位置修复前后最大误差为 `6.94e-18 m`。没有新增默认姿态或改变全零 init，也没有改变 URDF 轴与关节范围。

## 根因二：右手输出重复翻转 thumb_swing

文件：`src/utils/retarget_runtime.py`。

右手输入点云已经沿 x 反射到共享左手求解器。左右 URDF 在相同关节值下互为镜像，输出再执行 `2-q` 会把正确的 thumb_swing 移向另一端；代表帧从 1.937 变为 0.063，放大了拇指朝向偏差。

已删除输出端和 previous_joint_values 回灌端的 `_flip_right_thumb_swing`。两处现在都只更换左右手关节名前缀，保留输入点云的一次反射。显示、碰撞和后续下发获得同一姿态。Form2 手动值、旧动作文件转换和 H7 协议范围维持既有契约。

左右 URDF 的镜像回归测试独立计算全部 21 点位置，防止只检查数值往返、却漏掉实际几何方向错误。

## 验证结果

以下数值采用本次标注修复时的 V2 参数（历史先验 2.5/2.5、迭代 1/2）。后续已单独修复握拳伸展响应，当前实时参数及相应精度对照见[响应问题档案](20260918_v2_extension_response.md)，不要将两组配置的回放数值混为一谈。

### 运动学与合成姿态

将 thumb_PIP 从 0 调至 0.5、1.0、1.5 rad，模型标注点 IP 夹角：

| thumb_PIP | 修复前 | 修复后 |
|---:|---:|---:|
| 0 | 1.091° | 1.091° |
| 0.5 | 1.091° | 27.554° |
| 1.0 | 1.091° | 56.106° |
| 1.5 | 1.091° | 84.630° |

以修复后 FK 生成可达弯曲姿态（thumb_rota=0.8、swing=1.6、MCP=0.45、PIP=0.75 rad），采用默认求解参数、从零连续输入 120 次：末节几何夹角误差从 **37.82° 降至 1.80°**。修复后 PIP 解为 0.7185 rad。解析雅可比与独立 FK 有限差分一致，Pinocchio 与 NumPy 后端均通过此回归测试。

### 碰撞开启的真实点云回放

来源：`logs/control_sessions/hand_control_20260917_182133_877882.jsonl`，帧 18211–18511，共 301 帧。选择弯曲且拇指最靠近中指的帧 18411 周围连续窗口。两个版本都从冷启动开始，使用相同输入、默认求解权重及迭代数；MuJoCo 使用主程序的 20 子步、接触后最多 4 步 settle，并将碰撞输出回灌下一帧。

统计剔除最初 60 帧，保留人手 IP 弯曲大于 0.6 rad 的 95 帧。两版本均用修正后的解剖标注点测量模型姿态，避免拿旧错误标注的恒定夹角评估自身。以下均为中位数：

| 指标 | 修复前 | 修复后 |
|---|---:|---:|
| 碰撞前 IP 几何夹角绝对误差 | 20.326° | 4.712° |
| 碰撞后 IP 几何夹角绝对误差 | 20.314° | 4.699° |
| 碰撞前拇指尖到中指尖距离 | 24.247 mm | 20.771 mm |
| 碰撞后拇指尖到中指尖距离 | 24.331 mm | 20.595 mm |

同批人手指尖距离中位数为 18.869 mm。人手机器手尺寸不同，距离仅用于辅助比较，不要求相等；指尖标注点也不是胶囊/指腹表面的最近点。

这是修复前后按相同条件重新计算的控制目标对照，不是原会话 MuJoCo 初始状态、Ruckig 或屏幕渲染的逐帧重现。未关闭碰撞、未修改碰撞参数，也未修改 Ruckig。残留误差仍受人机结构、标定和实时优化影响，不表示已达到精确接触。

### 自动测试及复现产物

- `tests/test_thumb_retarget_geometry.py`：上游点不随下游弯曲移动、零位标定保持、右手输出/warm start 数值保持、左右真实 URDF 镜像、雅可比与弯曲姿态恢复。
- `tests/test_thumb_swing_rebase.py` 与 `tests/smoke_test_hand_angle_protocol.py`：重标定、旧动作文件、Form2、H7 数值契约回归。
- 定向测试：132 项通过。
- 完整检查：`python -m pytest tests tests/smoke_test_hand_angle_protocol.py online_control/online_motion_play/test_server.py online_control/online_motion_play/test_deployment.py -q --basetemp=online_control/online_motion_play/runtime/pytest-thumb-fix`，288 项通过（57.46 秒）。`git diff --check` 通过。
- 已重新生成 `online_control/online_motion_play/dist/online-motion-play.tar.gz`，包含更新后的共享运行时代码；未部署远程服务。
- 离线脚本和原始结果：`logs/thumb_fix_20260918/validate.py`、`keypoints_before.yaml`、`before_replay.json`、`after_replay.json`、`results.json`（本地分析产物，logs 不进版本库）。
- 可持久化的汇总：本档案旁的 `20260918_thumb_retarget_results.json`。

生效方式：重启当前主程序以重建求解器并重新加载标注 YAML。后续摄像头复测应覆盖拇指伸直/弯曲及靠近食指、中指，左右手各一次，保持碰撞开启。本次修复针对主窗口使用的 `TsenseI_hand` 模型；独立 V3.2 资产不在本问题的修改范围内。

## 补充核查：V3.2 是否存在相同问题

2026-09-18 核查本地 `assets/hands/TsenseI_hand_v3_2/hand_profile.yaml` 指向的左右手 URDF 与 `robot_keypoints_21.yaml` / `robot_keypoints_21_right.yaml`：**没有本次三点绑定同一末节的错误**。

V3.2 的 CMC 绑定 `thumb_rotaback_link2`，MCP 绑定 `thumb_MCP_link`，IP 与指尖绑定 `thumb_PIP_link`（各手使用对应前缀）。IP 的 local_xyz 为零，位于 PIP 关节旋转中心；与指尖同属末节是合理的。上游 MCP 留在近节连杆，末节旋转时两根骨向之间的夹角能够改变。

独立 FK 扫描左右手 thumb_PIP=0、0.52333、1.04667、1.57 rad，得到 IP 骨向夹角均为 0.393°、30.168°、60.151°、90.136°，MCP 标注点位移为零。扫描 thumb_MCP 同一组角度，对应 MCP 骨向夹角为 9.994°、22.327°、51.858°、81.670°，CMC 标注点位移为零。两种弯曲都能被标注点正确表达。

此结论只排除本次连杆绑定缺陷，不等同于验证所有标注位置或整体重定向精度，也未核查远程服务当前部署的资产版本。V3.2 配置和运行代码未修改。
