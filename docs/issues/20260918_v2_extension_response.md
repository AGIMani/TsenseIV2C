# V2 握拳后快速伸指，模型缓慢追随（2026-09-18）

已修复 V2 实时求解参数造成的额外拖尾；保留现有 Ruckig 速度、加速度、jerk 限制及碰撞处理。最新摄像头会话缺少逐帧完整输入与显示轨迹，本次验证是日志定位加独立离线复现，仍需重启后的现场复测。

## 最新日志能证明什么

输入日志：`logs/widget_20260918_125838.log`。固定右手，12:58:54 开启模型同步与碰撞；下发日志为 `usb=skip`，未实际驱动 H7。

124 条 Latency 样本：

| 阶段 | 中位耗时 | P95 | 最大 |
|---|---:|---:|---:|
| MediaPipe | 15.7 ms | 16.8 ms | 17.7 ms |
| retarget | 10.6 ms | 14.64 ms | 15.6 ms |
| MuJoCo 碰撞修正 | 0.5 ms | 1.785 ms | 2.8 ms |
| 日志 e2e | 29.3 ms | 33.755 ms | 35.3 ms |

这里的 e2e 是处理关键帧并进入下发采样的耗时，不是关节到达目标的时间，也不是屏幕显示延迟。一次求解很快，仍可能需要多帧才追上目标。

例如无名指 PIP 在 12:59:03.981、05.000、06.012 三条采样中，MediaPipe 弯曲量为 1.17、1.24、1.25 rad，而 retarget 输出为 0.49、0.94、1.37 rad。说明骨架已经弯曲，求解输出仍在逐渐追上。12:59:31.276 的伸展采样中，小指 PIP 的人手值已为 0.278 rad，retarget 为 0.604 rad；下一秒才到零。

这些日志约每秒输出一次，无法精确量出快速伸展的起止时刻；骨架几何角与机器人关节角也不完全等价。不能把采样间隔直接当作完整延迟。

## 原因及修改

1. `TsenseI_retargeting/toporetarget.py` 初始化残差含 `sqrt(lambda_smooth)*(q-q_prior)`，拓扑残差含 `sqrt(lambda_reg)*(q-q_prev)`。V2 原来两个权重均为 2.5，迭代上限为 1/2。当前骨架变化后，两次求解仍被历史弯曲姿态牵制。
2. `src/skeleton/tracker.py` 将碰撞后的控制姿态回灌为下一帧 `previous_joint_values`。握拳出现接触时，受阻姿态进一步增强历史约束；这与单帧碰撞计算耗时不同。
3. `src/app/widget.py` 主模型显示共享 Ruckig 的 `peek()`。骨架显示直接来自识别点，所以二者存在正常的轨迹响应差异。屈伸关节目前限制为 4 rad/s、30 rad/s²、200 rad/s³。静止的单关节从 1.2 rad 到零，在理想 500 Hz 下约 0.446 秒进入 5° 误差范围。

修复 V2 的 `src/utils/retarget_runtime.py` / Form3 默认参数及 `assets/hands/TsenseI_hand/hand_profile.yaml`：

| 参数 | 原值 | 新值 |
|---|---:|---:|
| lambda_smooth | 2.5 | 0.1 |
| lambda_reg | 2.5 | 0.1 |
| init_max_iters | 1 | 2 |
| topo_max_iters | 2 | 3 |

未改变库级 TopoRetargetConfig 默认值、V3.2 参数、关节限位、碰撞参数和 Ruckig 限速。仍保留小的历史先验及交互图稳定，避免完全无约束的解跳变。RetargetDiag 增加 `prior=0.1/0.1 | iters=2/3`，便于确认运行时实际参数。

## 可重复的对照实验

运行 `python tools/benchmark_v2_response.py --ablation`。

用当前 V2 FK 生成四指 MCP/PIP/DIP=1.2 rad 的握拳姿态，连续输入 60 次，然后把四指目标切到全零；拇指保持零。两版使用同样模型、关键点与输入序列。MuJoCo 开启时使用 20 子步、接触后最多 4 步 settle，并回灌碰撞输出；Ruckig 按 500 Hz 模拟推进。有效目标帧率设置为 28 Hz。

统计所有 12 个四指屈伸关节均在目标 5° 内并连续保持三帧，记首次满足的帧序号：

| 碰撞 | 阶段 | 修复前 | 修复后 |
|---|---|---:|---:|
| 关 | retarget 输出 | 第 8 帧 | 第 2 帧 |
| 关 | 共享 Ruckig 输出 | 第 15 帧 | 第 14 帧 |
| 开 | retarget 输出 | 第 14 帧 | 第 3 帧 |
| 开 | 碰撞后控制目标 | 第 15 帧 | 第 5 帧 |
| 开 | 共享 Ruckig 输出 | 第 20 帧 | 第 14 帧 |

按此模拟时间轴，碰撞开启后的 Ruckig 达标由约 0.714 秒缩短至 0.500 秒（约 30%）。**这是合成阶跃的时间，不是最新摄像头日志的实测改善。** 不含摄像头、识别、Qt 绘制和操作系统调度耗时。

单独增加迭代到 2/3、仍保留 2.5 历史权重时，碰撞后达标为第 18 帧；只减小权重时为第 6 帧，说明关键是减少历史牵制，增加迭代作为补充。

还测试了“其余手指保持握拳，只伸中指”：新配置求解器第 2 帧已达标，但开启碰撞时仍有接触阻碍，碰撞输出在 40 帧内未达到 5°。这是应保留的几何约束，不能为了响应速度绕过接触。未声称所有有接触姿态都能快速到达几何上不可行的目标。

## 抖动和拇指复核

- 固定可达姿态加入 0.1 mm 高斯点噪声，输出有限且在 URDF 限位内，相邻帧关节变化均小于 5°；固定输入尾段误差低于 0.1°。
- 使用 09-17 完整点云记录帧 18211–18511，剔除前 60 帧并筛选 IP 弯曲大于 0.6 rad 的 95 帧，在碰撞开启条件下复核：拇指 IP 几何误差中位数从 4.70° 变为 5.90°，拇指尖到中指尖距离从 20.60 mm 变为 20.39 mm。存在约 1.2° 的拟合取舍，不能声称各项精度都提高。
- 最新日志缺少完整点云，因此上述真人回放使用更早的记录，仅用于回归检查。

本地分析产物：`logs/v2_response_20260918/results.json`、`pinch_regression.json`、`cmc_comparison.json`。精简汇总保存在本档案旁的 `20260918_v2_response_results.json`。

## 使用与验证

重启主程序；Form3 新默认值为历史先验 0.1/0.1、迭代 2/3。RetargetDiag 应显示同样参数。未重启的旧进程仍持有原来的求解器参数。

回归覆盖：`tests/test_v2_response.py` 测试握拳伸展、碰撞开启、Ruckig 保留限速、参数恢复默认与噪声；同时运行拇指映射、标注及 V3.2 响应回归。

完整验证命令：`python -m pytest tests tests/smoke_test_hand_angle_protocol.py online_control/online_motion_play/test_server.py online_control/online_motion_play/test_deployment.py -q --basetemp=online_control/online_motion_play/runtime/pytest-v2-response`。结果：**292 项通过，73.74 秒**。`git diff --check` 通过。已重新生成 online_motion_play 本地部署包，未部署远程服务。

21 点标定和 CMC 的 rota/swing 选择见[标定说明](../keypoint_calibration.md)。
