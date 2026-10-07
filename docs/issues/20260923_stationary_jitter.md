# 17:07:07 录制的静止抖动分析

输入：`C:/Users/ONKIALL/Downloads/hand_control_20260923_170707_315000.jsonl`。

后续已完成自动恢复保留状态的修复及全日志对照回放，见 [修复记录](20260923_retarget_recovery_fix.md)。下文保留最初定位时的实验与结论。

结论：关键点滤波有实际降噪作用，但轨迹流频繁自动恢复会同时清空滤波状态和 retarget 的历史初值。**大幅周期性目标尖峰的明确来源是 retarget 冷启动；滤波状态重置也使该帧直接使用原始噪声。** 先解决恢复/重置链路，再评价静止滤波强度。本轮完成日志分析和离线复现，没有修改生产控制逻辑或滤波参数。

## 自动选择低运动片段

用户未提供静止标记，采用 2 秒滑窗、0.5 秒步长，计算腕部对齐后的原始 21 点各坐标时间标准差的 RMS；选取分数最低的 5 个互不重叠片段。每段至少 45 帧，帧间隔不超过 100 ms。这识别的是手指姿态变化较小的候选片段，不能排除真实微动或低速运动，也不代表相机位置完全静止。

| 时间 / s | 原始坐标波动 / mm | 滤波后 / mm | 波动最大 retarget 关节的峰峰值 / ° | 波动最大已写入 H7 关节的峰峰值 / ° |
|---|---:|---:|---:|---:|
| 17–19 | 0.847 | 0.465 | 25.55（食指 MCP） | 8.23（拇指 MCP） |
| 31.5–33.5 | 0.893 | 0.569 | 22.86（拇指 rotaback） | 7.36（拇指 rota block） |
| 36–38 | 0.659 | 0.402 | 23.09（拇指 rota block） | 5.33（食指 DIP） |
| 41–43 | 0.690 | 0.496 | 64.80（拇指 rota block） | 9.90（拇指 rota block） |
| 43.5–45.5 | 0.593 | 0.346 | 60.39（拇指 rotaback） | 7.09（拇指 rota block） |

角度峰峰值不是标准差，各列选择各自波动最大的关节，不能将不同关节的比值当作传递增益。H7 仅统计 `sent=true, source=shared_binary_stream`，证明实际写入命令存在波动，不能直接等同于机械角度。

## 主要证据

- 46.176 秒录制中有 1153 个 retarget 帧、16200 个流式成功写入命令，55 次 `trajectory_recovery`，且全部是 `stale_after_pacing`。
- 自动恢复间隔中位数 **0.769 s**；恢复耗时中位数 **65.5 ms**。各次恢复之间的写入速率中位数约 **421.44 Hz**。从 H7 记录的本机经过时间减去对应规划时间累加，恢复前累计落后中位数约 **96.03 ms**，与 100 ms 过期阈值吻合。说明消费节拍产生持续积压；仅凭此日志还不能把耗时完全归因于 USB、主线程、定时器或网络某一项。
- 将原始 world landmarks 用生产 `prepare_points` 转换后，与同 frame_id 的 `mediapipe.landmarks` 比较，发现 **51 帧完全一致**：第一个有效输入及之后 50 个滤波重新初始化帧。
- 重置后的 50 次角度变化中，每帧最大关节步长的中位数 **22.85°**、P95 **59.99°**；其余 1102 次变化中位数 **0.97°**。这个全段比较包含真实运动，仅用于定位重置尖峰。
- 413 个 planner 状态采样没有记录到 `held_fingers`，本次主要特征不同于上一份日志中的碰撞冻结；低频状态采样不能排除所有瞬时碰撞事件。

## 完全相同输入的对照实验

取原始 frame 1014 的 21 点，使用生产 `WebSession`、A3.0 retarget 与 Mink，连续 120 帧重复同一输入，30 Hz。第 80 帧分别执行以下操作：

| 操作 | 最大关节单帧跳变 |
|---|---:|
| 不重置 | 0° |
| 只重置关键点滤波 | 0° |
| 只重置 retarget 历史 | 21.719° |
| 同时重置两者 | 21.719° |

最大跳变发生在拇指 rota block。重置前最后 10 帧的最大步长为零。这证明即使完全没有 MediaPipe 噪声，清空求解历史本身也能制造尖峰。实验隔离了重置操作，没有模拟 USB、固件或完整浏览器恢复过程；“只重置滤波为零”仅适用于完全相同的输入，不代表噪声输入下重置滤波没有影响。

## 项目中的原因链

1. `static/a30-browser-hand-control.js::streamSample` 在本机按至少 2 ms 的写入起点间隔逐条消费规划样本；等待和实际写入时间造成消费速度低于生产速度时，样本不断变旧。超过 100 ms 后调用 `recoverTrajectory`。
2. `recoverTrajectory` 请求 `browser_rebase(resume_from_commanded_pose=true)`。
3. `a30/session.py::command` 在这个自动恢复分支中，仍无条件执行 `retargeter.reset()`、`filter.reset()`、`normalized_filter.reset()`。
4. `ResponsiveLandmarkFilter` 重置后的首帧直接接受原始关键点。`TopoRetargeter._relative_bone_direction_initialization` 在没有 `previous_init_q` 时，从零位 `init_q` 开始，只执行有限次数迭代，输出离开原稳定解后又在后续帧收敛回来。
5. Ruckig 将这些尖峰变成连续运动，但不能识别它们是噪声，所以成功写入的目标仍出现往返摆动。控制器本身重建也会重置轨迹导数，带来停顿。

该录制是 `direct_mode=false, ruckig_enabled=true, direct_filter=null`。实际关键点滤波来自 `landmark_filter.py` 的 ResponsiveLandmarkFilter（min cutoff 1.2 Hz、beta 80、noise speed 0.01 m/s、max cutoff 15 Hz）。`direct_filter.py` 的直接模式滤波没有参与；调它对本次数据无效。`noise_speed` 是自适应截止频率的噪声门槛，并不是静止姿态锁定或位置死区。

## 建议修复顺序

1. 将自动传输恢复与手势跟踪重启分开：重建下发轨迹需要同步最后已发送姿态，但同一跟踪会话的关键点滤波、retarget 历史初值及交互图历史应保留连续性。手别/模型切换、真正的跟踪中断和手动重新建立起始姿态另行处理。
2. 解决生产/消费节拍不匹配和积压，避免每约 0.77 秒触发一次恢复。保持过期保护与运动/路径检查；不能单纯提高 100 ms 阈值掩盖延迟，也不能未经验证跳过轨迹路径。
3. 在前两项修复后的连续运行日志上，再比较静止降噪、微小动作灵敏度、快速动作延迟，决定是否需要更低的静止截止频率、滞回或轻微死区。本次数据不足以证明应直接调大平滑强度。

## 其他数据限制及复现

4352 帧遥测均带 `decode_issues`，其中包括非有限字段、异常温度等，回传解析/固件格式需要单独核对。因此本报告不使用这些异常反馈量定量宣称真机位置抖动幅度。录制结束原因是达到 256 MB 上限，不是本次统计中的丢手或控制中断。

分析脚本：`tools/analyze_stationary_jitter.py`。结果：`logs/jitter_170707/metrics.json`，图：`logs/jitter_170707/diagnosis.png`。

```powershell
E:\conda\envs\hand_dex\python.exe tools/analyze_stationary_jitter.py "C:\Users\ONKIALL\Downloads\hand_control_20260923_170707_315000.jsonl" --output logs/jitter_170707
```

脚本只读原始记录，使用不带硬件控制的 WebSession 做离线对照，没有访问相机或串口。
