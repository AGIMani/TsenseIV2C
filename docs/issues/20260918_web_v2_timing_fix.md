# V2 网页跟随计时修复（2026-09-18）

已修复 V2 的 Ruckig 时间推进与真实时间脱节问题，并重启本地 `127.0.0.1:8766` 服务。A3.2 / V3.2 直接显示 retarget / Mink 结果；V2 按既定下位机链路显示 Ruckig 输出，因此旧版即使关闭避碰，也仍会被错误的轨迹计时拖慢。

## 新日志确认

| 记录 | 碰撞模式 | 控制平均频率 | 控制间隔中位数 | retarget 中位耗时 |
|---|---|---:|---:|---:|
| `hand_control_20260918_144202_875875.jsonl` | avoid | 89.07 Hz | 14.90 ms | 12.48 ms |
| `hand_control_20260918_144315_504085.jsonl` | off | 79.56 Hz | 15.04 ms | 14.08 ms |

旧实现每次 `sample()` 只推进固定 2 ms，却依赖实际无法稳定兑现 2 ms 的线程等待。关闭避碰的记录仍然只有约 80 Hz，轨迹推进约慢 6.3 倍。这个问题发生在 retarget 求解之后。

14:42 另有 184/204 个目标被避碰规划修改（90.2%）。碰撞目标本身与原始 retarget 的差异仍应单独评估；本次没有取消避碰、提高运动上限或将显示切换成未经限速的目标。

## 修改

- `src/utils/ruckig_interpolator.py`：两个 Ruckig 封装新增显式 `delta_time` 参数，在锁内设置当前 Ruckig 实例的步长。既有调用不传参数时仍保持固定步长，Qt 及原离线回放兼容。
- `online_control/online_motion_track/pipeline_v2/control_pipeline.py`：真实采样间隔由获得运动锁后的 `perf_counter()` 计算，在线控制用此间隔推进 Ruckig。错过调度期限时不突发发送旧采样；每次实际输出继续经过采样碰撞检查、坐标转换和 DownlinkGuard。
- `src/utils/control_clock.py`：Windows 控制线程运行期间配对使用 `timeBeginPeriod(1)` / `timeEndPeriod(1)`，异常退出也释放；其他平台不依赖该 API。高精度等待只是辅助，正确性不再要求操作系统始终达到 500 Hz。
- 超过 100 ms 的控制间隔触发目标阻断并撤销下发授权，保持上次接受的姿态；重新捕获走原来的确认和恢复流程。暂停、手别切换、找零及重新确认手势均重置计时，避免把等待时间积分成突然的大步动作。
- 网页增加实际控制频率显示。会话头保存 `control_timing`，原 `hand-control-session/v2` 的 `h7` 事件增加可选 `control_dt_s`，数据查看器继续兼容。状态返回计时模式、最后步长、累计推进时间和超时次数。

## 验证结果

**运动约束与回归：** 73 项相关测试通过，覆盖真实 Ruckig、不规则采样、速度/加速度/jerk 上限、全手相位路径、采样碰撞、丢手制动、重新捕获、线程长暂停、计时资源释放、左右手、内存串口、原生记录、Qt 默认调用方式以及 V3.2/Mink 回归。

```powershell
python -m pytest tests/test_web_v2_control_timing.py tests/test_light_control_pipeline.py tests/test_web_v2.py tests/test_control_chain_recording.py tests/test_control_session_recorder.py tests/test_form2_ruckig_downlink.py tests/test_v2_response.py tests/test_web_v3_2.py tests/test_mink_v3_2.py tests/test_light_test_pipeline.py -q --basetemp=logs/pytest-timing-full
```

**独立控制线程实测：** 单关节从 1.2 rad 伸展到 0、进入 5° 误差范围约 **0.449 秒**，接近理想 500 Hz 的 **0.446 秒**。该独立测试线程约 **495 Hz**，无超时。此前相同的 223 次采样放到旧记录的时间表上需约 **2.355 秒**。这是相同运动上限下的隔离实验，不是用户真人复测。

**新服务 HTTP 回放：** 向已经重启的 8766 服务分别发送 55 帧记录中的左手点云，测试避碰关闭和开启。两者均返回 `control_timing.mode=elapsed`，无控制超时，未连接串口、下发数为零。完整链路负载下实际控制频率约 **87 / 82 Hz**，累计轨迹推进约 **1.74 / 1.75 秒**。因此不能把独立线程的 495 Hz 描述成整个网页持续达到 500 Hz；修复的关键是即使采样率下降，轨迹仍按真实经过时间推进。

**相同记录的计时因素回放：** 固定原记录的安全目标与控制采样时间，比较旧的固定 2 ms 和新的真实间隔策略。关闭避碰记录的食指 PIP 跟踪 RMSE 从 37.17° 降至 13.58°，中指从 18.96° 降至 8.83°。这项实验未重算碰撞规划、恢复状态或摄像头识别；仅用于隔离计时问题，不能当作完整系统的现场改善幅度。

![关闭碰撞记录的计时回放](../../logs/web_v2_timing_fix_20260918/hand_control_20260918_144315_504085.png)

复现命令：

```powershell
python tools/replay_v2_control_timing.py online_control/online_motion_track/logs/v2/hand_control_20260918_144202_875875.jsonl online_control/online_motion_track/logs/v2/hand_control_20260918_144315_504085.jsonl
```

- [回放脚本](../../tools/replay_v2_control_timing.py)
- [关闭碰撞统计](../../logs/web_v2_timing_fix_20260918/hand_control_20260918_144315_504085.json)
- [开启避碰统计](../../logs/web_v2_timing_fix_20260918/hand_control_20260918_144202_875875.json)
- [独立控制线程结果](../../logs/web_v2_timing_fix_20260918/real_thread.json)
- [实际 HTTP 回放结果](../../logs/web_v2_timing_fix_20260918/http_replay.json)

## 当前可用状态

本地服务已经载入新代码，页面已刷新，保持 V2、左手、关闭碰撞的预览设置，点击“开始识别”即可复测。没有连接真实串口或驱动电机；远程 node0 未变更。

V2 仍保留真实运动限速、手势恢复和避碰约束，不能保证像直接显示 IK 目标的 A3.2 一样瞬时到位。前端单次推理和 retarget 仍为串行，本次未改变其队列策略。若仍有某段动作长时间受阻，可用新日志的实际步长和原始/安全/限速三层角度区分计时问题与接触约束。
