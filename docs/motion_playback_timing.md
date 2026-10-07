# 动作下发耗时问题记录与修复验证（2026-09-16）

## 问题现象与结论

用户反馈：动作文件约 18 秒，实际执行约 50 多秒。排查范围为
动作文件加载 → Form2 序列调度 → Ruckig/Guard → 角度转行程 → 串口发送至 H7。

本次发现的主要问题是上位机将大量 2 ms 关键帧作为独立动作段执行，
逐段重新计时、同步发送与界面/日志处理使额外时间不断累加。
文件读取不是主要瓶颈；不能仅根据界面显示的“500 Hz”判断实际下发频率。

修复后，同条件离线测试的播放工作线程总耗时由 **38.027 秒降至 19.190 秒**，
其中轨迹时间 **18.623 秒**，与文件规定的 **18.622 秒**基本一致。
Ruckig 参数与推进机制保持不变，约 0.525 秒终态收敛单独计入总耗时。
桌面和 online 共用修复后的调度。真实 H7 传输和机械到位时间尚未复测。

## 已有日志证据

本地 `logs/widget_20260915_174005.log` 记录了以下完整播放：

| 序列 | 段数 | 文件时长 | 主机日志播放时间 |
| --- | ---: | ---: | ---: |
| 密集轨迹 `hand_controller_trajectory_left_motion.txt` | 7859 | 18.621858 s | 47.800 s |
| 普通动作 `twinkle_v2_best_motion.txt` | 179 | 17.851 s | 18.214 s |

密集轨迹于 19:32:29.592 开始、19:33:17.392 结束，见日志第 19760–27620 行。
额外约 29.18 秒，摊到 7857 个短段约为每段 3.7 ms。
这证明上位机播放阶段本身就已明显变长，并非只能归因于电机动作慢。

该次运行使用更早的 `sine_ramp` 版本，尚非本次修改前的 Ruckig 版本。
因此可用来判断历史问题，但不能直接与下面的离线结果作同条件性能比较。
“播放完成”及主机串口写入成功也不代表 H7 已确认执行或电机已经到位。

## 原因与修复

原实现把每个关键帧作为独立动作段：从当前时刻开始等待该段时长，
段末同步发送并 flush，再开始下一段。段末发送和段间处理落在各段规定时长
之外；线程唤醒超时与 UI/日志竞争也会增加延迟，后续各段不会抵消这些延迟。

### 1. 每段重新设置 t0，延迟逐段累加

原 `_motion_stream_worker` 逐段调用 `_run_segment_send`，后者每次执行
`t0 = time.perf_counter()`。例如一段等待 2 ms，随后段末发送等工作花费
1 ms，下一段便从第 3 ms 开始；1000 段就可能从规定的 2 秒变成 3 秒。

排查时调用原调度函数：1000 个 2 ms 段、不接串口约 3.08 秒；
模拟每次发送额外耗时 1 ms 时约 4.64 秒。这些是局部诊断实验，
不包含完整 GUI 和实际 USB，作用是验证逐段累积机制。

### 2. 微小 interval 可能多等一个完整发送周期

原实现只在 2 ms 的发送节拍检查段是否结束。精确模拟时钟下：

- 2 ms 过渡 + 0 停留，在 2 ms 结束。
- 2 ms 过渡 + 1 微秒停留，在第一个节拍尚未达到 2.001 ms，可能等到 4 ms 才结束。

实际是否多等一个节拍取决于唤醒误差，不能将每一帧都视为必然翻倍。
累计时间轴修复后，停留按真实数值相加，不再在每个关键帧末尾向上取整。

### 3. 每段终态触发阻塞发送和 flush

原段末调用 `_send_form2_ruckig_step(..., stream=False)`，经过
`UsbController.send_joint_angles` 和 `send_hand_positions`，最终调用
`ser.flush()`，密集文件就会有约 7859 次段末阻塞发送。

本机 pyserial 3.5 的 Windows `flush()` 在 `out_waiting` 非零时按 50 ms
睡眠间隔轮询，属于潜在长停顿来源。已有记录不足以量化其在历史 47.8 秒
运行中的独立贡献，也不能把所有差值归因于 flush。它等待的是主机发送队列
清空，不是 H7 的执行完成应答。

### 4. 逐段 UI 通知、搜索与同步日志增加负载

原来每段都发出 Qt 信号，界面调用 `combo.findText(name)`，再更新动作选择
和时间输入框。近八千项的下拉框中，本机离线测得搜索/选择前部动作平均
约 0.023 ms，后部动作约 0.337 ms。逐段日志又同步打印、写文件并 flush，
增加发送线程开销及线程竞争。以上诊断数据不代表各项在实机中的精确占比。

### 5. 文件加载和 Ruckig 的处理范围

初次排查的文件读取、解析和角度映射约 0.265 秒；完整基准中约 0.50 秒，
均远小于几十秒的延长量。读取发生在播放前，应与播放线程耗时分开统计。

Ruckig 按固定控制步长推进，实际调用变慢时可能产生跟踪滞后，但历史
`sine_ramp` 日志已经存在同类时长问题，故它不是历史问题的原始原因。
按本次约定，不修改 Ruckig 参数、推进步长和终态收敛机制。

### 修复实现

当前序列改用 `_run_motion_timeline`：

- 预计算各段累计起止时间，整条序列共用一个 `perf_counter` 起点。
- 发送线程按时间轴取当前目标，迟到时跳过已过期目标，不突发补发积压帧。
- 中途所有位置帧使用流式发送，不在每个关键帧边界单独 flush。
- 仍经原有 Ruckig、协议转换、DownlinkGuard 和 UsbController 下发；
  运动限制、Ruckig 推进步长及终态收敛逻辑未改。
- 结束后仍执行原有可靠发送/终态收敛，因此整体时间包含额外收敛时间。
- 取消逐段 Qt 信号；已有 33 ms 定时器读取最新姿态与动作索引。
- 下拉框使用动作名到索引的缓存，避免每次 `findText` 扫描全部动作。
- 日志改为每秒进度汇总，结束输出文件、轨迹、终态收敛和总耗时。
- 单次“加载选定动作”的 `_run_segment_send` 保留，并将等待上限夹到
  段结束时刻，避免微小停留多等一个完整发送周期。

时间轴策略意味着不保证每个短关键帧都被单独下发；实际下发姿态仍通过
Ruckig/Guard 限制。线程卡顿时不会为维持文件进度而绕过运动限制。

## interval 是否可设为 0

可以。`_interval_s=0` 表示过渡之后不额外停留，原解析和 UI 都允许。
新调度保留所有正停留值的累计时长，即使为 1 微秒，也不按每段向上取整到 2 ms。

测试文件为 `motion/hand_controller_trajectory_left_motion.txt`：

- 7859 段，7857 段过渡为 2 ms。
- 过渡合计 18.514 秒，停留合计 0.107858 秒，总计 18.621858 秒。
- 停留包含 init 的 0.1 秒和其余段的微小停留。
- 将所有 interval 设为 0 后总长为 18.514 秒；若仅去掉微小停留、保留
  init 的 0.1 秒，则总长为 18.614 秒。
- 基准脚本不改写动作文件，零停留对比仅在测试内存中修改。

上述段数和时长对应测试当时的文件版本；工作副本后续可能已调整。
再次测试时先核对段数及 `_transition_s + _interval_s` 总和，不应将
当前同名文件直接视为与本次 18.621858 秒样本完全相同。

## 测试条件与结果

本机 `E:/conda/envs/hand_dex/python.exe`，包含真实 Form2 调度、Ruckig、
DownlinkGuard、角度转行程、UsbController 打包/CRC 和 Qt 动作下拉框事件循环。
仅串口句柄替换为内存接收器；不包含真实 USB 传输、H7 回传/机械运动、
3D 模型渲染和同步会话数据记录。初始 UI 姿态为全零。

测试时只枚举到 COM17/COM18 蓝牙串口，未发现 H7 USB CDC，因此下面均为
**离线主机发送链路计时，不是实机到位时间**。

| 条件 | 文件时长 | 轨迹时间 | Ruckig 终态收敛 | 播放工作线程总耗时 |
| --- | ---: | ---: | ---: | ---: |
| 修改前，同一原文件 | 18.621858 s | 37.496490 s（收敛前） | 0.530403 s | 38.027234 s |
| 修改后，同一原文件 | 18.621858 s | 18.622778 s | 0.524522 s | 19.189730 s |
| 修改后，所有 interval=0 | 18.514000 s | 18.515163 s | 0.530215 s | 19.089679 s |

修改后原文件时间轴预处理约 0.040 秒。上表总耗时从工作线程开始计时，
包含时间轴准备与终态收敛，不包含此前文件加载；本次加载约 0.50 秒。

原文件前后对比：

- 逐段 UI 事件：7859 → 0（改由定时器合并更新）。
- 串口 flush 调用：8114 → 253；剩余调用来自初始手型命令和既有终态收敛。
- 整手下发帧：9639 → 9427，全部通过协议结构与 CRC 校验。
- 修改后时间轴内发送 9175 帧，跳过 136 个已过期发送节拍。
- 修改后终态 UI 目标最大误差约 7.69e-7 rad，低于既有 1e-5 rad 收敛阈值。

这些是单轮同条件对比，实际时长和帧数仍受系统调度负载影响。
此前实机日志中的 47.800 秒来自更早的 sine_ramp 版本，不作为本轮
修改前后同条件性能比较的基准。

## online 同步情况

`online_control/online_motion_play/server.py` 的 `WebForm2Window` 直接继承共享
`Form2Window`，没有另写一套下发调度。因此统一时间轴、流式发送、UI 合并刷新
与日志限频同时适用于 online。旧的网页专用结束刷新覆盖方法已移除，避免
共享处理函数已经刷新最终姿态后再次重复刷新。

- `/api/state` 的 `playback_timing` 返回最近一轮播放统计；运行中不作为实时进度。
- 测试覆盖网页导入、播放入口、零/微小 interval、流式下发、最终姿态及帧 CRC。
- 包含 online、时间轴、Ruckig、找零和 USB 协议的相关回归共 **49 项通过**。
- 已重新生成 `online_control/online_motion_play/dist/online-motion-play.tar.gz`，并逐字节核对
  包内 Form2、服务端和 README 与当时工作区源码一致。
- 本次未部署远程服务器；已有服务需部署新包后生效，仅刷新网页不能更新服务端代码。

## 后续实机验证与计时口径

连接 H7 后，桌面或 online 播放同版本文件，保留开始/结束日志及终态收敛统计。
应分别观察：

1. 文件加载时间：文件读取、解析和角度映射。
2. 主机轨迹时间：从时间轴启动到最后一个轨迹目标的发送处理结束。
3. 主机播放总时间：时间轴准备 + 轨迹播放 + 终态收敛。
4. H7/机械执行时间：需要结合设备侧时间戳或实际反馈确认，不能用主机写入成功代替。

目前只验证了前述离线主机链路。真实 USB 阻塞、遥测线程、3D 显示及其他
系统负载仍可能降低有效发送频率或增加终态收敛时间。若问题再次出现，优先
比较“文件/轨迹/终态收敛/总耗时”四项日志及跳过节拍数，再定位传输与设备侧。

## 复现

```powershell
E:/conda/envs/hand_dex/python.exe tools/bench_motion_playback.py motion/hand_controller_trajectory_left_motion.txt --output logs/motion_timing_20260916/after.json
E:/conda/envs/hand_dex/python.exe tools/bench_motion_playback.py motion/hand_controller_trajectory_left_motion.txt --zero-interval --output logs/motion_timing_20260916/after_zero_interval.json
```

修改前源码快照和本次 JSON/log 均保存在本地 `logs/motion_timing_20260916/`。
基准工具从所选源码提取实际方法，并使用当前模块配置；`--source` 可以指定
同配置的修改前源码快照。本工具从不打开真实串口。

回归验证：

```powershell
E:/conda/envs/hand_dex/python.exe -m pytest tests/test_form2_motion_timeline.py tests/test_form2_ruckig_downlink.py tests/test_usb_v3_protocol_sync.py -q
```

21 项通过，覆盖密集帧累计计时、零/微小停留、慢发送、停止、循环、正常停留、
最终目标、UI 索引，以及既有 Ruckig/Guard 和 USB 协议回归。

包含 online 的回归命令（本次 49 项通过）：

```powershell
E:/conda/envs/hand_dex/python.exe -m pytest online_control/online_motion_play/test_server.py online_control/online_motion_play/test_deployment.py tests/test_form2_motion_timeline.py tests/test_form2_ruckig_downlink.py tests/test_form2_onekey_zero_v3_sync.py tests/test_usb_v3_protocol_sync.py -q
```

## 相关代码与记录

- [Form2 调度、UI 与日志](../src/ui/form2_window.py)
- [串口写入与协议打包](../src/usb/offline.py)
- [时间轴回归测试](../tests/test_form2_motion_timeline.py)
- [离线播放基准工具](../tools/bench_motion_playback.py)
- [online 服务端](../online_control/online_motion_play/server.py)与[部署说明](../online_control/online_motion_play/README.md)
- 本地原始测试记录：`logs/motion_timing_20260916/before.json`、`after.json`、
  `after_zero_interval.json` 及同名日志。日志目录不纳入版本控制，关键结果已在本文留存。
