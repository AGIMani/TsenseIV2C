# A30 动作播放：1 kHz Python 计算与离线基准

测量日期：2026-09-22。工作目录：`D:/Qtworks/hand`。本次只修改 Python 计算/发送适配及其测试、基准和本文档；未连接真实串口、未部署、未提交。JS、server.py、index、package 及 BrowserSerial 的并行改动由其他协作者负责。

## 实现与边界

- `A30Device.CONTROL_DT = 0.001`，Ruckig 步长和 I/O worker 采用独立的 1 ms 周期。共享 `online_control.shared.downlink.CONTROL_HZ = 500`、`CONTROL_DT = 0.002` 原样保留，手势网页仍是 500 Hz。
- 继续使用共享 `configure_ruckig`，动作播放速度限制为 swing 1.5 rad/s、其他关节 2.5 rad/s，加速度 10 rad/s²、jerk 80 rad/s³，`Synchronization.Time`。保留调用方设置的 `minimum_duration`。共享函数已有的 acceleration/jerk 可选参数未修改；手势调用方现有的 15/120 参数未修改。
- worker 继续以绝对 deadline 调度并跳过错过的周期。tick 累积 elapsed、按 1 ms 推进 Ruckig，但每次最多发送一个最新位置，绝不补发积压位置帧；`next_send` 保留原来的绝对相位。
- 保留 >100 ms 控制间隔停发、>500 ms 遥测过期停发、有限数值与完整关节名检查、模型/机构限位、查表误差界、CRC、线性行程符号与外部电机 50/30 减速比。
- 实时 tick 和每次 arm 的首个保持帧传入新的 `q` 列表：按 `mapper.joint_names` 排列的 20 个 Python float，单位 URDF radians。端口提供可调用的 `write_trajectory(frame, q)` 时优先调用；普通串口/FakePort 回退 `write(frame)`。arm 的 q 使用 `measured_pose` 解析/限位后的反馈基线，缺失轴使用明确 fallback，与 Ruckig current_position 一致。使能、回零、断电及独立 hold_feedback 路径继续只写普通帧。
- 任一种 writer 返回值都必须等于 `len(frame)`。只有成功写入后才增加 `tx_count`、打印原格式 HEX 日志，并在 tick 中推进 seq/last_send；短写触发原有 OSError 停止和断开路径，不计数、不记成功日志。传给端口的 wire frame 仍是 108 字节。

## 有测量依据的优化

第一轮对 10,000 次 convert + encode 做 cProfile：总计约 823 万次调用；`_finite_float32` 累计 0.861 s，`channel_mapping` 0.278 s，`isinstance` 累计 0.775 s（包含嵌套，不能相加）。Ruckig 常规 update + pass_to_input 的无 profiler p50 只有 0.8 µs。

因此只做两项计算优化：

1. mapper 与编码器对精确类型为内建 float/int 的值直接走原有数值/边界校验；其他类型仍通过 `numbers.Real` 检查。bool、非数值、NaN/Inf、越界和 float32 溢出仍拒绝，NumPy real 与 Fraction 仍支持。
2. 编码器按左右手缓存不可变通道布局及完整名称集合，避免每帧构造 20 个通道字典和名称集合。公开 `channel_mapping()` 仍返回独立的可修改列表，其修改不能污染缓存。

未改变 Ruckig 算法，也未省略几何检查或改变 LUT 精度。优化后相同 profile 约 428 万次调用、总时间 1.907 s（优化前 3.210 s）；常规输入的 numeric ABC 检查不再占据热点。profile 自身有测量开销，以下耗时均为无 profiler 测量。

## 可复现方法

使用 `E:/conda/envs/hand_dex/python.exe`：Python 3.11.15、Ruckig 0.17.3、NumPy 2.4.4，Windows build 22000，AMD64 Family 25 Model 80、16 个逻辑 CPU。系统默认 `python` 缺少 Ruckig，不用于本基准。

从仓库根目录执行：

```powershell
& E:/conda/envs/hand_dex/python.exe -X utf8 tools/benchmark_a30_python.py --iterations 10000 --warmup 1000 --profile --output docs/a30_python_1khz_rerun.json
```

脚本仅使用内存端口，不启动 server/browser/I/O worker，不调用物理串口构造器。每项预热 1,000 次、测量 10,000 次，随机种子 30、256 个合法 20 关节姿态，GC 保持启用，`perf_counter_ns` 逐调用计时，分位数使用 nearest rank。mapper 初始化/模型加载不计入稳态耗时。

编码项包含共享 `encode_control_targets` 的符号归一化、完整校验、108 字节打包和 CRC。Ruckig 项均用 1 ms 步长、20 轴、共享限制，包含 update + pass_to_input 及 Python 调用开销；常规步进每 250 次重置，避免只测 Finished；重规划项每次改变目标，包含目标列表构造。tick 项每 100 次更新一次目标；内存端口的 write_trajectory 只接收并丢弃数据，不模拟 BrowserSerial 的排队、封装或传输成本。

tick 使用模拟时钟分别推进 1/2/3/4 ms，遥测是新鲜的、所有轴离线的 484 字节缓存，arm 从合法 fallback 姿态起步；计入 read/require_feedback、Ruckig、转换、编码、锁和内存 writer，**不包含新遥测包解析、操作系统唤醒、HTTP、JS、USB、真实串口与 HEX 打印成本**。这些是 Python 计算预算结果，不是实测 wire 包间隔，也不保证 Windows 每次都能在 1 ms 唤醒。

修改前后原始结果、环境和源文件 SHA-256 分别保存在 [before JSON](a30_python_1khz_before.json) 和 [after JSON](a30_python_1khz_after.json)。采集时仓库 HEAD 为 `04fcde7fb686bcab54ffe9706cd4d950a0d89ae8`，但工作区已有他人未提交修改；JSON 中的内容散列标识实际被测文件，不能把整个 HEAD 当作当时工作区快照。before 是本次改代码前采集，after 是生产代码修改后采集，两次使用相同基准脚本；复测不要还原协作工作区。

## 结果（µs）

| 计算项 | 修改前 p50 | 修改后 p50 | 修改前 p99 | 修改后 p99 |
| --- | ---: | ---: | ---: | ---: |
| mapper.convert，20 关节 | 48.4 | 36.9 | 99.9 | 44.8 |
| 编码，108 字节 | 70.2 | 30.1 | 163.3 | 42.9 |
| convert + encode | 127.3 | 76.0 | 310.2 | 91.5 |
| Ruckig 常规步进 | 0.8 | 0.8 | 1.1 | 1.0 |
| Ruckig 每步重规划 | 18.3 | 18.1 | 54.1 | 44.1 |

convert p50 降低约 24%，编码降低约 57%，组合降低约 40%。各项分开采样，不能直接把其分位数相加。Ruckig 未修改，其前后差异视为运行噪声；单次顺序采样的长尾也会受系统负载和 CPU 频率影响。

| tick 调用间隔 | 原 500 Hz p50 | 新 1 kHz p50 | 新 1 kHz p95 | 新 1 kHz p99 | 新 1 kHz max |
| --- | ---: | ---: | ---: | ---: | ---: |
| 1 ms | 98.1 | 97.7 | 108.3 | 176.0 | 277.6 |
| 2 ms | 146.3 | 102.0 | 110.3 | 180.3 | 221.4 |
| 3 ms | 147.5 | 104.3 | 121.0 | 189.8 | 2286.9 |
| 4 ms | 149.1 | 107.0 | 116.0 | 182.6 | 284.0 |

旧设备周期为 2 ms，因此旧版在 1 ms 调用间隔下约一半 tick 不编码/发送，不能把这一行解读为相同工作量的性能比较。新版 1/2/3/4 ms 调用间隔分别推进约 1/2/3/4 个 Ruckig 步长，但都只发送一个最新位置。最终版本复测的 tick p99 为 0.176–0.190 ms；3 ms 调用间隔组观测到一次 max 2.287 ms 长尾，具体来源没有进一步定位。该结果支持常规计算预算有余量，但**不支持每次 tick 均小于 1 ms 的保证**，因此继续保留跳过错过周期机制。最终复测已包含最新要求的 arm 首帧 q 适配。

## 回归测试

272 项通过，耗时 11.86 s。覆盖新增双手 1 kHz 调度、1–4 ms 间隔、延迟追赶与绝对相位、不补发、worker 跳过周期、目标反向后的速度/加速度/jerk、minimum_duration、原有遥测失效保护；覆盖轨迹 writer 的 q 顺序/float 类型/快照、108 字节等价、seq 回绕、每次 arm 首帧携带解析后的反馈 q、普通端口回退、异常返回不计数/不打印与断开；原有 A30 mapper、参考固件打包、动作播放、手势和线性行程策略回归也通过。

```powershell
$a30TestTemp = Join-Path (Get-Location).Path ('online_control/online_motion_play/runtime/pytest-a30-python-' + [guid]::NewGuid().ToString('N'))
& E:/conda/envs/hand_dex/python.exe -X utf8 -m pytest tests/test_a30_device_timing.py tests/test_a30_mapper.py tests/test_a30_downlink.py tests/test_a30_motion_play_sync.py online_control/online_motion_play/test_a30.py online_control/online_motion_play/test_downlink_policy.py tests/test_a30_control.py tests/test_telemetry_signed_zero.py -q --tb=short --basetemp=$a30TestTemp
```

第一次扩展回归有 11 个 fixture setup 错误，原因是系统 `%TEMP%/pytest-of-ONKIALL` 拒绝访问；改用上面新的唯一临时目录后，同一批测试连同新测试全部通过。未删除或改权限于原临时目录。

## 本次文件

- `online_control/online_motion_play/a30_device.py`：独立 1 kHz、实时 tick 与 arm 基线的 trajectory writer 适配。
- `src/a30_kinematics/mapper.py`：内建数值校验快路径。
- `src/a30_kinematics/downlink.py`：编码快路径与不可变通道缓存。
- `tests/test_a30_device_timing.py`：新增离线设备调度/限幅/端口测试。
- `tests/test_a30_mapper.py`、`tests/test_a30_downlink.py`：数值兼容与缓存隔离回归。
- `tools/benchmark_a30_python.py`：可复现离线计算基准。
- `docs/a30_python_1khz.md`、`docs/a30_python_1khz_before.json`、`docs/a30_python_1khz_after.json`：说明和原始结果。
