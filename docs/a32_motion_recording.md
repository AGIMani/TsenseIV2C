# A3.2 手势跟踪与 motion_play 数据记录

## 文件和查看方法

两个平台共用 `src/a32_driver/recording.py` 的 `A32SessionRecorder`，
`online_control/online_motion_play/a32_recording.py` 保留旧导入路径兼容。
文件使用现有查看器的
`hand-control-session/v2` JSONL 格式，可在
`tools/hand_control_log_viewer.html` 中直接打开。

默认目录为 **`E:\TsenseI_hand_logs\A32`**，两款模型共用此目录，通过
`metadata.model_id` 区分裸手和 with ID，通过 `metadata.platform` 区分平台。
父目录保存在 `configs/control_log_parent.txt`；查看器可修改父目录，下一次
录制使用新目录，正在写入的文件保持原位置。代码或测试可显式指定 `output_dir`。

状态接口中的 `recording.path` 和 `recording.files` 给出实际绝对路径。
motion_play 页面“下载记录”通过 `/api/a32-log` 下载，多段记录作为 ZIP 导出；
`/control-log-viewer` 打开同一查看器。两个平台均支持查看器的
`GET /api/viewer/logs`、`GET /api/viewer/log-file` 和 `POST /api/viewer/log-parent`，
可从目录刷新、打开记录并查看时长汇总。这些本机日志接口限本机同源访问。
手势平台原有浏览器记录和下载功能仍保留，服务器 SDK 日志独立保存。
文件名是 `hand_control_时间.jsonl`，超过 512 MiB 后分段。

连接前开始记录，主动断开或关闭会话时结束。暂停实时关节控制、页面失去心跳、
SDK 意外失联或保持连接但停止运动时，继续记录状态，方便判断是串口断开、轨迹
暂停还是电机失能。
JSON 编码和文件写入由独立线程完成，控制线程只提交事件；每隔最多约 0.5 秒刷新
到文件。队列有容量上限，丢弃条数写入结束事件，写盘错误通过 `recording.error`
显示，不能冒充完整录制。

记录时段包含连接后的等待、暂停和保持连接但未下发的时间；查看器的记录时长
及汇总表示日志覆盖的时段，**不是电机通电时长或实际运动时长**。

## 时间字段与查看器功能

每条事件都保留以下时间；时间在控制/采样线程提交事件时获取，文件写入线程
不会把写盘完成时间当作采样时间。

| 字段 | 含义 |
| --- | --- |
| `t` | 从本次记录开始计算的单调相对时间，单位秒，用于事件排序和时间轴 |
| `wall_time_unix_ms` | 获取事件时的主机系统时间，Unix 毫秒 |
| `wall_time` | 同一主机时间的 ISO 8601 字符串，带时区偏移，显示到毫秒 |
| `tick_ms` | SDK 提供的设备时钟，单独保存，不当作主机实际时间 |
| `timestamp_ms` / `capture_unix_ms` | 手势浏览器的帧时间及拍摄系统时间，单独保存，不覆盖服务器采样时间 |
| `h7.control_dt_s` / `h7.control_period_ms` | 控制循环实际经过的秒数及毫秒数；没有实际周期的事件保留空值 |

系统校时可能使主机系统时间跳变，因此相对时间仍使用单调时钟。SDK 快照的
主机时间表示观察缓存的时刻，并不等同于固件采集或 USB 到达时刻。

查看器支持实际时间显示、相对时间范围选择、逐帧详情、控制周期的平均/P95/最大
统计及 CSV 导出。CSV 保留主机时间、设备时钟、控制周期、命令/测量位置和有效性。
老日志没有逐事件主机时间时，可按 `started_at + t` 推算并注明时间来源；没有
开始时间的日志仅显示相对秒数。

## 记录内容

| 事件 | 内容 |
| --- | --- |
| `session` | 型号、左右手、SDK 状态格式、物理单位、14 直线和 6 旋转槽位映射 |
| `mediapipe_raw` / `mediapipe` | 手势平台的原始检测、重定向输入关键点及拍摄时间 |
| `retarget` / `control_target` | 重定向关节角、碰撞检查后的目标及对应 `frame_id` |
| `retarget_skipped` / `control_target_skipped` | 无手、旧帧、求解或控制拒绝等未采用目标的原因 |
| `sdk_action` | 连接、使能、暂停、继续、断开等请求与结果 |
| `sdk_status` | 连接、使能、找零、错误、遥测年龄，以及 SDK 完整状态 |
| `conversion` | URDF 关节角和送入 SDK 的毫米/输出轴角度 |
| `downlink` | SDK 写入是否成功、失败原因、对应转换事件编号和实际目标数组 |
| `h7` | 控制采样关节角、真实采样间隔及是否已下发，兼容查看器关节曲线 |
| `telemetry` | SDK 最新遥测快照、14+6 规范化电机数组、完整原始 SDK 状态 |
| `board_telemetry` | SDK 可用的板载温度和母线电压 |
| `sdk_error` / `session_end` | 故障原因、结束原因和丢弃事件数 |

SDK 只暴露最近一帧的解码状态，因此这里记录的是**观察到的新快照**，不是全部
USB 原始帧。重复轮询同一 `sequence/tick_ms/rx_ok` 不增加遥测帧数。原始字节未
由 SDK 提供时，不构造 `payload_hex` 或 `wire_seq`。

手势 SDK 控制线程每约 20 ms 观察一次状态，即使浏览器没有轮询也会记录；
每次下发都直接提交记录，不受浏览器最近 800 条命令缓冲的限制。
浏览器提供的全部原始检测、重定向输入/结果、目标、SDK 写入通过 `frame_id`
关联；SDK 启用时写入的软件/反馈起步目标也单独记录。

`sent=true` 表示 SDK `sendOnce()` 返回成功，不能据此认定电机执行完毕。
`telemetry.latest_host_command` 仅关联观察此快照前最近一次成功的主机写入，
不代表固件逐条确认。失败下发不会覆盖这项关联。

### 单位与离线通道

- self 的 14 路直线回传位置按 `-SDK pos_act/pos_ref` 转为命令坐标的毫米值；
  self 旋转和 MIT 使用原方向的度数。实时反馈反算与启动姿态读取采用相同的直线符号约定。
- `telemetry.linear_feedback_sign: -1` 表示派生的 `pos_act_mm/pos_ref_mm` 已完成取负；
  查看器对旧 A3.2 日志补做一次转换，对带标记的新日志不重复取负。原始 `position/target`
  和 `sdk_state` 保留 SDK 数值，下发命令方向保持不变。
- MIT 已是输出轴角度，不再次除减速比；A3.2 保留自己的槽位顺序。
- SDK 原始字段始终保存在 `sdk_state`。离线、缺失、无效或遥测过期通道的绘图
  测量值为 `null`，曲线断开、CSV 测量列留空，不把缓存零值画成真实位置。
- 该绘图规则只影响记录展示，**不限制普通目标下发**。
- 软件初始姿态、最后下发姿态和实际反馈分开保留，不将软件姿态伪装成测量值。

查看器按 `metadata.slot_maps` 显示恰好 20 个通道。拇指可选
`thumb_rota_block_joint` 与 `thumb_rotaback_joint2`；逐帧详情还显示最近的
`sdk_status` 和 `sdk_action/sdk_error`。

## 集成接口

```python
from a32_driver.recording import A32SessionRecorder

# 不传 output_dir：每次 start 按配置选择日志父目录下的 A32 文件夹。
recorder = A32SessionRecorder(model_id=model_id, hand_side=side,
                              channels=device.channels)
recorder.start({"platform": "motion_play", "serial_port": requested_port})
recorder.record_action("connect_requested")
recorder.record_state(device.get_state(), force=True)

# 写入完成后调用；失败时传 sent=False 和原始异常，不能记录成成功。
recorder.record_command(targets, pose=pose, sent=True, control_dt_s=elapsed,
                        frame_id=frame_id)
recorder.record_error(error, action="tick")

# 状态轮询或控制周期观察 SDK 缓存。
recorder.record_state(device.get_state())
status = recorder.snapshot()
filename, data = recorder.read_download()  # 仅在 HTTP 请求线程调用
recorder.stop(reason="disconnected")
```

下载只允许本录制器生成的文件名。运行中下载导出已刷新的完整行，可能不包含末尾
约 0.5 秒；断开后导出包含结束事件。下载不调用硬件、不改变目标、不暂停控制。

`export_state_capture(samples, output_dir, ...)` 用于把只读 HTTP 快照转成同一格式。
它保留采样时间，记录 `sdk_status`，仅在确有 SDK 遥测时生成 `telemetry`，不会
生成任何 `conversion/downlink/h7`。2026-09-28 的现场只读采样中，225 次请求均
显示设备已断开，覆盖约 44.8 秒，因此对应文件只有状态记录，没有电机遥测或下发
曲线。已按原始绝对采样时间保存到：

```text
E:\TsenseI_hand_logs\A32\hand_control_20260928_163819_652621_sdk_capture.jsonl
```

## 验证

- `online_control/online_motion_play/test_a32_recording.py`：两款 A3.2 的槽位映射、
  失败写入关联、缓存去重、离线不补零、非有限数值、文件范围限制及只读快照导出。
- `tests/test_control_log_viewer.cjs`：两款 A3.2 均显示 20 通道、两路 MIT 拇指、
  状态/错误详情、时间/周期和缺失测量导出，并继续覆盖原有 A3.0/V2 查看器功能。
- `tests/test_a32_tracking_recording.py`：手势输入到 SDK 记录的完整事件链、两款模型、
  主机时间、独立状态采样和失败下发。
- 测试只使用构造状态和文件，不连接设备，也不调用 SDK exe。
