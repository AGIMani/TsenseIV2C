# A3.0 本地浏览器 Worker 与 1–4 ms 下发测试

2026-09-22，`online_motion_play` 工作区。**已实现并验证 Worker 隔离，但“每包均在 1–4 ms”未通过。** 没有连接实体 USB/H7，没有部署到 node0。最终版两次 Worker 测试连续运行 10 秒，仍有 10–12 ms 长尾；不能以平均 Hz 或 p99 代替每包上限。

## 最终浏览器结果

本机 Chromium 152 实测 DedicatedWorker 中 `navigator.serial`、`getPorts()` 存在，localhost 为安全上下文。基准在前台打开，Python/HTTP 与浏览器在同一电脑。每项采样 10 秒，20 关节目标每秒在各自范围的 15%/45% 之间切换，持续经过实际 Ruckig、机构转换和编码。主线程压力为每 100 ms 连续绘制 Canvas 40 ms。

| 场景 | 位置包数 | 中位间隔 | p99 | 最大间隔 | 1–4 ms 占比 | 超过 4 ms | 结束状态 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| Worker 正常负载 | 6,497 | 1.5 ms | 2.4 ms | 11.6 ms | 99.89% | 7 次 | 持续连接/使能 |
| Worker + 主线程繁忙 | 6,605 | 1.4 ms | 2.2 ms | 10.8 ms | 99.95% | 3 次 | 持续连接/使能 |
| 主线程收发 + 繁忙对照 | 66 | 1.6 ms | 45.6 ms | 45.6 ms | 98.46% | 1 次 | 跳变保护停止 |

三项均没有小于 1 ms 的间隔。Worker 有效发送频率约 649/659 Hz，**1 kHz 是规划采样率，不是已达成的 USB 发送率**。对照组仅发送约 0.146 秒即保护停止，不能把其 66 包除以 10 秒当作持续吞吐量；它是同一套新增保护逻辑放在主线程的对照，并非历史版本逐字节复测。

此前一次正常负载在第约 8 秒发生位置跳变并停止，另一次隐藏页测试触发网络积压检查。这些失败保留在 [结果 JSON](a30_browser_worker_results.json)，最终两次成功不能证明长时间不会停止，也不能证明缓冲池消除了调度长尾。具体长尾来自 Python/OS/HTTP/浏览器哪一段，尚未做跨进程时钟关联定位。

**测量点是模拟串口 WritableStream 的 write 回调。** 基准在真实 Worker 中运行生产 BrowserSerial 收发核心；生产 Worker 的 RPC、端口匹配和连接取消另有单元测试。模拟遥测每 4 ms 提供一帧，电机在线掩码均为零，使用现有 fallback 起步规则。未测真实 USB 驱动、H7 接收、CAN 转发或电机闭环，也未完整复现 3D 场景；40 ms Canvas 阻塞是明确可重复的主线程干扰。

## 实现

- 页面只做用户授权、界面和 3D。专用 Worker 持有已授权端口，执行网络流解析、遥测读取/上传、串口发送。两个已授权端口若 USB 标识相同会拒绝猜选，要求仅保留目标端口授权。
- 使用固定 1 MiB 流解析缓冲区、可回收网络记录缓冲池、复用 20 关节数组和预分配间隔环形数组。高频路径不用 Base64/HEX 字符串；正常关闭逐帧 HEX 日志。HTTP/Streams 自身及记录对象仍有分配，并非零 GC。
- Python 按绝对 deadline 调度，错过周期只发送当前最新位置。浏览器的网络读取与串口写入分别推进，仅合并相邻纯位置记录，不跨越使能/找零命令。串口逐包写，采用约 1.2 ms 最小发送间隔，并按关节限速延后发送；不突发补发网络积压历史位置。MessageChannel 让定时器从新的消息任务启动，避免递归 timer 的 4 ms 下限，不做同步忙等。
- A3.0 动作播放 Ruckig 步长由 2 ms 改成 1 ms；共享手势项目仍是 500 Hz。侧摆 1.5 rad/s、其他关节 2.5 rad/s，加速度 10 rad/s²、jerk 80 rad/s³，均未提高。采样周期和这些约束独立设置，与 [Ruckig 官方说明](https://docs.ruckig.com/tutorial.html) 一致。Worker Web Serial 的接口见 [接口说明](https://developer.mozilla.org/en-US/docs/Web/API/WorkerNavigator/serial)，本轮还做了实际浏览器能力探测。
- arm 首帧及实时轨迹在 HTTP 内携带 `JQP1 + 20 float32 + 原始108字节`，USB 前剥离元数据，H7 协议不变。USB write 成功后才更新“上一已发送关节位置”。最大允许跳步为 `v_max × 4 ms + 0.000015 rad`；长时间卡顿不会扩大该允许值。超出则停发、执行原停止命令并断开，需手动重新连接。位置队列驻留超过 20 ms 或控制队列超过 64 条也停止。独立找零保持原始遥测位置的维护命令不属于这一路 Ruckig 元数据检查。

相邻包的角度跳步上限约为侧摆 0.344°、其他关节 0.574°（含浮点容差）；这是模型关节角，不是直线毫米或电机端角度。该检查约束位置跳变和平均步进速度，**不证明物理发送时间轴上的加速度/jerk 与规划完全一致**。浏览器 write 完成也不是 H7 执行确认。

## 计算优化与回归

20 关节转换 p50 从 48.4 µs 降至 36.9 µs；108 字节编码从 70.2 µs 降至 30.1 µs。主要优化是内建数值校验快路径、缓存不可变通道布局；没有删除机构/有限数值检查。1–4 ms 模拟调用间隔的完整 tick p99 为 176–190 µs，另有一次 2.287 ms 最大长尾。详见 [计算基准与原始记录](a30_python_1khz.md)。

Python 计算/限位/轨迹回归 272 项通过；浏览器传输、HTTP、部署基础回归 42 项通过，新增元数据合并测试后相关传输/部署子集 20 项通过。JS 17 项通过，覆盖包校验、断开后不重放、过期包拒绝、USB 失败不推进连续性基线、元数据剥离、位置跳变停止、流拆包/粘包、相同设备 ID 拒绝猜选和连接取消竞态。

```powershell
& 'C:/Program Files/nodejs/node.exe' --experimental-default-type=module --test online_control/online_motion_play/test_browser_serial.mjs online_control/online_motion_play/test_serial_worker.mjs
```

## 本机复现

从仓库根目录创建独立配置（避免使用实际动作数据目录），再启动测试服务：

```powershell
$benchDir = Join-Path (Get-Location).Path 'online_control/online_motion_play/runtime/worker-benchmark'
New-Item -ItemType Directory -Force -Path $benchDir | Out-Null
@{host='127.0.0.1';port=8771;data_dir=$benchDir} | ConvertTo-Json | Set-Content -Encoding utf8 (Join-Path $benchDir 'config.json')
& E:/conda/envs/hand_dex/python.exe -X utf8 online_control/online_motion_play/server.py --config (Join-Path $benchDir 'config.json') --no-browser --serial-benchmark
```

打开 `http://127.0.0.1:8771/bench-worker.html`，保持前台，依次点击能力检查、Worker 正常、Worker 主线程繁忙、主线程对照。页面不申请 USB 权限，不连接物理串口。测试文件不进入部署包；只有显式开启 `--serial-benchmark` 且绑定 loopback 时才提供测试路由。生产主页的切后台停止逻辑保持不变。

## 对目标的判断

该方案能明显隔离渲染造成的数十毫秒阻塞，常态落在 1–4 ms；**当前 Windows/浏览器链路仍达不到每包 1–4 ms 的验收要求**。下一步实机应在 H7 收到完整帧处记录序号/时间戳，再与发送端比较，确认 USB 批量传输和固件处理；如果每包上限是硬要求，需要 H7 定时插补/本地实时执行端参与，仅继续提高浏览器采样率不足以证明达标。
