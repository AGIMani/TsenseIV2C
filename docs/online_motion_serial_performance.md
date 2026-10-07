# Online Motion Play 串口收发优化

以下保留 2026-09-21 早期二进制请求版本的基准。后续已迁移到 `online_control/` 并引入两网页共用的持续二进制推送；现行部署、架构和新增基准见 [online_cmd.md](../online_cmd.md)。下表不是当前 Chrome 或 H7 实测。

## 测量结果

Windows，本项目 hand_dex Python 3.11.15，localhost 独立测试服务，3秒采样。
JSON与二进制对比使用相同的新500 Hz规划器；JSON客户端保留原20 ms交换节拍。
这是传输方式对比，不是两份完整历史版本的整机对比。

| 测试方式 | 位置帧接收/写出 | 交换耗时 P95 | 遥测上传/接收 |
|---|---:|---:|---:|
| JSON + 20 ms交换 | 48.5 Hz | 1.243 ms | 23.5 KiB/s |
| 二进制 + 有帧唤醒 | 498.7 Hz | 2.265 ms | 239.6 KiB/s |
| 实际BrowserSerial JS + fetch + 模拟Web Serial | 499.7 Hz | 未单独采样 | 208.7 KiB/s |

二进制请求会等待下一帧，因此交换耗时包含生产等待，不能直接用它与立即返回的JSON请求比较网络延迟。
接收吞吐受模拟反馈生产频率影响，这些数值不是链路最大带宽。
JS测试使用Node的ReadableStream/WritableStream模拟USB，没有Chrome渲染负载、物理USB、真实H7或node0网络。

## 修改

- A3.0 的轨迹采样从100 Hz、位置生产从50 Hz提高为500 Hz，独立线程调度；Qt负责编辑器和序列状态更新。
- 独立线程与UI共用的设备状态由可重入锁保护，机构转换缓冲独立；关闭会话时回收工作线程。
- 当前Windows环境的 `time.get_clock_info('monotonic').resolution` 为0.015625秒，`perf_counter` 为0.0000001秒。高频调度、反馈时戳、帧年龄和动作计时改用后者。
- `/api/serial_binary` 在HTTP线程只操作同步字节缓冲；身份、会话token、请求序号及控制租约仍校验。
- 二进制请求包含 `MSR1`、32字节ASCII token、uint32小端序号、原始遥测；响应包含 `MST1`、同一token/序号、uint16帧数，以及逐帧uint16长度、float32年龄ms和原始帧。
- 只有新前端与新后端协商 `binary-v1` 才启用此路径；JSON交换仍兼容。等待新帧时释放缓冲锁，写入/关闭均唤醒等待者。旧会话不会消费新会话帧。
- HTTP/1.1复用连接，TCP_NODELAY，拒绝请求后关闭连接以避免未读请求体污染后续请求。
- 浏览器RX按块收集，打包一次复制；下行同批帧全部校验后一次写入，保留控制顺序；位置帧持续合并为最新值。
- 页面滚动统计位置帧Hz、接收KiB/s和交换耗时，不将“服务器产帧”当作“USB写出”。

控制速度/加速度/jerk、CRC、超时断开、过期帧丢弃、停止命令、缺失反馈与边界修正规则均保留。
没有改变实体串口波特率，也没有修改固件、桌面Form2或视觉网页。

## 复现

在仓库根目录执行：

```powershell
& E:/conda/envs/hand_dex/python.exe -X utf8 online_control/online_motion_play/bench_serial.py --browser --output online_control/online_motion_play/runtime/serial-benchmark.json
```

脚本创建临时数据目录和空闲localhost端口，启动独立server，只向内存串口发送模拟遥测，并在结束时回收子进程。
不会触碰8765上的现有服务或打开实体USB。

应用到本地需要重启server并刷新网页；应用到node0需要重新部署。
启用实时控制后以页面的实际“位置下发Hz”为准。无需额外安装运行时依赖。
