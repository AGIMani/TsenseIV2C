# 三种输入的 A3.2 软件链路延迟

本轮范围是 MediaPipe、Quest/WebXR、Wuji 手套，在隔离的 WSL 项目中优化。
Windows 原项目与 Conda 环境保留。实物未连接，本文的时间均为离线软件测量，
不代表摄像头曝光、SDK 设备内部处理、真实 Wi-Fi/以太网、USB/IP 或机械动作总时长。

## 链路与处理方式

当前输入策略：Quest/WebXR 与 Wuji 的 21 点直接进入坐标变换和重定向，不再执行关键点时间滤波；MediaPipe 保留原有滤波。该规则同时覆盖在线控制、纯预览和网页标定。手别、有限数值、骨架退化、帧序与时效检查继续执行。

A3.0、A3.0 with-ID、A3.2、A3.2 with-ID 的“直控模式”同时关闭 Ruckig、目标碰撞矫正、轨迹采样碰撞矫正和额外关节时间平滑，按现有控制节拍发送限位后的最新目标。切换模式结束当前会话，需手动重新开始识别、连接和开启下发。V2 的原有直控滤波不变。

~~~mermaid
flowchart LR
  Camera["MediaPipe 相机"] --> BrowserWorker["浏览器推理 Worker"]
  BrowserWorker --> Latest["仅保留最新待处理帧"]
  Latest --> Solver["WSL 隔离求解进程"]
  Quest["Quest WebXR"] --> Relay["发布骨架 / 事件唤醒"]
  Wuji["Wuji SDK 原生回调"] --> Relay
  Relay --> Solver
  Solver --> Control["A3.2 控制线程 / 2 ms 周期"]
  Control --> Native["Linux 原生 .so"]
  Native --> Device["设备通信与机械动作（未实测）"]
~~~

MediaPipe 的独立推理 Worker 和最新帧队列继续使用，先提交骨架，再绘图和记录。
Quest 发布骨架后由 WSL 消费，PC 页面不再取出骨架再发送回来。
Wuji SDK 回调只更新最新缓存并发通知，回调线程不承担求解；不经 node0。

常规网页控制三种输入都使用独立求解进程；网页的记录开关和控制日志仍可使用。
仅显式调用旧 API 的 recording=True 角度录像兼容路径保留进程内求解，避免改变其文件记录语义。
旧 HTTP bridge 和没有 A3.2 控制后端的预览路径保留原消费方式，不在 SDK 事件处理的收益统计内。

后台消费保留页面存活期限；求解过程不自行刷新页面心跳。客户端失联、断流、换流或关闭后，
在途计算不能继续提交旧目标。Quest/Wuji 输入超过 350 ms 失效；MediaPipe 待处理队列丢弃超过 120 ms 的候选帧。控制层原有 400 ms 输入超时、2 s 客户端超时、
限位、碰撞检测、运动限速和手动使能保留。过期帧直接丢弃，不排队补发。

这些修改不把 350 ms 超时当成处理等待；新鲜帧会尽快处理。500 Hz 是控制目标采样节拍，
不是相机/手套的新信息帧率，也不是 2 ms 的实物总响应时间。

## 滤波响应

原 A3.2 使用米为单位的关键点，但 One Euro beta=0.25 令运动时截止频率几乎保持 1.5 Hz。
现在使用已有因果自适应滤波算法，A3.2 参数为 min_cutoff=1.5 Hz、beta=82、
derivative_cutoff=1 Hz、noise_speed=0.02 m/s、max_cutoff=15 Hz。
归一化图像坐标滤波与 A3.0 参数不变；不使用未来帧或外推。

相同随机种子，25 mm 幅度、0.8 Hz 正弦运动叠加 0.5 mm 白噪声，
输入率分别为 30 / 60 / 90 / 120 Hz。结果为滤波自身引入的运动响应滞后：

| 指标 | 原 A3.2 | 优化后 |
|---|---:|---:|
| 正弦相位滞后（四种输入率） | 约 95 ms | 约 35 ms |
| 30 mm 阶跃达到 90% | 241.7–245.2 ms | 28.3–30.9 ms |
| 60 Hz 静止段 RMSE | 0.1321 mm | 0.1320 mm |
| 新滤波阶跃超调 | — | 0 |

相位搜索分辨率为 5 ms。阶跃统计以第一帧观测为起点；没有计入真实采样间隔和传感器延迟。
这些合成结果说明滤波响应改善，不等于真实机械手达到目标的时间。
实际动作仍受既有速度、加速度、jerk 和碰撞规则约束。
上述数据描述滤波算法本身；当前仅 MediaPipe 输入继续使用此米制滤波，WebXR/Wuji 已按来源跳过，不能把这些滤波滞后数字计入它们的新链路。下文历史 A/B 测量对应修改前的滤波配置。

复现（纯离线，不打开设备）：

~~~bash
OPENBLAS_NUM_THREADS=1 ~/.venvs/hand-track/bin/python \
  tools/benchmark_a32_input_latency.py --output /tmp/a32-filter-latency.json
OPENBLAS_NUM_THREADS=1 ~/.venvs/hand-track/bin/python -m pytest -q \
  tests/test_a32_input_filter_latency.py tests/test_responsive_landmark_filter.py
~~~

## 共享控制层离线 A/B

`control-isolation-ab.json` 为无 Chromium/pytest 并发负载的复测；首轮受 SwiftShader 干扰的结果单独保留在 `control-isolation-loaded-ab.json`，不用于净收益结论。每个来源做 before/after 与 after/before 两轮，每例预热后 2.5 秒。使用真实 FK 骨架、重定向、stop 碰撞检查、Ruckig、录制及本机 127.0.0.1 native TCP 写入；反馈健康状态为模拟。

两个版本都运行在同一 WSL 中，使用相同当前输入与滤波，仅替换共享控制实现。请求处理时间包括求解及状态返回，不含真实摄像头/Quest/手套采样，也不是电机响应时间。

| 来源 | 请求处理 P50：原实现 → 优化后 | native 写入间隔 P99：原实现 → 优化后 |
|---|---:|---:|
| MediaPipe | 40.66–45.34 → 11.70–12.07 ms | 4.44–4.85 → 3.42–3.53 ms |
| Quest/WebXR | 41.54–44.53 → 11.67–11.74 ms | 4.44–4.88 → 3.55–3.56 ms |
| Wuji | 11.77–12.25 → 11.60–11.96 ms | 原本已隔离，不能归因于新增隔离收益 |

全部样本没有出现小于 1 ms 的写入间隔，仍有少量超过 4 ms 的情况，不能保证硬实时。Wuji 在共享求解隔离这项上基本持平，事件唤醒效果另行测量。SDK 展示状态缓存显式提供龄期，真实写入/使能继续读取当前 SDK 状态；未以缓存代替安全检查。

## Quest / Wuji 接收到目标提交的 A/B

同一机器上每种来源交错测试优化前后各两次，每次 120 Hz 合成输入，60 帧预热后统计 360 帧。运行真实求解器与 localhost HTTP，测量终点为 `control.push`，未连接设备。两轮合计每个条件生成 720 帧。

| 来源 | 版本 | 有效求解 / 输入 | 接收到提交 P50 / P95 / P99 | 有效求解频率 |
|---|---|---:|---:|---:|
| Quest/WebXR | 原实现 | 331 / 720 | 16.04 / 22.53 / 41.22 ms | 55.2 Hz |
| Quest/WebXR | 优化后 | 600 / 720 | 14.12 / 18.32 / 19.04 ms | 100.0 Hz |
| Wuji | 原实现 | 577 / 720 | 12.87 / 17.23 / 18.42 ms | 96.2 Hz |
| Wuji | 优化后 | 612 / 720 | 13.90 / 18.15 / 20.49 ms | 102.0 Hz |

Quest 减少了页面二次转发，吞吐与尾延迟改善。Wuji 的有效求解频率小幅增加，但本窗口内单帧延迟小幅增加；不能把吞吐改善说成单帧降延迟。后台输入独立于 UI，仍受真实 IK 耗时和调度影响。这项对照同时包含输入、滤波和共享求解修改，不能把全部差异归因于事件唤醒。未处理输入保留在计数内，不记作零时延。

本表与前一表的请求处理时间具有不同边界，不能直接相减或相加。记录见 `metric-source-ab-summary.json`、`metric-source-ab-table.md` 和八份逐帧 JSON；复现脚本为 `benchmark_metric_ab.py`、`run_metric_ab.sh`。短窗口结果不等于设备上性能保证。

## 计时边界

必须分别记录以下时间，避免把不同含义的数字拼成“端到端”：

- 输入在服务端回调或发布完成，到求解目标提交：软件接收/唤醒/求解链路。
- 滤波对运动的响应：信号相位或阶跃响应，不是代码运行时间。
- 相邻 native 写入的间隔：控制节拍稳定性，不是新帧传播时延。
- 从真实传感器采样到电机反馈：需要设备和统一的时钟或测量装置，本轮未测。

证据存放于 E:\WSL\optimization；优化前代码备份为 pre-optimization-code.tgz，
迁移基线改动为 pre-optimization-working.patch。它们用于比对与恢复，不应直接覆盖正在使用的项目。

## 回归验收

统一 Python 回归：274 passed、1 skipped。唯一跳过项明确要求 Windows SDK；Linux 原生协议已有独立 TCP/PTY 验证。范围覆盖后台提交、取消/失联/换源、滤波、关节范围、碰撞、录制、SDK、WebXR 和打包入口。测试命令及完整输出为 `run-integration.sh`、`integration-pytest.log`。

扩展测试的 trimesh、OpenCV headless 和 cryptography 等依赖仅安装到项目 `.work/latency-test-deps`，三个运行 venv 未因此修改；测试用原始 URDF 只读复制到 `.work/latency-test-reference`。关节范围测试支持 `A32_REFERENCE_SDK`，无需假定 Windows 的相邻目录也存在于 Linux。版本和参考模型校验记录在 `test-environment.json`。首轮测试环境缺项已补齐，不将其当作运行时代码故障。

前端与真实浏览器验收使用合成输入和 fake SDK，不调用真实相机或机械手。SwiftShader 仅用于功能测试，不将浏览器测试中的时间当作实机性能证据。

最终前端结果：56 项 JS 检查通过；Quest XR 生命周期模拟通过；Quest 后台浏览器 71 个结果、fake SDK 后台浏览器 213 个结果、旧桥接兼容 125 个结果均通过。验证左右手、断流恢复、停止/切源，以及 SDK 持续产出但页面心跳过期后不再提交目标。无相机请求、真实连接、页面异常或多余 `/api/frame` 请求。使用真实 URDF / WebGL；SwiftShader 下测试场景缩小至 360×240 并跳过截图，功能断言保留。详见 `frontend-validation.json` 和 `frontend-audit.md`。

三个启动器 `hand-track --check`、`hand-wuji --check`、`hand-play --check` 通过；track/Wuji 本地首页和健康接口通过，原生控制能力可用且默认传输为 sdk。482 个打包文件存在，新后台模块、启动指南、性能说明和 Linux `.so` 均在 allowlist 中。详见 `startup-package-check.json`。所有启动检查未开始采集或使能设备。
