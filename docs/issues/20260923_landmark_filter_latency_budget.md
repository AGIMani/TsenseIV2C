# One Euro 滤波优化：延迟增幅不超过 5% 的离线验收

日期：2026-09-23。范围：A3.0 网页控制的米制关键点滤波。继续使用当前 MediaPipe 模型。

## 修改及约束

采用 [One Euro 作者的调参顺序](https://gery.casiez.net/1euro/)：先评估静止平滑，再调整速度响应；递推系数遵循 [MediaPipe 官方实现](https://github.com/google-ai-edge/mediapipe/blob/master/mediapipe/util/filtering/one_euro_filter.cc)。保留项目每个三维点统一权重的设计，以维持旋转不变性。

生产改动位于 `online_control/online_motion_track/landmark_filter.py`：

| 参数 | 修改前 | 修改后 |
|---|---:|---:|
| min_cutoff / Hz | 1.2 | 1.17 |
| beta | 80 | 82 |
| derivative_cutoff / Hz | 1 | 1 |
| noise_speed / m/s | 0.01 | 0.01 |
| max_cutoff / Hz | 15 | 15 |

将 `alpha = 1/(1 + 1/(2π f dt))` 等价改写为 `f/(f + 1/(2π dt))`，在速度、位置两个递推中共享标量项。用三维点积计算速度模长，减少临时数组和重复运算。700 个不等间隔输入的回归测试验证：在旧参数下，优化计算与旧实现输出数值一致，容差为 1e-14 绝对误差、1e-13 相对误差。

低速局部时间常数比例最多为 `1.2/1.17 = 1.025641`。这不是任意动作端到端延迟的证明；仍须用实际阶跃、相位、缓慢移动和整条服务端处理来验收。beta 小幅增加用于补偿动作起始响应。

本次保持单帧因果递推，不新增等待窗口、预测或姿态锁定。异常输入校验、真正丢手/超过 300 ms 的观测间断处理、自动恢复保留 retarget 历史和滤波状态均保留。

## 对照基线和方法

- 冻结本轮修改前的生产实现到 `tests/fixtures/landmark_filter_before_20260923.py`。对照不是更早、延迟更大的 legacy 滤波器。
- `tools/benchmark_landmark_latency.py` 将 1.05 设为通过上限；任一正式检查失败返回非零退出码。
- 合成输入包含 0.2–30 mm 的 7 个幅度、0.25/0.8/2 Hz 的 3 个频率，分别测正弦相位延迟、阶跃达到 50%/90% 的延迟以及恒速斜坡的跟随延迟。阶跃指标从输入出现时刻计时，首个观测即可达到阈值的情况记为零，不额外加一帧稀释增幅。
- 检查白噪声、有时间相关性的噪声、幅度保留、阶跃超调。测试使用独立于最初参数探索的噪声种子。
- 正式脚本覆盖 15/30/60 FPS，另对 24/50/90 FPS 作留出验证。
- 计算耗时采用同帧交错 A/B、交替执行顺序、丢弃预热帧；避免整批 A 后整批 B 的处理器频率或系统负载漂移影响比较。
- 回放 `hand_control_20260923_170707_315000.jsonl` 全部 1284 个原始观测，包含无手帧。两版均使用已修复 retarget 自动重置的代码。四轮交错运行真实 `A30WebSession.process`，覆盖关键点滤波、retarget、Mink、控制目标入队及状态组装；关闭实时控制线程，没有相机/网络/USB 操作。

## 验收结果

正式结果：`logs/landmark_latency_20260923/metrics.json`，`passes_offline_gate=true`。

| 检查项 | 修改前 | 修改后 | 变化 |
|---|---:|---:|---:|
| 滤波 CPU 中位数 | 0.0204 ms | 0.0177 ms | -13.24% |
| 滤波 CPU P95 | 0.0256 ms | 0.0223 ms | -12.89% |
| 服务端整帧处理中位数 | 10.05825 ms | 10.05380 ms | -0.04% |
| 服务端整帧处理 P95 | 11.06325 ms | 11.032625 ms | -0.28% |
| 合成动作响应最大延迟比 | 1 | 1.042762 | +4.28% |

每轮独立的服务端 P50/P95 也均通过 1.05 上限。服务端耗时差异很小，按“基本持平”解释，不将其宣称为有统计显著性的加速。

留出帧率结果在 `logs/landmark_latency_20260923/holdout_rates.json`：24/50/90 FPS 的最大动作延迟增幅分别为 3.50%、2.98%、2.73%，均通过。

合成静止白噪声 RMSE 降低约 1.04%–1.16%，相关噪声降低约 0.53%–0.88%。记录中预先选定的五个低运动片段如下；它们不是经过动作标记或外部设备确认的绝对静止。

| 片段 / s | 旧关键点波动 RMS / mm | 新关键点波动 RMS / mm | 降低 |
|---|---:|---:|---:|
| 17–19 | 0.395088 | 0.394359 | 0.18% |
| 31.5–33.5 | 0.498726 | 0.496319 | 0.48% |
| 36–38 | 0.346703 | 0.345399 | 0.38% |
| 41–43 | 0.475098 | 0.473805 | 0.27% |
| 43.5–45.5 | 0.259577 | 0.257782 | 0.69% |

改善幅度有限，是严格延迟预算下的保守调参。retarget 非线性求解后的关节波动并非所有窗口都下降：31.5–33.5 s 和 36–38 s 的关节标准差 RMS 分别上升约 0.34% 和 0.60%；其余三段下降。不能宣称真机抖动已消除，或所有关节均变得更稳定。

## 被拒绝的方案

降低到 0.9 Hz 会使慢动作跟随延迟增加约三成，未采用。1.15 Hz、beta=80 的方案虽然局部时间常数只增加 4.35%，但特定小基线阶跃延迟增加超过 20%，也未采用。失败结果保存在 `logs/landmark_latency_20260923/rejected_1_15.json`；最终参数按真实响应测试收紧到 1.17 Hz、beta=82。

## 回归与复现

88 项 Python 回归通过，包含滤波的 5% 响应门槛、幅度保留、无超调、不等间隔输入数值等价、旋转不变性、非法输入、丢手恢复、retarget 连续恢复、直接模式、碰撞保持及路径保护。

```powershell
E:\conda\envs\hand_dex\python.exe tools/benchmark_landmark_latency.py "C:\Users\ONKIALL\Downloads\hand_control_20260923_170707_315000.jsonl" --output logs/landmark_latency_20260923/metrics.json
E:\conda\envs\hand_dex\python.exe -m pytest tests/test_responsive_landmark_filter.py tests/test_a30_retarget_recovery.py tests/test_a30_control.py tests/test_a30_continuous.py tests/test_direct_filter.py tests/test_direct_preview.py tests/test_a30_plateau.py -q
```

本次通过的是上述可复现的离线延迟门槛。相机曝光、浏览器 MediaPipe 推理、网络、USB/固件和机械响应未实测，不能将服务端耗时当作真机端到端延迟保证。代码已在本地更新，重启实际运行的计算服务后生效；本轮未部署远端服务。
