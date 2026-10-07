# 关节轨迹参数与算法评估（2026-10-03）

## 当前默认值

`src/utils/motion_limits.py` 是 Python 轨迹规划默认参数的统一入口：

| 参数 | 所有关节的默认上限 |
| --- | --- |
| 速度 | 10 rad/s |
| 加速度 | 80 rad/s² |
| jerk（加速度变化率） | 200 rad/s³ |

覆盖 A3.0 / A3.2（含 withid）实时跟随和动作播放，以及 V2 / Qt 的 Ruckig 插值器，左右手、侧摆、旋转、屈曲使用同一默认值。A3.0 屈曲此前为 20/400/600，此次对它是降低上限。A3.2 实时此前为侧摆 1、其他 2 rad/s，a=8、j=80；播放此前为 1/4/40。

已有显式覆盖接口保留：旧版 Qt/V2 的 `RUCKIG_*` 环境变量、速度缩放，以及 `configure_ruckig` 的 acceleration/jerk 关键字参数。播放动作的 `minimum_duration` / transition 仍决定最短过渡时间。A3.0/A3.2（含 with-ID）手势直控模式直接发送限位后的最新关节目标，不使用 Ruckig、自适应指数滤波或碰撞矫正，也不受这组 v/a/j 约束；V2 原有直控自适应指数滤波保留。浏览器旧版下发保护默认速度同步为 10，若状态携带逐关节 motion_limits 则优先使用状态值。

控制周期不变：A3.0/A3.2 实时为 2 ms（500 Hz），A3.0 播放为 1 ms，A3.2 播放为 10 ms。修改仅作用于重新创建的控制器；已运行服务需要重启。本次未连接或驱动实物。

## 采样周期与响应

控制线程按周期从连续规划轨迹取得下一组关节目标，再编码下发。输入关键点帧率、IK 更新率、轨迹采样率、实际写入频率、电机内部控制频率彼此不同。

500 Hz 改成 2 kHz 会把周期从 2 ms 缩短为 0.5 ms。仅考虑随机到达目标等待下一个控制 tick，理想平均等待由 1 ms 降为 0.25 ms；不会把运动约束、输入滤波或求解滞后缩短四倍。当前 A3.2 原生发送接口在 Linux 和 Windows 都强制至少 1 ms 写入间隔，因此仅改 PERIOD 无法实现 2 kHz 实际下发。

## robotoy 评估

研究版本为 [julyfun/robotoy 331f676](https://github.com/julyfun/robotoy/tree/331f67614c30a7c1bdfacac15422a3aa77e730ce)。`ItpState` 是逐轴启发式 jerk 选择加离散积分，支持目标位置、速度和加速度，并非 Ruckig 接口的直接替代品。其 C++ 实现将 jerk 限幅乘以 1.1；没有当前安全层依赖的整段位置极值、按手指时间同步和状态提交接口。[源码](https://github.com/julyfun/robotoy/blob/331f67614c30a7c1bdfacac15422a3aa77e730ce/robotoy/itp_state/itp_state.cpp)

仓库说明还记录了在 `a_max² > v_max*j_max` 区间的跟随问题；10/80/200 满足 6400 > 2000。[作者说明](https://github.com/julyfun/robotoy/blob/331f67614c30a7c1bdfacac15422a3aa77e730ce/robotoy/README.md)

本机 WSL 离线对照：20 个同信号关节、5 个 Ruckig 手指组；Ruckig 0.19.4，robotoy C++ -O3；两者目标速度/加速度均为零；不含传感器、滤波、IK、碰撞和通信。运行脚本和原始结果位于 `.work/trajectory-review/benchmark.py`、`results.json`。时延数字仅是合成信号的规划响应，计算耗时包含 Python 调用和分组封装，不是纯 C++ 算法基准或实物端到端时延。

| 10/80/200，500 Hz | 当前 GroupedRuckig | robotoy |
| --- | ---: | ---: |
| 0.5 rad 阶跃达到 90% | 318 ms | 354 ms |
| 阶跃进入并保持 ±1% | 378 ms | 418 ms |
| 0.5 rad、0.5 Hz 正弦最佳拟合滞后 | 82 ms | 102 ms |
| 正弦每次调用耗时中位数 | 54.7 µs | 3.7 µs |
| 测得最大 jerk | 200 rad/s³（浮点容差内） | 220 rad/s³ |

Ruckig 使用旧 A3.2 非侧摆参数 2/8/80 时，阶跃 t90 为 452 ms；新参数降为 318 ms。新参数提高到 2 kHz 后，t90 为 316.5 ms，正弦最佳拟合滞后为 82.5 ms；周期提高对这些运动响应改善很小。旧参数下 robotoy 的 0.5 rad 阶跃超调约 0.141 rad。另保留了 1 Hz 压力用例；旧参数不够跟上其峰值速度/加速度，2 kHz Ruckig 发生一步求解异常，错误原文保存在结果中，不能宣称这些测试证明了 2 kHz 的稳定性。

结论：robotoy 的单次计算更省时，但本组新参数下跟随反而更慢，且不严格遵守所设 jerk 上限，不建议直接替换。若适配它，至少需要补齐严格运动限幅、制动极值/关节限位、按手指同步、碰撞重试时状态回滚和变步长行为；`ItpRingApi` 还会引入其配置的缓存时域，不适合原样接入最新目标链路。

## 其他候选

- [ReReflexxes](https://rpc.lirmm.net/rpc-framework/packages/rereflexxes/index.html)：Type II 在线轨迹生成，约束速度/加速度，不提供当前要求的 jerk 约束，不能等价替代。
- [rsruckig](https://github.com/petrikosk/rsruckig)：MIT 的 Rust 移植，提供独立实现的 Trackig 动态跟踪接口；可作为后续离线候选，但没有在本项目测量其时延，也不是现有 Python Ruckig API 的即插即用替换。
- [Ruckig 官方跟踪说明](https://github.com/pantor/ruckig#tracking-interface)：把不断变化的目标直接作为点到点终态会产生跟随滞后；Pro 的预测跟踪接口为另一种策略，非社区开源功能。当前项目 target_velocity/target_acceleration 为零，可继续评估目标导数与受限前瞻，但须保留原有限位/碰撞检查，不能从计算快直接推出实物响应快。

## 验证

360 项 Python 测试、32 项 JavaScript 测试通过；1 项仅适用于 Windows SDK 的测试在 Linux 跳过。覆盖实时轨迹、反向运动的 v/a/j 约束、左右手和两款 A3.2 参数、动作播放、碰撞、丢手制动、Qt 下发保护及浏览器协议。额外检查 track / track-full / play 打包清单均包含新的共用参数文件，downlink 版本哈希覆盖该文件，`git diff --check` 通过。日志保存在 `.work/trajectory-review/tests-*.log`。没有执行实物测试或重启正在运行的服务。
