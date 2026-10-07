# 2026-09-22：A3.0 手势 Ruckig 约束对齐 Qt

用户要求：将当前 Ruckig 的速度、加速度和 jerk 上限与 Qt 同步，并记录修改。

## 参数变更

范围为 `online_control/online_motion_track/a30/control.py` 的 A3.0 手势控制器，左右手均生效。基准为 `src/utils/ruckig_interpolator.py::_DEFAULT_CONSTRAINTS` 的 Qt Form2 源码默认值，不读取正在运行的 Qt 进程，也不继承其可能设置的 `RUCKIG_*` 环境变量。

| 关节类别 | 数量/手 | 速度：修改前 → 修改后（rad/s） | 加速度：修改前 → 修改后（rad/s²） | jerk：修改前 → 修改后（rad/s³） |
| --- | ---: | ---: | ---: | ---: |
| 弯曲 MCP/PIP/DIP | 14 | 2.5 → **4.0** | 15 → **30** | 120 → **200** |
| 四指侧摆 swing | 4 | 1.5 → **1.5** | 15 → **10** | 120 → **80** |
| 拇指旋转 | 2 | 2.5 → **2.5** | 15 → **20** | 120 → **120** |

A3.0 的 `thumb_rota_block` 与 `thumb_rotaback` 均映射到 Qt 的 rotation 类别。A3.0 没有 Qt 的独立 thumb_swing 类别；本地参数表仍完整保留该类别的 2 / 15 / 100，当前 20 轴不会使用它。

## 实现与行为

- 新增控制器局部参数表 `_QT_GESTURE_LIMITS`，按关节类型设置三组 Ruckig 上限；测试直接比对 Qt 默认表，避免两处参数无意漂移。
- 500 Hz / 2 ms 周期、整手 `Synchronization.Time`、零目标末速度/末加速度及状态连续传递均保持原有实现。此次对齐的是运动约束，不代表整个规划流程与 Qt 相同。
- 碰撞检查、关节范围、反馈起步和故障保护沿用现有逻辑。
- 参数策略只应用于手势控制器；公共 `configure_ruckig` 与动作播放控制器没有修改，避免影响独立的播放功能。
- 弯曲关节允许更快的运动变化，侧摆的加速度与 jerk 则降低至 Qt 默认值；尚未测量真实跟手延迟，不能据此承诺具体延迟下降幅度。

## 验证

执行并通过 **67 项**测试：

```powershell
& E:/conda/envs/hand_dex/python.exe -m pytest tests/test_a30_control.py tests/test_a30_device_timing.py --basetemp D:/Qtworks/hand/logs/gesture_pipeline_20260922/pytest_qt_limits_1 -q --tb=short
```

覆盖左右手 20 轴逐项对齐（14 弯曲、4 侧摆、2 旋转）；全关节目标反向变化时的速度/加速度/jerk 上界；反馈重置、碰撞暂停恢复、故障锁定、服务端不打开 USB，以及动作播放原有频率和约束的回归。

这是离线控制器/模拟设备验证，没有打开实际串口或相机，没有修改正在运行服务的参数。代码与记录已保存；结束当前控制会话后，重启本地手势服务才能加载新约束。本次未重启服务、未部署、未提交或推送工作区中的其他修改。

本记录接续 [帧链路与滤波优化报告](20260922_gesture_pipeline_optimization.md)。该报告中的 15 rad/s²、120 rad/s³ 是本次修改前的历史参数；当前手势源码参数以本记录为准。
