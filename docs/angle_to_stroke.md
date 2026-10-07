# V2 提速与 A3.2 独立转换（2026-09-18）

输入与边界测试、异常修复和15万组姿态验证见 [转换验证报告](conversion_input_validation.md)。

V2 保持 `kinematics.finger_linkage` → `usb.offline` 的现有路径。
A3.2 使用独立的 `a32_kinematics`，只在 `online_control/online_motion_track/pipeline_v3_2/pipeline.py`
及其 A3.2 网页流程中调用；不导入 V2 连杆、槽位或符号转换，也不修改 V2 全局状态。

## V2 优化

- 启动时预绑定电机限位、软饱和参数与均匀角度表；运行时直接计算表格下标。
  几何指纹按手指检查，减少逐电机的重复限位/表查找；参数替换、原地修改或显式清缓存后重建。
- 均匀表由原始分段线性表重采样，并逐一检查全部旧节点。两个线性插值函数的差值在
  两组节点之间仍为线性，新节点处差值为零，因此旧节点检查给出整个区间的误差上限。
  默认三个电机分别使用8,161、16,321、32,641个节点，共享相同几何的预计算结果。
- 相对原查表的额外误差限定为 **1e-5 mm**（应用行程符号/缩放系数前），浮点舍入另计。
  无法满足该上限的自定义几何回到原转换路径；不静默放宽误差。
- 保留上下限软饱和、NaN/Inf及角度换算溢出的零位处理、行程系数、单指失败保持，
  非默认电机绑定继续走原路径。`HAND_LINKAGE_FIT=0` 仍使用原精确二分模式。
- 修正旧 `bench_linkage_fit.py` 将 Hz 标成 kHz 的输出错误。

本轮优化前后的2,004组姿态（变化角、超限、NaN/Inf）共28,056路输出对照，
最大差值 **0.00000510 mm**。本轮引入了有界的重采样误差；上一轮仅缓存限位时的0差值不再适用。

## A3.2 接口

已内置用户提供的 `a32_fast.py`、参考求解器及所需 URDF/MAT 文件，运行时无需
`D:/Qtworks/gravity` 或 MATLAB。来源校验值见 `src/a32_kinematics/source_snapshot.json`。
依赖 NumPy、SciPy，项目现有依赖已包含。在项目 `src` 导入路径下：

```python
from a32_kinematics import A32StrokeMapper

mapper = A32StrokeMapper("left")  # 启动时一次；默认 lut，支持 right
pose = {name: 0.0 for name in mapper.joint_names}  # 完整20关节URDF弧度字典
targets = mapper.convert(pose)  # 严格模式，非有限/缺失/跨模型/超限输入报错
linear_mm = targets.linear_strokes_mm       # 14路，按A3.2项目关节名索引
rotary_deg = targets.rotary_positions_deg   # 6路，按A3.2项目关节名索引

preview = mapper.preview(pose)  # 显式限幅，返回 clamped_joints_rad 和 conversion_ms
exact_mapper = A32StrokeMapper("left", method="exact")  # 可选精确解析模式
```

底层 `A32Converter().prepare()` 保留原接口：输入度，输出直线米/旋转弧度，电机编号1到20。
项目适配器负责 URDF 弧度与 mm/deg 的转换，摄像头和网页均默认使用 **lut**，
日志中 `actuation.method` 也为 `lut`，插值误差界不超过 **1e-9 m**。
建表时验证精确几何，运行时检查输入有限性和机构角度范围；`method="exact"` 可恢复逐次完整几何检查。
底层原始 `A32Converter().prepare()` 的兼容默认仍为 exact，不影响项目接入已选用的 lut。

小指必须按轴向映射：

| 原始转换模型 | A3.2 项目关节 | 类型 |
|---|---|---|
| pinky_MCP | pinky_MCP | 直线，Y轴屈曲 |
| pinky_SWING | pinky_swing | 旋转，X轴侧摆 |

其余名称一致。右手使用 `right_hand_` 前缀和镜像 URDF 的同值角度，不叠加 V2 符号规则。
食指、无名指 MCP 机构上限约 **87.2557°**，低于 URDF 的约89.9544°；严格接口报超限，
预览接口限幅并记录实际应用的关节角，不改变显示姿态。

A3.2 摄像头/网页结果附带 `actuation`，JSONL 每帧保存相同结果，网页本地记录增加
`a32_conversion` 事件；会话描述给出20路名称、类型和机构限位。转换失败返回
`valid=false`、错误原因和空输出。部署包已包含独立转换模块和模型数据。

这些是机构几何输出，尚未配置 A3.2 固件槽位与电机正负方向；`hardware.enabled=false`
保持有效，A3.2 不会借用 V2 的14+7串口协议。没有连接或向实机发送指令。

## 性能与验证

同机 AMD Ryzen 7 5800H，变化角输入，预热后每项10,000次；仅测转换，不包含重定向、
避碰、轨迹插值、串口或电机响应。系统负载影响绝对耗时。

本轮 Python 3.11.15（hand_dex）同进程对照，基线为上一轮已缓存限位的版本：

| 项目 | 平均 | 中位数 |
|---|---:|---:|
| V2 本轮优化前完整21路转换 | 76.3 μs | 75.7 μs |
| V2 当前完整21路转换 | **43.5 μs** | **36.3 μs** |
| V2 当前14路行程 | 34.9 μs | 27.8 μs |
| V2 重复角命中缓存 | 5.6 μs | 5.4 μs |
| A3.2 完整20路转换（exact，含适配） | 82.3 μs | 81.5 μs |
| A3.2 当前完整20路转换（lut，含适配） | **57.4 μs** | **56.1 μs** |
| A3.2 预绑定查表核心（度数组→SI数组） | 29.6 μs | 29.3 μs |

V2 本轮完整转换平均提速约 **1.75倍**；首次初始化及建表约75 ms，不在每帧循环中执行。
A3.2 原文档19.94 μs对应最后一行的接口范围，不能直接与包含项目适配的完整转换比较。
历史109 μs受当时负载影响，本轮采用同进程基线作对照，不跨运行直接计算加速比。

本轮原始统计：`docs/benchmarks/hand_conversion_prepared_lut_py311_20260918.json`（含 P95/P99）。
前一轮报告 `hand_conversion_py311_20260918.json` / `hand_conversion_py313_20260918.json` 保留作历史记录。

```powershell
python tools/bench_hand_conversion.py
# 可选：与保存的优化前源文件做同进程对照
python tools/bench_hand_conversion.py --v2-baseline PATH_TO_OLD_finger_linkage.py --output result.json
```

`hand_dex` 下本轮通过104项测试和376个子测试，覆盖 MATLAB 参考值、原始向量几何、
精确解/查表误差界、限位与非法输入、CAD轴向/链条核对、左右手、V2全局状态隔离、
无V2依赖的最小打包运行、网页响应与日志、V2控制链、全区间重采样误差、软限位、
极端输入、参数变更与清缓存、精确模式回退、查表不可用回退及拇指不依赖F2几何。

## A3.0 适配（2026-09-20）

`a30_kinematics.A30StrokeMapper` 复用 A3.2 的 9 路直线连杆换算（四指 MCP/PIP、拇指 MCP），将四指 DIP 和拇指 PIP 改为直接输出角度；总输出为 9 路 mm + 11 路 deg。已接入网页每帧响应和本机 `a30_conversion` 记录。网页完整运行环境另接入 Ruckig、采样碰撞检查和浏览器 Web Serial，使用从 A3_USB_Downlink.pyw 提取的 H7 协议；纯转换器本身不打开串口。详见 [A3.0 接口及示例](hand_v3_0_setup.md#角度转行程与协议编码接口)。
