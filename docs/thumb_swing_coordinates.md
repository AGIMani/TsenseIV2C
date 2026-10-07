# thumb_swing 坐标统一（2026-09-16）

左右手当前 URDF 都采用 `q_new = 1 - q_old`，单位为 rad。此次调整重定义关节零位与正方向，保持对应物理姿态。

| 原 URDF / UI 值 | 新 URDF / UI / H7 值 | USB R05 角度 |
|---:|---:|---:|
| +1 | 0 | 0° |
| 0 | 1 | 57.2958° |
| −1 | 2 | 114.5916° |

## 模型与显示

- 左手：origin yaw 从 −0.29927 改为 −1.29927，axis 从 −Z 改为 +Z。
- 右手：origin yaw 从 +0.29927 改为 +1.29927，axis 从 +Z 改为 −Z。
- 两侧关节限位统一为 `[0,2]`；UI、URDF 显示、碰撞 MJCF、H7 使用同一数值。
- 项目唯一默认状态为全零 `init`（包括 thumb_swing=0）；量程中点 `1` 只表示几何中位，不是默认姿态。Ruckig 运动约束与算法不变。
- Form2 滑块及数值框范围为 `0..2 rad`。online 复用同一 Form2 后端并读取更新后的左右 URDF。
- Metal Studio 新场景采用 schema version 2；指向项目内置左右手模型的 version 1 场景会转换旧 thumb_swing 坐标。

## 下发与回读

`convert_ui_to_hardware` 和 `convert_hardware_to_ui` 对 thumb_swing 均为同值映射，夹紧到 `[0,2]`。USB 层只做 rad→deg，发送到 R05；不再做旧的 `1-q_ui`。

这保证同一物理姿态在改版前后的 H7 指令一致。新 UI 值 `0` 对应原 UI 值 `+1`，不能把旧 UI 数值直接传给新 UI 接口。

## 旧动作文件与外部协议

既有动作文件没有坐标版本标记，因此继续保留历史 `[-1,1]` 存储格式，不能根据数值是否落在 `0..1` 自动判别版本。

| 边界 | 转换 |
|---|---|
| 左手动作文件 → UI | `ui = 1 - file` |
| 右手动作文件 → UI | `ui = 1 + file` |
| UI → 左手动作文件 | `file = 1 - ui` |
| UI → 右手动作文件 | `file = ui - 1` |
| 历史 External / Protocol → UI | `ui = 1 - protocol` |
| UI → 历史 Protocol | `protocol = 1 - ui` |

旧动作文件无需改写；桌面端与 online 导入、保存、导出均通过上述转换。新动作应通过 UI 保存，不能将 UI `[0,2]` 值直接写入旧格式文件。

本项目原生 retarget 求解器直接读取新 URDF，输出已经属于新 `[0,2]` 坐标，不经过 External 旧协议转换。右手使用共享左手求解器：输入点云先镜像，输出与 warm start 只更换左右手键名前缀，关节数值保持不变。左右 URDF 已在相同 q 下互为镜像，2026-09-18 已移除错误的 `2-q` 补偿，详见[拇指姿态问题档案](issues/20260918_thumb_retarget.md)。

## 验证

`tests/test_thumb_swing_rebase.py` 检查左右 URDF 在端点及中间值的物理旋转不变、MJCF 对齐、旧动作文件到显示/H7 R05 的映射、遥测逆变换、默认姿态和右手 warm start。协议、Form2、online、Metal Studio、retarget 与控制链测试同时覆盖相关路径。

完整回归命令：

```powershell
E:/conda/envs/hand_dex/python.exe -m pytest tests tests/smoke_test_hand_angle_protocol.py online_control/online_motion_play/test_server.py online_control/online_motion_play/test_deployment.py -q --basetemp=online_control/online_motion_play/runtime/pytest-thumb-final
```

结果：216 项通过。另验证了右手生成器的镜像关节与当前右手 URDF 一致、独立 CLI 内嵌协议的语法和下发端点。online 部署包已重新生成。

当前验证为软件测试，没有连接 H7 实机执行。
