# 唯一默认状态：全零 init

项目默认状态统一为 **全部 21 个关节等于 0 rad**，包括 `thumb_swing`。左右手只更换关节名称前缀，没有各自可编辑的默认姿态表。

## 定义与入口

- `src/joint_schema.py`：`default_joint_position()` 始终返回 `0.0`，`get_init_pose()` 生成全零姿态的新副本。
- `src/utils/hand_init.py`：导出同一个 `get_init_pose()`，并集中保存回到 init、恢复跟随的过渡时长；原 `hand_origin.py` 已移除。
- 主窗启动、切换左右手、开启同步、手部消失后的回位、手部重新出现后的接回，统一使用 init。
- Form2 / online 滑块首次创建和缺省关节值为 0；两套 URDF 渲染器均通过 `reset_to_init_pose()` 恢复全零状态。
- Ruckig 初始位置和 reset 位置、retarget 冷启动参考均为全零。速度、加速度、jerk 等约束不变。
- Metal Studio 默认启动为 init，并提供一个「init（全零）」按钮。对指是用户主动选择的动作，不再作为启动默认姿态。
- 模型资源只保留 `assets/hands/TsenseI_hand/q_init.yaml`，替代重复的 `q_open.yaml` / `q_zero.yaml`。

## 坐标与数据边界

`thumb_swing` 仍采用上次统一的 `[0,2]` 坐标：新 0 对应旧 +1 的物理姿态，量程中点 1 不是默认状态。此次只改变默认状态，不再次调整 URDF 关节轴或零位。

全零 init 指 UI / URDF 关节空间。H7 下发继续使用既有转换和硬件限位：thumb_swing 下发 0；thumb_rota 和左手四指 swing 在下发边界仍夹紧至 0.5°。机械寻零命令不等同于软件 init。

历史动作文件的读写映射、用户保存的动作和显式载入的场景继续保留其姿态。读取真实硬件位置、加载动作或手势跟随后显示的非零值是当前状态，不是另一套默认状态。

## origin 命名清理

业务姿态、相关变量、方法、日志、时序常量和说明统一改称 `init`。URDF `<origin>`、几何坐标原点、HTTP Origin / 同源检查、CSS `subcontrol-origin` 属于格式或数学含义，保留其原名。

## 验证

`tests/test_init_pose.py` 检查左右手默认姿态副本、两套渲染器复位、模型资源、Studio 启动、retarget 冷启动以及 Ruckig 复位；online 测试检查 Form2 21 个滑块初值全为 0。

2026-09-16 验证结果：完整回归 **225 项通过**，118 个当前项目 Python 文件通过语法检查，online `--check --no-browser` 启动检查通过。部署包已重新生成；未连接 H7 实机执行。

```powershell
E:/conda/envs/hand_dex/python.exe -m pytest tests tests/smoke_test_hand_angle_protocol.py online_control/online_motion_play/test_server.py online_control/online_motion_play/test_deployment.py -q --basetemp=online_control/online_motion_play/runtime/pytest-init-final
```

已启动的桌面程序需要重启才能加载新默认值；已有 online 服务需要部署新包并重启服务。
