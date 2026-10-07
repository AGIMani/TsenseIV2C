# 加速算法接入核查（2026-09-18）

核查范围：本地 Qt 源码、本地网页服务 `127.0.0.1:8766`、node0 正在运行的网页服务。仅检查，没有重启或部署。

| 对象 | V2 | A3.2 |
| --- | --- | --- |
| 本地 Qt 源码入口 | 已调用预绑定等间隔查表 | 独立 A3.2 入口默认 `lut` |
| 本地网页运行进程 | 实测使用新查表 | 会话和帧响应均为 `exact` |
| node0 网页运行容器 | 实测使用新查表 | 会话和帧响应均为 `exact` |

## 证据

- Qt 主入口 `example/main.py` 优先加载本项目 `src`；`src/app/widget.py` 的下发经过 `usb.offline.send_joint_angles` 和 `joint_angles_to_hand_positions`，最终调用 `kinematics.finger_linkage`。
- A3.2 独立 Qt worker 和网页使用 `online_control/online_motion_track/pipeline_v3_2/pipeline.py`，创建独立 `A32StrokeMapper(hand_side)`。本地新进程加载该 mapper 默认返回 `lut`；没有与 V2 共用机构转换器。
- 本地网页 PID 23636 启动时间为 16:45:51，早于最新转换代码修改。实际 `/api/session` 和 `/api/frame` 的 A3.2 `actuation.method` 均为 `exact`，`valid=true`。
- node0 当前版本为 `20260918-164627`，容器 `tsensei-hand-web-app-1`，源码位于 `/srv/runtime/app`。容器中新进程加载 A3.2 默认是 `exact`；正在运行服务的会话和帧也返回 `exact`。
- 两端 V2 都使用独立临时会话的 `browser_rebase` 进行输出核验，未连接串口、未启用下发。选用能区分新旧查表结果的角度 `84.97106220060456°`，14 路实际行程与预绑定查表计算的最大差值为 `0.0 mm`，与禁用预绑定后的旧查表最大差值为 `1.1069908101291048e-6 mm`。
- 临时会话均在检查结束后调用 `/api/stop` 清理；测试没有操作硬件。
- 浏览器 `browser-hand-control.js` 使用服务端返回的 `linear_strokes_mm`，角度转行程算法运行于 Python 服务端。

## 版本差异

node0 三个核心文件的 SHA256 均与当前本地不同，尚未同步最新输入和边界修复：

| 文件 | 本地 SHA256 | node0 SHA256 |
| --- | --- | --- |
| `src/kinematics/finger_linkage.py` | `e0589c084708ddad2eb07a6e5ab2e2a86cec2326e43c4cf2450b5b73d034c4e9` | `63a0437a5e9808cd599221da04f126dd802e00923b1aefa1db15b4e82460283d` |
| `src/usb/offline.py` | `9428ad94439acb45b41f5a78d8f9247e49189a5cf23afc3d22668184983cf0e0` | `d2dfccd5241d1b5063dc960c9f797c8d87c842e8e5746ca3ac65cfd93dd946ea` |
| `src/a32_kinematics/mapper.py` | `fd2195b17205c17adce0c08bcf28da06be7926c92dc0a1269c7a502877ac6bb3` | `2e40cd2613922ca5c57de6875d607bfa321ddbb7e206d85b6122ccd15dfa3c94` |

## 生效所需操作

本地网页需要重启 Python 服务；刷新浏览器不能重新加载 Python 模块。node0 需要重新打包部署并重建容器，随后核对 A3.2 会话和帧的 `method=lut`。Qt 结论针对当前源码和新进程导入；未验证运行中的 Qt 实例或旧打包程序。
