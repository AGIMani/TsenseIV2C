# 当前关节下发符号总表

更新日期：2026-09-16

## 1. 权威下发链路

当前所有实际调用 `UsbController.send_joint_angles()` 的界面/控制路径，均先调用
`protocol.hand_angle_protocol.convert_ui_to_hardware()`：

```text
MediaPipe / CloudXR / Form2 / 动作文件
        ↓
retarget 或 UI 关节角 q（rad）
        ↓  不变号
物理碰撞（可选）→ Ruckig → UI q（rad）
        ↓
convert_ui_to_hardware(hand_side)
        ↓  HW 角度空间；DownlinkGuard 只限速/限跳，不固定变号
UsbController.send_joint_angles()
        ├─ MCP/PIP/DIP：连杆逆解，乘 stroke_sign=-1 → linear mm
        └─ rota/swing：rad × 180/π → rotary deg
        ↓
USB 0x20 HAND_POS_BATCH：linear[14] + rotary[7]
        ↓
H7
```

主界面手势链和 Form2 使用彼此独立的 Ruckig 状态，避免互相污染；Form2 的实时滑块、
加载动作和动作序列均在 `convert_ui_to_hardware()` 前通过 Ruckig，并在转换后通过
DownlinkGuard。Form2 中不存在绕过该出口的 `send_joint_angles()` 调用。

`UsbController.send_joint_angles()` 接收的已经是 **Hardware 空间**，USB 层不会再次执行
左右手角度取反。`0x2B hand_type` 只配置左右手类型/找零方向，不会改变 `0x20` 数组符号。

## 2. 记号

- `q`：retarget/UI/Ruckig 输出的关节角。
- `+q`：同号进入 Hardware 角度空间。
- `-q`：进入 Hardware 前取反。
- `-CFn(q)`：正屈曲角经过第 n 类连杆逆解后，再乘默认 `stroke_sign=-1`；这是非线性映射，不等于固定的 `-kq`。
- 右手在 UI/Ruckig 中可使用 `right_hand_*` 键，但进入协议时会规范化为同一套左手槽位名称；数值按右手规则处理。

## 3. 21关节符号与 H7 槽位

| # | 关节（短名） | Retarget/Ruckig q 范围 | 左手 HW(rad) | 右手 HW(rad) | USB/H7 槽位 | 最终 H7 数值 |
|---:|---|---:|---:|---:|---:|---|
| 1 | thumb_rota | UI/Ruckig 0..1.57rad（约89.95°） | `clamp(q,0.5°,89.5°)` | 同左手 | R04 | 同号、无系数，rad→deg |
| 2 | thumb_swing | 0..2rad（0..114.59°） | `q` → `[0,2]` | `q` → `[0,2]` | R05 | rad→deg，线值域 `[0°,114.59°]` |
| 3 | thumb_MCP | 0..90° | `+q` | `+q` | L12 | 左右均 `-CF3(q)` mm |
| 4 | thumb_PIP | 0..90° | `+q` | `+q` | L13 | 左右均 `-CF1(q)` mm |
| 5 | index_swing | 0..0.696rad | `clamp(q,0.5°,39.5°)` | `+q` | R00 | 左手截断、右手直接rad→deg |
| 6 | index_MCP | 0..90° | `+q` | `+q` | L06 | 左右均 `-CF3(q)` mm |
| 7 | index_PIP | 0..90° | `+q` | `+q` | L07 | 左右均 `-CF2(q)` mm |
| 8 | index_DIP | 0..90° | `+q` | `+q` | L08 | 左右均 `-CF1(q)` mm |
| 9 | mid_swing | 0..0.696rad | `clamp(q,0.5°,39.5°)` | `+q` | R01 | 左手截断、右手直接rad→deg |
| 10 | mid_MCP | 0..90° | `+q` | `+q` | L09 | 左右均 `-CF3(q)` mm |
| 11 | mid_PIP | 0..90° | `+q` | `+q` | L10 | 左右均 `-CF2(q)` mm |
| 12 | mid_DIP | 0..90° | `+q` | `+q` | L11 | 左右均 `-CF1(q)` mm |
| 13 | ring_swing | 0..0.696rad | `clamp(q,0.5°,39.5°)` | `+q` | R02 | 左手截断、右手直接rad→deg |
| 14 | ring_MCP | 0..90° | `+q` | `+q` | L00 | 左右均 `-CF3(q)` mm |
| 15 | ring_PIP | 0..90° | `+q` | `+q` | L01 | 左右均 `-CF2(q)` mm |
| 16 | ring_DIP | 0..90° | `+q` | `+q` | L02 | 左右均 `-CF1(q)` mm |
| 17 | pinky_rota | 0..90°，下发钳到50° | `+q` | `-q` | R06 | 左 `+deg(q)`；右 `-deg(q)` |
| 18 | pinky_swing | 0..0.696rad | `clamp(q,0.5°,39.5°)` | `+q` | R03 | 左手截断、右手直接rad→deg |
| 19 | pinky_MCP | 0..90° | `+q` | `+q` | L03 | 左右均 `-CF3(q)` mm |
| 20 | pinky_PIP | 0..90° | `+q` | `+q` | L04 | 左右均 `-CF2(q)` mm |
| 21 | pinky_DIP | 0..90° | `+q` | `+q` | L05 | 左右均 `-CF1(q)` mm |

## 4. 最简符号分组

| 关节组 | 左手 UI→HW | 右手 UI→HW | USB 打包后的额外处理 |
|---|---:|---:|---|
| thumb_rota | `+1` 后钳位 | `+1` 后钳位 | 无系数；直接钳到[0.5°,89.5°]，再rad→deg |
| thumb_swing | `q` | `q` | H7 中位 1rad；rad→deg；左手方向已经实机校正为与模型显示一致 |
| 四指 swing | `+1`，H7边界截断到[0.5°,39.5°] | `+1` | rad→deg后直接打包 |
| pinky_rota | `+1` | `-1` | rad→deg，同号 |
| 所有 MCP/PIP/DIP | `+1` | `+1` | 连杆逆解后 `stroke_sign=-1`，最终负 mm |

## 5. 右手 MediaPipe thumb_swing 的镜像

右手 MediaPipe 输入会镜像到左手 retarget 模型。左右 URDF 在同一关节值下互为镜像，输出只转换关节键名前缀：

```text
左手求解器内部值 q_solver
    ↓ retarget_runtime：只换 left_hand_ / right_hand_ 前缀
右手 UI/Ruckig 值 q_ui = q_solver
    ↓ convert_ui_to_hardware(right)：同值
H7 角度 q_hw = q_ui = q_solver
```

左右手的 `thumb_swing` 均为 `q_ui=q_solver=q_hw`，采用新 URDF 的 `[0,2]` 坐标。
warm start 也只转换键名前缀。2026-09-18 移除了错误的二次数值翻转，详见
[拇指姿态问题档案](issues/20260918_thumb_retarget.md)。Form2 中左右手都同值下发。

其他右手关节在 retarget runtime 中只更换 `left_hand_`/`right_hand_` 键名前缀，不额外改数值。

## 6. MediaPipe / retarget 原始范围与符号

MediaPipe 本身输出的是 21 个空间点，不直接输出机器人关节角。retarget 将手掌和各指骨方向残差最小化，
求解器的关节范围由左手 URDF 决定：

| 关节组 | 求解器原始范围 | 符号说明 |
|---|---:|---|
| thumb_rota | 0..1.57rad（约89.95°） | External/求解器与 UI 同范围；H7边界直接钳到[0.5°,89.5°] |
| thumb_swing | 0..2rad（0..114.59°） | retarget/UI/H7统一[0,2]rad；中位1 |
| 四指 swing | 0..0.696rad（约39.88°） | UI非负；H7均为正值，左手截断到[0.5°,39.5°] |
| pinky_rota | 0..90° | 非负；协议下发前钳到 50° |
| 所有 MCP/PIP/DIP | 0..90° | 非负；负 DIP landmark 异常在进入 retarget 前修复，不把机器人 DIP 设成负值 |

右手点云会先沿 x 镜像到左手求解器。求解结果只改左右手键名前缀，不改变关节值。

## 7. 正负角、URDF 轴与顺/逆时针

URDF 中 `q>0` 一律遵循关节局部轴的右手定则。若观察者位于局部轴**正端并朝原点看**，
正角看起来是逆时针；从原点沿正轴方向看则是顺时针。因此，脱离观察方向说“正角就是顺时针”并不成立。

| 关节组 | 左手 URDF axis | 右手 URDF axis | 同一个正 q 的模型效果 |
|---|---|---|---|
| thumb_rota | `+Y` | `+Y` | 两模型按各自局部 +Y 右手定则 |
| thumb_swing | `+Z` | `-Z` | 左右镜像；origin yaw 分别为 -1.29927 / +1.29927 |
| thumb_MCP/PIP | `-X` | `+X` | 左右镜像屈曲 |
| 四指 swing | `+X` | `-X` | 左右镜像摆动 |
| 四指 MCP/PIP/DIP | `+Y` | `+Y` | 正值屈曲 |
| pinky_rota | `-Z` | `+Z` | 左右镜像 |

H7 协议注释把旋转命令正值定义为“顺时针”、负值定义为“逆时针”。按这一固件数值定义，
四指 swing 当前左右手均发送正值，左手额外截断到[0.5°,39.5°]。仓库不包含H7固件和电机安装基准，
所以电机轴在用户观察视角下的实际顺/逆时针仍需上电低幅测试确认。

## 8. UI 与显示

- Form2 滑块、主界面角度、日志中的 `retarget_joints` / Ruckig 目标均处于 UI/URDF 数值空间。
- 四指 swing 的 UI 范围为 0..0.696rad；0 是init/范围下限，0.348rad只是量程中点。
- 左右手显示使用相同数值，只替换 `left_hand_*` / `right_hand_*` 键名前缀；右手 URDF 的镜像 axis 负责反向显示。
- UI 显示不经过 Hardware 符号转换；只有真实下发和硬件遥测反解经过 `convert_ui_to_hardware` / `convert_hardware_to_ui`。
- `thumb_rota` 不再使用下发系数；动作文件按内部 UI `[0,1.57rad]` 保存和读取。
- `thumb_swing` 的既有动作文件继续使用历史 `[-1,1]rad`；加载时左手 `ui=1-file`、右手 `ui=1+file`，保存时做逆变换。旧文件无需迁移，物理姿态和 H7 指令保持不变。
- DownlinkGuard 对 `thumb_rota` 使用普通 rotation 速度阈值；Ruckig约束本身不变。

## 9. 不改变固定符号的环节

- landmark 修复：只修改输入空间点，不直接翻转输出关节符号。
- 物理碰撞：可能改变目标角大小，但没有固定的左右手符号映射。
- Ruckig：保持目标角坐标空间和符号，只约束速度、加速度和 jerk。
- DownlinkGuard：工作在 Hardware 空间，限制异常跳变，不执行固定取反。
- USB 打包：旋转关节只做 rad→deg；直线关节只做连杆逆解和 `stroke_sign=-1`。
- 右手 URDF 显示：数值不变，只换右手键名并依靠镜像 URDF 轴表达镜像；这是显示旁路，不参与 H7 下发。

## 10. 当前代码依据

- `src/protocol/hand_angle_protocol.py`：左右手 UI↔Hardware 权威符号转换。
- `src/kinematics/finger_linkage.py`：14路直线槽顺序、连杆逆解和 `stroke_sign=-1`。
- `src/usb/offline.py`：14×mm + 7×deg 槽位打包；USB 层不再变号。
- `src/utils/retarget_runtime.py`：右手输入点云镜像；输出和 warm start 只转换关节键名前缀。
- `src/app/widget.py`、`src/ui/form2_window.py`、`src/ui/display_window.py`：各实际发送入口均先调用 `convert_ui_to_hardware()`。
