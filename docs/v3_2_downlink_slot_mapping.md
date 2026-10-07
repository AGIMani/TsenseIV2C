# V3.2 真机下发关节与槽位对应表（预填，待你确认）

更新日期：2026-09-20。关节与电机对应按你本次描述预填；槽位编号、类型和单位参考 `deprecated/A3_USB_Downlink.pyw`。外接电机 ID 已与所附图片核对。本文只是确认稿，未修改实际下发、转换算法或模型。

CAN1：小指、无名指；CAN2：中指、食指；CAN3：拇指。CAN2 按“中指对应小指那组、食指对应无名指那组”类推。左右手暂按相同接线编号分别列出，右手是否相同仍待你确认。

## 槽位和协议参考

| 项目 | 旧工具中的定义 / 待确认内容 |
| --- | --- |
| 自研槽位 | `self_pos[0..17]`，从 0 开始；CAN1 占 0–7，CAN2 占 8–15，CAN3 自研 M1/M2 占 16/17 |
| 外接槽位 | `mit[0]` = CAN3 外接 M1 / `0x505`；`mit[1]` = CAN3 外接 M2 / `0x509`；独立数组，不是 `self_pos[18/19]` |
| 自研位置数据 | 18 个小端 float32；旋转槽为 deg，直线槽为 mm |
| 外接数据 | 每台包含位置、速度、电流；旧工具界面为输出轴 deg / rpm / A，USB 使用电机端换算，减速比 1:50；实际是否沿用待确认 |
| 批量帧 | 旧工具：`AA 55` + 长度 u16 LE（100）+ DEV=3 + FUN=0x20 + 100B payload + CRC16 Modbus；整帧 108B |
| 实际固件协议版本是否与旧工具一致 | 待填写： |
| 左右手区分方式 / 接线是否相同 | 待填写： |

表中的 `M1..M8` 是各 CAN 总线内的电机编号，`self_pos[n]` / `mit[n]` 才是旧工具的载荷位置。它们不等于 A3.2 几何模型的 motor_index，也不使用 V2 槽位。

## 左手

| 总线 | 工具电机 / ID | 下发槽位 | 工具类型 / 单位 | 当前项目关节名称 | 备注 | 你的确认 / 修正 |
| --- | --- | --- | --- | --- | --- | --- |
| CAN1 | M1 | `self_pos[0]` | 旋转 / deg | `left_hand_pinky_DIP_joint` | ③ | |
| CAN1 | M2 | `self_pos[1]` | 旋转 / deg | `left_hand_pinky_swing_joint` | ②已改名：侧摆 | |
| CAN1 | M3 | `self_pos[2]` | 旋转 / deg | `left_hand_ring_DIP_joint` | ③ | |
| CAN1 | M4 | `self_pos[3]` | 旋转 / deg | `left_hand_ring_swing_joint` |  | |
| CAN1 | M5 | `self_pos[4]` | 直线 / mm | `left_hand_pinky_PIP_joint` |  | |
| CAN1 | M6 | `self_pos[5]` | 直线 / mm | `left_hand_pinky_MCP_joint` | ②已改名：屈伸 | |
| CAN1 | M7 | `self_pos[6]` | 直线 / mm | `left_hand_ring_PIP_joint` |  | |
| CAN1 | M8 | `self_pos[7]` | 直线 / mm | `left_hand_ring_MCP_joint` |  | |
| CAN2 | M1 | `self_pos[8]` | 旋转 / deg | `left_hand_mid_DIP_joint` | ③；按中指、食指顺序类推 | |
| CAN2 | M2 | `self_pos[9]` | 旋转 / deg | `left_hand_mid_swing_joint` | 按中指、食指顺序类推 | |
| CAN2 | M3 | `self_pos[10]` | 旋转 / deg | `left_hand_index_DIP_joint` | ③；按中指、食指顺序类推 | |
| CAN2 | M4 | `self_pos[11]` | 旋转 / deg | `left_hand_index_swing_joint` | 按中指、食指顺序类推 | |
| CAN2 | M5 | `self_pos[12]` | 直线 / mm | `left_hand_mid_PIP_joint` | 按中指、食指顺序类推 | |
| CAN2 | M6 | `self_pos[13]` | 直线 / mm | `left_hand_mid_MCP_joint` | 按中指、食指顺序类推 | |
| CAN2 | M7 | `self_pos[14]` | 直线 / mm | `left_hand_index_PIP_joint` | 按中指、食指顺序类推 | |
| CAN2 | M8 | `self_pos[15]` | 直线 / mm | `left_hand_index_MCP_joint` | 按中指、食指顺序类推 | |
| CAN3 | 自研 M1 | `self_pos[16]` | 旋转 / deg | `left_hand_thumb_PIP_joint` | ①已确认：拇指 PIP；③单位待适配 | |
| CAN3 | 自研 M2 | `self_pos[17]` | 直线 / mm | `left_hand_thumb_MCP_joint` |  | |
| CAN3 | 外接 M1 / 0x505 | `mit[0]` | 旋转 / deg（界面输出轴） | `left_hand_thumb_rotaback_joint2` | 与你提供的外接 M1 对应 | |
| CAN3 | 外接 M2 / 0x509 | `mit[1]` | 旋转 / deg（界面输出轴） | `left_hand_thumb_rota_block_joint` | 与你提供的外接 M2 对应 | |

## 右手（暂按相同接线预填）

| 总线 | 工具电机 / ID | 下发槽位 | 工具类型 / 单位 | 当前项目关节名称 | 备注 | 你的确认 / 修正 |
| --- | --- | --- | --- | --- | --- | --- |
| CAN1 | M1 | `self_pos[0]` | 旋转 / deg | `right_hand_pinky_DIP_joint` | ③ | |
| CAN1 | M2 | `self_pos[1]` | 旋转 / deg | `right_hand_pinky_swing_joint` | ②已改名：侧摆 | |
| CAN1 | M3 | `self_pos[2]` | 旋转 / deg | `right_hand_ring_DIP_joint` | ③ | |
| CAN1 | M4 | `self_pos[3]` | 旋转 / deg | `right_hand_ring_swing_joint` |  | |
| CAN1 | M5 | `self_pos[4]` | 直线 / mm | `right_hand_pinky_PIP_joint` |  | |
| CAN1 | M6 | `self_pos[5]` | 直线 / mm | `right_hand_pinky_MCP_joint` | ②已改名：屈伸 | |
| CAN1 | M7 | `self_pos[6]` | 直线 / mm | `right_hand_ring_PIP_joint` |  | |
| CAN1 | M8 | `self_pos[7]` | 直线 / mm | `right_hand_ring_MCP_joint` |  | |
| CAN2 | M1 | `self_pos[8]` | 旋转 / deg | `right_hand_mid_DIP_joint` | ③；按中指、食指顺序类推 | |
| CAN2 | M2 | `self_pos[9]` | 旋转 / deg | `right_hand_mid_swing_joint` | 按中指、食指顺序类推 | |
| CAN2 | M3 | `self_pos[10]` | 旋转 / deg | `right_hand_index_DIP_joint` | ③；按中指、食指顺序类推 | |
| CAN2 | M4 | `self_pos[11]` | 旋转 / deg | `right_hand_index_swing_joint` | 按中指、食指顺序类推 | |
| CAN2 | M5 | `self_pos[12]` | 直线 / mm | `right_hand_mid_PIP_joint` | 按中指、食指顺序类推 | |
| CAN2 | M6 | `self_pos[13]` | 直线 / mm | `right_hand_mid_MCP_joint` | 按中指、食指顺序类推 | |
| CAN2 | M7 | `self_pos[14]` | 直线 / mm | `right_hand_index_PIP_joint` | 按中指、食指顺序类推 | |
| CAN2 | M8 | `self_pos[15]` | 直线 / mm | `right_hand_index_MCP_joint` | 按中指、食指顺序类推 | |
| CAN3 | 自研 M1 | `self_pos[16]` | 旋转 / deg | `right_hand_thumb_PIP_joint` | ①已确认：拇指 PIP；③单位待适配 | |
| CAN3 | 自研 M2 | `self_pos[17]` | 直线 / mm | `right_hand_thumb_MCP_joint` |  | |
| CAN3 | 外接 M1 / 0x505 | `mit[0]` | 旋转 / deg（界面输出轴） | `right_hand_thumb_rotaback_joint2` | 与你提供的外接 M1 对应 | |
| CAN3 | 外接 M2 / 0x509 | `mit[1]` | 旋转 / deg（界面输出轴） | `right_hand_thumb_rota_block_joint` | 与你提供的外接 M2 对应 | |

## 已确认和已同步的名称

### ① 拇指 PIP

你已确认拇指末节就是 PIP。CAN3 自研 M1 → `thumb_PIP_joint`；CAN3 自研 M2 → `thumb_MCP_joint`。不再使用“拇指 DIP”的称呼。

### ② 小指 swing / MCP

已按你的要求在左右手 URDF 中直接互换两个关节名，并同步 MJCF、关节配置和 A3.2 转换器的名称映射：

| 当前关节后缀 | 物理功能 | 对应电机 / 槽位 |
| --- | --- | --- |
| `pinky_swing_joint` | 侧摆，限位 -0.35～0.35 rad，转换器输出 deg | CAN1 M2 / `self_pos[1]` |
| `pinky_MCP_joint` | 屈伸，限位 0～1.57 rad，转换器输出 mm | CAN1 M6 / `self_pos[5]` |

关节轴向、原点、限位、父子连接和网格未变。link/STL 仍保留原有名字，避免影响已有关键点标定和碰撞几何。以后旧记录若使用改名前的关节名称，小指这两个字段需按版本互换解释。

## ③ 为什么“指尖旋转电机”与当前转换器的输出不同

### 先区分三件事

1. **URDF 关节角**：描述手指实际弯了多少，例如食指 DIP 弯曲 30°，程序里记为约 0.523599 rad。
2. **机械传动量**：驱动这个关节需要什么输入，由机构决定。连杆加滑块可能需要一段直线行程；旋转传动可能需要一个电机角度。同样是让手指旋转，两种传动方式的控制量可以完全不同。
3. **下发协议的数字含义**：固件收到某个槽位的 float32 后，把它解释成 mm 还是 deg，由协议决定。数据里只有数值，通常没有跟着一个“mm/deg”单位标签。

所以，URDF 中关节类型为 revolute，并不意味着其驱动器必须接收角度。这里发现的差异来自机构配置与旧工具的协议定义，而不是从 URDF 的 revolute 类型推断的。

### 当前代码实际做了什么

`src/a32_kinematics/a32_angles_to_inputs.py` 从 `model/geometry_inferred.mat` 读取各电机的 `type` 和机构参数。该配置将四指 DIP 与拇指 PIP 归为 `linear`，利用闭环机构求解或 LUT 算出行程（原始单位 m）。项目 `A32StrokeMapper` 再乘 1000，放入 `linear_strokes_mm`。

但 `deprecated/A3_USB_Downlink.pyw` 将 CAN1/2 的 M1、M3 和 CAN3 的 M1 归为旋转槽，位置单位是 deg；这也与你描述的指尖旋转电机一致。

| 关节 | 旧工具槽位 | 当前转换器输出 | 旧工具要求 |
| --- | --- | --- | --- |
| 小指 DIP | CAN1 M1 / `self_pos[0]` | mm | deg |
| 无名指 DIP | CAN1 M3 / `self_pos[2]` | mm | deg |
| 中指 DIP | CAN2 M1 / `self_pos[8]` | mm | deg |
| 食指 DIP | CAN2 M3 / `self_pos[10]` | mm | deg |
| 拇指 PIP | CAN3 M1 / `self_pos[16]` | mm | deg |

当前转换器因此是 14 路行程 + 6 路旋转；旧工具描述的 20 路则是 9 路直线 + 11 路旋转（包含两台外接旋转电机）。两者相差的正是这五个指尖通道。

### 一个来自当前代码的数值例子

输入 `left_hand_index_DIP_joint = 30° = π/6 rad`，其余关节为零，当前 LUT 实际输出约 **4.256919 mm**。

如果仅按槽位复制这个值，`self_pos[10]` 会收到数字 `4.256919`，而固件按旧工具协议将它理解为 **4.256919°**，不会理解为行程，也不会知道原目标是手指弯曲 30°。

这里不能通过把字段名从 mm 改成 deg 来解决，因为长度和角度之间需要机械关系。例如：

- 如果这路协议位置就是关节输出角，且零位相同、方向相同、1:1，那么应下发 30°。
- 如果协议位置是电机端角，电机转角与关节角还可能有减速比、方向和零位差。一个简单线性关系可写成 `电机角 = 方向 × 传动比 × 关节角 + 零位偏置`；实际是否适用需由机构和固件定义确认。
- 如果这路旋转电机仍通过某种传动驱动滑块，就要根据该传动关系把机构需求转换为电机角，不能随意使用比例。

上面 30° 与 4.256919 mm 是当前转换器的实际输入/输出；“1:1”只是说明单位差异的假设，不是已确认的真机参数。

### 后续实现需要确定什么

这五路需要确认：协议中的角度表示电机端还是关节输出端、传动关系、正负方向、零位和允许角度范围。然后分别实现旋转目标映射；其余直线通道继续使用对应机构的角度转行程。

外接 `0x505/0x509` 的 1:50 换算是旧工具对那两台外接电机的专用处理，不能直接套到自研指尖电机。

**这与加速查表本身无关**：LUT 只是更快地求出已配置机构的同一结果；若配置输出的是行程，它计算得再快、误差再小，结果仍是 mm，不会自动成为旋转电机需要的 deg。本次仅同步小指关节名称，没有更改上述五路转换方式，也未启用真机下发。

后续确认：五路旋转角的物理含义 ______；方向 / 零位 / 传动关系 ______；CAN2 类推顺序 ______；左右手接线是否相同 ______。
