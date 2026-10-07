# A3.2 串口与遥测诊断

## 本次修改

原生封装继续在进程内调用 A3 C++ SDK 1.1.0，C ABI 保持版本 2，未修改导出函数签名，也未调用 SDK 的 exe。`a32_state()` 的 JSON 增加连接与发送诊断。遥测缺失仍不阻止普通目标下发。

诊断分开记录串口连接、电机遥测和控制停止：页面显示连接，不代表当前已使能或正在发送；停止轨迹也不等于串口已经断开。

## 遥测协议核对

SDK 使用 AA55 帧头、长度、设备码、功能码、载荷及 CRC16-Modbus。手部遥测为设备码 3、功能码 `0x02`。下表的偏移从载荷首字节开始计算，数值为小端序：

| 字段 | 偏移 / 长度 | 含义 |
|---|---|---|
| sequence | 0 / 2 字节 | 遥测序号 |
| MIT 在线位 | 2 / 1 字节 | bit 0 为 MIT 0，bit 1 为 MIT 1 |
| tick_ms | 4 / 4 字节 | 设备时钟 |
| self 在线掩码 | 8 / 4 字节 | bit i 对应 self i |
| self i | `12 + 24*i` / 24 字节 | 共 18 槽 |
| MIT i | `444 + 20*i` / 20 字节 | 共 2 槽 |

self 部分最少 444 字节；含两路 MIT 的完整载荷为 484 字节。self 槽包含 `homing_or_theta`、实际位置、测量电流、参考电流、参考位置、重力补偿电流和温度。MIT 位置由 SDK 按 M1=50、M2=50 换算为输出轴度数，NativeHand 和界面不再额外缩放。

这些长度可在 SDK 的 `include/a3/detail/Protocol.h` 核对。本次还只读检查了随包静态库的 `parseHandTelem()`：self 的 `online` 确实来自载荷掩码，表达式等价于 `(mask >> i) & 1`。封装直接传递 SDK 的结果，没有用“位置为零”推断离线。

### 截图中 self 8–13 离线

截图中其余 self 在线，对应 self 在线掩码 `0x3C0FF`，缺失掩码 `0x03F00`。

| 通道 | SDK 总线 / 节点 | 电机 |
|---|---|---|
| self 8–10 | CAN2 / 1–3 | 中指 DIP、PIP、MCP 直线电机 |
| self 11–13 | CAN2 / 4–6 | 食指 DIP、PIP、MCP 直线电机 |
| self 14–15 | CAN2 / 7–8 | 中指、食指侧摆旋转电机 |

self 14/15 同属 CAN2，截图中在线，因此不能仅凭截图判断整条 CAN2 已断开。在线位由设备上报；需要结合连续记录中的序号、设备时钟、掩码变化及实际固件版本，才能继续区分电机供电、分支接线、节点配置、CAN 遥测接收或固件路由问题。未收到的测量值不能当作真实的零位置。

### 找零值 254

`254`（`0xFE`）是 SDK 的 `HomingState::Theta`，表示该字段承载电角度，不是错误码 254。其余状态为：0 等待、1 寻零、2 完成、3 错误、4 释放、255 开环旋转。SDK 的 `homingIsDone()` 将在线通道的 Done 和 Theta 视为可接受状态；这不等于当前平台要求全部通道完成找零才能下发。

## 新增诊断字段

顶层 `last_error_code` 保留最近错误的 SDK `Err` 数值；`-1` 表示封装层异常或无法取得 SDK 错误码。原有 `last_error` 保留可读原因。同一次调用触发 SDK 断线回调时，封装优先保留回调原因，避免随后读取空白或笼统的 `lastError()` 将它覆盖。

`diagnostics` 对象包括：

| 字段 | 含义 |
|---|---|
| `last_failed_operation` | 最近失败的封装操作，例如 `send_once` |
| `disconnect_count` | SDK 意外断线回调次数；主动断开不计入 |
| `last_disconnect_error_code` | 最近断线的 SDK `Err` |
| `last_disconnect_reason` | 最近断线回调原始原因；后续操作失败不清除它 |
| `last_disconnect_at_unix_ms` | 最近断线 UTC Unix 毫秒时间，未发生时为 0 |
| `last_disconnect_age_ms` | 距离最近断线的单调时钟毫秒数，未发生时为 -1 |
| `send_attempts` | `a32_send_once()` 调用次数 |
| `send_successes` / `send_failures` | 上述调用的成功 / 失败次数 |
| `last_send_at_unix_ms` | 最近一次发送调用开始的 UTC Unix 毫秒时间 |
| `last_send_duration_ms` | 最近一次同步 SDK 发送调用耗时 |
| `max_send_duration_ms` | 当前原生对象的最大发送调用耗时 |

计数和断线历史在同一个原生对象重新连接后保留，销毁对象后重置；连接时会清除当前 `last_error`。发送次数只统计 `sendOnce()`，不包含使能、找零、停止等命令；未连接时的发送尝试也计入失败。发送成功表示 SDK 接受并完成该次写入，不证明电机已执行或回传确认。

常见 SDK 错误码：1 未连接、2 链路丢失、3 等待超时、4 找零失败、5 无效槽位、6 无效参数、7 I/O 失败、8 未找到端口、9 固件查询超时、10 固件不兼容。

## 如何区分约十秒后停止的原因

- `connected` 仍为真、`disconnect_count` 未增加，但控制消息显示停止：检查平台的停止原因、页面控制权和控制循环耗时。
- `disconnect_count` 增加：查看断线时间、错误码和原始原因，结合该时刻的 `send_failures` 与发送耗时定位。
- `connected` 为真、遥测序号不再变化：表示遥测没有更新，不能据此认定串口句柄已断开。
- self 掩码持续缺失相同节点，而序号正常增长：表示设备仍在发送手部遥测，缺失的是对应节点的在线位。

当前 SDK 静态库的 `UsbCdc` 使用非阻塞读取、1000 ms 写超时；一次 `WriteFile` 失败或未写满均会使 SDK 发送失败。SDK 发布记录说明发送失败会被当作链路丢失。本次没有在其接收循环中发现“十秒无遥测自动断开”的逻辑，也没有现场持续数据证明此次故障一定发生在 USB、固件或接线上。

SDK 没有通过公共接口暴露 Windows `GetLastError()` 或原始串口收包。本次字段记录的是 SDK 已提供的原因与调用耗时，不把推测补成原始系统错误。

## 构建与离线验证

2026-09-28：原生 C++ 修改已通过 MSVC Release 构建。现有服务占用了 `src/a32_driver/bin/a32_native.dll`，常规链接报告 LNK1104；未停止服务，改为输出：

`src/a32_driver/bin/staged/a32_native.dll`

随后将旧 DLL 重命名保留为 `bin/a32_native.before-diagnostics`，把新版本复制到默认 `bin/a32_native.dll`，并更新 `source_snapshot.json` 哈希。已运行的进程继续使用原来映射的旧版本，新进程默认加载新版本；无需设置环境变量。这个替换过程没有停止或操作硬件。

staged DLL SHA256：`BEB90CAFFEE3D7DDEF7072094222846F06B6B1DF5F48EDB067DD61AAAD2C872D`。

已在不建立任何设备连接的情况下，通过项目 `NativeHand` 加载 staged DLL，检查 ABI 2、SDK 1.1.0、初始诊断字段、三次未连接发送的失败计数、耗时统计、错误原因保留及主动断开后的计数保留。全部通过。未做实物串口稳定性或运动验证。
