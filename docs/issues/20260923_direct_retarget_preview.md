# 关闭 Ruckig 限速：直接映射与实时下发

在手势网页的采集设置区勾选“关闭 Ruckig 限速”。A3.0 和 V2 支持在识别过程中切换；V3.2 本身不经过 Ruckig。更新后直接模式也允许手动开启实时下发。

直接模式的链路：MediaPipe → 关键点处理 → retarget → 目标碰撞处理 → 低延迟自适应滤波 → 采样路径碰撞检查 → 角度/行程转换 → 浏览器本机 USB 串口。不再经过 Ruckig 速度、加速度、jerk 限制，也不再使用浏览器基于 Ruckig 速度上限的跳变检查与等待。关节范围、有限值、协议校验、流时效和通信保护仍保留；关闭碰撞检查仍不能开启实时下发。

未下发时，A3.0 模型显示服务器预览流的滤波与碰撞检查后样本；V2 使用状态采样。A3.0 正在下发时，模型显示最近成功写入 USB 的检查后样本，避免模型先于实际写入更新。

切换任一模式时先停发、取消排队旧任务并使旧流 epoch 失效。服务器确认模式后才允许手动开启实时下发；失败时保持禁用。恢复限速也不自动下发。关节电源和找零仍是独立操作，模式开关不发送电源或找零指令。

接口：`/api/a30/control` 或 `/api/v2/control`，`action=set_direct_mode`、`enabled=true/false`。创建会话可传布尔 `direct_mode`；状态返回 `direct_mode` 与 `ruckig_enabled`。浏览器忽略切换前的旧模式识别响应。

本地记录使用 `control_mode` 事件和 `direct_mode` 标记。`control_target` 为滤波输入；`direct_filter` 为碰撞检查后滤波输出，A3.0 按实际接收的预览/USB 样本记录，`sent` 区分是否已写入串口；`direct_filter_detail` 按状态轮询记录滤波参数、输入、碰撞前候选和检查后值（A3.0 约 10 Hz）。实际串口写入继续记录 `h7`、`conversion` 和 `downlink`，直接模式不再填充 `joints_ruckig_ui`。记录保存在本机。

滤波实现与本次日志的验证结果见 [直接链路低延迟滤波](20260923_direct_filter.md)。

验证：46 项 Python 测试、36 项 JavaScript 测试通过。Chrome 使用模拟相机和串口验证直接模式可手动下发、模式切换停发及恢复限速需手动重启；另验证较大角度目标不被速度检查拦截，同时保留碰撞暂停、CRC 和关节范围检查。截图：`logs/direct_downlink_browser/direct-downlink.png`。未操作真实电机，未部署 node0。

本地生效：重启手势服务并刷新网页。项目根目录运行 `start_local_and_update_node0.cmd --local-only`。
