# A3.0 上行遥测与查看器同步（2026-09-23）

## 已确认的证据

- 用户指定记录：`C:/Users/ONKIALL/Downloads/hand_control_20260922_171011_020000.jsonl`。
- JSONL 完整可读：81,210 条，4,450 帧遥测；记录因达到 256 MiB 上限结束，结束原因有保存。
- 项目根目录的 `A3_USB_Downlink.pyw` 与用户再次提供的微信 `tools(1)/A3_USB_Downlink.pyw` 完全相同，SHA-256 均为 `6bd13249664fbf398a5ecfba11b77b177befea6b3722f1428bf3c29db1967589`。
- 同目录 `protocol(1).zip` 中 `A3_CANFD_上行协议.md` 同样规定 12 字节头 + 18×24 字节自研槽 + 40 字节外接；自研槽为 `<5f2h>`。
- 使用 `runpy` 加载用户工具、替换表格为内存对象，直接调用其 `A3UsbApp._apply_telem()`，未创建窗口或连接设备。首帧小指 MCP 显示 `pref=nan mm`、开环频率 `-226907406336 Hz`；无名指 MCP 显示 `iq=nan`、`temp=-116.93 ℃`。原工具显示结果保存在 `logs/a30_reference_display.json`。

这证明给定旧记录按这份工具/协议解码也有异常，不能把缺失值直接修补成真实测量值。用户反馈原工具当前能正常显示；目前没有该次正常运行的原始报文，尚不能确定异常发生在何处，也不能据此认定下位机或当前工具有故障。旧日志没有保存完整帧头和收到的 CRC，无法对历史传输重新做独立 CRC 验证。

## 本地修改

1. 将上行解析独立为 `online_control/shared/a30-telemetry.js`；两个网页协议入口共享该实现。已有控制转换和机械减速比 `[50,30]` 保留，额外保存外接电机端原始角度/速度，可与参考工具的固定 ÷50 显示区分。
2. 新记录标明协议版本、关节槽映射、位置/电流单位、符号方向与减速比。每个自研槽保存实际关节名称及槽位；异常字段以 `decode_issues` 明示，不补零。温度范围诊断仅为提示，不修改控制决策。
3. 日志增加收到的帧头、CRC、原始载荷长度和扩展字节，可重建并复核完整上行帧；不重复保存两份完整载荷。
4. 独立 HTML 查看器嵌入同一解码器，可通过 file:// 离线打开。原始 A3 载荷自动复核；支持没有 session 头的原工具记录和 `event/elapsed_ms` 诊断文件。A3.0 使用 20 个执行器映射、真实拇指关节名称，移除不存在的小指固定旋转选择。
5. 逐帧详情显示所选电机、原始报文、异常字段和板级遥测；CSV 增加通道、原始值和异常原因。无效外接数据不再显示为虚构的大负角度。

维护解码器后运行 `python tools/sync_log_viewer_decoder.py`。测试验证查看器嵌入内容与公共模块一致，防止两份解析再次分叉。

## 已有日志处理

```powershell
node --experimental-default-type=module tools/redecode_a30_log.mjs INPUT.jsonl NEW_OUTPUT.jsonl
```

脚本不覆盖原文件，不改变下发记录；通过公共解码器复核遥测，另存 JSONL 和 `.report.json`。本次输出为 `logs/hand_control_20260922_171011_020000_redecoded.jsonl`。该文件保留真实异常，不宣称恢复了未知格式的测量值。

## 捕获原工具显示正常时的原始数据

```powershell
python tools/record_a30_reference.py --reference "D:\Y90\微信file\xwechat_files\wxid_fvvck9nq4ku622_72ba\msg\file\2026-09\tools(1)\A3_USB_Downlink.pyw"
```

用 `--reference "工具完整路径"` 指定可运行的原工具，保留其同目录的 `H7_USB_OTA.pyw` 依赖（项目根目录旧副本缺少该依赖）。脚本不自动连接、不找零、不使能、不下发运动；由用户在原工具中操作。CRC 检查后完整上行帧同步保存到 `logs/hand_reference_时间.jsonl`，可直接交给数据查看器加载。两个程序不可同时占用同一 USB 串口。

## 验证与范围

- 48 项 JS 协议/控制测试通过，包含全部 18 个自研槽与 Python 工具的 struct 对照、444/484 字节、外接电机、分片/CRC、完整帧重建、V2 回归。
- 独立查看器测试通过；Chrome 实际加载 81,210 条重新解码记录，20 个执行器均可选择，小指 MCP 曲线和 CSV 导出正常执行，无页面异常。非有限值仍明确缺失。
- 未连接真机，未部署 node0。正常实时数据与旧日志异常的根因仍需用上述原工具同步记录做进一步对照。
