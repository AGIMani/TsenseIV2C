# A3.2 SDK 1.1.15 更新记录

2026-10-12 导入用户提供的 `a3.2_cpp_sdk-main.zip`。压缩包 SHA-256 和逐文件校验记录在 `vendor/a3-cpp-sdk/sdk-source.json`，ZIP 注释提交为 `fd676e5a3296e5e3ada2734d290acc443206e000`。

本版把线性关节换算更新为四类连杆的均匀查表：四指 MCP、食/中/无名指 PIP 与拇指 MCP、四指 DIP 与拇指 PIP，以及小指 PIP。Python 离线换算与上游 `Stroke.cpp` 保持一致，反馈反算也使用 SDK 的查表和端点容差。

A3.2/withid 的右手硬件连接已开放。右手和左手共用 18 个 SELF、2 个 MIT 的槽位表，模型仍按各自手别使用 URDF 关节名；固件回传的 `firmware.hand_side` 仅作为连接状态诊断，不自动改变会话手别。

验证不连接实物：上游 SDK C++ 测试、Linux POSIX/PTY 集成测试、Python SDK 换算测试和左右手设备连接契约测试均应通过。实际 USB/TCP 运动仍需现场验收。
