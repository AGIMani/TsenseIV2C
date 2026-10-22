# A3.2 接入验证记录

日期：2026-09-28。环境：Windows x64，`E:/conda/envs/hand_dex/python.exe`，已安装 Chrome。

## 关节范围与 swing 坐标更新

两款 A3.2 的左右手现使用 PIP/DIP `[0.005, 1.565]`、MCP `[0.005, 1.3]`、
四指 swing 模型范围 `[0, 0.7]`、默认 `0` rad，裸手轴向已与带 ID 对齐。
原始 SDK 裸手角度加 `0.350`，原始 ID 角度先取反再加 `0.350`；两款当前模型角
下发时均限幅到 `[0.005, 0.695]`。URDF 姿态、MJCF、
SDK 换算/反馈及旧动作迁移同步。勾选实时控制和动作同步时只使能，等待姿态操作后才下发。具体说明见
[关节范围与坐标更新](a32_joint_ranges.md)。

| 检查 | 结果 |
| --- | --- |
| A3.2 范围、驱动、模型、重定向、碰撞、记录及 motion_play 回归 | 裸手轴向对齐后 492 项通过 |
| 裸手与带 ID 四指 swing 方向 | 左右手公共掌坐标系中方向余弦均大于 0.9999；摆动范围、中间姿态和源机构映射通过 |
| Three.js 与 Python FK 对照 | 两款模型的左右手共 4 组通过 |
| Chrome motion_play | 五型号切换、四个 A3.2 手别、新范围、20 滑块、保存/导出/离线播放通过 |
| Chrome 模拟设备控制 | 首次勾选/重新勾选后零姿态下发、实际编辑后发送、页面隐藏、心跳过期恢复、日志下载/查看器通过 |
| 重新导入源模型比对 | 通过，已应用坐标变换的模型不会重复变换或误判为不同源 |

浏览器截图：`.work/a32-bare-id-axis/motion-play-withid.png`。独立验证服务已关闭，
未连接实体手，未验证真实机械行程。原运行服务需重启并刷新页面后加载新范围。

## 时间记录、统一目录与查看器同步

两款 A3.2 在手势跟踪和 motion_play 中共用 `src/a32_driver/recording.py`，默认写入
`E:\TsenseI_hand_logs\A32`。每条记录包含单调相对秒数、采集时的主机 Unix 毫秒和
带时区的 ISO 时间；设备 tick 与浏览器拍摄时间单独保存。修改查看器父目录后，
同一会话下一次连接开始的录制也会使用新位置。

| 检查 | 结果 |
| --- | --- |
| 通用记录器、目录、A3.2 记录/驱动/跟踪及 motion_play 全模块 | 318 项通过 |
| A3.2 浏览器帧处理、就绪条件、重定向记录 | 16 项通过 |
| 查看器专项 | A30/V2 回归、两款 A3.2 的时间/周期/CSV、离线/过期/缺测断线通过 |
| 隔离打包目录 | A3.2 机构转换、共享记录器导入及带时间的 JSONL 写入通过，无硬件连接 |
| Chrome 模拟设备流程 | 页面隐藏/心跳过期保留连接、手动恢复、下载含时间日志、目录刷新与打开通过 |

补充验证覆盖系统时钟跳变、写盘延迟不改变采样时间、缓存跨过 500 ms 时及时记录
过期状态、SDK 原始 `target` 在离线 CSV 中保留、失败写入不标记成功。
所有运动测试使用模拟设备，测试日志写入隔离目录，未调用 SDK exe。

现场只读采样转换到
`E:\TsenseI_hand_logs\A32\hand_control_20260928_163819_652621_sdk_capture.jsonl`，
保留原始时间 `2026-09-28T16:08:55.314084+08:00` 至
`2026-09-28T16:09:40.115362+08:00`，共 225 条状态、覆盖 44.800311 秒。
当时设备均未连接，因此没有电机测量值或下发记录。最后只读检查的 8767 服务也
显示未连接；没有为采样而连接或使能实体手。

复现本次软件验证：

```powershell
$env:NO_PROXY='127.0.0.1,localhost,::1'
& E:/conda/envs/hand_dex/python.exe -m pytest tests/test_control_session_recorder.py tests/test_control_log_catalog.py tests/test_a32_tracking_recording.py tests/test_a32_tracking_control.py tests/test_a32_driver.py tests/test_web_v3_2.py online_control/online_motion_play -q --basetemp .pytest-runtime/a32-recording-final
& E:/conda/envs/hand_dex/python.exe -m pytest tests/test_a32_mapper.py::test_minimal_bundle_contains_independent_a32_runtime -q --basetemp .pytest-runtime/a32-recording-package-check
node tests/test_control_log_viewer.cjs
node --experimental-default-type=module --test online_control/online_motion_track/tests/frame-processing.test.mjs online_control/online_motion_track/tests/retarget-recording.test.mjs online_control/online_motion_track/tests/a32-readiness.test.mjs
```

浏览器验证使用 `test_a32_lifecycle_browser.mjs` 和独立 `FakeDevice` 服务；该验证
服务已关闭。应用这些更新仍需手动重启两套 Python 服务并刷新页面。

## 连接、控制恢复和记录修正

- motion_play 切换标签页/页面退出时改为停发并保留 SDK 连接；5 秒控制权心跳超时也保留连接、释放控制权，不反复发送停止命令。明确断开/释放控制仍关闭设备。
- 控制周期改由独立线程驱动，真实超过 100 ms 仍停发。暂停后实时控制复选框保持可用，手动勾选会重新使能；不自动恢复运动。
- SDK 诊断新增断线错误码、时间、次数和发送耗时；页面显示 self 在线掩码、CAN 节点、找零状态含义。原生 DLL 已在默认路径更新，ABI 仍为 2。
- 接入异步 `hand-control-session/v2` 记录和下载，查看器支持 A3.2 的 14+6 通道及 SDK 状态/错误。

验证：motion_play 全模块 + A3.2 驱动/跟踪共 285 项回归中，首次 284 项通过；一项 V2 弹窗超时回归发现兼容问题，修正后包含该项的 65 项针对性回归全部通过。另有底层记录器和查看器专项检查通过。Chrome 隔离模拟设备测试验证页面隐藏、心跳过期、保留连接、重新获取控制并恢复、日志下载与查看器路由，未连接真实设备。

实际服务另采集 45 秒、225 条只读快照：`logs/a32_live_diagnostics/hand_control_20260928_161509_171588_sdk_capture.jsonl`。全程设备已断开，因此没有有效电机遥测或下发数据，不能用于证明实体串口稳定性。

当前运行的 8767 服务仍使用旧 Python 代码；自动审批拒绝重启操作（`blocked by policy`）。需手动重启 motion_play 并刷新页面后生效；手势跟踪原有服务没有被停止。

## 遥测门槛修正（按 A3.0 控制方式）

2026-09-29 补充：下面的“完全无遥测也能开启”是 09-28 的历史行为。当前手势页面进一步对齐 A3.0，首次连接需要一包有效手部遥测，后续允许缺失或过期反馈；同时统一丢手、按指碰撞保持和限位恢复。驱动层及 motion_play 保留原行为。详见 [当前手势下发规则](a32_gesture_control_policy.md)。

2026-09-28：移除普通使能和姿态下发对完整遥测及找零 DONE/THETA 的强制依赖。手势与 motion_play 均已调整，原生 DLL 无需重新编译，也未调用 SDK exe。

| 本次检查 | 结果 |
| --- | --- |
| A3.2 核心与 A30/V2 回归 | 206 项通过，376 个子测试通过 |
| `online_motion_play` 全模块 | 221 项通过 |
| A3.2 按钮就绪条件、帧调度、记录、Three.js FK | 17 项通过 |

新增覆盖：两款 A3.2 完全无遥测时使能/下发、遥测过期或部分离线时继续发送、无需先找零、逐关节反馈与软件姿态混合起步、停止后从上次成功命令恢复、无复位遥测时重新使能、手势页面按钮可用、motion_play 实时控制与同步播放。断开连接、未使能、SDK 写入失败、非法目标与主动找零保护仍有测试。

上述 Python 命令仍可复现；JavaScript 命令增加 `online_control/online_motion_track/tests/a32-readiness.test.mjs`。本次没有连接实物。应用修改需重启手势和 motion_play 的 Python 服务并刷新页面。

## 初次接入结果

| 检查 | 结果 |
| --- | --- |
| A3.2 驱动、模型、末端碰撞、机构转换、跟踪，以及 A30/V2 相关回归 | 194 项通过，376 个子测试通过 |
| `online_motion_play` 全模块回归 | 220 项通过 |
| 帧调度、重定向记录、Three.js FK JavaScript 回归 | 16 项通过 |
| A3.2 手势页面 Chrome 验证 | 两型号、四个手别、真实网格、SDK 面板、MediaPipe 相机推理/帧请求/停止、会话、移动布局通过 |
| motion_play Chrome 验证 | 五型号、四个 A3.2 手别、20 关节、保存/导出/播放、旧型号切回；无页面异常或资产请求失败 |
| 两款 A3.2 Mink 初始化和初始姿态投影 | 通过 |
| 原生 DLL | SDK 1.1.0，C ABI v2，实际编译、离线加载/状态读取/析构通过 |
| 打包清单 | 两平台均包含 DLL、硬件配置、机构资源和 SDK 许可；不存在 SDK exe |
| 打包后的独立启动 | 手势 `server.py --check`、motion_play `server.py --check` 均通过；隔离 `python -I` 从打包目录加载 C ABI v2 DLL 通过 |

测试期间未连接 USB/ETH 实物、未使能、未发机械运动。DLL 离线加载只创建 SDK 对象和读取未连接状态；设备行为由明确的模拟对象验证。

## 复现

核心回归使用以下文件：

```powershell
$env:NO_PROXY='127.0.0.1,localhost'
$env:PYTHONIOENCODING='utf-8'
& E:/conda/envs/hand_dex/python.exe -m pytest tests/test_a32_driver.py tests/test_a32_models_integration.py tests/test_a32_distal_collision.py tests/test_a32_mapper.py tests/test_a32_fast.py tests/test_a32_tracking_control.py tests/test_hand_profile_v3_2.py tests/test_web_v3_2.py tests/test_a30_motion_play_sync.py tests/test_a30_mapper.py tests/test_a30_control.py tests/test_web_v2.py -q --basetemp .pytest-runtime/a32-validation-core

& E:/conda/envs/hand_dex/python.exe -m pytest online_control/online_motion_play -q --basetemp .pytest-runtime/a32-validation-play

node --experimental-default-type=module --test online_control/online_motion_track/tests/retarget-recording.test.mjs online_control/online_motion_track/tests/frame-processing.test.mjs online_control/online_motion_track/tests/kinematics.test.mjs
```

本机全局 HTTP 代理会把测试的 localhost 请求代理成 502，测试命令显式绕过本地地址。系统默认 pytest 临时目录存在 ACL 问题，所以使用项目内独立 `--basetemp`。

浏览器脚本分别为：

- `online_control/online_motion_track/tests/a32-sdk-browser.mjs`，通过 `WEB_URL` 指定测试服务地址，默认端口 8872。
- `online_control/online_motion_play/test_a32_browser.mjs`，通过 `MOTION_URL` 指定测试服务地址；应对接独立临时数据目录，因为测试会保存验证动作。

浏览器截图：`logs/a32_integration_qa/`；motion_play 截图：`.work/a32-browser/motion-play-withid.png`。末端碰撞 QA 及生成依据位于 `assets/hands/TsenseI_hand_v3_2_withid/qa/`。

## 已知环境与验收边界

原工作区 `online_motion_play/runtime` 中已有 A30 动作文件的型号元数据与当前型号不匹配，会使默认目录的 `server.py --check` 报错。本次使用独立数据目录完成五型号检查，没有改写这些用户动作文件。新的 A3.2 动作文件有独立命名和型号校验。

带 ID 21 点仍需人工标定；右手现已复用 A3.2 槽位表并开放实物连接。真实固件握手、在线掩码、找零电流、方向/零位和连续运动仍需现场验收。SDK 断线后的停机能力以供应商协议为限。
