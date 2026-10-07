# Wuji 手套：在 WSL2 中直接调用 SDK

WSL 版本默认通过本机 `wuji-sdk==2026.9.22` 订阅手套 `hand_skeleton`。网页服务、SDK、骨架验证和 TsenseI retarget 都在同一发行版中运行。

```text
手套 192.168.1.100:50001
    ↕ 以太网 UDP，经 WSL 网络
WSL 手势映射工作台：wuji_sdk → 21 点骨架 → 验证 → retarget
    ↓ 同源 HTTP
浏览器骨架和模型预览
```

Windows 只承担 WSL 宿主和以太网连接。正常启动不运行 Windows UDP 转发器、不访问 node0，也不启动独立 HTTP 手套桥。SDK 采集不会连接、使能或驱动机械手；实物输出仍由页面原有控制流程决定。

## 启动

WSL 的跟踪服务环境使用 `online_control/online_motion_track/requirements-wsl.txt`，包含固定 SDK 版本。已配置的启动器：

```bash
~/.local/bin/hand-wuji
```

也可以直接运行：

```bash
~/.venvs/hand-track/bin/python -m online_control.online_motion_track.main \
  --no-open --wuji-source sdk \
  --wuji-address 192.168.1.100:50001 \
  --wuji-serial WG1JA06260713017
```

以上地址、端口和序列号来自此前现场左手套记录，需与当前设备核对。地址是手套本身的地址，不能填旧 Windows 转发端口。主机以太网必须处于设备可达的子网；本机 WSL 网络检查说明见迁移目录中的 Wuji 网络文档。2026-09-30 的软件验收未连接实体手套，不能据此认定设备握手已通过。

不指定地址或序列号时，SDK 会在**开始手套输入后**扫描本机可见的 Wuji Glove；相同手别存在多个设备会报错。仅指定 `--wuji-serial` 时按序列号连接；同时指定地址和序列号时，用地址连接并核对返回的设备身份。手别以 SDK `hand_side()` 返回值为准。

服务启动、`--check`、`/api/health`、创建普通相机/WebXR 会话都不导入或扫描 Wuji SDK。手套输入会话首次请求最新骨架时才启动一个后台采集线程；所有手套会话共享它。`/api/health` 的 `wuji_transport: "sdk"` 表示配置选择，不能证明已收到真实骨架。

SDK 连接设置 `enable_bridge=False`，不共享设备给其他 SDK 实例；使用独立句柄名且只断开本进程创建的句柄。连接超时设置为 1000 ms、重试次数 1。SDK 的连接和清理语义见[官方说明](https://docs.wuji.tech/docs/en/wuji-sdk/latest/device-connection/)。默认设置 `WUJI_TELEMETRY_DISABLE=1`，关闭 SDK 匿名使用遥测。

## 明确选择旧 HTTP 桥

SDK 是默认来源。遗留 `WUJI_BRIDGE_URL` 环境变量不会自行切换到旧远程服务。需要调试旧桥时，明确选择：

```bash
~/.local/bin/hand-track --wuji-source bridge --wuji-url http://127.0.0.1:8787
```

显式使用旧 `--wuji-url URL` 仍会自动选择 bridge，保持既有调用兼容；同时指定 `--wuji-source sdk` 和 `--wuji-url` 会报错。桥接模式必须提供 URL，没有默认 node0 地址。也可显式设置 `WUJI_SOURCE=bridge` 与 `WUJI_BRIDGE_URL`。SDK 模式支持 `WUJI_ADDRESS`、`WUJI_SERIAL`；命令行参数优先。

`tools/wuji_glove_bridge.py` 作为可选适配器继续提供 `/latest` 和 `/health`，从项目根目录运行；其采集逻辑复用 `wuji_sdk_source.py`，不再把单个桥脚本当作完整部署包。旧 Windows/node0 的历史部署记录见 `docs/wuji_startup_network_20260929.md`，不适用于本次 WSL 默认启动链路。

## 数据与停止行为

- 格式保持 `tsensei.wuji_glove.skeleton.v1`：米制 21×3 点、21 个置信度、流 ID、序号、SDK 时间戳、手别及 `l_wrist/r_wrist` 坐标系。腕部归零后进入现有 retarget，不伪造图像坐标。
- 本地单调时钟计算回调后的龄期。重复或倒退序号不能刷新接收时间；350 ms 及以上即失效，消费快照前还会计算等待时间。此值不含传感器到 SDK 的时间，不能当成已测量的端到端延迟。
- SDK 断连、无效骨架、SDK 导入或连接异常会反映在 `source_metadata.event/error/transport`。无数据时不生成模拟帧。SDK 连接断开会清空观察值，断流仍遵循各型号原有的手势丢失处理。
- 断开与重连使用不同流 ID，旧订阅中迟到的回调不能写入新流。浏览器提交的骨架不被当作真实手套观察；会话手别绑定、版本去重和一次消费约束保持不变。
- 关闭最后一个手套输入会话会取消采集和后续重连，并释放订阅及自有连接。正在执行的原生 scan/connect 无法被 Python 强制中断；返回后立即检查取消并清理，不继续订阅或重连，也不允许新线程与旧所有者重叠。
- 清理失败会保留待清理资源，显示 `disconnect_error` 并停止重连；再次关闭可重试清理。错误不会被伪装成正常停止，也不会调用 `disconnect_all()` 影响其他 SDK 使用者。
- `wuji_glove_raw`、`wuji_glove` 记录方式与现有模型输入一致；原始设备时间戳和置信度保留在 `raw_wuji`。

## 离线验收

```bash
~/.venvs/hand-track/bin/python -B -m pytest \
  tests/test_wuji_sdk_direct.py tests/test_wuji_transport.py tests/test_wuji_glove_input.py \
  -q -p no:cacheprovider --basetemp=/home/onkiall/.cache/hand-wuji-direct-test
```

2026-09-30：上述 45 项测试通过。SDK 用例使用 fake manager/设备/回调，覆盖 21 点、重复帧、过期、迟消费、断连重连、旧回调、停止期间阻塞的 scan/connect、序列号错误、清理失败和最后会话释放；包含 fake SDK 回调 → 默认 SDK Application.wuji_frame → 真实 A3.2 solver 的整链集成测试，并阻止构造 HTTP 桥及打开机械手；同时保留原 fetch 注入、HTTP 桥及实际 retarget 测试。没有执行真实手套扫描或机械手运动。真实以太网收发、标定质量和实物控制需连接设备后另行验证。
