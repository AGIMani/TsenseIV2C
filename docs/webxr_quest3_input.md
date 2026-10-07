# Quest 3 WebXR 25 关节输入

`online_control` 网页接入见 [在线 Quest 3 使用说明](online_webxr_quest3.md)。下面介绍 Qt Form 的独立接收服务。

本功能是现有 Form 的第三种关键点输入源，不替换“本机视觉”或原有
“CloudXR Quest 3”。Quest 只在浏览器内识别手骨架，不向 Windows 传图像。

## 已实现的数据链路

```text
Quest 3 Browser / WebXR Hand Input（原生 25 点）
                  │
                  │ HTTPS + 同源 WSS / TCP 8446
                  ▼
Form 进程内 QuestWebXRServer
  /                 WebXR 采集页
  /hand-skeleton    WebSocket 骨架路由
  /healthz          接收状态与性能指标
                  │
                  ├─ 原始 WebXR25：原样保留，可选 JSONL
                  └─ 显式派生 21 点：进入现有 topo/retarget/控制链
```

接收服务和 Form 当前在同一个进程，因此不需要 Named Pipe。将来若拆成两个
进程，进程间接口应使用 `\\.\pipe\teleop-hand-skeleton`，不能再让两个进程
同时抢占 8446。

## Windows 首次设置

### 1. 防火墙和网络类别

Quest 所在 WLAN 必须是可信的专用网络。以管理员身份打开 PowerShell，运行：

```powershell
& 'D:\Qtworks\hand\tools\setup_quest_webxr_firewall.ps1' `
  -SetNetworkPrivate `
  -DisableBroadPythonRules
```

该命令会：

- 将 `WLAN` 从 Public 改为 Private；
- 禁用此 Python 环境原先“任意 TCP/UDP 端口”的宽泛入站规则（规则只禁用，
  不删除，可恢复）；
- 仅允许专用网络、本地子网访问
  `E:\conda\envs\hand_dex\python.exe` 的 TCP 8446；
- 不开放 UDP。

若无线网卡名称不是 `WLAN`，增加
`-InterfaceAlias '<实际名称>'`。若 Python 路径不同，增加
`-PythonPath '<python.exe绝对路径>'`。

### 2. TLS 证书

首次启动会自动生成本地 CA 和带本机 LAN IP SAN 的服务证书：

```text
C:\Users\ONKIALL\AppData\Local\TsenseIHand\webxr\quest_webxr_ca.crt
C:\Users\ONKIALL\AppData\Local\TsenseIHand\webxr\server.crt
C:\Users\ONKIALL\AppData\Local\TsenseIHand\webxr\server.key
```

Quest 只需要信任 `quest_webxr_ca.crt`。不要复制或安装
`quest_webxr_ca.key`、`server.key`。WebXR 的 `immersive-vr` 和手势能力要求
页面处于真正的安全上下文；仅能看到 HTTPS 页面不等于证书已经受信任。

先在 Quest Browser 打开采集页。如果页面提示“不是安全上下文”、
`navigator.xr` 不存在或没有进入 XR 的能力，需要把上述 CA 证书传到 Quest，
通过设备的证书安装界面安装为 CA，然后完全退出并重新打开 Browser。不同
Horizon OS 版本的设置入口可能不同；可用 Meta Quest Developer Hub 的文件传输
和 Browser 调试功能辅助安装与检查。

## 启动 Form

可以直接运行：

```powershell
conda activate hand_dex
cd D:\Qtworks\hand
python example\main.py
```

然后在主界面“输入源”选择 `Quest WebXR 25点`。地址保持 `0.0.0.0`，端口
保持 `8446`，服务会自动启动。切换回另外两个输入源时，8446 会停止并清理
追踪状态。

也可在启动时直接选中 WebXR：

```powershell
$env:HAND_LANDMARK_SOURCE = 'webxr'
$env:WEBXR_BIND_HOST = '0.0.0.0'
$env:WEBXR_PORT = '8446'
$env:WEBXR_ADVERTISE_HOST = '192.168.88.145'
python example\main.py
```

当前 WLAN 地址为 `192.168.88.145`，所以 Quest 打开：

```text
https://192.168.88.145:8446/
```

页面会自动连接同源地址：

```text
wss://192.168.88.145:8446/hand-skeleton
```

不要在 Quest 中单独“访问” WSS 路由。打开 `/` 后点击“进入 XR 并开始采集”，
允许手势追踪，并放下 Touch 控制器、将双手放进头显摄像头视野。

## 先运行独立探针（可选）

不启动 Qt Form，只验证 Quest 到 Windows 的链路：

```powershell
python tools\run_quest_webxr_server.py `
  --host 0.0.0.0 `
  --port 8446 `
  --advertise-host 192.168.88.145 `
  --jsonl logs\quest_webxr_probe.jsonl
```

独立探针与 Form 不能同时运行，因为它们都会使用 TCP 8446。探针停止后再启动
Form。

## 数据约定

- `schema`: `teleop.webxr.hand_skeleton.v1`
- `joint_set`: `webxr-hand-25`
- 关节顺序严格采用 WebXR Hand Input 25 点；不会标成 OpenXR 26 点。
- 每个关节包含 `name`、`position[x,y,z]`、
  `orientation[x,y,z,w]`、`radius`、`valid`。
- 每帧包含 `handedness`、`tracked`、`sequence`、`timestamp_ms`、
  `client_perf_ms`、`xr_predicted_display_time_ms`、`coordinate_space` 和
  `connection_state`。
- 原始坐标保留为 `webxr_local_floor`；若设备只允许回退参考空间，会明确标成
  `webxr_local`。
- 原始 25 点保存在接收服务中。进入旧控制链时，明确跳过四个非拇指
  metacarpal，派生为 MediaPipe 顺序的 21 点。
- 可选 26 点兼容视图会在 25 点前增加一个 `derived=true` 的 palm 点，原始帧
  不变。

## 实时和丢失策略

- Quest 每个 XR frame 采集一次。
- 浏览器端只有一个 `pendingFrame`；新帧覆盖尚未发送的旧帧，不形成无限队列。
- 状态事件独立发送，最多保留 32 条。
- Windows 的 Tracker 同样只保留一个待处理远程帧。
- 浏览器显式发送 `session_started`、`session_ended`、`tracking_lost`、
  `tracking_restored` 和 `client_error`。
- WSS 断开、会话结束或 350 ms 没有新骨架时，Windows 会生成丢失帧并清空
  控制状态，绝不拿最后一帧冒充实时数据。
- `/healthz` 提供 `effective_rate_hz`、`frame_interval_p95_ms`、连接数、有效/
  无效帧数和左右手追踪状态。

## 可选记录与固定标定

原始 WebXR25 JSONL 记录默认关闭。启动前设置：

```powershell
$env:WEBXR_JSONL_PATH = 'D:\Qtworks\hand\logs\quest_webxr.jsonl'
```

头显固定位置与操作方向相反时，可只对进入旧控制链的派生视图增加 180° yaw：

```powershell
$env:WEBXR_CALIBRATION_YAW_DEG = '180'
$env:WEBXR_CALIBRATION_TRANSLATION_M = '0,0,0'
```

标定后的坐标会标记为 `derived(...)`，原始 `webxr_local_floor` 数据和 JSONL
不会被覆盖。默认 yaw 为 0°。

## 验收

打开以下状态页：

```text
https://192.168.88.145:8446/healthz
```

重点检查：

- `joint_set` 为 `webxr-hand-25`，`native_joint_count` 为 25；
- 展示左/右手时 `tracked_hands` 均能变为 `true`；
- 稳定展示手后 `effective_rate_hz >= 30`；
- `frame_interval_p95_ms < 80`；
- 遮挡后对应手变为 `false`，恢复后变回 `true`；
- `connections` 在退出页面后回到 0；
- `invalid_messages` 保持 0。

实现依据：WebXR Hand Input 规范固定返回 25 个关节；WebXR immersive 会话要求安全
上下文和用户授权。参考 [W3C WebXR Hand Input](https://www.w3.org/TR/webxr-hand-input-1/)、
[Meta Quest Browser](https://developers.meta.com/horizon/documentation/web/) 和
[Meta 的 WebXR HTTPS 排障说明](https://developers.meta.com/horizon/documentation/iwsdk/troubleshooting/)。
