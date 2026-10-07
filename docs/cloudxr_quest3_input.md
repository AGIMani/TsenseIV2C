# CloudXR Quest 3 关键点输入

主界面现在有两个互斥但并存的 21 点输入源：

- **本机视觉**：保持原来的 MediaPipe / WiLoR / HandOccNet 相机检测路径，也是默认项。
- **CloudXR Quest 3**：接收 CloudXR/OpenXR 进程发布的 Quest 3 手部关键点。

两路数据只在 21 点入口前分流。选中的数据随后共用 Tracker 的关键点质量门控、Topo retarget、可选 MuJoCo 碰撞、Ruckig、URDF 显示和 H7 下发，不存在第二套关节控制链。

## 主界面使用

1. 在“输入源”中选择 **CloudXR Quest 3**。
2. 保持默认监听地址 `127.0.0.1:28770`，或按 CloudXR 桥接进程的部署位置修改。
3. 点击“启动监听”。选择 CloudXR 输入源时程序也会自动尝试启动监听。
4. 状态从“监听”变为“接收中”后，3D 骨架、retarget 和模型同步使用 Quest 3 数据。
5. 切回“本机视觉”即可恢复原相机检测路径；原功能和参数均保留。

同机 CloudXR 进程应使用 `127.0.0.1`。只有进程运行在另一台机器且网络可信时，才把监听地址改为 `0.0.0.0` 或指定网卡地址，并放行对应 UDP 端口。

也可在启动前设置：

```text
HAND_LANDMARK_SOURCE=cloudxr
CLOUDXR_LANDMARK_HOST=127.0.0.1
CLOUDXR_LANDMARK_PORT=28770
```

## 使用 AGIMani/Teleop 获取 Quest 3 手势

项目已提供独立桥接脚本 `tools/run_quest3_teleop_bridge.py`。它参考
`external/Teleop` 中的 Quest 输入实现，只创建 `HandTracker` 和
`raw_hand` 输出，不连接机械臂、不向灵巧手发送命令，也不修改 Teleop
仓库。完整数据流如下：

```text
Quest 3 光学手势
  → CloudXR WSS / OpenXR Runtime
  → IsaacTeleop HandTracker（OpenXR 26 点）
  → run_quest3_teleop_bridge.py（校验 + JSON/UDP）
  → Form CloudXRLandmarkReceiver（26 → 21）
  → 原有 SkeletonWorker / Topo retarget / 显示 / 可选硬件下发
```

桥接协议核心位于 `src/quest3_teleop_bridge.py`，只使用 Python 标准库，
因此云端 Teleop 环境不需要安装 PySide6、MediaPipe、WiLoR 等桌面端依赖。

### 1. 云端依赖

`external/Teleop` 是集成层，NVIDIA IsaacTeleop 是它的上游依赖。当前
Teleop 检出中如果没有 `external/IsaacTeleop`，请在**实际运行 CloudXR
的 Linux 主机**执行：

```bash
cd /path/to/Teleop
make clone-upstream
```

该命令会克隆 NVIDIA IsaacTeleop，并应用 Teleop 仓库需要的 CloudXR
overlay。不要只在 Windows Form 电脑上准备依赖后期待远端主机可见；桥接
脚本必须运行在拥有 `NV_CXR_RUNTIME_DIR/ipc_cloudxr` 的 CloudXR 主机上。

先做不打开 XR 会话的导入检查：

```bash
conda activate isaac_teleop
python /path/to/hand/tools/run_quest3_teleop_bridge.py \
  --teleop-root /path/to/Teleop \
  --check-only
```

看到 `check=ok` 说明 Teleop 与 IsaacTeleop 的 Python 环境已就绪。

### 2. 启动 CloudXR 与桥接进程

终端 A 保持 CloudXR 运行：

```bash
cd /path/to/Teleop
./scripts/start_cloudxr.sh
```

终端 B 使用相同的 IsaacTeleop 环境，并加载 CloudXR 运行时变量：

```bash
conda activate isaac_teleop
source ~/.cloudxr/run/cloudxr.env

python /path/to/hand/tools/run_quest3_teleop_bridge.py \
  --teleop-root /path/to/Teleop \
  --host <运行Form的电脑IP> \
  --port 28770 \
  --hand right \
  --loop-hz 60
```

如果 Form 和 CloudXR/桥接进程在同一台主机，`--host` 使用
`127.0.0.1`，Form 也监听 `127.0.0.1:28770`。如果两者在不同主机：

- Form 中监听 `0.0.0.0:28770`；
- `--host` 填 Form 电脑的 LAN/VPN IPv4，例如 `192.168.88.145`；
- Windows 防火墙需允许入站 UDP 28770；
- 不要把 Quest 浏览器要访问的 CloudXR 服务器 IP 与这里的 Form 接收 IP
  混为一谈。

Windows 可用以下命令确认 Form 已经绑定端口：

```powershell
Get-NetUDPEndpoint -LocalPort 28770
```

需要新增防火墙规则时，在管理员 PowerShell 中执行一次：

```powershell
New-NetFirewallRule -DisplayName "Quest3 Form UDP 28770" -Direction Inbound -Protocol UDP -LocalPort 28770 -Action Allow
```

### 3. Quest 3 连接地址

在 Quest 3 浏览器中访问的是 **CloudXR 主机**，不是 Form 的 UDP 端口：

1. 打开 `https://<CloudXR服务器IP>:8443/` 并接受自签名证书；
2. 打开一次 `https://<CloudXR服务器IP>:48322/`，接受 WSS 证书；
3. 返回 8443 页面，选择 Quest 3 配置并点击 `Connect`；
4. 在 Quest 中启用光学手部追踪，把 Touch 控制器放下并等待其休眠；
5. 回到 Form，选择 **CloudXR Quest 3** 并点击“启动监听”。

Teleop 在共享服务器上可能为不同用户分配不同 WSS 端口，应以
`./scripts/cloudxr_status.sh` 或启动日志显示的端口为准，48322 只是基础
默认值。这个 Teleop 版本的完整 Web 客户端监听 HTTPS 8443，
`http://<IP>:8080/` 不是这条接入链路的访问地址。

### 4. 运行状态与故障定位

正常时桥接终端依次出现：

```text
[quest3-bridge] cloudxr_ipc=.../ipc_cloudxr
[quest3-bridge] udp_destination=<Form IP>:28770
[quest3-bridge] tracking=active sent=1 ...
```

Form 状态会从“监听”变成“接收中”。常见异常：

- `Teleop/IsaacTeleop import failed`：未激活 `isaac_teleop` 环境，或尚未运行
  `make clone-upstream`；
- `NV_CXR_RUNTIME_DIR is not set`：当前终端没有执行
  `source ~/.cloudxr/run/cloudxr.env`；
- `ipc_cloudxr` 不存在或无法连接：CloudXR 没启动、已经退出或需要重启；
- 桥接显示 `tracking=active`，Form 仍只有“监听”：检查 `--host`、Form 的
  `0.0.0.0` 绑定、UDP 28770 防火墙和两机路由；
- 只显示 `tracking=lost`：确认 Quest 已进入光学手势模式、控制器已休眠、
  `--hand left/right` 与正在使用的手一致，并改善手部照明；
- `frame_dropped=required Form joints are invalid`：某些关键关节暂时无效；
  桥接会容忍 0.35 秒，持续无效后才让现有安全链进入“手消失”。

退出桥接进程时会额外发送一次 `tracked=false`，Form 不会继续保持最后一帧。

## CloudXR 进程输出协议

传输为 **JSON/UDP**，每个数据报是一帧。默认端点为 `127.0.0.1:28770`。Quest/OpenXR 坐标单位通常为米。

### 21 点报文

CloudXR 进程已经转换成 MediaPipe Hands 顺序时发送：

```json
{
  "type": "hand_landmarks",
  "sequence": 1842,
  "timestamp_ns": 3928445123000,
  "hand": "right",
  "tracked": true,
  "space": "openxr_local",
  "unit": "m",
  "landmarks": [
    [0.012, 1.104, -0.421]
  ]
}
```

示例只展示一个点；实际 `landmarks` 必须包含完整的 21 个 `[x,y,z]`。

### Quest/OpenXR 原生 26 点报文

也可直接发送 `XR_EXT_hand_tracking` 的 26 个关节：

```json
{
  "sequence": 1842,
  "handedness": "left",
  "tracked": true,
  "coordinate_space": "openxr_stage",
  "unit": "m",
  "joints": [
    {"position": {"x": 0.0, "y": 1.1, "z": -0.4}}
  ]
}
```

实际 `joints` 必须包含完整 26 点。接收端按以下索引自动转换成 MediaPipe 21 点：

```text
OpenXR:    1,
           2,3,4,5,
           7,8,9,10,
           12,13,14,15,
           17,18,19,20,
           22,23,24,25
MediaPipe: 0..20
```

即丢弃 OpenXR 的 `PALM(0)` 以及食指到小指各自额外的 metacarpal 点，保留 `WRIST(1)` 和各指与 MediaPipe 拓扑对应的四点。

### 字段约定

| 字段 | 必需 | 说明 |
|---|---:|---|
| `landmarks` / `keypoints` / `points` / `joints` | 跟踪时是 | 21 点 MediaPipe 顺序或 26 点 OpenXR 顺序 |
| `hand` / `handedness` / `side` / `hand_side` | 建议 | `left` 或 `right`，缺省为 `left` |
| `tracked` | 否 | `false` 可不带点，用于显式报告手消失 |
| `sequence` / `seq` / `frame_id` | 建议 | 帧序号，贯穿同步数据记录 |
| `timestamp_ms/us/ns` | 建议 | 采集时间；未提供时使用接收端单调时钟 |
| `unit` | 否 | `m`（默认）、`cm` 或 `mm`；内部统一转为米 |
| `space` / `coordinate_space` | 否 | OpenXR 的 local/stage/world 均可，接收后自动转腕坐标系 |

点既可写成 `[x,y,z]`，也可写成 `{"x":...,"y":...,"z":...}` 或带 `position` 的 OpenXR 风格对象。多手报文可使用 `{"hands":[...]}`；程序选择 `tracked=true` 且置信度最高的一只手。

如果 CloudXR 进程已经输出项目内部的 Topo 腕坐标系，可把 `space` 设为 `topo_wrist`，接收端将跳过坐标系规范化。其他空间都会根据 Wrist、IndexMCP、MiddleMCP 自动建立腕坐标系，因此不依赖 Quest 在云端场景中的绝对位置和朝向。

## 丢帧与安全行为

- UDP 路径只保留实时帧，不重放积压数据。
- 连续 `0.35 s` 未收到有效 CloudXR 报文时，Tracker 将手标记为消失并进入现有回零逻辑。
- 点数错误、非有限坐标或未知单位会丢弃，并在主界面状态中显示格式警告。
- CloudXR 源只替换 21 点输入；“模型同步”仍默认关闭，必须由用户手动开启后才允许手势路径下发 H7。
