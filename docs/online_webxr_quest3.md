# online_control：Quest 3 WebXR 关键点输入

手势映射工作台现在可选择「MediaPipe · 相机」或「Quest 3 · WebXR」。Quest
提供关节坐标，电脑继续运行现有重定向、模型预览、碰撞保护、Ruckig 和设备下发。
支持工作台中的 A3.0、A3.0 with ID、A3.2、A3.2 with ID 及 V2，实际可用型号和
设备接口仍由服务器运行环境决定。

## 使用步骤

1. 在电脑打开工作台，选择型号、左右手与「Quest 3 · WebXR」。也可以在网址后
   加 `?source=webxr`。停止现有会话后才能切换输入源。
2. 点击「开始识别」，页面显示固定的 `/q` 短入口和 **永久配对码 `123456`**。
3. 在 Quest Browser 首次打开该短网址并收藏。采集页自动填入 `123456` 并连接电脑，
   再点击「进入 XR 并开始采集」。以后直接从收藏打开，无需重新输入，重启服务也不换码。
   如果打开采集页时电脑还未开始识别，先在电脑开始，再点击「连接电脑」。
   同一服务器请只保留一个 Quest 控制会话；存在多个会话时会提示停止多余会话，避免配错。
   原 `/quest.html` 入口和旧长链接仍兼容。
   电脑和 Quest 必须访问同一台服务器。链接中的 `localhost` 指访问设备自身，
   普通无线连接需使用可访问的局域网 HTTPS 地址。
4. 在 Quest 采集页点击进入 XR，允许手部追踪，放下控制器，让手保持在头显视野内。
   AR 模式可看见周围环境；不支持时选择 VR 并重新点击启动。
5. 保持电脑控制页面在前台，确认 21 点骨架、左右手和模型跟随正确，再按已有流程
   连接设备、启用碰撞保护并手动开启实时下发。采集页不提供下发按钮。
6. 点击电脑「停止识别」会停止输入和控制会话，并撤销旧采集连接。配对码一直是
   `123456`；电脑下次开始后，重新打开采集页或点击「连接电脑」即可。

### 使用已有 node0 HTTPS 服务

将此版本按原部署流程发布后，电脑与 Quest 使用同一 HTTPS 工作台地址，采集页面
与 `/api/webxr/*` 共用该站点，不需要新增端口。Quest 必须信任该站点证书。
A3.0 / V2 的 Web Serial 仍在电脑浏览器执行；A3.2 原生 SDK 控制仍需运行在连接
设备的 Ubuntu 本机，公共服务不提供 A3.2 SDK 下发。

### 原生 Ubuntu 通过局域网连接

在项目根目录执行：

```bash
bash migration/ubuntu/start-track.sh
```

启动器使用独立 track 环境，自动选择物理 Wi-Fi/以太网的 IPv4 地址并生成 HTTPS 证书。
多网卡时可用 `HAND_QUEST_ADDRESS=<电脑局域网IP>` 指定。普通启动命令默认使用 HTTPS，
同一工作台可切换 Quest、Wuji、Leap Motion，无需重启。`start-quest.sh` 仅保留为兼容别名。

电脑控制页打开 `https://127.0.0.1:8766/?source=webxr`，Quest 打开
`https://<当前电脑IP>:8766/q`。点击「开始识别」后，电脑页面会显示正确的 Quest 地址。
A3.2 SDK 控制接口要求请求来自本机回环，因此电脑控制页继续使用 127.0.0.1。
头显上的 127.0.0.1 指向头显自身，不能连接电脑。

电脑与 Quest 需连接同一局域网，并允许访问 TCP 8766；此启动器不修改防火墙。
首次需让访问端信任 `runtime/ubuntu/quest-tls/quest_webxr_ca.crt`，不要复制私钥。
证书包含当前电脑 IP；更换网络后重新启动即可更新。WebXR 需要安全上下文，
仅能打开 HTTPS 页面不代表已完成证书信任。

### USB ADB 直连：21 点接入口

在 Quest 开启开发者模式，通过 USB 数据线连接 Ubuntu，并在头显内允许 USB 调试。
安装 ADB 后启动专用入口：

```bash
sudo apt install adb android-sdk-platform-tools-common
adb devices -l
bash migration/ubuntu/start-quest-adb.sh
```

电脑打开 `http://localhost:8766/?source=quest_adb`，选择「Quest 3 · USB ADB（21点）」、
型号和实际手别，点击「开始识别」。Quest Browser 打开 `http://localhost:8766/q/adb`，
使用自动填入的 `123456` 连接，然后点击「进入 XR 并开始采集」。USB 启动器使用独立 track
环境，将默认端口 8766 的工作台重启为回环 HTTP；原 HTTPS 收藏此时需要换成上述 HTTP 地址。
普通 `start-track.sh` 仍使用局域网 HTTPS。

启动器检查 USB 设备状态并建立 `adb reverse tcp:8766 tcp:8766`。已有相同转发会复用；
冲突转发不会被覆盖。多个 USB 设备时设置 `HAND_QUEST_ADB_SERIAL=<Quest序列号>`；
要更换端口，设置 `HAND_QUEST_ADB_PORT=9001`，电脑和头显均使用新端口。
`--check` 只运行无硬件软件检查，不调用 ADB。USB 断开或头显重启后，重新运行启动器。
结束时停止电脑识别、退出服务，需要清理转发可执行：

```bash
adb -s <Quest序列号> reverse --remove tcp:8766
```

这条链路仍由 Quest Browser 的 WebXR 获取手部追踪；ADB 负责 USB 传输，不是骨架采集 API，
无需额外原生 APK。头显将 25 关节映射为 MediaPipe 顺序的 21 点，在上传前去掉四个非拇指
metacarpal；电脑接收每只手的 21×3 米制坐标，减去手腕平移，保留 WebXR 轴向、不镜像，
接入现有重定向、记录和丢手处理。原 `/q` 和 25 点协议保持可用。

新增同源接口：

- `POST /api/quest-adb/pair`：`{"code":"123456"}`，返回当前控制会话的 `token`。
- `POST /api/quest-adb/publish`：`{"token":"配对返回值","message":{...}}`。
- 电脑仍使用 `/api/webxr/create`（附 `transport:"adb"`）和 `/api/webxr/latest`。
  后端管线来源为 `webxr`，记录中用 `source_metadata.transport="adb"`、
  `source_joint_count=21` 和原始 schema 区分 USB 21 点输入。

每个流先发 `client_hello`，再发 `hand_skeleton`，结束发 `session_ended`。
所有消息的 schema 为 `teleop.quest.adb.hand_skeleton.v1`，带 `stream_id`、递增安全整数
`sequence`。骨架消息还必须带 `joint_set:"mediapipe-21"`、`units:"m"`、
`coordinate_space:"webxr_local"` 或 `"webxr_local_floor"`、单调 `client_perf_ms`，以及：

```json
{"hands":[{"handedness":"left","tracked":false,"landmarks":[]}]}
```

以上表示未追踪。追踪时 `tracked:true`，`landmarks` 必须是恰好 21 个有限数值 xyz 三元组；
顺序为 wrist，然后拇指 CMC/MCP/IP/tip，再依次食指、中指、无名指、小指 MCP/PIP/DIP/tip。
可同时提供左右手，不能重复同一手别。空手、超时、旧序号、结束的流不会恢复有效输入；
同一 `stream_id` 不允许从 21 点切换为 25 点协议。原始 21 点包保留在 `raw_webxr` 中。

排错：`unauthorized` 需在头显授权，`offline` 需重连数据线；无设备时检查开发者模式、
USB 数据线和 Linux udev 权限。确认 `adb devices -l` 带 `usb:`，启动器不会选用无线 ADB。
头显只在这条已建立 reverse 的 USB 链路中使用 localhost；普通无线模式仍用局域网 HTTPS。
若日志出现证书拒绝，说明仍在访问 HTTPS 地址。若发布返回 400，查看采集页和服务端具体错误；
常见配对失效需先在电脑开始识别，再在头显重新连接，不能单凭 HTTP 状态判断为骨架格式错误。

本机回环与 USB 转发方式参见 [Meta Browser 调试文档](https://developers.meta.com/vr/documentation/web/browser-remote-debugging/)
和 [Android ADB reverse 文档](https://developer.android.com/develop/ui/views/layout/webapps/access-local-server)。

## 数据和精度

```text
Quest XRFrame / getJointPose：原生 25 关节，米
    → 同源 HTTPS POST /api/webxr/publish：单请求 + 最新待发帧
    → 与控制会话绑定的 relay：序号校验 / 输入时效检查
    → 电脑 latest → /api/frame：25 → 21，以手腕为原点
    → 原有平滑 / 手腕坐标对齐 / retarget
    → 模型碰撞 / 轨迹与采样检查 / 原有设备控制
```

25→21 复用 Qt 的 `WEBXR_25_TO_MEDIAPIPE_21`，跳过食指、中指、无名指、小指的
metacarpal，保持 MediaPipe 的关键点顺序。保留原始关节的 position、orientation、
radius、valid 及参考空间，实际重定向继续使用 21 个关节的位置。坐标不伪造为图像像素，
不进行额外镜像；V2 不套用摄像头 Wuji 固定骨长修正，保留 Quest 的米制骨架。

控制记录使用 `webxr_raw` 和明确的 WebXR 来源字段，保留原始 25 点与派生关系。
兼容对象中的 `normalized` 仅为旧接口占位，标记 `normalized_is_image=false`；
不能拿来叠加摄像头画面或计算图像空间误差。

接收超时阈值为 350 ms。服务器结合接收时刻与 Quest 的单调 `client_perf_ms`，
按同一 XR 流观测到的最小时间偏移估计额外传输延迟，并将该延迟计入输入龄期；
延迟达到 350 ms 或单调时间回退的骨架不会重新激活跟随，不要求两设备墙上时钟一致。
首次收包的绝对网络延迟未知：这项检查衡量相对最快已观测传输的延迟，不能证明首包
的绝对采集年龄。每次新 XR 会话重新建立基线，日志保留 `transport_delay_ms`。
丢失所选手、
关节无效、XR 结束、输入超时均产生空手观察，进入原有 tracking-lost 路径。
序号和 XR 流标识阻止重复帧与旧会话的迟到帧重新驱动控制。电脑页面隐藏/关闭或
接收请求失败会停止本次识别；恢复后按原控制界面重新开始。

Quest 的米制关节避免了摄像头 normalized Z 回退，但准确度仍受遮挡、光照、手部
追踪状态、机器人标定与重定向误差影响。WebXR 不提供统一的逐关节置信度或毫米级
准确度保证。建议真机验收比较张掌、握拳、拇食捏合、侧摆与左右手，并记录端到端延迟、
关节抖动、遮挡恢复和断流停发。软件自动测试不替代这些测量。

协议依据：[W3C WebXR Hand Input](https://www.w3.org/TR/webxr-hand-input-1/)、
[Meta WebXR Hands](https://developers.meta.com/horizon/documentation/web/webxr-hands/)。

## 软件验证

本次 Python 联合回归 75 项通过，覆盖 Qt 原协议、左右手和各型号映射、真实重定向、
失效/重放拒绝、HTTPS 和日志。JavaScript 联合回归 37 项通过；Chrome 验证包含
WebXR API 模拟采集，以及原生 25 点测试输入通过实际服务求解后驱动网页模型。
日志查看器测试与最小发布包的独立 HTTP → 求解验证也通过。测试未连接实体 Quest
或机械手，也未更新 node0。

```powershell
$env:NO_PROXY = '*'
& 'E:\conda\envs\hand_dex\python.exe' -m pytest tests/test_online_webxr_input.py tests/test_webxr_online_pipeline.py tests/test_webxr_input.py tests/test_web_v3_2.py tests/test_v3_2_recording.py tests/test_web_v2.py tests/test_a32_tracking_recording.py -q
npm run test:webxr --prefix online_control/online_motion_track
```

实际服务浏览器测试：先在一个终端运行
`python online_control/online_motion_track/main.py --port 18866 --no-open`，另一个终端
执行 `npm run test:webxr-online --prefix online_control/online_motion_track`。默认使用
已安装的 Chrome 和 `hand_dex` 环境，可通过 `CHROME_PATH`、`HAND_PYTHON`、`WEB_URL`
覆盖。该测试注入合成骨架、禁用摄像头请求，不会连接硬件。
