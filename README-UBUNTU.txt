hand 原生 Ubuntu 网页版迁移包
目标：Ubuntu 24.04 LTS，x86_64 / amd64，Python 3.12。
制作日期：2026-10-06（Asia/Shanghai）

一、包内内容
- 当前网页源码（包括未提交/未跟踪改动）、模型、网格、标定、motion、
  网页复用的底层模块、外部源码、现有导出成果。
- A3.2 Linux liba32_native.so 及完整原生源码/构建与离线测试。
- three / MediaPipe 浏览器运行资源和 hand_landmarker.task。
- 当前 Linux 环境导出的 track/play 依赖清单；不含 Python 虚拟环境。
- Leap Motion Linux 6.2.0 官方安装包、协议、Python 3.12 x86_64 wheels
  和官方 binding 源码，是重装材料，不是已经复制的 Python 环境。
- migration/ubuntu/preserved-runtime-data 中保存旧网页的两份动作文件。

按用户要求不含历史运行日志与独立 Qt 桌面界面。
网页动作播放后台已改为纯 Python 控制会话和事件循环，不需要 Qt/PySide6。
还排除了 WSL/Windows 桥接及启动脚本、旧部署凭据、Python/测试/构建缓存、
旧 runtime、Git 元数据与历史备份。
原工程没有被覆盖。排除明细见 migration/ubuntu/excluded-files.json 和 gui-exclusions.json。
源版本、原工作目录状态和初始快照哈希见 migration/ubuntu/source-*。
最终迁移改动清单见 migration/ubuntu/migration-changes.json；最终文件校验见 FILES.sha256。
这是当前文件快照，不含 Git 提交历史；继续版本管理可在新机自行 git init。
原功能文档含历史 Qt/Windows/WSL/node0 记录；原生网页版启动以本说明为准。

二、解压和安装
将 hand-ubuntu24.04-20261006.tar.gz 与 SHA256SUMS 复制到 Ubuntu。
在存放压缩包的目录运行：
  sha256sum -c SHA256SUMS
  mkdir -p ~/projects
  tar -xzf hand-ubuntu24.04-20261006.tar.gz -C ~/projects
  cd ~/projects/hand
如果 ~/projects/hand 已存在，请解压到新的空目录，不要覆盖已有项目。

先安装系统依赖（需联网）：
  sudo apt update
  sudo apt install -y python3.12 python3.12-venv python3.12-dev build-essential cmake \
    ninja-build ca-certificates pkg-config libffi-dev libgl1 libegl1 libopengl0 libglib2.0-0t64 \
    libusb-1.0-0 libudev-dev usbutils nodejs npm

安装两套独立环境：
  bash migration/ubuntu/install.sh all
只用手势工作台可改为 track；只用动作播放为 play。
新环境默认位于项目 .venvs/track、.venvs/play。
依赖需联网重新下载，这不是完全离线安装包。不要合并两套依赖清单。
play 已移除 PySide6/shiboken6；动作文件格式、网页接口、驱动与协议保持兼容。
不再提供桌面 GUI 启动器或 GUI 环境清单。重构范围与测试详见 migration/ubuntu/VALIDATION.txt。

A3.2/withid 已于 2026-10-10 更新到用户提供的 SDK 1.1.14，包括标定闭环
角度/行程换算、拇指 MCP 16.95 mm 上限、电角度分类修正和 SELF 安培电流。
源码及压缩包校验见 vendor/a3-cpp-sdk/sdk-source.json，详情见
docs/A32_NATIVE_SDK.md。同一旧动作角度的行程将按新版算法计算，旧日志保留原单位。
已有服务需在停止控制后分别重启 track/play，才能加载本次更新。
A3.2 驱动可在新机重新编译：
  bash src/a32_driver/build_native_linux.sh
附带 .so 是 Ubuntu 24.04 x86_64 构建；ARM64 需重建驱动和 Leap binding。
前端生产资源已经包含；需要重新安装时：
  (cd online_control/online_motion_track && npm ci --omit=dev)
日常运行不需要 Playwright；开发浏览器测试才需要安装开发依赖。

三、启动
在项目根目录执行：
  bash migration/ubuntu/start-track.sh       # 手势工作台，https://127.0.0.1:8766/
  bash migration/ubuntu/start-play.sh        # 动作播放，http://127.0.0.1:8765/
同一工作台可选择 Quest、wuji_glove、leapmotion 或摄像头。
新增「D455f 深度融合 · 21 点」：原生 SDK 同步读取 RGB/深度，MediaPipe
识别后结合深度、骨长和姿态先验修正21点，工作台与动态标定页均可选择。
入口：https://127.0.0.1:8766/?source=realsense_rgbd
此方式优先采用设备支持的30 FPS同步模式；深度缺失/过期时不提交单目替代结果。
依赖、原生USB权限与算法限制见 docs/realsense_rgbd_input.md。
原有下面的 D455f 普通 RGB 相机方式保留，停止识别后可切换。
RealSense 作为普通相机使用：选择「MediaPipe · 相机」，在「视频来源」选择
RealSense 的 RGB/彩色视频流，再点击「开始识别」。工作台与标定页会查询该视频流
在当前浏览器中的最高采集帧率，允许降低分辨率，配置并核对成功后才启动 MediaPipe。
画面状态显示分辨率与「采集设置 FPS」；这不是实测推理 FPS。其他普通相机保持 30 FPS。
本机 D455f 的 USB 3 RGB 模式最高为 90 FPS（640×360）；640×480 最高为 60 FPS。
程序按设备报告的能力选择，不将 90/120 FPS 写死；不提供帧率能力的浏览器会提示错误。
2026-10-10 实机验证：同一前端配置模块在浏览器中协商到 640×360 / 90 FPS，
通过 MediaStreamTrackProcessor 连续采样约 4 秒，帧时间戳计算约 89.87 FPS。
原生 V4L2 连续采集 450 帧也约 89.87 FPS。以上为采集验证，不代表 MediaPipe 推理速度。
摄像头画面叠加的 21 点骨架采用自适应 One Euro 平滑：静止时压低抖动，
运动时提高响应速度；使用每次识别的时间戳，不等待多帧、不预测未来位置。
采集设置新增两个独立开关，默认均开启，识别中可随时切换：
「画面骨架平滑」只影响相机画面的 21 点叠图；「MediaPipe 输入滤波」控制
送入重定向的 MediaPipe 三维与归一化关键点滤波。关闭输入滤波后使用当前原始点，
重新开启会从新帧建立滤波历史。原始记录始终保留原始点，每帧附带输入滤波状态。
双手检测顺序变化时按手别保留滤波历史；丢手立即清空，重新入镜或超过
300 ms 的识别间隔重新初始化。识别手别不明确时不混用两只手的历史。
画面骨架平滑适用于普通相机、RGB-D 的图像骨架和 Leap 红外相机预览；输入滤波
开关适用于普通 MediaPipe 相机和 Leap 红外 MediaPipe 输入。更新后停止识别，
重启跟踪服务，按 Ctrl+Shift+R 刷新网页，再开始识别即可。
遮挡或误识别的准确率仍需实机验证。
新增「Leap Motion 2 相机 · MediaPipe · 120 fps」：USB 3 接入后，把左路红外画面
交给 MediaPipe 识别。网页分开显示采集和识别帧率；120 fps 采集不保证 120 fps 推理。
相机检查页：https://127.0.0.1:8766/leap-camera-preview.html
工作台入口：https://127.0.0.1:8766/?source=leapmotion_camera
配置、限制和实测见 docs/leapmotion2_input.md。
停止当前识别后切换输入源，再点击「开始识别」，无需更换命令或重启服务。
start-wuji.sh 和 start-quest.sh 保留为兼容入口，平时只需 start-track.sh。
终端保持开启，Ctrl+C 退出。
网页不会自动连接、使能或驱动机械手。

3.1 Wuji 手套：网线直连 Ubuntu
给手套通电，用网线连接手套与运行工作台的 Ubuntu 电脑。
以下按此前现场手套地址 192.168.1.100:50001 配置；若设备 IP 已修改，
请使用实际手套 IP，并为电脑选择同一子网内未被占用的另一地址。

打开 Ubuntu「设置 → 网络 → 有线」中对应网口的齿轮，进入「IPv4」：
  IPv4 方法：手动
  地址：192.168.1.50        （电脑有线网卡 IP）
  子网掩码：255.255.255.0
  网关：留空
  DNS：关闭「自动」，留空
  路由：保持自动，无需添加静态路由
  勾选「仅对该网络上的资源使用此连接」
点击「应用」，关闭再打开该有线连接，让新配置生效。
192.168.1.100 是手套 IP，不要把它填为电脑地址或网关。
专用手套网口不设默认网关；电脑可继续通过 Wi-Fi 上网。

也可以使用命令完成上述填写，无需再在设置窗口逐项输入：
先查看网口和连接名称，确认哪一个接着手套：
  nmcli device status
  nmcli -f NAME,UUID,TYPE,DEVICE connection show
  nmcli -f GENERAL,WIRED-PROPERTIES device show eno1
本机已确认手套网口为 eno1，对应「有线连接 1」；另一个 USB 以太网设备
是 iPhone 网络，不要误选。换电脑时按实际结果替换下面两个变量，
不能仅凭 ethernet 类型或「有线连接 1」名称判断。

在 Bash 终端执行（配置会持久保存，重启后无需重复填写）：
  WUJI_IFACE='eno1'
  WUJI_CONNECTION='有线连接 1'
  nmcli connection modify "$WUJI_CONNECTION" \
    ipv4.method manual ipv4.addresses 192.168.1.50/24 \
    ipv4.gateway '' ipv4.dns '' ipv4.dns-search '' \
    ipv4.ignore-auto-dns yes ipv4.never-default yes \
    ipv6.never-default yes connection.autoconnect yes
  nmcli --wait 20 connection up "$WUJI_CONNECTION" ifname "$WUJI_IFACE"
  ip -4 address show dev "$WUJI_IFACE"
  ip route get 192.168.1.100
  ping -I "$WUJI_IFACE" -c 4 -W 2 192.168.1.100
  ip route show default
若提示权限不足，在两条修改/启用连接的 nmcli 命令前加 sudo 后重试。
/24 等价于子网掩码 255.255.255.0；never-default 对应仅将该连接用于
本地网络资源，避免手套网口抢占默认路由。修改会重新激活所选有线连接。
检查手套路由应经过 eno1、源地址应为 192.168.1.50，默认路由仍走原上网接口。
如果设置窗口在命令执行前已打开，关闭后重新打开查看，避免应用旧页面内容。

在终端检查有线地址及连通性：
  ip -4 address
  ping -c 4 192.168.1.100
应能看到电脑有线网卡的 192.168.1.50/24，并收到手套的 ping 回复。
无回复时先检查手套供电、网线、所选网口和设备实际 IP；ping 仅验证网络可达，
最终还需在网页确认 SDK 已连接并持续收到骨架。

在项目根目录启动：
  WUJI_DEVICE_ADDRESS=192.168.1.100:50001 bash migration/ubuntu/start-track.sh
电脑浏览器打开 https://127.0.0.1:8766/，选择模型、实际手别和 Wuji 手套
（wuji_glove）输入源，点击「开始识别」，确认骨架和模型随手部动作更新。
WUJI_DEVICE_ADDRESS 填手套地址和端口；不设置时使用 SDK 默认发现。
此流程通过 Linux SDK 原生联网，不需要 Windows UDP/HTTP 转发。

3.2 Quest 3：同一局域网内通过浏览器连接
电脑和 Quest 连接同一局域网，避开隔离客户端的访客 Wi-Fi。
Quest 使用电脑在该局域网中的 IP；Wuji 专用网口的 192.168.1.50 通常不适用。
在项目根目录启动：
  bash migration/ubuntu/start-track.sh
工作台默认在 8766 提供 HTTPS，Quest、Wuji、Leap Motion 共用该服务。
自动选择本机物理 Wi-Fi/以太网地址，生成含当前 IP 的证书；VPN 网卡不作为采集地址。
可用 ip -4 address 查看地址。多网卡时可显式指定，例如电脑 Wi-Fi IP 为
192.168.100.24 时（请替换为当前实际地址）：
  HAND_QUEST_ADDRESS=192.168.100.24 bash migration/ubuntu/start-track.sh
没有可用局域网地址时仍可使用本机工作台；联网后重新启动即可开放 Quest 访问。
首次需让访问端信任 runtime/ubuntu/quest-tls/quest_webxr_ca.crt；私钥不要传到设备。
WebXR 需要安全上下文；若出现证书错误或无法进入 XR，应先完成证书信任。
仅能打开 HTTPS 页面不代表已完成信任。更换电脑局域网 IP 后重新启动服务，
让启动器更新证书，并使用页面显示的新地址。

连接顺序：
1. 电脑打开 https://127.0.0.1:8766/?source=webxr，选择模型、实际手别和
   「Quest 3 · WebXR」，点击「开始识别」。
2. 在 Quest Browser 打开电脑页面显示的 https://<电脑局域网IP>:8766/q。
   例如上述 IP 对应 https://192.168.100.24:8766/q，可收藏此地址。
   头显中不能填 127.0.0.1 或 localhost，它们指向头显自身。
3. 采集页自动填入固定配对码 123456 并连接电脑。若未连接，确认电脑已经
   开始识别，再点击「连接电脑」；同一服务器只保留一个 Quest 控制会话。
4. 点击「进入 XR 并开始采集」，允许手部追踪，放下控制器，让手保持在
   头显摄像头视野内。确认电脑工作台的骨架与模型持续更新。
5. 结束时在电脑点击「停止识别」。下次先在电脑开始识别，再打开头显收藏
   或点击「连接电脑」；停止后旧采集连接会失效，配对码仍为 123456。
若网址仍超时，检查 Wi-Fi 客户端隔离和 Ubuntu 防火墙的局域网 TCP 8766 访问规则。

3.2.1 Quest 3：USB ADB 直连（21 个关节点，无需局域网证书）
先在 Quest 开启开发者模式，使用 USB 数据线连接 Ubuntu，并在头显内允许 USB 调试。
首次安装 ADB：
  sudo apt install adb android-sdk-platform-tools-common
检查设备（应为 device，并带 usb:；unauthorized 表示尚未授权）：
  adb devices -l
启动 USB 专用入口（使用独立 track 环境，会将原 8766 工作台重启为回环 HTTP）：
  bash migration/ubuntu/start-quest-adb.sh
电脑打开 http://localhost:8766/?source=quest_adb，选择「Quest 3 · USB ADB（21点）」
和实际手别，点击「开始识别」。Quest Browser 打开 http://localhost:8766/q/adb，
固定码 123456 连接后，点击「进入 XR 并开始采集」。
这里的 localhost 通过 adb reverse 转到 Ubuntu；USB 模式使用 http，不要沿用 https 收藏。
头显网页获取手部追踪，将 WebXR 25 关节映射为 21 点后经 USB 上传；ADB 本身不生成骨架。
无需原生 APK，也不需要电脑与头显在同一 Wi-Fi。断开 USB 后输入按 350 ms 超时失效。
如有多个 USB ADB 设备：
  HAND_QUEST_ADB_SERIAL=<Quest序列号> bash migration/ubuntu/start-quest-adb.sh
自定义端口用 HAND_QUEST_ADB_PORT，例如 9001，电脑和头显地址均同步改为 9001。
重插 USB 或重启头显后重新运行启动器建立转发。结束时先停止识别并退出服务；需要清理时：
  adb -s <Quest序列号> reverse --remove tcp:8766
无硬件软件检查：bash migration/ubuntu/start-quest-adb.sh --check（不操作 ADB）。
数据接口、21 点顺序和排错见 docs/online_webxr_quest3.md 的 USB ADB 一节。

3.3 Leap Motion 2：USB 连接
首次使用先完成第五节的 Linux 服务和 track 环境 Python binding 安装。
将 Leap Motion 2 直接接到运行工作台的 Ubuntu 电脑 USB 3.0 接口；
无需设置有线 IPv4、网关或手套地址。若浏览器在另一台电脑上，设备仍应接在
运行 Python 服务的 Ubuntu 主机上。

每次使用可先检查：
  systemctl status ultraleap-hand-tracking-service.service --no-pager
  leapctl devices
  lsusb -t
服务应为 active (running)，leapctl devices 应列出设备。
若服务未启动，运行：
  sudo systemctl start ultraleap-hand-tracking-service.service
若未列出设备，检查 USB 数据线、接口、服务状态和首次安装时的 EULA 接受情况。

在项目根目录运行：
  bash migration/ubuntu/start-track.sh
电脑浏览器打开 https://127.0.0.1:8766/，选择模型、实际手别和
「Leap Motion 2」（leapmotion），点击「开始识别」。当前按头显模式（HMD）使用，
设备朝向前方手部操作区域；打开设备后自动请求 high_hand_fidelity，确认骨架与模型更新。
摆放方向按现场设置，不自动沿用旧设备的 inverted 配置。
结束或切换输入源前点击「停止识别」；Ultraleap 系统服务可保持运行。

无硬件软件检查：
  bash migration/ubuntu/start-track.sh --check
  bash migration/ubuntu/start-play.sh --check
软件检查不验证真实 USB、骨架精度或机械手动作。

四、USB 机械手
原生 Ubuntu 直接插 USB，无需 usbipd-win、WSL 附加或 Windows 计划任务。
为当前用户配置串口权限：
  sudo usermod -aG dialout "$USER"
  sudo install -m 0644 migration/ubuntu/70-hand-h7.rules /etc/udev/rules.d/70-hand-h7.rules
  sudo udevadm control --reload-rules
注销并重新登录，重新插拔设备，然后检查：
  ls -l /dev/serial/by-id/ /dev/ttyACM*
规则对应现场 H7 VID:PID 0483:5740；其他设备按实际 ID/驱动调整。
串口号可能变化，在页面选择新机实际端口。上述检查不会发送运动指令。

五、Leap Motion 2（可选；使用 Leap 时安装）
安装提供的官方 Linux 服务：
  sudo apt install ./migration/ubuntu/ultraleap/tracking-software-linux-x64-6.2.0.deb
协议全文在同目录 installer-licenses.txt。阅读安装提示并按官方流程处理 EULA；
如需重新打开协议流程，运行 leapctl eula。本包不带原机服务配置和已接受状态。
登记 LeapC 动态库：
  echo /usr/lib/ultraleap-hand-tracking-service | sudo tee /etc/ld.so.conf.d/ultraleap.conf
  sudo ldconfig
使用原生 Ubuntu systemd 服务：
  sudo systemctl enable --now ultraleap-hand-tracking-service.service
  systemctl status ultraleap-hand-tracking-service.service --no-pager
为 track 环境安装 binding 和项目已有兼容修复：
  bash migration/ubuntu/install-leap-python.sh
启动器会设置当前环境的 LEAPSDK_INSTALL_LOCATION，不依赖旧用户名或旧环境。
安装完成后按第 3.3 节连接 USB、检查设备并在网页开始识别。

截至制作日，Ultraleap 官方 Linux 文档明确列出的系统仍为 Ubuntu 22.04；
附带 6.2.0 是项目现有 Ubuntu 24.04 开发环境使用的重装材料。
新机原生 Ubuntu 24.04 的持续 USB 采集、识别精度和机械手联动仍需实测。
官方说明：https://docs.ultraleap.com/linux/index.html
官方 binding：https://github.com/ultraleap/leapc-python-bindings
附带源码提交：2341c6c3db5dc08236bb52890d8c43a224139694。
项目已有诊断与历史故障记录见 docs/leapmotion2_input.md。

六、动作、日志和其他配置
新日志写入当前项目 logs/control_sessions；播放数据放在 runtime/ubuntu/play。
motion、网页配置与模型标定原样保留。
旧网页动作原件在 migration/ubuntu/preserved-runtime-data：
- motions.json：V2 左手，11 个动作。
- motions_a30_left.json：A3.0 左手 15 个动作原件。
- motions_a30_left_migrated.json：仅将旧型号名称 daxian_v3_0 改为
  TsenseI_v3_0 的可导入副本；姿态与时间参数原样保留，并通过当前读取校验。
文件未自动恢复到活动数据目录。选择对应模型和手别后，通过网页导入并先离线核对。

Quest HTTPS 证书在新机按新 IP 重新生成；不附旧私钥与信任设置。
网页 MediaPipe 在浏览器执行，Python 后端不需要旧 GUI 摄像头推理环境。
验收记录见 migration/ubuntu/VALIDATION.txt；软件验证不等同新机硬件验收。

GitHub 源码副本说明
- .venvs 和 runtime 为本机生成内容，不纳入 Git；按上述步骤重建环境与证书。
- migration/ubuntu/ultraleap/tracking-software-linux-x64-6.2.0.deb
  为约 917 MB 的离线安装包，未纳入普通 Git 上传；原本地文件保留。
  新机器需要另外获取对应的 Ultraleap Linux 安装包。
- FILES.sha256 是原迁移包快照校验清单，不是当前 Git 检出的完整校验清单。
