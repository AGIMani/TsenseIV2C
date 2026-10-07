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
停止当前识别后切换输入源，再点击「开始识别」，无需更换命令或重启服务。
Wuji 手套需要指定地址时：
  WUJI_DEVICE_ADDRESS=192.168.1.100:50001 bash migration/ubuntu/start-track.sh
地址是原现场设备示例，按实际设置；不设地址时使用 SDK 默认发现。
手套网卡应配置在设备网段，使用原生以太网，不需要 Windows UDP 转发。
start-wuji.sh 和 start-quest.sh 保留为兼容入口，平时只需 start-track.sh。
终端保持开启，Ctrl+C 退出。
网页不会自动连接、使能或驱动机械手。

Quest 3 网页手势输入也使用普通命令：
  bash migration/ubuntu/start-track.sh
工作台默认在 8766 提供 HTTPS，Quest、Wuji、Leap Motion 共用该服务。
自动选择本机物理 Wi-Fi/以太网地址，生成含当前 IP 的证书；VPN 网卡不作为采集地址。
多网卡时可显式指定：
  HAND_QUEST_ADDRESS=<电脑局域网IP> bash migration/ubuntu/start-track.sh
电脑打开 https://127.0.0.1:8766/?source=webxr，选择 Quest 并点击「开始识别」。
Quest 与电脑连接同一局域网，在 Quest Browser 打开网页显示的 https://<电脑IP>:8766/q。
不能在头显使用 127.0.0.1；该地址指向头显自身。没有可用局域网地址时仍可使用本机工作台；联网后重新启动即可开放 Quest 访问。
首次需让访问端信任 runtime/ubuntu/quest-tls/quest_webxr_ca.crt；私钥不要传到设备。
WebXR 需要安全上下文；若出现证书错误或无法进入 XR，应先完成证书信任。
若网址仍超时，检查 Wi-Fi 客户端隔离和 Ubuntu 防火墙的局域网 TCP 8766 访问规则。

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
设备直接插 USB 3.0，检查设备行实际协商速度：
  lsusb -t
  leapctl devices
  bash migration/ubuntu/start-track.sh
在网页选择 Leap Motion 2 和实际手别，保持手完整位于相机视野。
摆放方向按新现场设置，不自动沿用原机 inverted 配置。

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
