原生 Ubuntu 安装与启动（Ubuntu 24.04 系列 / x86_64 / Python 3.12）

以下命令在解压后的 hand 项目根目录执行。不要用 sudo 运行安装或启动脚本。
安装会联网下载依赖；压缩包不含旧 Python 环境，已包含 three / MediaPipe 的前端生产资源。

1. 查看系统依赖命令（脚本只显示 apt 命令，不自动安装系统包）：
   bash migration/ubuntu/install.sh --help
   按提示自行安装缺失系统依赖后，选择一个或全部环境：
   bash migration/ubuntu/install.sh all
   # 也可以选择 track 或 play

2. 启动（每个服务各开一个终端；Ctrl+C 停止）：
   bash migration/ubuntu/start-track.sh
   浏览器打开 https://127.0.0.1:8766
   bash migration/ubuntu/start-play.sh
   浏览器打开 http://127.0.0.1:8765

3. Wuji 手套由 Ubuntu 本机 SDK 直接连接：
   bash migration/ubuntu/start-track.sh
   不指定地址时使用 SDK 默认发现；也可以按实际设备设置地址：
   WUJI_DEVICE_ADDRESS=192.168.1.50:端口 bash migration/ubuntu/start-track.sh
   地址示例需换成设备的实际 IPv4:port，不沿用 WSL/Windows 桥接地址。
   可选 WUJI_DEVICE_SERIAL 限定设备序列号。启动器关闭 Wuji 遥测。
   Quest、Wuji、Leap Motion 在同一工作台选择；停止识别后切换，无需更换启动命令。
   本机网卡按设备要求配置 IP/网段；不需要 Windows、WSL 网络桥或 USB 转发。

4. 安装后可先做离线检查（不会连接手套/串口或启动服务）：
   bash migration/ubuntu/start-track.sh --check
   bash migration/ubuntu/start-play.sh --check

环境和数据位置：
- 默认项目/.venvs/track、.venvs/play，彼此独立。
- HAND_VENV_ROOT=/自选目录 可覆盖环境根目录；安装与启动要设置相同值。
  相对 HAND_VENV_ROOT 路径按项目根目录解释。
- 重新移动项目后应重建虚拟环境；venv 不是可迁移的文件格式。
- track 日志：项目/logs/control_sessions。
- play 配置：migration/ubuntu/play.json；新数据：项目/runtime/ubuntu/play。
  play.json 的相对路径按项目根目录解释。
- 2 份 requirements 从本机已经安装的 2 个环境 pip freeze 导出。
  track 的 leap/leapc_cffi 本地文件依赖另行安装，见迁移包的 Leap 说明。
- play 网页后台已重写为纯 Python，会话与控制线程不依赖 Qt/PySide6。
- play 的 Ruckig 0.17.3 通过 build-constraints.txt 约束构建依赖。
- 安装 track/all 会执行 npm ci --omit=dev --ignore-scripts 恢复前端资源。
  当前生产依赖无 Node engines 限制；Ubuntu 24.04 自带 Node >=18 / npm >=9
  满足运行资源安装。开发测试中的 Playwright 需要 Node >=20，未在此安装。
- 启动器接受服务器原有参数，例如 --port 8767；track 默认使用局域网 HTTPS，电脑控制页使用本机回环地址；play 仍仅绑定本机。
- USB 设备直接接 Ubuntu；串口权限通常由 dialout 组提供，HID/USB 按需配置
  设备厂商 udev 规则。此包不会自动修改权限、安装驱动或启动硬件。
