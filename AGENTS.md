# Agent instructions

本副本用于原生 Ubuntu 24.04 LTS，项目根目录是本文件所在目录。
Git、Python、Node、安装、构建和测试命令直接在当前 Linux 项目目录执行。
不要硬编码原开发机用户名、Windows 路径或 WSL 发行版。
依赖与启动步骤见 README-UBUNTU.txt 和 migration/ubuntu。
USB 使用 Linux 原生设备、udev 与 dialout 权限；手套使用 Linux SDK 直接联网。
不要引入 usbipd-win、Windows UDP/HTTP 转发或 WSL 专用启动依赖。
只维护网页功能；后台已经去除 Qt 依赖，保持无 Qt 环境下的全部网页回归测试通过。
保持 track、play 两套 Python 环境隔离。
