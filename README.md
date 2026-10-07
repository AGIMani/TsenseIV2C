# 达显身手 — 原生 Ubuntu 网页版

首先阅读 [Ubuntu 安装与启动说明](README-UBUNTU.txt)。此迁移副本面向 Ubuntu 24.04 LTS、x86_64、Python 3.12。

保留手势工作台、动作播放、网页标定、记录查看、模型与动作文件。动作播放的控制后台已从 Qt 窗口逻辑重写为纯 Python 会话、线程和事件循环。旧 Qt 桌面界面与 WSL/Windows 桥接不再包含。

## 安装和启动

按照安装说明准备系统依赖后：

    bash migration/ubuntu/install.sh all
    bash migration/ubuntu/start-track.sh

手势工作台地址：https://127.0.0.1:8766/ 。

另一个终端可运行：

    bash migration/ubuntu/start-play.sh

动作播放地址：http://127.0.0.1:8765/ 。分别选择新机的实际设备和手别。

Wuji、Leap Motion、USB 权限、A3.2 驱动重建和旧动作导入步骤都在安装说明中。环境、历史运行日志和旧部署凭据不包含在包内。

## 主要目录

- online_control/online_motion_track：手势工作台、标定及浏览器输入。
- online_control/online_motion_play：动作管理、原生 USB 和浏览器串口播放。
- src、TsenseI_retargeting：协议、驱动、运动学和重定向核心。
- assets、profiles、configs、motion：模型、标定、配置和动作。
- migration/ubuntu：安装/启动入口、重装材料、保留数据和验证记录。

[验证记录](migration/ubuntu/VALIDATION.txt) 说明重构范围与软件检查结果。真实设备在新机上的驱动、USB 采集及动作联动需要现场验收。
历史专题文档位于 docs，其中旧 Qt/Windows/WSL/node0 的操作步骤不适用于此迁移包。
