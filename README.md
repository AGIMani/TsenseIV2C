# 达显身手 — 原生 Ubuntu 网页版

首先阅读 [Ubuntu 安装与启动说明](README-UBUNTU.txt)。此迁移副本面向 Ubuntu 24.04 LTS、x86_64、Python 3.12。

网页手势工作台、动作播放和标定仅支持 A3.2 with ID，左右手均可选择。手势页面提供 Mink 自碰撞避让与关闭保护两种模式；动作播放页面不提供碰撞避让。动作播放的控制后台为纯 Python 会话、线程和事件循环，不依赖 Qt。

## 安装和启动

按照安装说明准备系统依赖后：

    bash migration/ubuntu/install.sh all
    bash migration/ubuntu/start-track.sh

手势工作台地址：https://127.0.0.1:8766/ 。

另一个终端可运行：

    bash migration/ubuntu/start-play.sh

动作播放地址：http://127.0.0.1:8765/ 。分别选择新机的实际设备和手别。

Wuji、Leap Motion、USB 权限和 A3.2 驱动重建步骤都在安装说明中。环境、历史运行日志和旧部署凭据不包含在包内。

## 主要目录

- online_control/online_motion_track：手势工作台、标定及浏览器输入。
- online_control/online_motion_play：动作管理和本机原生 SDK 播放。
- src、TsenseI_retargeting：协议、驱动、运动学和重定向核心。
- assets、profiles、configs、motion：模型、标定、配置和动作。
- migration/ubuntu：安装/启动入口、重装材料、保留数据和验证记录。

[验证记录](migration/ubuntu/VALIDATION.txt) 说明重构范围与软件检查结果。真实设备在新机上的驱动、USB 采集及动作联动需要现场验收。
历史专题文档位于 docs，其中旧 Qt/Windows/WSL/node0 的操作步骤不适用于此迁移包。
