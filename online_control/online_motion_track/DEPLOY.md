# A3.2 with ID 手势网页部署

手势工作台仅支持 `TsenseI_v3_2_withid`，左右手均可选择。碰撞选项为 Mink 自碰撞避让和关闭检测；选择直控时固定关闭碰撞检测。Mink 需要完整依赖、模型和采样碰撞检查。动作播放独立运行，不提供碰撞避让。

## 本机运行

在 Ubuntu 项目根目录按 `README-UBUNTU.txt` 安装，分别使用 `.venvs/track` 和 `.venvs/play`。手势服务使用：

```bash
bash migration/ubuntu/start-track.sh
```

浏览器访问 `https://127.0.0.1:8766/`。原生 SDK、USB 权限及输入设备说明见根目录安装文档。控制机械手需要在运行 Python 服务的本机页面连接 SDK；不需要 Qt 或浏览器串口桥接。

## 生成部署包

先安装手势网页的浏览器依赖，然后在项目根目录运行：

```bash
.venvs/track/bin/python online_control/online_motion_track/package.py --stage /tmp/tsensei-hand-full --full
```

`--full` 包含 Mink、采样碰撞检查所需代码以及 A3.2 with ID 左右手模型。新建目标目录后，在部署环境安装包内 `requirements-full.txt`，并运行：

```bash
python server.py --check --collision-mode mink
```

不带 `--full` 的轻量包仅支持关闭碰撞检测，适用于无需避碰的预览；其中不包含完整的碰撞检查模块。两个包都不包含旧型号的网页控制实现和模型资源。

## 服务器部署

`deploy.sh` 提供 systemd 和可选 Caddy 配置，需要目标 Ubuntu 主机、DNS 和管理员权限。要部署包含 Mink 的完整包，在源码目录或从完整包运行：

```bash
sudo bash deploy.sh --domain hand.example.com --full
```

使用现有 HTTPS 代理时可附加 `--no-caddy`，并将请求代理到本机服务端口。脚本在切换服务前创建独立环境，安装依赖，验证左右手与 Mink，再检查健康状态。该脚本启用 `--public`：远程网页用于预览，服务端不提供实机 USB 控制或服务器端录制。需要本机 SDK 控制时使用上面的本机运行方式。

部署失败时查看 `systemctl status tsensei-hand-web` 和 `journalctl -u tsensei-hand-web`。这里的发布流程没有连接实体手或验证真实设备运动。
