# Wuji CLI 安装、标定与现场网络验收

2026-09-30 在 WSL `Hand-Ubuntu-24.04` 中安装官方 CLI `2026.9.22`，路径 `/home/onkiall/.local/bin/wuji`。官方安装脚本校验了 GitHub 发布包 SHA-256；没有更改项目 Python 依赖。项目 SDK 同为 `2026.9.22`。

## 开始标定

Windows 双击 `E:\WSL\start-wuji-calibration.cmd`。该入口仅启动 WSL 内的 CLI，选用已创建的 `onkiall` 用户，并按已验证的地址直连左手套。请先关闭正在采集手套的工作台或其他 SDK 程序，把终端放大到至少 81 列、38 行，再按提示完成动作。Ctrl+C 取消。

也可以双击 `E:\WSL\open-wuji-cli.cmd`，在 Linux 终端执行：

```bash
export WUJI_TELEMETRY_DISABLE=1 WUJI_NO_UPDATE_CHECK=1
wuji user switch onkiall
wuji calib hand-model --address 192.168.1.100:50001
wuji user show
```

官方流程包含拇指与食指、中指、无名指、小指依次捏合，四指弯曲 90 度，以及平展手掌。标定结果保存在当前 Linux 用户的 `~/.wuji/sdk/` 数据中，由同一 Linux 用户运行的 CLI 和 SDK 共用。完成标定后重启手套采集程序以加载模型。默认 `Default` 用户不能标定。

本次仅完成安装、用户创建和连接验证，**尚未完成佩戴者动作标定**，当前左右手模型均显示未标定。后续如已有模型，交互标定会确认是否替换。

## 本次现场结果

- 以太网 Realtek PCIe GbE Family Controller 链路为 100 Mbps，原有地址为 `192.168.88.11/24`，缺少手套子网。
- 已添加持久地址 `192.168.1.50/24`，保留原有地址、网关与 WLAN 配置。Windows 和 WSL 均可到达 `192.168.1.100`。
- 修复地址后仍有 UDP 握手超时；用户将手套采集板重新上电后恢复。设备会话或固件的内部原因尚未确定，不能仅据此断言故障来源。
- CLI 完整握手识别左手 `WG1JA06260713017`，固件 `0.11.1`，端口 `50001/UDP`。
- 连续订阅 240 帧真实 `hand_skeleton`，每帧 21 点、坐标有限、`l_wrist` 坐标系；序号递增且无缺号，设备时间戳频率约 120 Hz。该短测不代表长期稳定性或标定精度验收。
- Windows 三个防火墙配置均保持开启，入站默认阻止、出站允许。限定到手套 IP 的临时 UDP 放行未解决超时，已删除，无需关闭防火墙。
- 使用系统 NAT 直连；未安装 Windows SDK、UDP 转发器或更改项目业务代码，也未连接或使能机械手。

Windows 环境脚本、网络配置备份和验收 JSON 位于 `E:\WSL\wuji-setup\`。原生 Ubuntu 迁移后直接配置以太网同网段并使用 Linux CLI，无需这些 Windows 环境脚本。

WSL NAT 环境请优先使用上述显式 `--address`；广播扫描为空不能代替直连握手判断。

## 复查

```bash
wuji --version
wuji ping --address 192.168.1.100:50001 --json
wuji sub hand_skeleton --address 192.168.1.100:50001 --count 10 --jsonl
```

官方参考：[安装](https://docs.wuji.tech/docs/en/wuji-cli/latest/getting-started/)、[手模型标定](https://docs.wuji.tech/docs/en/wuji-cli/latest/hand-model-calibration/)。
