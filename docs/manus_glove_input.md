# Manus 手套：客户包的 Linux SDK 接入

本次只从 `TsenseIV2C-source-20261116.zip` 提取 Manus 接入及所需依赖，使用客户提供的
Manus SDK **3.1.1** 和 Python 绑定。网页来源为 `manus_glove`，兼容客户入口 `?source=manus`。
客户包内的其他功能、机器人资产、通用依赖清单、Docker、示例程序和个人 `.mcal` 文件没有导入。
之前准备的独立 3.0.1 采集实现已移出项目，运行时只有这一套 Manus SDK。

## 安装和启动

在项目根目录执行，Python 依赖只安装到 **track** 环境：

```bash
.venvs/track/bin/python -m pip install -r requirements-manus.txt
bash scripts/build_manus.sh
bash migration/ubuntu/start-track.sh --manus-mode integrated
```

打开 `https://127.0.0.1:8766/?source=manus`，选择模型和输入手别，再点击开始识别。
服务和页面启动时不连接手套，开始输入会话后才加载 SDK。机械手连接和实时下发仍由用户
在网页上单独开启。动态标定入口为 `/calibration?source=manus`。

`requirements-manus.txt` 仅新增 `pybind11==2.13.6`，它用于编译绑定；NumPy 使用现有 track
依赖。play 环境和原有 Python、Node 依赖清单均不需要改动。

Ubuntu 24.04 缺少系统构建/运行依赖时，可按需安装：

```bash
sudo apt-get install build-essential cmake python3.12-dev libudev1 libusb-1.0-0 libzmq5
```

客户库仅提供 Linux x86_64。绑定必须使用运行网页服务的同一 Python 次版本构建；使用自定义
解释器时运行 `bash scripts/build_manus.sh /absolute/path/to/python`，并用该解释器启动服务。
扩展默认位于 `vendor/manussdk/build/python`，通过相对 RPATH 加载项目中的厂商库和配套
gRPC/protobuf 运行库，不要求设置全局 `LD_LIBRARY_PATH`。

大型 `.so` 本机已提取，构建输出和厂商运行库不进入普通 Git。新检出项目可从客户包恢复，
恢复工具只读取固定白名单成员，并逐个核验 SHA-256，不执行包内脚本：

```bash
python3 tools/install_manus_runtime.py /path/to/TsenseIV2C-source-20261116.zip
python3 tools/install_manus_runtime.py --check
bash scripts/build_manus.sh
```

网页部署包同样包含 Manus 源码和构建材料，默认不打入大型厂商库及本机编译产物。
在部署包的 `app/` 目录执行上述恢复命令，用部署服务的 Python 安装
`requirements-manus.txt` 并作为参数传给 `scripts/build_manus.sh`，即可保持解释器一致。

若普通用户没有接收器或授权设备访问权限，可安装项目提供的规则，再重新插拔设备：

```bash
sudo install -m 644 migration/ubuntu/70-manus-hid.rules /etc/udev/rules.d/70-manus-hid.rules
sudo udevadm control --reload-rules
```

规则使用 `uaccess` 和 `dialout` 组权限。安装/恢复脚本不改系统权限，不启动手套或机器人。
Integrated 模式仍需相应的 SDK Integrated 授权；版本和授权要求以所用设备及厂商许可为准。
参见[官方 Linux 指南](https://docs.manus-meta.com/3.0.0/Plugins/SDK/Linux/)。

## 客户接口与数据流程

| 服务参数 | 默认值 | 用途 |
| --- | --- | --- |
| `--manus-mode` | `integrated` | 原生接收器；可选 `local` 或 `remote` Core 模式 |
| `--manus-module-dir` | `vendor/manussdk/build/python` | 自定义绑定目录 |
| `--manus-left-calibration` | 不加载文件 | 用户指定的左手 `.mcal` |
| `--manus-right-calibration` | 不加载文件 | 用户指定的右手 `.mcal` |
| `--manus-connect-timeout` | `10` | 连接超时，1–60 秒 |

默认 Integrated 直接在 Ubuntu 使用 SDK，不引入 Windows、WSL 或转发服务。
`local`、`remote` 是客户保留的可选 Core 连接模式，不是启动工作台的依赖。
设备选择沿用客户绑定：每侧使用 SDK 设备列表中的第一只手套，当前没有按 glove ID 选择的参数。
未指定标定文件时使用 SDK 当前标定；不会自动加载客户个人标定。手套 `.mcal` 与本项目
机器人关键点 YAML 是两种不同的标定，只有明确传入参数才向 SDK 加载 `.mcal`。

SDK 请求米制世界坐标，坐标系为右手系、Z 向上、X 视线方向。按节点 ID、链类型和
关节语义选取腕部与五指，减去腕部位置，映射成现有求解器的21点。拇指保留真实关节，
不复用 Wuji 的拇指中点补偿；SDK 未提供的追踪质量不伪造为置信度。

左右手和浏览器会话共享一个 SDK 连接，各自维护消费状态。原生回调记录主机单调时钟，
轮询不会刷新数据年龄；超过 **200 ms** 的帧不提交新目标。手别、会话、重连代次、页面心跳
以及求解完成时的数据年龄均参与有效性检查。浏览器只引用服务端帧 revision，上传自制点位
不会替代 Manus 数据。最后一个会话退出后停止采集线程并关闭 SDK；底层厂商 SDK 的关闭
调用如果阻塞，网页会先停止使用它的数据，同进程不会并发启动第二个 SDK 所有者。

录制保留 `raw_manus`、N×10 原始骨架、节点映射、主机接收时间和来源元数据。
`source_timestamp_raw` 是 SDK 的不透明64位时间值，以十进制字符串保存，不能当作 Unix
时间或微秒时间使用。真实设备的坐标方向、手套标定和机器人跟随效果仍需实机验证。

## 提取范围与来源

- 接入：客户 `manus_source.py`、`manus_input.py`、Manus 所需的 `metric_input.py`，以及当前
  网页服务、来源选择、动态标定和录制中的 Manus 接点；共享 relay 只用于 Manus。
- 绑定：`vendor/manussdk/bindings/python` 的源代码、构建配置及离线测试。
- SDK：三份头文件、一份厂商 `.so`、四份配套运行库及 SONAME 链接、厂商许可证。
- 构建：`scripts/build_manus.sh`、单独的 `requirements-manus.txt`。
- 未导入：客户其他模型、驱动改动、控制功能、全量 requirements、虚拟环境、容器、示例和
  个人标定；本次之前已完成的 A3.2 with ID 更新保留。

客户来源信息见 [UPSTREAM.md](../vendor/manussdk/UPSTREAM.md)，运行库白名单及校验值见
[runtime-manifest.json](../vendor/manussdk/runtime-manifest.json)。客户 ZIP SHA-256：
`ca2df7abde488dfb20b4eb7073226f890a09ffb89f82cea48b5ce911c3daced7`。
压缩包中的说明和历史验证记录只用作参考，不代表本项目已完成同样验证。

## 当前项目验证

- Manus 后端、网页 API、模拟 A3.2 控制、运行库恢复及部署白名单共 60 项 Python 测试通过。
- 现有网页、Wuji、Leap、Quest、RealSense 等相关 Python 回归 354 项通过、2 项跳过。
- track 前端 220 项测试通过。真实 Chrome 配合模拟 SDK 跑通左右手、200 ms 过期恢复、
  网络中断重试、停止/切源、动态标定与冻结，无 JavaScript 错误。
- 服务 `--check`、Manus 参数解析及运行库 SHA-256 校验通过。
- track Python 3.12 原生绑定编译、2 项 CTest、实际扩展导入及移动目录后的导入通过；
  动态库依赖完整，RPATH 使用相对路径。
- 未进行真实 Manus 手套或实体机械手动作验证。

复跑 Manus 离线测试：

```bash
.venvs/track/bin/python -m pytest -q tests/test_manus_*.py
node --test online_control/online_motion_track/tests/manus-receiver.test.mjs
HAND_PYTHON="$PWD/.venvs/track/bin/python" node online_control/online_motion_track/tests/manus-browser.mjs
```
