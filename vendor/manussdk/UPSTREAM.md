# SDK 来源与本地适配

此目录从用户提供的 `TsenseIV2C-source-20261116.zip` 中 `TsenseIV2C-source/vendor/manussdk` 精简提取。客户包的原说明将 Python 绑定及打包归属于 Xense 维护的 `lerobot-xensehand/third_party/manussdk`，并记录子仓库提交 `8a5ef6b1f370ad369643f54084aac8fba86955b6`、厂商运行库 Manus 3.1.1。该上游提交来自客户声明，本次未另行取得或核验其 Git 仓库。

保留三个厂商头文件、原始 [Manus 许可证](license/license.txt)、Python 绑定和必要构建/测试文件。实际链接 `vendor/manus/lib/libManusSDK.so`，它同时提供 Core 和 Integrated 接口；仅随带其所需的四个 gRPC/protobuf 运行库与四个 SONAME 相对链接。客户包中的独立 `libManusSDK_Integrated.so`、Docker、示例、个人标定文件和已有构建缓存未导入。运行库内容及客户归属由 `runtime-manifest.json` 记录；从原包恢复时使用仓库提供的导入工具核对哈希。

本地适配保留客户 pybind 接口，并增加：与骨架行原子对应的 `node_ids`、Integrated 模式的空 host 连接、进程内单实例保护、连接/关闭串行化、非阻塞协作取消、失败初始化清理以及离线回归。`source_timestamp_raw` 仍为不透明的 Manus 压缩时间戳，不转换为微秒或纳秒。Python 输入层按同一手套递增的 SDK 回调序号接收新帧，以回调到达时的 Linux `CLOCK_MONOTONIC` 时间计算帧龄。

构建入口为仓库根目录的 `scripts/build_manus.sh`，只为传入的 Python 环境构建扩展；默认使用 track 环境。构建后的 RPATH 仅包含相对 `$ORIGIN` 路径，不依赖原开发机目录。离线测试不调用真实 SDK 初始化或探测设备。

`request_disconnect()` 能取消绑定自身的重试/诊断等待；厂商初始化、连接或关闭函数内部阻塞时，当前客户线程架构不能强行中断它。网页停止会立即撤销输入，SDK owner 会保留到工作线程实际完成关闭，避免并发初始化。
