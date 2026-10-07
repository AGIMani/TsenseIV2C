# 角度转行程输入与边界验证（2026-09-18）

结论：当前项目默认几何及默认查表模式下，15万组整手姿态和1,040,644个单通道采样点
全部通过；有效范围内未观察到异常、非有限输出或行程越界，V2没有触发失败保持。
159项回归测试和376个子测试通过。所有测试离线进行，没有连接或驱动实机。

## 覆盖范围

| 路径 | 整手姿态 | 密集单通道采样 | 结果 |
|---|---:|---:|---|
| V2 完整转换及float32报文打包 | 50,000 | 57,484 | 0异常、0越界、0非有限值、0失败保持 |
| A3.2 左手查表 | 50,000 | 491,580 | 0异常、0越界、0非有限值 |
| A3.2 右手查表 | 50,000 | 491,580 | 0异常、0越界、0非有限值 |

固定随机种子32020918，整手样本每十组有一组专门组合各关节上下端点。
V2每个直线通道扫描4,097个均匀角度，并增加机构上下限、软饱和起点及相邻浮点数。
A3.2每个通道扫描8,193个角度及两侧相邻浮点数，与精确模式逐点对照；另对随机整手姿态抽样交叉验证。
两个转换器的各通道独立计算，密集扫描覆盖各自一维范围，并非穷举20/21维所有姿态。

- V2 行程正解回算角度与原软限位目标最大差值：**0.00043834°**。
- A3.2 查表与精确模式最大行程差值：**0.0000006870 mm**，低于1e-6 mm插值容差。
- 检查了输出长度、有限性、行程边界、单调性、默认查表模式和float32可打包性。

单元测试还覆盖：零位、负角、超限、极限左右两侧、NaN/Inf、±1e308、超大整数、
None/字符串/列表/复数/布尔值、非映射输入、缺失/未知关节、左右手/版本隔离、
参数修改、缓存失效、精确回退、失败保持和出错后恢复。

## 本轮发现并修复的问题

1. V2遇到不合法关节值后，错误日志再次调用float而抛异常；日志现保留原值，
   单指计算失败时正常保持该指上次有效行程，其余手指继续计算。
2. 极大有限角度在缓存量化乘法中溢出，导致round异常；超出量化范围时使用原浮点值作缓存键。
   NaN/Inf不再使用可能与合法有限角度碰撞的有限数哨兵。
3. V2旋转输入NaN/Inf可能进入输出，极大值还可能使float32打包溢出；现在在转换阶段拒绝，
   直线输出也检查float32可表示范围。`send_joint_angles` 返回False，并在任何设备写入前结束。
4. 行程缩放乘法溢出可能返回Inf；现在保持该关节上次有限行程（首次无历史时为0）。
5. A3.2非映射输入或超大整数可能产生未处理异常；统一转成明确的验证错误，
   `preview` 返回 `valid=false`、错误原因和空输出。混合类型的未知键也不会再使错误格式化失败。

## 输入处理约定

| 输入情况 | V2 | A3.2 |
|---|---|---|
| 当前机构有效范围内的完整角度 | 正常转换 | 正常转换 |
| 有限但超限的直线角度 | 原软限位/钳位行为 | convert严格报错；preview限幅并记录 |
| 缺失关节 | 保留原部分姿态接口：缺失补0 | 必须完整20关节，缺失则拒绝 |
| 直线角度NaN/Inf或换算溢出 | 保留原零位处理 | 明确拒绝 |
| 错误类型 | 直接行程mapper按指保持；整手发送入口拒绝 | 明确拒绝 |
| 旋转NaN/Inf、无法表示的报文数值 | 拒绝该帧，不发送 | 非有限输入拒绝；有限超限按接口模式处理 |

非法输入不会被伪装为成功转换。这里的“0失败”指报告中有效输入与默认几何的测试范围，
不代表任意损坏几何配置、任意未知数据或固件/机械动作都得到保证。
A3.2机构几何结果仍与V2独立，不使用V2下发协议。

## 复现与证据

```powershell
E:/conda/envs/hand_dex/python.exe tools/validate_hand_conversion.py --output docs/validation/hand_conversion_20260918.json
E:/conda/envs/hand_dex/python.exe -m pytest tests/test_conversion_inputs.py tests/test_prepared_linkage.py tests/test_finger_linkage.py tests/test_a32_fast.py tests/test_a32_mapper.py tests/test_web_v3_2.py tests/test_v3_2_recording.py tests/test_light_test_v3_2.py tests/test_web_v2.py tests/test_web_v2_control_timing.py tests/test_browser_serial_api.py tests/test_control_chain_recording.py tests/test_usb_v3_protocol_sync.py -q --basetemp=.pytest-runtime/conversion_validation_new_run
```

- 压测原始数据及被测文件SHA256：`docs/validation/hand_conversion_20260918.json`。
- 修复后性能：`docs/benchmarks/hand_conversion_validated_py311_20260918.json`。
  V2完整转换平均44.5 μs（中位40.4 μs）；A3.2查表完整转换平均63.5 μs（中位57.4 μs）。
  每项预热后测10,000次，系统负载会影响绝对时间。
