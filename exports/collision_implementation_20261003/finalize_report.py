"""Publish measured final-geometry benchmark results into the technical note."""
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
data = json.loads((HERE / 'runtime_benchmark.json').read_text())
doc = ROOT / 'docs/collision_outer_envelopes.md'
text = doc.read_text().split('\n## 性能结果\n')[0]
rows = [
    '', '## 性能结果', '',
    'WSL、Python 3.12.3、MuJoCo 3.10.0；每个型号/手侧 56 个连续目标，优化版和参考版交错运行三轮，各 168 次投影。目标包含张手、弯曲、对捏、交叉、释放及一个记录姿态的往返。只测碰撞投影，不包含 retarget、相机、通信和渲染。', '',
    '参考版使用相同最终几何、间距、QP、路径检查与分离证书，保留失败早退；关闭重复查询缓存和几何筛选，Stop 使用原完整 mj_forward。下表隔离的是查询优化收益，不能当作相对原胶囊系统的整体收益。', '',
    '| 型号 / 手侧 | Mink 参考 p95 ms | Mink 优化 p95 ms | 减时 | Stop 参考 p95 ms | Stop 优化 p95 ms |',
    '| --- | ---: | ---: | ---: | ---: | ---: |',
]
for name, entry in data['models'].items():
    m, s = entry['mink'], entry['stop']
    assert m['max_output_difference_rad'] == s['max_output_difference_rad'] == 0
    old, new = m['reference']['p95_ms'], m['optimized']['p95_ms']
    label = name.replace('TsenseI_v3_', 'A3.').replace('_withid', ' with-ID').replace('/left', ' / 左').replace('/right', ' / 右')
    rows.append(f"| {label} | {old:.2f} | {new:.2f} | {(1-new/old)*100:.1f}% | {s['reference']['p95_ms']:.2f} | {s['optimized']['p95_ms']:.2f} |")
    print(f'{label}: Mink {old:.2f}->{new:.2f} ms, Stop {s["reference"]["p95_ms"]:.2f}->{s["optimized"]["p95_ms"]:.2f} ms')
rows += [
    '', '八组模型的优化版和参考版输出逐帧完全一致（最大角度差 0 rad）。`distance_queries` 只统计本模块安全/诊断查询，不包含 Mink 内部 QP 组装调用。', '',
    '**精度成本：** 更完整的外壳、掌部覆盖和采样间隔检查改变了允许的运动，不能保证所有动作比原胶囊更快。下列旧系统数据仅各测一轮，作为量级参考，不用于输出等价或严格统计推断。', '',
    '| 型号 / 手侧 | 原系统 p95 ms | 最终 Mink p95 ms |', '| --- | ---: | ---: |',
]
for name, entry in data['models'].items():
    if 'p95_ms' not in entry.get('original_mink_old_geometry', {}):
        continue
    label = name.replace('TsenseI_v3_', 'A3.').replace('_withid', ' with-ID').replace('/left', ' / 左').replace('/right', ' / 右')
    rows.append(f'| {label} | {entry["original_mink_old_geometry"]["p95_ms"]:.2f} | {entry["mink"]["optimized"]["p95_ms"]:.2f} |')
rows += [
    '', '原始结果：`exports/collision_implementation_20261003/runtime_benchmark.json`。复现同几何对照：', '',
    '```bash', 'PYTHONPATH=src:. python tools/benchmark_collision_runtime.py --repeats 3 --output exports/collision_runtime_check.json', '```', '',
    '## 验证与边界', '',
    '- 覆盖检查包含所有 174 个 URDF 碰撞链接的 STL 往返、源顶点包含、最大外扩、凸性、水密性、镜像和重新生成一致性。',
    '- 回归覆盖八套初姿态、掌部真实接触截停、指尖跨采样点穿透、随机目标/路径、分指保持恢复、缓存与原碰撞流程对照、部署依赖、off/monitor 手动下发及心跳/故障停发。',
    '- 浏览器使用内存串口和禁用硬件连接的临时服务。WSL 软件 WebGL 会阻塞页面导致原看门狗超时；关闭测试副本中的 GPU draw 后，真实 DOM、API、数据流与模拟下发通过完整生命周期测试，产品渲染代码和超时阈值未放宽。记录见 `exports/collision_design_20261003/browser_collision_off/smoke_summary.json`。',
    '- 扩展检查发现原有的 with-ID 关键点镜像断言、缺失外部 SDK 源目录、V2 旧速度上限断言问题；已在保存的原始资产上确认关键点镜像失败，本轮未改相关标定或 V2 参数，也未宣称全仓测试全绿。',
    '- 未连接实体手、未执行实体电机运动。', '',
]
doc.write_text(text + '\n'.join(rows))
