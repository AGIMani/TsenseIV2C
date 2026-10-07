"""Render an aggregated flame graph from real VizTracer call stacks (no synthetic timings)."""
import argparse
import html
import json
from pathlib import Path


def flame(trace):
    events = [e for e in json.loads(trace.read_text(encoding='utf-8'))['traceEvents']
              if e.get('ph') == 'X' and e.get('dur', 0) > 0]
    roots = sorted([e for e in events if e['name'].startswith('WebSession.process (')], key=lambda e:e['ts'])[10:]
    if not roots:
        raise ValueError('No steady WebSession.process call stacks found')
    tid, lo, hi = roots[0]['tid'], roots[0]['ts'], roots[-1]['ts']+roots[-1]['dur']
    events = sorted([e for e in events if e['tid']==tid and lo<=e['ts']<=hi], key=lambda e:(e['ts'],-e['dur']))
    root = {'name':'WebSession.process (steady frames)', 'value':0., 'children':{}}
    stack=[]
    for e in events:
        while stack and e['ts'] >= stack[-1][0] - .00001:
            stack.pop()
        if e['name'].startswith('WebSession.process ('):
            root['value']+=e['dur']; stack=[(e['ts']+e['dur'],root)]
        elif stack:
            parent=stack[-1][1]
            node=parent['children'].setdefault(e['name'], {'name':e['name'],'value':0.,'children':{}})
            node['value']+=e['dur']; stack.append((e['ts']+e['dur'],node))
    parts=['<svg xmlns="http://www.w3.org/2000/svg" width="1440" height="620" viewBox="0 0 1440 620">',
           '<rect width="1440" height="620" fill="#f6f9fa"/>',
           '<text x="20" y="30" font-family="sans-serif" font-size="20">A3.0 server replay — measured call-stack flame graph</text>',
           '<text x="20" y="55" font-family="sans-serif" font-size="13">Width = cumulative traced time; hover for function/path. Warmup and setup excluded. Not end-to-end latency.</text>']
    def draw(node,x,depth):
        width=node['value']/root['value']*1400
        if width<3 or depth>17:return
        y=580-depth*28
        color=f'hsl({22+depth*7},75%,{66-depth%3*5}%)'
        name=node['name']; label=name.split(' (')[0]
        title=html.escape(f"{name}\n{node['value']/1000:.2f} ms inclusive; {node['value']/root['value']*100:.2f}% of process")
        parts.append(f'<g><title>{title}</title><rect x="{x:.2f}" y="{y}" width="{max(0,width-1):.2f}" height="26" fill="{color}"/>')
        if width>42:
            label=label[:max(3,int(width/7)-1)]
            parts.append(f'<text x="{x+3:.2f}" y="{y+18}" font-family="monospace" font-size="11">{html.escape(label)}</text>')
        parts.append('</g>')
        at=x
        for child in sorted(node['children'].values(),key=lambda n:-n['value']):
            draw(child,at,depth+1); at+=child['value']/root['value']*1400
    draw(root,20,0); parts.append('</svg>')
    return ''.join(parts)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,default=Path('logs/retarget_profile'))
    args=parser.parse_args(); root=args.root
    svg=flame(root/'after/trace.json'); (root/'flamegraph.svg').write_text(svg,encoding='utf-8')
    before=json.loads((root/'before/metrics.json').read_text())
    after=json.loads((root/'after/metrics.json').read_text())
    blocks=[('访问者电脑 · 相机 RGB', '640×480；镜像只影响预览'),
            ('浏览器 Worker · MediaPipe GPU / CPU 回退', '输出 world[21×3]、normalized[21×3]、左右手标签'),
            ('按左右手选择 → /api/frame', 'world 用于空间求解；normalized 用于相机叠加并随包记录；一次只处理一笔识别＋请求'),
            ('服务器 · WebSession → One Euro', '时间戳／序号检查；21×3 有限值检查；world 和 normalized 分别滤波'),
            ('prepare_points → A3.0 TopoRetarget', '腕部坐标／左右手处理 → 掌宽归一化 → 标定 FK / Jacobian → 骨方向初始化＋DAQP SQP'),
            ('Mink / 碰撞停止 → 机构可达范围', '21 点在此已变为 20 个模型关节角 rad；保留 retarget 原始角与避碰后目标'),
            ('500 Hz 目标 · 20 DOF Ruckig / Time 同步', '速度上限不变；加速度 15 rad/s²；jerk 120 rad/s³；实际频率需测量'),
            ('采样碰撞检查 → 角度转行程／旋转角', '9 路直线 mm ＋ 11 路旋转 deg；四指 DIP 直接角度；协议还包含符号与减速比'),
            ('二进制 HTTP 流 → 浏览器 Web Serial → 下位机', '校验时效／范围／跳变／CRC；电机使能、找零和日志在访问者电脑'),
            ('实际遥测 → 浏览器 → 启动 rebase', '反馈超模型范围截断；用于启动与监控，不是服务器持续位置闭环')]
    flow=''.join(f'<div class="step"><b>{html.escape(a)}</b><p>{html.escape(b)}</p></div>' for a,b in blocks)
    rows=''.join(f'<tr><td>{m}</td><td>{before[m]["steady_total"]["median_ms"]:.2f}</td><td>{after[m]["steady_total"]["median_ms"]:.2f}</td></tr>' for m in ('off','mink'))
    page=f'''<!doctype html><meta charset="utf-8"><title>A3.0 延迟分析</title>
<style>body{{font:16px/1.65 system-ui;margin:40px auto;max-width:1200px;padding:0 24px;background:#f6f9fa;color:#20363d}}h1,h2{{color:#157888}}.step{{background:white;border-left:4px solid #24899b;padding:12px 20px;margin:12px 0}}p{{margin:4px 0}}table{{border-collapse:collapse}}td,th{{padding:10px 24px;border-bottom:1px solid #bcc}}img{{width:100%}}a{{color:#157888}}</style>
<h1>A3.0 · 21 点数据通路与延迟分析</h1><p>真实记录：2026-09-21 20:38；本机回放 150 个左手输入帧，前 10 帧不计入稳态耗时。未连接真机。</p>
<h2>21 点的去向</h2>{flow}
<p>索引：0 手腕；1–4 拇指；5–8 食指；9–12 中指；13–16 无名指；17–20 小拇指。normalized 分支用于相机叠加；world 分支驱动求解；Three.js 显示响应中的模型姿态。</p>
<h2>不改变求解精度的加速</h2><table><tr><th>模式</th><th>修改前中位 ms</th><th>修改后中位 ms</th></tr>{rows}</table>
<p>批量计算交互图距离权重、专用三维叉积。未减少迭代／约束／关键点。两种模式各 150 帧的原始 retarget 角度逐项相同；不是对所有未来输入的误差保证。</p>
<h2>实机记录中的延迟</h2><p>推理中位 13.30 ms / P95 28.36 ms；求解＋碰撞中位 20.20 ms / P95 23.95 ms；推理结束到响应记录中位 23.80 ms。后两项重叠，不能相加。匹配帧的推理＋响应记录中位 38.10 ms / P95 52.74 ms。</p>
<p>这些不包含相机曝光等待、USB 完成和电机响应。历史记录未提供它们的完整关联时间戳；不能声称获得真机端到端延迟。13,705 条 downlink 事件也不能当作固件执行确认。</p>
<h2>真实调用栈火焰图</h2><p>宽度为带剖析开销的累计耗时，不是未插桩毫秒。悬停看路径。下图去除初始化和前 10 帧；父子包含关系不能重复相加。</p>{svg}
<p><a href="after/flamegraph.html">打开可缩放的调用栈时间线</a> · <a href="after/hotspots.txt">查看 cProfile 热点</a> · <a href="accuracy.json">逐项精度对比</a></p>
<h2>下一步优化优先级</h2><p>① FK/Jacobian 与残差重复计算；② 骨方向初始化和交互图；③ 推理尖峰与浏览器/网络帧节奏；④ Ruckig 同步策略与跟随相位差。耗时快不等于电机跟随快，需分别测量。</p>
<p>本次火焰图覆盖滤波、retarget、目标避碰和转换，不覆盖独立的 500 Hz 控制线程。未部署 node0；重启本地服务后新参数生效。</p>'''
    (root/'report.html').write_text(page,encoding='utf-8')
    print(root/'report.html')


if __name__=='__main__':main()
