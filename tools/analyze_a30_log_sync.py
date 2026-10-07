"""Read recorded host events without connecting a camera, serial port or server."""
import argparse
from collections import Counter, defaultdict
import hashlib
import json
from pathlib import Path

import numpy as np


def percentiles(values):
    if not len(values):
        return {}
    return dict(zip(('p50', 'p95', 'max'), map(float, np.percentile(values, [50, 95, 100]))))


def bend(pose):
    values = [v for n, v in pose.items() if '_thumb_' not in n
              and any(f'_{joint}_' in n for joint in ('MCP', 'PIP', 'DIP'))]
    return float(np.rad2deg(np.mean(values))) if values else float('nan')


def analyze(path):
    digest = hashlib.sha256()
    counts, skips, recoveries, masks, phases = (Counter() for _ in range(5))
    timings = defaultdict(list)
    events, tx, targets, commands, raw = [], [], [], [], []
    metadata = None
    with path.open('rb') as source:
        for line in source:
            digest.update(line)
            r = json.loads(line)
            kind = r['type']; counts[kind] += 1
            if kind == 'session':
                metadata = r
            elif kind == 'retarget_skipped':
                skips[r.get('reason')] += 1
            elif kind == 'retarget':
                timings['solve_ms'].append(r['solve_ms'])
                raw.append((r['t'], bend(r['joints'])))
            elif kind == 'control_target':
                for key, value in r.get('pipeline_timing', {}).items():
                    timings[key].append(value)
                timings['collision_ms'].append(r.get('self_collision', {}).get('solve_ms', 0))
                targets.append((r['t'], bend(r['joints'])))
            elif kind == 'planner_sample':
                phases[r.get('tracking_transition', {}).get('phase')] += 1
                timings['planner_hz'].append(r['control_timing']['actual_sample_hz'])
            elif kind == 'usb_tx' and r.get('sent'):
                tx.append(r['t'])
            elif kind == 'h7' and r.get('sent') and r.get('source') == 'shared_binary_stream':
                commands.append((r['t'], bend(r['joints_ui'])))
            elif kind == 'serial_io_stats':
                timings['reported_max_usb_write_ms'].append(r['io']['max_write_ms'])
            elif kind == 'telemetry':
                masks[str((r.get('self_online_mask'), r.get('external_online_mask')))] += 1
            if kind in ('trajectory_recovery', 'trajectory_recovered', 'control_disarm', 'control_arm'):
                events.append({k: r[k] for k in ('type', 't', 'reason', 'duration_ms', 'age_ms', 'tick') if k in r})
                if kind == 'trajectory_recovery':
                    recoveries[r['reason']] += 1
    result = dict(file=path.name, sha256=digest.hexdigest(), started_at=metadata['started_at'],
                  model_id=metadata['metadata']['model_id'], counts=counts, skips=skips,
                  recoveries=recoveries, phases=phases, telemetry_masks=masks,
                  timing={k: percentiles(v) for k, v in timings.items()},
                  usb_interval_ms=percentiles(np.diff(tx)*1000), events=events)
    return result, dict(raw=raw, targets=targets, commands=commands, tx=tx)


def chart(result, series, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True, layout='constrained',
                             gridspec_kw={'height_ratios': [2, 1, 1]})
    for name, label, color in (('raw', 'Retarget request', '#94a3b8'),
                               ('targets', 'Collision-checked target', '#137b80'),
                               ('commands', 'Successful host write', '#d05932')):
        data = np.asarray(series[name])
        if len(data):
            if name == 'commands':
                axes[0].step(data[:, 0], data[:, 1], where='post', color=color,
                             label=label+' (held until next write)', linewidth=1.25)
            else:
                # Never invent motion between observations separated by hand
                # loss. The command line above deliberately holds its last value.
                gaps = np.flatnonzero(np.diff(data[:, 0]) > .3)+1
                data = np.insert(data, gaps, np.nan, axis=0)
                axes[0].plot(data[:, 0], data[:, 1], color=color, label=label, linewidth=1.25)
    axes[0].set_ylabel('Mean finger flexion (deg)')
    axes[0].legend(loc='upper left', ncol=3, fontsize=8)
    recovery = [r for r in result['events'] if r['type'] == 'trajectory_recovery']
    axes[1].scatter([r['t'] for r in recovery], [r.get('age_ms', 0) for r in recovery],
                    s=18, color='#b24356')
    axes[1].axhline(100, color='#64748b', linestyle='--', linewidth=.8)
    axes[1].set_ylabel('Recovery sample age (ms)')
    bins = np.arange(0, max(series['tx'])+1, .5)
    axes[2].stairs(np.histogram(series['tx'], bins=bins)[0]/.5, bins, color='#137b80')
    axes[2].set_ylabel('Host USB writes / s')
    axes[2].set_xlabel('Seconds after recording start')
    for event in result['events']:
        if event['type'] == 'control_disarm' and '2 秒' in event.get('reason', ''):
            for ax in axes:
                ax.axvline(event['t'], color='#b24356', linestyle='--', linewidth=1)
            axes[0].annotate('Automatic stop: recovery timeout', (event['t'], 10),
                             xytext=(event['t']+4, 20), fontsize=9,
                             arrowprops={'arrowstyle': '->', 'color': '#b24356'})
    for ax in axes:
        ax.grid(alpha=.2)
    fig.suptitle(result['file']+'\nRecorded host commands; not firmware receipt or mechanical position', fontsize=12)
    fig.savefig(output, dpi=150)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('logs', nargs='+', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(); args.output.mkdir(parents=True, exist_ok=True)
    results = []
    for path in args.logs:
        result, series = analyze(path)
        results.append(result)
        chart(result, series, args.output/(path.stem+'.png'))
        print(json.dumps({'file': path.name, 'recoveries': result['recoveries'],
                          'usb_interval_ms': result['usb_interval_ms']}, ensure_ascii=False), flush=True)
    (args.output/'analysis.json').write_text(json.dumps(results, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')


if __name__ == '__main__':
    main()
