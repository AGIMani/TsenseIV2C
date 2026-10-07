"""Read A32 JSONL logs; distinguish host command timing from physical packets."""
import argparse
from collections import Counter, defaultdict
import json
from pathlib import Path

import numpy as np


def stats(values):
    data = np.asarray(values, dtype=float)
    data = data[np.isfinite(data)]
    if not len(data):
        return None
    return dict(n=len(data), **dict(zip(
        ('min', 'p50', 'p95', 'p99', 'max'),
        map(float, np.quantile(data, [0, .5, .95, .99, 1])))))


def analyze(path):
    counts, timings = Counter(), defaultdict(list)
    commands, samples, diagnostics, actions = [], [], [], []
    metadata, ended = {}, False
    with Path(path).open(encoding='utf-8-sig') as stream:
        for number, line in enumerate(stream, 1):
            try:
                row = json.loads(line)
            except ValueError as exc:
                raise ValueError(f'{path}:{number}: invalid JSONL') from exc
            kind = row.get('type')
            counts[kind] += 1
            if kind == 'session':
                metadata = row.get('metadata', {})
            if kind == 'session_end':
                ended = True
            for field in ('control_timing', 'pipeline_timing'):
                for key, value in (row.get(field) or {}).items():
                    if type(value) in (int, float):
                        timings[key].append(value)
            if kind == 'retarget' and type(row.get('solve_ms')) in (int, float):
                timings['solve_ms'].append(row['solve_ms'])
            if kind == 'downlink' and row.get('sent') and row.get('source') == 'a32_tracking':
                commands.append(row)
            if kind == 'h7' and row.get('sent'):
                samples.append(row)
            if kind == 'sdk_status':
                diagnostics.append((row.get('sdk_state') or {}).get('diagnostics', {}))
            if kind in ('sdk_action', 'sdk_error', 'control_disarm'):
                actions.append({k: row[k] for k in ('type', 'action', 'reason', 'error', 't') if k in row})
    native = bool(commands)
    # Browser event t is batch delivery time, so never use it for SDK cadence.
    timed = commands if native else [r for r in samples if type(r.get('sent_at_unix_ms')) in (int, float)]
    times = [r['t'] * 1000 if native else r['sent_at_unix_ms'] for r in timed]
    gaps = np.diff(times)
    cadence = {'basis': 'host JSONL capture after successful SDK write (QPC)' if native
               else 'host sample publication wall clock; browser delivery times excluded',
               'not_device_arrival_timestamps': True, 'statistics_ms': stats(gaps),
               'below_1ms': int(sum(gaps < 1)), 'above_4ms': int(sum(gaps > 4)),
               'within_1_to_4ms': int(sum((gaps >= 1) & (gaps <= 4))),
               'nonpositive_intervals': int(sum(gaps <= 0)),
               'all_intervals_included': True}
    if len(times) > 1 and times[-1] > times[0]:
        cadence['average_hz'] = (len(times)-1)*1000/(times[-1]-times[0])
    joint_deltas = defaultdict(list)
    tracking = [r for r in samples if r.get('source') in ('a32_tracking', 'a32_sdk_successful_write')]
    native_intervals = [(r.get('control_timing') or {}).get('native_write_interval_ms') for r in tracking[1:]]
    native_intervals = np.asarray([v for v in native_intervals if type(v) in (float, int) and v >= 0])
    native_cadence = dict(basis='native SDK sendOnce start-to-start; not device arrival',
        statistics_ms=stats(native_intervals), below_1ms=int(sum(native_intervals < 1)),
        above_4ms=int(sum(native_intervals > 4)),
        within_1_to_4ms=int(sum((native_intervals >= 1) & (native_intervals <= 4))))
    for previous, current in zip(tracking, tracking[1:]):
        before = previous.get('joints_hw') or previous.get('joints') or {}
        after = current.get('joints_hw') or current.get('joints') or {}
        for key in before.keys() & after.keys():
            joint_deltas[key].append(abs(after[key] - before[key]))
    channel_deltas = {}
    for key in ('linear_mm', 'rotary_deg'):
        values = []
        for previous, current in zip(commands, commands[1:]):
            before, after = previous.get(key, []), current.get(key, [])
            if before and len(before) == len(after) and all(v is not None for v in before + after):
                values.append(max(abs(a-b) for a, b in zip(after, before)))
        channel_deltas[key] = stats(values)
    progress = None
    if native and len(tracking) > 1:
        wall = tracking[-1]['t'] - tracking[0]['t']
        planned = sum(r.get('planner_dt_s') or 0 for r in tracking[1:])
        progress = dict(wall_s=wall, planner_s=planned,
                        planner_to_wall_ratio=planned/wall if wall > 0 else None)
    sdk = {key: stats([r[key] for r in diagnostics if type(r.get(key)) in (int, float)])
           for key in ('last_send_duration_ms', 'max_send_duration_ms', 'send_successes',
                       'send_failures', 'disconnect_count')}
    return dict(path=str(Path(path).resolve()), ended=ended,
                model=metadata.get('model_id'), counts=dict(counts), cadence=cadence, native_cadence=native_cadence,
                timing={k: stats(v) for k, v in timings.items()},
                adjacent_joint_delta_rad={k: stats(v) for k, v in joint_deltas.items()},
                adjacent_packet_max_channel_delta=channel_deltas, trajectory_progress=progress,
                sdk_status_samples=sdk, actions=actions)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('logs', nargs='+', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    results = [analyze(path) for path in args.logs]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(results, ensure_ascii=False, indent=2), encoding='utf-8')
    for result in results:
        print(json.dumps({k: result[k] for k in ('path', 'ended', 'cadence', 'trajectory_progress')},
                         ensure_ascii=False))
