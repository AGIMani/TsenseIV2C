"""Read-only timing analysis and optional offline replay; no camera or USB I/O."""
import argparse
import collections
import hashlib
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'src')]
import numpy as np


def stats(values):
    a = np.asarray(values, float)
    if not len(a):
        return {'count': 0}
    return {'count': len(a), 'mean': float(a.mean()), **dict(zip(
        ('p50', 'p95', 'p99', 'max'), map(float, np.percentile(a, [50, 95, 99, 100]))))}


def analyze_diagnostic(path, started_at):
    rows = [json.loads(line) for line in path.open(encoding='utf-8-sig')]
    selected = [r for r in rows if r.get('wall_time', '') >= started_at]
    keys = ('event', 'wall_time', 'reason', 'age_ms', 'tick', 'message')
    return {'input': str(path.resolve()), 'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
        'origin': next(r['origin'] for r in rows if r.get('event') == 'page_open'),
        'events': [{k:r[k] for k in keys if k in r} for r in selected
            if r.get('event') in ('trajectory_recovery', 'control_disarm', 'notice')],
        'recovery_sample_age_ms': stats([r['age_ms'] for r in selected
            if r.get('event') == 'trajectory_recovery' and 'age_ms' in r])}


def analyze(path):
    raw, targets, retarget, first_write = {}, {}, {}, {}
    counts, recoveries = collections.Counter(), collections.Counter()
    usb_times, io, planner_hz = [], [], []
    last_usb = None
    metadata = None
    digest = hashlib.sha256()
    with path.open('rb') as f:
        for line in f:
            digest.update(line)
            r = json.loads(line); k = r['type']; counts[k] += 1
            frame = r.get('frame_id')
            if k == 'session':
                metadata = r
            elif k == 'mediapipe_raw':
                raw[frame] = r
            elif k == 'control_target':
                targets[frame] = r
            elif k == 'retarget':
                retarget[frame] = r
            elif k == 'usb_tx' and r.get('sent'):
                last_usb = r
            elif k == 'downlink' and r.get('sent') and r.get('source') == 'shared_binary_stream':
                # usb_tx is emitted after writer.write completes; downlink
                # supplies its frame identity after conversion/h7 logging.
                if last_usb is None or not last_usb['seq'] < r['seq']:
                    raise ValueError('Missing preceding successful USB write')
                if r['t']-last_usb['t'] > .02:
                    raise ValueError('Ambiguous write/frame association')
                first_write.setdefault(frame, last_usb['t'])
                usb_times.append(last_usb['t'])
                last_usb = None
            elif k == 'serial_io_stats':
                io.append(r['io'])
            elif k == 'planner_sample':
                planner_hz.append(r['control_timing']['actual_sample_hz'])
            elif k == 'trajectory_recovery':
                recoveries[r.get('reason')] += 1
    values = collections.defaultdict(list)
    rows = []
    for frame, target in targets.items():
        if frame not in raw or frame not in retarget:
            continue
        r, solver = raw[frame], retarget[frame]
        t = target.get('pipeline_timing')
        if not t:
            continue
        capture_t = target['t'] - t['capture_to_response_ms']/1000
        row = {'frame': frame, 'capture_t': capture_t, 'inference_ms': r['infer_ms'],
            **t, 'solve_including_collision_ms': solver['solve_ms'],
            'collision_ms': target['self_collision'].get('solve_ms', 0)}
        row['retarget_and_pose_ms'] = solver['solve_ms'] - row['collision_ms']
        row['image_transfer_ui_other_ms'] = (t['capture_to_response_ms']-
            t['request_response_ms']-t['queue_wait_ms']-r['infer_ms'])
        row['request_other_ms'] = t['request_response_ms']-solver['solve_ms']
        if frame in first_write:
            row['capture_to_first_usb_ms'] = (first_write[frame]-capture_t)*1000
            row['response_to_first_usb_ms'] = (first_write[frame]-target['t'])*1000
            row['post_inference_to_first_usb_ms'] = row['capture_to_first_usb_ms'] - (
                t['capture_to_response_ms']-t['request_response_ms']-t['queue_wait_ms'])
        rows.append(row)
        # Exclude startup frames, report the first inference separately.
        if frame >= 10:
            for key, value in row.items():
                if key not in ('frame', 'capture_t'):
                    values[key].append(value)
    times = [r['timestamp_ms'] for f, r in raw.items() if f >= 10]
    active_rows = [r for r in rows if 'capture_to_first_usb_ms' in r and r['frame'] >= 10]
    output = {'input': str(path.resolve()), 'sha256': digest.hexdigest(),
        'started_at': metadata['started_at'], 'metadata': metadata['metadata'],
        'counts': counts, 'timing_ms': {k: stats(v) for k, v in values.items()},
        'first_inference_ms': next(iter(raw.values()))['infer_ms'],
        'captured_frame_interval_ms': stats(np.diff(times)),
        'usb_interval_ms': stats(np.diff(usb_times)*1000),
        'usb_average_hz': (len(usb_times)-1)/(usb_times[-1]-usb_times[0]),
        'planner_hz': stats(planner_hz),
        'max_reported_usb_write_ms': max(r['max_write_ms'] for r in io),
        'recoveries': recoveries,
        'active_frame_count': len(active_rows),
        'scope': 'Steady statistics exclude frame_id < 10. End-to-end matches a frame ID to its first successful host USB write, not firmware receipt or mechanical arrival. Frame ID may lead regenerated wire by one controller tick. Capture time is reconstructed from control_target.t minus capture_to_response_ms; small log-call overhead remains. Image residual includes bitmap/worker/UI/log overhead; request residual includes preprocessing/serialization/HTTP/UI, not pure network latency.',
        'active_timing_ms': {k: stats([r[k] for r in active_rows]) for k in active_rows[0]
                             if k not in ('frame', 'capture_t')},
        'rows': rows}
    return output, raw


def replay(raw, count):
    from online_control.online_motion_track.main import WebSession
    from online_control.online_motion_track.a30.control import A30Control
    from hand_profiles import load_hand_profile
    selected = []
    for frame, r in raw.items():
        if frame < 10:
            continue
        labels = r.get('handedness', [])
        index = next((i for i, v in enumerate(labels) if v and v[0]['label']=='Left'), None)
        if index is not None:
            selected.append(r | {'hand_index': index})
        if len(selected) >= count+20:
            break
    session = WebSession('left', False, collision_mode='mink', model_id='TsenseI_v3_0')
    measured = collections.defaultdict(list)
    targets = []
    phases = collections.defaultdict(float)
    def instrument(obj, method, label):
        original = getattr(obj, method)
        def timed(*args, **kwargs):
            start = time.perf_counter()
            try:
                return original(*args, **kwargs)
            finally:
                phases[label] += (time.perf_counter()-start)*1000
        setattr(obj, method, timed)
    instrument(session.pipeline.session, 'retarget', 'retarget_ms')
    instrument(session.filter, 'filter', 'world_filter_ms')
    instrument(session.normalized_filter, 'filter', 'normalized_filter_ms')
    instrument(session.pipeline.mink_guard, 'project', 'target_collision_ms')
    try:
        for i, r in enumerate(selected):
            idx = r['hand_index']; phases.clear()
            packet = {'sequence': i, 'timestamp_ms': r['timestamp_ms'], 'capture_unix_ms': 0,
                'camera_size': [640,480], 'hand': {'label':'Left',
                    'world': r['world_landmarks'][idx], 'normalized': r['normalized_landmarks'][idx]}}
            start = time.perf_counter(); result = session.process(packet)
            elapsed = (time.perf_counter()-start)*1000
            if not result['tracking']:
                raise RuntimeError(result['message'])
            targets.append(result['pose'])
            if i >= 20:
                measured['server_process_ms'].append(elapsed)
                for k,v in phases.items(): measured[k].append(v)
    finally:
        session.close()
    control = A30Control(load_hand_profile('TsenseI_v3_0'), 'left', 'mink', start=False)
    instrument(control.guard, 'project', 'sample_collision_ms')
    instrument(control, '_checked_step', 'checked_ruckig_ms')
    ticks = collections.defaultdict(list)
    held = 0
    try:
        for i, target in enumerate(targets):
            control.push(target, i)
            for _ in range(25):  # 50 ms gesture target interval, 2 ms control interval
                phases.clear(); start = time.perf_counter(); control.step(.002)
                elapsed = (time.perf_counter()-start)*1000
                if not control.valid: raise RuntimeError(control.error)
                if i >= 20:
                    ticks['control_tick_ms'].append(elapsed)
                    for k,v in phases.items(): ticks[k].append(v)
                    held += bool(control.finger_holds)
        limits = control.snapshot()['motion_limits']
    finally:
        control.close()
    return {'scope': 'Current local code, replayed recorded landmarks, first 20 frames warmup; no HTTP, camera, USB or realtime scheduling. Control replay uses 50 ms targets and 2 ms ticks.',
        'server': {k: stats(v) for k,v in measured.items()},
        'control': {k: stats(v) for k,v in ticks.items()}, 'held_ticks': held, 'motion_limits': limits}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--diagnostic', type=Path)
    parser.add_argument('--replay-frames', type=int, default=0)
    args = parser.parse_args()
    report, raw = analyze(args.input)
    if args.diagnostic: report['diagnostic'] = analyze_diagnostic(args.diagnostic, report['started_at'])
    if args.replay_frames: report['current_code_replay'] = replay(raw, args.replay_frames)
    report['source_sha256'] = {p: hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in [
        'TsenseI_retargeting/toporetarget.py', 'online_control/online_motion_track/main.py',
        'online_control/online_motion_track/a30/control.py', 'online_control/online_motion_track/static/app.js']}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k not in ('metadata','rows','source_sha256')}, ensure_ascii=False, indent=2))
