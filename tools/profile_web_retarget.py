"""Replay recorded browser landmarks without camera, network or hardware I/O.

python tools/profile_web_retarget.py --input recording.jsonl --output logs/retarget_profile
Optional --trace writes an interactive VizTracer HTML call-stack timeline.
Uninstrumented timings are measured separately from profiling.
"""
from pathlib import Path
import argparse
import collections
import cProfile
import json
import pstats
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'src')]
import numpy as np
from online_control.online_motion_track.main import WebSession


def stats(values):
    return ({'count': len(values), 'median_ms': float(np.median(values)),
             'p95_ms': float(np.percentile(values, 95)), 'max_ms': float(max(values))}
            if values else {'count': 0})


def read_records(path, limit):
    packets, infer, combined, transaction, detected_reply = [], [], [], [], []
    starts = {}
    counts = collections.Counter()
    with path.open(encoding='utf-8') as f:
        for line in f:
            row = json.loads(line); kind = row.get('type'); counts[kind] += 1
            if kind == 'mediapipe_raw':
                infer.append(row['infer_ms'])
                starts[row['frame_id']] = (row['t'], row['infer_ms'])
                labels = row.get('handedness', [])
                index = next((i for i, c in enumerate(labels) if c and c[0]['label'] == 'Left'), None)
                if index is not None and len(packets) < limit:
                    packets.append({'sequence': len(packets), 'timestamp_ms': row['timestamp_ms'],
                        'capture_unix_ms': 0, 'camera_size': [640, 480],
                        'hand': {'label': 'Left', 'world': row['world_landmarks'][index],
                                 'normalized': row['normalized_landmarks'][index]}})
            if kind == 'retarget':
                combined.append(row['solve_ms'])
                if row['frame_id'] in starts:
                    at, inference_ms = starts.pop(row['frame_id'])
                    elapsed = (row['t'] - at) * 1000
                    transaction.append(elapsed)
                    detected_reply.append(elapsed + inference_ms)
    if not packets:
        raise ValueError('No left-hand raw landmark frames found')
    return packets, {'counts': counts, 'inference': stats(infer),
                     'recorded_solve_including_collision': stats(combined),
                     'post_inference_to_recorded_reply': stats(transaction),
                     'inference_plus_reply_matched_frames': stats(detected_reply)}


def run(packets, mode, backend):
    session = WebSession('left', False, collision_mode=mode, model_id='TsenseI_v3_0')
    if backend != 'numpy':
        from dataclasses import replace
        from TsenseI_retargeting import TopoRetargeter
        old = session.pipeline.session.retargeter
        session.pipeline.session.retargeter = TopoRetargeter(replace(old.config, kinematics_backend=backend))
    rows, outputs = [], []
    try:
        for i, packet in enumerate(packets):
            begin = time.perf_counter()
            result = session.process(packet)
            elapsed = (time.perf_counter() - begin) * 1000
            rows.append({'frame': i, 'total_ms': elapsed, 'solve_ms': result['solve_ms'],
                         'collision_ms': result['collision'].get('solve_ms', 0),
                         'tracking': result['tracking']})
            outputs.append(result['retarget_joint_angles_rad'])
    finally:
        session.close()
    return rows, outputs


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--frames', type=int, default=150)
    parser.add_argument('--backend', default='numpy', choices=['numpy', 'pinocchio'])
    parser.add_argument('--trace', action='store_true')
    args = parser.parse_args(); args.output.mkdir(parents=True, exist_ok=True)
    packets, recorded = read_records(args.input, args.frames)
    report = {'input': str(args.input.resolve()), 'backend': args.backend, 'recorded': recorded,
              'scope': 'Local replay, first left-hand frames; excludes browser/GPU/network/USB and 500Hz controller.'}
    for mode in ('off', 'mink'):
        rows, outputs = run(packets, mode, args.backend)
        report[mode] = {'steady_total': stats([r['total_ms'] for r in rows[10:]]),
                        'tracking_frames': sum(r['tracking'] for r in rows), 'frames': rows}
        (args.output / f'{mode}_angles.json').write_text(json.dumps(outputs), encoding='utf-8')
    profiler = cProfile.Profile()
    profiler.runcall(run, packets, 'mink', args.backend)
    profiler.dump_stats(str(args.output / 'profile.pstats'))
    with (args.output / 'hotspots.txt').open('w', encoding='utf-8') as f:
        pstats.Stats(profiler, stream=f).strip_dirs().sort_stats('cumulative').print_stats(45)
    if args.trace:
        from viztracer import VizTracer
        tracer = VizTracer(output_file=str(args.output / 'flamegraph.html'), tracer_entries=3000000)
        with tracer:
            run(packets[:40], 'mink', args.backend)
        tracer.save(str(args.output / 'trace.json'))
    (args.output / 'metrics.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps({k: v if k == 'recorded' else v.get('steady_total') for k,v in report.items()
                      if k in ('recorded','off','mink')}, indent=2))


if __name__ == '__main__':
    main()
