"""Replay recorded targets through real guards; no camera, serial or motor I/O."""
import argparse
import json
from pathlib import Path
import sys
import time

import numpy as np
import ruckig

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'src')]
from hand_profiles import load_hand_profile
from online_control.online_motion_track.a30.control import A30Control


def replay(path, destination):
    rows = [json.loads(line) for line in path.open(encoding='utf-8-sig')]
    targets = [r for r in rows if r['type'] == 'control_target']
    conversions = {r['frame_id']: r for r in rows if r['type'] == 'a30_conversion'}
    limits = next(r['motion_limits'] for r in rows if r['type'] == 'planner_sample')
    side = rows[0]['metadata']['hand_side']
    tt = np.array([r['t'] for r in targets])
    summary = {'input': str(path), 'scope': 'Recorded collision-safe targets, 2 ms fixed replay clock, real combined-pose collision and position checks. No USB or hardware. Lag is waveform alignment, not a per-frame transport time.', 'cases': {}}
    traces = {'t': tt}
    for label in ('whole_hand_recorded_limits', 'groups_recorded_limits', 'groups_current_limits'):
        c = A30Control(load_hand_profile('TsenseI_v3_0'), side, 'mink', start=False)
        try:
            names = list(c.names)
            if label != 'groups_current_limits':
                for field in ('velocity', 'acceleration', 'jerk'):
                    setattr(c.input, 'max_' + field, [limits[n][field] for n in names])
            if label.startswith('whole'):
                c.otg = ruckig.Ruckig(20, .002)
                c.output = ruckig.OutputParameter(20)
            grid = np.arange(tt[0] - .002, tt[-1] + .004, .002)
            positions = []; tick_ms = []; peak = np.zeros(20); holds = 0; index = 0
            for at in grid:
                while index < len(targets) and tt[index] <= at:
                    c.push(targets[index]['joints'], targets[index]['frame_id']); index += 1
                start = time.perf_counter()
                c.step(.002, sample_at=at)
                tick_ms.append((time.perf_counter()-start)*1000)
                if not c.valid:
                    raise RuntimeError(f'{label}: {c.error}')
                assert all(lo-1e-7 <= c.pose[n] <= hi+1e-7 for n, (lo, hi) in c.limits.items())
                positions.append([c.pose[n] for n in names])
                peak = np.maximum(peak, np.abs(c.output.new_velocity))
                holds += bool(c.finger_holds)
            positions = np.array(positions)
            predicted = np.column_stack([np.interp(tt, grid, positions[:,i]) for i in range(20)])
            target = np.array([[r['joints'][n] for n in names] for r in targets])
            observed = np.array([[conversions[r['frame_id']]['joint_angles_rad'][n] for n in names] for r in targets])
            mask = tt > tt[0] + 1
            flex = [i for i,n in enumerate(names) if 'swing' not in n and 'rota' not in n]
            losses = []
            for lag in np.arange(0, .501, .01):
                past = np.column_stack([np.interp(tt[mask]-lag, tt, target[:,i]) for i in flex])
                losses.append(np.mean((predicted[mask][:,flex]-past)**2))
            metrics = {
                'angle_rmse_deg': float(np.sqrt(np.mean((predicted[mask]-target[mask])**2))*180/np.pi),
                'flex_waveform_lag_ms': int(np.argmin(losses))*10,
                'vs_original_output_rmse_deg': float(np.sqrt(np.mean((predicted[mask]-observed[mask])**2))*180/np.pi),
                'peak_flex_rad_s': float(peak[flex].max()), 'collision_hold_ticks': holds,
                'limit_recoveries': c.limit_recoveries,
                'tick_ms_p50_p95_p99': np.percentile(tick_ms, [50,95,99]).tolist(),
                'limits': {n: dict(velocity=c.input.max_velocity[i], acceleration=c.input.max_acceleration[i], jerk=c.input.max_jerk[i]) for i,n in enumerate(names)}}
            summary['cases'][label] = metrics
            traces[label] = predicted
            print(label, json.dumps({k:v for k,v in metrics.items() if k!='limits'}), flush=True)
        finally:
            c.close()
    summary['joint_names'] = names
    traces.update(target=target, original_output=observed)
    destination.mkdir(parents=True, exist_ok=True)
    (destination/'metrics.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')
    np.savez_compressed(destination/'traces.npz', **traces)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log', type=Path)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    replay(args.log, args.out)
