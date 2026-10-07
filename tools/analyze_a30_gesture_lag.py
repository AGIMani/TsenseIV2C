"""Offline stage/phase-lag diagnosis. Reads JSONL; never starts USB or a server."""
import argparse
import collections
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'src'), str(ROOT / 'src/tracking')]
import numpy as np
import ruckig
from wuji_mediapipe import _OneEuroLandmarkFilter
from online_control.shared.downlink import configure_ruckig


def stats(values):
    a = np.asarray(values, float)
    return dict(zip(('p50', 'p95', 'p99', 'max'), map(float, np.percentile(a, [50, 95, 99, 100])))) if a.size else {}


def lag(tx, x, ty, y, max_lag=.8):
    # Compare y(t) to x(t-lag) on a fixed common interval for all candidates.
    grid = np.arange(max(tx[0], ty[0]) + max_lag, min(tx[-1], ty[-1]), .01)
    if len(grid) < 30 or np.std(x) < .015 or np.std(y) < .015:
        return None
    b = np.interp(grid, ty, y)
    scores = [np.corrcoef(np.interp(grid-d, tx, x), b)[0, 1] for d in np.arange(0, max_lag+.001, .005)]
    i = int(np.nanargmax(scores))
    return {'lag_ms': i*5, 'correlation': float(scores[i]), 'zero_lag_correlation': float(scores[0])}


def replay(target_times, targets, names, mode, dt=.002):
    otg = ruckig.Ruckig(20, dt)
    inp, out = ruckig.InputParameter(20), ruckig.OutputParameter(20)
    configure_ruckig(inp, names, acceleration=15., jerk=120.)
    inp.synchronization = getattr(ruckig.Synchronization, mode)
    inp.current_position = [0.]*20
    inp.current_velocity = inp.current_acceleration = [0.]*20
    inp.target_velocity = inp.target_acceleration = [0.]*20
    times = np.arange(target_times[0], target_times[-1], dt)
    values = np.zeros((len(times), 20)); index = 0
    for i, t in enumerate(times):
        while index+1 < len(target_times) and target_times[index+1] <= t:
            index += 1
        inp.target_position = targets[index].tolist()
        result = otg.update(inp, out)
        if result not in (ruckig.Result.Working, ruckig.Result.Finished):
            raise RuntimeError(result)
        values[i] = out.new_position
        out.pass_to_input(inp)
    return times, values


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(); args.output.mkdir(parents=True, exist_ok=True)
    e = collections.defaultdict(list)
    keep = {'session','mediapipe_raw','retarget','control_target','planner_sample','h7','control_disarm'}
    counts = collections.Counter()
    for line in args.input.open(encoding='utf-8-sig'):
        row = json.loads(line); kind = row['type']; counts[kind] += 1
        if kind in keep: e[kind].append(row)
    raw = {r['frame_id']: r for r in e['mediapipe_raw']}
    ret = e['retarget']; safe = e['control_target']
    safe_by_id = {r['frame_id']: r for r in safe}
    names = list(ret[0]['joints']); tt = np.array([r['t'] for r in ret])
    qr = np.array([[r['joints'][n] for n in names] for r in ret])
    qs = np.array([[safe_by_id[r['frame_id']]['joints'][n] for n in names] for r in ret])
    h7 = [r for r in e['h7'] if r.get('frame_id', -1) >= 0]
    th = np.array([r['t'] for r in h7]); qh = np.array([[r['joints_ui'][n] for n in names] for r in h7])
    collisions = [r['self_collision'] for r in safe]
    report = {'input': str(args.input), 'sha256': hashlib.sha256(args.input.read_bytes()).hexdigest(),
              'metadata': e['session'][0]['metadata'], 'counts': counts,
              'scope': 'Logged host timestamps; phase estimates are correlations, not firmware latency. Replay fixes logged collision-safe targets and excludes live collision checks/network/USB.',
              'timing_ms': {
                  'inference_matched': stats([raw[r['frame_id']]['infer_ms'] for r in ret]),
                  'solve_including_collision': stats([r['solve_ms'] for r in ret]),
                  'inference_plus_response': stats([raw[r['frame_id']]['infer_ms']+(r['t']-raw[r['frame_id']]['t'])*1000 for r in ret]),
                  'post_inference_response': stats([(r['t']-raw[r['frame_id']]['t'])*1000 for r in ret]),
                  'valid_target_interval': stats(np.diff(tt)*1000),
                  'collision_compute': stats([r['solve_ms'] for r in collisions])},
              'collision_status': collections.Counter(r['status'] for r in collisions),
              'collision_correction_deg': stats(np.max(np.abs(qr-qs), axis=1)*180/np.pi),
              'sample_collision_limited': sum(bool(r['sample_collision'].get('limited')) for r in e['planner_sample']),
              'sample_hz': stats([r['control_timing']['actual_sample_hz'] for r in e['planner_sample'] if r['frame_id'] >= 0]),
              'disarms': [{'t':r['t'],'reason':r['reason']} for r in e['control_disarm']]}
    report['logged_target_to_output'] = {n:lag(tt,qs[:,i],th,qh[:,i]) for i,n in enumerate(names)}
    # Replay exactly the selected raw world landmarks through the current filter.
    filt = _OneEuroLandmarkFilter(min_cutoff=1.5,beta=.25,derivative_cutoff=1.)
    raw_points=[]; filtered=[]; tf=[]; last=-1.
    for r in e['mediapipe_raw']:
        labels=r['handedness']; match=next((i for i,c in enumerate(labels) if c and c[0]['label']=='Left'), None)
        if match is None:filt.reset();last=r['timestamp_ms'];continue
        if r['timestamp_ms']-last>300:filt.reset()
        p=np.array(r['world_landmarks'][match]);last=r['timestamp_ms']
        raw_points.append(p);filtered.append(filt.filter(p,last/1000));tf.append(last/1000)
    def flex(points, a,b,c):
        p=np.array(points);u=p[:,b]-p[:,a];v=p[:,c]-p[:,b]
        return np.arccos(np.clip(np.sum(u*v,axis=1)/np.linalg.norm(u,axis=1)/np.linalg.norm(v,axis=1),-1,1))
    report['one_euro_flex_lag']={}
    for finger,base in [('index',5),('mid',9),('ring',13),('pinky',17)]:
        for joint,offset in [('PIP',0),('DIP',1)]:
            args_f=(base+offset,base+offset+1,base+offset+2)
            report['one_euro_flex_lag'][finger+'_'+joint]=lag(tf,flex(raw_points,*args_f),tf,flex(filtered,*args_f),.3)
    sims={}
    for mode in ['Time','No']:
        ts, q = replay(tt,qs,names,mode);sims[mode]=(ts,q)
        report['replay_'+mode]={n:lag(tt,qs[:,i],ts,q[:,i]) for i,n in enumerate(names)}
    fine_t,fine_q=replay(tt,qs,names,'Time',dt=.001)
    coarse_t,coarse_q=sims['Time']
    fine_on_coarse=np.column_stack([np.interp(coarse_t,fine_t,fine_q[:,i]) for i in range(20)])
    report['Time_1ms_vs_2ms_abs_difference_deg']=stats(np.abs(fine_on_coarse-coarse_q)*180/np.pi)
    report['replay_Time_1ms']={n:lag(tt,qs[:,i],fine_t,fine_q[:,i]) for i,n in enumerate(names)}
    # Align the independent replay and observed host output on their common range.
    compare_t=np.arange(max(th[0],coarse_t[0])+1,min(th[-1],coarse_t[-1]),.01)
    report['Time_replay_vs_logged_mae_deg']={n:float(np.mean(np.abs(np.interp(compare_t,th,qh[:,i])-np.interp(compare_t,coarse_t,coarse_q[:,i])))*180/np.pi) for i,n in enumerate(names)}
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes=plt.subplots(3,1,figsize=(13,9),sharex=True)
    for ax,kind in zip(axes,['MCP','PIP','DIP']):
        name='left_hand_pinky_'+kind+'_joint';i=names.index(name)
        ax.plot(tt,qr[:,i]*180/np.pi,label='Retarget target',alpha=.7)
        ax.plot(tt,qs[:,i]*180/np.pi,label='Collision-safe target',alpha=.8)
        ax.plot(th,qh[:,i]*180/np.pi,label='Logged Ruckig output',lw=1.5)
        ax.plot(sims['No'][0],sims['No'][1][:,i]*180/np.pi,label='No-sync replay (not deployed)',ls='--',alpha=.8)
        ax.set_ylabel(kind+' / deg');ax.grid(alpha=.2)
    axes[0].legend(ncol=2);axes[-1].set_xlabel('Browser recording time / s')
    fig.suptitle('A3.0 gesture lag — recorded targets/output and isolated Ruckig replay')
    fig.tight_layout();fig.savefig(args.output/'gesture_lag.png',dpi=150)
    axes[-1].set_xlim(34,40);fig.savefig(args.output/'gesture_lag_detail.png',dpi=150);plt.close(fig)
    (args.output/'metrics.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k in ['timing_ms','collision_status','sample_hz','one_euro_flex_lag','Time_1ms_vs_2ms_abs_difference_deg']},ensure_ascii=False,indent=2))
    for n in names:
        if 'pinky' in n or 'thumb' in n:print(n,'logged',report['logged_target_to_output'][n],'Time',report['replay_Time'][n],'No',report['replay_No'][n])


if __name__ == '__main__': main()
