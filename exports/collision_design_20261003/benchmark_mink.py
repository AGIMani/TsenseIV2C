"""Isolated offline Mink benchmark; does not drive hardware or edit production."""
from pathlib import Path
import sys, json, time, functools, collections, platform
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT),str(ROOT/'src')]
import mujoco, mink
from hand_profiles import load_hand_profile
from online_control.online_motion_track.mink_guard import MinkCollisionGuard

class CachedBoundedGuard(MinkCollisionGuard):
    def __init__(self,*a,**kw):
        self._last_q=None
        self._cache={}
        super().__init__(*a,**kw)
        finite=(self.model.geom_rbound>0)&np.isfinite(self.model.geom_rbound)
        self.bounded_pairs=np.all(finite[self.pair_ids],axis=1)
    def _update(self,q):
        if self._last_q is None or not np.array_equal(q,self._last_q):
            super()._update(q)
            self._last_q=q.copy()
    def _feasible(self,q):
        key=q.tobytes()
        if key not in self._cache:
            self._cache[key]=super()._feasible(q)
        return self._cache[key]
    def project(self,pose):
        self._cache={}
        return super().project(pose)

def targets(profile):
    init=profile.init_pose('left')
    # Synthetic joint-space motion; not measured tracking recordings.
    rows=[]
    def blend(a,b,t): return {n:a[n]+t*(b[n]-a[n]) for n in a}
    neutral=dict(init)
    for n in neutral:
        if '_swing_' in n: neutral[n]=.35
    flex=dict(neutral)
    for n in flex:
        if n.endswith('_MCP_joint'): flex[n]=.65
        elif n.endswith('_PIP_joint'): flex[n]=.8
        elif n.endswith('_DIP_joint'): flex[n]=.55
    pinch=dict(neutral)
    for n in pinch:
        if '_index_MCP_' in n: pinch[n]=.65
        if '_index_PIP_' in n: pinch[n]=.5
        if '_index_DIP_' in n: pinch[n]=.3
        if '_thumb_rota_block_' in n: pinch[n]=1.2
        if '_thumb_rotaback_' in n: pinch[n]=.85
        if '_thumb_MCP_' in n: pinch[n]=.45
        if '_thumb_PIP_' in n: pinch[n]=.65
    crossed=dict(neutral)
    crossed['left_hand_ring_swing_joint']=.695
    crossed['left_hand_pinky_swing_joint']=.005
    for start,end,category in [(init,neutral,'open'),(neutral,flex,'flex'),(flex,pinch,'pinch_synthetic'),(pinch,crossed,'crossed'),(crossed,init,'release')]:
        for t in np.linspace(.125,1,8): rows.append((category,blend(start,end,t)))
    return rows

def summarize(rows):
    ts=np.array([r['wall_ms'] for r in rows])
    return {'calls':len(rows),'p50_ms':float(np.median(ts)),'p95_ms':float(np.percentile(ts,95)),'max_ms':float(ts.max()),'mean_ms':float(ts.mean()),'statuses':dict(collections.Counter(r['status'] for r in rows)),'mean_iterations':float(np.mean([r['iterations'] for r in rows]))}

def run(cls,profile,trace=False):
    g=cls(profile,'left')
    # Warm imports / solver using a separate guard, so measured starts always match.
    warm=cls(profile,'left'); warm.project(targets(profile)[20][1])
    rows=[]; counts=collections.Counter(); times=collections.Counter(); maxerr=0
    methods=['_update','_feasible','_path_feasible','_distances','solve_qp']
    def wrap(fn,label):
        @functools.wraps(fn)
        def timed(*a,**kw):
            start=time.perf_counter_ns()
            try:return fn(*a,**kw)
            finally:
                counts[label]+=1; times[label]+=(time.perf_counter_ns()-start)/1e6
        return timed
    origdist=mujoco.mj_geomDistance
    if trace:
        for label in methods:setattr(g,label,wrap(getattr(g,label),label))
        for limit in g.limits:
            limit.compute_qp_inequalities=wrap(limit.compute_qp_inequalities,'inequalities')
        mujoco.mj_geomDistance=wrap(origdist,'mj_geomDistance')
    outputs=[]
    try:
        for category,pose in targets(profile):
            start=time.perf_counter_ns(); result=g.project(pose); wall=(time.perf_counter_ns()-start)/1e6
            rows.append({'category':category,'wall_ms':wall,**g.diagnostics})
            outputs.append(g.q.copy())
    finally:mujoco.mj_geomDistance=origdist
    return {'summary':summarize(rows),'by_category':{cat:summarize([r for r in rows if r['category']==cat]) for cat in dict.fromkeys(r['category'] for r in rows)},'ngeom':g.model.ngeom,'nmesh':g.model.nmesh,'pair_count':len(g.pairs),'trace_counts':dict(counts),'trace_inclusive_ms':dict(times),'rows':rows},np.array(outputs)

def main():
    result={'environment':{'python':platform.python_version(),'mujoco':mujoco.__version__,'mink_module':mink.__file__,'platform':platform.platform()},'notes':['40 deterministic sequential synthetic joint-space targets per timing run; no hardware, no retarget or Ruckig timing','Timer wrappers are only enabled for a separate profile run; profile timings are inclusive and must not be summed','Prototype caches exact repeated configurations and feasibility results within one project call, broadphase uses finite positive geom_rbound for all geometry types','Reported geometry differences change actual motion/feasibility; this is not an apples-to-apples shape fidelity comparison'],'models':{}}
    for model in ('TsenseI_v3_2','TsenseI_v3_2_withid'):
        p=load_hand_profile(model)
        baseline,baseout=run(MinkCollisionGuard,p)
        prototype,out=run(CachedBoundedGuard,p)
        traced,_=run(MinkCollisionGuard,p,True)
        # Confirm broadphase/caching did not change the output trajectory in this corpus.
        result['models'][model]={'baseline':baseline,'prototype':prototype,'profile_baseline':traced,'max_joint_output_difference_rad':float(np.max(abs(baseout-out)))}
        print(model,json.dumps({'baseline':baseline['summary'],'prototype':prototype['summary'],'difference':float(np.max(abs(baseout-out))),'calls':traced['trace_counts'],'times':traced['trace_inclusive_ms']}),flush=True)
    (ROOT/'exports/collision_design_20261003/benchmark_results.json').write_text(json.dumps(result,indent=2))

if __name__=='__main__':main()
