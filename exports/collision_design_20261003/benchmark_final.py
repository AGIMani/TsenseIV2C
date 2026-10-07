from benchmark_mink import *
class CacheOnly(CachedBoundedGuard):
    def __init__(self,*a,**kw):
        super().__init__(*a,**kw)
        capsule=int(mujoco.mjtGeom.mjGEOM_CAPSULE)
        self.bounded_pairs=np.all(self.model.geom_type[self.pair_ids]==capsule,axis=1)
result={'notes':['Synthetic open/flex/pinch/crossed/release, 40 sequential targets per run, 3 alternating timing runs per variant','No geometry changes. Cache-only requires exact q byte keys, per-call reset, _update tracks actual configuration and only skips identical q','No hardware, retargeting, Ruckig, or end-to-end IO; observed milliseconds on this WSL host are not realtime guarantees','Instrumented times are inclusive; profile instrumentation overhead excluded from timing runs'],'models':{}}
for model in ('TsenseI_v3_2','TsenseI_v3_2_withid'):
    p=load_hand_profile(model); br=[]; cr=[]; maxdiff=0; trials=[]
    for trial in range(3):
        if trial%2:
            c,cq=run(CacheOnly,p); b,bq=run(MinkCollisionGuard,p)
        else:
            b,bq=run(MinkCollisionGuard,p); c,cq=run(CacheOnly,p)
        br+=b['rows']; cr+=c['rows']; maxdiff=max(maxdiff,float(np.max(abs(bq-cq))))
        trials.append({'baseline':b['summary'],'cache_only':c['summary']})
    bp,_=run(MinkCollisionGuard,p,True); cp,_=run(CacheOnly,p,True)
    out={'ngeom':bp['ngeom'],'nmesh':bp['nmesh'],'pair_count':bp['pair_count'],'baseline_summary':summarize(br),'cache_only_summary':summarize(cr),'max_output_difference_rad':maxdiff,'trials':trials,'baseline_profile':{k:bp[k] for k in ('summary','trace_counts','trace_inclusive_ms')},'cache_only_profile':{k:cp[k] for k in ('summary','trace_counts','trace_inclusive_ms')}}
    result['models'][model]=out
    print(model,json.dumps(out),flush=True)
(ROOT/'exports/collision_design_20261003/benchmark_final.json').write_text(json.dumps(result,indent=2))
