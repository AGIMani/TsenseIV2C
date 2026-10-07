from benchmark_mink import *
class OverrideProfile:
    def __init__(self,profile,mjcf):self.original=profile; self.mjcf=Path(mjcf)
    def __getattr__(self,name):return getattr(self.original,name)
    def path(self,key,side='left'):
        return self.mjcf if key=='mjcf' else self.original.path(key,side)
result={'notes':['Same 40 deterministic target sequence as benchmark_mink.py; same production guard, only one thumb root geometry replaced','Different geometry changes accepted paths and iterations, so timing is not a fixed-pair microbenchmark','These are unsigned fit candidates, not validated hardware geometry or replacement settings'],'models':{}}
p=load_hand_profile('TsenseI_v3_2')
base,bq=run(MinkCollisionGuard,p)
result['models']['baseline']=base
for budget in (64,96):
    mjcf=ROOT/f'exports/collision_design_20261003/geometry/a32_thumb_root_outer_{budget}_model.xml'
    candidate=OverrideProfile(p,mjcf)
    try:
        data,cq=run(MinkCollisionGuard,candidate)
        data['max_output_difference_from_baseline_rad']=float(np.max(abs(cq-bq)))
        result['models'][f'thumb_root_outer_{budget}']=data
        print(budget,json.dumps(data['summary']),'output_difference',data['max_output_difference_from_baseline_rad'],flush=True)
    except Exception as exc:
        result['models'][f'thumb_root_outer_{budget}']={'error':repr(exc)}
        print(budget,'failed',repr(exc),flush=True)
print('baseline',base['summary'])
(ROOT/'exports/collision_design_20261003/benchmark_geometry.json').write_text(json.dumps(result,indent=2))
