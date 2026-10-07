from benchmark_mink import *
class CacheOnly(CachedBoundedGuard):
    def __init__(self,*a,**kw):
        super().__init__(*a,**kw)
        capsule=int(mujoco.mjtGeom.mjGEOM_CAPSULE)
        self.bounded_pairs=np.all(self.model.geom_type[self.pair_ids]==capsule,axis=1)
class BoundsOnly(MinkCollisionGuard):
    def __init__(self,*a,**kw):
        super().__init__(*a,**kw)
        finite=(self.model.geom_rbound>0)&np.isfinite(self.model.geom_rbound)
        self.bounded_pairs=np.all(finite[self.pair_ids],axis=1)
result={}
for model in ('TsenseI_v3_2','TsenseI_v3_2_withid'):
    p=load_hand_profile(model)
    result[model]={}
    base,bq=run(MinkCollisionGuard,p)
    result[model]['baseline']=base
    for name,cls in [('cache_only',CacheOnly),('bounds_only',BoundsOnly)]:
        row,q=run(cls,p); row['max_joint_output_difference_rad']=float(np.max(abs(q-bq)))
        result[model][name]=row
        print(model,name,row['summary'],'max_difference',row['max_joint_output_difference_rad'],flush=True)
    b=MinkCollisionGuard(p,'left'); c=BoundsOnly(p,'left'); rng=np.random.default_rng(442)
    differences=[]
    for k in range(1000):
        q=rng.uniform(b.lower,b.upper)
        bf=b._feasible(q); cf=c._feasible(q)
        if bf!=cf:
            d=b._distances(q)
            centers=b.configuration.data.geom_xpos[b.pair_ids]
            spheres=np.linalg.norm(centers[:,0]-centers[:,1],axis=1)-b.model.geom_rbound[b.pair_ids].sum(axis=1)
            violations=np.flatnonzero(d < b.margins-1e-7)
            differences.append({'index':k,'q':q.tolist(),'baseline':bf,'bounds':cf,'violations':[{'pair':[b.model.geom(g).name for g in b.pairs[i]],'distance':float(d[i]),'spherical_gap':float(spheres[i]),'margin':float(b.margins[i])} for i in violations]})
            if len(differences)>=4:break
    result[model]['broadphase_reference_differences']=differences
    print('feasibility differences',model,len(differences),differences[:1],flush=True)
(ROOT/'exports/collision_design_20261003/benchmark_variants.json').write_text(json.dumps(result,indent=2))
