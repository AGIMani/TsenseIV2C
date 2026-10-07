from pathlib import Path
import json, sys, importlib.util
import numpy as np
import mujoco
ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT/'src')]
from hand_profiles import load_hand_profile
spec=importlib.util.spec_from_file_location('baseline','/tmp/hand-collision-baseline/mink_guard.py')
mod=importlib.util.module_from_spec(spec); spec.loader.exec_module(mod)
p=load_hand_profile('TsenseI_v3_2_withid')
g=mod.MinkCollisionGuard(p,'left')
rows=json.loads((ROOT/'exports/collision_design_20261003/benchmark_variants.json').read_text())['TsenseI_v3_2_withid']['broadphase_reference_differences']
for row in rows:
 q=np.array(row['q']); g._update(q)
 for v in row['violations']:
  a,b=[g.model.geom(n).id for n in v['pair']]
  print('PAIR',v)
  for i in (a,b):
   mid=g.model.geom_dataid[i]; vs=g.model.mesh_vert[g.model.mesh_vertadr[mid]:g.model.mesh_vertadr[mid]+g.model.mesh_vertnum[mid]]
   print('bound',g.model.geom_rbound[i], 'maxvertex',np.linalg.norm(vs,axis=1).max(),'range',vs.min(0),vs.max(0))
  for native in (True,False):
   g.model.opt.disableflags=0 if native else int(mujoco.mjtDisableBit.mjDSBL_NATIVECCD)
   for horizon in (.0005,.001,.005,.015,.025,.05,.1,1):
    ft=np.zeros(6); d=mujoco.mj_geomDistance(g.model,g.configuration.data,a,b,horizon,ft)
    print('native',native,'horizon',horizon,'dist',d,'fromto',ft.tolist())
  for tol in (1e-4,1e-6,1e-8,1e-10,1e-12):
   g.model.opt.disableflags=0; g.model.opt.ccd_tolerance=tol
   print('tol',tol,'dist',mujoco.mj_geomDistance(g.model,g.configuration.data,a,b,.05,None))
  for iters in (35,50,100,200,1000):
   g.model.opt.disableflags=0; g.model.opt.ccd_iterations=iters
   print('iters',iters,'dist',mujoco.mj_geomDistance(g.model,g.configuration.data,a,b,.05,None))
  g.model.opt.disableflags=0
  mujoco.mj_forward(g.model,g.configuration.data)
  print('forward',mujoco.mj_geomDistance(g.model,g.configuration.data,a,b,.05,None))
  g.model.opt.disableflags=int(mujoco.mjtDisableBit.mjDSBL_MULTICCD)
  print('single contact',mujoco.mj_geomDistance(g.model,g.configuration.data,a,b,.05,None))
  for i in (a,b):g.model.mesh_graphadr[g.model.geom_dataid[i]]=-1
  print('exhaustive support',mujoco.mj_geomDistance(g.model,g.configuration.data,a,b,.05,None))
