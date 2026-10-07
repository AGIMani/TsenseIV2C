import sys,json
from pathlib import Path
import numpy as np,mujoco
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT),str(ROOT/'src')]
from hand_profiles import load_hand_profile
from online_control.online_motion_track.mink_guard import MinkCollisionGuard
p=load_hand_profile('TsenseI_v3_2'); g=MinkCollisionGuard(p,'right'); rng=np.random.default_rng(731)
for frame in range(50):
 pose={n:rng.uniform(lo,hi) for n,(lo,hi) in p.joint_limits('right').items()}
 before=g.q.copy();g.project(pose)
 for t in np.linspace(0,1,11):
  q=before+t*(g.q-before);d=g._distances(q)
  if d.min() < -1e-6:
   g._update(q);mujoco.mj_collision(g.model,g.configuration.data)
   data={'frame':frame,'t':float(t),'qstart':before.tolist(),'qend':g.q.tolist(),'qbad':q.tolist(),'distance':float(d.min()),'pair':[g.model.geom(x).name for x in g.pairs[int(d.argmin())]],'max_delta':float(np.max(abs(g.q-before))),'contacts':[{'names':[g.model.geom(int(c.geom1)).name,g.model.geom(int(c.geom2)).name],'distance':float(c.dist)} for c in g.configuration.data.contact]}
   print(json.dumps(data,indent=2));(ROOT/'exports/collision_implementation_20261003/path_regression.json').write_text(json.dumps(data,indent=2));sys.exit(1)
