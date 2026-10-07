import sys
from pathlib import Path
import numpy as np, mujoco
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT),str(ROOT/'src')]
from hand_profiles import load_hand_profile
from online_control.online_motion_track.mink_guard import MinkCollisionGuard
from online_control.online_motion_track.collision_queries import ConservativePairBounds
class Probe(MinkCollisionGuard):
 def _feasible(self,q):return True
for model in ['TsenseI_v3_0','TsenseI_v3_0_withid','TsenseI_v3_2','TsenseI_v3_2_withid']:
 for side in ('left','right'):
  g=Probe(load_hand_profile(model),side)
  d=g._distances(g.q); bounds=ConservativePairBounds(g.model,g.pairs).lower_bounds(g.configuration.data)
  print(model,side,'min_distance',d.min(),'min_bound',bounds.min())
  for i in np.flatnonzero(d<g.margins-1e-7):
   a,b=g.pairs[i]
   print('bad',g.model.geom(a).name,g.model.geom(b).name,'d',d[i],'bound',bounds[i])
