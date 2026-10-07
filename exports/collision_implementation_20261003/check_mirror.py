from pathlib import Path
import sys
import numpy as np
from scipy.spatial import cKDTree
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT),str(ROOT/'src'),str(ROOT/'tools')]
from collision_outer_hull import source_points
from TsenseI_retargeting.fk.urdf import UrdfModel
from hand_profiles import load_hand_profile,A30_MODEL_IDS,A32_MODEL_IDS
for mid in (*A30_MODEL_IDS,*A32_MODEL_IDS):
    p=load_hand_profile(mid)
    left,right=[UrdfModel(p.path('urdf',s)) for s in ('left','right')]
    sign=np.array([-1,1,1] if p.mirror_axis=='x' else [1,-1,1])
    ds=[]
    for link in left.links:
        if not left.visual_meshes_by_link().get(link):continue
        a=source_points(left,link)
        rlink=link.replace('left_hand','right_hand').replace('Left_hand','Right_hand')
        b=source_points(right,rlink)*sign
        ds.append(cKDTree(a).query(b)[0].max())
    print(mid,max(ds),p.mirror_axis)
