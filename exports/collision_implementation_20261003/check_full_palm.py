from pathlib import Path
from dataclasses import replace
import sys,copy,json
import xml.etree.ElementTree as ET
import numpy as np
import mujoco,yaml
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT),str(ROOT/'src'),str(ROOT/'tools')]
from hand_profiles import load_hand_profile
from generate_profile_collision import build_mjcf
from TsenseI_retargeting.fk.urdf import UrdfModel
from online_control.online_motion_track.mink_guard import MinkCollisionGuard
OUT=Path(__file__).parent
for mid in ('TsenseI_v3_0','TsenseI_v3_2'):
    profile=load_hand_profile(mid)
    geom=yaml.safe_load((profile.root/'collision_geometry.yaml').read_text())
    for g in geom['geoms']:
        if g.get('finger'):continue
        g.update(type='mesh',name=g['link']+'_collision',region='palm',
                 mesh_files={s:f'collision_meshes/{s}_{s.title()}_hand_palm_link.stl' for s in ('left','right')})
        g.pop('size',None);g.pop('pos',None)
    for side in ('left','right'):
        root=build_mjcf(UrdfModel(profile.path('urdf',side)),geom,side,mid)
        for m in root.findall('asset/mesh'):
            m.set('file',str(profile.root/'mjcf'/m.get('file')))
        path=OUT/f'{mid}_{side}_full_palm.xml';ET.ElementTree(root).write(path)
        sides=copy.deepcopy(profile.sides);sides[side]['mjcf']=str(path)
        p=replace(profile,sides=sides)
        model=mujoco.MjModel.from_xml_path(str(path));data=mujoco.MjData(model)
        q=p.init_pose(side)
        for name,value in q.items():data.qpos[model.joint(name).qposadr]=value
        mujoco.mj_forward(model,data)
        print(mid,side,'init_contacts',data.ncon,[(model.geom(c.geom1).name,model.geom(c.geom2).name,c.dist) for c in data.contact],flush=True)
        try:
            guard=MinkCollisionGuard(p,side)
            ds=guard.distances(q)
            palm=[i for i,pair in enumerate(guard.pairs) if any('palm' in model.geom(g).name for g in pair)]
            print('mink init OK, minimum palm distance',min(ds[i] for i in palm),'pairs',len(guard.pairs),flush=True)
        except Exception as exc:print('mink init FAILED',repr(exc),flush=True)
