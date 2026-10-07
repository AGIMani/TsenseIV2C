from pathlib import Path
import sys,itertools,json
import numpy as np,mujoco
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT),str(ROOT/'src')]
from hand_profiles import load_hand_profile
from online_control.online_motion_track.pipeline_v3_0.collision import StopCollisionGuard
for mid in ('TsenseI_v3_0','TsenseI_v3_2'):
    p=load_hand_profile(mid)
    for side in ('left','right'):
        m=mujoco.MjModel.from_xml_path(str(p.path('mjcf',side)));d=mujoco.MjData(m)
        limits=p.joint_limits(side);initial=p.init_pose(side)
        found=False
        for finger in ('index','mid','ring','pinky','thumb'):
            names=[f'{side}_hand_{finger}_{part}_joint' for part in ('MCP','PIP','DIP') if f'{side}_hand_{finger}_{part}_joint' in limits]
            for fractions in itertools.product((.35,.65,.95),repeat=len(names)):
                q=dict(initial)
                for name,frac in zip(names,fractions):
                    lo,hi=limits[name];q[name]=lo+(hi-lo)*frac
                for name,v in q.items():d.qpos[m.joint(name).qposadr]=v
                mujoco.mj_forward(m,d)
                palm=[(m.geom(c.geom1).name,m.geom(c.geom2).name,float(c.dist)) for c in d.contact if c.dist<-.0002 and ('palm' in m.geom(c.geom1).name or 'palm' in m.geom(c.geom2).name)]
                if palm:
                    guard=StopCollisionGuard(p,side);out=guard.project(q)
                    print(mid,side,finger,fractions,'palmcontacts',palm,'stop_limited',guard.diagnostics.get('limited'),'guardcontacts',guard.diagnostics.get('contacts'),flush=True)
                    (Path(__file__).parent/f'palm_contact_{mid}_{side}.json').write_text(json.dumps({'target':q,'result':out,'contact':palm,'diagnostics':guard.diagnostics},indent=2))
                    found=True;break
            if found:break
        if not found:
            rng=np.random.default_rng(618)
            for attempt in range(10000):
                q={name:rng.uniform(*bounds) if '_thumb_' in name else initial[name] for name,bounds in limits.items()}
                for name,v in q.items():d.qpos[m.joint(name).qposadr]=v
                mujoco.mj_forward(m,d)
                palm=[(m.geom(c.geom1).name,m.geom(c.geom2).name,float(c.dist)) for c in d.contact if c.dist<-.0002 and ('palm' in m.geom(c.geom1).name or 'palm' in m.geom(c.geom2).name)]
                if palm:
                    guard=StopCollisionGuard(p,side);out=guard.project(q)
                    if not any('palm' in c.get('g1','') or 'palm' in c.get('g2','') for c in guard.diagnostics.get('contacts',[])):
                        continue
                    print(mid,side,'random',attempt,'palmcontacts',palm,'stop_limited',guard.diagnostics.get('limited'),'guardcontacts',guard.diagnostics.get('contacts'),flush=True)
                    (Path(__file__).parent/f'palm_contact_{mid}_{side}.json').write_text(json.dumps({'target':q,'result':out,'contact':palm,'diagnostics':guard.diagnostics},indent=2))
                    found=True;break
        assert found,(mid,side)
