from pathlib import Path
import json
import sys
import mujoco
ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / 'src')]
from hand_profiles import load_hand_profile, A30_MODEL_IDS, A32_MODEL_IDS
for model_id in (*A32_MODEL_IDS, *A30_MODEL_IDS):
    profile = load_hand_profile(model_id)
    for side in ('left', 'right'):
        model = mujoco.MjModel.from_xml_path(str(profile.path('mjcf', side)))
        data = mujoco.MjData(model)
        for name, value in profile.init_pose(side).items():
            data.qpos[model.joint(name).qposadr] = value
        mujoco.mj_forward(model, data)
        print(model_id, side, model.ngeom, data.ncon,
              [(model.geom(c.geom1).name,model.geom(c.geom2).name,c.dist) for c in data.contact])
        for a in range(model.ngeom):
            for b in range(a + 1, model.ngeom):
                if not ((model.geom_contype[a] & model.geom_conaffinity[b]) or
                        (model.geom_contype[b] & model.geom_conaffinity[a])):
                    continue
                d=mujoco.mj_geomDistance(model,data,a,b,1e6,None)
                if d < .0005:
                    print('  small-distance', model.geom(a).name,model.geom(b).name,d,
                          mujoco.mj_geomDistance(model,data,a,b,.015,None))
