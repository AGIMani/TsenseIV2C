from pathlib import Path
import sys
ROOT = Path(__file__).resolve().parents[3]
sys.path[:0]=[str(ROOT),str(ROOT/'src')]
from hand_profiles import load_hand_profile
import mujoco
for mid in ('TsenseI_v3_0','TsenseI_v3_0_withid','TsenseI_v3_2','TsenseI_v3_2_withid'):
    p=load_hand_profile(mid); f=p.create_collision_filter('left'); m=f.model
    print(mid,'mirror',p.mirror_axis,'ngeom',m.ngeom,'ncon',f.data.ncon,'npair',m.npair)
    for i in range(m.ngeom):
        print(i,mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,i),int(m.geom_type[i]),int(m.geom_contype[i]),int(m.geom_conaffinity[i]))
