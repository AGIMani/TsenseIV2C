from pathlib import Path
import sys
import numpy as np
import trimesh
from scipy.spatial import ConvexHull
ROOT=Path(__file__).resolve().parents[2]
for path in (ROOT/'assets/hands/TsenseI_hand_v3_2_withid/mjcf/collision_meshes').glob('left_left_hand_*DIP_link.stl'):
    mesh=trimesh.load_mesh(path,process=True)
    hull=ConvexHull(mesh.vertices)
    n=mesh.face_normals
    offset=-(n*mesh.vertices[mesh.faces[:,0]]).sum(axis=1)
    violation=(mesh.vertices@n.T+offset).max()
    scaled=mesh.copy(); scaled.apply_scale(1000)
    print(path.name,'isconvex',mesh.is_convex,'scaled',scaled.is_convex,'max_face_plane_violation_m',violation,
          'convex_volume_difference',abs(hull.volume-mesh.volume),'watertight',mesh.is_watertight)
