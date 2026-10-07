"""Remove the user-marked flat FPC tab projecting from the little-finger side.

The tab occupies y=-31.43..-31.16 mm. Clip only that thin sheet at x=60 mm,
inside the palm shell, retaining the portion concealed inside the assembly.
"""
import json
from pathlib import Path
import shutil

import numpy as np
import trimesh

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / 'TsenseI_A3.0_hand_withid'
MESH = Path('meshes/Left_hand_palm_reconstructed.STL')


def main():
    backup = PACKAGE / 'backups/before_tab_removal'
    for rel in [MESH, Path('meshes/Left_hand_palm_reconstructed_part1.STL'),
                Path('meshes/Left_hand_palm_reconstructed_part2.STL'),
                Path('mujoco/A3_hand.xml'), Path('qa/after.png'), Path('qa/curl.png')]:
        dst = backup / rel
        if not dst.exists():
            dst.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(PACKAGE / rel, dst)
    mesh = trimesh.load_mesh(backup / MESH, process=False)
    triangles = np.asarray(mesh.triangles)
    cut = .060
    sheet = ((triangles[:, :, 1].min(1) >= -.03143) &
             (triangles[:, :, 1].max(1) <= -.03116))
    selected = sheet & (triangles[:, :, 0].max(1) > cut)
    assert selected.sum() > 0
    kept = [triangles[~selected]]
    clipped = []
    for triangle in triangles[selected]:
        polygon = []
        for i in range(3):
            a, b = triangle[i], triangle[(i+1) % 3]
            inside_a, inside_b = a[0] <= cut, b[0] <= cut
            if inside_a:
                polygon.append(a)
            if inside_a != inside_b:
                v = a + (b-a) * ((cut-a[0]) / (b[0]-a[0]))
                v[0] = cut
                polygon.append(v)
        for i in range(1, len(polygon)-1):
            tri = np.array([polygon[0], polygon[i], polygon[i+1]])
            if np.linalg.norm(np.cross(tri[1]-tri[0], tri[2]-tri[0])) > 1e-15:
                clipped.append(tri)
    kept.append(np.asarray(clipped))
    result = np.concatenate(kept)
    output = trimesh.Trimesh(vertices=result.reshape(-1, 3),
                             faces=np.arange(len(result)*3).reshape(-1, 3), process=False)
    output.export(PACKAGE / MESH)
    remaining = result[(result[:, :, 1].min(1) >= -.03143) &
                       (result[:, :, 1].max(1) <= -.03116)]
    assert remaining[:, :, 0].max() <= cut + 1e-12
    report = {'description': 'User-marked protruding palm-side FPC sheet removed',
              'selection_y_m': [-.03143, -.03116], 'clip_x_m': cut,
              'source_faces': len(triangles), 'selected_faces': int(selected.sum()),
              'retained_clipped_triangles': len(clipped), 'output_faces': len(result),
              'untouched_faces': int((~selected).sum()),
              'note': 'Only selected sheet triangles are clipped; new vertices lie on the cut plane inside the shell.'}
    (PACKAGE / 'qa/tab_removal.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
