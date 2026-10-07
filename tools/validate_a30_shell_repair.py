"""Validate shell naming, kinematic preservation, mesh partitioning and MuJoCo."""
import importlib.util
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import trimesh

from render_a30_shell_qa import transforms, origin
from repair_a30_palm_mesh import stats

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / 'TsenseI_A3.0_hand_withid'
BACKUP = PACKAGE / 'backups/before_shell_repair'
REL = Path('urdf/A3-2691-1053yw1.urdf')


def main():
    old = ET.parse(BACKUP / REL).getroot()
    new = ET.parse(PACKAGE / REL).getroot()
    mapping = json.loads((PACKAGE / 'repair_mapping.json').read_text(encoding='utf-8'))
    canonical = ET.parse(ROOT / 'assets/hands/TsenseI_hand_v3_0/urdf/TsenseI_hand_left_v3_0.urdf').getroot()
    sync_file = PACKAGE / 'frame_sync.json'
    sync = json.loads(sync_file.read_text()) if sync_file.exists() else None
    if sync:
        from sync_a30_shell_frames import reference_root, REFERENCE
        canonical = reference_root()
        assert hashlib.sha256(REFERENCE.read_bytes()).hexdigest() == sync['reference_sha256']
        for path, digest in sync['mesh_sha256'].items():
            assert hashlib.sha256((PACKAGE / path).read_bytes()).hexdigest() == digest
    joints = {j.get('name'): j for j in new.findall('joint') if j.get('type') != 'fixed'}
    assert set(joints) == {j.get('name') for j in canonical.findall('joint') if j.get('type') != 'fixed'}
    assert len(new.findall('link')) == 22 and len(new.findall('joint')) == 21
    assert len({l.get('name') for l in new.findall('link')}) == 22
    links = mapping['link_mapping'] | {'base_link': 'Left_hand_palm_link'}
    rng = np.random.default_rng(20260921)
    maximum_error = 0.0
    if sync:
        links = {l.get('name'): l.get('name') for l in canonical.findall('link')}
        for ref_joint in canonical.findall('joint'):
            actual = new.find(f"joint[@name='{ref_joint.get('name')}']")
            assert actual is not None and actual.attrib == ref_joint.attrib
            for tag in ('origin', 'axis', 'parent', 'child'):
                assert actual.find(tag).attrib == ref_joint.find(tag).attrib
            if actual.get('type') != 'fixed':
                for key in ('lower', 'upper'):
                    assert actual.find('limit').get(key) == ref_joint.find('limit').get(key)
    for trial in range(101):
        if sync:
            values = {n: 0.0 if trial == 0 else float(rng.uniform(float(j.find('limit').get('lower')), float(j.find('limit').get('upper')))) for n, j in joints.items()}
            before, after = transforms(canonical, values), transforms(new, values)
        else:
            values = {item['old']: 0.0 if trial == 0 else float(rng.uniform(-.3, 1.2)) for item in mapping['joint_mapping']}
            renamed = {item['new']: values[item['old']] for item in mapping['joint_mapping']}
            before, after = transforms(old, values), transforms(new, renamed)
        for src, dst in links.items():
            error = float(np.max(np.abs(before[src] - after[dst])))
            maximum_error = max(maximum_error, error)
            assert error < 1e-12, (trial, src, dst, error)
    if sync:
        before_frames = ET.parse(PACKAGE / 'backups/before_frame_sync' / REL).getroot()
        for link in before_frames.findall('link'):
            name = link.get('name'); C = np.array(sync['mesh_frame_transforms'][name])
            actual = new.find(f"link[@name='{name}']")
            for tag in ('inertial', 'visual', 'collision'):
                assert len(link.findall(tag)) == len(actual.findall(tag))
                for src, dst in zip(link.findall(tag), actual.findall(tag)):
                    np.testing.assert_allclose(origin(dst.find('origin')), C @ origin(src.find('origin')), atol=1e-12)
                    if tag == 'inertial':
                        assert src.find('mass').attrib == dst.find('mass').attrib
                        assert src.find('inertia').attrib == dst.find('inertia').attrib
                    else:
                        assert src.find('geometry/mesh').attrib == dst.find('geometry/mesh').attrib
        for n, j in joints.items():
            source_joint = before_frames.find(f"joint[@name='{n}']")
            C = np.array(sync['mesh_frame_transforms'][j.find('child').get('link')])
            a = np.fromstring(j.find('axis').get('xyz'), sep=' ')
            source_a = np.fromstring(source_joint.find('axis').get('xyz'), sep=' ')
            np.testing.assert_allclose(C[:3, :3] @ source_a * sync['joints'][n]['source_angle_sign'], a, atol=1e-6)
            assert np.linalg.norm(np.cross(C[:3, 3], a)) < 1e-10
    else:
        for src, dst in links.items():
            a = old.find(f"link[@name='{src}']/inertial")
            b = new.find(f"link[@name='{dst}']/inertial")
            assert [(n.tag, n.attrib) for n in a.iter()] == [(n.tag, n.attrib) for n in b.iter()]
    for mesh in new.findall('.//mesh'):
        path = (PACKAGE / REL).parent / mesh.get('filename')
        assert path.is_file(), path
        geometry = trimesh.load_mesh(path, process=False)
        assert np.isfinite(geometry.vertices).all()
        assert len(geometry.faces) <= 180000
    full = trimesh.load_mesh(PACKAGE / 'meshes/Left_hand_palm_reconstructed.STL', process=False)
    palm = new.find("link[@name='Left_hand_palm_link']")
    parts = [trimesh.load_mesh((PACKAGE / REL).parent / m.get('filename'), process=False)
             for m in palm.findall('visual/geometry/mesh')]
    assert np.array_equal(np.concatenate([m.triangles for m in parts]), full.triangles)
    original_mesh = trimesh.load_mesh(BACKUP / 'meshes/base_link.STL')
    full_processed = trimesh.load_mesh(PACKAGE / 'meshes/Left_hand_palm_reconstructed.STL')
    original_points = set(map(tuple, original_mesh.vertices))
    tab_report_path = PACKAGE / 'qa/tab_removal.json'
    tab_report = json.loads(tab_report_path.read_text()) if tab_report_path.exists() else None
    new_vertices = [p for p in full_processed.vertices if tuple(p) not in original_points]
    if tab_report:
        lo, hi = tab_report['selection_y_m']; cut = tab_report['clip_x_m']
        assert all(abs(p[0] - cut) < 1e-8 and lo <= p[1] <= hi for p in new_vertices)
        before_tab = trimesh.load_mesh(PACKAGE / 'backups/before_tab_removal/meshes/Left_hand_palm_reconstructed.STL', process=False)
        tri = np.asarray(before_tab.triangles)
        selected = ((tri[:, :, 1].min(1) >= lo) & (tri[:, :, 1].max(1) <= hi) &
                    (tri[:, :, 0].max(1) > cut))
        unchanged = tri[~selected]
        assert np.array_equal(full.triangles[:len(unchanged)], unchanged)
        final_tri = np.asarray(full.triangles)
        sheet = (final_tri[:, :, 1].min(1) >= lo) & (final_tri[:, :, 1].max(1) <= hi)
        assert final_tri[sheet, :, 0].max() <= cut + 1e-8
    else:
        assert not new_vertices
    model = mujoco.MjModel.from_xml_path(str(PACKAGE / 'mujoco/A3_hand.xml'))
    assert model.nq == model.nv == model.nu == 20
    actual_names = {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(model.njnt)}
    assert actual_names == set(joints)
    spec = importlib.util.spec_from_file_location('shell_simulate', PACKAGE / 'mujoco/simulate.py')
    simulation = importlib.util.module_from_spec(spec); spec.loader.exec_module(simulation)
    data = mujoco.MjData(model)
    maximum_anchor_error = maximum_axis_error = 0.0
    if sync:
        for trial in range(101):
            values = {n: 0.0 if trial == 0 else float(rng.uniform(float(j.find('limit').get('lower')), float(j.find('limit').get('upper')))) for n, j in joints.items()}
            ref_tf = transforms(canonical, values)
            for n, value in values.items():
                data.qpos[int(model.joint(n).qposadr[0])] = value
            mujoco.mj_forward(model, data)
            for n, joint in joints.items():
                index = int(model.joint(n).id)
                expected = ref_tf[joint.find('child').get('link')]
                a = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
                maximum_anchor_error = max(maximum_anchor_error, float(np.linalg.norm(data.xanchor[index] - expected[:3, 3])))
                maximum_axis_error = max(maximum_axis_error, float(np.linalg.norm(data.xaxis[index] - expected[:3, :3] @ a)))
        assert maximum_anchor_error < 1e-10 and maximum_axis_error < 1e-10
    for t in np.linspace(0, 5, 101):
        data.qpos[:] = simulation.grasp_qpos(float(t), model)
        mujoco.mj_forward(model, data)
        assert np.isfinite(data.xpos).all()
        for name in joints:
            j = model.joint(name)
            q = float(data.qpos[int(j.qposadr[0])])
            if '_swing_' in name:
                assert q == 0.0
            assert j.range[0] <= q <= j.range[1]
    report = {'moving_joints': 20, 'links': 22, 'fk_poses_checked': 101,
              'fk_reference': 'bare A3.0' if sync else 'original shell',
              'max_fk_matrix_difference': maximum_error,
              'inertial_physical_properties_preserved': True, 'all_mesh_paths_resolve': True,
              'all_limits_and_joint_frames_match_reference': bool(sync),
              'mujoco_reference_poses_checked': 101 if sync else 0,
              'mujoco_max_anchor_error_m': maximum_anchor_error,
              'mujoco_max_axis_vector_error': maximum_axis_error,
              'palm_parts_faces': [len(p.faces) for p in parts],
              'partition_preserves_every_triangle': True,
              'palm_vertices_verified': True,
              'tab_removed': bool(tab_report),
              'new_cut_plane_vertices': len(new_vertices),
              'palm_before': stats(original_mesh), 'palm_after': stats(full_processed),
              'mujoco_nq_nv_nu': [model.nq, model.nv, model.nu],
              'mujoco_grasp_poses_checked': 101,
              'limitation': 'Residual open/nonmanifold edges remain. This is a conservative surface reconstruction, not a watertight CAD restoration.'}
    path = PACKAGE / 'qa/validation.json'
    path.write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
