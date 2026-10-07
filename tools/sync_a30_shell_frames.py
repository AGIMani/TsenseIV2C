"""Rebase shell joints to bare A3.0 using hinge axes and inter-joint geometry.

Keep STL coordinates; transform visual/collision/inertial origins. The inertia
tensor remains in its original inertial frame, whose origin is also transformed.
"""
import copy
import hashlib
import json
from pathlib import Path
import shutil
import warnings
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from render_a30_shell_qa import origin, transforms

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / 'TsenseI_A3.0_hand_withid'
REL = Path('urdf/A3-2691-1053yw1.urdf')
REFERENCE = ROOT / 'TsenseI_A3.0_hand/urdf/V6 26.8.4.urdf'
BACKUP = PACKAGE / 'backups/before_frame_sync'
PINKY = {'rotaback': 'MCP', 'MCP': 'swing', 'swing': 'PIP', 'PIP': 'DIP'}


def reference_root():
    root = ET.parse(REFERENCE).getroot()
    if root.find("joint[@name='left_hand_pinky_DIP_joint']") is None:
        raise ValueError('Reference URDF must use canonical pinky names')
    return root


def axis(joint):
    return np.fromstring(joint.find('axis').get('xyz'), sep=' ')


def basis(u, v):
    u = u / np.linalg.norm(u)
    v = v - u * np.dot(u, v)
    if np.linalg.norm(v) < 1e-8:
        raise ValueError('Collinear frame vectors')
    v /= np.linalg.norm(v)
    return np.column_stack((u, v, np.cross(u, v)))


def derive_frames(source, reference):
    sj = {j.find('child').get('link'): j for j in source.findall('joint') if j.get('type') != 'fixed'}
    rj = {j.find('child').get('link'): j for j in reference.findall('joint') if j.get('type') != 'fixed'}
    descendants = {j.find('parent').get('link'): name for name, j in rj.items()}
    sign = {n: (-1 if n in ('left_hand_thumb_rota_block_link', 'left_hand_thumb_rotaback_link2') else 1) for n in sj}
    rotations = {}
    for name in rj:
        if name not in descendants:
            continue
        child = descendants[name]
        src, ref = origin(sj[child].find('origin')), origin(rj[child].find('origin'))
        u, v = axis(sj[name]) * sign[name], axis(rj[name])
        u2, v2 = src[:3, :3] @ axis(sj[child]) * sign[child], ref[:3, :3] @ axis(rj[child])
        if np.linalg.norm(np.cross(u, u2)) < 1e-5:
            # Parallel hinges: the inter-axis vector determines the bone zero.
            u2, v2 = src[:3, 3], ref[:3, 3]
        rotations[name] = basis(v, v2) @ basis(u, u2).T
    for name in rj:
        if name not in rotations:
            parent = rj[name].find('parent').get('link')
            rotations[name] = (origin(rj[name].find('origin'))[:3, :3].T @ rotations[parent]
                               @ origin(sj[name].find('origin'))[:3, :3])
    # CAD link origins lie on their hinges. Solve only translations along axes.
    names = list(rj)
    matrix, target, edges = [], [], []
    for name, joint in rj.items():
        parent = joint.find('parent').get('link')
        if parent not in rj:
            continue
        src, ref = origin(sj[name].find('origin')), origin(joint.find('origin'))
        row = np.zeros((3, len(names)))
        row[:, names.index(parent)] = axis(rj[parent])
        row[:, names.index(name)] = -ref[:3, :3] @ axis(joint)
        matrix.extend(row)
        target.extend(ref[:3, 3] - rotations[parent] @ src[:3, 3])
        edges.append(name)
    matrix, target = np.array(matrix), np.array(target)
    scalars, _, rank, _ = np.linalg.lstsq(matrix, target, rcond=None)
    residual = (matrix @ scalars - target).reshape(-1, 3)
    if rank != len(names) or np.linalg.norm(residual, axis=1).max() > .0002:
        raise ValueError('Underdetermined mapping or CAD mismatch exceeding 0.2 mm')
    frames = {}
    for i, name in enumerate(names):
        C = np.eye(4); C[:3, :3] = rotations[name]; C[:3, 3] = axis(rj[name]) * scalars[i]
        if np.linalg.norm(C[:3, :3] @ axis(sj[name]) * sign[name] - axis(rj[name])) > 1e-6:
            raise ValueError(f'Inconsistent axis mapping for {name}')
        frames[name] = C
    ts, tr = transforms(source), transforms(reference)
    A = np.eye(4); A[:3, :3] = Rotation.from_euler('z', 90, degrees=True).as_matrix()
    A[:3, 3] = tr['left_hand_mid_MCP_link'][:3, 3] - A[:3, :3] @ ts['left_hand_mid_MCP_link'][:3, 3]
    frames['base_link'] = A
    frames['Left_hand_palm_link'] = np.linalg.inv(tr['Left_hand_palm_link']) @ A @ ts['Left_hand_palm_link']
    changes = {}
    for name, joint in rj.items():
        parent = joint.find('parent').get('link')
        parent_R = frames[parent][:3, :3] if parent in rj else A[:3, :3]
        old_R = parent_R @ origin(sj[name].find('origin'))[:3, :3]
        new_R = origin(joint.find('origin'))[:3, :3] @ frames[name][:3, :3]
        offset = float(np.dot(Rotation.from_matrix(old_R.T @ new_R).as_rotvec(), axis(sj[name])))
        changes[joint.get('name')] = dict(source_angle_sign=sign[name], source_zero_offset_rad=offset,
                                         lower=float(joint.find('limit').get('lower')),
                                         upper=float(joint.find('limit').get('upper')))
    return frames, dict(global_shell_to_reference=A.tolist(), joints=changes,
                        assembly_residual_mm={n: (v*1000).tolist() for n, v in zip(edges, residual)},
                        max_assembly_residual_mm=float(np.linalg.norm(residual, axis=1).max()*1000))


def write_origin(parent, matrix):
    element = parent.find('origin')
    if element is None:
        element = ET.Element('origin'); parent.insert(0, element)
    with warnings.catch_warnings():
        warnings.simplefilter('ignore', UserWarning)
        rpy = Rotation.from_matrix(matrix[:3, :3]).as_euler('xyz')
    element.set('xyz', ' '.join(f'{v:.16g}' for v in matrix[:3, 3]))
    element.set('rpy', ' '.join(f'{v:.16g}' for v in rpy))


def write_xml(path, root):
    ET.indent(root, space='  ')
    ET.ElementTree(root).write(path, encoding='utf-8', xml_declaration=True)


def synchronize(source_root=None):
    for rel in (REL, Path('mujoco/A3_hand.urdf'), Path('mujoco/A3_hand.xml'), Path('qa/after.png'),
                Path('qa/curl.png'), Path('qa/validation.json'), Path('repair_mapping.json')):
        dst = BACKUP / rel
        if not dst.exists() and (PACKAGE / rel).exists():
            dst.parent.mkdir(parents=True, exist_ok=True); shutil.copy2(PACKAGE / rel, dst)
    source = copy.deepcopy(source_root) if source_root is not None else ET.parse(BACKUP / REL).getroot()
    reference = reference_root()
    frames, report = derive_frames(source, reference)
    output = copy.deepcopy(source)
    sj = {j.get('name'): j for j in source.findall('joint')}
    for joint in list(output.findall('joint')):
        output.remove(joint)
    for joint in reference.findall('joint'):
        result = copy.deepcopy(joint)
        if result.get('type') != 'fixed':
            # Copy angle limits; keep useful effort/velocity, not CAD zeros.
            old_limit = sj[result.get('name')].find('limit')
            for key in ('effort', 'velocity'):
                result.find('limit').set(key, old_limit.get(key))
        output.append(result)
    for link in output.findall('link'):
        for tag in ('visual', 'collision', 'inertial'):
            for element in link.findall(tag):
                write_origin(element, frames[link.get('name')] @ origin(element.find('origin')))
    write_xml(PACKAGE / REL, output)
    write_xml(PACKAGE / 'mujoco/A3_hand.urdf', copy.deepcopy(output))
    config = 'controller_joint_names:\n' + ''.join(f'  - {j.get("name")}\n' for j in output.findall('joint') if j.get('type') != 'fixed')
    (PACKAGE / 'config/joint_names_A3-2691-1053yw1.yaml').write_text(config, encoding='utf-8')
    report.update(reference=str(REFERENCE.relative_to(ROOT)), reference_sha256=hashlib.sha256(REFERENCE.read_bytes()).hexdigest(),
                  mesh_frame_transforms={name: value.tolist() for name, value in frames.items()},
                  mesh_sha256={str(p.relative_to(PACKAGE)): hashlib.sha256(p.read_bytes()).hexdigest()
                               for p in sorted((PACKAGE / 'meshes').glob('*.STL'))},
                  method='Hinge-axis/inter-joint-vector alignment with hinge-line translation constraints',
                  note='Origin/axis/topology/lower/upper follow functionally mapped bare CAD. STL data, removed tab, masses and inertial-frame tensors are retained.')
    (PACKAGE / 'frame_sync.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    print('Synchronized 20 joints; maximum CAD assembly residual (mm):', report['max_assembly_residual_mm'])
    for name, change in report['joints'].items():
        if 'thumb' in name:
            print(name, change)


if __name__ == '__main__':
    synchronize()
