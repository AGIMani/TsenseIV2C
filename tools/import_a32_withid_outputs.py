"""Import a 20-output CAD URDF and derive an equal-angle XZ right mirror.

Run with the track environment. Source frames, limits and metre meshes are
retained; no legacy CAD origin offsets or measured inertias are applied.
"""
from pathlib import Path
import argparse
import copy
import hashlib
import json
import shutil
import sys
import xml.etree.ElementTree as ET

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'src'), str(ROOT / 'tools')]
from generate_right_hand_urdf import _mirror_xyz, _mirror_rpy, _mirror_axis
from TsenseI_retargeting.fk.urdf import UrdfModel
from generate_profile_collision import generate_profile

DEST = ROOT / 'assets/hands/TsenseI_hand_v3_2_withid'
CAD_VERSION = 'middle-dip-standard-20-outputs-v1'


def right_name(value):
    return value.replace('left_hand', 'right_hand').replace('Left_hand', 'Right_hand')


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_xml(path, root):
    ET.indent(root, space='  ')
    ET.ElementTree(root).write(path, encoding='utf-8', xml_declaration=True)


def mirror_stl(source, target):
    # Preserve every source triangle; reflect normals and reverse winding.
    raw = bytearray(source.read_bytes())
    count = int.from_bytes(raw[80:84], 'little')
    if len(raw) != 84 + count * 50:
        raise ValueError(f'Expected binary STL: {source}')
    dtype = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])
    triangles = np.ndarray(count, dtype=dtype, buffer=raw, offset=84)
    triangles['normal'][:, 1] *= -1
    triangles['vertices'][:, :, 1] *= -1
    triangles['vertices'][:] = triangles['vertices'][:, ::-1].copy()
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_bytes(raw)


def install(source):
    source = source.resolve()
    tree = ET.parse(source)
    root = tree.getroot()
    old_path = DEST / 'urdf/TsenseI_hand_left_v3_2_withid.urdf'
    old = UrdfModel(old_path)
    new = UrdfModel(source)
    if set(old.joint_names()) != set(new.joint_names()) or len(new.joint_names()) != 20:
        raise ValueError('Source must retain the twenty A3.2 output joint names')
    if new.missing_meshes():
        raise ValueError(new.missing_meshes())
    if ET.parse(old_path).getroot().get('a32_cad_source') == CAD_VERSION:
        raise ValueError('Already imported; preserve the current annotations before another CAD update')
    source_hashes = {'urdf/' + source.name: sha(source)}
    snapshot = DEST / 'source_20_outputs'
    snapshot.mkdir(exist_ok=True)
    shutil.copy2(source, snapshot / source.name)
    for mesh in root.iter('mesh'):
        src = (source.parent / mesh.get('filename')).resolve()
        target = DEST / 'meshes/left' / src.name
        source_hashes['meshes/left/' + src.name] = sha(src)
        shutil.copy2(src, target)
        snapshot_mesh = snapshot / 'meshes/left' / src.name
        snapshot_mesh.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, snapshot_mesh)
        mirror_stl(src, DEST / 'meshes/right' / right_name(src.name))
        mesh.set('filename', '../meshes/left/' + src.name)
    snapshot_urdf = snapshot / source.name
    snapshot_urdf.write_text(source.read_text(encoding='utf-8').replace('../meshes/left/', 'meshes/left/'), encoding='utf-8')
    # The checked-in snapshot is the reproducible source artifact used by tests;
    # its path rewrite is intentional, so record that snapshot hash rather than
    # the hash of the external file before import.
    source_hashes['urdf/' + source.name] = sha(snapshot_urdf)
    root.set('a32_cad_source', CAD_VERSION)
    # These are derived collision elements, not edits to source kinematics.
    for link in root.findall('link'):
        if link.find('visual') is not None:
            collision = copy.deepcopy(link.find('visual'))
            collision.tag = 'collision'
            for material in collision.findall('material'):
                collision.remove(material)
            link.append(collision)
    write_xml(old_path, root)
    right = copy.deepcopy(root)
    right.set('name', 'TsenseI_hand_v3_2_withid_right')
    for node in right.iter():
        for key in ('name', 'link', 'joint'):
            if node.get(key):
                node.set(key, right_name(node.get(key)))
        if node.tag == 'origin':
            if node.get('xyz'):
                node.set('xyz', _mirror_xyz(node.get('xyz')))
            if node.get('rpy'):
                node.set('rpy', _mirror_rpy(node.get('rpy')))
        elif node.tag == 'axis':
            node.set('xyz', _mirror_axis(node.get('xyz')))
        elif node.tag == 'mesh':
            node.set('filename', '../meshes/right/' + right_name(Path(node.get('filename')).name))
    write_xml(DEST / 'urdf/TsenseI_hand_right_v3_2_withid.urdf', right)
    # Re-express existing zero-pose annotations in the new local link frames.
    # Their old calibration is retained in the caller's backup, never claimed
    # to be calibration of the replacement CAD.
    before = old.forward_kinematics({})
    after = new.forward_kinematics({})
    points = yaml.safe_load((DEST / 'robot_keypoints_21.yaml').read_text())
    for row in points['keypoints']:
        link = row['link']
        xyz = np.r_[row['local_xyz'], 1.]
        row['local_xyz'] = (np.linalg.inv(after[link]) @ before[link] @ xyz)[:3].tolist()
        row['annotation_status'] = 'auto_initialized'
        row['note'] = 'Previous zero-pose position reprojected into replacement CAD; recalibration pending.'
    points.update(quality_status='Draft', generated_by='tools/import_a32_withid_outputs.py', annotation_version=2)
    for side in ('left', 'right'):
        suffix = '' if side == 'left' else '_right'
        data = copy.deepcopy(points)
        data.update(hand_side=side, urdf_path=f'urdf/TsenseI_hand_{side}_v3_2_withid.urdf')
        if side == 'right':
            for row in data['keypoints']:
                row['link'] = right_name(row['link'])
                row['local_xyz'][1] *= -1
        (DEST / f'robot_keypoints_21{suffix}.yaml').write_text(yaml.safe_dump(data, sort_keys=False))
        pose = {right_name(n) if side == 'right' else n: min(hi, max(lo, 0.))
                for n, (lo, hi) in new.joint_limits().items()}
        (DEST / f'q_init{suffix}.yaml').write_text(yaml.safe_dump({'q': pose}, sort_keys=False))
    profile_path = DEST / 'hand_profile.yaml'
    profile = yaml.safe_load(profile_path.read_text())
    profile['notes'] = [
        '20-output middle-DIP CAD imported with source joint frames, limits and metre meshes.',
        'Right generated by XZ reflection; axial vectors use -M so equal q produces mirrored poses.',
        'Keypoints reprojected from previous zero pose; manual recalibration pending.',
        'Collision envelopes rebuilt from new meshes; source has no identified mass or inertia.',
        'A3.2 SDK 1.1.15 and left/right hardware slot mapping retained.']
    profile_path.write_text(yaml.safe_dump(profile, sort_keys=False))
    manifest = {'source': 'TsenseI_hand_20_outputs_description', 'source_sha256': source_hashes,
                'source_urdf_snapshot': 'source_20_outputs/' + source.name,
                'cad_version': CAD_VERSION, 'imported_on': '2026-10-12',
                'geometry_changed': True, 'right_mirror': 'XZ: M=diag(1,-1,1); rotation axes -M; equal q',
                'kinematics': 'Source origins, axes and limits retained without legacy CAD offsets',
                'inertials': 'Source has none; MJCF uses simulation proxies only',
                'keypoints': 'Reprojected at zero pose; manual recalibration pending'}
    (DEST / 'source_mapping.yaml').write_text(yaml.safe_dump(manifest, sort_keys=False))
    generate_profile('TsenseI_v3_2_withid')
    manifest['installed_sha256'] = {str(p.relative_to(DEST)): sha(p)
        for pattern in ('urdf/*.urdf', 'meshes/left/*.STL', 'meshes/right/*.STL', 'mjcf/mjmodel*.xml')
        for p in sorted(DEST.glob(pattern))}
    (DEST / 'source_mapping.yaml').write_text(yaml.safe_dump(manifest, sort_keys=False))
    print(json.dumps({'imported': str(source), 'joints': 20, 'mirror': 'XZ'}))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('urdf', type=Path)
    install(parser.parse_args().urdf)
