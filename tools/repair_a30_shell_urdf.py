"""Apply A3.0 functional names to the shell export, with immutable backups.

The initial repair keeps source frames. Once frame synchronization has been
enabled, regeneration reapplies it and retains the repaired, trimmed palm mesh.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET
import trimesh

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / 'TsenseI_A3.0_hand_withid'
URDF = Path('urdf/A3-2691-1053yw1.urdf')
REFERENCE = ROOT / 'TsenseI_A3.0_hand/urdf/V6 26.8.4.urdf'


def names():
    joints = {}
    for digit, finger in [('2', 'index'), ('3', 'mid'), ('4', 'ring'), ('5', 'pinky')]:
        for stage, part in enumerate(('MCP', 'swing', 'PIP', 'DIP'), 1):
            joints[f'{digit}{stage}'] = f'left_hand_{finger}_{part}_joint'
    joints.update({'11': 'left_hand_thumb_rota_block_joint',
                   '12': 'left_hand_thumb_rotaback_joint2',
                   '13': 'left_hand_thumb_MCP_joint',
                   '14': 'left_hand_thumb_PIP_joint'})
    links = {f'Empty_Link{old}': new.replace('_joint', '_link') for old, new in joints.items()}
    return joints, links


def write_xml(path, root):
    ET.indent(root, space='  ')
    ET.ElementTree(root).write(path, encoding='utf-8', xml_declaration=True)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--palm-mesh', type=Path, help='Optional reconstructed palm STL to use')
    args = p.parse_args()
    repaired_palm = PACKAGE / 'meshes/Left_hand_palm_reconstructed.STL'
    if args.palm_mesh is None and repaired_palm.exists():
        args.palm_mesh = repaired_palm
    backup = PACKAGE / 'backups/before_shell_repair'
    tracked = [URDF, Path('meshes/base_link.STL'), Path('mujoco/A3_hand.urdf'),
               Path('mujoco/A3_hand.xml'), Path('mujoco/simulate.py'),
               Path('config/joint_names_A3-2691-1053yw1.yaml')]
    for rel in tracked:
        dst = backup / rel
        if not dst.exists() and (PACKAGE / rel).exists():
            dst.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(PACKAGE / rel, dst)
    original = ET.parse(backup / URDF).getroot()
    root = copy.deepcopy(original)
    root.set('name', 'tsense_a3_0_hand_with_shell')
    joint_names, link_names = names()
    ref = ET.parse(REFERENCE).getroot()
    # The reference CAD's little finger labels are shifted one stage.
    pinky = {'rotaback': 'MCP', 'MCP': 'swing', 'swing': 'PIP', 'PIP': 'DIP'}
    limits = {}
    for joint in ref.findall('joint'):
        name = joint.get('name')
        if name.startswith('left_hand_pinky_'):
            part = name.removeprefix('left_hand_pinky_').removesuffix('_joint')
            name = f'left_hand_pinky_{pinky[part]}_joint'
        if joint.find('limit') is not None:
            limits[name] = joint.find('limit').attrib

    changes = []
    for joint in root.findall('joint'):
        old = joint.get('name'); new = joint_names[old]
        change = {'old': old, 'new': new, 'old_limits': dict(joint.find('limit').attrib)}
        joint.set('name', new)
        # Correct the swapped functional ranges of the first two finger joints.
        # Thumb zero poses differ, so copying thumb limits/signs would be unsafe.
        if not old.startswith('1'):
            for key in ('lower', 'upper'):
                joint.find('limit').set(key, limits[new][key])
        change['new_limits'] = dict(joint.find('limit').attrib)
        changes.append(change)
    for node in root.iter():
        for key in ('name', 'link', 'joint'):
            if node.get(key) in link_names:
                node.set(key, link_names[node.get(key)])
    # Keep the original root frame and complete palm mass exactly once.
    palm = root.find("link[@name='base_link']")
    palm.set('name', 'Left_hand_palm_link')
    for parent in root.findall('joint/parent'):
        if parent.get('link') == 'base_link':
            parent.set('link', 'Left_hand_palm_link')
    root.insert(0, ET.Element('link', name='base_link'))
    fixed = ET.Element('joint', name='Left_hand_palm_joint', type='fixed')
    ET.SubElement(fixed, 'origin', xyz='0 0 0', rpy='0 0 0')
    ET.SubElement(fixed, 'parent', link='base_link')
    ET.SubElement(fixed, 'child', link='Left_hand_palm_link')
    root.insert(2, fixed)
    for link in root.findall('link'):
        for material in link.findall('visual/material'):
            material.set('name', link.get('name') + '_material')
    if args.palm_mesh:
        target = PACKAGE / 'meshes/Left_hand_palm_reconstructed.STL'
        if args.palm_mesh.resolve() != target.resolve():
            shutil.copy2(args.palm_mesh, target)
        surface = trimesh.load_mesh(target, process=False)
        # Each MuJoCo mesh asset must have fewer than 200,000 faces. Partition
        # the full triangle list; never discard every other face or decimate it.
        meshes = []
        for index, start in enumerate(range(0, len(surface.faces), 180000), 1):
            part = target.with_name(f'Left_hand_palm_reconstructed_part{index}.STL')
            trimesh.Trimesh(vertices=surface.vertices,
                            faces=surface.faces[start:start + 180000], process=False).export(part)
            meshes.append(part)
        for tag in ('visual', 'collision'):
            template = copy.deepcopy(palm.find(tag))
            palm.remove(palm.find(tag))
            for index, part in enumerate(meshes, 1):
                node = copy.deepcopy(template)
                node.set('name', f'palm_{tag}_{index}')
                node.find('geometry/mesh').set('filename', '../meshes/' + part.name)
                palm.append(node)
    write_xml(PACKAGE / URDF, root)
    write_xml(PACKAGE / 'mujoco/A3_hand.urdf', copy.deepcopy(root))
    config = 'controller_joint_names:\n' + ''.join(f'  - {j.get("name")}\n' for j in root.findall('joint') if j.get('type') != 'fixed')
    (PACKAGE / 'config/joint_names_A3-2691-1053yw1.yaml').write_text(config, encoding='utf-8')
    report = {'reference': str(REFERENCE.relative_to(ROOT)), 'joint_mapping': changes,
              'link_mapping': link_names, 'palm': 'base_link -> fixed Left_hand_palm_link (identity)',
              'preserved': ['all original joint origins and axes', 'inertials', 'thumb limits', 'finger meshes'],
              'note': 'Shell frames/zero poses differ from bare CAD. Same names do not make calibrated poses interchangeable.',
              'backup_sha256': {str(rel): hashlib.sha256((backup / rel).read_bytes()).hexdigest()
                                for rel in tracked if (backup / rel).exists()}}
    (PACKAGE / 'repair_mapping.json').write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
    if (PACKAGE / 'frame_sync.json').exists():
        from sync_a30_shell_frames import synchronize
        synchronize(source_root=root)
        report['frame_synchronization'] = 'frame_sync.json'
        report['preserved'] = ['STL data including palm repair and tab removal', 'masses and inertia tensors in rebased inertial frames']
        report['note'] = 'Current frames and all angle limits follow the bare A3.0 CAD. Original name mapping remains here; current frame transforms and ranges are in frame_sync.json.'
        (PACKAGE / 'repair_mapping.json').write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
    print('Updated', PACKAGE / URDF)


if __name__ == '__main__':
    main()
