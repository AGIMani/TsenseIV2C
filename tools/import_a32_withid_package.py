"""Install the supplied raised-thumb left/right CAD package into the web profile.

Build in a staging directory, preserve the old profile in a dated backup, retain
source URDFs verbatim and share their unchanged meshes with the installed models.
Run using .venvs/track/bin/python; never connects to hardware.
"""
from pathlib import Path
from datetime import datetime
import argparse
import hashlib
import json
import shutil
import sys
import tempfile
import xml.etree.ElementTree as ET

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'src'), str(ROOT / 'tools')]
from apply_a32_joint_ranges import apply_urdf_coordinates, write_xml
from a32_kinematics.joint_coordinates import initial_pose
from generate_profile_collision import generate_profile
from TsenseI_retargeting.fk.urdf import UrdfModel

MODEL = 'TsenseI_v3_2_withid'
VERSION = 'raised5mm-noupper-mid-20261027'
DEST = ROOT / 'assets/hands/TsenseI_hand_v3_2_withid'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def install(source):
    source = source.resolve()
    sources = {}
    for side in ('left', 'right'):
        path = source / f'urdf/TsenseI_hand_{side}_v3_2_withid.urdf'
        model = UrdfModel(path)
        old = UrdfModel(DEST / 'urdf' / path.name)
        if set(model.joint_names()) != set(old.joint_names()) or len(model.joint_names()) != 20:
            raise ValueError(f'{side}: expected the twenty existing output joint names')
        if model.missing_meshes():
            raise ValueError(model.missing_meshes())
        thumb = model.joint_by_name[f'{side}_hand_thumb_rotaback_joint2']
        if (thumb.lower, thumb.upper) not in ((-1.74, 0.), (0., 1.74)):
            raise ValueError('Expected rotation2 range [-1.74, 0] or [0, 1.74]')
        sources[side] = path
    stage = Path(tempfile.mkdtemp(prefix='.a32-import-', dir=DEST.parent))
    try:
        shutil.copytree(DEST, stage, dirs_exist_ok=True)
        for directory in ('meshes', 'source_20_outputs', 'source_package'):
            if (stage / directory).exists():
                shutil.rmtree(stage / directory)
        snapshot = stage / 'source_20_outputs'
        snapshot.mkdir()
        hashes = {}
        for side, path in sources.items():
            shutil.copy2(path, snapshot / path.name)
            hashes[f'source_20_outputs/{path.name}'] = sha(path)
            root = ET.parse(path).getroot()
            for mesh in root.iter('mesh'):
                original = (path.parent / mesh.get('filename')).resolve()
                relative = original.relative_to(source)
                if relative.parts[:2] != ('meshes', side):
                    raise ValueError(f'Unexpected mesh location: {relative}')
                target = stage / relative
                target.parent.mkdir(parents=True, exist_ok=True)
                if not target.exists():
                    shutil.copy2(original, target)
                    hashes[str(relative)] = sha(original)
            root.set('a32_cad_source', VERSION)
            write_xml(stage / 'urdf' / path.name, apply_urdf_coordinates(root, model_id=MODEL))
            suffix = '' if side == 'left' else '_right'
            new = UrdfModel(stage / 'urdf' / path.name)
            (stage / f'q_init{suffix}.yaml').write_text(yaml.safe_dump({'q': initial_pose(new.joint_limits())}, sort_keys=False))
            points_path = stage / f'robot_keypoints_21{suffix}.yaml'
            points = yaml.safe_load(points_path.read_text())
            # Keep each hand's independent local annotations attached to its
            # named links so they follow the new CAD, including the raised thumb.
            for row in points['keypoints']:
                row['annotation_status'] = 'auto_initialized'
                row['note'] = 'Previous local link annotation retained on replacement CAD; review required.'
                if (side == 'right' and row['link'] == 'base_link'
                        and ET.parse(DEST / 'urdf' / path.name).getroot().get('a32_cad_source') != VERSION):
                    row['local_xyz'][1] += .008
            points.update(quality_status='Draft', generated_by='tools/import_a32_withid_package.py',
                          annotation_version=3)
            points_path.write_text(yaml.safe_dump(points, sort_keys=False))
        # Preserve supplied configuration as provenance, not executable project policy.
        for directory in ('config', 'docs', 'checks'):
            if (source / directory).is_dir():
                shutil.copytree(source / directory, stage / 'source_package' / directory)
        profile_path = stage / 'hand_profile.yaml'
        profile = yaml.safe_load(profile_path.read_text())
        profile['mirror_plane_offset_m'] = .004
        profile['notes'] = [
            'Supplied raised5mm/noupper_mid left and right CAD; all 367 visual meshes per hand retained.',
            'Right CAD reflects about base Y=4 mm; it is a geometric mirror, not independently measured CAD.',
            'Rotation2 normalized to positive application angles; source may use either signed convention.',
            'Source joint frames and URDF mass/inertia retained; application flexion/swing limits retained.',
            'Each hand retains its local keypoint annotations; review on replacement geometry is pending.',
            'Collision envelopes rebuilt; existing different-finger-only policy and simulation inertias retained.']
        profile_path.write_text(yaml.safe_dump(profile, sort_keys=False))
        generate_profile(MODEL, profile_root=stage)
        manifest = {
            'source': source.name, 'cad_version': VERSION, 'imported_on': '2026-10-27',
            'source_sha256': hashes,
            'source_urdf_snapshots': {side: f'source_20_outputs/{path.name}' for side, path in sources.items()},
            'right_mirror': 'Y=0.004 m; y_right=0.008-y_left; equal application angles',
            'source_joint_signs': {side: {'thumb_rotaback_joint2':
                -1 if UrdfModel(path).joint_by_name[f'{side}_hand_thumb_rotaback_joint2'].lower < 0 else 1}
                for side, path in sources.items()},
            'kinematics': 'Source origins retained; negative rotation2 normalized to positive; application flexion/swing limits retained',
            'inertials': 'Supplied URDF inertials retained; MJCF remains a kinematic collision proxy',
            'keypoints': 'Existing per-hand local coordinates retained; review pending',
            'installed_sha256': {str(p.relative_to(stage)): sha(p)
                for pattern in ('urdf/*.urdf', 'mjcf/mjmodel*.xml') for p in sorted(stage.glob(pattern))},
        }
        (stage / 'source_mapping.yaml').write_text(yaml.safe_dump(manifest, sort_keys=False))
        backups = ROOT.parent / 'model-backups'
        backups.mkdir(exist_ok=True)
        backup = backups / (DEST.name + '-' + datetime.now().strftime('%Y%m%d-%H%M%S-%f'))
        DEST.rename(backup)
        try:
            stage.rename(DEST)
        except BaseException:
            backup.rename(DEST)
            raise
        print(json.dumps({'installed': str(DEST), 'backup': str(backup), 'source_meshes': len(hashes)-2}))
    finally:
        if stage.exists():
            shutil.rmtree(stage)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('package', type=Path)
    install(parser.parse_args().package)
