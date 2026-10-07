"""Mirror each registered model's own left 21-point calibration to its right."""
from __future__ import annotations

import argparse
import copy
from datetime import datetime
import math
from pathlib import Path
import os
import shutil
import sys
import tempfile
import xml.etree.ElementTree as ET

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from hand_profiles import PROFILE_PATHS


def right_name(name):
    return name.replace('left_hand', 'right_hand').replace('Left_hand', 'Right_hand')


def prepare(profile_path):
    profile_path = Path(profile_path)
    profile = yaml.safe_load(profile_path.read_text(encoding='utf-8'))
    root = profile_path.parent
    left, right = (profile['sides'][side] for side in ('left', 'right'))
    source = (root / left['keypoints']).resolve()
    target = (root / right['keypoints']).resolve()
    model = profile['model_id']
    if source == target:
        if right.get('solver_side') != 'left':
            raise ValueError(f'{model}: shared keypoints without a shared left solver')
        if not source.is_file():
            raise ValueError(f'{model}: shared calibration is missing: {source}')
        return model, target, None
    axis = profile.get('mirror_axis')
    if axis not in ('x', 'y', 'z'):
        raise ValueError(f'{model}: missing or invalid mirror_axis')
    data = yaml.safe_load(source.read_text(encoding='utf-8'))
    rows = data['keypoints']
    if len(rows) != 21 or {row['id'] for row in rows} != set(range(21)):
        raise ValueError(f'{model}: expected exactly 21 unique keypoint IDs (0..20)')
    links = []
    for side in (left, right):
        urdf = ET.parse(root / side['urdf']).getroot()
        links.append({node.attrib['name'] for node in urdf.findall('link')})
    result = copy.deepcopy(data)
    for row in result['keypoints']:
        if row['link'] not in links[0] or right_name(row['link']) not in links[1]:
            raise ValueError(f"{model}: invalid link mapping for point {row['id']}: {row['link']}")
        xyz = row['local_xyz']
        if (len(xyz) != 3 or any(isinstance(x, bool) or not isinstance(x, (int, float))
                                or not math.isfinite(x) for x in xyz)):
            raise ValueError(f"{model}: invalid coordinates for point {row['id']}")
        row['link'] = right_name(row['link'])
        row['local_xyz'] = [(-x if i == 'xyz'.index(axis) else x) for i, x in enumerate(xyz)]
    if 'base_link' in result:
        result['base_link'] = right_name(result['base_link'])
    result.update(hand_side='right', urdf_path=right['urdf'],
                  mirrored_from=left['keypoints'], mirror_plane={'x': 'YZ', 'y': 'XZ', 'z': 'XY'}[axis])
    return model, target, result


def sync_all(profile_paths, *, dry_run=False):
    # Validate every model before writing the first file.
    plans = [prepare(path) for path in profile_paths]
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    for model, target, data in plans:
        if data is None:
            print(f'[SHARED] {model}: right solver already uses left calibration')
            continue
        if target.exists() and yaml.safe_load(target.read_text(encoding='utf-8')) == data:
            print(f'[UNCHANGED] {model}: {target}')
            continue
        if dry_run:
            print(f'[WOULD SYNC] {model}: 21 points, mirror {data["mirror_plane"]} -> {target}')
            continue
        backup = target.with_name(target.name + '.bak_' + stamp)
        if target.exists():
            shutil.copy2(target, backup)
            print(f'[BACKUP] {backup}')
        target.parent.mkdir(parents=True, exist_ok=True)
        payload = yaml.safe_dump(data, sort_keys=False, allow_unicode=True)
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', newline='\n',
                                             dir=target.parent, suffix='.tmp', delete=False) as stream:
                temporary = Path(stream.name)
                stream.write(payload)
            os.replace(temporary, target)
        finally:
            if temporary is not None and temporary.exists():
                temporary.unlink()
        print(f'[SYNCED] {model}: 21 points, mirror {data["mirror_plane"]} -> {target}')
    print('Check complete; no files changed.' if dry_run else 'Synchronization complete. Restart local services; redeploy node0 to update it.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dry-run', action='store_true', help='Validate all models and preview changes without writing')
    args = parser.parse_args()
    try:
        sync_all(PROFILE_PATHS.values(), dry_run=args.dry_run)
    except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError, ET.ParseError) as exc:
        parser.exit(1, f'Synchronization failed: {exc}\n')


if __name__ == '__main__':
    main()
