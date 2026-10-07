from pathlib import Path
import sys

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
from sync_all_right_keypoints import prepare, sync_all, PROFILE_PATHS


def fixture_profile(root, axis='x'):
    root.mkdir(parents=True)
    for side in ('left', 'right'):
        (root / f'{side}.urdf').write_text(f'<robot><link name="{side}_hand_tip"/></robot>')
    left = {'keypoints': [{'id': i, 'link': 'left_hand_tip', 'local_xyz': [1., 2., 3.],
                          'weight': .7, 'semantic': f'point{i}'} for i in range(21)]}
    (root / 'left.yaml').write_text(yaml.safe_dump(left))
    (root / 'right.yaml').write_text('old: calibration\n')
    profile = {'model_id': root.name, 'mirror_axis': axis, 'sides': {
        side: {'urdf': f'{side}.urdf', 'keypoints': f'{side}.yaml'} for side in ('left', 'right')}}
    path = root / 'profile.yaml'
    path.write_text(yaml.safe_dump(profile))
    return path


@pytest.mark.parametrize('axis,expected', [('x', [-1, 2, 3]), ('y', [1, -2, 3])])
def test_mirror_backup_and_repeated_run(tmp_path, axis, expected):
    profile = fixture_profile(tmp_path / 'model', axis)
    left = (profile.parent / 'left.yaml').read_bytes()
    sync_all([profile])
    right = yaml.safe_load((profile.parent / 'right.yaml').read_text())
    assert len(right['keypoints']) == 21
    assert right['keypoints'][0]['local_xyz'] == expected
    assert right['keypoints'][0]['link'] == 'right_hand_tip'
    assert right['keypoints'][0]['weight'] == .7
    assert (profile.parent / 'left.yaml').read_bytes() == left
    backups = list(profile.parent.glob('right.yaml.bak_*'))
    assert len(backups) == 1 and backups[0].read_text() == 'old: calibration\n'
    sync_all([profile])
    assert list(profile.parent.glob('right.yaml.bak_*')) == backups


def test_dry_run_and_invalid_model_do_not_write(tmp_path):
    first = fixture_profile(tmp_path / 'first')
    second = fixture_profile(tmp_path / 'second')
    sync_all([first, second], dry_run=True)
    assert not list(tmp_path.rglob('*.bak_*'))
    (second.parent / 'left.yaml').write_text('keypoints: []\n')
    with pytest.raises(ValueError, match='21 unique'):
        sync_all([first, second])
    assert (first.parent / 'right.yaml').read_text() == 'old: calibration\n'
    assert not list(tmp_path.rglob('*.bak_*'))


def test_current_registered_models_and_shared_v2():
    plans = {model: (target, data) for model, target, data in map(prepare, PROFILE_PATHS.values())}
    assert plans['TsenseI_v2'][1] is None
    for model, (_, data) in plans.items():
        if model != 'TsenseI_v2':
            assert len(data['keypoints']) == 21
            assert data['mirror_plane'] == ('YZ' if model == 'TsenseI_v3_2' else 'XZ')
