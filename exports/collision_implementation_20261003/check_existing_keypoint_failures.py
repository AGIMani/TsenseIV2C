"""Confirm unrelated keypoint mirror assertions already fail on saved assets."""
from pathlib import Path
from dataclasses import replace
import importlib.util
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / 'src'), str(ROOT / 'tools')]
from hand_profiles import load_hand_profile

def baseline_profile(name):
    profile = load_hand_profile(name)
    return replace(profile, root=Path('/tmp/hand-collision-baseline/hands') / profile.root.name)

for filename, function in [
    ('test_hand_v3_0_withid', 'test_fresh_landmarks_and_mirrored_fk'),
    ('test_a32_models_integration', 'test_id_right_and_keypoints_are_equal_angle_xz_mirrors'),
]:
    spec = importlib.util.spec_from_file_location(filename, ROOT / 'tests' / f'{filename}.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.load_hand_profile = baseline_profile
    try:
        getattr(module, function)()
    except AssertionError:
        print(f'CONFIRMED pre-existing on original assets: {filename}::{function}')
    else:
        raise RuntimeError(f'Unexpected baseline success: {function}')
