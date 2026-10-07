"""Preset anatomy targets, unchanged bone lengths and model/mirror independence."""
from pathlib import Path
import sys

import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'src'), str(ROOT / 'tools')]
from hand_profiles import load_hand_profile
from keypoint_pose_presets import COMMON_PRESETS, kapandji_target, make_preset, model_flexion_hint

PRESETS = [key for key, _ in COMMON_PRESETS] + [f'kapandji_{i}' for i in range(1, 11)]
EDGES = [(0, base) for base in (1, 5, 9, 13, 17)] + [
    (index, index + 1) for base in (1, 5, 9, 13, 17) for index in range(base, base + 3)]


@pytest.fixture(scope='module', params=[(model, side) for model in ('TsenseI_v3_2', 'TsenseI_v3_2_withid')
                                       for side in ('left', 'right')])
def skeleton(request):
    model, side = request.param
    profile = load_hand_profile(model)
    solver = profile.create_retargeter(side)
    rows = yaml.safe_load(profile.path('keypoints', side).read_text())['keypoints']
    return solver.fk.compute({}), model_flexion_hint(solver.fk.model, rows)


@pytest.mark.parametrize('preset_id', PRESETS)
def test_presets_keep_bone_lengths_and_palm_anchors(skeleton, preset_id):
    points, hint = skeleton
    original = points.copy()
    preset = make_preset(points, preset_id, hint)
    np.testing.assert_array_equal(points, original)
    assert np.isfinite(preset.points).all()
    np.testing.assert_allclose(preset.points[[0, 1, 5, 9, 13, 17]], points[[0, 1, 5, 9, 13, 17]], atol=1e-12)
    for a, b in EDGES:
        assert np.linalg.norm(preset.points[a] - preset.points[b]) == pytest.approx(
            np.linalg.norm(points[a] - points[b]), abs=1e-10)
    if preset.target is not None:
        assert preset.gap_m == pytest.approx(np.linalg.norm(preset.points[4] - preset.target))
        if preset_id in ('pinch_index', 'kapandji_3'):
            np.testing.assert_array_equal(preset.target, preset.points[8])
        if preset_id in ('pinch_middle', 'kapandji_4'):
            np.testing.assert_array_equal(preset.target, preset.points[12])
        if preset_id in ('pinch_ring', 'kapandji_5'):
            np.testing.assert_array_equal(preset.target, preset.points[16])
        if preset_id in ('pinch_little', 'kapandji_6'):
            np.testing.assert_array_equal(preset.target, preset.points[20])


@pytest.mark.parametrize('preset_id', PRESETS)
def test_presets_follow_mirror_scale_and_translation(skeleton, preset_id):
    points, hint = skeleton
    expected = make_preset(points, preset_id, hint)
    mirror = np.diag([-1, 1, 1])
    shift = np.array([.12, -.07, .03])
    actual = make_preset(points @ mirror * 1.3 + shift, preset_id, hint @ mirror)
    np.testing.assert_allclose(actual.points, expected.points @ mirror * 1.3 + shift, atol=1e-9)


def test_kapandji_surface_and_crease_order():
    points = np.arange(63, dtype=float).reshape(21, 3) / 1000
    radial, normal, width = np.array([1., 0, 0]), np.array([0., 1, 0]), .08
    np.testing.assert_allclose(kapandji_target(points, 1, radial, normal, width),
                               (points[6] + points[7]) / 2 + radial * width * .045)
    np.testing.assert_allclose(kapandji_target(points, 2, radial, normal, width),
                               (points[7] + points[8]) / 2 + radial * width * .04)
    for stage, index in ((7, 19), (8, 18), (9, 17)):
        np.testing.assert_allclose(kapandji_target(points, stage, radial, normal, width),
                                   points[index] + normal * width * .045)
    with pytest.raises(ValueError):
        kapandji_target(points, 0, radial, normal, width)


def test_impossible_target_leaves_a_reported_gap(skeleton):
    points, hint = skeleton
    # Very short thumb cannot reach the other fingers; do not stretch the bones.
    points = points.copy()
    points[2:5] = points[1] + (points[2:5] - points[1]) * .05
    preset = make_preset(points, 'kapandji_9', hint)
    assert preset.gap_m > .001
    for a, b in ((1, 2), (2, 3), (3, 4)):
        assert np.linalg.norm(preset.points[a] - preset.points[b]) == pytest.approx(np.linalg.norm(points[a] - points[b]))


def test_invalid_calibration_is_rejected(skeleton):
    points, hint = skeleton
    with pytest.raises(ValueError, match='未知'):
        make_preset(points, 'missing', hint)
    with pytest.raises(ValueError):
        make_preset(points, 'kapandji_11', hint)
    invalid = points.copy()
    invalid[2] = invalid[1]
    with pytest.raises(ValueError, match='骨节'):
        make_preset(invalid, 'pinch_index', hint)
