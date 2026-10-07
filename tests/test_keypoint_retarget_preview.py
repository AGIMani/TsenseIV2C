"""Real FK/solver checks for unsaved browser calibration previews."""
import copy
from dataclasses import replace
import os
from pathlib import Path
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'src'), str(ROOT / 'tools')]

import numpy as np
import pytest
import yaml

from hand_profiles import load_hand_profile
from keypoint_preview_solver import PreviewSolver
from TsenseI_retargeting import TopoRetargeter


@pytest.mark.parametrize('model', ['TsenseI_v3_2', 'TsenseI_v3_2_withid'])
@pytest.mark.parametrize('side', ['left', 'right'])
def test_unsaved_calibration_matches_fresh_solver_including_link_and_palm_width(model, side, tmp_path):
    profile = load_hand_profile(model)
    config = profile.create_retargeter(side).config
    engine = PreviewSolver(config)
    data = yaml.safe_load(profile.path('keypoints', side).read_text())
    source = engine.solver.fk.compute({})
    engine.solve(source, data)
    edited = copy.deepcopy(data)
    edited['keypoints'][5]['local_xyz'][0] += .006
    edited['keypoints'][8]['local_xyz'][2] += .008
    edited['keypoints'][1]['link'] = edited['keypoints'][2]['link']
    result = engine.solve(source, edited)
    path = tmp_path / 'edited.yaml'
    path.write_text(yaml.safe_dump(edited))
    fresh = TopoRetargeter(replace(config, keypoints_path=path))
    expected = fresh.retarget(source)
    np.testing.assert_allclose(list(result['joint_values'].values()), list(expected.joint_values.values()), atol=1e-10)
    np.testing.assert_allclose(result['points'], fresh.fk.compute(expected.joint_values), atol=1e-12)
    np.testing.assert_allclose(engine.solver.robot_topo_frame, fresh.robot_topo_frame, atol=1e-12)
    assert engine.solver.robot_palm_width_m == pytest.approx(fresh.robot_palm_width_m)
    # The installed calibration has not been touched by previewing the edit.
    assert yaml.safe_load(profile.path('keypoints', side).read_text()) == data


def test_input_changes_drive_joints_and_invalid_input_recovers():
    profile = load_hand_profile('TsenseI_v3_2_withid')
    engine = PreviewSolver(profile.create_retargeter('left').config)
    data = yaml.safe_load(profile.path('keypoints').read_text())
    source = engine.solver.fk.compute({})
    first = engine.solve(source, data)
    bent = engine.solver.fk.compute({name: .8 for name in engine.solver.joint_names if 'index' in name and 'swing' not in name})
    for _ in range(12):
        final = engine.solve(bent, data)
    assert max(abs(final['joint_values'][name] - first['joint_values'][name]) for name in final['joint_values']) > .05
    invalid = bent.copy()
    invalid[17] = invalid[5]
    with pytest.raises(ValueError, match='掌宽'):
        engine.solve(invalid, data)
    assert np.isfinite(engine.solve(bent, data)['points']).all()

