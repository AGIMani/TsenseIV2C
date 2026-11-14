"""Live MediaPipe filtering choices must reach every web solver path."""
from copy import deepcopy
import json
from types import SimpleNamespace

import numpy as np
import pytest

from test_metric_landmark_passthrough import MODELS, moving_points, packet
from online_control.online_motion_track.main import WebSession, StaleFrameError
from online_control.online_motion_track.a32.solver_process import SolverProcess
from online_control.online_motion_track.pipeline_v3_2.pipeline import prepare_points

CHOICES = (True, True, False, False, True, True)


@pytest.mark.parametrize('model', MODELS)
@pytest.mark.parametrize('side', ('left', 'right'))
def test_live_filter_choice_and_recorded_points(model, side, tmp_path):
    session = WebSession(side, recording=True, log_dir=tmp_path, model_id=model)
    observations = []
    original = session.pipeline.process
    def observe(value):
        observations.append(value)
        return original(value)
    session.pipeline.process = observe
    try:
        rest = session.pipeline.session.retargeter.fk.compute() * .9
        for index, enabled in enumerate(CHOICES):
            world = moving_points(rest, index)
            data = packet('mediapipe', side, index, world)
            if index:  # Legacy clients still default to filtering enabled.
                data['mediapipe_filter_enabled'] = enabled
            result = session.process(data)
            assert result['tracking'], result['message']
            assert result['mediapipe_filter_enabled'] is enabled
            observation = observations[-1]
            for actual in (observation.world_points, observation.normalized_points):
                if not enabled or index in (0, 4):
                    np.testing.assert_allclose(actual, world, atol=1e-8)
                else:
                    assert np.max(np.abs(actual - world)) > 1e-5
            np.testing.assert_array_equal(observation.raw_world_points, world)
        # Older or invalid flags cannot modify the current filter state.
        with pytest.raises(StaleFrameError):
            session.process(data | {'mediapipe_filter_enabled': False})
        assert session.mediapipe_filter_enabled is True
        with pytest.raises(ValueError, match='布尔值'):
            session.process(packet('mediapipe', side, 6, world) | {'mediapipe_filter_enabled': 'false'})
        assert session.mediapipe_filter_enabled is True
    finally:
        session.close()
    rows = [json.loads(line) for line in session.recorder.path.read_text().splitlines()]
    frames = [row for row in rows if row['type'] == 'frame']
    assert [row['mediapipe_filter_enabled'] for row in frames] == list(CHOICES)
    assert frames[2]['raw_world_points_21_m'] == frames[2]['smoothed_world_points_21_m']


@pytest.mark.parametrize('model', MODELS)
def test_filter_toggle_reaches_separate_solver_process(model):
    reference = WebSession(recording=False, model_id=model)
    try:
        rest = reference.pipeline.session.retargeter.fk.compute() * .9
    finally:
        reference.close()
    solver = SolverProcess(side='left', collision_mode='off', model_id=model, input_source='mediapipe')
    try:
        for index, enabled in enumerate(CHOICES):
            world = moving_points(rest, index)
            result = solver.process(packet('mediapipe', 'left', index, world) |
                                    {'mediapipe_filter_enabled': enabled})
            assert result['tracking'], result['message']
            assert result['mediapipe_filter_enabled'] is enabled
            expected = prepare_points(world, 'left')
            if not enabled or index in (0, 4):
                np.testing.assert_allclose(result['points'], expected, atol=1e-8)
            else:
                assert np.max(np.abs(np.asarray(result['points']) - expected)) > 1e-5
    finally:
        solver.close()
