"""Expired metric observations must not change an editable calibration draft."""

from copy import deepcopy
from pathlib import Path
import sys

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'src'), str(ROOT / 'tools')]
from online_control.online_motion_track.calibration import CalibrationSession


@pytest.fixture
def session():
    value = CalibrationSession(input_source='realsense_rgbd')
    value.set_input(dict(enabled=True, source='realsense_rgbd', side='left'))
    yield value
    value.close()


def packet_for(session):
    world = session.input_points.copy()
    world[8] += [.008, -.003, .006]
    rows = deepcopy(session.data['keypoints'])
    rows[8]['local_xyz'][0] += .003
    return dict(sequence=0, timestamp_ms=100, input_epoch=session.input_epoch,
                source='realsense_rgbd', calibration_rows=rows,
                hand=dict(label='Left', world=world.tolist()))


@pytest.mark.parametrize('raises', [False, True])
def test_expired_frame_restores_draft_pose_and_solver_history(session, raises):
    before_points = session.input_points.copy()
    before_result, before_data = deepcopy(session.result), deepcopy(session.data)
    before_engine = session.engine
    calls = []

    def reject(action):
        calls.append(action)
        # The solve actually ran and changed both the input and calibration.
        assert not np.array_equal(session.input_points, before_points)
        assert session.data != before_data
        if raises:
            raise ValueError('source expired')
        return False

    response = session.process_metric(packet_for(session), reject)
    assert len(calls) == 1
    assert response['tracking'] is False
    assert response['display_discarded']
    assert response['discard_reason'] == 'source_cancelled_or_expired'
    np.testing.assert_array_equal(session.input_points, before_points)
    np.testing.assert_array_equal(session.result['points'], before_result['points'])
    np.testing.assert_array_equal(response['input_points'], before_points)
    np.testing.assert_array_equal(response['result_points'], before_result['points'])
    assert response['pose'] == {**before_result['joint_values'], **session.profile.fixed_joints}
    assert session.data == before_data
    assert (session.last_sequence, session.last_timestamp) == (-1, -1)
    assert session.engine is not before_engine


def test_current_frame_commits_draft_and_keeps_solver(session):
    packet = packet_for(session)
    before_engine = session.engine
    calls = []

    def accept(action):
        calls.append(action)
        action()
        return True

    response = session.process_metric(packet, accept)
    assert response['tracking']
    assert 'display_discarded' not in response
    assert session.data['keypoints'] == session._validated_rows(packet['calibration_rows'], session.data['keypoints'])
    assert (session.last_sequence, session.last_timestamp) == (0, 100)
    assert session.engine is before_engine
    assert len(calls) == 1


def test_old_calibration_epoch_never_reaches_commit(session):
    packet = packet_for(session)
    packet['input_epoch'] -= 1
    before_points, before_data = session.input_points.copy(), deepcopy(session.data)

    def unexpected(action):
        pytest.fail('An old input epoch cannot reach commit')

    with pytest.raises(ValueError, match='旧帧'):
        session.process_metric(packet, unexpected)
    np.testing.assert_array_equal(session.input_points, before_points)
    assert session.data == before_data

