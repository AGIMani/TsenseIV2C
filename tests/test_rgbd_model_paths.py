"""Real RGB-D fusion and model solvers, with no physical camera or robot."""

import json
from pathlib import Path
import sys

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'src')]
from online_control.online_motion_track import main
from online_control.online_motion_track.rgbd_fusion import fuse_hand
from utils.mediapipe_frame import apply_mediapipe_transformations
from test_realsense_application import StoredCamera, packet
from test_rgbd_fusion import CAMERA, hand_scene


class CountingCamera(StoredCamera):
    def __init__(self, side):
        super().__init__()
        self.normalized, self.prior, self.depth, self.truth = hand_scene(
            mirror=side == 'right', prior_wrong_z=True)
        self.commits = []

    def commit_if_current(self, token, snapshot, callback):
        self.commits.append(snapshot['frame_id'])
        return super().commit_if_current(token, snapshot, callback)


@pytest.mark.parametrize('side', ['left', 'right'])
def test_v2_rgbd_uses_metric_path_and_multiple_guarded_commits(side, tmp_path, monkeypatch):
    pytest.importorskip('ruckig')
    from online_control.online_motion_track.v2 import session as module
    from test_web_v2 import MemoryUsb
    monkeypatch.setattr(module, 'UsbController', MemoryUsb)

    def forbidden(*args, **kwargs):
        pytest.fail('Fused metric landmarks must not enter Wuji bone normalization')

    monkeypatch.setattr(module, 'preprocess_wuji_world_points', forbidden)
    app = main.Application(tmp_path)
    app.realsense = CountingCamera(side)
    created = app.create(dict(model_id='TsenseI_v2', side=side, collision_mode='off',
                              input_source='realsense_rgbd', recording=True))
    token, session = created['session'], app.get(created['session'])
    try:
        session.command('tracking_start', {})
        for sequence in range(5):
            frame = packet(app, token, sequence=sequence)
            frame['hand']['label'] = side.title()
            result = app.frame(frame)
        assert result['tracking'], result['message']
        assert result['depth_source'] == 'realsense_rgbd_fused'
        assert app.realsense.commits.count(app.realsense.sequence) == 2
        expected = apply_mediapipe_transformations(np.array(result['fused_world']), side)
        np.testing.assert_allclose(result['points'], expected, atol=1e-8)
        assert session.control.snapshot()['target_ready']
        assert not session.control.snapshot()['armed']
        assert not session.control.usb.writes
        before_target = session.control.snapshot()['target_joint_angles_rad']
        before_frame = session.control.snapshot()['frame_id']
        app.realsense.depth.fill(0)
        invalid = packet(app, token, sequence=5)
        invalid['hand']['label'] = side.title()
        rejected = app.frame(invalid)
        assert not rejected['tracking']
        assert rejected['depth_source'] == 'none'
        # V2's existing loss transition holds the previous safe target. A
        # missing depth frame must neither retarget nor replace that target.
        assert session.control.snapshot()['target_joint_angles_rad'] == before_target
        assert session.control.snapshot()['frame_id'] == before_frame
    finally:
        app.close()
    rows = [json.loads(line) for line in Path(created['log_file']).read_text().splitlines()]
    assert rows[0]['metadata']['camera_source'] == 'realsense-rgbd'
    assert 'realsense_rgbd metric' in rows[0]['metadata']['control_order']
    assert {'realsense_rgbd_raw', 'realsense_rgbd', 'control_target'} <= {row['type'] for row in rows}


@pytest.mark.parametrize('side', ['left', 'right'])
def test_a32_rgbd_target_is_guarded_and_uses_fused_camera_axes(side, tmp_path, monkeypatch):
    pytest.importorskip('ruckig')
    from online_control.online_motion_track.a32 import session as module
    from online_control.online_motion_track.a32.control import A32Control
    from test_a32_tracking_control import FakeDevice
    devices = []

    def control(profile, hand_side, collision_mode, **kwargs):
        device = FakeDevice(profile.model_id, hand_side)
        devices.append(device)
        return A32Control(profile, hand_side, collision_mode, device=device, start=False,
                          recording_dir=kwargs.get('recording_dir'))

    monkeypatch.setattr(module, 'A32Control', control)
    monkeypatch.setattr(main, 'a32_control_available', lambda: True)
    app = main.Application(tmp_path)
    app.realsense = CountingCamera(side)
    token = app.create(dict(model_id='TsenseI_v3_2', side=side, collision_mode='off',
                            input_source='realsense_rgbd', recording=False))['session']
    session = app.get(token)
    try:
        session.control.start_tracking()
        frame = packet(app, token)
        frame['hand']['label'] = side.title()
        result = app.frame(frame)
        assert result['tracking'], result['message']
        assert len(app.realsense.commits) == 1
        assert result['depth_source'] == 'realsense_rgbd_fused'
        np.testing.assert_allclose(result['points'],
            apply_mediapipe_transformations(np.array(result['fused_world']), side), atol=1e-8)
        before = dict(session.control.target)
        process = session.pipeline.process

        def expire(observation):
            solved = process(observation)
            app.realsense.expired = True
            return solved

        monkeypatch.setattr(session.pipeline, 'process', expire)
        frame = packet(app, token, sequence=1)
        frame['hand']['label'] = side.title()
        rejected = app.frame(frame)
        assert not rejected['tracking']
        assert rejected['background_discarded']
        assert rejected['depth_source'] == 'none'
        assert session.control.target == before
        assert not devices[0].writes
    finally:
        app.close()


def test_a32_solver_subprocess_accepts_rgbd_packet_and_serializable_diagnostics():
    from online_control.online_motion_track.a32.solver_process import SolverProcess
    normalized, prior, depth, _ = hand_scene(prior_wrong_z=True)
    fused = fuse_hand(normalized, prior, depth, CAMERA)
    solver = SolverProcess(side='left', collision_mode='off', model_id='TsenseI_v3_2',
                           input_source='realsense_rgbd')
    try:
        result = solver.process(dict(sequence=0, timestamp_ms=1000, capture_unix_ms=1000,
            source='realsense_rgbd', camera_size=[640, 480],
            source_metadata={'depth_fusion': fused['diagnostics']},
            hand={'label': 'Left', 'world': fused['world'], 'normalized': fused['normalized']}))
        assert result['tracking'], result['message']
        assert result['source'] == 'realsense_rgbd'
        assert result['source_metadata']['depth_fusion']['measured_count'] == 21
        assert result['depth_source'] == 'realsense_rgbd_fused'
        np.testing.assert_allclose(result['points'],
            apply_mediapipe_transformations(np.array(fused['world']), 'left'), atol=1e-8)
    finally:
        solver.close()
