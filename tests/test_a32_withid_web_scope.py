"""Web model/collision contract for the retained A3.2 with ID hand."""
from pathlib import Path
import json
import sys
import threading
import urllib.error
import urllib.request

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'src')]

from hand_profiles import load_hand_profile
from online_control.online_motion_track import main
from online_control.online_motion_track.a32.control import A32Control
from online_control.online_motion_track.calibration import CalibrationSession
from online_control.online_motion_track.pipeline_v3_2.pipeline import MediaPipeHandPipeline
from test_a32_tracking_control import FakeDevice

MODEL = 'TsenseI_v3_2_withid'
OLD_MODELS = ('TsenseI_v2', 'TsenseI_v3_0', 'TsenseI_v3_0_withid', 'TsenseI_v3_2')


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('mode', ['off', 'mink'])
def test_retained_model_creates_preview_and_processes_a_frame(monkeypatch, side, mode):
    monkeypatch.setattr(main, 'a32_control_available', lambda: False)
    app = main.Application()
    try:
        result = app.create({'model_id': MODEL, 'side': side, 'collision_mode': mode, 'recording': False})
        session = app.get(result['session'])
        assert result['model_id'] == MODEL
        assert result['collision_mode'] == mode
        assert (session.pipeline.mink_guard is not None) == (mode == 'mink')
        world = session.pipeline.session.retargeter.fk.compute().tolist()
        frame = session.process({'sequence': 0, 'timestamp_ms': 1000, 'capture_unix_ms': 1,
                                 'hand': {'label': side.title(), 'world': world, 'normalized': world}})
        assert frame['tracking'], frame['message']
        assert frame['actuation']['model_id'] == MODEL
        assert frame['collision'].get('mode') == (mode if mode == 'mink' else None)
        assert len(frame['pose']) == 20
    finally:
        app.close()


@pytest.mark.parametrize('old_model', OLD_MODELS)
def test_old_models_rejected_by_session_calibration_and_model_descriptor(old_model):
    app = main.Application()
    try:
        with pytest.raises(ValueError):
            app.create({'model_id': old_model, 'recording': False})
        with pytest.raises(ValueError):
            app.create({'model_id': old_model, 'mode': 'calibration', 'recording': False})
        with pytest.raises(ValueError):
            main.model_description('left', old_model)
        with pytest.raises(ValueError):
            CalibrationSession(model_id=old_model)
        with pytest.raises(ValueError):
            MediaPipeHandPipeline(model_id=old_model)
    finally:
        app.close()


@pytest.mark.parametrize('old_mode', ['monitor', 'stop', 'filter', 'avoid'])
@pytest.mark.parametrize('direct', [False, True])
def test_removed_modes_are_rejected_without_constructing_session(old_mode, direct):
    app = main.Application()
    try:
        with pytest.raises(ValueError, match='碰撞模式'):
            app.create({'model_id': MODEL, 'collision_mode': old_mode, 'direct_mode': direct, 'recording': False})
        assert not app.sessions
        with pytest.raises(ValueError):
            MediaPipeHandPipeline(collision_mode=old_mode)
    finally:
        app.close()


def test_missing_daqp_or_sample_checker_disables_mink_but_not_off(monkeypatch):
    find_spec = main.importlib.util.find_spec
    monkeypatch.setattr(main.importlib.util, 'find_spec', lambda name: None if name == 'daqp' else find_spec(name))
    assert main.a32_collision_modes() == ['off']
    monkeypatch.setattr(main, 'a32_control_available', lambda: False)
    app = main.Application()
    try:
        with pytest.raises(ValueError):
            app.create({'model_id': MODEL, 'collision_mode': 'mink', 'recording': False})
        assert app.create({'model_id': MODEL, 'collision_mode': 'off', 'recording': False})['collision_mode'] == 'off'
    finally:
        app.close()


def test_missing_sample_checker_is_not_advertised_as_available(monkeypatch):
    monkeypatch.setattr(main, '_a3_sample_checker_available', lambda model_id=MODEL: False)
    app = main.Application()
    try:
        assert not app.mink_available
        assert main.a32_collision_modes() == ['off']
    finally:
        app.close()


@pytest.mark.parametrize('side', ['left', 'right'])
def test_mink_keeps_separate_sample_safety_gate(side):
    profile = load_hand_profile(MODEL)
    device = FakeDevice(MODEL, side)
    control = A32Control(profile, side, 'mink', device=device, start=False)
    try:
        assert control.guard is not None
        current = profile.init_pose(side)
        control._seed(current)
        rng = np.random.default_rng(6502)
        for _ in range(8):
            target = {name: rng.uniform(lo, hi) for name, (lo, hi) in profile.joint_limits(side).items()}
            safe = control.guard.project(target)
            control.guard.filter.sync_pose_from_dict(safe)
            assert control.guard.filter.max_penetration_mm() <= 1e-7
        assert not device.writes
    finally:
        control.close()


def test_http_reports_one_model_and_rejects_removed_modes(monkeypatch):
    monkeypatch.setattr(main, 'a32_control_available', lambda: False)
    server = main.make_server(0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base = f'http://127.0.0.1:{server.server_port}'
    try:
        with urllib.request.urlopen(base + '/api/health') as response:
            health = json.load(response)
        assert health['models'] == [MODEL]
        assert set(health['a32_collision_modes'][MODEL]) == {'off', 'mink'}
        for mode in ('stop', 'monitor'):
            request = urllib.request.Request(base + '/api/session', method='POST',
                data=json.dumps({'model_id': MODEL, 'collision_mode': mode, 'recording': False}).encode(),
                headers={'Content-Type': 'application/json'})
            with pytest.raises(urllib.error.HTTPError) as error:
                urllib.request.urlopen(request)
            assert error.value.code == 400
        with pytest.raises(urllib.error.HTTPError):
            urllib.request.urlopen(base + '/api/model?model=TsenseI_v2')
    finally:
        server.shutdown()
        server.server_close()
        server.application.close()
        thread.join(timeout=5)
