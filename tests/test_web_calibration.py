"""Calibration uses real FK and retarget; persistence tests only write temp files."""
from copy import deepcopy
from pathlib import Path
import json
import sys
import threading
import urllib.request
import urllib.error

import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'src'), str(ROOT / 'tools')]
from online_control.online_motion_track.main import Application, make_server
from online_control.online_motion_track.calibration import CalibrationSession
from keypoint_preview_solver import PreviewSolver
from hand_profiles import A30_MODEL_IDS, A32_MODEL_IDS
from webxr import WEBXR_25_TO_MEDIAPIPE_21, WEBXR_HAND_JOINT_NAMES, WEBXR_PROTOCOL_SCHEMA


@pytest.mark.parametrize('model', (*A32_MODEL_IDS, *A30_MODEL_IDS))
@pytest.mark.parametrize('side', ('left', 'right'))
def test_models_use_same_zero_pose_and_unsaved_solver(model, side):
    session = CalibrationSession(side, model_id=model)
    original = session.path.read_bytes()
    try:
        np.testing.assert_array_equal(session.input_points, session.world_points())
        assert session.engine.solver.config.kinematics_backend == 'numpy'
        rows = deepcopy(session.data['keypoints'])
        rows[8]['local_xyz'][2] += .005
        rows[5]['local_xyz'][0] += .002
        result = session.update(dict(calibration_rows=rows, preset='pinch_index', edit_revision=4))
        fresh = PreviewSolver(session.config).solve(result['input_points'], session.data)
        np.testing.assert_allclose(list(result['pose'].values()), list(fresh['joint_values'].values()), atol=1e-10)
        assert result['edit_revision'] == 4 and result['preset']['target']
        assert session.path.read_bytes() == original
        assert not hasattr(session, 'control')
        for preset in session.description()['presets']:
            result = session.update({'preset': preset['id']})
            assert np.isfinite(result['input_points']).all()
    finally:
        session.close()


def test_stream_reference_is_fixed_during_edit_freeze_rejects_old_frames():
    session = CalibrationSession()
    try:
        original = session.input_points.copy()
        epoch = session.set_input(dict(enabled=True, source='mediapipe', side='left'))['input_epoch']
        packet = dict(sequence=0, timestamp_ms=100, input_epoch=epoch, hand=dict(label='Left', world=original.tolist()))
        first = session.process(packet)
        changed = deepcopy(session.data['keypoints']); changed[5]['local_xyz'][0] += .005
        result = session.process({**packet, 'sequence': 1, 'timestamp_ms': 150, 'calibration_rows': changed})
        np.testing.assert_allclose(result['input_points'], first['input_points'])
        assert max(abs(result['pose'][key]-first['pose'][key]) for key in result['pose']) > 1e-6
        loss = session.process({**packet, 'sequence': 2, 'timestamp_ms': 200, 'hand': None})
        assert loss['tracking'] is False
        np.testing.assert_array_equal(loss['input_points'], result['input_points'])
        with pytest.raises(ValueError, match='冻结'):
            session.update({'preset': 'fist'})
        session.set_input(dict(enabled=False))
        with pytest.raises(ValueError, match='冻结'):
            session.process({**packet, 'sequence': 3, 'timestamp_ms': 250})
        session.update({'input_points': original.tolist()})
    finally:
        session.close()


def test_save_export_reload_backup_conflict_and_invalid_rows(tmp_path):
    first, second = CalibrationSession(), CalibrationSession()
    try:
        original = first.path.read_bytes()
        target = tmp_path / 'points.yaml'; target.write_bytes(original)
        first.path = second.path = target
        rows = deepcopy(first.data['keypoints']); rows[8]['local_xyz'][0] += .008
        first.update(dict(calibration_rows=rows, preset='fist'))
        exported = first.export({})
        assert target.read_bytes() == original
        assert 'input_points' not in yaml.safe_load(exported['yaml'])
        saved = first.save({})
        assert Path(saved['backup']).read_bytes() == original
        assert yaml.safe_load(target.read_text())['keypoints'][8]['local_xyz'] == rows[8]['local_xyz']
        with pytest.raises(ValueError, match='其他页面'):
            second.save({})
        second.reload()
        np.testing.assert_array_equal(second.data['keypoints'][8]['local_xyz'], rows[8]['local_xyz'])
        for bad in ('unknown_link', float('nan'), 'duplicate'):
            invalid = deepcopy(rows)
            if bad == 'unknown_link': invalid[3]['link'] = bad
            elif bad == 'duplicate': invalid[3]['id'] = 2
            else: invalid[3]['local_xyz'][0] = bad
            with pytest.raises(ValueError): first.update({'calibration_rows': invalid})
        first.update({'input_points': second.input_points.tolist()})
    finally:
        first.close(); second.close()


def test_existing_quest_relay_drives_calibration_and_rejects_browser_forgery():
    app = Application()
    try:
        created = app.create(dict(mode='calibration', input_source='webxr', model_id='TsenseI_v3_2_withid'))
        token = created['session']
        epoch = app.calibration_action(dict(session=token, enabled=True, source='webxr'), 'input')['input_epoch']
        connection = app.webxr_create(dict(session=token))
        assert not connection['background_processing']
        reverse = dict(zip(WEBXR_25_TO_MEDIAPIPE_21, created['input_points']))
        joints = [dict(name=name, valid=True, position=reverse.get(i, created['input_points'][0]),
                       orientation=[0,0,0,1], radius=.004) for i, name in enumerate(WEBXR_HAND_JOINT_NAMES)]
        message = dict(schema=WEBXR_PROTOCOL_SCHEMA, stream_id='cal-test', sequence=0, timestamp_ms=100, client_perf_ms=100,
                       type='hand_skeleton', joint_set='webxr-hand-25', coordinate_space='webxr_local_floor',
                       hands=[dict(handedness='left', tracked=True, joints=joints)])
        app.webxr.publish(connection['token'], {**message, 'type': 'client_hello'})
        app.webxr.publish(connection['token'], {**message, 'sequence': 1})
        snapshot = app.webxr_latest(dict(session=token))
        assert snapshot['hand']
        result = app.frame(dict(session=token, source='webxr', input_epoch=epoch, sequence=1,
                                timestamp_ms=150, webxr_revision=snapshot['revision'],
                                hand=dict(label='Right',world=[[99]*3]*21)))
        assert result['tracking'] and result['source'] == 'webxr'
        assert np.isfinite(result['result_points']).all()
    finally:
        app.close()


def test_http_page_and_public_save_restriction(tmp_path):
    server = make_server(0, tmp_path, public=True)
    worker = threading.Thread(target=server.serve_forever, daemon=True); worker.start()
    base = f'http://127.0.0.1:{server.server_port}'
    def post(route, payload):
        request = urllib.request.Request(base+route, json.dumps(payload).encode(),
                                         {'Content-Type':'application/json', 'Origin':base})
        return json.load(urllib.request.urlopen(request))
    try:
        html = urllib.request.urlopen(base+'/calibration').read().decode()
        assert 'calibration-stage' in html and 'result-stage' in html
        created = post('/api/session',dict(mode='calibration',model_id='TsenseI_v3_2_withid'))
        assert not created['save_allowed']
        assert post('/api/calibration/export',dict(session=created['session']))['yaml']
        with pytest.raises(urllib.error.HTTPError) as error:
            post('/api/calibration/save',dict(session=created['session']))
        assert error.value.code == 400
        post('/api/stop',dict(session=created['session']))
    finally:
        server.shutdown();server.server_close();server.application.close();worker.join()


def test_server_import_and_bundle_have_no_qt_requirement():
    from online_control.online_motion_track.package import files
    paths = {name for _, name in files()}
    assert 'app/tools/keypoint_preview_solver.py' in paths
    assert 'app/online_control/online_motion_track/calibration.py' in paths
    assert any(path.endswith('/controls/TransformControls.js') for path in paths)
    assert not any('keypoint_retarget_preview.py' in path for path in paths)
