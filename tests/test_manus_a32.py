"""Manus frames through the real A3.2 control session with simulated transport."""
import json
from pathlib import Path
import sys
import time

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT/'src')]
from a32_driver import A32Device
from online_control.online_motion_track import main
from online_control.online_motion_track.a32 import session as session_module
from online_control.online_motion_track.a32.control import A32Control
from test_a32_driver import SimulatedNativeHand
from test_manus_input import device, wait_for


@pytest.fixture
def app(device, tmp_path, monkeypatch):
    clock, glove, relay = device
    backends = []
    def fake_control(profile, side, mode, **options):
        backend = SimulatedNativeHand()
        backends.append(backend)
        return A32Control(profile, side, mode,
            device=A32Device(model_id=profile.model_id, hand_side=side, backend=backend),
            start=False, **options)
    monkeypatch.setattr(session_module, 'A32Control', fake_control)
    monkeypatch.setattr(main, 'a32_control_available', lambda: True)
    value = main.Application(tmp_path)
    value.manus = relay
    yield value, clock, glove, backends
    value.close()


def start(app, glove, *, side='left', model='TsenseI_v3_2_withid'):
    token = app.create(dict(side=side, model_id=model, input_source='manus_glove', recording=False))['session']
    session = app.get(token)
    assert isinstance(session, session_module.A32WebSession)
    session.command('connect', dict(port='SIM_ONLY'))
    session.command('tracking_start', {})
    pose = {name: min(.5, hi) if '_mid_' in name else 0
            for name, (lo, hi) in session.pipeline.session.limits.items()}
    glove.publish(session.pipeline.session.retargeter.fk.compute(pose), side=side)
    connection = app.manus_create(dict(session=token))
    assert connection['background_processing']
    return token, session


def solved(app, token):
    return wait_for(lambda: app.manus_frame(dict(session=token)),
        lambda frame: bool(frame.get('update', {}).get('tracking')) if frame.get('update') else False)


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('model', ['TsenseI_v3_2_withid'])
def test_manus_updates_target_then_manual_arm_sends_and_expiry_holds(app, side, model):
    application, clock, glove, backends = app
    token, session = start(application, glove, side=side, model=model)
    frame = solved(application, token)
    assert frame['update']['depth_source'] == 'manus_glove_metric'
    assert session.control.target[f'{side}_hand_mid_PIP_joint'] > .1
    backend = backends[0]
    assert backend.calls == [('usb', 'SIM_ONLY')]
    # Arm uses feedback as its seed. A new callback then supplies the next target.
    session.command('arm', {'enabled': True})
    glove.publish(session.pipeline.session.retargeter.fk.compute(
        {f'{side}_hand_mid_PIP_joint': .6}), side=side, sequence=2)
    wait_for(lambda: application.manus_frame(dict(session=token)),
        lambda value:value['source_metadata'].get('sequence') == 2 and
                     value.get('update') is not None and value['update']['tracking'])
    session.control.step(.002)
    assert any(call[0] == 'send' for call in backend.calls)
    target_at = session.control.target_at
    held_target = dict(session.control.target)
    application.manus_frame(dict(session=token))
    assert session.control.target_at == target_at  # Repeated polls never push a new target.
    clock.value += .21
    lost = wait_for(lambda: application.manus_frame(dict(session=token)),
        lambda value:value.get('update') is not None and not value['update']['tracking'])
    assert lost['hand'] is None
    assert session.control.target_at == target_at
    assert session.control.target == held_target
    assert session.control.phase == 'holding'
    log_path = session.control.recorder.writer.path
    application.remove(token)
    records = [json.loads(row) for row in Path(log_path).read_text().splitlines()]
    assert records[0]['metadata']['camera_source'] == 'manus-glove'
    raw = next(row for row in records if row['type'] == 'manus_glove_raw' and row.get('raw_manus'))
    assert raw['raw_manus']['source_timestamp_raw'] == '123456'
    assert raw['input_source'] == 'manus_glove'
    assert not any(row['type'] == 'mediapipe_raw' for row in records)


@pytest.mark.parametrize('invalidate', ['expire', 'stop'])
def test_frame_invalidated_during_solve_cannot_replace_control_target(app, monkeypatch, invalidate):
    application, clock, glove, _ = app
    token, session = start(application, glove)
    solved(application, token)
    original_target = dict(session.control.target)
    original_at = session.control.target_at
    process = session.pipeline.process
    triggered = []
    def delayed(observation):
        result = process(observation)
        if observation is not None:
            triggered.append(True)
            if invalidate == 'expire':
                clock.value += .21
            else:
                application.manus.close(token)
        return result
    monkeypatch.setattr(session.pipeline, 'process', delayed)
    glove.publish(session.pipeline.session.retargeter.fk.compute(
        {'left_hand_index_PIP_joint': .8}), sequence=2)
    worker = application.metric_workers[token]
    wait_for(worker.read, lambda value: bool(value.get('update', {}).get('background_discarded'))
        if value.get('update') else False)
    assert triggered
    assert session.control.target_at == original_at
    # A subsequent loss can hold the current pose, but cannot commit the rejected pose.
    assert session.control.target in (original_target, session.control.pose)


def test_stopping_browser_lease_rejects_new_manus_target(app):
    application, clock, glove, _ = app
    token, session = start(application, glove)
    solved(application, token)
    worker = application.metric_workers[token]
    target, at = dict(session.control.target), session.control.target_at
    with worker.lock:
        worker.last_client = worker.clock()-worker.CLIENT_TIMEOUT-1
    glove.publish(session.pipeline.session.retargeter.fk.compute(
        {'left_hand_index_PIP_joint': .8}), sequence=2)
    time.sleep(.04)
    assert session.control.target == target and session.control.target_at == at
    assert not session.control.armed
