"""Full A3 session creation and isolated solver mode changes without hardware."""
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'src')]

from hand_profiles import A30_MODEL_IDS, A32_MODEL_IDS, HandModelProfile
from online_control.online_motion_track import main
from online_control.online_motion_track.a30 import session as a30_session
from online_control.online_motion_track.a30.control import A30Control
from online_control.online_motion_track.a32 import session as a32_session
from online_control.online_motion_track.a32.control import A32Control
from online_control.online_motion_track.a32.solver_process import SolverProcess
from test_web_v3_2 import packet
from online_control.online_motion_track.command_compensation import compensate_command_pose
from hand_profiles import load_hand_profile


def collision_forbidden(*args, **kwargs):
    raise AssertionError('direct creation/processing entered a collision path')


@pytest.mark.parametrize('model_id', (*A30_MODEL_IDS, *A32_MODEL_IDS))
@pytest.mark.parametrize('requested_collision', ['stop', 'mink'])
@pytest.mark.parametrize('preview_only', [False, True])
def test_application_direct_creation_forces_off_before_collision_construction(
        model_id, requested_collision, preview_only, tmp_path, monkeypatch):
    from online_control.online_motion_track.pipeline_v3_0 import collision
    from online_control.online_motion_track import mink_guard
    from online_control.online_motion_track.a30 import control as a30_control
    monkeypatch.setattr(main, 'a30_control_available', lambda: not preview_only)
    monkeypatch.setattr(main, 'a32_control_available', lambda: not preview_only)
    monkeypatch.setattr(a30_session, 'A30Control', lambda *args, **kwargs: A30Control(*args, **kwargs, start=False))
    monkeypatch.setattr(a32_session, 'A32Control', lambda *args, **kwargs: A32Control(*args, **kwargs, start=False))
    monkeypatch.setattr(HandModelProfile, 'create_collision_filter', collision_forbidden)
    monkeypatch.setattr(collision, 'StopCollisionGuard', collision_forbidden)
    monkeypatch.setattr(a30_control, 'StopCollisionGuard', collision_forbidden)
    monkeypatch.setattr(mink_guard, 'MinkCollisionGuard', collision_forbidden)
    app = main.Application(log_dir=tmp_path)
    try:
        result = app.create(dict(model_id=model_id, side='left', direct_mode=True,
                                 collision_mode=requested_collision, recording=False,
                                 input_source='webxr'))
        session = app.get(result['session'])
        assert result['direct_mode'] and result['collision_mode'] == 'off'
        assert session.direct_mode and session.pipeline.collision_bypass
        assert session.pipeline.collision_mode == 'off' and session.pipeline.mink_guard is None
        assert hasattr(session, 'control') == (not preview_only)
        if not preview_only:
            state = session.status()
            assert state['direct_mode'] and not state['ruckig_enabled']
            assert not state['collision_protection_enabled'] and not state['armed']
            assert not state['connected']
        monkeypatch.setattr(session.pipeline, '_update_contacts', collision_forbidden)
        monkeypatch.setattr(session.pipeline.session, 'ensure_collision', collision_forbidden)
        world = session.pipeline.session.retargeter.fk.compute()
        observed = session.process({**packet(world), 'source': 'webxr'})
        assert observed['tracking'] and observed['direct_mode']
        assert len(observed['pose']) == 20 and observed['collision'] == {}
        assert observed['direct_joint_angles_rad'] == compensate_command_pose(
            observed['retarget_joint_angles_rad'], session.pipeline.session.limits)
        assert observed['mink_joint_angles_rad'] is None
        lost = session.process({**packet(world, sequence=1), 'hand': None, 'source': 'webxr'})
        assert not lost['tracking'] and lost['direct_mode'] and lost['collision'] == {}
        assert lost['direct_joint_angles_rad'] is None and lost['mink_joint_angles_rad'] is None
    finally:
        app.close()


@pytest.mark.parametrize('model_id', A32_MODEL_IDS)
@pytest.mark.parametrize('collision_mode', ['stop', 'mink'])
def test_real_solver_process_bypasses_and_restores_target_guard(model_id, collision_mode):
    if collision_mode == 'mink' and not main.mink_available(model_id):
        pytest.skip('Mink dependencies are unavailable')
    reference = main.WebSession(recording=False, model_id=model_id, collision_mode='off', input_source='webxr')
    try:
        world = reference.pipeline.session.retargeter.fk.compute()
        seed = dict(reference.pipeline.session.pose)
    finally:
        reference.close()
    solver = SolverProcess(side='left', model_id=model_id, collision_mode=collision_mode, input_source='webxr')
    try:
        protected = solver.process({**packet(world), 'source': 'webxr'})
        assert protected['tracking'] and not protected['direct_mode']
        assert protected['collision']['mode'] == collision_mode
        assert bool(protected['mink_joint_angles_rad']) == (collision_mode == 'mink')
        assert protected['direct_joint_angles_rad'] is None
        solver.call('set_direct_mode', True)
        solver.call('seed', seed)
        direct = solver.process({**packet(world, sequence=1), 'source': 'webxr'})
        assert direct['tracking'] and direct['direct_mode']
        assert direct['collision'] == {}
        assert direct['pose'] == compensate_command_pose(
            direct['retarget_joint_angles_rad'], load_hand_profile(model_id).joint_limits('left'))
        assert direct['direct_joint_angles_rad'] == direct['pose']
        assert direct['mink_joint_angles_rad'] is None
        lost = solver.process({**packet(world, sequence=2), 'hand': None, 'source': 'webxr'})
        assert not lost['tracking'] and lost['direct_mode'] and lost['collision'] == {}
        solver.call('set_direct_mode', False)
        solver.call('seed', seed)
        restored = solver.process({**packet(world, sequence=3), 'source': 'webxr'})
        assert restored['tracking'] and not restored['direct_mode']
        assert restored['collision']['mode'] == collision_mode
        assert len(restored['pose']) == 20
    finally:
        solver.close()
