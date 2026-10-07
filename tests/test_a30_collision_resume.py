"""Collision holds must retry when another finger removes the obstruction."""
from pathlib import Path
import json
import sys
import pytest

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'src')]
from hand_profiles import load_hand_profile
from online_control.online_motion_track.a30.control import A30Control
from online_control.online_motion_track.mink_guard import MinkCollisionGuard


@pytest.mark.parametrize('model',['TsenseI_v3_0','TsenseI_v3_0_withid'])
@pytest.mark.parametrize('side',['left','right'])
def test_direct_retreat_reaches_open_target_without_collision_holds(model, side, monkeypatch):
    case=json.loads((ROOT/'tests/fixtures/a30_withid_direct_resume.json').read_text())
    profile=load_hand_profile(model)
    controller=A30Control(profile,side,'mink',start=False)
    normal_guard=controller.guard
    def forbidden(*args, **kwargs):
        pytest.fail('Direct retreat must not invoke the normal collision path')
    monkeypatch.setattr(normal_guard,'project',forbidden)
    try:
        controller.set_direct_mode(True)
        origin={n.replace('left_',side+'_'):q for n,q in case['pose'].items()}
        controller._seed(controller.mapper.convert(origin,clamp=True).joint_angles_rad)
        open_pose=profile.init_pose(side)
        controller.push(open_pose,42)
        controller.step(.003)
        assert controller.valid,controller.error
        assert controller.pose==open_pose
        assert controller.sample_frame_id==42
        assert controller.guard is None
        assert not controller.finger_holds
        assert len(controller.wire)==108
        controller.set_direct_mode(False)
        assert controller.guard is normal_guard
        assert controller.snapshot()['collision_protection_enabled']
    finally:controller.close()


@pytest.mark.parametrize('side',['left','right'])
def test_withid_open_gesture_recovers_after_other_fingers_separate(side):
    case=json.loads((ROOT/'tests/fixtures/a30_withid_collision_resume.json').read_text())
    profile=load_hand_profile('TsenseI_v3_0_withid')
    controller=A30Control(profile,side,'mink',start=False)
    guard=MinkCollisionGuard(profile,side)
    target={n.replace('left_',side+'_'):q for n,q in case['target'].items()}
    open_pose=profile.init_pose(side)
    held=False
    try:
        for frame in range(110):
            requested=guard.project(target if frame<10 else open_pose)
            controller.push(requested,frame)
            for _ in range(15):
                controller.step(.002)
                assert controller.valid,controller.error
                held |= bool(controller.finger_holds)
                controller.guard.filter.sync_pose_from_dict(controller.pose)
                assert controller.guard.filter.data.ncon==0
                assert len(controller.wire)==108
        assert held  # Exercise an actual collision hold, not only a clear path.
        assert guard.diagnostics['status']=='clear'
        assert not controller.finger_holds
        assert controller.pose==pytest.approx(open_pose,abs=1e-7)
    finally:controller.close()


@pytest.mark.parametrize('model',['TsenseI_v3_0','TsenseI_v3_0_withid'])
def test_unchanged_finger_target_retried_when_partner_moves(model):
    controller=A30Control(load_hand_profile(model),'left','stop',start=False)
    index='left_hand_index_PIP_joint';mid='left_hand_mid_PIP_joint'
    old=dict(controller.pose)
    def barrier(pose):
        # The index request is blocked until the middle finger has moved away.
        hit=pose[index]>.0001 and pose[mid]<.1
        controller.guard.diagnostics={'limited':hit,'contacts':[
            {'g1':'left_hand_index_PIP_link_capsule','g2':'left_hand_mid_PIP_link_capsule'}] if hit else []}
        return dict(controller.pose) if hit else dict(pose)
    controller.guard.project=barrier
    try:
        target={**old,index:.2}
        controller.push(target,0)
        for _ in range(100):controller.step(.002)
        assert 'index' in controller.finger_holds
        # Same unsafe request must still be stopped.
        controller.push(target,1)
        for _ in range(100):controller.step(.002)
        assert controller.pose[index]<=.0001
        # The index target stays exactly unchanged; only its neighbour moves.
        target[mid]=.3
        for frame in range(2,42):
            controller.push(target,frame)
            for _ in range(15):
                controller.step(.002)
                assert controller.valid,controller.error
                assert not (controller.pose[index]>.0001 and controller.pose[mid]<.1)
        assert not controller.finger_holds
        assert controller.pose[index]==pytest.approx(.2,abs=1e-7)
    finally:controller.close()
