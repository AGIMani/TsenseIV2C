"""Atomic estimator/controller transactions and stale-request isolation."""
from concurrent.futures import ThreadPoolExecutor
from functools import partial
from pathlib import Path
import sys
import threading
import pytest

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'src')]
from online_control.online_motion_track.a30 import session as module
from online_control.online_motion_track.main import WebSession


@pytest.fixture
def session(monkeypatch):
    monkeypatch.setattr(module,'A30Control',partial(module.A30Control,start=False))
    s=module.A30WebSession(recording=False,collision_mode='stop',model_id='TsenseI_v3_0_withid')
    yield s
    s.close()


def packet(s,sequence):
    world=s.pipeline.session.retargeter.fk.compute().tolist()
    return {'sequence':sequence,'timestamp_ms':1000+sequence*33,'capture_unix_ms':0,
            'hand':{'label':'Left','world':world,'normalized':world}}


def test_old_frame_cannot_latch_a_healthy_controller(session):
    p=packet(session,2);session.process(p);session.control.step(.002)
    before=(dict(session.control.target),session.control.wire,session.control.frame_id)
    with pytest.raises(ValueError,match='过期'):session.process(p)
    assert session.control.valid
    assert (session.control.target,session.control.wire,session.control.frame_id)==before
    assert session.process(packet(session,3))['tracking']


def test_nonfinite_new_input_is_still_a_hard_fault(session):
    p=packet(session,0);p['timestamp_ms']=float('nan')
    with pytest.raises(ValueError):session.process(p)
    assert not session.control.valid and session.control.phase=='blocked'
    session.process(packet(session,1))
    assert not session.control.valid and not session.control.wire


def test_status_does_not_wait_for_slow_retarget_transaction(session):
    # A long Mink solve owns the estimator lock, but the independent control
    # loop must remain observable and its timestamp must not be delayed by it.
    with ThreadPoolExecutor(max_workers=1) as pool:
        with session.lock:
            reply=pool.submit(session.command,'status',{})
            assert reply.result(timeout=.2)['control_stream']


def test_transport_recovery_does_not_wait_for_slow_retarget(session):
    with ThreadPoolExecutor(max_workers=1) as pool:
        with session.lock:
            reply=pool.submit(session.command,'browser_rebase',{
                'telemetry':bytes(484).hex(),'pose':session.control.pose,
                'resume_from_commanded_pose':True})
            assert reply.result(timeout=.2)['browser_epoch']==1


def test_automatic_transport_recovery_cannot_clear_a_latched_fault(session):
    session.control.invalidate('invalid input')
    with pytest.raises(ValueError,match='故障'):
        session.command('browser_rebase',{'telemetry':bytes(484).hex(),'pose':session.control.pose,
                                         'resume_from_commanded_pose':True})
    assert not session.control.valid


@pytest.mark.parametrize('action',['rebase','mode','frame','close'])
def test_no_interleaving_between_estimator_return_and_controller_commit(session,monkeypatch,action):
    reached=threading.Event();release=threading.Event();finished=threading.Event();attempted=threading.Event()
    original=WebSession.process
    def paused(self,p):
        result=original(self,p)
        if p['sequence']==0:
            reached.set()
            assert release.wait(3)
        return result
    monkeypatch.setattr(WebSession,'process',paused)
    first=packet(session,0);second=packet(session,1)
    def intervene():
        attempted.set()
        try:
            if action=='rebase':session.command('browser_rebase',{'telemetry':bytes(484).hex(),'pose':session.control.pose})
            elif action=='mode':session.command('set_direct_mode',{'enabled':True})
            elif action=='frame':session.process(second)
            else:session.close()
        finally:finished.set()
    with ThreadPoolExecutor(max_workers=2) as pool:
        a=pool.submit(session.process,first)
        assert reached.wait(3)
        b=pool.submit(intervene)
        assert attempted.wait(3)
        try:
            assert not finished.wait(.08),'control mutation crossed an uncommitted frame'
        finally:release.set()
        a.result(3);b.result(3)
    if action=='frame':assert session.control.frame_id==1
