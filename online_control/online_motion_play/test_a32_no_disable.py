"""Playback lifecycle reaches the real device adapter without disabling motors."""
from pathlib import Path
import sys
import time

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / "src"), str(ROOT / "tests")]

from a32_driver import A32Device
from a32_session import A32Session, MODEL_IDS
from test_a32 import Clock, tick, widget
from test_server import bridge

from test_a32_driver import SimulatedNativeHand


def active_session(tmp_path, model_id):
    backend = SimulatedNativeHand()
    device = A32Device(model_id=model_id, backend=backend)
    session = A32Session(tmp_path, model_id=model_id, device=device, clock=Clock(),
                         start=False, recording_dir=tmp_path / "logs")
    session.dispatch("connect", {"port": "SIM_ONLY"})
    widget(session, "realtimeJointControl", True)
    widget(session, "left_hand_index_PIP_joint", .2)
    tick(session)
    assert session.armed and session.sample_ticks == 1
    assert ("enable", True) in backend.calls and ("send",) in backend.calls
    return session, backend


def assert_stopped(session, backend):
    assert not session.armed and not session.sync and session.playback is None
    assert not session.device.get_state()["enabled"]  # Host authorization only.
    calls_after_stop = list(backend.calls)
    backend.fail_send = False
    for _ in range(3):
        tick(session)
    assert backend.calls == calls_after_stop
    assert ("estop",) not in backend.calls
    assert ("enable", False) not in backend.calls


@pytest.mark.parametrize("model_id", MODEL_IDS)
@pytest.mark.parametrize("trigger", [
    "user_stop", "page_hidden", "disable_button", "realtime_off", "sync_off",
    "playback_stop", "disconnect", "close", "sampling_timeout", "write_failure",
    "homing", "planner_failure",
])
def test_playback_stop_paths_never_disable_motors(tmp_path, model_id, trigger):
    session, backend = active_session(tmp_path, model_id)
    try:
        if trigger in ("user_stop", "page_hidden"):
            session.dispatch("stop", {"reason": trigger})
            assert session.connected
            assert session.stop_reason == trigger
        elif trigger == "disable_button":
            widget(session, "a32Disable")
        elif trigger == "realtime_off":
            widget(session, "realtimeJointControl", False)
        elif trigger == "sync_off":
            widget(session, "isConnectOffline", True)
            widget(session, "isConnectOffline", False)
        elif trigger == "playback_stop":
            session.persist({"test": {"pose": session.target_pose, "transition": .4, "interval": 0}})
            widget(session, "isConnectOffline", True)
            widget(session, "motionstreamplay")
            tick(session)
            assert session.playback is not None
            widget(session, "motionstreamplay")
        elif trigger == "disconnect":
            session.dispatch("disconnect", {})
        elif trigger == "close":
            session.close()
        elif trigger == "sampling_timeout":
            tick(session, .101)
            assert "100 ms" in session.message
        elif trigger == "write_failure":
            backend.fail_send = True
            tick(session)
            assert "simulated link loss" in session.message
        elif trigger == "homing":
            backend.state["self"][0]["homing"] = 1
            tick(session)
            assert "找零" in session.message
        elif trigger == "planner_failure":
            class FailedPlanner:
                def update(self, *args):
                    raise ValueError("synthetic trajectory failure")
            session.planner = FailedPlanner()
            tick(session)
            assert "synthetic trajectory failure" in session.message
        assert_stopped(session, backend)
    finally:
        session.close()
    assert ("estop",) not in backend.calls
    assert ("enable", False) not in backend.calls


@pytest.mark.parametrize("model_id", MODEL_IDS)
@pytest.mark.parametrize("trigger", ["heartbeat", "release", "shutdown"])
def test_playback_ownership_stop_never_disables_motors(bridge, tmp_path, model_id, trigger):
    session, backend = active_session(tmp_path, model_id)
    bridge.a32 = session
    client = "synthetic-owner"
    bridge.dispatch("control", {}, client=client)
    if trigger == "heartbeat":
        bridge.last_seen = time.monotonic() - bridge.CONTROLLER_LEASE_S - 1
        bridge.check_lease()
        assert session.connected
        assert session.stop_reason == "controller_heartbeat_timeout"
    elif trigger == "release":
        bridge.dispatch("release", {}, client=client)
        assert not session.connected
        assert session.stop_reason == "controller_release"
    else:
        bridge.close()
        assert not session.connected
        assert session.stop_reason == "session_closed"
    assert bridge.controller is None
    assert_stopped(session, backend)
    bridge.close()
    assert ("estop",) not in backend.calls
    assert ("enable", False) not in backend.calls
