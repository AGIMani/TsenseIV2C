"""Tracking stops revoke host commands without sending a motor-disable request."""
from pathlib import Path
import sys
import time

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "src")]

from hand_profiles import A32_MODEL_IDS
from online_control.online_motion_track.a32 import session as session_module
from test_a32_tracking_recording import make_session


@pytest.mark.parametrize("model_id", A32_MODEL_IDS)
@pytest.mark.parametrize("trigger", [
    "disable", "emergency_stop", "cancel_home", "arm_off", "disconnect", "close",
    "heartbeat", "motor_fault", "invalid_dt", "solver_failure", "write_failure",
    "unsafe_trajectory",
])
def test_tracking_stop_paths_never_disable_motors(tmp_path, monkeypatch, model_id, trigger):
    # This fixture uses the real A32Device and records every native SDK call.
    session, backend = make_session(tmp_path, monkeypatch, model_id)
    control = session.control
    try:
        session.command("connect", {"port": "SIM_ONLY"})
        session.command("arm", {"enabled": True})
        control.push({**control.pose, "left_hand_index_PIP_joint": .2}, 1)
        control.step(.002)
        assert control.armed and control.ticks == 1
        assert ("enable", True) in backend.calls and ("send",) in backend.calls

        if trigger in ("disable", "emergency_stop", "cancel_home", "disconnect"):
            session.command(trigger, {})
        elif trigger == "arm_off":
            session.command("arm", {"enabled": False})
        elif trigger == "close":
            session.close()
        elif trigger == "heartbeat":
            control.client_at = time.monotonic() - control.CLIENT_TIMEOUT - 1
            control.step(.002)
            assert "心跳超时" in control.last_error
        elif trigger == "motor_fault":
            backend.state["mit"][0]["fault"] = 1
            control.step(.002)
            assert "故障" in control.last_error
        elif trigger == "invalid_dt":
            control.step(float("nan"))
            assert "采样间隔无效" in control.last_error
        elif trigger == "solver_failure":
            def fail_solve(*args, **kwargs):
                raise ValueError("synthetic solver failure")
            monkeypatch.setattr(session_module.WebSession, "process", fail_solve)
            with pytest.raises(ValueError, match="synthetic solver failure"):
                session.process({"sequence": 2})
            assert "synthetic solver failure" in control.last_error
        elif trigger == "write_failure":
            backend.fail_send = True
            control.step(.002)
            assert "simulated link loss" in control.last_error
        elif trigger == "unsafe_trajectory":
            def fail_trajectory(*args, **kwargs):
                raise ValueError("synthetic collision cannot be resolved")
            monkeypatch.setattr(control, "_checked_step", fail_trajectory)
            control.step(.002)
            assert "collision cannot be resolved" in control.last_error

        assert not control.armed
        assert not control.device.get_state()["enabled"]  # Host authorization only.
        calls_after_stop = list(backend.calls)
        backend.fail_send = False
        backend.state["mit"][0]["fault"] = 0
        # Fresh input and liveness cannot silently restart the command stream.
        control.touch()
        control.push({**control.pose, "left_hand_index_PIP_joint": .3}, 3)
        for _ in range(3):
            control.step(.002)
        assert backend.calls == calls_after_stop
    finally:
        session.close()
    assert ("estop",) not in backend.calls
    assert ("enable", False) not in backend.calls
