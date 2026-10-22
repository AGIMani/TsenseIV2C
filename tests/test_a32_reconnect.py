"""Replay real reconnect feedback against fake transports; no hardware I/O."""
import copy
import json
from pathlib import Path

import pytest

from test_a32_driver import SimulatedNativeHand, connected_device
from test_a32_tracking_recording import make_session


CASE = json.loads((Path(__file__).parent / "fixtures/a32_reconnect_feedback.json").read_text())


@pytest.mark.parametrize("model_id", ["TsenseI_v3_2", "TsenseI_v3_2_withid"])
@pytest.mark.parametrize("transport", ["usb", "eth"])
@pytest.mark.parametrize("link_loss", [False, True])
def test_reconnect_never_replays_previously_sent_pose(model_id, transport, link_loss):
    device, backend = connected_device(model_id)
    try:
        device.enable_all()
        device.send_joint_targets(CASE["previous_pose"], clamp=False)
        if link_loss:
            backend.state["connected"] = False
            assert not device.get_state()["enabled"]
        else:
            device.disconnect()
        before = len(backend.calls)
        if transport == "usb":
            device.connect_usb("SIM_ONLY")
        else:
            device.connect_eth("127.0.0.1", 5001)
        assert len(backend.calls) == before + 1  # Connection only, no command.
        backend.state = copy.deepcopy(CASE["feedback"])
        seed = device.starting_pose(CASE["previous_pose"])
        assert not device.get_state()["seed_missing"]
        device.enable_all(start_pose=seed)
        own, mit = backend.calls[-3][1:]
        assert backend.calls[-2:] == [("send",), ("enable", True)]
        for channel in device.channels:
            row = backend.state[channel["channel"]][channel["index"]]
            measured = device.feedback_position(channel, row["position"])
            target = (own if channel["channel"] == "self" else mit)[channel["index"]]
            assert abs(target - measured) < .1
        # Previously 13 slots jumped to 10.77--19.54 mm. Now every linear
        # target stays below 0.07 mm, including the negative zero noise.
        linear = [own[c["index"]] for c in device.channels if c["unit"] == "mm"]
        assert max(linear) < .07
        assert ("estop",) not in backend.calls and ("enable", False) not in backend.calls
    finally:
        device.close()


@pytest.mark.parametrize("model_id", ["TsenseI_v3_2", "TsenseI_v3_2_withid"])
@pytest.mark.parametrize("direct_mode", [False, True])
def test_tracking_reconnect_seeds_new_feedback_and_waits_for_new_target(tmp_path, monkeypatch, model_id, direct_mode):
    session, backend = make_session(tmp_path, monkeypatch, model_id)
    control = session.control
    try:
        session.command("connect", {"port": "SIM_ONLY"})
        control.set_direct_mode(direct_mode)
        session.command("arm", {"enabled": True})
        control.device.send_joint_targets(CASE["previous_pose"], clamp=False)
        control._seed(CASE["previous_pose"])
        control.push(CASE["previous_pose"], 123)
        # Reconnect before the worker observes loss: armed and old input still
        # exist at the moment the connect request arrives.
        backend.state["connected"] = False
        before = len(backend.calls)
        session.command("connect", {"port": "SIM_ONLY"})
        assert len(backend.calls) == before + 1
        assert not control.armed and control.target_at == 0 and control.frame_id == -1
        control.step(.002)
        assert len(backend.calls) == before + 1
        backend.state = copy.deepcopy(CASE["feedback"])
        session.command("arm", {"enabled": True})
        assert max(q for n, q in control.pose.items() if "swing" not in n and "rota" not in n) == pytest.approx(.005)
        assert control.target == control.pose and control.target_at == 0
        first = backend.calls[-3]
        assert first[0] == "targets"
        for _ in range(4):
            control.step(.002)
        assert all(call == first for call in backend.calls[before:] if call[0] == "targets")
        # A genuinely new nonzero pose is also respected on a later resume;
        # the fix must not replace old targets with a fixed all-zero pose.
        control.stop()
        measured_pose = {n: (lo + hi) * .4 for n, (lo, hi) in control.device.joint_limits().items()}
        measured_targets = control.device.prepare_targets(measured_pose, clamp=False)
        for channel in control.device.channels:
            key = "self" if channel["channel"] == "self" else "mit_pos"
            value = measured_targets[key][channel["index"]]
            backend.state[channel["channel"]][channel["index"]]["position"] = -value if channel["unit"] == "mm" else value
        session.command("arm", {"enabled": True})
        assert control.pose == pytest.approx(measured_pose, abs=2e-7)
        assert backend.calls[-3][1] == pytest.approx(measured_targets["self"])
    finally:
        session.close()
    assert ("estop",) not in backend.calls and ("enable", False) not in backend.calls
