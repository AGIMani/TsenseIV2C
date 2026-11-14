"""Measured comparison poses, using synthetic observations and no hardware."""
import copy
from functools import partial
from http.server import ThreadingHTTPServer
import json
import math
from pathlib import Path
import sys
import threading
from types import SimpleNamespace
import urllib.error
import urllib.request

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "src")]
from hand_profiles import load_hand_profile
A32_MODEL_IDS = ("TsenseI_v3_2_withid",)
from online_control.online_motion_track.feedback import feedback_pose


def a32_state(model="TsenseI_v3_2_withid"):
    from a32_driver import A32Device
    device = A32Device(model_id=model)
    pose = {name: low + .4 * (high - low) for name, (low, high) in device.joint_limits().items()}
    targets = device.prepare_targets(pose, clamp=False)
    state = {"model_id": model, "hand_side": "left", "connected": True, "telemetry_age_ms": 10,
             "self": [{"online": True,
                       "position": value if index in (6, 7, 14, 15) else -value,
                       "target": 0.}
                      for index, value in enumerate(targets["self"])],
             "mit": [{"online": True, "valid": True, "fault": 0, "position": value}
                     for value in targets["mit_pos"]]}
    assert device._backend is None
    return pose, state


@pytest.mark.parametrize("model", A32_MODEL_IDS)
def test_a32_uses_calibrated_sdk_table_without_loading_native_transport(model, monkeypatch):
    from a32_driver import NativeHand
    monkeypatch.setattr(NativeHand, "__init__", lambda *a, **kw: pytest.fail("must not load native SDK"))
    pose, state = a32_state(model)
    original = copy.deepcopy(state)
    result = feedback_pose({"model_id": model, "side": "left", "telemetry": state})
    assert result["pose"] == pytest.approx(pose, abs=2e-7)
    assert result["missing_joints"] == []
    assert state == original


@pytest.mark.parametrize("model", A32_MODEL_IDS)
@pytest.mark.parametrize("angle", [-.5, -.35, -.2, 0., .2, .35, .5])
def test_a32_measured_swing_preserves_sign_and_uses_urdf_bounds(model, angle):
    _, state = a32_state(model)
    slots = {6: "pinky", 7: "ring", 14: "mid", 15: "index"}
    for index in slots:
        state["self"][index]["position"] = math.degrees(angle)
        state["self"][index]["target"] = 12.  # Measured sign must win over the target.
    result = feedback_pose({"model_id": model, "side": "left", "telemetry": state})
    assert result["missing_joints"] == []
    for finger in slots.values():
        assert result["pose"][f"left_hand_{finger}_swing_joint"] == pytest.approx(
            min(.35, max(-.35, angle)), abs=2e-7)


def test_a32_missing_faulted_out_of_range_and_stale_feedback():
    pose, state = a32_state()
    state["self"][0]["online"] = False
    state["self"][1]["position"] = float("nan")
    state["self"][6]["position"] = 999
    state["mit"][1]["fault"] = 1
    request = {"model_id": "TsenseI_v3_2_withid", "side": "left", "telemetry": state}
    result = feedback_pose(request)
    assert len(result["pose"]) == 17
    assert len(result["missing_joints"]) == 3
    assert result["pose"]["left_hand_pinky_swing_joint"] == load_hand_profile("TsenseI_v3_2_withid").joint_limits()["left_hand_pinky_swing_joint"][1]
    state["telemetry_age_ms"] = 500
    assert feedback_pose(request)["pose"] == {}
    state["telemetry_age_ms"] = 0
    state["connected"] = False
    assert feedback_pose(request)["pose"] == {}


@pytest.mark.parametrize("model", A32_MODEL_IDS)
@pytest.mark.parametrize("value", [-999., 999.])
def test_a32_finite_feedback_overruns_stay_visible_within_original_range(model, value):
    from a32_driver import A32Device
    _, state = a32_state(model)
    for row in [*state["self"], *state["mit"]]:
        row["position"] = value
    result = feedback_pose({"model_id": model, "side": "left", "telemetry": state})
    assert result["missing_joints"] == []
    limits = load_hand_profile(model).joint_limits()
    assert all(limits[name][0] <= q <= limits[name][1] for name, q in result["pose"].items())
    for name, q in result["pose"].items():
        if "_swing_" in name:
            assert q == limits[name][0 if value < 0 else 1]
    device = A32Device(model_id=model)
    for channel in device.channels:
        if channel["unit"] == "mm":
            name = channel["joint"]
            endpoint = device.joint_limits()[name][1 if value < 0 else 0]
            assert result["pose"][name] == pytest.approx(endpoint, abs=2e-7)
    if value < 0:
        assert result["pose"]["left_hand_thumb_rotaback_joint2"] == 0
    else:
        assert result["pose"]["left_hand_thumb_rotaback_joint2"] == limits["left_hand_thumb_rotaback_joint2"][1]


@pytest.mark.parametrize("model", A32_MODEL_IDS)
def test_a32_right_feedback_uses_shared_hardware_map(model):
    _, state = a32_state(model)
    state["hand_side"] = "right"
    result = feedback_pose({"model_id": model, "side": "right", "telemetry": state})
    assert len(result["pose"]) == 20
    assert result["missing_joints"] == []
    assert result["issues"] == []
    assert all(name.startswith("right_hand_") for name in result["pose"])


@pytest.mark.parametrize("data", [None, {}, {"model_id": "wrong", "side": "left"},
    {"model_id": "TsenseI_v2", "side": "wrong"},
    {"model_id": "TsenseI_v2", "side": "left", "telemetry": "00"},
    {"model_id": "TsenseI_v3_0", "side": "left", "telemetry": "z" * 968},
    {"model_id": "TsenseI_v3_2_withid", "side": "left", "telemetry": {"self": [{}] * 19, "mit": []}},
    {"model_id": "TsenseI_v3_2_withid", "side": "left", "telemetry": {"hand_side": "right"}},
])
def test_bad_request_is_rejected(data):
    with pytest.raises(ValueError):
        feedback_pose(data)


def test_http_route_is_pure_and_needs_no_control_session():
    from online_control.online_motion_track.main import Handler
    # No Application/controller is instantiated, so the handler cannot read
    # hardware or mutate control state even accidentally.
    app = SimpleNamespace(public=False, allowed_hosts={"127.0.0.1"})
    server = ThreadingHTTPServer(("127.0.0.1", 0), partial(Handler, application=app))
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base = f"http://127.0.0.1:{server.server_port}"
    _, state = a32_state()
    def post(data, origin=base):
        request = urllib.request.Request(base + "/api/feedback-pose", json.dumps(data).encode(),
                                         {"Content-Type": "application/json", "Origin": origin})
        with urllib.request.urlopen(request) as response:
            return json.load(response)
    try:
        request = {"model_id": "TsenseI_v3_2_withid", "side": "left", "telemetry": state}
        assert len(post(request)["pose"]) == 20
        for model in A32_MODEL_IDS:
            pose, state = a32_state(model)
            assert post({"model_id": model, "side": "left", "telemetry": state})["pose"] == pytest.approx(pose, abs=2e-7)
        with pytest.raises(urllib.error.HTTPError) as exc:
            post({**request, "telemetry": "bad"})
        assert exc.value.code == 400
        with pytest.raises(urllib.error.HTTPError) as exc:
            post(request, "http://different.example")
        assert exc.value.code == 403
    finally:
        server.shutdown()
        server.server_close()
        thread.join()
