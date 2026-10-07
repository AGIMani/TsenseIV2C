"""Measured comparison poses, using synthetic observations and no hardware."""
import copy
from functools import partial
from http.server import ThreadingHTTPServer
import json
import math
from pathlib import Path
import struct
import sys
import threading
from types import SimpleNamespace
import urllib.error
import urllib.request

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "src")]
from hand_profiles import A30_MODEL_IDS, A32_MODEL_IDS, load_hand_profile
from online_control.online_motion_track.feedback import feedback_pose


def a30_packet(model="TsenseI_v3_0", side="left"):
    from a30_kinematics import A30StrokeMapper
    from a30_kinematics.downlink import channel_mapping
    mapper = A30StrokeMapper(side, model_id=model)
    pose = {name: low + .4 * (high - low) for name, (low, high) in
            load_hand_profile(model).joint_limits(side).items()}
    targets = mapper.convert(pose)
    packet = bytearray(484)
    struct.pack_into("<HHII", packet, 0, 7, 3, 1000, 0x3ffff)
    for row in channel_mapping(side):
        value = (targets.linear_strokes_mm if row["unit"] == "mm"
                 else targets.rotary_positions_deg)[row["joint"]]
        if row["field"] == "self_pos":
            struct.pack_into("<f", packet, 16 + row["slot"] * 24, value * row["position_sign"])
        else:
            offset = 444 + row["slot"] * 20
            packet[offset:offset + 2] = bytes([5 + 4 * row["slot"], 5])
            struct.pack_into("<I", packet, offset + 2,
                             round(value * row["gear_ratio"] / 360 * 2097152) + 0x80000000)
            packet[offset + 16] = 4
    return pose, packet


def v2_packet(side="left", legacy=False):
    from usb.offline import joint_angles_to_hand_positions
    from protocol.hand_angle_protocol import convert_ui_to_hardware
    pose = {name: .25 for name in load_hand_profile("TsenseI_v2").joint_limits()}
    linear, rotary = joint_angles_to_hand_positions(convert_ui_to_hardware(pose, hand_side=side))
    packet = bytearray(932 if legacy else 780)
    struct.pack_into("<HHI", packet, 0, 9, 0, 1234)
    stride = 44 if legacy else 32
    for i, value in enumerate(linear):
        struct.pack_into("<f", packet, 8 + i * stride + (24 if legacy else 4), value)
    for i, value in enumerate(rotary):
        struct.pack_into("<f", packet, 8 + 14 * stride + i * 44 + 24, value)
    if not legacy:
        packet[8 + 14 * stride + 1] = int(side == "right")
    return {name.replace("left_hand_", f"{side}_hand_"): value for name, value in pose.items()}, packet


def a32_state(model="TsenseI_v3_2"):
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


@pytest.mark.parametrize("model", A30_MODEL_IDS)
@pytest.mark.parametrize("side", ["left", "right"])
def test_a30_roundtrip_uses_linkage_sign_units_and_mit_reduction(model, side):
    pose, packet = a30_packet(model, side)
    result = feedback_pose({"model_id": model, "side": side, "telemetry": packet.hex()})
    assert result["pose"] == pytest.approx({name: max(0., q) for name, q in pose.items()}, abs=1e-6)
    assert result["missing_joints"] == []
    assert result["total_joints"] == 20


def test_a30_missing_faulted_nonfinite_channels_are_never_filled_and_limits_are_clipped():
    pose, packet = a30_packet()
    struct.pack_into("<I", packet, 8, 0x3ffff & ~(1 << 4))
    struct.pack_into("<f", packet, 16 + 5 * 24, float("nan"))
    struct.pack_into("<f", packet, 16, 999.)
    struct.pack_into("<I", packet, 444 + 12, 1)
    result = feedback_pose({"model_id": "TsenseI_v3_0", "side": "left", "telemetry": packet.hex()})
    missing = {"left_hand_pinky_PIP_joint", "left_hand_pinky_MCP_joint", "left_hand_thumb_rotaback_joint2"}
    assert set(result["missing_joints"]) == missing
    assert set(result["pose"]) == set(pose) - missing
    assert result["pose"]["left_hand_pinky_DIP_joint"] == load_hand_profile("TsenseI_v3_0").joint_limits()["left_hand_pinky_DIP_joint"][1]
    short = feedback_pose({"model_id": "TsenseI_v3_0", "side": "left", "telemetry": packet[:444].hex()})
    assert "left_hand_thumb_rota_block_joint" in short["missing_joints"]


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("legacy", [False, True])
def test_v2_roundtrip_uses_original_four_finger_thumb_and_side_maps(side, legacy):
    pose, packet = v2_packet(side, legacy)
    result = feedback_pose({"model_id": "TsenseI_v2", "side": side, "telemetry": packet.hex()})
    assert result["pose"] == pytest.approx(pose, abs=1e-5)
    assert result["missing_joints"] == []
    assert result["total_joints"] == 21


def test_v2_negative_angle_becomes_zero_and_stroke_overrun_is_bounded():
    pose, packet = v2_packet()
    struct.pack_into("<f", packet, 8 + 4, -999.)
    struct.pack_into("<f", packet, 8 + 14 * 32 + 24, -50.)
    result = feedback_pose({"model_id": "TsenseI_v2", "side": "left", "telemetry": packet.hex()})
    assert result["missing_joints"] == []
    assert result["pose"]["left_hand_index_swing_joint"] == 0
    assert 0 <= result["pose"]["left_hand_ring_MCP_joint"] <= load_hand_profile("TsenseI_v2").joint_limits()["left_hand_ring_MCP_joint"][1]
    assert result["pose"]["left_hand_ring_PIP_joint"] == pytest.approx(pose["left_hand_ring_PIP_joint"], abs=1e-5)


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


def test_a32_missing_faulted_out_of_range_and_stale_feedback():
    pose, state = a32_state()
    state["self"][0]["online"] = False
    state["self"][1]["position"] = float("nan")
    state["self"][6]["position"] = 999
    state["mit"][1]["fault"] = 1
    request = {"model_id": "TsenseI_v3_2", "side": "left", "telemetry": state}
    result = feedback_pose(request)
    assert len(result["pose"]) == 17
    assert len(result["missing_joints"]) == 3
    assert result["pose"]["left_hand_pinky_swing_joint"] == load_hand_profile("TsenseI_v3_2").joint_limits()["left_hand_pinky_swing_joint"][1]
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
    assert all(max(0., limits[name][0]) <= q <= limits[name][1] for name, q in result["pose"].items())
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
def test_a32_right_is_explicitly_unavailable(model):
    _, state = a32_state(model)
    state["hand_side"] = "right"
    result = feedback_pose({"model_id": model, "side": "right", "telemetry": state})
    assert result["pose"] == {}
    assert len(result["missing_joints"]) == 20
    assert "待标定" in result["issues"][0]


@pytest.mark.parametrize("data", [None, {}, {"model_id": "wrong", "side": "left"},
    {"model_id": "TsenseI_v2", "side": "wrong"},
    {"model_id": "TsenseI_v2", "side": "left", "telemetry": "00"},
    {"model_id": "TsenseI_v3_0", "side": "left", "telemetry": "z" * 968},
    {"model_id": "TsenseI_v3_2", "side": "left", "telemetry": {"self": [{}] * 19, "mit": []}},
    {"model_id": "TsenseI_v3_2", "side": "left", "telemetry": {"hand_side": "right"}},
])
def test_bad_request_is_rejected(data):
    with pytest.raises(ValueError):
        feedback_pose(data)


def test_v2_wrong_hand_is_rejected():
    _, packet = v2_packet("right")
    with pytest.raises(ValueError, match="左右手"):
        feedback_pose({"model_id": "TsenseI_v2", "side": "left", "telemetry": packet.hex()})


def test_http_route_is_pure_and_needs_no_control_session():
    from online_control.online_motion_track.main import Handler
    # No Application/controller is instantiated, so the handler cannot read
    # hardware or mutate control state even accidentally.
    app = SimpleNamespace(public=False, allowed_hosts={"127.0.0.1"})
    server = ThreadingHTTPServer(("127.0.0.1", 0), partial(Handler, application=app))
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base = f"http://127.0.0.1:{server.server_port}"
    _, packet = a30_packet()
    def post(data, origin=base):
        request = urllib.request.Request(base + "/api/feedback-pose", json.dumps(data).encode(),
                                         {"Content-Type": "application/json", "Origin": origin})
        with urllib.request.urlopen(request) as response:
            return json.load(response)
    try:
        request = {"model_id": "TsenseI_v3_0", "side": "left", "telemetry": packet.hex()}
        assert len(post(request)["pose"]) == 20
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
