"""Native Quest ADB 21-point validation and shared relay lifecycle regression."""
from copy import deepcopy
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "src"), str(ROOT / "src/tracking")]
from webxr import WEBXR_PROTOCOL_SCHEMA
from online_control.online_motion_track.quest_adb_input import QUEST_ADB_PROTOCOL_SCHEMA
from online_control.online_motion_track.webxr_input import WebXRInputRelay


def event(kind="client_hello", sequence=0, stream="adb-a", **fields):
    return {"schema": QUEST_ADB_PROTOCOL_SCHEMA, "type": kind,
            "sequence": sequence, "stream_id": stream, **fields}


def skeleton(sequence=1, side="left", stream="adb-a"):
    return event("hand_skeleton", sequence, stream, joint_set="mediapipe-21", units="m",
                 coordinate_space="webxr_local_floor", client_perf_ms=100 + sequence,
                 hands=[{"handedness": side, "tracked": True,
                         "landmarks": [[10 + i * .001, 1 + i * .002, -2 + i * .003]
                                       for i in range(21)]}])


@pytest.fixture
def relay():
    now = [10.0]
    instance = WebXRInputRelay(clock=lambda: now[0])
    token = instance.create("pc", "left")["token"]
    instance.publish_quest_adb(token, event())
    return instance, token, now


def test_21_metric_points_keep_axes_raw_schema_and_explicit_mapping(relay):
    instance, token, _ = relay
    raw = skeleton()
    raw["normalized"] = [[999, 999, 999]] * 21
    instance.publish_quest_adb(token, raw)
    latest = instance.latest("pc")
    assert latest["source"] == "webxr"
    assert latest["tracking"] and latest["connected"]
    assert latest["hand"]["world"][0] == [0, 0, 0]
    for index, point in enumerate(latest["hand"]["world"]):
        assert point == pytest.approx([index * .001, index * .002, index * .003])
    assert latest["hand"]["normalized"] == latest["hand"]["world"]
    assert latest["hand"]["normalized_is_image"] is False
    assert latest["source_metadata"]["transport"] == "adb"
    assert latest["source_metadata"]["source_joint_count"] == 21
    assert latest["source_metadata"]["source_joint_set"] == "mediapipe-21"
    assert latest["source_metadata"]["joint_mapping"] == list(range(21))
    assert latest["raw_webxr"] == raw
    assert "joints" not in latest["raw_webxr"]["hands"][0]
    latest["raw_webxr"]["hands"][0]["landmarks"][0][0] = -1
    raw["hands"][0]["landmarks"][0][0] = -2
    assert instance.latest("pc")["raw_webxr"]["hands"][0]["landmarks"][0][0] == 10


@pytest.mark.parametrize("mutation", [
    lambda m: m.update(schema=WEBXR_PROTOCOL_SCHEMA),
    lambda m: m.update(joint_set="webxr-hand-25"),
    lambda m: m.pop("joint_set"),
    lambda m: m.update(units="mm"),
    lambda m: m.pop("units"),
    lambda m: m.update(coordinate_space="camera"),
    lambda m: m.update(coordinate_space=[]),
    lambda m: m.pop("coordinate_space"),
    lambda m: m.update(type="unsupported_event"),
    lambda m: m.update(type=[]),
    lambda m: m.update(stream_id=""),
    lambda m: m.update(sequence=-1),
    lambda m: m.update(sequence=True),
    lambda m: m.update(sequence=1.5),
    lambda m: m.update(sequence=2**53),
    lambda m: m.pop("client_perf_ms"),
    lambda m: m.update(client_perf_ms=True),
    lambda m: m.update(client_perf_ms="101"),
    lambda m: m.update(client_perf_ms=None),
    lambda m: m.update(client_perf_ms=-1),
    lambda m: m.update(client_perf_ms=10**400),
    lambda m: m.update(timestamp_ms=False),
    lambda m: m.update(xr_predicted_display_time_ms=float("inf")),
    lambda m: m.update(extra_metadata=float("nan")),
    lambda m: m.update(handedness="unknown"),
    lambda m: m.update(hands={}),
    lambda m: m["hands"].append(deepcopy(m["hands"][0])),
    lambda m: m["hands"][0].update(handedness="Left"),
    lambda m: m["hands"][0].pop("tracked"),
    lambda m: m["hands"][0].update(tracked=1),
    lambda m: m["hands"][0].update(tracked="true"),
    lambda m: m["hands"][0].update(landmarks=[]),
    lambda m: m["hands"][0]["landmarks"].pop(),
    lambda m: m["hands"][0]["landmarks"].extend([[0, 0, 0]] * 4),
    lambda m: m["hands"][0]["landmarks"].__setitem__(0, [0, 1]),
    lambda m: m["hands"][0]["landmarks"][0].__setitem__(0, True),
    lambda m: m["hands"][0]["landmarks"][0].__setitem__(0, "10"),
    lambda m: m["hands"][0]["landmarks"][0].__setitem__(0, float("nan")),
    lambda m: m["hands"][0]["landmarks"][0].__setitem__(0, 10**400),
    lambda m: m["hands"][0]["landmarks"][1].__setitem__(0, 21),
])
def test_malformed_packet_never_replaces_latest(relay, mutation):
    instance, token, _ = relay
    instance.publish_quest_adb(token, skeleton())
    before = instance.latest("pc")
    malformed = skeleton(2)
    mutation(malformed)
    with pytest.raises(ValueError):
        instance.publish_quest_adb(token, malformed)
    assert instance.latest("pc") == before


def test_selected_hand_two_hand_frames_and_untracked_empty_points(relay):
    instance, token, _ = relay
    message = skeleton()
    message["hands"].insert(0, skeleton(side="right")["hands"][0])
    instance.publish_quest_adb(token, message)
    assert instance.latest("pc")["hand"]["label"] == "Left"
    instance.publish_quest_adb(token, event("tracking_lost", 2, handedness="right"))
    assert instance.latest("pc")["tracking"]
    instance.publish_quest_adb(token, event("tracking_lost", 3, handedness="left"))
    assert instance.latest("pc")["hand"] is None
    instance.publish_quest_adb(token, skeleton(4))
    for sequence, points in ((5, []), (6, skeleton()["hands"][0]["landmarks"])):
        message = skeleton(sequence)
        message["hands"][0].update(tracked=False, landmarks=points)
        instance.publish_quest_adb(token, message)
        assert instance.latest("pc")["hand"] is None
    instance.publish_quest_adb(token, skeleton(7, side="right"))
    assert instance.latest("pc")["hand"] is None
    message = skeleton(8)
    message["hands"] = []
    instance.publish_quest_adb(token, message)
    assert instance.latest("pc")["hand"] is None


def test_tracking_restored_does_not_invent_or_restore_an_old_observation(relay):
    instance, token, _ = relay
    instance.publish_quest_adb(token, event("tracking_restored", 1, handedness="left"))
    assert instance.latest("pc")["hand"] is None
    instance.publish_quest_adb(token, skeleton(2))
    first = instance.latest("pc")
    instance.publish_quest_adb(token, event("tracking_lost", 3, handedness="left"))
    instance.publish_quest_adb(token, event("tracking_restored", 4, handedness="left"))
    assert instance.latest("pc")["source_metadata"]["event"] == "tracking_restored"
    assert instance.latest("pc")["hand"] is None
    assert instance.consume("pc", first["revision"])["hand"] is None
    instance.publish_quest_adb(token, skeleton(5))
    assert instance.latest("pc")["tracking"]


def test_freshness_transport_delay_polling_and_consumption_share_safety_rules(relay):
    instance, token, now = relay
    instance.publish_quest_adb(token, skeleton())
    first = instance.latest("pc")
    instance.publish_quest_adb(token, skeleton(2))
    assert instance.consume("pc", first["revision"])["tracking"]
    assert instance.consume("pc", first["revision"])["hand"] is None
    now[0] += .36
    assert instance.latest("pc")["hand"] is None
    instance.publish_quest_adb(token, skeleton(3))
    latest = instance.latest("pc")
    assert latest["hand"] is None
    assert latest["source_metadata"]["discard_reason"] == "late_transport"
    assert latest["source_metadata"]["transport_delay_ms"] == pytest.approx(359)
    recovery = skeleton(4)
    recovery["client_perf_ms"] = 464
    instance.publish_quest_adb(token, recovery)
    assert instance.latest("pc")["tracking"]
    regressed = skeleton(5)
    instance.publish_quest_adb(token, regressed)
    assert instance.latest("pc")["source_metadata"]["discard_reason"] == "client_clock_regressed"


@pytest.mark.parametrize("ending", ["session_ended", "client_error"])
def test_sequence_lifecycle_restart_and_subscriber(relay, ending):
    instance, token, _ = relay
    updates = []
    unsubscribe = instance.subscribe("pc", lambda: updates.append(instance.latest("pc")))
    instance.publish_quest_adb(token, skeleton(10))
    for sequence in (9, 10):
        assert instance.publish_quest_adb(token, skeleton(sequence))["reason"] == "stale_sequence"
    instance.publish_quest_adb(token, event(ending, 11))
    assert instance.latest("pc")["hand"] is None
    assert instance.consume("pc", updates[0]["revision"])["hand"] is None
    assert instance.publish_quest_adb(token, skeleton(12))["reason"] == "stream_ended"
    assert len(updates) == 2
    instance.publish_quest_adb(token, event("session_started", 0, "adb-b"))
    instance.publish_quest_adb(token, skeleton(1, stream="adb-b"))
    assert instance.latest("pc")["tracking"]
    assert instance.publish_quest_adb(token, event(sequence=99))["reason"] == "retired_stream"
    unsubscribe()
    instance.publish_quest_adb(token, skeleton(2, stream="adb-b"))
    assert len(updates) == 4


def test_protocols_cannot_be_mixed_inside_one_stream_or_wrong_endpoint(relay):
    instance, token, _ = relay
    instance.publish_quest_adb(token, skeleton())
    with pytest.raises(ValueError, match="schema"):
        instance.publish(token, skeleton(2))
    webxr_hello = {**event(sequence=2), "schema": WEBXR_PROTOCOL_SCHEMA}
    with pytest.raises(ValueError, match="不能切换"):
        instance.publish(token, webxr_hello)
    with pytest.raises(ValueError, match="schema"):
        instance.publish_quest_adb(token, webxr_hello)
    webxr_hello["stream_id"] = "browser-b"
    instance.publish(token, webxr_hello)
    metadata = instance.latest("pc")["source_metadata"]
    assert metadata["source_joint_count"] == 25
    assert "transport" not in metadata
    with pytest.raises(ValueError, match="不能切换"):
        instance.publish_quest_adb(token, event(sequence=3, stream="browser-b"))
    instance.publish_quest_adb(token, event(stream="adb-c"))
    assert instance.latest("pc")["source_metadata"]["source_joint_count"] == 21


def test_new_stream_requires_hello_and_pair_revocation_rejects_old_token(relay):
    instance, token, _ = relay
    with pytest.raises(ValueError, match="必须先发送"):
        instance.publish_quest_adb(token, skeleton(stream="adb-new"))
    instance.close("pc")
    with pytest.raises(ValueError, match="失效"):
        instance.publish_quest_adb(token, skeleton())
