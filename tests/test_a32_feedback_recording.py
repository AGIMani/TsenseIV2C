"""Normalize A3.2 measured positions without changing SDK evidence or commands."""
import copy
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "src")]

from a32_driver.recording import default_channels, recording_metadata, telemetry_event


def test_sdk_1_1_14_currents_are_recorded_in_amperes_without_second_conversion():
    channels = default_channels()
    state = {"connected": True, "telemetry_age_ms": 1,
             "telemetry_format": "a32-native-sdk-state/v2",
             "telemetry_units": {"self_current": "A", "external_current": "A"},
             "self": [{"online": True, "position": 1., "iq": -.1374, "iq_ref": .2748,
                       "current_unit": "A"} for _ in range(18)],
             "mit": [{"online": True, "valid": True, "position": 1., "iq": -2.75,
                      "current_unit": "A"} for _ in range(2)]}
    original = copy.deepcopy(state)
    event = telemetry_event(state, channels)
    assert event["telemetry_format"] == "a32-native-sdk-state/v2"
    assert all(row["current_unit"] == "A" for row in event["linear"] + event["rotary"])
    assert event["linear"][0]["iq_meas"] == -.1374
    assert event["linear"][0]["iq_ref"] == .2748
    assert event["rotary"][0]["iq"] == -.1374
    assert event["rotary"][4]["iq"] == -2.75
    assert event["sdk_state"] == state == original
    metadata = recording_metadata("TsenseI_v3_2", "left", channels)
    assert metadata["telemetry_units"]["self_current"] == "A"
    assert metadata["homing_current_unit"] == "counts"


@pytest.mark.parametrize("format", [None, "a32-native-sdk-state/v1"])
def test_old_state_captures_keep_native_self_current_units(format):
    state = {"connected": True, "telemetry_age_ms": 1, "self": [
        {"online": True, "position": 1., "iq": -100., "iq_ref": 200.}]}
    if format is not None:
        state["telemetry_format"] = format
    event = telemetry_event(state, default_channels())
    assert event["linear"][0]["iq_meas"] == -100.
    assert event["linear"][0]["current_unit"] == "SDK native"
    assert event["telemetry_units"]["self_current"] == "SDK native"
    assert event["rotary"][4]["current_unit"] == "A"


@pytest.mark.parametrize("side", ["left", "right"])
def test_all_linear_feedback_positions_are_negated_once_and_raw_sdk_is_preserved(side):
    channels = default_channels(side)
    state = {"connected": True, "telemetry_age_ms": 1, "self": [], "mit": []}
    for group, count in (("self", 18), ("mit", 2)):
        state[group] = [{"online": True, "valid": True, "position": -i - .25,
                         "target": -i - .5, "iq": -2., "iq_ref": 3., "temperature_centi": 3500}
                        for i in range(count)]
    original = copy.deepcopy(state)
    event = telemetry_event(state, channels)
    assert event["linear_feedback_sign"] == -1
    assert len(event["linear"]) == 14 and len(event["rotary"]) == 6
    for row in event["linear"]:
        assert row["pos_act_mm"] == -row["position"]
        assert row["pos_ref_mm"] == -row["target"]
        assert row["iq_meas"] == -2. and row["iq_ref"] == 3.
        assert row["mcu_temp_c"] == 35.
    for row in event["rotary"]:
        assert row["pos_actual_mm"] == row["position"]
        assert row["pos_ref_mm"] == row["target"]
    assert event["sdk_state"] == state == original
    event["sdk_state"]["self"][0]["position"] = 999
    assert state == original
    assert recording_metadata("TsenseI_v3_2", side, channels)["linear_feedback_sign"] == -1


@pytest.mark.parametrize("mode", ["zero", "offline", "stale", "nonfinite", "missing"])
def test_linear_sign_keeps_zero_and_missing_measurements(mode):
    channels = default_channels()
    state = {"connected": True, "telemetry_age_ms": 1, "self": [
        {"online": True, "position": 0., "target": 0.}], "mit": []}
    if mode == "offline":
        state["self"][0]["online"] = False
    elif mode == "stale":
        state["telemetry_age_ms"] = 501
    elif mode == "nonfinite":
        state["self"][0].update(position=float("nan"), target=float("inf"))
    elif mode == "missing":
        state["self"][0].pop("position")
        state["self"][0].pop("target")
    row = telemetry_event(state, channels)["linear"][0]
    expected = 0. if mode == "zero" else None
    assert row["pos_act_mm"] == expected
    assert row["pos_ref_mm"] == expected
    assert row["measurement_valid"] == (mode == "zero")
