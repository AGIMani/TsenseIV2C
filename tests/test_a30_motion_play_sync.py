"""Gesture targets and ranges must match the online motion player's wire policy."""
from dataclasses import replace
from pathlib import Path
import struct
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "src")]
from a30_kinematics.downlink import channel_mapping, crc16_modbus
from online_control.online_motion_track.pipeline_v3_0.actuation import A30Actuation
from online_control.online_motion_track.a30.control import A30Control
from hand_profiles import load_hand_profile
from online_control.online_motion_play.downlink_policy import a30_ranges, encode_online_targets


@pytest.mark.parametrize("side", ["left", "right"])
def test_gesture_frames_and_limits_match_motion_player(side):
    adapter = A30Actuation(side)
    control = A30Control(load_hand_profile("TsenseI_v3_0"), side, "off", start=False)
    try:
        description = {c["joint"]: c for c in adapter.description()["channels"]}
        ranges = a30_ranges(adapter)
        assert len(description) == len(control.limits) == len(ranges) == 20
        for name, limits in control.limits.items():
            assert list(limits) == description[name]["limits_rad"]
            assert description[name]["downlink"] == ranges[name]

        for fraction in (0., .25, .5, .75, 1.):
            pose = {n: lo + (hi-lo)*fraction for n, (lo, hi) in control.limits.items()}
            targets = adapter.convert(pose)
            for sign in (1, -1):
                signed = replace(targets, linear_strokes_mm={n: sign*v for n, v in targets.linear_strokes_mm.items()})
                frame = adapter.encode(signed, seq=257)
                assert frame == encode_online_targets(signed, hand_side=side, seq=257)
                assert struct.unpack_from("<H", frame, 106)[0] == crc16_modbus(frame[:106])
                for row in channel_mapping(side):
                    offset = 7 + 4*(row["slot"] if row["field"] == "self_pos" else 18+row["slot"])
                    value = struct.unpack_from("<f", frame, offset)[0]
                    spec = ranges[row["joint"]]
                    assert spec["min"]-1e-5 <= value <= spec["max"]+1e-5
                    if row["unit"] == "mm":
                        assert value == pytest.approx(-abs(signed.linear_strokes_mm[row["joint"]]))
                    if row["field"] == "mit_pos_deg":
                        ratio = 50 if row["joint"].endswith("thumb_rotaback_joint2") else 30
                        assert value == pytest.approx(targets.rotary_positions_deg[row["joint"]]*ratio)

        # Out-of-range recognition targets are bounded before interpolation.
        for endpoint, delta in ((0, -1.), (1, 1.)):
            control.push({n: lim[endpoint]+delta for n, lim in control.limits.items()}, endpoint)
            assert control.target == {n: lim[endpoint] for n, lim in control.limits.items()}
    finally:
        control.close()
