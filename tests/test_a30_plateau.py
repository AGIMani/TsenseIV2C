"""Regression for the 2026-09-23 14:56:28 recording's contact deadlock."""
import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "src")]
from hand_profiles import load_hand_profile
from online_control.online_motion_track.a30.control import A30Control


def test_recorded_retreat_does_not_freeze_both_contact_fingers():
    case = json.loads((ROOT / "tests/fixtures/a30_plateau_145628.json").read_text())
    c = A30Control(load_hand_profile("TsenseI_v3_0"), "left", "mink", start=False)
    try:
        c._seed(case["seed_pose"])
        c.target = dict(c.pose)
        name = "left_hand_index_swing_joint"
        initial = c.pose[name]
        target_index = 0
        first_motion = None
        for tick in range(1500):
            stamp = case["targets"][0]["t"] + tick * .002
            while target_index < len(case["targets"]) and case["targets"][target_index]["t"] <= stamp:
                row = case["targets"][target_index]
                c.push(row["joints"], row["frame_id"])
                target_index += 1
            c.step(.002)
            assert c.valid, c.error
            # Independent geometry check of every published sample: no relaxed
            # penetration allowance or unchecked escape pose is acceptable.
            c.guard.filter.sync_pose_from_dict(c.pose)
            assert c.guard.filter.data.ncon == 0
            assert all(lo <= c.pose[n] <= hi for n, (lo, hi) in c.limits.items())
            assert len(c.wire) == 108
            if first_motion is None and abs(c.pose[name] - initial) > .01:
                first_motion = tick * .002
        assert first_motion is not None and first_motion < .25
        assert c.pose[name] > .2
    finally:
        c.close()


@pytest.mark.parametrize("side", ["left", "right"])
def test_separating_finger_moves_but_blocking_finger_stays(side):
    c = A30Control(load_hand_profile("TsenseI_v3_0"), side, "stop", start=False)
    try:
        origin = dict(c.pose)
        blocked = f"{side}_hand_thumb_PIP_joint"
        retreat = f"{side}_hand_index_PIP_joint"
        def guard(pose):
            hit = abs(pose[blocked] - origin[blocked]) > 1e-8
            c.guard.diagnostics = {"limited": hit, "contacts": [
                {"g1": f"{side}_hand_thumb_PIP_link_capsule",
                 "g2": f"{side}_hand_index_DIP_link_capsule"}] if hit else []}
            return dict(origin) if hit else dict(pose)
        c.guard.project = guard
        target = dict(origin); target[blocked] = .2; target[retreat] = .2
        c.push(target, 1)
        for _ in range(150):
            c.step(.002)
            assert c.valid, c.error
            assert c.pose[blocked] == origin[blocked]
        assert set(c.finger_holds) == {"thumb"}
        assert c.pose[retreat] > .05
    finally:
        c.close()
