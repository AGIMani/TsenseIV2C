"""The web skeleton follows solved FK, including V2's shared-left right hand."""
from pathlib import Path
import sys

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "src")]
from hand_profiles import A30_MODEL_IDS, A32_MODEL_IDS
from online_control.online_motion_track.main import WebSession
from online_control.online_motion_track.optimized_skeleton import optimized_skeleton
from online_control.online_motion_track.v2.session import V2WebSession
from test_web_v3_2 import packet
from test_web_v2 import MemoryUsb, packet as v2_packet


@pytest.mark.parametrize("model", (*A30_MODEL_IDS, *A32_MODEL_IDS))
@pytest.mark.parametrize("side", ["left", "right"])
def test_web_frame_exposes_solved_points_and_clears_on_loss(model, side):
    session = WebSession(side, recording=False, model_id=model)
    solver = session.pipeline.session.retargeter
    world = solver._robot_points_palm_m(.7*solver.lower+.3*solver.upper)
    try:
        result = session.process(packet(world, side))
        assert result["tracking"]
        expected = solver.fk.compute(result["retarget_joint_angles_rad"])
        expected -= expected[0].copy()
        np.testing.assert_allclose(result["optimized_points"], expected, atol=1e-10)
        assert not np.allclose(result["optimized_points"], world)
        # Display evaluation must use explicit solved angles, not seeded history.
        solver.previous_q = solver.lower.copy()
        np.testing.assert_allclose(optimized_skeleton(solver, result["retarget_joint_angles_rad"]), expected)
        lost = packet(world, side, 1)
        lost["hand"] = None
        assert session.process(lost)["optimized_points"] is None
        wrong = packet(world, "right" if side == "left" else "left", 2)
        assert session.process(wrong)["optimized_points"] is None
    finally:
        session.close()


@pytest.mark.parametrize("side", ["left", "right"])
def test_v2_frame_displays_the_current_solution_in_physical_hand_chirality(side):
    session = V2WebSession(side, recording=False, collision_mode="off", usb=MemoryUsb())
    try:
        session.command("tracking_start", {})
        for index in range(3):  # Existing tracking gate requires three valid frames.
            result = session.process(v2_packet(session, index))
        assert result["tracking"], result["message"]
        raw = {name.replace("right_hand_", "left_hand_", 1): value
               for name, value in result["retarget_joint_angles_rad"].items()}
        expected = session.solver.fk.compute(raw)
        expected -= expected[0].copy()
        if side == "right":
            expected[:, 0] *= -1
        np.testing.assert_allclose(result["optimized_points"], expected, atol=1e-10)
        lost = v2_packet(session, 3)
        lost["hand"] = None
        assert session.process(lost)["optimized_points"] is None
    finally:
        session.close()
