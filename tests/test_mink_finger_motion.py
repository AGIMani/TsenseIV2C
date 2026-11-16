"""Contact-local projection: free fingers track, touching chains can articulate."""
from pathlib import Path
from types import SimpleNamespace
import sys

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "src")]
mujoco = pytest.importorskip("mujoco")
pytest.importorskip("mink")
from hand_profiles import load_hand_profile
from online_control.online_motion_track.mink_guard import MinkCollisionGuard


def assert_safe_motion(guard, before, output):
    """Check the final clearance and a separate MuJoCo contact pipeline."""
    after = np.array([output[n] for n in guard.names])
    assert np.all(guard.distances(output) >= guard.margins - 1e-7)
    data = mujoco.MjData(guard.model)
    for t in np.linspace(0, 1, 51):
        data.qpos[guard.qadr] = before + t * (after - before)
        mujoco.mj_forward(guard.model, data)
        assert not any(c.dist < -1e-7 for c in data.contact)


def converging_pose(profile, side):
    pose = profile.init_pose(side)
    pose[f"{side}_hand_ring_swing_joint"] = .695
    pose[f"{side}_hand_pinky_swing_joint"] = .005
    return pose


@pytest.mark.parametrize("side", ["left", "right"])
def test_free_fingers_track_while_touching_fingers_bend_together(side):
    profile = load_hand_profile("TsenseI_v3_2_withid")
    guard = MinkCollisionGuard(profile, side)
    target = converging_pose(profile, side)
    for _ in range(3):
        guard.project(target)
    flexion = [f"{side}_hand_{finger}_{link}_joint"
               for finger in ("ring", "pinky") for link in ("MCP", "PIP", "DIP")]
    free = [n for n in guard.names if not any(f in n for f in ("ring", "pinky"))]
    for name in flexion:
        target[name] = .3
    # Retain the colliding lateral target while flexing the contacting fingers
    # and changing all unrelated targets. A whole-finger stop cannot pass this.
    for thumb in (.25, .4, .1, .35, .2):
        for name in free:
            if "thumb" in name:
                target[name] = thumb
        target[f"{side}_hand_index_MCP_joint"] = .1
        target[f"{side}_hand_mid_MCP_joint"] = .075
        before = guard.q.copy()
        result = guard.project(target)
        np.testing.assert_allclose([result[n] for n in free], [target[n] for n in free], atol=1e-12)
        assert_safe_motion(guard, before, result)
        assert all(guard.diagnostics["correction_by_finger_deg"][f] < 1e-8
                   for f in ("thumb", "index", "mid"))
    assert all(result[n] > .27 for n in flexion)
    # The lateral collision remains constrained; both fingers share the change.
    assert result[f"{side}_hand_ring_swing_joint"] < guard._vector(target)[guard.names.index(
        f"{side}_hand_ring_swing_joint")] - .01
    assert result[f"{side}_hand_pinky_swing_joint"] > target[f"{side}_hand_pinky_swing_joint"] + .01
    assert guard.diagnostics["blocking_pairs"]
    # Moving away releases immediately, without a sticky collision state.
    opened = profile.init_pose(side)
    before = guard.q.copy()
    assert guard.project(opened) == opened
    assert_safe_motion(guard, before, opened)


@pytest.mark.parametrize("side", ["left", "right"])
def test_solver_failure_holds_only_blocked_chains(side, monkeypatch):
    profile = load_hand_profile("TsenseI_v3_2_withid")
    guard = MinkCollisionGuard(profile, side)
    target = converging_pose(profile, side)
    for name in target:
        if "thumb" in name:
            target[name] = .4
    target[f"{side}_hand_index_MCP_joint"] = .2
    target[f"{side}_hand_mid_MCP_joint"] = .1
    before = guard.q.copy()
    monkeypatch.setattr(guard, "solve_qp", lambda *a, **kw: None)
    result = guard.project(target)
    for i, name in enumerate(guard.names):
        expected = before[i] if any(f in name for f in ("ring", "pinky")) else target[name]
        assert result[name] == expected
    assert guard.diagnostics["status"] == "held_solver_failure"
    assert_safe_motion(guard, before, result)


@pytest.fixture
def crossing_profile(tmp_path):
    # Two tiny spheres cross between the original .025-rad samples; a third
    # independent chain remains far away. Primitive distances are deterministic
    # and do not depend on the hand mesh revision or GJK degeneracies.
    path = tmp_path / "crossing.xml"
    path.write_text(f'''<mujoco>
      <compiler angle="radian"/>
      <default><joint type="hinge" axis="0 0 1" range="-1 1"/>
        <geom type="sphere" size=".0001"/></default>
      <worldbody>
        <body name="index"><joint name="left_hand_index_swing_joint"/>
          <inertial pos="0 0 0" mass=".1" diaginertia=".01 .01 .01"/>
          <geom name="index_contact" pos="1 0 0"/></body>
        <body name="mid"><joint name="left_hand_mid_swing_joint"/>
          <inertial pos="0 0 0" mass=".1" diaginertia=".01 .01 .01"/>
          <geom name="mid_contact" pos="{np.cos(.004)} {np.sin(.004)} 0"/></body>
        <body name="thumb" pos="0 0 1"><joint name="left_hand_thumb_swing_joint"/>
          <inertial pos="0 0 0" mass=".1" diaginertia=".01 .01 .01"/>
          <geom name="thumb_contact" pos=".1 0 0"/></body>
      </worldbody></mujoco>''', encoding="utf-8")
    initial = {"left_hand_index_swing_joint": -.012, "left_hand_mid_swing_joint": 0.,
               "left_hand_thumb_swing_joint": 0.}
    return SimpleNamespace(model_id="crossing", path=lambda *args: path,
                           joint_limits=lambda side: {n: (-1., 1.) for n in initial},
                           init_pose=lambda side: dict(initial))


@pytest.mark.parametrize("budget", [1, 128])
def test_swept_collision_and_exhausted_budget_leave_free_finger_moving(crossing_profile, budget):
    guard = MinkCollisionGuard(crossing_profile, "left", max_path_refinements=budget)
    start = guard.q.copy()
    target = dict(zip(guard.names, start))
    target["left_hand_index_swing_joint"] = .012
    target["left_hand_thumb_swing_joint"] = .02
    # Test the crossing without the thumb first, so the coarse grid has no
    # interior samples that could accidentally catch the small sphere contact.
    crossing = start.copy()
    crossing[0] = .012
    assert guard._feasible(start) and guard._feasible(crossing)
    assert not guard._path_feasible(start, crossing)
    if budget == 1:
        assert guard._path_budget_exhausted
    spent = guard._path_refinements
    assert not guard._path_feasible(start, crossing)
    assert guard._path_refinements == spent
    result = guard.project(target)
    assert result["left_hand_thumb_swing_joint"] == .02
    assert guard.diagnostics["path_refinements"] <= budget
    if budget == 1:
        assert guard.diagnostics["path_budget_exhausted"]
    assert_safe_motion(guard, start, result)


@pytest.mark.parametrize("side", ["left", "right"])
def test_combined_finger_paths_remain_safe_for_fast_targets(side):
    profile = load_hand_profile("TsenseI_v3_2_withid")
    guard = MinkCollisionGuard(profile, side)
    rng = np.random.default_rng(9017)
    for _ in range(30):
        target = {n: rng.uniform(lo, hi) for n, (lo, hi) in profile.joint_limits(side).items()}
        before = guard.q.copy()
        output = guard.project(target)
        assert_safe_motion(guard, before, output)
