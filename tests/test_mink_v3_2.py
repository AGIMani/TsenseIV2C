"""Exercise actual Mink constraints against both V3.2 collision models."""
import json
from pathlib import Path
import sys

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "src")]
pytest.importorskip("mink")
from hand_profiles import load_hand_profile
from online_control.online_motion_track.mink_guard import MinkCollisionGuard
from online_control.online_motion_track.main import WebSession, Application


def test_recorded_a32_mesh_pose_uses_same_distance_query_as_final_validation():
    guard = MinkCollisionGuard(load_hand_profile('TsenseI_v3_2_withid'), 'left')
    pose = json.loads((ROOT / 'tests/fixtures/wuji_a32_mesh_boundary.json').read_text())
    q = guard._vector(pose)
    reference = bool(np.all(guard._distances(q) >= guard.margins - 1e-7))
    # The fitted outer hulls no longer collide at this old capsule-era boundary.
    # Keep this recorded pose as a regression for consistent horizon/cache use.
    assert reference
    guard._update(q)
    guard.mj.mj_collision(guard.model, guard.configuration.data)
    assert not any(contact.dist < 0 for contact in guard.configuration.data.contact)
    assert guard._feasible(q) == reference
    projected = guard.project(pose)
    assert np.all(guard.distances(projected) >= guard.margins - 1e-7)


@pytest.mark.parametrize('model', ['TsenseI_v3_0', 'TsenseI_v3_0_withid', 'TsenseI_v3_2', 'TsenseI_v3_2_withid'])
@pytest.mark.parametrize('side', ['left', 'right'])
def test_fast_feasibility_matches_all_pair_clearance_validation(model, side):
    guard = MinkCollisionGuard(load_hand_profile(model), side)
    rng = np.random.default_rng(9029)
    for _ in range(150):
        q = rng.uniform(guard.lower, guard.upper)
        reference = bool(np.all(guard._distances(q) >= guard.margins - 1e-7))
        assert guard._feasible(q) == reference


@pytest.mark.parametrize("side", ["left", "right"])
def test_converging_fingers_are_separated_and_release_without_lag(side):
    profile = load_hand_profile("TsenseI_v3_2")
    guard = MinkCollisionGuard(profile, side)
    pose = profile.init_pose(side)
    pose[f"{side}_hand_ring_swing_joint"] = .695
    # Pinky swing names the lateral hinge in both project models.
    pose[f"{side}_hand_pinky_swing_joint"] = .005
    assert guard.distances(pose).min() < -.01
    result = guard.project(pose)
    assert np.all(guard.distances(result) >= guard.margins - 1e-7)
    assert guard.diagnostics["correction_deg"] > 1
    opened = profile.init_pose(side)
    assert guard.project(opened) == opened
    assert guard.diagnostics["correction_deg"] == 0


@pytest.mark.parametrize("side", ["left", "right"])
def test_thumb_and_random_fast_targets_stay_feasible(side):
    profile = load_hand_profile("TsenseI_v3_2")
    guard = MinkCollisionGuard(profile, side)
    rng = np.random.default_rng(731)
    collisions = thumb_collisions = 0
    for _ in range(50):
        pose = {n: rng.uniform(lo, hi) for n, (lo, hi) in profile.joint_limits(side).items()}
        raw = guard.distances(pose)
        collisions += int(raw.min() < 0)
        thumb_collisions += sum(d < 0 and any("thumb" in guard.model.geom(g).name for g in pair)
                                for d, pair in zip(raw, guard.pairs))
        before = guard.q.copy()
        result = guard.project(pose)
        assert np.all(guard.distances(result) >= guard.margins - 1e-7)
        # Also check a straight joint-space segment between consecutive outputs.
        for t in np.linspace(0, 1, 11):
            intermediate = before + t * (guard.q - before)
            # The internal multi-step path alone is insufficient if a downstream
            # consumer interpolates directly between published poses.
            assert guard._distances(intermediate).min() >= -1e-6
    assert collisions > 20 and thumb_collisions > 0


def test_failed_qp_holds_safe_pose_and_nonfinite_input_is_rejected():
    p = load_hand_profile("TsenseI_v3_2")
    g = MinkCollisionGuard(p, "left")
    q = p.init_pose()
    q["left_hand_ring_swing_joint"] = .695
    q["left_hand_pinky_swing_joint"] = .005
    g.solve_qp = lambda *args, **kwargs: None
    assert g.project(q) == p.init_pose()
    assert g.diagnostics["status"] == "held_solver_failure"
    q["left_hand_ring_swing_joint"] = float("nan")
    with pytest.raises(ValueError, match="有限"):
        g.project(q)


def test_web_records_raw_and_guarded_angles_and_keeps_corrected_history(tmp_path):
    s = WebSession("left", True, tmp_path, collision_mode="mink")
    solver = s.pipeline.session.retargeter
    q = s.pipeline.session.profile.init_pose()
    q["left_hand_ring_swing_joint"] = .695
    q["left_hand_pinky_swing_joint"] = .005
    world = solver.fk.compute(q).tolist()
    try:
        for i in range(3):
            result = s.process({"sequence": i, "timestamp_ms": 1000 + i * 50,
                "capture_unix_ms": 1789600000000 + i * 50,
                "hand": {"label": "Left", "world": world, "normalized": world}})
            assert result["tracking"], result["message"]
            assert result["collision"]["mode"] == "mink"
            np.testing.assert_allclose(solver.previous_q, [result["pose"][n] for n in solver.joint_names])
    finally:
        s.close()
    rows = [json.loads(line) for line in s.recorder.path.read_text(encoding="utf-8").splitlines()]
    assert rows[1]["collision_diagnostics"]["mode"] == "mink"
    angles = [row for row in rows if row["type"] == "joint_angles"]
    assert {row["stage"] for row in angles} == {"retarget", "mink"}
    assert len(angles) == 6
    assert all(row["joints"] for row in angles)
    assert angles[-1]["joints"] == result["mink_joint_angles_rad"]


def test_invalid_mode_never_silently_disables_collision():
    app = Application()
    with pytest.raises(ValueError, match="(避碰|碰撞)模式"):
        app.create({"collision_mode": "typo", "recording": False})
    assert not app.sessions
