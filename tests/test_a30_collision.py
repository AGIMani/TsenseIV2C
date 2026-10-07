from pathlib import Path
import sys
import json

import numpy as np
import pytest
import mujoco

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "src")]
from hand_profiles import load_hand_profile
from TsenseI_retargeting.fk.urdf import UrdfModel
from online_control.online_motion_track.pipeline_v3_0.collision import StopCollisionGuard
from online_control.online_motion_track.mink_guard import MinkCollisionGuard
from online_control.online_motion_track.main import WebSession


@pytest.mark.parametrize("side", ["left", "right"])
def test_mjcf_matches_urdf_at_random_angles(side):
    p = load_hand_profile("TsenseI_v3_0")
    model = mujoco.MjModel.from_xml_path(str(p.path("mjcf", side)))
    data = mujoco.MjData(model); urdf = UrdfModel(p.path("urdf", side))
    assert model.nq == model.nu == 20 and model.ngeom == 16
    rng = np.random.default_rng(30)
    for i in range(12):
        q = p.init_pose(side) if i == 0 else {n: rng.uniform(lo, hi) for n, (lo, hi) in p.joint_limits(side).items()}
        for n, v in q.items():
            j = model.joint(n); data.qpos[j.qposadr[0]] = v
            np.testing.assert_allclose(j.range, p.joint_limits(side)[n])
        mujoco.mj_forward(model, data)
        if i == 0: assert data.ncon == 0
        for name, pose in urdf.forward_kinematics(q).items():
            b = data.body(name)
            np.testing.assert_allclose(b.xpos, pose[:3, 3], atol=1e-12)
            np.testing.assert_allclose(b.xmat.reshape(3, 3), pose[:3, :3], atol=1e-12)


def crossed(p, side):
    q = p.init_pose(side)
    q[f"{side}_hand_ring_swing_joint"] = -.35
    q[f"{side}_hand_pinky_swing_joint"] = .35
    return q


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("mode", ["stop", "mink"])
def test_collisions_are_limited_including_motion_and_release(side, mode):
    p = load_hand_profile("TsenseI_v3_0")
    geometry = MinkCollisionGuard(p, side)
    guard = geometry if mode == "mink" else StopCollisionGuard(p, side)
    assert len(geometry.pairs) == 100
    target = crossed(p, side)
    assert geometry.distances(target).min() < -.01
    current = p.init_pose(side)
    rng = np.random.default_rng(301)
    thumb_contacts = 0
    targets = [target] + [{n: rng.uniform(lo, hi) for n, (lo, hi) in p.joint_limits(side).items()} for _ in range(30)]
    for target in targets:
        distances = geometry.distances(target)
        thumb_contacts += sum(d < 0 and any("thumb" in geometry.model.geom(g).name for g in pair)
                              for d, pair in zip(distances, geometry.pairs))
        result = guard.project(target)
        margin = geometry.margins if mode == "mink" else 0
        assert np.all(geometry.distances(result) >= margin - 1e-7)
        for alpha in np.linspace(0, 1, 21):
            point = {n: current[n] + alpha*(result[n] - current[n]) for n in current}
            assert geometry.distances(point).min() >= -1e-6
        current = result
    assert thumb_contacts > 0
    # Start from the known converging-fingers case to check prompt release.
    guard = MinkCollisionGuard(p, side) if mode == "mink" else StopCollisionGuard(p, side)
    guard.project(crossed(p, side))
    assert guard.project(p.init_pose(side)) == p.init_pose(side)


def test_mirrored_bounds_and_safety_agree_and_pinch_pairs_remain_enabled():
    p = load_hand_profile("TsenseI_v3_0")
    left, right = (MinkCollisionGuard(p, s) for s in ("left", "right"))
    rng = np.random.default_rng(17)
    for _ in range(10):
        q = {n: rng.uniform(lo, hi) for n, (lo, hi) in p.joint_limits().items()}
        lq = left._vector(q)
        rq = right._vector({n.replace('left_', 'right_'): v for n, v in q.items()})
        # A conservative certificate and a native near-pair distance can differ
        # across mirrored GJK queries. Compare geometric bounds and the safety
        # decision, rather than treating diagnostic lower bounds as metrology.
        np.testing.assert_allclose(left._bounds_at(lq), right._bounds_at(rq), atol=1e-12)
        assert left._feasible(lq) == right._feasible(rq)
    for finger in ("index", "mid", "ring", "pinky"):
        expected = {"left_hand_thumb_PIP_link", f"left_hand_{finger}_DIP_link"}
        assert any({left.model.body(left.model.geom_bodyid[g]).name for g in pair} == expected
                   for pair in left.pairs)


@pytest.mark.parametrize("mode", ["monitor", "stop", "mink"])
def test_web_conversion_and_log_use_post_collision_pose(mode, tmp_path):
    s = WebSession(recording=True, log_dir=tmp_path, collision_mode=mode, model_id="TsenseI_v3_0")
    try:
        world = s.pipeline.session.retargeter.fk.compute(crossed(s.pipeline.session.profile, "left")).tolist()
        result = s.process({"sequence": 0, "timestamp_ms": 1000, "capture_unix_ms": 1789600000000,
                            "hand": {"label": "Left", "world": world, "normalized": world}})
        assert result["tracking"], result["message"]
        assert result["collision"]["mode"] == mode
        expected = s.pipeline.stroke_mapper.preview(result["pose"])
        assert result["actuation"]["linear_strokes_mm"] == expected["linear_strokes_mm"]
        assert result["actuation"]["rotary_positions_deg"] == expected["rotary_positions_deg"]
        if mode == "monitor":
            assert result["pose"] == result["retarget_joint_angles_rad"]
    finally: s.close()
    row = json.loads(s.recorder.path.read_text(encoding="utf-8").splitlines()[1])
    assert row["collision_diagnostics"]["mode"] == mode
