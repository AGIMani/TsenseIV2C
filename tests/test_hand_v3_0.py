"""A3.0 CAD preservation, reflection, retarget and web isolation regressions."""
from pathlib import Path
import json
import sys
import threading
import urllib.request

import numpy as np
import pytest
import trimesh

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "src"), str(ROOT / "tools")]
from hand_profiles import load_hand_profile
from TsenseI_retargeting.fk.urdf import UrdfModel
from TsenseI_retargeting.fk.fk import RobotKeypointFK
from prepare_hand_v3_0 import SOURCE_URDF, RENAMES, MIRROR, right_name
from online_control.online_motion_track.main import WebSession, make_server, model_description
from online_control.online_motion_track.pipeline_v3_0.actuation import A30Actuation, ActuatorTargets


def test_preserve_source_geometry_and_mirror_all_link_poses():
    profile = load_hand_profile("TsenseI_v3_0")
    source = UrdfModel(SOURCE_URDF)
    left, right = [UrdfModel(profile.path("urdf", s)) for s in ("left", "right")]
    assert len(left.joint_names()) == 20
    assert not left.missing_meshes() and not right.missing_meshes()
    reflection = np.eye(4); reflection[:3, :3] = MIRROR
    rng = np.random.default_rng(30)
    for _ in range(5):
        q = {n: rng.uniform(lo, hi) for n, (lo, hi) in source.joint_limits().items()}
        ql = {RENAMES.get(n, n): v for n, v in q.items()}
        a, b, c = source.forward_kinematics(q), left.forward_kinematics(ql), right.forward_kinematics({right_name(n): v for n, v in ql.items()})
        for name, pose in a.items():
            new = RENAMES.get(name, name)
            np.testing.assert_allclose(b[new], pose, atol=1e-12)
            np.testing.assert_allclose(c[right_name(new)], reflection @ pose @ reflection, atol=1e-12)
    for name, visuals in source.visual_meshes_by_link().items():
        ln = RENAMES.get(name, name)
        for original, lv, rv in zip(visuals, left.visual_meshes_by_link()[ln], right.visual_meshes_by_link()[right_name(ln)], strict=True):
            assert original.path.read_bytes() == lv.path.read_bytes()
            lm, rm = trimesh.load_mesh(lv.path, process=False), trimesh.load_mesh(rv.path, process=False)
            np.testing.assert_allclose(rm.triangles, lm.triangles[:, ::-1] @ MIRROR, atol=1e-8)


def test_keypoints_follow_mirrored_motion_and_display_meshes_load():
    profile = load_hand_profile("TsenseI_v3_0")
    left, right = [RobotKeypointFK(profile.path("urdf", s), profile.path("keypoints", s)) for s in ("left", "right")]
    for fraction in (0, .4, .8):
        q = {n: lo + fraction * (hi - lo) for n, (lo, hi) in profile.joint_limits().items()}
        np.testing.assert_allclose(right.compute({right_name(n): v for n, v in q.items()}), left.compute(q) @ MIRROR, atol=1e-12)
    visuals = UrdfModel(profile.path("urdf")).visual_meshes_by_link()
    meshes = [trimesh.load_mesh(v.path, process=False) for group in visuals.values() for v in group]
    assert len(meshes) == 22
    assert all(len(mesh.triangles) > 0 for mesh in meshes)


@pytest.mark.parametrize("side", ["left", "right"])
def test_web_retarget_uses_a30_and_records_without_hardware(side, tmp_path):
    session = WebSession(side, True, tmp_path, model_id="TsenseI_v3_0")
    try:
        solver = session.pipeline.session.retargeter
        assert solver._analytic_kinematics is not None
        limits = session.pipeline.session.limits
        outputs = []
        for i in range(10):
            pose = {n: min(.7, hi) if i >= 5 and "_mid_" in n and "swing" not in n else 0 for n, (lo, hi) in limits.items()}
            world = solver.fk.compute(pose)
            result = session.process({"sequence": i, "timestamp_ms": 1000 + 50*i,
                "capture_unix_ms": 1789600000000 + 50*i,
                "hand": {"label": side.title(), "world": world.tolist(), "normalized": world.tolist()}})
            assert result["tracking"], result["message"]
            assert len(result["pose"]) == 20
            assert all(lo <= result["pose"][n] <= hi for n, (lo, hi) in limits.items())
            assert result["actuation"]["model_id"] == "TsenseI_v3_0"
            assert result["actuation"]["valid"]
            assert len(result["actuation"]["linear_strokes_mm"]) == 9
            assert len(result["actuation"]["rotary_positions_deg"]) == 11
            assert not result["actuation"]["hardware_enabled"]
            outputs.append(result["pose"])
        assert abs(outputs[-1][f"{side}_hand_mid_PIP_joint"] - outputs[4][f"{side}_hand_mid_PIP_joint"]) > .1
        with pytest.raises(RuntimeError):
            session.pipeline.session.send_to_hardware()
    finally:
        session.close()
    rows = [json.loads(line) for line in session.recorder.path.read_text(encoding="utf-8").splitlines()]
    assert rows[0]["model_id"] == "TsenseI_v3_0"
    assert rows[0]["schema_name"] == "TsenseI_v3_0_angle_comparison"
    assert rows[-1]["frame_count"] == 10


def test_actuation_converts_and_encodes_without_transport():
    adapter = A30Actuation("left")
    result = adapter.convert(load_hand_profile("TsenseI_v3_0").init_pose(), hand_side="left")
    assert len(result.linear_strokes_mm) == 9 and len(result.rotary_positions_deg) == 11
    with pytest.raises(ValueError):
        adapter.convert({}, hand_side="left")
    assert len(adapter.encode(result, hand_side="left")) == 108
    with pytest.raises(ValueError):
        adapter.encode(ActuatorTargets({}, {}), hand_side="left")
    with pytest.raises(ValueError):
        WebSession(recording=False, model_id="TsenseI_v3_0", collision_mode="unknown")


def test_a30_http_assets_session_and_v2_control_isolation():
    server = make_server(0)
    thread = threading.Thread(target=server.serve_forever, daemon=True); thread.start()
    base = f"http://127.0.0.1:{server.server_port}"
    def get(path):
        with urllib.request.urlopen(base + path) as response:
            return response.read()
    try:
        assert "TsenseI_v3_0" in json.loads(get("/api/health"))["models"]
        for side in ("left", "right"):
            desc = json.loads(get(f"/api/model?model=TsenseI_v3_0&side={side}"))
            for link in desc["links"]:
                for v in link["visuals"]:
                    assert v["mesh"].startswith("/assets-v3-0/")
                    assert len(get(v["mesh"])) > 84
        response = server.application.create({"model_id": "TsenseI_v3_0", "recording": False})
        assert response["model_id"] == "TsenseI_v3_0"
        with pytest.raises(ValueError):
            server.application.v2_command({"session": response["session"], "action": "connect"})
    finally:
        server.application.close(); server.shutdown(); server.server_close(); thread.join()


def test_portable_bundle_contains_a30_model_pipeline_and_calibration():
    from online_control.online_motion_track.package import files
    for full in (False, True):
        manifest = dict((name, path) for path, name in files(full=full))
        for relative in ("src/a30_kinematics/mapper.py", "online_control/online_motion_track/pipeline_v3_0/actuation.py", "online_control/online_motion_track/pipeline_v3_0/pipeline.py",
                         "assets/hands/TsenseI_hand_v3_0/robot_keypoints_21.yaml"):
            assert manifest["app/" + relative].is_file()
        for side in ("left", "right"):
            desc = model_description(side, "TsenseI_v3_0")
            for link in desc["links"]:
                for v in link["visuals"]:
                    relative = "assets/hands/TsenseI_hand_v3_0/" + v["mesh"].removeprefix("/assets-v3-0/")
                    assert "app/" + relative in manifest
