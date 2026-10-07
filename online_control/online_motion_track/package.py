"""Allowlisted, relocatable server bundle with vendored browser runtime.

Excludes logs, recordings, personal screenshots, credentials, virtualenvs,
Qt/control applications, test dependencies and unused hand models.
"""
import argparse
import hashlib
import io
import json
from pathlib import Path
import tarfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
DEPLOY_FILES = ("server.py", "package.py", "deploy.sh", "requirements-server.txt", "requirements-full.txt", "requirements-ubuntu.txt", "DEPLOY.md", "THIRD_PARTY_NOTICES.txt", "APACHE-2.0.txt")


def files(*, full=False):
    if (HERE / "manifest.json").exists() and (HERE / "app").is_dir():
        manifest = json.loads((HERE / "manifest.json").read_text(encoding="utf-8"))
        for relative, checksum in manifest["sha256"].items():
            path = (HERE / relative).resolve()
            if not path.is_relative_to(HERE) or not path.is_file():
                raise ValueError(f"Invalid bundle path: {relative}")
            if hashlib.sha256(path.read_bytes()).hexdigest() != checksum:
                raise ValueError(f"Bundle checksum mismatch: {relative}")
            yield path, relative
        return
    for name in DEPLOY_FILES:
        yield HERE / name, name
    for name in ("a32_platform_integration.md", "A32_NATIVE_SDK.md", "a32_motion_play.md", "a32_integration_validation.md",
                 "a32_motion_recording.md", "a32_connection_diagnostics.md", "a32_joint_ranges.md", "a32_gesture_control_policy.md",
                 "online_webxr_quest3.md", "webxr_quest3_input.md", "leapmotion2_input.md", "online_control_latency.md", "motion_limits.md"):
        yield ROOT / "docs" / name, "docs/" + name
    for path in sorted((ROOT / 'online_control/shared').glob('*')):
        if path.suffix in ('.py', '.js', '.json'):
            yield path, 'app/' + path.relative_to(ROOT).as_posix()
    yield ROOT / 'online_control/__init__.py', 'app/online_control/__init__.py'
    shared = ["src/hand_profiles.py", "src/config_path.py", "src/tracking/wuji_mediapipe.py",
              "src/tracking/webxr.py", "src/tracking/cloudxr.py",
              "src/utils/__init__.py", "src/utils/control_log_catalog.py", "src/utils/control_session_recorder.py", "src/utils/mediapipe_frame.py", "src/utils/comm_diag.py",
              "src/utils/log_control.py", "src/utils/tracker_config.py", "src/utils/control_clock.py", "src/utils/motion_limits.py",
              "online_control/online_motion_track/pipeline_v3_2/pipeline.py", "online_control/online_motion_track/pipeline_v3_2/recording.py",
              "online_control/online_motion_track/pipeline_v3_2/skeleton_angles.py", "hand_landmarker.task",
              "TsenseI_retargeting/__init__.py", "TsenseI_retargeting/toporetarget.py", "TsenseI_retargeting/landmarks.py",
              "tools/hand_control_log_viewer.html", "tools/keypoint_preview_solver.py",
              "tools/keypoint_pose_presets.py", "tools/keypoint_mediapipe_input.py"]
    for relative in shared:
        yield ROOT / relative, "app/" + relative
    for path in sorted((ROOT / "TsenseI_retargeting/fk").glob("*.py")):
        yield path, "app/" + path.relative_to(ROOT).as_posix()
    for path in sorted((ROOT / "online_control/online_motion_track/a30").glob("*.py")):
        yield path, "app/" + path.relative_to(ROOT).as_posix()
    for path in sorted((ROOT / "online_control/online_motion_track/a32").glob("*.py")):
        yield path, "app/" + path.relative_to(ROOT).as_posix()
    for path in sorted((ROOT / "src/a32_driver").rglob("*")):
        if any(part in ("build", "build-linux", "staged", "__pycache__") for part in path.relative_to(ROOT / "src/a32_driver").parts):
            continue
        if path.is_file() and (path.suffix.lower() in (".py", ".json", ".cpp", ".h", ".hpp", ".so", ".sh", ".md") or path.name in ("CMakeLists.txt", "SDK_LICENSE.txt")) and "build" not in path.parts:
            yield path, "app/" + path.relative_to(ROOT).as_posix()
    for path in sorted((ROOT / "online_control/online_motion_track/pipeline_v3_0").glob("*.py")):
        yield path, "app/" + path.relative_to(ROOT).as_posix()
    for folder in ("TsenseI_hand_v3_0", "TsenseI_hand_v3_0_withid", "TsenseI_hand_v3_2_withid"):
        for path in sorted((ROOT / "assets/hands" / folder).rglob("*")):
            if path.is_file() and path.suffix.lower() in (".urdf", ".stl", ".yaml", ".xml"):
                yield path, "app/" + path.relative_to(ROOT).as_posix()
    for path in sorted((ROOT / "src/a32_kinematics").glob("*.py")):
        yield path, "app/" + path.relative_to(ROOT).as_posix()
    for path in sorted((ROOT / "src/a30_kinematics").glob("*.py")):
        yield path, "app/" + path.relative_to(ROOT).as_posix()
    for name in ("A3.2_source.urdf", "geometry_inferred.mat"):
        path = ROOT / "src/a32_kinematics/model" / name
        yield path, "app/" + path.relative_to(ROOT).as_posix()
    web = "app/online_control/online_motion_track/"
    yield HERE / "main.py", web + "main.py"
    yield HERE / "feedback.py", web + "feedback.py"
    yield HERE / "calibration.py", web + "calibration.py"
    yield HERE / "landmark_filter.py", web + "landmark_filter.py"
    yield HERE / "direct_filter.py", web + "direct_filter.py"
    yield HERE / "safe_trajectory.py", web + "safe_trajectory.py"
    yield HERE / "collision_queries.py", web + "collision_queries.py"
    yield HERE / "webxr_input.py", web + "webxr_input.py"
    for name in ("wuji_input.py", "wuji_transport.py", "wuji_sdk_source.py", "source_worker.py", "leapmotion_input.py", "leapmotion_source.py"):
        yield HERE / name, web + name
    for path in sorted((HERE / "static").rglob("*")):
        if path.is_file():
            yield path, web + path.relative_to(HERE).as_posix()
    model = ROOT / "assets/hands/TsenseI_hand_v3_2"
    for folder, suffix in (("urdf", ".urdf"), ("meshes", ".stl"), ("mjcf/collision_meshes", ".stl")):
        for path in sorted((model / folder).rglob("*")):
            if path.is_file() and path.suffix.lower() == suffix:
                yield path, "app/" + path.relative_to(ROOT).as_posix()
    for name in ("hand_profile.yaml", "robot_keypoints_21.yaml", "robot_keypoints_21_right.yaml",
                 "eval_config.yaml", "eval_config_right.yaml"):
        yield model / name, "app/" + (model / name).relative_to(ROOT).as_posix()
    vendor = HERE / "node_modules"
    for name in ("three/LICENSE", "three/build/three.module.js", "three/build/three.core.js",
                 "three/examples/jsm/controls/OrbitControls.js", "three/examples/jsm/controls/TransformControls.js", "three/examples/jsm/loaders/STLLoader.js",
                 "@mediapipe/tasks-vision/vision_bundle.mjs", "@mediapipe/tasks-vision/package.json"):
        yield vendor / name, web + "node_modules/" + name
    for path in sorted((vendor / "@mediapipe/tasks-vision/wasm").glob("*")):
        if path.suffix in (".js", ".wasm"):
            yield path, web + "node_modules/" + path.relative_to(vendor).as_posix()
    if full:
        # Runtime-only allowlist: no Qt, user recordings, motion files or keys.
        extra = [
            "online_control/online_motion_track/v2/__init__.py", "online_control/online_motion_track/v2/session.py",
            "online_control/online_motion_track/mink_guard.py",
            "online_control/online_motion_track/pipeline_v2/collision_guard.py", "online_control/online_motion_track/pipeline_v2/control_pipeline.py",
            "online_control/online_motion_track/pipeline_v2/moveit_style_planner.py", "online_control/online_motion_track/pipeline_v2/pipeline_geometry.py",
            "online_control/online_motion_track/pipeline_v2/session_logger.py", "online_control/online_motion_track/pipeline_v2/tracking_transition.py",
            "src/joint_schema.py", "src/kinematics/__init__.py", "src/kinematics/finger_linkage.py",
            "src/physics/__init__.py", "src/physics/mujoco_filter.py",
            "src/protocol/__init__.py", "src/protocol/hand_angle_protocol.py", "src/protocol/hand_display_protocol.py",
            "src/usb/__init__.py", "src/usb/offline.py", "src/usb/telemetry.py",
            "src/utils/control_session_recorder.py",
            "src/utils/downlink_guard.py", "src/utils/hand_init.py",
            "src/utils/retarget_runtime.py", "src/utils/ruckig_interpolator.py",
        ]
        for relative in extra:
            yield ROOT / relative, "app/" + relative
        v2 = ROOT / "assets/hands/TsenseI_hand"
        for folder, suffix in (("urdf", ".urdf"), ("meshes", ".stl"), ("mjcf", ".xml")):
            for path in sorted((v2 / folder).rglob("*")):
                if path.is_file() and path.suffix.lower() == suffix:
                    yield path, "app/" + path.relative_to(ROOT).as_posix()
        for path in sorted(v2.glob("*.yaml")):
            yield path, "app/" + path.relative_to(ROOT).as_posix()
        for path in sorted((model / "mjcf").glob("*.xml")):
            yield path, "app/" + path.relative_to(ROOT).as_posix()


def payloads(*, full=False):
    for path, relative in files(full=full):
        if not path.is_file():
            raise FileNotFoundError(f"Missing {path}; install browser runtime with npm ci first")
        data = path.read_bytes()
        if path.suffix == ".sh":
            data = data.replace(b"\r\n", b"\n")
        yield relative, data


def build(destination, archive=False, *, full=False):
    destination = Path(destination).resolve()
    if not archive and (destination == HERE or destination in HERE.parents or destination.exists()):
        raise ValueError("Stage directory must be new and must not replace source files")
    checksums = {}
    bundle = None
    if archive:
        if destination.exists():
            raise FileExistsError(f"Refusing to replace existing bundle: {destination}")
        destination.parent.mkdir(parents=True, exist_ok=True)
        bundle = tarfile.open(destination, "w:gz")
    else:
        destination.mkdir(parents=True)
    def write(relative, data):
        if bundle:
            info = tarfile.TarInfo("tsensei-hand-web/" + relative)
            info.size = len(data)
            info.mode = 0o755 if relative.endswith(".sh") else 0o644
            bundle.addfile(info, io.BytesIO(data))
        else:
            path = destination / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(data)
            path.chmod(0o755 if relative.endswith(".sh") else 0o644)
    try:
        for relative, data in payloads(full=full):
            checksums[relative] = hashlib.sha256(data).hexdigest()
            write(relative, data)
        write("manifest.json", json.dumps({"application": "TsenseI Hand Web",
              "features": ["v3.2", "v3.2_withid", "a32_native_sdk", "v3.0", "v2", "mink"] if full else ["v3.2", "v3.2_withid", "v3.0"], "sha256": checksums}, indent=2).encode())
    finally:
        if bundle:
            bundle.close()
    return destination


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--stage", type=Path)
    group.add_argument("--output", type=Path)
    parser.add_argument("--full", action="store_true", help="Include V2/Ruckig and V3.2 Mink preview runtime")
    args = parser.parse_args()
    print(build(args.stage or args.output, archive=args.output is not None, full=args.full))
