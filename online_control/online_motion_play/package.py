"""Build a relocatable pure Python playback server bundle without environments or credentials."""
import argparse
import json
from pathlib import Path
import shutil
import tarfile

HERE = Path(__file__).resolve().parent
ROOT = HERE / "app" if (HERE / "app/src").is_dir() else HERE.parents[1]
WEB_FILES = ("server.py", "v2_session.py", "v2_homing.py", "downlink_policy.py", "joint_ranges.md", "a30_device.py", "a30_session.py", "a32_session.py", "a32_recording.py", "browser_serial.py", "browser-serial.js", "serial-protocol.js", "a30-serial-protocol.js", "access.py", "index.html", "deploy.sh", "deploy_config.py",
             "package.py", "generate_joint_ranges.py", "requirements-server.txt", "build-constraints.txt", "README.md",
             "serial-worker.js", "worker-client.js", "position-guard.js", "motion-stream.js")


def files():
    for path in sorted((ROOT / 'online_control/shared').glob('*')):
        if path.suffix in ('.py', '.js', '.json'):
            yield path, 'app/' + path.relative_to(ROOT).as_posix()
    yield ROOT / 'online_control/__init__.py', 'app/online_control/__init__.py'
    yield ROOT / 'tools/hand_control_log_viewer.html', 'app/tools/hand_control_log_viewer.html'
    for name in WEB_FILES:
        yield HERE / name, name
    for name in ("Dockerfile", ".dockerignore", "update.sh", "Caddyfile"):
        relative = "deployment/node0/" + name
        if (HERE / relative).is_file():
            yield HERE / relative, relative
    for path in sorted((HERE / "vendor").glob("*")):
        if path.is_file():
            yield path, "vendor/" + path.name
    # Keep shared source in its original structure; never include logs, working
    # poses, credentials, environments, camera weights or other workspace files.
    for path in sorted((ROOT / "src").rglob("*.py")):
        if "ui" not in path.relative_to(ROOT / "src").parts:
            yield path, "app/" + path.relative_to(ROOT).as_posix()
    for folder in ("TsenseI_hand", "TsenseI_hand_v3_0", "TsenseI_hand_v3_0_withid", "TsenseI_hand_v3_2", "TsenseI_hand_v3_2_withid"):
        models = ROOT / "assets/hands" / folder
        for directory, suffix in (("urdf", ".urdf"), ("meshes", ".stl")):
            for path in sorted((models / directory).rglob("*")):
                if path.is_file() and path.suffix.lower() == suffix:
                    yield path, "app/" + path.relative_to(ROOT).as_posix()
        if folder != "TsenseI_hand":
            for path in sorted((models / "mjcf/collision_meshes").glob("*.stl")):
                yield path, "app/" + path.relative_to(ROOT).as_posix()
            path = models / "hand_profile.yaml"
            yield path, "app/" + path.relative_to(ROOT).as_posix()
    # Native shim source/config and an explicitly built shared library only.
    # SDK example/bridge executable programs are never deployed or invoked.
    for path in sorted((ROOT / "src/a32_driver").rglob("*")):
        if any(part in ("build", "build-linux", "staged", "__pycache__") for part in path.relative_to(ROOT / "src/a32_driver").parts):
            continue
        if path.is_file() and (path.suffix.lower() in (".cpp", ".h", ".hpp", ".json", ".so", ".sh") or path.name in ("CMakeLists.txt", "SDK_LICENSE.txt")):
            yield path, "app/" + path.relative_to(ROOT).as_posix()
    # Native CMake resolves ../../../vendor/a3-cpp-sdk. Include only imported
    # source dependencies and test inputs, never the vendor CAD or binaries.
    sdk = ROOT / "vendor/a3-cpp-sdk"
    source_manifest = json.loads((sdk / "sdk-source.json").read_text(encoding="utf-8"))
    sdk_files = {"CMakeLists.txt", "LICENSE", "CHANGELOG.md", "COMPAT.md", "README.md", "README.en.md",
                 "examples/CommandJson.h", "linux/99-a3-h7-cdc.rules"}
    for relative in sorted(source_manifest["files"]):
        path = Path(relative)
        if path.is_absolute() or ".." in path.parts:
            raise ValueError(f"Invalid SDK source path: {relative}")
        if relative in sdk_files or path.parts[0] in ("include", "src", "cmake", "tests"):
            yield sdk / relative, "app/vendor/a3-cpp-sdk/" + relative
    yield sdk / "sdk-source.json", "app/vendor/a3-cpp-sdk/sdk-source.json"
    yield sdk / "README-migration.txt", "app/vendor/a3-cpp-sdk/README-migration.txt"
    for path in sorted((ROOT / "src/a32_kinematics/model").glob("*")):
        if path.suffix.lower() in (".mat", ".urdf"):
            yield path, "app/" + path.relative_to(ROOT).as_posix()
    for name in ("a32_motion_play.md", "A32_NATIVE_SDK.md", "a32_platform_integration.md", "a32_integration_validation.md", "a32_connection_diagnostics.md", "a32_motion_recording.md", "a32_joint_ranges.md", "motion_limits.md"):
        path = ROOT / "docs" / name
        if path.is_file():
            yield path, "docs/" + name


def stage(destination):
    destination = Path(destination).resolve()
    if HERE.is_relative_to(destination) or ROOT.is_relative_to(destination):
        raise ValueError("Destination must not replace the source project")
    manifest = list(files())
    for source, relative in manifest:
        if not source.is_file():
            raise FileNotFoundError(source)
        target = destination / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        if source.suffix == ".sh":
            target.write_bytes(source.read_bytes().replace(b"\r\n", b"\n"))
            target.chmod(0o755)
        else:
            shutil.copy2(source, target)
    return destination


def archive(output):
    output = Path(output).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    with tarfile.open(output, "w:gz") as bundle:
        for source, relative in files():
            if not source.is_file():
                raise FileNotFoundError(source)
            # Normalize executable permissions and line endings for Linux even
            # when this bundle is produced by a Windows git checkout.
            if source.suffix == ".sh":
                import io
                data = source.read_bytes().replace(b"\r\n", b"\n")
                info = tarfile.TarInfo("online-motion-play/" + relative)
                info.size, info.mode = len(data), 0o755
                bundle.addfile(info, io.BytesIO(data))
            else:
                bundle.add(source, arcname="online-motion-play/" + relative, recursive=False)
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--stage", type=Path)
    group.add_argument("--output", type=Path, default=HERE / "dist/online-motion-play.tar.gz")
    args = parser.parse_args()
    print(stage(args.stage) if args.stage else archive(args.output))


if __name__ == "__main__":
    main()
