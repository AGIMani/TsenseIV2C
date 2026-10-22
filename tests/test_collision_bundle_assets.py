"""Relocatable bundles retain collision meshes and native SDK build inputs."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "src")]
from hand_profiles import A30_MODEL_IDS, A32_MODEL_IDS, load_hand_profile
from online_control.online_motion_track import package as track_package
from online_control.online_motion_play import package as play_package


@pytest.mark.parametrize("bundle", ["track", "track_full", "play"])
def test_all_non_v2_urdf_collision_references_survive_packaging(bundle):
    entries = (play_package.files() if bundle == "play"
               else track_package.files(full=bundle == "track_full"))
    manifest = {source.resolve(): relative for source, relative in entries}
    for model_id in (*A30_MODEL_IDS, *A32_MODEL_IDS):
        profile = load_hand_profile(model_id)
        for side in ("left", "right"):
            urdf = profile.path("urdf", side)
            assert urdf.resolve() in manifest
            for mesh in ET.parse(urdf).findall(".//collision/geometry/mesh"):
                dependency = (urdf.parent / mesh.get("filename")).resolve()
                assert dependency.is_file()
                assert dependency in manifest
                assert manifest[dependency] == "app/" + dependency.relative_to(ROOT).as_posix()
    if bundle != "play":
        helper = ROOT / "online_control/online_motion_track/collision_queries.py"
        assert helper in manifest


@pytest.mark.parametrize("bundle", ["track", "track_full", "play"])
def test_native_sdk_sources_can_configure_after_relocation(bundle, tmp_path):
    entries = (play_package.files() if bundle == "play"
               else track_package.files(full=bundle == "track_full"))
    manifest = {relative: source for source, relative in entries}
    prefix = "app/vendor/a3-cpp-sdk/"
    vendor_files = {name.removeprefix(prefix): source for name, source in manifest.items()
                    if name.startswith(prefix)}
    required = {"CMakeLists.txt", "LICENSE", "sdk-source.json", "README-migration.txt",
                "cmake/Version.h.in", "cmake/a3-cpp-sdk-config.cmake.in",
                "include/a3/A3HandApi.h", "include/a3/Stroke.h", "include/a3/detail/Protocol.h",
                "src/A3HandApi.cpp", "src/Protocol.cpp", "src/Stroke.cpp",
                "src/Tcp5001.cpp", "src/UsbCdc.cpp", "tests/test_protocol.cpp",
                "tests/test_web_commands.cpp", "examples/CommandJson.h"}
    assert required <= vendor_files.keys()
    assert not any(Path(name).parts[0] in {"daxian_hand_v3_2", "dist", "build", "webui"}
                   or Path(name).suffix.lower() in {".exe", ".dll", ".lib", ".so", ".stl"}
                   for name in vendor_files)
    assert "examples/enable_and_hold.cpp" not in vendor_files
    assert "examples/web_bridge.cpp" not in vendor_files
    provenance = json.loads(vendor_files["sdk-source.json"].read_text(encoding="utf-8"))
    for name, source in vendor_files.items():
        assert source.is_file()
        if name in provenance["files"]:
            assert hashlib.sha256(source.read_bytes()).hexdigest() == provenance["files"][name]
    # Configure from only the packaged subtree to catch missing add_subdirectory
    # sources/templates and broken relative paths without recompiling a driver.
    if not shutil.which("cmake") or not shutil.which("c++"):
        pytest.skip("CMake and a C++ compiler are required for relocation verification")
    for relative, source in manifest.items():
        if relative.startswith((prefix, "app/src/a32_driver/native_linux/")):
            target = tmp_path / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, target)
    build = tmp_path / "build"
    result = subprocess.run(
        ["cmake", "-S", str(tmp_path / "app/src/a32_driver/native_linux"), "-B", str(build),
         "-DBUILD_TESTING=ON", f"-DA32_NATIVE_OUTPUT={tmp_path / 'artifacts'}"],
        capture_output=True, text=True, timeout=60)
    assert result.returncode == 0, result.stdout + result.stderr
    cache = (build / "CMakeCache.txt").read_text(encoding="utf-8")
    assert "A3_SDK_BUILD_SHARED:BOOL=OFF" in cache
    assert "A3_SDK_BUILD_EXAMPLES:BOOL=OFF" in cache
    assert "A3_SDK_BUILD_TESTS:BOOL=ON" in cache
    assert "linux_offline_transport" in (build / "CTestTestfile.cmake").read_text(encoding="utf-8")
    upstream_tests = (build / "a3-cpp-sdk/CTestTestfile.cmake").read_text(encoding="utf-8")
    assert "a3_test_protocol" in upstream_tests and "a3_test_web_commands" in upstream_tests
