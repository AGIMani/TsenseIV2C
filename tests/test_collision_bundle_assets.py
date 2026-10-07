"""Relocatable bundles must retain every URDF collision-mesh dependency."""
from pathlib import Path
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
