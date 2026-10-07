"""Apply the A3.2 application ranges without changing the source CAD files."""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "src"), str(ROOT / "tools")]
from a32_kinematics.joint_coordinates import (
    COORDINATE_VERSION, SWING_AXIS_ALIGNMENT, SWING_CENTER, SWING_COMMAND_LIMITS,
    THUMB_ZERO_ALIGNMENT, THUMB_ROTABACK_ORIGIN_OFFSET,
    initial_pose, is_finger_swing, requested_limits, thumb_rotaback_origin_offset,
)


def apply_urdf_coordinates(root, *, model_id="TsenseI_v3_2"):
    """Apply ranges, swing alignment and the bare thumb mechanical zero once."""
    if model_id not in ("TsenseI_v3_2", "TsenseI_v3_2_withid"):
        raise ValueError(f"Unknown A3.2 model: {model_id}")
    version = root.get("a32_joint_coordinates")
    if version not in (None, COORDINATE_VERSION):
        raise ValueError(f"Unknown A3.2 coordinates: {version}")
    alignment = root.get("a32_swing_axis_alignment")
    if alignment not in (None, SWING_AXIS_ALIGNMENT):
        raise ValueError(f"Unknown A3.2 swing alignment: {alignment}")
    thumb_zero = root.get("a32_thumb_zero_alignment")
    if model_id == "TsenseI_v3_2" and thumb_zero not in (None, THUMB_ZERO_ALIGNMENT):
        raise ValueError(f"Unknown A3.2 thumb zero: {thumb_zero}")
    for joint in root.findall("joint"):
        name = joint.get("name", "")
        offset = thumb_rotaback_origin_offset(model_id, name)
        if offset and thumb_zero is None:
            axis = np.fromstring(joint.find("axis").get("xyz"), sep=" ")
            origin = joint.find("origin")
            old = Rotation.from_euler("xyz", np.fromstring(origin.get("rpy", "0 0 0"), sep=" "))
            shift = Rotation.from_rotvec(axis / np.linalg.norm(axis) * offset)
            origin.set("rpy", " ".join(format(v, ".17g") for v in (old * shift).as_euler("xyz")))
        bounds = requested_limits(name)
        if bounds is None:
            continue
        if is_finger_swing(name) and version is None:
            axis = np.fromstring(joint.find("axis").get("xyz"), sep=" ")
            origin = joint.find("origin")
            old = Rotation.from_euler("xyz", np.fromstring(origin.get("rpy", "0 0 0"), sep=" "))
            shift = Rotation.from_rotvec(axis / np.linalg.norm(axis) * SWING_CENTER)
            origin.set("rpy", " ".join(format(v, ".17g") for v in (old * shift).as_euler("xyz")))
            joint.find("axis").set("xyz", " ".join(format(-v, ".17g") for v in axis))
        if is_finger_swing(name) and model_id == "TsenseI_v3_2" and alignment is None:
            # Reflect the existing 0..0.7 coordinate about 0.35. Rebase the
            # origin too, preserving the full physical range and midpoint.
            axis = np.fromstring(joint.find("axis").get("xyz"), sep=" ")
            origin = joint.find("origin")
            old = Rotation.from_euler("xyz", np.fromstring(origin.get("rpy", "0 0 0"), sep=" "))
            shift = Rotation.from_rotvec(axis / np.linalg.norm(axis) * (2 * SWING_CENTER))
            origin.set("rpy", " ".join(format(v, ".17g") for v in (old * shift).as_euler("xyz")))
            joint.find("axis").set("xyz", " ".join(format(-v, ".17g") for v in axis))
        joint.find("limit").set("lower", str(bounds[0]))
        joint.find("limit").set("upper", str(bounds[1]))
    root.set("a32_joint_coordinates", COORDINATE_VERSION)
    if model_id == "TsenseI_v3_2":
        root.set("a32_swing_axis_alignment", SWING_AXIS_ALIGNMENT)
        root.set("a32_thumb_zero_alignment", THUMB_ZERO_ALIGNMENT)
    return root


def write_xml(path, root):
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)


def sync_installed_assets(model_ids=None):
    from hand_profiles import A32_MODEL_IDS, load_hand_profile
    from TsenseI_retargeting.fk.urdf import UrdfModel
    from generate_profile_collision import build_mjcf
    from a32_kinematics.a32_angles_to_inputs import (
        load_configuration, load_kinematic_model, a32_angle_limits, a32_loop_inverse,
    )

    for model_id in A32_MODEL_IDS if model_ids is None else model_ids:
        profile = load_hand_profile(model_id)
        geometry = yaml.safe_load((profile.root / "collision_geometry.yaml").read_text(encoding="utf-8"))
        for side in ("left", "right"):
            path = profile.path("urdf", side)
            write_xml(path, apply_urdf_coordinates(ET.parse(path).getroot(), model_id=model_id))
            model = UrdfModel(path)
            write_xml(profile.path("mjcf", side), build_mjcf(model, geometry, side, model_id))
            suffix = "" if side == "left" else "_right"
            (profile.root / f"q_init{suffix}.yaml").write_text(
                yaml.safe_dump({"q": initial_pose(model.joint_limits())}, sort_keys=False), encoding="utf-8")
        manifest_path = profile.root / "source_mapping.yaml"
        manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
        manifest.update(joint_coordinate_version=COORDINATE_VERSION,
                        joint_coordinate_change=("new bare swing = original SDK swing + 0.350; axes aligned with ID" if model_id == "TsenseI_v3_2"
                                                 else "new ID swing = -original ID swing + 0.350")
                        + "; model [0, 0.7], default 0; command clipped to [0.005, 0.695]")
        manifest["swing_axis_alignment"] = SWING_AXIS_ALIGNMENT
        if model_id == "TsenseI_v3_2":
            manifest["thumb_zero_alignment"] = THUMB_ZERO_ALIGNMENT
            manifest["thumb_rotaback_origin_offset_rad"] = THUMB_ROTABACK_ORIGIN_OFFSET
        if "sdk_joint_signs" in manifest:
            manifest["legacy_source_joint_signs"] = manifest.pop("sdk_joint_signs")
        manifest_path.write_text(yaml.safe_dump(manifest, sort_keys=False, allow_unicode=True), encoding="utf-8")

    # Preserve the mechanism's original zero and geometry. Only physical command
    # bounds change to the requested application endpoints (metres -> mm).
    model, config = load_kinematic_model(), load_configuration()
    indices = {name: i for i, name in enumerate(model.q_names)}
    path = ROOT / "src/a32_driver/a32_hardware.json"
    hardware = json.loads(path.read_text(encoding="utf-8"))
    for channel in hardware["channels"]:
        name = channel["joint"]
        if is_finger_swing(name):
            channel["limits"] = [math.degrees(q) for q in SWING_COMMAND_LIMITS]
        elif (bounds := requested_limits(name)) is not None:
            i = indices[name]
            motor = config["motors"][i]
            mechanism_bounds = a32_angle_limits(model, config, i + 1)
            bounds = (max(bounds[0], mechanism_bounds[0]), min(bounds[1], mechanism_bounds[1]))
            channel["limits"] = [float(np.clip(a32_loop_inverse(motor, model.axes[i], q),
                                               *motor["strokeRange"])) * 1000 for q in bounds]
    hardware["joint_coordinate_version"] = COORDINATE_VERSION
    hardware["range_note"] = "2026-09-28 user-requested joint ranges; linear limits derived from existing geometry; self swing is positive radians converted to SDK degrees. No hardware motion used for validation. 2026-10-05: four-finger DIP mechanism and command upper limits reduced to 8 mm."
    path.write_text(json.dumps(hardware, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    bare = load_hand_profile("TsenseI_v3_2")
    path = bare.root / "sdk_source_mapping.yaml"
    manifest = yaml.safe_load(path.read_text(encoding="utf-8"))
    manifest["joint_coordinate_version"] = COORDINATE_VERSION
    manifest["swing_axis_alignment"] = SWING_AXIS_ALIGNMENT
    manifest["thumb_zero_alignment"] = THUMB_ZERO_ALIGNMENT
    manifest["thumb_rotaback_origin_offset_rad"] = THUMB_ROTABACK_ORIGIN_OFFSET
    manifest["comparison"] = "Original mesh geometry retained; bare swing axes aligned with ID; thumb rotaback origin calibrated to mechanical stop; documented origin/axis and limit changes."
    for side in ("left", "right"):
        manifest["sides"][side]["installed_urdf_sha256"] = hashlib.sha256(bare.path("urdf", side).read_bytes()).hexdigest()
    path.write_text(yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8")


if __name__ == "__main__":
    sync_installed_assets()
