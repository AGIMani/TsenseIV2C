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
    COORDINATE_VERSION, SWING_AXIS_ALIGNMENT, SWING_COMMAND_LIMITS,
    POSITIVE_COORDINATE_VERSION, POSITIVE_SWING_AXIS_ALIGNMENT, POSITIVE_SWING_CENTER,
    THUMB_ZERO_ALIGNMENT, LEGACY_THUMB_ZERO_ALIGNMENT, LEGACY_THUMB_ROTABACK_ORIGIN_OFFSET,
    WITHID_CAD_VERSION, WITHID_THUMB_ZERO_ALIGNMENT,
    thumb_zero_manifest,
    initial_pose, is_finger_swing, requested_limits, thumb_origin_offset,
)


def apply_urdf_coordinates(root, *, model_id="TsenseI_v3_2"):
    """Apply ranges, swing alignment and the reference thumb zero once."""
    authored_cad = (model_id == "TsenseI_v3_2_withid"
                    and root.get("a32_cad_source") == WITHID_CAD_VERSION)
    replacement_cad = (model_id == "TsenseI_v3_2_withid"
                       and root.get("a32_cad_source") in (
                           "middle-dip-standard-20-outputs-v1", "raised5mm-noupper-mid-20261027")
                       and root.get("a32_joint_coordinates") in (None, COORDINATE_VERSION))
    if model_id not in ("TsenseI_v3_2", "TsenseI_v3_2_withid"):
        raise ValueError(f"Unknown A3.2 model: {model_id}")
    version = root.get("a32_joint_coordinates")
    if version not in (None, POSITIVE_COORDINATE_VERSION, COORDINATE_VERSION):
        raise ValueError(f"Unknown A3.2 coordinates: {version}")
    alignment = root.get("a32_swing_axis_alignment")
    if alignment not in (None, POSITIVE_SWING_AXIS_ALIGNMENT, SWING_AXIS_ALIGNMENT):
        raise ValueError(f"Unknown A3.2 swing alignment: {alignment}")
    thumb_zero = root.get("a32_thumb_zero_alignment")
    if thumb_zero not in (None, LEGACY_THUMB_ZERO_ALIGNMENT, THUMB_ZERO_ALIGNMENT,
                          WITHID_THUMB_ZERO_ALIGNMENT if authored_cad else None):
        raise ValueError(f"Unknown A3.2 thumb zero: {thumb_zero}")
    if authored_cad and version in (None, COORDINATE_VERSION):
        # This complete replacement supplies its own assembly zero, axes and
        # exact limits. Do not reapply older shell offsets or rounded ranges.
        root.set("a32_joint_coordinates", COORDINATE_VERSION)
        root.set("a32_swing_axis_alignment", SWING_AXIS_ALIGNMENT)
        root.set("a32_thumb_zero_alignment", WITHID_THUMB_ZERO_ALIGNMENT)
        return root
    for joint in root.findall("joint"):
        name = joint.get("name", "")
        offset = 0. if replacement_cad or authored_cad else thumb_origin_offset(model_id, name)
        if (thumb_zero == LEGACY_THUMB_ZERO_ALIGNMENT and model_id == "TsenseI_v3_2"
                and name.endswith("_thumb_rotaback_joint2")):
            offset -= LEGACY_THUMB_ROTABACK_ORIGIN_OFFSET
        if offset and thumb_zero != THUMB_ZERO_ALIGNMENT:
            axis = np.fromstring(joint.find("axis").get("xyz"), sep=" ")
            origin = joint.find("origin")
            old = Rotation.from_euler("xyz", np.fromstring(origin.get("rpy", "0 0 0"), sep=" "))
            shift = Rotation.from_rotvec(axis / np.linalg.norm(axis) * offset)
            origin.set("rpy", " ".join(format(v, ".17g") for v in (old * shift).as_euler("xyz")))
        bounds = requested_limits(name, model_id=model_id if authored_cad else "TsenseI_v3_2")
        if not authored_cad and name.endswith(("_thumb_rota_block_joint", "_thumb_rotaback_joint2")):
            # The older source already carries these root ranges; preserve
            # their values and XML spelling while updating flexion/swing.
            bounds = None
        if replacement_cad and name.endswith("_thumb_rotaback_joint2"):
            if (root.get("a32_cad_source") == "raised5mm-noupper-mid-20261027"
                    and version is None and float(joint.find("limit").get("lower")) < 0):
                # New CAD uses q_source=-q_app. Flip the axis, retaining the
                # same physical poses and the existing positive SDK commands.
                axis = joint.find("axis")
                axis.set("xyz", " ".join(str(-float(v)) for v in axis.get("xyz").split()))
                joint.find("limit").set("lower", "0.0")
            joint.find("limit").set("upper", "1.74")
        if bounds is None:
            continue
        if replacement_cad:
            # The replacement CAD already supplies the intended zero and axis;
            # retain those frames while applying the project's actuator domain.
            pass
        elif is_finger_swing(name) and version != COORDINATE_VERSION:
            axis_node = joint.find("axis")
            axis = np.fromstring(axis_node.get("xyz"), sep=" ")
            origin = joint.find("origin")
            if version == POSITIVE_COORDINATE_VERSION:
                # Undo the translated origin, then restore the reference CAD
                # direction: q_new = 0.350 - q_positive. Old bare assets
                # without the alignment marker used the opposite direction.
                aligned = model_id.endswith("withid") or alignment == POSITIVE_SWING_AXIS_ALIGNMENT
                old = Rotation.from_euler("xyz", np.fromstring(origin.get("rpy", "0 0 0"), sep=" "))
                shift = Rotation.from_rotvec(axis / np.linalg.norm(axis) * POSITIVE_SWING_CENTER)
                origin.set("rpy", " ".join(format(v, ".17g") for v in (old * shift).as_euler("xyz")))
                if aligned:
                    axis = -axis
            elif model_id == "TsenseI_v3_2":
                # Bare CAD and supplied ID CAD have opposite swing directions.
                # Preserve bare frames/meshes while aligning its signed axis.
                axis = -axis
            axis_node.set("xyz", " ".join(format(v, ".17g") for v in axis))
        joint.find("limit").set("lower", str(bounds[0]))
        joint.find("limit").set("upper", str(bounds[1]))
    root.set("a32_joint_coordinates", COORDINATE_VERSION)
    root.set("a32_swing_axis_alignment", SWING_AXIS_ALIGNMENT)
    root.set("a32_thumb_zero_alignment", WITHID_THUMB_ZERO_ALIGNMENT if authored_cad else THUMB_ZERO_ALIGNMENT)
    return root


def write_xml(path, root):
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)


def sync_installed_assets(model_ids=None):
    from hand_profiles import A32_MODEL_IDS, load_hand_profile
    from TsenseI_retargeting.fk.urdf import UrdfModel
    from generate_profile_collision import build_mjcf
    from a32_driver.limits import sdk_channel_limits
    from a32_kinematics.sdk_stroke import METHOD, SDK_VERSION

    selected_models = tuple(A32_MODEL_IDS if model_ids is None else model_ids)
    for model_id in selected_models:
        profile = load_hand_profile(model_id)
        geometry = yaml.safe_load((profile.root / "collision_geometry.yaml").read_text(encoding="utf-8"))
        for side in ("left", "right"):
            path = profile.path("urdf", side)
            write_xml(path, apply_urdf_coordinates(ET.parse(path).getroot(), model_id=model_id))
            model = UrdfModel(path)
            write_xml(profile.path("mjcf", side), build_mjcf(model, geometry, side, model_id))
            suffix = "" if side == "left" else "_right"
            init_path = profile.root / f"q_init{suffix}.yaml"
            init_data = {"q": initial_pose(model.joint_limits())}
            if not init_path.exists() or yaml.safe_load(init_path.read_text()) != init_data:
                init_path.write_text(yaml.safe_dump(init_data, sort_keys=False), encoding="utf-8")
        manifest_path = profile.root / "source_mapping.yaml"
        manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
        authored_cad = model.root.get("a32_cad_source") == WITHID_CAD_VERSION
        manifest["joint_coordinate_version"] = COORDINATE_VERSION
        if authored_cad:
            manifest["joint_coordinate_change"] = (
                "Customer CAD origins, axes and exact limits retained; thumb roots 130/100 deg, "
                "thumb MCP/IP 90 deg, finger flexion 1.57 rad and swing +/-20 deg; default 0. "
                "SDK and hardware commands use the same limits, including swing +/-20 deg.")
            manifest["thumb_zero_alignment"] = WITHID_THUMB_ZERO_ALIGNMENT
            manifest["thumb_zero_reference"] = WITHID_CAD_VERSION
            manifest["thumb_origin_offsets_rad"] = {
                "thumb_rota_block_joint": 0., "thumb_rotaback_joint2": 0.}
            manifest.pop("thumb_zero_reference_sha256", None)
        else:
            manifest["joint_coordinate_change"] = (
                "new bare swing = -original SDK swing; axes aligned with reference ID CAD" if model_id == "TsenseI_v3_2"
                else "new ID swing = original ID swing; reference CAD origin and axis restored"
            ) + "; model [-0.35, 0.35], default 0; command clipped to [-0.34, 0.34]"
            manifest.update(thumb_zero_manifest(model_id))
        manifest["swing_axis_alignment"] = SWING_AXIS_ALIGNMENT
        manifest.pop("thumb_rotaback_origin_offset_rad", None)
        if "sdk_joint_signs" in manifest:
            manifest["legacy_source_joint_signs"] = manifest.pop("sdk_joint_signs")
        manifest["installed_sha256"] = {
            str(path.relative_to(profile.root)): hashlib.sha256(path.read_bytes()).hexdigest()
            for pattern in ("urdf/*.urdf", "mjcf/mjmodel*.xml")
            for path in sorted(profile.root.glob(pattern))}
        manifest_path.write_text(yaml.safe_dump(manifest, sort_keys=False, allow_unicode=True), encoding="utf-8")

    # An ID-only rebuild must not rewrite the bare profile or shared hardware
    # calibration. Its new CAD boundaries do not authorize wider motor limits.
    if "TsenseI_v3_2" not in selected_models:
        return

    # Model limits stay in radians; the SDK defines calibrated actuator curves.
    # Rebuilding assets must not restore the former unscaled stroke bounds.
    path = ROOT / "src/a32_driver/a32_hardware.json"
    hardware = json.loads(path.read_text(encoding="utf-8"))
    for channel in hardware["channels"]:
        name = channel["joint"]
        if is_finger_swing(name):
            channel["limits"] = [math.degrees(q) for q in SWING_COMMAND_LIMITS]
        elif channel["unit"] == "mm":
            channel["limits"] = list(sdk_channel_limits(channel["channel"], channel["index"]))
    hardware["joint_coordinate_version"] = COORDINATE_VERSION
    hardware["sdk_version"] = SDK_VERSION
    hardware["conversion"] = f"{METHOD}-calibrated-linkage"
    hardware["range_note"] = f"SDK {SDK_VERSION} calibrated closed-loop conversion, one curve per motor size; thumb MCP slot 17 uses the medium 16.95 mm stroke. With-ID commands match the 2026-11-16 CAD, including thumb roots 130/100 deg and swing +/-20 deg. Bare commands retain their model-specific limits. Preserve calibrated direction/offset and source thumb zero. No hardware motion used for validation."
    path.write_text(json.dumps(hardware, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    bare = load_hand_profile("TsenseI_v3_2")
    path = bare.root / "sdk_source_mapping.yaml"
    manifest = yaml.safe_load(path.read_text(encoding="utf-8"))
    manifest["joint_coordinate_version"] = COORDINATE_VERSION
    manifest["swing_axis_alignment"] = SWING_AXIS_ALIGNMENT
    manifest.update(thumb_zero_manifest("TsenseI_v3_2"))
    manifest.pop("thumb_rotaback_origin_offset_rad", None)
    manifest["comparison"] = "Original mesh geometry retained; bare swing axes aligned with ID; thumb zero restored to user-confirmed reference CAD; documented origin/axis and limit changes."
    for side in ("left", "right"):
        manifest["sides"][side]["installed_urdf_sha256"] = hashlib.sha256(bare.path("urdf", side).read_bytes()).hexdigest()
    path.write_text(yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8")


if __name__ == "__main__":
    sync_installed_assets()
