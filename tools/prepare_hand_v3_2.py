#!/usr/bin/env python3
"""Build the independent bare V3.2 package from the supplied A3 C++ SDK.

Same-source keypoints/collision geometry are preserved; a different source
archives the entire old package before creating fresh annotations.
Use --sync-right-keypoints after manually calibrating the left hand.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import os
from pathlib import Path
import shutil
import sys
import xml.etree.ElementTree as ET

import numpy as np
import trimesh
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
if str(ROOT / "tools") not in sys.path:
    sys.path.append(str(ROOT / "tools"))
from generate_right_hand_urdf import _matrix_to_rpy
from TsenseI_retargeting.fk.urdf import UrdfModel, rpy_matrix
from apply_a32_joint_ranges import apply_urdf_coordinates
from a32_kinematics.joint_coordinates import (
    COORDINATE_VERSION, SWING_AXIS_ALIGNMENT,
    thumb_zero_manifest, initial_pose, reverse_swing,
)

SDK_ROOT = Path(os.environ.get("A32_SDK_ROOT", ROOT / "vendor/a3-cpp-sdk"))
SOURCE = SDK_ROOT / "daxian_hand_v3_2"
SOURCE_URDF = SOURCE / "urdf/daxian_hand_left_v3_2.urdf"
DEST = ROOT / "assets/hands/TsenseI_hand_v3_2"
MIRROR = np.diag([-1.0, 1.0, 1.0])  # New CAD: palm width is X, thickness is Y.
FINGERS = ("thumb", "index", "mid", "ring", "pinky")
# The SDK already provides functional joint names. Its historical pinky link
# labels remain as authored; applying the old CAD renames a second time would
# silently exchange flexion and abduction.
RENAMES = {}


def source_reference() -> str:
    return Path(os.path.relpath(SOURCE_URDF, ROOT)).as_posix()


def dump_yaml(path: Path, data: dict, *, preserve: bool = False) -> None:
    if preserve and path.exists():
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(data, sort_keys=False, allow_unicode=True), encoding="utf-8")


def write_xml(path: Path, root: ET.Element) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)


def right_name(name: str) -> str:
    return name.replace("left_hand", "right_hand").replace("Left_hand", "Right_hand")


def prepare_urdfs() -> None:
    root = ET.parse(SOURCE_URDF).getroot()
    root.set("name", "TsenseI_hand_v3_2_left")
    meshes = DEST / "meshes"
    meshes.mkdir(parents=True, exist_ok=True)
    copied = set()
    for node in root.iter():
        for key in ("name", "link", "joint"):
            if node.get(key) in RENAMES:
                node.set(key, RENAMES[node.get(key)])
        if node.tag == "mesh":
            source_name = Path(node.get("filename")).name
            stem = Path(source_name).stem
            target_name = RENAMES.get(stem, stem) + Path(source_name).suffix
            if source_name not in copied:
                shutil.copy2(SOURCE / "meshes" / source_name, meshes / target_name)
                copied.add(source_name)
            node.set("filename", "../meshes/" + target_name)
    apply_urdf_coordinates(root)
    write_xml(DEST / "urdf/TsenseI_hand_left_v3_2.urdf", root)

    right = copy.deepcopy(root)
    right.set("name", "TsenseI_hand_v3_2_right")
    mirrored = set()
    for node in right.iter():
        for key in ("name", "link", "joint"):
            if node.get(key):
                node.set(key, right_name(node.get(key)))
        if node.tag == "origin":
            if node.get("xyz"):
                node.set("xyz", fmt(MIRROR @ np.fromstring(node.get("xyz"), sep=" ")))
            if node.get("rpy"):
                rotation = rpy_matrix(*np.fromstring(node.get("rpy"), sep=" "))
                node.set("rpy", fmt(_matrix_to_rpy(MIRROR @ rotation @ MIRROR)))
        elif node.tag == "axis":
            node.set("xyz", fmt(-MIRROR @ np.fromstring(node.get("xyz"), sep=" ")))
        elif node.tag == "inertia":
            keys = ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")
            a, b, c, d, e, f = (float(node.get(k)) for k in keys)
            inertia = MIRROR @ np.array([[a,b,c], [b,d,e], [c,e,f]]) @ MIRROR
            node.attrib.update(zip(keys, (f"{v:.15g}" for v in inertia[np.triu_indices(3)])))
        elif node.tag == "mesh":
            name = Path(node.get("filename")).name
            target = right_name(name) if name != "base_link.STL" else "right_base_link.STL"
            if name not in mirrored:
                mesh = trimesh.load_mesh(meshes / name, process=False)
                trimesh.Trimesh(vertices=mesh.vertices @ MIRROR, faces=mesh.faces[:, ::-1], process=False).export(meshes / target)
                mirrored.add(name)
            node.set("filename", "../meshes/" + target)
    write_xml(DEST / "urdf/TsenseI_hand_right_v3_2.urdf", right)


def fmt(values) -> str:
    return " ".join(f"{float(v):.15g}" for v in values)


def source_fingerprint() -> str:
    """Check every referenced mesh before replacement and detect in-place CAD updates."""
    digest = hashlib.sha256(SOURCE_URDF.read_bytes())
    root = ET.parse(SOURCE_URDF).getroot()
    for name in sorted({Path(n.get("filename")).name for n in root.iter("mesh")}):
        digest.update(name.encode("utf-8"))
        digest.update((SOURCE / "meshes" / name).read_bytes())
    return digest.hexdigest()


def archive_if_source_changed() -> None:
    """Preserve earlier annotations before changing their underlying geometry."""
    mapping = DEST / "source_mapping.yaml"
    if not mapping.exists():
        return
    previous = yaml.safe_load(mapping.read_text(encoding="utf-8"))
    if (previous.get("source") == source_reference()
            and previous.get("source_package_sha256") == source_fingerprint()):
        return
    # Source location/name migrations do not invalidate reviewed annotations.
    # Require every geometric XML attribute and referenced mesh to match before
    # preserving an existing package under a new provenance path.
    installed = DEST / "urdf/TsenseI_hand_left_v3_2.urdf"
    if installed.exists() and equivalent_source_geometry(installed):
        return
    from datetime import datetime
    # Both absolute targets must remain inside assets/hands before moving.
    parent = (ROOT / "assets/hands").resolve()
    original = DEST.resolve()
    backup = (parent / "_backups" / (DEST.name + "_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f"))).resolve()
    if not original.is_relative_to(parent) or not backup.is_relative_to(parent) or original == parent:
        raise ValueError("Invalid asset archive path")
    backup.parent.mkdir(parents=True, exist_ok=True)
    shutil.move(str(original), str(backup))
    print(f"Previous model and annotations archived: {backup}")


def equivalent_source_geometry(installed: Path) -> bool:
    def geometry(path: Path):
        result = []
        for node in apply_urdf_coordinates(ET.parse(path).getroot()).iter():
            attrs = dict(node.attrib)
            if node.tag == "robot":
                attrs.pop("name", None)
            if node.tag == "mesh":
                filename = attrs.pop("filename")
                attrs["mesh_sha256"] = hashlib.sha256((path.parent / filename).resolve().read_bytes()).hexdigest()
            result.append((node.tag, attrs))
        return result
    return geometry(SOURCE_URDF) == geometry(installed)


def draft_keypoints(model: UrdfModel) -> dict:
    """Seed points from V3 geometry, not V2 local coordinates. Human review required."""
    template = yaml.safe_load((ROOT / "assets/hands/TsenseI_hand/robot_keypoints_21.yaml").read_text(encoding="utf-8"))
    rows = template["keypoints"]
    visuals = model.visual_meshes_by_link()
    wrist_visual = visuals[model.base_link][0]
    wrist_mesh = trimesh.load_mesh(wrist_visual.path, process=False)
    wrist_vertices = (np.asarray(wrist_mesh.vertices) * wrist_visual.scale) @ wrist_visual.origin[:3, :3].T + wrist_visual.origin[:3, 3]
    # This CAD's base frame is near the finger roots, not at the anatomical
    # wrist. Use the proximal end of its fixed wrist mesh as an explicit seed.
    proximal = wrist_vertices[wrist_vertices[:, 2] <= np.quantile(wrist_vertices[:, 2], .05)]
    rows[0].update(link=model.base_link, local_xyz=proximal.mean(axis=0).tolist())
    poses = model.forward_kinematics({name: reverse_swing(name, 0.) for name in model.joint_names()})
    joint_by_child = {joint.child: joint for joint in model.joints}
    link_chains = {
        "thumb": ("thumb_rotaback_link2", "thumb_MCP", "thumb_PIP", "thumb_PIP"),
        "index": ("index_swing", "index_PIP", "index_DIP", "index_DIP"),
        "mid": ("mid_swing", "mid_PIP", "mid_DIP", "mid_DIP"),
        "ring": ("ring_swing", "ring_PIP", "ring_DIP", "ring_DIP"),
        "pinky": ("pinky_MCP", "pinky_PIP", "pinky_DIP", "pinky_DIP"),
    }
    # Thumb anatomy has 3 joint proxies plus a tip. Place proxies at moving
    # link origins; they are intentionally a draft for the calibration editor.
    for finger_index, finger in enumerate(FINGERS):
        for offset, part in enumerate(link_chains[finger]):
            row = rows[1 + 4 * finger_index + offset]
            link = "left_hand_" + part + ("" if part.endswith("link2") else "_link")
            local = np.zeros(3)
            if offset == 0 and finger != "thumb":
                # MCP proxy is at the proximal flexion joint, expressed in the
                # downstream moving link frame used by the calibration editor.
                parent = joint_by_child[link].parent
                local = (np.linalg.inv(poses[link]) @ poses[parent])[:3, 3]
            if offset == 3:
                visual = model.visual_meshes_by_link()[link][0]
                mesh = trimesh.load_mesh(visual.path, process=False)
                vertices = np.asarray(mesh.vertices) * visual.scale
                vertices = vertices @ visual.origin[:3, :3].T + visual.origin[:3, 3]
                distal = vertices[vertices[:, 2] >= np.quantile(vertices[:, 2], 0.95)]
                local = distal.mean(axis=0)
            row.update(link=link, local_xyz=local.tolist())
    for row in rows:
        row["annotation_status"] = "auto_initialized"
        row["note"] = "V3.2 geometry seed; manually calibrate before retarget evaluation."
    template.update(robot_name="TsenseI_V3.2", urdf_path="urdf/TsenseI_hand_left_v3_2.urdf",
                    quality_status="Draft", annotation_version=1, hand_side="left",
                    generated_by="tools/prepare_hand_v3_2.py (wrist mesh + URDF joint centers + mesh tips)")
    return template


def sync_right_keypoints() -> None:
    data = yaml.safe_load((DEST / "robot_keypoints_21.yaml").read_text(encoding="utf-8"))
    data.update(hand_side="right", urdf_path="urdf/TsenseI_hand_right_v3_2.urdf",
                mirrored_from="robot_keypoints_21.yaml", mirror_plane="YZ")
    for row in data["keypoints"]:
        row["link"] = right_name(row["link"])
        xyz = row["local_xyz"]
        row["local_xyz"] = (MIRROR @ xyz).tolist()
    target = DEST / "robot_keypoints_21_right.yaml"
    if target.exists():
        from datetime import datetime
        shutil.copy2(target, target.with_name(target.name + ".bak_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f")))
    dump_yaml(target, data)


def prepare_configs() -> None:
    sides = {}
    for side in ("left", "right"):
        model = UrdfModel(DEST / f"urdf/TsenseI_hand_{side}_v3_2.urdf")
        names = model.joint_names()
        suffix = "" if side == "left" else "_right"
        kp = f"robot_keypoints_21{suffix}.yaml"
        groups = {finger: {"joints": [n for n in names if f"_{finger}_" in n]} for finger in FINGERS}
        dump_yaml(DEST / f"joint_groups{suffix}.yaml", {"active_joints": names, "fingers": groups})
        dump_yaml(DEST / f"eval_config{suffix}.yaml", {
            "robot_name": "TsenseI_V3.2", "urdf_path": f"urdf/{model.urdf_path.name}",
            "keypoint_config": kp, "joint_groups": f"joint_groups{suffix}.yaml",
            "base_link": "base_link", "active_joints": names,
        })
        dump_yaml(DEST / f"q_init{suffix}.yaml", {"q": initial_pose(model.joint_limits())})
        sides[side] = {"urdf": f"urdf/{model.urdf_path.name}", "keypoints": kp,
                       "eval_config": f"eval_config{suffix}.yaml", "mjcf": f"mjcf/mjmodel{suffix}.xml"}
        if side == "left":
            dump_yaml(DEST / kp, draft_keypoints(model), preserve=True)
    if not (DEST / "robot_keypoints_21_right.yaml").exists():
        sync_right_keypoints()
    dump_yaml(DEST / "hand_profile.yaml", {
        "schema_version": 1, "model_id": "TsenseI_v3_2", "display_name": "TsenseI_V3.2",
        "coordinate_space": "urdf_radians", "sides": sides,
        "fixed_joints": {}, "hardware": {"enabled": False, "adapter": None},
        "retarget": {"lambda_smooth": 0.1, "lambda_reg": 0.1,
                     "init_max_iters": 2, "topo_max_iters": 3},
        "mirror_axis": "x",
        "notes": ["SDK meshes and functional joint names retained; application joint ranges are versioned.",
                  "PIP/DIP/MCP 0..1.57; commands intersect mechanism limits; swing aligned with reference ID CAD, model -0.35..0.35, command -0.34..0.34 rad; thumb zero matches user reference CAD.",
                  "Keypoints are draft until manually calibrated; collision proxies are geometric approximations."],
    }, preserve=True)
    write_source_mapping()


def write_source_mapping() -> None:
    dump_yaml(DEST / "source_mapping.yaml", {
        "source": source_reference(), "renames": RENAMES,
        "naming": "SDK functional joint names retained; original link labels preserved",
        "source_urdf_sha256": hashlib.sha256(SOURCE_URDF.read_bytes()).hexdigest(),
        "source_package_sha256": source_fingerprint(),
        "geometry_changed": False, "joint_coordinate_version": COORDINATE_VERSION,
        "swing_axis_alignment": SWING_AXIS_ALIGNMENT,
        **thumb_zero_manifest("TsenseI_v3_2"),
        "joint_coordinate_change": "bare swing = -original SDK swing; axes aligned with reference ID CAD; thumb zero restored to reference CAD; application flexion limits",
        "right_mirror": "YZ: M=diag(-1,1,1), rotation axes -M; equal q",
    })


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sync-right-keypoints", action="store_true")
    args = parser.parse_args()
    if args.sync_right_keypoints:
        sync_right_keypoints()
    else:
        source_fingerprint()  # Missing/invalid source must not archive a working package.
        archive_if_source_changed()
        prepare_urdfs()
        prepare_configs()
        from generate_profile_collision import generate_profile
        generate_profile("TsenseI_v3_2", profile_root=DEST)
    print(DEST)


if __name__ == "__main__":
    main()
