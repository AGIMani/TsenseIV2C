#!/usr/bin/env python3
"""Build the independent A3.0 package from the TsenseI_A3.0_hand CAD export.

Same-source keypoint annotations are preserved; a different source
archives the entire old package before creating fresh annotations.
Use --sync-right-keypoints after manually calibrating the left hand.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
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

SOURCE = ROOT / "TsenseI_A3.0_hand"
SOURCE_URDF = SOURCE / "urdf/V6 26.8.4.urdf"
DEST = ROOT / "assets/hands/TsenseI_hand_v3_0"
MIRROR = np.diag([1.0, -1.0, 1.0])  # A3.0 CAD: palm width is Y, thickness is X.
FINGERS = ("thumb", "index", "mid", "ring", "pinky")
# Simultaneous substitution, so old swing/PIP names cannot collide with new ones.
PINKY_PARTS = {"rotaback": "MCP", "MCP": "swing", "swing": "PIP", "PIP": "DIP"}
LEGACY_RENAMES = {f"left_hand_pinky_{old}_{kind}": f"left_hand_pinky_{new}_{kind}"
           for old, new in PINKY_PARTS.items() for kind in ("joint", "link")}
# Historical mapping only: source URDF names are now canonical.
RENAMES = {}
MODEL_ID = "TsenseI_v3_0"
MODEL_SUFFIX = "v3_0"
DISPLAY_NAME = "TsenseI A3.0"


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
    root.set("name", f"TsenseI_hand_{MODEL_SUFFIX}_left")
    meshes = DEST / "meshes"
    meshes.mkdir(parents=True, exist_ok=True)
    copied = set()
    for node in root.iter():
        if node.tag == "mesh":
            source_name = Path(node.get("filename")).name
            stem = Path(source_name).stem
            target_name = source_name
            if source_name not in copied:
                shutil.copy2(SOURCE / "meshes" / source_name, meshes / target_name)
                copied.add(source_name)
            node.set("filename", "../meshes/" + target_name)
    write_xml(DEST / f"urdf/TsenseI_hand_left_{MODEL_SUFFIX}.urdf", root)

    right = copy.deepcopy(root)
    right.set("name", f"TsenseI_hand_{MODEL_SUFFIX}_right")
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
            target = right_name(name)
            if target == name:
                target = "right_" + name
            if name not in mirrored:
                mesh = trimesh.load_mesh(meshes / name, process=False)
                trimesh.Trimesh(vertices=mesh.vertices @ MIRROR, faces=mesh.faces[:, ::-1], process=False).export(meshes / target)
                mirrored.add(name)
            node.set("filename", "../meshes/" + target)
    write_xml(DEST / f"urdf/TsenseI_hand_right_{MODEL_SUFFIX}.urdf", right)


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
    if (previous.get("source") == SOURCE_URDF.relative_to(ROOT).as_posix()
            and previous.get("source_package_sha256") == source_fingerprint()):
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


def draft_keypoints(model: UrdfModel) -> dict:
    """Seed points from V3 geometry, not V2 local coordinates. Human review required."""
    template = yaml.safe_load((ROOT / "assets/hands/TsenseI_hand/robot_keypoints_21.yaml").read_text(encoding="utf-8"))
    rows = template["keypoints"]
    visuals = model.visual_meshes_by_link()
    poses = model.forward_kinematics({})
    wrist_link = model.base_link if visuals.get(model.base_link) else "Left_hand_palm_link"
    clouds = []
    for visual in visuals[wrist_link]:
        mesh = trimesh.load_mesh(visual.path, process=False)
        transform = np.linalg.inv(poses[model.base_link]) @ poses[wrist_link] @ visual.origin
        clouds.append((np.asarray(mesh.vertices) * visual.scale) @ transform[:3, :3].T + transform[:3, 3])
    wrist_vertices = np.concatenate(clouds)
    # This CAD's base frame is near the finger roots, not at the anatomical
    # wrist. Use the proximal end of its fixed wrist mesh as an explicit seed.
    proximal = wrist_vertices[wrist_vertices[:, 2] <= np.quantile(wrist_vertices[:, 2], .05)]
    rows[0].update(link=model.base_link, local_xyz=proximal.mean(axis=0).tolist())
    poses = model.forward_kinematics({})
    joint_by_child = {joint.child: joint for joint in model.joints}
    link_chains = {
        "thumb": ("thumb_rotaback_link2", "thumb_MCP", "thumb_PIP", "thumb_PIP"),
        "index": ("index_swing", "index_PIP", "index_DIP", "index_DIP"),
        "mid": ("mid_swing", "mid_PIP", "mid_DIP", "mid_DIP"),
        "ring": ("ring_swing", "ring_PIP", "ring_DIP", "ring_DIP"),
        "pinky": ("pinky_swing", "pinky_PIP", "pinky_DIP", "pinky_DIP"),
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
        row["note"] = f"{DISPLAY_NAME} geometry seed; manually calibrate before retarget evaluation."
    template.update(robot_name=DISPLAY_NAME, urdf_path=f"urdf/TsenseI_hand_left_{MODEL_SUFFIX}.urdf",
                    quality_status="Draft", annotation_version=1, hand_side="left",
                    generated_by="tools/prepare_hand_v3_0.py (wrist mesh + URDF joint centers + mesh tips)")
    return template


def sync_right_keypoints() -> None:
    data = yaml.safe_load((DEST / "robot_keypoints_21.yaml").read_text(encoding="utf-8"))
    data.update(hand_side="right", urdf_path=f"urdf/TsenseI_hand_right_{MODEL_SUFFIX}.urdf",
                mirrored_from="robot_keypoints_21.yaml", mirror_plane="XZ")
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
        model = UrdfModel(DEST / f"urdf/TsenseI_hand_{side}_{MODEL_SUFFIX}.urdf")
        names = model.joint_names()
        suffix = "" if side == "left" else "_right"
        kp = f"robot_keypoints_21{suffix}.yaml"
        groups = {finger: {"joints": [n for n in names if f"_{finger}_" in n]} for finger in FINGERS}
        dump_yaml(DEST / f"joint_groups{suffix}.yaml", {"active_joints": names, "fingers": groups})
        dump_yaml(DEST / f"eval_config{suffix}.yaml", {
            "robot_name": "TsenseI_A3.0", "urdf_path": f"urdf/{model.urdf_path.name}",
            "keypoint_config": kp, "joint_groups": f"joint_groups{suffix}.yaml",
            "base_link": "base_link", "active_joints": names,
        })
        dump_yaml(DEST / f"q_init{suffix}.yaml", {"q": dict.fromkeys(names, 0.0)})
        sides[side] = {"urdf": f"urdf/{model.urdf_path.name}", "keypoints": kp,
                       "eval_config": f"eval_config{suffix}.yaml", "mjcf": f"mjcf/mjmodel{suffix}.xml"}
        if side == "left":
            dump_yaml(DEST / kp, draft_keypoints(model), preserve=True)
    if not (DEST / "robot_keypoints_21_right.yaml").exists():
        sync_right_keypoints()
    dump_yaml(DEST / "hand_profile.yaml", {
        "schema_version": 1, "model_id": MODEL_ID, "display_name": DISPLAY_NAME,
        "coordinate_space": "urdf_radians", "sides": sides,
        "fixed_joints": {}, "hardware": {"enabled": False, "adapter": None},
        "retarget": {"lambda_smooth": 0.1, "lambda_reg": 0.1,
                     "init_max_iters": 2, "topo_max_iters": 3},
        "mirror_axis": "y",
        "notes": ["Pinky renamed by function: MCP flexion, swing abduction, PIP, DIP; CAD geometry and limits unchanged.",
                  "Keypoints are draft until manually calibrated; generate collision MJCF with tools/generate_profile_collision.py.",
                  "A3.0 uses A3.2 MCP/PIP linkage conversion and direct rotary DIP; downlink is reserved, hardware disabled."],
    }, preserve=True)
    dump_yaml(DEST / "source_mapping.yaml", {
        "source": SOURCE_URDF.relative_to(ROOT).as_posix(), "renames": {},
        "naming": "canonical functional names stored directly in source URDF",
        "source_urdf_sha256": hashlib.sha256(SOURCE_URDF.read_bytes()).hexdigest(),
        "source_package_sha256": source_fingerprint(),
        "geometry_changed": False, "right_mirror": "XZ: M=diag(1,-1,1), rotation axes -M; equal q",
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
        generate_profile(MODEL_ID, profile_root=DEST)
    print(DEST)


if __name__ == "__main__":
    main()
