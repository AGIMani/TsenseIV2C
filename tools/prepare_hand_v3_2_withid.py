"""Import the supplied A3.2 ID shell, retaining its joint frames and mesh scale.

The SDK bare model is already installed as TsenseI_v3_2. Verify its geometry
without overwriting the user's annotations. ID landmarks and collision proxies
are independent. Neither the SDK's executables nor its GUI are needed.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import shutil
import sys
import xml.etree.ElementTree as ET

import numpy as np
import trimesh
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "src"), str(ROOT / "tools")]
from TsenseI_retargeting.fk.urdf import UrdfModel
from generate_profile_collision import generate_profile
from collision_outer_hull import VERSION as COLLISION_PROXY_VERSION
from a32_kinematics.mapper import model_joint_to_source_sign
from a32_kinematics.joint_coordinates import (
    COORDINATE_VERSION, SWING_AXIS_ALIGNMENT,
    thumb_zero_manifest, initial_pose, reverse_swing,
)
from apply_a32_joint_ranges import apply_urdf_coordinates
from apply_a32_withid_inertials import (
    PARAMETERS_NAME, apply_urdf_inertials, inertial_manifest_fields,
)

SOURCE = ROOT / "TsenseI_V3.2_hand_withid_fixed"
DEST = ROOT / "assets/hands/TsenseI_hand_v3_2_withid"
BARE = ROOT / "assets/hands/TsenseI_hand_v3_2"
MODEL_ID = "TsenseI_v3_2_withid"
MIRROR = np.diag([1.0, -1.0, 1.0])
FINGERS = ("thumb", "index", "mid", "ring", "pinky")


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def dump(path: Path, data: dict, *, preserve: bool = False) -> None:
    if preserve and path.exists():
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(data, sort_keys=False, allow_unicode=True), encoding="utf-8")


def write_xml(path: Path, root: ET.Element) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)


def source_manifest() -> dict:
    files = {"collision_proxies.json", "model_derivation.json"}
    for side in ("left", "right"):
        path = SOURCE / f"urdf/TsenseI_{side}.urdf"
        files.add(path.relative_to(SOURCE).as_posix())
        for mesh in ET.parse(path).getroot().iter("mesh"):
            files.add((path.parent / mesh.get("filename")).resolve().relative_to(SOURCE.resolve()).as_posix())
    return {name: sha256(SOURCE / name) for name in sorted(files)}


def verify_sdk_bare(sdk_root: Path) -> None:
    """Verify SDK visual geometry and kinematics; collision meshes are derived."""
    verified = {}
    for side in ("left", "right"):
        source = sdk_root / f"daxian_hand_v3_2/urdf/daxian_hand_{side}_v3_2.urdf"
        installed = BARE / f"urdf/TsenseI_hand_{side}_v3_2.urdf"
        a, b = apply_urdf_coordinates(ET.parse(source).getroot()), ET.parse(installed).getroot()
        # Collision envelopes are an intentional derived asset. Visual mesh
        # hashes, joints, inertials and every other attribute remain checked.
        def canonical(root, urdf):
            for link in root.findall("link"):
                for collision in link.findall("collision"):
                    link.remove(collision)
            result = []
            for node in root.iter():
                attrs = dict(node.attrib)
                if node.tag == "robot":
                    attrs.pop("name", None)
                if node.tag == "mesh":
                    filename = attrs.pop("filename")
                    attrs["mesh_sha256"] = sha256((urdf.parent / filename).resolve())
                result.append((node.tag, attrs))
            return result
        if canonical(a, source) != canonical(b, installed):
            raise ValueError(f"Installed {side} bare geometry differs from SDK; retain annotations and review the change")
        verified[side] = {"sdk_urdf_sha256": sha256(source), "installed_urdf_sha256": sha256(installed)}
    dump(BARE / "sdk_source_mapping.yaml", {
        "source": "vendor/a3-cpp-sdk/daxian_hand_v3_2", "geometry_matches": True,
        "comparison": "non-collision XML attributes and visual mesh hashes after application joint coordinate transform; ignore robot label and mesh URI; generated collision envelopes verified separately",
        "joint_coordinate_version": COORDINATE_VERSION,
        "swing_axis_alignment": SWING_AXIS_ALIGNMENT,
        **thumb_zero_manifest("TsenseI_v3_2"),
        "sides": verified, "existing_keypoint_annotations_preserved": True,
    })


def import_urdfs() -> None:
    for side in ("left", "right"):
        source = SOURCE / f"urdf/TsenseI_{side}.urdf"
        root = ET.parse(source).getroot()
        root.set("name", f"TsenseI_hand_v3_2_withid_{side}")
        for mesh in root.iter("mesh"):
            original = (source.parent / mesh.get("filename")).resolve()
            target = DEST / "meshes" / side / original.name
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(original, target)
            mesh.set("filename", f"../meshes/{side}/{original.name}")
        # Keep 0.001 scale on base_link.STL; all other meshes are in metres.
        apply_urdf_coordinates(root, model_id=MODEL_ID)
        if (DEST / PARAMETERS_NAME).exists():
            apply_urdf_inertials(root, side, DEST / PARAMETERS_NAME)
        write_xml(DEST / f"urdf/TsenseI_hand_{side}_v3_2_withid.urdf", root)


def mesh_points(model: UrdfModel, link: str) -> np.ndarray:
    clouds = []
    for visual in model.visual_meshes_by_link()[link]:
        vertices = np.asarray(trimesh.load_mesh(visual.path, process=False).vertices) * visual.scale
        clouds.append(vertices @ visual.origin[:3, :3].T + visual.origin[:3, 3])
    return np.concatenate(clouds)


def keypoint_seed(model: UrdfModel, source_proxies: dict) -> dict:
    template = yaml.safe_load((ROOT / "assets/hands/TsenseI_hand/robot_keypoints_21.yaml").read_text(encoding="utf-8"))
    rows = template["keypoints"]
    wrist = mesh_points(model, model.base_link)
    wrist = wrist[wrist[:, 2] <= np.quantile(wrist[:, 2], .05)].mean(axis=0)
    rows[0].update(link=model.base_link, local_xyz=wrist.tolist())
    fk = model.forward_kinematics({name: reverse_swing(name, 0.) for name in model.joint_names()})
    for fi, finger in enumerate(FINGERS):
        parts = ("rotaback_link2", "MCP_link", "PIP_link", "PIP_link") if finger == "thumb" else (
            "swing_link", "PIP_link", "DIP_link", "DIP_link")
        for offset, part in enumerate(parts):
            link = f"left_hand_{finger}_{part}"
            xyz = np.zeros(3)
            if offset == 0 and finger != "thumb":
                parent = model.parent_joint_by_child[link].parent
                xyz = (np.linalg.inv(fk[link]) @ fk[parent])[:3, 3]
            if offset == 3:
                xyz = np.asarray(source_proxies[link]["preserved_fingertip"])
            rows[1 + fi * 4 + offset].update(link=link, local_xyz=xyz.tolist())
    for row in rows:
        row.update(annotation_status="auto_initialized", note="Independent ID-shell geometry seed; manual calibration pending.")
    template.update(robot_name="TsenseI A3.2 with ID", quality_status="Draft", hand_side="left",
                    urdf_path="urdf/TsenseI_hand_left_v3_2_withid.urdf", annotation_version=1,
                    generated_by="tools/prepare_hand_v3_2_withid.py: uploaded geometry, joint centers and preserved fingertip sites")
    return template


def prepare() -> None:
    fingerprints = source_manifest()
    manifest_path = DEST / "source_mapping.yaml"
    if manifest_path.exists():
        previous = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
        if previous.get("source_sha256") != fingerprints:
            raise ValueError("Source ID geometry changed; preserve and review existing annotations before reimport")
    import_urdfs()
    proxies = json.loads((SOURCE / "collision_proxies.json").read_text(encoding="utf-8"))["hands"]["left"]
    sides = {}
    for side in ("left", "right"):
        suffix = "" if side == "left" else "_right"
        urdf = f"urdf/TsenseI_hand_{side}_v3_2_withid.urdf"
        model = UrdfModel(DEST / urdf)
        names = model.joint_names()
        kp = f"robot_keypoints_21{suffix}.yaml"
        dump(DEST / f"joint_groups{suffix}.yaml", {"active_joints": names,
             "fingers": {finger: {"joints": [n for n in names if f"_{finger}_" in n]} for finger in FINGERS}})
        dump(DEST / f"eval_config{suffix}.yaml", {"robot_name": "TsenseI A3.2 with ID", "urdf_path": urdf,
             "keypoint_config": kp, "joint_groups": f"joint_groups{suffix}.yaml", "base_link": "base_link", "active_joints": names})
        dump(DEST / f"q_init{suffix}.yaml", {"q": initial_pose(model.joint_limits())})
        sides[side] = {"urdf": urdf, "keypoints": kp, "eval_config": f"eval_config{suffix}.yaml", "mjcf": f"mjcf/mjmodel{suffix}.xml"}
        if side == "left":
            dump(DEST / kp, keypoint_seed(model, proxies), preserve=True)
        else:
            points = copy.deepcopy(yaml.safe_load((DEST / "robot_keypoints_21.yaml").read_text(encoding="utf-8")))
            points.update(hand_side="right", urdf_path=urdf, mirrored_from="robot_keypoints_21.yaml", mirror_plane="XZ")
            for row in points["keypoints"]:
                row["link"] = row["link"].replace("left_hand", "right_hand").replace("Left_hand", "Right_hand")
                row["local_xyz"] = (MIRROR @ row["local_xyz"]).tolist()
            dump(DEST / kp, points, preserve=True)
    bare_profile = yaml.safe_load((BARE / "hand_profile.yaml").read_text(encoding="utf-8"))
    dump(DEST / "hand_profile.yaml", {
        "schema_version": 1, "model_id": MODEL_ID, "display_name": "TsenseI A3.2 with ID",
        "coordinate_space": "urdf_radians", "sides": sides, "fixed_joints": {},
        "hardware": {"enabled": False, "adapter": "a32_native_sdk", "supported_sides": ["left"]},
        "retarget": bare_profile["retarget"], "mirror_axis": "y", "right_hand_provisional": True,
        "collision_scope": "different_fingers_only",
        "notes": ["Independent uploaded ID-shell URDF, meshes, landmarks and collision geometry.",
                  "Base STL scale and link geometry retained; application limits and swing origin/axis are transformed.",
                  "Both variants command signed model swing directly, clipped to -0.34..0.34 rad; thumb zero matches user reference CAD.",
                  "Right is the supplied provisional XZ mirror; A3.2 hardware connection supports both sides.",
                  "21 point annotations and collision envelopes require hardware calibration."],
    }, preserve=True)
    generate_profile(MODEL_ID, profile_root=DEST)
    names = UrdfModel(DEST / sides["left"]["urdf"]).joint_names()
    dump(manifest_path, {"source": "TsenseI_V3.2_hand_withid_fixed", "source_sha256": fingerprints,
        **inertial_manifest_fields(DEST),
        "geometry_changed": False, "joint_coordinate_version": COORDINATE_VERSION,
        "joint_coordinate_change": "swing = original ID swing; thumb origin alignment to user reference CAD; application flexion limits",
        "renames": {}, "right_mirror": "XZ; supplied provisional right, equal joint angles",
        "collision_proxy_version": COLLISION_PROXY_VERSION,
        "collision_proxy_scope": "15 verified low-complexity outer meshes; different fingers only",
        "collision_mesh_sha256": {path.relative_to(DEST).as_posix(): sha256(path)
                                  for path in sorted((DEST / "mjcf/collision_meshes").glob("*.stl"))},
        "legacy_source_joint_signs": {name: model_joint_to_source_sign(MODEL_ID, name) for name in names},
        **thumb_zero_manifest(MODEL_ID),
        "axis_comparison": "Rz(+90deg) aligns bare root orientation to ID root; installed swing axis cosines > 0.9999 after bare alignment; original SDK swing axes are opposite to ID"})


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sdk-root", type=Path, default=Path(os.environ.get("A32_SDK_ROOT", ROOT / "vendor/a3-cpp-sdk")))
    args = parser.parse_args()
    verify_sdk_bare(args.sdk_root)
    prepare()
    print(DEST)


if __name__ == "__main__":
    main()
