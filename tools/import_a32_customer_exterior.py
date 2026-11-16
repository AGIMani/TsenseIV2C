"""Install both hands from the 2026-11-16 shell package.

The application's stable 20 channel names are aliases for the supplied names.
No source joint angle, origin, inertia or mesh is altered. Run with the track
environment; this tool only generates assets and never connects to hardware.
"""
from __future__ import annotations

import argparse
import copy
from datetime import datetime
import hashlib
import json
from pathlib import Path, PurePosixPath
import shutil
import sys
import tempfile
import xml.etree.ElementTree as ET
import zipfile

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "src"), str(ROOT / "tools")]
from TsenseI_retargeting.fk.urdf import UrdfModel
from collision_outer_hull import source_points
from generate_profile_collision import generate_profile

MODEL = "TsenseI_v3_2_withid"
VERSION = "customer-shell-20261116"
DEST = ROOT / "assets/hands/TsenseI_hand_v3_2_withid"
MIRROR = np.array([1., -1., 1.])
FINGERS = {"index": "index", "middle": "mid", "ring": "ring", "little": "pinky"}
JOINT_SUFFIXES = {
    "thumb_base_rotation_1_joint": "thumb_rota_block_joint",
    "thumb_base_rotation_2_joint": "thumb_rotaback_joint2",
    "thumb_mcp_flexion_joint": "thumb_MCP_joint",
    "thumb_ip_flexion_joint": "thumb_PIP_joint",
    **{f"{source}_{part}_joint": f"{target}_{suffix}_joint"
       for source, target in FINGERS.items()
       for part, suffix in (("mcp_flexion", "MCP"), ("mcp_swing", "swing"),
                            ("pip_flexion", "PIP"), ("dip_flexion", "DIP"))},
}
LINK_SUFFIXES = {
    "thumb_base_axis1_carrier_link": "thumb_rota_block_link",
    "thumb_metacarpal_link": "thumb_rotaback_link2",
    "thumb_proximal_phalanx_link": "thumb_MCP_link",
    "thumb_distal_phalanx_link": "thumb_PIP_link",
    **{f"{source}_{part}_link": f"{target}_{suffix}_link"
       for source, target in FINGERS.items()
       for part, suffix in (("mcp_carrier", "MCP"), ("proximal_phalanx", "swing"),
                            ("middle_phalanx", "PIP"), ("distal_phalanx", "DIP"))},
}


def fmt(values):
    return " ".join(format(float(v), ".17g") for v in values)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_xml(path, root):
    path.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)


def write_yaml(path, data):
    path.write_text(yaml.safe_dump(data, sort_keys=False, allow_unicode=True), encoding="utf-8")


def mirror_stl(raw):
    """Reflect vertices and normals and reverse triangle winding, in binary STL."""
    if len(raw) < 84:
        raise ValueError("Expected a binary STL")
    count = int.from_bytes(raw[80:84], "little")
    if len(raw) != 84 + count * 50:
        raise ValueError("Expected a complete binary STL")
    dtype = np.dtype([("normal", "<f4", 3), ("vertices", "<f4", (3, 3)), ("attr", "<u2")])
    triangles = np.frombuffer(raw, dtype=dtype, offset=84).copy()
    triangles["normal"] *= MIRROR
    triangles["vertices"] *= MIRROR
    triangles["vertices"] = triangles["vertices"][:, [0, 2, 1]]
    return raw[:84] + triangles.tobytes()


def normalized_urdf(source, side):
    root = copy.deepcopy(source)
    names = {name: f"{side}_hand_{suffix}" for name, suffix in LINK_SUFFIXES.items()}
    names["hand_base_link"] = "base_link"
    joints = {name: f"{side}_hand_{suffix}" for name, suffix in JOINT_SUFFIXES.items()}
    for link in root.findall("link"):
        link.set("name", names[link.get("name")])
    for joint in root.findall("joint"):
        joint.set("name", joints[joint.get("name")])
        for tag in ("parent", "child"):
            element = joint.find(tag)
            element.set("link", names[element.get("link")])
    for mesh in root.iter("mesh"):
        relative = PurePosixPath(mesh.get("filename"))
        if relative.parts[:2] != ("..", "meshes"):
            raise ValueError(f"Unexpected source mesh: {relative}")
        mesh.set("filename", str(PurePosixPath("../meshes") / side / PurePosixPath(*relative.parts[2:])))
    # Historical consumers address this palm frame. It is an identity alias,
    # with no duplicate mesh or mass; the supplied base retains its full inertia.
    palm = f"{side.title()}_hand_palm_link"
    ET.SubElement(root, "link", name=palm)
    fixed = ET.SubElement(root, "joint", name=f"{side.title()}_hand_palm_joint", type="fixed")
    ET.SubElement(fixed, "parent", link="base_link")
    ET.SubElement(fixed, "child", link=palm)
    ET.SubElement(fixed, "origin", xyz="0 0 0", rpy="0 0 0")
    # Stable channel order is part of saved motions and hardware adapters.
    order = list(JOINT_SUFFIXES)
    elements = {j.get("name"): j for j in root.findall("joint") if j.get("type") != "fixed"}
    for element in elements.values():
        root.remove(element)
    for name in order:
        root.append(elements[joints[name]])
    root.set("name", f"TsenseI_hand_v3_2_withid_{side}_customer_shell")
    root.set("a32_cad_source", VERSION)
    root.set("a32_joint_coordinates", "a32-signed-swing-v2")
    root.set("a32_swing_axis_alignment", "id-cad-direction-v2")
    root.set("a32_thumb_zero_alignment", "source-cad-20261116")
    return root, joints, names


def annotate_keypoints(stage, template):
    # Each supplied side owns its geometry, including its wrist mounting frame.
    for side in ("left", "right"):
        model = UrdfModel(stage / f"urdf/TsenseI_hand_{side}_v3_2_withid.urdf")
        rows = copy.deepcopy(template["keypoints"])
        palm = source_points(model, "base_link")
        cap = palm[palm[:, 2] <= palm[:, 2].min() + .001]
        rows[0].update(link="base_link", local_xyz=((cap.min(axis=0) + cap.max(axis=0)) / 2).tolist())
        for index, suffix in zip((1, 2, 3), ("thumb_rotaback_link2", "thumb_MCP_link", "thumb_PIP_link")):
            rows[index].update(link=f"{side}_hand_" + suffix, local_xyz=[0., 0., 0.])
        for finger, start in (("index", 5), ("mid", 9), ("ring", 13), ("pinky", 17)):
            for index, part in zip(range(start, start + 3), ("MCP", "PIP", "DIP")):
                rows[index].update(link=f"{side}_hand_{finger}_{part}_link", local_xyz=[0., 0., 0.])
        for index in (4, 8, 12, 16, 20):
            link = rows[index - 1]["link"]
            points = source_points(model, link)
            # +Y follows the left distal phalanx; the supplied right is its Y mirror.
            direction = 1 if side == "left" else -1
            extent = points[:, 1] * direction
            cap = points[extent >= extent.max() - .001]
            rows[index].update(link=link, local_xyz=((cap.min(axis=0) + cap.max(axis=0)) / 2).tolist())
        for row in rows:
            row.update(annotation_status="auto_initialized",
                       note="Supplied shell joint centre or distal/wrist surface cap; physical calibration pending.")
        data = copy.deepcopy(template)
        data.update(hand_side=side, urdf_path=f"urdf/TsenseI_hand_{side}_v3_2_withid.urdf",
                    quality_status="Draft", annotation_version=5,
                    generated_by="tools/import_a32_customer_exterior.py", keypoints=rows)
        write_yaml(stage / ("robot_keypoints_21" + ("_right" if side == "right" else "") + ".yaml"), data)


def install(package, *, destination=DEST):
    package, destination = Path(package).resolve(), Path(destination).resolve()
    with zipfile.ZipFile(package) as archive:
        sources, prefixes = {}, {}
        for side in ("left", "right"):
            candidates = [n for n in archive.namelist() if n.endswith(f"/urdf/tsensei_hand_{side}.urdf")]
            if len(candidates) != 1:
                raise ValueError(f"Expected exactly one supplied {side} URDF")
            name = candidates[0]
            prefixes[side] = name.rsplit("urdf/", 1)[0]
            source = sources[side] = ET.fromstring(archive.read(name))
            if {j.get("name") for j in source.findall("joint")} != set(JOINT_SUFFIXES):
                raise ValueError("Source does not contain the expected twenty independent joints")
            limits = json.loads(archive.read(prefixes[side] + "config/joint_limits.json"))
            expected = {j["joint"]: (j["lower_rad"], j["upper_rad"]) for j in limits["joints"]}
            actual = {j.get("name"): tuple(float(j.find("limit").get(k)) for k in ("lower", "upper"))
                      for j in source.findall("joint")}
            if actual != expected or len(source.findall("link")) != 21:
                raise ValueError("Source structure or joint_limits.json disagrees with URDF")
        stage = Path(tempfile.mkdtemp(prefix=".a32-customer-", dir=destination.parent))
        try:
            for name in ("hand_profile.yaml", "joint_groups.yaml", "joint_groups_right.yaml",
                         "eval_config.yaml", "eval_config_right.yaml", "collision_geometry.yaml"):
                shutil.copy2(destination / name, stage / name)
            template = yaml.safe_load((destination / "robot_keypoints_21.yaml").read_text())
            imported_hashes, maps, source_joints, source_inertials = {}, {}, {}, {}
            for side, prefix in prefixes.items():
                for name in archive.namelist():
                    if not name.startswith(prefix) or name.endswith("/"):
                        continue
                    relative = PurePosixPath(name[len(prefix):])
                    if relative.is_absolute() or ".." in relative.parts:
                        raise ValueError("Unsafe archive path")
                    if relative.parts[0] == "meshes" and relative.suffix.lower() == ".stl":
                        target = stage / "meshes" / side / Path(*relative.parts[1:])
                    elif relative.parts[0] == "config" and relative.suffix.lower() in (".json", ".yaml"):
                        target = stage / "source_package" / side / Path(relative)
                    else:
                        # Do not retain additional URDFs, ROS duplicates, launch
                        # scripts or references to a removed model in this project.
                        continue
                    raw = archive.read(name)
                    target.parent.mkdir(parents=True, exist_ok=True)
                    target.write_bytes(raw)
                    imported_hashes[str(target.relative_to(stage))] = hashlib.sha256(raw).hexdigest()
                source = sources[side]
                root, joints, links = normalized_urdf(source, side)
                filename = f"TsenseI_hand_{side}_v3_2_withid.urdf"
                write_xml(stage / "urdf" / filename, root)
                model = UrdfModel(stage / "urdf" / filename)
                if model.missing_meshes():
                    raise ValueError(model.missing_meshes())
                suffix = "_right" if side == "right" else ""
                write_yaml(stage / f"q_init{suffix}.yaml", {"q": {name: 0. for name in model.joint_names()}})
                maps[side] = {"joints": joints, "links": links}
                source_joints[side] = {j.get("name"): {tag: dict(j.find(tag).attrib)
                    for tag in ("parent", "child", "origin", "axis", "limit")} for j in source.findall("joint")}
                source_inertials[side] = {l.get("name"): {tag: dict(l.find("inertial/" + tag).attrib)
                    for tag in ("origin", "mass", "inertia")} for l in source.findall("link")}
            annotate_keypoints(stage, template)
            profile = yaml.safe_load((stage / "hand_profile.yaml").read_text())
            profile.update(mirror_axis="y", mirror_plane_offset_m=0., right_hand_provisional=False,
                           cad_version=VERSION, notes=[
                               "Both hands supplied by the 2026-11-16 shell package; mesh bytes, inertias, joint origins, axes and limits retained.",
                               "Base coordinates use the supplied wrist mounting hole; no extra mirror or root rotation applied.",
                               "Only the two active runtime URDFs are retained; source provenance is recorded as metadata and hashes.",
                               "Stable application names alias the supplied semantic names; thumb IP uses the historical thumb_PIP channel.",
                               "The palm frame is a massless identity alias of the supplied base.",
                               "Keypoints regenerated per side from joint centres and surface caps; physical calibration remains pending.",
                               "Collision envelopes rebuilt; different-finger-only collision policy retained."])
            write_yaml(stage / "hand_profile.yaml", profile)
            generate_profile(MODEL, profile_root=stage, force=True)
            manifest = {
                "source": package.name, "cad_version": VERSION, "imported_on": "2026-11-16",
                "archive_sha256": sha(package), "imported_files_sha256": imported_hashes,
                "source_archive_members": {side: prefix + f"urdf/tsensei_hand_{side}.urdf" for side, prefix in prefixes.items()},
                "source_urdf_sha256": {side: hashlib.sha256(archive.read(prefix + f"urdf/tsensei_hand_{side}.urdf")).hexdigest() for side, prefix in prefixes.items()},
                "name_mapping": maps, "source_joint_attributes": source_joints, "source_inertial_attributes": source_inertials,
                "kinematics": "Both supplied sides retained exactly; names and asset paths only aliased.",
                "joint_coordinate_version": "a32-signed-swing-v2", "swing_axis_alignment": "id-cad-direction-v2",
                "thumb_zero_alignment": "source-cad-20261116", "thumb_origin_offsets_rad": {},
                "inertials": "Physical supplied URDF inertials retained; MJCF is a kinematic collision proxy.",
                "keypoints": "Joint centres and 1 mm distal/wrist surface caps from each supplied side; physical calibration pending.",
                "installed_sha256": {str(p.relative_to(stage)): sha(p) for pattern in ("urdf/*.urdf", "mjcf/mjmodel*.xml") for p in sorted(stage.glob(pattern))},
            }
            write_yaml(stage / "source_mapping.yaml", manifest)
            backups = ROOT.parent / "model-backups"
            backups.mkdir(exist_ok=True)
            backup = backups / (destination.name + "-" + datetime.now().strftime("%Y%m%d-%H%M%S-%f"))
            destination.rename(backup)
            try:
                stage.rename(destination)
            except BaseException:
                backup.rename(destination)
                raise
            print(json.dumps({"installed": str(destination), "backup": str(backup), "cad_version": VERSION}))
        finally:
            if stage.exists():
                shutil.rmtree(stage)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("package", type=Path)
    install(parser.parse_args().package)
