#!/usr/bin/env python3
"""Generate conservative low-poly A3.0/A3.2 collision assets and MJCFs.

Pair masks are editable in collision_geometry.yaml. Finger and enabled palm
hulls use each side's URDF visual frames; reruns cache verified meshes.
These inertias/gains are simulation proxies, not identified physical dynamics.
"""
from __future__ import annotations

import argparse
from dataclasses import replace
import hashlib
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "src")]
from hand_profiles import A30_MODEL_IDS, A32_MODEL_IDS, load_hand_profile
from TsenseI_retargeting.fk.urdf import UrdfModel
from collision_outer_hull import DEFAULT_ERROR, VERSION as COLLISION_PROXY_VERSION, upgrade_geometry

FINGERS = ("thumb", "index", "mid", "ring", "pinky")


def fmt(values) -> str:
    return " ".join(f"{float(v):.15g}" for v in np.asarray(values).reshape(-1))


def initial_geometry(model: UrdfModel, model_id="TsenseI_v3_2") -> dict:
    specs = []
    # Cover actual finger phalanges, skipping the compact mounting blocks
    # inside the palm. Cross-finger contact is enabled on every proxy.
    finger_parts = {
        "thumb": ("rotaback_link2", "MCP_link", "PIP_link"),
        "index": ("swing_link", "PIP_link", "DIP_link"),
        "mid": ("swing_link", "PIP_link", "DIP_link"),
        "ring": ("swing_link", "PIP_link", "DIP_link"),
        "pinky": ("MCP_link", "PIP_link", "DIP_link"),
    }
    if model_id in A30_MODEL_IDS or model_id == "TsenseI_v3_2_withid":
        finger_parts["pinky"] = ("swing_link", "PIP_link", "DIP_link")
    for fi, finger in enumerate(FINGERS):
        for pi, part in enumerate(finger_parts[finger]):
            link = f"left_hand_{finger}_{part}"
            # Mesh files are supplied by upgrade_geometry before MJCF output.
            specs.append({"link": link, "type": "mesh", "finger": finger,
                          "contype": 1 << fi, "conaffinity": (31 ^ (1 << fi)) | (32 if pi > 0 and not model_id.endswith("_withid") else 0)})
    if model_id.endswith("_withid"):
        return {"schema_version": 1, "unit": "meter", "quality_status": "geometry_seed",
                "collision_scope": "different_fingers_only",
                "notes": ["15 finger envelopes fitted to each side's URDF visual meshes.",
                          "Only different fingers collide; palm, wrist, mounting shell and same-finger pairs excluded."],
                "geoms": specs}
    palm = "Left_hand_palm_link"
    specs.append({"link": palm, "type": "mesh", "region": "palm",
                  "contype": 32, "conaffinity": 0})
    return {"schema_version": 1, "unit": "meter", "quality_status": "geometry_seed",
            "notes": ["Conservative outer envelopes are generated from the URDF visual meshes.",
                      "Same-finger pairs are disabled; palm contact applies to distal two links.",
                      "Mounting blocks are omitted. Inspect open, fist and opposition poses."],
            "geoms": specs}


def build_mjcf(model: UrdfModel, geometry: dict, side: str, model_id="TsenseI_v3_2") -> ET.Element:
    root = ET.Element("mujoco", model=f"{model_id}_{side}_contact")
    ET.SubElement(root, "compiler", angle="radian", autolimits="true")
    ET.SubElement(root, "option", timestep="0.002", gravity="0 0 0", iterations="50", integrator="implicitfast")
    default = ET.SubElement(root, "default")
    ET.SubElement(default, "joint", damping="0.5")
    ET.SubElement(default, "geom", friction="0.8 0.005 0.0001", condim="3",
                  solref="0.01 1", solimp="0.9 0.95 0.001 0.5 2")
    assets = ET.SubElement(root, "asset") if any(spec["type"] == "mesh" for spec in geometry["geoms"]) else None
    world = ET.SubElement(root, "worldbody")
    bodies = {model.base_link: ET.SubElement(world, "body", name=model.base_link)}
    pending = list(model.joints)
    while pending:
        remaining = []
        for joint in pending:
            if joint.parent not in bodies:
                remaining.append(joint)
                continue
            xyzw = Rotation.from_matrix(joint.origin[:3, :3]).as_quat()
            body = ET.SubElement(bodies[joint.parent], "body", name=joint.child,
                                 pos=fmt(joint.origin[:3, 3]), quat=fmt(xyzw[[3, 0, 1, 2]]))
            bodies[joint.child] = body
            ET.SubElement(body, "inertial", mass="0.008", pos="0 0 0", diaginertia="2e-6 2e-6 2e-6")
            if joint.type != "fixed":
                ET.SubElement(body, "joint", name=joint.name, type="hinge", axis=fmt(joint.axis),
                              range=fmt([joint.lower, joint.upper]))
        if len(remaining) == len(pending):
            raise ValueError("Disconnected/cyclic URDF")
        pending = remaining
    mirror = np.ones(3)
    if side == "right":
        mirror_axis = geometry.get("mirror_axis", "y" if model_id in A30_MODEL_IDS or model_id.endswith("_withid") else "x")
        mirror = np.array([1, -1, 1] if mirror_axis == "y" else [-1, 1, 1])
    for spec in geometry["geoms"]:
        link = spec["link"]
        if side == "right":
            link = link.replace("left_hand", "right_hand").replace("Left_hand", "Right_hand")
        name = spec.get("name", spec["link"] + "_" + spec["type"])
        if side == "right":
            name = name.replace("left_hand", "right_hand").replace("Left_hand", "Right_hand")
        attrs = {"name": name, "type": spec["type"],
                 "contype": str(spec["contype"]), "conaffinity": str(spec["conaffinity"])}
        if spec["type"] == "mesh":
            attrs["mesh"] = link + "_collision_mesh"
            ET.SubElement(assets, "mesh", name=attrs["mesh"], file=spec["mesh_files"][side])
        elif spec["type"] == "capsule":
            attrs["size"] = fmt(spec["size"])
            attrs["fromto"] = fmt(np.array(spec["fromto"]).reshape(2, 3) * mirror)
        else:
            attrs["size"] = fmt(spec["size"])
            attrs["pos"] = fmt(np.array(spec["pos"]) * mirror)
        ET.SubElement(bodies[link], "geom", **attrs)
    actuators = ET.SubElement(root, "actuator")
    for joint in model.joints:
        if joint.type != "fixed":
            ET.SubElement(actuators, "position", name=joint.name + "_act", joint=joint.name,
                          kp="20", ctrlrange=fmt([joint.lower, joint.upper]))
    return root


def generate_profile(model_id: str, *, max_error: float = DEFAULT_ERROR,
                     force: bool = False, profile_root: Path | None = None) -> None:
    profile = load_hand_profile(model_id)
    if profile_root is not None:
        data = yaml.safe_load((profile_root / "hand_profile.yaml").read_text(encoding="utf-8"))
        profile = replace(profile, root=profile_root, sides=data["sides"])
    geom_path = profile.root / "collision_geometry.yaml"
    if not geom_path.exists():
        geom_path.write_text(yaml.safe_dump(initial_geometry(UrdfModel(profile.path("urdf")), model_id),
                                            sort_keys=False), encoding="utf-8")
    geometry = yaml.safe_load(geom_path.read_text(encoding="utf-8"))
    geometry = upgrade_geometry(profile, geometry, max_error=max_error, force=force)
    geom_path.write_text(yaml.safe_dump(geometry, sort_keys=False), encoding="utf-8")
    for side in ("left", "right"):
        root = build_mjcf(UrdfModel(profile.path("urdf", side)), geometry, side, model_id)
        path = profile.path("mjcf", side)
        path.parent.mkdir(parents=True, exist_ok=True)
        ET.indent(root, space="  ")
        ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)
        print(path)
    # Canonical URDF fingerprints include derived collision elements. Preserve
    # all SDK/source hashes and coordinate notes, refreshing installed hashes.
    mapping_path = profile.root / "sdk_source_mapping.yaml"
    if mapping_path.exists():
        mapping = yaml.safe_load(mapping_path.read_text(encoding="utf-8"))
        for side, entry in mapping.get("sides", {}).items():
            if side in profile.sides:
                entry["installed_urdf_sha256"] = hashlib.sha256(profile.path("urdf", side).read_bytes()).hexdigest()
        mapping["collision_proxy_version"] = COLLISION_PROXY_VERSION
        mapping_path.write_text(yaml.safe_dump(mapping, sort_keys=False, allow_unicode=True), encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", choices=[*A32_MODEL_IDS, *A30_MODEL_IDS, "all"], default="all")
    parser.add_argument("--max-excess-mm", type=float, default=DEFAULT_ERROR * 1000)
    parser.add_argument("--force", action="store_true", help="Recompute even verified cached hulls")
    args = parser.parse_args()
    if args.max_excess_mm <= 0:
        parser.error("--max-excess-mm must be positive")
    for model_id in [*A32_MODEL_IDS, *A30_MODEL_IDS] if args.model == "all" else [args.model]:
        generate_profile(model_id, max_error=args.max_excess_mm / 1000, force=args.force)


if __name__ == "__main__":
    main()
