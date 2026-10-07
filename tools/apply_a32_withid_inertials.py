"""Apply the supplied 952 g ID-hand inertials by joint/child-link identity.

The stored values are already scaled from the original CAD inertia. Copy them
directly; applying this operation again must not scale inertia a second time.
Joint coordinates, limits, visuals and collision geometry are left intact.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "assets/hands/TsenseI_hand_v3_2_withid"
PARAMETERS_NAME = "inertial_parameters.json"


def inertial_manifest_fields(package: Path) -> dict:
    path = package / PARAMETERS_NAME
    if not path.exists():
        return {}
    return {"inertial_parameters": PARAMETERS_NAME,
            "inertial_parameters_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            "inertial_source": "TsenseI_V3.2_hand_mass_inertia_corrected_952g",
            "inertial_mass_per_hand_kg": 0.952}


def apply_urdf_inertials(root: ET.Element, side: str, parameters: Path) -> ET.Element:
    data = json.loads(parameters.read_text(encoding="utf-8"))
    if data["schema_version"] != 1 or data["model_id"] != "TsenseI_v3_2_withid":
        raise ValueError("Unsupported ID-hand inertial parameters")
    values = data["hands"][side]
    links = {link.get("name"): link for link in root.findall("link")}
    if links.keys() != values.keys():
        raise ValueError("Inertial parameters must match every URDF link exactly")
    joints = {joint.find("child").get("link"): joint for joint in root.findall("joint")}
    total = 0.
    for name, row in values.items():
        joint = joints.get(name)
        identity = (joint.get("name"), joint.find("parent").get("link")) if joint is not None else (None, None)
        if identity != (row["joint"], row["parent_link"]):
            raise ValueError(f"Joint/child-link mapping changed: {name}")
        inertial = links[name].find("inertial")
        if inertial is None:
            raise ValueError(f"Missing inertial frame: {name}")
        for key in ("xyz", "rpy"):
            actual = np.fromstring(inertial.find("origin").get(key, "0 0 0"), sep=" ")
            expected = np.fromstring(row["origin"][key], sep=" ")
            if actual.shape != (3,) or expected.shape != (3,) or not np.allclose(actual, expected, rtol=0, atol=1e-12):
                raise ValueError(f"Inertial frame changed: {name}/{key}")
        mass = float(row["mass_kg"])
        a, b, c, d, e, f = (float(row["inertia"][key]) for key in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz"))
        matrix = np.array([[a, b, c], [b, d, e], [c, e, f]])
        if not np.isfinite(mass) or mass <= 0 or not np.isfinite(matrix).all():
            raise ValueError(f"Invalid mass/inertia: {name}")
        eigenvalues = np.linalg.eigvalsh(matrix)
        if eigenvalues[0] <= 0 or eigenvalues[2] > eigenvalues[:2].sum() + 1e-12:
            raise ValueError(f"Nonphysical inertia tensor: {name}")
        total += mass
    if not np.isclose(total, data["mass_per_hand_kg"], rtol=0, atol=1e-12):
        raise ValueError("Unexpected total hand mass")
    # Validate the whole mapping before changing any node.
    for name, row in values.items():
        inertial = links[name].find("inertial")
        inertial.find("mass").set("value", row["mass_kg"])
        inertial.find("inertia").attrib.update(row["inertia"])
    return root


def sync_installed_assets(package: Path = PACKAGE) -> None:
    outputs = []
    for side in ("left", "right"):
        path = package / f"urdf/TsenseI_hand_{side}_v3_2_withid.urdf"
        root = apply_urdf_inertials(ET.parse(path).getroot(), side, package / PARAMETERS_NAME)
        ET.indent(root, space="  ")
        outputs.append((path, root))
    for path, root in outputs:
        ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)
    manifest_path = package / "source_mapping.yaml"
    manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    manifest.update(inertial_manifest_fields(package))
    manifest_path.write_text(yaml.safe_dump(manifest, sort_keys=False, allow_unicode=True), encoding="utf-8")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--package", type=Path, default=PACKAGE)
    args = parser.parse_args()
    sync_installed_assets(args.package)
