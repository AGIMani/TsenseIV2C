"""Measure the user-confirmed thumb zero; never connect to hardware or edit models."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "src")]
from TsenseI_retargeting.fk.urdf import UrdfModel
from a32_kinematics.joint_coordinates import THUMB_ZERO_REFERENCE

THUMB_JOINTS = ("left_hand_thumb_rota_block_joint", "left_hand_thumb_rotaback_joint2")


def palm_relative_thumb_rotation(model, pose=None):
    """Use the complete MCP frame, not just its long axis, to fix both DOFs."""
    fk = model.forward_kinematics(pose or {})
    return fk["Left_hand_palm_link"][:3, :3].T @ fk["left_hand_thumb_MCP_link"][:3, :3]


def measure(reference=ROOT / THUMB_ZERO_REFERENCE):
    reference = Path(reference).resolve()
    ref = UrdfModel(reference)
    fk = ref.forward_kinematics({})
    thumb = fk["left_hand_thumb_MCP_link"][:3, 2]
    middle = fk["left_hand_mid_MCP_link"][:3, 2]
    target = palm_relative_thumb_rotation(ref)
    models = {}
    sources = {
        "TsenseI_v3_2": ROOT / "vendor/a3-cpp-sdk/daxian_hand_v3_2/urdf/daxian_hand_left_v3_2.urdf",
        "TsenseI_v3_2_withid": ROOT / "TsenseI_V3.2_hand_withid_fixed/urdf/TsenseI_left.urdf",
    }
    for name, source in sources.items():
        model = UrdfModel(source)

        def residual(q):
            actual = palm_relative_thumb_rotation(model, dict(zip(THUMB_JOINTS, q)))
            return Rotation.from_matrix(actual.T @ target).as_rotvec()

        fit = least_squares(residual, [0., 0.], bounds=(-np.pi / 2, np.pi / 2),
                            xtol=1e-14, ftol=1e-14, gtol=1e-14)
        error = float(np.rad2deg(np.linalg.norm(fit.fun)))
        if not fit.success or error > .001:
            raise ValueError(f"Reference thumb orientation cannot be matched: {name}, {error} deg")
        models[name] = {
            "source": source.relative_to(ROOT).as_posix(),
            "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
            "origin_offsets_rad": dict(zip(THUMB_JOINTS, fit.x.tolist())),
            "origin_offsets_deg": dict(zip(THUMB_JOINTS, np.rad2deg(fit.x).tolist())),
            "orientation_residual_deg": error,
        }
    return {
        "reference": reference.relative_to(ROOT).as_posix() if reference.is_relative_to(ROOT) else str(reference),
        "reference_sha256": hashlib.sha256(reference.read_bytes()).hexdigest(),
        "method": "All joints at zero. Match palm-relative thumb MCP orientation using the two thumb base axes; preserve each model's link lengths and joint translations.",
        "opening_definition": "3D angle between the +Z longitudinal axes of thumb MCP and middle MCP at q=0; not a screenshot projection or a motor angle.",
        "thumb_middle_opening_deg": float(np.rad2deg(np.arccos(np.clip(thumb @ middle, -1, 1)))),
        "models": models,
        "hardware_command_offsets_changed": False,
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reference", type=Path, default=ROOT / THUMB_ZERO_REFERENCE)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = json.dumps(measure(args.reference), ensure_ascii=False, indent=2) + "\n"
    if args.output:
        args.output.write_text(result, encoding="utf-8")
    else:
        print(result, end="")
