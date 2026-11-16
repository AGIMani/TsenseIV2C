"""Geometry and source fidelity checks for the 2026-11-16 replacement CAD."""
from pathlib import Path
import sys
import hashlib
import os
import unittest
import xml.etree.ElementTree as ET

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "src"), str(ROOT / "tools")]
from TsenseI_retargeting.fk.urdf import UrdfModel
from import_a32_customer_exterior import JOINT_SUFFIXES, LINK_SUFFIXES, MIRROR, mirror_stl

ASSETS = ROOT / "assets/hands/TsenseI_hand_v3_2_withid"


class CustomerExteriorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.models = {side: UrdfModel(ASSETS / "urdf" / f"TsenseI_hand_{side}_v3_2_withid.urdf")
                      for side in ("left", "right")}
        cls.points = {side: yaml.safe_load((ASSETS / ("robot_keypoints_21" +
                         ("_right" if side == "right" else "") + ".yaml")).read_text())["keypoints"]
                      for side in cls.models}

    def test_source_joint_frames_and_exact_limits_survive_aliasing(self):
        mapping = yaml.safe_load((ASSETS / "source_mapping.yaml").read_text())
        for side, model in self.models.items():
            by_name = {j.get("name"): j for j in model.root.findall("joint")}
            for name, source in mapping["source_joint_attributes"][side].items():
                joint = by_name[mapping["name_mapping"][side]["joints"][name]]
                for element in ("origin", "axis", "limit"):
                    self.assertEqual(source[element], joint.find(element).attrib)
                for element in ("parent", "child"):
                    self.assertEqual(mapping["name_mapping"][side]["links"][source[element]["link"]],
                                     joint.find(element).get("link"))
            self.assertEqual(len(model.root.findall(".//visual")), 37)
            self.assertEqual(len(model.joint_names()), 20)
            self.assertFalse(model.missing_meshes())

    def test_every_supplied_mesh_is_byte_identical_and_hashes_match(self):
        mapping = yaml.safe_load((ASSETS / "source_mapping.yaml").read_text())
        for name, digest in mapping["imported_files_sha256"].items():
            self.assertEqual(hashlib.sha256((ASSETS / name).read_bytes()).hexdigest(), digest, name)
        for name, digest in mapping["installed_sha256"].items():
            self.assertEqual(hashlib.sha256((ASSETS / name).read_bytes()).hexdigest(), digest, name)
        for side in ("left", "right"):
            self.assertGreater(len(list((ASSETS / "meshes" / side).rglob("*.stl"))), 50)
            source = mapping["source_inertial_attributes"][side]
            for name, attrs in source.items():
                link = self.models[side].root.find("./link[@name='" + mapping["name_mapping"][side]["links"][name] + "']")
                for tag, expected in attrs.items():
                    self.assertEqual(link.find("inertial/" + tag).attrib, expected)

    def test_project_contains_only_the_two_active_urdfs(self):
        found = set()
        for folder, dirs, files in os.walk(ROOT):
            dirs[:] = [d for d in dirs if d not in (".git", ".venvs", "node_modules", ".work")]
            found.update(Path(folder) / name for name in files if name.lower().endswith((".urdf", ".xacro")))
        self.assertEqual(found, {ASSETS / "urdf" / f"TsenseI_hand_{side}_v3_2_withid.urdf" for side in self.models})
        for side in self.models:
            base = __import__('json').loads((ASSETS / f"source_package/{side}/config/base_frame.json").read_text())
            self.assertEqual(base["selected_hole_number"], 1)
            self.assertEqual(base["mounting_face_new_z_m"], 0)

    def test_all_link_poses_and_keypoints_mirror_at_equal_angles(self):
        rng = np.random.default_rng(20261116)
        mirror = np.diag([1., -1., 1., 1.])
        limits = self.models["left"].joint_limits()
        poses = [{name: 0. for name in limits},
                 {name: lo for name, (lo, hi) in limits.items()},
                 {name: hi for name, (lo, hi) in limits.items()}]
        poses += [{name: rng.uniform(lo, hi) for name, (lo, hi) in limits.items()} for _ in range(12)]
        for left_q in poses:
            left = self.models["left"].forward_kinematics(left_q)
            right = self.models["right"].forward_kinematics({n.replace("left_hand_", "right_hand_"): v for n, v in left_q.items()})
            for name, transform in left.items():
                other = name.replace("left_hand_", "right_hand_").replace("Left_hand_", "Right_hand_")
                np.testing.assert_allclose(right[other], mirror @ transform @ mirror, atol=3e-14)
            for a, b in zip(self.points["left"], self.points["right"]):
                p = left[a["link"]] @ np.r_[a["local_xyz"], 1.]
                q = right[b["link"]] @ np.r_[b["local_xyz"], 1.]
                np.testing.assert_allclose(q, mirror @ p, atol=3e-14)

    def test_physical_inertias_and_centres_are_mirrored(self):
        links = {side: {l.get("name"): l for l in model.root.findall("link")}
                 for side, model in self.models.items()}
        total = 0.
        reflection = np.diag(MIRROR)
        for name, left in links["left"].items():
            a = left.find("inertial")
            b = links["right"][name.replace("left_hand_", "right_hand_").replace("Left_hand_", "Right_hand_")].find("inertial")
            if a is None:
                self.assertIsNone(b)
                continue
            total += float(a.find("mass").get("value"))
            self.assertEqual(a.find("mass").attrib, b.find("mass").attrib)
            np.testing.assert_allclose(np.fromstring(b.find("origin").get("xyz"), sep=" "),
                np.fromstring(a.find("origin").get("xyz"), sep=" ") * MIRROR, atol=1e-16)
            def tensor(element):
                p = {k: float(v) for k, v in element.find("inertia").attrib.items()}
                return np.array([[p["ixx"], p["ixy"], p["ixz"]], [p["ixy"], p["iyy"], p["iyz"]], [p["ixz"], p["iyz"], p["izz"]]])
            np.testing.assert_allclose(tensor(b), reflection @ tensor(a) @ reflection, atol=1e-18)
            self.assertGreater(np.linalg.eigvalsh(tensor(a)).min(), 0.)
        self.assertAlmostEqual(total, 1.053011880784252, places=12)

    def test_keypoint_chains_observe_each_flexion_joint(self):
        for side, model in self.models.items():
            points = self.points[side]
            self.assertEqual([p["id"] for p in points], list(range(21)))
            def positions(q):
                fk = model.forward_kinematics(q)
                return np.array([(fk[p["link"]] @ np.r_[p["local_xyz"], 1.])[:3] for p in points])
            zero = positions({})
            # Every controlled joint must affect some subsequent anatomical point.
            # In particular the distal flexion is only observable at its fingertip.
            for name in model.joint_names():
                changed = positions({name: .1})
                self.assertGreater(np.linalg.norm(changed - zero, axis=1).max(), .0005, name)
            for start in (1, 5, 9, 13, 17):
                lengths = np.linalg.norm(np.diff(zero[start:start + 4], axis=0), axis=1)
                self.assertTrue(np.all((lengths > .01) & (lengths < .075)), (side, start, lengths))


if __name__ == "__main__":
    unittest.main()
