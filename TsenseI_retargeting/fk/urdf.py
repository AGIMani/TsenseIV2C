from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import math
import re
import xml.etree.ElementTree as ET

import numpy as np

from .mesh import stl_bounds


def _floats(text: str | None, default: tuple[float, ...]) -> tuple[float, ...]:
    if not text:
        return default
    return tuple(float(x) for x in text.split())


def rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=float)
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=float)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=float)
    return rz @ ry @ rx


def origin_matrix(xyz: tuple[float, float, float], rpy: tuple[float, float, float]) -> np.ndarray:
    out = np.eye(4, dtype=float)
    out[:3, :3] = rpy_matrix(*rpy)
    out[:3, 3] = np.array(xyz, dtype=float)
    return out


def axis_angle_matrix(axis: np.ndarray, angle: float) -> np.ndarray:
    norm = np.linalg.norm(axis)
    if norm == 0:
        return np.eye(4, dtype=float)
    x, y, z = axis / norm
    c, s = math.cos(angle), math.sin(angle)
    c1 = 1 - c
    rot = np.array(
        [
            [c + x * x * c1, x * y * c1 - z * s, x * z * c1 + y * s],
            [y * x * c1 + z * s, c + y * y * c1, y * z * c1 - x * s],
            [z * x * c1 - y * s, z * y * c1 + x * s, c + z * z * c1],
        ],
        dtype=float,
    )
    out = np.eye(4, dtype=float)
    out[:3, :3] = rot
    return out


@dataclass(frozen=True)
class UrdfJoint:
    name: str
    type: str
    parent: str
    child: str
    origin: np.ndarray
    axis: np.ndarray
    lower: float | None
    upper: float | None
    effort: float | None
    velocity: float | None
    mimic_joint: str | None
    mimic_multiplier: float
    mimic_offset: float


@dataclass(frozen=True)
class VisualMesh:
    link: str
    filename: str
    path: Path
    origin: np.ndarray
    scale: np.ndarray


class UrdfModel:
    def __init__(self, urdf_path: str | Path):
        self.urdf_path = Path(urdf_path)
        self.package_root = self._find_package_root(self.urdf_path)
        self.tree = ET.parse(self.urdf_path)
        self.root = self.tree.getroot()
        self.robot_name = self.root.attrib.get("name", self.urdf_path.stem)
        self.links = [link.attrib["name"] for link in self.root.findall("link")]
        self.joints = [self._parse_joint(item) for item in self.root.findall("joint")]
        self.joint_by_name = {joint.name: joint for joint in self.joints}
        self.children_by_parent: dict[str, list[UrdfJoint]] = {}
        self.parent_joint_by_child: dict[str, UrdfJoint] = {}
        for joint in self.joints:
            self.children_by_parent.setdefault(joint.parent, []).append(joint)
            self.parent_joint_by_child[joint.child] = joint
        self.base_link = self._infer_base_link()

    @staticmethod
    def _find_package_root(urdf_path: Path) -> Path:
        current = urdf_path.resolve().parent
        for parent in [current, *current.parents]:
            if (parent / "package.xml").exists():
                return parent
        return urdf_path.resolve().parent.parent

    def _parse_joint(self, node: ET.Element) -> UrdfJoint:
        origin = node.find("origin")
        xyz = _floats(origin.attrib.get("xyz") if origin is not None else None, (0.0, 0.0, 0.0))
        rpy = _floats(origin.attrib.get("rpy") if origin is not None else None, (0.0, 0.0, 0.0))
        axis_node = node.find("axis")
        axis = np.array(_floats(axis_node.attrib.get("xyz") if axis_node is not None else None, (1.0, 0.0, 0.0)), dtype=float)
        limit = node.find("limit")
        mimic = node.find("mimic")

        def attr(name: str) -> float | None:
            if limit is None or name not in limit.attrib:
                return None
            return float(limit.attrib[name])

        return UrdfJoint(
            name=node.attrib["name"],
            type=node.attrib.get("type", "fixed"),
            parent=node.find("parent").attrib["link"],
            child=node.find("child").attrib["link"],
            origin=origin_matrix(xyz, rpy),
            axis=axis,
            lower=attr("lower"),
            upper=attr("upper"),
            effort=attr("effort"),
            velocity=attr("velocity"),
            mimic_joint=None if mimic is None else mimic.attrib.get("joint"),
            mimic_multiplier=1.0 if mimic is None else float(mimic.attrib.get("multiplier", 1.0)),
            mimic_offset=0.0 if mimic is None else float(mimic.attrib.get("offset", 0.0)),
        )

    def _infer_base_link(self) -> str:
        children = {joint.child for joint in self.joints}
        roots = [link for link in self.links if link not in children]
        if len(roots) == 1:
            return roots[0]
        for candidate in ("base_link", "palm_link", "hand_base"):
            if candidate in self.links:
                return candidate
        return self.links[0]

    def joint_names(self, movable_only: bool = True, include_mimic: bool = False) -> list[str]:
        if not movable_only:
            return [joint.name for joint in self.joints]
        return [
            joint.name
            for joint in self.joints
            if joint.type != "fixed" and (include_mimic or joint.mimic_joint is None)
        ]

    def joint_limits(self, include_mimic: bool = False) -> dict[str, tuple[float | None, float | None]]:
        limits = {
            joint.name: (joint.lower, joint.upper)
            for joint in self.joints
            if joint.type != "fixed" and (include_mimic or joint.mimic_joint is None)
        }
        if include_mimic:
            return limits
        for joint in self.joints:
            if joint.type == "fixed" or joint.mimic_joint is None:
                continue
            root, multiplier, offset = self.mimic_expression(joint.name)
            if root not in limits or abs(multiplier) <= 1.0e-12:
                continue
            mapped = [
                (bound - offset) / multiplier
                for bound in (joint.lower, joint.upper)
                if bound is not None
            ]
            if not mapped:
                continue
            dependent_lower = min(mapped) if joint.lower is not None and joint.upper is not None else None
            dependent_upper = max(mapped) if joint.lower is not None and joint.upper is not None else None
            if joint.lower is None:
                if multiplier > 0.0:
                    dependent_upper = mapped[0]
                else:
                    dependent_lower = mapped[0]
            elif joint.upper is None:
                if multiplier > 0.0:
                    dependent_lower = mapped[0]
                else:
                    dependent_upper = mapped[0]
            lower, upper = limits[root]
            if dependent_lower is not None:
                lower = dependent_lower if lower is None else max(lower, dependent_lower)
            if dependent_upper is not None:
                upper = dependent_upper if upper is None else min(upper, dependent_upper)
            if lower is not None and upper is not None and lower > upper:
                raise ValueError(f"URDF mimic limits are infeasible for independent joint {root!r}")
            limits[root] = (lower, upper)
        return limits

    def mimic_expression(self, name: str) -> tuple[str, float, float]:
        resolving: set[str] = set()

        def expression(joint_name: str) -> tuple[str, float, float]:
            if joint_name in resolving:
                raise ValueError(f"cyclic URDF mimic relation at joint {joint_name!r}")
            joint = self.joint_by_name.get(joint_name)
            if joint is None:
                raise ValueError(f"URDF mimic references missing joint {joint_name!r}")
            if joint.mimic_joint is None:
                return joint.name, 1.0, 0.0
            resolving.add(joint_name)
            root, multiplier, offset = expression(joint.mimic_joint)
            resolving.remove(joint_name)
            return (
                root,
                joint.mimic_multiplier * multiplier,
                joint.mimic_multiplier * offset + joint.mimic_offset,
            )

        return expression(name)

    def joint_center(self) -> dict[str, float]:
        out: dict[str, float] = {}
        for joint in self.joints:
            if joint.type == "fixed" or joint.mimic_joint is not None:
                continue
            if joint.lower is not None and joint.upper is not None:
                out[joint.name] = 0.5 * (joint.lower + joint.upper)
            else:
                out[joint.name] = 0.0
        return out

    def forward_kinematics(self, q: dict[str, float] | None = None) -> dict[str, np.ndarray]:
        q_resolved = self.expand_joint_values(q)
        poses = {self.base_link: np.eye(4, dtype=float)}
        stack = [self.base_link]
        while stack:
            parent = stack.pop()
            for joint in self.children_by_parent.get(parent, []):
                transform = poses[parent] @ joint.origin
                if joint.type in {"revolute", "continuous"}:
                    transform = transform @ axis_angle_matrix(joint.axis, q_resolved.get(joint.name, 0.0))
                elif joint.type == "prismatic":
                    slide = np.eye(4, dtype=float)
                    slide[:3, 3] = joint.axis * q_resolved.get(joint.name, 0.0)
                    transform = transform @ slide
                poses[joint.child] = transform
                stack.append(joint.child)
        return poses

    def expand_joint_values(self, q: dict[str, float] | None = None) -> dict[str, float]:
        supplied = q or {}
        resolved: dict[str, float] = {}
        resolving: set[str] = set()

        def value(name: str) -> float:
            if name in resolved:
                return resolved[name]
            if name in resolving:
                raise ValueError(f"cyclic URDF mimic relation at joint {name!r}")
            joint = self.joint_by_name.get(name)
            if joint is None:
                raise ValueError(f"URDF mimic references missing joint {name!r}")
            resolving.add(name)
            if joint.mimic_joint is None:
                result = float(supplied.get(name, 0.0))
            else:
                result = joint.mimic_multiplier * value(joint.mimic_joint) + joint.mimic_offset
            resolving.remove(name)
            resolved[name] = result
            return result

        for joint in self.joints:
            if joint.type != "fixed":
                value(joint.name)
        return resolved

    def resolve_mesh_path(self, filename: str) -> Path:
        match = re.match(r"package://([^/]+)/(.+)", filename)
        if match:
            _package, rel = match.groups()
            # Portable packs may rename the folder while URDF keeps the original package:// name.
            return self.package_root / rel
        return (self.urdf_path.parent / filename).resolve()

    def mesh_refs(self) -> list[str]:
        return [mesh.attrib["filename"] for mesh in self.root.findall(".//mesh") if "filename" in mesh.attrib]

    def visual_mesh_refs_by_link(self) -> dict[str, list[str]]:
        out: dict[str, list[str]] = {}
        for link, meshes in self.visual_meshes_by_link().items():
            out[link] = [mesh.filename for mesh in meshes]
        return out

    def visual_meshes_by_link(self) -> dict[str, list[VisualMesh]]:
        out: dict[str, list[VisualMesh]] = {}
        for link in self.root.findall("link"):
            link_name = link.attrib["name"]
            meshes = []
            for visual in link.findall("./visual"):
                origin_node = visual.find("origin")
                xyz = _floats(origin_node.attrib.get("xyz") if origin_node is not None else None, (0.0, 0.0, 0.0))
                rpy = _floats(origin_node.attrib.get("rpy") if origin_node is not None else None, (0.0, 0.0, 0.0))
                origin = origin_matrix(xyz, rpy)
                mesh = visual.find("./geometry/mesh")
                if mesh is None:
                    continue
                filename = mesh.attrib.get("filename")
                if filename:
                    scale = np.array(_floats(mesh.attrib.get("scale"), (1.0, 1.0, 1.0)), dtype=float)
                    meshes.append(
                        VisualMesh(
                            link=link_name,
                            filename=filename,
                            path=self.resolve_mesh_path(filename),
                            origin=origin,
                            scale=scale,
                        )
                    )
            out[link_name] = meshes
        return out

    def missing_meshes(self) -> list[str]:
        missing = []
        for ref in self.mesh_refs():
            if not self.resolve_mesh_path(ref).exists():
                missing.append(ref)
        return missing

    def link_visual_bounds(self, link: str) -> tuple[np.ndarray, np.ndarray] | None:
        corners = self.link_visual_bbox_corners(link)
        if corners is None:
            return None
        return corners.min(axis=0), corners.max(axis=0)

    def link_visual_bbox_corners(self, link: str) -> np.ndarray | None:
        corners = []
        for mesh in self.visual_meshes_by_link().get(link, []):
            if mesh.path.suffix.lower() != ".stl" or not mesh.path.exists():
                continue
            lo, hi = stl_bounds(mesh.path)
            local = _bbox_corners(lo, hi) * mesh.scale.reshape(1, 3)
            homogeneous = np.c_[local, np.ones(local.shape[0])]
            transformed = (mesh.origin @ homogeneous.T).T[:, :3]
            corners.append(transformed)
        if not corners:
            return None
        return np.vstack(corners)

    def link_chain(self, link: str) -> list[str]:
        if link not in self.links:
            raise KeyError(link)
        chain = [link]
        current = link
        while current in self.parent_joint_by_child:
            joint = self.parent_joint_by_child[current]
            chain.append(joint.parent)
            current = joint.parent
        return list(reversed(chain))


def _bbox_corners(lo: np.ndarray, hi: np.ndarray) -> np.ndarray:
    return np.array(
        [
            [lo[0], lo[1], lo[2]],
            [lo[0], lo[1], hi[2]],
            [lo[0], hi[1], lo[2]],
            [lo[0], hi[1], hi[2]],
            [hi[0], lo[1], lo[2]],
            [hi[0], lo[1], hi[2]],
            [hi[0], hi[1], lo[2]],
            [hi[0], hi[1], hi[2]],
        ],
        dtype=float,
    )
