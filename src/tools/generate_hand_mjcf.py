"""
generate_hand_mjcf.py — 从 URDF 生成「网格碰撞」MJCF（遗留/离线工具）

当前运行时碰撞滤波使用手工维护的胶囊模型：
  assets/hands/TsenseI_hand/mjcf/mjmodel.xml
  → physics.mujoco_filter.MuJoCoPhysicsFilter

本脚本会把 URDF 碰撞网格抽稀后写出 mesh 版 MJCF 与 collision_meshes/，
供离线实验或将来切回网格接触时使用；默认写出 mjmodel_mesh_legacy.xml，
不会覆盖运行时胶囊 mjmodel.xml。

规则（网格方案）：
  - base_link（1mm 坐标占位块）与 *_tip_link（虚拟指尖）跳过
  - 其余连杆用抽稀后的碰撞网格
  - 碰撞对建议只在不同手指之间、以及手指与掌心之间检测

运行：
  E:\\conda\\envs\\hand_dex\\python.exe src/tools/generate_hand_mjcf.py
"""

from __future__ import annotations

import math
import os
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import trimesh

from config_path import (
    COLLISION_MESHES_DIR,
    DEFAULT_URDF_PATH,
    MJCF_DIR,
)

_URDF_PATH = str(DEFAULT_URDF_PATH)
_MJCF_DIR = str(MJCF_DIR)
_MJCF_OUT = str(MJCF_DIR / "mjmodel_mesh_legacy.xml")
_MESH_OUT_DIR = str(COLLISION_MESHES_DIR)

# 跳过碰撞几何的连杆：base_link 是 1mm 坐标占位块；*_tip_link 是零体积虚拟点
_SKIP_LINKS = {"base_link"}
_SKIP_LINK_SUFFIXES = ("_tip_link",)

# 每个连杆抽稀目标面数（palm 网格非流形，实际达不到这么低，
# fast_simplification 会在可行范围内尽量逼近）
_DECIMATE_TARGET_FACES = {
    "Left_hand_palm_link": 400,
}
_DECIMATE_TARGET_DEFAULT = 120


def _parse_vec(text: Optional[str], default: Tuple[float, ...]) -> List[float]:
    if not text:
        return list(default)
    return [float(x) for x in text.split()]


def _rpy_to_quat_wxyz(r: float, p: float, y: float) -> Tuple[float, float, float, float]:
    """URDF rpy（外部 xyz 固定轴，先绕 x 再绕 y 再绕 z）转 MJCF 四元数 (w,x,y,z)。"""
    cr, sr = math.cos(r * 0.5), math.sin(r * 0.5)
    cp, sp = math.cos(p * 0.5), math.sin(p * 0.5)
    cy, sy = math.cos(y * 0.5), math.sin(y * 0.5)
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy
    return qw, qx, qy, qz


@dataclass
class _UrdfJoint:
    name: str
    jtype: str
    parent: str
    child: str
    xyz: List[float]
    rpy: List[float]
    axis: List[float]
    lower: Optional[float]
    upper: Optional[float]


@dataclass
class _UrdfLink:
    name: str
    mesh_filename: Optional[str]


def _parse_urdf(urdf_path: str) -> Tuple[Dict[str, _UrdfLink], Dict[str, _UrdfJoint], Dict[str, List[str]]]:
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    links: Dict[str, _UrdfLink] = {}
    for link_el in root.findall("link"):
        name = link_el.get("name")
        mesh_filename = None
        coll = link_el.find("collision")
        if coll is not None:
            mesh_el = coll.find("geometry/mesh")
            if mesh_el is not None:
                mesh_filename = mesh_el.get("filename")
        links[name] = _UrdfLink(name, mesh_filename)

    joints: Dict[str, _UrdfJoint] = {}
    children_of: Dict[str, List[str]] = {}
    for joint_el in root.findall("joint"):
        name = joint_el.get("name")
        jtype = joint_el.get("type")
        parent = joint_el.find("parent").get("link")
        child = joint_el.find("child").get("link")
        origin_el = joint_el.find("origin")
        xyz = _parse_vec(origin_el.get("xyz") if origin_el is not None else None, (0.0, 0.0, 0.0))
        rpy = _parse_vec(origin_el.get("rpy") if origin_el is not None else None, (0.0, 0.0, 0.0))
        axis_el = joint_el.find("axis")
        axis = _parse_vec(axis_el.get("xyz") if axis_el is not None else None, (0.0, 0.0, 1.0))
        limit_el = joint_el.find("limit")
        lower = float(limit_el.get("lower")) if limit_el is not None and limit_el.get("lower") is not None else None
        upper = float(limit_el.get("upper")) if limit_el is not None and limit_el.get("upper") is not None else None
        joints[name] = _UrdfJoint(name, jtype, parent, child, xyz, rpy, axis, lower, upper)
        children_of.setdefault(parent, []).append(name)

    return links, joints, children_of


def _should_skip_link(name: str) -> bool:
    if name in _SKIP_LINKS:
        return True
    return any(name.endswith(suf) for suf in _SKIP_LINK_SUFFIXES)


def _decimate_mesh(urdf_dir: str, mesh_rel_path: str, link_name: str, out_dir: str) -> Optional[str]:
    """抽稀网格并导出为 STL，返回相对 out_dir 的文件名。"""
    src = os.path.join(urdf_dir, mesh_rel_path)
    mesh = trimesh.load(src, force="mesh")
    target = _DECIMATE_TARGET_FACES.get(link_name, _DECIMATE_TARGET_DEFAULT)
    target = min(target, max(4, len(mesh.faces) - 1))
    try:
        simplified = mesh.simplify_quadric_decimation(face_count=target)
        if len(simplified.faces) < 4:
            raise ValueError("抽稀结果面数过少")
    except Exception as exc:
        print(f"  [警告] {link_name} 抽稀失败({exc})，改用凸包近似")
        simplified = mesh.convex_hull

    out_name = f"{link_name}.stl"
    out_path = os.path.join(out_dir, out_name)
    simplified.export(out_path)
    print(f"  {link_name}: {len(mesh.faces)} -> {len(simplified.faces)} 面")
    return out_name


def generate(urdf_path: str = _URDF_PATH, mjcf_out: str = _MJCF_OUT, mesh_out_dir: str = _MESH_OUT_DIR) -> str:
    urdf_dir = os.path.dirname(os.path.abspath(urdf_path))
    links, joints, children_of = _parse_urdf(urdf_path)
    os.makedirs(mesh_out_dir, exist_ok=True)

    # 找到根连杆（不是任何 joint 的 child）
    all_children = {j.child for j in joints.values()}
    roots = [name for name in links if name not in all_children]
    if len(roots) != 1:
        raise RuntimeError(f"期望恰好一个根连杆，实际找到: {roots}")
    root_link = roots[0]

    mesh_assets: List[Tuple[str, str]] = []  # (mesh_name, file)
    link_mesh_name: Dict[str, str] = {}

    print("正在抽稀碰撞网格 ...")
    for name, link in links.items():
        if _should_skip_link(name) or not link.mesh_filename:
            continue
        out_name = _decimate_mesh(urdf_dir, link.mesh_filename, name, mesh_out_dir)
        if out_name:
            mesh_name = f"{name}_collision"
            mesh_assets.append((mesh_name, out_name))
            link_mesh_name[name] = mesh_name

    lines: List[str] = []
    lines.append('<mujoco model="Left_hand_URDF_model_collision">')
    lines.append('  <compiler angle="radian" meshdir="collision_meshes"/>')
    lines.append("  <asset>")
    for mesh_name, file_name in mesh_assets:
        lines.append(f'    <mesh name="{mesh_name}" file="{file_name}"/>')
    lines.append("  </asset>")
    lines.append("  <worldbody>")

    def emit_body(link_name: str, indent: int, incoming_joint: Optional[_UrdfJoint]) -> None:
        pad = "  " * indent
        pos = incoming_joint.xyz if incoming_joint else [0.0, 0.0, 0.0]
        rpy = incoming_joint.rpy if incoming_joint else [0.0, 0.0, 0.0]
        qw, qx, qy, qz = _rpy_to_quat_wxyz(*rpy)
        lines.append(
            f'{pad}<body name="{link_name}" pos="{pos[0]:.8g} {pos[1]:.8g} {pos[2]:.8g}" '
            f'quat="{qw:.8g} {qx:.8g} {qy:.8g} {qz:.8g}">'
        )
        if incoming_joint is not None and incoming_joint.jtype == "revolute":
            lo = incoming_joint.lower if incoming_joint.lower is not None else -3.14159
            hi = incoming_joint.upper if incoming_joint.upper is not None else 3.14159
            ax = incoming_joint.axis
            lines.append(
                f'{pad}  <joint name="{incoming_joint.name}" type="hinge" '
                f'axis="{ax[0]:.8g} {ax[1]:.8g} {ax[2]:.8g}" range="{lo:.8g} {hi:.8g}"/>'
            )
        mesh_name = link_mesh_name.get(link_name)
        if mesh_name:
            lines.append(f'{pad}  <geom name="{link_name}_collision" type="mesh" mesh="{mesh_name}"/>')
        for child_joint_name in children_of.get(link_name, []):
            child_joint = joints[child_joint_name]
            if _should_skip_link(child_joint.child):
                continue
            emit_body(child_joint.child, indent + 1, child_joint)
        lines.append(f"{pad}</body>")

    emit_body(root_link, 2, None)

    lines.append("  </worldbody>")
    lines.append("</mujoco>")
    lines.append("")

    with open(mjcf_out, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))

    print(f"已生成: {mjcf_out}")
    print(f"碰撞网格目录: {mesh_out_dir}")
    return mjcf_out


if __name__ == "__main__":
    generate()
