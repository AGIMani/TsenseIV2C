#!/usr/bin/env python3
"""从左手 TsenseI URDF 生成右手模型（关于 XZ 平面镜像，Y → -Y）。

输出：
  assets/hands/TsenseI_hand/urdf/TsenseI__hand_right_v1.urdf
  assets/hands/TsenseI_hand/meshes/right_*.STL / Right_*.STL
"""
from __future__ import annotations

import math
import re
import shutil
from pathlib import Path

import numpy as np
import trimesh

ROOT = Path(__file__).resolve().parents[1]
HAND = ROOT / "assets" / "hands" / "TsenseI_hand"
LEFT_URDF = HAND / "urdf" / "TsenseI__hand_left_v1.urdf"
RIGHT_URDF = HAND / "urdf" / "TsenseI__hand_right_v1.urdf"
MESH_DIR = HAND / "meshes"

# 镜像矩阵：关于 XZ 平面（左右手）
_M = np.diag([1.0, -1.0, 1.0])


def _rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """ROS URDF: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=float)
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=float)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=float)
    return rz @ ry @ rx


def _matrix_to_rpy(rm: np.ndarray) -> tuple[float, float, float]:
    """与 _rpy_to_matrix 互逆（ROS 固定轴 RPY）。"""
    sy = -rm[2, 0]
    cy = math.sqrt(max(0.0, 1.0 - sy * sy))
    if cy > 1e-8:
        pitch = math.atan2(sy, cy)
        roll = math.atan2(rm[2, 1], rm[2, 2])
        yaw = math.atan2(rm[1, 0], rm[0, 0])
    else:
        # 万向节锁
        pitch = math.copysign(math.pi / 2, sy)
        roll = math.atan2(-rm[0, 1], rm[1, 1])
        yaw = 0.0
    return roll, pitch, yaw


def _fmt_num(v: float) -> str:
    if abs(v) < 1e-15:
        return "0"
    s = f"{v:.15g}"
    return s


def _fmt_xyz(x: float, y: float, z: float) -> str:
    return f"{_fmt_num(x)} {_fmt_num(y)} {_fmt_num(z)}"


def _parse_xyz(s: str) -> tuple[float, float, float]:
    a, b, c = [float(x) for x in s.split()]
    return a, b, c


def _mirror_xyz(s: str) -> str:
    x, y, z = _parse_xyz(s)
    return _fmt_xyz(x, -y, z)


def _mirror_rpy(s: str) -> str:
    r, p, y = _parse_xyz(s)
    rm = _rpy_to_matrix(r, p, y)
    rm2 = _M @ rm @ _M
    r2, p2, y2 = _matrix_to_rpy(rm2)
    return _fmt_xyz(r2, p2, y2)


def _mirror_axis(s: str) -> str:
    """轴向向量在反射下：a' = det(M) M a（保持同号关节角对应镜像姿态）。

    M=diag(1,-1,1), det=-1 → a' = (-ax, ay, -az)。
    若只用 M@a，Y 轴关节会变成 (0,-1,0)，正角度弯曲方向反掉。
    """
    ax, ay, az = _parse_xyz(s)
    vx, vy, vz = -ax, ay, -az
    n = math.sqrt(vx * vx + vy * vy + vz * vz)
    if n > 1e-12:
        vx, vy, vz = vx / n, vy / n, vz / n
    return _fmt_xyz(vx, vy, vz)


def _mirror_inertia_attrs(ixx, ixy, ixz, iyy, iyz, izz) -> tuple[str, ...]:
    # I' = M I M → ixy、iyz 变号
    return (
        _fmt_num(float(ixx)),
        _fmt_num(-float(ixy)),
        _fmt_num(float(ixz)),
        _fmt_num(float(iyy)),
        _fmt_num(-float(iyz)),
        _fmt_num(float(izz)),
    )


def _rename_token(name: str) -> str:
    name = name.replace("Left_hand", "Right_hand")
    name = name.replace("left_hand", "right_hand")
    name = name.replace("Left_hand_URDF_model", "Right_hand_URDF_model")
    return name


def _mesh_right_name(filename: str) -> str:
    base = Path(filename).name
    if base.startswith("Left_"):
        return "Right_" + base[len("Left_") :]
    if base.startswith("left_"):
        return "right_" + base[len("left_") :]
    if base.lower().startswith("base_link"):
        return base  # base 可共用或镜像同名覆盖到 right 侧引用原文件
    return "right_" + base


def _mirror_stl(src: Path, dst: Path) -> None:
    mesh = trimesh.load_mesh(src, force="mesh")
    if isinstance(mesh, trimesh.Scene):
        mesh = trimesh.util.concatenate(tuple(mesh.geometry.values()))
    verts = np.asarray(mesh.vertices, dtype=np.float64)
    verts[:, 1] *= -1.0
    faces = np.asarray(mesh.faces)
    # 翻转绕序以保持法线朝外
    faces = faces[:, ::-1]
    out = trimesh.Trimesh(vertices=verts, faces=faces, process=False)
    dst.parent.mkdir(parents=True, exist_ok=True)
    out.export(dst)


def _transform_urdf_text(text: str) -> str:
    # robot name
    text = text.replace('name="Left_hand_URDF_model"', 'name="Right_hand_URDF_model"')

    def repl_origin(m: re.Match) -> str:
        xyz, rpy = m.group(1), m.group(2)
        return f'<origin\n        xyz="{_mirror_xyz(xyz)}"\n        rpy="{_mirror_rpy(rpy)}" />'

    text = re.sub(
        r'<origin\s+xyz="([^"]+)"\s+rpy="([^"]+)"\s*/>',
        repl_origin,
        text,
    )

    def repl_axis(m: re.Match) -> str:
        return f'<axis\n      xyz="{_mirror_axis(m.group(1))}" />'

    text = re.sub(r'<axis\s+xyz="([^"]+)"\s*/>', repl_axis, text)

    def repl_inertia(m: re.Match) -> str:
        ixx, ixy, ixz, iyy, iyz, izz = _mirror_inertia_attrs(
            m.group(1), m.group(2), m.group(3), m.group(4), m.group(5), m.group(6)
        )
        return (
            f'<inertia\n        ixx="{ixx}"\n        ixy="{ixy}"\n        ixz="{ixz}"\n'
            f'        iyy="{iyy}"\n        iyz="{iyz}"\n        izz="{izz}" />'
        )

    text = re.sub(
        r'<inertia\s+ixx="([^"]+)"\s+ixy="([^"]+)"\s+ixz="([^"]+)"\s+'
        r'iyy="([^"]+)"\s+iyz="([^"]+)"\s+izz="([^"]+)"\s*/>',
        repl_inertia,
        text,
        flags=re.S,
    )

    def repl_mesh(m: re.Match) -> str:
        path = m.group(1)
        name = Path(path).name
        right = _mesh_right_name(name)
        # 保持相对路径 ../meshes/
        prefix = str(Path(path).parent).replace("\\", "/")
        if prefix in (".", ""):
            new_path = right
        else:
            new_path = f"{prefix}/{right}"
        return f'filename="{new_path}"'

    text = re.sub(r'filename="([^"]+)"', repl_mesh, text)

    # 链接/关节名
    text = text.replace("Left_hand", "Right_hand")
    text = text.replace("left_hand", "right_hand")

    # 右手 thumb_rota 限位与左手一致 [0, 1.57]；下发亦同号（正），不再对右手取负
    text = re.sub(
        r'(name="right_hand_thumb_rota_joint"[\s\S]*?<limit\s+lower=")[^"]+("\s+upper=")[^"]+',
        r"\g<1>0\g<2>1.57",
        text,
        count=1,
    )
    return text


def main() -> None:
    if not LEFT_URDF.is_file():
        raise SystemExit(f"missing left URDF: {LEFT_URDF}")

    left_text = LEFT_URDF.read_text(encoding="utf-8")
    # 收集左手 mesh 文件名
    mesh_names = sorted(set(re.findall(r'filename="[^"]*/([^/"]+\.STL)"', left_text, flags=re.I)))
    print(f"meshes to mirror: {len(mesh_names)}")

    mirrored = 0
    for name in mesh_names:
        src = MESH_DIR / name
        dst = MESH_DIR / _mesh_right_name(name)
        if not src.is_file():
            print(f"  WARN missing {src}")
            continue
        if src.resolve() == dst.resolve():
            # base_link 等同名：写到临时再覆盖不合适，生成 right_base_link 若需要
            dst = MESH_DIR / f"right_{name}"
            # 更新将在 urdf 里对 base 特殊处理
        _mirror_stl(src, dst)
        mirrored += 1
        print(f"  {name} -> {dst.name}")

    right_text = _transform_urdf_text(left_text)
    # base_link 网格：若生成了 right_base_link.STL 则替换引用
    if (MESH_DIR / "right_base_link.STL").is_file():
        right_text = right_text.replace(
            'filename="../meshes/base_link.STL"',
            'filename="../meshes/right_base_link.STL"',
        )
    elif (MESH_DIR / "Right_base_link.STL").is_file():
        right_text = right_text.replace(
            'filename="../meshes/base_link.STL"',
            'filename="../meshes/Right_base_link.STL"',
        )

    RIGHT_URDF.parent.mkdir(parents=True, exist_ok=True)
    RIGHT_URDF.write_text(right_text, encoding="utf-8")
    print(f"wrote {RIGHT_URDF}")
    print(f"mirrored meshes: {mirrored}")


if __name__ == "__main__":
    main()
