#!/usr/bin/env python3
"""Render the robot-side MediaPipe 21 keypoint mapping on the URDF mesh."""

from __future__ import annotations

import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from matplotlib.lines import Line2D
import numpy as np
import trimesh
import yaml


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from TsenseI_retargeting import TopoRetargetConfig, TopoRetargeter  # noqa: E402


ROBOT_ROOT = ROOT / "assets" / "hands" / "TsenseI_hand"
URDF_PATH = ROBOT_ROOT / "urdf" / "TsenseI__hand_left_v1.urdf"
KEYPOINT_PATH = ROBOT_ROOT / "robot_keypoints_21.yaml"
OUTPUT_PATH = ROOT / "tools" / "robot_keypoints_21_visualization.png"

FINGER_COLORS = {
    "palm": "#263238",
    "thumb": "#f59e0b",
    "index": "#2563eb",
    "middle": "#16a34a",
    "ring": "#9333ea",
    "little": "#dc2626",
}

EDGES = (
    (0, 1), (1, 2), (2, 3), (3, 4),
    (0, 5), (5, 6), (6, 7), (7, 8),
    (0, 9), (9, 10), (10, 11), (11, 12),
    (0, 13), (13, 14), (14, 15), (15, 16),
    (0, 17), (17, 18), (18, 19), (19, 20),
)


def _xyz(text: str | None, default: tuple[float, float, float]) -> np.ndarray:
    if not text:
        return np.asarray(default, dtype=np.float64)
    return np.asarray([float(item) for item in text.split()], dtype=np.float64)


def _load_keypoint_rows(path: Path = KEYPOINT_PATH) -> list[dict]:
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    return sorted(data["keypoints"], key=lambda item: int(item["id"]))


def _load_zero_pose_geometry(
    solver: TopoRetargeter,
    *, wrist_relative: bool = True,
) -> tuple[np.ndarray, list[np.ndarray]]:
    kin = solver._analytic_kinematics
    if kin is None:
        raise RuntimeError(f"Pinocchio backend unavailable: {solver._analytic_kinematics_error}")

    q = np.zeros(len(solver.joint_names), dtype=np.float64)
    keypoints = kin.compute_points(q)
    wrist = keypoints[0].copy() if wrist_relative else np.zeros(3)
    keypoints -= wrist

    urdf_path = Path(solver.urdf_path)
    root = ET.parse(urdf_path).getroot()
    transformed_meshes: list[np.ndarray] = []
    for link in root.findall("link"):
        link_name = str(link.get("name"))
        frame_id = int(kin.model.getFrameId(link_name))
        if frame_id >= kin.model.nframes:
            continue
        placement = kin.data.oMf[frame_id]

        for visual in link.findall("visual"):
            mesh_node = visual.find("geometry/mesh")
            if mesh_node is None or not mesh_node.get("filename"):
                continue
            mesh_path = solver.fk.model.resolve_mesh_path(str(mesh_node.get("filename")))
            mesh = trimesh.load_mesh(mesh_path, process=True)
            if isinstance(mesh, trimesh.Scene):
                mesh = mesh.to_geometry()

            # The original CAD export is close to 480k triangles.  Simplifying
            # is only for rasterization; FK/keypoint coordinates stay exact.
            face_target = 9000 if "palm" in link_name.lower() else 1800
            if len(mesh.faces) > face_target:
                mesh = mesh.simplify_quadric_decimation(face_count=face_target)

            scale = _xyz(mesh_node.get("scale"), (1.0, 1.0, 1.0))
            vertices = np.asarray(mesh.vertices, dtype=np.float64) * scale

            origin = visual.find("origin")
            origin_xyz = _xyz(origin.get("xyz") if origin is not None else None, (0.0, 0.0, 0.0))
            origin_rpy = _xyz(origin.get("rpy") if origin is not None else None, (0.0, 0.0, 0.0))
            visual_rotation = kin.pin.rpy.rpyToMatrix(origin_rpy)
            vertices = vertices @ visual_rotation.T + origin_xyz
            vertices = vertices @ placement.rotation.T + placement.translation - wrist
            transformed_meshes.append(vertices[np.asarray(mesh.faces, dtype=np.intp)])

    return keypoints, transformed_meshes


def _setup_axis(ax, *, title: str, xlabel: str, ylabel: str) -> None:
    ax.set_title(title, fontsize=14, fontweight="bold", pad=12)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, color="#cbd5e1", linewidth=0.7, alpha=0.6)
    ax.set_facecolor("#f8fafc")
    for spine in ax.spines.values():
        spine.set_color("#94a3b8")


def _draw_projected_mesh(ax, meshes: list[np.ndarray], dims: tuple[int, int]) -> None:
    for triangles in meshes:
        projected = triangles[:, :, dims]
        collection = PolyCollection(
            projected,
            facecolors="#dce5ef",
            edgecolors="none",
            alpha=0.34,
            rasterized=True,
        )
        ax.add_collection(collection)


def _draw_keypoints(
    ax,
    points: np.ndarray,
    rows: list[dict],
    *,
    dims: tuple[int, int],
    labels: bool,
) -> None:
    for a, b in EDGES:
        finger = str(rows[b]["finger"])
        ax.plot(
            points[[a, b], dims[0]],
            points[[a, b], dims[1]],
            color=FINGER_COLORS[finger],
            linewidth=2.4,
            alpha=0.86,
            zorder=5,
        )

    for row, point in zip(rows, points, strict=True):
        finger = str(row["finger"])
        color = FINGER_COLORS[finger]
        x, y = point[list(dims)]
        ax.scatter(
            [x], [y], s=92, c=[color], edgecolors="white", linewidths=1.5,
            zorder=7,
        )
        ax.text(
            x, y, str(row["id"]), ha="center", va="center", color="white",
            fontsize=7.5, fontweight="bold", zorder=8,
        )
        if labels:
            offset_x = 6 if float(x) >= 0.0 else -6
            ax.annotate(
                str(row["source_name"]),
                (x, y),
                xytext=(offset_x, 7),
                textcoords="offset points",
                ha="left" if offset_x > 0 else "right",
                fontsize=7.5,
                color="#0f172a",
                bbox={"boxstyle": "round,pad=0.15", "fc": "white", "ec": "none", "alpha": 0.72},
                zorder=9,
            )


def render(output_path: Path = OUTPUT_PATH, *, config: TopoRetargetConfig | None = None) -> Path:
    solver = TopoRetargeter(config or TopoRetargetConfig(robot_root=ROBOT_ROOT))
    rows = _load_keypoint_rows(Path(solver.keypoint_path))
    points, meshes = _load_zero_pose_geometry(solver)
    width_axis = int(np.argmax(np.ptp(points[[5,9,13,17], :2], axis=0)))
    front_dims, side_dims = (width_axis, 2), (1-width_axis, 2)

    fig = plt.figure(figsize=(19.2, 10.8), dpi=150, facecolor="white")
    grid = fig.add_gridspec(2, 3, width_ratios=(1.35, 1.0, 0.9), height_ratios=(1.0, 1.0))
    front = fig.add_subplot(grid[:, 0])
    side = fig.add_subplot(grid[0, 1])
    legend_ax = fig.add_subplot(grid[1, 1:])
    note_ax = fig.add_subplot(grid[0, 2])

    _draw_projected_mesh(front, meshes, front_dims)
    _draw_keypoints(front, points, rows, dims=front_dims, labels=True)
    _setup_axis(front, title="Palm view", xlabel=f"Robot {'XY'[width_axis]} / m", ylabel="Robot Z / m")

    _draw_projected_mesh(side, meshes, side_dims)
    _draw_keypoints(side, points, rows, dims=side_dims, labels=False)
    _setup_axis(side, title="Side view", xlabel=f"Robot {'XY'[1-width_axis]} / m", ylabel="Robot Z / m")

    all_vertices = np.concatenate([mesh.reshape(-1, 3) for mesh in meshes] + [points], axis=0)
    for ax, dims in ((front, front_dims), (side, side_dims)):
        xy = all_vertices[:, list(dims)]
        lower = xy.min(axis=0)
        upper = xy.max(axis=0)
        margin = np.maximum((upper - lower) * 0.08, 0.008)
        ax.set_xlim(lower[0] - margin[0], upper[0] + margin[0])
        ax.set_ylim(lower[1] - margin[1], upper[1] + margin[1])

    note_ax.axis("off")
    note_ax.text(
        0.0,
        1.0,
        "Robot-side MediaPipe mapping\nURDF zero pose",
        va="top",
        fontsize=19,
        fontweight="bold",
        color="#0f172a",
    )
    note_ax.text(
        0.0,
        0.77,
        "Colored dots are the exact robot points used by retarget.\n"
        "Numbers follow MediaPipe Hands landmark IDs (0-20).\n"
        "Gray surface is the project URDF visual mesh.\n"
        "Point positions use Pinocchio FK at q = 0 rad.",
        va="top",
        fontsize=11,
        linespacing=1.6,
        color="#334155",
    )
    handles = [
        Line2D([0], [0], marker="o", color="none", markerfacecolor=color,
               markeredgecolor="white", markersize=10, label=finger.title())
        for finger, color in FINGER_COLORS.items()
    ]
    note_ax.legend(handles=handles, loc="lower left", frameon=False, fontsize=10)

    legend_ax.axis("off")
    columns = []
    for start in (0, 7, 14):
        group = rows[start:start + 7]
        columns.append("\n".join(f"{int(item['id']):>2}  {item['source_name']}" for item in group))
    for x, text in zip((0.02, 0.35, 0.68), columns, strict=True):
        legend_ax.text(
            x, 0.96, text, va="top", family="monospace", fontsize=11,
            linespacing=1.55, color="#172033",
        )
    legend_ax.text(
        0.02, 0.05,
        f"Source: {solver.keypoint_path}",
        fontsize=9, color="#64748b",
    )

    fig.suptitle(
        "TsenseI Hand: MediaPipe 21 Robot Keypoint Locations",
        fontsize=23,
        fontweight="bold",
        y=0.985,
        color="#0f172a",
    )
    fig.subplots_adjust(left=0.045, right=0.98, top=0.92, bottom=0.06, wspace=0.25, hspace=0.22)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, bbox_inches="tight")
    plt.close(fig)
    return output_path


if __name__ == "__main__":
    import argparse
    sys.path.insert(0, str(ROOT / "src"))
    from hand_profiles import PROFILE_PATHS, load_hand_profile
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", choices=PROFILE_PATHS, default="TsenseI_v2")
    parser.add_argument("--hand-side", choices=("left", "right"), default="left")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    profile = load_hand_profile(args.model)
    solver = profile.create_retargeter(args.hand_side)
    result = render(args.output or profile.root / f"keypoints_{args.hand_side}.png", config=solver.config)
    print(result)
