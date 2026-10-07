#!/usr/bin/env python3
"""Render A3.0/A3.2 collision proxy overlays on the visual meshes for inspection."""
from pathlib import Path
import argparse
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
import numpy as np
import trimesh
import mujoco
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "src")]
from hand_profiles import A30_MODEL_IDS, A32_MODEL_IDS, load_hand_profile
from TsenseI_retargeting.fk.urdf import UrdfModel


def render(model_id="TsenseI_v3_2"):
    profile = load_hand_profile(model_id)
    fig, axes = plt.subplots(2, 3, figsize=(14, 11))
    colors = {"thumb": "#d97706", "index": "#2563eb", "mid": "#16a34a", "ring": "#9333ea", "pinky": "#dc2626"}
    geometry = yaml.safe_load((profile.root / "collision_geometry.yaml").read_text(encoding="utf-8"))
    for row, side in enumerate(("left", "right")):
        urdf = UrdfModel(profile.path("urdf", side))
        filt = profile.create_collision_filter(side)
        clouds = {}
        for link, visuals in urdf.visual_meshes_by_link().items():
            clouds[link] = []
            for v in visuals:
                mesh = trimesh.load_mesh(v.path)
                if len(mesh.faces) > 1000:
                    mesh = mesh.simplify_quadric_decimation(face_count=1000)
                vertices = mesh.vertices * v.scale
                vertices = vertices @ v.origin[:3, :3].T + v.origin[:3, 3]
                clouds[link].append((vertices, mesh.faces))
        front, lateral = ((1, 2), (0, 2)) if profile.mirror_axis == "y" else ((0, 2), (1, 2))
        for col, (name, dims) in enumerate((("Zero / palm", front), ("Zero / side", lateral), ("Flexion / side", lateral))):
            q = profile.init_pose(side)
            if col == 2:
                for joint, (lo, hi) in profile.joint_limits(side).items():
                    if any(part in joint for part in ("_PIP_", "_DIP_", "_MCP_")):
                        q[joint] = min(1.0, hi)
            poses = urdf.forward_kinematics(q)
            filt.sync_pose_from_dict(q)
            ax = axes[row, col]
            for link, parts in clouds.items():
                pose = poses[link]
                for vertices, faces in parts:
                    points = vertices @ pose[:3, :3].T + pose[:3, 3]
                    ax.add_collection(PolyCollection(points[faces][:, :, dims], facecolor="#cbd5e1", edgecolor="none", alpha=.22))
            for a, b, radius, finger in filt.get_capsule_render_data():
                circles = []
                angle = np.linspace(0, 2 * np.pi, 45)
                circle = np.column_stack((np.cos(angle), np.sin(angle))) * radius
                for t in np.linspace(0, 1, 15):
                    p = a * (1 - t) + b * t
                    circles.append(p[list(dims)] + circle)
                from scipy.spatial import ConvexHull
                points = np.concatenate(circles)
                hull = points[ConvexHull(points).vertices]
                ax.fill(hull[:, 0], hull[:, 1], facecolor=colors[finger], edgecolor=colors[finger], alpha=.24, linewidth=1)
            for spec in geometry["geoms"]:
                if spec["type"] != "mesh":
                    continue
                link = spec["link"].replace("left_hand", f"{side}_hand")
                hull = trimesh.load_mesh(profile.path("mjcf", side).parent / spec["mesh_files"][side], process=False)
                points = np.asarray(hull.vertices) @ poses[link][:3, :3].T + poses[link][:3, 3]
                from scipy.spatial import ConvexHull
                projected = points[:, dims]
                border = projected[ConvexHull(projected).vertices]
                ax.fill(border[:, 0], border[:, 1], facecolor=colors[spec["finger"]], edgecolor=colors[spec["finger"]], alpha=.28, linewidth=1)
            for gid in range(filt.model.ngeom):
                if filt.model.geom_type[gid] != mujoco.mjtGeom.mjGEOM_BOX:
                    continue
                import itertools
                corners = np.array(list(itertools.product((-1, 1), repeat=3))) * filt.model.geom_size[gid]
                corners = corners @ filt.data.geom_xmat[gid].reshape(3, 3).T + filt.data.geom_xpos[gid]
                from scipy.spatial import ConvexHull
                points = corners[:, dims]
                hull = points[ConvexHull(points).vertices]
                ax.fill(hull[:, 0], hull[:, 1], facecolor="#334155", edgecolor="#334155", alpha=.3)
            ax.autoscale_view()
            ax.set_aspect("equal")
            ax.set_title(f"{side.title()} · {name}\ncontacts: {filt.data.ncon}")
            ax.set_xlabel("XYZ"[dims[0]] + " / m")
            ax.set_ylabel("Z / m")
            ax.grid(alpha=.2)
    scope = "Inter-finger only (gray palm/shell excluded)" if model_id.endswith("_withid") else "Capsule / palm collision proxies"
    fig.suptitle(f"{model_id} — {scope}", fontsize=14)
    from matplotlib.patches import Patch
    fig.legend(handles=[Patch(color=c, alpha=.5, label=n.title()) for n,c in colors.items()], loc="lower center", ncol=5)
    fig.tight_layout()
    path = profile.root / "qa/collision_overlay.png"
    path.parent.mkdir(exist_ok=True)
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(path)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", choices=(*A32_MODEL_IDS, *A30_MODEL_IDS), default="TsenseI_v3_2")
    render(parser.parse_args().model)
