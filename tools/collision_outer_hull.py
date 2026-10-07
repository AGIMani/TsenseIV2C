"""Conservative, low-complexity collision meshes generated from URDF visuals.

All geometry is expressed in the link's local frame in metres. A bounded
intersection of supporting halfspaces encloses the source convex hull. Unlike
triangle decimation it cannot cut through the source shell. The error bound is
Euclidean excess over the full convex hull; concave cavities remain filled.
"""
from __future__ import annotations

import hashlib
import io
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial import ConvexHull, HalfspaceIntersection
import trimesh

VERSION = "supporting_outer_hull_v1"
ROUNDTRIP_PADDING = 2e-7  # 0.2 micrometre; protects vertex grid + STL rounding.
VERTEX_GRID = 1e-7  # Avoid sub-10-nm duplicate corners/skinny triangles in STL.
DEFAULT_ERROR = 0.00035


def convex_mesh(points: np.ndarray) -> tuple[ConvexHull, trimesh.Trimesh]:
    hull = ConvexHull(points)
    faces = hull.simplices.copy()
    normal = np.cross(points[faces[:, 1]] - points[faces[:, 0]],
                      points[faces[:, 2]] - points[faces[:, 0]])
    flip = np.einsum("ij,ij->i", normal, hull.equations[:, :3]) < 0
    faces[flip] = faces[flip, ::-1]
    mesh = trimesh.Trimesh(vertices=points, faces=faces, process=False)
    mesh.remove_unreferenced_vertices()
    return hull, mesh


def max_excess(mesh: trimesh.Trimesh, reference: trimesh.Trimesh) -> float:
    # Distance to a convex set is convex; the maximum over a polytope occurs
    # at a vertex. Checking these vertices bounds its whole surface/volume.
    # trimesh triangle predicates use absolute tolerances. Evaluate in mm to
    # avoid incorrectly classifying the tiny CAD facets as degenerate in metres.
    scaled_reference = reference.copy()
    scaled_reference.vertices = scaled_reference.vertices * 1000
    return max(float(trimesh.proximity.closest_point_naive(scaled_reference, vertices * 1000)[1].max())
               for vertices in np.array_split(mesh.vertices, max(1, len(mesh.vertices) // 12))) / 1000


def containment_violation(points: np.ndarray, equations: np.ndarray) -> float:
    return max(float((chunk @ equations[:, :3].T + equations[:, 3]).max())
               for chunk in np.array_split(points, max(1, len(points) // 2048)))


def source_points(model, link: str) -> np.ndarray:
    clouds = []
    for visual in model.visual_meshes_by_link().get(link, []):
        mesh = trimesh.load_mesh(visual.path, process=False)
        points = np.asarray(mesh.vertices) * visual.scale
        clouds.append(points @ visual.origin[:3, :3].T + visual.origin[:3, 3])
    if not clouds:
        raise ValueError(f"No visual mesh for collision link {link}")
    return np.unique(np.concatenate(clouds), axis=0)


def outer_envelope(points: np.ndarray, *, max_error: float = DEFAULT_ERROR,
                   max_planes: int = 256, fit_mirror=None) -> tuple[bytes, dict]:
    """Return binary STL and metrics, with containment checked after roundtrip."""
    points = np.unique(np.asarray(points, dtype=np.float64), axis=0)
    original_points = points
    _, original_hull = convex_mesh(original_points)
    mirror = np.ones(3) if fit_mirror is None else np.asarray(fit_mirror)
    # Use a shared fitting frame for mirrored CAD. A picometre quantization
    # removes transform roundoff before deterministic support-plane selection;
    # the final containment check still uses unrounded original visual points.
    points = np.unique(np.round(points * mirror, 12), axis=0)
    full_hull, full = convex_mesh(points)
    center = full.vertices.mean(axis=0)
    equations = np.unique(np.round(full_hull.equations, 13), axis=0)
    planes = np.column_stack((np.r_[np.eye(3), -np.eye(3)],
                              -np.r_[points.max(axis=0), -points.min(axis=0)]))
    planes[:, 3] -= ROUNDTRIP_PADDING
    best = None
    selected: set[int] = set()
    # Tiny polyhedra need no simplification. For detailed CAD, stop as soon as
    # the measured distance tolerance is met, rather than imposing a budget
    # that might silently lose containment or accuracy.
    if len(full.vertices) > 64:
        for count in range(6, max_planes + 1):
            vertices = HalfspaceIntersection(planes, center).intersections
            if count >= 18 and (count % 6 == 0 or count == max_planes):
                _, candidate = convex_mesh(vertices)
                if len(candidate.vertices) >= len(full.vertices):
                    break
                if max_excess(candidate, full) <= max_error - 2 * ROUNDTRIP_PADDING:
                    best = candidate
                    break
            violation = vertices @ equations[:, :3].T + equations[:, 3]
            scores = violation.max(axis=0)
            if selected:
                scores[list(selected)] = -np.inf
            index = int(np.argmax(scores))
            if scores[index] <= ROUNDTRIP_PADDING or count == max_planes:
                break
            selected.add(index)
            plane = equations[index].copy()
            plane[3] -= ROUNDTRIP_PADDING
            planes = np.vstack((planes, plane))
    method = "supporting_halfspaces"
    if best is None:
        # Full hull fallback is still inexpensive for coarse authored parts.
        # Homothetic expansion gives a precise conservative float32 envelope.
        distance = -(full_hull.equations[:, :3] @ center + full_hull.equations[:, 3])
        scale = 1 + ROUNDTRIP_PADDING / distance.min()
        _, best = convex_mesh(center + (full.vertices - center) * scale)
        method = "full_hull"
    # Float32 serialization can split nominally coplanar vertices and create
    # skinny faces. Build the final hull from the actual serialized vertices;
    # a 100-nm grid first prevents downstream STL welders removing tiny edges.
    vertices = np.round(best.vertices * mirror / VERTEX_GRID) * VERTEX_GRID
    vertices = np.unique(vertices.astype(np.float32).astype(np.float64), axis=0)
    _, best = convex_mesh(vertices)
    raw = best.export(file_type="stl")
    rounded = trimesh.load_mesh(io.BytesIO(raw), file_type="stl", process=False)
    round_hull, round_mesh = convex_mesh(np.unique(rounded.vertices, axis=0))
    violation = containment_violation(original_points, round_hull.equations)
    error = max_excess(round_mesh, original_hull)
    if violation > 1e-10:
        raise ValueError(f"STL roundtrip failed containment: {violation:g} m")
    if error > max_error:
        raise ValueError(f"STL roundtrip error {error:g} exceeds {max_error:g} m")
    metrics = {"method": method, "source_vertices": len(original_points),
               "full_hull_vertices": len(original_hull.vertices), "full_hull_faces": len(original_hull.faces),
               "vertices": len(np.unique(rounded.vertices, axis=0)), "faces": len(rounded.faces),
               "convex_vertices": len(round_mesh.vertices), "convex_faces": len(round_mesh.faces),
               "max_excess_mm": error * 1000,
               "max_source_violation_mm": violation * 1000,
               "source_contained": True, "allowed_excess_mm": max_error * 1000,
               "vertex_grid_m": VERTEX_GRID,
               "stl_sha256": hashlib.sha256(raw).hexdigest()}
    return raw, metrics


def update_urdf_collisions(path: Path, mesh_files: dict[str, str]) -> None:
    """Change collision elements only; retain the user's exact joint/visual XML."""
    text = path.read_text(encoding="utf-8")
    def link_block(match):
        block = match.group(0)
        name = ET.fromstring(block).get("name")
        if name not in mesh_files:
            return block
        collision = (f'<collision name="{name}_outer_hull">\n'
                     f'      <origin xyz="0 0 0" rpy="0 0 0" />\n'
                     f'      <geometry>\n'
                     f'        <mesh filename="{mesh_files[name]}" />\n'
                     f'      </geometry>\n'
                     f'    </collision>')
        count = 0
        def replace(_match):
            nonlocal count
            count += 1
            return _match.group("indent") + collision if count == 1 else ""
        return re.sub(r"(?P<indent>[ \t]*)<collision\b[^>]*>.*?</collision>", replace, block, flags=re.S)
    updated = re.sub(r"<link\b[^>]*(?<!/)>.*?</link>", link_block, text, flags=re.S)
    if updated != text:
        path.write_text(updated, encoding="utf-8")


def upgrade_geometry(profile, geometry: dict, *, max_error: float = DEFAULT_ERROR,
                     force: bool = False) -> dict:
    """Regenerate all canonical collision links and the active runtime subset."""
    from TsenseI_retargeting.fk.urdf import UrdfModel
    report_path = profile.root / "collision_mesh_report.json"
    previous = json.loads(report_path.read_text()) if report_path.exists() else {}
    previous_by_key = {(row["side"], row["link"]): row for row in previous.get("links", [])}
    rows = []
    for side in ("left", "right"):
        model = UrdfModel(profile.path("urdf", side))
        fit_mirror = np.ones(3)
        if side == "right":
            fit_mirror[0 if profile.mirror_axis == "x" else 1] = -1
        collision_links = {link.get("name") for link in model.root.findall("link")
                           if link.findall("collision")}
        mesh_files = {}
        for link in model.links:
            if link not in collision_links:
                continue
            points = source_points(model, link)
            source_sha = hashlib.sha256(points.tobytes()).hexdigest()
            # Side prefix also prevents shared base_link meshes from overwriting
            # each other when the URDF applies different visual frames/scales.
            filename = f"{side}_{link}.stl"
            target = profile.root / "mjcf/collision_meshes" / filename
            previous_row = previous_by_key.get((side, link), {})
            if (not force and previous.get("version") == VERSION
                    and previous_row.get("source_sha256") == source_sha
                    and previous_row.get("allowed_excess_mm") == max_error * 1000
                    and previous_row.get("fit_mirror") == fit_mirror.tolist()
                    and previous_row.get("vertex_grid_m") == VERTEX_GRID
                    and target.exists()
                    and hashlib.sha256(target.read_bytes()).hexdigest() == previous_row.get("stl_sha256")):
                row = previous_row
            else:
                raw, metrics = outer_envelope(points, max_error=max_error, fit_mirror=fit_mirror)
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_bytes(raw)
                row = {"side": side, "link": link, "source_sha256": source_sha,
                       "fit_mirror": fit_mirror.tolist(),
                       "mesh": str(target.relative_to(profile.root)), **metrics}
            rows.append(row)
            mesh_files[link] = "../" + row["mesh"]
            print(f"{profile.model_id} {side} {link}: {row['full_hull_vertices']} -> "
                  f"{row['vertices']} vertices, excess {row['max_excess_mm']:.3f} mm", flush=True)
        update_urdf_collisions(model.urdf_path, mesh_files)
    by_key = {(row["side"], row["link"]): row for row in rows}
    for spec in geometry["geoms"]:
        if not spec.get("finger") and "palm" not in spec["link"].lower():
            continue
        link = spec["link"]
        spec["type"] = "mesh"
        for key in ("size", "fromto", "pos", "hull_vertices", "hull_faces"):
            spec.pop(key, None)
        spec["name"] = link + "_collision"
        spec["region"] = ("palm" if "palm" in link.lower()
                          else "distal" if link.endswith("DIP_link") or "thumb_PIP" in link
                          else "intermediate" if link.endswith("PIP_link") or "thumb_MCP" in link
                          else "proximal")
        spec["fit"] = VERSION
        spec["mesh_files"] = {}
        for side in ("left", "right"):
            side_link = link.replace("left_hand", f"{side}_hand").replace("Left_hand", f"{side.title()}_hand")
            row = by_key[side, side_link]
            spec["mesh_files"][side] = str(Path(row["mesh"]).relative_to("mjcf"))
    geometry.update(schema_version=2, proxy_version=VERSION, quality_status="verified_outer_envelopes",
                    max_excess_mm=max_error * 1000,
                    source="canonical URDF visuals with per-side mesh scale and visual origin applied",
                    notes=["All active finger bodies use one conservative low-complexity convex outer envelope.",
                           "Each side is fitted to its own transformed URDF visual vertices; binary STL containment is verified.",
                           "Error is Euclidean excess over the full convex hull; concave cavities remain filled.",
                           "Existing collision pair masks are preserved; enabled palms use the full visual-shell envelope."])
    report_path.write_text(json.dumps({"version": VERSION, "model_id": profile.model_id,
                                      "unit": "meter", "links": rows}, indent=2) + "\n")
    return geometry
