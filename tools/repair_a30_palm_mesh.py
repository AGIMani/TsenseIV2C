"""Conservatively recover triangular/quadrilateral holes in the damaged A3 palm.

Only existing vertices and boundary edges are used. This is a reconstruction,
not a substitute for a full-resolution CAD re-export.
"""
from __future__ import annotations

import argparse
from itertools import combinations
from pathlib import Path

import numpy as np
from scipy.sparse import coo_matrix, triu
import trimesh


def recover_loops(mesh, max_size=8):
    """Trace small, consistently oriented boundary loops sharing CAD vertices."""
    import mapbox_earcut
    from shapely.geometry import Polygon
    vertices = np.asarray(mesh.vertices)
    normals = np.asarray(mesh.face_normals)
    edges, first, counts = np.unique(mesh.edges_sorted, axis=0, return_index=True, return_counts=True)
    ids = first[counts == 1]
    directed = np.asarray(mesh.edges)[ids][:, ::-1]
    fids = np.asarray(mesh.edges_face)[ids]
    lookup = {tuple(e): int(f) for e, f in zip(directed, fids)}
    all_edges = set(map(tuple, edges))
    outgoing = {}
    for x, y in directed:
        outgoing.setdefault(x, []).append(y)
    loops = []
    for start in sorted(outgoing):
        pending = [([start], [])]
        visits = 0
        while pending and visits < 400:
            path, neighbors = pending.pop()
            visits += 1
            choices = outgoing.get(path[-1], [])
            if len(choices) > 6:
                continue
            for end in choices:
                fid = lookup[(path[-1], end)]
                if neighbors and np.min(normals[neighbors] @ normals[fid]) < -.05:
                    continue
                if end == start:
                    if len(path) >= 3:
                        loops.append((tuple(path), neighbors + [fid]))
                elif end > start and end not in path and len(path) < max_size:
                    pending.append((path + [end], neighbors + [fid]))
    print('Small oriented loops:', len(loops), flush=True)
    candidates = []
    for loop, neighbor_ids in loops:
        vv = vertices[list(loop)]
        nn = normals[neighbor_ids].sum(0)
        if np.linalg.norm(nn) < 1e-10:
            continue
        nn /= np.linalg.norm(nn)
        axis = np.eye(3)[np.argmin(np.abs(nn))]
        right = np.cross(axis, nn); right /= np.linalg.norm(right)
        up = np.cross(nn, right)
        xy = np.column_stack((vv @ right, vv @ up))
        polygon = Polygon(xy)
        if not polygon.is_valid or polygon.area < 1e-15:
            continue
        indices = mapbox_earcut.triangulate_float64(xy, np.array([len(loop)], dtype=np.uint32)).reshape(-1, 3)
        if len(indices) != len(loop) - 2:
            continue
        triangles = np.asarray(loop)[indices]
        tv = vertices[triangles]
        fn = np.cross(tv[:, 1]-tv[:, 0], tv[:, 2]-tv[:, 0])
        length = np.linalg.norm(fn, axis=1)
        if np.min(length) < 1e-15:
            continue
        fn /= length[:, None]
        if np.mean(fn @ nn) < 0:
            triangles = triangles[:, ::-1]; fn = -fn
        score = float(np.min(fn @ normals[neighbor_ids].T))
        if score < -.01:
            continue
        perimeter = {tuple(sorted((loop[i], loop[(i+1) % len(loop)]))) for i in range(len(loop))}
        new_edges = {tuple(sorted((t[i], t[(i+1) % 3]))) for t in triangles for i in range(3)}
        if any(e in all_edges for e in new_edges - perimeter):
            continue
        candidates.append((len(loop), -score, new_edges, triangles))
    used = set(); additions = []
    for _, _, new_edges, triangles in sorted(candidates, key=lambda x: (x[0], x[1])):
        if used.intersection(new_edges):
            continue
        used.update(new_edges); additions.extend(triangles)
    if additions:
        mesh.faces = np.concatenate((mesh.faces, additions))
    return len(additions)


def recover(mesh: trimesh.Trimesh, size: int) -> int:
    vertices = np.asarray(mesh.vertices)
    normals = np.asarray(mesh.face_normals)
    edges, first, counts = np.unique(mesh.edges_sorted, axis=0,
                                     return_index=True, return_counts=True)
    boundary = edges[counts == 1]
    indices = first[counts == 1]
    directions = np.asarray(mesh.edges)[indices]
    face_ids = np.asarray(mesh.edges_face)[indices]
    lookup = {tuple(e): (int(f), tuple(d))
              for e, f, d in zip(boundary, face_ids, directions)}
    all_edges = set(map(tuple, edges))
    old_faces = set(map(tuple, np.sort(mesh.faces, axis=1)))
    adj = coo_matrix((np.ones(len(boundary) * 2, dtype=np.int32),
                      (np.r_[boundary[:, 0], boundary[:, 1]],
                       np.r_[boundary[:, 1], boundary[:, 0]])),
                     shape=(len(vertices), len(vertices))).tocsr()

    def neighbors(v):
        return adj.indices[adj.indptr[v]:adj.indptr[v + 1]]

    candidates = []
    if size == 3:
        for x, y in boundary:
            common = np.intersect1d(neighbors(x), neighbors(y), assume_unique=True)
            for z in common[common > y]:
                if (x, y, z) not in old_faces:
                    candidates.append((x, y, z))
    elif size == 4:
        paths = triu(adj @ adj, k=1).tocoo()
        seen = set()
        for x, z in zip(paths.row[paths.data >= 2], paths.col[paths.data >= 2]):
            if (x, z) in all_edges:
                continue
            common = np.intersect1d(neighbors(x), neighbors(z), assume_unique=True)
            for y, w in combinations(common, 2):
                if tuple(sorted((y, w))) in all_edges:
                    continue
                key = tuple(sorted((x, y, z, w)))
                if key not in seen:
                    seen.add(key)
                    candidates.append((x, y, z, w))
    else:
        raise ValueError(size)

    accepted = []
    for loop in candidates:
        directed = [(loop[i], loop[(i + 1) % size]) for i in range(size)]
        keys = [tuple(sorted(e)) for e in directed]
        adjacent = [lookup[e] for e in keys]
        votes = [e == d for e, (_, d) in zip(directed, adjacent)]
        if any(votes) and not all(votes):
            continue
        if all(votes):
            loop = tuple(reversed(loop))
        options = [[(loop[0], loop[1], loop[2])]] if size == 3 else [
            [(loop[0], loop[1], loop[2]), (loop[0], loop[2], loop[3])],
            [(loop[0], loop[1], loop[3]), (loop[1], loop[2], loop[3])],
        ]
        best = None
        for triangles in options:
            tv = vertices[np.asarray(triangles)]
            nn = np.cross(tv[:, 1] - tv[:, 0], tv[:, 2] - tv[:, 0])
            length = np.linalg.norm(nn, axis=1)
            if np.min(length) < 1e-15:
                continue
            nn /= length[:, None]
            score = float(np.min(nn @ normals[[i[0] for i in adjacent]].T))
            # Do not span sharply opposed surfaces or fold a quad across itself.
            if score < -0.01 or (size == 4 and np.dot(nn[0], nn[1]) < 0.5):
                continue
            if best is None or score > best[0]:
                best = score, triangles
        if best is not None:
            accepted.append((best[0], keys, best[1]))
    used = set()
    faces = []
    for _, keys, triangles in sorted(accepted, key=lambda item: item[0], reverse=True):
        new_edges = {tuple(sorted((t[i], t[(i+1) % 3]))) for t in triangles for i in range(3)}
        if any(e in used for e in new_edges):
            continue
        used.update(new_edges)
        faces.extend(triangles)
    if faces:
        mesh.faces = np.concatenate((mesh.faces, np.asarray(faces)))
    return len(faces)


def stats(mesh):
    _, counts = np.unique(mesh.edges_sorted, axis=0, return_counts=True)
    return dict(faces=len(mesh.faces), boundary_edges=int((counts == 1).sum()),
                nonmanifold_edges=int((counts > 2).sum()),
                winding_consistent=bool(mesh.is_winding_consistent))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--quads-only', action='store_true')
    parser.add_argument('--loops-only', action='store_true')
    parser.add_argument('--max-loop', type=int, default=8)
    args = parser.parse_args()
    mesh = trimesh.load_mesh(args.source)
    print('Before:', stats(mesh), flush=True)
    mesh.update_faces(mesh.unique_faces())
    mesh.update_faces(mesh.nondegenerate_faces())
    mesh.remove_unreferenced_vertices()
    for size in ([] if args.loops_only else ([4] if args.quads_only else [3, 4])):
        added = recover(mesh, size)
        print(f'{size}-edge holes: added {added} triangles', stats(mesh), flush=True)
    if not args.quads_only:
        added = recover_loops(mesh, args.max_loop)
        print('Small boundary loops: added', added, stats(mesh), flush=True)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    mesh.export(args.output)


if __name__ == '__main__':
    main()
