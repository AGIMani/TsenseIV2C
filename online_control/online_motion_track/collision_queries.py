"""Kinematic collision queries for the A3 hands (never used by the V2 pipeline).

The contact checker runs only forward kinematics and collision detection.  Static
joint-path checks do not need dynamics, constraint factorization or contact forces.
"""
from __future__ import annotations

import numpy as np


class KinematicContactChecker:
    """Use an existing filter's model/data without advancing its simulation."""

    def __init__(self, physics_filter):
        import mujoco

        self.mj = mujoco
        self.model = physics_filter.model
        self.data = physics_filter.data
        self._filter = physics_filter
        self.names = tuple(self.model.joint(i).name for i in range(self.model.njnt))
        self._qadr = np.array(self.model.jnt_qposadr, copy=True)
        self._geom_names = tuple(self.model.geom(i).name for i in range(self.model.ngeom))
        self._last_key = None
        self._contacts = []
        self.reset_counters()

    def reset_counters(self):
        self.kinematics_calls = 0
        self.collision_calls = 0

    def __getattr__(self, name):
        return getattr(self._filter, name)

    def sync_pose_from_dict(self, pose):
        values = np.array([pose[n] for n in self.names], dtype=float)
        if not np.isfinite(values).all():
            raise ValueError("joint target contains NaN or infinity")
        key = values.tobytes()
        if key == self._last_key:
            return
        self.data.qpos[self._qadr] = values
        self.mj.mj_kinematics(self.model, self.data)
        self.mj.mj_collision(self.model, self.data)
        self.kinematics_calls += 1
        self.collision_calls += 1
        self._last_key = key
        rows = []
        for index in range(self.data.ncon):
            contact = self.data.contact[index]
            distance = float(contact.dist)
            if not np.isfinite(distance):
                raise RuntimeError("non-finite MuJoCo collision distance")
            rows.append({
                "g1": self._geom_names[int(contact.geom1)],
                "g2": self._geom_names[int(contact.geom2)],
                "dist_mm": distance * 1000,
                "pen_mm": max(0.0, -distance * 1000),
                # No force solver runs for a purely kinematic check.
                "force_n": 0.0,
            })
        rows.sort(key=lambda row: row["pen_mm"], reverse=True)
        self._contacts = rows

    def contact_summary(self):
        return [dict(row) for row in self._contacts]

    def reset(self):
        self._filter.reset()
        self._last_key = None
        self._contacts = []


class ConservativePairBounds:
    """Verified enclosing spheres and rotated local AABBs for finite geoms.

    Positive gaps are lower bounds on Euclidean clearance, never estimates of the
    surface distance. Mesh bounds are recomputed from the compiled vertices so the
    proof does not rely on a proxy's visual name or an unchecked radius.
    """

    def __init__(self, model, pairs):
        import mujoco

        self.model = model
        self.ids = np.asarray(pairs, dtype=int).reshape(-1, 2)
        self._local_center = np.zeros((model.ngeom, 3))
        self._half_extent = np.zeros((model.ngeom, 3))
        self._radius = np.zeros(model.ngeom)
        self._vertices = {}
        self._face_normals = {}
        bounded = np.zeros(model.ngeom, dtype=bool)
        for geom in range(model.ngeom):
            kind = int(model.geom_type[geom])
            if kind == int(mujoco.mjtGeom.mjGEOM_MESH):
                mesh = model.geom_dataid[geom]
                first = model.mesh_vertadr[mesh]
                vertices = model.mesh_vert[first:first + model.mesh_vertnum[mesh]].astype(float)
                if len(vertices) == 0 or not np.isfinite(vertices).all():
                    raise ValueError("invalid collision mesh vertices")
                low, high = vertices.min(axis=0), vertices.max(axis=0)
                self._local_center[geom] = (low + high) * .5
                self._half_extent[geom] = (high - low) * .5
                self._radius[geom] = np.linalg.norm(vertices, axis=1).max()
                self._vertices[geom] = vertices
                face_first = model.mesh_faceadr[mesh]
                faces = model.mesh_face[face_first:face_first + model.mesh_facenum[mesh]]
                triangles = vertices[faces]
                normals = np.cross(triangles[:, 1] - triangles[:, 0],
                                   triangles[:, 2] - triangles[:, 0])
                lengths = np.linalg.norm(normals, axis=1)
                normals = normals[lengths > 1e-12] / lengths[lengths > 1e-12, None]
                self._face_normals[geom] = np.unique(np.round(normals, 12), axis=0)
            elif kind == int(mujoco.mjtGeom.mjGEOM_BOX):
                self._half_extent[geom] = model.geom_size[geom]
                self._radius[geom] = np.linalg.norm(model.geom_size[geom])
            elif kind == int(mujoco.mjtGeom.mjGEOM_CAPSULE):
                radius, half_length = model.geom_size[geom, :2]
                self._half_extent[geom] = [radius, radius, radius + half_length]
                self._radius[geom] = radius + half_length
            elif kind == int(mujoco.mjtGeom.mjGEOM_SPHERE):
                self._half_extent[geom] = model.geom_size[geom, 0]
                self._radius[geom] = model.geom_size[geom, 0]
            else:
                continue
            bounded[geom] = True
        # Inflate both enclosing shapes by 1 micrometre to absorb transforms and
        # float32 mesh storage roundoff.  A positive result is still conservative.
        self._radius += 1e-6
        self._half_extent += 1e-6
        self._bounded_pairs = np.all(bounded[self.ids], axis=1)
        self._all_bounded = bool(np.all(self._bounded_pairs))
        self._radius_sum = self._radius[self.ids].sum(axis=1)
        # Maximum lever arm of each enclosing geom about each ancestor hinge.
        # Link-offset norms give a posture-independent upper bound, including
        # recentered mesh geoms. These coefficients bound swept displacement.
        motion_radii = np.zeros((model.ngeom, model.njnt))
        for geom in range(model.ngeom):
            body = int(model.geom_bodyid[geom])
            radius = float(np.linalg.norm(model.geom_pos[geom]) + self._radius[geom])
            while body:
                for joint in range(model.body_jntadr[body],
                                   model.body_jntadr[body] + model.body_jntnum[body]):
                    if model.jnt_type[joint] != mujoco.mjtJoint.mjJNT_HINGE:
                        raise ValueError("A3 swept collision checking requires hinge joints")
                    motion_radii[geom, joint] = radius + np.linalg.norm(model.jnt_pos[joint])
                radius += np.linalg.norm(model.body_pos[body])
                body = int(model.body_parentid[body])
        first, second = motion_radii[self.ids[:, 0]], motion_radii[self.ids[:, 1]]
        # Shared ancestor motion is a rigid transform of both shapes and cannot
        # change their relative separation.
        self._shared_motion = (first > 0) & (second > 0)
        self._motion_radii = motion_radii
        self._pair_motion_radii = np.where(self._shared_motion, 0, first + second)

    def swept_displacement(self, delta_q):
        """Lipschitz bound on pair-distance change along a joint segment."""
        travel = self._pair_motion_radii @ np.abs(delta_q)
        if not self._all_bounded:
            travel[~self._bounded_pairs] = np.inf
        return travel

    def segment_motion_radii(self, data, delta_q):
        """Tighten lever arms at this segment's start, with a proven motion pad.

        Common ancestor rotations do not change a point's distance to a hinge
        axis. The total possible descendant displacement is no larger than the
        static full-chain displacement used here as a padding term. Reuse these
        coefficients on all subdivisions of this same segment.
        """
        relative = data.geom_xpos[:, None, :] - data.xanchor[None, :, :]
        perpendicular = np.cross(relative, data.xaxis[None, :, :])
        current = np.linalg.norm(perpendicular, axis=2) + self._radius[:, None]
        padding = self._motion_radii @ np.abs(delta_q)
        radius = np.minimum(self._motion_radii, current + padding[:, None])
        first, second = radius[self.ids[:, 0]], radius[self.ids[:, 1]]
        pairs = np.where(self._shared_motion, 0, first + second)
        if not self._all_bounded:
            pairs[~self._bounded_pairs] = np.inf
        return pairs

    def witness_lower_bound(self, index, data, fromto):
        """Verify separation along a suspect GJK witness using support intervals.

        A zero from MuJoCo is never made positive solely because its witnesses
        differ. Both complete convex shapes must have disjoint support intervals.
        This also works when coarse enclosing boxes overlap.
        """
        if not self._bounded_pairs[index]:
            return 0.0
        direction = fromto[3:] - fromto[:3]
        length = np.linalg.norm(direction)
        if not np.isfinite(length) or length < 1e-12:
            return 0.0
        direction = direction / length
        a, b = self.ids[index]
        rotation = data.geom_xmat.reshape(-1, 3, 3)
        high, low = [], []
        for geom in (a, b):
            local = rotation[geom].T @ direction
            vertices = self._vertices.get(geom)
            if vertices is not None:
                projection = vertices @ local
                low.append(float(projection.min()))
                high.append(float(projection.max()))
            else:
                # The verified box includes any supported primitive. Using it
                # can only weaken this separation certificate.
                radius = float(np.abs(local) @ self._half_extent[geom])
                center = float(local @ self._local_center[geom])
                low.append(center - radius)
                high.append(center + radius)
        delta = float((data.geom_xpos[b] - data.geom_xpos[a]) @ direction)
        gap = max(0.0, delta + low[1] - high[0] - 2e-6,
                  -delta + low[0] - high[1] - 2e-6)
        if gap > 0 or a not in self._vertices or b not in self._vertices:
            return gap
        # A degenerate witness need not give the correct normal. Check face
        # normals against the actual compiled hull vertices before accepting a
        # positive clearance; otherwise keep the conservative zero result.
        directions = np.concatenate((self._face_normals[a] @ rotation[a].T,
                                     self._face_normals[b] @ rotation[b].T))
        lengths = np.linalg.norm(directions, axis=1)
        directions = directions[lengths > 1e-12] / lengths[lengths > 1e-12, None]
        first = self._vertices[a] @ (directions @ rotation[a]).T
        second = self._vertices[b] @ (directions @ rotation[b]).T
        delta = directions @ (data.geom_xpos[b] - data.geom_xpos[a])
        gaps = np.maximum(delta + second.min(axis=0) - first.max(axis=0),
                          -delta + first.min(axis=0) - second.max(axis=0))
        return max(0.0, float(gaps.max(initial=0)) - 2e-6)

    def lower_bounds(self, data):
        positions = data.geom_xpos
        a, b = self.ids.T
        sphere = np.linalg.norm(positions[a] - positions[b], axis=1) - self._radius_sum
        rotation = data.geom_xmat.reshape(-1, 3, 3)
        centers = positions + np.einsum("gij,gj->gi", rotation, self._local_center)
        extent = np.einsum("gij,gj->gi", np.abs(rotation), self._half_extent)
        gaps = np.maximum(np.abs(centers[a] - centers[b]) - extent[a] - extent[b], 0)
        box = np.linalg.norm(gaps, axis=1)
        # An overlapping AABB cannot lower-bound *signed* penetration by zero.
        bound = np.where(box > 0, np.maximum(sphere, box), sphere)
        # Only refine nearby pairs. A weaker but valid bound on other pairs is
        # enough for safety and avoids paying six projection axes for all pairs.
        near = np.flatnonzero(bound < .0005)
        if near.size:
            aa, bb = a[near], b[near]
            ra, rb = rotation[aa], rotation[bb]
            delta = (centers[aa] - centers[bb])[..., None]
            rat, rbt = ra.transpose(0, 2, 1), rb.transpose(0, 2, 1)
            relative = np.abs(rat @ rb)
            ea, eb = self._half_extent[aa], self._half_extent[bb]
            gap_a = np.abs((rat @ delta)[..., 0]) - ea - (relative @ eb[..., None])[..., 0]
            gap_b = np.abs((rbt @ delta)[..., 0]) - eb - (relative.transpose(0, 2, 1) @ ea[..., None])[..., 0]
            bound[near] = np.maximum(bound[near], np.maximum(gap_a.max(axis=1), gap_b.max(axis=1)))
        return np.where(self._bounded_pairs, bound, -np.inf)
