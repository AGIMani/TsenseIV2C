"""Optional local Mink projection of retarget targets onto collision constraints.

Uses Mink's real CollisionAvoidanceLimit with DAQP; no dynamic mj_step or PD lag.
Distances refer to the existing MJCF proxies, not the displayed triangle meshes.
"""
from __future__ import annotations

import time

import numpy as np


class MinkCollisionGuard:
    def __init__(self, profile, side):
        import mink
        import mujoco
        from qpsolvers import solve_qp

        self.mj = mujoco
        self.solve_qp = solve_qp
        self.model = mujoco.MjModel.from_xml_path(str(profile.path("mjcf", side)))
        self.configuration = mink.Configuration(self.model)
        self.names = [self.model.joint(i).name for i in range(self.model.njnt)]
        if (self.model.nq != self.model.nv or self.model.nv != len(self.names)
                or set(self.names) != set(profile.joint_limits(side))):
            raise ValueError("Mink 模型关节与 URDF 不一致")
        self.qadr = self.model.jnt_qposadr.copy()
        self.dadr = self.model.jnt_dofadr.copy()
        self.lower = np.array([profile.joint_limits(side)[n][0] for n in self.names])
        self.upper = np.array([profile.joint_limits(side)[n][1] for n in self.names])
        ids = list(range(self.model.ngeom))
        all_pairs = mink.CollisionAvoidanceLimit(self.model, [(ids, ids)]).geom_id_pairs
        normal, tips = [], []
        thumb_tip = f"{side}_hand_thumb_PIP_link_capsule"
        for a, b in sorted(all_pairs):
            names = (self.model.geom(a).name, self.model.geom(b).name)
            is_pinch = thumb_tip in names and any("_DIP_link_capsule" in n for n in names)
            (tips if is_pinch else normal).append(([a], [b]))
        # Small margins fit a hand better than Mink's general-purpose 5 mm default.
        # Only thumb/distal-finger pairs may approach contact; no pair is disabled.
        self.limits = [mink.CollisionAvoidanceLimit(
            self.model, pairs, gain=1.0, minimum_distance_from_collisions=margin,
            collision_detection_distance=.015, bound_relaxation=0.0)
            for pairs, margin in ((normal, .0005), (tips, .00005)) if pairs]
        self.pairs = [p for limit in self.limits for p in limit.geom_id_pairs]
        self.margins = np.array([limit.minimum_distance_from_collisions
                                for limit in self.limits for _ in limit.geom_id_pairs])
        self.pair_ids = np.asarray(self.pairs, dtype=int)
        # MuJoCo's bounding spheres enclose capsules. Keep other geometry types
        # on the exact-distance path; only provably separated capsules are skipped.
        capsule = int(mujoco.mjtGeom.mjGEOM_CAPSULE)
        self.bounded_pairs = np.all(self.model.geom_type[self.pair_ids] == capsule, axis=1)
        self.pair_bounds = self.model.geom_rbound[self.pair_ids].sum(axis=1) + self.margins + 1e-6
        self.q = self._vector(profile.init_pose(side))
        if not self._feasible(self.q):
            raise ValueError("Mink 初始姿态碰撞，请先检查碰撞代理几何")
        self.diagnostics = {"mode": "mink", "pair_count": len(self.pairs)}

    def _vector(self, pose):
        q = np.array([pose[n] for n in self.names], dtype=float)
        if not np.isfinite(q).all():
            raise ValueError("Mink 目标角度必须有限")
        return np.clip(q, self.lower, self.upper)

    def _update(self, q):
        values = np.empty(self.model.nq)
        values[self.qadr] = q
        self.configuration.update(values)

    def distances(self, pose):
        """Signed distance in metres, in the same order as self.pairs."""
        return self._distances(self._vector(pose))

    def _distances(self, q):
        self._update(q)
        return np.array([self.mj.mj_geomDistance(
            self.model, self.configuration.data, a, b, .05, None) for a, b in self.pairs])

    def _feasible(self, q):
        self._update(q)
        centers = self.configuration.data.geom_xpos[self.pair_ids]
        delta = centers[:, 0] - centers[:, 1]
        far = self.bounded_pairs & (np.einsum('ij,ij->i', delta, delta) > self.pair_bounds ** 2)
        for index in np.flatnonzero(~far):
            a, b = self.pairs[index]
            margin = self.margins[index]
            # Use the same query horizon as _distances/final validation. Mesh
            # queries clipped to the margin can disagree with the full query.
            if self.mj.mj_geomDistance(self.model, self.configuration.data, a, b,
                                      .05, None) < margin - 1e-7:
                return False
        return True

    def _path_feasible(self, start, end):
        # Sample the motion as well as the endpoint to limit between-frame crossing.
        # This is a discrete check, not a continuous collision certificate.
        count = max(1, int(np.ceil(np.max(np.abs(end - start)) / .025)))
        if not self._feasible(end):
            return False
        return all(self._feasible(start + (end - start) * (i / count))
                   for i in range(1, count))

    def project(self, pose):
        started = time.perf_counter()
        target = self._vector(pose)
        raw_distances = self._distances(target)
        q = self.q.copy()
        iterations = 0
        status = "clear"
        if self._path_feasible(q, target):
            q = target
        else:
            status = "limited"
            for iterations in range(1, 13):
                self._update(q)
                matrices, bounds = [], []
                for limit in self.limits:
                    # Mink 1.3 returns normal-velocity inequalities. dt=1 maps
                    # the variable directly to joint displacement for this SQP step.
                    constraint = limit.compute_qp_inequalities(self.configuration, dt=1.0)
                    active = np.isfinite(constraint.h)
                    matrices.append(constraint.G[active][:, self.dadr])
                    bounds.append(constraint.h[active])
                delta = self.solve_qp(
                    np.eye(len(q)), q - target, np.vstack(matrices), np.concatenate(bounds),
                    lb=np.maximum(self.lower - q, -.12),
                    ub=np.minimum(self.upper - q, .12), solver="daqp")
                if delta is None or not np.isfinite(delta).all():
                    status = "held_solver_failure"
                    break
                if np.max(np.abs(delta)) < 1e-5:
                    break
                accepted = False
                for backtrack in range(9):
                    candidate = np.clip(q + delta * (.5 ** backtrack), self.lower, self.upper)
                    if (self._path_feasible(q, candidate)
                            and self._path_feasible(self.q, candidate)):
                        accepted = True
                        break
                if not accepted:
                    status = "held_distance_check"
                    break
                q = candidate
                if np.max(np.abs(target - q)) < 1e-4:
                    break
        final_distances = self._distances(q)
        if np.any(final_distances < self.margins - 1e-7):
            raise RuntimeError("Mink 输出距离校验失败")
        self.q = q.copy()
        index = int(np.argmin(final_distances))
        self.diagnostics = {
            "mode": "mink", "status": status, "pair_count": len(self.pairs),
            "iterations": iterations, "raw_min_distance_mm": float(raw_distances.min() * 1000),
            "min_distance_mm": float(final_distances.min() * 1000),
            "correction_deg": float(np.rad2deg(np.max(np.abs(q - target)))),
            "closest_pair": [self.model.geom(g).name for g in self.pairs[index]],
            "solve_ms": (time.perf_counter() - started) * 1000,
        }
        return dict(zip(self.names, map(float, q)))
