"""Optional local Mink projection of retarget targets onto collision constraints.

Uses Mink's real CollisionAvoidanceLimit with DAQP; no dynamic mj_step or PD lag.
Contact normals constrain joint motion jointly. Path rejection is local to the
participating kinematic finger chains, so other fingers still reach their targets.
Clearances refer to the MJCF collision envelopes. Positive values may be
conservative geometric lower bounds; near pairs use MuJoCo signed distances.
"""
from __future__ import annotations

import time
from numbers import Integral

import numpy as np

from online_control.online_motion_track.collision_queries import ConservativePairBounds


class MinkCollisionGuard:
    def __init__(self, profile, side, *, max_path_refinements=128):
        import mink
        import mujoco
        from qpsolvers import solve_qp

        if (isinstance(max_path_refinements, bool)
                or not isinstance(max_path_refinements, Integral) or max_path_refinements < 1):
            raise ValueError("max_path_refinements must be a positive integer")
        self.max_path_refinements = int(max_path_refinements)

        class EnvelopeCollisionLimit(mink.CollisionAvoidanceLimit):
            """Keep Mink's Jacobian/QP assembly, strengthen only its broadphase."""

            def _broadphase_survivors(limit, data):
                lower = self._current_bounds()[limit._guard_pair_indices]
                return np.flatnonzero(lower <= limit.collision_detection_distance)

        self.mj = mujoco
        self.solve_qp = solve_qp
        self.model = mujoco.MjModel.from_xml_path(str(profile.path("mjcf", side)))
        self.configuration = mink.Configuration(self.model)
        self.names = [self.model.joint(i).name for i in range(self.model.njnt)]
        joint_limits = profile.joint_limits(side)
        if (self.model.nq != self.model.nv or self.model.nv != len(self.names)
                or set(self.names) != set(joint_limits)):
            raise ValueError("Mink 模型关节与 URDF 不一致")
        self.qadr = self.model.jnt_qposadr.copy()
        self.dadr = self.model.jnt_dofadr.copy()
        self.lower = np.array([joint_limits[n][0] for n in self.names])
        self.upper = np.array([joint_limits[n][1] for n in self.names])
        ids = list(range(self.model.ngeom))
        all_pairs = mink.CollisionAvoidanceLimit(self.model, [(ids, ids)]).geom_id_pairs
        normal, tips = [], []
        thumb_tip = f"{side}_hand_thumb_PIP_link"
        for a, b in sorted(all_pairs):
            names = tuple(self.model.body(self.model.geom_bodyid[g]).name for g in (a, b))
            # Preserve the previous effective margins: the existing with-ID
            # meshes always used 0.5 mm; ordinary hands allow a 0.05 mm pinch.
            is_pinch = (not profile.model_id.endswith("_withid") and thumb_tip in names
                        and any(n.endswith("_DIP_link") for n in names))
            (tips if is_pinch else normal).append(([a], [b]))
        # Small margins fit a hand better than Mink's general-purpose 5 mm default.
        # Only thumb/distal-finger pairs may approach contact; no pair is disabled.
        self.limits = [EnvelopeCollisionLimit(
            self.model, pairs, gain=1.0, minimum_distance_from_collisions=margin,
            collision_detection_distance=.015, bound_relaxation=0.0)
            for pairs, margin in ((normal, .0005), (tips, .00005)) if pairs]
        self.pairs = [p for limit in self.limits for p in limit.geom_id_pairs]
        self.margins = np.array([limit.minimum_distance_from_collisions
                                for limit in self.limits for _ in limit.geom_id_pairs])
        self.pair_ids = np.asarray(self.pairs, dtype=int)
        self.bounds = ConservativePairBounds(self.model, self.pairs)
        self._build_finger_groups()
        offset = 0
        for limit in self.limits:
            limit._guard_pair_indices = np.arange(offset, offset + len(limit.geom_id_pairs))
            # Mink versions with broadphase support normally skip tiny groups;
            # these bounds are already shared across groups and are cheap to reuse.
            limit.broadphase_min_pairs = 0
            offset += len(limit.geom_id_pairs)
        self._last_q_key = None
        self._hessian = np.eye(len(self.names))
        self._fromto = np.zeros(6)
        self._sweep_numeric_pad = max(2e-7, 2 * float(self.model.opt.ccd_tolerance))
        self._begin_frame()
        self.q = self._vector(profile.init_pose(side))
        if not self._feasible(self.q):
            raise ValueError("Mink 初始姿态碰撞，请先检查碰撞代理几何")
        self.diagnostics = {"mode": "mink", "pair_count": len(self.pairs)}

    def _build_finger_groups(self):
        # Group by the first movable ancestor, not by geom/joint name. Include
        # the whole chain in the QP, including joints distal to a contact point.
        ancestors = np.zeros((self.model.nbody, len(self.names)), dtype=bool)
        for body in range(1, self.model.nbody):
            ancestors[body] = ancestors[self.model.body_parentid[body]]
            first = self.model.body_jntadr[body]
            ancestors[body, first:first + self.model.body_jntnum[body]] = True
        roots = [np.flatnonzero(ancestors[body])[0] for body in self.model.jnt_bodyid]
        unique = sorted(set(roots))
        self._joint_group = np.array([unique.index(root) for root in roots])
        self._group_joints = [np.flatnonzero(self._joint_group == group)
                              for group in range(len(unique))]
        self._group_names = [self.names[root].split("_hand_")[-1].split("_")[0]
                             for root in unique]
        # Common ancestors move both geoms rigidly and cannot change clearance.
        support = ancestors[self.model.geom_bodyid]
        pair_support = support[self.pair_ids[:, 0]] ^ support[self.pair_ids[:, 1]]
        self._pair_groups = np.column_stack([
            np.any(pair_support[:, joints], axis=1) for joints in self._group_joints])

    def _vector(self, pose):
        q = np.array([pose[n] for n in self.names], dtype=float)
        if not np.isfinite(q).all():
            raise ValueError("Mink 目标角度必须有限")
        return np.clip(q, self.lower, self.upper)

    def _update(self, q):
        key = q.tobytes()
        if key == self._last_q_key:
            return
        values = np.empty(self.model.nq)
        values[self.qadr] = q
        self.configuration.update(values)
        self._last_q_key = key
        self._kinematics_calls += 1

    def _begin_frame(self):
        # Exact values only: quantizing cache keys could turn a near collision
        # into a cache hit for a different, safe configuration.
        self._distance_cache = {}
        self._feasibility_cache = {}
        self._bound_cache = {}
        self._queried_cache = {}
        self._path_cache = {}
        self._cache_hits = 0
        self._distance_queries = 0
        self._bounds_skipped = 0
        self._kinematics_calls = 0
        self._witness_certificates = 0
        self._path_refinements = 0
        self._refinement_keys = set()
        self._path_budget_exhausted = False
        self._blocking_pairs = set()

    def _bounds_at(self, q):
        key = q.tobytes()
        bounds = self._bound_cache.get(key)
        if bounds is None:
            self._update(q)
            bounds = self.bounds.lower_bounds(self.configuration.data)
            self._bound_cache[key] = bounds
        return bounds

    def _current_bounds(self):
        bounds = self._bound_cache.get(self._last_q_key)
        if bounds is None:
            bounds = self.bounds.lower_bounds(self.configuration.data)
            self._bound_cache[self._last_q_key] = bounds
        return bounds

    def _query(self, index):
        a, b = self.pairs[index]
        self._distance_queries += 1
        distance = self.mj.mj_geomDistance(self.model, self.configuration.data, a, b, .05, self._fromto)
        if not np.isfinite(distance):
            raise RuntimeError("non-finite MuJoCo collision distance")
        if distance == 0.0:
            # MuJoCo 3.10 can return zero with separated witnesses after a
            # degenerate GJK simplex. Independently check a separating support
            # plane; ambiguous/touching results retain zero and remain blocked.
            certified = self.bounds.witness_lower_bound(index, self.configuration.data, self._fromto)
            if certified > 0:
                self._witness_certificates += 1
                distance = certified
        return distance

    def distances(self, pose):
        """Conservative signed clearance in metres, in self.pairs order.

        A positive geometric lower bound may replace a far-pair distance.
        Values are suitable for safety comparisons, not exact metrology.
        """
        return self._distances(self._vector(pose))

    def _distances(self, q):
        key = q.tobytes()
        if key in self._distance_cache:
            self._cache_hits += 1
            return self._distance_cache[key].copy()
        bounds = self._bounds_at(q)
        far = bounds >= self.margins
        distances = np.minimum(bounds, .05).copy()
        self._bounds_skipped += int(far.sum())
        if not np.all(far):
            self._update(q)
            for index in np.flatnonzero(~far):
                distances[index] = self._query(index)
        self._distance_cache[key] = distances
        self._queried_cache[key] = ~far
        self._feasibility_cache[key] = bool(np.all(distances >= self.margins - 1e-7))
        return distances.copy()

    def _feasible(self, q):
        key = q.tobytes()
        if key in self._feasibility_cache:
            self._cache_hits += 1
            return self._feasibility_cache[key]
        bounds = self._bounds_at(q)
        far = bounds >= self.margins
        distances = np.minimum(bounds, .05).copy()
        self._bounds_skipped += int(far.sum())
        if not np.all(far):
            self._update(q)
        for index in np.flatnonzero(~far):
            margin = self.margins[index]
            # Use the same query horizon as _distances/final validation. Mesh
            # queries clipped to the margin can disagree with the full query.
            distances[index] = self._query(index)
            if distances[index] < margin - 1e-7:
                self._feasibility_cache[key] = False
                return False
        self._feasibility_cache[key] = True
        self._distance_cache[key] = distances
        self._queried_cache[key] = ~far
        return True

    def _refine_clearances(self, q, indices):
        """Refine coarse positive bounds only when a swept proof needs them."""
        key = q.tobytes()
        distances = self._distances(q)
        queried = self._queried_cache[key]
        remaining = indices[~queried[indices]]
        if remaining.size:
            self._update(q)
            for index in remaining:
                # A proven positive separation remains valid if native GJK
                # spuriously reports a smaller value or zero.
                distances[index] = max(distances[index], self._query(index))
            queried[remaining] = True
            self._distance_cache[key] = distances
        return distances

    def _path_feasible(self, start, end):
        return not self._path_blockers(start, end)

    def _path_blockers(self, start, end):
        """Return a witness set of unsafe/uncertified pairs, never just a flag."""
        key = (start.tobytes(), end.tobytes())
        if key not in self._path_cache:
            self._path_cache[key] = self._check_path(start, end)
        else:
            self._cache_hits += 1
        return self._path_cache[key]

    def _violating_pairs(self, q):
        return tuple(np.flatnonzero(self._distances(q) < self.margins - 1e-7))

    def _check_path(self, start, end):
        # Keep the original coarse samples and margins, then certify that no
        # penetration can hide between them. Only uncertain segments subdivide.
        count = max(1, int(np.ceil(np.max(np.abs(end - start)) / .025)))
        if not self._feasible(end):
            return self._violating_pairs(end)
        samples = [start] + [start + (end - start) * (i / count) for i in range(1, count)] + [end]
        for q in samples[1:-1]:
            if not self._feasible(q):
                return self._violating_pairs(q)
        for first, second in zip(samples, samples[1:]):
            blockers = self._segment_blockers(first, second, depth=0)
            if blockers:
                return blockers
        return ()

    def _segment_blockers(self, start, end, depth, radii=None):
        first, second = self._distances(start), self._distances(end)
        delta = end - start
        travel = self.bounds.swept_displacement(delta) if radii is None else radii @ np.abs(delta)
        # Distance is Lipschitz under bounded point displacement. The two
        # endpoint cones cover the whole interval when their clearances sum
        # exceeds total travel. This certifies >0 physical clearance, not the
        # full endpoint margin, so a pose on its margin can still move away.
        if np.all(first + second > travel + self._sweep_numeric_pad):
            return ()
        uncertain = np.flatnonzero(first + second <= travel + self._sweep_numeric_pad)
        first = self._refine_clearances(start, uncertain)
        second = self._refine_clearances(end, uncertain)
        if np.all(first + second > travel + self._sweep_numeric_pad):
            return ()
        if radii is None:
            self._update(start)
            radii = self.bounds.segment_motion_radii(self.configuration.data, delta)
        uncertain = tuple(np.flatnonzero(
            first + second <= radii @ np.abs(delta) + self._sweep_numeric_pad))
        if not uncertain:
            return ()
        if depth >= 10:
            return uncertain
        midpoint = (start + end) * .5
        key = midpoint.tobytes()
        if key not in self._refinement_keys:
            if self._path_refinements >= self.max_path_refinements:
                self._path_budget_exhausted = True
                return uncertain
            self._refinement_keys.add(key)
            self._path_refinements += 1
        if not self._feasible(midpoint):
            return self._violating_pairs(midpoint)
        return (self._segment_blockers(start, midpoint, depth + 1, radii)
                or self._segment_blockers(midpoint, end, depth + 1, radii))

    def _advance_fingers(self, q, desired, *, backtrack):
        """Validate the combined pose; reduce only chains in a blocking pair.

        A newly exposed cross-finger collision joins both chains to the solve.
        Every accepted pose also has a certified straight path from the last
        published pose, not merely from an internal SQP iterate.
        """
        delta = desired - q
        reductions = np.zeros(len(self._group_joints), dtype=int)
        involved = np.zeros(len(self._group_joints), dtype=bool)
        candidate = desired.copy()
        # Each failing pass reduces at least one chain, eventually to zero.
        attempts = 9 if backtrack else 1
        for _ in range(attempts * len(reductions) + 1):
            blockers = (self._path_blockers(q, candidate)
                        or self._path_blockers(self.q, candidate))
            if not blockers:
                return candidate, involved
            self._blocking_pairs.update(blockers)
            groups = np.any(self._pair_groups[list(blockers)], axis=0)
            involved |= groups
            reducible = groups & (reductions < attempts)
            if not np.any(reducible):
                break
            reductions[reducible] += 1
            scales = np.where(reductions < attempts, .5 ** reductions, 0.0)
            joint_scales = scales[self._joint_group]
            candidate = np.where(joint_scales == 1, desired, q + delta * joint_scales)
        return q.copy(), involved

    def _contact_constraints(self, q):
        self._update(q)
        matrices, bounds = [], []
        for limit in self.limits:
            # Mink returns contact-normal velocity inequalities. dt=1 maps
            # the variable directly to joint displacement for this SQP step.
            constraint = limit.compute_qp_inequalities(self.configuration, dt=1.0)
            matrices.append(constraint.G[:, self.dadr])
            bounds.append(constraint.h)
        return np.vstack(matrices), np.concatenate(bounds)

    def _contact_step(self, q, target, joints):
        matrix, bound = self._contact_constraints(q)
        active = np.isfinite(bound)
        matrix, bound = matrix[active], bound[active]
        lower = np.maximum(self.lower - q, -.12)[joints]
        upper = np.minimum(self.upper - q, .12)[joints]
        hessian = self._hessian[np.ix_(joints, joints)]
        linear = (q - target)[joints]
        step = self.solve_qp(hessian, linear, matrix[:, joints], bound,
                             lb=lower, ub=upper, solver="daqp")
        if step is None or not np.isfinite(step).all():
            return None
        delta = np.zeros_like(q)
        delta[joints] = step
        # A tangent step at a curved surface can fail the nonlinear check even
        # though its normal velocity is valid. Re-linearize at the trial contact
        # and jointly correct the joints, instead of only shrinking their motion
        # until the entire finger stalls on the clearance boundary.
        for _ in range(3):
            candidate = q + delta
            violated = self._violating_pairs(candidate)
            if not violated:
                break
            self._blocking_pairs.update(violated)
            trial_matrix, trial_bound = self._contact_constraints(candidate)
            rows = np.array([i for i in violated if np.isfinite(trial_bound[i])], dtype=int)
            if not rows.size:
                break
            correction = trial_matrix[rows]
            # Mink clamps its bound at zero inside the margin; restoring an
            # infeasible trial needs the actual signed clearance deficit.
            clearance = self._distances(candidate)[rows] - self.margins[rows]
            restored = self.solve_qp(
                hessian, linear, np.vstack((matrix, correction))[:, joints],
                np.concatenate((bound, clearance + correction @ delta - self._sweep_numeric_pad)),
                lb=lower, ub=upper, solver="daqp")
            if restored is None or not np.isfinite(restored).all():
                break
            delta[joints] = restored
        return delta

    def project(self, pose):
        started = time.perf_counter()
        self._begin_frame()
        target = self._vector(pose)
        raw_distances = self._distances(target)
        q = self.q.copy()
        iterations = 0
        status = "clear"
        if self._path_feasible(q, target):
            q = target
        else:
            status = "limited"
            # First deliver independently safe finger targets exactly. Only the
            # blocked chains enter the iterative contact-normal projection.
            q, involved = self._advance_fingers(q, target, backtrack=False)
            for iterations in range(1, 13):
                joints = np.flatnonzero(involved[self._joint_group])
                # Uninvolved chains are already at their requested targets.
                # Keep all joints of the involved chains free: Mink imposes
                # only n.T @ (J_b - J_a) @ dq >= margin - distance, not a lock
                # on individual joints or on tangential contact-point motion.
                delta = self._contact_step(q, target, joints)
                if delta is None:
                    status = "held_solver_failure"
                    break
                if np.max(np.abs(delta)) < 1e-5:
                    break
                candidate, blocked = self._advance_fingers(
                    q, np.clip(q + delta, self.lower, self.upper), backtrack=True)
                expanded = np.any(blocked & ~involved)
                involved |= blocked
                if np.array_equal(candidate, q) and not expanded:
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
            "blocking_pairs": [[self.model.geom(g).name for g in self.pairs[i]]
                               for i in sorted(self._blocking_pairs)],
            "correction_by_finger_deg": {
                name: float(np.rad2deg(np.max(np.abs(q[joints] - target[joints]))))
                for name, joints in zip(self._group_names, self._group_joints)},
            "solve_ms": (time.perf_counter() - started) * 1000,
            "distance_kind": "conservative_clearance",
            "distance_queries": self._distance_queries,  # Safety/diagnostic scans; excludes Mink's QP assembly.
            "bounds_skipped": self._bounds_skipped,
            "cache_hits": self._cache_hits,
            "kinematics_calls": self._kinematics_calls,
            "witness_certificates": self._witness_certificates,
            "path_refinements": self._path_refinements,
            "path_refinement_budget": self.max_path_refinements,
            "path_budget_exhausted": self._path_budget_exhausted,
        }
        return dict(zip(self.names, map(float, q)))
