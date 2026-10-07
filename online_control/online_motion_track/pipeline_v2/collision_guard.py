"""Collision-aware joint projection for the light TsenseI hand viewer.

Two policies share the same MuJoCo capsule model:

``stop``
    Continuous joint-path checking followed by bisection at the first unsafe
    point.  This is deterministic and is the fail-safe policy.

``avoid``
    A MoveIt-inspired whole-hand planner first searches for a collision-free
    joint-space route and executes its waypoints.  A predictive distance
    barrier remains as the fast local fallback, while ``stop`` remains the
    final fail-safe for every segment and every post-Ruckig sample.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Mapping

import numpy as np

from config_path import MJCF_MODEL_PATH
try:
    from .moveit_style_planner import JointSpaceRRTConnectPlanner
except ImportError:  # Legacy standalone light_test entry point.
    from moveit_style_planner import JointSpaceRRTConnectPlanner
from physics.mujoco_filter import MuJoCoPhysicsFilter


COLLISION_STRATEGY_STOP = "stop"
COLLISION_STRATEGY_AVOID = "avoid"
COLLISION_STRATEGIES = frozenset(
    {COLLISION_STRATEGY_STOP, COLLISION_STRATEGY_AVOID}
)
_FIXED_JOINT_SUFFIXES = ("_pinky_rota_joint",)


def _normalise_strategy(value: str) -> str:
    strategy = str(value).strip().lower()
    if strategy not in COLLISION_STRATEGIES:
        raise ValueError(
            f"collision strategy must be one of {sorted(COLLISION_STRATEGIES)}, "
            f"got {value!r}"
        )
    return strategy


class SelfCollisionGuard:
    """Apply predictive avoidance or stop projection to a retarget pose.

    TsenseI's MJCF uses capsule geometry and collision masks between different
    fingers.  ``avoid`` combines a bounded global joint-space planner with
    compliant contact dynamics and a local signed-distance barrier; ``stop``
    continuously checks the segment from the last accepted pose and bisects at
    the first unsafe point.  Both policies end at the same deterministic
    penetration check.
    """

    def __init__(
        self,
        *,
        mjcf_path: str | Path = MJCF_MODEL_PATH,
        allowed_penetration_mm: float = 0.20,
        path_step_rad: float = 0.08,
        bisection_iterations: int = 10,
        max_path_steps: int = 24,
        strategy: str = COLLISION_STRATEGY_STOP,
        avoidance_max_evaluations: int = 64,
        avoidance_drive_step_rad: float = 0.035,
        avoidance_waypoint_tolerance_rad: float = 0.018,
        safety_clearance_mm: float = 0.25,
        activation_distance_mm: float = 1.25,
        cbf_gamma: float = 0.35,
        distance_jacobian_epsilon_rad: float = 0.003,
        cbf_projection_iterations: int = 10,
        compliance_steps: int = 20,
        planner_max_iterations: int = 56,
        planner_extension_step_rad: float = 0.14,
        planner_goal_bias: float = 0.24,
        planner_max_edge_checks: int = 160,
        planner_target_replan_rad: float = 0.10,
        planner_goal_evaluations: int = 40,
        physics_filter: Any | None = None,
        compliance_filter: Any | None = None,
    ) -> None:
        self.allowed_penetration_mm = max(float(allowed_penetration_mm), 0.0)
        self.path_step_rad = max(float(path_step_rad), 1.0e-4)
        self.bisection_iterations = max(1, int(bisection_iterations))
        self.max_path_steps = max(1, int(max_path_steps))
        self.strategy = _normalise_strategy(strategy)
        self.avoidance_max_evaluations = max(1, int(avoidance_max_evaluations))
        self.avoidance_drive_step_rad = max(
            1.0e-4, float(avoidance_drive_step_rad)
        )
        self.avoidance_waypoint_tolerance_rad = max(
            1.0e-4, float(avoidance_waypoint_tolerance_rad)
        )
        self.safety_clearance_mm = max(float(safety_clearance_mm), 0.0)
        self.activation_distance_mm = max(
            float(activation_distance_mm), self.safety_clearance_mm + 1.0e-3
        )
        self.cbf_gamma = max(0.01, min(float(cbf_gamma), 1.0))
        self.distance_jacobian_epsilon_rad = max(
            1.0e-5, float(distance_jacobian_epsilon_rad)
        )
        self.cbf_projection_iterations = max(
            1, int(cbf_projection_iterations)
        )
        self.compliance_steps = max(1, int(compliance_steps))
        self.planner_target_replan_rad = max(
            1.0e-4, float(planner_target_replan_rad)
        )
        self.planner_goal_evaluations = max(1, int(planner_goal_evaluations))
        self._joint_planner = JointSpaceRRTConnectPlanner(
            max_iterations=planner_max_iterations,
            extension_step_rad=planner_extension_step_rad,
            goal_bias=planner_goal_bias,
            max_edge_checks=planner_max_edge_checks,
        )
        self._filter = physics_filter or MuJoCoPhysicsFilter(
            mjcf_path=mjcf_path,
            warmup_steps=0,
            hard_contact=False,
        )
        if compliance_filter is not None:
            self._compliance_filter = compliance_filter
        elif self.strategy == COLLISION_STRATEGY_AVOID and physics_filter is None:
            # Match the collision dynamics that made the pre-lightweight main
            # application feel compliant, while a separate kinematic model is
            # retained for non-mutating distance/Jacobian queries.
            self._compliance_filter = MuJoCoPhysicsFilter(
                mjcf_path=mjcf_path,
                warmup_steps=50,
                default_n_steps=self.compliance_steps,
                hard_contact=True,
                retarget_pull_steps=4,
            )
        elif self.strategy == COLLISION_STRATEGY_AVOID and callable(
            getattr(physics_filter, "step_dict", None)
        ):
            self._compliance_filter = physics_filter
        else:
            self._compliance_filter = None
        self._previous_safe: dict[str, float] | None = None
        self._avoidance_phase: str | None = None
        self._avoidance_waypoint: dict[str, float] | None = None
        self._avoidance_goal: dict[str, float] | None = None
        self._avoidance_detour_values: dict[str, float] = {}
        self._avoidance_contact_fingers: list[str] = []
        self._avoidance_plan_id = 0
        self._global_waypoints: list[dict[str, float]] = []
        self._global_target: dict[str, float] | None = None
        self._global_goal: dict[str, float] | None = None
        self._global_plan_method: str | None = None
        self._global_plan_diagnostics: dict[str, Any] = {}

    @staticmethod
    def _pose(values: Mapping[str, float]) -> dict[str, float]:
        pose = {str(name): float(value) for name, value in values.items()}
        if any(not math.isfinite(value) for value in pose.values()):
            raise ValueError("joint target contains NaN or infinity")
        for name in pose:
            if name.endswith(_FIXED_JOINT_SUFFIXES):
                pose[name] = 0.0
        return pose

    @staticmethod
    def _is_fixed_joint(name: str) -> bool:
        return str(name).endswith(_FIXED_JOINT_SUFFIXES)

    @staticmethod
    def _interpolate(
        start: Mapping[str, float],
        target: Mapping[str, float],
        alpha: float,
    ) -> dict[str, float]:
        a = float(alpha)
        return {
            name: float(start.get(name, value) + a * (value - start.get(name, value)))
            for name, value in target.items()
        }

    @staticmethod
    def _max_penetration(contacts: list[dict]) -> float:
        return max(
            (float(contact.get("pen_mm", 0.0)) for contact in contacts),
            default=0.0,
        )

    def _evaluate(self, pose: Mapping[str, float]) -> tuple[float, list[dict]]:
        self._filter.sync_pose_from_dict(dict(pose))
        contacts = self._filter.contact_summary()
        return self._max_penetration(contacts), contacts

    def _is_safe(self, penetration_mm: float) -> bool:
        return penetration_mm <= self.allowed_penetration_mm + 1.0e-9

    def _has_distance_backend(self) -> bool:
        return bool(
            callable(getattr(self._filter, "collision_pair_distances", None))
            and callable(
                getattr(self._filter, "collision_pair_distance_mm", None)
            )
        )

    @staticmethod
    def _pair_key(row: Mapping[str, Any]) -> tuple[int, int]:
        return int(row["geom1"]), int(row["geom2"])

    def _proximity_map(
        self,
        pose: Mapping[str, float],
        *,
        max_distance_mm: float,
    ) -> dict[tuple[int, int], dict[str, Any]]:
        self._filter.sync_pose_from_dict(dict(pose))
        rows = self._filter.collision_pair_distances(
            max_distance_mm=float(max_distance_mm)
        )
        return {self._pair_key(row): dict(row) for row in rows}

    def _pair_distance_mm(
        self,
        pair: tuple[int, int],
        *,
        max_distance_mm: float,
    ) -> float:
        return float(
            self._filter.collision_pair_distance_mm(
                pair[0],
                pair[1],
                max_distance_mm=float(max_distance_mm),
            )
        )

    def _limited_delta(
        self,
        name: str,
        start_value: float,
        desired_value: float,
    ) -> float:
        if self._is_fixed_joint(name):
            return -float(start_value)
        delta = float(desired_value) - float(start_value)
        delta = max(
            -self.avoidance_drive_step_rad,
            min(self.avoidance_drive_step_rad, delta),
        )
        lo, hi = self._joint_bounds(name)
        return max(lo - float(start_value), min(hi - float(start_value), delta))

    def _compliant_nominal(
        self,
        start: Mapping[str, float],
        target: Mapping[str, float],
    ) -> tuple[dict[str, float], dict[str, Any]]:
        """Get one bounded increment from the original MuJoCo contact model."""
        desired = dict(target)
        diagnostics: dict[str, Any] = {
            "compliance_backend": "joint_delta",
            "compliance_contacts": 0,
            "compliance_max_correction_rad": 0.0,
        }
        backend = self._compliance_filter
        if backend is not None:
            try:
                backend.sync_pose_from_dict(dict(start))
                desired = self._pose(
                    backend.step_dict(dict(target), n_steps=self.compliance_steps)
                )
                contact_summary = getattr(backend, "contact_summary", None)
                contacts = contact_summary() if callable(contact_summary) else []
                diagnostics = {
                    "compliance_backend": "mujoco_contact_dynamics",
                    "compliance_contacts": len(contacts),
                    "compliance_max_correction_rad": round(
                        max(
                            (
                                abs(float(target[name]) - float(desired[name]))
                                for name in target
                            ),
                            default=0.0,
                        ),
                        8,
                    ),
                }
            except Exception as exc:  # noqa: BLE001
                desired = dict(target)
                diagnostics = {
                    "compliance_backend": "joint_delta_fallback",
                    "compliance_contacts": 0,
                    "compliance_error": f"{type(exc).__name__}: {exc}",
                    "compliance_max_correction_rad": 0.0,
                }

        bounded = {
            name: float(start[name])
            + self._limited_delta(name, float(start[name]), float(desired[name]))
            for name in target
        }
        return self._pose(bounded), diagnostics

    def _distance_gradient(
        self,
        start: Mapping[str, float],
        pair: tuple[int, int],
        pair_row: Mapping[str, Any],
        names: list[str],
        *,
        max_distance_mm: float,
    ) -> np.ndarray:
        """Numerically linearise one signed capsule distance in joint space."""
        first_finger = self._finger_from_text(pair_row.get("g1", ""))
        second_finger = self._finger_from_text(pair_row.get("g2", ""))
        affected_fingers = {first_finger, second_finger} - {None}
        gradient = np.zeros(len(names), dtype=np.float64)
        epsilon = self.distance_jacobian_epsilon_rad
        for index, name in enumerate(names):
            if self._is_fixed_joint(name):
                continue
            joint_finger = self._finger_from_text(name)
            if affected_fingers and joint_finger not in affected_fingers:
                continue
            center = float(start[name])
            lo, hi = self._joint_bounds(name)
            lower = max(lo, center - epsilon)
            upper = min(hi, center + epsilon)
            if upper - lower <= 1.0e-9:
                continue
            low_pose = dict(start)
            low_pose[name] = lower
            self._filter.sync_pose_from_dict(low_pose)
            low_distance = self._pair_distance_mm(
                pair, max_distance_mm=max_distance_mm
            )
            high_pose = dict(start)
            high_pose[name] = upper
            self._filter.sync_pose_from_dict(high_pose)
            high_distance = self._pair_distance_mm(
                pair, max_distance_mm=max_distance_mm
            )
            gradient[index] = (high_distance - low_distance) / (upper - lower)
        self._filter.sync_pose_from_dict(dict(start))
        return gradient

    def _cbf_project_increment(
        self,
        start: Mapping[str, float],
        target: Mapping[str, float],
    ) -> tuple[dict[str, float], dict[str, Any]]:
        """Weighted projection of a nominal increment onto CBF half spaces.

        This solves the small convex safety subproblem with cyclic weighted
        half-space projections.  For each nearby capsule pair it enforces the
        discrete barrier condition ``J_dist dq >= -gamma * (d - d_safe)``.
        Joint/rate boxes are projected after every pass.  A continuous path
        check still validates the result, so numerical linearisation can never
        weaken the stop policy.
        """
        names = list(target)
        nominal, compliance_diag = self._compliant_nominal(start, target)
        nominal_delta = np.asarray(
            [float(nominal[name]) - float(start[name]) for name in names],
            dtype=np.float64,
        )
        evaluation_limit_mm = max(self.activation_distance_mm + 6.0, 10.0)
        start_pairs = self._proximity_map(
            start, max_distance_mm=evaluation_limit_mm
        )
        nominal_pairs = self._proximity_map(
            nominal, max_distance_mm=evaluation_limit_mm
        )
        active_keys = sorted(
            key
            for key in set(start_pairs) | set(nominal_pairs)
            if min(
                float(start_pairs.get(key, {}).get("distance_mm", evaluation_limit_mm)),
                float(nominal_pairs.get(key, {}).get("distance_mm", evaluation_limit_mm)),
            )
            <= self.activation_distance_mm
        )

        constraints: list[tuple[np.ndarray, float, float, dict[str, Any]]] = []
        for pair in active_keys:
            row = start_pairs.get(pair) or nominal_pairs[pair]
            start_distance = float(
                start_pairs.get(pair, {}).get("distance_mm", evaluation_limit_mm)
            )
            gradient = self._distance_gradient(
                start,
                pair,
                row,
                names,
                max_distance_mm=evaluation_limit_mm,
            )
            norm_sq = float(np.dot(gradient, gradient))
            if norm_sq <= 1.0e-12:
                continue
            rhs = -self.cbf_gamma * (
                start_distance - self.safety_clearance_mm
            )
            constraints.append((gradient, rhs, start_distance, row))

        # Lower objective weights let ab/adduction and axial rotation yield
        # first, producing the human-like sidestep requested by the UI policy.
        weights = np.ones(len(names), dtype=np.float64)
        for index, name in enumerate(names):
            lower_name = name.lower()
            if "swing" in lower_name:
                weights[index] = 0.22
            elif "rota" in lower_name and not self._is_fixed_joint(name):
                weights[index] = 0.45
        inverse_weights = 1.0 / weights

        lower_delta = np.empty(len(names), dtype=np.float64)
        upper_delta = np.empty(len(names), dtype=np.float64)
        for index, name in enumerate(names):
            center = float(start[name])
            if self._is_fixed_joint(name):
                lower_delta[index] = upper_delta[index] = -center
                continue
            lo, hi = self._joint_bounds(name)
            lower_delta[index] = max(
                lo - center, -self.avoidance_drive_step_rad
            )
            upper_delta[index] = min(
                hi - center, self.avoidance_drive_step_rad
            )

        delta = np.clip(nominal_delta, lower_delta, upper_delta)
        for _ in range(self.cbf_projection_iterations):
            changed = False
            for gradient, rhs, _, _ in constraints:
                violation = float(rhs - np.dot(gradient, delta))
                if violation <= 1.0e-9:
                    continue
                direction = inverse_weights * gradient
                denominator = float(np.dot(gradient, direction))
                if denominator <= 1.0e-12:
                    continue
                delta += (violation / denominator) * direction
                delta = np.clip(delta, lower_delta, upper_delta)
                changed = True
            if not changed:
                break

        residual_mm = max(
            (
                float(rhs - np.dot(gradient, delta))
                for gradient, rhs, _, _ in constraints
            ),
            default=0.0,
        )
        candidate = self._pose(
            {
                name: float(start[name]) + float(delta[index])
                for index, name in enumerate(names)
            }
        )
        projected, path_diag = self._path_projection(start, candidate)
        output_pairs = self._proximity_map(
            projected, max_distance_mm=evaluation_limit_mm
        )
        min_clearance = min(
            (
                float(row["distance_mm"])
                for row in output_pairs.values()
            ),
            default=evaluation_limit_mm,
        )
        driven = [
            name
            for index, name in enumerate(names)
            if abs(float(delta[index])) > 1.0e-8
        ]
        safety_driven = [
            name
            for index, name in enumerate(names)
            if abs(float(delta[index] - nominal_delta[index])) > 1.0e-6
        ]
        phase = "barrier" if constraints else "approach"
        return projected, {
            **compliance_diag,
            "avoidance_backend": "mujoco_cbf_qp",
            "avoidance_phase": phase,
            "avoidance_active": True,
            "avoidance_drive_step_rad": self.avoidance_drive_step_rad,
            "avoidance_driven_joints": driven,
            "safety_driven_joints": safety_driven,
            "cbf_constraints": len(constraints),
            "cbf_projection_iterations": self.cbf_projection_iterations,
            "cbf_max_residual_mm": round(max(0.0, residual_mm), 8),
            "safety_clearance_mm": self.safety_clearance_mm,
            "activation_distance_mm": self.activation_distance_mm,
            "output_min_clearance_mm": round(float(min_clearance), 6),
            "increment_path_limited": bool(path_diag["limited"]),
        }

    @staticmethod
    def _finger_from_text(value: object) -> str | None:
        text = str(value).lower()
        aliases = {
            "thumb": "thumb",
            "index": "index",
            "middle": "mid",
            "mid": "mid",
            "ring": "ring",
            "little": "pinky",
            "pinky": "pinky",
        }
        for token, canonical in aliases.items():
            if token in text:
                return canonical
        return None

    def _contact_fingers(self, contacts: list[dict]) -> set[str]:
        fingers: set[str] = set()
        for contact in contacts:
            for key in ("g1", "g2"):
                finger = self._finger_from_text(contact.get(key, ""))
                if finger is not None:
                    fingers.add(finger)
        return fingers

    def _joint_bounds(self, name: str) -> tuple[float, float]:
        """Read MJCF limits when available, with conservative hand fallbacks."""
        canonical = str(name)
        if canonical.startswith("right_hand_"):
            canonical = "left_hand_" + canonical[len("right_hand_") :]
        model = getattr(self._filter, "model", None)
        names = getattr(self._filter, "_mjcf_joint_names", None)
        if model is not None and isinstance(names, list) and canonical in names:
            jid = names.index(canonical)
            try:
                if bool(model.jnt_limited[jid]):
                    lo, hi = model.jnt_range[jid]
                    return float(lo), float(hi)
            except Exception:
                pass
        lower_name = canonical.lower()
        if "thumb_swing" in lower_name:
            return 0.0, 2.0
        if "swing" in lower_name:
            return 0.0, 0.696
        if "rota" in lower_name:
            return 0.0, 1.57
        if any(token in lower_name for token in ("_mcp_", "_pip_", "_dip_")):
            return 0.0, 1.57
        return -math.pi, math.pi

    def _clamp_joint(self, name: str, value: float) -> float:
        lo, hi = self._joint_bounds(name)
        return max(lo, min(hi, float(value)))

    @staticmethod
    def _candidate_score(
        candidate: Mapping[str, float],
        target: Mapping[str, float],
    ) -> float:
        """Squared target error; lateral deviations are intentionally softer."""
        score = 0.0
        for name, target_value in target.items():
            delta = float(candidate.get(name, target_value)) - float(target_value)
            lower_name = str(name).lower()
            weight = 0.45 if ("swing" in lower_name or "rota" in lower_name) else 1.0
            score += weight * delta * delta
        return float(score)

    def _path_projection(
        self,
        start: Mapping[str, float],
        target: Mapping[str, float],
    ) -> tuple[dict[str, float], dict[str, Any]]:
        """Project one straight joint-space segment onto its safe prefix."""
        target_penetration, target_contacts = self._evaluate(target)
        max_delta = max(
            (abs(float(target[name]) - float(start[name])) for name in target),
            default=0.0,
        )
        path_steps = min(
            self.max_path_steps,
            max(1, int(math.ceil(max_delta / self.path_step_rad))),
        )

        last_safe_alpha = 0.0
        first_unsafe_alpha: float | None = None
        first_unsafe_contacts: list[dict] = []
        for step in range(1, path_steps + 1):
            alpha = step / path_steps
            candidate = self._interpolate(start, target, alpha)
            penetration, contacts = self._evaluate(candidate)
            if not self._is_safe(penetration):
                first_unsafe_alpha = alpha
                first_unsafe_contacts = contacts
                break
            last_safe_alpha = alpha

        safe_alpha = 1.0
        if first_unsafe_alpha is not None:
            low = last_safe_alpha
            high = float(first_unsafe_alpha)
            for _ in range(self.bisection_iterations):
                middle = 0.5 * (low + high)
                candidate = self._interpolate(start, target, middle)
                penetration, _ = self._evaluate(candidate)
                if self._is_safe(penetration):
                    low = middle
                else:
                    high = middle
            safe_alpha = low

        output = self._interpolate(start, target, safe_alpha)
        return output, {
            "limited": first_unsafe_alpha is not None,
            "safe_fraction": float(safe_alpha),
            "path_steps": path_steps,
            "target_max_penetration_mm": float(target_penetration),
            "target_contacts": target_contacts,
            "blocking_contacts": (
                first_unsafe_contacts
                if first_unsafe_alpha is not None
                else target_contacts
            ),
        }

    def _avoidance_candidates(
        self,
        start: Mapping[str, float],
        target: Mapping[str, float],
        contacts: list[dict],
    ):
        """Yield deterministic nearby poses, lateral DOFs before flexion retreat."""
        affected = self._contact_fingers(contacts)
        names = list(target)
        lateral = [
            name
            for name in names
            if ("swing" in name.lower() or "rota" in name.lower())
            and not self._is_fixed_joint(name)
        ]
        affected_lateral = [
            name
            for name in lateral
            if self._finger_from_text(name) in affected
        ]
        ordered_lateral = affected_lateral + [
            name for name in lateral if name not in affected_lateral
        ]

        def lateral_candidates(joint_names: list[str]):
            lateral_steps = (0.06, 0.12, 0.20, 0.32)
            for name in joint_names:
                for step in lateral_steps:
                    for sign in (-1.0, 1.0):
                        candidate = dict(target)
                        candidate[name] = self._clamp_joint(
                            name, float(target[name]) + sign * step
                        )
                        if candidate[name] != float(target[name]):
                            yield candidate

        # MoveIt request-adapter ordering: first alter only joints belonging to
        # the blocking links.  This keeps the bounded goal-search budget from
        # being consumed by unrelated fingers before flexion retreat is tried.
        yield from lateral_candidates(affected_lateral)

        # Some contacts need two adjacent fingers to share the avoidance.
        pair_names = affected_lateral[:3]
        for i, first in enumerate(pair_names):
            for second in pair_names[i + 1 :]:
                for first_sign, second_sign in ((-1.0, 1.0), (1.0, -1.0)):
                    candidate = dict(target)
                    candidate[first] = self._clamp_joint(
                        first, float(target[first]) + first_sign * 0.14
                    )
                    candidate[second] = self._clamp_joint(
                        second, float(target[second]) + second_sign * 0.14
                    )
                    yield candidate

        # Last local option: relax only the colliding finger(s), not the hand.
        for finger in affected:
            flexion = [
                name
                for name in names
                if self._finger_from_text(name) == finger
                and any(token in name.lower() for token in ("_mcp_", "_pip_", "_dip_"))
            ]
            for fraction in (0.8, 0.6, 0.4):
                candidate = dict(target)
                for name in flexion:
                    base = float(start.get(name, target[name]))
                    candidate[name] = self._clamp_joint(
                        name,
                        base + fraction * (float(target[name]) - base),
                    )
                if flexion:
                    yield candidate

                # Combine a modest flexion retreat with each available lateral DOF.
                for lateral_name in [
                    name
                    for name in ordered_lateral
                    if self._finger_from_text(name) == finger
                ]:
                    for sign in (-1.0, 1.0):
                        combined = dict(candidate)
                        combined[lateral_name] = self._clamp_joint(
                            lateral_name,
                            float(target[lateral_name]) + sign * 0.14,
                        )
                        yield combined

        # Only after exhausting the blocking fingers do we move an unrelated
        # lateral joint.  It remains useful when a broad palm/thumb collision
        # cannot be attributed cleanly from geometry names.
        remaining_lateral = [
            name for name in ordered_lateral if name not in affected_lateral
        ]
        yield from lateral_candidates(remaining_lateral)

    def _find_avoidance(
        self,
        start: Mapping[str, float],
        target: Mapping[str, float],
        contacts: list[dict],
        stop_output: Mapping[str, float],
    ) -> tuple[dict[str, float] | None, dict[str, Any]]:
        best: dict[str, float] | None = None
        best_score = self._candidate_score(stop_output, target)
        evaluations = 0
        seen: set[tuple[tuple[str, float], ...]] = set()
        for candidate in self._avoidance_candidates(start, target, contacts):
            signature = tuple(
                (name, round(float(candidate[name]), 7)) for name in target
            )
            if signature in seen:
                continue
            seen.add(signature)
            evaluations += 1
            if evaluations > self.avoidance_max_evaluations:
                break
            penetration, _ = self._evaluate(candidate)
            if not self._is_safe(penetration):
                continue
            score = self._candidate_score(candidate, target)
            if score >= best_score - 1.0e-12:
                continue
            projected, trace = self._path_projection(start, candidate)
            if trace["limited"]:
                continue
            best = projected
            best_score = score

        return best, {
            "search_evaluations": evaluations,
            "contact_fingers": sorted(self._contact_fingers(contacts)),
            "stop_target_error": round(
                self._candidate_score(stop_output, target), 8
            ),
            "avoid_target_error": (
                None if best is None else round(best_score, 8)
            ),
        }

    def _planning_goal_and_routes(
        self,
        start: Mapping[str, float],
        target: Mapping[str, float],
        contacts: list[dict],
    ) -> tuple[
        dict[str, float] | None,
        list[list[dict[str, float]]],
        dict[str, Any],
    ]:
        """Adapt an invalid goal and build deterministic hand-specific routes.

        MoveIt rejects a colliding goal.  A live retarget stream cannot simply
        drop every such frame, so the lightweight request adapter chooses the
        closest collision-free hand pose first.  Temporary lateral spread is
        represented as an explicit waypoint before flexion.
        """

        target_penetration, _ = self._evaluate(target)
        target_is_safe = self._is_safe(target_penetration)
        candidates: list[tuple[float, dict[str, float]]] = []
        seen: set[tuple[tuple[str, float], ...]] = set()
        evaluations = 0

        for raw_candidate in self._avoidance_candidates(start, target, contacts):
            if evaluations >= self.planner_goal_evaluations:
                break
            candidate = self._pose(raw_candidate)
            signature = tuple(
                (name, round(float(candidate[name]), 7)) for name in target
            )
            if signature in seen:
                continue
            seen.add(signature)
            evaluations += 1
            penetration, _ = self._evaluate(candidate)
            if not self._is_safe(penetration):
                continue
            score = self._candidate_score(candidate, target)
            if self._max_joint_distance(start, candidate) <= 1.0e-7:
                continue
            candidates.append((score, candidate))

        candidates.sort(key=lambda item: item[0])
        if target_is_safe:
            goal = dict(target)
        elif candidates:
            goal = dict(candidates[0][1])
        else:
            return None, [], {
                "goal_search_evaluations": evaluations,
                "goal_candidates": 0,
                "goal_adjusted": True,
            }

        # For an adjusted goal, only routes leading to that exact goal are
        # relevant.  For a safe but edge-blocked target, candidate poses act as
        # detour waypoints before returning to the original target.
        route_candidates = (
            [(self._candidate_score(goal, target), goal)]
            if not target_is_safe
            else candidates[:8]
        )
        preferred_routes: list[list[dict[str, float]]] = []
        for _, candidate in route_candidates:
            lateral_changes = {
                name: float(candidate[name])
                for name in target
                if not self._is_fixed_joint(name)
                and any(token in name.lower() for token in ("swing", "rota"))
                and abs(float(candidate[name]) - float(target[name])) > 1.0e-7
            }
            route: list[dict[str, float]] = []
            if lateral_changes:
                preposition = dict(start)
                preposition.update(lateral_changes)
                preposition = self._pose(preposition)
                penetration, _ = self._evaluate(preposition)
                if self._is_safe(penetration):
                    route.append(preposition)
            if target_is_safe and self._max_joint_distance(candidate, target) > 1.0e-7:
                route.append(dict(candidate))
            if route:
                preferred_routes.append(route)

        return goal, preferred_routes, {
            "goal_search_evaluations": evaluations,
            "goal_candidates": len(candidates),
            "goal_adjusted": not target_is_safe,
            "planned_goal_target_error": round(
                self._candidate_score(goal, target), 8
            ),
        }

    def _clear_global_plan(self) -> None:
        self._global_waypoints = []
        self._global_target = None
        self._global_goal = None
        self._global_plan_method = None
        self._global_plan_diagnostics = {}

    def _start_global_plan(
        self,
        start: Mapping[str, float],
        target: Mapping[str, float],
        contacts: list[dict],
    ) -> tuple[bool, dict[str, Any]]:
        goal, preferred_routes, goal_diag = self._planning_goal_and_routes(
            start,
            target,
            contacts,
        )
        if goal is None:
            self._clear_global_plan()
            return False, {
                **goal_diag,
                "planning_backend": "joint_space_rrt_connect",
                "planning_result": "no_safe_goal",
            }

        bounds = {
            name: ((0.0, 0.0) if self._is_fixed_joint(name) else self._joint_bounds(name))
            for name in target
        }
        contact_fingers = self._contact_fingers(contacts)
        exploration_joints = [
            name
            for name in target
            if not self._is_fixed_joint(name)
            and (
                any(token in name.lower() for token in ("swing", "rota"))
                or self._finger_from_text(name) in contact_fingers
            )
        ]

        def state_is_valid(pose: dict[str, float]) -> bool:
            penetration, _ = self._evaluate(pose)
            return self._is_safe(penetration)

        def edge_is_valid(
            first: dict[str, float],
            second: dict[str, float],
        ) -> bool:
            _, trace = self._path_projection(first, second)
            return not bool(trace["limited"])

        plan = self._joint_planner.plan(
            start,
            goal,
            bounds=bounds,
            state_is_valid=state_is_valid,
            edge_is_valid=edge_is_valid,
            preferred_routes=preferred_routes,
            exploration_joints=exploration_joints,
        )
        if plan is None or len(plan.waypoints) < 2:
            self._clear_global_plan()
            return False, {
                **goal_diag,
                "planning_backend": "joint_space_rrt_connect",
                "planning_result": "failed",
            }

        self._clear_global_plan()
        self._global_waypoints = [dict(pose) for pose in plan.waypoints[1:]]
        self._global_target = dict(target)
        self._global_goal = dict(goal)
        self._global_plan_method = plan.method
        self._avoidance_plan_id += 1
        self._global_plan_diagnostics = {
            **goal_diag,
            "planning_backend": "joint_space_rrt_connect",
            "planning_result": plan.method,
            "planning_iterations": plan.iterations,
            "planning_state_checks": plan.state_checks,
            "planning_edge_checks": plan.edge_checks,
            "planning_sampled_states": plan.sampled_states,
            "planning_waypoints": len(plan.waypoints) - 1,
        }
        return True, dict(self._global_plan_diagnostics)

    def _follow_global_plan(
        self,
        start: Mapping[str, float],
        target: Mapping[str, float],
        contacts: list[dict],
    ) -> tuple[dict[str, float] | None, dict[str, Any]]:
        def current_phase() -> str:
            if not self._global_waypoints:
                return "hold"
            if self._global_plan_method == "structured":
                return "sidestep" if len(self._global_waypoints) > 1 else "advance"
            return (
                "global_goal"
                if len(self._global_waypoints) == 1
                else "global_waypoint"
            )

        reused = False
        if self._global_target is not None:
            target_drift = self._max_joint_distance(self._global_target, target)
            reused = target_drift <= self.planner_target_replan_rad
            if not reused:
                self._clear_global_plan()

        if self._global_target is None:
            started, diagnostics = self._start_global_plan(
                start,
                target,
                contacts,
            )
            if not started:
                return None, diagnostics

        while self._global_waypoints and self._max_joint_distance(
            start, self._global_waypoints[0]
        ) <= self.avoidance_waypoint_tolerance_rad:
            self._global_waypoints.pop(0)

        if self._global_waypoints:
            desired = dict(self._global_waypoints[0])
            phase = current_phase()
        elif self._global_goal is not None:
            desired = dict(self._global_goal)
            phase = "hold"
        else:
            return None, {
                "planning_backend": "joint_space_rrt_connect",
                "planning_result": "lost_plan",
            }

        # The actual Ruckig state may lag behind the camera-rate planner.  Join
        # it back to the next global waypoint only through a validated edge.
        _, desired_trace = self._path_projection(start, desired)
        if desired_trace["limited"]:
            self._clear_global_plan()
            started, diagnostics = self._start_global_plan(
                start,
                target,
                contacts,
            )
            if not started or not self._global_waypoints:
                return None, diagnostics
            desired = dict(self._global_waypoints[0])
            phase = current_phase()
            reused = False

        driven, driven_joints = self._drive_towards(start, desired)
        output, driven_trace = self._path_projection(start, driven)
        if driven_trace["limited"]:
            self._clear_global_plan()
            return None, {
                "planning_backend": "joint_space_rrt_connect",
                "planning_result": "execution_edge_invalid",
            }

        if self._global_waypoints and self._max_joint_distance(
            output, self._global_waypoints[0]
        ) <= self.avoidance_waypoint_tolerance_rad:
            self._global_waypoints.pop(0)

        diagnostics = {
            **self._global_plan_diagnostics,
            "planning_reused": reused,
            "planning_remaining_waypoints": len(self._global_waypoints),
            "avoidance_backend": "moveit_style_hybrid",
            "avoidance_active": True,
            "avoidance_phase": phase,
            "avoidance_driven_joints": driven_joints,
            "avoidance_drive_step_rad": self.avoidance_drive_step_rad,
        }
        return output, diagnostics

    @staticmethod
    def _max_joint_distance(
        first: Mapping[str, float],
        second: Mapping[str, float],
    ) -> float:
        return max(
            (
                abs(float(second[name]) - float(first.get(name, second[name])))
                for name in second
            ),
            default=0.0,
        )

    def _drive_towards(
        self,
        start: Mapping[str, float],
        desired: Mapping[str, float],
    ) -> tuple[dict[str, float], list[str]]:
        """Advance one camera-frame-sized step toward an avoidance waypoint."""
        output: dict[str, float] = {}
        driven: list[str] = []
        step_limit = self.avoidance_drive_step_rad
        for name, desired_value in desired.items():
            current = float(start.get(name, desired_value))
            delta = float(desired_value) - current
            if abs(delta) > step_limit:
                value = current + math.copysign(step_limit, delta)
            else:
                value = float(desired_value)
            if abs(value - current) > 1.0e-9:
                driven.append(name)
            output[name] = value
        return self._pose(output), driven

    def _clear_avoidance(self) -> None:
        self._avoidance_phase = None
        self._avoidance_waypoint = None
        self._avoidance_goal = None
        self._avoidance_detour_values = {}
        self._avoidance_contact_fingers = []
        self._clear_global_plan()

    def _start_avoidance_plan(
        self,
        start: Mapping[str, float],
        target: Mapping[str, float],
        contacts: list[dict],
        stop_output: Mapping[str, float],
    ) -> tuple[bool, dict[str, Any]]:
        alternative, search_diag = self._find_avoidance(
            start,
            target,
            contacts,
            stop_output,
        )
        if alternative is None:
            self._clear_avoidance()
            return False, search_diag

        detour_values = {
            name: float(alternative[name])
            for name in target
            if not self._is_fixed_joint(name)
            and abs(float(alternative[name]) - float(target[name])) > 1.0e-7
        }
        if not detour_values:
            self._clear_avoidance()
            return False, search_diag

        # The critical fix: command the avoidance DOFs first while all other
        # joints remain at the current safe pose.  Only after the real Ruckig
        # pose reaches this waypoint may flexion advance toward the hand pose.
        waypoint = {
            name: float(start.get(name, value))
            for name, value in target.items()
        }
        waypoint.update(detour_values)
        waypoint = self._pose(waypoint)
        _, waypoint_trace = self._path_projection(start, waypoint)
        if waypoint_trace["limited"]:
            self._clear_avoidance()
            return False, search_diag

        self._avoidance_plan_id += 1
        self._avoidance_phase = "sidestep"
        self._avoidance_waypoint = waypoint
        self._avoidance_goal = self._pose(alternative)
        self._avoidance_detour_values = detour_values
        self._avoidance_contact_fingers = list(search_diag["contact_fingers"])
        return True, search_diag

    def _refresh_avoidance_goal(
        self,
        target: Mapping[str, float],
    ) -> dict[str, float]:
        goal = dict(target)
        for name, value in self._avoidance_detour_values.items():
            if name in goal:
                goal[name] = self._clamp_joint(name, value)
        self._avoidance_goal = self._pose(goal)
        return self._avoidance_goal

    def reset(self) -> None:
        self._previous_safe = None
        self._clear_avoidance()
        reset = getattr(self._filter, "reset", None)
        if callable(reset):
            reset()

    def rebase(self, current_values: Mapping[str, float] | None) -> None:
        """Discard stale routes and restart planning at the accepted hand pose.

        Used after tracking reacquisition; unlike reset(), this never restores
        the physics model's warm-up pose or assumes the previous goal was met.
        """
        self._clear_avoidance()
        self._previous_safe = None
        if current_values:
            current = self._pose(current_values)
            penetration, _ = self._evaluate(current)
            if not self._is_safe(penetration):
                raise RuntimeError("reacquisition pose exceeds collision tolerance")
            self._previous_safe = dict(current)
            if self._compliance_filter is not None:
                self._compliance_filter.sync_pose_from_dict(dict(current))

    def project(
        self,
        target_values: Mapping[str, float],
        *,
        current_values: Mapping[str, float] | None = None,
    ) -> tuple[dict[str, float], dict[str, Any]]:
        """Return ``(safe_pose, diagnostics)`` for one retarget output frame.

        ``current_values`` should be the latest post-Ruckig pose.  It prevents
        the camera-rate planner from assuming that a previously commanded
        waypoint has already been reached.
        """
        target = self._pose(target_values)
        if not target:
            return {}, {
                "enabled": True,
                "strategy": self.strategy,
                "limited": False,
                "avoided": False,
                "resolution": "pass",
                "safe_fraction": 1.0,
                "path_steps": 0,
                "target_max_penetration_mm": 0.0,
                "output_max_penetration_mm": 0.0,
                "max_joint_correction_rad": 0.0,
                "contacts": [],
            }

        if current_values:
            current = self._pose(current_values)
            start = {
                name: float(current.get(name, target.get(name, 0.0)))
                for name in target
            }
        elif self._previous_safe is None:
            # q=0 is the model's collision-free open-hand reference.  Starting
            # there also protects the first camera frame from tunnelling.
            start = {name: 0.0 for name in target}
        else:
            start = {
                name: float(self._previous_safe.get(name, value))
                for name, value in target.items()
            }

        start_penetration, _ = self._evaluate(start)
        if start_penetration > self.allowed_penetration_mm + 1.0e-9:
            # This should only be possible when the MJCF zero pose/geometry was
            # edited.  Refuse to call an already-penetrating pose "safe".
            raise RuntimeError(
                "self-collision guard has no safe start pose "
                f"({start_penetration:.3f} mm penetration)"
            )

        stop_output, trace = self._path_projection(start, target)
        target_penetration = float(trace["target_max_penetration_mm"])
        target_contacts = list(trace["target_contacts"])
        blocking_contacts = list(trace["blocking_contacts"])
        limited = bool(trace["limited"])
        safe_alpha = float(trace["safe_fraction"])
        path_steps = int(trace["path_steps"])
        output = stop_output
        avoided = False
        driven_joints: list[str] = []
        if not limited and self._global_target is not None:
            self._clear_global_plan()
        planning_fallback_diag: dict[str, Any] = {}
        avoidance_diag: dict[str, Any] = {
            "search_evaluations": 0,
            "contact_fingers": list(self._avoidance_contact_fingers),
            "stop_target_error": 0.0,
            "avoid_target_error": None,
        }
        if self.strategy == COLLISION_STRATEGY_AVOID and limited:
            planned_output, planning_diag = self._follow_global_plan(
                start,
                target,
                blocking_contacts or target_contacts,
            )
            if planned_output is not None:
                output_penetration, _ = self._evaluate(planned_output)
                self._previous_safe = dict(planned_output)
                max_correction = max(
                    abs(float(target[name]) - float(planned_output[name]))
                    for name in target
                )
                diagnostics = {
                    "enabled": True,
                    "strategy": self.strategy,
                    "limited": True,
                    "avoided": True,
                    "resolution": "avoid",
                    "safe_fraction": round(float(safe_alpha), 6),
                    "path_steps": path_steps,
                    "allowed_penetration_mm": self.allowed_penetration_mm,
                    "target_max_penetration_mm": round(target_penetration, 6),
                    "output_max_penetration_mm": round(
                        output_penetration, 6
                    ),
                    "max_joint_correction_rad": round(max_correction, 8),
                    "contacts": [dict(row) for row in blocking_contacts[:8]],
                    "avoidance_plan_id": self._avoidance_plan_id,
                    "fixed_pinky_rota_rad": 0.0,
                    "contact_fingers": sorted(
                        self._contact_fingers(blocking_contacts)
                    ),
                    "stop_target_error": round(
                        self._candidate_score(stop_output, target), 8
                    ),
                    "avoid_target_error": round(
                        self._candidate_score(planned_output, target), 8
                    ),
                    **planning_diag,
                }
                return planned_output, diagnostics
            planning_fallback_diag = dict(planning_diag)

        if (
            self.strategy == COLLISION_STRATEGY_AVOID
            and limited
            and self._has_distance_backend()
        ):
            cbf_output, cbf_diag = self._cbf_project_increment(start, target)
            progressed = self._max_joint_distance(start, cbf_output) > 1.0e-8
            if progressed:
                self._clear_avoidance()
                self._avoidance_plan_id += 1
                output_penetration, _ = self._evaluate(cbf_output)
                self._previous_safe = dict(cbf_output)
                max_correction = max(
                    abs(float(target[name]) - float(cbf_output[name]))
                    for name in target
                )
                diagnostics = {
                    "enabled": True,
                    "strategy": self.strategy,
                    "limited": True,
                    "avoided": True,
                    "resolution": "avoid",
                    "safe_fraction": round(float(safe_alpha), 6),
                    "path_steps": path_steps,
                    "allowed_penetration_mm": self.allowed_penetration_mm,
                    "target_max_penetration_mm": round(target_penetration, 6),
                    "output_max_penetration_mm": round(
                        output_penetration, 6
                    ),
                    "max_joint_correction_rad": round(max_correction, 8),
                    "contacts": [dict(row) for row in blocking_contacts[:8]],
                    "avoidance_plan_id": self._avoidance_plan_id,
                    "fixed_pinky_rota_rad": 0.0,
                    "contact_fingers": sorted(
                        self._contact_fingers(blocking_contacts)
                    ),
                    "search_evaluations": 0,
                    "stop_target_error": round(
                        self._candidate_score(stop_output, target), 8
                    ),
                    "avoid_target_error": round(
                        self._candidate_score(cbf_output, target), 8
                    ),
                    **planning_fallback_diag,
                    **cbf_diag,
                }
                return cbf_output, diagnostics

        if self.strategy == COLLISION_STRATEGY_AVOID:
            target_is_safe = self._is_safe(target_penetration) and not limited

            # An opened/moved-away human hand may cancel an in-flight detour.
            if self._avoidance_phase is not None and target_is_safe:
                self._avoidance_phase = "release"

            if self._avoidance_phase is None and limited:
                started, avoidance_diag = self._start_avoidance_plan(
                    start,
                    target,
                    blocking_contacts,
                    stop_output,
                )
                if not started:
                    output = stop_output

            phase = self._avoidance_phase
            desired: dict[str, float] | None = None
            if phase == "sidestep" and self._avoidance_waypoint is not None:
                if self._max_joint_distance(start, self._avoidance_waypoint) <= (
                    self.avoidance_waypoint_tolerance_rad
                ):
                    self._avoidance_phase = "advance"
                    phase = "advance"
                else:
                    desired = dict(self._avoidance_waypoint)

            if phase in {"advance", "hold"}:
                goal = self._refresh_avoidance_goal(target)
                goal_penetration, _ = self._evaluate(goal)
                _, goal_trace = self._path_projection(start, goal)
                if not self._is_safe(goal_penetration) or goal_trace["limited"]:
                    # The MediaPipe target moved enough to invalidate the old
                    # corridor. Re-plan once from the actual safe pose.
                    self._clear_avoidance()
                    started, avoidance_diag = self._start_avoidance_plan(
                        start,
                        target,
                        blocking_contacts or target_contacts,
                        stop_output,
                    )
                    if started:
                        desired = dict(self._avoidance_waypoint or start)
                    else:
                        desired = None
                else:
                    desired = goal
                    if self._max_joint_distance(start, goal) <= (
                        self.avoidance_waypoint_tolerance_rad
                    ):
                        self._avoidance_phase = "hold"

            if phase == "release":
                if target_is_safe:
                    desired = dict(target)
                else:
                    self._clear_avoidance()
                    started, avoidance_diag = self._start_avoidance_plan(
                        start,
                        target,
                        blocking_contacts or target_contacts,
                        stop_output,
                    )
                    desired = (
                        dict(self._avoidance_waypoint or start)
                        if started
                        else None
                    )

            if desired is not None and self._avoidance_phase is not None:
                driven, driven_joints = self._drive_towards(start, desired)
                driven_output, driven_trace = self._path_projection(start, driven)
                output = driven_output
                avoided = True
                if driven_trace["limited"]:
                    # Numerical/model changes cannot bypass the fail-safe.
                    output = stop_output
                    driven_joints = []
                elif (
                    self._avoidance_phase == "release"
                    and self._max_joint_distance(output, target)
                    <= self.avoidance_waypoint_tolerance_rad
                ):
                    self._clear_avoidance()

        output_penetration, output_contacts = self._evaluate(output)
        self._previous_safe = dict(output)
        max_correction = max(
            abs(float(target[name]) - float(output[name])) for name in target
        )
        diagnostics: dict[str, Any] = {
            "enabled": True,
            "strategy": self.strategy,
            "limited": limited,
            "avoided": avoided,
            "resolution": (
                "avoid"
                if avoided
                else ("stop" if limited else "pass")
            ),
            "safe_fraction": round(float(safe_alpha), 6),
            "path_steps": path_steps,
            "allowed_penetration_mm": self.allowed_penetration_mm,
            "target_max_penetration_mm": round(target_penetration, 6),
            "output_max_penetration_mm": round(output_penetration, 6),
            "max_joint_correction_rad": round(max_correction, 8),
            "contacts": [dict(row) for row in blocking_contacts[:8]],
            "avoidance_active": self._avoidance_phase is not None,
            "avoidance_phase": self._avoidance_phase,
            "avoidance_plan_id": self._avoidance_plan_id,
            "avoidance_drive_step_rad": self.avoidance_drive_step_rad,
            "avoidance_driven_joints": driven_joints,
            "avoidance_waypoint_distance_rad": (
                None
                if self._avoidance_waypoint is None
                else round(
                    self._max_joint_distance(start, self._avoidance_waypoint), 8
                )
            ),
            "avoidance_goal_distance_rad": (
                None
                if self._avoidance_goal is None
                else round(self._max_joint_distance(start, self._avoidance_goal), 8)
            ),
            "fixed_pinky_rota_rad": 0.0,
            **planning_fallback_diag,
            **avoidance_diag,
        }
        return output, diagnostics


__all__ = [
    "COLLISION_STRATEGIES",
    "COLLISION_STRATEGY_AVOID",
    "COLLISION_STRATEGY_STOP",
    "SelfCollisionGuard",
]
