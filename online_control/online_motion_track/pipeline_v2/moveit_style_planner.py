"""Small joint-space planner used by the light-test collision guard.

The implementation mirrors the useful part of a MoveIt/OMPL pipeline without
requiring ROS 2: states and interpolated edges are checked by the caller's
planning scene, a deterministic RRT-Connect search finds a route when a direct
joint-space segment is blocked, and collision-checked shortcutting removes
unnecessary waypoints before execution.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Mapping, Sequence
import zlib

import numpy as np


Pose = Mapping[str, float]
StateValidator = Callable[[dict[str, float]], bool]
EdgeValidator = Callable[[dict[str, float], dict[str, float]], bool]


@dataclass(frozen=True)
class JointPathPlan:
    """Collision-checked path and bounded planning diagnostics."""

    waypoints: tuple[dict[str, float], ...]
    method: str
    iterations: int
    state_checks: int
    edge_checks: int
    sampled_states: int


@dataclass
class _Tree:
    nodes: list[np.ndarray]
    parents: list[int]
    root_kind: str


class JointSpaceRRTConnectPlanner:
    """Deterministic, bounded RRT-Connect planner for small hand joint spaces."""

    def __init__(
        self,
        *,
        max_iterations: int = 56,
        extension_step_rad: float = 0.14,
        connect_steps: int = 10,
        goal_bias: float = 0.24,
        max_edge_checks: int = 160,
        shortcut_checks: int = 24,
        seed: int = 0xD4A1,
    ) -> None:
        self.max_iterations = max(1, int(max_iterations))
        self.extension_step_rad = max(1.0e-4, float(extension_step_rad))
        self.connect_steps = max(1, int(connect_steps))
        self.goal_bias = max(0.0, min(1.0, float(goal_bias)))
        self.max_edge_checks = max(1, int(max_edge_checks))
        self.shortcut_checks = max(0, int(shortcut_checks))
        self.seed = int(seed) & 0xFFFFFFFF

    @staticmethod
    def _deduplicate(points: Sequence[np.ndarray]) -> list[np.ndarray]:
        output: list[np.ndarray] = []
        for point in points:
            value = np.asarray(point, dtype=np.float64).copy()
            if output and np.max(np.abs(value - output[-1])) <= 1.0e-10:
                continue
            output.append(value)
        return output

    @staticmethod
    def _trace(tree: _Tree, index: int) -> list[np.ndarray]:
        reverse_path: list[np.ndarray] = []
        cursor = int(index)
        while cursor >= 0:
            reverse_path.append(tree.nodes[cursor])
            cursor = tree.parents[cursor]
        reverse_path.reverse()
        return [point.copy() for point in reverse_path]

    def plan(
        self,
        start: Pose,
        goal: Pose,
        *,
        bounds: Mapping[str, tuple[float, float]],
        state_is_valid: StateValidator,
        edge_is_valid: EdgeValidator,
        preferred_routes: Sequence[Sequence[Pose]] = (),
        exploration_joints: Sequence[str] = (),
    ) -> JointPathPlan | None:
        """Plan from ``start`` to ``goal`` within a finite collision budget.

        Preferred routes are tried first.  They let the hand pre-position
        ab/adduction joints before flexion, which is safer than relying on
        independent time profiles to reproduce a diagonal joint-space edge.
        """

        names = tuple(str(name) for name in goal)
        if not names:
            return JointPathPlan(({},), "empty", 0, 0, 0, 0)
        if set(start) != set(names):
            raise ValueError("planner start and goal joint sets differ")

        lower = np.asarray([float(bounds[name][0]) for name in names])
        upper = np.asarray([float(bounds[name][1]) for name in names])
        if np.any(~np.isfinite(lower)) or np.any(~np.isfinite(upper)):
            raise ValueError("planner bounds must be finite")
        if np.any(lower > upper):
            raise ValueError("planner lower bound exceeds upper bound")

        start_q = np.asarray([float(start[name]) for name in names])
        goal_q = np.asarray([float(goal[name]) for name in names])
        start_q = np.clip(start_q, lower, upper)
        goal_q = np.clip(goal_q, lower, upper)

        state_checks = 0
        edge_checks = 0

        def as_pose(q: np.ndarray) -> dict[str, float]:
            return {name: float(q[index]) for index, name in enumerate(names)}

        def valid_state(q: np.ndarray) -> bool:
            nonlocal state_checks
            state_checks += 1
            return bool(state_is_valid(as_pose(q)))

        def valid_edge(first: np.ndarray, second: np.ndarray) -> bool:
            nonlocal edge_checks
            if edge_checks >= self.max_edge_checks:
                return False
            edge_checks += 1
            return bool(edge_is_valid(as_pose(first), as_pose(second)))

        if not valid_state(start_q) or not valid_state(goal_q):
            return None

        # MoveIt-style request adapters are represented by deterministic
        # preferred routes.  For hands this usually means spreading first,
        # flexing second, and only then releasing the temporary spread.
        for route in preferred_routes:
            points = [start_q]
            route_valid = True
            for pose in route:
                try:
                    point = np.asarray([float(pose[name]) for name in names])
                except (KeyError, TypeError, ValueError):
                    route_valid = False
                    break
                point = np.clip(point, lower, upper)
                if not valid_state(point):
                    route_valid = False
                    break
                points.append(point)
            if not route_valid:
                continue
            points.append(goal_q)
            points = self._deduplicate(points)
            if all(
                valid_edge(points[index], points[index + 1])
                for index in range(len(points) - 1)
            ):
                poses = tuple(as_pose(point) for point in points)
                return JointPathPlan(
                    poses,
                    "structured",
                    0,
                    state_checks,
                    edge_checks,
                    0,
                )

        if valid_edge(start_q, goal_q):
            return JointPathPlan(
                (as_pose(start_q), as_pose(goal_q)),
                "direct",
                0,
                state_checks,
                edge_checks,
                0,
            )

        span = np.maximum(upper - lower, 1.0e-3)
        metric_scale = np.maximum(span, 0.25)
        exploration_indices = [
            names.index(name)
            for name in exploration_joints
            if name in names and upper[names.index(name)] > lower[names.index(name)]
        ]
        movable_indices = [
            index for index in range(len(names)) if upper[index] > lower[index]
        ]
        if not exploration_indices:
            exploration_indices = movable_indices

        seed_payload = np.concatenate((start_q, goal_q)).round(6).tobytes()
        rng_seed = (zlib.crc32(seed_payload) ^ self.seed) & 0xFFFFFFFF
        rng = np.random.default_rng(rng_seed)

        def nearest(tree: _Tree, target_q: np.ndarray) -> int:
            distances = [
                float(np.linalg.norm((node - target_q) / metric_scale))
                for node in tree.nodes
            ]
            return int(np.argmin(distances))

        def steer(source: np.ndarray, target_q: np.ndarray) -> np.ndarray:
            delta = target_q - source
            max_delta = float(np.max(np.abs(delta)))
            if max_delta <= self.extension_step_rad:
                return np.clip(target_q, lower, upper)
            return np.clip(
                source + delta * (self.extension_step_rad / max_delta),
                lower,
                upper,
            )

        def extend(tree: _Tree, target_q: np.ndarray) -> tuple[int | None, bool]:
            parent_index = nearest(tree, target_q)
            new_q = steer(tree.nodes[parent_index], target_q)
            if np.max(np.abs(new_q - tree.nodes[parent_index])) <= 1.0e-10:
                return parent_index, True
            if not valid_state(new_q):
                return None, False
            if not valid_edge(tree.nodes[parent_index], new_q):
                return None, False
            tree.nodes.append(new_q)
            tree.parents.append(parent_index)
            new_index = len(tree.nodes) - 1
            reached = np.max(np.abs(new_q - target_q)) <= 1.0e-10
            return new_index, reached

        def connect(tree: _Tree, target_q: np.ndarray) -> tuple[int | None, bool]:
            last_index: int | None = None
            for _ in range(self.connect_steps):
                last_index, reached = extend(tree, target_q)
                if last_index is None or reached:
                    return last_index, reached
                if edge_checks >= self.max_edge_checks:
                    break
            return last_index, False

        start_tree = _Tree([start_q], [-1], "start")
        goal_tree = _Tree([goal_q], [-1], "goal")
        tree_a = start_tree
        tree_b = goal_tree
        sampled_states = 0

        for iteration in range(1, self.max_iterations + 1):
            if edge_checks >= self.max_edge_checks:
                break
            if rng.random() < self.goal_bias:
                sample = tree_b.nodes[0].copy()
            else:
                # Most samples stay near the start-goal corridor and perturb a
                # few useful finger DOFs.  Occasional full samples preserve the
                # ability to escape a deeper configuration-space obstacle.
                if rng.random() < 0.78:
                    alpha = float(rng.random())
                    sample = start_q + alpha * (goal_q - start_q)
                    count = min(
                        len(exploration_indices),
                        max(1, int(rng.integers(1, min(4, len(exploration_indices)) + 1))),
                    )
                    selected = rng.choice(
                        exploration_indices, size=count, replace=False
                    )
                    for index in np.atleast_1d(selected):
                        idx = int(index)
                        sample[idx] = float(rng.uniform(lower[idx], upper[idx]))
                else:
                    sample = rng.uniform(lower, upper)
                sample = np.clip(sample, lower, upper)
            sampled_states += 1

            new_index, _ = extend(tree_a, sample)
            if new_index is not None:
                meet_q = tree_a.nodes[new_index]
                other_index, connected = connect(tree_b, meet_q)
                if connected and other_index is not None:
                    path_a = self._trace(tree_a, new_index)
                    path_b = self._trace(tree_b, other_index)
                    if tree_a.root_kind == "start":
                        path = path_a + list(reversed(path_b))[1:]
                    else:
                        path = path_b + list(reversed(path_a))[1:]
                    path = self._deduplicate(path)

                    # Collision-checked greedy shortcutting, equivalent to the
                    # path-shortening stage commonly applied after sampling.
                    shortcut_budget = self.shortcut_checks
                    shortened: list[np.ndarray] = [path[0]]
                    cursor = 0
                    while cursor < len(path) - 1:
                        chosen = cursor + 1
                        for candidate in range(len(path) - 1, cursor + 1, -1):
                            if shortcut_budget <= 0:
                                break
                            shortcut_budget -= 1
                            if valid_edge(path[cursor], path[candidate]):
                                chosen = candidate
                                break
                        shortened.append(path[chosen])
                        cursor = chosen
                    poses = tuple(as_pose(point) for point in shortened)
                    return JointPathPlan(
                        poses,
                        "rrt_connect",
                        iteration,
                        state_checks,
                        edge_checks,
                        sampled_states,
                    )

            tree_a, tree_b = tree_b, tree_a

        return None


__all__ = ["JointPathPlan", "JointSpaceRRTConnectPlanner"]
