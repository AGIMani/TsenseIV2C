from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Any

import daqp
import numpy as np
import yaml

from .fk import RobotKeypointFK
from .landmarks import (
    BONE_DIRECTION_EDGES,
    FINGER_CHAINS,  # noqa: F401
    INDEX_MCP_ID,
    MIDDLE_MCP_ID,
    NUM_LANDMARKS,
    PINCH_PAIRS,
    PINKY_MCP_ID,
    ROOTED_FINGER_CHAINS,
    SKELETON_EDGES,  # noqa: F401
    WRIST_ID,
    palm_width_m,
    points21,
    points_from_record,
)

try:
    from scipy.spatial import Delaunay
except Exception as exc:  # pragma: no cover - import-time operator error
    raise RuntimeError("TsenseI_retargeting requires scipy: pip install scipy") from exc


PACKAGE_ROOT = Path(__file__).resolve().parent
# Live / monorepo: single robot package under assets/hands/TsenseI_hand
# (URDF viewer + MuJoCo + keypoints). Standalone copies must pass robot_root.
_REPO_HAND_ROOT = PACKAGE_ROOT.parent / "assets" / "hands" / "TsenseI_hand"
ROBOTS_ROOT = PACKAGE_ROOT / "robots"
DEFAULT_ROBOT_ROOT = _REPO_HAND_ROOT if _REPO_HAND_ROOT.is_dir() else ROBOTS_ROOT / "left_hand"
PINCH_FINGERS = ("index", "middle", "ring", "pinky")
_PINCH_TIPS = np.array([tip for _, tip in PINCH_PAIRS], dtype=int)

JOINT_NAMES: tuple[str, ...] = (
    "left_hand_thumb_rota_joint",
    "left_hand_thumb_swing_joint",
    "left_hand_thumb_MCP_joint",
    "left_hand_thumb_PIP_joint",
    "left_hand_index_swing_joint",
    "left_hand_index_MCP_joint",
    "left_hand_index_PIP_joint",
    "left_hand_index_DIP_joint",
    "left_hand_mid_swing_joint",
    "left_hand_mid_MCP_joint",
    "left_hand_mid_PIP_joint",
    "left_hand_mid_DIP_joint",
    "left_hand_ring_swing_joint",
    "left_hand_ring_MCP_joint",
    "left_hand_ring_PIP_joint",
    "left_hand_ring_DIP_joint",
    "left_hand_pinky_rota_joint",
    "left_hand_pinky_swing_joint",
    "left_hand_pinky_MCP_joint",
    "left_hand_pinky_PIP_joint",
    "left_hand_pinky_DIP_joint",
)

# ---------------------------------------------------------------------------
# 关键点标定（robot_keypoints_21.yaml）
# ---------------------------------------------------------------------------
# 1. 确认 robot_root/eval_config.yaml 中 urdf_path、active_joints、base_link 与实机 URDF 一致。
# 2. 在零位姿态下编辑 robot_keypoints_21.yaml：每个 MediaPipe id(0..20) 指定 link + local_xyz
#    （米，相对该 link 坐标系）。腕=0，MCP/PIP/DIP/指尖贴到对应解剖位置；指尖 local_xyz 需目视对齐接触点。
# 3. 实机验证：张开看伸展、握拳看弯曲、捏合看拇指；偏差则微调对应点 local_xyz，并标
#    annotation_status: manually_adjusted。
# 4. 尺度由算法用 IndexMCP(5)–PinkyMCP(17) 对齐机器人掌宽，不要靠整体缩放 YAML 来“拉大/缩小手”。
# Live 工程默认 robot_root = assets/hands/TsenseI_hand（与 URDF 可视化一致）。
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class TopoRetargetConfig:
    # --- 资产路径 ---
    # 机器人资源根目录（含 eval_config.yaml / URDF / robot_keypoints_21.yaml）
    robot_root: str | Path = DEFAULT_ROBOT_ROOT
    # 覆盖 eval_config 中的 URDF 路径；None 则用 eval_config.urdf_path
    urdf_path: str | Path | None = None
    # 覆盖关键点 YAML；None 则用 eval_config.keypoint_config
    keypoints_path: str | Path | None = None
    # 覆盖 DOF 关节名顺序；None 则用 eval_config.active_joints / 默认 JOINT_NAMES
    joint_names: tuple[str, ...] | None = None
    # Optional model neutral pose; absent keeps the existing zero-pose behavior.
    initial_joint_values: dict[str, float] | None = None

    # --- 求解开关 ---
    # True：骨向初始化后再跑交互网格拓扑优化；False：只用骨向初始化结果（更快、更跟手，精度略低）
    use_topo_optimization: bool = True

    # 拇指与四指分别接触：各对独立按当前输入 5–17 掌宽计算权重。
    pinch_enabled: bool = True
    pinch_full_ratio: float = 0.08
    pinch_start_ratio: float = 0.50
    pinch_weight: float = 65.0

    # --- 初始化阶段权重（骨方向 warm-start）---
    # 相对骨方向残差权重：越大越贴人手骨骼朝向
    lambda_warm: float = 2.0
    # 相对上一帧 q 的逆解先验：用于欠观测关节选解，不属于下发轨迹滤波
    lambda_smooth: float = 2.5

    # --- 拓扑优化阶段权重 ---
    # 交互网格（Delaunay Laplacian）残差权重：越大越保持人手点云拓扑形状
    lambda_im: float = 12.0
    # 交互边衰减 κ：edge_w = exp(-κ ‖pi-pj‖² / palm_width²)；越大远端边越弱
    kappa: float = 30.0
    # Delaunay 邻接/边权跨帧稳定系数：1.0=完全使用当前帧；0.5=当前/历史各半。
    # 只稳定交互图，不平均 landmark 或关节角；避免 Qhull 边翻转把微小点变化放大为关节尖峰。
    interaction_graph_ema_alpha: float = 0.5
    # 拓扑阶段骨方向残差权重
    lambda_bone: float = 0.5
    # 拇指根部（腕→CMC、CMC→MCP）骨方向残差倍率；1.0 与 Quest v3 原版等权
    thumb_root_bone_weight: float = 1.0
    # 拉向初始化 q_init 的权重：防止拓扑阶段漂得太远
    lambda_init: float = 0.1
    # 拉向上一帧 q_prev 的逆解正则：抑制21点欠约束导致的错误解分支
    lambda_reg: float = 2.5

    # --- 迭代与收敛 ---
    # 初始化阶段 DAQP-SQP 最大迭代次数
    init_max_iters: int = 1
    # 拓扑阶段 DAQP-SQP 最大迭代次数
    topo_max_iters: int = 2
    # 步长 ∞-范数低于此值视为收敛
    ftol: float = 1.0e-5
    # 有限差分雅可比步长（use_analytic_jacobian=False 或解析失败时）
    finite_difference_eps: float = 1.0e-5

    # --- DAQP / QP 数值 ---
    # Hessian 阻尼：JᵀJ + d·I，增大更稳、更钝
    daqp_hessian_damping: float = 1.0e-6
    # 初始化阶段每步关节角增量上限（弧度）
    daqp_init_step_limit_rad: float = 0.35
    # 拓扑阶段每步关节角增量上限（弧度）
    daqp_topo_step_limit_rad: float = 0.15
    # 交互网格软 L2：√(‖ΔL‖² + ε²)，避免零范数奇异
    interaction_l2_epsilon_m: float = 1.0e-7
    # True：优先 Pinocchio 解析雅可比；失败则回退有限差分
    use_analytic_jacobian: bool = True
    # Explicit portable backend for the web deployment. Existing auto behavior
    # keeps the Pinocchio path used by desktop sessions.
    kinematics_backend: str = "auto"

    def __post_init__(self) -> None:
        values = (self.pinch_full_ratio, self.pinch_start_ratio, self.pinch_weight)
        if not np.isfinite(values).all():
            raise ValueError("pinch parameters must be finite")
        if not 0 <= self.pinch_full_ratio < self.pinch_start_ratio <= 1.5:
            raise ValueError("pinch ratios require 0 <= full < start <= 1.5")
        if not 0 <= self.pinch_weight <= 1000:
            raise ValueError("pinch_weight must be between 0 and 1000")


def pinch_observation(source_points: np.ndarray, config: TopoRetargetConfig) -> dict[str, Any]:
    """Independent thumb/finger intent in the same frame, before robot scaling.

    The scalar fields retain their original thumb/index meaning for callers;
    ``pairs`` contains all four contacts, with no cross-finger gating or history.
    """
    points = points21(source_points)
    width = palm_width_m(points)
    gaps = np.linalg.norm(points[4] - points[_PINCH_TIPS], axis=1)
    ratios = gaps / width
    u = np.clip((config.pinch_start_ratio - ratios) /
                (config.pinch_start_ratio - config.pinch_full_ratio), 0.0, 1.0)
    intent = u * u * (3.0 - 2.0 * u)
    effective = bool(config.pinch_enabled and config.use_topo_optimization and config.pinch_weight > 0)
    pairs = {name: {"pair": list(pair), "input_distance_m": float(gap),
                    "ratio": float(ratio), "intent_alpha": float(alpha),
                    "alpha": float(alpha) if effective else 0.0}
             for name, pair, gap, ratio, alpha in zip(PINCH_FINGERS, PINCH_PAIRS, gaps, ratios, intent)}
    return {"enabled": bool(config.pinch_enabled), "effective": effective,
            "palm_width_m": width, "full_threshold_m": config.pinch_full_ratio * width,
            "start_threshold_m": config.pinch_start_ratio * width,
            "weight": float(config.pinch_weight), **pairs["index"], "pairs": pairs}


@dataclass(frozen=True)
class TopoRetargetResult:
    joint_values: dict[str, float]
    robot_keypoints_21: list[list[float]]
    source_keypoints_21_robot_scale: list[list[float]]
    success: bool
    loss: dict[str, float]
    diagnostics: dict[str, Any]

    def to_dict(self) -> dict[str, Any]:
        return {
            "joint_values": self.joint_values,
            "robot_keypoints_21": self.robot_keypoints_21,
            "source_keypoints_21_robot_scale": self.source_keypoints_21_robot_scale,
            "success": self.success,
            "loss": self.loss,
            "diagnostics": self.diagnostics,
        }


class TopoRetargeter:
    def __init__(self, config: TopoRetargetConfig | None = None):
        self.config = config or TopoRetargetConfig()
        self.robot_root, self.urdf_path, self.keypoint_path, joint_names = resolve_robot_assets(self.config)
        self.fk = RobotKeypointFK(self.urdf_path, self.keypoint_path)
        self.joint_names = list(joint_names)
        self._validate_joint_names()
        self.lower, self.upper = self._joint_limit_arrays()
        self.init_q = np.zeros(len(self.joint_names), dtype=float)
        if self.config.initial_joint_values is not None:
            values = self.config.initial_joint_values
            if set(values) != set(self.joint_names):
                raise ValueError("initial_joint_values must name every retarget joint")
            self.init_q = np.array([values[name] for name in self.joint_names], dtype=float)
            if (not np.isfinite(self.init_q).all() or np.any(self.init_q < self.lower)
                    or np.any(self.init_q > self.upper)):
                raise ValueError("initial_joint_values must be finite and inside joint limits")
        self.robot_palm_width_m = self._robot_palm_width_m()
        self.robot_topo_frame = self._robot_topo_frame()
        self._analytic_kinematics, self._analytic_kinematics_error = self._make_analytic_kinematics()
        self.previous_init_q: np.ndarray | None = None
        self.previous_q: np.ndarray | None = None
        self._previous_interaction_weight_matrix: np.ndarray | None = None
        self._clear_pinch()

    def retarget(
        self,
        joint_positions_xyz: np.ndarray,
        *,
        joint_orientations_xyzw: np.ndarray | None = None,
        joint_valid: np.ndarray | None = None,
        previous_joint_values: dict[str, float] | np.ndarray | None = None,
        include_artifacts: bool = True,
    ) -> TopoRetargetResult:
        del joint_orientations_xyzw, joint_valid
        self._clear_pinch()
        source_raw = points21(joint_positions_xyz)
        pinch = pinch_observation(source_raw, self.config)
        gains = np.sqrt(self.config.pinch_weight * np.array(
            [pair["alpha"] for pair in pinch["pairs"].values()])) / self.robot_palm_width_m
        active = gains > 0
        self._pinch_tips = _PINCH_TIPS[active]
        self._pinch_gains = gains[active, None]
        source_robot_scale = self._source_points_in_robot_scale(source_raw)
        source_topo_points = _canonical_hand_points(source_robot_scale)
        source_bone_terms = _relative_bone_terms(source_robot_scale)
        candidate_graph = _build_interaction_graph(
            source_topo_points,
            self.config,
            distance_scale_m=self.robot_palm_width_m,
        )
        graph = self._stabilize_interaction_graph(candidate_graph)
        source_laplacian = _laplacian_coordinates(source_topo_points, graph)

        q_init, init_info = self._relative_bone_direction_initialization(source_bone_terms)
        q_prev = self._previous_q(previous_joint_values, q_init)
        if self.config.use_topo_optimization:
            q_opt, solve_info = self._solve_with_daqp(
                q_init=q_init,
                q_prev=q_prev,
                source_laplacian=source_laplacian,
                source_bone_terms=source_bone_terms,
                graph=graph,
            )
        else:
            q_opt = q_init.copy()
            solve_info = {
                "success": bool(init_info["success"]),
                "message": "topology optimization disabled; using q_init",
                "num_iters": 0,
                "objective_evals": 0,
                "qp_solve_time_s": 0.0,
                "qp_iterations": 0,
                "line_search_backtracks": 0,
                "converged": True,
                "initial_objective": None,
                "final_objective": None,
                "objective_improvement": None,
                "last_qp_exitflag": None,
            }
        q_opt = np.clip(q_opt, self.lower, self.upper)
        self.previous_init_q = q_init.copy()
        self.previous_q = q_opt.copy()

        # GUI实时控制只消费 joint_values/solver diagnostics。21点结果、round/list
        # 转换与完整loss会额外触发FK/残差计算，仅在离线分析或全链路调试时生成。
        if include_artifacts:
            robot_points = np.round(self._robot_points_palm_m(q_opt), 10).tolist()
            gaps = np.linalg.norm(np.asarray(robot_points)[4] - np.asarray(robot_points)[_PINCH_TIPS], axis=1)
            for pair, gap in zip(pinch["pairs"].values(), gaps):
                pair["output_distance_m"] = float(gap)
            pinch["output_distance_m"] = pinch["pairs"]["index"]["output_distance_m"]
            source_points = np.round(source_robot_scale, 10).tolist()
            loss = self._loss_breakdown(
                q_opt,
                q_init,
                q_prev,
                source_laplacian,
                source_bone_terms,
                graph,
            )
        else:
            robot_points = []
            source_points = []
            loss = {}
        return TopoRetargetResult(
            joint_values=self._q_to_dict(q_opt),
            robot_keypoints_21=robot_points,
            source_keypoints_21_robot_scale=source_points,
            success=bool(solve_info["success"]),
            loss=loss,
            diagnostics={
                "pinch": pinch,
                "algorithm": "TsenseI_mediapipe_toporetarget",
                "solver": "daqp_sqp" if self.config.use_topo_optimization else "relative_bone_direction_init",
                "initialization": init_info,
                "message": str(solve_info["message"]),
                "init_num_iters": int(init_info["num_iters"]),
                "init_objective_evals": int(init_info["objective_evals"]),
                "init_qp_solve_time_s": float(init_info["qp_solve_time_s"]),
                "init_qp_iterations": int(init_info["qp_iterations"]),
                "init_converged": bool(init_info["converged"]),
                "init_initial_objective": init_info["initial_objective"],
                "init_final_objective": init_info["final_objective"],
                "init_objective_improvement": init_info["objective_improvement"],
                "init_last_qp_exitflag": init_info["last_qp_exitflag"],
                "topo_num_iters": int(solve_info["num_iters"]),
                "topo_objective_evals": int(solve_info["objective_evals"]),
                "topo_qp_solve_time_s": float(solve_info["qp_solve_time_s"]),
                "topo_qp_iterations": int(solve_info["qp_iterations"]),
                "topo_line_search_backtracks": int(solve_info["line_search_backtracks"]),
                "topo_converged": bool(solve_info["converged"]),
                "topo_initial_objective": solve_info["initial_objective"],
                "topo_final_objective": solve_info["final_objective"],
                "topo_objective_improvement": solve_info["objective_improvement"],
                "topo_last_qp_exitflag": solve_info["last_qp_exitflag"],
                "topo_initial_guess": solve_info.get("initial_guess"),
                "delaunay_ok": graph.delaunay_ok,
                "num_vertices": 21,
                "num_edges": len(graph.edges),
                "candidate_num_edges": len(candidate_graph.edges),
                "interaction_graph_stabilized": bool(
                    float(self.config.interaction_graph_ema_alpha) < 1.0
                ),
                "parameters": {
                    "pinch_enabled": self.config.pinch_enabled,
                    "pinch_full_ratio": self.config.pinch_full_ratio,
                    "pinch_start_ratio": self.config.pinch_start_ratio,
                    "pinch_weight": self.config.pinch_weight,
                    "use_topo_optimization": self.config.use_topo_optimization,
                    "init_max_iters": self.config.init_max_iters,
                    "topo_max_iters": self.config.topo_max_iters,
                    "lambda_warm": self.config.lambda_warm,
                    "lambda_smooth": self.config.lambda_smooth,
                    "lambda_im": self.config.lambda_im,
                    "kappa": self.config.kappa,
                    "interaction_graph_ema_alpha": self.config.interaction_graph_ema_alpha,
                    "lambda_bone": self.config.lambda_bone,
                    "lambda_init": self.config.lambda_init,
                    "lambda_reg": self.config.lambda_reg,
                    "daqp_hessian_damping": self.config.daqp_hessian_damping,
                    "daqp_init_step_limit_rad": self.config.daqp_init_step_limit_rad,
                    "daqp_topo_step_limit_rad": self.config.daqp_topo_step_limit_rad,
                    "interaction_l2_epsilon_m": self.config.interaction_l2_epsilon_m,
                    "use_analytic_jacobian": self.config.use_analytic_jacobian,
                    "analytic_jacobian_available": self._analytic_kinematics is not None,
                    "analytic_jacobian_error": self._analytic_kinematics_error,

                    "residual_jacobian": (
                        "analytic_chain"
                        if self._analytic_kinematics is not None
                        else "finite_difference"
                    ),
                    "interaction_distance_scale_m": self.robot_palm_width_m,
                    "interaction_frame": "palm_canonical",
                },
            },
        )

    def reset(self) -> None:
        self._clear_pinch()
        self.previous_init_q = None
        self.previous_q = None
        self._previous_interaction_weight_matrix = None

    def _clear_pinch(self) -> None:
        self._pinch_tips = np.empty(0, dtype=int)
        self._pinch_gains = np.empty((0, 1), dtype=float)

    def _pinch_residual(self, points: np.ndarray) -> np.ndarray:
        return (self._pinch_gains * (points[4] - points[self._pinch_tips])).ravel()

    def _stabilize_interaction_graph(self, candidate: "InteractionGraph") -> "InteractionGraph":
        """Continuously blend Delaunay weights so edge flips cannot create an objective jump.

        The current source points are still used immediately by the Laplacian and bone
        residuals.  Only the discrete graph/normalized neighbor weights carry state.
        This leaves landmark latency, objective coefficients and solver iteration counts
        unchanged while making the interaction residual continuous across Qhull flips.
        """
        current = np.asarray(candidate.weight_matrix, dtype=float)
        alpha = float(np.clip(self.config.interaction_graph_ema_alpha, 0.0, 1.0))
        previous = self._previous_interaction_weight_matrix
        if previous is None or previous.shape != current.shape or alpha >= 1.0:
            blended = current.copy()
        elif alpha <= 0.0:
            blended = previous.copy()
        else:
            blended = (1.0 - alpha) * previous + alpha * current
        graph = _interaction_graph_from_weight_matrix(
            blended,
            delaunay_ok=candidate.delaunay_ok,
        )
        self._previous_interaction_weight_matrix = np.asarray(
            graph.weight_matrix,
            dtype=float,
        ).copy()
        return graph

    def _validate_joint_names(self) -> None:
        available = set(self.fk.model.joint_names())
        missing = [name for name in self.joint_names if name not in available]
        if missing:
            raise ValueError(
                "topo retarget joint_names are not present in the robot URDF: "
                + ", ".join(missing)
            )

    def _joint_limit_arrays(self) -> tuple[np.ndarray, np.ndarray]:
        limits = self.fk.model.joint_limits()
        lower = []
        upper = []
        for name in self.joint_names:
            lo, hi = limits[name]
            lower.append(-np.pi if lo is None else float(lo))
            upper.append(np.pi if hi is None else float(hi))
        return np.asarray(lower, dtype=float), np.asarray(upper, dtype=float)

    def _make_analytic_kinematics(self) -> tuple[Any | None, str | None]:
        if not bool(self.config.use_analytic_jacobian):
            return None, "disabled"
        if self.config.kinematics_backend == "numpy":
            from .fk.numpy_kinematics import NumpyKeypointKinematics
            return NumpyKeypointKinematics(self.urdf_path, self.fk.keypoints,
                                           self.joint_names, self.fk.model), None
        if self.config.kinematics_backend not in ("auto", "pinocchio"):
            raise ValueError("Unknown kinematics backend")
        try:
            return _PinocchioKeypointKinematics(
                self.urdf_path,
                self.fk.keypoints,
                self.joint_names,
                self.fk.model,
            ), None
        except Exception as exc:  # pragma: no cover - depends on optional Pinocchio/runtime URDF support
            return None, str(exc)

    def _robot_palm_width_m(self) -> float:
        points = self.fk.compute(self._q_to_dict(self.init_q))
        return palm_width_m(points)

    def _robot_topo_frame(self) -> np.ndarray:
        return _hand_frame(self._robot_points_palm_m(self.init_q))

    def _source_points_in_robot_scale(self, source_raw: np.ndarray) -> np.ndarray:
        palm_width = palm_width_m(source_raw)
        return (source_raw - source_raw[WRIST_ID].reshape(1, 3)) / palm_width * self.robot_palm_width_m

    def _robot_points_palm_m(self, q: np.ndarray) -> np.ndarray:
        # 目标函数和线搜索只需要关键点位置。Pinocchio 可用时，
        # 避免慢速的 Python URDF FK 以及随即丢弃的 Jacobian 计算。
        analytic = getattr(self, "_analytic_kinematics", None)
        if analytic is not None:
            points = analytic.compute_points(q)
            return points - points[WRIST_ID].reshape(1, 3)
        points = self.fk.compute(self._q_to_dict(q))
        return points - points[WRIST_ID].reshape(1, 3)

    def _robot_points_and_jacobian_palm_m(self, q: np.ndarray) -> tuple[np.ndarray, np.ndarray] | None:
        if self._analytic_kinematics is None:
            return None
        points, jacobian = self._analytic_kinematics.compute_points_and_jacobian(q)
        wrist = points[WRIST_ID].reshape(1, 3)
        wrist_jacobian = jacobian[WRIST_ID].reshape(1, 3, jacobian.shape[2])
        return points - wrist, jacobian - wrist_jacobian

    def _robot_points_topo_m(self, q: np.ndarray) -> np.ndarray:
        return _canonical_hand_points(self._robot_points_palm_m(q), frame=self.robot_topo_frame)

    def _previous_q(self, previous_joint_values: dict[str, float] | np.ndarray | None, q_init: np.ndarray) -> np.ndarray:
        if previous_joint_values is not None:
            if isinstance(previous_joint_values, dict):
                return np.asarray([float(previous_joint_values.get(name, q_init[idx])) for idx, name in enumerate(self.joint_names)], dtype=float)
            return np.asarray(previous_joint_values, dtype=float).reshape(-1)
        if self.previous_q is not None:
            return self.previous_q.copy()
        return q_init.copy()

    def _solve_with_daqp(
        self,
        *,
        q_init: np.ndarray,
        q_prev: np.ndarray,
        source_laplacian: np.ndarray,
        source_bone_terms: dict[tuple[int, int], np.ndarray],
        graph: "InteractionGraph",
    ) -> tuple[np.ndarray, dict[str, Any]]:
        q_from_init = np.clip(np.asarray(q_init, dtype=float).copy(), self.lower, self.upper)
        q_from_prev = np.clip(np.asarray(q_prev, dtype=float).copy(), self.lower, self.upper)
        q = q_from_init
        initial_guess = "q_init"
        initial_guess_evals = 0
        if self.previous_q is not None:
            init_residual = self._weighted_residual(q_from_init, q_init, q_prev, source_laplacian, source_bone_terms, graph)
            prev_residual = self._weighted_residual(q_from_prev, q_init, q_prev, source_laplacian, source_bone_terms, graph)
            initial_guess_evals = 2
            if _squared_residual_objective(prev_residual) <= _squared_residual_objective(init_residual):
                q = q_from_prev
                initial_guess = "q_prev"
        max_iters = max(1, int(self.config.topo_max_iters))
        eps = max(1.0e-8, float(self.config.finite_difference_eps))
        damping = max(1.0e-12, float(self.config.daqp_hessian_damping))
        step_limit = max(float(self.config.ftol), float(self.config.daqp_topo_step_limit_rad))
        objective_evals = initial_guess_evals
        qp_solve_time_s = 0.0
        qp_iterations = 0
        line_search_backtracks = 0
        success = True
        message = "DAQP SQP converged"
        converged = False
        initial_objective: float | None = None
        final_objective: float | None = None
        last_qp_exitflag: int | None = None

        for iteration in range(max_iters):
            analytic = self._weighted_residual_and_jacobian(
                q,
                q_init,
                q_prev,
                source_laplacian,
                source_bone_terms,
                graph,
                eps=eps,
            )
            if analytic is None:
                residual = self._weighted_residual(q, q_init, q_prev, source_laplacian, source_bone_terms, graph)
                jacobian = np.empty((residual.shape[0], q.shape[0]), dtype=float)
                for col in range(q.shape[0]):
                    upper_room = max(0.0, self.upper[col] - q[col])
                    lower_room = max(0.0, q[col] - self.lower[col])
                    if upper_room >= eps:
                        step = eps
                    elif lower_room >= eps:
                        step = -eps
                    elif upper_room >= lower_room and upper_room > 0.0:
                        step = upper_room
                    elif lower_room > 0.0:
                        step = -lower_room
                    else:
                        jacobian[:, col] = 0.0
                        continue
                    q_step = q.copy()
                    q_step[col] += step
                    jacobian[:, col] = (self._weighted_residual(q_step, q_init, q_prev, source_laplacian, source_bone_terms, graph) - residual) / step
                    objective_evals += 1
            else:
                residual, jacobian = analytic
                objective_evals += 1
            objective = _squared_residual_objective(residual)
            if initial_objective is None:
                initial_objective = objective
            objective_evals += 1

            hessian = jacobian.T @ jacobian + damping * np.eye(q.shape[0], dtype=float)
            gradient = jacobian.T @ residual
            if not np.isfinite(hessian).all() or not np.isfinite(gradient).all():
                success = False
                message = "DAQP SQP failed: non-finite QP matrix"
                break
            delta_lower = np.maximum(self.lower - q, -step_limit)
            delta_upper = np.minimum(self.upper - q, step_limit)
            try:
                delta, _fval, exitflag, info = daqp.solve(
                    np.ascontiguousarray(hessian, dtype=float),
                    np.ascontiguousarray(gradient, dtype=float),
                    np.empty((0, q.shape[0]), dtype=float),
                    np.ascontiguousarray(delta_upper, dtype=float),
                    np.ascontiguousarray(delta_lower, dtype=float),
                    primal_start=np.zeros_like(q),
                )
            except Exception as exc:  # pragma: no cover - solver/runtime failure path
                success = False
                message = f"DAQP failed: {exc}"
                break

            qp_solve_time_s += float(info.get("solve_time", 0.0))
            qp_iterations += int(info.get("iterations", 0) or 0)
            last_qp_exitflag = int(exitflag)
            if last_qp_exitflag != 1:
                success = False
                message = f"DAQP exitflag={exitflag}"
                break
            delta = np.asarray(delta, dtype=float).reshape(q.shape)
            if not np.isfinite(delta).all():
                success = False
                message = "DAQP SQP failed: non-finite QP step"
                break
            if float(np.linalg.norm(delta, ord=np.inf)) < float(self.config.ftol):
                converged = True
                message = "DAQP SQP converged: step below ftol"
                break

            q_next = q
            accepted = False
            for alpha in (1.0, 0.5, 0.25, 0.125, 0.0625):
                candidate = np.clip(q + alpha * delta, self.lower, self.upper)
                candidate_residual = self._weighted_residual(
                    candidate,
                    q_init,
                    q_prev,
                    source_laplacian,
                    source_bone_terms,
                    graph,
                )
                objective_evals += 1
                if _squared_residual_objective(candidate_residual) <= objective + 1.0e-12:
                    q_next = candidate
                    accepted = True
                    break
                line_search_backtracks += 1
            if not accepted:
                message = "DAQP SQP stopped: no descent step accepted"
                break
            if float(np.linalg.norm(q_next - q, ord=np.inf)) < float(self.config.ftol):
                q = q_next
                converged = True
                message = "DAQP SQP converged: accepted step below ftol"
                break
            q = q_next
        else:
            message = "DAQP SQP iteration limit reached"

        final_objective = _squared_residual_objective(
            self._weighted_residual(q, q_init, q_prev, source_laplacian, source_bone_terms, graph)
        )
        return q, {
            "success": success,
            "message": message,
            "num_iters": iteration + 1,
            "objective_evals": objective_evals,
            "qp_solve_time_s": qp_solve_time_s,
            "qp_iterations": qp_iterations,
            "line_search_backtracks": line_search_backtracks,
            "converged": converged,
            "initial_objective": initial_objective,
            "final_objective": final_objective,
            "objective_improvement": None if initial_objective is None else initial_objective - final_objective,
            "last_qp_exitflag": last_qp_exitflag,
            "initial_guess": initial_guess,
        }

    def _weighted_residual(
        self,
        q: np.ndarray,
        q_init: np.ndarray,
        q_prev: np.ndarray,
        source_laplacian: np.ndarray,
        source_bone_terms: dict[tuple[int, int], np.ndarray],
        graph: "InteractionGraph",
    ) -> np.ndarray:
        robot_bone_points = self._robot_points_palm_m(q)
        return self._weighted_residual_from_points(
            robot_bone_points,
            q,
            q_init,
            q_prev,
            source_laplacian,
            source_bone_terms,
            graph,
        )

    def _weighted_residual_from_points(
        self,
        robot_bone_points: np.ndarray,
        q: np.ndarray,
        q_init: np.ndarray,
        q_prev: np.ndarray,
        source_laplacian: np.ndarray,
        source_bone_terms: dict[tuple[int, int], np.ndarray],
        graph: "InteractionGraph",
    ) -> np.ndarray:
        robot_topo_points = _canonical_hand_points(robot_bone_points, frame=self.robot_topo_frame)
        residual = np.concatenate(
            [
                np.sqrt(float(self.config.lambda_im))
                * _interaction_mesh_residual(
                    robot_topo_points,
                    source_laplacian,
                    graph,
                    epsilon_m=float(self.config.interaction_l2_epsilon_m),
                    normalization_m=self.robot_palm_width_m,
                ),
                np.sqrt(float(self.config.lambda_bone))
                * _bone_direction_residual(
                    robot_bone_points,
                    source_bone_terms,
                    thumb_root_weight=float(self.config.thumb_root_bone_weight),
                ),
                np.sqrt(float(self.config.lambda_init)) * (q - q_init),
                np.sqrt(float(self.config.lambda_reg)) * (q - q_prev),
            ]
        )
        if self._pinch_tips.size:
            residual = np.concatenate((residual, self._pinch_residual(robot_bone_points)))
        return residual

    def _weighted_residual_and_jacobian(
        self,
        q: np.ndarray,
        q_init: np.ndarray,
        q_prev: np.ndarray,
        source_laplacian: np.ndarray,
        source_bone_terms: dict[tuple[int, int], np.ndarray],
        graph: "InteractionGraph",
        *,
        eps: float,
    ) -> tuple[np.ndarray, np.ndarray] | None:
        del eps  # analytic path; kept for call-site compatibility
        points_and_jacobian = self._robot_points_and_jacobian_palm_m(q)
        if points_and_jacobian is None:
            return None
        robot_bone_points, point_jacobian = points_and_jacobian
        return self._weighted_residual_and_jacobian_analytic(
            robot_bone_points,
            point_jacobian,
            q,
            q_init,
            q_prev,
            source_laplacian,
            source_bone_terms,
            graph,
        )

    def _weighted_residual_and_jacobian_analytic(
        self,
        robot_bone_points: np.ndarray,
        point_jacobian: np.ndarray,
        q: np.ndarray,
        q_init: np.ndarray,
        q_prev: np.ndarray,
        source_laplacian: np.ndarray,
        source_bone_terms: dict[tuple[int, int], np.ndarray],
        graph: "InteractionGraph",
    ) -> tuple[np.ndarray, np.ndarray]:
        """Chain-rule residual Jacobian: dr/dpoints @ Pinocchio point Jacobian."""
        frame = np.asarray(self.robot_topo_frame, dtype=float)
        robot_topo_points = _canonical_hand_points(robot_bone_points, frame=frame)
        # Fixed zero-pose palm frame: p_topo = Rᵀ p  ⇒  Jp_topo = Rᵀ Jp
        topo_point_jacobian = np.einsum("ij,njk->nik", frame.T, point_jacobian)

        im_res, im_jac = _interaction_mesh_residual_and_jacobian_q(
            robot_topo_points,
            topo_point_jacobian,
            source_laplacian,
            graph,
            epsilon_m=float(self.config.interaction_l2_epsilon_m),
            normalization_m=self.robot_palm_width_m,
        )
        bone_res, bone_jac = _bone_direction_residual_and_jacobian_q(
            robot_bone_points,
            point_jacobian,
            source_bone_terms,
            thumb_root_weight=float(self.config.thumb_root_bone_weight),
        )

        nq = int(q.shape[0])
        eye = np.eye(nq, dtype=float)
        sqrt_im = np.sqrt(float(self.config.lambda_im))
        sqrt_bone = np.sqrt(float(self.config.lambda_bone))
        sqrt_init = np.sqrt(float(self.config.lambda_init))
        sqrt_reg = np.sqrt(float(self.config.lambda_reg))

        residual = np.concatenate(
            [
                sqrt_im * im_res,
                sqrt_bone * bone_res,
                sqrt_init * (q - q_init),
                sqrt_reg * (q - q_prev),
            ]
        )
        jacobian = np.concatenate(
            [
                sqrt_im * im_jac,
                sqrt_bone * bone_jac,
                sqrt_init * eye,
                sqrt_reg * eye,
            ],
            axis=0,
        )
        if self._pinch_tips.size:
            residual = np.concatenate((residual, self._pinch_residual(robot_bone_points)))
            contact_jacobian = self._pinch_gains[:, :, None] * (point_jacobian[4] - point_jacobian[self._pinch_tips])
            jacobian = np.concatenate((jacobian, contact_jacobian.reshape(-1, len(q))), axis=0)
        return residual, jacobian

    def _relative_bone_direction_initialization(
        self,
        source_bone_terms: dict[tuple[int, int], np.ndarray],
    ) -> tuple[np.ndarray, dict[str, Any]]:
        q_prior = self.previous_init_q.copy() if self.previous_init_q is not None else self.init_q.copy()
        q = np.clip(q_prior.copy(), self.lower, self.upper)
        max_iters = max(1, int(self.config.init_max_iters))
        eps = max(1.0e-8, float(self.config.finite_difference_eps))
        damping = max(1.0e-12, float(self.config.daqp_hessian_damping))
        step_limit = max(float(self.config.ftol), float(self.config.daqp_init_step_limit_rad))
        objective_evals = 0
        qp_solve_time_s = 0.0
        qp_iterations = 0
        last_qp_exitflag: int | None = None
        message = "relative bone-direction initialization converged"
        success = True
        converged = False
        initial_objective: float | None = None
        final_objective: float | None = None

        for iteration in range(max_iters):
            analytic = self._initialization_residual_and_jacobian(q, q_prior, source_bone_terms, eps=eps)
            if analytic is None:
                residual = self._initialization_residual(q, q_prior, source_bone_terms)
                jacobian = np.empty((residual.shape[0], q.shape[0]), dtype=float)
                for col in range(q.shape[0]):
                    upper_room = max(0.0, self.upper[col] - q[col])
                    lower_room = max(0.0, q[col] - self.lower[col])
                    if upper_room >= eps:
                        step = eps
                    elif lower_room >= eps:
                        step = -eps
                    elif upper_room >= lower_room and upper_room > 0.0:
                        step = upper_room
                    elif lower_room > 0.0:
                        step = -lower_room
                    else:
                        jacobian[:, col] = 0.0
                        continue
                    q_step = q.copy()
                    q_step[col] += step
                    jacobian[:, col] = (self._initialization_residual(q_step, q_prior, source_bone_terms) - residual) / step
                    objective_evals += 1
            else:
                residual, jacobian = analytic
                objective_evals += 1
            objective = _squared_residual_objective(residual)
            if initial_objective is None:
                initial_objective = objective
            objective_evals += 1

            hessian = jacobian.T @ jacobian + damping * np.eye(q.shape[0], dtype=float)
            gradient = jacobian.T @ residual
            if not np.isfinite(hessian).all() or not np.isfinite(gradient).all():
                success = False
                message = "relative bone-direction initialization failed: non-finite QP matrix"
                break
            try:
                delta, _fval, exitflag, info = daqp.solve(
                    np.ascontiguousarray(hessian, dtype=float),
                    np.ascontiguousarray(gradient, dtype=float),
                    np.empty((0, q.shape[0]), dtype=float),
                    np.ascontiguousarray(np.minimum(self.upper - q, step_limit), dtype=float),
                    np.ascontiguousarray(np.maximum(self.lower - q, -step_limit), dtype=float),
                    primal_start=np.zeros_like(q),
                )
            except Exception as exc:  # pragma: no cover - solver/runtime failure path
                success = False
                message = f"relative bone-direction initialization DAQP failed: {exc}"
                break
            qp_solve_time_s += float(info.get("solve_time", 0.0))
            qp_iterations += int(info.get("iterations", 0) or 0)
            last_qp_exitflag = int(exitflag)
            if last_qp_exitflag != 1:
                success = False
                message = f"relative bone-direction initialization DAQP exitflag={exitflag}"
                break
            delta = np.asarray(delta, dtype=float).reshape(q.shape)
            if not np.isfinite(delta).all():
                success = False
                message = "relative bone-direction initialization failed: non-finite QP step"
                break
            if float(np.linalg.norm(delta, ord=np.inf)) < float(self.config.ftol):
                converged = True
                message = "relative bone-direction initialization converged: step below ftol"
                break

            q_next = q
            accepted = False
            for alpha in (1.0, 0.5, 0.25, 0.125, 0.0625):
                candidate = np.clip(q + alpha * delta, self.lower, self.upper)
                candidate_residual = self._initialization_residual(candidate, q_prior, source_bone_terms)
                objective_evals += 1
                if _squared_residual_objective(candidate_residual) <= objective + 1.0e-12:
                    q_next = candidate
                    accepted = True
                    break
            if not accepted:
                message = "relative bone-direction initialization stopped: no descent step accepted"
                break
            if float(np.linalg.norm(q_next - q, ord=np.inf)) < float(self.config.ftol):
                q = q_next
                converged = True
                message = "relative bone-direction initialization converged: accepted step below ftol"
                break
            q = q_next
        else:
            message = "relative bone-direction initialization iteration limit reached"

        final_objective = _squared_residual_objective(self._initialization_residual(q, q_prior, source_bone_terms))
        return q, {
            "method": "relative_bone_direction",
            "success": bool(success),
            "message": message,
            "num_iters": int(iteration + 1),
            "objective_evals": int(objective_evals),
            "qp_solve_time_s": float(qp_solve_time_s),
            "qp_iterations": int(qp_iterations),
            "converged": bool(converged),
            "initial_objective": initial_objective,
            "final_objective": final_objective,
            "objective_improvement": None if initial_objective is None else initial_objective - final_objective,
            "last_qp_exitflag": last_qp_exitflag,
        }

    def _initialization_residual(
        self,
        q: np.ndarray,
        q_prior: np.ndarray,
        source_bone_terms: dict[tuple[int, int], np.ndarray],
    ) -> np.ndarray:
        robot_points = self._robot_points_palm_m(q)
        return self._initialization_residual_from_points(robot_points, q, q_prior, source_bone_terms)

    def _initialization_residual_from_points(
        self,
        robot_points: np.ndarray,
        q: np.ndarray,
        q_prior: np.ndarray,
        source_bone_terms: dict[tuple[int, int], np.ndarray],
    ) -> np.ndarray:
        return np.concatenate(
            [
                np.sqrt(float(self.config.lambda_warm))
                * _bone_direction_residual(
                    robot_points,
                    source_bone_terms,
                    thumb_root_weight=float(self.config.thumb_root_bone_weight),
                ),
                np.sqrt(float(self.config.lambda_smooth)) * (q - q_prior),
            ]
        )

    def _initialization_residual_and_jacobian(
        self,
        q: np.ndarray,
        q_prior: np.ndarray,
        source_bone_terms: dict[tuple[int, int], np.ndarray],
        *,
        eps: float,
    ) -> tuple[np.ndarray, np.ndarray] | None:
        del eps
        points_and_jacobian = self._robot_points_and_jacobian_palm_m(q)
        if points_and_jacobian is None:
            return None
        robot_points, point_jacobian = points_and_jacobian
        return self._initialization_residual_and_jacobian_analytic(
            robot_points, point_jacobian, q, q_prior, source_bone_terms
        )

    def _initialization_residual_and_jacobian_analytic(
        self,
        robot_points: np.ndarray,
        point_jacobian: np.ndarray,
        q: np.ndarray,
        q_prior: np.ndarray,
        source_bone_terms: dict[tuple[int, int], np.ndarray],
    ) -> tuple[np.ndarray, np.ndarray]:
        bone_res, bone_jac = _bone_direction_residual_and_jacobian_q(
            robot_points,
            point_jacobian,
            source_bone_terms,
            thumb_root_weight=float(self.config.thumb_root_bone_weight),
        )
        nq = int(q.shape[0])
        sqrt_warm = np.sqrt(float(self.config.lambda_warm))
        sqrt_smooth = np.sqrt(float(self.config.lambda_smooth))
        residual = np.concatenate(
            [
                sqrt_warm * bone_res,
                sqrt_smooth * (q - q_prior),
            ]
        )
        jacobian = np.concatenate(
            [
                sqrt_warm * bone_jac,
                sqrt_smooth * np.eye(nq, dtype=float),
            ],
            axis=0,
        )
        return residual, jacobian

    def _joint_from_ratio(self, joint_name: str, ratio: float) -> float:
        idx = self.joint_names.index(joint_name)
        return float(self.lower[idx] + _clamp01(ratio) * (self.upper[idx] - self.lower[idx]))

    def _signed_joint_from_ratio(self, joint_name: str, ratio: float) -> float:
        idx = self.joint_names.index(joint_name)
        signed = _clamp(float(ratio), -1.0, 1.0)
        return float(signed * self.upper[idx] if signed >= 0.0 else -signed * self.lower[idx])

    def _q_to_dict(self, q: np.ndarray) -> dict[str, float]:
        return {name: float(value) for name, value in zip(self.joint_names, np.asarray(q, dtype=float), strict=True)}

    def _loss_breakdown(
        self,
        q: np.ndarray,
        q_init: np.ndarray,
        q_prev: np.ndarray,
        source_laplacian: np.ndarray,
        source_bone_terms: dict[tuple[int, int], np.ndarray],
        graph: "InteractionGraph",
    ) -> dict[str, float]:
        robot_bone_points = self._robot_points_palm_m(q)
        robot_topo_points = _canonical_hand_points(robot_bone_points, frame=self.robot_topo_frame)
        e_im = _interaction_mesh_energy(
            robot_topo_points,
            source_laplacian,
            graph,
            epsilon_m=float(self.config.interaction_l2_epsilon_m),
            normalization_m=self.robot_palm_width_m,
        )
        e_bone = _bone_direction_energy(
            robot_bone_points,
            source_bone_terms,
            thumb_root_weight=float(self.config.thumb_root_bone_weight),
        )
        e_init = 0.5 * self.config.lambda_init * float(np.sum((q - q_init) ** 2))
        e_reg = 0.5 * self.config.lambda_reg * float(np.sum((q - q_prev) ** 2))
        contact = self._pinch_residual(robot_bone_points)
        e_pinch = 0.5 * float(contact @ contact)
        total = self.config.lambda_im * e_im + self.config.lambda_bone * e_bone + e_init + e_reg + e_pinch
        return {
            "total": float(total),
            "interaction_mesh_raw": float(e_im),
            "interaction_mesh_weighted": float(self.config.lambda_im * e_im),
            "bone_raw": float(e_bone),
            "bone_weighted": float(self.config.lambda_bone * e_bone),
            "init_weighted": float(e_init),
            "regularization_weighted": float(e_reg),
            "pinch_weighted": e_pinch,
        }


@dataclass(frozen=True)
class InteractionGraph:
    edges: tuple[tuple[int, int], ...]
    neighbors: tuple[tuple[int, ...], ...]
    weights: tuple[tuple[float, ...], ...]
    delaunay_ok: bool
    # Dense row-stochastic neighbor weights; empty rows stay zero (no neighbors).
    weight_matrix: np.ndarray | None = None


def _interaction_graph_from_weight_matrix(
    weight_matrix: np.ndarray,
    *,
    delaunay_ok: bool,
    prune_epsilon: float = 1.0e-12,
) -> InteractionGraph:
    """Rebuild the sparse graph view from a blended row-weight matrix."""
    matrix = np.asarray(weight_matrix, dtype=float).copy()
    if matrix.shape != (NUM_LANDMARKS, NUM_LANDMARKS):
        raise ValueError(
            "interaction weight matrix must be "
            f"{NUM_LANDMARKS}x{NUM_LANDMARKS}, got {matrix.shape}"
        )
    if not np.isfinite(matrix).all():
        raise ValueError("interaction weight matrix contains NaN/Inf")
    matrix = np.maximum(matrix, 0.0)
    np.fill_diagonal(matrix, 0.0)
    matrix[matrix < max(0.0, float(prune_epsilon))] = 0.0
    row_sums = matrix.sum(axis=1)
    valid_rows = row_sums > 1.0e-12
    matrix[valid_rows] /= row_sums[valid_rows, None]

    undirected = np.triu((matrix + matrix.T) > 0.0, k=1)
    edge_array = np.argwhere(undirected)
    edges = tuple((int(a), int(b)) for a, b in edge_array)
    if not edges:
        raise ValueError("stabilized interaction graph has no edges")

    neighbors: list[tuple[int, ...]] = []
    weights: list[tuple[float, ...]] = []
    for idx in range(NUM_LANDMARKS):
        neighbor_ids = tuple(int(item) for item in np.flatnonzero(matrix[idx] > 0.0))
        neighbors.append(neighbor_ids)
        weights.append(tuple(float(matrix[idx, item]) for item in neighbor_ids))
    return InteractionGraph(
        edges=edges,
        neighbors=tuple(neighbors),
        weights=tuple(weights),
        delaunay_ok=bool(delaunay_ok),
        weight_matrix=matrix,
    )


class _PinocchioKeypointKinematics:
    def __init__(self, urdf_path: str | Path, keypoints: Any, joint_names: list[str], urdf_model: Any):
        import pinocchio as pin

        self.pin = pin
        self.model = pin.buildModelFromUrdf(str(urdf_path))
        if self.model.nq != self.model.nv:
            raise ValueError("analytic Jacobian backend requires nq == nv")
        self.data = self.model.createData()
        self.joint_names = list(joint_names)

        full_q_indices: dict[str, int] = {}
        full_v_indices: dict[str, int] = {}
        for joint_id, name in enumerate(self.model.names):
            if int(self.model.nqs[joint_id]) == 0:
                continue
            if int(self.model.nqs[joint_id]) != 1 or int(self.model.nvs[joint_id]) != 1:
                raise ValueError(f"analytic Jacobian backend only supports 1-DoF joints, got {name!r}")
            full_q_indices[str(name)] = int(self.model.idx_qs[joint_id])
            full_v_indices[str(name)] = int(self.model.idx_vs[joint_id])
        missing = [name for name in self.joint_names if name not in full_q_indices or name not in full_v_indices]
        if missing:
            raise ValueError("analytic Jacobian backend missing joints: " + ", ".join(missing))
        controlled_mimic = [name for name in self.joint_names if urdf_model.joint_by_name[name].mimic_joint is not None]
        if controlled_mimic:
            raise ValueError("analytic Jacobian controlled joints must not be URDF mimic joints: " + ", ".join(controlled_mimic))

        controlled_index = {name: idx for idx, name in enumerate(self.joint_names)}
        dependencies: dict[str, tuple[np.ndarray, float]] = {}
        resolving: set[str] = set()

        def dependency(name: str) -> tuple[np.ndarray, float]:
            if name in dependencies:
                return dependencies[name]
            if name in resolving:
                raise ValueError(f"cyclic URDF mimic relation at joint {name!r}")
            resolving.add(name)
            vector = np.zeros(len(self.joint_names), dtype=float)
            offset = 0.0
            if name in controlled_index:
                vector[controlled_index[name]] = 1.0
            else:
                joint = urdf_model.joint_by_name.get(name)
                if joint is not None and joint.mimic_joint is not None:
                    parent_vector, parent_offset = dependency(joint.mimic_joint)
                    vector = joint.mimic_multiplier * parent_vector
                    offset = joint.mimic_multiplier * parent_offset + joint.mimic_offset
            resolving.remove(name)
            dependencies[name] = (vector, offset)
            return dependencies[name]

        self._q_dependency = np.zeros((self.model.nq, len(self.joint_names)), dtype=float)
        self._q_offset = np.zeros(self.model.nq, dtype=float)
        self._v_dependency = np.zeros((self.model.nv, len(self.joint_names)), dtype=float)
        for name, q_idx in full_q_indices.items():
            vector, offset = dependency(name)
            self._q_dependency[q_idx] = vector
            self._q_offset[q_idx] = offset
            self._v_dependency[full_v_indices[name]] = vector

        sorted_keypoints = sorted(keypoints.keypoints, key=lambda item: item.id)
        if [kp.id for kp in sorted_keypoints] != list(range(len(sorted_keypoints))):
            raise ValueError("analytic Jacobian backend expects contiguous keypoint ids")
        self._frame_ids = []
        self._local_points = []
        for kp in sorted_keypoints:
            frame_id = int(self.model.getFrameId(kp.link))
            if frame_id >= self.model.nframes:
                raise ValueError(f"analytic Jacobian backend missing keypoint link frame: {kp.link!r}")
            self._frame_ids.append(frame_id)
            self._local_points.append(np.asarray(kp.local_xyz, dtype=float))
        self._local_points = np.asarray(self._local_points, dtype=float)

    def _full_q(self, q: np.ndarray) -> np.ndarray:
        q_arr = np.asarray(q, dtype=float).reshape(-1)
        if q_arr.shape[0] != len(self.joint_names):
            raise ValueError(f"q has {q_arr.shape[0]} values but analytic backend expects {len(self.joint_names)}")
        return self._q_offset + self._q_dependency @ q_arr

    def compute_points(self, q: np.ndarray) -> np.ndarray:
        """仅计算关键点位置，不执行此路径不需要的 Jacobian 工作。"""
        q_full = self._full_q(q)
        pin = self.pin
        pin.forwardKinematics(self.model, self.data, q_full)
        pin.updateFramePlacements(self.model, self.data)

        points = np.empty((len(self._frame_ids), 3), dtype=float)
        for idx, (frame_id, local_point) in enumerate(zip(self._frame_ids, self._local_points, strict=True)):
            placement = self.data.oMf[frame_id]
            points[idx] = placement.translation + placement.rotation @ local_point
        return points

    def compute_points_and_jacobian(self, q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        q_full = self._full_q(q)

        pin = self.pin
        pin.forwardKinematics(self.model, self.data, q_full)
        pin.computeJointJacobians(self.model, self.data, q_full)
        pin.updateFramePlacements(self.model, self.data)

        points = np.empty((len(self._frame_ids), 3), dtype=float)
        jacobian = np.empty((len(self._frame_ids), 3, len(self.joint_names)), dtype=float)
        for idx, (frame_id, local_point) in enumerate(zip(self._frame_ids, self._local_points, strict=True)):
            placement = self.data.oMf[frame_id]
            offset_world = placement.rotation @ local_point
            points[idx] = placement.translation + offset_world
            frame_jacobian = pin.getFrameJacobian(
                self.model,
                self.data,
                frame_id,
                pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
            )
            point_jacobian = frame_jacobian[:3] - _skew(offset_world) @ frame_jacobian[3:]
            jacobian[idx] = point_jacobian @ self._v_dependency
        return points, jacobian


def retarget_mediapipe_landmarks(
    joint_positions_xyz: np.ndarray,
    *,
    joint_orientations_xyzw: np.ndarray | None = None,
    joint_valid: np.ndarray | None = None,
    previous_joint_values: dict[str, float] | np.ndarray | None = None,
    retargeter: TopoRetargeter | None = None,
    config: TopoRetargetConfig | None = None,
) -> dict[str, float]:
    solver = retargeter or TopoRetargeter(config)
    result = solver.retarget(
        joint_positions_xyz,
        joint_orientations_xyzw=joint_orientations_xyzw,
        joint_valid=joint_valid,
        previous_joint_values=previous_joint_values,
    )
    return result.joint_values


def _build_interaction_graph(source_points: np.ndarray, config: TopoRetargetConfig, *, distance_scale_m: float) -> InteractionGraph:
    tetra = Delaunay(source_points, qhull_options="QJ")
    simplices = np.asarray(tetra.simplices, dtype=np.intp)
    # 每个四面体产生 6 条无向边；np.unique 的字典序与原 sorted(edges) 一致。
    local_pairs = np.asarray(((0, 1), (0, 2), (0, 3), (1, 2), (1, 3), (2, 3)), dtype=np.intp)
    edge_array = simplices[:, local_pairs].reshape(-1, 2)
    edge_array.sort(axis=1)
    edge_array = np.unique(edge_array, axis=0)
    if edge_array.size == 0:
        raise ValueError("interaction graph has no edges")
    edges = tuple((int(a), int(b)) for a, b in edge_array)
    neighbors_raw: list[list[int]] = [[] for _ in range(NUM_LANDMARKS)]
    for a, b in edges:
        neighbors_raw[a].append(b)
        neighbors_raw[b].append(a)
    weights = []
    scale_m = max(1.0e-9, float(distance_scale_m))
    # Evaluate all 21x21 distances at once; preserve neighbor order and the
    # original row normalization, thresholds and Delaunay topology.
    delta = source_points[:, None, :] - source_points[None, :, :]
    pair_weights = np.exp(-config.kappa * np.sum(delta ** 2, axis=2) / (scale_m**2))
    weight_matrix = np.zeros((NUM_LANDMARKS, NUM_LANDMARKS), dtype=float)
    for idx, neighbor_ids in enumerate(neighbors_raw):
        if not neighbor_ids:
            weights.append(tuple())
            continue
        raw = pair_weights[idx, neighbor_ids]
        total = float(np.sum(raw))
        if total <= 1.0e-12:
            raw = np.ones_like(raw) / len(raw)
        else:
            raw = raw / total
        weights.append(tuple(float(item) for item in raw))
        for neighbor, weight in zip(neighbor_ids, raw, strict=True):
            weight_matrix[idx, int(neighbor)] = float(weight)
    return InteractionGraph(
        edges=edges,
        neighbors=tuple(tuple(item) for item in neighbors_raw),
        weights=tuple(weights),
        delaunay_ok=True,
        weight_matrix=weight_matrix,
    )


def _laplacian_coordinates(points: np.ndarray, graph: InteractionGraph) -> np.ndarray:
    pts = np.asarray(points, dtype=float)
    weights = graph.weight_matrix
    # Σw_ij(pi-pj) == (Σw_ij)pi-W@p；保留空邻接行等边界情形的原语义。
    return weights.sum(axis=1, keepdims=True) * pts - weights @ pts


def _interaction_mesh_energy(
    robot_points: np.ndarray,
    source_laplacian: np.ndarray,
    graph: InteractionGraph,
    *,
    epsilon_m: float,
    normalization_m: float,
) -> float:
    errors = _interaction_mesh_l2_errors(robot_points, source_laplacian, graph, epsilon_m=epsilon_m)
    return float(np.mean(errors) / max(1.0e-9, float(normalization_m)))


def _interaction_mesh_residual(
    robot_points: np.ndarray,
    source_laplacian: np.ndarray,
    graph: InteractionGraph,
    *,
    epsilon_m: float,
    normalization_m: float,
) -> np.ndarray:
    errors = _interaction_mesh_l2_errors(robot_points, source_laplacian, graph, epsilon_m=epsilon_m)
    scale = np.sqrt(2.0 / max(1, robot_points.shape[0]))
    normalized_errors = errors / max(1.0e-9, float(normalization_m))
    return scale * np.sqrt(normalized_errors)


def _interaction_mesh_l2_errors(
    robot_points: np.ndarray,
    source_laplacian: np.ndarray,
    graph: InteractionGraph,
    *,
    epsilon_m: float,
) -> np.ndarray:
    robot_laplacian = _laplacian_coordinates(robot_points, graph)
    delta = robot_laplacian - source_laplacian
    epsilon = max(0.0, float(epsilon_m))
    return np.sqrt(np.sum(delta**2, axis=1) + epsilon**2)


def _graph_weight_matrix(graph: InteractionGraph) -> np.ndarray:
    if graph.weight_matrix is not None:
        return np.asarray(graph.weight_matrix, dtype=float)
    weight_matrix = np.zeros((NUM_LANDMARKS, NUM_LANDMARKS), dtype=float)
    for idx, (neighbor_ids, neighbor_weights) in enumerate(zip(graph.neighbors, graph.weights, strict=True)):
        for neighbor, weight in zip(neighbor_ids, neighbor_weights, strict=True):
            weight_matrix[idx, int(neighbor)] = float(weight)
    return weight_matrix


def _laplacian_coordinates_and_jacobian_q(
    points: np.ndarray,
    point_jacobian: np.ndarray,
    weight_matrix: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """L_i = (Σ_j W_ij) p_i − Σ_j W_ij p_j, with dL/dq via the same linear map."""
    row_sum = weight_matrix.sum(axis=1)  # (N,)
    laplacian = row_sum[:, None] * points - weight_matrix @ points
    # dL_i/dq = row_sum_i * Jp_i - Σ_j W_ij Jp_j
    # Vectorized: for each xyz component, Jac_xyz = diag(row_sum) @ Jp_xyz - W @ Jp_xyz
    lap_jacobian = np.empty_like(point_jacobian)
    for axis in range(3):
        jp_axis = point_jacobian[:, axis, :]  # (N, nq)
        lap_jacobian[:, axis, :] = row_sum[:, None] * jp_axis - weight_matrix @ jp_axis
    return laplacian, lap_jacobian


def _interaction_mesh_residual_and_jacobian_q(
    robot_points: np.ndarray,
    point_jacobian: np.ndarray,
    source_laplacian: np.ndarray,
    graph: InteractionGraph,
    *,
    epsilon_m: float,
    normalization_m: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Analytic Jacobian of soft-L2 interaction-mesh residual w.r.t. q.

    r_i = scale * sqrt(e_i / norm),  e_i = sqrt(||L_i - S_i||^2 + eps^2)
    """
    weight_matrix = _graph_weight_matrix(graph)
    laplacian, lap_jacobian = _laplacian_coordinates_and_jacobian_q(
        robot_points, point_jacobian, weight_matrix
    )
    delta = laplacian - np.asarray(source_laplacian, dtype=float)
    epsilon = max(0.0, float(epsilon_m))
    e = np.sqrt(np.sum(delta**2, axis=1) + epsilon**2)
    n_points = max(1, int(robot_points.shape[0]))
    scale = np.sqrt(2.0 / n_points)
    norm = max(1.0e-9, float(normalization_m))
    residual = scale * np.sqrt(e / norm)

    nq = int(point_jacobian.shape[2])
    jacobian = np.zeros((n_points, nq), dtype=float)
    # dr_i/ddelta = scale * delta_i / (2 * e_i^{3/2} * sqrt(norm))
    denom = 2.0 * np.power(e, 1.5) * np.sqrt(norm)
    for idx in range(n_points):
        if denom[idx] <= 1.0e-18:
            continue
        coeff = scale * delta[idx] / denom[idx]  # (3,)
        jacobian[idx] = coeff @ lap_jacobian[idx]  # (nq,)
    return residual, jacobian


def _normalize_and_jacobian(vector: np.ndarray, vector_jacobian: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """u = v/||v||, du/dq = (I − uuᵀ)/||v|| · dv/dq."""
    arr = np.asarray(vector, dtype=float).reshape(3)
    jac = np.asarray(vector_jacobian, dtype=float).reshape(3, -1)
    norm = float(np.linalg.norm(arr))
    if norm < 1.0e-9:
        return np.zeros(3, dtype=float), np.zeros_like(jac)
    unit = arr / norm
    projector = np.eye(3, dtype=float) - np.outer(unit, unit)
    return unit, (projector / norm) @ jac


def _hand_frame_and_jacobian_q(
    points: np.ndarray,
    point_jacobian: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Return palm frame columns (across, forward, normal) and their dq Jacobians."""
    across_raw = points[INDEX_MCP_ID] - points[PINKY_MCP_ID]
    forward_raw = points[MIDDLE_MCP_ID] - points[WRIST_ID]
    d_across_raw = point_jacobian[INDEX_MCP_ID] - point_jacobian[PINKY_MCP_ID]
    d_forward_raw = point_jacobian[MIDDLE_MCP_ID] - point_jacobian[WRIST_ID]

    across, d_across = _normalize_and_jacobian(across_raw, d_across_raw)
    forward, d_forward = _normalize_and_jacobian(forward_raw, d_forward_raw)
    normal_raw = np.cross(across, forward)
    # d(a×f)/dq = [a]_× df − [f]_× da
    d_normal_raw = _skew(across) @ d_forward - _skew(forward) @ d_across
    normal, d_normal = _normalize_and_jacobian(normal_raw, d_normal_raw)

    if not np.any(normal):
        forward_raw = points[12] - points[WRIST_ID]
        d_forward_raw = point_jacobian[12] - point_jacobian[WRIST_ID]
        forward, d_forward = _normalize_and_jacobian(forward_raw, d_forward_raw)
        normal_raw = np.cross(across, forward)
        d_normal_raw = _skew(across) @ d_forward - _skew(forward) @ d_across
        normal, d_normal = _normalize_and_jacobian(normal_raw, d_normal_raw)

    if not np.any(across) or not np.any(forward) or not np.any(normal):
        nq = int(point_jacobian.shape[2])
        eye = np.eye(3, dtype=float)
        zero = np.zeros((3, nq), dtype=float)
        return eye[:, 0], eye[:, 1], eye[:, 2], (zero, zero, zero)

    # Re-orthogonalize across = normalize(forward × normal)
    across_raw2 = np.cross(forward, normal)
    d_across_raw2 = _skew(forward) @ d_normal - _skew(normal) @ d_forward
    across, d_across = _normalize_and_jacobian(across_raw2, d_across_raw2)
    return across, forward, normal, (d_across, d_forward, d_normal)


def _bone_direction_residual_and_jacobian_q(
    robot_points: np.ndarray,
    point_jacobian: np.ndarray,
    source_terms: dict[tuple[int, int], np.ndarray],
    *,
    thumb_root_weight: float = 1.0,
) -> tuple[np.ndarray, np.ndarray]:
    """Analytic Jacobian of relative bone-direction residual w.r.t. q."""
    if tuple(source_terms) == _RELATIVE_TERM_KEYS:
        edge_dirs, edge_jacs = _bone_direction_array_and_jacobian_q(robot_points, point_jacobian)
        source = np.asarray(tuple(source_terms.values()), dtype=float)
        residual = edge_dirs[_RELATIVE_TERM_A_INDEX] - edge_dirs[_RELATIVE_TERM_B_INDEX] - source
        jacobian = edge_jacs[_RELATIVE_TERM_A_INDEX] - edge_jacs[_RELATIVE_TERM_B_INDEX]
        root_scale = float(np.sqrt(max(0.0, float(thumb_root_weight))))
        if root_scale != 1.0:
            residual[_RELATIVE_TERM_THUMB_ROOT] *= root_scale
            jacobian[_RELATIVE_TERM_THUMB_ROOT] *= root_scale
        return residual.reshape(-1), jacobian.reshape(-1, point_jacobian.shape[2])

    across, forward, normal, (d_across, d_forward, d_normal) = _hand_frame_and_jacobian_q(
        robot_points, point_jacobian
    )
    frame = np.column_stack((across, forward, normal))
    nq = int(point_jacobian.shape[2])

    edge_dirs: dict[tuple[int, int], np.ndarray] = {}
    edge_jacs: dict[tuple[int, int], np.ndarray] = {}
    for edge in BONE_DIRECTION_EDGES:
        i, j = edge
        raw = robot_points[j] - robot_points[i]
        d_raw = point_jacobian[j] - point_jacobian[i]
        u_world, du_world = _normalize_and_jacobian(raw, d_raw)
        # d_hand = Rᵀ u ;  dd_hand/dq includes both dR and du
        d_hand = frame.T @ u_world
        # row k: d(R[:,k]·u)/dq = R[:,k]·du + u·dR[:,k]
        dd_hand = np.empty((3, nq), dtype=float)
        dd_hand[0] = across @ du_world + u_world @ d_across
        dd_hand[1] = forward @ du_world + u_world @ d_forward
        dd_hand[2] = normal @ du_world + u_world @ d_normal
        edge_dirs[edge] = d_hand
        edge_jacs[edge] = dd_hand

    root_scale = float(np.sqrt(max(0.0, float(thumb_root_weight))))
    residuals: list[np.ndarray] = []
    jacobians: list[np.ndarray] = []
    for encoded, source_term in source_terms.items():
        a_code, b_code = encoded
        a = (a_code // 100, a_code % 100)
        b = (b_code // 100, b_code % 100)
        delta = (edge_dirs[a] - edge_dirs[b]) - np.asarray(source_term, dtype=float)
        jac = edge_jacs[a] - edge_jacs[b]
        if a in _THUMB_ROOT_EDGES or b in _THUMB_ROOT_EDGES:
            delta = root_scale * delta
            jac = root_scale * jac
        residuals.append(delta)
        jacobians.append(jac)

    if not residuals:
        return np.zeros(0, dtype=float), np.zeros((0, nq), dtype=float)
    return np.concatenate(residuals), np.concatenate(jacobians, axis=0)


def _relative_bone_terms(points: np.ndarray) -> dict[tuple[int, int], np.ndarray]:
    directions = _bone_directions_in_hand_frame(points)
    terms = {}
    for chain in ROOTED_FINGER_CHAINS.values():
        bone_ids = list(range(len(chain) - 1))
        for idx in range(len(bone_ids) - 1):
            a = (chain[idx], chain[idx + 1])
            b = (chain[idx + 1], chain[idx + 2])
            terms[(a[0] * 100 + a[1], b[0] * 100 + b[1])] = directions[a] - directions[b]
    return terms


# 拇指根部骨边：腕→CMC、CMC→MCP；对应 thumb_rota / thumb_swing 可观测性最强的一段
_THUMB_ROOT_EDGES = frozenset({(WRIST_ID, 1), (1, 2)})
_BONE_EDGE_ARRAY = np.asarray(BONE_DIRECTION_EDGES, dtype=np.intp)
_BONE_EDGE_INDEX = {edge: idx for idx, edge in enumerate(BONE_DIRECTION_EDGES)}
_RELATIVE_TERM_KEYS = tuple(
    (
        chain[idx] * 100 + chain[idx + 1],
        chain[idx + 1] * 100 + chain[idx + 2],
    )
    for chain in ROOTED_FINGER_CHAINS.values()
    for idx in range(len(chain) - 2)
)
_RELATIVE_TERM_A_INDEX = np.asarray(
    [_BONE_EDGE_INDEX[(a // 100, a % 100)] for a, _ in _RELATIVE_TERM_KEYS], dtype=np.intp
)
_RELATIVE_TERM_B_INDEX = np.asarray(
    [_BONE_EDGE_INDEX[(b // 100, b % 100)] for _, b in _RELATIVE_TERM_KEYS], dtype=np.intp
)
_RELATIVE_TERM_THUMB_ROOT = np.asarray(
    [
        (a // 100, a % 100) in _THUMB_ROOT_EDGES or (b // 100, b % 100) in _THUMB_ROOT_EDGES
        for a, b in _RELATIVE_TERM_KEYS
    ],
    dtype=bool,
)


def _bone_direction_energy(
    robot_points: np.ndarray,
    source_terms: dict[tuple[int, int], np.ndarray],
    *,
    thumb_root_weight: float = 1.0,
) -> float:
    residual = _bone_direction_residual(
        robot_points,
        source_terms,
        thumb_root_weight=thumb_root_weight,
    )
    count = len(source_terms)
    return 0.0 if count == 0 else float(np.sum(residual**2)) / count


def _bone_direction_residual(
    robot_points: np.ndarray,
    source_terms: dict[tuple[int, int], np.ndarray],
    *,
    thumb_root_weight: float = 1.0,
) -> np.ndarray:
    if tuple(source_terms) != _RELATIVE_TERM_KEYS:
        directions = _bone_directions_in_hand_frame(robot_points)
        root_scale = float(np.sqrt(max(0.0, float(thumb_root_weight))))
        residuals = []
        for encoded, source_term in source_terms.items():
            a_code, b_code = encoded
            a = (a_code // 100, a_code % 100)
            b = (b_code // 100, b_code % 100)
            delta = (directions[a] - directions[b]) - source_term
            if a in _THUMB_ROOT_EDGES or b in _THUMB_ROOT_EDGES:
                delta = root_scale * delta
            residuals.append(delta)
        if not residuals:
            return np.zeros(0, dtype=float)
        return np.concatenate(residuals)

    directions = _bone_direction_array_in_hand_frame(robot_points)
    source = np.asarray(tuple(source_terms.values()), dtype=float)
    residual = directions[_RELATIVE_TERM_A_INDEX] - directions[_RELATIVE_TERM_B_INDEX] - source
    root_scale = float(np.sqrt(max(0.0, float(thumb_root_weight))))
    if root_scale != 1.0:
        residual[_RELATIVE_TERM_THUMB_ROOT] *= root_scale
    return residual.reshape(-1)


def _squared_residual_objective(residual: np.ndarray) -> float:
    return 0.5 * float(np.dot(residual, residual))


def _bone_directions_in_hand_frame(points: np.ndarray) -> dict[tuple[int, int], np.ndarray]:
    directions = _bone_direction_array_in_hand_frame(points)
    return {edge: directions[idx] for idx, edge in enumerate(BONE_DIRECTION_EDGES)}


def _bone_direction_array_in_hand_frame(points: np.ndarray) -> np.ndarray:
    pts = np.asarray(points, dtype=float)
    raw = pts[_BONE_EDGE_ARRAY[:, 1]] - pts[_BONE_EDGE_ARRAY[:, 0]]
    norms = np.linalg.norm(raw, axis=1)
    unit = np.zeros_like(raw)
    valid = norms >= 1.0e-9
    unit[valid] = raw[valid] / norms[valid, None]
    return unit @ _hand_frame(pts)


def _bone_direction_array_and_jacobian_q(
    points: np.ndarray,
    point_jacobian: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    pts = np.asarray(points, dtype=float)
    point_jac = np.asarray(point_jacobian, dtype=float)
    across, forward, normal, (d_across, d_forward, d_normal) = _hand_frame_and_jacobian_q(
        pts, point_jac
    )
    frame = np.column_stack((across, forward, normal))
    d_frame = np.stack((d_across, d_forward, d_normal), axis=1)

    starts = _BONE_EDGE_ARRAY[:, 0]
    ends = _BONE_EDGE_ARRAY[:, 1]
    raw = pts[ends] - pts[starts]
    d_raw = point_jac[ends] - point_jac[starts]
    norms = np.linalg.norm(raw, axis=1)
    valid = norms >= 1.0e-9
    unit = np.zeros_like(raw)
    du_world = np.zeros_like(d_raw)
    unit[valid] = raw[valid] / norms[valid, None]
    if np.any(valid):
        unit_valid = unit[valid]
        projectors = np.eye(3, dtype=float) - unit_valid[:, :, None] * unit_valid[:, None, :]
        projectors /= norms[valid, None, None]
        du_world[valid] = np.einsum("eij,ejq->eiq", projectors, d_raw[valid], optimize=True)

    directions = unit @ frame
    jacobians = np.einsum("wk,ewq->ekq", frame, du_world, optimize=True)
    jacobians += np.einsum("ew,wkq->ekq", unit, d_frame, optimize=True)
    return directions, jacobians


def _canonical_hand_points(points: np.ndarray, *, frame: np.ndarray | None = None) -> np.ndarray:
    hand_frame = _hand_frame(points) if frame is None else np.asarray(frame, dtype=float)
    return (hand_frame.T @ np.asarray(points, dtype=float).T).T


def _cross3(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Cross product for known 3-vectors without generic axis/broadcast setup."""
    return np.array((a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]))


def _hand_frame(points: np.ndarray) -> np.ndarray:
    palm_across = _safe_normalize(points[INDEX_MCP_ID] - points[PINKY_MCP_ID])
    palm_forward = _safe_normalize(points[MIDDLE_MCP_ID] - points[WRIST_ID])
    palm_normal = _safe_normalize(_cross3(palm_across, palm_forward))
    if not np.any(palm_normal):
        palm_forward = _safe_normalize(points[12] - points[WRIST_ID])  # MiddleTip fallback
        palm_normal = _safe_normalize(_cross3(palm_across, palm_forward))
    if not np.any(palm_across) or not np.any(palm_forward) or not np.any(palm_normal):
        return np.eye(3, dtype=float)
    palm_across = _safe_normalize(_cross3(palm_forward, palm_normal))
    return np.column_stack((palm_across, palm_forward, palm_normal))


def _segment_angle(points: np.ndarray, a: int, b: int, c: int) -> float:
    lhs = points[b] - points[a]
    rhs = points[c] - points[b]
    lhs_unit = _safe_normalize(lhs)
    rhs_unit = _safe_normalize(rhs)
    if not np.any(lhs_unit) or not np.any(rhs_unit):
        return 0.0
    cosine = float(np.clip(np.dot(lhs_unit, rhs_unit), -1.0, 1.0))
    return float(np.arccos(cosine))


def _finger_curl_ratio(points: np.ndarray, indices: tuple[int, int, int, int, int]) -> float:
    angles = (
        _segment_angle(points, indices[0], indices[1], indices[2]),
        _segment_angle(points, indices[1], indices[2], indices[3]),
        _segment_angle(points, indices[2], indices[3], indices[4]),
    )
    weighted_angle = 0.55 * angles[0] + 0.30 * angles[1] + 0.15 * angles[2]
    return _clamp01(weighted_angle / 1.35)


def _thumb_curl_ratio(points: np.ndarray, indices: tuple[int, int, int, int]) -> float:
    angles = (
        _segment_angle(points, indices[0], indices[1], indices[2]),
        _segment_angle(points, indices[1], indices[2], indices[3]),
    )
    weighted_angle = 0.65 * angles[0] + 0.35 * angles[1]
    return _clamp01(weighted_angle / 1.10)


def _project_onto_plane(vector: np.ndarray, normal: np.ndarray) -> np.ndarray:
    return vector - float(np.dot(vector, normal)) * normal


def _angle_delta(angle: float, reference: float) -> float:
    return float(np.arctan2(np.sin(angle - reference), np.cos(angle - reference)))


def _signed_spread_ratio(
    direction_a: np.ndarray,
    direction_b: np.ndarray,
    normal: np.ndarray,
    *,
    max_angle_rad: float = 0.50,
) -> float:
    if not np.any(direction_a) or not np.any(direction_b) or not np.any(normal):
        return 0.0
    angle = float(np.arctan2(np.dot(np.cross(direction_b, direction_a), normal), np.dot(direction_b, direction_a)))
    return _clamp(angle / max_angle_rad, -1.0, 1.0)


def _safe_normalize(vector: np.ndarray) -> np.ndarray:
    arr = np.asarray(vector, dtype=float)
    norm = float(np.linalg.norm(arr))
    if norm < 1.0e-9:
        return np.zeros_like(arr, dtype=float)
    return arr / norm


def _skew(vector: np.ndarray) -> np.ndarray:
    x, y, z = np.asarray(vector, dtype=float).reshape(3)
    return np.array(
        [
            [0.0, -z, y],
            [z, 0.0, -x],
            [-y, x, 0.0],
        ],
        dtype=float,
    )



def _clamp01(value: float) -> float:
    return _clamp(value, 0.0, 1.0)


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, float(value)))


def resolve_robot_root(robot: str | Path) -> Path:
    path = Path(robot).expanduser()
    if path.exists():
        return path.resolve()
    candidate = ROBOTS_ROOT / str(robot)
    if candidate.exists():
        return candidate.resolve()
    raise FileNotFoundError(
        f"robot asset not found: {robot}. Pass an existing path or a name under {ROBOTS_ROOT}"
    )


def resolve_robot_assets(config: TopoRetargetConfig) -> tuple[Path, Path, Path, tuple[str, ...]]:
    robot_root = resolve_robot_root(config.robot_root)
    eval_config = _load_robot_eval_config(robot_root)
    urdf_path = _resolve_robot_file(
        robot_root,
        explicit=config.urdf_path,
        configured=eval_config.get("urdf_path"),
        default_glob=("urdf", "*.urdf"),
        label="URDF",
    )
    keypoints_path = _resolve_robot_file(
        robot_root,
        explicit=config.keypoints_path,
        configured=eval_config.get("keypoint_config"),
        default_name="robot_keypoints_21.yaml",
        label="keypoint config",
    )
    joint_names = config.joint_names
    if joint_names is None:
        active_joints = eval_config.get("active_joints")
        if isinstance(active_joints, list) and active_joints:
            joint_names = tuple(str(name) for name in active_joints)
        else:
            joint_names = JOINT_NAMES
    if not joint_names:
        raise ValueError("topo retarget joint_names cannot be empty")
    return robot_root, urdf_path, keypoints_path, tuple(joint_names)


def _load_robot_eval_config(robot_root: Path) -> dict[str, Any]:
    path = robot_root / "eval_config.yaml"
    if not path.exists():
        return {}
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if not isinstance(data, dict):
        raise ValueError(f"eval_config.yaml must contain a mapping: {path}")
    return data


def _resolve_robot_file(
    robot_root: Path,
    *,
    explicit: str | Path | None,
    configured: Any,
    label: str,
    default_name: str | None = None,
    default_glob: tuple[str, str] | None = None,
) -> Path:
    raw = explicit if explicit is not None else configured
    if raw is not None:
        path = Path(raw).expanduser()
        if not path.is_absolute():
            path = robot_root / path
        if not path.exists():
            raise FileNotFoundError(f"{label} path does not exist: {path}")
        return path.resolve()
    if default_name is not None:
        path = robot_root / default_name
        if path.exists():
            return path.resolve()
    if default_glob is not None:
        folder, pattern = default_glob
        matches = sorted((robot_root / folder).glob(pattern))
        if len(matches) == 1:
            return matches[0].resolve()
        raise FileNotFoundError(f"expected exactly one {label} under {robot_root / folder}")
    raise FileNotFoundError(f"could not resolve {label} for robot root {robot_root}")


def load_mediapipe_record_points(path: str | Path, record_id: str | None = None) -> np.ndarray:
    p = Path(path)
    if p.suffix == ".jsonl":
        first = None
        for line in p.read_text().splitlines():
            if not line.strip():
                continue
            data = json.loads(line)
            if first is None:
                first = data
            if record_id is not None and data.get("record_id") == record_id:
                return points_from_record(data)
        if record_id is not None:
            raise KeyError(f"record_id {record_id!r} not found in {p}")
        if first is None:
            raise ValueError(f"{p} contains no records")
        return points_from_record(first)
    data = json.loads(p.read_text())
    return points_from_record(data)


