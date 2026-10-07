"""Runtime retarget: TsenseI_retargeting TopoRetargeter (Form3 / tracker)."""

from __future__ import annotations

import os
import sys
from dataclasses import dataclass, fields
from pathlib import Path
import threading
import time
from typing import Any, Literal

import numpy as np
from .log_control import emit as log_emit

os.environ.setdefault("KMP_DUPLICATE_LIB_OK", "TRUE")
os.environ.setdefault("OMP_NUM_THREADS", "1")

# TsenseI_retargeting lives at repo root; live app usually puts only src/ on PYTHONPATH.
_REPO_ROOT = Path(__file__).resolve().parents[2]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

RetargetAlgorithm = Literal["topo"]

# MediaPipe Hands landmark count (must match TsenseI_retargeting.landmarks.NUM_LANDMARKS).
NUM_HAND_POINTS = 21

# ``pinky_rota`` is mechanically fixed in the light pipeline.  It must not be
# an optimisation variable: merely overwriting it after TopoRetarget would
# still let the solver use that DOF to reduce landmark loss and would bias the
# remaining 20 angles.  The joint is injected back into public poses at zero so
# the URDF, Ruckig and H7 conversion keep their stable 21-joint contract.
_FIXED_LEFT_JOINT_VALUES: dict[str, float] = {
    "left_hand_pinky_rota_joint": 0.0,
}


@dataclass
class ParamSpec:
    key: str
    label: str
    kind: str  # float | int | bool
    default: float
    minimum: float
    maximum: float
    step: float = 0.01
    group: str = "general"

    def slider_range(self) -> tuple[int, int, int]:
        """Return (min, max, default) in slider integer units."""
        if self.kind == "bool":
            return 0, 1, 1 if self.default else 0
        scale = int(round(1.0 / self.step))
        return (
            int(round(self.minimum * scale)),
            int(round(self.maximum * scale)),
            int(round(self.default * scale)),
        )

    def from_slider(self, value: int) -> Any:
        if self.kind == "bool":
            return bool(value)
        if self.kind == "int":
            return int(value)
        return float(value) * self.step

    def format_value(self, value: Any) -> str:
        if self.kind == "bool":
            return "开" if value else "关"
        if self.kind == "int":
            return str(int(value))
        return f"{float(value):.3f}"


# V2 实时参数与 assets/hands/TsenseI_hand/hand_profile.yaml 对齐。
# 两阶段历史先验过大会让握拳后的伸展逐帧爬行，碰撞输出回灌会进一步放大。
# 保留小先验用于欠观测选解；速度/加速度/jerk 约束仍由 Ruckig 负责。
TOPO_PARAM_SPECS: tuple[ParamSpec, ...] = (
    ParamSpec("use_topo_optimization", "拓扑优化", "bool", 1.0, 0.0, 1.0, 1.0, "solver"),
    ParamSpec("pinch_enabled", "拇指与四指捏合", "bool", 1.0, 0.0, 1.0, 1.0, "weights"),
    ParamSpec("pinch_full_ratio", "完整捏合/掌宽", "float", 0.08, 0.0, 1.49, 0.01, "weights"),
    ParamSpec("pinch_start_ratio", "开始吸合/掌宽", "float", 0.50, 0.01, 1.5, 0.01, "weights"),
    ParamSpec("pinch_weight", "捏合权重", "float", 65.0, 0.0, 1000.0, 1.0, "weights"),
    ParamSpec("lambda_warm", "骨方向(初)", "float", 2.0, 0.0, 20.0, 0.1, "weights"),
    ParamSpec("lambda_smooth", "逆解先验(初)", "float", 0.1, 0.0, 20.0, 0.1, "weights"),
    ParamSpec("lambda_im", "交互网格", "float", 12.0, 0.0, 50.0, 0.5, "weights"),
    ParamSpec("kappa", "边衰减κ", "float", 30.0, 1.0, 100.0, 0.5, "weights"),
    ParamSpec(
        "interaction_graph_ema_alpha",
        "拓扑连续系数",
        "float",
        0.5,
        0.1,
        1.0,
        0.05,
        "solver",
    ),
    ParamSpec("lambda_bone", "骨方向(拓扑)", "float", 0.5, 0.0, 10.0, 0.05, "weights"),
    ParamSpec(
        "thumb_root_bone_weight",
        "拇指根骨向",
        "float",
        1.0,
        0.5,
        5.0,
        0.1,
        "weights",
    ),
    ParamSpec("lambda_init", "贴初始化", "float", 0.1, 0.0, 10.0, 0.05, "weights"),
    ParamSpec("lambda_reg", "逆解先验(拓扑)", "float", 0.1, 0.0, 20.0, 0.1, "weights"),
    # 实时默认少迭代：完整精度可在 Form3 按需调高
    ParamSpec("init_max_iters", "初始化迭代", "int", 2.0, 1.0, 20.0, 1.0, "solver"),
    ParamSpec("topo_max_iters", "拓扑迭代", "int", 3.0, 1.0, 20.0, 1.0, "solver"),
    ParamSpec("ftol", "收敛 ftol", "float", 1.0e-5, 1.0e-7, 1.0e-3, 1.0e-6, "solver"),
    ParamSpec("daqp_hessian_damping", "Hessian 阻尼", "float", 1.0e-6, 1.0e-9, 1.0e-3, 1.0e-7, "solver"),
    ParamSpec(
        "daqp_init_step_limit_rad",
        "初始化步长(rad)",
        "float",
        0.35,
        0.05,
        1.0,
        0.01,
        "solver",
    ),
    ParamSpec(
        "daqp_topo_step_limit_rad",
        "拓扑步长(rad)",
        "float",
        0.15,
        0.02,
        0.5,
        0.01,
        "solver",
    ),
)

ALGORITHM_LABELS: dict[RetargetAlgorithm, str] = {
    "topo": "Topo 拓扑重定向",
}

_lock = threading.Lock()
_retarget_lock = threading.RLock()
_algorithm: RetargetAlgorithm = "topo"
_topo_values: dict[str, Any] = {spec.key: spec.default for spec in TOPO_PARAM_SPECS}

_topo_retargeter = None
_topo_ok = False
_topo_import_error: str | None = None
_topo_error_logged = False
_last_valid_landmarks: np.ndarray | None = None
_active_hand_side: Literal["left", "right"] = "left"
_last_solve_ms: float = 0.0

_MP_PARENT: dict[int, int] = {
    1: 0, 2: 1, 3: 2, 4: 3,
    5: 0, 6: 5, 7: 6, 8: 7,
    9: 0, 10: 9, 11: 10, 12: 11,
    13: 0, 14: 13, 15: 14, 16: 15,
    17: 0, 18: 17, 19: 18, 20: 19,
}


def _specs_for(algo: RetargetAlgorithm) -> tuple[ParamSpec, ...]:
    return TOPO_PARAM_SPECS


def get_algorithm() -> RetargetAlgorithm:
    with _lock:
        return _algorithm


def set_algorithm(algo: str) -> bool:
    global _algorithm
    if algo not in ALGORITHM_LABELS:
        return False
    with _lock:
        _algorithm = algo  # type: ignore[assignment]
    _ensure_topo_ready()
    return True


def list_algorithms() -> list[tuple[RetargetAlgorithm, str]]:
    return list(ALGORITHM_LABELS.items())


def get_param_specs(algo: RetargetAlgorithm | None = None) -> tuple[ParamSpec, ...]:
    return _specs_for(algo or get_algorithm())


def get_param_values(algo: RetargetAlgorithm | None = None) -> dict[str, Any]:
    with _lock:
        return dict(_topo_values)


def set_param_value(key: str, value: Any, algo: RetargetAlgorithm | None = None) -> bool:
    algo = algo or get_algorithm()
    specs = {spec.key: spec for spec in _specs_for(algo)}
    if key not in specs:
        return False
    spec = specs[key]
    if key.startswith("pinch_") and spec.kind != "bool" and not np.isfinite(float(value)):
        return False
    if spec.kind == "bool":
        value = bool(value)
    elif spec.kind == "int":
        value = int(round(float(value)))
    else:
        value = float(value)
        value = max(spec.minimum, min(spec.maximum, value))
    with _lock:
        if key == "pinch_full_ratio" and value >= _topo_values["pinch_start_ratio"]:
            return False
        if key == "pinch_start_ratio" and value <= _topo_values["pinch_full_ratio"]:
            return False
        _topo_values[key] = value
    _apply_runtime_params(algo)
    return True


def reset_params_to_defaults(algo: RetargetAlgorithm | None = None) -> None:
    algo = algo or get_algorithm()
    with _lock:
        for spec in TOPO_PARAM_SPECS:
            _topo_values[spec.key] = spec.default
    _apply_runtime_params(algo)


def _apply_runtime_params(algo: RetargetAlgorithm) -> None:
    if _topo_retargeter is not None:
        _sync_topo_config()


def _coerce_topo_field(spec: ParamSpec, value: Any) -> Any:
    if spec.kind == "bool":
        return bool(value)
    if spec.kind == "int":
        return int(round(float(value)))
    return float(value)


def _build_topo_config():
    """Build frozen TopoRetargetConfig from Form3 values + HAND_MODEL_DIR."""
    from config_path import HAND_MODEL_DIR
    from TsenseI_retargeting import TopoRetargetConfig
    from TsenseI_retargeting.toporetarget import JOINT_NAMES

    specs = {spec.key: spec for spec in TOPO_PARAM_SPECS}
    kwargs: dict[str, Any] = {
        "robot_root": HAND_MODEL_DIR,
        "joint_names": tuple(
            name for name in JOINT_NAMES if name not in _FIXED_LEFT_JOINT_VALUES
        ),
    }
    for field in fields(TopoRetargetConfig):
        if field.name in _topo_values:
            raw = _topo_values[field.name]
            if field.name in specs:
                raw = _coerce_topo_field(specs[field.name], raw)
            kwargs[field.name] = raw
    return TopoRetargetConfig(**kwargs)


def _rebuild_retargeter(cfg, *, keep_warm_start: bool = True):
    """Construct TopoRetargeter; optionally preserve previous_q warm-start."""
    global _topo_retargeter
    from TsenseI_retargeting import TopoRetargeter

    prev_q = None
    prev_init = None
    if keep_warm_start and _topo_retargeter is not None:
        prev_q = _topo_retargeter.previous_q
        prev_init = _topo_retargeter.previous_init_q
    retargeter = TopoRetargeter(cfg)
    if keep_warm_start:
        expected_shape = (len(retargeter.joint_names),)
        if prev_q is not None and np.asarray(prev_q).shape == expected_shape:
            retargeter.previous_q = prev_q
        if prev_init is not None and np.asarray(prev_init).shape == expected_shape:
            retargeter.previous_init_q = prev_init
    _topo_retargeter = retargeter
    return retargeter


def _sync_topo_config() -> None:
    """Rebuild retargeter when Form3 params diverge (config is frozen)."""
    global _topo_retargeter
    if _topo_retargeter is None:
        return
    new_cfg = _build_topo_config()
    if new_cfg == _topo_retargeter.config:
        return
    _rebuild_retargeter(new_cfg, keep_warm_start=True)


def _ensure_topo_ready() -> bool:
    global _topo_retargeter, _topo_ok, _topo_import_error, _topo_error_logged

    if _topo_ok and _topo_retargeter is not None:
        try:
            n_kp = len(_topo_retargeter.fk.keypoints.keypoints)
        except Exception:
            n_kp = -1
        if n_kp != NUM_HAND_POINTS:
            _topo_retargeter = None
            _topo_ok = False
        else:
            _sync_topo_config()
            return True
    try:
        cfg = _build_topo_config()
        _rebuild_retargeter(cfg, keep_warm_start=False)
        _topo_ok = True
        _topo_import_error = None
        _topo_error_logged = False
        return True
    except Exception as exc:
        _topo_retargeter = None
        _topo_ok = False
        _topo_import_error = str(exc)
        if not _topo_error_logged:
            _topo_error_logged = True
            log_emit(f"Topo 初始化失败: {exc}", "RetargetRuntime")
        return False


def is_retarget_available(algo: RetargetAlgorithm | None = None) -> bool:
    return _ensure_topo_ready()


def get_retarget_status() -> dict[str, Any]:
    return {
        "algorithm": get_algorithm(),
        "topo_available": _topo_ok,
        "topo_error": _topo_import_error,
        "params": get_param_values(),
    }


def _fill_landmark_index(out: np.ndarray, idx: int, repaired: list[int]) -> bool:
    if np.isfinite(out[idx]).all():
        return True
    if _last_valid_landmarks is not None and _last_valid_landmarks.shape == (NUM_HAND_POINTS, 3):
        if np.isfinite(_last_valid_landmarks[idx]).all():
            out[idx] = _last_valid_landmarks[idx]
            repaired.append(idx)
            return True
    cur = idx
    while cur in _MP_PARENT:
        parent = _MP_PARENT[cur]
        if np.isfinite(out[parent]).all():
            if (
                _last_valid_landmarks is not None
                and np.isfinite(_last_valid_landmarks[idx]).all()
                and np.isfinite(_last_valid_landmarks[parent]).all()
            ):
                out[idx] = out[parent] + (_last_valid_landmarks[idx] - _last_valid_landmarks[parent])
            else:
                out[idx] = out[parent]
            repaired.append(idx)
            return True
        cur = parent
    return False


def _sanitize_landmarks(keypoints: np.ndarray) -> tuple[np.ndarray | None, list[int]]:
    """Replace per-landmark NaN/Inf using last valid frame or parent-joint fallback."""
    global _last_valid_landmarks
    out = np.asarray(keypoints, dtype=np.float64).copy()
    repaired: list[int] = []
    if not np.any(~np.isfinite(out)):
        _last_valid_landmarks = out.copy()
        return out, repaired
    for idx in range(NUM_HAND_POINTS):
        if not np.isfinite(out[idx]).all():
            if not _fill_landmark_index(out, idx, repaired):
                return None, repaired
    if np.any(~np.isfinite(out)):
        return None, repaired
    _last_valid_landmarks = out.copy()
    return out, repaired


def _topo_solver_failed(diagnostics: dict[str, Any]) -> bool:
    """Return True only on hard failure — do not reset warm-start on benign stalls."""
    init_info = diagnostics.get("initialization") or {}
    topo_imp = diagnostics.get("topo_objective_improvement")
    init_imp = init_info.get("objective_improvement")
    if isinstance(topo_imp, (int, float)) and topo_imp > 1.0e-9:
        return False
    if isinstance(init_imp, (int, float)) and init_imp > 1.0e-9:
        return False
    if bool(diagnostics.get("topo_converged")) or bool(init_info.get("converged")):
        return False
    solve_msg = str(diagnostics.get("message", ""))
    init_msg = str(init_info.get("message", ""))
    if "stalled after improvement" in solve_msg or "stalled after improvement" in init_msg:
        return False
    if "warm-start" in init_msg:
        return False
    if "iteration limit reached" in solve_msg or "iteration limit reach" in init_msg:
        return False
    if "non-finite" in solve_msg or "non-finite" in init_msg:
        return True
    if "DAQP failed" in solve_msg or "DAQP failed" in init_msg:
        return True
    return False


def _normalize_hand_side(hand_side: str | None) -> Literal["left", "right"]:
    side = str(hand_side or "left").strip().lower()
    return "right" if side.startswith("r") else "left"


def _to_left_joint_names(joint_values: dict[str, float] | None) -> dict[str, float] | None:
    if not joint_values:
        return None
    out: dict[str, float] = {}
    for name, val in joint_values.items():
        if name.startswith("right_hand_"):
            out["left_hand_" + name[len("right_hand_"):]] = float(val)
        else:
            out[str(name)] = float(val)
    return out


def _to_right_joint_names(joint_values: dict[str, float]) -> dict[str, float]:
    out: dict[str, float] = {}
    for name, val in joint_values.items():
        if name.startswith("left_hand_"):
            out["right_hand_" + name[len("left_hand_"):]] = float(val)
        else:
            out[str(name)] = float(val)
    return out


def _with_fixed_left_joints(joint_values: dict[str, float]) -> dict[str, float]:
    out = dict(joint_values)
    out.update(_FIXED_LEFT_JOINT_VALUES)
    return out


def _prepare_keypoints_for_solver(
    keypoints: np.ndarray,
    hand_side: str,
) -> np.ndarray:
    """Map physical-hand wrist points into the shared left-hand solver.

    ``apply_mediapipe_transformations`` and Topo's palm frame are rotations;
    neither changes geometric chirality.  The solver asset is a left hand, so
    a true right-hand point set needs exactly one reflection at this boundary.
    Mirrored URDFs use identical joint values; output only renames joints.
    """
    side = _normalize_hand_side(hand_side)
    solver_points = np.asarray(keypoints, dtype=np.float64).copy()
    if side == "right":
        solver_points[:, 0] *= -1.0
    return solver_points


def set_retarget_hand_side(hand_side: str) -> None:
    """Update active hand side for retarget runtime."""
    global _active_hand_side
    side = _normalize_hand_side(hand_side)
    if side == _active_hand_side:
        return
    _active_hand_side = side
    # Side switch changes warm-start semantics; clear solver state.
    reset_retarget_state()


def retarget_hand(
    landmarks_21,
    *,
    frame_id: int | None = None,
    previous_joint_values: dict[str, float] | None = None,
    hand_side: str = "left",
) -> dict[str, float] | None:
    """Unified retarget entry used by skeleton tracker (MediaPipe 21 → joint dict)."""
    from utils import pipeline_log

    algo = get_algorithm()
    if isinstance(landmarks_21, list):
        keypoints = np.array(landmarks_21, dtype=np.float64)
    else:
        keypoints = np.asarray(landmarks_21, dtype=np.float64)
    fid = int(frame_id) if frame_id is not None else 0
    side = _normalize_hand_side(hand_side or _active_hand_side)
    if keypoints.shape != (NUM_HAND_POINTS, 3):
        if pipeline_log.enabled():
            pipeline_log.log_retarget_miss(algo, f"bad shape {keypoints.shape}", frame_id=fid)
        return None
    repaired: list[int] = []
    if np.any(~np.isfinite(keypoints)):
        keypoints, repaired = _sanitize_landmarks(keypoints)
        if keypoints is None:
            if _topo_retargeter is not None:
                _topo_retargeter.reset()
            if pipeline_log.enabled():
                pipeline_log.log_retarget_miss(algo, "NaN/Inf in landmarks (unrepairable)", frame_id=fid)
            return None
        if repaired and pipeline_log.enabled():
            pipeline_log.log_landmarks_repair(repaired, frame_id=fid)

    if pipeline_log.enabled():
        pipeline_log.log_retarget_start(algo, keypoints, frame_id=fid)

    if not _ensure_topo_ready() or _topo_retargeter is None:
        if pipeline_log.enabled():
            pipeline_log.log_retarget_miss(
                algo,
                _topo_import_error or "topo retargeter not ready",
                frame_id=fid,
            )
        return None
    with _retarget_lock:
        global _last_solve_ms
        _sync_topo_config()
        keypoints_in = _prepare_keypoints_for_solver(keypoints, side)
        prev_in = (
            _to_left_joint_names(previous_joint_values)
            if side == "right"
            else previous_joint_values
        )
        t0 = time.perf_counter()
        result = _topo_retargeter.retarget(
            keypoints_in,
            previous_joint_values=prev_in,
            # 控制链只需要joint_values；全链路调试时再生成21点和完整loss。
            include_artifacts=pipeline_log.enabled(),
        )
        solve_ms = (time.perf_counter() - t0) * 1000.0
        _last_solve_ms = float(solve_ms)
        joint_values = _with_fixed_left_joints(dict(result.joint_values))
        if pipeline_log.enabled():
            pipeline_log.log_topo_output(
                frame_id=fid,
                joint_values=joint_values,
                success=result.success,
                loss=result.loss,
                diagnostics=result.diagnostics,
            )
        diag = dict(result.diagnostics or {})
        diag["solve_ms"] = round(solve_ms, 2)
        pipeline_log.log_retarget_diag(
            keypoints,
            joint_values,
            frame_id=fid,
            success=result.success,
            diagnostics=diag,
        )
        if _topo_solver_failed(result.diagnostics):
            _topo_retargeter.reset()
        if side == "right":
            # 左右 URDF 在相同 q 下互为镜像。输入点云已镜像一次，
            # 输出和 warm start 只换关节前缀，不能再翻转 thumb_swing。
            return _to_right_joint_names(joint_values)
        return joint_values


def last_retarget_solve_ms() -> float:
    """最近一次 ``retarget_hand`` 的求解耗时（毫秒）。"""
    return float(_last_solve_ms)


def reset_retarget_state() -> None:
    """Reset filters / warm-start when hand tracking is lost."""
    global _last_valid_landmarks
    with _retarget_lock:
        _last_valid_landmarks = None
        if _topo_retargeter is not None:
            _topo_retargeter.reset()
