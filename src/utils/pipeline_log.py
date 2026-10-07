"""Pipeline debug logging: MediaPipe → retarget → MuJoCo collision.

Enable with environment variable ``PIPELINE_LOG=1`` (default off).
Rate-limited by ``PIPELINE_LOG_INTERVAL`` seconds (default 0.5).
Set ``PIPELINE_LOG_EVERY_FRAME=1`` to log every frame.
Set ``PIPELINE_LOG_VERBOSE=1`` to dump all 21 landmarks and all joint angles.
"""

from __future__ import annotations

import json
import os
import threading
import time
from typing import Any, Mapping, Sequence

import numpy as np

from utils.log_control import emit, register_tag

TAG = "PipelineLog"
register_tag(TAG, "手势→重定向→碰撞 全链路调试")
register_tag("RetargetDiag", "Retarget 精简诊断（默认开）")
register_tag("RetargetAngleCmp", "MediaPipe角度 vs 模型角度")

_ENABLED = os.environ.get("PIPELINE_LOG", "0").strip().lower() in ("1", "true", "yes", "on")
_EVERY_FRAME = os.environ.get("PIPELINE_LOG_EVERY_FRAME", "0").strip().lower() in ("1", "true", "yes", "on")
_VERBOSE = os.environ.get("PIPELINE_LOG_VERBOSE", "0").strip().lower() in ("1", "true", "yes", "on")
_INTERVAL_S = float(os.environ.get("PIPELINE_LOG_INTERVAL", "0.5"))
# 精简 retarget 诊断：默认开（约 1 行/秒）；RETARGET_DIAG=0 关闭。
_DIAG_ENABLED = os.environ.get("RETARGET_DIAG", "1").strip().lower() not in ("0", "false", "no", "off")
_DIAG_INTERVAL_S = float(os.environ.get("RETARGET_DIAG_INTERVAL", "1.0"))
_MP_MODEL_COMPARE_ENABLED = os.environ.get("RETARGET_MP_COMPARE", "1").strip().lower() not in ("0", "false", "no", "off")
_DIAG_LAST = 0.0

_LOCK = threading.Lock()
_LAST_EMIT = 0.0
_FRAME_SEQ = 0
_FRAME_LOGGING = False

_FINGERTIP_IDS = (4, 8, 12, 16, 20)
_KEY_JOINTS = (
    "left_hand_thumb_rota_joint",
    "left_hand_thumb_MCP_joint",
    "left_hand_thumb_PIP_joint",
    "left_hand_thumb_swing_joint",
    "left_hand_index_MCP_joint",
    "left_hand_index_PIP_joint",
    "left_hand_index_swing_joint",
    "left_hand_mid_MCP_joint",
    "left_hand_mid_swing_joint",
    "left_hand_ring_MCP_joint",
    "left_hand_ring_swing_joint",
    "left_hand_pinky_MCP_joint",
    "left_hand_pinky_swing_joint",
)
_JOINT_SHORT = {
    "left_hand_thumb_rota_joint": "th_rot",
    "left_hand_thumb_MCP_joint": "th_mcp",
    "left_hand_thumb_PIP_joint": "th_pip",
    "left_hand_thumb_swing_joint": "th_sw",
    "left_hand_index_MCP_joint": "ix_mcp",
    "left_hand_index_PIP_joint": "ix_pip",
    "left_hand_index_swing_joint": "ix_sw",
    "left_hand_mid_MCP_joint": "md_mcp",
    "left_hand_mid_PIP_joint": "md_pip",
    "left_hand_mid_swing_joint": "md_sw",
    "left_hand_ring_MCP_joint": "rg_mcp",
    "left_hand_ring_PIP_joint": "rg_pip",
    "left_hand_ring_swing_joint": "rg_sw",
    "left_hand_pinky_MCP_joint": "pk_mcp",
    "left_hand_pinky_PIP_joint": "pk_pip",
    "left_hand_index_DIP_joint": "ix_dip",
    "left_hand_mid_DIP_joint": "md_dip",
    "left_hand_ring_DIP_joint": "rg_dip",
    "left_hand_pinky_DIP_joint": "pk_dip",
    "left_hand_pinky_swing_joint": "pk_sw",
}

_MP_ANGLE_TRIPLETS: tuple[tuple[str, int, int, int], ...] = (
    ("left_hand_thumb_MCP_joint", 1, 2, 3),
    ("left_hand_thumb_PIP_joint", 2, 3, 4),
    ("left_hand_index_MCP_joint", 0, 5, 6),
    ("left_hand_index_PIP_joint", 5, 6, 7),
    ("left_hand_index_DIP_joint", 6, 7, 8),
    ("left_hand_mid_MCP_joint", 0, 9, 10),
    ("left_hand_mid_PIP_joint", 9, 10, 11),
    ("left_hand_mid_DIP_joint", 10, 11, 12),
    ("left_hand_ring_MCP_joint", 0, 13, 14),
    ("left_hand_ring_PIP_joint", 13, 14, 15),
    ("left_hand_ring_DIP_joint", 14, 15, 16),
    ("left_hand_pinky_MCP_joint", 0, 17, 18),
    ("left_hand_pinky_PIP_joint", 17, 18, 19),
    ("left_hand_pinky_DIP_joint", 18, 19, 20),
)


def enabled() -> bool:
    return _ENABLED


def diag_enabled() -> bool:
    return _DIAG_ENABLED


def _allow_emit() -> bool:
    global _LAST_EMIT
    if not _ENABLED:
        return False
    if _EVERY_FRAME:
        return True
    with _LOCK:
        if _FRAME_LOGGING:
            return True
        now = time.monotonic()
        if now - _LAST_EMIT < _INTERVAL_S:
            return False
        _LAST_EMIT = now
        return True


def next_frame_id() -> int:
    """Start a new pipeline frame; subsequent logs in this burst are not rate-limited."""
    global _FRAME_SEQ, _FRAME_LOGGING
    with _LOCK:
        _FRAME_SEQ += 1
        _FRAME_LOGGING = True
        return _FRAME_SEQ


def end_frame() -> None:
    """End the current pipeline frame burst (re-enables interval throttling)."""
    global _FRAME_LOGGING, _LAST_EMIT
    with _LOCK:
        if _FRAME_LOGGING:
            _FRAME_LOGGING = False
            _LAST_EMIT = time.monotonic()


def _fmt_xyz(p: Sequence[float]) -> str:
    return f"({float(p[0]):+.4f},{float(p[1]):+.4f},{float(p[2]):+.4f})"


def _landmarks_summary(arr: np.ndarray) -> str:
    pts = np.asarray(arr, dtype=float)
    if pts.shape != (21, 3):
        return f"shape={pts.shape}"
    parts = [f"wrist={_fmt_xyz(pts[0])}"]
    for idx in _FINGERTIP_IDS:
        parts.append(f"tip[{idx}]={_fmt_xyz(pts[idx])}")
    palm_w = float(np.linalg.norm(pts[5] - pts[17]))
    parts.append(f"palm_w={palm_w:.4f}")
    if _VERBOSE:
        parts.append("all21=" + json.dumps(np.round(pts, 4).tolist(), ensure_ascii=False))
    return " | ".join(parts)


def _joints_summary(joints: Mapping[str, float], *, prefix: str = "", compact: bool = False) -> str:
    if not joints:
        return f"{prefix}(empty)"
    parts = []
    for name in _KEY_JOINTS:
        if name in joints:
            short = _JOINT_SHORT.get(name, name)
            parts.append(f"{short}={float(joints[name]):+.3f}")
    vals = np.array(list(joints.values()), dtype=float)
    parts.append(f"n={len(joints)} min={vals.min():+.3f} max={vals.max():+.3f}")
    if _VERBOSE and not compact:
        ordered = {k: round(float(v), 4) for k, v in sorted(joints.items())}
        parts.append("all=" + json.dumps(ordered, ensure_ascii=False))
    return f"{prefix}" + " | ".join(parts)


def _joint_delta(before: Mapping[str, float], after: Mapping[str, float]) -> str:
    names = sorted(set(before) | set(after))
    if not names:
        return "delta=(none)"
    deltas = []
    max_name = ""
    max_abs = 0.0
    for name in names:
        d = float(after.get(name, before.get(name, 0.0))) - float(before.get(name, 0.0))
        deltas.append(abs(d))
        if abs(d) > max_abs:
            max_abs = abs(d)
            max_name = name
    arr = np.array(deltas, dtype=float)
    return (
        f"delta_n={len(names)} max|dq|={max_abs:.4f}@{max_name} "
        f"mean|dq|={float(arr.mean()):.4f} rms|dq|={float(np.sqrt((arr**2).mean())):.4f}"
    )


def _joint_flex_angle(points: np.ndarray, a: int, b: int, c: int) -> float:
    """Return flexion proxy in radians: 0=open, larger=more curled."""
    ba = np.asarray(points[a], dtype=float) - np.asarray(points[b], dtype=float)
    bc = np.asarray(points[c], dtype=float) - np.asarray(points[b], dtype=float)
    nba = float(np.linalg.norm(ba))
    nbc = float(np.linalg.norm(bc))
    if nba <= 1.0e-9 or nbc <= 1.0e-9:
        return 0.0
    cosine = float(np.clip(np.dot(ba, bc) / (nba * nbc), -1.0, 1.0))
    interior = float(np.arccos(cosine))
    return float(np.clip(np.pi - interior, 0.0, np.pi))


def _estimate_mediapipe_joint_angles(landmarks_21: np.ndarray) -> dict[str, float]:
    pts = np.asarray(landmarks_21, dtype=float)
    if pts.shape != (21, 3) or not np.isfinite(pts).all():
        return {}
    out: dict[str, float] = {}
    for name, a, b, c in _MP_ANGLE_TRIPLETS:
        out[name] = _joint_flex_angle(pts, a, b, c)
    return out


def _mp_model_compare_line(landmarks_21: np.ndarray, joint_values: Mapping[str, float]) -> str:
    mp = _estimate_mediapipe_joint_angles(landmarks_21)
    parts = []
    max_name = ""
    max_abs = 0.0
    for name, mp_val in mp.items():
        if name not in joint_values:
            continue
        model_val = float(joint_values[name])
        delta = model_val - float(mp_val)
        if abs(delta) > max_abs:
            max_abs = abs(delta)
            max_name = name
        short = _JOINT_SHORT.get(name, name)
        parts.append(f"{short}:mp={mp_val:+.3f} mdl={model_val:+.3f} d={delta:+.3f}")
    if not parts:
        return "cmp=(no overlap joints)"
    return f"max|d|={max_abs:.3f}@{_JOINT_SHORT.get(max_name, max_name)} | " + " ; ".join(parts)


def log(stage: str, message: str) -> None:
    if not _allow_emit():
        return
    emit(f"[{stage}] {message}", TAG)


def log_mediapipe_input(landmarks_21: np.ndarray, *, frame_id: int) -> None:
    if not _allow_emit():
        return
    emit(f"#{frame_id} [1.MP_IN] {_landmarks_summary(landmarks_21)}", TAG)


def log_retarget_start(algo: str, landmarks_21: np.ndarray, *, frame_id: int) -> None:
    if not _allow_emit():
        return
    emit(f"#{frame_id} [2.RETARGET_IN] algo={algo} {_landmarks_summary(landmarks_21)}", TAG)


def log_landmarks_repair(repaired_indices: Sequence[int], *, frame_id: int) -> None:
    if not _allow_emit():
        return
    ids = sorted({int(i) for i in repaired_indices})
    emit(f"#{frame_id} [1b.MP_REPAIR] filled_landmarks={ids}", TAG)


def log_topo_output(
    *,
    frame_id: int,
    joint_values: Mapping[str, float],
    success: bool,
    loss: Mapping[str, Any] | None = None,
    diagnostics: Mapping[str, Any] | None = None,
) -> None:
    if not _allow_emit():
        return
    loss = loss or {}
    diag = diagnostics or {}
    solver = diag.get("solver", "?")
    topo_iters = diag.get("topo_num_iters", "?")
    init_iters = diag.get("init_num_iters", "?")
    converged = diag.get("topo_converged", diag.get("init_converged"))
    msg = diag.get("message", "")
    loss_total = loss.get("total", loss.get("weighted_total"))
    loss_str = f" loss_total={loss_total:.6f}" if isinstance(loss_total, (int, float)) else ""
    q_init = diag.get("q_init_joint_values")
    q_init_str = f" q_init={_joints_summary(q_init)}" if isinstance(q_init, Mapping) and q_init else ""
    emit(
        f"#{frame_id} [3.TOPO_OUT] ok={success} solver={solver} "
        f"init_iters={init_iters} topo_iters={topo_iters} converged={converged}{loss_str}{q_init_str} "
        f"{_joints_summary(joint_values)} msg={msg!r}",
        TAG,
    )
    if _VERBOSE and loss:
        emit(f"#{frame_id} [3.TOPO_LOSS] {json.dumps(loss, ensure_ascii=False, default=str)}", TAG)
    if _VERBOSE and diag:
        compact = {
            k: diag[k]
            for k in (
                "initialization",
                "topo_initial_objective",
                "topo_final_objective",
                "topo_objective_improvement",
                "num_edges",
                "parameters",
            )
            if k in diag
        }
        emit(f"#{frame_id} [3.TOPO_DIAG] {json.dumps(compact, ensure_ascii=False, default=str)}", TAG)


def log_TsenseI_output(joint_values: Mapping[str, float], *, frame_id: int) -> None:
    if not _allow_emit():
        return
    emit(f"#{frame_id} [3.TsenseI_OUT] {_joints_summary(joint_values)}", TAG)


def log_retarget_miss(algo: str, reason: str, *, frame_id: int) -> None:
    if not _allow_emit():
        return
    emit(f"#{frame_id} [3.RETARGET_MISS] algo={algo} reason={reason}", TAG)


def log_physics_input(joint_values: Mapping[str, float], *, frame_id: int, active: bool) -> None:
    if not _allow_emit():
        return
    state = "ON" if active else "OFF(skip)"
    emit(f"#{frame_id} [4.PHYS_IN] collision={state} {_joints_summary(joint_values)}", TAG)


def log_physics_output(
    before: Mapping[str, float],
    after: Mapping[str, float],
    *,
    frame_id: int,
    ncon: int = -1,
    elapsed_ms: float | None = None,
    skipped: bool = False,
) -> None:
    if not _allow_emit():
        return
    if skipped:
        emit(f"#{frame_id} [5.PHYS_OUT] skipped (collision off) {_joints_summary(after)}", TAG)
        return
    extra = []
    if ncon >= 0:
        extra.append(f"ncon={ncon}")
    if elapsed_ms is not None:
        extra.append(f"ms={elapsed_ms:.2f}")
    suffix = (" " + " ".join(extra)) if extra else ""
    emit(
        f"#{frame_id} [5.PHYS_OUT]{suffix} {_joints_summary(after)} | {_joint_delta(before, after)}",
        TAG,
    )


def log_emit_joints(joint_values: Mapping[str, float], *, frame_id: int) -> None:
    if not _allow_emit():
        return
    emit(f"#{frame_id} [6.EMIT] ->URDF/USB {_joints_summary(joint_values)}", TAG)


def _finger_geometry(arr: np.ndarray) -> str:
    """Heuristic open/curl metrics from wrist-frame 21 landmarks."""
    pts = np.asarray(arr, dtype=float)
    if pts.shape != (21, 3) or not np.isfinite(pts).all():
        return "geom=invalid"
    palm_w = float(np.linalg.norm(pts[5] - pts[17]))
    # tip-to-wrist distances (larger ≈ more open)
    tips = []
    for name, idx in (("th", 4), ("ix", 8), ("md", 12), ("rg", 16), ("pk", 20)):
        tips.append(f"{name}={float(np.linalg.norm(pts[idx] - pts[0])):.3f}")
    # spread: index-pinky tip distance
    spread = float(np.linalg.norm(pts[8] - pts[20]))
    return f"palm_w={palm_w:.3f} spread={spread:.3f} tip_len[{' '.join(tips)}]"


def log_retarget_diag(
    landmarks_21: np.ndarray | None,
    joint_values: Mapping[str, float] | None,
    *,
    frame_id: int = 0,
    success: bool | None = None,
    diagnostics: Mapping[str, Any] | None = None,
) -> None:
    """One-line retarget health check (default on, rate-limited)."""
    global _DIAG_LAST
    if not _DIAG_ENABLED:
        return
    now = time.monotonic()
    with _LOCK:
        if now - _DIAG_LAST < _DIAG_INTERVAL_S:
            return
        _DIAG_LAST = now

    parts = [f"#{frame_id}"]
    if landmarks_21 is not None:
        parts.append(_finger_geometry(np.asarray(landmarks_21, dtype=float)))
    if joint_values is not None:
        parts.append(_joints_summary(joint_values, compact=True))
    else:
        parts.append("joints=(none)")
    if success is not None:
        parts.append(f"ok={success}")
    diag = diagnostics or {}
    if "solve_ms" in diag:
        parts.append(f"ms={diag['solve_ms']}")
    if "topo_objective_improvement" in diag:
        parts.append(f"topo_dJ={diag['topo_objective_improvement']}")
    if "topo_converged" in diag:
        parts.append(f"conv={diag['topo_converged']}")
    parameters = diag.get("parameters") or {}
    if parameters:
        parts.append(f"prior={parameters.get('lambda_smooth')}/{parameters.get('lambda_reg')}")
        parts.append(f"iters={parameters.get('init_max_iters')}/{parameters.get('topo_max_iters')}")
    init = diag.get("initialization") or {}
    if isinstance(init, Mapping) and init.get("message"):
        parts.append(f"init={str(init.get('message'))[:60]!r}")
    msg = diag.get("message")
    if msg:
        parts.append(f"msg={str(msg)[:60]!r}")
    emit(" | ".join(parts), "RetargetDiag")
    if (
        _MP_MODEL_COMPARE_ENABLED
        and landmarks_21 is not None
        and joint_values is not None
    ):
        try:
            cmp_line = _mp_model_compare_line(np.asarray(landmarks_21, dtype=float), joint_values)
            emit(f"#{frame_id} | {cmp_line}", "RetargetAngleCmp")
        except Exception as exc:  # pragma: no cover - debug logging must never break pipeline
            emit(f"#{frame_id} | cmp_error={exc}", "RetargetAngleCmp")
