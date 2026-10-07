"""手势轨迹数据落盘：仅在 finger_linkage 逆解失败时写入。

文件：``logs/data_track_YYYYMMDD_HHMMSS.log``（每次启动一份，JSON Lines）。

对 raw（Ruckig 前）/ ruckig（Ruckig 后）姿态分别做严格逆解（无 soft-clamp）：
  1) UI 空间
  2) HW 空间（``convert_ui_to_hardware`` 之后）
任一侧有失败才写一行，含 21 角、行程与 ``ik_failures`` 详情。

环境变量：
  DATA_TRACK_LOG=1          开启（默认关；开启会在下发线程做 IK 探测，易卡顿）
  DATA_TRACK_RUCKIG_HZ=10   Ruckig 侧失败探测上限频率
"""
from __future__ import annotations

import json
import math
import os
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional

from config_path import LOGS_DIR

# 默认关闭：火焰图显示 log_ruckig/_log_if_ik_fail 可占下发路径数个百分点
_ENABLED = os.environ.get("DATA_TRACK_LOG", "0").strip().lower() in (
    "1", "true", "yes", "on",
)
# Ruckig 采样约 500Hz，探测必须限频；默认 10Hz 足够排障
_RUCKIG_CHECK_HZ = float(os.environ.get("DATA_TRACK_RUCKIG_HZ", "10"))

_LOCK = threading.Lock()
_FILE = None  # type: ignore[var-annotated]
_PATH: Optional[Path] = None
_SESSION_KEY = ""
_SEQ = 0
_MAPPER = None
_LAST_RUCKIG_CHECK_T = 0.0

_JOINT_SHORT: tuple[str, ...] = (
    "thumb_rota",
    "thumb_swing",
    "thumb_MCP",
    "thumb_PIP",
    "index_swing",
    "index_MCP",
    "index_PIP",
    "index_DIP",
    "mid_swing",
    "mid_MCP",
    "mid_PIP",
    "mid_DIP",
    "ring_swing",
    "ring_MCP",
    "ring_PIP",
    "ring_DIP",
    "pinky_rota",
    "pinky_swing",
    "pinky_MCP",
    "pinky_PIP",
    "pinky_DIP",
)


def open_session() -> Optional[Path]:
    """确保本次会话日志文件已打开，返回路径。"""
    if not _ENABLED:
        return None
    with _LOCK:
        _ensure_file()
        return _PATH


def enabled() -> bool:
    return _ENABLED


def path() -> Optional[Path]:
    return _PATH


def _ensure_file() -> None:
    global _FILE, _PATH, _SESSION_KEY
    if _FILE is not None and _PATH is not None:
        return
    LOGS_DIR.mkdir(parents=True, exist_ok=True)
    _SESSION_KEY = datetime.now().strftime("%Y%m%d_%H%M%S")
    _PATH = LOGS_DIR / f"data_track_{_SESSION_KEY}.log"
    _FILE = open(_PATH, "a", encoding="utf-8", buffering=1)
    _FILE.write(
        f"# data_track (ik_fail only) start "
        f"{datetime.now().isoformat(timespec='milliseconds')}\n"
    )
    _FILE.flush()


def _get_mapper():
    global _MAPPER
    if _MAPPER is None:
        from kinematics.finger_linkage import create_default_hand_stroke_mapper

        _MAPPER = create_default_hand_stroke_mapper()
    return _MAPPER


def _to_canonical(cfg: Mapping[str, Any]) -> Dict[str, float]:
    out: Dict[str, float] = {}
    for raw_name, raw_val in cfg.items():
        name = str(raw_name)
        if name.startswith("right_hand_"):
            name = "left_hand_" + name[len("right_hand_") :]
        try:
            val = float(raw_val)
        except (TypeError, ValueError):
            continue
        if not math.isfinite(val):
            continue
        out[name] = val
    return out


def _angles_vec(canon: Mapping[str, float]) -> List[float]:
    return [
        round(float(canon.get(f"left_hand_{short}_joint", 0.0)), 6)
        for short in _JOINT_SHORT
    ]


def _angles_by_name(canon: Mapping[str, float]) -> Dict[str, float]:
    return {
        short: round(float(canon.get(f"left_hand_{short}_joint", 0.0)), 6)
        for short in _JOINT_SHORT
    }


def _strokes_for(
    ui_cfg: Mapping[str, float],
    *,
    hand_side: str,
) -> tuple[List[float], List[float]]:
    from protocol import convert_ui_to_hardware
    from usb.offline import joint_angles_to_hand_positions

    hw = convert_ui_to_hardware(dict(ui_cfg), hand_side=hand_side)
    linear, rotary = joint_angles_to_hand_positions(hw)
    return (
        [round(float(x), 4) for x in linear],
        [round(float(x), 4) for x in rotary],
    )


def _probe_ik_failures(
    joint_cfg: Mapping[str, float],
    *,
    space: str,
) -> List[Dict[str, Any]]:
    """严格逆解探测：不做 soft-clamp；目标角超可达范围或闭环不可解即记失败。"""
    from kinematics.finger_linkage import (
        LinkageKinematicsError,
        MOTOR_F1,
        MOTOR_F2,
        MOTOR_F3,
        cf_to_phi_deg,
        clamp_joint_rad_for_motor,
        joint_rad_to_phi_deg,
        motor_phi_deg_limits,
    )

    def _phi_to_cf_strict(phi_deg: float, motor: int, params) -> float:
        cf_lo, cf_hi = params.motor_cf_limits(motor)
        if not math.isfinite(phi_deg):
            raise LinkageKinematicsError(
                f"F{motor + 1} 逆解输入非有限 phi={phi_deg}",
                context={"motor": motor + 1, "phi_deg": phi_deg},
            )
        try:
            phi_lo = cf_to_phi_deg(cf_lo, motor, params)
            phi_hi = cf_to_phi_deg(cf_hi, motor, params)
        except LinkageKinematicsError as exc:
            raise LinkageKinematicsError(
                f"F{motor + 1} 端点 CF 不可解: {exc}",
                context={"motor": motor + 1, "phi_in_deg": float(phi_deg)},
            ) from exc
        lo_p, hi_p = (phi_lo, phi_hi) if phi_lo <= phi_hi else (phi_hi, phi_lo)
        if phi_deg < lo_p - 1e-3 or phi_deg > hi_p + 1e-3:
            raise LinkageKinematicsError(
                f"F{motor + 1} 目标角超出可达范围 phi={phi_deg:.4f} "
                f"not in [{lo_p:.4f}, {hi_p:.4f}] deg",
                context={
                    "motor": motor + 1,
                    "phi_in_deg": float(phi_deg),
                    "reachable_phi_deg": [lo_p, hi_p],
                    "phi_lim_deg": list(motor_phi_deg_limits(motor, params)),
                },
            )
        # 可达范围内：委托生产二分（内部仍会 soft-clamp，但目标已在范围内）
        from kinematics.finger_linkage import phi_deg_to_cf_mm

        return phi_deg_to_cf_mm(phi_deg, motor, params)

    mapper = _get_mapper()
    fails: List[Dict[str, Any]] = []

    for binding in mapper.finger_bindings:
        p = mapper.four_finger.params
        raw_mcp = float(joint_cfg.get(binding.mcp_joint, 0.0))
        raw_pip = float(joint_cfg.get(binding.pip_joint, 0.0))
        raw_dip = float(joint_cfg.get(binding.dip_joint, 0.0))
        c_mcp = clamp_joint_rad_for_motor(raw_mcp, binding.mcp_motor, p)
        c_pip = clamp_joint_rad_for_motor(raw_pip, binding.pip_motor, p)
        c_dip = clamp_joint_rad_for_motor(raw_dip, binding.dip_motor, p)
        base = {
            "space": space,
            "finger": binding.finger,
            "raw_rad": {
                "mcp": round(raw_mcp, 6),
                "pip": round(raw_pip, 6),
                "dip": round(raw_dip, 6),
            },
            "raw_deg": {
                "mcp": round(math.degrees(raw_mcp), 3),
                "pip": round(math.degrees(raw_pip), 3),
                "dip": round(math.degrees(raw_dip), 3),
            },
            "clamped_rad": {
                "mcp": round(c_mcp, 6),
                "pip": round(c_pip, 6),
                "dip": round(c_dip, 6),
            },
            "out_of_range": any(
                abs(a - b) > 1e-6
                for a, b in ((raw_mcp, c_mcp), (raw_pip, c_pip), (raw_dip, c_dip))
            ),
        }
        try:
            # 与 joint_rad_to_cf_mm 顺序一致：F1=DIP, F2=PIP, F3=MCP
            _phi_to_cf_strict(joint_rad_to_phi_deg(raw_dip), MOTOR_F1, p)
            _phi_to_cf_strict(joint_rad_to_phi_deg(raw_pip), MOTOR_F2, p)
            _phi_to_cf_strict(joint_rad_to_phi_deg(raw_mcp), MOTOR_F3, p)
        except LinkageKinematicsError as exc:
            fails.append(
                {
                    **base,
                    "reason": str(exc),
                    "context": getattr(exc, "context", {}) or {},
                }
            )
        except Exception as exc:  # noqa: BLE001
            fails.append({**base, "reason": f"{type(exc).__name__}: {exc}"})

    if mapper.thumb.is_configured():
        thumb = mapper.thumb
        p = thumb.params
        raw_mcp = float(joint_cfg.get(thumb.mcp_joint, 0.0))
        raw_pip = float(joint_cfg.get(thumb.pip_joint, 0.0))
        c_mcp = clamp_joint_rad_for_motor(raw_mcp, thumb.mcp_motor, p)
        c_pip = clamp_joint_rad_for_motor(raw_pip, thumb.pip_motor, p)
        base = {
            "space": space,
            "finger": "thumb",
            "raw_rad": {
                "mcp": round(raw_mcp, 6),
                "pip": round(raw_pip, 6),
            },
            "raw_deg": {
                "mcp": round(math.degrees(raw_mcp), 3),
                "pip": round(math.degrees(raw_pip), 3),
            },
            "clamped_rad": {
                "mcp": round(c_mcp, 6),
                "pip": round(c_pip, 6),
            },
            "out_of_range": abs(raw_mcp - c_mcp) > 1e-6 or abs(raw_pip - c_pip) > 1e-6,
        }
        try:
            _phi_to_cf_strict(joint_rad_to_phi_deg(raw_mcp), thumb.mcp_motor, p)
            _phi_to_cf_strict(joint_rad_to_phi_deg(raw_pip), thumb.pip_motor, p)
        except LinkageKinematicsError as exc:
            fails.append(
                {
                    **base,
                    "reason": str(exc),
                    "context": getattr(exc, "context", {}) or {},
                }
            )
        except Exception as exc:  # noqa: BLE001
            fails.append({**base, "reason": f"{type(exc).__name__}: {exc}"})
    return fails


def _collect_failures(
    ui_canon: Mapping[str, float],
    *,
    hand_side: str,
) -> List[Dict[str, Any]]:
    from protocol import convert_ui_to_hardware

    fails = _probe_ik_failures(ui_canon, space="ui")
    hw = convert_ui_to_hardware(dict(ui_canon), hand_side=hand_side)
    # HW 键仍为 left_hand_*
    fails.extend(_probe_ik_failures(hw, space="hw"))
    return fails


def _write(record: Dict[str, Any]) -> None:
    global _SEQ
    if not _ENABLED:
        return
    with _LOCK:
        _ensure_file()
        assert _FILE is not None
        _SEQ += 1
        record["seq"] = _SEQ
        record["t"] = datetime.now().isoformat(timespec="milliseconds")
        _FILE.write(json.dumps(record, ensure_ascii=False, separators=(",", ":")) + "\n")


def _log_if_ik_fail(
    cfg: Mapping[str, Any] | None,
    *,
    kind: str,
    hand_side: str,
    source: str,
) -> None:
    if not _ENABLED or not cfg:
        return
    try:
        canon = _to_canonical(cfg)
        if not canon:
            return
        side = "right" if str(hand_side).lower().startswith("r") else "left"
        fails = _collect_failures(canon, hand_side=side)
        if not fails:
            return
        linear, rotary = _strokes_for(canon, hand_side=side)
        _write(
            {
                "kind": kind,
                "source": source,
                "hand_side": side,
                "angles_rad": _angles_vec(canon),
                "angles_by_name": _angles_by_name(canon),
                "linear_mm": linear,
                "rotary_deg": rotary,
                "ik_failures": fails,
            }
        )
    except Exception as exc:  # noqa: BLE001
        with _LOCK:
            _ensure_file()
            if _FILE is not None:
                _FILE.write(f"# {kind} ik_fail log error: {exc}\n")


def log_raw(
    cfg: Mapping[str, Any] | None,
    *,
    hand_side: str = "left",
    source: str = "tracker",
) -> None:
    """仅在逆解失败时记录 Ruckig 前的原始姿态。"""
    _log_if_ik_fail(cfg, kind="raw", hand_side=hand_side, source=source)


def log_ruckig(
    cfg: Mapping[str, Any] | None,
    *,
    hand_side: str = "left",
    source: str = "sample",
) -> None:
    """仅在逆解失败时记录 Ruckig 后的平滑姿态（探测限频）。"""
    global _LAST_RUCKIG_CHECK_T
    if not _ENABLED or not cfg:
        return
    now = time.perf_counter()
    min_dt = 1.0 / max(_RUCKIG_CHECK_HZ, 1.0)
    if now - _LAST_RUCKIG_CHECK_T < min_dt:
        return
    _LAST_RUCKIG_CHECK_T = now
    _log_if_ik_fail(cfg, kind="ruckig", hand_side=hand_side, source=source)


def close() -> None:
    global _FILE
    with _LOCK:
        if _FILE is not None:
            try:
                _FILE.write(
                    f"# data_track end "
                    f"{datetime.now().isoformat(timespec='milliseconds')}\n"
                )
                _FILE.close()
            except Exception:
                pass
            _FILE = None
