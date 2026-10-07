"""电机电流日志与异常预警（按指汇总，可由日志面板开关）。

周期汇总 tag: MotorCurrent（默认关）
异常预警 tag: MotorCurrentWarn（默认关）
"""
from __future__ import annotations

import time
from typing import Any, Dict, List, Optional, Sequence, Tuple

from utils.log_control import emit as log_emit, register_tag

# 与 usb.offline / main_app_v3 槽位映射一致：(指, ((部位, "L"|"R", 槽下标), ...))
FINGER_CURRENT_LOG_GROUPS: Tuple[Tuple[str, Tuple[Tuple[str, str, int], ...]], ...] = (
    ("拇指", (("MCP", "L", 12), ("PIP", "L", 13), ("rota", "R", 4), ("swing", "R", 5))),
    # 与 LINEAR_JOINT_SLOTS 四指 DIP/PIP 正式映射一致（已实机验证）
    ("食指", (("MCP", "L", 6), ("PIP", "L", 7), ("DIP", "L", 8), ("swing", "R", 0))),
    ("中指", (("MCP", "L", 9), ("PIP", "L", 10), ("DIP", "L", 11), ("swing", "R", 1))),
    ("无名指", (("MCP", "L", 0), ("PIP", "L", 1), ("DIP", "L", 2), ("swing", "R", 2))),
    ("小指", (("MCP", "L", 3), ("PIP", "L", 4), ("DIP", "L", 5), ("swing", "R", 3), ("rota", "R", 6))),
)

# iq_meas / rotary iq 大于该值时标红并输出严重预警。
# 按业务要求只判断正向值 ``iq > 2000``，负值不触发此电流阈值。
FINGER_IQ_RED_THRESHOLD = 2000.0
FINGER_IQ_LOG_INTERVAL_S = 1.0
FINGER_IQ_WARN_COOLDOWN_S = 2.0

TAG_PERIODIC = "MotorCurrent"
TAG_WARN = "MotorCurrentWarn"

register_tag(TAG_PERIODIC, "分指电机电流周期汇总", enabled=False)
register_tag(TAG_WARN, "电机电流异常/故障预警", enabled=False)

_slot_active_fns = None
_decode_status_a = None


def _lazy_main_app_helpers():
    global _slot_active_fns, _decode_status_a
    if _slot_active_fns is None:
        from usb.telemetry import (
            decode_status_a,
            slot_is_active_linear,
            slot_is_active_rotary,
        )

        _slot_active_fns = (slot_is_active_linear, slot_is_active_rotary)
        _decode_status_a = decode_status_a
    return _slot_active_fns, _decode_status_a


def _finger_slot_iq(
    kind: str,
    idx: int,
    linear: Sequence[Any],
    rotary: Sequence[Any],
) -> Tuple[Optional[float], Optional[float], str]:
    """返回 (iq_meas, iq_ref, 状态附注)；空槽 (None, None, \"\")。"""
    (slot_is_active_linear, slot_is_active_rotary), decode_status_a = _lazy_main_app_helpers()
    if kind == "L":
        if idx < 0 or idx >= len(linear):
            return None, None, ""
        slot = linear[idx]
        if not slot_is_active_linear(slot, idx):
            return None, None, ""
        note = ""
        try:
            sa = decode_status_a(int(getattr(slot, "status_a_u32", 0) or 0))
            if sa.get("status_valid") and (
                not sa.get("drv1_nfault") or not sa.get("drv2_nfault")
            ):
                note = "DRV_FAULT"
            elif int(getattr(slot, "mech_home_state", 0) or 0) == 3:
                note = "HOME_ERR"
        except Exception:
            pass
        return float(slot.iq_meas), float(slot.iq_ref), note
    if kind == "R":
        if idx < 0 or idx >= len(rotary):
            return None, None, ""
        slot = rotary[idx]
        if not slot_is_active_rotary(slot):
            return None, None, ""
        return float(slot.iq), float(slot.iq_ref), ""
    return None, None, ""


class MotorCurrentMonitor:
    """根据最新遥测帧输出分指电流汇总与超限预警。"""

    def __init__(self) -> None:
        self._last_periodic_mono = 0.0
        self._warn_last_mono: Dict[str, float] = {}

    def reset(self) -> None:
        self._last_periodic_mono = 0.0
        self._warn_last_mono.clear()

    def _warn_allowed(self, key: str, now_mono: float) -> bool:
        last = self._warn_last_mono.get(key, 0.0)
        if (now_mono - last) < FINGER_IQ_WARN_COOLDOWN_S:
            return False
        self._warn_last_mono[key] = now_mono
        return True

    @staticmethod
    def form5_key(kind: str, idx: int) -> str:
        """Form5 groupBox 键：lm_L12 / rm_R00。"""
        if kind == "L":
            return f"lm_L{idx:02d}"
        return f"rm_R{idx:02d}"

    def update(
        self, frame: Optional[dict], *, now_mono: Optional[float] = None
    ) -> Dict[str, str]:
        """处理一帧遥测：写日志，并返回 Form5 键 → 严重度 (ok/warn/crit/fault)。"""
        levels: Dict[str, str] = {}
        if not frame:
            return levels

        from utils.log_control import is_enabled

        want_periodic = is_enabled(TAG_PERIODIC)
        want_warn = is_enabled(TAG_WARN)

        now = time.monotonic() if now_mono is None else float(now_mono)
        linear = frame.get("linear") or []
        rotary = frame.get("rotary") or []
        if not isinstance(linear, list) or not isinstance(rotary, list):
            return levels

        finger_parts: List[str] = []
        warn_lines: List[str] = []

        for finger, parts in FINGER_CURRENT_LOG_GROUPS:
            part_bits: List[str] = []
            for part, kind, idx in parts:
                ui_key = self.form5_key(kind, idx)
                iq, _iq_ref, note = _finger_slot_iq(kind, idx, linear, rotary)
                if iq is None:
                    part_bits.append(f"{part}=—")
                    continue
                part_bits.append(f"{part}={iq:.3f}")
                slot_tag = f"{kind}#{idx}"
                level = "ok"
                if note in ("DRV_FAULT", "HOME_ERR"):
                    level = "fault"
                elif iq > FINGER_IQ_RED_THRESHOLD:
                    level = "crit"
                levels[ui_key] = level

                if not want_warn:
                    continue
                if note == "DRV_FAULT":
                    key = f"{finger}:{part}:drv"
                    if self._warn_allowed(key, now):
                        warn_lines.append(
                            f"[电流预警] {finger} {part} {slot_tag} DRV nFAULT iq={iq:.3f}"
                        )
                elif note == "HOME_ERR":
                    key = f"{finger}:{part}:home"
                    if self._warn_allowed(key, now):
                        warn_lines.append(
                            f"[电流预警] {finger} {part} {slot_tag} 回零 ERR iq={iq:.3f}"
                        )
                if iq > FINGER_IQ_RED_THRESHOLD:
                    key = f"{finger}:{part}:crit"
                    if self._warn_allowed(key, now):
                        warn_lines.append(
                            f"[电流严重] {finger} {part} {slot_tag} "
                            f"iq={iq:.3f} > {FINGER_IQ_RED_THRESHOLD:g}"
                        )
            finger_parts.append(f"{finger}[{' '.join(part_bits)}]")

        for line in warn_lines:
            log_emit(line, TAG_WARN)

        if want_periodic and (now - self._last_periodic_mono) >= FINGER_IQ_LOG_INTERVAL_S:
            self._last_periodic_mono = now
            seq = frame.get("seq", "—")
            log_emit(f"[分指电流] seq={seq} " + " | ".join(finger_parts), TAG_PERIODIC)

        return levels

