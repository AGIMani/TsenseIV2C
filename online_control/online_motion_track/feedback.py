"""Read-only measured motor positions -> URDF radians for the comparison viewer.

These adapters never open a transport, seed a controller, or substitute a
commanded/initial pose for a missing measurement. Browser packet freshness is
checked by the browser; SDK snapshots carry their own age and online flags.
"""
from functools import lru_cache
import math
from numbers import Real
import threading

from hand_profiles import load_hand_profile


_LOCK = threading.RLock()


def _finite(value):
    return isinstance(value, Real) and not isinstance(value, bool) and math.isfinite(value)


@lru_cache(maxsize=10)
def _limits(model, side):
    return load_hand_profile(model).joint_limits(side)


@lru_cache(maxsize=4)
def _a32_device(model, side):
    from a32_driver import A32Device
    # Construction loads calibration only. No NativeHand is instantiated.
    return A32Device(hand_side=side, model_id=model)


def _a32(state, model, side, issues):
    from a32_kinematics.mapper import model_joint_to_source_sign

    if not isinstance(state, dict):
        raise ValueError("A3.2 遥测需要 SDK 状态对象")
    if state.get("hand_side") not in (None, side):
        raise ValueError("遥测左右手与所选模型不一致")
    if state.get("model_id") not in (None, model):
        raise ValueError("遥测型号与所选模型不一致")
    for key, count in (("self", 18), ("mit", 2)):
        if not isinstance(state.get(key), list) or len(state[key]) > count or any(
                not isinstance(row, dict) for row in state[key]):
            raise ValueError(f"A3.2 {key} 遥测通道格式无效")
    age = state.get("telemetry_age_ms")
    if not state.get("connected") or not _finite(age) or not 0 <= age < 500:
        issues.append("SDK 未连接或手部遥测已过期")
        return {}
    device = _a32_device(model, side)
    pose = {}
    for channel in device.channels:
        rows = state[channel["channel"]]
        index = channel["index"]
        if index >= len(rows):
            continue
        row = rows[index]
        value = row.get("position")
        if not row.get("online") or not _finite(value):
            continue
        if channel["channel"] == "mit" and (not row.get("valid") or row.get("fault") or row.get("state") == 5):
            continue
        value = device.feedback_position(channel, value)
        if channel["unit"] == "deg":
            # Display finite rotary measurements using the model's bounds
            # below, including overruns beyond the stricter command/SDK range.
            # Control and feedback seeding still use the strict inverse API.
            source = (value - channel["offset"]) / channel["direction"]
            pose[channel["joint"]] = (math.radians(source) *
                                      model_joint_to_source_sign(model, channel["joint"]))
            continue
        try:
            q = device._inverse(channel, value)
        except (ValueError, ArithmeticError):
            # Outside the linkage's assembly branch, use its calibrated
            # endpoint. This still derives solely from the measured value.
            bounded = min(channel["limits"][1], max(channel["limits"][0], value))
            if bounded == value:
                continue
            try:
                q = device._inverse(channel, bounded)
            except (ValueError, ArithmeticError):
                continue
        pose[channel["joint"]] = q
    return pose


def feedback_pose(data):
    """Convert one supplied observation; no session or hardware side effects."""
    if not isinstance(data, dict):
        raise ValueError("需要 JSON 对象")
    model, side = data.get("model_id"), data.get("side")
    if model != "TsenseI_v3_2_withid":
        raise ValueError("未知手型")
    if side not in ("left", "right"):
        raise ValueError("手别必须为 left 或 right")
    with _LOCK:
        limits, issues = _limits(model, side), []
        pose = _a32(data.get("telemetry"), model, side, issues)
        valid = {}
        for name, q in pose.items():
            if name not in limits or not _finite(q):
                continue
            low, high = limits[name]
            valid[name] = min(high, max(low, float(q)))
        missing = [name for name in limits if name not in valid]
        if missing and not issues:
            issues.append("部分关节缺少有效反馈")
        return {"model_id": model, "side": side, "pose": valid,
                "missing_joints": missing, "total_joints": len(limits), "issues": issues}
