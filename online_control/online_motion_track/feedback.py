"""Read-only measured motor positions -> URDF radians for the comparison viewer.

These adapters never open a transport, seed a controller, or substitute a
commanded/initial pose for a missing measurement. Browser packet freshness is
checked by the browser; SDK snapshots carry their own age and online flags.
"""
from functools import lru_cache
import math
from numbers import Real
import struct
import threading

from hand_profiles import A30_MODEL_IDS, A32_MODEL_IDS, load_hand_profile


_LOCK = threading.RLock()


def _finite(value):
    return isinstance(value, Real) and not isinstance(value, bool) and math.isfinite(value)


@lru_cache(maxsize=10)
def _limits(model, side):
    return load_hand_profile(model).joint_limits(side)


def _payload(value, sizes):
    if not isinstance(value, str) or len(value) not in {n * 2 for n in sizes}:
        raise ValueError("遥测十六进制 payload 长度无效")
    try:
        payload = bytes.fromhex(value)
    except ValueError as exc:
        raise ValueError("遥测 payload 不是有效十六进制") from exc
    if len(payload) not in sizes:
        raise ValueError("遥测 payload 长度无效")
    return payload


def _v2(payload, side, issues):
    from usb.telemetry import parse_hand_telem_v2, parse_hand_telem_legacy
    from usb.offline import get_hand_stroke_mapper
    from kinematics.finger_linkage import (
        LINEAR_SLOT_JOINTS, ROTARY_SLOT_JOINTS, cf_to_phi_deg, joint_deg_to_phi_rad,
        MOTOR_F1, MOTOR_F2, MOTOR_F3,
    )
    parsed = (parse_hand_telem_v2(payload) if len(payload) == 780
              else parse_hand_telem_legacy(payload))
    if parsed is None:
        raise ValueError("V2 遥测解析失败")
    if parsed.get("hand_type", int(side == "right")) != int(side == "right"):
        raise ValueError("遥测左右手与所选模型不一致")
    mapper = get_hand_stroke_mapper()
    pose = {}
    # Use the original closed-loop kernels one measured channel at a time, so
    # absent channels remain absent. Encoder overruns use the linkage bounds.
    for name, row in zip(LINEAR_SLOT_JOINTS, parsed["linear"]):
        value = row.pos_act_mm
        if not _finite(value):
            continue
        params = mapper.thumb.params if "_thumb_" in name else mapper.four_finger.params
        motor = MOTOR_F3 if "_MCP_" in name else MOTOR_F2 if "_PIP_" in name and "_thumb_" not in name else MOTOR_F1
        try:
            q = joint_deg_to_phi_rad(cf_to_phi_deg(-value, motor, params, clamp=True))
        except (ValueError, ArithmeticError):
            continue
        pose[name.replace("left_hand_", f"{side}_hand_")] = q
    for name, row in zip(ROTARY_SLOT_JOINTS, parsed["rotary"]):
        value = row.pos_actual_mm  # The legacy field name carries degrees.
        if not _finite(value):
            continue
        # Same hardware/URDF direction mapping as hand_angle_protocol. Apply
        # the requested display bounds below, not the command-side 0.5 deg
        # minimum which would turn a negative measurement into a positive one.
        q = math.radians(value) * (-1 if side == "right" and "_pinky_rota_" in name else 1)
        pose[name.replace("left_hand_", f"{side}_hand_")] = q
    return pose


@lru_cache(maxsize=4)
def _a30_mapper(model, side):
    from a30_kinematics import A30StrokeMapper
    return A30StrokeMapper(side, model_id=model)


def _a30(payload, model, side, issues):
    from a30_kinematics.feedback import measured_pose
    mapper = _a30_mapper(model, side)
    # A fault is missing feedback for that axis, rather than permission to use
    # its target, or a reason to discard other healthy measured axes.
    packet = bytearray(payload)
    if len(packet) == 484:
        mask = struct.unpack_from("<H", packet, 2)[0]
        for i in range(2):
            offset = 444 + 20 * i
            if struct.unpack_from("<I", packet, offset + 12)[0] or packet[offset + 16] == 5:
                mask &= ~(1 << i)
                issues.append(f"外接拇指电机 M{i + 1} 报告故障")
        struct.pack_into("<H", packet, 2, mask)
    missing, clipped = [], []
    # The existing inverse accepts partial packets using a placeholder pose.
    # Remove every placeholder before publishing. Finite out-of-range
    # measurements retain the user's requested bounded display coordinate.
    pose = measured_pose(packet, mapper, fallback_pose=load_hand_profile(model).init_pose(side),
                         missing_joints=missing, clamp=True, clipped_joints=clipped)
    for name in missing:
        pose.pop(name, None)
    return pose


@lru_cache(maxsize=2)
def _a32_device(model):
    from a32_driver import A32Device
    # Construction loads calibration only. No NativeHand is instantiated.
    return A32Device(model_id=model)


def _a32(state, model, side, issues):
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
    if side == "right":
        issues.append("A3.2 右手硬件映射待标定")
        return {}
    age = state.get("telemetry_age_ms")
    if not state.get("connected") or not _finite(age) or not 0 <= age < 500:
        issues.append("SDK 未连接或手部遥测已过期")
        return {}
    device = _a32_device(model)
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
    if model not in ("TsenseI_v2", *A30_MODEL_IDS, *A32_MODEL_IDS):
        raise ValueError("未知手型")
    if side not in ("left", "right"):
        raise ValueError("手别必须为 left 或 right")
    with _LOCK:
        limits, issues = _limits(model, side), []
        if model == "TsenseI_v2":
            pose = _v2(_payload(data.get("telemetry"), (780, 932)), side, issues)
        elif model in A30_MODEL_IDS:
            pose = _a30(_payload(data.get("telemetry"), (444, 484)), model, side, issues)
        else:
            pose = _a32(data.get("telemetry"), model, side, issues)
        valid = {name: min(limits[name][1], max(limits[name][0], 0., float(q)))
                 for name, q in pose.items() if name in limits and _finite(q)}
        missing = [name for name in limits if name not in valid]
        if missing and not issues:
            issues.append("部分关节缺少有效反馈")
        return {"model_id": model, "side": side, "pose": valid,
                "missing_joints": missing, "total_joints": len(limits), "issues": issues}
