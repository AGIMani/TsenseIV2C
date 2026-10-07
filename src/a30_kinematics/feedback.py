"""A3 444/484-byte telemetry and measured pose recovery; no device access."""
import math
import struct
from .downlink import channel_mapping, mit_motor_to_out


def measured_pose(payload, mapper, *, fallback_pose=None, missing_joints=None,
                  clamp=False, clipped_joints=None, rotary_tolerance_rad=.002):
    if not math.isfinite(rotary_tolerance_rad) or rotary_tolerance_rad < 0:
        raise ValueError("A3 旋转反馈容差必须是非负有限值")
    if isinstance(payload, str):
        if len(payload) not in (888, 968):
            raise ValueError("A3 遥测必须是 444 或 484 字节")
        payload = bytes.fromhex(payload)
    if len(payload) not in (444, 484):
        raise ValueError("A3 遥测必须是 444 或 484 字节")
    seq, mit_mask, tick, mask = struct.unpack_from("<HHII", payload)
    if len(payload) == 444:
        mit_mask = 0  # This firmware does not include the two external motors.
        payload = bytes(payload) + bytes(40)
    partial = fallback_pose is not None
    if not partial and (mask & 0x3ffff != 0x3ffff or mit_mask & 3 != 3):
        raise ValueError("A3 必须收到全部 18+2 路电机的有效遥测")
    positions = [struct.unpack_from("<f", payload, 16 + 24*i)[0] for i in range(18)]
    if partial:
        positions = [v if mask & (1 << i) and math.isfinite(v) else None for i, v in enumerate(positions)]
    for i in range(2):
        offset = 444 + i*20
        if partial and not mit_mask & (1 << i):
            positions.append(None)
            continue
        # A3_USB_Downlink accepts mit_ident_ok(raw, i) OR the online bit.
        # Online feedback is usable even with a different cached identifier;
        # identifier-only/stale feedback above still uses the fallback pose.
        if struct.unpack_from("<I", payload, offset+12)[0] or payload[offset+16] == 5:
            raise ValueError("外接电机报告故障")
        code = struct.unpack_from("<I", payload, offset+2)[0] - 0x80000000
        positions.append(mit_motor_to_out(code / 2097152 * 360, i))
    if not all(v is None and partial or v is not None and math.isfinite(v) for v in positions):
        raise ValueError("A3 电机位置反馈不是有限值")
    rows = channel_mapping(mapper.hand_side)
    linear_index = {mapper.joint_names[i]: k for k, i in enumerate(mapper._linear_indices)}
    # Prepare single-joint geometry calls once per mapper, not once per frame.
    if not hasattr(mapper, "_inverse_kernels"):
        from a32_kinematics.a32_fast import default_converter
        mapper._inverse_kernels = [default_converter().prepare(i+1, method="exact") for i in mapper._linear_indices]
    limits = dict(zip(mapper.joint_names, mapper._limits))
    if partial and (not isinstance(fallback_pose, dict) or set(fallback_pose) != set(limits) or any(
            not isinstance(fallback_pose[n], (int, float)) or not math.isfinite(fallback_pose[n])
            or not lo-1e-7 <= fallback_pose[n] <= hi+1e-7
            for n, (lo, hi) in limits.items())):
        raise ValueError("A3 缺失反馈的备用姿态无效")
    result = {}
    for row, value in zip(rows, positions):
        name = row["joint"]; lo, hi = limits[name]
        if value is None:
            result[name] = float(fallback_pose[name])
            if missing_joints is not None:
                missing_joints.append(name)
            continue
        # Restore geometry coordinates before limits/inverse kinematics.
        value *= row["position_sign"]
        if row["unit"] == "deg":
            q = math.radians(value)
            if not clamp and (q < lo-rotary_tolerance_rad or q > hi+rotary_tolerance_rad):
                raise ValueError(
                    f"{name} 位置反馈超出限位：实测 {value:.4f} deg ({q:.6f} rad)，"
                    f"允许 {math.degrees(lo):.4f}～{math.degrees(hi):.4f} deg，"
                    f"边界容差 {math.degrees(rotary_tolerance_rad):.4f} deg；请核对零位和电机映射")
            lower, upper = math.degrees(lo), math.degrees(hi)
            applied = min(upper, max(lower, value))
            q = min(hi, max(lo, q))
        else:
            kernel = mapper._inverse_kernels[linear_index[name]]
            low, high = lo, hi
            a, b = (kernel.scalar(math.degrees(v))*1000 for v in (lo, hi))
            if not clamp and (value < min(a, b)-.02 or value > max(a, b)+.02):
                raise ValueError(f"{name} 行程反馈超出机构范围")
            lower, upper = min(a, b), max(a, b)
            applied = min(upper, max(lower, value))
            for _ in range(35):
                mid = (low+high)/2
                stroke = kernel.scalar(math.degrees(mid))*1000
                if (stroke < applied) == (b > a): low = mid
                else: high = mid
            q = (low+high)/2
        if clipped_joints is not None and abs(applied-value) > 1e-9:
            clipped_joints.append({"joint": name, "raw": value, "applied": applied,
                                   "unit": row["unit"], "lower": lower, "upper": upper})
        result[name] = q
    return result
