"""Read-only joint feedback for the editor; never changes control targets."""
import math
import struct
import time


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def a32_feedback(device, state):
    age = state.get("telemetry_age_ms")
    if not state.get("connected") or not finite(age) or not 0 <= age <= 500:
        return {}
    result = {}
    for channel in getattr(device, "channels", ()):
        rows = state.get(channel["channel"], [])
        row = rows[channel["index"]] if channel["index"] < len(rows) else {}
        value = row.get("position")
        if not row.get("online") or not finite(value):
            continue
        if channel["channel"] == "mit" and (not row.get("valid") or row.get("fault")):
            continue
        # Invert actual position without command compensation. Stroke display
        # retains the wire coordinate, including the linear feedback sign.
        stroke = device.feedback_position(channel, value)
        try:
            radians = device._inverse(channel, stroke)
        except (ValueError, ArithmeticError):
            radians = None
        result[channel["joint"]] = {"rad": radians if finite(radians) else None, "stroke": stroke}
    return result


def a30_feedback(device):
    from a30_kinematics.downlink import channel_mapping
    from a30_kinematics.feedback import measured_pose

    payload = device.payload
    if not device.connected or payload is None or not 0 <= device.clock() - device.feedback_at <= .5:
        return {}
    payload = bytearray(payload)
    mask = struct.unpack_from("<I", payload, 8)[0]
    mit_mask = struct.unpack_from("<H", payload, 2)[0] if len(payload) == 484 else 0
    values = {}
    for channel in channel_mapping(device.mapper.hand_side):
        slot = channel["slot"]
        if channel["field"] == "self_pos":
            value = struct.unpack_from("<f", payload, 16 + 24 * slot)[0]
            if not mask & (1 << slot) or not finite(value):
                mask &= ~(1 << slot)
                continue
        else:
            if not mit_mask & (1 << slot):
                continue
            offset = 444 + 20 * slot
            if struct.unpack_from("<I", payload, offset + 12)[0] or payload[offset + 16] == 5:
                mit_mask &= ~(1 << slot)
                continue
            value = (struct.unpack_from("<I", payload, offset + 2)[0] - 0x80000000) / 2097152 * 360
        values[channel["joint"]] = value
    struct.pack_into("<I", payload, 8, mask)
    struct.pack_into("<H", payload, 2, mit_mask)
    clipped = []
    pose = measured_pose(payload, device.mapper, fallback_pose=device.pose,
                         clamp=True, clipped_joints=clipped)
    invalid = {row["joint"] for row in clipped if row["unit"] == "mm"}
    # Rotary readings do not need a bounded inverse: preserve actual angles.
    for channel in channel_mapping(device.mapper.hand_side):
        name = channel["joint"]
        if name in values and channel["unit"] == "deg":
            pose[name] = math.radians(values[name] * channel["position_sign"] / channel.get("gear_ratio", 1))
    return {name: {"rad": None if name in invalid else pose[name], "stroke": value}
            for name, value in values.items()}


def v2_feedback(usb, side):
    from usb.offline import LINEAR_JOINT_SLOTS, ROTARY_JOINT_SLOTS, joint_cfg_from_telemetry_frame

    # Use the receiver timestamp, not the UI poll time, so a cached frame expires.
    if not usb.is_connected or not 0 <= time.perf_counter() - getattr(usb, "_telem_recv_mono", 0) <= .5:
        return {}
    frame = usb.read_latest_telemetry()
    if not frame:
        return {}
    try:
        pose = joint_cfg_from_telemetry_frame(frame, hand_side=side, strict=True)
    except (ValueError, ArithmeticError):
        return {}
    result = {}
    for slots, rows, field in ((LINEAR_JOINT_SLOTS, frame["linear"], "pos_act_mm"),
                               (ROTARY_JOINT_SLOTS, frame["rotary"], "pos_actual_mm")):
        for name, slot in slots.items():
            result[name] = {"rad": pose[name], "stroke": getattr(rows[slot], field)}
    return result
