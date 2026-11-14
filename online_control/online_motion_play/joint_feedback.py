"""Read-only joint feedback for the editor; never changes control targets."""
import math


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
