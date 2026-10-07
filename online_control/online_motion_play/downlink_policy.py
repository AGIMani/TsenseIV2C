"""Online V2 wire policy and shared A3.0 control/range exports (no device I/O)."""
import copy
from functools import lru_cache
import math
from numbers import Real

from usb.offline import UsbController, LINEAR_JOINT_SLOTS, ROTARY_JOINT_SLOTS, get_hand_stroke_mapper
from protocol.hand_angle_protocol import convert_ui_to_hardware
from a30_kinematics.policy import (
    encode_control_targets as encode_online_targets,
    downlink_ranges as a30_ranges,
)


def negative_linear(value):
    """Keep zero at zero and both positive/negative strokes nonpositive."""
    if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value):
        raise ValueError("直线电机行程必须是有限数值")
    return -abs(float(value)) if value else 0.0


class OnlineUsbController(UsbController):
    def normalize_linear_positions(self, linear):
        return [negative_linear(value) for value in linear]


def range_spec(values, unit, slot):
    return {"min": min(values), "max": max(values), "unit": unit, "slot": slot,
            "at_ui_min": values[0], "at_ui_max": values[1]}


def v2_ranges(side, joints):
    mapper = get_hand_stroke_mapper()
    limits = tuple((j["name"], j["min"], j["max"]) for j in joints)
    scales = tuple(sorted(mapper.get_stroke_scales().items()))
    return _v2_ranges(side, limits, scales)


@lru_cache(maxsize=16)
def _v2_ranges(side, limits, scales):
    # A private copy prevents range inspection from changing live failure-hold
    # positions or the USB angle conversion cache.
    mapper = copy.deepcopy(get_hand_stroke_mapper())
    mapper.set_stroke_scales(dict(scales))
    endpoints = []
    for index in (1, 2):
        hw = convert_ui_to_hardware({row[0]: row[index] for row in limits}, hand_side=side)
        strokes = mapper.linear_strokes_mm(hw)
        endpoints.append({name: negative_linear(strokes[name]) if name in LINEAR_JOINT_SLOTS
                          else math.degrees(hw[name]) for name, *_ in limits})
    return {name: range_spec([p[name] for p in endpoints],
                            "mm" if name in LINEAR_JOINT_SLOTS else "deg",
                            f"L{LINEAR_JOINT_SLOTS[name]:02d}" if name in LINEAR_JOINT_SLOTS
                            else f"R{ROTARY_JOINT_SLOTS[name]:02d}") for name, *_ in limits}
