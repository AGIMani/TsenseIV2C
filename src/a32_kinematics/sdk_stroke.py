"""Pure Python counterpart of A3 SDK 1.1.15 ``Stroke.cpp``.

The 1.1.15 SDK uses four calibrated linkage curves and uniform lookup tables
for both forward and feedback conversion.  This module mirrors that algorithm
for offline previews and planning; it never opens a device or loads the native
transport library.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import math
from numbers import Real

from .a32_angles_to_inputs import A32Error
from .sdk_stroke_data import LINK_ROWS, JOINT_ROWS, SDK_VERSION, SOURCE_JOINT_NAMES, SOURCE_SHA256


METHOD = f"sdk-{SDK_VERSION}"
DEG = math.pi / 180.0
SPAN = math.pi / 2.0
LINEAR_ANGLE_MIN_DEG = 0.0
LINEAR_ANGLE_MAX_DEG = 90.0
ACTUATOR_ROUNDING_TOLERANCE_MM = 1e-5
ANGLE_ROUNDING_TOLERANCE_RAD = 1e-6
FORWARD_LUT_TOLERANCE_M = 1e-9
INVERSE_LUT_TOLERANCE_DEG = 1e-6


@dataclass
class _Link:
    b: float
    c: float
    d: float
    length: float
    base: float = field(init=False)
    stroke_hi: float = field(init=False)
    forward_table: "_Table | None" = field(default=None, init=False, repr=False)
    inverse_table: "_Table | None" = field(default=None, init=False, repr=False)

    @property
    def stroke_lo(self):
        return 0.0


@dataclass(frozen=True)
class _Table:
    scale: float
    values: tuple[float, ...]
    ok: bool = True


@dataclass(frozen=True)
class Joint:
    linear: bool
    lo: float
    hi: float
    destination: int
    link: int
    linkage: _Link | None

    # Existing callers used ``loop`` for the old three-curve SDK.
    @property
    def loop(self):
        return self.link


def _stroke_mm(link: _Link, radians: float) -> float:
    c = math.cos(radians)
    s = math.sin(radians)
    u = link.d - link.b * s - link.c * c
    inner = link.length * link.length - u * u
    return (link.b * (1.0 - c) + link.c * s + link.base -
            math.sqrt(max(0.0, inner)))


def _make_link(values):
    b, c, d, length = map(float, values)
    link = _Link(b, c, d, length)
    link.base = math.sqrt(length * length - (d - c) * (d - c))
    link.stroke_hi = _stroke_mm(link, SPAN)
    return link


_LINKS = (None,) + tuple(_make_link(values) for values in LINK_ROWS[1:])


def _curvature_bound(link: _Link) -> float:
    bc = math.hypot(link.b, link.c)
    u0 = link.d - link.c
    u90 = link.d - link.b
    u_min = link.d - bc
    umax = max(abs(u_min), abs(u0), abs(u90))
    rmin2 = link.length * link.length - umax * umax
    if not (rmin2 > 0.0 and bc > 0.0):
        return -1.0
    rmin = math.sqrt(rmin2)
    v2 = bc * bc
    return bc + (v2 + umax * bc) / rmin + (umax * umax * v2) / (rmin2 * rmin)


def _stroke_rate(link: _Link, radians: float) -> float:
    c = math.cos(radians)
    s = math.sin(radians)
    u = link.d - link.b * s - link.c * c
    v = -link.b * c + link.c * s
    w = link.b * s + link.c * c
    r2 = link.length * link.length - u * u
    if not (r2 > 0.0):
        return 0.0
    return w + (u * v) / math.sqrt(r2)


def _inverse_curvature(link: _Link) -> float:
    xpp = _curvature_bound(link)
    if not (xpp > 0.0 and SPAN > 0.0):
        return -1.0
    for count in (1024, 4096, 16384, 65536):
        half = SPAN / (2.0 * count)
        low = min(_stroke_rate(link, SPAN * n / count) for n in range(count + 1))
        low -= xpp * half
        if low > 0.0:
            return xpp / (low * low * low)
    return -1.0


def _lookup(table: _Table, value: float) -> float:
    position = value * table.scale
    last = len(table.values) - 2
    index = int(position)
    index = max(0, min(index, last))
    fraction = max(0.0, min(1.0, position - index))
    return table.values[index] + fraction * (table.values[index + 1] - table.values[index])


def _make_forward_table(link: _Link) -> _Table:
    curvature = _curvature_bound(link)
    if not (curvature > 0.0 and math.isfinite(curvature)):
        return _Table(0.0, (), False)
    raw = SPAN * math.sqrt(curvature / (8.0 * FORWARD_LUT_TOLERANCE_M * 0.9))
    if not math.isfinite(raw) or raw > 262144.0:
        return _Table(0.0, (), False)
    intervals = max(1, math.ceil(raw))
    values = tuple(max(0.0, _stroke_mm(link, SPAN * n / intervals))
                   for n in range(intervals + 1))
    return _Table(intervals / SPAN, values)


def _angle_of_stroke(link: _Link, mm: float) -> float | None:
    y = mm - link.b - link.base
    a = 2.0 * (y * link.b - link.d * link.c)
    b = -2.0 * (y * link.c + link.d * link.b)
    rhs = (link.length * link.length - link.b * link.b - link.c * link.c -
           link.d * link.d - y * y)
    radius = math.hypot(a, b)
    # The upstream formula is expressed in millimetres and checks R > 1.
    # Linkage data here stays in metres and R is quadratic in those lengths,
    # so the equivalent threshold is 1e-6.
    if not (radius > 1e-6):
        return None
    ratio = rhs / radius
    if abs(ratio) > 1.0 + 1e-9:
        return None
    phi = math.atan2(b, a)
    delta = math.acos(max(-1.0, min(1.0, ratio)))
    best_error, best_deg = 1.0, 0.0
    found = False
    for candidate in (phi + delta, phi - delta):
        q = math.remainder(candidate, 2.0 * math.pi)
        if q < -1e-8 or q > SPAN + 1e-8:
            continue
        q = max(0.0, min(SPAN, q))
        error = abs(_stroke_mm(link, q) - mm)
        if error < best_error:
            best_error, best_deg, found = error, q / DEG, True
    if not found or best_error > 1e-6:
        return None
    return best_deg


def _make_inverse_table(link: _Link) -> _Table:
    curvature = _inverse_curvature(link) / DEG
    if not (curvature > 0.0 and math.isfinite(curvature) and link.stroke_hi > 0.0):
        return _Table(0.0, (), False)
    raw = link.stroke_hi * math.sqrt(curvature / (8.0 * INVERSE_LUT_TOLERANCE_DEG * 0.9))
    if not math.isfinite(raw) or raw > 262144.0:
        return _Table(0.0, (), False)
    intervals = max(1, math.ceil(raw))
    values = []
    for n in range(intervals + 1):
        degree = _angle_of_stroke(link, link.stroke_hi * n / intervals)
        if degree is None:
            return _Table(0.0, (), False)
        values.append(max(0.0, min(90.0, degree)))
    return _Table(intervals / link.stroke_hi, tuple(values))


def _forward_table(link: _Link):
    if link.forward_table is None:
        link.forward_table = _make_forward_table(link)
    return link.forward_table


def _inverse_table(link: _Link):
    if link.inverse_table is None:
        link.inverse_table = _make_inverse_table(link)
    return link.inverse_table


JOINTS = tuple(
    Joint(linear, lo, hi, destination, link, _LINKS[link])
    for linear, lo, hi, destination, link in JOINT_ROWS
)


def _finite_real(value, label):
    if isinstance(value, bool) or not isinstance(value, Real):
        raise A32Error("Input", f"{label} must be a finite real number")
    try:
        value = float(value)
    except (ValueError, TypeError, OverflowError) as exc:
        raise A32Error("Input", f"{label} must be a finite real number") from exc
    if not math.isfinite(value):
        raise A32Error("Input", f"{label} must be a finite real number")
    return value


def _solve_linear(link: _Link, degrees: float) -> float:
    degrees = max(0.0, min(90.0, degrees))
    table = _forward_table(link)
    if not table.ok:
        raise A32Error("LoopGeometry", "SDK linkage has no finite stroke")
    stroke = _lookup(table, degrees * DEG)
    tolerance_m = ACTUATOR_ROUNDING_TOLERANCE_MM / 1000.0
    if not math.isfinite(stroke) or stroke < -tolerance_m or stroke > link.stroke_hi + tolerance_m:
        raise A32Error("StrokeLimit", "SDK linkage outside stroke limits")
    return max(0.0, min(link.stroke_hi, stroke)) * 1000.0


def joint_angle_deg_to_actuator(index, angle_deg):
    """Convert one SDK joint angle in degrees to millimetres or degrees."""
    degrees = _finite_real(angle_deg, "angle")
    joint = JOINTS[index]
    if joint.linear:
        return _solve_linear(joint.linkage, degrees)
    radians = degrees * DEG
    if radians < joint.lo - 1e-10 or radians > joint.hi + 1e-10:
        raise A32Error("AngleLimit", f"SDK joint {index + 1}: {degrees} degrees outside limits")
    return math.degrees(max(joint.lo, min(joint.hi, radians)))


def angles_deg_to_actuators(angles_deg):
    """SDK mechanism order: 20 joint degrees -> 14 mm and 6 degree values."""
    angles = tuple(angles_deg)
    if len(angles) != len(JOINTS):
        raise A32Error("Input", "SDK conversion requires 20 joint angles")
    return tuple(joint_angle_deg_to_actuator(i, angle) for i, angle in enumerate(angles))


def _inverse_linear(link: _Link, value: float) -> float:
    if not math.isfinite(value):
        raise A32Error("Input", "actuator position must be a finite real number")
    tolerance_m = ACTUATOR_ROUNDING_TOLERANCE_MM / 1000.0
    stroke = value / 1000.0
    if stroke < -tolerance_m or stroke > link.stroke_hi + tolerance_m:
        raise A32Error("StrokeLimit", f"actuator position {value} mm outside stroke limits")
    stroke = max(0.0, min(link.stroke_hi, stroke))
    table = _inverse_table(link)
    if not table.ok:
        raise A32Error("LoopGeometry", "SDK linkage has no finite feedback angle")
    return max(0.0, min(90.0, _lookup(table, stroke)))


def inverse_actuator(index, value):
    """Invert one actuator value to joint radians, including endpoints."""
    value = _finite_real(value, "actuator position")
    joint = JOINTS[index]
    if not joint.linear:
        radians = value * DEG
        if radians < joint.lo - ANGLE_ROUNDING_TOLERANCE_RAD or radians > joint.hi + ANGLE_ROUNDING_TOLERANCE_RAD:
            raise A32Error("AngleLimit", f"SDK joint {index + 1}: feedback outside angle limits")
        return max(joint.lo, min(joint.hi, radians))
    return math.radians(_inverse_linear(joint.linkage, value))
