"""A3.2 关节角度（deg）→ 相对直线行程（m）/旋转输入（rad）。

移植自同目录的 a32_angles_to_inputs.m、a32_angle_limits.m、
a32_loop_inverse.m 和 a32_loop_kinematics.m，不依赖 MATLAB 运行时。
公开接口的电机序号沿用 MATLAB 的 1～20；NumPy 结果数组用 0 起始下标。
只包含角度/行程运动学，不包含重力、质量模型或图形界面。
高频转换请使用 a32_fast.A32Converter；本模块保留完整参考计算和诊断信息。
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from .geometry_constants import configuration


MODEL_DIR = Path(__file__).resolve().parent / "model"


class A32Error(ValueError):
    """保留 MATLAB 错误标识，便于调用方识别限位、分支和几何错误。"""

    def __init__(self, code: str, message: str):
        self.code = f"A32:{code}"
        super().__init__(f"{self.code}: {message}")


@dataclass
class KinematicModel:
    """按源 URDF 顺序保存运动学所需字段；不是完整重力模型 M。"""

    robot_name: str
    q_names: tuple[str, ...]
    axes: np.ndarray
    lower: np.ndarray
    upper: np.ndarray

    @property
    def nq(self) -> int:
        return len(self.q_names)


def _vector(value, name, size=None):
    raw = np.asarray(value)
    if raw.dtype.kind not in "iuf" or raw.ndim > 2 or (
        raw.ndim == 2 and min(raw.shape) != 1
    ):
        raise A32Error("Input", f"{name} 必须是实数标量或向量。")
    result = raw.astype(float).reshape(-1)
    if not result.size or not np.all(np.isfinite(result)):
        raise A32Error("Input", f"{name} 不能为空或包含 NaN/Inf。")
    if size is not None and result.size != size:
        raise A32Error("Input", f"{name} 需要 {size} 个数，实际为 {result.size} 个。")
    return result


def _axis(value):
    axis = _vector(value, "axis", 3)
    length = np.linalg.norm(axis)
    if length < np.finfo(float).eps:
        raise A32Error("LoopGeometry", "关节转轴不能为零。")
    return axis / length


def _rotation(axis, q):
    x, y, z = axis
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + np.sin(q) * skew + (1 - np.cos(q)) * (skew @ skew)


def load_kinematic_model(urdf_path=None):
    """读取源 URDF 的活动旋转关节名称、轴向和限位，保留原始顺序。"""
    root = ET.parse(urdf_path or MODEL_DIR / "A3.2_source.urdf").getroot()
    names, axes, lower, upper = [], [], [], []
    for joint in root.findall("joint"):
        kind = joint.attrib["type"]
        if kind == "fixed":
            continue
        if kind not in ("revolute", "continuous") or joint.find("mimic") is not None:
            raise A32Error("Joint", "角度转换仅支持独立旋转关节。")
        name = joint.attrib["name"]
        if name in names:
            raise A32Error("Names", f"重复关节名称：{name}")
        axis_node = joint.find("axis")
        axis = [float(v) for v in (
            axis_node.attrib["xyz"] if axis_node is not None else "1 0 0"
        ).split()]
        if kind == "continuous":
            lo, hi = -np.inf, np.inf
        else:
            limit = joint.find("limit")
            if limit is None:
                raise A32Error("Limits", f"{name} 缺少角度限位。")
            lo, hi = float(limit.attrib["lower"]), float(limit.attrib["upper"])
            if not np.all(np.isfinite([lo, hi])) or lo > hi:
                raise A32Error("Limits", f"{name} 的限位无效。")
        names.append(name)
        axes.append(_axis(axis))
        lower.append(lo)
        upper.append(hi)
    if not names:
        raise A32Error("Empty", "URDF 没有活动关节。")
    return KinematicModel(root.attrib["name"], tuple(names), np.array(axes),
                          np.array(lower), np.array(upper))


def load_configuration():
    """Load independent Python parameters with current project range overrides."""
    return configuration()


def _geometry(p, axis):
    axis = _axis(axis)
    a0, b, u = (_vector(p[key], key, 3) for key in ("a0", "b", "u"))
    length = _vector(p["L"], "L", 1)[0]
    if length <= 0 or abs(np.linalg.norm(u) - 1) > 1e-10 or abs(axis @ u) > 1e-10:
        raise A32Error("LoopGeometry", "杆长须为正，导轨须为单位向量且垂直于关节转轴。")
    if p["sliderOn"] not in ("parent", "child"):
        raise A32Error("LoopGeometry", "sliderOn 必须为 parent 或 child。")
    return axis, a0, b, u, length


def a32_loop_inverse(p, axis, q):
    """关节角 q（rad）→ 行程 s（m）；由初始零位确定二次方程的根。"""
    q = _vector(q, "q", 1)[0]
    axis, a0, b, u, length = _geometry(p, axis)
    rotation = _rotation(axis, q)
    v = (rotation @ b if p["sliderOn"] == "parent" else rotation.T @ b) - a0
    t = u @ v
    r2 = length**2 - np.linalg.norm(v - u * t)**2
    if r2 < -1e-14:
        raise A32Error("LoopUnreachable", "目标角度无法满足当前杆长与导轨约束。")
    v0 = b - a0
    t0 = u @ v0
    z = np.sqrt(max(0, length**2 - np.linalg.norm(v0 - u * t0)**2))
    roots0 = np.array([t0 - z, t0 + z])
    index = np.argmin(np.abs(roots0))
    if abs(roots0[index]) > 1e-9 or abs(roots0[1] - roots0[0]) < 1e-10:
        raise A32Error("LoopBranch", "无法根据初始零位确定唯一的行程装配分支。")
    return float(t + (-1 if index == 0 else 1) * np.sqrt(max(0, r2)))


def a32_loop_kinematics(p, axis, s, q_limits):
    """行程（m）→ 闭环状态字典；角度 rad，导数 rad/m，坐标 m。"""
    s = _vector(s, "s", 1)[0]
    limits = _vector(q_limits, "q_limits", 2)
    if limits[0] > limits[1]:
        raise A32Error("Limits", "角度限位顺序错误。")
    axis, a0, b, u, length = _geometry(p, axis)
    a = a0 + u * s
    parent = p["sliderOn"] == "parent"
    v, w = (b, a) if parent else (a, b)
    vp = axis * (axis @ v)
    vn = v - vp
    coef_a, coef_b = w @ vn, w @ np.cross(axis, v)
    d = (v @ v + w @ w - 2 * (w @ vp) - length**2) / 2
    rho = np.hypot(coef_a, coef_b)
    if rho < 1e-14:
        raise A32Error("LoopSingular", "退化闭环，无法唯一确定关节角。")
    ratio = d / rho
    if abs(ratio) > 1 + 1e-10:
        raise A32Error("LoopUnreachable", f"行程 {s:.9g} m 无法满足杆长。")
    if p["branchSign"] not in (-1, 1):
        raise A32Error("LoopBranch", "branchSign 必须为 -1 或 +1。")
    angle = np.arctan2(coef_b, coef_a) + p["branchSign"] * np.arccos(np.clip(ratio, -1, 1))
    first = np.ceil((limits[0] - angle) / (2 * np.pi) - 1e-12)
    last = np.floor((limits[1] - angle) / (2 * np.pi) + 1e-12)
    if first != last:
        raise A32Error("LoopBranch", "关节限位内没有唯一的装配分支。")
    q = float(angle + 2 * np.pi * first)
    rotation = _rotation(axis, q)
    if parent:
        ap, bp = a, rotation @ b
        aq, bq, a_s = np.zeros(3), np.cross(axis, bp), u
    else:
        ap, bp = rotation @ a, b
        aq, bq, a_s = np.cross(axis, ap), np.zeros(3), rotation @ u
    delta = bp - ap
    hq, hs = 2 * (delta @ (bq - aq)), 2 * (delta @ -a_s)
    if abs(hq) < 1e-10 * max(length**2, 1e-8):
        raise A32Error("LoopSingular", "此死点位置的 dq/ds 为奇异值。")
    dqds = -hs / hq
    ap_s, bp_s = a_s + aq * dqds, bq * dqds
    delta0 = b - a0
    n0, n = delta0 - axis * (axis @ delta0), delta - axis * (axis @ delta)
    if min(np.linalg.norm(n0), np.linalg.norm(n)) < 1e-10:
        raise A32Error("LoopSingular", "无法确定连杆旋转轴。")
    if abs(np.linalg.norm(delta0) - length) > 1e-9:
        raise A32Error("LoopInitial", "初始铰点间距与杆长不一致。")
    rod_angle = np.arctan2(axis @ np.cross(n0, n), n0 @ n)
    rod_rotation = _rotation(axis, rod_angle)
    if np.linalg.norm(rod_rotation @ delta0 - delta) > 1e-8:
        raise A32Error("LoopGeometry", "铰点运动超出平行转轴连杆模型。")
    return dict(q=q, dqds=float(dqds), rotation=rotation, A=ap, B=bp,
                dAds=ap_s, dBds=bp_s, rodRotation=rod_rotation,
                rodAngle=float(rod_angle),
                dRodAngleDs=float(axis @ np.cross(n, bp_s - ap_s) / (n @ n)),
                lengthResidual=float(np.linalg.norm(delta) - length), stroke=float(s))


def _indices(indices, nq):
    if indices is None or np.asarray(indices).size == 0:
        return np.arange(1, nq + 1)
    values = _vector(indices, "indices")
    if np.any(values != np.floor(values)) or np.any(values < 1) or np.any(values > nq):
        raise A32Error("AngleIndices", f"电机序号必须为 1～{nq} 的整数。")
    if np.unique(values).size != values.size:
        raise A32Error("AngleIndices", "同一次转换不能重复指定同一关节。")
    return values.astype(int)


def _check_configuration(model, config):
    motors = config["motors"]
    if len(motors) != model.nq or any(
        p["jointName"] != name for p, name in zip(motors, model.q_names)
    ) or config.get("sourceRobotName", model.robot_name) != model.robot_name:
        raise A32Error("Configuration", "配置的机器人、关节数量或排列与 URDF 不一致。")


def a32_angle_limits(model, config, k):
    """指定电机序号（从 1 开始）的有效角度范围，单位 rad。"""
    k = int(_indices(_vector(k, "k", 1), model.nq)[0])
    _check_configuration(model, config)
    i = k - 1
    limits = np.array([model.lower[i], model.upper[i]])
    p = config["motors"][i]
    if p["type"] == "rotary":
        return limits
    if p["type"] != "linear":
        raise A32Error("AngleMotorType", "无法识别该电机类型。")
    limits[0] = max(0, limits[0])
    strokes = _vector(p["strokeRange"], "strokeRange", 2)
    if strokes[0] > strokes[1]:
        raise A32Error("AngleStrokeLimit", "行程限位顺序错误。")
    ends = [a32_loop_kinematics(p, model.axes[i], s, limits)["q"] for s in strokes]
    result = np.array([max(limits[0], min(ends)), min(limits[1], max(ends))])
    if result[0] > result[1]:
        raise A32Error("AngleLimit", "几何行程与关节角度范围没有交集。")
    return result


def a32_angles_to_inputs(model, config, angle_deg, indices=None):
    """与 MATLAB 同名接口对应，返回 (一维 xSI 数组, info 字典)。

    angle_deg 是相对上一指节的角度（度）；省略 indices 时需全部 nq 个角度。
    xSI 中直线电机为 m，旋转电机为 rad；info['strokeMm'] 的旋转项为 NaN。
    所有检查通过才返回，不修改 model、config 或调用方输入。
    """
    _check_configuration(model, config)
    indices = _indices(indices, model.nq)
    degrees = _vector(angle_deg, "angle_deg", indices.size)
    inputs = np.zeros(indices.size)
    recovered = np.zeros(indices.size)
    strokes_mm = np.full(indices.size, np.nan)
    for row, k in enumerate(indices):
        i = k - 1
        p, axis = config["motors"][i], model.axes[i]
        q = np.deg2rad(degrees[row])
        lo, hi = a32_angle_limits(model, config, k)
        if q < lo - 1e-10 or q > hi + 1e-10:
            raise A32Error("AngleLimit", f"{model.q_names[i]}：输入 {degrees[row]:.8g}° "
                           f"超出 [{np.rad2deg(lo):.8g}, {np.rad2deg(hi):.8g}]°。")
        q = np.clip(q, lo, hi)
        if p["type"] == "linear":
            q_limits = [max(0, model.lower[i]), model.upper[i]]
            zero = a32_loop_kinematics(p, axis, 0, q_limits)
            if abs(zero["q"]) > 1e-9:
                raise A32Error("AngleZero", "行程零位未对应关节零位。")
            s = a32_loop_inverse(p, axis, q)
            s_lo, s_hi = p["strokeRange"]
            if not np.isfinite(s) or s < s_lo - 1e-10 or s > s_hi + 1e-10:
                raise A32Error("AngleStrokeLimit", "目标角度对应的行程超出几何范围。")
            s = float(np.clip(s, s_lo, s_hi))
            check = a32_loop_kinematics(p, axis, s, q_limits)
            if abs(check["q"] - q) > 1e-8 or abs(check["lengthResidual"]) > 1e-9:
                raise A32Error("AngleRoundTrip", "角度与行程往返校验失败，请检查几何和装配分支。")
            inputs[row], strokes_mm[row] = s, s * 1000
            recovered[row] = np.rad2deg(check["q"])
        else:
            inputs[row], recovered[row] = q, np.rad2deg(q)
    info = dict(jointIndices=indices, angleDeg=degrees, inputSI=inputs,
                strokeMm=strokes_mm, angleRecoveredDeg=recovered,
                maxAngleErrorDeg=float(np.max(np.abs(recovered - degrees))))
    return inputs, info


def angles_to_inputs(angle_deg, indices=None, *, model=None, config=None):
    """便捷入口：angles_to_inputs(45, 7) 或 angles_to_inputs(整手20个角度)。"""
    if model is None:
        model = load_kinematic_model()
    if config is None:
        config = load_configuration()
    return a32_angles_to_inputs(model, config, angle_deg, indices)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("angles", type=float, nargs="+", help="关节角，单位度")
    parser.add_argument("--indices", type=int, nargs="+", help="电机编号 1～20；省略则输入整手")
    args = parser.parse_args()
    try:
        model = load_kinematic_model()
        x, info = angles_to_inputs(args.angles, args.indices, model=model,
                                   config=load_configuration())
    except (ValueError, OSError) as error:
        parser.error(str(error))
    for row, k in enumerate(info["jointIndices"]):
        output = (f"{info['strokeMm'][row]:.9f} mm" if np.isfinite(info["strokeMm"][row])
                  else f"{x[row]:.12g} rad")
        print(f"{k:2d}  {model.q_names[k - 1]}  {info['angleDeg'][row]:g} deg -> {output}")


if __name__ == "__main__":
    main()
