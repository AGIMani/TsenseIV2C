"""A3.2 低延迟角度转换：预计算解析系数、均匀查表及批量转换。

converter = A32Converter()                 # 启动时执行一次
hand = converter.prepare()                 # 缓存编号，默认精确解析
x = hand([0.0] * 20)                       # m / rad，一维数组
pip = converter.prepare(7, method="lut")   # 可选近似查表
s = pip.scalar(45.0)                       # m，避免小数组开销
"""

from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache
import math
from numbers import Real

import numpy as np

from .a32_angles_to_inputs import (
    A32Error, _check_configuration, _geometry, _indices, _vector,
    a32_angle_limits, a32_angles_to_inputs, a32_loop_kinematics,
    load_configuration, load_kinematic_model,
)


_DEG = math.pi / 180


@dataclass(frozen=True)
class _Kernel:
    name: str
    linear: bool
    lo: float
    hi: float
    stroke_lo: float = 0.0
    stroke_hi: float = 0.0
    # t = tc*cos(q) + ts*sin(q) + t0
    tc: float = 0.0
    ts: float = 0.0
    t0: float = 0.0
    # 根号项 r = rc + rcos*cos(q) + rsin*sin(q) + factor*t*t
    rc: float = 0.0
    rcos: float = 0.0
    rsin: float = 0.0
    factor: float = 1.0
    root_sign: float = 1.0
    # 压缩后的正解：A(s)cos(q)+B(s)sin(q)=D(s)
    a: float = 0.0
    da: float = 0.0
    b: float = 0.0
    db: float = 0.0
    d: float = 0.0
    dd: float = 0.0
    d2: float = 0.0
    branch: float = 1.0
    length: float = 0.0

    def angle(self, degrees):
        q = degrees * _DEG
        if q < self.lo - 1e-10 or q > self.hi + 1e-10:
            raise A32Error("AngleLimit", f"{self.name}：角度超出允许范围。")
        return min(self.hi, max(self.lo, q))

    def exact(self, q):
        if not self.linear:
            return q
        co, si = math.cos(q), math.sin(q)
        t = self.tc * co + self.ts * si + self.t0
        r = self.rc + self.rcos * co + self.rsin * si + self.factor * t * t
        if r < -1e-14:
            raise A32Error("LoopUnreachable", f"{self.name}：目标角度不可达。")
        s = t + self.root_sign * math.sqrt(max(0.0, r))
        if s < self.stroke_lo - 1e-10 or s > self.stroke_hi + 1e-10:
            raise A32Error("AngleStrokeLimit", f"{self.name}：行程超出几何范围。")
        s = min(self.stroke_hi, max(self.stroke_lo, s))
        # 仍逐次核对正解支路、角度和杆长，只计算必要的标量。
        a, b = self.a + self.da * s, self.b + self.db * s
        d = self.d + self.dd * s + self.d2 * s * s
        rho = math.hypot(a, b)
        if rho < 1e-14 or abs(d) > rho * (1 + 1e-10):
            raise A32Error("LoopUnreachable", f"{self.name}：闭环退化或不可达。")
        recovered = math.atan2(b, a) + self.branch * math.acos(max(-1.0, min(1.0, d / rho)))
        h = 2 * (d - a * co - b * si)
        residual = math.sqrt(max(0.0, self.length**2 + h)) - self.length
        hq = 2 * (a * si - b * co)
        if abs(hq) < 1e-10 * max(self.length**2, 1e-8):
            raise A32Error("LoopSingular", f"{self.name}：闭环处于死点。")
        if abs(math.remainder(recovered - q, 2 * math.pi)) > 1e-8 or abs(residual) > 1e-9:
            raise A32Error("AngleRoundTrip", f"{self.name}：闭环往返校验失败。")
        return s

    def exact_batch(self, q):
        if not self.linear:
            return q
        co, si = np.cos(q), np.sin(q)
        t = self.tc * co + self.ts * si + self.t0
        r = self.rc + self.rcos * co + self.rsin * si + self.factor * t * t
        if np.any(r < -1e-14):
            raise A32Error("LoopUnreachable", f"{self.name}：批量角度含不可达值。")
        s = t + self.root_sign * np.sqrt(np.maximum(0, r))
        if np.any(s < self.stroke_lo - 1e-10) or np.any(s > self.stroke_hi + 1e-10):
            raise A32Error("AngleStrokeLimit", f"{self.name}：批量行程越界。")
        s = np.clip(s, self.stroke_lo, self.stroke_hi)
        a, b = self.a + self.da * s, self.b + self.db * s
        d = self.d + self.dd * s + self.d2 * s * s
        rho = np.hypot(a, b)
        if np.any(rho < 1e-14) or np.any(np.abs(d) > rho * (1 + 1e-10)):
            raise A32Error("LoopUnreachable", f"{self.name}：批量闭环退化或不可达。")
        recovered = np.arctan2(b, a) + self.branch * np.arccos(np.clip(d / rho, -1, 1))
        angle_error = (recovered - q + np.pi) % (2 * np.pi) - np.pi
        h = 2 * (d - a * co - b * si)
        residual = np.sqrt(np.maximum(0, self.length**2 + h)) - self.length
        if np.any(np.abs(2 * (a * si - b * co)) < 1e-10 * max(self.length**2, 1e-8)):
            raise A32Error("LoopSingular", f"{self.name}：批量闭环包含死点。")
        if np.any(np.abs(angle_error) > 1e-8) or np.any(np.abs(residual) > 1e-9):
            raise A32Error("AngleRoundTrip", f"{self.name}：批量闭环往返校验失败。")
        return s


@dataclass(frozen=True)
class _Table:
    lo: float
    hi: float
    scale: float
    values: tuple[float, ...]
    array: np.ndarray
    error_bound_m: float

    def evaluate(self, q):
        position = (q - self.lo) * self.scale
        i = min(len(self.values) - 2, max(0, int(position)))
        fraction = min(1.0, max(0.0, position - i))
        return self.values[i] + fraction * (self.values[i + 1] - self.values[i])

    def batch(self, q):
        position = (q - self.lo) * self.scale
        i = np.clip(position.astype(np.int64), 0, len(self.values) - 2)
        fraction = np.clip(position - i, 0, 1)
        return self.array[i] + fraction * (self.array[i + 1] - self.array[i])


def _make_table(k, tolerance):
    """用 |s''| 的保守上界选择网格，线性插值误差 <= max|s''|*h²/8。

    这里的上界描述实数运算下的插值截断误差；浮点舍入另由回归测试量化。
    不以几个采样点的误差充当整个区间的保证。
    """
    if k.hi <= k.lo or not math.isfinite(k.hi - k.lo):
        raise A32Error("LookupGeometry", "查表需要有限且非零的角度区间。")
    tp = abs(k.tc) + abs(k.ts)
    tm = tp + abs(k.t0)
    rthird = abs(k.rcos) + abs(k.rsin) + 2 * abs(k.factor) * (3 * tp * tp + tm * tp)
    # 对网格上 r 的最小值减去 Lipschitz 余量，界定连续区间的最小根号项。
    for count in (1024, 4096, 16384, 65536):
        q = np.linspace(k.lo, k.hi, count + 1)
        co, si = np.cos(q), np.sin(q)
        t = k.tc * co + k.ts * si + k.t0
        dt = -k.tc * si + k.ts * co
        ddt = -k.tc * co - k.ts * si
        r = k.rc + k.rcos * co + k.rsin * si + k.factor * t * t
        # 舍入余量避免把近零根号项错误地认证为正。
        margin = 64 * np.finfo(float).eps * (
            abs(k.rc) + abs(k.rcos) + abs(k.rsin) + abs(k.factor) * tm * tm)
        half_step = (k.hi - k.lo) / (2 * count)
        # 依次用三阶导数界限制二阶导数、用二阶界限制一阶；比各系数绝对值
        # 相加更紧，同时覆盖网格之间的位置，避免仅以离散最大值估计曲率。
        rpp = float(np.max(np.abs(-k.rcos * co - k.rsin * si +
                                  2 * k.factor * (dt * dt + t * ddt)))) + rthird * half_step + margin
        rp = float(np.max(np.abs(-k.rcos * si + k.rsin * co +
                                 2 * k.factor * t * dt))) + rpp * half_step + margin
        rmin = float(np.min(r)) - rp * half_step - margin
        if rmin > 0:
            break
    else:
        raise A32Error("LookupGeometry", f"{k.name}：无法界定查表曲率，请使用 exact。")
    curvature = tp + rpp / (2 * math.sqrt(rmin)) + rp * rp / (4 * rmin**1.5)
    # 留出 10% 插值误差余量；过严精度要求明确拒绝，不静默降低精度。
    intervals = max(1, math.ceil((k.hi - k.lo) * math.sqrt(curvature / (8 * tolerance * .9))))
    if intervals > 262144:
        raise A32Error("LookupSize", f"{k.name}：所需查表过大，请增大容差或使用 exact。")
    q = np.linspace(k.lo, k.hi, intervals + 1)
    values = k.exact_batch(q)
    values.setflags(write=False)
    error_bound = curvature * ((k.hi - k.lo) / intervals)**2 / 8
    return _Table(k.lo, k.hi, intervals / (k.hi - k.lo),
                  tuple(values.tolist()), values, error_bound)


class PreparedConversion:
    """已固定电机编号和方法的调用器；可反复调用，无磁盘读写。

    __call__ 返回形状 (K,) 的 SI 输入；scalar 用于 K=1；
    batch 接受 (N,K) 并返回 (N,K)。不创建旧接口的诊断 info 字典。
    """

    def __init__(self, kernels, indices, method, tables):
        self._kernels = tuple(kernels)
        self.indices = tuple(int(i) for i in indices)
        self.method = method
        self._tables = tuple(tables)
        self.n = len(kernels)

    @property
    def limits_deg(self):
        return np.array([(k.lo / _DEG, k.hi / _DEG) for k in self._kernels])

    @property
    def lookup_error_bound_m(self):
        """每个直线通道的插值截断误差上界；旋转通道/精确模式为0。"""
        return np.array([t.error_bound_m if t is not None else 0 for t in self._tables])

    @property
    def lookup_samples(self):
        return tuple(len(t.values) if t is not None else 0 for t in self._tables)

    def _one(self, degrees, k, table):
        q = k.angle(degrees)
        return table.evaluate(q) if table is not None else k.exact(q)

    def scalar(self, angle_deg):
        if self.n != 1:
            raise A32Error("Input", "scalar 仅适用于单个电机；请先 prepare(编号)。")
        if isinstance(angle_deg, (bool, np.bool_)) or not isinstance(angle_deg, Real):
            raise A32Error("Input", "角度必须为有限实数标量。")
        angle_deg = float(angle_deg)
        if not math.isfinite(angle_deg):
            raise A32Error("Input", "角度不能为 NaN/Inf。")
        return self._one(angle_deg, self._kernels[0], self._tables[0])

    def __call__(self, angle_deg):
        degrees = _vector(angle_deg, "angle_deg", self.n)
        return np.fromiter((self._one(float(d), k, t) for d, k, t in
                            zip(degrees, self._kernels, self._tables)), dtype=float, count=self.n)

    def batch(self, angle_deg):
        degrees = np.asarray(angle_deg)
        if degrees.dtype.kind not in "iuf" or degrees.ndim != 2 or degrees.shape[1] != self.n:
            raise A32Error("Input", f"批量角度必须为实数二维数组 (N,{self.n})。")
        if not np.all(np.isfinite(degrees)):
            raise A32Error("Input", "批量角度不能包含 NaN/Inf。")
        q = degrees.astype(float) * _DEG
        result = np.empty_like(q)
        for col, (k, table) in enumerate(zip(self._kernels, self._tables)):
            values = q[:, col]
            if np.any(values < k.lo - 1e-10) or np.any(values > k.hi + 1e-10):
                raise A32Error("AngleLimit", f"{k.name}：批量角度越界。")
            values = np.clip(values, k.lo, k.hi)
            result[:, col] = table.batch(values) if table is not None else k.exact_batch(values)
        return result


class A32Converter:
    """初始化时复制为不可变的标量系数；原 model/config 改动不会污染缓存。

    exact 保留逐次的可达性、行程、奇异性、正解支路和往返校验。
    lut 用有曲率误差界的均匀线性插值，初始化校验节点；调用时检查角度
    范围及有限性，不逐次计算正解。默认容差 1e-9 m = 1e-6 mm。
    修改几何或限位后必须重建转换器；初始化不应放在控制循环内。
    """

    def __init__(self, model=None, config=None, *, lut_tolerance_m=1e-9):
        if isinstance(lut_tolerance_m, (bool, np.bool_)) or not isinstance(lut_tolerance_m, Real) or not math.isfinite(lut_tolerance_m) or lut_tolerance_m < 1e-12:
            raise A32Error("LookupTolerance", "查表误差容差必须为不小于 1e-12 m 的有限实数。")
        model = load_kinematic_model() if model is None else model
        config = load_configuration() if config is None else config
        _check_configuration(model, config)
        kernels = []
        for i, p in enumerate(config["motors"]):
            lo, hi = map(float, a32_angle_limits(model, config, i + 1))
            if p["type"] == "rotary":
                kernels.append(_Kernel(model.q_names[i], False, lo, hi))
                continue
            if hi - lo >= 2 * math.pi:
                raise A32Error("LoopBranch", "快速闭环需要小于一周的唯一角度范围。")
            axis, a0, b, u, length = _geometry(p, model.axes[i])
            bp = axis * (axis @ b)
            bn = b - bp
            bc = np.cross(axis, b)
            direction = 1 if p["sliderOn"] == "parent" else -1
            v0 = b - a0
            t0 = u @ v0
            z = math.sqrt(max(0, length**2 - np.linalg.norm(v0 - u * t0)**2))
            roots = [t0 - z, t0 + z]
            root_index = int(np.argmin(np.abs(roots)))
            if abs(roots[root_index]) > 1e-9 or abs(roots[1] - roots[0]) < 1e-10:
                raise A32Error("LoopBranch", "无法根据零位确定行程装配分支。")
            raw = dict(stroke_lo=p["strokeRange"][0], stroke_hi=p["strokeRange"][1],
                       tc=u @ bn, ts=direction * (u @ bc), t0=u @ (bp - a0),
                       rc=length**2 - b @ b - a0 @ a0 + 2 * (a0 @ bp),
                       rcos=2 * (a0 @ bn), rsin=2 * direction * (a0 @ bc),
                       factor=2 - u @ u, root_sign=-1 if root_index == 0 else 1,
                       a=a0 @ bn, da=u @ bn,
                       b=direction * (a0 @ bc), db=direction * (u @ bc),
                       d=(b @ b + a0 @ a0 - 2 * (a0 @ bp) - length**2) / 2,
                       dd=a0 @ u - u @ bp, d2=(u @ u) / 2,
                       branch=p["branchSign"], length=length)
            k = _Kernel(model.q_names[i], True, lo, hi, **{key: float(v) for key, v in raw.items()})
            # 原求解器核对零位及整个有效范围的代表点，避免简化公式掩盖配置错误。
            zero = a32_loop_kinematics(p, axis, 0, [max(0, model.lower[i]), model.upper[i]])
            if abs(zero["q"]) > 1e-9:
                raise A32Error("AngleZero", f"{k.name}：行程零位未对应关节零位。")
            probes = np.linspace(lo, hi, 17)
            for q in probes:
                expected, _ = a32_angles_to_inputs(model, config, float(q / _DEG), i + 1)
                if abs(k.exact(k.angle(float(q / _DEG))) - expected[0]) > 1e-12:
                    raise A32Error("FastValidation", f"{k.name}：预计算与原求解器不一致。")
            kernels.append(k)
        self._kernels = tuple(kernels)
        self.nq = len(kernels)
        self.lut_tolerance_m = float(lut_tolerance_m)
        self._tables = {}

    def prepare(self, indices=None, *, method="exact"):
        """预绑定编号（沿用1起始）和算法；exact 精确，lut 近似查表。"""
        if method not in ("exact", "lut"):
            raise A32Error("Method", "method 必须为 exact 或 lut。")
        indices = _indices(indices, self.nq)
        kernels, tables = [], []
        for index in indices:
            k = self._kernels[index - 1]
            table = None
            if method == "lut" and k.linear:
                if index not in self._tables:
                    self._tables[index] = _make_table(k, self.lut_tolerance_m)
                table = self._tables[index]
            kernels.append(k)
            tables.append(table)
        return PreparedConversion(kernels, indices, method, tables)


@lru_cache(maxsize=1)
def default_converter():
    """进程内缓存当前默认几何；修改文件后调用 clear_default_cache() 或重建实例。"""
    return A32Converter()


@lru_cache(maxsize=128)
def _default_prepared(indices, method):
    return default_converter().prepare(indices, method=method)


def fast_angles_to_inputs(angle_deg, indices=None, *, method="exact"):
    """方便调用，返回一维 SI 数组；高频循环优先保存 prepare() 的返回值。

    默认模型首次使用时初始化；不返回原接口的 info，诊断请用原接口。
    """
    key = tuple(int(i) for i in _indices(indices, default_converter().nq))
    return _default_prepared(key, method)(angle_deg)


def clear_default_cache():
    """默认 URDF/Python 机构参数变更后清空缓存；已取得的调用器仍代表旧配置。"""
    _default_prepared.cache_clear()
    default_converter.cache_clear()
