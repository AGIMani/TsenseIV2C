"""
finger_linkage.py — 三直线电机闭环机构：行程 CF ↔ 关节角

算法来源（2026-07 最新 MATLAB，四文件几何一致）：
  - general_finger_gravity(1).m   一般指 F1/F2/F3
  - little_finger_gravity.m       左小指（直线几何同一般指）
  - right_little_finger_gravity.m 右小指（直线几何同；仅掌系镜像）
  - thumb_gravity.m              拇指 F1/F3（无 F2）
  - 正运动学：CF (mm) → phi (相对角 deg)  via solveClosedLoopAngleDeg（闭式，
    无需查找表；遥测用。输入可 ``clamp=True`` 钳到 [cfMin, cfMax]，吞掉零点噪声）
  - 逆运动学：phi (deg) → CF (mm)          默认查找表插值（HAND_LINKAGE_FIT=1）；
    亦可 HAND_LINKAGE_FIT=0 回退 [cfMin, cfMax] 二分（下发用；表只加速这一方向）
  - 电机顺序：F1 最上，F3 最下接手掌（与 MATLAB 一致）
  - cfMax = min(cfPhysicalMaxMm, cfGeometricMaxMm)
    一般指/小指 cfPhysicalMaxMm = [15, 16.5, 18]
    拇指       cfPhysicalMaxMm = [15, 18]（映射到 F1/F3 槽）

四指（食/中/无名/小指）默认约定：
  URDF 相对关节角 MCP/PIP/DIP (rad) ↔ 相对弯曲角 phi3/phi2/phi1
  F3(近掌)↔MCP, F2↔PIP, F1(远端)↔DIP

拇指：仅 F3 + F1 两级直线电机（无 F2），几何与四指的 F3/F1 相同。
  URDF thumb_MCP → F3，thumb_PIP → F1。

依赖：仅标准库（math/json/os/time）；可被 main_app_v3 直接
  from finger_linkage import ... 导入，不依赖 src/ 工程包。
"""

from __future__ import annotations

import math
import json
import os
import time
from bisect import bisect_right
from dataclasses import dataclass, field
from typing import Dict, List, Mapping, Optional, Sequence, Tuple

# 可选工程日志；独立运行 main_app_v3 时降级为 no-op。
try:
    from utils.log_control import emit as log_emit, register_tag
except Exception:  # noqa: BLE001
    def log_emit(msg: str, tag: str = "") -> None:  # type: ignore[misc]
        return None

    def register_tag(tag: str, description: str = "", *, enabled: bool = True) -> None:  # type: ignore[misc]
        return None

# 电机下标：0=F1(上), 1=F2(中), 2=F3(下/接掌)
MOTOR_F1, MOTOR_F2, MOTOR_F3 = 0, 1, 2
MOTOR_COUNT = 3

_RAD_TO_DEG = 180.0 / math.pi
_DEG_TO_RAD = math.pi / 180.0

_LOG_TAG = "FingerLinkage"
register_tag(_LOG_TAG, "闭环连杆角度↔行程逆解", enabled=True)

# 同指失败日志限频，避免 1kHz 下发/遥测刷屏
_KIN_FAIL_LOG_INTERVAL_S = 0.5
_kin_fail_last_log: Dict[str, float] = {}
_KIN_FAIL_PREFIX = {
    "ik": "逆解失败",  # phi → CF（下发）
    "fk": "正解失败",  # CF → phi（遥测）
}

# USB 整手批量槽位（与 main_app_v3.py / usb.offline.py 一致）
LINEAR_SLOT_JOINTS: Tuple[str, ...] = (
    "left_hand_ring_MCP_joint",
    "left_hand_ring_PIP_joint",  # DIP/PIP 槽位已实机验证
    "left_hand_ring_DIP_joint",  # DIP/PIP 槽位已实机验证
    "left_hand_pinky_MCP_joint",
    "left_hand_pinky_PIP_joint",  # DIP/PIP 槽位已实机验证
    "left_hand_pinky_DIP_joint",  # DIP/PIP 槽位已实机验证
    "left_hand_index_MCP_joint",
    "left_hand_index_PIP_joint",  # DIP/PIP 槽位已实机验证
    "left_hand_index_DIP_joint",  # DIP/PIP 槽位已实机验证
    "left_hand_mid_MCP_joint",
    "left_hand_mid_PIP_joint",  # DIP/PIP 槽位已实机验证
    "left_hand_mid_DIP_joint",  # DIP/PIP 槽位已实机验证
    "left_hand_thumb_MCP_joint",
    "left_hand_thumb_PIP_joint",
)
ROTARY_SLOT_JOINTS: Tuple[str, ...] = (
    "left_hand_index_swing_joint",
    "left_hand_mid_swing_joint",
    "left_hand_ring_swing_joint",
    "left_hand_pinky_swing_joint",
    "left_hand_thumb_rota_joint",
    "left_hand_thumb_swing_joint",
    "left_hand_pinky_rota_joint",
)


class LinkageKinematicsError(ValueError):
    """闭环不可装配、越界或逆解失败。"""

    def __init__(self, message: str, *, context: Optional[Dict] = None):
        super().__init__(message)
        self.context: Dict = dict(context or {})


def _log_kin_failure(
    key: str,
    reason: str,
    *,
    direction: str = "ik",
    **params,
) -> None:
    """打印连杆运动学失败原因与参数（按 key 限频）。

    ``direction``:
      - ``\"ik\"`` → 前缀「逆解失败」（phi→CF，下发）
      - ``\"fk\"`` → 前缀「正解失败」（CF→phi，遥测）
    """
    now = time.monotonic()
    last = _kin_fail_last_log.get(key, 0.0)
    if now - last < _KIN_FAIL_LOG_INTERVAL_S:
        return
    _kin_fail_last_log[key] = now
    prefix = _KIN_FAIL_PREFIX.get(direction, "运动学失败")
    parts = [f"{prefix} [{key}] {reason}"]
    for name, val in params.items():
        if isinstance(val, float):
            parts.append(f"{name}={val:.6g}")
        else:
            parts.append(f"{name}={val}")
    log_emit(" | ".join(parts), _LOG_TAG)


# 一般指 / 左右小指：MATLAB cfPhysicalMaxMm
_CF_PHYSICAL_MAX_MM_GENERAL = (15.0, 16.5, 18.0)
# 拇指 F1/F3：MATLAB [15, 18]，嵌入三槽时 F2 槽不用
_CF_PHYSICAL_MAX_MM_THUMB = (15.0, 16.5, 18.0)


@dataclass
class FingerLinkageParams:
    """与 MATLAB defaultParameters + finalizeParameters 一致的几何参数（mm）。

    三槽顺序恒为 F1/F2/F3。拇指只用 F1+F3，F2 槽保留但不参与映射。
    """

    stator_length_mm: Tuple[float, float, float] = (29.5, 42.5, 53.5)
    mover_length_mm: Tuple[float, float, float] = (19.3, 30.3, 41.3)
    mover_diameter_mm: Tuple[float, float, float] = (11.5, 11.5, 11.5)
    stator_inner_diameter_mm: Tuple[float, float, float] = (11.8, 11.8, 11.8)
    stator_outer_diameter_mm: Tuple[float, float, float] = (20.0, 20.0, 20.0)
    rod_length_mm: Tuple[float, float, float] = (13.5, 12.5, 13.5)
    joint_gap_mm: Tuple[float, float, float] = (2.3, 0.3, 1.3)
    flexion_direction: Tuple[int, int, int] = (1, 1, 1)
    # MATLAB cfPhysicalMaxMm；finalize 后 cf_max = min(phys, geo)
    cf_physical_max_mm: Tuple[float, float, float] = _CF_PHYSICAL_MAX_MM_GENERAL

    # finalize 后填充
    outer_radius_mm: Tuple[float, float, float] = field(default_factory=lambda: (0.0, 0.0, 0.0))
    s_a0_mm: Tuple[float, float, float] = field(default_factory=lambda: (0.0, 0.0, 0.0))
    parent_radius_mm: Tuple[float, float, float] = field(default_factory=lambda: (0.0, 0.0, 0.0))
    a_joint_offset_mm: Tuple[float, float, float] = field(default_factory=lambda: (0.0, 0.0, 0.0))
    cf_min_mm: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    cf_max_mm: Tuple[float, float, float] = field(default_factory=lambda: (0.0, 0.0, 0.0))
    max_relative_angle_deg: Tuple[float, float, float] = field(default_factory=lambda: (0.0, 0.0, 0.0))

    @classmethod
    def from_matlab_defaults(cls) -> "FingerLinkageParams":
        """一般指 / 小指：general_finger / little_finger / right_little_finger。"""
        return finalize_linkage_params(cls())

    @classmethod
    def from_thumb_matlab_defaults(cls) -> "FingerLinkageParams":
        """拇指 thumb_gravity.m：仅 F1+F3 有意义；几何嵌在三槽 F1/F3。"""
        return finalize_linkage_params(
            cls(cf_physical_max_mm=_CF_PHYSICAL_MAX_MM_THUMB)
        )

    def motor_cf_limits(self, motor: int) -> Tuple[float, float]:
        return self.cf_min_mm[motor], self.cf_max_mm[motor]


def finalize_linkage_params(p: FingerLinkageParams) -> FingerLinkageParams:
    """复现 MATLAB finalizeParameters（cfMax=min(phys,geo)）。"""
    outer = tuple(0.5 * d for d in p.stator_outer_diameter_mm)
    s_a0 = tuple(ls - lm for ls, lm in zip(p.stator_length_mm, p.mover_length_mm))
    # F_i 的父级半径：非末级用下一级外径，末级用自身（与 MATLAB parentRadiusMm 一致）
    parent_r = tuple(
        outer[i + 1] if i < MOTOR_COUNT - 1 else outer[i] for i in range(MOTOR_COUNT)
    )

    a_off = list(p.a_joint_offset_mm)
    for i in range(MOTOR_COUNT):
        dx = parent_r[i] - outer[i]
        lab = p.rod_length_mm[i]
        if abs(dx) >= lab:
            raise LinkageKinematicsError(f"F{i + 1} 横向尺寸使 AB 连杆无法闭合")
        req_axial = math.sqrt(lab * lab - dx * dx)
        a_off[i] = req_axial - (s_a0[i] + p.joint_gap_mm[i])

    cf_phys_max = tuple(float(v) for v in p.cf_physical_max_mm)
    if len(cf_phys_max) != MOTOR_COUNT:
        raise LinkageKinematicsError(
            f"cf_physical_max_mm 长度须为 {MOTOR_COUNT}，得到 {len(cf_phys_max)}"
        )
    cf_geo_max = []
    for i in range(MOTOR_COUNT):
        pp = FingerLinkageParams(
            stator_length_mm=p.stator_length_mm,
            mover_length_mm=p.mover_length_mm,
            mover_diameter_mm=p.mover_diameter_mm,
            stator_inner_diameter_mm=p.stator_inner_diameter_mm,
            stator_outer_diameter_mm=p.stator_outer_diameter_mm,
            rod_length_mm=p.rod_length_mm,
            joint_gap_mm=p.joint_gap_mm,
            flexion_direction=p.flexion_direction,
            cf_physical_max_mm=cf_phys_max,
            outer_radius_mm=outer,
            s_a0_mm=s_a0,
            parent_radius_mm=parent_r,
            a_joint_offset_mm=tuple(a_off),
            cf_min_mm=p.cf_min_mm,
            cf_max_mm=cf_phys_max,
        )
        cf_geo_max.append(_find_geometric_cf_limit(i, pp))

    cf_max = tuple(min(a, b) for a, b in zip(cf_phys_max, cf_geo_max))
    max_phi = tuple(
        cf_to_phi_deg(
            cf_max[i],
            i,
            _with_limits(p, outer, s_a0, parent_r, tuple(a_off), cf_phys_max, cf_max),
        )
        for i in range(MOTOR_COUNT)
    )

    p.outer_radius_mm = outer
    p.s_a0_mm = s_a0
    p.parent_radius_mm = parent_r
    p.a_joint_offset_mm = tuple(a_off)
    p.cf_physical_max_mm = cf_phys_max
    p.cf_max_mm = cf_max
    p.max_relative_angle_deg = max_phi
    return p


def _with_limits(
    base: FingerLinkageParams,
    outer,
    s_a0,
    parent_r,
    a_off,
    cf_phys_max,
    cf_max,
) -> FingerLinkageParams:
    return FingerLinkageParams(
        stator_length_mm=base.stator_length_mm,
        mover_length_mm=base.mover_length_mm,
        mover_diameter_mm=base.mover_diameter_mm,
        stator_inner_diameter_mm=base.stator_inner_diameter_mm,
        stator_outer_diameter_mm=base.stator_outer_diameter_mm,
        rod_length_mm=base.rod_length_mm,
        joint_gap_mm=base.joint_gap_mm,
        flexion_direction=base.flexion_direction,
        cf_physical_max_mm=cf_phys_max,
        outer_radius_mm=outer,
        s_a0_mm=s_a0,
        parent_radius_mm=parent_r,
        a_joint_offset_mm=a_off,
        cf_min_mm=base.cf_min_mm,
        cf_max_mm=cf_max,
    )


def cf_to_phi_deg(
    cf_mm: float,
    motor: int,
    params: FingerLinkageParams,
    *,
    clamp: bool = False,
) -> float:
    """CF (mm) → 相对弯曲角 phi (deg)，对应 MATLAB solveClosedLoopAngleDeg。

    闭式解，O(1)；**不需要**查找表加速（``HAND_LINKAGE_FIT`` 只服务逆解 phi→CF）。

    ``clamp=True``：先把 CF 钳到 ``[cf_min, cf_max]``。遥测路径应开启——
    编码器零点噪声常把 CF 打到略负，硬解会报「无符合弯曲方向的闭环解」。
    """
    if not math.isfinite(cf_mm):
        raise LinkageKinematicsError(
            f"F{motor + 1} 正解输入非有限 CF={cf_mm}",
            context={"motor": motor + 1, "cf_mm": cf_mm},
        )
    if clamp:
        cf_lo, cf_hi = params.motor_cf_limits(motor)
        if cf_mm < cf_lo:
            cf_mm = cf_lo
        elif cf_mm > cf_hi:
            cf_mm = cf_hi

    d = params.flexion_direction[motor]
    r_child = params.outer_radius_mm[motor]
    r_parent = params.parent_radius_mm[motor]
    g = params.joint_gap_mm[motor]
    lab = params.rod_length_mm[motor]

    s_a_phys = params.s_a0_mm[motor] - cf_mm
    z_a = g * 0.5 + s_a_phys + params.a_joint_offset_mm[motor]

    ax, az = -d * r_child, z_a
    bx, bz = -d * r_parent, -g * 0.5

    a_coef = ax * bx + az * bz
    b_coef = az * bx - ax * bz
    c_coef = (ax * ax + az * az + bx * bx + bz * bz - lab * lab) * 0.5
    amplitude = math.hypot(a_coef, b_coef)

    if amplitude < 1e-12:
        raise LinkageKinematicsError(f"F{motor + 1} 闭环几何退化")

    ratio = c_coef / amplitude
    if ratio < -1.0 - 1e-9 or ratio > 1.0 + 1e-9:
        raise LinkageKinematicsError(
            f"F{motor + 1} 在 CF={cf_mm:.5f} mm 时闭环不可装配"
        )
    ratio = max(-1.0, min(1.0, ratio))

    delta_deg = math.degrees(math.atan2(b_coef, a_coef))
    alpha_deg = math.degrees(math.acos(ratio))

    roots = [delta_deg - alpha_deg, delta_deg + alpha_deg]
    base = list(roots)
    roots.extend(r - 360.0 for r in base)
    roots.extend(r + 360.0 for r in base)

    valid = [r for r in roots if d * r >= -1e-8 and abs(r) <= 179.999]
    if not valid:
        raise LinkageKinematicsError(f"F{motor + 1} 无符合弯曲方向的闭环解")

    phi_deg = min(valid, key=abs)
    if abs(cf_mm - params.cf_min_mm[motor]) < 1e-10 and abs(phi_deg) < 1e-7:
        phi_deg = 0.0
    return phi_deg


def phi_deg_to_cf_mm(
    phi_deg: float,
    motor: int,
    params: FingerLinkageParams,
    *,
    tol_deg: float = 1e-4,
    max_iter: int = 60,
    exact: Optional[bool] = None,
) -> float:
    """相对角 phi (deg) → CF (mm)。

    默认用离线采样 + 线性插值表（``HAND_LINKAGE_FIT=1``），避免每次二分
    反复调用 ``cf_to_phi_deg``。设 ``HAND_LINKAGE_FIT=0`` 或 ``exact=True``
    时回退到精确二分。
    """
    use_fit = _linkage_fit_enabled() if exact is None else (not exact)
    if not math.isfinite(phi_deg):
        raise LinkageKinematicsError(
            f"F{motor + 1} 逆解输入非有限 phi={phi_deg}",
            context={"motor": motor + 1, "phi_deg": phi_deg},
        )
    if use_fit:
        try:
            # 拟合表按正向几何 (CF→phi) 建立；输入先硬钳到表端点
            return _get_phi_cf_lut(motor, params).eval(phi_deg)
        except LinkageKinematicsError:
            pass
    return _phi_deg_to_cf_mm_bisection(
        phi_deg, motor, params, tol_deg=tol_deg, max_iter=max_iter
    )


def _phi_deg_to_cf_mm_bisection(
    phi_deg: float,
    motor: int,
    params: FingerLinkageParams,
    *,
    tol_deg: float = 1e-4,
    max_iter: int = 60,
) -> float:
    """精确二分逆解（原实现）。"""
    cf_lo, cf_hi = params.motor_cf_limits(motor)
    if not math.isfinite(phi_deg):
        raise LinkageKinematicsError(
            f"F{motor + 1} 逆解输入非有限 phi={phi_deg}",
            context={
                "motor": motor + 1,
                "phi_deg": phi_deg,
                "cf_lo": cf_lo,
                "cf_hi": cf_hi,
            },
        )
    target = clamp_phi_deg_to_motor(phi_deg, motor, params)
    phi_lim = motor_phi_deg_limits(motor, params)

    lo, hi = cf_lo, cf_hi
    try:
        phi_lo = cf_to_phi_deg(lo, motor, params)
    except LinkageKinematicsError as exc:
        raise LinkageKinematicsError(
            f"F{motor + 1} 逆解下界 CF 不可解: {exc}",
            context={
                "motor": motor + 1,
                "phi_in_deg": float(phi_deg),
                "phi_target_deg": float(target),
                "phi_lim_deg": phi_lim,
                "cf_lo": cf_lo,
                "cf_hi": cf_hi,
            },
        ) from exc

    mid_fail_count = 0
    for _ in range(max_iter):
        mid = 0.5 * (lo + hi)
        try:
            phi_mid = cf_to_phi_deg(mid, motor, params)
        except LinkageKinematicsError:
            # 中点不可装配时收缩上界，避免二分发散
            hi = mid
            mid_fail_count += 1
            continue
        if abs(phi_mid - target) < tol_deg:
            return mid
        if (phi_mid - target) * (phi_lo - target) <= 0:
            hi = mid
        else:
            lo = mid
            phi_lo = phi_mid

    if mid_fail_count > 0 and abs(hi - lo) < 1e-9:
        raise LinkageKinematicsError(
            f"F{motor + 1} 逆解二分耗尽且区间塌缩 (mid_fail={mid_fail_count})",
            context={
                "motor": motor + 1,
                "phi_in_deg": float(phi_deg),
                "phi_target_deg": float(target),
                "phi_lim_deg": phi_lim,
                "cf_lo": cf_lo,
                "cf_hi": cf_hi,
                "mid_fail_count": mid_fail_count,
            },
        )
    return 0.5 * (lo + hi)


# ── phi→CF 拟合表（正向 cf→phi 闭式采样，在线 O(1) 插值）──
# HAND_LINKAGE_FIT=0 强制精确二分；HAND_LINKAGE_LUT_N 控制表点数（默认 256）
_LINKAGE_FIT_DEFAULT = os.environ.get("HAND_LINKAGE_FIT", "1").strip().lower() not in (
    "0",
    "false",
    "no",
    "off",
)
_LINKAGE_LUT_N = max(32, int(float(os.environ.get("HAND_LINKAGE_LUT_N", "256"))))
_PHI_CF_LUT_CACHE: Dict[Tuple[int, Tuple[float, ...]], "PhiCfLut"] = {}
_PREPARED_LINKAGE_CACHE: Dict[tuple, tuple] = {}
_LINKAGE_CACHE_GENERATION = 0
# Additional error relative to the existing piecewise-linear table, in mm.
_UNIFORM_LUT_ERROR_MM = 1e-5


def _linkage_fit_enabled() -> bool:
    return _LINKAGE_FIT_DEFAULT


def _params_lut_key(params: FingerLinkageParams) -> Tuple[float, ...]:
    """几何指纹：参数变了就重建表。"""
    return (
        *params.stator_length_mm,
        *params.mover_length_mm,
        *params.rod_length_mm,
        *params.joint_gap_mm,
        *params.cf_min_mm,
        *params.cf_max_mm,
        *params.cf_physical_max_mm,
        *params.outer_radius_mm,
        *params.s_a0_mm,
        *params.parent_radius_mm,
        *params.a_joint_offset_mm,
        *params.flexion_direction,
    )


@dataclass(frozen=True)
class PhiCfLut:
    """单电机 phi(deg)→CF(mm) 查找表；phi 升序，线性插值。"""

    motor: int
    phi: Tuple[float, ...]
    cf: Tuple[float, ...]

    def eval(self, phi_deg: float) -> float:
        if not math.isfinite(phi_deg):
            raise LinkageKinematicsError(
                f"F{self.motor + 1} 拟合逆解输入非有限 phi={phi_deg}"
            )
        phis = self.phi
        cfs = self.cf
        n = len(phis)
        if n == 0:
            raise LinkageKinematicsError(f"F{self.motor + 1} 拟合表为空")
        if n == 1 or phi_deg <= phis[0]:
            return cfs[0]
        if phi_deg >= phis[-1]:
            return cfs[-1]
        hi = bisect_right(phis, phi_deg)
        lo = hi - 1
        denom = phis[hi] - phis[lo]
        if abs(denom) < 1e-15:
            return cfs[lo]
        u = (phi_deg - phis[lo]) / denom
        return cfs[lo] + u * (cfs[hi] - cfs[lo])


def build_phi_cf_lut(
    motor: int,
    params: FingerLinkageParams,
    *,
    n_samples: Optional[int] = None,
) -> PhiCfLut:
    """用正向 ``cf_to_phi_deg`` 均匀采 CF，得到单调 (phi, cf) 表后插值求逆。"""
    n = int(n_samples) if n_samples is not None else _LINKAGE_LUT_N
    n = max(64, n)
    cf_lo, cf_hi = params.motor_cf_limits(motor)
    raw: List[Tuple[float, float]] = []
    for i in range(n):
        cf = cf_lo + (i / (n - 1)) * (cf_hi - cf_lo)
        try:
            phi = float(cf_to_phi_deg(cf, motor, params))
        except LinkageKinematicsError:
            continue
        if math.isfinite(phi):
            raw.append((phi, float(cf)))
    if len(raw) < 8:
        raise LinkageKinematicsError(
            f"F{motor + 1} 拟合表采样不足 ({len(raw)} 点)",
            context={"cf_lo": cf_lo, "cf_hi": cf_hi, "n": n},
        )
    raw.sort(key=lambda x: x[0])
    # 去重：同 phi 保留最后（更大 CF 侧）
    dedup: List[Tuple[float, float]] = [raw[0]]
    for phi, cf in raw[1:]:
        if abs(phi - dedup[-1][0]) <= 1e-10:
            dedup[-1] = (phi, cf)
        elif phi > dedup[-1][0]:
            dedup.append((phi, cf))
    return PhiCfLut(
        motor=motor,
        phi=tuple(p for p, _ in dedup),
        cf=tuple(c for _, c in dedup),
    )


def _get_phi_cf_lut(motor: int, params: FingerLinkageParams) -> PhiCfLut:
    key = (int(motor), _params_lut_key(params))
    lut = _PHI_CF_LUT_CACHE.get(key)
    if lut is None:
        lut = build_phi_cf_lut(motor, params)
        _PHI_CF_LUT_CACHE[key] = lut
    return lut


def clear_phi_cf_lut_cache() -> None:
    """测试 / 改参数后清空拟合表缓存。"""
    global _LINKAGE_CACHE_GENERATION
    _PHI_CF_LUT_CACHE.clear()
    _PREPARED_LINKAGE_CACHE.clear()
    _LINKAGE_CACHE_GENERATION += 1


@dataclass(frozen=True)
class UniformPhiCfLut:
    """Uniform angle grid, directly indexed; no per-call interval search."""

    lo: float
    hi: float
    scale: float
    values: Tuple[float, ...]
    max_source_error_mm: float

    def eval(self, phi: float) -> float:
        if not math.isfinite(phi):
            raise LinkageKinematicsError("Uniform lookup input must be finite")
        if phi <= self.lo:
            return self.values[0]
        if phi >= self.hi:
            return self.values[-1]
        position = (phi - self.lo) * self.scale
        i = min(int(position), len(self.values) - 2)
        a = self.values[i]
        return a + (position - i) * (self.values[i + 1] - a)


def _uniform_phi_cf_lut(source: PhiCfLut) -> UniformPhiCfLut:
    """Resample with a bound over the WHOLE existing interpolant.

    The difference of two piecewise-linear functions is linear between the
    union of their knots. It is zero at new knots, so checking every original
    knot bounds the whole interval (apart from floating-point roundoff).
    No geometric accuracy claim is inferred from a sparse random scan.
    """
    lo, hi = source.phi[0], source.phi[-1]
    if not hi > lo:
        raise LinkageKinematicsError("Uniform lookup needs a nonzero angle span")
    intervals = max(64, len(source.phi) - 1)
    while intervals <= 262144:
        values = tuple(source.eval(lo + (hi - lo) * i / intervals)
                       for i in range(intervals + 1))
        table = UniformPhiCfLut(lo, hi, intervals / (hi - lo), values, 0.0)
        error = max(abs(table.eval(phi) - cf) for phi, cf in zip(source.phi, source.cf))
        if error <= _UNIFORM_LUT_ERROR_MM:
            return UniformPhiCfLut(lo, hi, table.scale, values, error)
        intervals *= 2
    raise LinkageKinematicsError("Uniform lookup could not meet the error bound")


@dataclass(frozen=True)
class PreparedMotorConversion:
    """Prebind the existing soft limit and lookup for one motor."""

    lo: float
    hi: float
    blend: float
    table: UniformPhiCfLut

    def joint_rad_to_cf_mm(self, rad: float) -> float:
        # Preserve zero-angle behavior for NaN/Inf and rad->deg overflow,
        # including custom nonzero lower limits.
        phi = rad * _RAD_TO_DEG
        if not math.isfinite(phi):
            return self.table.eval(0.0)
        phi = max(phi, self.lo)
        if self.blend > 0.0:
            edge = self.hi - self.blend
            if phi > edge:
                phi = edge + self.blend * (1.0 - math.exp(-(phi - edge) / self.blend))
        else:
            phi = min(phi, self.hi)
        return self.table.eval(phi)


def _get_prepared_motors(params: FingerLinkageParams) -> tuple:
    # One geometry check per finger, instead of one per limit AND lookup per
    # motor. Value fingerprints also detect mutable custom parameter edits.
    key = (_LINKAGE_CACHE_GENERATION, _LINKAGE_LUT_N, _params_lut_key(params))
    cached = getattr(params, "_prepared_conversion", None)
    if cached is not None and cached[0] == key:
        return cached[1]
    prepared = _PREPARED_LINKAGE_CACHE.get(key)
    if prepared is None:
        motors = []
        for motor in (MOTOR_F1, MOTOR_F2, MOTOR_F3):
            try:
                lo, hi = motor_phi_deg_limits(motor, params)
                table = _uniform_phi_cf_lut(_get_phi_cf_lut(motor, params))
            except LinkageKinematicsError:
                # Custom geometry which cannot be certified keeps the original
                # conversion/failure path; never silently reduce accuracy.
                motors.append(None)
            else:
                motors.append(PreparedMotorConversion(lo, hi, _smooth_limit_blend_deg(lo, hi, None), table))
        prepared = tuple(motors)
        _PREPARED_LINKAGE_CACHE[key] = prepared
    params._prepared_conversion = (key, prepared)
    return prepared


def linkage_fit_max_error_mm(
    motor: int,
    params: FingerLinkageParams,
    *,
    n_check: int = 200,
) -> Tuple[float, float, float]:
    """在表覆盖的 phi 内对比拟合 vs 精确二分，返回 (max, mean, max|phi|)。"""
    lut = build_phi_cf_lut(motor, params)
    phi0, phi1 = lut.phi[0], lut.phi[-1]
    # 避开软饱和上界附近（精确路径会 soft-clamp，拟合按几何硬表）
    span = phi1 - phi0
    lo = phi0 + 0.02 * span
    hi = phi1 - 0.08 * span
    if hi <= lo:
        lo, hi = phi0, phi1
    abs_err = 0.0
    sum_err = 0.0
    count = 0
    for i in range(n_check):
        phi = lo + (i / max(n_check - 1, 1)) * (hi - lo)
        exact = _phi_deg_to_cf_mm_bisection(phi, motor, params)
        approx = lut.eval(phi)
        e = abs(approx - exact)
        abs_err = max(abs_err, e)
        sum_err += e
        count += 1
    return abs_err, (sum_err / count if count else 0.0), max(abs(phi0), abs(phi1))


def _closed_loop_feasible(cf_mm: float, motor: int, params: FingerLinkageParams) -> bool:
    try:
        cf_to_phi_deg(cf_mm, motor, params)
        return True
    except LinkageKinematicsError:
        return False


def _find_geometric_cf_limit(motor: int, params: FingerLinkageParams) -> float:
    """对应 MATLAB findGeometricCfLimit：上界取 cfPhysicalMaxMm。"""
    upper = params.cf_physical_max_mm[motor]
    if upper <= 0.0:
        upper = (
            params.cf_max_mm[motor]
            if params.cf_max_mm[motor] > 0
            else params.s_a0_mm[motor]
        )
    if _closed_loop_feasible(upper, motor, params):
        return upper
    lo, hi = params.cf_min_mm[motor], upper
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        if _closed_loop_feasible(mid, motor, params):
            lo = mid
        else:
            hi = mid
    return lo


def joint_rad_to_phi_deg(rad: float) -> float:
    return rad * _RAD_TO_DEG


def joint_deg_to_phi_rad(deg: float) -> float:
    return deg * _DEG_TO_RAD


def motor_phi_deg_limits(
    motor: int, params: FingerLinkageParams,
) -> Tuple[float, float]:
    """单级电机闭环逆解允许的 phi 范围 (deg)。"""
    # Limits depend only on these per-motor values. Check their values rather
    # than object identity: callers may edit/finalize mutable parameter objects.
    key = (
        params.cf_min_mm[motor], params.cf_max_mm[motor],
        params.flexion_direction[motor], params.outer_radius_mm[motor],
        params.parent_radius_mm[motor], params.joint_gap_mm[motor],
        params.rod_length_mm[motor], params.s_a0_mm[motor],
        params.a_joint_offset_mm[motor],
    )
    cache = getattr(params, "_phi_limits_cache", None)
    if cache is None:
        cache = params._phi_limits_cache = {}
    cached = cache.get(motor)
    if cached is not None and cached[0] == key:
        return cached[1]
    cf_lo, cf_hi = params.motor_cf_limits(motor)
    phi_lo = cf_to_phi_deg(cf_lo, motor, params)
    phi_hi = cf_to_phi_deg(cf_hi, motor, params)
    if phi_lo > phi_hi:
        phi_lo, phi_hi = phi_hi, phi_lo
    limits = (phi_lo, phi_hi)
    cache[motor] = (key, limits)
    return limits


def _smooth_limit_blend_deg(lo: float, hi: float, blend_deg: Optional[float]) -> float:
    span = hi - lo
    if span <= 1e-6:
        return 0.0
    if blend_deg is None:
        blend_deg = min(5.0, span * 0.12)
    return max(0.5, min(blend_deg, span * 0.45))


def _smooth_saturate_upper(phi_deg: float, hi: float, blend: float) -> float:
    """上界软饱和：线性区外以指数渐近 hi，一阶导连续且趋近 0。"""
    if blend <= 0.0:
        return min(phi_deg, hi)
    edge = hi - blend
    if phi_deg <= edge:
        return phi_deg
    excess = phi_deg - edge
    return edge + blend * (1.0 - math.exp(-excess / blend))


def smooth_limit_phi_deg_to_motor(
    phi_deg: float,
    motor: int,
    params: FingerLinkageParams,
    *,
    blend_deg: Optional[float] = None,
) -> float:
    """将 phi 软限制到闭环可达范围；下界硬钳位（零位精确为 0），上界软饱和无尖点。"""
    if not math.isfinite(phi_deg):
        return 0.0
    lo, hi = motor_phi_deg_limits(motor, params)
    blend = _smooth_limit_blend_deg(lo, hi, blend_deg)
    phi = max(phi_deg, lo)
    return _smooth_saturate_upper(phi, hi, blend)


def clamp_phi_deg_to_motor(
    phi_deg: float, motor: int, params: FingerLinkageParams,
) -> float:
    return smooth_limit_phi_deg_to_motor(phi_deg, motor, params)


def clamp_joint_rad_for_motor(
    rad: float, motor: int, params: FingerLinkageParams,
) -> float:
    """将关节角 (rad) 软限制到该电机闭环逆解可达范围（限位附近平滑）。"""
    if not math.isfinite(rad):
        return 0.0
    phi = smooth_limit_phi_deg_to_motor(joint_rad_to_phi_deg(rad), motor, params)
    return joint_deg_to_phi_rad(phi)


@dataclass
class FingerMotorBinding:
    """单根手指：URDF 关节名 ↔ F1/F2/F3 电机下标。"""

    finger: str
    mcp_joint: str
    pip_joint: str
    dip_joint: str
    mcp_motor: int = MOTOR_F3
    pip_motor: int = MOTOR_F2
    dip_motor: int = MOTOR_F1
    stroke_sign: float = -1.0  # 下发 mm 符号；MATLAB CF+ 常对应硬件负行程


@dataclass
class FourFingerStrokeMapper:
    """四指：URDF MCP/PIP/DIP (rad) → [CF1, CF2, CF3] (mm)。"""

    params: FingerLinkageParams = field(default_factory=FingerLinkageParams.from_matlab_defaults)

    def joint_rad_to_cf_mm(
        self,
        mcp_rad: float,
        pip_rad: float,
        dip_rad: float,
    ) -> Tuple[float, float, float]:
        """相对关节角 → 三级电机 CF 行程 (mm)，顺序 [F1, F2, F3]。"""
        phi = (
            joint_rad_to_phi_deg(dip_rad),
            joint_rad_to_phi_deg(pip_rad),
            joint_rad_to_phi_deg(mcp_rad),
        )
        return self.phi_deg_to_cf_mm(*phi)

    def phi_deg_to_cf_mm(
        self,
        phi_f1_deg: float,
        phi_f2_deg: float,
        phi_f3_deg: float,
    ) -> Tuple[float, float, float]:
        return (
            phi_deg_to_cf_mm(phi_f1_deg, MOTOR_F1, self.params),
            phi_deg_to_cf_mm(phi_f2_deg, MOTOR_F2, self.params),
            phi_deg_to_cf_mm(phi_f3_deg, MOTOR_F3, self.params),
        )

    def cf_mm_to_joint_rad(
        self,
        cf1: float,
        cf2: float,
        cf3: float,
        *,
        clamp: bool = True,
    ) -> Tuple[float, float, float]:
        """正运动学：CF → MCP/PIP/DIP 相对角 (rad)，顺序 (mcp, pip, dip)。

        默认 ``clamp=True``（遥测）：CF 先钳到电机行程，避免零点噪声略负导致失败。
        精确几何校验可传 ``clamp=False``。
        """
        phi1 = cf_to_phi_deg(cf1, MOTOR_F1, self.params, clamp=clamp)
        phi2 = cf_to_phi_deg(cf2, MOTOR_F2, self.params, clamp=clamp)
        phi3 = cf_to_phi_deg(cf3, MOTOR_F3, self.params, clamp=clamp)
        return (
            joint_deg_to_phi_rad(phi3),
            joint_deg_to_phi_rad(phi2),
            joint_deg_to_phi_rad(phi1),
        )

    def stroke_mm_for_finger(
        self,
        binding: FingerMotorBinding,
        joint_angles: Dict[str, float],
    ) -> Dict[str, float]:
        """返回该指三个 URDF 关节对应的下发行程 mm（已乘 stroke_sign）。"""
        p = self.params
        raw_mcp = float(joint_angles.get(binding.mcp_joint, 0.0))
        raw_pip = float(joint_angles.get(binding.pip_joint, 0.0))
        raw_dip = float(joint_angles.get(binding.dip_joint, 0.0))
        try:
            if (_linkage_fit_enabled() and
                    (binding.mcp_motor, binding.pip_motor, binding.dip_motor) == (MOTOR_F3, MOTOR_F2, MOTOR_F1)):
                f1, f2, f3 = _get_prepared_motors(p)
                if f1 is not None and f2 is not None and f3 is not None:
                    return {
                        binding.mcp_joint: binding.stroke_sign * f3.joint_rad_to_cf_mm(raw_mcp),
                        binding.pip_joint: binding.stroke_sign * f2.joint_rad_to_cf_mm(raw_pip),
                        binding.dip_joint: binding.stroke_sign * f1.joint_rad_to_cf_mm(raw_dip),
                    }
            mcp = clamp_joint_rad_for_motor(raw_mcp, binding.mcp_motor, p)
            pip = clamp_joint_rad_for_motor(raw_pip, binding.pip_motor, p)
            dip = clamp_joint_rad_for_motor(raw_dip, binding.dip_motor, p)
            cf1, cf2, cf3 = self.joint_rad_to_cf_mm(mcp, pip, dip)
            cfs = (cf1, cf2, cf3)
            out: Dict[str, float] = {}
            for joint, motor in (
                (binding.mcp_joint, binding.mcp_motor),
                (binding.pip_joint, binding.pip_motor),
                (binding.dip_joint, binding.dip_motor),
            ):
                out[joint] = binding.stroke_sign * cfs[motor]
            return out
        except LinkageKinematicsError as exc:
            ctx = {
                "finger": binding.finger,
                "raw_rad": (raw_mcp, raw_pip, raw_dip),
                "stroke_sign": binding.stroke_sign,
                "cf_limits_mm": [p.motor_cf_limits(i) for i in range(MOTOR_COUNT)],
                "phi_limits_deg": [motor_phi_deg_limits(i, p) for i in range(MOTOR_COUNT)],
                "max_relative_angle_deg": p.max_relative_angle_deg,
            }
            if getattr(exc, "context", None):
                ctx["detail"] = exc.context
            raise LinkageKinematicsError(
                f"{binding.finger} 四指逆解失败: {exc}",
                context=ctx,
            ) from exc


class ThumbStrokeMapper:
    """拇指直线行程映射：仅 F3（MCP）+ F1（PIP），对应 thumb_gravity.m。"""

    def __init__(self) -> None:
        self.params: FingerLinkageParams = FingerLinkageParams.from_thumb_matlab_defaults()
        self.mcp_joint: str = "left_hand_thumb_MCP_joint"
        self.pip_joint: str = "left_hand_thumb_PIP_joint"
        self.mcp_motor: int = MOTOR_F3
        self.pip_motor: int = MOTOR_F1
        self.stroke_sign: float = -1.0
        self.use_linkage: bool = True
        self._linear_k: Dict[str, float] = {}

    def is_configured(self) -> bool:
        return self.use_linkage or bool(self._linear_k)

    def configure(
        self,
        mcp_joint: str = "left_hand_thumb_MCP_joint",
        pip_joint: str = "left_hand_thumb_PIP_joint",
        *,
        params: Optional[FingerLinkageParams] = None,
        stroke_sign: float = -1.0,
        use_linkage: bool = True,
        mcp_stroke_at_90_mm: float = 16.0,
        pip_stroke_at_90_mm: float = 14.0,
    ) -> None:
        """配置拇指映射。默认闭环逆解；use_linkage=False 时回退线性。"""
        self.mcp_joint = mcp_joint
        self.pip_joint = pip_joint
        self.stroke_sign = stroke_sign
        self.use_linkage = use_linkage
        if params is not None:
            self.params = finalize_linkage_params(params)
        else:
            self.params = FingerLinkageParams.from_thumb_matlab_defaults()
        if not use_linkage:
            ref = 0.5 * math.pi
            self._linear_k = {
                mcp_joint: stroke_sign * (mcp_stroke_at_90_mm / ref),
                pip_joint: stroke_sign * (pip_stroke_at_90_mm / ref),
            }
        else:
            self._linear_k = {}

    def joint_rad_to_cf_mm(self, mcp_rad: float, pip_rad: float) -> Tuple[float, float]:
        """拇指 MCP/PIP (rad) → [CF3, CF1] (mm)。"""
        cf3 = phi_deg_to_cf_mm(
            joint_rad_to_phi_deg(mcp_rad), MOTOR_F3, self.params
        )
        cf1 = phi_deg_to_cf_mm(
            joint_rad_to_phi_deg(pip_rad), MOTOR_F1, self.params
        )
        return cf3, cf1

    def stroke_mm_for_thumb(self, joint_angles: Dict[str, float]) -> Dict[str, float]:
        """URDF 拇指 MCP/PIP → 下发行程 mm（已乘 stroke_sign）。"""
        p = self.params
        raw_mcp = float(joint_angles.get(self.mcp_joint, 0.0))
        raw_pip = float(joint_angles.get(self.pip_joint, 0.0))
        try:
            if (self.use_linkage and _linkage_fit_enabled() and
                    (self.mcp_motor, self.pip_motor) == (MOTOR_F3, MOTOR_F1)):
                f1, _, f3 = _get_prepared_motors(p)
                if f1 is not None and f3 is not None:
                    return {
                        self.mcp_joint: self.stroke_sign * f3.joint_rad_to_cf_mm(raw_mcp),
                        self.pip_joint: self.stroke_sign * f1.joint_rad_to_cf_mm(raw_pip),
                    }
            mcp = clamp_joint_rad_for_motor(raw_mcp, self.mcp_motor, p)
            pip = clamp_joint_rad_for_motor(raw_pip, self.pip_motor, p)
            if not self.use_linkage:
                return {
                    self.mcp_joint: mcp * self._linear_k[self.mcp_joint],
                    self.pip_joint: pip * self._linear_k[self.pip_joint],
                }
            cf3, cf1 = self.joint_rad_to_cf_mm(mcp, pip)
            return {
                self.mcp_joint: self.stroke_sign * cf3,
                self.pip_joint: self.stroke_sign * cf1,
            }
        except LinkageKinematicsError as exc:
            ctx = {
                "finger": "thumb",
                "raw_rad": (raw_mcp, raw_pip),
                "stroke_sign": self.stroke_sign,
                "use_linkage": self.use_linkage,
                "cf_limits_mm": {
                    "F1": p.motor_cf_limits(MOTOR_F1),
                    "F3": p.motor_cf_limits(MOTOR_F3),
                },
                "phi_limits_deg": {
                    "F1": motor_phi_deg_limits(MOTOR_F1, p),
                    "F3": motor_phi_deg_limits(MOTOR_F3, p),
                },
            }
            if getattr(exc, "context", None):
                ctx["detail"] = exc.context
            raise LinkageKinematicsError(
                f"thumb 拇指逆解失败: {exc}",
                context=ctx,
            ) from exc

    def joint_rad_to_stroke_mm(self, joint_name: str, rad: float) -> Optional[float]:
        if not self.is_configured():
            return None
        partial = {self.mcp_joint: 0.0, self.pip_joint: 0.0}
        partial[joint_name] = rad
        return self.stroke_mm_for_thumb(partial).get(joint_name)


@dataclass
class HandStrokeMapper:
    """整手行程映射：四指闭环 + 拇指独立。"""

    four_finger: FourFingerStrokeMapper = field(
        default_factory=FourFingerStrokeMapper
    )
    thumb: ThumbStrokeMapper = field(default_factory=ThumbStrokeMapper)
    finger_bindings: Tuple[FingerMotorBinding, ...] = field(default_factory=tuple)
    use_linkage_for_four_fingers: bool = True
    # 每关节角度→行程后的缩放系数（对角“系数矩阵”），默认 1.0
    stroke_scale: Dict[str, float] = field(default_factory=dict)
    # 最近一次成功算出的下发行程；逆解失败时回填，避免跳到线性换算
    _last_linear_strokes: Dict[str, float] = field(default_factory=dict, repr=False)

    def managed_linear_joints(self) -> Tuple[str, ...]:
        """应由闭环（或拇指映射）覆盖的直线关节名。"""
        names: List[str] = []
        if self.use_linkage_for_four_fingers:
            for binding in self.finger_bindings:
                names.extend((binding.mcp_joint, binding.pip_joint, binding.dip_joint))
        if self.thumb.is_configured():
            names.extend((self.thumb.mcp_joint, self.thumb.pip_joint))
        return tuple(names)

    def ensure_stroke_scales(self) -> None:
        """为所有管理关节补齐默认系数 1.0。"""
        for name in self.managed_linear_joints():
            if name not in self.stroke_scale:
                self.stroke_scale[name] = 1.0

    def get_stroke_scales(self) -> Dict[str, float]:
        self.ensure_stroke_scales()
        return {k: float(self.stroke_scale.get(k, 1.0)) for k in self.managed_linear_joints()}

    def set_stroke_scale(self, joint_name: str, scale: float) -> None:
        """设置单个关节的行程缩放系数（非有限值忽略）。"""
        if not math.isfinite(scale):
            return
        self.stroke_scale[joint_name] = float(scale)

    def set_stroke_scales(self, scales: Dict[str, float]) -> None:
        """批量设置行程缩放系数。"""
        for name, scale in scales.items():
            self.set_stroke_scale(name, scale)

    def apply_stroke_scales(self, strokes: Dict[str, float]) -> Dict[str, float]:
        """对已算行程逐关节乘以系数：mm' = mm * scale。"""
        self.ensure_stroke_scales()
        out: Dict[str, float] = {}
        for name, mm in strokes.items():
            scale = float(self.stroke_scale.get(name, 1.0))
            if not math.isfinite(scale):
                scale = 1.0
            value = float(mm) * scale
            # An invalid multiplier/result must not enter the next command or
            # erase the previous valid stroke used by failure recovery.
            out[name] = value if math.isfinite(value) else self._last_linear_strokes.get(name, 0.0)
        return out

    def last_stroke_mm(self, joint_name: str) -> Optional[float]:
        val = self._last_linear_strokes.get(joint_name)
        return float(val) if val is not None else None

    def _hold_or_zero(self, joint_names: Sequence[str]) -> Dict[str, float]:
        out: Dict[str, float] = {}
        for name in joint_names:
            prev = self._last_linear_strokes.get(name)
            out[name] = float(prev) if prev is not None else 0.0
        return out

    def linear_strokes_mm(self, joint_angles: Dict[str, float]) -> Dict[str, float]:
        """URDF 关节角 → 各直线关节名下发行程 mm（超限角钳位到闭环可达范围）。

        单指逆解失败时保持该指上次有效行程，不抛异常、不省略关节键。
        新算出的行程再乘以各关节 stroke_scale；hold 值已含系数，不再重复乘。
        """
        if not isinstance(joint_angles, Mapping):
            raise ValueError("V2 pose must be a joint-name mapping")
        raw: Dict[str, float] = {}
        held: Dict[str, float] = {}
        if self.use_linkage_for_four_fingers:
            for binding in self.finger_bindings:
                joints = (binding.mcp_joint, binding.pip_joint, binding.dip_joint)
                try:
                    raw.update(
                        self.four_finger.stroke_mm_for_finger(binding, joint_angles)
                    )
                except LinkageKinematicsError as exc:
                    held_vals = self._hold_or_zero(joints)
                    held.update(held_vals)
                    scales = {j: float(self.stroke_scale.get(j, 1.0)) for j in joints}
                    _log_kin_failure(
                        binding.finger,
                        str(exc),
                        direction="ik",
                        raw_mcp=joint_angles.get(binding.mcp_joint, 0.0),
                        raw_pip=joint_angles.get(binding.pip_joint, 0.0),
                        raw_dip=joint_angles.get(binding.dip_joint, 0.0),
                        stroke_scales=scales,
                        held_mm=held_vals,
                        context=getattr(exc, "context", {}),
                    )
                except Exception as exc:
                    held_vals = self._hold_or_zero(joints)
                    held.update(held_vals)
                    _log_kin_failure(
                        binding.finger,
                        f"未预期异常 {type(exc).__name__}: {exc}",
                        direction="ik",
                        raw_mcp=joint_angles.get(binding.mcp_joint, 0.0),
                        raw_pip=joint_angles.get(binding.pip_joint, 0.0),
                        raw_dip=joint_angles.get(binding.dip_joint, 0.0),
                        held_mm=held_vals,
                    )
        if self.thumb.is_configured():
            joints = (self.thumb.mcp_joint, self.thumb.pip_joint)
            try:
                raw.update(self.thumb.stroke_mm_for_thumb(joint_angles))
            except LinkageKinematicsError as exc:
                held_vals = self._hold_or_zero(joints)
                held.update(held_vals)
                scales = {j: float(self.stroke_scale.get(j, 1.0)) for j in joints}
                _log_kin_failure(
                    "thumb",
                    str(exc),
                    direction="ik",
                    raw_mcp=joint_angles.get(self.thumb.mcp_joint, 0.0),
                    raw_pip=joint_angles.get(self.thumb.pip_joint, 0.0),
                    stroke_scales=scales,
                    held_mm=held_vals,
                    context=getattr(exc, "context", {}),
                )
            except Exception as exc:
                held_vals = self._hold_or_zero(joints)
                held.update(held_vals)
                _log_kin_failure(
                    "thumb",
                    f"未预期异常 {type(exc).__name__}: {exc}",
                    direction="ik",
                    raw_mcp=joint_angles.get(self.thumb.mcp_joint, 0.0),
                    raw_pip=joint_angles.get(self.thumb.pip_joint, 0.0),
                    held_mm=held_vals,
                )

        strokes = self.apply_stroke_scales(raw)
        strokes.update(held)

        # 只缓存有限值，避免污染下次 hold
        for name, mm in strokes.items():
            if math.isfinite(mm):
                self._last_linear_strokes[name] = float(mm)
        return strokes


# Form3 / 配置用：短名 → URDF 关节，按手指分组
STROKE_SCALE_JOINT_GROUPS: Tuple[Tuple[str, Tuple[Tuple[str, str], ...]], ...] = (
    (
        "thumb",
        (
            ("mcp", "left_hand_thumb_MCP_joint"),
            ("pip", "left_hand_thumb_PIP_joint"),
        ),
    ),
    (
        "index",
        (
            ("mcp", "left_hand_index_MCP_joint"),
            ("pip", "left_hand_index_PIP_joint"),
            ("dip", "left_hand_index_DIP_joint"),
        ),
    ),
    (
        "mid",
        (
            ("mcp", "left_hand_mid_MCP_joint"),
            ("pip", "left_hand_mid_PIP_joint"),
            ("dip", "left_hand_mid_DIP_joint"),
        ),
    ),
    (
        "ring",
        (
            ("mcp", "left_hand_ring_MCP_joint"),
            ("pip", "left_hand_ring_PIP_joint"),
            ("dip", "left_hand_ring_DIP_joint"),
        ),
    ),
    (
        "pinky",
        (
            ("mcp", "left_hand_pinky_MCP_joint"),
            ("pip", "left_hand_pinky_PIP_joint"),
            ("dip", "left_hand_pinky_DIP_joint"),
        ),
    ),
)

_STROKE_SCALE_FINGER_LABELS = {
    "thumb": "拇指",
    "index": "食指",
    "mid": "中指",
    "ring": "无名指",
    "pinky": "小指",
}

_STROKE_SCALE_PART_LABELS = {
    "mcp": "MCP",
    "pip": "PIP",
    "dip": "DIP",
}


def stroke_scale_ui_specs() -> List[Tuple[str, str, str]]:
    """返回 Form3 行程系数控件规格: (joint_name, finger_label, part_label)。"""
    specs: List[Tuple[str, str, str]] = []
    for finger, parts in STROKE_SCALE_JOINT_GROUPS:
        fl = _STROKE_SCALE_FINGER_LABELS.get(finger, finger)
        for part, joint in parts:
            pl = _STROKE_SCALE_PART_LABELS.get(part, part.upper())
            specs.append((joint, fl, pl))
    return specs


def default_finger_bindings() -> Tuple[FingerMotorBinding, ...]:
    """四指默认绑定（几何参数相同；后续可为每指单独 override params）。"""
    def b(finger: str) -> FingerMotorBinding:
        p = f"left_hand_{finger}"
        return FingerMotorBinding(
            finger=finger,
            mcp_joint=f"{p}_MCP_joint",
            pip_joint=f"{p}_PIP_joint",
            dip_joint=f"{p}_DIP_joint",
        )
    return tuple(b(f) for f in ("index", "mid", "ring", "pinky"))


def create_default_hand_stroke_mapper() -> HandStrokeMapper:
    mapper = HandStrokeMapper(finger_bindings=default_finger_bindings())
    mapper.thumb.configure()  # F3(MCP)+F1(PIP)，thumb_gravity 物理行程上限
    mapper.ensure_stroke_scales()
    # 预热 F1/F2/F3 与拇指参数的拟合表，避免首帧下发卡顿
    if _linkage_fit_enabled():
        try:
            _get_prepared_motors(mapper.four_finger.params)
            _get_prepared_motors(mapper.thumb.params)
        except LinkageKinematicsError:
            pass
    return mapper


def linear_slot_urdf_short(slot: int) -> str:
    """返回直线 USB 槽对应的短关节名。"""
    if not 0 <= int(slot) < len(LINEAR_SLOT_JOINTS):
        return "—"
    return LINEAR_SLOT_JOINTS[int(slot)].removeprefix("left_hand_").removesuffix("_joint")


def rotary_slot_urdf_short(slot: int) -> str:
    """返回旋转 USB 槽对应的短关节名。"""
    if not 0 <= int(slot) < len(ROTARY_SLOT_JOINTS):
        return "—"
    return ROTARY_SLOT_JOINTS[int(slot)].removeprefix("left_hand_").removesuffix("_joint")


# ── 动作播放用关节重映射（权威实现：protocol.hand_angle_protocol）──
try:
    from protocol import (
        remap_joint_angles_for_downlink,
        remap_motion_cfg_file_to_ui,
        swing_hw_to_urdf,
        swing_urdf_to_hw,
        thumb_swing_hw_to_ui,
        thumb_swing_ui_to_hw,
    )
except ImportError:  # pragma: no cover - 独立脚本无 src/protocol 时
    raise ImportError(
        "finger_linkage MotionPlaylist 需要 protocol 包；请从工程 src 运行"
    ) from None


_PLAYLIST_STROKE_MAPPER: Optional[HandStrokeMapper] = None


def joint_angles_to_hand_positions(
    joint_angles: Dict[str, float],
) -> Tuple[List[float], List[float]]:
    """URDF/硬件关节角 (rad) → 整手批量 (14×mm + 7×deg)。"""
    global _PLAYLIST_STROKE_MAPPER
    if _PLAYLIST_STROKE_MAPPER is None:
        _PLAYLIST_STROKE_MAPPER = create_default_hand_stroke_mapper()
    strokes = _PLAYLIST_STROKE_MAPPER.linear_strokes_mm(joint_angles)
    linear = [float(strokes.get(name, 0.0)) for name in LINEAR_SLOT_JOINTS]
    rotary = [
        math.degrees(float(joint_angles.get(name, 0.0)))
        for name in ROTARY_SLOT_JOINTS
    ]
    return linear, rotary


@dataclass
class MotionKeyframe:
    """动作文件中的单个姿态（UI/内存角空间，单位 rad）。

    由 load_motion_file 从文件硬件空间经 remap_motion_cfg_file_to_ui 转换而来。
    """

    name: str
    joint_angles: Dict[str, float]
    transition_s: float = 0.5
    interval_s: float = 0.1


class MotionPlaylist:
    """将动作 JSON 转为 14 路直线行程 + 7 路旋转角的连续时间播放列表。

    与 Form2 权威路径一致：
      UI 关键帧 → remap_joint_angles_for_downlink(hand_side)
               → joint_angles_to_hand_positions
    """

    def __init__(
        self,
        keyframes: Sequence[MotionKeyframe],
        *,
        hand_side: str = "left",
        loop: bool = False,
    ) -> None:
        self.keyframes = list(keyframes)
        self.hand_side = "right" if str(hand_side).lower() == "right" else "left"
        self.loop = bool(loop)
        self._timeline: List[Tuple[float, float, float, MotionKeyframe]] = []
        self.duration_s = 0.0
        self._build_timeline()

    def _build_timeline(self) -> None:
        self._timeline.clear()
        cursor = 0.0
        for keyframe in self.keyframes:
            transition = max(0.0, float(keyframe.transition_s))
            interval = max(0.0, float(keyframe.interval_s))
            self._timeline.append(
                (cursor, cursor + transition, cursor + transition + interval, keyframe)
            )
            cursor += transition + interval
        self.duration_s = cursor

    def _positions(self, cfg: Dict[str, float]) -> Tuple[List[float], List[float]]:
        remapped = remap_joint_angles_for_downlink(cfg, hand_side=self.hand_side)
        return joint_angles_to_hand_positions(remapped)

    @staticmethod
    def _blend(
        start: Sequence[float], end: Sequence[float], alpha: float
    ) -> List[float]:
        # 三次 smoothstep：端点速度为 0，降低电机冲击。
        x = min(1.0, max(0.0, float(alpha)))
        s = x * x * (3.0 - 2.0 * x)
        return [float(a) + (float(b) - float(a)) * s for a, b in zip(start, end)]

    def hand_positions_at(self, elapsed_s: float) -> Tuple[List[float], List[float]]:
        if not self.keyframes:
            return [0.0] * len(LINEAR_SLOT_JOINTS), [0.0] * len(ROTARY_SLOT_JOINTS)
        t = max(0.0, float(elapsed_s))
        if self.loop and self.duration_s > 0.0:
            t %= self.duration_s
        elif t >= self.duration_s:
            return self._positions(self.keyframes[-1].joint_angles)

        prev_linear, prev_rotary = self._positions({})
        for start_t, transition_end, hold_end, keyframe in self._timeline:
            target_linear, target_rotary = self._positions(keyframe.joint_angles)
            if t <= transition_end:
                duration = transition_end - start_t
                alpha = 1.0 if duration <= 0.0 else (t - start_t) / duration
                return (
                    self._blend(prev_linear, target_linear, alpha),
                    self._blend(prev_rotary, target_rotary, alpha),
                )
            if t <= hold_end:
                return target_linear, target_rotary
            prev_linear, prev_rotary = target_linear, target_rotary
        return prev_linear, prev_rotary

    def segment_label_at(self, elapsed_s: float) -> str:
        """供 main_app_v3 状态栏显示当前过渡/保持段标签。"""
        if not self.keyframes:
            return "—"
        t = max(0.0, float(elapsed_s))
        if self.loop and self.duration_s > 0.0:
            t %= self.duration_s
        elif self.duration_s > 0.0 and t >= self.duration_s:
            return self.keyframes[-1].name

        prev_name = "—"
        for start_t, transition_end, hold_end, keyframe in self._timeline:
            if t <= transition_end:
                if transition_end <= start_t:
                    return keyframe.name
                return f"{prev_name}→{keyframe.name}"
            if t <= hold_end:
                return keyframe.name
            prev_name = keyframe.name
        return self.keyframes[-1].name


def load_motion_file(path: str) -> MotionPlaylist:
    """读取 JSON 内容的 .txt/.json 动作文件并建立播放列表。

    文件角为硬件约定；关键帧存 UI 空间（与 Form2 一致）。
    """
    with open(os.fspath(path), "r", encoding="utf-8-sig") as stream:
        data = json.load(stream)
    if not isinstance(data, dict):
        raise ValueError("动作文件根节点必须是对象")
    hand_side = str(data.get("_hand_side", data.get("hand_side", "left"))).lower()
    keyframes: List[MotionKeyframe] = []
    for name, value in data.items():
        if name in {"_hand_side", "hand_side", "_meta"}:
            continue
        if not isinstance(value, dict):
            continue
        angles: Dict[str, float] = {}
        for joint, raw in value.items():
            if not isinstance(joint, str) or not joint.endswith("_joint"):
                continue
            try:
                val = float(raw)
            except (TypeError, ValueError):
                continue
            if math.isfinite(val):
                angles[joint] = val
        if not angles:
            continue
        keyframes.append(
            MotionKeyframe(
                name=str(name),
                joint_angles=remap_motion_cfg_file_to_ui(angles, hand_side=hand_side),
                transition_s=float(value.get("_transition_s", 0.5)),
                interval_s=float(value.get("_interval_s", 0.1)),
            )
        )
    if not keyframes:
        raise ValueError("动作文件中没有有效姿态")
    return MotionPlaylist(keyframes, hand_side=hand_side)
