"""闭环行程逆解：失败条件探测 + 禁止线性回退导致突变。"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from kinematics.finger_linkage import (
    FingerLinkageParams,
    LinkageKinematicsError,
    MOTOR_F1,
    MOTOR_F2,
    MOTOR_F3,
    cf_to_phi_deg,
    create_default_hand_stroke_mapper,
    motor_phi_deg_limits,
    phi_deg_to_cf_mm,
)
from usb.offline import (
    LINEAR_JOINT_SLOTS,
    configure_hand_stroke_mapper,
    joint_angles_to_hand_positions,
)


@pytest.fixture
def params() -> FingerLinkageParams:
    return FingerLinkageParams.from_matlab_defaults()


@pytest.fixture
def mapper():
    m = create_default_hand_stroke_mapper()
    configure_hand_stroke_mapper(m)
    yield m
    configure_hand_stroke_mapper(create_default_hand_stroke_mapper())


def _full_cfg(angle_rad: float) -> dict:
    cfg = {name: 0.0 for name in LINEAR_JOINT_SLOTS}
    for finger in ("index", "mid", "ring", "pinky"):
        for joint in ("MCP", "PIP", "DIP"):
            cfg[f"left_hand_{finger}_{joint}_joint"] = angle_rad
    cfg["left_hand_thumb_MCP_joint"] = angle_rad
    cfg["left_hand_thumb_PIP_joint"] = angle_rad
    return cfg


# ── 正运动学失败条件（CF → phi）────────────────────────────────


class TestCfToPhiFailures:
    """cf_to_phi_deg 在哪些 CF 下会抛 LinkageKinematicsError。"""

    @pytest.mark.parametrize("motor", [MOTOR_F1, MOTOR_F2, MOTOR_F3])
    def test_negative_cf_fails(self, params, motor):
        with pytest.raises(LinkageKinematicsError):
            cf_to_phi_deg(-0.5, motor, params)

    @pytest.mark.parametrize("motor", [MOTOR_F1, MOTOR_F2, MOTOR_F3])
    def test_far_above_limit_fails(self, params, motor):
        _lo, hi = params.motor_cf_limits(motor)
        with pytest.raises(LinkageKinematicsError):
            cf_to_phi_deg(hi + 40.0, motor, params)

    @pytest.mark.parametrize("motor", [MOTOR_F1, MOTOR_F2, MOTOR_F3])
    def test_cf_inside_limits_ok(self, params, motor):
        lo, hi = params.motor_cf_limits(motor)
        for cf in (lo, 0.5 * (lo + hi), hi):
            phi = cf_to_phi_deg(cf, motor, params)
            assert math.isfinite(phi)


# ── 逆运动学（phi → CF）边界 ───────────────────────────────────


class TestPhiToCfRobustness:
    """phi_deg_to_cf_mm 应钳位，超限/非有限输入不抛异常。"""

    @pytest.mark.parametrize("motor", [MOTOR_F1, MOTOR_F2, MOTOR_F3])
    def test_out_of_range_phi_clamped_not_raised(self, params, motor):
        lo, hi = motor_phi_deg_limits(motor, params)
        for phi in (lo - 30.0, hi + 30.0, 180.0, -90.0):
            cf = phi_deg_to_cf_mm(phi, motor, params)
            cf_lo, cf_hi = params.motor_cf_limits(motor)
            assert cf_lo - 1e-6 <= cf <= cf_hi + 1e-6

    @pytest.mark.parametrize("bad", [float("nan"), float("inf"), float("-inf")])
    @pytest.mark.parametrize("motor", [MOTOR_F1, MOTOR_F2, MOTOR_F3])
    def test_nonfinite_phi_raises(self, params, motor, bad):
        with pytest.raises(LinkageKinematicsError):
            phi_deg_to_cf_mm(bad, motor, params)


# ── 失败/缺失时保持上次行程，不突跳 ─────────────────────────────


class TestNoLinearFallbackOnFailure:
    def test_disabled_linkage_outputs_zero(self, mapper):
        """关闭闭环覆盖后，下发为 0（无线性回退）。"""
        ang = 1.2
        mapper.use_linkage_for_four_fingers = False
        mapper.thumb.use_linkage = False
        mapper.thumb._linear_k = {}
        lin, _ = joint_angles_to_hand_positions(_full_cfg(ang))
        name = "left_hand_index_MCP_joint"
        slot = LINEAR_JOINT_SLOTS[name]
        assert lin[slot] == 0.0

    def test_forced_ik_failure_holds_last_and_logs(self, mapper, capsys):
        from kinematics.finger_linkage import FourFingerStrokeMapper

        name = "left_hand_mid_PIP_joint"
        slot = LINEAR_JOINT_SLOTS[name]
        lin0, _ = joint_angles_to_hand_positions(_full_cfg(0.5))
        held = lin0[slot]
        orig = FourFingerStrokeMapper.stroke_mm_for_finger.__get__(
            mapper.four_finger, FourFingerStrokeMapper
        )

        def boom(binding, joint_angles):
            if binding.finger == "mid":
                raise LinkageKinematicsError(
                    "mid fail",
                    context={"motor": 2, "cf_lo": 0.0, "cf_hi": 12.2},
                )
            return orig(binding, joint_angles)

        mapper.four_finger.stroke_mm_for_finger = boom  # type: ignore[method-assign]
        lin1, _ = joint_angles_to_hand_positions(_full_cfg(1.0))
        assert lin1[slot] == pytest.approx(held, abs=1e-6)
        idx_slot = LINEAR_JOINT_SLOTS["left_hand_index_MCP_joint"]
        assert abs(lin1[idx_slot] - lin0[idx_slot]) > 0.1

    def test_nonfinite_joint_does_not_jump_to_max_stroke(self, mapper):
        name = "left_hand_index_MCP_joint"
        slot = LINEAR_JOINT_SLOTS[name]
        cfg = _full_cfg(0.4)
        cfg[name] = float("nan")
        lin1, _ = joint_angles_to_hand_positions(cfg)
        # NaN 应被当作 0 钳位，而不是冲到最大行程 (~-12mm)
        assert abs(lin1[slot]) < 1.0


class TestStrokeScale:
    def test_scale_scales_downlink_mm(self, mapper):
        from usb.offline import (
            LINEAR_JOINT_SLOTS,
            configure_hand_stroke_mapper,
            configure_stroke_scales,
            joint_angles_to_hand_positions,
        )

        configure_hand_stroke_mapper(mapper)
        name = "left_hand_index_MCP_joint"
        slot = LINEAR_JOINT_SLOTS[name]
        cfg = _full_cfg(0.6)
        lin0, _ = joint_angles_to_hand_positions(cfg)
        configure_stroke_scales({name: 0.5})
        lin1, _ = joint_angles_to_hand_positions(cfg)
        assert lin1[slot] == pytest.approx(0.5 * lin0[slot], rel=1e-5)

    def test_default_scales_are_one(self, mapper):
        scales = mapper.get_stroke_scales()
        assert scales
        assert all(abs(v - 1.0) < 1e-9 for v in scales.values())


class TestStrokeContinuity:
    def test_dense_ramp_no_sudden_jump(self, mapper):
        name = "left_hand_ring_MCP_joint"
        slot = LINEAR_JOINT_SLOTS[name]
        prev = None
        max_step = 0.0
        for i in range(0, 161):
            ang = (i / 160.0) * (math.pi / 2)
            lin, _ = joint_angles_to_hand_positions(_full_cfg(ang))
            cur = lin[slot]
            if prev is not None:
                max_step = max(max_step, abs(cur - prev))
            prev = cur
        # 约 1° 一步，闭环行程步进应远小于线性回退造成的数 mm 跳变
        assert max_step < 0.5

    def test_roundtrip_cf_within_limits(self, params):
        for motor in (MOTOR_F1, MOTOR_F2, MOTOR_F3):
            lo, hi = params.motor_cf_limits(motor)
            # 上界附近有软饱和，phi→CF 会略低于物理 cf_max，故不含 hi 端点
            for cf in (lo, 0.25 * hi, 0.5 * hi, 0.85 * hi):
                phi = cf_to_phi_deg(cf, motor, params)
                cf2 = phi_deg_to_cf_mm(phi, motor, params)
                assert abs(cf2 - cf) < 0.05


class TestLinkageFitLut:
    """拟合表相对精确二分的误差应很小（避开软饱和上界）。"""

    @pytest.mark.parametrize("motor", [MOTOR_F1, MOTOR_F2, MOTOR_F3])
    def test_fit_vs_exact_max_error(self, params, motor):
        from kinematics.finger_linkage import (
            clear_phi_cf_lut_cache,
            linkage_fit_max_error_mm,
            phi_deg_to_cf_mm as phi2cf,
        )

        clear_phi_cf_lut_cache()
        max_err, mean_err, _ = linkage_fit_max_error_mm(motor, params, n_check=120)
        assert max_err < 0.05, f"F{motor+1} max_err={max_err}"
        assert mean_err < 0.01, f"F{motor+1} mean_err={mean_err}"

        lo, hi = motor_phi_deg_limits(motor, params)
        # 中段对比拟合与精确路径
        for i in range(1, 10):
            phi = lo + (i / 10.0) * (hi - lo) * 0.85
            a = phi2cf(phi, motor, params, exact=False)
            b = phi2cf(phi, motor, params, exact=True)
            assert abs(a - b) < 0.05


@pytest.mark.parametrize("motor", [MOTOR_F1, MOTOR_F2, MOTOR_F3])
def test_cached_limits_follow_parameter_changes(params, motor):
    original = motor_phi_deg_limits(motor, params)
    values = list(params.cf_max_mm)
    values[motor] *= .75
    params.cf_max_mm = tuple(values)
    changed = motor_phi_deg_limits(motor, params)
    expected = sorted(cf_to_phi_deg(cf, motor, params) for cf in params.motor_cf_limits(motor))
    assert changed == tuple(expected)
    assert changed != original
    assert motor_phi_deg_limits(motor, params) == changed


def test_limit_cache_avoids_repeated_forward_solves(params, monkeypatch):
    import kinematics.finger_linkage as linkage
    expected = motor_phi_deg_limits(MOTOR_F3, params)
    def forbidden(*args, **kwargs):
        pytest.fail("Unchanged geometry should reuse angle limits")
    monkeypatch.setattr(linkage, "cf_to_phi_deg", forbidden)
    assert motor_phi_deg_limits(MOTOR_F3, params) == expected
