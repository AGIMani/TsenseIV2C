"""Verify analytic residual Jacobians vs FD + timing (hand_dex / Pinocchio)."""
from __future__ import annotations

import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "src"))

from config_path import HAND_MODEL_DIR  # noqa: E402
from TsenseI_retargeting import TopoRetargetConfig, TopoRetargeter  # noqa: E402
from TsenseI_retargeting import toporetarget as tr  # noqa: E402


def make_points(seed: int = 0) -> np.ndarray:
    rng = np.random.default_rng(seed)
    pts = np.zeros((21, 3), dtype=float)
    pts[0] = [0.0, 0.0, 0.0]
    for i, x in enumerate([0.02, 0.04, 0.06, 0.08]):
        pts[1 + i] = [x * 0.35, x, 0.012]
    for f, base in enumerate([5, 9, 13, 17]):
        for j in range(4):
            pts[base + j] = [(f - 1.5) * 0.022, 0.028 + 0.018 * j, 0.002 * j]
    # slight curl
    pts[8, 1] -= 0.01
    pts[12, 1] -= 0.015
    pts *= 0.085 / max(float(np.linalg.norm(pts[5] - pts[17])), 1e-6)
    return pts + rng.normal(0.0, 0.0003, pts.shape)


def rel_err(a: np.ndarray, b: np.ndarray) -> float:
    num = float(np.linalg.norm(a - b))
    den = max(float(np.linalg.norm(b)), 1e-12)
    return num / den


def fd_weighted_jacobian(retargeter, q, q_init, q_prev, source_laplacian, bones, graph, eps=1e-6):
    residual = retargeter._weighted_residual(q, q_init, q_prev, source_laplacian, bones, graph)
    jac = np.zeros((residual.shape[0], q.shape[0]), dtype=float)
    for col in range(q.shape[0]):
        step = eps
        if q[col] + step > retargeter.upper[col]:
            step = -eps
        q2 = q.copy()
        q2[col] += step
        r2 = retargeter._weighted_residual(q2, q_init, q_prev, source_laplacian, bones, graph)
        jac[:, col] = (r2 - residual) / step
    return residual, jac


def fd_init_jacobian(retargeter, q, q_prior, bones, eps=1e-6):
    residual = retargeter._initialization_residual(q, q_prior, bones)
    jac = np.zeros((residual.shape[0], q.shape[0]), dtype=float)
    for col in range(q.shape[0]):
        step = eps
        if q[col] + step > retargeter.upper[col]:
            step = -eps
        q2 = q.copy()
        q2[col] += step
        r2 = retargeter._initialization_residual(q2, q_prior, bones)
        jac[:, col] = (r2 - residual) / step
    return residual, jac


def prepare(retargeter, pts):
    src = retargeter._source_points_in_robot_scale(tr.points21(pts))
    topo = tr._canonical_hand_points(src)
    bones = tr._relative_bone_terms(src)
    graph = tr._build_interaction_graph(
        topo, retargeter.config, distance_scale_m=retargeter.robot_palm_width_m
    )
    lap = tr._laplacian_coordinates(topo, graph)
    return bones, graph, lap


def test_jacobian_accuracy(retargeter: TopoRetargeter) -> None:
    assert retargeter._analytic_kinematics is not None, retargeter._analytic_kinematics_error
    print("\n=== Jacobian accuracy vs central/forward FD (true FK) ===")
    worst_topo = 0.0
    worst_init = 0.0
    for seed in range(8):
        pts = make_points(seed)
        bones, graph, lap = prepare(retargeter, pts)
        rng = np.random.default_rng(seed + 100)
        q = 0.15 * rng.standard_normal(len(retargeter.joint_names))
        q = np.clip(q, retargeter.lower + 0.02, retargeter.upper - 0.02)
        q_init = q + 0.05 * rng.standard_normal(q.shape[0])
        q_prev = q - 0.03 * rng.standard_normal(q.shape[0])

        r_ana, j_ana = retargeter._weighted_residual_and_jacobian(
            q, q_init, q_prev, lap, bones, graph, eps=1e-5
        )
        r_fd, j_fd = fd_weighted_jacobian(retargeter, q, q_init, q_prev, lap, bones, graph)
        assert r_ana is not None
        err_r = rel_err(r_ana, r_fd)
        err_j = rel_err(j_ana, j_fd)
        worst_topo = max(worst_topo, err_j)
        print(f"  topo seed={seed}: residual_rel={err_r:.3e}  jac_rel={err_j:.3e}")

        r_i, j_i = retargeter._initialization_residual_and_jacobian(q, q_prev, bones, eps=1e-5)
        r_ifd, j_ifd = fd_init_jacobian(retargeter, q, q_prev, bones)
        err_ir = rel_err(r_i, r_ifd)
        err_ij = rel_err(j_i, j_ifd)
        worst_init = max(worst_init, err_ij)
        print(f"  init seed={seed}: residual_rel={err_ir:.3e}  jac_rel={err_ij:.3e}")

    # Block-wise check on one pose
    pts = make_points(0)
    bones, graph, lap = prepare(retargeter, pts)
    q = np.zeros(len(retargeter.joint_names))
    q[0] = 0.2
    q[5] = 0.3
    q_init = np.zeros_like(q)
    q_prev = np.zeros_like(q)
    r_ana, j_ana = retargeter._weighted_residual_and_jacobian(
        q, q_init, q_prev, lap, bones, graph, eps=1e-5
    )
    _, j_fd = fd_weighted_jacobian(retargeter, q, q_init, q_prev, lap, bones, graph, eps=1e-7)
    n_im = 21
    n_bone = r_ana.shape[0] - n_im - 2 * q.shape[0]
    blocks = {
        "IM": (0, n_im),
        "bone": (n_im, n_im + n_bone),
        "q_init": (n_im + n_bone, n_im + n_bone + q.shape[0]),
        "q_reg": (n_im + n_bone + q.shape[0], r_ana.shape[0]),
    }
    print("\n  block-wise jac rel-err (seed0):")
    for name, (lo, hi) in blocks.items():
        e = rel_err(j_ana[lo:hi], j_fd[lo:hi])
        print(f"    {name:8s}  {e:.3e}  rows={hi-lo}")

    assert worst_topo < 5e-3, f"topo jac rel-err too large: {worst_topo}"
    assert worst_init < 5e-3, f"init jac rel-err too large: {worst_init}"
    print(f"\n  PASS  worst topo jac={worst_topo:.3e}  worst init jac={worst_init:.3e}")


def test_timing(retargeter: TopoRetargeter) -> None:
    print("\n=== Timing (analytic chain vs FD-on-residual) ===")
    pts = make_points(1)
    bones, graph, lap = prepare(retargeter, pts)
    q = np.clip(
        0.1 * np.random.default_rng(1).standard_normal(len(retargeter.joint_names)),
        retargeter.lower + 0.01,
        retargeter.upper - 0.01,
    )
    q0 = np.zeros_like(q)

    def mean_ms(fn, n=40, warmup=5):
        for _ in range(warmup):
            fn()
        xs = []
        for _ in range(n):
            t0 = time.perf_counter()
            fn()
            xs.append((time.perf_counter() - t0) * 1000.0)
        return float(np.mean(xs))

    t_ana = mean_ms(
        lambda: retargeter._weighted_residual_and_jacobian(q, q0, q0, lap, bones, graph, eps=1e-5)
    )
    # Old-style: pinocchio points + FD residual_from_points columns
    points, pj = retargeter._robot_points_and_jacobian_palm_m(q)

    def old_fd_style():
        residual = retargeter._weighted_residual_from_points(points, q, q0, q0, lap, bones, graph)
        jac = np.empty((residual.shape[0], q.shape[0]))
        eps = 1e-5
        for col in range(q.shape[0]):
            q_step = q.copy()
            q_step[col] += eps
            points_step = points + eps * pj[:, :, col]
            jac[:, col] = (
                retargeter._weighted_residual_from_points(
                    points_step, q_step, q0, q0, lap, bones, graph
                )
                - residual
            ) / eps
        return residual, jac

    t_old = mean_ms(old_fd_style, n=15)
    print(f"  residual+J analytic_chain: {t_ana:.3f} ms")
    print(f"  residual+J old FD-on-res:   {t_old:.3f} ms")
    print(f"  speedup:                   {t_old / t_ana:.1f}x")

    # End-to-end retarget
    r = retargeter
    r.reset()
    for i in range(3):
        r.retarget(make_points(i))
    xs = []
    for i in range(40):
        t0 = time.perf_counter()
        out = r.retarget(make_points(10 + i))
        xs.append((time.perf_counter() - t0) * 1000.0)
    arr = np.asarray(xs)
    d = out.diagnostics
    print(
        f"  end-to-end retarget: mean={arr.mean():.2f} p50={np.median(arr):.2f} "
        f"p95={np.percentile(arr,95):.2f} ms"
    )
    print(
        f"  diagnostics: residual_jacobian={d['parameters'].get('residual_jacobian')} "
        f"iters={d['init_num_iters']}/{d['topo_num_iters']} "
        f"evals={d['init_objective_evals']}/{d['topo_objective_evals']}"
    )


def test_retarget_sanity(retargeter: TopoRetargeter) -> None:
    print("\n=== Retarget sanity ===")
    r = retargeter
    r.reset()
    prev = None
    for i in range(5):
        out = r.retarget(make_points(i), previous_joint_values=prev)
        assert out.success or out.diagnostics.get("topo_objective_improvement") is not None
        assert len(out.joint_values) == len(r.joint_names)
        assert all(np.isfinite(v) for v in out.joint_values.values())
        prev = out.joint_values
        print(
            f"  frame {i}: success={out.success} loss_total={out.loss.get('total', float('nan')):.4f} "
            f"msg={out.diagnostics.get('message')}"
        )
    print("  PASS")


def main():
    import pinocchio as pin

    print(f"python={sys.executable}")
    print(f"pinocchio={pin.__version__}")
    print(f"HAND_MODEL_DIR={HAND_MODEL_DIR}")

    cfg = TopoRetargetConfig(
        robot_root=HAND_MODEL_DIR,
        init_max_iters=1,
        topo_max_iters=2,
        use_topo_optimization=True,
        use_analytic_jacobian=True,
    )
    retargeter = TopoRetargeter(cfg)
    print(
        f"analytic_kinematics={retargeter._analytic_kinematics is not None} "
        f"err={retargeter._analytic_kinematics_error!r}"
    )
    if retargeter._analytic_kinematics is None:
        raise SystemExit("Pinocchio analytic kinematics required for this test")

    test_jacobian_accuracy(retargeter)
    test_timing(retargeter)
    test_retarget_sanity(retargeter)
    print("\nALL TESTS PASSED")


if __name__ == "__main__":
    main()
