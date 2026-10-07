"""Portable backend must preserve the calibrated model and solver geometry."""
import sys
from pathlib import Path
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "src")]
from online_control.online_motion_track.main import WebSession


@pytest.mark.parametrize("side", ["left", "right"])
def test_analytic_numpy_jacobian_against_independent_fk_finite_differences(side):
    session = WebSession(side, recording=False)
    solver = session.pipeline.session.retargeter
    backend = solver._analytic_kinematics
    assert backend is not None
    rng = np.random.default_rng(926)
    for _ in range(3):
        q = rng.uniform(solver.lower, solver.upper)
        actual, jac = backend.compute_points_and_jacobian(q)
        def fk(values):
            return solver.fk.compute(dict(zip(solver.joint_names, values)))
        np.testing.assert_allclose(actual, fk(q), atol=1e-12)
        epsilon = 1e-6
        numerical = np.stack([(fk(q + delta) - fk(q - delta)) / (2 * epsilon)
                              for delta in np.eye(len(q)) * epsilon], axis=-1)
        np.testing.assert_allclose(jac, numerical, atol=1e-8, rtol=1e-5)
    session.close()


@pytest.mark.parametrize("side", ["left", "right"])
def test_numpy_matches_original_pinocchio(side):
    pytest.importorskip("pinocchio")
    from TsenseI_retargeting.toporetarget import _PinocchioKeypointKinematics
    session = WebSession(side, recording=False)
    solver = session.pipeline.session.retargeter
    other = _PinocchioKeypointKinematics(solver.urdf_path, solver.fk.keypoints,
                                         solver.joint_names, solver.fk.model)
    rng = np.random.default_rng(31)
    for _ in range(8):
        q = rng.uniform(solver.lower, solver.upper)
        a, aj = solver._analytic_kinematics.compute_points_and_jacobian(q)
        b, bj = other.compute_points_and_jacobian(q)
        np.testing.assert_allclose(a, b, atol=1e-12)
        np.testing.assert_allclose(aj, bj, atol=1e-12)
    session.close()


def test_batched_tree_preserves_prismatic_fixed_and_mimic_branches(tmp_path):
    from types import SimpleNamespace
    from TsenseI_retargeting.fk.urdf import UrdfModel
    from TsenseI_retargeting.fk.numpy_kinematics import NumpyKeypointKinematics
    path=tmp_path/'mixed.urdf'
    path.write_text('''<robot name="mixed">
      <link name="base"/><link name="rot"/><link name="fixed"/><link name="slide"/><link name="mimic"/>
      <joint name="r" type="revolute"><parent link="base"/><child link="rot"/>
        <origin xyz=".1 .2 .3" rpy=".1 .2 .3"/><axis xyz="0 0 1"/><limit lower="-2" upper="2"/></joint>
      <joint name="f" type="fixed"><parent link="rot"/><child link="fixed"/>
        <origin xyz=".3 .1 0" rpy=".2 0 0"/></joint>
      <joint name="s" type="prismatic"><parent link="base"/><child link="slide"/>
        <origin xyz="0 .4 0" rpy="0 .2 0"/><axis xyz="1 0 0"/><limit lower="-1" upper="1"/></joint>
      <joint name="m" type="revolute"><parent link="fixed"/><child link="mimic"/>
        <origin xyz="0 .2 .1"/><axis xyz="0 1 0"/><mimic joint="r" multiplier="-.7" offset=".13"/></joint>
    </robot>''',encoding='utf-8')
    model=UrdfModel(path)
    points=[SimpleNamespace(id=i,link=link,local_xyz=(.01,.02,.03))
            for i,link in enumerate(['fixed','slide','mimic'])]
    backend=NumpyKeypointKinematics(path,SimpleNamespace(keypoints=points),['r','s'],model)
    def reference(q):
        poses=model.forward_kinematics(dict(zip(['r','s'],q)))
        return np.array([(poses[p.link]@np.array([*p.local_xyz,1]))[:3] for p in points])
    for q in [np.array([.4,.2]),np.array([-.6,-.3])]:
        actual,jac=backend.compute_points_and_jacobian(q)
        np.testing.assert_allclose(actual,reference(q),atol=1e-12)
        eps=1e-6
        numeric=np.stack([(reference(q+d)-reference(q-d))/(2*eps) for d in np.eye(2)*eps],axis=-1)
        np.testing.assert_allclose(jac,numeric,atol=1e-9)
