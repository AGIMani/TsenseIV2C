"""加速器与原 MATLAB/Python、查表误差界和批处理的交叉验证。"""

import copy
import json
from pathlib import Path
import unittest
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

import numpy as np

from a32_kinematics.a32_angles_to_inputs import (
    A32Error, a32_angles_to_inputs, a32_loop_inverse,
    load_configuration, load_kinematic_model,
)
from a32_kinematics.a32_fast import A32Converter, clear_default_cache, fast_angles_to_inputs


class FastConversionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model, cls.config = load_kinematic_model(), load_configuration()
        cls.converter = A32Converter(cls.model, cls.config)
        cls.exact = cls.converter.prepare()
        cls.lut = cls.converter.prepare(method="lut")
        cls.reference = json.loads((Path(__file__).parent / "fixtures" /
                                    "a32_angle_reference.json").read_text(encoding="utf-8"))

    def test_original_matlab_reference(self):
        for case in self.reference["cases"]:
            k, degrees, expected = case["index"], case["angleDeg"], case["inputSI"]
            for method, tolerance in (("exact", 1e-12), ("lut", 1e-9)):
                with self.subTest(index=k, degrees=degrees, method=method):
                    f = self.converter.prepare(k, method=method)
                    self.assertLess(abs(f.scalar(degrees) - expected), tolerance)
                    np.testing.assert_allclose(f(degrees)[0], f.scalar(degrees), atol=1e-15, rtol=0)
        angles = self.reference["wholeHandAngles"]
        np.testing.assert_allclose(self.exact(angles), self.reference["wholeHandInputs"], atol=1e-12, rtol=0)

    def test_four_finger_dip_stroke_boundary(self):
        for k in (8, 12, 16, 20):
            for method in ("exact", "lut"):
                with self.subTest(index=k, method=method):
                    f = self.converter.prepare(k, method=method)
                    lo, hi = f.limits_deg[0]
                    self.assertAlmostEqual(f.scalar(lo), 0., places=12)
                    self.assertAlmostEqual(f.scalar(hi), self.config["motors"][k - 1]["strokeRange"][1], places=12)
                    with self.assertRaisesRegex(A32Error, "AngleLimit"):
                        f.scalar(hi + .001)

    def test_random_against_original_vector_geometry(self):
        rng = np.random.default_rng(3201)
        for i, p in enumerate(self.config["motors"]):
            if p["type"] != "linear":
                continue
            f = self.converter.prepare(i + 1)
            angles = rng.uniform(*f.limits_deg[0], size=200)
            expected = [a32_loop_inverse(p, self.model.axes[i], np.deg2rad(q)) for q in angles]
            np.testing.assert_allclose(f.batch(angles[:, None])[:, 0], expected, atol=1e-12, rtol=0)

    def test_lookup_bound_all_cell_midpoints_and_quarters(self):
        # 覆盖每个表格单元内的点，而不是只测试本来就精确的表格节点。
        for k in range(1, 21):
            lut = self.converter.prepare(k, method="lut")
            exact = self.converter.prepare(k)
            count = lut.lookup_samples[0]
            if not count:
                continue
            lo, hi = lut.limits_deg[0]
            self.assertLessEqual(lut.lookup_error_bound_m[0], 1e-9)
            for fraction in (.25, .5, .75):
                degrees = lo + (np.arange(count - 1) + fraction) * (hi - lo) / (count - 1)
                error = np.max(np.abs(lut.batch(degrees[:, None]) - exact.batch(degrees[:, None])))
                self.assertLessEqual(error, lut.lookup_error_bound_m[0] + 1e-14)
            endpoints = np.array([lo, hi])[:, None]
            np.testing.assert_allclose(lut.batch(endpoints), exact.batch(endpoints), atol=1e-14, rtol=0)

    def test_batch_subset_order_empty_and_scalar_equivalence(self):
        selected = [7, 1, 20, 6]
        rng = np.random.default_rng(7)
        for method in ("exact", "lut"):
            f = self.converter.prepare(selected, method=method)
            lo, hi = f.limits_deg.T
            angles = rng.uniform(lo, hi, size=(129, 4))
            output = f.batch(angles)
            np.testing.assert_allclose(output, np.array([f(row) for row in angles]), atol=1e-14, rtol=0)
            self.assertEqual(f.batch(np.empty((0, 4))).shape, (0, 4))
            np.testing.assert_array_equal(f(angles[:1]), f(angles[0]))
            self.assertEqual(f.indices, tuple(selected))
            np.testing.assert_allclose(output[:, 1], np.deg2rad(angles[:, 1]), atol=1e-15, rtol=0)

    def test_reject_invalid_values_indices_and_methods(self):
        for method in ("exact", "lut"):
            f = self.converter.prepare(7, method=method)
            for bad in (-1, 95, np.nan, np.inf, 45 + 1j, "45", True, None):
                with self.subTest(bad=bad, method=method), self.assertRaises(A32Error):
                    f.scalar(bad)
            for bad in ([45, -1], [[45], [np.nan]], [[95]], [[1, 2]], [[True]]):
                with self.assertRaises(A32Error):
                    f.batch(bad)
            with self.assertRaises(A32Error):
                self.converter.prepare(5, method=method).scalar(89)
            with self.assertRaises(A32Error):
                self.converter.prepare(13, method=method).scalar(89)
        for indices in ([7, 7], 0, 21, 1.5, True, np.nan):
            with self.assertRaises(A32Error):
                self.converter.prepare(indices)
        with self.assertRaises(A32Error):
            self.converter.prepare(method="unknown")
        with self.assertRaises(A32Error):
            self.exact.scalar(45)
        with self.assertRaises(A32Error):
            self.exact(np.zeros(19))
        for tolerance in (True, -1, 0, 1e-15, np.nan, np.inf):
            with self.assertRaises(A32Error):
                A32Converter(lut_tolerance_m=tolerance)

    def test_snapshot_and_custom_child_slider(self):
        m, c = copy.deepcopy(self.model), copy.deepcopy(self.config)
        c["motors"][6]["sliderOn"] = "child"
        m.axes[6] *= -1
        converter = A32Converter(m, c)
        f = converter.prepare(7)
        expected = a32_angles_to_inputs(m, c, 45, 7)[0][0]
        self.assertAlmostEqual(f.scalar(45), expected, places=12)
        # 外部配置被修改后已创建转换器仍是同一个快照。
        c["motors"][6]["L"] *= 2
        m.axes[6] *= -1
        self.assertAlmostEqual(f.scalar(45), expected, places=12)
        with self.assertRaises(A32Error):
            A32Converter(m, c)

    def test_narrowed_stroke_range_and_atomic_failure(self):
        c = copy.deepcopy(self.config)
        c["motors"][6]["strokeRange"] = [.002, .01]
        fast = A32Converter(self.model, c).prepare(7)
        expected = a32_angles_to_inputs(self.model, c, 45, 7)[0][0]
        self.assertAlmostEqual(fast.scalar(45), expected, places=12)
        angles = np.zeros((2, 20))
        angles[1, 18] = -1
        before = angles.copy()
        with self.assertRaises(A32Error):
            self.lut.batch(angles)
        np.testing.assert_array_equal(angles, before)

    def test_default_cached_convenience(self):
        try:
            for method in ("exact", "lut"):
                output = fast_angles_to_inputs([45, 30], [7, 11], method=method)
                expected = self.converter.prepare([7, 11], method=method)([45, 30])
                np.testing.assert_array_equal(output, expected)
        finally:
            clear_default_cache()


if __name__ == "__main__":
    unittest.main()
