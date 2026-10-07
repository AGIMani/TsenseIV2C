"""Python runtime geometry matches the archived MATLAB reference."""
from pathlib import Path
import sys
import json
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from a32_kinematics.geometry_constants import configuration, DIP_STROKE_OVERRIDES_M
from a32_kinematics.a32_angles_to_inputs import MODEL_DIR, load_configuration, load_kinematic_model
from a32_kinematics.a32_fast import A32Converter


def test_archived_mat_matches_original_constants():
    from scipy.io import loadmat
    archive = loadmat(MODEL_DIR / 'geometry_inferred.mat', simplify_cells=True)['C']
    original = configuration(original=True)
    assert archive['sourceRobotName'] == original['sourceRobotName']
    for a, p in zip(archive['motors'], original['motors'], strict=True):
        for key, value in p.items():
            if isinstance(value, str):
                assert a[key] == value
            else:
                np.testing.assert_array_equal(a[key], value)


def test_original_full_domain_matches_matlab_reference():
    converter = A32Converter(config=configuration(original=True))
    cases = json.loads((Path(__file__).parent / 'fixtures/a32_angle_reference.json').read_text())['cases']
    for case in cases:
        for method, tolerance in [('exact', 1e-12), ('lut', 1e-9)]:
            actual = converter.prepare(case['index'], method=method).scalar(case['angleDeg'])
            assert abs(actual - case['inputSI']) < tolerance


def test_runtime_does_not_read_mat_and_config_is_independent(monkeypatch):
    import scipy.io
    def forbidden(*args, **kwargs):
        raise AssertionError('runtime must not read MAT')
    monkeypatch.setattr(scipy.io, 'loadmat', forbidden)
    first = load_configuration()
    first['motors'][0]['type'] = 'changed'
    current = load_configuration()
    assert current['motors'][0]['type'] == 'rotary'
    assert not DIP_STROKE_OVERRIDES_M
    assert current == configuration(original=True)
    from a32_driver import A32Device
    device = A32Device()
    poses = [{n: limits[i] for n, limits in device.joint_limits().items()} for i in (0, 1)]
    for pose in poses:
        device.mapper.convert(pose)
    assert device._backend is None
