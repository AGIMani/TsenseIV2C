from online_control.online_motion_track import package


def test_web_bundle_contains_manus_modules_and_only_required_build_inputs():
    entries = dict((relative, path) for path, relative in package.files())
    for name in ('manus_input.py', 'manus_source.py', 'metric_input.py'):
        path = entries['app/online_control/online_motion_track/' + name]
        compile(path.read_bytes(), str(path), 'exec')
    for name in ('requirements-manus.txt', 'scripts/build_manus.sh', 'tools/install_manus_runtime.py'):
        assert entries['app/' + name].is_file()
    vendor = {key: value for key, value in entries.items() if '/vendor/manussdk/' in key}
    assert len(vendor) == 12
    assert all(path.is_file() for path in vendor.values())
    assert not any(part in key for key in vendor for part in ('/build/', '/runtime/linux-', '/calibration/', '/examples/', '/docker/'))
    assert 'app/online_control/online_motion_track/static/manus-receiver.js' in entries
