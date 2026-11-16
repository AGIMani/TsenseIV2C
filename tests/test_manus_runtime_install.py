import hashlib
import importlib.util
from pathlib import Path
import zipfile

import pytest

_spec = importlib.util.spec_from_file_location('manus_runtime_installer',
    Path(__file__).resolve().parents[1] / 'tools/install_manus_runtime.py')
_installer = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_installer)
check, install = _installer.check, _installer.install


def bundle(tmp_path, data=b'customer-library', expected=b'customer-library'):
    archive = tmp_path / 'customer.zip'
    name = 'runtime/linux-x86_64/libexample.so.1.0'
    link = 'runtime/linux-x86_64/libexample.so.1'
    target = b'libexample.so.1.0'
    with zipfile.ZipFile(archive, 'w') as output:
        output.writestr('source/' + name, data)
        output.writestr('source/' + link, target)
        output.writestr('source/requirements.txt', 'unrelated-dependency')
        output.writestr('../../escape', 'unrelated-content')
    manifest = {'files': {
        name: {'archive_path': 'source/' + name, 'sha256': hashlib.sha256(expected).hexdigest()},
        link: {'archive_path': 'source/' + link, 'sha256': hashlib.sha256(target).hexdigest(),
               'symlink': target.decode()},
    }}
    return archive, manifest


def test_only_verified_manus_runtime_is_restored(tmp_path):
    archive, manifest = bundle(tmp_path)
    destination = tmp_path / 'installed'
    install(archive, destination, manifest)
    check(destination, manifest)
    assert {str(path.relative_to(destination)) for path in destination.rglob('*') if path.is_file()} == set(manifest['files'])
    assert (destination / 'runtime/linux-x86_64/libexample.so.1').is_symlink()
    assert not (tmp_path / 'escape').exists()
    install(archive, destination, manifest)
    check(destination, manifest)


def test_wrong_archive_leaves_existing_runtime_unchanged(tmp_path):
    archive, manifest = bundle(tmp_path, data=b'wrong-version')
    destination = tmp_path / 'installed'
    existing = destination / next(iter(manifest['files']))
    existing.parent.mkdir(parents=True)
    existing.write_bytes(b'previous-install')
    with pytest.raises(ValueError, match='checksum mismatch'):
        install(archive, destination, manifest)
    assert existing.read_bytes() == b'previous-install'


def test_runtime_check_detects_replaced_library(tmp_path):
    archive, manifest = bundle(tmp_path)
    destination = tmp_path / 'installed'
    install(archive, destination, manifest)
    (destination / next(iter(manifest['files']))).write_bytes(b'corrupted')
    with pytest.raises(ValueError, match='checksum mismatch'):
        check(destination, manifest)
