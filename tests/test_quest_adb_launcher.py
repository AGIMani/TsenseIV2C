"""USB setup is exercised with a fake adb, never real devices or services."""
import json
import importlib.util
import os
from pathlib import Path
import shutil
import subprocess
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('prepare_quest_adb', ROOT / 'tools/prepare_quest_adb.py')
prepare_quest_adb = importlib.util.module_from_spec(spec)
spec.loader.exec_module(prepare_quest_adb)


@pytest.fixture
def fake_adb(monkeypatch):
    calls = []
    output = {
        ('devices', '-l'): 'List of devices attached\nQUEST123 device usb:1-2 product:hollywood model:Quest_3\n',
        ('-s', 'QUEST123', 'reverse', '--list'): '',
    }
    monkeypatch.setattr(prepare_quest_adb.shutil, 'which', lambda _: '/fake/adb')
    def run(args, **kwargs):
        assert kwargs['timeout'] == 15
        calls.append(args)
        return subprocess.CompletedProcess(args, 0, output.get(tuple(args[1:]), ''), '')
    monkeypatch.setattr(prepare_quest_adb.subprocess, 'run', run)
    return calls, output


def test_usb_only_selection_and_reverse_direction(fake_adb):
    calls, output = fake_adb
    output[('devices', '-l')] += '192.0.2.2:5555 device product:hollywood\nemulator-5554 device\n'
    assert prepare_quest_adb.prepare(8766) == 'QUEST123'
    assert calls[-1] == ['adb', '-s', 'QUEST123', 'reverse', '--no-rebind', 'tcp:8766', 'tcp:8766']


def test_existing_mapping_is_reused_but_unrelated_one_is_not_overwritten(fake_adb):
    calls, output = fake_adb
    output[('-s', 'QUEST123', 'reverse', '--list')] = 'UsbFfs tcp:8766 tcp:8766\n'
    prepare_quest_adb.prepare(8766)
    assert len(calls) == 2
    output[('-s', 'QUEST123', 'reverse', '--list')] = 'UsbFfs tcp:8766 tcp:9999\n'
    with pytest.raises(ValueError, match='已转发'):
        prepare_quest_adb.prepare(8766)
    assert len(calls) == 4


@pytest.mark.parametrize('listing, error', [
    ('', '未找到'),
    ('QUEST123 unauthorized usb:1-2\n', 'unauthorized'),
    ('QUEST123 offline usb:1-2\n', 'offline'),
    ('QUEST123 device usb:1-2\nPHONE device usb:1-3\n', '多个'),
    ('192.0.2.2:5555 device product:hollywood\n', '未找到'),
])
def test_unavailable_or_ambiguous_device_never_changes_reverse(fake_adb, listing, error):
    calls, output = fake_adb
    output[('devices', '-l')] = listing
    with pytest.raises(ValueError, match=error):
        prepare_quest_adb.prepare(8766)
    assert len(calls) == 1


def test_explicit_serial_selects_one_usb_device(fake_adb):
    calls, output = fake_adb
    output[('devices', '-l')] += 'PHONE device usb:1-3\n'
    assert prepare_quest_adb.prepare(9000, 'QUEST123') == 'QUEST123'
    assert calls[-1][-2:] == ['tcp:9000', 'tcp:9000']
    with pytest.raises(ValueError, match='未找到'):
        prepare_quest_adb.prepare(8766, 'missing')


def test_missing_adb_is_actionable(monkeypatch):
    monkeypatch.setattr(prepare_quest_adb.shutil, 'which', lambda _: None)
    with pytest.raises(ValueError, match='sudo apt install adb'):
        prepare_quest_adb.prepare(8766)


def test_adb_errors_and_timeouts_are_reported(monkeypatch):
    monkeypatch.setattr(prepare_quest_adb.subprocess, 'run', lambda *a, **k:
                        subprocess.CompletedProcess(a, 1, '', 'device disconnected'))
    with pytest.raises(ValueError, match='device disconnected'):
        prepare_quest_adb.adb('devices')
    def timeout(*a, **k):
        raise subprocess.TimeoutExpired('adb', 15)
    monkeypatch.setattr(prepare_quest_adb.subprocess, 'run', timeout)
    with pytest.raises(ValueError, match='超时'):
        prepare_quest_adb.adb('devices')


@pytest.fixture
def shell_launcher(tmp_path):
    root = tmp_path / 'project with spaces'
    scripts = root / 'migration/ubuntu'
    scripts.mkdir(parents=True)
    for name in ('common.sh', 'start-track.sh', 'start-quest-adb.sh'):
        shutil.copy2(ROOT / 'migration/ubuntu' / name, scripts / name)
    python = root / '.venvs/track/bin/python'
    python.parent.mkdir(parents=True)
    python.symlink_to(sys.executable)
    tools = root / 'tools'
    tools.mkdir()
    (tools / 'prepare_quest_adb.py').write_text("from pathlib import Path\nPath('adb-prepared').touch()\n")
    (tools / 'restart_local_service.py').write_text("import json, sys\nprint(json.dumps(sys.argv[1:]))\n")
    def run(args=(), **env):
        clean = {k:v for k,v in os.environ.items() if not k.startswith(('HAND_', 'WUJI_', 'LEAPSDK_'))}
        result = subprocess.run(['bash', str(scripts / 'start-quest-adb.sh'), *args], cwd=tmp_path,
                                env={**clean, **env}, text=True, capture_output=True)
        return root, result
    return run


@pytest.mark.parametrize('args', [(), ('--check',), ('--help',)])
def test_launcher_uses_isolated_track_loopback_http_without_tls(shell_launcher, args):
    root, result = shell_launcher(args, HAND_QUEST_ADB_PORT='9001')
    assert result.returncode == 0, result.stderr
    argv = json.loads(result.stdout)
    assert argv[0] == 'track'
    assert '.venvs/track/bin/python' in argv[1]
    assert argv[argv.index('--host') + 1] == '127.0.0.1'
    assert argv[-(len(args) + 2):][:2] == ['--port', '9001']
    assert '--tls-cert' not in argv and '--public' not in argv
    assert (root / 'adb-prepared').exists() == (not args)


@pytest.mark.parametrize('args, port', [(('--tls-cert', 'x'), '8766'), (('--host=0.0.0.0',), '8766'),
                                       (('--public',), '8766'), (('--port', '1234'), '8766'),
                                       ((), '0'), ((), '65536'), ((), 'x')])
def test_launcher_rejects_conflicting_network_settings_before_adb(shell_launcher, args, port):
    root, result = shell_launcher(args, HAND_QUEST_ADB_PORT=port)
    assert result.returncode == 2
    assert not (root / 'adb-prepared').exists()
