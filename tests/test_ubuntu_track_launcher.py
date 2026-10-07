"""Exercise native shell entrypoints without replacing services or opening hardware."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def launcher(tmp_path):
    root = tmp_path / 'project with spaces'
    scripts = root / 'migration/ubuntu'
    scripts.mkdir(parents=True)
    for name in ('common.sh', 'start-track.sh', 'start-quest.sh', 'start-wuji.sh'):
        shutil.copy2(ROOT / 'migration/ubuntu' / name, scripts / name)
    python = root / '.venvs/track/bin/python'
    python.parent.mkdir(parents=True)
    python.symlink_to(sys.executable)
    tools = root / 'tools'
    tools.mkdir()
    (tools / 'prepare_quest_https.py').write_text(
        "from pathlib import Path\nimport os\n"
        "Path('prepared').touch()\nprint(os.environ.get('TEST_LAN_ADDRESS', '192.168.10.2'))\n")
    (tools / 'restart_local_service.py').write_text(
        "import json, os, sys\n"
        "print(json.dumps({'args':sys.argv[1:], 'leap':os.environ['LEAPSDK_INSTALL_LOCATION']}))\n")

    def run(name='start-track.sh', args=(), **env):
        clean = {k:v for k,v in os.environ.items() if not k.startswith(('HAND_', 'WUJI_', 'LEAPSDK_'))}
        result = subprocess.run(['bash', str(scripts / name), *args], cwd=tmp_path,
                                env={**clean, **env}, capture_output=True, text=True, check=True)
        return root, json.loads(result.stdout)
    return run


@pytest.mark.parametrize('name', ['start-track.sh', 'start-quest.sh', 'start-wuji.sh'])
def test_all_entrypoints_share_lan_https_and_local_sdk(launcher, name):
    root, result = launcher(name, WUJI_DEVICE_ADDRESS='192.168.20.2:50001')
    args = result['args']
    assert args[0] == 'track'
    assert args[args.index('--host') + 1] == '0.0.0.0'
    assert args[args.index('--allowed-host') + 1] == '192.168.10.2'
    assert args[args.index('--tls-cert') + 1] == str(root / 'runtime/ubuntu/quest-tls/server.crt')
    assert args[args.index('--wuji-address') + 1] == '192.168.20.2:50001'
    assert '--public' not in args
    assert result['leap']


def test_no_lan_still_starts_local_https_for_other_inputs(launcher):
    _, result = launcher(TEST_LAN_ADDRESS='')
    args = result['args']
    assert args[args.index('--host') + 1] == '127.0.0.1'
    assert '--tls-cert' in args
    assert '--allowed-host' not in args


@pytest.mark.parametrize('args', [('--check',), ('--help',), ('--host', '127.0.0.1'),
                                 ('--public', '--allowed-host', 'example.test')])
def test_diagnostics_and_explicit_deployments_skip_auto_network_setup(launcher, args):
    root, result = launcher(args=args)
    assert not (root / 'prepared').exists()
    assert '--tls-cert' not in result['args']
    assert result['args'][-len(args):] == list(args)
