"""Playback supports one model while retaining A3.2 SDK control and ownership."""
import sys
import time
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / 'src')]

from a32_session import A32Session
from server import Bridge

MODEL = 'TsenseI_v3_2_withid'


def test_default_bridge_exposes_withid_and_rejects_removed_models(tmp_path):
    bridge = Bridge(tmp_path)
    try:
        state = bridge.snapshot()
        assert state['model_id'] == MODEL
        assert state['side'] == 'left' and len(state['joints']) == 20
        assert bridge.dispatch('model', {'model_id': MODEL}) is True
        for model in ['TsenseI_v2', 'TsenseI_v3_0', 'TsenseI_v3_0_withid', 'TsenseI_v3_2']:
            with pytest.raises(ValueError, match='仅支持'):
                bridge.dispatch('model', {'model_id': model})
            assert bridge.snapshot()['model_id'] == MODEL
        assert not any(name in sys.modules for name in ('PySide6', 'PyQt6'))
    finally:
        bridge.close()


def test_side_switch_and_controller_timeout_without_hardware(tmp_path):
    session = A32Session(tmp_path, model_id=MODEL, start=False)
    bridge = Bridge(tmp_path, session=session)
    try:
        owner = 'test-client-123456789'
        bridge.dispatch('control', {}, owner)
        old_id = bridge.a32.generation
        bridge.dispatch('widget', {'id': old_id + ':handSide', 'value': 1}, owner)
        state = bridge.snapshot()
        assert state['model_id'] == MODEL and state['side'] == 'right'
        assert not state['connected']
        with pytest.raises(ValueError):
            bridge.dispatch('widget', {'id': old_id + ':handSide', 'value': 0}, owner)
        bridge.last_seen = time.monotonic() - bridge.CONTROLLER_LEASE_S - 1
        bridge.check_lease()
        assert bridge.controller is None
        assert bridge.snapshot()['device']['stop_reason'] == 'controller_heartbeat_timeout'
    finally:
        bridge.close()
