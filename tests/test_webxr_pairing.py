"""Human pairing codes leave stream capabilities and tracking lifetime separate."""
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'src/tracking')]
from online_control.online_motion_track.webxr_input import WebXRInputRelay


def test_fixed_code_never_expires_and_refresh_keeps_stream_capability():
    now = [100.]
    relay = WebXRInputRelay(clock=lambda: now[0])
    first = relay.create('pc', 'left')
    assert first['pairing_code'] == '123456'
    assert first['pairing_expires_in_s'] is None
    assert first['entry_path'] == '/q'
    assert relay.pair(first['pairing_code'], 'quest')['token'] == first['token']
    now[0] += 10 * 365 * 24 * 60 * 60
    assert relay.pair('123456', 'quest')['token'] == first['token']
    refreshed = relay.renew_code('pc')
    assert refreshed['pairing_code'] == first['pairing_code']
    assert relay.pair(refreshed['pairing_code'], 'quest')['token'] == first['token']
    relay.close('pc')
    with pytest.raises(ValueError, match='开始识别'):
        relay.pair(refreshed['pairing_code'], 'quest')


def test_shared_fixed_code_never_selects_an_ambiguous_control_session():
    relay = WebXRInputRelay()
    first = relay.create('left', 'left')
    second = relay.create('right', 'right')
    assert first['pairing_code'] == second['pairing_code'] == '123456'
    with pytest.raises(ValueError, match='多个'):
        relay.pair('123456', 'quest')
    relay.close('right')
    assert relay.pair('123456', 'quest')['token'] == first['token']


def test_attempt_limits_expire_without_invalidating_other_sessions():
    now = [10.]
    relay = WebXRInputRelay(clock=lambda: now[0])
    pairing = relay.create('pc', 'left')
    for _ in range(10):
        with pytest.raises(ValueError, match='6 位'):
            relay.pair('not-a-code', 'peer')
    with pytest.raises(ValueError, match='频繁'):
        relay.pair(pairing['pairing_code'], 'peer')
    assert relay.pair(pairing['pairing_code'], 'other')['token'] == pairing['token']
    now[0] += 60
    assert relay.pair(pairing['pairing_code'], 'peer')['token'] == pairing['token']


def test_recreating_or_restarting_keeps_code_and_revokes_old_capability():
    relay = WebXRInputRelay()
    old = relay.create('pc', 'left')
    new = relay.create('pc', 'left')
    assert old['token'] not in relay.tokens
    assert old['pairing_code'] == new['pairing_code'] == '123456'
    assert relay.pair(new['pairing_code'], 'quest')['token'] == new['token']
    restarted = WebXRInputRelay()
    pairing = restarted.create('new-pc', 'right')
    assert pairing['pairing_code'] == old['pairing_code']
    assert restarted.pair('123456', 'quest')['token'] == pairing['token']
    assert old['token'] not in restarted.tokens
    with pytest.raises(ValueError, match='无效'):
        restarted.pair('654321', 'quest')
