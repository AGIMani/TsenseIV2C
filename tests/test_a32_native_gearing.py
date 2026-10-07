"""Exercise the real SDK adapter offline; no USB/TCP connection or motor commands."""
import json
from pathlib import Path
import sys
import threading
from unittest.mock import Mock

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from a32_driver.native import NativeHand


def test_both_mit_targets_use_50_to_1_without_extra_scaling():
    hand = NativeHand.__new__(NativeHand)
    hand._call = Mock()
    hand.set_targets([0.] * 18, [20., 60.])
    call = hand._call.call_args.args
    assert list(call[2]) == pytest.approx([20., 60.])
    # Both channels use the SDK's fixed x50 exactly once.
    assert call[2][0] * 50 == pytest.approx(1000.)
    assert call[2][1] * 50 == pytest.approx(3000.)


def test_feedback_position_velocity_and_target_use_physical_output_units():
    hand = NativeHand.__new__(NativeHand)
    hand._lock, hand._handle, hand._dll = threading.RLock(), 1, Mock()
    raw = dict(mit=[dict(index=0, position=20., velocity_rpm=3., iq=2.),
                    dict(index=1, position=36., velocity_rpm=6., iq=4.)],
               targets=dict(self=[1.]*18, mit_pos=[20., 36.]))
    hand._text = Mock(return_value=json.dumps(raw))
    try:
        state = hand.get_state()
        assert state['mit'][0]['position'] == 20.
        assert state['mit'][0]['velocity_rpm'] == 3.
        assert state['mit'][1]['position'] == pytest.approx(36.)
        assert state['mit'][1]['velocity_rpm'] == pytest.approx(6.)
        assert state['mit'][1]['iq'] == 4.
        assert state['targets']['mit_pos'] == pytest.approx([20., 36.])
        assert state['targets']['self'] == [1.]*18
        assert state['mit_gear_ratios'] == [50., 50.]
        raw['mit'][1]['position'] = None
        hand._text.return_value = json.dumps(raw)
        assert hand.get_state()['mit'][1]['position'] is None
    finally:
        hand._handle = None


@pytest.mark.skipif(sys.platform != 'win32', reason='Supplied native SDK is Windows x64')
def test_actual_native_dll_target_roundtrip_without_connection():
    hand = NativeHand()
    try:
        hand.set_targets([0.] * 18, [20., 60.])
        raw = json.loads(hand._text(hand._dll.a32_state))
        assert raw['targets']['mit_pos'] == pytest.approx([20., 60.])
        state = hand.get_state()
        assert not state['connected'] and not state['streaming']
        assert state['targets']['mit_pos'] == pytest.approx([20., 60.])
        assert state['mit_gear_ratios'] == [50., 50.]
    finally:
        hand.close()
