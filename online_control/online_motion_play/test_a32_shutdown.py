"""Closed sessions must not retain native bindings until interpreter teardown."""
import pytest

from a32_session import A32Session
from test_a32 import FakeDevice, Clock


@pytest.mark.parametrize('failure', [None, 'device', 'recorder'])
def test_close_releases_planner_even_when_session_is_retained(tmp_path, monkeypatch, failure):
    session = A32Session(tmp_path, device=FakeDevice(), clock=Clock(), start=False)
    assert session.planner is not None
    if failure:
        def fail(*args, **kwargs):
            raise OSError('close failed')
        monkeypatch.setattr(session.device if failure == 'device' else session.recorder,
                            'close' if failure == 'device' else 'stop', fail)
        with pytest.raises(OSError, match='close failed'):
            session.close()
    else:
        session.close()
    session.close()
    assert session.planner is session.input is session.output is None


def test_close_joins_control_thread_before_releasing_planner(tmp_path):
    session = A32Session(tmp_path, device=FakeDevice())
    session.close()
    assert not session._io_thread.is_alive()
    assert session.planner is session.input is session.output is None
