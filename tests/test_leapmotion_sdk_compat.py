"""Optional installed SDK regression: transform then tracking keeps polling."""
import pytest


def test_transform_notification_does_not_kill_tracking_poll_thread(monkeypatch):
    leap = pytest.importorskip("leap")
    from leapc_cffi import ffi
    from leap import connection as module
    received = []
    message_count = [0]
    connection = leap.Connection()
    tracking = ffi.new("LEAP_TRACKING_EVENT*")

    def poll(function, pointer, timeout, message):
        message_count[0] += 1
        if message_count[0] > 2:
            raise RuntimeError("Tracking callback was not delivered")
        if message_count[0] == 1:
            message.type = leap.EventType.NewDeviceTransform.value
            message.pointer = ffi.NULL
        else:
            message.type = leap.EventType.Tracking.value
            message.tracking_event = tracking

    class Listener(leap.Listener):
        def on_tracking_event(self, event):
            received.append(event)
            connection._stop_poll_flag = True

    monkeypatch.setattr(module, "success_or_raise", poll)
    connection.add_listener(Listener())
    connection._poll_loop()
    assert message_count[0] == 2
    assert len(received) == 1
