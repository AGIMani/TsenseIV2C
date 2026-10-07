"""Repair the missing 6.2 NewDeviceTransform wrapper in official Python bindings.

Run with the same Python environment as the workbench. This changes only the
installed optional SDK, retaining .before-hand-transform backups.
"""
from pathlib import Path
import shutil


def main():
    import leap.events
    import leap.event_listener
    events = Path(leap.events.__file__)
    listeners = Path(leap.event_listener.__file__)
    text = events.read_text()
    if "class NewDeviceTransformEvent" not in text:
        anchor = "class NoneEvent(Event):"
        if text.count(anchor) != 1:
            raise RuntimeError("Unrecognized SDK events.py; no changes made")
        text = text.replace(anchor,
            'class NewDeviceTransformEvent(Event):\n'
            '    """Metadata-only notification; tracking remains in TrackingEvent."""\n'
            '    _EVENT_TYPE = EventType.NewDeviceTransform\n'
            '    _EVENT_ATTRIBUTE = "pointer"\n\n\n' + anchor)
        anchor = "        EventType.IMU: IMUEvent,"
        if text.count(anchor) != 1:
            raise RuntimeError("Unrecognized SDK event factory; no changes made")
        text = text.replace(anchor, anchor + "\n        EventType.NewDeviceTransform: NewDeviceTransformEvent,")
    dispatch = listeners.read_text()
    anchor = '        EventType.IMU: "on_imu_event",'
    if 'EventType.NewDeviceTransform:' not in dispatch:
        if dispatch.count(anchor) != 1:
            raise RuntimeError("Unrecognized SDK listener; no changes made")
        dispatch = dispatch.replace(anchor, anchor + '\n        EventType.NewDeviceTransform: "on_none_event",')
    for path, result in ((events, text), (listeners, dispatch)):
        if path.read_text() != result:
            backup = path.with_name(path.name + ".before-hand-transform")
            if not backup.exists():
                shutil.copy2(path, backup)
            path.write_text(result)
            print("Patched", path)


if __name__ == "__main__":
    main()
