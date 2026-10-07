"""Compatibility imports; A3.2 platforms share the same SDK recorder."""
from a32_driver.recording import (
    A32SessionRecorder, default_channels, export_state_capture,
    recording_metadata, telemetry_event,
)

__all__ = ["A32SessionRecorder", "default_channels", "export_state_capture",
           "recording_metadata", "telemetry_event"]
