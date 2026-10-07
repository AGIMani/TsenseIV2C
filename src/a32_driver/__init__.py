"""A3.2 in-process SDK integration; importing this package never opens hardware."""

from .device import A32Device, A32DriverError
from .native import NativeHand

__all__ = ["A32Device", "A32DriverError", "NativeHand"]
