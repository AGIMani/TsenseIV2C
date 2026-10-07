"""Tracking APIs; optional camera backends load only when explicitly requested."""
from importlib import import_module

__all__ = [
    "HandOccNetBackend",
    "get_handoccnet_backend",
    "WiLoRBackend",
    "get_wilor_backend",
    "WujiMediaPipe21",
    "WujiMediaPipeConfig",
    "WujiMediaPipeFrame",
    "preprocess_wuji_normalized_points",
    "preprocess_wuji_world_points",
]

_BACKEND_MODULES = {
    "HandOccNetBackend": ".handoccnet",
    "get_handoccnet_backend": ".handoccnet",
    "WiLoRBackend": ".wilor",
    "get_wilor_backend": ".wilor",
    "WujiMediaPipe21": ".wuji_mediapipe",
    "WujiMediaPipeConfig": ".wuji_mediapipe",
    "WujiMediaPipeFrame": ".wuji_mediapipe",
    "preprocess_wuji_normalized_points": ".wuji_mediapipe",
    "preprocess_wuji_world_points": ".wuji_mediapipe",
}


def __getattr__(name):
    module_name = _BACKEND_MODULES.get(name)
    if module_name is None:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    value = getattr(import_module(module_name, __name__), name)
    globals()[name] = value
    return value


def __dir__():
    return sorted(set(globals()) | set(__all__))
