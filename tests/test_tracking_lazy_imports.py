"""Network tracking and public preprocessing APIs do not import camera extras."""
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]


def test_network_tracking_and_public_exports_work_without_desktop_dependencies():
    code = """
import importlib.abc
import sys
import types
sys.path.insert(0, 'src')
class BlockDesktopImports(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in {'cv2', 'torch', 'PySide6', 'mediapipe'}:
            raise AssertionError('Unexpected desktop import: ' + fullname)
sys.meta_path.insert(0, BlockDesktopImports())
import tracking
assert 'tracking.handoccnet' not in sys.modules
assert 'tracking.wilor' not in sys.modules
assert 'tracking.wuji_mediapipe' not in sys.modules
from tracking import cloudxr, webxr
from tracking import WujiMediaPipeConfig, preprocess_wuji_world_points
from tracking.wuji_mediapipe import WujiMediaPipeConfig as DirectConfig
assert WujiMediaPipeConfig is DirectConfig
assert callable(preprocess_wuji_world_points)
assert set(tracking.__all__) <= set(dir(tracking))
assert not {'cv2', 'torch', 'PySide6', 'mediapipe'} & sys.modules.keys()
# Preserve the public optional-backend names while loading them on demand.
fake = types.ModuleType('tracking.handoccnet')
fake.HandOccNetBackend = type('HandOccNetBackend', (), {})
fake.get_handoccnet_backend = lambda: None
sys.modules[fake.__name__] = fake
from tracking import HandOccNetBackend, get_handoccnet_backend
assert HandOccNetBackend is fake.HandOccNetBackend
assert get_handoccnet_backend is fake.get_handoccnet_backend
assert tracking.HandOccNetBackend is fake.HandOccNetBackend
try:
    tracking.unknown_backend
except AttributeError:
    pass
else:
    raise AssertionError('Unknown API must raise AttributeError')
"""
    subprocess.run([sys.executable, '-c', code], cwd=ROOT, check=True,
                   capture_output=True, text=True)
