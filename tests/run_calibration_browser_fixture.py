"""Real web/bridge fixture; calibration saves are redirected into a temp folder."""
from pathlib import Path
import os
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'src'), str(ROOT / 'tools'), str(ROOT / 'tests')]
from online_control.online_motion_track.calibration import CalibrationSession

if __name__ == '__main__':
    original = CalibrationSession.__init__
    with tempfile.TemporaryDirectory(prefix='hand-calibration-browser-') as folder:
        def isolated_init(self, *args, **kwargs):
            original(self, *args, **kwargs)
            target = Path(folder) / self.model_id / self.path.name
            target.parent.mkdir(parents=True, exist_ok=True)
            if not target.exists():
                target.write_bytes(self.path.read_bytes())
            self.path = target
            self._read()
        CalibrationSession.__init__ = isolated_init
        if os.environ.get('CALIBRATION_WUJI_SOURCE') == 'sdk':
            from run_wuji_sdk_browser_fixture import main
        else:
            from run_wuji_browser_fixture import main
            os.environ['WUJI_SOURCE'] = 'bridge'
        main()
