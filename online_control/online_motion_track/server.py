"""Relocatable deployment entrypoint; also runnable in the source tree."""
from pathlib import Path
import runpy

here = Path(__file__).resolve().parent
entry = here / "app/online_control/online_motion_track/main.py"
if not entry.exists():
    entry = here / "main.py"
runpy.run_path(str(entry), run_name="__main__")
