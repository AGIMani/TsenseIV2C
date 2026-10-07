"""Embed the shared decoder so the log viewer also works offline via file://."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BEGIN = "// BEGIN GENERATED A3 TELEMETRY DECODER"
END = "// END GENERATED A3 TELEMETRY DECODER"

def embedded_source():
    source = (ROOT / "online_control/shared/a30-telemetry.js").read_text(encoding="utf-8")
    # A local namespace avoids clashes with viewer code; source stays testable ESM.
    source = source.replace("export ", "")
    return BEGIN + "\nconst A3Telemetry = (() => {\n" + source + "\nreturn {telemetry,boardTelemetry,telemetryMetadata};\n})();\n" + END

def sync():
    path = ROOT / "tools/hand_control_log_viewer.html"
    html = path.read_text(encoding="utf-8")
    if BEGIN in html:
        start, end = html.index(BEGIN), html.index(END)+len(END)
        html = html[:start] + embedded_source() + html[end:]
    else:
        html = html.replace('"use strict";', '"use strict";\n' + embedded_source(), 1)
    path.write_text(html, encoding="utf-8")

if __name__ == "__main__":
    sync()
