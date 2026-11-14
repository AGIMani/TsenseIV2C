from hashlib import sha256
from pathlib import Path


# Covers the A3.2 SDK conversion and motion policy shared by both web bundles.
def downlink_version():
    import a32_driver.device as device
    import a32_kinematics.mapper as mapper
    import utils.motion_limits as motion_limits
    shared = Path(__file__).parent
    paths = [Path(device.__file__), Path(mapper.__file__), Path(motion_limits.__file__),
             *(shared / name for name in ("version.py", "aluminum.js", "ui-language.js", "ui-translations.js"))]
    digest = sha256()
    for path in paths:
        digest.update(path.read_bytes().replace(b'\r\n', b'\n'))
    return digest.hexdigest()
