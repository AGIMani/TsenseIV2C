from hashlib import sha256
from pathlib import Path

# Covers policy plus low-level conversion/protocol code shared by both bundles.
def downlink_version():
    import a30_kinematics.downlink as protocol
    import utils.motion_limits as motion_limits
    paths = [Path(protocol.__file__), Path(motion_limits.__file__), *sorted(p for p in Path(__file__).parent.iterdir()
              if p.suffix in ('.py', '.js'))]
    digest = sha256()
    for path in paths:
        digest.update(path.read_bytes().replace(b'\r\n',b'\n'))
    return digest.hexdigest()
