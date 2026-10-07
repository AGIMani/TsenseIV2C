"""Save/check pre-change kinematic and V2 invariants without touching models."""
from pathlib import Path
import hashlib
import json
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src'))
from hand_profiles import A30_MODEL_IDS, A32_MODEL_IDS, load_hand_profile

def normalized(node):
    return [node.tag, sorted(node.attrib.items()), (node.text or '').strip(),
            [normalized(child) for child in node]]

def snapshot():
    urdfs = {}
    for model in (*A30_MODEL_IDS, *A32_MODEL_IDS):
        profile = load_hand_profile(model)
        for side in ('left', 'right'):
            path = profile.path('urdf', side)
            tree = ET.parse(path).getroot()
            for link in tree.findall('link'):
                for collision in link.findall('collision'):
                    link.remove(collision)
            urdfs[str(path.relative_to(ROOT))] = normalized(tree)
    v2 = {}
    for path in sorted((ROOT / 'assets/hands/TsenseI_hand').rglob('*')):
        if path.is_file() and path.suffix.lower() in ('.xml', '.urdf', '.yaml', '.stl'):
            v2[str(path.relative_to(ROOT))] = hashlib.sha256(path.read_bytes()).hexdigest()
    return {'non_collision_urdf': urdfs, 'v2_assets': v2}

target = Path(__file__).with_name('invariants_before.json')
if '--check' in sys.argv:
    before = json.loads(target.read_text())
    after = json.loads(json.dumps(snapshot()))
    assert before == after, 'URDF visual/kinematic/inertial or V2 asset changed'
    print('8 canonical URDFs: visuals, joints, inertias unchanged; V2 assets unchanged')
else:
    if target.exists():
        raise FileExistsError(target)
    target.write_text(json.dumps(snapshot(), indent=2))
    print(target)
