from pathlib import Path
import hashlib,json
ROOT=Path(__file__).resolve().parents[2]
snapshot=json.loads((Path(__file__).parent/'geometry_regeneration_before.json').read_text())
changed=[name for name,digest in snapshot.items() if hashlib.sha256((ROOT/name).read_bytes()).hexdigest()!=digest]
print('Changed on cached rerun:', changed)
assert not changed
