from pathlib import Path
import hashlib,json,re,sys
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT),str(ROOT/'src')]
from hand_profiles import load_hand_profile,A30_MODEL_IDS,A32_MODEL_IDS
reports=[]
for mid in (*A30_MODEL_IDS,*A32_MODEL_IDS):
    p=load_hand_profile(mid)
    r=json.loads((p.root/'collision_mesh_report.json').read_text())
    rows=r['links']
    maxerror=max(v['max_excess_mm'] for v in rows)
    for side in ('left','right'):
        path=p.path('urdf',side)
        baseline=ROOT/'exports/collision_implementation_20261003/baseline_geometry'/p.root.name/'urdf'/path.name
        without=lambda txt:'\n'.join(line.rstrip() for line in re.sub(r'<collision\b[^>]*>.*?</collision>','',txt,flags=re.S).splitlines())
        assert without(path.read_text())==without(baseline.read_text()),path
    print(mid,'collisionlinks',len(rows),'full',sum(v['full_hull_vertices'] for v in rows),
          'simplified',sum(v['vertices'] for v in rows),'maxerror_mm',maxerror,
          'min_vertices',min(v['vertices'] for v in rows),'max_vertices',max(v['vertices'] for v in rows))
    reports.append(dict(model_id=mid,links=len(rows),full_vertices=sum(v['full_hull_vertices'] for v in rows),
                        simplified_vertices=sum(v['vertices'] for v in rows),max_excess_mm=maxerror))
(Path(__file__).parent/'geometry_summary.json').write_text(json.dumps(reports,indent=2))
snapshot={str(f.relative_to(ROOT)):hashlib.sha256(f.read_bytes()).hexdigest()
          for mid in (*A30_MODEL_IDS,*A32_MODEL_IDS) for f in load_hand_profile(mid).root.rglob('*')
          if f.is_file() and (f.suffix in ('.urdf','.xml','.stl') or f.name in ('collision_geometry.yaml','collision_mesh_report.json'))}
(Path(__file__).parent/'geometry_regeneration_before.json').write_text(json.dumps(snapshot,indent=2))
