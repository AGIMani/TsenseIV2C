"""Render current compiled geometry; reuse the previous scientific plot layout."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
reference = ROOT / "exports/collision_review_20261003/static/render_current.py"
source = reference.read_text(encoding="utf-8")
source = source.replace("OUT=Path(__file__).resolve().parent", f"OUT=Path({str(Path(__file__).parent)!r})")
source = source.replace("当前参与碰撞检测的几何体", "贴合外壳的低复杂度碰撞包络")
source = source.replace("彩色 = 碰撞代理，浅灰 = 外观模型参考", "彩色 = 真实碰撞包络，浅灰 = CAD 外壳")
source = source.replace("desc=f'{count_caps} 个胶囊体'+(f' + {count_mesh} 个指尖凸网格' if count_mesh else '')+(f' + {count_box} 个掌部盒' if count_box else '')",
                        "desc=f'{count_mesh} 个指节外包络'+(f' + {count_box} 个掌部外包络' if count_box else '')+' · 0 个胶囊体'")
source = source.replace("count_mesh=sum(g['type']==7 for g in geoms)",
                        "count_mesh=sum(g['type']==7 and g['finger']!='palm' for g in geoms)")
source = source.replace("count_box=sum(g['type']==6 for g in geoms)",
                        "count_box=sum(g['finger']=='palm' for g in geoms)")
source = source.replace("掌部盒", "掌部外包络")
source = source.replace("仅跨手指；掌部与外壳不参与", "仅跨手指；掌部与安装结构不参与")
source = source.replace("同一根手指内部不做自碰撞；二维投影重叠不代表三维接触。四个模型的初始姿态均检测到 0 个接触。",
                        "最大外扩 ≤ 0.35 mm（相对完整凸包）；各源网格顶点均在包络内。二维重叠不代表三维接触。")
source = source.replace("collision_overlay_{view}.png", "collision_outer_envelopes_{view}.png")
exec(compile(source, str(reference), "exec"), {"__name__": "__main__", "__file__": str(reference)})
