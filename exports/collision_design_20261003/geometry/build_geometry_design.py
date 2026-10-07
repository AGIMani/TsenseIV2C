"""Isolated geometric prototype. Units: m internally, mm in figures/metrics.

An outer convex approximation made by intersecting a subset of supporting
halfspaces of the exact input convex hull. Unlike ordinary mesh decimation,
this construction cannot shrink through the input surface (up to roundoff).
The reported excess is against the full convex hull, not a concave shell.
"""
from pathlib import Path
import json
import math
import xml.etree.ElementTree as ET
import numpy as np
import trimesh
from scipy.spatial import ConvexHull, HalfspaceIntersection
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from matplotlib.font_manager import FontProperties

ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent
FONT = FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
plt.rcParams.update({'font.family': FONT.get_name(), 'axes.unicode_minus': False, 'font.size': 11})

def triangulated_hull(vertices):
    h = ConvexHull(vertices)
    m = trimesh.Trimesh(vertices=vertices, faces=h.simplices, process=True)
    m.remove_unreferenced_vertices()
    trimesh.repair.fix_normals(m)
    return h, m

def excess_distance(poly, reference):
    # Distance to a convex set is convex: its maximum over a polytope is
    # attained at a vertex. Thus checking all vertices bounds all its surface.
    ds=[]
    for start in range(0, len(poly.vertices), 8):
        ds.extend(trimesh.proximity.closest_point_naive(reference, poly.vertices[start:start+8])[1])
    return float(max(ds))

def prototype(source, stem, budgets=(32,64,96)):
    visual=trimesh.load(source, force='mesh', process=True)
    vs=np.asarray(visual.vertices)
    full_h, full=triangulated_hull(vs)
    full.export(OUT/f'{stem}_full_hull.stl')
    lo, hi=vs.min(axis=0), vs.max(axis=0)
    planes=[]
    for i in range(3):
        n=np.zeros(3); n[i]=1
        planes.append(np.r_[n,-hi[i]])
        planes.append(np.r_[-n,lo[i]])
    center=vs.mean(axis=0)
    selected=set()
    snapshots={}
    for count in range(6,max(budgets)+1):
        pv=HalfspaceIntersection(np.asarray(planes),center).intersections
        ph, pm=triangulated_hull(pv)
        if count in budgets:
            name=f'{stem}_outer_{count}.stl'
            pm.export(OUT/name)
            err=excess_distance(pm,full)
            containment=float(np.max(vs@ph.equations[:,:3].T+ph.equations[:,3]))
            snapshots[count]=(pm,dict(support_planes=count,vertices=len(pm.vertices),triangles=len(pm.faces),
                volume_mm3=float(ph.volume*1e9),max_outward_distance_to_full_hull_mm=err*1000,
                max_source_vertex_violation_mm=containment*1000,
                source_vertices_contained=bool(containment<1e-9),file=name))
            print(stem,count,snapshots[count][1],flush=True)
        if count == max(budgets): break
        violation=pv@full_h.equations[:,:3].T+full_h.equations[:,3]
        scores=violation.max(axis=0)
        for i in selected: scores[i]=-np.inf
        idx=int(np.argmax(scores)); selected.add(idx)
        planes.append(full_h.equations[idx])
    return visual, full, snapshots, dict(source=str(source.relative_to(ROOT)),
        source_vertices=len(vs),source_triangles=len(visual.faces),
        full_hull_vertices=len(full.vertices),full_hull_triangles=len(full.faces),
        full_hull_volume_mm3=float(full_h.volume*1e9),
        candidates=[snapshots[k][1] for k in snapshots])

def poly2(v,axes):
    pts=np.asarray(v)[:,axes]*1000
    h=ConvexHull(pts)
    return pts[h.vertices]

def shell(ax,mesh,axes):
    tri=mesh.vertices[mesh.faces][:,:,axes]*1000
    ax.add_collection(PolyCollection(tri,facecolor='#737d87',edgecolor='none',alpha=.18,rasterized=True))

def overlay(ax,vis,proxy,axes,color):
    shell(ax,vis,axes)
    pp=poly2(proxy.vertices,axes)
    ax.fill(pp[:,0],pp[:,1],color=color,alpha=.14)
    ax.plot(np.r_[pp[:,0],pp[0,0]],np.r_[pp[:,1],pp[0,1]],color=color,lw=2.1)
    ax.set_aspect('equal'); ax.grid(alpha=.17)
    ax.set_xlabel(['x','y','z'][axes[0]]+' / mm')
    ax.set_ylabel(['x','y','z'][axes[1]]+' / mm')

def main():
    OUT.mkdir(parents=True,exist_ok=True)
    a32xml=ROOT/'assets/hands/TsenseI_hand_v3_2/mjcf/mjmodel.xml'
    thumb=prototype(ROOT/'TsenseI_A3.2_hand/meshes/left_hand_thumb_rotaback_link2.STL','a32_thumb_root')
    tip=prototype(ROOT/'assets/hands/TsenseI_hand_v3_2_withid/mjcf/collision_meshes/left_hand_index_DIP_link.stl','a32_withid_index_tip',(32,64,96,128))
    cg=ET.parse(a32xml).find('.//geom[@name="left_hand_thumb_rotaback_link2_capsule"]')
    a,b=np.fromstring(cg.get('fromto'),sep=' ').reshape(2,3)
    radius=float(cg.get('size')); length=float(np.linalg.norm(b-a))
    cap=trimesh.creation.capsule(height=length,radius=radius,count=[32,64])
    # trimesh capsule is centred at origin, with hemisphere centers at +/- h/2.
    cap.apply_transform(trimesh.geometry.align_vectors([0,0,1],(b-a)/length))
    cap.apply_translation((a+b)/2)
    vs=thumb[0].vertices
    t=np.clip((vs-a)@(b-a)/length**2,0,1)
    signed=np.linalg.norm(vs-(a+t[:,None]*(b-a)),axis=1)-radius
    cap_stats=dict(radius_mm=radius*1000,axis_length_mm=length*1000,
        exact_volume_mm3=float((math.pi*radius**2*length+4/3*math.pi*radius**3)*1e9),
        source_vertices_outside=int(np.count_nonzero(signed>1e-9)),
        source_vertices_total=len(vs),maximum_source_vertex_protrusion_mm=float(signed.max()*1000),
        maximum_capsule_sample_outward_distance_to_full_hull_mm=excess_distance(cap,thumb[1])*1000)
    thumb[3]['current_capsule']=cap_stats
    metrics=dict(method='Outer polytope from exact input hull support planes; candidate containment verified on all source vertices.',
        caution='Error is Euclidean excess beyond convex hull, not beyond concave visual shell. Geometry metrics do not establish runtime or hardware clearance.',
        thumb=thumb[3],withid_fingertip=tip[3])
    (OUT/'geometry_metrics.json').write_text(json.dumps(metrics,indent=2,ensure_ascii=False))
    # Keep geom name/masks to allow benchmark against exactly the same pairs.
    for planes in (32,64,96):
        tree=ET.parse(a32xml); root=tree.getroot(); asset=ET.Element('asset')
        ET.SubElement(asset,'mesh',name='prototype_thumb_root',file=str(OUT/f'a32_thumb_root_outer_{planes}.stl'))
        root.insert(3,asset)
        g=root.find('.//geom[@name="left_hand_thumb_rotaback_link2_capsule"]')
        del g.attrib['size']; del g.attrib['fromto']
        g.set('type','mesh'); g.set('mesh','prototype_thumb_root')
        tree.write(OUT/f'a32_thumb_root_outer_{planes}_model.xml',encoding='utf-8',xml_declaration=True)
    fig, axs=plt.subplots(2,3,figsize=(13.5,9.8),layout='constrained')
    chosen=thumb[2][64]
    titles=[f'当前胶囊体\n半径 {radius*1000:.2f} mm',
            f'完整凸包\n{len(thumb[1].vertices):,} 顶点',
            f'外包络低面数凸体\n{len(chosen[0].vertices)} 顶点 · 最大外扩 {chosen[1]["max_outward_distance_to_full_hull_mm"]:.2f} mm']
    for row,axes in enumerate(([0,2],[1,2])):
        for col,(proxy,color) in enumerate(((cap,'#dc724d'),(thumb[1],'#557ccc'),(chosen[0],'#159184'))):
            ax=axs[row,col]; overlay(ax,thumb[0],proxy,axes,color)
            ax.set_xlim((-24,38) if row==0 else (-40,22)); ax.set_ylim(-14,54)
            if row==0: ax.set_title(titles[col],pad=10,fontsize=13)
    fig.suptitle('A3.2 拇指根部：贴近实际外壳的碰撞体设计',fontsize=19,fontweight='bold')
    fig.supxlabel('灰色＝实际 STL 外壳投影；彩色＝碰撞包络。每行使用同一比例。\n低面数凸体包含全部源顶点；外扩误差相对完整凸包，凹槽仍由凸包填平。',fontsize=11)
    fig.savefig(OUT/'thumb_root_comparison.png',dpi=170,facecolor='#fbfcfd')
    plt.close(fig)
    fig,axs=plt.subplots(1,3,figsize=(12.6,5.4),layout='constrained')
    for ax,budget in zip(axs,(64,96,128)):
        mesh,stats=tip[2][budget]
        overlay(ax,tip[0],mesh,[0,2],'#159184')
        ax.set_xlim(-18,10); ax.set_ylim(-8,29)
        ax.set_title(f'{budget} 支撑面 / {stats["vertices"]} 顶点\n最大外扩 {stats["max_outward_distance_to_full_hull_mm"]:.3f} mm',fontsize=12)
    fig.suptitle(f'A3.2 with-ID 食指指尖：完整凸包 {len(tip[1].vertices):,} 顶点 → 外包络凸体',fontsize=16,fontweight='bold')
    fig.supxlabel('灰色＝当前碰撞 mesh；绿色＝候选包络。误差相对完整凸包；全部源顶点均被包含。',fontsize=11)
    fig.savefig(OUT/'withid_tip_comparison.png',dpi=170,facecolor='#fbfcfd')
    print(json.dumps(metrics,indent=2,ensure_ascii=False),flush=True)

if __name__=='__main__': main()
