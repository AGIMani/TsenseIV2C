"""Read current compiled MJCF proxies and URDF visuals; write inspection PNGs only."""
from pathlib import Path
import sys, itertools, json
ROOT=Path(__file__).resolve().parents[3]
sys.path[:0]=[str(ROOT), str(ROOT/'src')]
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from matplotlib.patches import Patch
from matplotlib import font_manager
import numpy as np
import trimesh
import mujoco
from scipy.spatial import ConvexHull
from hand_profiles import load_hand_profile
from TsenseI_retargeting.fk.urdf import UrdfModel

OUT=Path(__file__).resolve().parent
FONT='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
font_manager.fontManager.addfont(FONT)
plt.rcParams.update({'font.family':font_manager.FontProperties(fname=FONT).get_name(), 'axes.unicode_minus':False})
COLORS={'thumb':'#e59523','index':'#2686d9','mid':'#199578','ring':'#8d60bf','pinky':'#d94f83','palm':'#465768'}
LABELS={'thumb':'拇指','index':'食指','mid':'中指','ring':'无名指','pinky':'小指','palm':'掌部盒'}
MODEL_IDS=('TsenseI_v3_0','TsenseI_v3_0_withid','TsenseI_v3_2','TsenseI_v3_2_withid')
TITLES=('A3.0','A3.0 / withid','A3.2','A3.2 / withid')

def convex2(points):
    points=np.unique(np.round(points,9),axis=0)
    if len(points)<3:return points
    return points[ConvexHull(points).vertices]

def read_model(mid):
    p=load_hand_profile(mid)
    q=p.init_pose('left')
    u=UrdfModel(p.path('urdf','left')); poses=u.forward_kinematics(q)
    f=p.create_collision_filter('left'); f.sync_pose_from_dict(q)
    shells=[]
    for link,visuals in u.visual_meshes_by_link().items():
        for v in visuals:
            mesh=trimesh.load_mesh(v.path,process=False)
            points=np.asarray(mesh.vertices)*v.scale
            points=points@v.origin[:3,:3].T+v.origin[:3,3]
            points=points@poses[link][:3,:3].T+poses[link][:3,3]
            shells.append((points,np.asarray(mesh.faces)))
    geoms=[]; m=f.model; d=f.data
    for gid in range(m.ngeom):
        name=mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,gid) or ''
        finger=next((k for k in COLORS if k in name.lower()),'palm')
        gt=int(m.geom_type[gid]); pos=d.geom_xpos[gid]; rot=d.geom_xmat[gid].reshape(3,3)
        spec={'name':name,'finger':finger,'type':gt,'position':pos.copy()}
        if gt==int(mujoco.mjtGeom.mjGEOM_CAPSULE):
            half=m.geom_size[gid,1]; axis=rot[:,2]
            spec.update(a=pos-half*axis,b=pos+half*axis,r=m.geom_size[gid,0])
        elif gt==int(mujoco.mjtGeom.mjGEOM_BOX):
            vertices=np.array(list(itertools.product((-1,1),repeat=3)))*m.geom_size[gid]
            spec['points']=vertices@rot.T+pos
        elif gt==int(mujoco.mjtGeom.mjGEOM_MESH):
            mesh_id=int(m.geom_dataid[gid]); start=m.mesh_vertadr[mesh_id]; count=m.mesh_vertnum[mesh_id]
            vertices=m.mesh_vert[start:start+count].copy()
            spec['points']=vertices@rot.T+pos
        else: raise ValueError((mid,name,gt))
        geoms.append(spec)
    return dict(mid=mid,p=p,filter=f,shells=shells,geoms=geoms)

def project_geom(spec,dims,sign):
    if 'a' in spec:
        angles=np.linspace(0,2*np.pi,48)
        circle=np.column_stack((np.cos(angles),np.sin(angles)))*spec['r']
        points=np.concatenate((spec['a'][dims]+circle,spec['b'][dims]+circle))
    else:points=spec['points'][:,dims]
    points=points.copy();points[:,0]*=sign
    return convex2(points)*1000

def render(models,view):
    fig,axs=plt.subplots(2,2,figsize=(15,15),facecolor='#f4f7fa')
    fig.subplots_adjust(top=.875,bottom=.11,left=.045,right=.97,wspace=.1,hspace=.24)
    fig.text(.05,.967,'当前参与碰撞检测的几何体',fontsize=25,fontweight='bold',color='#183144')
    direction='掌面视角' if view=='front' else '侧面视角'
    fig.text(.05,.934,f'左手 · 初始姿态 · {direction}  /  彩色 = 碰撞代理，浅灰 = 外观模型参考',fontsize=13,color='#546b7d')
    for idx,(ax,model) in enumerate(zip(axs.flat,models)):
        p=model['p']; geoms=model['geoms']
        dims=list((1,2) if p.mirror_axis=='y' else (0,2)) if view=='front' else list((0,2) if p.mirror_axis=='y' else (1,2))
        thumb=np.mean([g['position'][dims[0]] for g in geoms if g['finger']=='thumb'])
        pinky=np.mean([g['position'][dims[0]] for g in geoms if g['finger']=='pinky'])
        sign=1 if thumb<pinky else -1
        shellpoints=[]
        for points,faces in model['shells']:
            xy=points[:,dims].copy()*1000;xy[:,0]*=sign
            shellpoints.append(xy)
            # Opaque pale visual triangles leave exact shell silhouette as a quiet reference.
            ax.add_collection(PolyCollection(xy[faces],facecolor='#dce3e9',edgecolor='none',rasterized=True,zorder=1))
        for g in geoms:
            border=project_geom(g,dims,sign)
            color=COLORS[g['finger']]
            ax.fill(border[:,0],border[:,1],facecolor=color,alpha=.46,edgecolor=color,linewidth=1.3,zorder=3)
            ax.plot(*np.vstack((border,border[0])).T,color=color,alpha=.92,linewidth=.85,zorder=4)
        points=np.concatenate(shellpoints);lo=points.min(0);hi=points.max(0)
        # Clip long wrist assemblies only below a consistent physical display range.
        proxy_low=min(project_geom(g,dims,sign)[:,1].min() for g in geoms)
        lo[1]=max(lo[1],proxy_low-48)
        ax.set_xlim(lo[0]-9,hi[0]+9);ax.set_ylim(lo[1]-3,hi[1]+12)
        ax.set_aspect('equal');ax.set_facecolor('white')
        ax.spines[['top','right']].set_visible(False)
        for sp in ax.spines.values():sp.set_color('#c4cdd6')
        ax.tick_params(colors='#6d8090',labelsize=9)
        ax.set_xlabel('横向 / mm',fontsize=10,color='#6d8090');ax.set_ylabel('高度 / mm',fontsize=10,color='#6d8090')
        ax.grid(alpha=.12,zorder=0)
        count_mesh=sum(g['type']==7 for g in geoms);count_box=sum(g['type']==6 for g in geoms);count_caps=sum(g['type']==3 for g in geoms)
        desc=f'{count_caps} 个胶囊体'+(f' + {count_mesh} 个指尖凸网格' if count_mesh else '')+(f' + {count_box} 个掌部盒' if count_box else '')
        ax.set_title(TITLES[idx]+'\n'+desc,loc='left',fontsize=15,fontweight='bold',color='#183144',pad=14)
        scope='跨手指 + 指中/末段 ↔ 掌部' if count_box else '仅跨手指；掌部与外壳不参与'
        ax.text(.01,.975,scope,transform=ax.transAxes,va='top',fontsize=10,color='#536c7e',bbox=dict(facecolor='white',edgecolor='none',alpha=.9,pad=4))
        if count_box and view=='front':
            g=next(g for g in geoms if g['finger']=='palm');xy=np.mean(project_geom(g,dims,sign),axis=0)
            ax.text(*xy,'掌部盒',fontsize=9,color='#172f42',ha='center',va='center',zorder=5)
    fig.legend(handles=[Patch(facecolor=c,edgecolor=c,alpha=.7,label=LABELS[k]) for k,c in COLORS.items()],loc='lower center',bbox_to_anchor=(.5,.064),ncol=6,frameon=False,fontsize=12)
    fig.text(.05,.039,'同一根手指内部不做自碰撞；二维投影重叠不代表三维接触。四个模型的初始姿态均检测到 0 个接触。',fontsize=11,color='#546b7d')
    fig.text(.05,.02,'来源：各 hand_profile 指向的 MJCF（MuJoCo 编译后几何）及 URDF 外观；2026-10-03 当前工作区。',fontsize=10,color='#8093a1')
    output=OUT/f'collision_overlay_{view}.png'
    fig.savefig(output,dpi=160,facecolor=fig.get_facecolor());plt.close(fig)
    print(output,flush=True)

if __name__=='__main__':
    models=[]
    for mid in MODEL_IDS:
        models.append(read_model(mid));print('loaded',mid,flush=True)
    render(models,'front');render(models,'side')
