#!/usr/bin/env python3
"""Export current compiled collision proxies and enabled pairs without changing models."""
from pathlib import Path
import json
import sys
import itertools
import hashlib

import numpy as np
import mujoco
import mink
from scipy.spatial import ConvexHull

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / 'src')]
from hand_profiles import load_hand_profile

FINGERS = ('thumb', 'index', 'mid', 'ring', 'pinky')
NAMES = dict(zip(FINGERS, ('拇指', '食指', '中指', '无名指', '小指')))
NAMES['palm'] = '掌部'


def values(array):
    return np.round(np.asarray(array, dtype=float), 10).tolist()


def enabled_pairs(model):
    """MuJoCo rigid geom filters, including the world-weld parent exception."""
    excluded = set(map(int, model.exclude_signature))
    pairs = set()
    mask_pairs = set()
    filter_parent = not (int(model.opt.disableflags) & int(mujoco.mjtDisableBit.mjDSBL_FILTERPARENT))
    for a, b in itertools.combinations(range(model.ngeom), 2):
        if not ((int(model.geom_contype[a]) & int(model.geom_conaffinity[b])) or
                (int(model.geom_contype[b]) & int(model.geom_conaffinity[a]))):
            continue
        mask_pairs.add((a, b))
        ba, bb = map(int, model.geom_bodyid[[a, b]])
        wa, wb = map(int, model.body_weldid[[ba, bb]])
        if wa == wb:
            continue
        if (min(ba, bb) << 16) + max(ba, bb) in excluded:
            continue
        if filter_parent and wa != 0 and wb != 0:
            pa = int(model.body_weldid[model.body_parentid[wa]])
            pb = int(model.body_weldid[model.body_parentid[wb]])
            if wa == pb or wb == pa:
                continue
        pairs.add((a, b))
    pairs.update(tuple(sorted(map(int, pair))) for pair in zip(model.pair_geom1, model.pair_geom2))
    return sorted(pairs), sorted(mask_pairs)


def extract(profile, side):
    path = profile.path('mjcf', side)
    model = mujoco.MjModel.from_xml_path(str(path))
    data = mujoco.MjData(model)
    pose = profile.init_pose(side)
    for jid in range(model.njnt):
        data.qpos[model.jnt_qposadr[jid]] = pose[model.joint(jid).name]
    mujoco.mj_forward(model, data)
    geoms = []
    for gid in range(model.ngeom):
        name = model.geom(gid).name
        finger = next((f for f in FINGERS if f in name), 'palm')
        kind = mujoco.mjtGeom(int(model.geom_type[gid])).name.removeprefix('mjGEOM_').lower()
        segment = ('指尖' if ('_DIP_' in name or (finger == 'thumb' and '_PIP_' in name)) else
                   '中段' if ('_PIP_' in name or (finger == 'thumb' and '_MCP_' in name)) else
                   '根部' if finger != 'palm' else '碰撞垫')
        geom = dict(id=gid, name=name, body=model.body(int(model.geom_bodyid[gid])).name,
                    type=kind, size=values(model.geom_size[gid]), pos=values(data.geom_xpos[gid]),
                    mat=values(data.geom_xmat[gid]), contype=int(model.geom_contype[gid]),
                    conaffinity=int(model.geom_conaffinity[gid]), group=finger, finger=finger,
                    label=NAMES[finger] + ' · ' + segment)
        if kind == 'mesh':
            mid = int(model.geom_dataid[gid])
            start, count = int(model.mesh_vertadr[mid]), int(model.mesh_vertnum[mid])
            verts = np.asarray(model.mesh_vert[start:start+count])
            hull = ConvexHull(verts)
            used = sorted(set(map(int, hull.simplices.flatten())))
            remap = {old: new for new, old in enumerate(used)}
            # MuJoCo collisions use this convex hull; coordinates are geom-local.
            geom['vertices'] = values(verts[used])
            geom['faces'] = [[remap[int(i)] for i in face] for face in hull.simplices]
        geoms.append(geom)
    pairs, mask_pairs = enabled_pairs(model)
    ids = list(range(model.ngeom))
    mink_pairs = sorted(mink.CollisionAvoidanceLimit(model, [(ids, ids)]).geom_id_pairs)
    contacts = [dict(pair=sorted([int(c.geom1), int(c.geom2)]), distanceMm=float(c.dist * 1000))
                for c in data.contact[:data.ncon]]
    distances = [dict(pair=list(pair), distanceMm=round(float(mujoco.mj_geomDistance(
        model, data, *pair, 1.0, None)) * 1000, 6)) for pair in pairs]
    return dict(geoms=geoms, pairs=pairs, minkPairs=mink_pairs, maskPairs=mask_pairs,
                contacts=contacts, distances=distances, source=str(path.relative_to(ROOT)),
                sourceSha256=hashlib.sha256(path.read_bytes()).hexdigest(), pose=pose,
                poseLabel='profile.init_pose 初始姿态', mirrorAxis=profile.mirror_axis,
                excludedBodyPairCount=int(model.nexclude), explicitPairCount=int(model.npair))


def main():
    models = []
    for mid, label in [('TsenseI_v3_0', 'A3.0'), ('TsenseI_v3_0_withid', 'A3.0 withid'),
                       ('TsenseI_v3_2', 'A3.2'), ('TsenseI_v3_2_withid', 'A3.2 withid')]:
        profile = load_hand_profile(mid)
        item = dict(id=mid, label=label, mirrorAxis=profile.mirror_axis,
                    sides={side: extract(profile, side) for side in ('left', 'right')})
        models.append(item)
        for side, data in item['sides'].items():
            print(label, side, len(data['geoms']), 'geoms;', len(data['pairs']), 'stop pairs;',
                  len(data['minkPairs']), 'mink pairs;', len(data['contacts']), 'contacts', flush=True)
    result = dict(schemaVersion=1, units='meter', generatedDate='2026-10-03',
                  poseDescription='各型号配置的初始姿态；并非实时 retarget 姿态。',
                  pairDescription='pairs 是 stop 的有效碰撞检测对；启用检测不代表当前已经相交。',
                  meshDescription='mesh 顶点为编译 MJCF 的 geom 局部坐标，faces 是实际碰撞凸包。',
                  models=models)
    target = Path(__file__).with_name('collision_data.json')
    target.write_text(json.dumps(result, ensure_ascii=False, separators=(',', ':')), encoding='utf-8')
    print(target)


if __name__ == '__main__':
    main()
