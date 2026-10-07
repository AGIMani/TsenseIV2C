"""With-ID assets, calibration isolation, collision masks and shared A3 wire path."""
from pathlib import Path
import sys
import threading
import urllib.request
import json
import xml.etree.ElementTree as ET
import numpy as np
import pytest
import trimesh
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'src'), str(ROOT / 'tools')]
from hand_profiles import load_hand_profile
from TsenseI_retargeting.fk.urdf import UrdfModel
from TsenseI_retargeting.fk.fk import RobotKeypointFK
from a30_kinematics import A30StrokeMapper
from online_control.online_motion_track.pipeline_v3_0.actuation import A30Actuation
from online_control.online_motion_track.main import Application, WebSession, make_server, model_description
from prepare_hand_v3_0 import MIRROR, right_name

MODEL = 'TsenseI_v3_0_withid'


def test_new_cad_preserved_and_right_meshes_do_not_overwrite_left():
    profile = load_hand_profile(MODEL)
    source = UrdfModel(ROOT / 'TsenseI_A3.0_hand_withid/urdf/A3-2691-1053yw1.urdf')
    left, right = [UrdfModel(profile.path('urdf', s)) for s in ('left', 'right')]
    assert not left.missing_meshes() and not right.missing_meshes()
    assert len(left.joint_names()) == 20
    assert left.joint_limits() == load_hand_profile('TsenseI_v3_0').joint_limits()
    for name, visuals in source.visual_meshes_by_link().items():
        for original, lv, rv in zip(visuals, left.visual_meshes_by_link()[name], right.visual_meshes_by_link()[right_name(name)], strict=True):
            assert original.path.read_bytes() == lv.path.read_bytes()
            assert lv.path != rv.path
            lm, rm = [trimesh.load_mesh(v.path, process=False) for v in (lv, rv)]
            np.testing.assert_allclose(rm.triangles, lm.triangles[:, ::-1] @ MIRROR, atol=1e-8)
    reflection=np.eye(4); reflection[:3,:3]=MIRROR
    rng=np.random.default_rng(30)
    for _ in range(10):
        pose={n:rng.uniform(lo,hi) for n,(lo,hi) in left.joint_limits().items()}
        a,b,c=source.forward_kinematics(pose),left.forward_kinematics(pose),right.forward_kinematics({right_name(n):v for n,v in pose.items()})
        for name in a:
            np.testing.assert_allclose(a[name],b[name],atol=1e-12)
            np.testing.assert_allclose(c[right_name(name)],reflection@b[name]@reflection,atol=1e-12)


def test_fresh_landmarks_and_mirrored_fk():
    profile=load_hand_profile(MODEL)
    old=load_hand_profile('TsenseI_v3_0')
    data=yaml.safe_load(profile.path('keypoints').read_text(encoding='utf-8'))
    old_data=yaml.safe_load(old.path('keypoints').read_text(encoding='utf-8'))
    assert profile.calibration_status()['point_count']==21
    assert profile.path('keypoints') != old.path('keypoints')
    assert data['robot_name'] != old_data['robot_name']
    assert not np.allclose([r['local_xyz'] for r in data['keypoints']], [r['local_xyz'] for r in old_data['keypoints']])
    left,right=[RobotKeypointFK(profile.path('urdf',s),profile.path('keypoints',s)) for s in ('left','right')]
    for fraction in (0,.4,.8):
        q={n:lo+fraction*(hi-lo) for n,(lo,hi) in profile.joint_limits().items()}
        np.testing.assert_allclose(right.compute({right_name(n):v for n,v in q.items()}),left.compute(q)@MIRROR,atol=1e-12)


@pytest.mark.parametrize('side',['left','right'])
def test_only_cross_finger_pairs_in_mujoco_and_mink(side):
    from online_control.online_motion_track.mink_guard import MinkCollisionGuard
    profile=load_hand_profile(MODEL)
    filt=profile.create_collision_filter(side)
    model=filt.model
    assert model.ngeom==15 and filt.data.ncon==0
    fingers=('thumb','index','mid','ring','pinky')
    finger=lambda i:next(f for f in fingers if f'_{f}_' in model.geom(i).name)
    expected=set()
    for a in range(model.ngeom):
        for b in range(a+1,model.ngeom):
            enabled=bool(model.geom_contype[a]&model.geom_conaffinity[b] or model.geom_contype[b]&model.geom_conaffinity[a])
            assert enabled==(finger(a)!=finger(b))
            if enabled: expected.add((a,b))
    assert len(expected)==90
    guard=MinkCollisionGuard(profile,side)
    assert {tuple(sorted(p)) for p in guard.pairs}==expected
    pose={n:min(1.,hi) if any(t in n for t in ('_MCP_','_PIP_','_DIP_')) else 0. for n,(lo,hi) in profile.joint_limits(side).items()}
    filt.sync_pose_from_dict(pose)
    assert filt.data.ncon>0  # Finger contact is still checked.
    for contact in filt.data.contact:
        assert finger(contact.geom1)!=finger(contact.geom2)


@pytest.mark.parametrize('side',['left','right'])
def test_same_pose_has_identical_wire_bytes_but_distinct_identity(side):
    old,new=[A30Actuation(side,model_id=m) for m in ('TsenseI_v3_0',MODEL)]
    rng=np.random.default_rng(32)
    for _ in range(10):
        pose={n:rng.uniform(lo,hi) for n,(lo,hi) in zip(new.joint_names,new._limits)}
        a,b=old.convert(pose),new.convert(pose)
        assert b.as_dict()['model_id']==MODEL
        assert old.encode(a)==new.encode(b)


@pytest.mark.parametrize('side',['left','right'])
def test_retarget_session_uses_independent_keypoints_and_records_identity(side,tmp_path):
    s=WebSession(side,True,tmp_path,model_id=MODEL)
    try:
        solver=s.pipeline.session.retargeter
        assert 'withid' in str(solver.keypoint_path)
        world=solver.fk.compute()
        result=s.process({'sequence':0,'timestamp_ms':1000,'capture_unix_ms':1790175000000,
            'hand':{'label':side.title(),'world':world.tolist(),'normalized':world.tolist()}})
        assert result['tracking'] and result['actuation']['model_id']==MODEL
    finally:s.close()
    assert json.loads(s.recorder.path.read_text(encoding='utf-8').splitlines()[0])['model_id']==MODEL


def test_http_assets_shared_module_control_and_package_inclusion():
    server=make_server(0)
    thread=threading.Thread(target=server.serve_forever,daemon=True); thread.start()
    opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
    def get(path):
        with opener.open(f'http://127.0.0.1:{server.server_port}'+path,timeout=10) as response:return response.read()
    try:
        assert MODEL in json.loads(get('/api/health'))['models']
        assert MODEL.encode() in get('/shared/hand-models.js')
        for side in ('left','right'):
            desc=json.loads(get(f'/api/model?model={MODEL}&side={side}'))
            assert desc['calibration']['point_count']==21
            for link in desc['links']:
                for v in link['visuals']:
                    assert v['mesh'].startswith('/assets-v3-0-withid/')
                    assert len(get(v['mesh']))>84
            response=server.application.create({'model_id':MODEL,'side':side,'recording':False,'collision_mode':'stop'})
            assert response['hardware']['model_id']==MODEL
            assert response['actuation']['model_id']==MODEL
    finally:
        server.application.close();server.shutdown();server.server_close();thread.join()
    from online_control.online_motion_track.package import files
    manifest={name:path for path,name in files(full=True)}
    assert all(path.is_file() for path in manifest.values())
    for name in ('hand_profile.yaml','robot_keypoints_21.yaml','robot_keypoints_21_right.yaml','mjcf/mjmodel.xml'):
        assert f'app/assets/hands/TsenseI_hand_v3_0_withid/{name}' in manifest
