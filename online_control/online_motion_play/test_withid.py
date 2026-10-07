"""Independent with-ID editor storage, selection and static asset delivery."""
import json
from pathlib import Path
import pytest
import server
from a30_session import A30Session
from test_server import bridge

MODEL='TsenseI_v3_0_withid'


def test_switch_side_preserves_model_and_rejects_other_model_motion(bridge):
    bridge.dispatch('model',{'model_id':MODEL})
    state=bridge.snapshot()
    assert state['model_id']==MODEL and len(state['joints'])==20
    assert '_withid/' in state['urdf'] and state['file']=='motions_a30_withid_left.json'
    pose={j['name']:j['value'] for j in state['joints']}
    bridge.a30.persist({'pose':{'pose':pose,'transition':1.,'interval':.2}})
    saved=bridge.a30.path
    assert json.loads(saved.read_text(encoding='utf-8'))['_model_id']==MODEL
    with pytest.raises(ValueError):
        bridge.a30.validate_file({'_model_id':'TsenseI_v3_0','_hand_side':'left','motions':{}})
    bridge.dispatch('widget',{'id':bridge.a30.generation+':handSide','value':1})
    assert bridge.a30.model_id==MODEL and bridge.a30.side=='right'
    assert bridge.a30.path != saved
    bridge.dispatch('model',{'model_id':'TsenseI_v3_0'})
    assert not bridge.a30.motions
    assert bridge.a30.path.name=='motions_a30_left.json'
    bridge.dispatch('model',{'model_id':MODEL})
    assert 'pose' in bridge.a30.motions
    bridge.dispatch('model',{'model_id':'TsenseI_v2'})


def test_bundle_contains_both_models_and_shared_dispatcher():
    import package
    manifest={name:path for path,name in package.files()}
    assert all(p.is_file() for p in manifest.values())
    assert 'app/online_control/shared/hand-models.js' in manifest
    for folder in ('TsenseI_hand_v3_0','TsenseI_hand_v3_0_withid'):
        assert f'app/assets/hands/{folder}/hand_profile.yaml' in manifest
