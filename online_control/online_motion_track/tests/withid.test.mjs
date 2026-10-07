import assert from 'node:assert/strict';
import test from 'node:test';
import {isA30Model} from '../../shared/hand-models.js';
import {emergencyFrames} from '../../online_motion_play/browser-serial.js';
import {A30BrowserHandControl} from '../static/a30-browser-hand-control.js';
import {armBlockers} from '../static/control-readiness.js';

test('with-ID uses A3 emergency opcodes and keeps its model identity in events',()=>{
  const modelId='TsenseI_v3_0_withid';
  assert.ok(isA30Model(modelId));assert.ok(!isA30Model('TsenseI_v3_2'));
  assert.deepEqual(emergencyFrames(modelId),emergencyFrames('TsenseI_v3_0'));
  const events=[];
  const c=new A30BrowserHandControl({modelId,recorder:{record:(type,data)=>events.push({type,...data})}});
  c.event('test',{model_id:'TsenseI_v3_0'});
  assert.equal(events.at(-1).model_id,modelId);
  const reasons=armBlockers({allowed:true,connected:true,active:true,mode:'stop',requireHandConfirmation:false,
    hardware:{model_id:modelId,rebase_required:true},handAgeMs:0});
  assert.ok(!reasons.some(r=>r.includes('映射目标未就绪')));
});
