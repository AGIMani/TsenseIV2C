import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import vm from 'node:vm';

const app=readFileSync(new URL('../static/app.js',import.meta.url),'utf8');
const source=app.slice(app.indexOf('function applyFrameUpdate'),app.indexOf('function offerQuestFrame'));
function harness({direct=false,a30=false,a32=true}={}){
  const poses=[],hardware=[],records=[],tracking=[],elements=new Map();
  const context={recorder:{record:(type,data)=>records.push({type,data})},
    $:id=>{if(!elements.has(id))elements.set(id,{value:id==='side'?'left':'stop',
      classList:{toggle:(_name,value)=>tracking.push(value)}});return elements.get(id);},
    isA32Model:()=>false,isA30:()=>a30,isA32:()=>a32,isV2:()=>false,isControlModel:()=>false,
    directMode:()=>direct,showResponsePose:pose=>poses.push(pose),a32Panel:{update:value=>hardware.push(value)},
    renderHardware:value=>hardware.push(value),performance:{now:()=>120},latestWorld:[],
    skeleton:{setPoints:()=>{}},fpsCount:0,fpsStart:0};
  return {poses,hardware,records,tracking,elements,apply:vm.runInNewContext(source+';applyFrameUpdate;',context)};
}
for(const flag of ['background_discarded','display_discarded'])test(`${flag} solve is recorded but cannot move viewer or replace hardware state`,()=>{
  const h=harness();
  h.apply({frameId:3,hand:{label:'Left'},expected:'Left',timestamp_ms:100,
    packet:{session:'old-epoch',source:'webxr'}},{[flag]:true,tracking:false,
    pose:{q:.9},retarget_joint_angles_rad:{q:.9},hardware:{armed:true},message:'source replaced'});
  assert.deepEqual(h.poses,[]);assert.deepEqual(h.hardware,[]);assert.deepEqual(h.tracking,[false]);
  assert.equal(h.elements.get('tracking-text').textContent,'source replaced');
  assert.deepEqual(h.records.map(r=>r.type),['retarget','control_target_skipped']);
  assert.equal(h.records[0].data.joints.q,.9);
});
test('a valid safe hold still displays its safe pose when tracking ends',()=>{
  const h=harness();
  h.apply({frameId:5,hand:null,expected:'Left',timestamp_ms:100},{tracking:false,
    pose:{q:.05},hardware:{armed:false},message:'held safely'});
  assert.equal(h.poses[0].q,.05);assert.equal(h.hardware.length,1);assert.deepEqual(h.tracking,[false]);
});
test('accepted solve still updates the viewer and hardware panel',()=>{
  const h=harness();
  h.apply({frameId:4,hand:{label:'Left',world:[]},expected:'Left',timestamp_ms:100,
    requestStartedAt:105,inferredAt:100},{tracking:true,solve_ms:10,pose:{q:.1},
    retarget_joint_angles_rad:{q:.1},hardware:{armed:false}});
  assert.equal(h.poses[0].q,.1);assert.equal(h.hardware.length,1);assert.deepEqual(h.tracking,[true]);
});
for(const direct of [false,true])test(`A32 unarmed preview follows every mapped pose, never the held command (direct=${direct})`,()=>{
  const h=harness({direct});
  for(const q of [.2,.6,.1])h.apply({frameId:4,hand:{label:'Left',world:[]},expected:'Left',timestamp_ms:100,
    requestStartedAt:105,inferredAt:100},{tracking:true,solve_ms:1,direct_mode:direct,
    pose:{q},command_joint_angles_rad:{q:0},retarget_joint_angles_rad:{q},hardware:{armed:false}});
  assert.deepEqual(h.poses.map(p=>p.q),[.2,.6,.1]);
});
test('A32 armed direct preview uses the command pose while late results from an old mode are ignored',()=>{
  const h=harness({direct:true});
  const frame={frameId:4,hand:{label:'Left',world:[]},expected:'Left',timestamp_ms:100,requestStartedAt:105,inferredAt:100};
  h.apply(frame,{tracking:true,solve_ms:1,direct_mode:true,pose:{q:.9},command_joint_angles_rad:{q:.7},hardware:{armed:true}});
  h.apply(frame,{tracking:true,solve_ms:1,direct_mode:false,pose:{q:.2},hardware:{armed:false}});
  assert.deepEqual(h.poses.map(p=>p.q),[.7]);
});
