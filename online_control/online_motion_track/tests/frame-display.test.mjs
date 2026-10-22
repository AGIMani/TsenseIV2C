import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import vm from 'node:vm';

const app=readFileSync(new URL('../static/app.js',import.meta.url),'utf8');
const source=app.slice(app.indexOf('function applyFrameUpdate'),app.indexOf('function offerQuestFrame'));
function harness({direct=false,a30=false,a32=true}={}){
  const poses=[],hardware=[],records=[],tracking=[],skeletons=[],elements=new Map();
  const context={recorder:{record:(type,data)=>records.push({type,data})},
    $:id=>{if(!elements.has(id))elements.set(id,{value:id==='side'?'left':'stop',
      classList:{toggle:(_name,value)=>tracking.push(value)}});return elements.get(id);},
    isA32Model:()=>false,isA30:()=>a30,isA32:()=>a32,isV2:()=>false,isControlModel:()=>false,
    directMode:()=>direct,showResponsePose:pose=>poses.push(pose),a32Panel:{update:value=>hardware.push(value)},
    renderHardware:value=>hardware.push(value),performance:{now:()=>120},latestWorld:[],
    skeleton:{setPoints:(points,side)=>skeletons.push({points,side})},fpsCount:0,fpsStart:0};
  return {poses,hardware,records,tracking,skeletons,elements,apply:vm.runInNewContext(source+';applyFrameUpdate;',context)};
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
  assert.deepEqual(h.skeletons,[]);
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
test('A30 distinguishes tracked left-hand input from a blocked model trajectory',()=>{
  const h=harness({a30:true,a32:false});
  h.apply({frameId:4,hand:{label:'Left',world:[]},expected:'Left',timestamp_ms:100,
    requestStartedAt:105,inferredAt:100,packet:{source:'wuji_glove'}},
    {tracking:true,solve_ms:1,direct_mode:false,pose:{q:0},optimized_points:Array.from({length:21},(_,i)=>[i*.01,0,0]),
      retarget_joint_angles_rad:{q:.5},hardware:{armed:false,
        tracking_transition:{phase:'blocked'},last_error:'控制采样超时'}});
  assert.equal(h.elements.get('tracking-text').textContent,'手势已识别 · 模型保持：控制采样超时');
  assert.equal(h.elements.get('tracking-text').title,'控制采样超时');
  assert.equal(h.elements.get('skeleton-status').textContent,'Wuji 21 点 · 优化后 · 左手镜像 · 实时');
  assert.equal(h.records[0].data.joints.q,.5);
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

test('skeleton renders optimized points, not raw input or the sampled command pose',()=>{
  const h=harness();
  const raw=Array.from({length:21},(_,i)=>[i*.01,0,0]);
  const optimized=raw.map(([x])=>[x,.02,.03]);
  h.apply({frameId:1,hand:{label:'Left',world:raw},expected:'Left',timestamp_ms:100},
    {tracking:true,solve_ms:1,pose:{q:0},retarget_joint_angles_rad:{q:.5},optimized_points:optimized});
  assert.equal(h.skeletons[0].points,optimized);
  assert.equal(h.skeletons[0].side,'Left');
  assert.match(h.elements.get('skeleton-status').textContent,/优化后/);
});

for(const optimized of [undefined,[],Array.from({length:21},()=>[NaN,0,0])])
  test(`missing/invalid optimized skeleton never falls back to raw points (${String(optimized)})`,()=>{
    const h=harness();
    h.apply({frameId:1,hand:{label:'Left',world:Array.from({length:21},()=>[1,2,3])},expected:'Left',timestamp_ms:100},
      {tracking:true,solve_ms:1,pose:{q:0},retarget_joint_angles_rad:{q:.5},optimized_points:optimized});
    assert.equal(h.skeletons[0].points,null);
    assert.equal(h.elements.get('skeleton-status').textContent,'等待优化结果');
  });

test('lost tracking clears optimized skeleton and an old mode response cannot replace it',()=>{
  const h=harness({direct:true});
  const frame={frameId:1,hand:null,expected:'Right',timestamp_ms:100};
  const optimized=Array.from({length:21},()=>[1,2,3]);
  h.apply(frame,{tracking:false,direct_mode:true,optimized_points:optimized,message:'lost'});
  assert.equal(h.skeletons[0].points,null);
  h.apply(frame,{tracking:true,direct_mode:false,solve_ms:1,retarget_joint_angles_rad:{q:.5},optimized_points:optimized});
  assert.equal(h.skeletons.length,1);
});
