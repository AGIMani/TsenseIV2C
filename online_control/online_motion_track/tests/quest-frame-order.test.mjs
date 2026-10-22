import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import vm from 'node:vm';
import {collectHands, toAdbHand} from '../static/quest-capture-core.js';

const text=readFileSync(new URL('../static/quest-capture.js',import.meta.url),'utf8');
const source=text.slice(text.indexOf('function frameLoop('),text.indexOf('\nasync function start'));
test('validated Quest observations publish before stereo drawing and retain loss ordering',()=>{
  const events=[],published=[];
  const hands=[{handedness:'left',tracked:true},{handedness:'right',tracked:false}];
  const c={tracked:{left:false,right:true},frames:0,referenceSpace:{},
    session:{inputSources:[],visibilityState:'visible',requestAnimationFrame:()=>events.push('schedule')},
    renderer:{draw:()=>events.push('draw')},publisher:{stats:{},offer:m=>{events.push(m.type);published.push(m);}}};
  const context={current:c,syncCaptureVisibility:()=>false,
    collectHands:()=>{events.push('collect');return hands;},message:(_c,type,fields)=>({type,...fields}),
    adbMode:false,skeletonFormat:{joint_set:'webxr-hand-25',joint_names:[]},
    updateStatus:()=>{},finish:()=>assert.fail('unexpected XR failure')};
  const frameLoop=vm.runInNewContext(source+';frameLoop;',context);
  frameLoop(c,100,{getViewerPose:()=>({views:[]})});
  assert.deepEqual(events,['collect','tracking_restored','tracking_lost','hand_skeleton','draw','schedule']);
  assert.equal(published.at(-1).hands,hands);
  context.current=null;frameLoop(c,110,{});assert.equal(events.length,6);
});

test('USB ADB publishes 21 valid joints before rendering native25 and reports recovery once',()=>{
  const events=[],published=[];
  const native=collectHands({getJointPose:name=>name==='index-finger-metacarpal'?null:{
    transform:{position:{x:.1,y:.2,z:-.5},orientation:{x:0,y:0,z:0,w:1}},
  }},[{handedness:'left',hand:{get:name=>name}}],{});
  let drawn;
  const c={tracked:{left:false,right:false},frames:0,referenceSpace:{},
    session:{inputSources:[],visibilityState:'visible',requestAnimationFrame:()=>events.push('schedule')},
    renderer:{draw:(_c,_pose,hands)=>{drawn=hands;events.push('draw');}},
    publisher:{stats:{},offer:m=>{events.push(m.type);published.push(m);}}};
  const context={current:c,syncCaptureVisibility:()=>false,collectHands:()=>native,toAdbHand,
    message:(_c,type,fields)=>({type,...fields}),adbMode:true,
    skeletonFormat:{joint_set:'mediapipe-21',units:'m'},updateStatus:()=>{},
    finish:()=>assert.fail('unexpected XR failure')};
  const frameLoop=vm.runInNewContext(source+';frameLoop;',context);
  frameLoop(c,100,{getViewerPose:()=>({views:[]})});
  assert.deepEqual(events,['tracking_restored','hand_skeleton','draw','schedule']);
  assert.equal(published[0].reason,'all_21_joints_valid');
  assert.equal(published[1].hands[0].landmarks.length,21);
  assert.equal(published[1].hands[0].tracked,true);
  assert.equal(published[1].units,'m');
  assert.equal(drawn,native);
  assert.equal(drawn[0].joints.length,25);
  assert.equal(drawn[0].tracked,false);
  frameLoop(c,110,{getViewerPose:()=>({views:[]})});
  assert.equal(published.filter(item=>item.type==='tracking_restored').length,1);
});
