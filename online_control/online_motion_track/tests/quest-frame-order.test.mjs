import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import vm from 'node:vm';

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
    JOINT_SET:'webxr-hand-25',JOINT_NAMES:[],updateStatus:()=>{},finish:()=>assert.fail('unexpected XR failure')};
  const frameLoop=vm.runInNewContext(source+';frameLoop;',context);
  frameLoop(c,100,{getViewerPose:()=>({views:[]})});
  assert.deepEqual(events,['collect','tracking_restored','tracking_lost','hand_skeleton','draw','schedule']);
  assert.equal(published.at(-1).hands,hands);
  context.current=null;frameLoop(c,110,{});assert.equal(events.length,6);
});
