// Execute the production metric ingress without a camera, browser or hardware.
import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import vm from 'node:vm';

const app=readFileSync(new URL('../static/app.js',import.meta.url),'utf8');
const source=app.slice(app.indexOf('function offerQuestFrame'),app.indexOf('function configureInputSource'));
function harness(){
  const uploaded=[],applied=[],observed=[],records=[];
  const display={value:'left',classList:{remove:()=>{}},textContent:'',title:''};
  const context={active:true,generation:2,session:{session:'synthetic'},sequence:0,
    performance:{now:()=>100},Date,$:()=>display,isControlModel:()=>true,latestWorld:[],skeleton:{setPoints:()=>{}},
    localControl:{observeHand:(...args)=>observed.push(args)},
    recorder:{record:(...args)=>records.push(args)},framePipeline:{offer:f=>uploaded.push(f)},
    applyFrameUpdate:(...args)=>applied.push(args)};
  return {context,uploaded,applied,observed,records,
    offer:vm.runInNewContext(source+';offerQuestFrame;',context)};
}
for(const source of ['webxr','wuji_glove','leapmotion']){
  test(`${source} combined result is applied without reuploading the skeleton`,()=>{
    const h=harness(),update={sequence:42,tracking:true,source_age_at_solve_ms:9};
    h.offer(2,{source,combined_frame:true,update,revision:7,update_revision:3,
      request_started_ms:80,age_ms:10,hand:{label:'Left',world:[]}});
    assert.equal(h.uploaded.length,0);assert.equal(h.applied.length,1);
    assert.equal(h.applied[0][0].frameId,42);assert.equal(h.applied[0][1],update);
    assert.equal(h.records.length,1);assert.equal(h.records[0][0],`${source}_raw`);
    h.offer(2,{source,combined_frame:true,update:null,hand:null,age_ms:400});
    assert.equal(h.uploaded.length,0);assert.equal(h.applied.length,1);
    assert.equal(h.observed.at(-1)[0],false,'loss immediately invalidates browser control');
    assert.equal(h.context.latestWorld,null,'loss immediately clears the tracked skeleton');
    assert.equal(h.records.at(-1)[1].frame_id,null,'unsolved loss must not reuse a server solve frame ID');
  });
}
test('legacy preview still submits selected observations, while a cancelled session ignores late results',()=>{
  const h=harness();h.offer(2,{source:'webxr',revision:4,hand:{label:'Left'},age_ms:5});
  assert.equal(h.uploaded.length,1);assert.equal(h.uploaded[0].packet.webxr_revision,4);
  h.context.generation=3;
  h.offer(2,{source:'webxr',combined_frame:true,update:{sequence:5},hand:{label:'Left'},age_ms:5});
  assert.equal(h.applied.length,0);assert.equal(h.records.length,1);
});
test('Leap observation revision, source epoch and native bones survive preview and raw recording',()=>{
  const h=harness(),raw={frame_id:512,bones:[{prev_joint:[1,2,3],next_joint:[4,5,6]}]};
  h.offer(2,{source:'leapmotion',revision:9,source_epoch:4,raw_leapmotion:raw,
    source_metadata:{source_epoch:4},hand:{label:'Left'},age_ms:5});
  const packet=h.uploaded[0].packet;
  assert.equal(packet.leapmotion_revision,9);assert.equal(packet.source_epoch,4);
  assert.equal(packet.raw_leapmotion,raw);assert(!('webxr_revision' in packet));
  assert.equal(h.records[0][0],'leapmotion_raw');
  assert.equal(h.records[0][1].raw_leapmotion,raw);assert.equal(h.records[0][1].source_epoch,4);
});
