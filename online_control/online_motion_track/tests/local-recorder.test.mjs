import test from 'node:test';
import assert from 'node:assert/strict';
import {LocalRecorder} from '../static/local-recorder.js';

test('capacity failure seals without loading/exporting the recording; later stop exports once',async()=>{
  let sealing,exports=0;const rows=[];
  const recorder=new LocalRecorder(()=>{sealing=recorder.stop('recording_error',{exportFile:false});});
  Object.assign(recorder,{active:true,bytes:512*1024*1024,id:'fixture',seq:0,t0:performance.now(),
    info:{id:'fixture'},db:{transaction(){
      const tx={objectStore:()=>({put:()=>{}})};queueMicrotask(()=>tx.oncomplete?.());return tx;
    }}});
  recorder.flush=async()=>{rows.push(...recorder.pending.map(JSON.parse));recorder.pending=[];};
  recorder.export=async id=>{exports++;assert.equal(id,'fixture');return {name:'test.jsonl'};};
  recorder.record('downlink',{sent:true});
  await sealing;
  assert(!recorder.active&&recorder.error);assert.equal(exports,0);
  assert.equal(rows.filter(r=>r.type==='session_end').length,1);
  assert.equal(rows.at(-1).reason,'recording_error');
  assert.deepEqual(await recorder.stop('control_closed'),{name:'test.jsonl'});
  assert.equal(exports,1);assert.equal(await recorder.stop('again'),null);
});

test('ordinary stop still exports and does not trigger a capacity callback for session_end',async()=>{
  const recorder=new LocalRecorder(()=>assert.fail('session_end must not recursively stop recording'));
  Object.assign(recorder,{active:true,bytes:512*1024*1024,id:'fixture',seq:0,t0:performance.now(),
    info:{id:'fixture'},db:{transaction(){
      const tx={objectStore:()=>({put:()=>{}})};queueMicrotask(()=>tx.oncomplete?.());return tx;
    }}});
  recorder.flush=async()=>{};recorder.export=async()=>({name:'normal.jsonl'});
  assert.deepEqual(await recorder.stop(),{name:'normal.jsonl'});
  assert.equal(recorder.info.ended,true);
});
