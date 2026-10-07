import test from 'node:test';
import assert from 'node:assert/strict';
import {WorkerRecorder} from '../static/worker-recorder.js';

function setup(onError=()=>{}){
  const messages=[];
  const worker={postMessage(m){messages.push(structuredClone(m));}};
  const r=new WorkerRecorder(onError,{makeWorker:()=>worker});
  const reply=(m,result={})=>worker.onmessage({data:{request:m.request,result}});
  async function start(){const p=r.start({model_id:'TsenseI_v3_0'});await Promise.resolve();
    reply(messages.at(-1),{id:'test',info:{id:'test'},seq:1});await p;}
  return {r,worker,messages,reply,start};
}

test('source sequence and time remain synchronous while worker receives immutable snapshots',async()=>{
  const h=setup();await h.start();
  const data={frame_id:3,joints:{a:1}};
  assert.equal(h.r.record('downlink',data),2);data.joints.a=9;
  const m=h.messages.at(-1);assert.equal(m.data.joints.a,1);assert(m.t>=0);
  assert.equal(h.r.record('h7',{frame_id:3}),3);
  const stop=h.r.stop('done',{exportFile:false});
  assert.equal(h.r.record('late',{}),null);
  const seal=h.messages.at(-1);assert.equal(seal.method,'stop');
  assert.equal(seal.args[1].exportFile,false);h.reply(seal);await stop;
  assert(!h.messages.some(m=>m.method==='export'));
  const exportLater=h.r.stop('download');await Promise.resolve();
  h.reply(h.messages.at(-1),{name:'test.jsonl'});
  assert.deepEqual(await exportLater,{name:'test.jsonl'});
});

test('stalled logging has a finite queue and cannot enqueue unbounded control copies',async()=>{
  let error;const h=setup(e=>{error=e;});await h.start();
  for(let i=0;i<2100;i++)h.r.record('downlink',{frame_id:i});
  assert.equal(h.messages.filter(m=>m.method==='record').length,2000);
  assert.match(error.message,/积压/);
  const stop=h.r.stop('recording_error',{exportFile:false});
  const m=h.messages.at(-1);assert.equal(m.recordingError,error.message);h.reply(m);await stop;
  assert.equal(m.args[1].sourceDropped,1);
});

test('worker progress releases bounded queue capacity',async()=>{
  const h=setup(()=>assert.fail());await h.start();
  for(let i=0;i<4000;i++){
    h.r.record('h7',{frame_id:i});
    if(i%50===49)h.worker.onmessage({data:{event:'progress',accepted:i+1,bytes:i*100}});
  }
  assert.equal(h.r.sent,4000);assert.equal(h.r.accepted,4000);
});

test('worker termination rejects pending storage operations and reports an error once',async()=>{
  let errors=0;const h=setup(()=>errors++);await h.start();
  const pending=h.r.flush();h.worker.onerror({message:'worker failed'});
  await assert.rejects(pending,/worker failed/);
  await assert.rejects(h.r.flush(),/worker failed/);assert.equal(errors,1);
});

test('concurrent starts share one session and stop waits for its initialization',async()=>{
  const h=setup(),first=h.r.start({}),second=h.r.start({});
  const stopped=h.r.stop('cancel_start',{exportFile:false});
  await Promise.resolve();
  assert.equal(first,second);assert.equal(h.messages.length,1);
  h.reply(h.messages[0],{id:'started',info:{id:'started'},seq:1});
  await first;await Promise.resolve();
  assert.equal(h.messages.length,2);assert.equal(h.messages[1].method,'stop');
  h.reply(h.messages[1]);await stopped;assert.equal(h.r.active,false);
});
