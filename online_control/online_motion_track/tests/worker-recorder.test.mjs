import test from 'node:test';
import assert from 'node:assert/strict';
import {WorkerRecorder,jointAngleRecords} from '../static/worker-recorder.js';

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

test('five angle stages retain their own samples and reject invalid or unsuccessful SDK output',()=>{
  for(const stage of ['retarget','direct','ruckig','mink','sdk']) {
    const rows=jointAngleRecords('joint_angles',{stage,joints:{q:.3},sent:true,frame_id:7,landmarks:[1,2,3]});
    assert.equal(rows.length,1);assert.equal(rows[0].stage,stage);assert.equal(rows[0].joints.q,.3);
    assert.equal(rows[0].frame_id,7);assert.equal(rows[0].landmarks,undefined);
  }
  for(const joints of [undefined,{},[],{q:null},{q:NaN}])
    assert.deepEqual(jointAngleRecords('retarget',{joints}),[]);
  assert.deepEqual(jointAngleRecords('joint_angles',{stage:'sdk',joints:{q:1},sent:false}),[]);
  assert.deepEqual(jointAngleRecords('joint_angles',{stage:'candidate',joints:{q:1}}),[]);
  assert.deepEqual(jointAngleRecords('a32_sdk_sample',{joints:{q:1},sent:true}),[]);
  assert.deepEqual(jointAngleRecords('h7',{joints_ui:{q:1},sent:false}),[]);
});

test('actual planner output stays separate from collision-adjusted successful SDK angles',async()=>{
  const h=setup();await h.start();
  h.r.record('h7',{joints_ui:{q:.2},joints_ruckig_ui:{q:.2},ruckig_output_joint_angles_rad:{q:.4},
    sent:true,frame_id:4,control_dt_s:.002});
  const messages=h.messages.filter(m=>m.method==='record');
  assert.deepEqual(messages.map(m=>m.type),['h7','joint_angles','joint_angles']);
  assert.equal(messages[0].data.joints_ui,undefined);assert.equal(messages[0].data.control_dt_s,.002);
  assert.equal(messages[1].data.stage,'ruckig');assert.equal(messages[1].data.joints.q,.4);
  assert.equal(messages[2].data.stage,'sdk');assert.equal(messages[2].data.joints.q,.2);
  assert.deepEqual(jointAngleRecords('h7',{joints_ui:{q:.2},joints_ruckig_ui:{q:.2},sent:true})
    .map(r=>r.stage),['sdk'],'a final pose must not be relabeled as a raw Ruckig output');
});

test('temperature, currents, raw conversion and transport data still reach storage',async()=>{
  const h=setup();await h.start();
  for(const type of ['telemetry','board_telemetry','conversion','a32_conversion','a32_sdk_sample','downlink']) {
    const data={temperature:42,current:1.3,linear_mm:{q:2.1},joints_hw:{q:.5},wire_hex:'abcd'};
    h.r.record(type,data);
    const m=h.messages.at(-1);assert.equal(m.type,type);assert.deepEqual(m.data,data);
  }
});

test('planner polling deduplicates ticks and batched SDK samples retain actual write times',async()=>{
  const h=setup();await h.start();
  h.r.record('ruckig',{joints:{q:.1},sample_ticks:3,source:'status'});
  h.r.record('ruckig',{joints:{q:.1},sample_ticks:3,source:'status'});
  h.r.record('ruckig',{joints:{q:.2},sample_ticks:4,source:'status'});
  assert.equal(h.messages.filter(m=>m.type==='joint_angles').length,2);
  for(const [tick,ms] of [[10,20],[11,22]])
    h.r.record('h7',{sent:true,joints_ui:{q:tick/100},sample_ticks:tick,sent_at_unix_ms:h.r.wallStart+ms});
  const sdk=h.messages.filter(m=>m.type==='joint_angles'&&m.data.stage==='sdk');
  assert.deepEqual(sdk.map(m=>m.t),[.02,.022]);
  assert.deepEqual(h.messages.filter(m=>m.type==='h7').map(m=>m.t),[.02,.022]);
  assert.equal(sdk[1].data.sent_at_unix_ms-sdk[0].data.sent_at_unix_ms,2);
  h.r.record('h7',{sent:true,joints_ui:{q:.2},written_at:h.r.t0+35});
  assert.equal(h.messages.at(-1).t,.035);
  assert.equal(h.messages.at(-2).type,'h7');assert.equal(h.messages.at(-2).t,.035);
  h.r.record('downlink',{sent:true,linear_mm:[1],written_at:h.r.t0+37});
  assert.equal(h.messages.at(-1).t,.037);
});

test('retarget diagnostics and filter parameters survive removal of duplicate angle arrays',async()=>{
  const h=setup();await h.start();
  h.r.record('retarget',{joints:{q:.2},solve_ms:5,tracking:false,message:'collision held'});
  const rows=h.messages.filter(m=>m.method==='record');
  assert.equal(rows[0].type,'retarget');assert.equal(rows[0].data.joints,undefined);
  assert.equal(rows[0].data.solve_ms,5);assert.equal(rows[0].data.message,'collision held');
  assert.equal(rows[1].type,'joint_angles');assert.equal(rows[1].data.joints.q,.2);
  h.r.record('direct_filter_detail',{tau:.01,input_joint_angles_rad:{q:.3},filtered_joint_angles_rad:{q:.2},checked_joint_angles_rad:{q:.1}});
  assert.deepEqual(h.messages.at(-1).data,{tau:.01});
});
