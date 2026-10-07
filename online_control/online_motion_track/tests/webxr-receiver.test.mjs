import test from 'node:test';
import assert from 'node:assert/strict';
import {WebXRReceiver, questStatus} from '../static/webxr-receiver.js';

function harness(){
  let time=100,delay=0;
  const frames=[],states=[],errors=[];
  const data={revision:1,age_ms:10,connected:true,hand:{label:'Left'}};
  const receiver=new WebXRReceiver({session:'session',now:()=>time,
    fetcher:async()=>{time+=delay;return {ok:true,json:async()=>({...data})};},
    onFrame:s=>frames.push(s),onStatus:s=>states.push(s),onError:e=>errors.push(e)});
  const poll=async()=>{await receiver.poll();clearTimeout(receiver.timer);};
  return {receiver,frames,states,errors,data,poll,advance:ms=>{time+=ms;},delay:ms=>{delay=ms;}};
}
test('each fresh revision is consumed once; duplicate polls never renew the hand',async()=>{
  const h=harness();await h.poll();await h.poll();
  assert.equal(h.frames.length,1);assert.equal(h.frames[0].hand.label,'Left');
  h.data.revision++;await h.poll();assert.equal(h.frames.length,2);h.receiver.close();
});
test('server age plus request time expires a frame without synchronizing device clocks',async()=>{
  const h=harness();h.data.age_ms=300;h.delay(60);await h.poll();
  assert.equal(h.frames[0].hand,null);assert.equal(h.frames[0].age_ms,360);h.receiver.close();
});
test('a previously consumed revision becomes loss; loss heartbeats remain bounded',async()=>{
  const h=harness();await h.poll();h.data.age_ms=400;await h.poll();await h.poll();
  assert.equal(h.frames.length,2);assert.equal(h.frames[1].hand,null);
  h.advance(101);await h.poll();assert.equal(h.frames.length,3);h.receiver.close();
});
test('closed receiver ignores a late network response',async()=>{
  const h=harness();let resolve;
  h.receiver.fetcher=()=>new Promise(r=>{resolve=r;});
  const pending=h.poll();h.receiver.close();resolve({ok:true,json:async()=>h.data});await pending;
  assert.equal(h.frames.length,0);assert.equal(h.states.length,0);
});
test('HTTP errors close the input and report once',async()=>{
  const h=harness();h.receiver.fetcher=async()=>({ok:false,json:async()=>({error:'会话已停止'})});
  await h.poll();await h.poll();assert.equal(h.errors.length,1);assert.equal(h.frames.length,0);
});

test('a slow request invalidates input, keeps pairing and resumes only a new revision',async()=>{
  const h=harness();await h.poll();
  const fetcher=h.receiver.fetcher;
  h.receiver.staleAfterMs=15;
  h.receiver.fetcher=(_url,{signal})=>new Promise((_resolve,reject)=>{
    signal.addEventListener('abort',()=>reject(new DOMException('aborted','AbortError')),{once:true});
  });
  await h.poll();
  assert.equal(h.receiver.closed,false);assert.equal(h.errors.length,0);
  assert.equal(h.frames.at(-1).hand,null);assert.equal(h.frames.at(-1).revision,-1);
  assert.equal(h.states.at(-1).connection_error,'timeout');
  h.receiver.staleAfterMs=350;h.receiver.fetcher=fetcher;
  await h.poll();assert.equal(h.frames.at(-1).hand,null,'old revision must not revive input');
  h.data.revision++;await h.poll();assert.equal(h.frames.at(-1).hand.label,'Left');
  h.receiver.close();
});

test('temporary HTTP failures and offline transport retry but expired sessions stop',async()=>{
  for(const status of [408,429,503]){
    const h=harness();
    h.receiver.fetcher=async()=>({ok:false,status,json:async()=>({error:'temporary'})});
    await h.poll();assert.equal(h.receiver.closed,false);assert.equal(h.frames.at(-1).hand,null);
    h.receiver.fetcher=async()=>{throw new TypeError('Failed to fetch');};
    await h.poll();assert.equal(h.receiver.closed,false);
    h.receiver.fetcher=async()=>({ok:false,status:400,json:async()=>({error:'session expired'})});
    await h.poll();assert.equal(h.receiver.closed,true);assert.equal(h.errors.length,1);
  }
});

test('status distinguishes unpaired, ended XR, hidden XR, stale input and missing selected hand',()=>{
  const status=(event,extra={})=>questStatus({connected:true,hand:null,age_ms:10,
    source_metadata:{event,stale_after_ms:350},...extra});
  assert.match(status('waiting'),/配对/);
  assert.match(status('session_ended'),/XR 已结束/);
  assert.match(status('tracking_lost',{raw_webxr:{reason:'xr_not_visible'}}),/前台/);
  assert.match(status('hand_skeleton',{age_ms:400}),/过期/);
  assert.match(status('hand_skeleton'),/未检测到所选手/);
});

test('background results acknowledge solve revisions only, including loss and recovery',async()=>{
  const requests=[],frames=[];
  let snapshot={revision:10,update_revision:0,update:null,connected:true,age_ms:5,hand:{label:'Left'}};
  const receiver=new WebXRReceiver({session:'background',combined:true,background:true,now:()=>100,
    fetcher:async(_url,options)=>{requests.push(JSON.parse(options.body));return {ok:true,json:async()=>snapshot};},
    onFrame:f=>frames.push(f),onStatus:()=>{},onError:e=>{throw e;}});
  const poll=async()=>{await receiver.poll();clearTimeout(receiver.timer);};
  await poll(); // Observation exists but has not finished solving.
  assert.equal(frames.length,0);
  snapshot={...snapshot,update_revision:1,update:{sequence:7,tracking:true}};
  await poll();await poll(); // Duplicate solve must not redraw or renew control.
  assert.equal(frames.length,1);assert.equal(frames[0].combined_frame,true);
  snapshot={...snapshot,revision:11,update:null};await poll();
  snapshot={...snapshot,update_revision:2,update:{sequence:8,tracking:false},hand:null};await poll();
  snapshot={...snapshot,revision:12,update_revision:3,update:{sequence:9,tracking:true},hand:{label:'Left'}};await poll();
  assert.deepEqual(frames.map(f=>f.update?.sequence),[7,8,9]);
  assert.deepEqual(requests.map(r=>r.after_revision),[-1,-1,1,1,1,2]);
  assert(requests.every(r=>!('hand' in r)&&!('raw_webxr' in r)));
  receiver.close();
});

test('late combined solve preserves raw result but cannot report active tracking',async()=>{
  const frames=[];
  const receiver=new WebXRReceiver({session:'background',combined:true,background:true,
    fetcher:async()=>({ok:true,json:async()=>({revision:1,update_revision:1,connected:true,age_ms:400,
      hand:{label:'Left'},update:{sequence:1,tracking:true,retarget_joint_angles_rad:{q:.1}}})}),
    onFrame:f=>frames.push(f),onStatus:()=>{},onError:e=>{throw e;}});
  await receiver.poll();receiver.close();
  assert.equal(frames[0].hand,null);assert.equal(frames[0].update.tracking,false);
  assert.equal(frames[0].update.display_discarded,true);
  assert.equal(frames[0].update.retarget_joint_angles_rad.q,.1);
});

test('poll cadence includes transport time and retains the independent 350 ms deadline',async t=>{
  const delays=[];
  t.mock.method(globalThis,'setTimeout',(_fn,delay)=>{delays.push(delay);return 1;});
  t.mock.method(globalThis,'clearTimeout',()=>{});
  const h=harness();h.delay(10);await h.poll();
  assert.deepEqual(delays,[350,6]);
  delays.length=0;h.delay(20);await h.poll();
  assert.deepEqual(delays,[350,0]);h.receiver.close();
});
