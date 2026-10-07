import test from 'node:test';
import assert from 'node:assert/strict';
import {LatestFrameQueue} from '../static/latest-frame-queue.js';

const flush = () => new Promise(resolve => setImmediate(resolve));
const frame = (id, timestamp=0, hand={label:'Left'}) => ({id,packet:{timestamp_ms:timestamp,hand}});
function harness() {
  const calls=[],results=[],errors=[],releases=[],drops=[];
  let now=0,inFlight=0,peak=0;
  const queue=new LatestFrameQueue({now:()=>now,
    process:f=>{calls.push(f);peak=Math.max(peak,++inFlight);return new Promise((resolve,reject)=>
      releases.push({resolve:()=>{inFlight--;resolve(f.id);},reject:e=>{inFlight--;reject(e);}}));},
    onResult:(f,r)=>results.push(r),onError:e=>errors.push(e),onDrop:(f,reason)=>drops.push([f.id,reason])});
  return {queue,calls,results,errors,releases,drops,setNow:v=>now=v,peak:()=>peak};
}

test('one request at a time; pending targets collapse to the latest', async()=>{
  const h=harness();
  h.queue.offer(frame(1));h.queue.offer(frame(2));h.queue.offer(frame(3));
  assert.deepEqual(h.calls.map(f=>f.id),[1]);
  h.releases[0].resolve();await flush();
  assert.deepEqual(h.calls.map(f=>f.id),[1,3]);
  h.releases[1].resolve();await flush();
  assert.deepEqual(h.results,[1,3]);assert.equal(h.peak(),1);
  assert.deepEqual(h.queue.stats,{offered:3,processed:2,replaced:1,expired:0});
  assert.deepEqual(h.drops,[[2,'queue_replaced']]);
});

test('a current no-hand observation replaces a pending hand target', async()=>{
  const h=harness();h.queue.offer(frame(1));h.queue.offer(frame(2));h.queue.offer(frame(3,0,null));
  h.releases[0].resolve();await flush();
  assert.equal(h.calls[1].packet.hand,null);
  h.releases[1].resolve();await flush();
});

test('pending frames that have become stale are skipped without replay', async()=>{
  const h=harness();h.queue.offer(frame(1));h.queue.offer(frame(2));
  h.setNow(121);h.releases[0].resolve();await flush();
  assert.equal(h.calls.length,1);assert.equal(h.queue.stats.expired,1);
  assert.deepEqual(h.drops,[[2,'queue_expired']]);
  h.queue.offer(frame(3,121));assert.equal(h.calls[1].id,3);
  h.releases[1].resolve();await flush();
});

test('stop suppresses late responses and queued work; a new session is independent', async()=>{
  const old=harness();old.queue.offer(frame(1));old.queue.offer(frame(2));old.queue.close();
  const fresh=harness();fresh.queue.offer(frame(100));
  old.releases[0].resolve();fresh.releases[0].resolve();await flush();
  old.queue.offer(frame(3));
  assert.deepEqual(old.results,[]);assert.equal(old.calls.length,1);
  assert.deepEqual(fresh.results,[100]);
  old.queue.close();assert.deepEqual(old.drops,[[2,'queue_closed'],[1,'request_cancelled']]);
});

test('request failure closes the queue and reports once', async()=>{
  const h=harness();h.queue.offer(frame(1));h.queue.offer(frame(2));
  h.releases[0].reject(new Error('offline'));await flush();h.queue.offer(frame(3));
  assert.equal(h.queue.closed,true);assert.equal(h.calls.length,1);
  assert.equal(h.errors.length,1);assert.equal(h.errors[0].message,'offline');
  assert.deepEqual(h.drops,[[1,'request_failed'],[2,'queue_closed']]);
});

test('a rendering failure also stops delivery', async()=>{
  let errors=0;
  const q=new LatestFrameQueue({process:async()=>({}),onResult:()=>{throw Error('render');},
    onError:()=>errors++});
  q.offer(frame(1,performance.now()));await flush();
  assert.equal(q.closed,true);assert.equal(errors,1);
});
