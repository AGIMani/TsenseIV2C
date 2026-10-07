import test from 'node:test';
import assert from 'node:assert/strict';
import {A30PreviewStream} from '../static/a30-preview-stream.js';

test('ongoing preview refreshes its clock anchor after a slow status response',async(t)=>{
  let output;
  const stream=new ReadableStream({start:c=>output=c});
  t.mock.method(globalThis,'fetch',async()=>new Response(stream));
  const p=new A30PreviewStream({now:()=>2000});
  const state={browser_epoch:2,server_time:1,control_joint_names:Array.from({length:20},(_,i)=>`joint${i}`)};
  p.start('session',state,800); // First status was already delayed by 200 ms.
  const active=p.abort;
  p.start('session',{...state,server_time:2},2000);
  assert.equal(p.abort,active); // Clock refresh must not open a second stream.
  const b=new Uint8Array(112),v=new DataView(b.buffer);
  v.setUint32(0,108,true);b.set([71,84,82,49],4);
  v.setUint32(8,2,true);v.setUint32(12,4,true);v.setFloat64(20,2,true);v.setUint32(28,1,true);
  output.enqueue(b);
  try{
    await new Promise(r=>setImmediate(r));
    assert.equal(p.latest?.tick,4);
  }finally{p.stop();await p.task;}
});

test('preview accepts newest fresh checked sample and rejects stale epochs and faults',()=>{
  const p=new A30PreviewStream({now:()=>1000});
  const state={browser_epoch:2,server_time:1,control_joint_names:Array.from({length:20},(_,i)=>`joint${i}`)};
  const packet=(tick,{epoch=2,at=1,flags=1}={})=>{
    const b=new Uint8Array(108),v=new DataView(b.buffer);b.set([71,84,82,49]);
    v.setUint32(4,epoch,true);v.setUint32(8,tick,true);v.setFloat64(16,at,true);v.setUint32(24,flags,true);
    v.setFloat32(28,.25,true);return b;
  };
  p.accept(packet(3),state,1000);
  assert.equal(p.latest.tick,3);assert.equal(p.latest.pose.joint0,.25);
  for(const b of [packet(2),packet(4,{epoch:1}),packet(4,{at:.8}),packet(4,{flags:2}),packet(4,{flags:4})])
    p.accept(b,state,1000);
  assert.equal(p.latest.tick,3);
  p.accept(packet(5),state,1000);assert.equal(p.latest.tick,5);
  p.stop();assert.equal(p.latest,null);
});

test('preview refuses a downlink payload and nonfinite angles',()=>{
  const p=new A30PreviewStream();
  assert.throws(()=>p.accept(new Uint8Array(216),{},0),/格式/);
  const b=new Uint8Array(108),v=new DataView(b.buffer);b.set([71,84,82,49]);
  v.setUint32(24,1,true);v.setFloat32(28,NaN,true);
  assert.throws(()=>p.accept(b,{browser_epoch:0,server_time:0,control_joint_names:Array(20).fill('joint')},performance.now()),/角度/);
});
