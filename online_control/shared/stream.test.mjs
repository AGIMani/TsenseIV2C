import test from 'node:test';
import assert from 'node:assert/strict';
import {consumeRecords} from './stream.js';
import {A30BrowserHandControl} from '../online_motion_track/static/a30-browser-hand-control.js';
import * as protocol from './a30-serial-protocol.js';

function framed(bytes){const b=new Uint8Array(4+bytes.length);new DataView(b.buffer).setUint32(0,bytes.length,true);b.set(bytes,4);return b;}
test('fragmented network chunks preserve every valid tracking sample',async()=>{
  const all=new Uint8Array([...framed(new Uint8Array([1,2])),...framed(new Uint8Array([3]))]);
  const abort=new AbortController(),seen=[];
  const response=new Response(new ReadableStream({start(c){c.enqueue(all.slice(0,3));c.enqueue(all.slice(3));}}));
  await consumeRecords(response,b=>{seen.push([...b]);if(seen.length===2)abort.abort();},{signal:abort.signal});
  assert.deepEqual(seen,[[1,2],[3]]);
});
test('invalid record length fails before writing',async()=>{
  const response=new Response(new ReadableStream({start(c){c.enqueue(new Uint8Array([255,255,255,255]));c.close();}}));
  await assert.rejects(consumeRecords(response,()=>assert.fail()),/长度/);
});
function control(){
  const c=new A30BrowserHandControl({now:()=>1000});c.side='left';c.connected=c.armed=true;
  c.epoch=1;c.lastSample=-1;c.lastSentAt=999;c.handAt=c.telemetryAt=1000;
  c.clockAnchor={server:10,local:1000};c.freshTelemetry=()=>true;c.accepted=true;
  c.held={};c.limits={};
  const linear={},rotary={};
  for(const n of protocol.LINEAR_NAMES)linear['left_hand_'+n]=0;
  for(const n of protocol.ROTARY_NAMES)rotary['left_hand_'+n]=0;
  const names=[...Object.keys(linear),...Object.keys(rotary)];
  for(const n of names){c.held[n]=0;c.limits[n]=[-1,1];}
  const wire=protocol.positions(0,linear,rotary,'left'),bytes=new Uint8Array(108+wire.length),v=new DataView(bytes.buffer);
  bytes.set(new TextEncoder().encode('GTR1'));v.setUint32(4,1,true);v.setUint32(8,5,true);v.setFloat64(16,10,true);v.setUint32(24,1,true);bytes.set(wire,108);
  let writes=0;c.writer={write:async()=>{writes++;}};
  return {c,bytes,v,names,writes:()=>writes};
}
test('tracking streams exact server bytes without the old 20ms gate',async()=>{
  const x=control();await x.c.streamSample(x.bytes,x.names,x.c.job);assert.equal(x.writes(),1);
  await x.c.streamSample(x.bytes,x.names,x.c.job);assert.equal(x.writes(),1);
});
test('tracking rejects stale, corrupt and out-of-range streams before USB',async()=>{
  for(const change of [x=>x.v.setFloat64(16,9,true),x=>x.bytes[110]^=1,x=>x.v.setFloat32(28,4,true)]){
    const x=control();change(x);await assert.rejects(x.c.streamSample(x.bytes,x.names,x.c.job));assert.equal(x.writes(),0);
  }
});
