import test from 'node:test';
import assert from 'node:assert/strict';
import {makePacer,intervalSummary} from './serial-timing.js';
import {SerialTrace} from '../online_motion_track/static/serial-trace.js';

test('closing the pacer releases all waits, including a timer already started',async()=>{
  const pacer=makePacer();
  const waits=[pacer.wait(10000),pacer.wait(10000)];
  await new Promise(resolve=>setTimeout(resolve,10));
  pacer.close();
  await Promise.all(waits);
  await pacer.wait(10000);
});

test('interval statistics expose jitter instead of reporting only mean Hz',()=>{
  assert.deepEqual(intervalSummary([]),{samples:0});
  assert.deepEqual(intervalSummary([.1,1,2,3,8]),{
    samples:5,min_ms:.1,max_ms:8,p50_ms:2,p95_ms:8,p99_ms:8,within_1_3_ms_percent:60});
});

test('serial intervals count only successful position writes, not power commands',()=>{
  let now=0;
  const trace=new SerialTrace({emit(){},now:()=>now});
  const position=new Uint8Array([0xaa,0x55,0,0,3,0x20]);
  const power=new Uint8Array([0xaa,0x55,0,0,3,0x21]);
  for(const [start,bytes,sent] of [[0,position,true],[1,power,true],[2,position,true],[3,position,false],[5,position,true]]){
    now=start+.4;trace.written(bytes,start,sent);
  }
  assert.deepEqual(trace.summary().position_write_intervals,{
    samples:2,min_ms:2,max_ms:3,p50_ms:2,p95_ms:3,p99_ms:3,within_1_3_ms_percent:100});
});
