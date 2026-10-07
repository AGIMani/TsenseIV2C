// Run: node --experimental-default-type=module --test test_browser_serial.mjs
// Memory streams only. No browser permission request or physical USB access.
import test from 'node:test';
import assert from 'node:assert/strict';
import {BrowserSerial,emergencyFrames,serialRequest,serialResponse} from './browser-serial.js';
import {frame,crc16} from './serial-protocol.js';
import {PositionGuard} from './position-guard.js';
import {consumeRecords} from './motion-stream.js';

const wait = ms => new Promise(resolve => setTimeout(resolve,ms));
async function until(condition) {
  const deadline = performance.now()+1800;
  while (!condition()) { if (performance.now()>deadline) throw Error('Timed out'); await wait(5); }
}
function port() {
  const p = {writes:[],closed:false,async open(options) {
    this.options = options;
    this.readable = new ReadableStream({start:c=>this.input=c});
    this.writable = new WritableStream({write:bytes=>this.writes.push(bytes.slice())});
  }, async setSignals(value) { this.signals=value; },async close() { this.closed=true; }};
  return p;
}
function frames(writes) {
  const result=[];
  for (const bytes of writes) for (let i=0;i<bytes.length;) {
    const size=8+bytes[i+2]+(bytes[i+3]<<8),data=bytes.slice(i,i+size);
    assert.equal(crc16(data.subarray(0,-2)),data.at(-2)|(data.at(-1)<<8));
    result.push(data);i+=size;
  }
  return result;
}
function api(exchange=()=>[]) {
  const calls=[];
  const fn=async(action,body)=>{
    calls.push([action,body]);
    if(action==='connect') return {result:{token:'test-token'}};
    if(action==='serial') return {result:{token:body.token,seq:body.seq,frames:await exchange(body)}};
    return {result:true};
  };
  fn.calls=calls;return fn;
}

test('visitor port owns USB: options, telemetry upload and exact command download',async t=>{
  const bytes=frame(0x20,new Uint8Array(100));
  const transport=api(body=>body.seq===1?[{data:Buffer.from(bytes).toString('base64'),age_ms:0}]:[]);
  const p=port(),link=new BrowserSerial(transport);t.after(()=>link.disconnect());
  await link.connect(p,'TsenseI_v3_0');p.input.enqueue(new Uint8Array([11,22,33]));
  await until(()=>transport.calls.some(([a,b])=>a==='serial'&&b.rx===Buffer.from([11,22,33]).toString('base64')));
  assert.deepEqual(p.writes[0],bytes);assert.equal(p.options.baudRate,115200);
  assert.deepEqual(p.signals,{dataTerminalReady:true,requestToSend:false});
  assert.deepEqual(transport.calls[0],['connect',{port:'browser'}]);
  await link.disconnect();assert.ok(p.closed);assert.equal(link.port,null);
});

test('late network commands are discarded; local A3 stop needs no server response',async t=>{
  const command=frame(0x20,new Uint8Array(100));
  const transport=api(async()=>{await wait(240);return [{data:Buffer.from(command).toString('base64'),age_ms:0}];});
  const p=port(),link=new BrowserSerial(transport);t.after(()=>link.disconnect());
  await link.connect(p,'TsenseI_v3_0');await until(()=>p.closed);await wait(100);
  const sent=frames(p.writes);assert.equal(sent.length,36);
  assert.ok(sent.every(f=>f[5]!==0x20&&f[8]===2));assert.equal(link.active,false);
});

test('old server frames and CRC errors close the local port',async t=>{
  for (const corrupt of [false,true]) {
    const bytes=frame(0x20,new Uint8Array(88));if(corrupt)bytes[10]^=1;
    const transport=api(()=>[{data:Buffer.from(bytes).toString('base64'),age_ms:corrupt?0:210}]);
    const p=port(),link=new BrowserSerial(transport);t.after(()=>link.disconnect());
    await link.connect(p,'TsenseI_v2');await until(()=>p.closed);
    assert.ok(frames(p.writes).every(f=>f[5]!==0x20));
    assert.match(link.error,corrupt?/校验/:/过期/);
  }
});

test('different models have different local homing cancellation opcodes',()=>{
  const a=emergencyFrames('TsenseI_v3_0'),v=emergencyFrames('TsenseI_v2');
  assert.equal(a.filter(f=>f[5]===0x22&&f[8]===2).length,18);
  assert.equal(a.filter(f=>f[5]===0x21&&f[8]===2).length,18);
  assert.equal(v.filter(f=>f[5]===0x12&&f[8]===2).length,14);
  assert.equal(v.filter(f=>f[5]===0x11&&f[8]===2).length,14);
  assert.equal(v.filter(f=>f[5]===0x22&&f[7]===0&&f[8]===0).length,4);
  frames([...a,...v]);
});

test('disconnect during asynchronous port opening releases the newly opened port',async()=>{
  const p=port(),open=p.open;p.open=async function(options){await wait(30);await open.call(this,options);};
  const transport=api(),link=new BrowserSerial(transport);
  const connecting=link.connect(p,'TsenseI_v2');
  const rejected=assert.rejects(connecting,/取消/);
  await link.disconnect();await rejected;
  assert.equal(p.closed,true);assert.equal(p.writes.length,0);
  assert.equal(transport.calls.length,0);
});

test('USB removal disconnects without reconnecting or replaying',async t=>{
  const transport=api(),p=port(),link=new BrowserSerial(transport);t.after(()=>link.disconnect());
  await link.connect(p,'TsenseI_v2');p.input.close();await until(()=>p.closed);
  assert.equal(transport.calls.filter(([action])=>action==='connect').length,1);
  assert.equal(link.active,false);assert.match(link.error,/拔出/);
});

test('wrong exchange identity can never reach USB',async t=>{
  const normal=api();
  const transport=async(action,body)=>{const result=await normal(action,body);if(action==='serial')result.result.token='someone-else';return result;};
  const p=port(),link=new BrowserSerial(transport);t.after(()=>link.disconnect());
  await link.connect(p,'TsenseI_v2');await until(()=>p.closed);
  assert.match(link.error,/会话/);assert.ok(frames(p.writes).every(f=>f[5]!==0x20));
});

const binaryToken = '1234567890abcdef1234567890abcdef';
function binaryResponse(seq, packets, token=binaryToken) {
  const bytes = new Uint8Array(42 + packets.reduce((sum,p)=>sum+6+p.bytes.length,0));
  bytes.set(new TextEncoder().encode('MST1'+token));
  const view = new DataView(bytes.buffer); view.setUint32(36,seq,true); view.setUint16(40,packets.length,true);
  let offset = 42;
  for (const packet of packets) {
    view.setUint16(offset,packet.bytes.length,true);view.setFloat32(offset+2,packet.age_ms||0,true);
    bytes.set(packet.bytes,offset+6);offset += 6+packet.bytes.length;
  }
  return bytes;
}
function binaryApi(exchange) {
  return async(action,body)=>{
    if(action==='connect')return {result:{token:binaryToken,transport:'binary-v1'}};
    if(action==='serial_binary') { await wait(2); return exchange(body); }
    return {result:true};
  };
}

test('binary wire uses exact unencoded chunks, detects truncated and trailing bytes',()=>{
  const request=serialRequest(binaryToken,12,[new Uint8Array([0,255]),new Uint8Array([128,1])],4);
  assert.equal(new TextDecoder().decode(request.subarray(0,36)),'MSR1'+binaryToken);
  assert.equal(new DataView(request.buffer).getUint32(36,true),12);
  assert.deepEqual([...request.subarray(40)],[0,255,128,1]);
  const command=frame(0x20,new Uint8Array(100));
  const result=binaryResponse(12,[{bytes:command,age_ms:1.25}]);
  const parsed=serialResponse(result);assert.equal(parsed.seq,12);assert.equal(parsed.token,binaryToken);
  assert.deepEqual(parsed.frames[0].bytes,command);assert.equal(parsed.frames[0].age_ms,1.25);
  assert.throws(()=>serialResponse(result.subarray(0,-1)),/截断|长度/);
  assert.throws(()=>serialResponse(new Uint8Array([...result,0])),/多余/);
});

test('binary loop separates position writes while preserving command order and RX',async t=>{
  const commands=[frame(0x20,new Uint8Array(100)),frame(0x21,new Uint8Array(4)),frame(0x22,new Uint8Array(12))];
  const uploaded=[];
  const transport=binaryApi(body=>{
    uploaded.push(...body.subarray(40));
    const seq=new DataView(body.buffer).getUint32(36,true);
    return binaryResponse(seq,seq===1?commands.map(bytes=>({bytes})):[]);
  });
  const p=port(),link=new BrowserSerial(transport);t.after(()=>link.disconnect());
  await link.connect(p,'TsenseI_v3_0');
  for(let i=0;i<256;i++)p.input.enqueue(new Uint8Array([i]));
  await until(()=>uploaded.length===256 && p.writes.length===2);
  assert.deepEqual(uploaded,Array.from({length:256},(_,i)=>i));
  assert.deepEqual(frames(p.writes),commands);
  assert.equal(link.txCount,3);assert.equal(link.positionCount,1);
});

test('binary invalid last frame prevents the entire batch from reaching USB',async t=>{
  const good=frame(0x20,new Uint8Array(100)),bad=good.slice();bad[10]^=1;
  const p=port(),link=new BrowserSerial(binaryApi(body=>binaryResponse(new DataView(body.buffer).getUint32(36,true),[{bytes:good},{bytes:bad}])));
  t.after(()=>link.disconnect());await link.connect(p,'TsenseI_v3_0');await until(()=>p.closed);
  assert.ok(frames(p.writes).every(f=>f[5]!==0x20));assert.match(link.error,/校验/);
});

test('binary late responses after disconnect cannot replay positions',async t=>{
  const command=frame(0x20,new Uint8Array(100));
  const p=port(),link=new BrowserSerial(binaryApi(async body=>{
    await wait(240);return binaryResponse(new DataView(body.buffer).getUint32(36,true),[{bytes:command}]);
  }));
  t.after(()=>link.disconnect());await link.connect(p,'TsenseI_v3_0');await until(()=>p.closed);await wait(100);
  assert.ok(frames(p.writes).every(f=>f[5]!==0x20));
});

function trajectory(value){
  const bytes=new Uint8Array(192);bytes.set(new TextEncoder().encode('JQP1'));
  const view=new DataView(bytes.buffer);for(let i=0;i<20;i++)view.setFloat32(4+i*4,value,true);
  bytes.set(frame(0x20,new Uint8Array(100)),84);return bytes;
}
test('coalesced Ruckig positions retain USB connection and strip trajectory metadata',async t=>{
  const normal=binaryApi(body=>{
    const seq=new DataView(body.buffer).getUint32(36,true);
    return binaryResponse(seq,seq<=4?[{bytes:trajectory([0,.001,.8,.801][seq-1])}]:[]);
  });
  const transport=async(a,b)=>{const r=await normal(a,b);if(a==='connect')r.result.max_velocity=Array(20).fill(2.5);return r;};
  const p=port(),link=new BrowserSerial(transport);t.after(()=>link.disconnect());
  await link.connect(p,'TsenseI_v3_0');await until(()=>link.positionCount===4);
  assert.equal(frames(p.writes).filter(f=>f[5]===0x20).length,4);
  assert.ok(p.writes.every(f=>f.length===108&&f[0]===0xaa));
  assert.equal(link.active,true);assert.equal(p.closed,false);assert.equal(link.error,'');
  assert.ok(Math.abs(link.guard.last[0]-.801)<1e-6);
  assert.ok([...link.intervals.slice(0,link.intervalCount)].every(ms=>ms>=2));
});
test('failed USB write never advances the continuity baseline',async t=>{
  const p=port(),link=new BrowserSerial(api());t.after(()=>link.disconnect());
  await link.connect(p,'TsenseI_v3_0');link.guard=new PositionGuard(Array(20).fill(2.5));
  link.guard.commit(new Float32Array(20),0);
  const real=link.writer;
  link.writer={write:async()=>{throw Error('injected USB failure');},abort:async()=>{}};
  await assert.rejects(link.write(frame(0x20,new Uint8Array(100)),link.generation,Infinity,{position:true,q:Float32Array.from({length:20},()=>.001)}),/injected/);
  assert.equal(link.guard.last[0],0);assert.equal(link.lastPositionAt,null);link.writer=real;
});
test('position metadata rejects invalid values but never imposes a jump threshold',()=>{
  const guard=new PositionGuard(Array(20).fill(2.5));guard.commit(new Float32Array(20),100);
  assert.doesNotThrow(()=>guard.check(Array(20).fill(.8)));
  assert.throws(()=>guard.check(Array(20).fill(NaN)),/无效/);
  assert.throws(()=>guard.check(Array(20).fill(Infinity)),/无效/);
  assert.throws(()=>guard.check(Array(19).fill(0)),/无效/);
});
test('fragmented and combined network records survive reusable parser compaction',async()=>{
  const records=[new Uint8Array([1,2,3]),new Uint8Array([4,5])];
  const wire=new Uint8Array(13),view=new DataView(wire.buffer);view.setUint32(0,3,true);wire.set(records[0],4);view.setUint32(7,2,true);wire.set(records[1],11);
  const abort=new AbortController(),seen=[];
  const body=new ReadableStream({start(c){for(const [a,b] of [[0,2],[2,6],[6,13]])c.enqueue(wire.slice(a,b));}});
  await consumeRecords({ok:true,body},b=>{seen.push([...b]);if(seen.length===2)abort.abort();},{signal:abort.signal});
  assert.deepEqual(seen,records.map(b=>[...b]));
});
