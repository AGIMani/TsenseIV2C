import test from 'node:test';
import assert from 'node:assert/strict';
// Node-only Worker harness: no browser or USB, capture its watchdog callback.
globalThis.self={};const intervals=[];const interval=globalThis.setInterval;
globalThis.setInterval=fn=>{intervals.push(fn);return 0;};
const {matchingPort}=await import('./serial-worker.js');globalThis.setInterval=interval;
const info={usbVendorId:123,usbProductId:456};
const mockPort=()=>({getInfo:()=>info,open(){throw Error('must never open');}});
test('Worker uniquely matches authorized device, rejecting absent and duplicate IDs',async()=>{
  const one=mockPort();assert.equal(await matchingPort({getPorts:async()=>[one]},info),one);
  for(const ports of [[],[one,mockPort()]])await assert.rejects(matchingPort({getPorts:async()=>ports},info),/唯一/);
});
test('disconnect during getPorts cancels the pending connection and blocks overlapping connect',async()=>{
  let resolve;const replies=[];
  globalThis.postMessage=m=>replies.push(m);
  Object.defineProperty(globalThis,'navigator',{configurable:true,value:{serial:{getPorts:()=>new Promise(r=>resolve=r)}}});
  const pending=self.onmessage({data:{type:'connect',id:1,info,model:'TsenseI_v3_0',clientId:'test'}});
  await self.onmessage({data:{type:'disconnect',id:2}});
  await self.onmessage({data:{type:'connect',id:3,info}});
  await self.onmessage({data:{type:'connect',id:4,info}});
  resolve([mockPort()]);await pending;
  assert.match(replies.find(r=>r.id===1).error,/取消/);
  assert.match(replies.find(r=>r.id===3).error,/正在连接/);
  assert.match(replies.find(r=>r.id===4).error,/正在连接/);
  assert.equal(replies.find(r=>r.id===2).result,true);
});
