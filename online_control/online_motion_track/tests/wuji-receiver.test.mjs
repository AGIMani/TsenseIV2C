import test from 'node:test';
import assert from 'node:assert/strict';
import {WujiReceiver} from '../static/wuji-receiver.js';

test('combined receiver uses one request and delivers only solved updates',async()=>{
  const requests=[],frames=[];
  let update={sequence:0,tracking:true};
  const receiver=new WujiReceiver({session:'session',combined:true,now:()=>100,
    fetcher:async(url,options)=>{requests.push([url,JSON.parse(options.body)]);return {ok:true,json:async()=>({
      source:'wuji_glove',revision:1,connected:true,hand:{label:'Left'},age_ms:5,update})};},
    onFrame:f=>frames.push(f),onStatus:()=>{},onError:e=>{throw e;}});
  await receiver.poll();clearTimeout(receiver.timer);
  update=null;await receiver.poll();clearTimeout(receiver.timer);receiver.close();
  assert.equal(requests[0][0],'/api/wuji/frame');
  assert.deepEqual(requests.map(r=>r[1].sequence),[0,1]);
  assert.equal(requests[0][1].timestamp_ms,100);
  assert(!('hand' in requests[0][1]));
  assert.equal(frames.length,1);assert.equal(frames[0].combined_frame,true);
});

test('older running servers keep the legacy path until their restart',async()=>{
  let endpoint,body;const frames=[];
  const receiver=new WujiReceiver({session:'old',fetcher:async(url,options)=>{
    endpoint=url;body=JSON.parse(options.body);return {ok:true,json:async()=>({revision:1,age_ms:0,connected:true,hand:{}})};},
    onFrame:f=>frames.push(f),onStatus:()=>{},onError:e=>{throw e;}});
  await receiver.poll();receiver.close();
  assert.equal(endpoint,'/api/wuji/latest');assert.deepEqual(body,{session:'old'});
  assert.equal(frames[0].combined_frame,false);
});

test('background glove results are read once with a solve revision cursor',async()=>{
  const requests=[],frames=[];
  const receiver=new WujiReceiver({session:'background',combined:true,background:true,
    fetcher:async(url,options)=>{requests.push([url,JSON.parse(options.body)]);return {ok:true,json:async()=>({
      revision:6,update_revision:2,age_ms:0,connected:true,hand:{},update:{sequence:9}})};},
    onFrame:f=>frames.push(f),onStatus:()=>{},onError:e=>{throw e;}});
  await receiver.poll();clearTimeout(receiver.timer);await receiver.poll();receiver.close();
  assert.equal(frames.length,1);assert.deepEqual(requests.map(r=>r[1].after_revision),[-1,2]);
  assert(requests.every(r=>!('sequence' in r[1])&&!('timestamp_ms' in r[1])));
  assert.equal(receiver.pollMs,16,'UI polling no longer sets the solver cadence');
});
