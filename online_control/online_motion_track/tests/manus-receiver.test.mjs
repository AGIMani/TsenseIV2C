import test from 'node:test';
import assert from 'node:assert/strict';
import {ManusReceiver, manusStatus} from '../static/manus-receiver.js';

test('Manus reads background results with a cursor, without uploading skeletons', async () => {
  const requests=[], frames=[];
  const receiver=new ManusReceiver({session:'manus',combined:true,background:true,now:()=>100,
    fetcher:async(url,options)=>{
      requests.push([url,JSON.parse(options.body)]);
      return {ok:true,json:async()=>({source:'manus_glove',revision:8,update_revision:3,
        connected:true,age_ms:2,hand:{label:'Left'},raw_manus:{sequence:8},
        update:{sequence:4,tracking:true}})};
    },onFrame:f=>frames.push(f),onStatus:()=>{},onError:e=>{throw e;}});
  try {
    await receiver.poll(); clearTimeout(receiver.timer);
    await receiver.poll(); clearTimeout(receiver.timer);
    assert.deepEqual(requests,[['/api/manus/frame',{session:'manus',after_revision:-1}],
      ['/api/manus/frame',{session:'manus',after_revision:3}]]);
    assert.equal(frames.length,1);
    assert.equal(frames[0].combined_frame,true);
    assert.deepEqual(frames[0].raw_manus,{sequence:8});
    assert.equal(receiver.source,'manus_glove');
  } finally {receiver.close();}
});

test('Manus calibration reads observations independently of the control worker', async () => {
  let request;
  const receiver=new ManusReceiver({session:'calibration',combined:false,
    fetcher:async(url,options)=>{
      request=[url,JSON.parse(options.body)];
      return {ok:true,json:async()=>({revision:1,age_ms:0,connected:true,hand:{}})};
    },onFrame:()=>{},onStatus:()=>{},onError:e=>{throw e;}});
  try {await receiver.poll(); assert.deepEqual(request,['/api/manus/latest',{session:'calibration'}]);}
  finally {receiver.close();}
});

test('Manus status distinguishes missing SDK/authorization, stale data and transport loss', () => {
  assert.match(manusStatus({hand:null,source_metadata:{error:'SDK file is missing'}}),/SDK file is missing/);
  assert.match(manusStatus({hand:null,source_metadata:{error:'SDK Feature license unavailable'}}),/Feature license unavailable/);
  assert.match(manusStatus({hand:null,age_ms:400,source_metadata:{}}),/骨架已过期/);
  assert.doesNotMatch(manusStatus({hand:null,age_ms:Infinity,source_metadata:{}}),/骨架已过期/);
  assert.match(manusStatus({connection_error:'timeout'}),/接收中断/);
  assert.match(manusStatus({hand:{},age_ms:12}),/Manus 已连接.*12 ms/);
});
