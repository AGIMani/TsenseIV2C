import test from 'node:test';
import assert from 'node:assert/strict';
import {decodeRealSenseImage, RealSenseCameraClient, realSenseCameraStatus,
  realSenseFrameFields, realSenseFusionStatus} from '../static/realsense-camera.js';

function packet(meta={}, pixels=[255,0,4,1,127,0]) {
  const header=new TextEncoder().encode(JSON.stringify({format:'rgb8',width:2,height:1,
    has_frame:true,revision:1,frame_id:101,stream_id:'capture-A',capture_unix_ms:123456,
    connected:true,age_ms:2,stale_after_ms:350,...meta}));
  const buffer=new ArrayBuffer(4+header.length+pixels.length);
  new DataView(buffer).setUint32(0,header.length,true);
  new Uint8Array(buffer,4,header.length).set(header);
  new Uint8Array(buffer,4+header.length).set(pixels);
  return buffer;
}
const tick=()=>new Promise(resolve=>setImmediate(resolve));
function fixture(t) {
  let now=100;
  const requests=[],bitmaps=[];
  const ctx={createImageData:(w,h)=>({data:new Uint8ClampedArray(w*h*4)}),putImageData:image=>ctx.last=[...image.data]};
  const canvas={width:0,height:0,getContext:()=>ctx};
  const client=new RealSenseCameraClient(canvas,{clock:()=>now,
    fetchImpl:(url,options)=>new Promise((resolve,reject)=>{
      options.signal.addEventListener('abort',()=>reject(Error('aborted')),{once:true});
      requests.push({url,options,respond:(meta,pixels)=>resolve({ok:true,arrayBuffer:async()=>packet(meta,pixels)})});
    }),createBitmap:async()=>{
      const bitmap={pixels:ctx.last,closed:false,close(){this.closed=true;}};bitmaps.push(bitmap);return bitmap;
    }});
  t.after(()=>client.close());
  return {client,requests,ctx,bitmaps,advance:ms=>now+=ms};
}

test('paired RGB decoder rejects corrupt dimensions, payloads, metadata and depth identity',()=>{
  assert.deepEqual([...decodeRealSenseImage(packet()).pixels],[255,0,4,1,127,0]);
  assert.throws(()=>decodeRealSenseImage(new ArrayBuffer(3)),/不完整/);
  const bad=new ArrayBuffer(4);new DataView(bad).setUint32(0,65537,true);
  assert.throws(()=>decodeRealSenseImage(bad),/图像头/);
  for(const meta of [{format:'gray8'},{width:8192},{height:0},{width:1.5},{stream_id:null},
    {stream_id:''},{frame_id:null},{frame_id:-1},{frame_id:1.5},{age_ms:null},{stale_after_ms:0}])
    assert.throws(()=>decodeRealSenseImage(packet(meta)),/尺寸、帧标识或格式/);
  assert.throws(()=>decodeRealSenseImage(packet({},[1,2,3])),/尺寸、帧标识或格式/);
  assert.equal(decodeRealSenseImage(packet({has_frame:false,frame_id:null,stream_id:null},[])).pixels.length,0);
});

test('RGB channels and consumed frame identity remain paired while newer capture replaces the pending bitmap',async t=>{
  const {client,requests,bitmaps}=fixture(t);
  const read=client.read('rgbd-session');requests[0].respond();
  assert.equal(await read,bitmaps[0]);
  assert.deepEqual(bitmaps[0].pixels,[255,0,4,255,1,127,0,255]);
  const inferenceMeta={...client.meta};
  await tick();
  assert.equal(requests[1].url,'/api/realsense/frame');
  assert.deepEqual(JSON.parse(requests[1].options.body),{session:'rgbd-session',after_revision:1});
  requests[1].respond({revision:2,frame_id:102},[0,1,2,3,4,5]);await tick();
  requests[2].respond({revision:3,frame_id:103},[6,7,8,9,10,11]);await tick();
  assert.equal(client.meta.frame_id,101,'transport must not replace the metadata owned by active inference');
  assert.ok(bitmaps[1].closed,'superseded queued image released');
  assert.equal(await client.read('rgbd-session'),bitmaps[2]);
  assert.equal(client.meta.frame_id,103);
  const submitted=realSenseFrameFields(inferenceMeta);
  assert.equal(submitted.source,'realsense_rgbd');
  assert.equal(submitted.realsense_frame_id,101);
  assert.equal(submitted.realsense_stream_id,'capture-A');
  assert.equal(submitted.source_metadata.camera.capture_unix_ms,123456);
  assert.equal(bitmaps[0].closed,false,'inference owns transferred bitmap');
});

test('transport, bitmap queue wait and empty frames never reuse an earlier paired frame',async t=>{
  const {client,requests,advance,bitmaps}=fixture(t);
  const read=client.read('x');advance(400);requests[0].respond();assert.equal(await read,null);
  assert.equal(bitmaps.length,0);
  await tick();requests[1].respond({revision:2,frame_id:102});await tick();advance(351);
  assert.equal(await client.read('x'),null);assert.ok(bitmaps[0].closed);
  const empty=client.read('x');requests[2].respond({has_frame:false,age_ms:null,frame_id:null,stream_id:null},[]);
  assert.equal(await empty,null);
  assert.equal(realSenseFrameFields(client.meta).realsense_frame_id,null);
});

test('stop aborts capture, releases the latest bitmap and settles an outstanding read',async t=>{
  const {client,requests,bitmaps}=fixture(t);
  const read=client.read('x');requests[0].respond();await read;await tick();
  requests[1].respond({revision:2,frame_id:102});await tick();client.close();
  assert.ok(bitmaps[1].closed);assert.ok(requests.at(-1).options.signal.aborted);
  assert.equal(await client.read('x'),null);
  const waiting=fixture(t),pending=waiting.client.read('y');waiting.client.close();assert.equal(await pending,null);
});

test('UI distinguishes acquisition, inference, fusion success and unavailable depth',()=>{
  const meta=decodeRealSenseImage(packet({capture_fps:30})).meta;
  assert.match(realSenseCameraStatus(meta,24),/RGB \+ 深度采集 30.0 FPS · MediaPipe 识别 24.0 FPS/);
  assert.match(realSenseCameraStatus({...meta,error:'USB disconnected'},24),/USB disconnected/);
  assert.match(realSenseFusionStatus({status:'fused',measured_count:17},true),/21 点 · 17 点有效深度/);
  assert.match(realSenseFusionStatus({status:'unavailable'},false),/保持姿态/);
});
