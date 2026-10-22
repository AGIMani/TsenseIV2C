import test from 'node:test';
import assert from 'node:assert/strict';
import {decodeLeapImage, LeapCameraClient, leapCameraStatus} from '../static/leap-camera.js';

function packet(meta={}, pixels=[0,127,128,255]) {
  const header=new TextEncoder().encode(JSON.stringify({format:'gray8',width:2,height:2,
    has_frame:true,revision:1,connected:true,age_ms:2,stale_after_ms:350,...meta}));
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
  const client=new LeapCameraClient(canvas,{clock:()=>now,
    fetchImpl:(_url,options)=>new Promise((resolve,reject)=>{
      options.signal.addEventListener('abort',()=>reject(Error('aborted')),{once:true});
      requests.push({options,respond:(meta,pixels)=>resolve({ok:true,arrayBuffer:async()=>packet(meta,pixels)})});
    }),createBitmap:async()=>{
      const bitmap={closed:false,close(){this.closed=true;}};bitmaps.push(bitmap);return bitmap;
    }});
  t.after(()=>client.close());
  return {client,requests,ctx,bitmaps,advance:ms=>now+=ms};
}

test('binary decoder checks truncation, format and exact pixel size',()=>{
  assert.deepEqual([...decodeLeapImage(packet()).pixels],[0,127,128,255]);
  assert.throws(()=>decodeLeapImage(new ArrayBuffer(3)),/不完整/);
  const bad=new ArrayBuffer(4);new DataView(bad).setUint32(0,65537,true);
  assert.throws(()=>decodeLeapImage(bad),/图像头/);
  for(const meta of [{format:'rgb8'},{width:4096},{height:0},{width:1.5}])assert.throws(()=>decodeLeapImage(packet(meta)),/尺寸或格式/);
  assert.throws(()=>decodeLeapImage(packet({},[1])),/尺寸或格式/);
  assert.equal(decodeLeapImage(packet({has_frame:false},[])).pixels.length,0);
});

test('gray8 expands to RGB and next capture overlaps inference',async t=>{
  const {client,requests,ctx,bitmaps}=fixture(t);
  const read=client.read('camera-session');
  requests[0].respond();
  assert.equal(await read,bitmaps[0]);
  assert.deepEqual(ctx.last,[0,0,0,255,127,127,127,255,128,128,128,255,255,255,255,255]);
  await tick();
  assert.equal(requests.length,2,'already fetching during inference');
  assert.deepEqual(JSON.parse(requests[1].options.body),{session:'camera-session',after_revision:1});
  requests[1].respond({revision:2});await tick();
  requests[2].respond({revision:3});await tick();
  assert.ok(bitmaps[1].closed,'superseded image released');
  assert.equal(await client.read('camera-session'),bitmaps[2]);
  assert.equal(client.meta.revision,3);
  assert.equal(bitmaps[0].closed,false,'transferred/consumed frame remains owned by detector');
});

test('transport and waiting age both expire frames, without reusing old pixels',async t=>{
  const {client,requests,advance,bitmaps}=fixture(t);
  const read=client.read('x');advance(400);requests[0].respond();
  assert.equal(await read,null);
  assert.equal(bitmaps.length,0);
  await tick();requests[1].respond({revision:2});await tick();
  advance(351);
  assert.equal(await client.read('x'),null);
  assert.ok(bitmaps[0].closed);
});

test('close aborts long-poll, settles consumers and closes pending bitmap',async t=>{
  const {client,requests,bitmaps}=fixture(t);
  const read=client.read('x');requests[0].respond();await read;await tick();
  requests[1].respond({revision:2});await tick();
  client.close();
  assert.ok(bitmaps[1].closed);
  assert.ok(requests.at(-1).options.signal.aborted);
  assert.equal(await client.read('x'),null);
  const waiting=fixture(t),pending=waiting.client.read('y');
  waiting.client.close();assert.equal(await pending,null);
});

test('missing image clears detection input and FPS labels distinguish acquisition and inference',async t=>{
  const {client,requests,advance}=fixture(t);
  const read=client.read('x');requests[0].respond({has_frame:false,age_ms:null},[]);
  assert.equal(await read,null);
  for(let i=0;i<=90;i++){client.recordInference();advance(1000/60);}
  assert.ok(Math.abs(client.inferenceFps-60)<.01);
  const message=leapCameraStatus({connected:true,age_ms:1,stale_after_ms:350,capture_fps:90,fps_status:'below_target'},60);
  assert.match(message,/采集 90.0 \/ 120 FPS/);
  assert.match(message,/识别 60.0 FPS/);
  assert.match(message,/未达到 120 FPS/);
});
