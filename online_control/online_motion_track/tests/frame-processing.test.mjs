// Exercise the actual frame handler with fake camera/worker interfaces. No
// camera, USB, live server or browser session is opened by these unit tests.
import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import vm from 'node:vm';

const app=readFileSync(new URL('../static/app.js',import.meta.url),'utf8');
const source=app.slice(app.indexOf('async function processFrame(token)'),app.indexOf('function applyFrameUpdate'));
const deferred=()=>{let resolve;const promise=new Promise(r=>resolve=r);return {resolve,promise};};
const flush=()=>new Promise(resolve=>setImmediate(resolve));
function setup({inputFiltering=true, overlaySmoothing=true}={}){
  const inference=deferred(),events=[],packets=[],records=[],overlays=[];
  const bitmap={closed:0,close(){this.closed++;}};
  const video={readyState:2,currentTime:1,videoWidth:640,videoHeight:480};
  const context={active:true,generation:1,lastVideoTime:-1,sequence:0,performance:{now:()=>100},Date,
    isLeapCamera:()=>false,leapCamera:null,leapCameraStatus:()=> 'camera status',
    $:id=>id==='video'?video:{value:'left',checked:id==='mediapipe-filter'?inputFiltering:overlaySmoothing},createImageBitmap:async()=>bitmap,
    client:{ask:()=>{events.push('infer');return inference.promise;}},
    overlay:{setHands:(...args)=>{overlays.push(args);events.push('overlay');},clear:()=>events.push('clear')},schedule:()=>events.push('schedule'),
    recorder:{record:(type,data)=>records.push({type,data})},session:{session:'unit-session'},
    localControl:{observeHand:()=>events.push('hand')},isControlModel:()=>true,isV2:()=>false,isA32:()=>false,
    framePipeline:{offer:frame=>{packets.push(frame);events.push('offer');}},
    stop:async()=>{events.push('stop');context.generation++;context.active=false;},
    notice:message=>events.push(message),cameraError:error=>error.message,
  };
  const process=vm.runInNewContext(source+';processFrame;',context);
  const points=Array.from({length:21},(_,i)=>({x:i*.001,y:.02,z:.03}));
  const detection={inference_ms:16,result:{worldLandmarks:[points],landmarks:[points],
    handedness:[[{categoryName:'Left',score:.95}]]}};
  return {process,context,inference,events,packets,records,overlays,bitmap,detection};
}

test('registers next camera frame after inference, without waiting on retarget', async()=>{
  const h=setup(),pending=h.process(1);
  await flush();assert.deepEqual(h.events,['infer']);
  h.inference.resolve(h.detection);await pending;
  assert.deepEqual(h.events,['infer','schedule','hand','offer','overlay']);
  assert.equal(h.packets[0].packet.session,'unit-session');
  assert.equal(h.packets[0].packet.hand.world.length,21);
  assert.equal(h.packets[0].packet.hand.world,h.records[0].data.world_landmarks[0]);
  assert.equal(h.packets[0].packet.hand.normalized,h.records[0].data.normalized_landmarks[0]);
  assert.equal(h.overlays[0][0],h.detection.result.landmarks);
  assert.equal(h.overlays[0][2].timestampMs,h.packets[0].packet.timestamp_ms);
  assert.deepEqual([...h.overlays[0][2].labels],['Left']);
});

for (const inputFiltering of [false, true]) for (const overlaySmoothing of [false, true]) {
  test(`input filter=${inputFiltering}, overlay=${overlaySmoothing}: raw packet and choices stay independent`, async () => {
    const h=setup({inputFiltering,overlaySmoothing}), pending=h.process(1);
    await flush(); h.inference.resolve(h.detection); await pending;
    assert.equal(h.packets[0].packet.mediapipe_filter_enabled,inputFiltering);
    assert.equal(h.records[0].data.mediapipe_filter_enabled,inputFiltering);
    assert.equal(h.records[0].data.overlay_smoothing_enabled,overlaySmoothing);
    assert.equal(h.packets[0].packet.hand.world,h.records[0].data.world_landmarks[0]);
    assert.equal(h.overlays[0][0],h.detection.result.landmarks);
  });
}

test('stop or model change during inference cannot submit a late target', async()=>{
  const h=setup(),pending=h.process(1);await flush();
  h.context.generation=2;h.context.active=false;
  h.inference.resolve(h.detection);await pending;
  assert.equal(h.packets.length,0);assert.deepEqual(h.events,['infer']);
});

test('stop during bitmap creation releases the image and never invokes inference', async()=>{
  const h=setup(),image=deferred();h.context.createImageBitmap=()=>image.promise;
  const pending=h.process(1);h.context.generation=2;image.resolve(h.bitmap);await pending;
  assert.equal(h.bitmap.closed,1);assert.deepEqual(h.events,[]);assert.equal(h.packets.length,0);
});

test('no-hand frames still reach the queue to end tracking', async()=>{
  const h=setup(),pending=h.process(1);await flush();
  h.inference.resolve({result:{},inference_ms:16});await pending;
  assert.equal(h.packets[0].packet.hand,null);
});

test('A3.2 sends raw detection and capture time to the server recorder', async()=>{
  const h=setup();h.context.isA32=()=>true;h.context.isControlModel=()=>false;
  h.detection.result.worldLandmarks.push(h.detection.result.worldLandmarks[0]);
  h.detection.result.landmarks.push(h.detection.result.landmarks[0]);
  h.detection.result.handedness.push([{categoryName:'Right',score:.9}]);
  const pending=h.process(1);await flush();h.inference.resolve(h.detection);await pending;
  const packet=h.packets[0].packet;
  assert.equal(packet.raw_detection.world.length,2);
  assert.equal(packet.raw_detection.normalized.length,2);
  assert.equal(packet.raw_detection.labels[1].label,'Right');
  assert.equal(packet.raw_detection.world,h.records[0].data.world_landmarks);
  assert(packet.capture_unix_ms>0);
});

test('a new image already decoded during inference is consumed without another video wait',()=>{
  const source=app.slice(app.indexOf('function schedule(token)'),app.indexOf('async function processFrame(token)'));
  for(const changed of [true,false]){
    const queued=[],calls=[];
    const video={readyState:2,currentTime:changed?1.033:1,requestVideoFrameCallback:fn=>{calls.push('video');queued.push(fn);return 9;}};
    const context={active:true,generation:1,lastVideoTime:1,frameCallback:null,$:()=>video,
      isLeapCamera:()=>false,
      queueMicrotask:fn=>{calls.push('ready');queued.push(fn);},processFrame:token=>calls.push(token)};
    const schedule=vm.runInNewContext(source+';schedule;',context);
    schedule(1);
    assert.deepEqual(calls,[changed?'ready':'video']);
    queued[0]();assert.equal(calls.at(-1),1);
    context.active=false;schedule(1);assert.equal(queued.length,1);
  }
});

function cameraSetup() {
  const h=setup();
  const elements=new Map();
  h.context.$=id=>{
    if(!elements.has(id))elements.set(id,{value:'left',parentElement:{classList:{toggle(){}}}});
    return elements.get(id);
  };
  h.context.isLeapCamera=()=>true;
  h.context.leapCamera={read:async()=>h.bitmap,recordInference(){},inferenceFps:60,
    meta:{source:'leapmotion_camera',width:512,height:512,age_ms:8,stale_after_ms:350}};
  return h;
}

test('Leap IR feeds MediaPipe image points and preserves camera provenance',async()=>{
  const h=cameraSetup(),pending=h.process(1);await flush();
  h.inference.resolve(h.detection);await pending;
  assert.equal(h.packets[0].packet.source,'mediapipe');
  assert.deepEqual([...h.packets[0].packet.camera_size],[512,512]);
  assert.equal(h.packets[0].packet.source_metadata.camera.source,'leapmotion_camera');
  assert.equal(h.packets[0].hand.world.length,21);
});

test('camera loss and an image expiring during inference both clear the hand',async()=>{
  for(const lost of [true,false]) {
    const h=cameraSetup();
    if(lost)h.context.leapCamera.read=async()=>null;
    const pending=h.process(1);await flush();
    if(!lost){h.context.performance.now=()=>500;h.inference.resolve(h.detection);}
    await pending;
    assert.equal(h.packets[0].packet.hand,null);
    if(lost)assert.ok(!h.events.includes('infer'));
  }
});

test('Leap IR scheduling is independent of display refresh and video readiness',()=>{
  const source=app.slice(app.indexOf('function schedule(token)'),app.indexOf('async function processFrame(token)'));
  const callbacks=[];
  const context={active:true,generation:1,isLeapCamera:()=>true,frameCallback:null,
    queueMicrotask:fn=>callbacks.push(fn),processFrame:()=>{},$:()=>{throw Error('webcam should not be queried');}};
  vm.runInNewContext(source+';schedule(1);',context);
  assert.equal(callbacks.length,1);
});
