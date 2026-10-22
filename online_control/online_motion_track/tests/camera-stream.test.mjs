import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import vm from 'node:vm';
import {openMediaPipeCamera, cameraCaptureLabel} from '../static/camera-stream.js';

const deferred=()=>{let resolve;const promise=new Promise(r=>resolve=r);return {promise,resolve};};
const flush=()=>new Promise(resolve=>setImmediate(resolve));
function fixture({label='Intel(R) RealSense(TM) Depth Camera 455f RGB',maximum=90}={}) {
  const calls=[],applied=[];
  const settings={width:640,height:480,frameRate:30,deviceId:'rgb-camera'};
  const track={label,readyState:'live',stopped:0,
    stop(){this.stopped++;this.readyState='ended';},
    getSettings:()=>({...settings}),
    getCapabilities:()=>({frameRate:{min:5,max:maximum},resizeMode:['none','crop-and-scale']}),
    async applyConstraints(value){
      applied.push(value);
      // A real D455f RGB mode: 90 FPS requires reducing height to 360.
      if(value.frameRate.exact>60 && value.height?.exact===480)
        throw new DOMException('unsupported resolution','OverconstrainedError');
      Object.assign(settings,{height:maximum>60?360:480,frameRate:maximum});
    },
  };
  const stream={getVideoTracks:()=>[track],getTracks:()=>[track]};
  const mediaDevices={async getUserMedia(constraints){calls.push(constraints);return stream;}};
  return {track,stream,settings,mediaDevices,calls,applied};
}

test('RealSense prioritizes its reported maximum over the old resolution/30 FPS cap',async()=>{
  const h=fixture(),statuses=[];
  const camera=await openMediaPipeCamera({mediaDevices:h.mediaDevices,deviceId:'rgb-camera',onStatus:s=>statuses.push(s)});
  assert.equal(camera.realSense,true);assert.equal(camera.maxFrameRate,90);
  assert.equal(camera.settings.height,360);assert.equal(camera.settings.frameRate,90);
  assert.deepEqual(h.applied[0],{frameRate:{exact:90},width:{ideal:640},height:{ideal:480},resizeMode:{exact:'none'}});
  assert.deepEqual(h.calls[0].video.deviceId,{exact:'rgb-camera'});
  assert.match(statuses[0],/查询最高/);assert.match(statuses.at(-1),/640 × 360 · 采集设置 90 FPS/);
  assert.equal(h.track.stopped,0);
});

test('known RealSense skips the initial 30 FPS cap; default camera is also detected after permission',async()=>{
  for(const known of [false,true]) {
    const h=fixture();
    const camera=await openMediaPipeCamera({mediaDevices:h.mediaDevices,label:known?h.track.label:''});
    assert.equal(camera.maxFrameRate,90);
    assert.deepEqual(h.calls[0].video.facingMode,'user');
    assert.equal(Boolean(h.calls[0].video.frameRate),!known);
  }
});

test('ordinary webcams keep their 640x480 / 30 FPS request and need no capabilities API',async()=>{
  const h=fixture({label:'UGREEN Camera'});delete h.track.getCapabilities;
  const camera=await openMediaPipeCamera({mediaDevices:h.mediaDevices});
  assert.equal(camera.realSense,false);assert.equal(camera.maxFrameRate,null);
  assert.equal(h.calls[0].video.frameRate.max,30);assert.equal(h.applied.length,0);
  assert.match(cameraCaptureLabel(camera),/^CAMERA/);
});

test('maximum comes from the selected stream, including USB 2, IR and fractional modes',async()=>{
  for(const maximum of [30,60,89.999,100,120]) {
    const h=fixture({maximum});
    const camera=await openMediaPipeCamera({mediaDevices:h.mediaDevices});
    assert.equal(camera.maxFrameRate,maximum);assert.equal(h.applied[0].frameRate.exact,maximum);
  }
});

test('missing capability/invalid maximum releases the stream instead of silently starting at 30',async()=>{
  for(const maximum of [undefined,NaN,0,Infinity]) {
    const h=fixture();h.track.getCapabilities=()=>({frameRate:{max:maximum}});
    await assert.rejects(openMediaPipeCamera({mediaDevices:h.mediaDevices}),/无法确认最高帧率/);
    assert.equal(h.track.stopped,1);
  }
});

test('a browser unable to raise an active UVC mode reopens the same camera with required FPS',async()=>{
  for(const rejected of [false,true]) {
    const h=fixture(),next=fixture();next.settings.frameRate=90;next.settings.height=360;
    h.track.applyConstraints=async()=>{if(rejected)throw new DOMException('mode change','OverconstrainedError');};
    h.mediaDevices.getUserMedia=async constraints=>{h.calls.push(constraints);return h.calls.length===1?h.stream:next.stream;};
    const camera=await openMediaPipeCamera({mediaDevices:h.mediaDevices});
    assert.equal(h.track.stopped,1);assert.equal(camera.stream,next.stream);
    assert.deepEqual(h.calls[1].video.deviceId,{exact:'rgb-camera'});
    assert.deepEqual(h.calls[1].video.frameRate,{exact:90});
    assert.equal(next.track.stopped,0);
  }
});

test('a reopened camera still below maximum fails and releases both streams',async()=>{
  const h=fixture(),next=fixture();h.track.applyConstraints=async()=>{};
  h.mediaDevices.getUserMedia=async()=>h.track.stopped?next.stream:h.stream;
  await assert.rejects(openMediaPipeCamera({mediaDevices:h.mediaDevices}),/未达到.*最高 90 FPS/);
  assert.equal(h.track.stopped,1);assert.equal(next.track.stopped,1);
});

test('device busy/disconnected during configuration fails without selecting another camera',async()=>{
  for(const name of ['NotReadableError','NotAllowedError']) {
    const h=fixture();h.track.applyConstraints=async()=>{throw new DOMException('unavailable',name);};
    await assert.rejects(openMediaPipeCamera({mediaDevices:h.mediaDevices}),{name});
    assert.equal(h.calls.length,1);assert.equal(h.track.stopped,1);
  }
});

test('cancel during permission acquisition closes the late stream without configuring it',async()=>{
  const h=fixture(),permission=deferred(),controller=new AbortController();
  h.mediaDevices.getUserMedia=()=>permission.promise;
  const pending=openMediaPipeCamera({mediaDevices:h.mediaDevices,signal:controller.signal});
  controller.abort();permission.resolve(h.stream);
  await assert.rejects(pending,{name:'AbortError'});
  assert.equal(h.track.stopped,1);assert.equal(h.applied.length,0);
});

test('cancel during mode negotiation immediately releases the camera and suppresses ready status',async()=>{
  const h=fixture(),mode=deferred(),controller=new AbortController(),statuses=[];
  h.track.applyConstraints=()=>mode.promise;
  const pending=openMediaPipeCamera({mediaDevices:h.mediaDevices,signal:controller.signal,onStatus:s=>statuses.push(s)});
  await flush();controller.abort();assert.equal(h.track.readyState,'ended');mode.resolve();
  await assert.rejects(pending,{name:'AbortError'});
  assert.ok(!statuses.some(s=>s.includes('采集设置')));
});

test('cancel during reopen also releases the replacement camera',async()=>{
  const h=fixture(),next=fixture(),replacement=deferred(),controller=new AbortController();
  h.track.applyConstraints=async()=>{};
  h.mediaDevices.getUserMedia=async()=>h.track.stopped?replacement.promise:h.stream;
  const pending=openMediaPipeCamera({mediaDevices:h.mediaDevices,signal:controller.signal});
  await flush();controller.abort();replacement.resolve(next.stream);
  await assert.rejects(pending,{name:'AbortError'});assert.equal(next.track.stopped,1);
});

test('page startup awaits RealSense configuration, while Leap keeps its separate image path',async()=>{
  for(const page of ['app','calibration']) for(const leap of [false,true]) {
    const h=fixture(),mode=deferred(),init=deferred(),events=[],elements=new Map();
    h.track.addEventListener=()=>{};
    h.track.applyConstraints=async()=>{await mode.promise;h.settings.frameRate=90;h.settings.height=360;};
    const context={AbortController,active:false,starting:false,generation:0,clearTimeout,setTimeout,
      $:id=>{if(!elements.has(id))elements.set(id,{value:'rgb-camera',selectedOptions:[{textContent:h.track.label}],play:async()=>{},parentElement:{classList:{add(){}}}});return elements.get(id);},
      window:{isSecureContext:true},navigator:{mediaDevices:h.mediaDevices},
      diagnostics:{record(){}},diagnosticState(){},refreshControls(){},refresh(){},notice(){},
      ensureLocalRecording:async()=>{},isLeapCamera:()=>leap,isLeapMotion:()=>false,isWuji:()=>false,isWebXR:()=>false,isMetricInput:()=>false,
      LeapCameraClient:class {},cameraCaptureLabel,browserCamera:null,gpuBadge(){},
      isControlModel:()=>false,isA32:()=>false,directMode:()=>false,showResponsePose(){},recorder:{active:false},
      enumerateCameras:async()=>{},cameras:async()=>{},LatestFrameQueue:class {},createQueue(){},performance,
      schedule:()=>events.push('frames'),cameraFrame:()=>events.push('frames'),
      inputSource:()=> 'mediapipe',api:async()=>({input_epoch:1}),session:{session:'test'},
      queue:null,inFlight:null,settleTimer:null,currentRows:()=>[],clearPreset(){},
      openMediaPipeCamera:options=>openMediaPipeCamera({...options,mediaDevices:h.mediaDevices}),
      DetectorClient:class {constructor(){events.push('detector');} ask(){events.push('init');return init.promise;}},
    };
    const source=readFileSync(new URL(`../static/${page}.js`,import.meta.url),'utf8');
    const name=page==='app'?'start':'startInput';
    const end=page==='app'?'$("start").addEventListener':'async function cameraFrame';
    const handler=vm.runInNewContext(source.slice(source.indexOf(`async function ${name}()`),source.indexOf(end))+`;${name};`,context);
    const pending=handler();await flush();assert.deepEqual(events,leap?['detector','init']:[]);
    mode.resolve();await flush();assert.deepEqual(events,['detector','init']);
    init.resolve({delegate:'CPU'});await pending;
    assert.deepEqual(events,['detector','init','frames']);
    assert.equal(h.calls.length,leap?0:1);
    if(!leap)assert.match(elements.get(page==='app'?'camera-caption':'input-status').textContent,/采集设置 90 FPS/);
  }
});

test('calibration submits the negotiated image dimensions instead of assuming 640x480',async()=>{
  const page=readFileSync(new URL('../static/calibration.js',import.meta.url),'utf8');
  const source=page.slice(page.indexOf('async function cameraFrame('),page.indexOf('function applyDescription('));
  for(const height of [360,480]) {
    const video={readyState:2,currentTime:1,videoWidth:640,videoHeight:height},packets=[];
    const context={generation:1,active:true,leapCamera:null,performance,Date,session:{session:'test'},
      $:id=>id==='video'?video:{value:'left'},createImageBitmap:async input=>{assert.equal(input,video);return {};},
      detector:{ask:async()=>({result:{}})},offer:packet=>packets.push(packet),setTimeout:()=>1};
    await vm.runInNewContext(source+';cameraFrame(1,-1);',context);
    assert.deepEqual([...packets[0].frame.camera_size],[640,height]);
    assert.equal(packets[0].frame.source,'mediapipe');
  }
});
