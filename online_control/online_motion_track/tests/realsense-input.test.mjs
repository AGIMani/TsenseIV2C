// Execute the page handlers with a deterministic RGB-D transport and detector.
// No browser webcam, native SDK, hardware connection or synthetic control output.
import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import vm from 'node:vm';
import {realSenseFrameFields,realSenseCameraStatus,realSenseFusionStatus} from '../static/realsense-camera.js';
const files=Object.fromEntries(['app','calibration'].map(name=>[name,readFileSync(new URL(`../static/${name}.js`,import.meta.url),'utf8')]));
const flush=()=>new Promise(resolve=>setImmediate(resolve));
const deferred=()=>{let resolve;const promise=new Promise(r=>resolve=r);return {resolve,promise};};
function elements() {
  const items=new Map();
  const $=id=>{
    if(!items.has(id))items.set(id,{value:id==='input-source'?'realsense_rgbd':'left',
      parentElement:{classList:{add(){},remove(){},toggle(){}}},classList:{add(){},remove(){},toggle(){}},
      setAttribute(){},getTracks:()=>[],selectedOptions:[],play:async()=>{}});
    return items.get(id);
  };
  return {$,items};
}
function slice(name,start,end) {const text=files[name];return text.slice(text.indexOf(start),text.indexOf(end));}
function frameFixture(page) {
  const {items,$}=elements(),inference=deferred(),frames=[],logs=[],errors=[],scheduled=[];
  let now=100;
  const bitmap={close(){this.closed=true;}};
  const camera={meta:{width:640,height:480,frame_id:71,stream_id:'stream-A',revision:17,
    capture_unix_ms:456789,age_ms:5,stale_after_ms:350,connected:true,has_frame:true},
    read:async session=>{assert.equal(session,'rgbd-session');return bitmap;},recordInference(){},inferenceFps:25};
  const context={$,generation:1,active:true,leapCamera:camera,performance:{now:()=>now},Date,
    session:{session:'rgbd-session'},sequence:0,lastVideoTime:-1,
    client:{ask:packet=>{assert.equal(packet.bitmap,bitmap);return inference.promise;}},
    detector:{ask:packet=>{assert.equal(packet.bitmap,bitmap);return inference.promise;}},
    realSenseFrameFields,realSenseCameraStatus,realSenseFusionStatus,
    isLeapCamera:()=>false,isControlModel:()=>false,isV2:()=>false,isA32:()=>true,
    recorder:{record:(type,data)=>logs.push({type,data})},overlay:{setHands(){},clear(){}},
    framePipeline:{offer:frame=>frames.push(frame.packet)},offer:({frame})=>frames.push(frame),
    schedule:()=>scheduled.push('next'),queueMicrotask:()=>scheduled.push('next'),
    notice:message=>errors.push(message),cameraError:error=>error.message,
    stop:async()=>errors.push('stop'),stopInput:async()=>errors.push('stop'),
    createImageBitmap:()=>{throw Error('RGB-D must not use browser video');},
  };
  const code=page==='app'?slice('app','async function processFrame(token)','function applyFrameUpdate')
    :slice('calibration','async function cameraFrame(','function applyDescription(');
  const handler=vm.runInNewContext(code+(page==='app'?';processFrame;':';cameraFrame;'),context);
  const points=Array.from({length:21},(_,i)=>({x:i/100,y:i/200,z:-i/500}));
  const detection={result:{worldLandmarks:[points],landmarks:[points],handedness:[[{categoryName:'Left',score:.9}]]}};
  return {context,handler,inference,camera,frames,logs,errors,items,scheduled,detection,bitmap,advance:ms=>now+=ms};
}
for(const page of ['app','calibration']) {
  test(`${page} submits the consumed RGB frame identity, not metadata from capture during inference`,async()=>{
    const h=frameFixture(page),pending=h.handler(1,-1);await flush();
    h.camera.meta={...h.camera.meta,frame_id:99,stream_id:'stream-B',capture_unix_ms:999999};
    h.inference.resolve(h.detection);await pending;
    assert.deepEqual(h.errors,[]);assert.equal(h.frames.length,1);
    const p=h.frames[0];
    assert.equal(p.source,'realsense_rgbd');assert.equal(p.realsense_frame_id,71);
    assert.equal(p.realsense_stream_id,'stream-A');assert.equal(p.capture_unix_ms,456789);
    assert.equal(p.source_metadata.camera.frame_id,71);assert.equal(p.hand.world.length,21);
    assert.equal(p.hand.normalized.length,21);assert.deepEqual([...p.camera_size],[640,480]);
    if(page==='app')assert.equal(h.logs[0].type,'realsense_rgbd_raw');
  });
  test(`${page} missing or expired paired depth input sends no hand rather than a MediaPipe fallback`,async()=>{
    for(const missing of [false,true]) {
      const h=frameFixture(page);
      if(missing)h.camera.read=async()=>null;
      const pending=h.handler(1,-1);await flush();
      if(!missing){h.advance(351);h.inference.resolve(h.detection);}
      await pending;assert.deepEqual(h.errors,[]);
      assert.equal(h.frames[0].source,'realsense_rgbd');assert.equal(h.frames[0].hand,null);
    }
  });
  test(`${page} stop during RGB inference cannot submit a late frame`,async()=>{
    const h=frameFixture(page),pending=h.handler(1,-1);await flush();
    h.context.generation++;h.context.active=false;h.inference.resolve(h.detection);await pending;
    assert.equal(h.frames.length,0);assert.equal(h.scheduled.length,0);
  });
  test(`${page} starts the RGB-D client and detector without opening a browser camera`,async()=>{
    const {items,$}=elements(),events=[],requests=[];
    const context={$,active:false,starting:false,generation:0,clearTimeout,settleTimer:null,
      diagnostics:{record(){}},diagnosticState(){},refreshControls(){},refresh(){},notice:message=>{if(message)throw Error(message);},
      ensureLocalRecording:async()=>{},isLeapCamera:()=>false,isLeapMotion:()=>false,isWuji:()=>false,isWebXR:()=>false,isMetricInput:()=>false,
      RealSenseCameraClient:class {constructor(){events.push('rgbd');}},browserCamera:null,gpuBadge(){},
      isControlModel:()=>false,isA32:()=>false,directMode:()=>false,showResponsePose(){},recorder:{active:false},
      enumerateCameras:async()=>{throw Error('browser camera enumerated');},cameras:async()=>{throw Error('browser camera enumerated');},
      LatestFrameQueue:class {},createQueue(){},performance,
      schedule:()=>events.push('frames'),cameraFrame:()=>events.push('frames'),
      inputSource:()=> 'realsense_rgbd',api:async(url,packet)=>{requests.push({url,packet});return {input_epoch:1,session:'rgbd-session'};},
      session:{session:'rgbd-session'},queue:null,inFlight:null,currentRows:()=>[],clearPreset(){},
      DetectorClient:class {constructor(){events.push('detector');}async ask(){events.push('init');return {delegate:'CPU'};}},
      stop:async()=>{throw Error('unexpected stop');},stopInput:async()=>{throw Error('unexpected stop');},
    };
    const code=page==='app'?slice(page,'async function start()','$("start").addEventListener')
      :slice(page,'async function startInput()','async function cameraFrame');
    const handler=vm.runInNewContext(code+(page==='app'?';start;':';startInput;'),context);
    await handler();assert.deepEqual(events,['rgbd','detector','init','frames']);
    assert.equal(requests[0].packet[page==='app'?'input_source':'source'],'realsense_rgbd');
    assert.equal(requests[0].packet.camera_source,'realsense_rgbd');
    assert.equal(context.active,true);
    if(page==='app')assert.match(items.get('camera-caption').textContent,/D455f RGB-D/);
  });
}

test('tracking skeleton displays the optimized RGB-D solution and clears it when depth becomes unavailable',()=>{
  const {$,items}=elements(),shown=[],records=[];
  const context={$,recorder:{record:(type,data)=>records.push({type,data})},isA32Model:()=>false,
    isA30:()=>false,isA32:()=>false,isV2:()=>false,isControlModel:()=>false,directMode:()=>false,
    showResponsePose(){},realSenseFusionStatus,performance:{now:()=>120},latestWorld:null,
    skeleton:{setPoints:points=>shown.push(points)},fpsCount:0,fpsStart:0};
  const apply=vm.runInNewContext(slice('app','function applyFrameUpdate','function offerQuestFrame')+';applyFrameUpdate;',context);
  const world=Array.from({length:21},()=>[1,2,3]),fused=world.map(()=>[.01,.02,.03]);
  const optimized=fused.map(()=>[.02,.03,.04]);
  const frame={frameId:1,hand:{world,label:'Left'},expected:'Left',timestamp_ms:100,
    packet:{source:'realsense_rgbd',session:'s',realsense_frame_id:71,realsense_stream_id:'stream-A'}};
  apply(frame,{tracking:true,solve_ms:1,fused_world:fused,optimized_points:optimized,
    retarget_joint_angles_rad:{q:.2},depth_fusion:{status:'fused',measured_count:19}});
  assert.equal(shown[0],optimized);assert.match(items.get('realsense-fusion-state').textContent,/19 点有效深度/);
  assert.match(items.get('skeleton-status').textContent,/D455f 深度融合/);
  apply(frame,{tracking:false,depth_fusion:{status:'unavailable'},message:'Depth stale'});
  assert.equal(shown[1],null);assert.match(items.get('realsense-fusion-state').textContent,/保持姿态/);
  assert.equal(records.filter(record=>record.type==='realsense_depth_fusion')[0].data.realsense_frame_id,71);
});

test('both pages retain ordinary RGB, Leap and all prior source choices beside RGB-D',()=>{
  for(const name of ['index','calibration']) {
    const html=readFileSync(new URL(`../static/${name}.html`,import.meta.url),'utf8');
    for(const source of ['mediapipe','realsense_rgbd','leapmotion_camera','webxr','leapmotion','wuji_glove'])
      assert.equal(html.match(new RegExp(`<option value="${source}">`,'g'))?.length,1,`${name}: ${source}`);
  }
});

test('tracking stop closes the paired camera and stops the owning backend session',async()=>{
  const {$}=elements(),events=[],requests=[];
  const context={$,stopping:false,generation:1,active:true,starting:false,cameraOpening:null,browserCamera:null,
    questReceiver:null,leapCamera:{close:()=>events.push('camera-close')},framePipeline:{close:()=>events.push('queue-close')},
    frameCallback:null,client:{close:()=>events.push('detector-close')},stream:null,overlay:{clear(){}},
    session:{session:'rgbd-session'},a32Panel:{finish:async()=>{},reset(){}},a30Preview:{stop(){}},
    localControl:{disarm(){},disconnect:async()=>{}},displayedSample:null,hardwarePoll:null,clearTimeout,
    latestWorld:null,skeleton:{setPoints(){}},gpuBadge(){},refreshControls(){},
    api:async(url,packet)=>requests.push({url,packet}),recorder:{stop:async()=>null},
    showDownload(){},refreshRecordings:async()=>{},document:{hidden:false},renderHardware(){},notice(){},
  };
  const stop=vm.runInNewContext(slice('app','async function stop({','function schedule(token)')+';stop;',context);
  await stop();assert.deepEqual(events,['camera-close','queue-close','detector-close']);
  assert.equal(requests[0].url,'/api/stop');assert.equal(requests[0].packet.session,'rgbd-session');
  assert.equal(context.leapCamera,null);assert.equal(context.session,null);assert.equal(context.active,false);
});

test('calibration freeze closes the native transport and disables the paired input lease',async()=>{
  const {$}=elements(),events=[],requests=[];
  const context={$,generation:1,active:true,starting:false,clearTimeout,settleTimer:null,videoTimer:null,
    cameraOpening:null,browserCamera:null,receiver:null,detector:{close:()=>events.push('detector-close')},
    leapCamera:{close:()=>events.push('camera-close')},stream:null,queue:{close:()=>events.push('queue-close')},
    refresh(){},inFlight:null,session:{session:'rgbd-session'},inputEpoch:1,
    api:async(url,packet)=>{requests.push({url,packet});return {input_epoch:2};},createQueue(){},offer(){},
  };
  const stop=vm.runInNewContext(slice('calibration','function closeLocalInput()','async function startInput()')+';stopInput;',context);
  await stop();assert.deepEqual(events,['detector-close','camera-close','queue-close']);
  assert.equal(requests[0].url,'/api/calibration/input');assert.equal(requests[0].packet.session,'rgbd-session');
  assert.equal(requests[0].packet.enabled,false);assert.equal(context.leapCamera,null);assert.equal(context.active,false);
});

test('RGB-D local hand confirmation requires accepted fused points instead of raw image landmarks',async()=>{
  const h=frameFixture('app'),observations=[];
  h.context.isControlModel=()=>true;
  h.context.localControl={observeHand:(...args)=>observations.push(args)};
  const pending=h.handler(1,-1);await flush();h.inference.resolve(h.detection);await pending;
  assert.deepEqual(h.errors,[]);assert.equal(observations.length,0,'raw MediaPipe must not enable the hand gate');
  Object.assign(h.context,{isA32Model:()=>false,isA30:()=>false,directMode:()=>false,
    showResponsePose(){},latestWorld:null,skeleton:{setPoints(){}},fpsCount:0,fpsStart:0});
  const apply=vm.runInNewContext(slice('app','function applyFrameUpdate','function offerQuestFrame')+';applyFrameUpdate;',h.context);
  const frame={frameId:1,hand:h.frames[0].hand,expected:'Left',timestamp_ms:100,packet:h.frames[0]};
  apply(frame,{tracking:true,solve_ms:1,fused_world:[[0,0,0]],depth_fusion:{status:'fused'}});
  apply(frame,{tracking:false,depth_fusion:{status:'unavailable'}});
  apply(frame,{tracking:true,solve_ms:1,fused_world:[[0,0,0]],display_discarded:true});
  assert.deepEqual(observations,[[true,100],[false,100],[false,100]]);
});
