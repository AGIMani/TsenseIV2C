import {translate} from "../../shared/ui-language.js";
import {CalibrationViewer} from './calibration-viewer.js';
import {DetectorClient} from './detector-client.js';
import {LatestFrameQueue} from './latest-frame-queue.js';
import {WebXRReceiver, questStatus} from './webxr-receiver.js';
import {WujiReceiver, wujiStatus} from './wuji-receiver.js';
import {LeapMotionReceiver, leapMotionStatus} from './leapmotion-receiver.js';
import {LeapCameraClient, leapCameraStatus} from './leap-camera.js';
import {RealSenseCameraClient, realSenseCameraStatus, realSenseFusionStatus, realSenseFrameFields} from './realsense-camera.js';
import {openMediaPipeCamera, cameraCaptureLabel} from './camera-stream.js';
import {bindCameraSelector} from './camera-selector.js';

const $=id=>document.getElementById(id), copy=value=>structuredClone(value);
let session=null, rows=[], baseline=[], inputPoints=[], revision=0, generation=0;
let active=false, starting=false, loading=true, fileBusy=false, inputEpoch=0, sequence=0;
let stream=null, detector=null, receiver=null, videoTimer=null, queue=null, inFlight=null;
let leapCamera=null;
let cameraOpening=null, browserCamera=null;
const isLeapCamera=()=>$('input-source').value==='leapmotion_camera';
const inputSource=()=>isLeapCamera()?'mediapipe':$('input-source').value;
let dirty=false, solveFailed=false, settleTimer=null, settleCount=0, stableCount=0;
const inputViewer=new CalibrationViewer($('input-stage'),{editable:true,
  onSelect:index=>selectPoint('input',index),onMove:(index,point)=>moveInput(index,point)});
const staticViewer=new CalibrationViewer($('calibration-stage'),{editable:true,
  onSelect:index=>selectPoint('calibration',index),onMove:(index,point)=>moveCalibration(index,point)});
const resultViewer=new CalibrationViewer($('result-stage'));

function notice(message='') {$('notice').textContent=message;$('notice').hidden=!message;}
async function api(url,data) {
  const response=await fetch(url,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data),signal:AbortSignal.timeout(20000)});
  const body=await response.json();if(!response.ok)throw Error(body.error||`请求失败 (${response.status})`);return body;
}
function currentRows() {return rows.map(({id,link,local_xyz})=>({id,link,local_xyz:[...local_xyz]}));}
function fingerprint(value) {return JSON.stringify(value.map(({id,link,local_xyz})=>({id,link,local_xyz})));}
function refresh() {
  dirty=rows.length>0 && fingerprint(rows)!==fingerprint(baseline);
  $('draft-state').textContent=loading?'正在加载':dirty?'有未保存标定':'标定已载入';
  $('draft-state').classList.toggle('dirty',dirty);
  const unavailable=loading||fileBusy||!session;
  $('start').disabled=unavailable;
  $('start').textContent=starting?'取消接入':active?'停止 / 冻结':'接入实时输入';
  for(const id of ['hand-model','side'])$(id).disabled=unavailable||starting||active;
  for(const id of ['input-source','input-side','camera'])$(id).disabled=unavailable||starting||active;
  for(const id of ['input-point','input-x','input-y','input-z','reset-input'])$(id).disabled=unavailable||active||starting;
  for(const id of ['calibration-point','point-link','calibration-x','calibration-y','calibration-z','reset-point','surface-snap'])$(id).disabled=unavailable;
  for(const button of document.querySelectorAll('[data-preset]'))button.disabled=unavailable||active||starting;
  $('save').disabled=unavailable||session?.save_allowed===false;
  $('export').disabled=unavailable;$('reload').disabled=unavailable||starting;
  inputViewer.setEditable(!unavailable&&!active&&!starting); staticViewer.setEditable(!unavailable);
  $('input-mode').textContent=active?'实时接入 · 21 点':'可拖动 · 21 点';
}
function fields(kind) {
  const index=Number($(kind+'-point').value||0);
  const xyz=kind==='input'?inputPoints[index]:rows[index]?.local_xyz;
  if(!xyz)return;
  ['x','y','z'].forEach((axis,i)=>$(kind+'-'+axis).value=(xyz[i]*1000).toFixed(3));
  if(kind==='calibration') {
    staticViewer.setBindings(rows);
    $('point-link').value=rows[index].link;
    staticViewer.snapLink=$('surface-snap').checked?rows[index].link:null;
  }
}
function selectPoint(kind,index) {
  $(kind+'-point').value=String(index);(kind==='input'?inputViewer:staticViewer).select(index);fields(kind);
}
function clearPreset() {
  inputViewer.setTarget(null);
  document.querySelectorAll('[data-preset]').forEach(button=>button.classList.remove('selected'));
}
function changed() {revision++;settleCount=stableCount=0;clearTimeout(settleTimer);$('solve-status').textContent='正在求解…';refresh();offer();}
function moveInput(index,point) {
  if(active||starting||loading||fileBusy)return;
  inputPoints[index]=point;inputViewer.setPoints(inputPoints);clearPreset();fields('input');changed();
}
function moveCalibration(index,point) {
  if(loading||fileBusy)return;
  rows[index].local_xyz=staticViewer.localPoint(rows[index].link,point);
  staticViewer.setPoints(rows.map(row=>staticViewer.worldPoint(row)));fields('calibration');changed();
}
function createQueue() {
  queue?.close();const token=generation;
  queue=new LatestFrameQueue({maxAgeMs:Infinity,
    process:async item=>{
      inFlight=api(item.url,item.packet).catch(error=>({error:error.message,edit_revision:item.packet.edit_revision}));
      try{return await inFlight;}finally{inFlight=null;}
    },
    onResult:(item,result)=>{
      if(token!==generation||result.edit_revision!==revision)return;
      if(result.error){solveFailed=true;$('solve-status').textContent='求解失败 · 保持上次结果';notice(result.error);return;}
      if(solveFailed){solveFailed=false;notice();}
      inputPoints=copy(result.input_points);inputViewer.setPoints(inputPoints);fields('input');
      resultViewer.setPose(result.pose);resultViewer.setPoints(result.result_points);
      resultViewer.pointGroup.visible=$('result-points').checked;
      $('solve-ms').textContent=`${result.solve_ms.toFixed(1)} ms`;
      $('solve-status').textContent=active?'实时求解 · 模型跟随':'求解更新 · 当前输入姿态';
      renderJoints(result.pose);
      if(item.url==='/api/frame')$('input-status').textContent=result.message+(item.packet.source==='realsense_rgbd'?' · '+realSenseFusionStatus(result.depth_fusion,result.tracking)+' · '+realSenseCameraStatus(item.packet.source_metadata?.camera,leapCamera?.inferenceFps)
        :leapCamera?' · '+leapCameraStatus(leapCamera.meta,leapCamera.inferenceFps)
        :browserCamera?' · '+cameraCaptureLabel(browserCamera):'');
      if(result.preset) {
        inputViewer.setTarget(result.preset.target);
        $('preset-detail').textContent=`${result.preset.label} · 拇指与目标间距 ${result.preset.gap_mm.toFixed(2)} mm。目标位置由骨架估算，可继续拖动调整。`;
      }
      if(!active&&!starting&&settleCount++<60) {
        stableCount=result.delta!==null&&result.delta<1e-5?stableCount+1:0;
        if(stableCount<3) {
          const edit=revision;
          settleTimer=setTimeout(()=>{if(edit===revision&&!active&&!starting&&!queue.pending)offer();},16);
        }
      }
    },onError:error=>notice(error.message),
  });
}
function offer({frame=null,preset=null}={}) {
  if(!session||loading||!queue)return;
  const packet={session:session.session,edit_revision:revision,calibration_rows:currentRows(),timestamp_ms:performance.now()};
  if(frame) Object.assign(packet,frame,{input_epoch:inputEpoch,sequence:sequence++});
  else if(preset)packet.preset=preset;
  else if(!active)packet.input_points=copy(inputPoints);
  queue.offer({url:frame?'/api/frame':'/api/calibration/update',packet});
}
function renderJoints(pose) {
  $('joint-values').replaceChildren(...Object.entries(pose).map(([name,value])=>{
    const row=document.createElement('div'),label=document.createElement('span'),number=document.createElement('span');
    label.textContent=name;number.textContent=(value*180/Math.PI).toFixed(1)+'°';row.append(label,number);return row;
  }));
}
function sourceUI() {
  const source=$('input-source').value;
  $('camera-control').hidden=source!=='mediapipe';
  $('source-help').textContent=source==='mediapipe'?'相机由当前浏览器直接采集；RealSense 自动选择最高可用采集帧率，再开始识别。冻结后可拖动输入关键点。'
    :source==='realsense_rgbd'?'D455f 的同帧 RGB + 深度由 Linux 服务采集；MediaPipe 提供 21 点初值，深度与骨长约束修正相对三维位置。深度不可用时保持姿态。'
    :isLeapCamera()?'Leap Motion 2 左侧红外相机 → MediaPipe。目标采集 120 fps；识别速度取决于 GPU，三维点来自 MediaPipe 估计。'
    :source==='webxr'?'复用 Quest WebXR 配对。电脑与头显访问同一可信 HTTPS 站点，25 关节自动转为 21 点。'
    :source==='leapmotion'?'Linux 服务需安装 Ultraleap Tracking Service 和 LeapC SDK，并连接 Leap Motion 2；将「输入手别」所选手放入设备视野，接收 21 点骨架。'
    :'复用当前服务的 Wuji SDK / 桥接配置；接入后接收所选手套的 21 点骨架。';
  updateLinks();
}
function updateLinks() {
  const params=new URLSearchParams({model:$('hand-model').value,side:$('side').value,source:$('input-source').value});
  $('tracking-link').href='/?'+params;history.replaceState(null,'','/calibration?'+params);
}
const cameraSelector=bindCameraSelector($('camera'),{onError:error=>notice(error.message)});
function cameras() {return cameraSelector.refresh();}
function closeLocalInput() {
  generation++;active=false;starting=false;clearTimeout(settleTimer);clearTimeout(videoTimer);
  cameraOpening?.abort();cameraOpening=null;browserCamera=null;
  receiver?.close();receiver=null;detector?.close();detector=null;
  leapCamera?.close();leapCamera=null;$('leap-camera-canvas').hidden=true;
  stream?.getTracks().forEach(track=>track.stop());stream=null;$('video').srcObject=null;$('video').hidden=true;
  queue?.close();$('quest-pairing').hidden=true;
}
async function stopInput() {
  closeLocalInput();refresh();await inFlight;
  if(session) {
    const state=await api('/api/calibration/input',{session:session.session,enabled:false});inputEpoch=state.input_epoch;
  }
  createQueue();$('input-status').textContent='输入已冻结 · 可手动拖点';refresh();offer();
}
async function startInput() {
  if(active||starting){await stopInput();return;}
  clearTimeout(settleTimer);queue?.close();await inFlight;
  const token=++generation;starting=true;refresh();notice();
  $('input-status').textContent='正在接入…';
  try {
    const state=await api('/api/calibration/input',{session:session.session,enabled:true,
      source:inputSource(),camera_source:isLeapCamera()?'leapmotion_camera':$('input-source').value==='realsense_rgbd'?'realsense_rgbd':'browser',
      side:$('input-side').value,calibration_rows:currentRows()});
    if(token!==generation)return;
    inputEpoch=state.input_epoch;sequence=0;clearPreset();
    const source=inputSource();
    if(source==='mediapipe'||source==='realsense_rgbd') {
      if(isLeapCamera()||source==='realsense_rgbd') {
        leapCamera=source==='realsense_rgbd'?new RealSenseCameraClient($('leap-camera-canvas')):new LeapCameraClient($('leap-camera-canvas'));
        $('leap-camera-canvas').ariaLabel=source==='realsense_rgbd'?'D455f 深度融合 RGB 画面':'Leap Motion 2 红外画面';
        $('leap-camera-canvas').hidden=false;
      } else {
      if(!navigator.mediaDevices?.getUserMedia)throw Error('相机需要 HTTPS 或 localhost，请从安全地址打开网页');
      const opening=new AbortController();cameraOpening=opening;
      const camera=await openMediaPipeCamera({deviceId:$('camera').value,label:$('camera').selectedOptions[0]?.textContent||'',signal:opening.signal,
        onStatus:message=>{if(token===generation)$('input-status').textContent=message;}});
      const acquired=camera.stream;
      if(token!==generation){acquired.getTracks().forEach(track=>track.stop());return;}
      cameraOpening=null;browserCamera=camera;
      stream=acquired;$('video').srcObject=stream;$('video').hidden=false;await $('video').play();
      if(token!==generation)return;
      stream.getVideoTracks()[0].addEventListener('ended',()=>{if(token===generation)void stopInput().then(()=>notice('摄像头已断开'));});
      }
      detector=new DetectorClient();const ready=await detector.ask({type:'init'});
      if(token!==generation)return;
      if(ready.fallbackReason)notice('浏览器 GPU 初始化失败，已使用 CPU 识别');
      if(!isLeapCamera()&&source!=='realsense_rgbd')await cameras();if(token!==generation)return;
    }
    starting=false;active=true;createQueue();refresh();
    $('input-status').textContent=(browserCamera?cameraCaptureLabel(browserCamera)+' · ':'')+'输入已接入 · 等待所选手的关键点';
    if(source==='mediapipe'||source==='realsense_rgbd') {void cameraFrame(token,-1);return;}
    const connection=await api(source==='webxr'?'/api/webxr/create':source==='leapmotion'?'/api/leapmotion/create':'/api/wuji/create',{session:session.session});
    if(token!==generation)return;
    if(source==='webxr') {
      $('quest-pairing').hidden=false;$('quest-code').textContent=connection.pairing_code;
      $('quest-link').value=new URL(connection.entry_path,location.href).href;$('quest-open').href=$('quest-link').value;
    }
    const Receiver=source==='webxr'?WebXRReceiver:source==='leapmotion'?LeapMotionReceiver:WujiReceiver;
    receiver=new Receiver({session:session.session,combined:false,staleAfterMs:connection.stale_after_ms,
      onFrame:snapshot=>{if(token===generation&&active)offer({frame:{source,
        [source==='webxr'?'webxr_revision':source==='leapmotion'?'leapmotion_revision':'wuji_revision']:snapshot.revision,
        source_epoch:snapshot.source_epoch??snapshot.source_metadata?.source_epoch,
        hand:snapshot.hand,capture_unix_ms:Date.now()}});},
      onStatus:snapshot=>{if(token===generation)$('input-status').textContent=source==='webxr'?questStatus(snapshot):source==='leapmotion'?leapMotionStatus(snapshot):wujiStatus(snapshot);},
      onError:async error=>{if(token===generation){await stopInput();notice(error.message);}},
    });void receiver.poll();
  } catch(error) {
    if(token!==generation)return;
    await stopInput().catch(()=>{});notice(`接入失败：${error.message}`);
  }
}
async function cameraFrame(token,lastTime) {
  if(token!==generation||!active)return;
  const video=$('video'), at=video.currentTime;
  try {
    const camera=leapCamera, rgbd=$('input-source').value==='realsense_rgbd';
    if(camera||(video.readyState>=2&&at!==lastTime)) {
      const bitmap=camera?await camera.read(session.session):await createImageBitmap(video);
      if(token!==generation){bitmap?.close();return;}
      const timestamp_ms=performance.now(), cameraMeta=camera?{...camera.meta}:null;
      const capture_unix_ms=rgbd&&Number.isFinite(cameraMeta?.capture_unix_ms)?cameraMeta.capture_unix_ms:Date.now()-(cameraMeta?.age_ms||0);
      if(camera)$('input-status').textContent=rgbd?realSenseCameraStatus(cameraMeta,camera.inferenceFps):leapCameraStatus(cameraMeta,camera.inferenceFps);
      const detection=bitmap?await detector.ask({type:'frame',bitmap,timestamp_ms},[bitmap]):{result:{}};
      if(token!==generation)return;
      if(camera&&bitmap) {
        camera.recordInference();
        if(cameraMeta.age_ms+performance.now()-timestamp_ms>=cameraMeta.stale_after_ms)detection.result={};
      }
      const result=detection.result, labels=result.handedness||result.handednesses||[];
      const index=labels.findIndex(categories=>categories[0]?.categoryName===$('input-side').value.replace(/^./,s=>s.toUpperCase()));
      const xyz=points=>points.map(p=>[p.x,p.y,p.z]);
      const world=result.worldLandmarks?.[index], normalized=result.landmarks?.[index];
      const hand=world?.length===21&&normalized?.length===21?{world:xyz(world),normalized:xyz(normalized),label:labels[index][0].categoryName,score:labels[index][0].score}:null;
      offer({frame:{source:'mediapipe',timestamp_ms,hand,capture_unix_ms,
        camera_size:cameraMeta?[cameraMeta.width,cameraMeta.height]:[video.videoWidth,video.videoHeight],
        ...(rgbd?realSenseFrameFields(cameraMeta):cameraMeta?{source_metadata:{camera:cameraMeta}}:{})}});
    }
    if(camera)queueMicrotask(()=>void cameraFrame(token,at));
    else videoTimer=setTimeout(()=>void cameraFrame(token,at),10);
  } catch(error) {if(token===generation){await stopInput().catch(()=>{});notice(`相机识别中断：${error.message}`);}}
}
function applyDescription(data,{resetInput=true}={}) {
  rows=copy(data.calibration_rows);baseline=copy(rows);
  if(resetInput)inputPoints=copy(data.input_points);
  staticViewer.setPoints(rows.map(row=>staticViewer.worldPoint(row)));inputViewer.setPoints(inputPoints);
  resultViewer.setPose(data.pose);resultViewer.setPoints(data.result_points);resultViewer.pointGroup.visible=$('result-points').checked;
  for(const kind of ['input','calibration']) {
    const selected=$(kind+'-point').value||'0';
    $(kind+'-point').replaceChildren(...rows.map(row=>new Option(`${String(row.id).padStart(2,'0')} · ${row.source_name}`,row.id)));
    selectPoint(kind,Number(selected));
  }
  $('calibration-path').textContent=data.path;
}
async function load() {
  loading=true;closeLocalInput();refresh();notice();await inFlight;
  if(session){await api('/api/stop',{session:session.session}).catch(()=>{});session=null;}
  try {
    const model=$('hand-model').value,side=$('side').value;
    const descriptions=await Promise.all([inputViewer.load(side,model),staticViewer.load(side,model),resultViewer.load(side,model)]);
    inputViewer.robot.root.visible=false;staticViewer.zeroPose();inputViewer.zeroPose();
    $('point-link').replaceChildren(...descriptions[1].links.map(link=>new Option(link.name,link.name)));
    session=await api('/api/session',{mode:'calibration',model_id:model,side,input_source:inputSource(),camera_source:$('input-source').value==='realsense_rgbd'?'realsense_rgbd':'browser',recording:false});
    applyDescription(session);
    $('common-presets').replaceChildren();$('kapandji-presets').replaceChildren();
    for(const preset of session.presets) {
      const button=document.createElement('button');button.textContent=preset.label;button.dataset.preset=preset.id;
      button.addEventListener('click',()=>applyPreset(preset.id));
      $(preset.id.startsWith('kapandji_')?'kapandji-presets':'common-presets').append(button);
    }
    revision++;settleCount=stableCount=0;createQueue();clearPreset();
    $('input-status').textContent='手动输入 · 初始骨架与标定相同';
    $('save-status').textContent=session.save_allowed?'保存会生成备份，仅写入当前模型与手别的标定。':'公共服务支持导出 YAML；在本地服务应用标定。';
  } catch(error){notice(`加载失败：${error.message}`);$('input-status').textContent='模型或标定加载失败，请刷新重试';}
  finally {loading=false;refresh();updateLinks();if(session)offer();}
}
function applyPreset(id) {
  if(active||starting||loading)return;
  revision++;settleCount=stableCount=0;clearTimeout(settleTimer);clearPreset();
  $('solve-status').textContent='正在求解…';
  document.querySelector(`[data-preset="${id}"]`)?.classList.add('selected');offer({preset:id});
}
async function fileAction(action) {
  if(!session||fileBusy)return;
  if(action==='reload'&&dirty&&!confirm(translate('重新加载会丢弃当前未保存的标定修改。是否继续？')))return;
  fileBusy=true;refresh();
  try {
    if(action==='reload') {
      await stopInput();queue.close();clearTimeout(settleTimer);await inFlight;
      const data=await api('/api/calibration/reload',{session:session.session});
      revision++;applyDescription(data,{resetInput:false});clearPreset();createQueue();
      $('save-status').textContent='已重新加载标定；保留当前输入姿态。';
    } else {
      const result=await api('/api/calibration/'+action,{session:session.session,calibration_rows:currentRows()});
      if(action==='save') {
        baseline=copy(result.calibration_rows);$('save-status').textContent=`已保存。备份：${result.backup}`;
      } else {
        const url=URL.createObjectURL(new Blob([result.yaml],{type:'application/yaml'}));
        const link=document.createElement('a');link.href=url;link.download=result.filename;link.click();setTimeout(()=>URL.revokeObjectURL(url),1000);
        $('save-status').textContent='已导出当前标定 YAML。';
      }
    }
    notice();
  } catch(error){notice(error.message);}
  finally {fileBusy=false;refresh();if(action==='reload')offer();}
}

$('start').addEventListener('click',()=>startInput().catch(error=>notice(error.message)));
for(const kind of ['input','calibration']) {
  $(kind+'-point').addEventListener('change',()=>selectPoint(kind,Number($(kind+'-point').value)));
  for(const axis of ['x','y','z'])$(kind+'-'+axis).addEventListener('change',()=>{
    const point=['x','y','z'].map(a=>$(kind+'-'+a).valueAsNumber/1000);
    if(!point.every(Number.isFinite)){fields(kind);notice('坐标必须是有限数值');return;}
    const index=Number($(kind+'-point').value);
    if(kind==='input')moveInput(index,point);
    else {rows[index].local_xyz=point;staticViewer.setPoints(rows.map(row=>staticViewer.worldPoint(row)));changed();}
  });
}
$('point-link').addEventListener('change',()=>{
  const index=Number($('calibration-point').value), world=staticViewer.worldPoint(rows[index]);
  rows[index].link=$('point-link').value;rows[index].local_xyz=staticViewer.localPoint(rows[index].link,world);fields('calibration');changed();
});
$('surface-snap').addEventListener('change',()=>fields('calibration'));
$('reset-point').addEventListener('click',()=>{
  const index=Number($('calibration-point').value);rows[index]=copy(baseline[index]);
  staticViewer.setPoints(rows.map(row=>staticViewer.worldPoint(row)));fields('calibration');changed();
});
$('reset-input').addEventListener('click',()=>applyPreset('neutral'));
$('reset-views').addEventListener('click',()=>[inputViewer,staticViewer,resultViewer].forEach(view=>view.resetView()));
$('result-points').addEventListener('change',()=>resultViewer.pointGroup.visible=$('result-points').checked);
for(const action of ['save','export','reload'])$(action).addEventListener('click',()=>fileAction(action));
$('quest-renew').addEventListener('click',async()=>{
  try{const result=await api('/api/webxr/code',{session:session.session});$('quest-code').textContent=result.pairing_code;notice();}
  catch(error){notice(error.message);}
});
$('input-source').addEventListener('change',sourceUI);
for(const id of ['hand-model','side'])$(id).addEventListener('change',()=>{
  if(dirty&&!confirm(translate('切换模型会丢弃未保存的标定修改。是否继续？'))){$('hand-model').value=session.model_id;$('side').value=session.side;return;}
  $('input-side').value=$('side').value;void load();
});
navigator.mediaDevices?.addEventListener('devicechange',()=>cameras().catch(()=>{}));
window.addEventListener('beforeunload',event=>{if(dirty){event.preventDefault();event.returnValue='';}});
document.addEventListener('visibilitychange',()=>{if(document.hidden&&(active||starting))void stopInput().catch(error=>notice(error.message));});
window.addEventListener('pagehide',()=>{
  closeLocalInput();if(session)navigator.sendBeacon('/api/stop',new Blob([JSON.stringify({session:session.session})],{type:'application/json'}));
  [inputViewer,staticViewer,resultViewer].forEach(view=>view.dispose());
});
setInterval(()=>{if(session)api('/api/calibration/touch',{session:session.session}).catch(error=>notice(error.message));},30000);
const params=new URLSearchParams(location.search);
for(const [key,id] of [['model','hand-model'],['side','side'],['source','input-source']]) {
  if([...$(id).options].some(option=>option.value===params.get(key)))$(id).value=params.get(key);
}
$('input-side').value=$('side').value;sourceUI();void cameras().catch(()=>{});void load();
