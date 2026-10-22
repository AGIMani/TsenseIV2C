import {LeapCameraClient, leapCameraStatus} from './leap-camera.js';
import {DetectorClient} from './detector-client.js';
import {CameraOverlay} from './overlay.js';

const $=id=>document.getElementById(id);
const overlay=new CameraOverlay($('overlay'),$('image'));
let generation=0, session=null, camera=null, detector=null;
async function api(path,data) {
  const response=await fetch(path,{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify(data),signal:AbortSignal.timeout(20000)});
  const body=await response.json();if(!response.ok)throw Error(body.error);return body;
}
async function stop() {
  generation++;camera?.close();camera=null;detector?.close();detector=null;
  const previous=session;session=null;overlay.clear();
  $('start').disabled=false;$('stop').disabled=true;$('state').textContent='相机已停止';
  $('delegate').textContent='已停止';$('notice').textContent='';$('notice').title='';
  $('hands').textContent='等待手部画面';
  if(previous)await api('/api/stop',{session:previous}).catch(()=>{});
}
async function start() {
  const token=++generation;
  $('start').disabled=true;$('stop').disabled=false;$('notice').textContent='';
  $('state').textContent='正在连接相机与加载 MediaPipe…';
  try {
    const created=await api('/api/session',{input_source:'mediapipe',camera_source:'leapmotion_camera',recording:false});
    if(token!==generation){await api('/api/stop',{session:created.session});return;}
    session=created.session;camera=new LeapCameraClient($('image'));detector=new DetectorClient();
    const ready=await detector.ask({type:'init'});
    if(token!==generation)return;
    $('delegate').textContent=`MediaPipe ${ready.delegate}`;
    if(ready.fallbackReason)$('notice').textContent='当前浏览器的 GPU 不可用，已切换 CPU 识别。';
    while(token===generation) {
      const bitmap=await camera.read(session);
      if(token!==generation){bitmap?.close();return;}
      const meta={...camera.meta}, timestamp_ms=performance.now();
      let detection={result:{}};
      if(bitmap&&detector) {
        try { detection=await detector.ask({type:'frame',bitmap,timestamp_ms},[bitmap]); }
        catch(error) {
          if(token!==generation)return;
          detector.close();detector=null;
          $('delegate').textContent='MediaPipe 不可用';
          $('notice').textContent='当前浏览器不能完成 MediaPipe 图像处理。请在支持 WebGL 2 的浏览器中启用图形加速后重试。红外相机继续采集。';
          $('notice').title=error.message;
        }
      } else bitmap?.close();
      if(token!==generation)return;
      if(bitmap&&detector)camera.recordInference();
      const fresh=meta.age_ms!=null&&meta.age_ms+performance.now()-timestamp_ms<meta.stale_after_ms;
      const hands=fresh?(detection.result.landmarks||[]):[];
      const labels=detection.result.handedness||detection.result.handednesses||[];
      overlay.setHands(hands,0,{timestampMs:timestamp_ms,labels:labels.map(categories=>categories[0]?.categoryName)});
      $('state').textContent=!detector&&meta.connected&&!meta.error&&fresh
        ?`相机采集 ${meta.capture_fps?.toFixed(1)??'测量中'} / 120 FPS · MediaPipe 未运行`+(meta.fps_status==='below_target'?' · 采集未达目标，请检查 USB 3 连接和系统负载':'')
        :leapCameraStatus(meta,camera.inferenceFps);
      $('hands').textContent=`${meta.width} × ${meta.height}`+(detector?` · 检出 ${hands.length} 只手 · 单次识别 ${(detection.inference_ms||0).toFixed(1)} ms`:' · 仅预览红外画面');
    }
  } catch(error) {
    if(token!==generation)return;
    await stop();$('notice').textContent=error.message;
  }
}
$('start').addEventListener('click',()=>void start());
$('stop').addEventListener('click',()=>void stop());
window.addEventListener('pagehide',()=>{
  camera?.close();detector?.close();
  if(session)fetch('/api/stop',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({session}),keepalive:true}).catch(()=>{});
});
