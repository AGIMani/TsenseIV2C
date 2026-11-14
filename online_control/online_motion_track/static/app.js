import { DetectorClient } from "./detector-client.js";
import { A32SdkPanel } from "./a32-sdk-control.js";
import { HandViewer, SkeletonViewer } from "./viewer.js";
import { CameraOverlay } from "./overlay.js";
import { WorkerRecorder } from "./worker-recorder.js";
import { DiagnosticLogger, errorDetails } from "./diagnostic-logger.js";
import { LatestFrameQueue } from "./latest-frame-queue.js";
import { WebXRReceiver, questStatus } from "./webxr-receiver.js";
import { WujiReceiver, wujiStatus } from "./wuji-receiver.js";
import { LeapMotionReceiver, leapMotionStatus } from "./leapmotion-receiver.js";
import { FeedbackPreview } from "./feedback-preview.js";
import { bindCameraSelector } from "./camera-selector.js";
import { openMediaPipeCamera, cameraCaptureLabel } from "./camera-stream.js";
import { LeapCameraClient, leapCameraStatus } from "./leap-camera.js";
import { RealSenseCameraClient, realSenseCameraStatus, realSenseFusionStatus, realSenseFrameFields } from "./realsense-camera.js";

const $ = (id) => document.getElementById(id);
const overlay = new CameraOverlay($("camera-overlay"), $("video"));
let recordingAllowed = true;
let capabilities = {}, hardware = {}, controlBusy = false, sessionPromise = null;
let stopping = false, lastDownloadUrl = null, downloadRecordingId = null;
const recorder = new WorkerRecorder(error => {
  // Seal without reading hundreds of MB back into the control page's thread.
  notice(`本机记录已停止，控制继续：${error.message}`);
  recorder.stop("recording_error",{exportFile:false}).then(()=>{
    $("record-status").textContent="记录已封存，控制继续；结束控制后生成下载文件";
    return refreshRecordings();
  }).catch(e=>notice(`记录封存失败：${e.message}；控制继续`));
});
let directControl = false;
const directMode = () => directControl;
const modelLabel = () => "A3.2 with ID";
let viewer, feedbackViewer, feedbackTimer, skeleton, stream, client, session, framePipeline;
let controlPreviewOnly = false;
let questReceiver;
let leapCamera; // Native image client for Leap IR or paired RealSense RGB-D.
let cameraOpening, browserCamera;
const isLeapCamera = () => $("input-source").value === "leapmotion_camera";
const isQuestAdb = () => $("input-source").value === "quest_adb";
const isWebXR = () => ["webxr", "quest_adb"].includes($("input-source").value);
const inputSource = () => isQuestAdb() ? "webxr" : isLeapCamera() ? "mediapipe" : $("input-source").value;
const isWuji = () => inputSource() === "wuji_glove";
const isLeapMotion = () => inputSource() === "leapmotion";
const isMetricInput = () => isWebXR() || isWuji() || isLeapMotion();
let generation = 0,
  active = false,
  starting = false,
  modelReady = false;
let latestWorld = null,
  frameCallback = null,
  lastVideoTime = -1;
let sequence = 0,
  fpsCount = 0,
  fpsStart = 0;

const a32Panel = new A32SdkPanel({api, ensureSession: ensureControlSession,
  currentSession: () => session, isRunning: () => active, notice,
  record: (type, data) => recorder.record(type, data), display: pose => {
    if (!controlPreviewOnly) viewer?.setPose(pose);
  }});

const feedbackPreview = new FeedbackPreview({
  request: data => api("/api/feedback-pose", data),
  // A3.2 supports signed swing; the viewer enforces its URDF bounds.
  display: pose => feedbackViewer?.setPose(pose),
  changed: renderFeedback,
});

function renderFeedback(state) {
  const messages = {
    disconnected: "未连接机械手",
    waiting: "等待关节遥测",
    live: "实测关节角 · 实时反馈",
    stale: "遥测已过期",
    incomplete: `缺少 ${state.missingCount || 0} 个关节反馈 · 缺失关节保持`,
    unsupported: "硬件反馈映射不可用",
    error: "遥测换算失败",
  };
  const holding = state.hasPose && !["live", "incomplete"].includes(state.status);
  const message = (messages[state.status] || messages.waiting) + (holding ? " · 保持上次有效姿态" : "");
  $("feedback-status").textContent = message;
  $("feedback-status").title = state.error || message;
  $("feedback-age").textContent = Number.isFinite(state.ageMs) ? `遥测 ${Math.round(state.ageMs)} ms` : "遥测 —";
  $("feedback-dot").classList.toggle("online", state.status === "live");
  $("feedback-dot").classList.toggle("stale", holding);
  $("feedback-empty").hidden = !!state.hasPose;
  $("feedback-empty").textContent = state.status === "disconnected" ? "连接机械手后显示遥测姿态" : message;
  if (feedbackViewer?.robot) feedbackViewer.robot.root.visible = !!state.hasPose;
}

function refreshFeedback() {
  if (!modelReady || document.hidden) return;
  const state = a32Panel.state;
  const age = state.telemetry_age_ms;
  feedbackPreview.update({
    connected: !!state.connected,
    supported: ["left", "right"].includes($("side").value),
    telemetry: {self: state.self || [], mit: state.mit || [], connected: !!state.connected,
      telemetry_age_ms: age, hand_side: state.hand_side, model_id: state.model_id},
    identity: state,
    receivedAt: Number.isFinite(age) && age >= 0 && Number.isFinite(a32Panel.updatedAt)
      ? a32Panel.updatedAt - age : null,
  });
  void feedbackPreview.tick();
}

const diagnostics=new DiagnosticLogger({context:()=>({model_id:$("hand-model").value,side:$("side").value,
  session_id:session?.session||null,collision_mode:$("collision-mode").value,visibility:document.visibilityState}),changed:()=>{
  $("diagnostic-status").textContent=diagnostics.fileError?`文件写入失败：${diagnostics.fileError} · 可下载浏览器日志`:
    diagnostics.storageError?`浏览器缓存失败：${diagnostics.storageError} · 请及时下载`:
    diagnostics.fileHandle?`自动写入 ${diagnostics.directoryName} / ${diagnostics.name}`:"诊断日志自动保存在本机浏览器";
}});
const recordAngles=recorder.record.bind(recorder);
recorder.record=(type,data)=>{diagnostics.controlEvent(type,data);return recordAngles(type,data);};
function diagnosticState(){
  return {active,starting,control_busy:controlBusy,model_ready:modelReady,direct_mode:directMode(),hardware:a32Panel.state,
    target_ready:a32Panel.state.target_ready,server_error:a32Panel.state.last_error,
    frame_pipeline:framePipeline?{...framePipeline.stats}:null,
    recording:{enabled:$("recording").checked,active:recorder.active,error:recorder.error?.message||null,
      events_sent:recorder.sent,worker_received:recorder.accepted,approx_bytes:recorder.bytes},
    tracking_transition:a32Panel.state.tracking_transition,frame_id:a32Panel.state.frame_id,
    camera_status:$("camera-state").textContent,tracking_status:$("tracking-text").textContent,
    arm_disabled:$("a32-arm").disabled,arm_blockers:$("a32-arm").title,notice:$("notice").textContent};
}
window.addEventListener("error",event=>diagnostics.record("javascript_error",{...errorDetails(event.error||event.message),file:event.filename,line:event.lineno,column:event.colno}));
window.addEventListener("unhandledrejection",event=>diagnostics.record("unhandled_rejection",errorDetails(event.reason)));
document.addEventListener("pointerdown",event=>{
  const button=event.target.closest?.("button");if(button?.id)diagnostics.record("button_press",{id:button.id,label:button.textContent,disabled:button.disabled,state:diagnosticState()});
},true);
document.addEventListener("change",event=>{const e=event.target;if(e.matches?.("select,input"))diagnostics.record("setting_change",{
  id:e.id,joint:e.dataset.joint,value:e.type==="checkbox"?e.checked:e.value});},true);
setInterval(()=>{if(active||a32Panel.state.connected||controlBusy)diagnostics.record("state_snapshot",diagnosticState());},1000);

function notice(message = "") {
  diagnostics.change("notice",{message});
  $("notice").textContent = message;
  $("notice").hidden = !message;
}
function refreshControls() {
  const busy = active || starting;
  $("side").disabled = busy;
  $("hand-model").disabled = starting || stopping || !modelReady || controlBusy;
  $("side").disabled = starting || stopping || !modelReady || controlBusy;
  $("camera").disabled = busy;
  $("input-source").disabled = busy || stopping || controlBusy || !!session || recorder.active;
  $("collision-mode").disabled = busy || !!session || controlBusy || directMode();
  $("recording").disabled = busy || !!session || recorder.active || !recordingAllowed;
  $("overlay-smoothing").disabled = starting || stopping;
  $("mediapipe-filter").disabled = starting || stopping;
  $("direct-preview").disabled = !modelReady || starting || stopping || controlBusy || a32Panel.busy || !!hardware.homing_active || !!hardware.power_busy || !!a32Panel.state.homing_active;
  $("direct-preview-note").textContent = directMode()
    ? "直控 · 关节限位后直接映射 · Ruckig、碰撞处理和额外关节平滑已关闭"
    : capabilities.a32_control_available ? "Ruckig 限速 · 碰撞处理可单独选择" : "预览模式 · 碰撞处理可单独选择";
  $("direct-preview").closest("label").title = "直控将限位后的映射目标直接下发，关闭 Ruckig、目标及采样碰撞处理和额外关节平滑。切换将结束当前会话，需手动重新开始识别并开启下发。";
  $("start").disabled = stopping || !modelReady || controlBusy || !!hardware.homing_active;
  $("start").classList.toggle("running", busy);
  $("start").querySelector("span").textContent = busy
    ? starting
      ? "取消启动"
      : "停止识别"
    : "开始识别";
  $("start")
    .querySelector("path")
    .setAttribute("d", busy ? "M5 5h10v10H5Z" : "m7 4 9 6-9 6Z");

}

function configureModelControls() {
  const select = $("collision-mode");
  select.replaceChildren(new Option("Mink · 避碰", "mink"), new Option("关闭保护 · 可下发", "off"));
  const modes = capabilities.a32_collision_modes?.[$("hand-model").value] || ["off"];
  for (const option of select.options) option.disabled = !modes.includes(option.value);
  select.value = directMode() || !modes.includes("mink") ? "off" : "mink";
  a32Panel.configure($("hand-model").value, $("side").value, capabilities.a32_control_available);
  $("model-version").textContent = modelLabel();
  hardware = {};
  refreshControls();
}

function renderHardware(snapshot) {
  if (snapshot) hardware = snapshot;
  refreshControls();
}

function showResponsePose(pose) { viewer.setPose(pose); }

async function ensureControlSession() {
  if (session) return session;
  if (!sessionPromise) {
    const token = generation;
    sessionPromise = api("/api/session", {model_id: $("hand-model").value, side: $("side").value,
      input_source: inputSource(),
      camera_source: isLeapCamera() ? 'leapmotion_camera' : $('input-source').value === 'realsense_rgbd' ? 'realsense_rgbd' : 'browser',
      recording: false, collision_mode: $("collision-mode").value, direct_mode:directMode()}).then(async created => {
      if (token !== generation) {
        await api("/api/stop", {session: created.session});
        throw new Error("启动已取消");
      }
      session = created;
      if (created.hardware) { a32Panel.update(created.hardware); a32Panel.begin(); }
      return created;
    }).finally(() => { sessionPromise = null; });
  }
  return sessionPromise;
}

async function ensureLocalRecording() {
  if (!$("recording").checked || recorder.active) return;
  await recorder.start({model_id: $("hand-model").value, hand_side: $("side").value,
    hand_name:$("recording-hand-name").value.trim(),
    direct_mode:directMode(),
    transport:"a32_native_sdk", hardware_location:"local Python service / native A3 SDK",
    sampling:"browser-received source/retarget results and SDK successful-write samples; SDK feedback cache polled about every 200 ms; full host-control log is recorded separately by the local service",
    source: `web/${inputSource()}`, self_collision_mode: $("collision-mode").value,
    collision_protection_enabled: !directMode() && $("collision-mode").value !== "off",
    control_order: (directMode()
      ? "MediaPipe -> retarget -> joint limits -> linkage/slot mapping -> native A3.2 SDK; no Ruckig, collision correction or extra joint smoothing"
      : "MediaPipe -> retarget -> model collision -> Ruckig -> sample collision -> A3.2 linkage/slot mapping -> native A3.2 SDK")
      .replace("MediaPipe", isLeapMotion() ? "Leap Motion 2 -> 21" : isWuji() ? "Wuji Glove 21" : isQuestAdb() ? "Quest USB ADB 21" : isWebXR() ? "Quest WebXR 25 -> 21" : "MediaPipe")});
  $("record-status").textContent = "关节角与遥测记录保存在本机浏览器 · 每段 512 MiB · 结束后保存";
}

function showDownload(file, automatic = false) {
  if (!file) return;
  if (lastDownloadUrl) URL.revokeObjectURL(lastDownloadUrl);
  lastDownloadUrl = file.url;
  for (const id of ["download-local"]) {
    const a = $(id); a.href = file.url; a.download = file.name; a.hidden = false;
  }
  downloadRecordingId=file.id;
  const count=file.partCount||1,select=$("recording-part");
  select.replaceChildren(...Array.from({length:count},(_,i)=>new Option(`第 ${i+1} / ${count} 段`,String(i))));
  select.value=String(file.partIndex||0);select.hidden=count===1;
  $("download-recording-part").hidden=count===1;
  $("record-status").textContent = `本机记录：${file.name}${count>1?` · 共 ${count} 段，请逐段下载`:""}${file.incomplete ? "（恢复的未完成记录）" : ""}`;
  if(file.savedPaths?.length)$("record-status").textContent=`已保存 ${file.savedPaths.length} 个文件：${file.savedPaths[0]}${file.incomplete?'（未完整封存）':''}`;
  if(file.saveError)$("record-status").textContent+=` · 目录保存失败：${file.saveError}；可下载恢复`;
  if (automatic&&!file.savedPaths?.length) $("download-local").click();
}
function gpuBadge(delegate = null) {
  $("gpu-badge").className =
    `chip ${delegate === "GPU" ? "ready" : delegate === "CPU" ? "cpu" : "muted"}`;
  $("gpu-badge").querySelector("span").textContent = delegate
    ? `${delegate} 识别`
    : "GPU 待初始化";
}
async function api(path, data) {
  const began=performance.now();let response;
  try{
  response = await fetch(path, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(data),
    signal: AbortSignal.timeout(15000),
  });
  const result = await response.json();
  if (!response.ok)
    throw new Error(result.error || `服务返回 ${response.status}`);
  const duration_ms=performance.now()-began;
  if(data.action&&data.action!=="status")diagnostics.record("api_action",{path,action:data.action,status:response.status,duration_ms});
  else if(duration_ms>100)diagnostics.controlEvent("api_slow",{path,action:data.action||null,duration_ms});
  return result;
  }catch(error){diagnostics.record("api_error",{path,action:data.action||null,status:response?.status||null,duration_ms:performance.now()-began,
    error:errorDetails(error),...(data.action==="browser_rebase"?{telemetry:data.telemetry,pose:data.pose}:{})});throw error;}
}
function cameraError(error) {
  if (error.name === "NotAllowedError")
    return "相机权限未开启。请在地址栏允许访问摄像头，再打开视频来源或开始识别。";
  if (error.name === "NotFoundError")
    return "没有找到摄像头。请连接或启用电脑内置摄像头后重试。";
  if (error.name === "NotReadableError")
    return "摄像头被其他程序占用或无法读取，请关闭其他相机程序后重试。";
  if (error.name === "OverconstrainedError")
    return "所选摄像头已不可用，请重新选择视频来源。";
  return error.message || String(error);
}

const cameraSelector = bindCameraSelector($("camera"), {
  onError: error => notice(cameraError(error)),
});
function enumerateCameras() { return cameraSelector.refresh(); }

async function loadModel() {
  updateCalibrationLink();
  modelReady = false;
  feedbackPreview.configure($("hand-model").value, $("side").value, []);
  $("feedback-loading").hidden = false;
  $("feedback-loading").querySelector("span:last-child").textContent = "加载完整 URDF 模型";
  $("model-loading").hidden = false;
  $("model-loading").querySelector("span:last-child").textContent =
    "加载完整 URDF 模型";
  $("side").disabled = true;
  refreshControls();
  $("side").disabled = true;
  try {
    const side = $("side").value;
    const model = $("hand-model").value;
    const [description] = await Promise.all([viewer.load(side, model), feedbackViewer.load(side, model)]);
    feedbackPreview.configure(model, side, Object.keys(description.initial_pose));
    $("model-caption").textContent = `TSENSEI ${modelLabel()} / ${side.toUpperCase()}${description.calibration?.pending ? " · 21 点待人工确认" : ""}`;
    $("collision-status").textContent = `原版模型 / ${Object.keys(description.initial_pose).length} 个关节`;
    $("model-loading").hidden = true;
    $("feedback-caption").textContent = `TSENSEI ${modelLabel()} / ${side.toUpperCase()} · 实测关节角`;
    $("feedback-loading").hidden = true;
    modelReady = true;
    refreshFeedback();
    notice();
  } catch (error) {
    $("model-loading").querySelector("span:last-child").textContent =
      "模型加载失败";
    $("feedback-loading").querySelector("span:last-child").textContent = "模型加载失败";
    notice(`模型加载失败：${error.message}。请检查服务并刷新页面。`);
  } finally {
    refreshControls();
  }
}

async function stop({
  message = "已停止，模型保持最后姿态",
  announce = true,
} = {}) {
  if (stopping) return;
  stopping = true;
  ++generation;
  cameraOpening?.abort();
  cameraOpening = null;
  browserCamera = null;
  questReceiver?.close();
  questReceiver = null;
  leapCamera?.close();
  leapCamera = null;
  $("leap-camera-state").textContent = '已停止';
  $("realsense-camera-state").textContent = '已停止';
  $("realsense-fusion-state").textContent = '等待同帧深度约束';
  $("quest-pairing").hidden = true;
  $("quest-link").value = "";
  $("quest-token").textContent = "";
  $("quest-state").textContent = "已停止";
  $("wuji-state").textContent = "已停止";
  $("leapmotion-state").textContent = "已停止";
  framePipeline?.close();
  framePipeline = null;
  controlPreviewOnly = false;
  active = false;
  starting = false;
  if (frameCallback !== null) {
    if ($("video").cancelVideoFrameCallback)
      $("video").cancelVideoFrameCallback(frameCallback);
    else cancelAnimationFrame(frameCallback);
    frameCallback = null;
  }
  client?.close();
  client = null;
  stream?.getTracks().forEach((track) => track.stop());
  stream = null;
  $("video").srcObject = null;
  overlay.clear();
  const oldSession = session;
  try { await a32Panel.finish(oldSession); }
  catch (error) { notice(`SDK 停止响应：${error.message}`); }
  session = null;
  a32Panel.reset();
  hardware = {};
  $("camera-placeholder").hidden = false;
  $("camera-state").textContent = "已停止";
  $("camera-caption").textContent = "CAMERA / 待机";
  $("camera-caption").parentElement.classList.remove("live");
  $("tracking-dot").classList.remove("online");
  $("tracking-text").textContent = message;
  $("fps").textContent = "—";
  $("solve-ms").textContent = "—";
  $("skeleton-status").textContent = "未检测";
  latestWorld = null;
  skeleton?.setPoints(null);
  gpuBadge();
  refreshControls();
  if (oldSession) {
    try {
      await api("/api/stop", { session: oldSession.session });
    } catch (error) {
      if (!active && !starting) notice(`停止服务连接失败：${error.message}`);
    }
  }
  try { showDownload(await recorder.stop(message), announce && !document.hidden); await refreshRecordings(); }
  catch (error) { notice(`本机记录导出失败：${error.message}。可使用“本机历史记录”恢复。`); }
  stopping = false;
  renderHardware();
}

function schedule(token) {
  if (!active || token !== generation) return;
  const callback = () => {
    frameCallback = null;
    processFrame(token);
  };
  // Native capture and long-poll supply the clock; never cap this input to display RAF/30 Hz.
  if (isLeapCamera() || $("input-source").value === "realsense_rgbd") { queueMicrotask(callback); return; }
  const video=$("video");
  // Consume a frame decoded during inference without waiting another camera
  // period. Only one inference is in flight; unchanged frames use video time.
  if(video.readyState>=2&&lastVideoTime>=0&&video.currentTime!==lastVideoTime){
    queueMicrotask(callback);
    return;
  }
  frameCallback = $("video").requestVideoFrameCallback
    ? $("video").requestVideoFrameCallback(callback)
    : requestAnimationFrame(callback);
}

async function processFrame(token) {
  if (!active || token !== generation) return;
  const video = $("video");
  if (!leapCamera && (video.readyState < 2 || video.currentTime === lastVideoTime)) {
    schedule(token);
    return;
  }
  lastVideoTime = video.currentTime;
  let timestamp_ms = performance.now(), capture_unix_ms = Date.now();
  try {
    const camera = leapCamera;
    const rgbd = $("input-source").value === "realsense_rgbd";
    const bitmap = camera ? await camera.read(session.session) : await createImageBitmap(video);
    if (token !== generation) { bitmap?.close(); return; }
    const cameraMeta = camera ? {...camera.meta} : null;
    if (camera) {
      timestamp_ms = performance.now();
      capture_unix_ms = rgbd && Number.isFinite(cameraMeta.capture_unix_ms) ? cameraMeta.capture_unix_ms : Date.now() - (cameraMeta.age_ms || 0);
      $(rgbd ? "realsense-camera-state" : "leap-camera-state").textContent = rgbd
        ? realSenseCameraStatus(cameraMeta, camera.inferenceFps) : leapCameraStatus(cameraMeta, camera.inferenceFps);
      $("camera-state").textContent = bitmap ? (rgbd ? 'RGB + 深度实时' : '红外实时') : (rgbd ? '等待 RGB + 深度' : '等待红外画面');
      $("camera-placeholder").hidden = !!bitmap;
      $("camera-caption").textContent = rgbd
        ? `D455f RGB-D / ${cameraMeta.width || "—"} × ${cameraMeta.height || "—"} · 同帧深度融合`
        : `LEAP IR / ${cameraMeta.width} × ${cameraMeta.height} · 目标 120 FPS`;
      $("camera-caption").parentElement.classList.toggle('live', !!bitmap);
    }
    if (!bitmap) {
      overlay.clear();
      const frameId = sequence++, expected = $("side").value === 'left' ? 'Left' : 'Right';
      framePipeline.offer({frameId, hand:null, expected, timestamp_ms, inferredAt:performance.now(),
        packet:{session:session.session, sequence:frameId, timestamp_ms, capture_unix_ms,
          mediapipe_filter_enabled: $("mediapipe-filter").checked,
          hand:null, ...(rgbd ? realSenseFrameFields(cameraMeta) : {source:'mediapipe', source_metadata:{camera:cameraMeta}})}});
      schedule(token); return;
    }
    const bitmapReadyAt=performance.now();
    if (token !== generation) {
      bitmap.close();
      return;
    }
    const detection = await client.ask(
      { type: "frame", bitmap, timestamp_ms },
      [bitmap],
    );
    const detectionReadyAt=performance.now();
    if (token !== generation) return;
    if (camera) {
      camera.recordInference();
      $(rgbd ? "realsense-camera-state" : "leap-camera-state").textContent = rgbd
        ? realSenseCameraStatus(cameraMeta, camera.inferenceFps) : leapCameraStatus(cameraMeta, camera.inferenceFps);
      if (cameraMeta.age_ms + detectionReadyAt - timestamp_ms >= cameraMeta.stale_after_ms) {
        detection.result = {}; // Never reuse a hand from an expired camera observation.
      }
    }
    // Register before UI/log serialization so they cannot delay registration
    // past the next video tick. The previous inference has already completed.
    schedule(token);
    const expected = $("side").value === "left" ? "Left" : "Right";
    const result = detection.result;
    const handedness = result.handedness || result.handednesses || [];
    let index = handedness.findIndex(
      (categories) => categories[0]?.categoryName === expected,
    );
    if (index < 0 && handedness.length) index = 0;
    const xyz = (points) => points.map((p) => [p.x, p.y, p.z]);
    const allWorld = (result.worldLandmarks || []).map(xyz);
    const allNormalized = (result.landmarks || []).map(xyz);
    const world = allWorld[index], normalized = allNormalized[index];
    const hand =
      index >= 0 && world?.length === 21 && normalized?.length === 21
        ? {
            label: handedness[index][0].categoryName,
            score: handedness[index][0].score,
            world,
            normalized,
          }
        : null;
    // Inference and retarget overlap. The queue permits one request plus one
    // newest pending detection, preserving order without retaining old frames.
    const frameId = sequence++;
    // RGB-D may have valid image landmarks but unusable depth. Only the
    // accepted fused response can satisfy the local hand-confirmation gate.
    const packet = {
      session: session.session,
      sequence: frameId,
      mediapipe_filter_enabled: $("mediapipe-filter").checked,
      timestamp_ms,
      capture_unix_ms,
      camera_size: cameraMeta ? [cameraMeta.width, cameraMeta.height] : [video.videoWidth, video.videoHeight],
      ...(rgbd ? realSenseFrameFields(cameraMeta) : cameraMeta ? {source:'mediapipe', source_metadata:{camera:cameraMeta}} : {}),
      hand,
      inference_ms: detection.inference_ms || 0,
      raw_detection: {
        world: allWorld,
        normalized: allNormalized,
        labels: handedness.map(c => ({label: c[0]?.categoryName, score: c[0]?.score})),
      },
    };
    framePipeline.offer({packet,frameId,hand,expected,timestamp_ms,inferredAt:performance.now()});
    // Start transport before the canvas draw and log snapshot. Recording
    // remains ordered: fetch resolves asynchronously, after this raw record.
    recorder.record(rgbd ? "realsense_rgbd_raw" : "mediapipe_raw", {frame_id: frameId, tracking_session_id:session.session, timestamp_ms, infer_ms: detection.inference_ms || 0,
      mediapipe_filter_enabled: packet.mediapipe_filter_enabled,
      overlay_smoothing_enabled: $("overlay-smoothing").checked,
      ...(cameraMeta ? {camera:cameraMeta} : {}),
      capture_timing:{bitmap_ms:bitmapReadyAt-timestamp_ms,
        worker_roundtrip_ms:detectionReadyAt-bitmapReadyAt,capture_to_detection_ms:detectionReadyAt-timestamp_ms},
      normalized_landmarks: allNormalized,
      world_landmarks: allWorld,
      handedness: handedness.map(c => c.map(v => ({label: v.categoryName, score: v.score})))});
    overlay.setHands(
      result.landmarks || [],
      handedness[index]?.[0]?.categoryName === expected ? index : -1,
      {timestampMs: timestamp_ms, labels: handedness.map(categories => categories[0]?.categoryName)},
    );
  } catch (error) {
    if (token !== generation) return;
    await stop({ message: "识别已中断" });
    notice(cameraError(error));
  }
}

function submitQueuedFrame(frame) {
    // Measure with the browser's monotonic clock, without rewriting capture
    // time. The server independently includes transport, lock and solve time.
    const packet = frame.packet;
    if ((packet.source || "mediapipe") === "mediapipe" && session?.hardware
        && !(Number.isFinite(session.camera_freshness_limit_ms) && session.camera_freshness_limit_ms > 0)) {
      throw new Error("相机预览已更新，请先重启本机手势服务以启用 H7 观测新鲜度检查，再开始识别");
    }
    return api("/api/frame", (packet.source || "mediapipe") === "mediapipe"
      ? {...packet, source_age_ms: performance.now() - packet.timestamp_ms} : packet);
}

function applyFrameUpdate(frame, update) {
    const {frameId,hand,expected,timestamp_ms} = frame;
    const fromWebXR = frame.packet?.source === "webxr";
    const source = frame.packet?.source || "mediapipe";
    const controlAccepted = !!update.tracking && update.control_accepted !== false;
    if (update.points) recorder.record(source, {landmarks: update.points, frame_id: frameId,
      mediapipe_filter_enabled: update.mediapipe_filter_enabled,
      hand_side: $("side").value, source,
      depth_source: update.depth_source || (source === "mediapipe" ? "mediapipe_world" : `${source}_metric`), timestamp_ms});
    if (update.actuation?.model_id === "TsenseI_v3_2_withid") {
      recorder.record("a32_conversion", {frame_id: frameId, ...update.actuation});
    }
    const rawPose = update.retarget_joint_angles_rad;
    // A completed solve can still be rejected by collision/control gates.
    // Preserve the raw result independently of permission to move the hand.
    const hasRetarget=rawPose&&typeof rawPose==='object'&&Object.keys(rawPose).length>0&&Object.values(rawPose).every(Number.isFinite);
    if (hasRetarget) {
      recorder.record("retarget", {joints: rawPose, frame_id: frameId, tracking_session_id:frame.packet?.session, hand_side: $("side").value,
        solve_ms: update.solve_ms, landmark_repairs: [],tracking:!!update.tracking,control_accepted:controlAccepted,
        source_age_at_solve_ms:update.source_age_at_solve_ms??null,
        server_session_timing:update.server_session_timing,solver_process_roundtrip_ms:update.solver_process_roundtrip_ms,
        message:update.control_message||update.message||""});
    } else {
      recorder.record("retarget_skipped", {frame_id: frameId, tracking_session_id:frame.packet?.session, hand_side: $("side").value,
        timestamp_ms, detected_label: hand?.label || null, expected_label: expected,
        reason: !hand ? "no_hand" : hand.label !== expected ? "wrong_hand"
          : rawPose ? "invalid_retarget" : update.tracking ? "missing_retarget" : "solve_rejected",
        message: update.message || "", tracking: !!update.tracking});
    }
    const context = {frame_id:frameId, tracking_session_id:frame.packet?.session,
      hand_side:$("side").value, source};
    if (update.mink_joint_angles_rad) recorder.record('mink', {...context, joints:update.mink_joint_angles_rad});
    if (update.direct_joint_angles_rad) recorder.record('direct_command', {...context, joints:update.direct_joint_angles_rad});
    if(hasRetarget&&controlAccepted){
      recorder.record("control_target", {
        frame_id: frameId, hand_side: $("side").value, self_collision: update.collision,
        pipeline_timing:{capture_to_response_ms:source==='mediapipe'?performance.now()-timestamp_ms:null,
          source_age_at_receive_ms:frame.sourceAgeMs??null,
          source_age_at_solve_ms:update.source_age_at_solve_ms??null,
          source_to_response_upper_ms:Number.isFinite(frame.sourceAgeMs)
            ?frame.sourceAgeMs+performance.now()-(frame.receivedAt??timestamp_ms):null,
          queue_wait_ms:frame.requestStartedAt-(frame.queuedAt??frame.inferredAt),
          request_response_ms:performance.now()-frame.requestStartedAt}});
    }else if(hasRetarget){
      recorder.record("control_target_skipped",{frame_id:frameId,tracking_session_id:frame.packet?.session,
        hand_side:$("side").value,reason:update.control_skip_reason||'control_rejected',
        source_age_at_solve_ms:update.source_age_at_solve_ms??null,
        message:update.control_message||update.message||"",self_collision:update.collision});
    }
    // Mode switches may race an older /api/frame response. Never display the
    // wrong source or let a sampled trajectory overwrite direct targets.
    const displayCurrent = !update.background_discarded && !update.display_discarded
      && !!update.direct_mode === directMode();
    if (displayCurrent) {
      controlPreviewOnly = !!update.tracking && update.control_accepted === false;
      showResponsePose(update.hardware?.armed && !controlPreviewOnly
        ? update.command_joint_angles_rad || update.pose : update.pose);
    }
    if (update.hardware&&!update.background_discarded&&!update.display_discarded) {
      a32Panel.update(update.hardware);
    }
    const collision = update.collision;
    $("collision-status").textContent = directMode() ? "直控 · 碰撞处理关闭 · 关节限位后直接映射" : collision?.mode === "mink" && update.tracking
      ? `Mink · 间距 ${collision.min_distance_mm.toFixed(2)} mm · 修正 ${collision.correction_deg.toFixed(1)}°`
      : $("collision-mode").value === "mink" ? "Mink 避碰 · 等待手势" : "避碰关闭 · 原始映射";
    $("collision-status").title = collision?.mode === "mink" && update.tracking
      ? `原始间距 ${collision.raw_min_distance_mm.toFixed(2)} mm；避碰耗时 ${collision.solve_ms.toFixed(1)} ms；${collision.closest_pair.join(" ↔ ")}；${collision.status}`
      : "距离按碰撞代理计算；停止识别后可以切换模式对比。";
    $("solve-ms").textContent = update.tracking
      ? update.solve_ms.toFixed(1)
      : "—";
    $("tracking-dot").classList.toggle("online", update.tracking);
    const plannerBlocked = update.hardware?.tracking_transition?.phase === "blocked";
    const previewMessage = update.tracking && update.control_accepted === false
      ? update.control_message || "预览已更新 · 观测过期，未更新 H7 目标" : null;
    $("tracking-text").textContent = previewMessage || (plannerBlocked
      ? `手势已识别 · 模型保持：${update.hardware.last_error || "控制器已暂停"}` : update.tracking
      ? `${expected === "Left" ? "左手" : "右手"}已识别 · 正在跟随`
      : update.message);
    $("tracking-text").title = previewMessage || (plannerBlocked ? update.hardware.last_error || "控制器已暂停" : update.message);
    if (displayCurrent) {
      // Render the solved pose's calibrated 21 points, never raw camera landmarks.
      const optimized = update.optimized_points;
      latestWorld = update.tracking && hasRetarget && Array.isArray(optimized)
        && optimized.length === 21 && optimized.every(p => Array.isArray(p)
          && p.length === 3 && p.every(Number.isFinite)) ? optimized : null;
      $("skeleton-status").textContent = latestWorld
        ? `${fromWebXR ? (isQuestAdb() ? "USB ADB · 21 点" : "WebXR 25 → 21 点") : source === "wuji_glove" ? "Wuji 21 点" : source === "leapmotion" ? "Leap Motion 2 · 21 点" : source === "realsense_rgbd" ? "D455f 深度融合 · 21 点" : "21 点"} · 优化后 · ${expected === "Left" ? "左手" : "右手"}镜像 · 实时`
        : update.tracking ? "等待优化结果" : "未检测";
      skeleton?.setPoints(latestWorld, expected);
    }
    if (source === "realsense_rgbd") {
      $("realsense-fusion-state").textContent = realSenseFusionStatus(update.depth_fusion, update.tracking);
      recorder.record("realsense_depth_fusion", {frame_id:frameId, tracking_session_id:frame.packet?.session,
        source, depth_fusion:update.depth_fusion, tracking:update.tracking,
        realsense_frame_id:frame.packet?.realsense_frame_id, realsense_stream_id:frame.packet?.realsense_stream_id});
    }
    ++fpsCount;
    const now = performance.now();
    if (now - fpsStart >= 1000) {
      const rate = (fpsCount * 1000) / (now - fpsStart);
      $("fps").textContent = rate.toFixed(rate < 10 ? 1 : 0);
      fpsCount = 0;
      fpsStart = now;
    }
}

function offerQuestFrame(token, snapshot) {
  if (!active || token !== generation || !session) return;
  const receivedAt=performance.now();
  const directMetric=snapshot.combined_frame&&snapshot.update;
  const timestamp_ms = directMetric?snapshot.request_started_ms:performance.now();
  const frameId = directMetric?snapshot.update.sequence:snapshot.combined_frame?null:sequence++;
  const expected = $("side").value === "left" ? "Left" : "Right";
  const hand = snapshot.hand;
  const source = snapshot.source || "webxr";
  const packet = {session:session.session, source,
    [source === "wuji_glove" ? "wuji_revision" : source === "leapmotion" ? "leapmotion_revision" : "webxr_revision"]:snapshot.revision,
    sequence:frameId, timestamp_ms, capture_unix_ms:Date.now(), camera_size:[0,0], hand,
    source_epoch:snapshot.source_epoch??snapshot.source_metadata?.source_epoch,
    source_metadata:snapshot.source_metadata, raw_webxr:snapshot.raw_webxr, raw_wuji:snapshot.raw_wuji, raw_leapmotion:snapshot.raw_leapmotion};
  recorder.record(`${source}_raw`, {frame_id:frameId, tracking_session_id:session.session,
    observation_revision:snapshot.revision,update_revision:snapshot.update_revision??null,
    timestamp_ms, source, age_ms:Number.isFinite(snapshot.age_ms)?snapshot.age_ms:null,
    tracked:!!hand, source_epoch:packet.source_epoch,
    source_metadata:snapshot.source_metadata, raw_webxr:snapshot.raw_webxr, raw_wuji:snapshot.raw_wuji, raw_leapmotion:snapshot.raw_leapmotion});
  const frame={packet,frameId,hand,expected,timestamp_ms,inferredAt:timestamp_ms,receivedAt,
    sourceAgeMs:Number.isFinite(snapshot.age_ms)?snapshot.age_ms:null};
  if(directMetric){
    frame.requestStartedAt=timestamp_ms;
    applyFrameUpdate(frame,snapshot.update);
  }else if(!snapshot.combined_frame)framePipeline.offer(frame);
  else if(!hand){
    // A failed read has no server solve to render. Clear the local tracking
    // indication immediately while the independent server lease expires.
    const message=snapshot.connection_error?'输入接收中断，正在重试':'等待新的有效骨架';
    $("tracking-dot").classList.remove("online");
    $("tracking-text").textContent=message;
    $("tracking-text").title=message;
    $("solve-ms").textContent="—";
    $("skeleton-status").textContent="未检测";
    latestWorld=null;skeleton?.setPoints(null);
  }
}

function updateCalibrationLink() {
  const params = new URLSearchParams({model: $("hand-model").value, side: $("side").value, source: isLeapCamera() ? 'leapmotion_camera' : inputSource()});
  $("calibration-link").href = '/calibration?' + params;
}

function configureInputSource() {
  updateCalibrationLink();
  const quest = isWebXR();
  const wuji = isWuji(), leap = isLeapMotion(), metric = isMetricInput();
  const rgbd = $("input-source").value === "realsense_rgbd", nativeCamera = isLeapCamera() || rgbd;
  $("quest-input-panel").hidden = !quest;
  $("wuji-input-panel").hidden = !wuji;
  $("leapmotion-input-panel").hidden = !leap;
  $("leap-camera-panel").hidden = !isLeapCamera();
  $("realsense-camera-panel").hidden = !rgbd;
  $("camera-control").hidden = metric || nativeCamera;
  $("overlay-smoothing-control").hidden = metric;
  $("mediapipe-filter-control").hidden = inputSource() !== "mediapipe";
  $("video").hidden = nativeCamera;
  $("leap-camera-canvas").hidden = !nativeCamera;
  $("leap-camera-canvas").setAttribute("aria-label", rgbd ? "D455f 深度融合 RGB 画面" : "Leap Motion 2 红外画面");
  overlay.video = nativeCamera ? $("leap-camera-canvas") : $("video");
  $("camera-title").textContent = "原始关键点输入";
  $("video").parentElement.querySelector(".mirror-label").hidden = metric;
  const placeholder = $("camera-placeholder");
  placeholder.querySelector("h3").textContent = leap ? "用 Leap Motion 2 采集手势" : wuji ? "用 Wuji 手套采集手势" : quest ? "用 Quest 采集手势" : "让手势进入画面";
  placeholder.querySelector("p").textContent = leap ? "启动 Linux Ultraleap Tracking Service 后点击「开始识别」，将所选手放入设备视野" : wuji ? "连接手套后点击「开始识别」，在下方查看骨架与模型" : quest ? "开始识别后，按上方连接信息在 Quest 进入 XR" : "点击「开始识别」，允许浏览器使用相机";
  placeholder.querySelector(":scope > span").textContent = metric ? "在下方骨架视图查看 21 点与模型映射" : "建议保持手掌完整入镜";
  if (isLeapCamera()) {
    placeholder.querySelector('h3').textContent = '用 Leap Motion 2 红外相机识别手势';
    placeholder.querySelector('p').textContent = '通过 USB 3 连接到服务所在电脑，点击「开始识别」；MediaPipe 从红外画面检测手部';
  }
  if (rgbd) {
    placeholder.querySelector('h3').textContent = '用 D455f 深度约束修正 21 点骨架';
    placeholder.querySelector('p').textContent = '通过 USB 3 连接到服务所在电脑，点击「开始识别」；同帧彩色与深度由 Linux 服务采集';
  }
  $("quest-origin-note").textContent = isQuestAdb()
    ? "USB 模式：先连接 Quest 并允许 USB 调试，在电脑运行 bash migration/ubuntu/start-quest-adb.sh。启动器建立 ADB 转发；开始识别后，在头显打开上方 localhost 采集地址。每只手发送 21 个关节点（米）。"
    : location.protocol !== "https:" || ["localhost","127.0.0.1","[::1]"].includes(location.hostname)
    ? "Quest 需要可访问且证书受信任的 HTTPS 地址。电脑可使用本机地址控制；头显请使用上方单独显示的局域网采集地址。电脑停止后，采集连接随之停止。"
    : "两台设备需使用同一 HTTPS 站点。放下控制器，将手保持在头显视野内；350 ms 无新数据会进入现有手势丢失处理。";
  refreshControls();
}

async function start() {
  diagnostics.record("tracking_button",{state:diagnosticState()});
  if (active || starting) {
    await stop();
    return;
  }
  const token = ++generation;
  starting = true;
  refreshControls();
  notice();
  $("tracking-text").textContent = isLeapMotion() ? "正在连接 Leap Motion 2…" : isWuji() ? "正在连接 Wuji 手套…" : isWebXR() ? "正在创建 Quest 输入会话…" : "正在打开相机与识别引擎…";
  try {
    await ensureLocalRecording();
    if (!isMetricInput()) {
    if (!isLeapCamera() && $("input-source").value !== "realsense_rgbd") {
    if (!window.isSecureContext || !navigator.mediaDevices?.getUserMedia)
      throw new Error(
        "访问公网摄像头需要 HTTPS。请使用本站的 https:// 地址；本机开发可使用 localhost。",
      );
    const selected = $("camera").value;
    const opening = new AbortController();
    cameraOpening = opening;
    const camera = await openMediaPipeCamera({
      deviceId: selected, label: $("camera").selectedOptions[0]?.textContent || '', signal: opening.signal,
      onStatus: message => {
        if (token !== generation) return;
        $("camera-state").textContent = '配置相机';
        $("tracking-text").textContent = message;
      },
    });
    const acquired = camera.stream;
    if (token !== generation) {
      acquired.getTracks().forEach((track) => track.stop());
      return;
    }
    cameraOpening = null;
    browserCamera = camera;
    stream = acquired;
    diagnostics.record('camera_configured', {realSense: camera.realSense, maximum_fps: camera.maxFrameRate,
      width: camera.settings.width, height: camera.settings.height, frameRate: camera.settings.frameRate});
    stream.getVideoTracks()[0].addEventListener("ended", () => {
      if (token === generation) {
        stop({ message: "摄像头已断开" });
        notice("摄像头已断开，请重新连接后开始。");
      }
    });
    $("video").srcObject = stream;
    await $("video").play();
    if (token !== generation) return;
    $("camera-placeholder").hidden = true;
    $("camera-state").textContent = "启动中";
    } else {
      leapCamera = $("input-source").value === "realsense_rgbd"
        ? new RealSenseCameraClient($("leap-camera-canvas")) : new LeapCameraClient($("leap-camera-canvas"));
    }
    client = new DetectorClient();
    const ready = await client.ask({ type: "init" });
    if (token !== generation) return;
    gpuBadge(ready.delegate);
    if (ready.fallbackReason)
      notice(
        "当前浏览器的 GPU 识别初始化失败，已切换为 CPU。可在浏览器设置中检查图形加速是否开启。",
      );
    }
    const created = await ensureControlSession();
    if (token !== generation) {
      await api("/api/stop", { session: created.session });
      return;
    }
    session = created;
    controlPreviewOnly = false;
    if (created.hardware) a32Panel.update(await api('/api/a32/control', {session: created.session, action: 'tracking_start'}));
    showResponsePose(created.pose);
    $("record-status").textContent = recorder.active
      ? "关节角与遥测记录保存在本机浏览器 · 每段 512 MiB · 结束后保存"
      : "角度记录已关闭";
    if (!isMetricInput() && !isLeapCamera() && $("input-source").value !== "realsense_rgbd") await enumerateCameras();
    if (token !== generation) return;
    starting = false;
    framePipeline = new LatestFrameQueue({
      process: frame => submitQueuedFrame(frame),
      onDrop:(frame,reason,error)=>recorder.record("retarget_skipped",{
        frame_id:frame.frameId,tracking_session_id:frame.packet.session,hand_side:$("side").value,
        reason,tracking:false,stage:'browser_queue',age_ms:performance.now()-frame.timestamp_ms,
        queue_wait_ms:performance.now()-frame.queuedAt,
        message:error?.message||null}),
      onResult: (frame,update) => {
        if(active && token===generation)applyFrameUpdate(frame,update);
      },
      onError: async error => {
        if(token!==generation)return;
        await stop({message:"识别已中断"});notice(cameraError(error));
      },
    });
    active = true;
    sequence = 0;
    fpsCount = 0;
    fpsStart = performance.now();
    lastVideoTime = -1;
    if (isLeapMotion()) {
      const connection = await api('/api/leapmotion/create', {session:created.session});
      if(token !== generation)return;
      $("camera-caption").textContent = "LEAP MOTION 2 / 21 POINTS";
      $("gpu-badge").className = "chip ready";
      $("gpu-badge").querySelector("span").textContent = "Leap Motion 2";
      questReceiver = new LeapMotionReceiver({session:created.session, staleAfterMs:connection.stale_after_ms,
        combined:connection.combined_frame===true, background:connection.background_processing===true,
        onFrame:snapshot=>offerQuestFrame(token,snapshot),
        onDiagnostic:(event,detail)=>diagnostics.record(event,detail),
        onStatus:snapshot=>{
          if(token!==generation)return;
          $("leapmotion-state").textContent=leapMotionStatus(snapshot);
          $("camera-state").textContent=snapshot.hand?"实时":"等待 Leap Motion 2";
          $("camera-placeholder").querySelector("h3").textContent=snapshot.hand?"Leap Motion 2 已连接":"等待 Leap Motion 2 手势";
          $("camera-placeholder").querySelector("p").textContent=leapMotionStatus(snapshot);
        },
        onError:async error=>{if(token===generation){await stop({message:"Leap Motion 2 输入已停止"});notice(error.message);}},
      });
      refreshControls();
      void questReceiver.poll();
      return;
    }
    if (isWuji()) {
      const connection = await api('/api/wuji/create', {session:created.session});
      if(token !== generation)return;
      $("camera-caption").textContent = "WUJI / 21 POINTS";
      $("gpu-badge").className = "chip ready";
      $("gpu-badge").querySelector("span").textContent = "Wuji 手套";
      questReceiver = new WujiReceiver({session:created.session, staleAfterMs:connection.stale_after_ms,
        combined:connection.combined_frame===true,
        background:connection.background_processing===true,
        onFrame:snapshot=>offerQuestFrame(token,snapshot),
        onDiagnostic:(event,detail)=>diagnostics.record(event,detail),
        onStatus:snapshot=>{
          if(token!==generation)return;
          $("wuji-state").textContent=wujiStatus(snapshot);
          $("camera-state").textContent=snapshot.hand?"实时":"等待手套";
          $("camera-placeholder").querySelector("h3").textContent=snapshot.hand?"Wuji 手套已连接":"等待 Wuji 手套";
          $("camera-placeholder").querySelector("p").textContent=wujiStatus(snapshot);
        },
        onError:async error=>{if(token===generation){await stop({message:"手套输入已停止"});notice(error.message);}},
      });
      refreshControls();
      void questReceiver.poll();
      return;
    }
    if (isWebXR()) {
      const adb = isQuestAdb();
      const pairing = await api("/api/webxr/create", {session: created.session, transport: adb ? "adb" : "webxr"});
      if (token !== generation) return;
      $("quest-link").value = adb
        ? (pairing.adb_entry_url ?? pairing.entry_url ?? new URL(pairing.adb_entry_path || pairing.entry_path || "/q/adb", location.href).href)
        : pairing.entry_url ?? new URL(pairing.entry_path, location.href).href;
      $("quest-link").placeholder = pairing.connection_error ? (adb ? "请通过 Quest ADB 启动器连接" : "局域网采集地址暂不可用") : "";
      $("quest-copy").disabled = !$("quest-link").value;
      $("quest-connection-note").textContent = pairing.connection_error || (adb
        ? "USB ADB · 21 点：请保持 USB 连接，用 Quest ADB 启动器建立转发后，在头显打开上方 localhost 采集地址，然后进入 XR。"
        : "电脑与 Quest 请连接同一局域网；在头显打开上方 HTTPS 地址。");
      $("quest-token").textContent = pairing.pairing_code;
      $("quest-pairing").hidden = false;
      $("camera-state").textContent = "等待 Quest";
      $("camera-caption").textContent = adb ? "QUEST / USB ADB · 21" : "QUEST / WEBXR 25 → 21";
      $("gpu-badge").className = "chip ready";
      $("gpu-badge").querySelector("span").textContent = adb ? "Quest USB ADB" : "Quest WebXR";
      questReceiver = new WebXRReceiver({session: created.session,
        staleAfterMs: pairing.stale_after_ms,
        combined:pairing.combined_frame===true, background:pairing.background_processing===true,
        onDiagnostic: (event, detail) => diagnostics.record(event, detail),
        onFrame: snapshot => offerQuestFrame(token, snapshot),
        onStatus: snapshot => {
          if (token !== generation) return;
          $("quest-state").textContent = (adb ? "USB ADB · 21 点 · " : "") + questStatus(snapshot);
          $("camera-state").textContent = snapshot.hand ? "实时" : "等待 Quest";
          $("camera-placeholder").querySelector("h3").textContent = snapshot.hand ? "Quest 手势已连接" : "等待 Quest 手势";
          $("camera-placeholder").querySelector("p").textContent = snapshot.hand
            ? (adb ? "USB ADB 21 个关节点已接入，在下方查看三维骨架" : "原生 25 关节已接入，在下方查看三维骨架") : "请在 Quest 进入 XR，并将所选手放入头显视野";
        },
        onError: async error => {if (token === generation) {await stop({message:"Quest 输入已中断"});notice(error.message);}},
      });
      refreshControls();
      void questReceiver.poll();
      return;
    }
    $("camera-state").textContent = "实时";
    $("camera-caption").textContent = isLeapCamera() ? 'LEAP MOTION 2 / 等待图像' : $("input-source").value === "realsense_rgbd" ? 'D455f RGB-D / 等待同帧图像' : cameraCaptureLabel(browserCamera);
    $("camera-caption").parentElement.classList.add("live");
    refreshControls();
    schedule(token);
  } catch (error) {
    if (token !== generation) return;
    await stop({ message: "启动失败，请重试" });
    notice(cameraError(error));
  }
}

$("start").addEventListener("click", start);
$("input-source").addEventListener("change", configureInputSource);
$("overlay-smoothing").addEventListener("change", () => {
  const enabled = $("overlay-smoothing").checked;
  overlay.setSmoothingEnabled(enabled);
  recorder.record("overlay_smoothing", {enabled});
});
$("mediapipe-filter").addEventListener("change", () => {
  recorder.record("mediapipe_filter", {enabled: $("mediapipe-filter").checked});
});
$("quest-copy").addEventListener("click", async () => {
  try {await navigator.clipboard.writeText($("quest-link").value);notice("Quest 连接地址已复制");}
  catch (_) {$("quest-link").select();notice("请手动复制已选中的连接地址");}
});
$("direct-preview").addEventListener("change", async () => {
  if (controlBusy || a32Panel.busy || starting || stopping) {
    $("direct-preview").checked = directMode(); return;
  }
  const enabled = $("direct-preview").checked;
  directControl = enabled;
  controlBusy = true;
  refreshControls();
  try {
    if (session || active || starting || recorder.active) {
      a32Panel.busy = true; a32Panel.render();
      await stop({message:"切换控制模式，当前会话已结束",announce:false});
    }
    if (enabled) $("collision-mode").value = "off";
    recorder.record("control_mode", {direct_mode:enabled, ruckig_enabled:!enabled,
      collision_protection_enabled:!enabled && $("collision-mode").value!=="off"});
    notice(enabled ? "直控已开启：Ruckig、碰撞处理和额外关节平滑已关闭；请手动开始识别并开启下发"
      : "Ruckig 已恢复，可选择碰撞处理；请手动开始识别并开启下发");
  } catch(error) {
    notice(`切换失败，实时下发保持禁用：${error.message}`);
  } finally { controlBusy=false; refreshControls(); }
});
async function switchModel() {
  await stop({message: "切换模型，原会话已结束"});
  configureModelControls(); await loadModel();
  const url = new URL(location.href); url.searchParams.set("model", $("hand-model").value);
  history.replaceState(null, "", url);
}
$("side").addEventListener("change", switchModel);
$("hand-model").addEventListener("change", switchModel);
$("reset-view").addEventListener("click", () => viewer?.resetView());
$("reset-feedback-view").addEventListener("click", () => feedbackViewer?.resetView());
document.addEventListener("visibilitychange", () => {
  diagnostics.record("visibility_change",{visibility:document.visibilityState});diagnostics.flush();
  if (document.hidden && (active || starting || session?.model_id === "TsenseI_v3_2_withid"))
    stop({ message: "页面已隐藏，识别已暂停" });
});
window.addEventListener("pagehide", () => {
  diagnostics.record("page_hide",{state:diagnosticState()});diagnostics.flush();
  recorder.flush();
  client?.close();
  leapCamera?.close();
  cameraOpening?.abort();
  stream?.getTracks().forEach((track) => track.stop());
  questReceiver?.close();
  if (session)
    navigator.sendBeacon(
      "/api/stop",
      new Blob([JSON.stringify({ session: session.session })], {
        type: "application/json",
      }),
    );
  viewer?.dispose();
  clearInterval(feedbackTimer);
  feedbackPreview.dispose();
  feedbackViewer?.dispose();
  skeleton?.dispose();
  overlay.dispose();
});
window.addEventListener("beforeunload", event => {
  if (a32Panel.state.connected || recorder.active) { event.preventDefault(); event.returnValue = ""; }
});

async function refreshRecordings() {
  try {
    const entries = await recorder.list();
    $("local-recordings").replaceChildren(new Option("本机历史记录", ""));
    for (const item of entries) {
      const option = new Option(`${item.name}${item.ended ? "" : " · 可恢复"}`, item.id);
      option.translate = false;
      $("local-recordings").add(option);
    }
  } catch (error) { notice(`本机存储不可用：${error.message}`); }
}
$("recording-directory").addEventListener('click',async()=>{
  try{
    if(!window.showDirectoryPicker)throw Error('请使用 Chrome / Edge 选择独立日志目录');
    const directory=await window.showDirectoryPicker({id:'tsensei-control-logs',mode:'readwrite'});
    for(const model of ['A32'])await directory.getDirectoryHandle(model,{create:true});
    await recorder.setDirectory(directory);
    $("recording-directory-status").textContent=`已授权 ${directory.name} · 结束会话自动按 A32 保存 · 每段 512 MiB`;
  }catch(error){if(error.name!=='AbortError')notice(error.message);}
});
recorder.getDirectory().then(directory=>{
  if(directory)$("recording-directory-status").textContent=directory.permission==='granted'
    ?`已授权 ${directory.name} · 结束会话按型号保存 · 每段 512 MiB`
    :`上次目录 ${directory.name} · 请点击“选择日志根目录”重新授权`;
}).catch(error=>{$("recording-directory-status").textContent=`目录设置不可用：${error.message} · 可使用下载`;});
$("recover-recording").addEventListener("click", async () => {
  try {
    if(a32Panel.state.armed||a32Panel.busy||a32Panel.state.homing_active){notice("请先结束实时下发或找零，再导出历史记录");return;}
    const id = $("local-recordings").value;
    if (id) { await recorder.flush(); showDownload(await recorder.export(id), true); }
  } catch (error) { notice(error.message); }
});
$("download-recording-part").addEventListener("click",async()=>{
  try{
    if(a32Panel.state.armed||a32Panel.busy||a32Panel.state.homing_active){notice("请先结束实时下发或找零，再导出历史记录");return;}
    if(downloadRecordingId)showDownload(await recorder.export(downloadRecordingId,Number($("recording-part").value)),true);
  }
  catch(error){notice(`分段导出失败：${error.message}`);}
});
$("delete-recording").addEventListener("click", async () => {
  try { const id = $("local-recordings").value; if (id) { await recorder.remove(id); await refreshRecordings(); } }
  catch (error) { notice(error.message); }
});

let diagnosticDownloadUrl;
async function downloadDiagnostics(id){
  try{
    diagnostics.record("diagnostic_export",{current:!id});
    const file=await diagnostics.export(id);
    if(diagnosticDownloadUrl)URL.revokeObjectURL(diagnosticDownloadUrl);
    diagnosticDownloadUrl=file.url;const link=document.createElement("a");link.href=file.url;link.download=file.name;link.click();
  }catch(e){$("diagnostic-status").textContent=`日志导出失败：${e.message}`;}
}
$("download-diagnostics").addEventListener("click",()=>downloadDiagnostics());
$("diagnostic-directory").addEventListener("click",async()=>{
  try{await diagnostics.chooseDirectory();}catch(e){if(e.name!=="AbortError")$("diagnostic-status").textContent=`选择目录失败：${e.message}`;}
});
$("diagnostic-history-refresh").addEventListener("click",async()=>{
  try{
    const entries=await diagnostics.list();$("diagnostic-history").replaceChildren(new Option("选择历史诊断日志",""));
    for(const item of entries)$("diagnostic-history").add(new Option(item.name,item.id));
  }catch(e){$("diagnostic-status").textContent=`读取历史失败：${e.message}`;}
});
$("diagnostic-history-download").addEventListener("click",()=>{
  const id=$("diagnostic-history").value;if(id)downloadDiagnostics(id);
});

try {
  const health = await fetch("/api/health");
  if (!health.ok) throw new Error("服务未连接");
  const service = await health.json();
  capabilities = service;
  diagnostics.record("service_ready",service);
  for (const option of $("hand-model").options) option.disabled = !service.models?.includes(option.value);
  const requestedModel = new URLSearchParams(location.search).get("model");
  if (service.models?.includes(requestedModel)) $("hand-model").value = requestedModel;
  const requestedSource = new URLSearchParams(location.search).get("source");
  if (["webxr", "quest_adb", "wuji_glove", "leapmotion", "leapmotion_camera", "realsense_rgbd"].includes(requestedSource)) $("input-source").value = requestedSource;
  const requestedSide = new URLSearchParams(location.search).get("side");
  if (["left", "right"].includes(requestedSide)) $("side").value = requestedSide;
  configureInputSource();
  configureModelControls();
  recordingAllowed = true;
  $("recording").closest("label").title = "仅保存在这台电脑的浏览器，可下载到本机";
  $("connection-dot").classList.add("online");
  $("connection-text").textContent = "服务已连接";
  skeleton = new SkeletonViewer($("skeleton-stage"));
  viewer = new HandViewer($("model-stage"));
  feedbackViewer = new HandViewer($("feedback-stage"));
  feedbackTimer = setInterval(refreshFeedback, 50);
  await loadModel();
  await enumerateCameras();
  await refreshRecordings();
} catch (error) {
  $("connection-text").textContent = "初始化失败";
  notice(`无法启动工作台：${error.message}`);
}
