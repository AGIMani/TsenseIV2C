import { DetectorClient } from "./detector-client.js";
import {isA30Model, A30_MODEL_IDS, isA32Model, A32_MODEL_IDS} from "../../shared/hand-models.js";
import { A32SdkPanel } from "./a32-sdk-control.js";
import { HandViewer, SkeletonViewer } from "./viewer.js";
import { CameraOverlay } from "./overlay.js";
import { BrowserHandControl } from "./browser-hand-control.js";
import { A30BrowserHandControl } from "./a30-browser-hand-control.js";
import { telemetryMetadata as a30TelemetryMetadata } from "./a30-serial-protocol.js";
import { WorkerRecorder } from "./worker-recorder.js";
import { armBlockers, collisionProtectionLabel } from "./control-readiness.js";
import { DiagnosticLogger, errorDetails } from "./diagnostic-logger.js";
import { LatestFrameQueue } from "./latest-frame-queue.js";
import { A30PreviewStream } from "./a30-preview-stream.js";
import { WebXRReceiver, questStatus } from "./webxr-receiver.js";
import { WujiReceiver, wujiStatus } from "./wuji-receiver.js";
import { LeapMotionReceiver, leapMotionStatus } from "./leapmotion-receiver.js";
import { FeedbackPreview } from "./feedback-preview.js";

const $ = (id) => document.getElementById(id);
const overlay = new CameraOverlay($("camera-overlay"), $("video"));
let recordingAllowed = true;
let capabilities = {}, hardware = {}, controlBusy = false, sessionPromise = null;
let hardwarePoll = null;
const a30Preview = new A30PreviewStream({sampled:(sample,state)=>{
  if(state.direct_mode)recorder.record('direct_command',{joints:sample.pose,frame_id:sample.frame_id,
    sample_ticks:sample.tick,browser_epoch:sample.epoch,planner_at:sample.planner_at,
    hand_side:$("side").value,direct_mode:true,ruckig_enabled:false,sent:false,
    stage:'checked',source:'planner_preview_stream'});
}});
let displayedSample = null;
let serverHardware = {}, ports = [], stopping = false, lastDownloadUrl = null, downloadRecordingId = null;
const recorder = new WorkerRecorder(error => {
  // Seal without reading hundreds of MB back into the control page's thread.
  notice(`本机记录已停止，控制继续：${error.message}`);
  recorder.stop("recording_error",{exportFile:false}).then(()=>{
    $("record-status").textContent="记录已封存，控制继续；结束控制后生成下载文件";
    return refreshRecordings();
  }).catch(e=>notice(`记录封存失败：${e.message}；控制继续`));
});
let localControl = new BrowserHandControl({recorder, changed: () => renderHardware()});
const isV2 = () => $("hand-model").value === "TsenseI_v2";
const isA30 = () => isA30Model($("hand-model").value);
const isA32 = () => isA32Model($("hand-model").value);
const isControlModel = () => isV2() || isA30() && capabilities.a30_control_available;
const supportsDirectMode = () => isV2() || isA30() || isA32();
const directMode = () => supportsDirectMode() && localControl.directMode;
const controlUrl = () => isA30() ? "/api/a30/control" : "/api/v2/control";
const modelLabel = () => isV2() ? "V2" : isA30() ? ($("hand-model").value.endsWith("_withid") ? "A3.0 with ID" : "A3.0") : ($("hand-model").value.endsWith("_withid") ? "A3.2 with ID" : "A3.2");
const jointSettings = new Map(); // Separate model/side settings, retained for this page lifetime.
let viewer, feedbackViewer, feedbackTimer, skeleton, stream, client, session, framePipeline;
let questReceiver;
const isWebXR = () => $("input-source").value === "webxr";
const inputSource = () => $("input-source").value;
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
  record: (type, data) => recorder.record(type, data), display: pose => viewer?.setPose(pose)});

const feedbackPreview = new FeedbackPreview({
  request: data => api("/api/feedback-pose", data),
  // The viewer also enforces the original URDF limits after zeroing negatives.
  display: pose => feedbackViewer?.setPose(Object.fromEntries(
    Object.entries(pose).map(([name, angle]) => [name, Math.max(0, angle)]))),
  changed: renderFeedback,
});

function renderFeedback(state) {
  const messages = {
    disconnected: "未连接机械手",
    waiting: "等待关节遥测",
    live: "实测关节角 · 实时反馈",
    stale: "遥测已过期",
    incomplete: `缺少 ${state.missingCount || 0} 个关节反馈 · 缺失关节保持`,
    unsupported: "右手硬件反馈映射待标定",
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
  if (isA32()) {
    const state = a32Panel.state;
    const age = state.telemetry_age_ms;
    feedbackPreview.update({
      connected: !!state.connected,
      supported: $("side").value === "left",
      // Keep command samples and recording metadata out of the feedback path.
      telemetry: {self: state.self || [], mit: state.mit || [], connected: !!state.connected,
        telemetry_age_ms: age, hand_side: state.hand_side, model_id: state.model_id},
      identity: state,
      receivedAt: Number.isFinite(age) && age >= 0 && Number.isFinite(a32Panel.updatedAt)
        ? a32Panel.updatedAt - age : null,
    });
  } else {
    feedbackPreview.update({
      connected: !!localControl.connected,
      telemetry: localControl.latestPayload,
      identity: localControl.latest,
      receivedAt: localControl.latest ? localControl.telemetryAt : null,
    });
  }
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
  return {active,starting,control_busy:controlBusy,model_ready:modelReady,direct_mode:directMode(),hardware:localControl.snapshot(),
    hand_age_ms:Number.isFinite(localControl.handAt)?Math.round(performance.now()-localControl.handAt):null,
    target_ready:serverHardware.target_ready,server_error:serverHardware.last_error,
    frame_pipeline:framePipeline?{...framePipeline.stats}:null,
    recording:{enabled:$("recording").checked,active:recorder.active,error:recorder.error?.message||null,
      events_sent:recorder.sent,worker_received:recorder.accepted,approx_bytes:recorder.bytes},
    tracking_transition:serverHardware.tracking_transition,frame_id:serverHardware.frame_id,
    camera_status:$("camera-state").textContent,tracking_status:$("tracking-text").textContent,
    arm_disabled:$("arm-hand").disabled,arm_blockers:$("arm-blockers").textContent,notice:$("notice").textContent};
}
window.addEventListener("error",event=>diagnostics.record("javascript_error",{...errorDetails(event.error||event.message),file:event.filename,line:event.lineno,column:event.colno}));
window.addEventListener("unhandledrejection",event=>diagnostics.record("unhandled_rejection",errorDetails(event.reason)));
document.addEventListener("pointerdown",event=>{
  const button=event.target.closest?.("button");if(button?.id)diagnostics.record("button_press",{id:button.id,label:button.textContent,disabled:button.disabled,state:diagnosticState()});
},true);
document.addEventListener("change",event=>{const e=event.target;if(e.matches?.("select,input"))diagnostics.record("setting_change",{
  id:e.id,joint:e.dataset.joint,value:e.type==="checkbox"?e.checked:e.value});},true);
setInterval(()=>{if(active||localControl.connected||controlBusy)diagnostics.record("state_snapshot",diagnosticState());},1000);

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
  $("collision-mode").disabled = busy || !!session || controlBusy || (directMode() && !isV2());
  $("recording").disabled = busy || !!session || recorder.active || !recordingAllowed;
  $("direct-preview").disabled = !supportsDirectMode() || !modelReady || starting || stopping || controlBusy || a32Panel.busy || !!hardware.homing_active || !!hardware.power_busy || !!a32Panel.state.homing_active;
  $("direct-preview-note").textContent = directMode()
    ? isV2() ? "低延迟滤波 · 可实时下发（无 Ruckig 限速）" : "直控 · 关节限位后直接映射 · Ruckig、碰撞处理和额外关节平滑已关闭"
    : isControlModel() || (isA32() && capabilities.a32_control_available)
      ? "Ruckig 限速 · 碰撞处理可单独选择" : "预览模式 · 碰撞处理可单独选择";
  $("direct-preview").closest("label").title = isV2()
    ? "映射目标经过低延迟自适应平滑后下发；启用碰撞保护时检查路径；不使用 Ruckig 速度、加速度和 jerk 限制"
    : "直控将限位后的映射目标直接下发，关闭 Ruckig、目标及采样碰撞处理和额外关节平滑。切换将结束当前会话，需手动重新开始识别并开启下发。";
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
  for (const input of $("joint-enable-panel").querySelectorAll("input,button"))
    input.disabled = !modelReady || stopping || controlBusy || !!hardware.homing_active || !!hardware.power_busy
      || (input.dataset.joint ? !isControlModel() || !localControl.powerSupported(input.dataset.joint) : !isControlModel());
}

function refreshJointSelection() {
  const inputs = [...$("joint-enable-groups").querySelectorAll("input")];
  for (const input of inputs) {
    input.checked = localControl.jointEnabled[input.dataset.joint] !== false;
    const supported=isControlModel()&&localControl.powerSupported(input.dataset.joint);
    input.parentElement.querySelector("small").textContent = !supported ? "无电源指令" :
      `${input.checked ? "使能" : "断电"}${localControl.powerSent[input.dataset.joint]===input.checked ? " · 已发指令" : " · 待下发"}`;
  }
  const supported=inputs.filter(i=>isControlModel()&&localControl.powerSupported(i.dataset.joint));
  $("joint-enable-count").textContent = `${supported.filter(i => i.checked).length} / ${supported.length} 选择使能`;
}

function loadJointSelection(description) {
  const key = `${$("hand-model").value}/${$("side").value}`;
  const previous = jointSettings.get(key) || {};
  localControl.side = $("side").value;
  localControl.jointEnabled = Object.fromEntries(Object.keys(description.initial_pose).map(n => [n, previous[n] !== false]));
  jointSettings.set(key, localControl.jointEnabled);
  const groups = $("joint-enable-groups"); groups.replaceChildren();
  const fingers = {thumb: "拇指", index: "食指", mid: "中指", ring: "无名指", pinky: "小拇指"};
  const labels = {swing: "侧摆 swing", rota: "旋转 rota", rota_block: "旋转 rota block", rotaback: "侧摆 rotaback"};
  for (const [finger, label] of Object.entries(fingers)) {
    const group = document.createElement("fieldset"); group.className = "joint-enable-group";
    const legend = document.createElement("legend"); legend.textContent = label; group.append(legend);
    for (const joint of Object.keys(localControl.jointEnabled).filter(n => n.includes(`_hand_${finger}_`))) {
      const row = document.createElement("label"); row.className = "joint-enable-toggle"; row.title = joint;
      const input = document.createElement("input"); input.type = "checkbox"; input.dataset.joint = joint;
      input.setAttribute("aria-label", `${label} ${joint.split(`_hand_${finger}_`)[1].replace(/_joint\d*$/, "")} 下发使能`);
      const name = joint.split(`_hand_${finger}_`)[1].replace(/_joint\d*$/, "");
      const text = document.createElement("span"); text.textContent = labels[name] || name;
      row.append(input, text, document.createElement("small")); group.append(row);
      input.addEventListener("change", () => changeJointPower({[joint]:input.checked}));
    }
    groups.append(group);
  }
  $("joint-enable-note").textContent = isControlModel()
    ? "失能发送电机断电指令，位置槽位不清零；已断电关节跳过找零。切换电源会暂停实时下发，重新开启后从反馈姿态起步。已发指令不代表固件确认。" +
      (isA30() ? "两路外接拇指电机的源协议未提供电源指令。" : "")
    : "当前型号尚未接通下位机，暂不支持电源控制。";
  refreshJointSelection();
}
for (const [id, enabled] of [["enable-all-joints", true], ["disable-all-joints", false]])
  $(id).addEventListener("click", () => changeJointPower(Object.fromEntries(
    Object.keys(localControl.jointEnabled).filter(n=>localControl.powerSupported(n)).map(n=>[n,enabled]))));

async function changeJointPower(selection) {
  if(controlBusy)return;
  controlBusy=true;renderHardware();
  try {await localControl.setJointsEnabled(selection);notice();}
  catch(error){notice(error.message);}
  finally{controlBusy=false;refreshJointSelection();renderHardware();}
}

function configureModelControls() {
  a30Preview.stop();displayedSample=null;
  const select = $("collision-mode");
  select.replaceChildren();
  const choices = isV2()
    ? [["avoid", "V2 · 避碰规划"], ["stop", "V2 · 碰撞停止"], ["off", "关闭 · 仅预览"]]
    : isA30() ? [["off", "关闭保护 · 可下发"], ["monitor", "仅检测 · 不修正"], ["stop", "碰撞停止 · V2 策略"], ["mink", "Mink · 避碰"]]
    : [["off", "关闭保护 · 可下发"], ["monitor", "仅检测 · 不修正"], ["stop", "碰撞停止"], ["mink", "Mink · 避碰"]];
  for (const [value, label] of choices) select.add(new Option(label, value));
  if (isV2()) select.value = "avoid";
  else if (isA30()) {
    const modes = capabilities.a30_collision_modes || ["off"];
    for (const option of select.options) option.disabled = !modes.includes(option.value);
    select.value = modes.includes("mink") ? "mink" : modes.includes("stop") ? "stop" : "off";
  }
  else if (!isA30()) {
    const modes = capabilities.a32_collision_modes?.[$("hand-model").value] || ['off'];
    for (const option of select.options) option.disabled = !modes.includes(option.value);
    select.value = modes.includes('stop') ? 'stop' : 'off';
  }
  localControl = isA30() ? new A30BrowserHandControl({modelId:$("hand-model").value, recorder, changed: () => renderHardware()})
    : new BrowserHandControl({recorder, changed: () => renderHardware()});
  localControl.directMode = supportsDirectMode() && $("direct-preview").checked;
  if (directMode() && !isV2()) select.value = "off";
  serverHardware = {};
  $("hardware-panel").hidden = !isControlModel();
  $("hardware-title").textContent = `${modelLabel()} 下位机控制`;
  $("emergency-hand").textContent = isA30() ? "停止 / 自研断电" : "停止并断电";
  $("a30-integration").hidden = !isA30();
  $("joint-enable-panel").hidden = isA32();
  a32Panel.configure($("hand-model").value, $("side").value, capabilities.a32_control_available);
  $("model-version").textContent = modelLabel();
  hardware = {};
  renderHardware();
}

function renderHardware(snapshot) {
  if (snapshot) serverHardware = snapshot;
  hardware = {...serverHardware, ...localControl.snapshot()};
  const connected = !!hardware.connected, homing = !!hardware.homing_active;
  const allowed = window.isSecureContext && !!navigator.serial;
  $("hardware-status").textContent = !allowed ? "本机串口需要 Chrome / Edge，并使用可信 HTTPS"
    : homing ? hardware.homing_status
    : hardware.last_error || (connected ? `${hardware.device || "串口已连接"} · ${hardware.trajectory_recovering ? "正在恢复轨迹连接" : hardware.armed ? "正在下发" : "下发关闭"}` : directMode()?"未连接 · 直接预览":"未连接 · Ruckig 已开启");
  if(isA30()&&connected&&!hardware.armed&&!homing&&!localControl.armPending&&!localControl.enabling)
    $("hardware-status").textContent+=' · 模型为预览，未下发';
  if(connected&&hardware.feedback_note)$("hardware-status").textContent+=` · ${hardware.feedback_note}`;
  $("refresh-ports").disabled = !allowed || connected || controlBusy;
  $("serial-port").disabled = connected || controlBusy;
  $("serial-baud").disabled = connected || controlBusy;
  $("connect-serial").disabled = !allowed || controlBusy;
  $("connect-serial").textContent = connected ? "断开串口" : "连接串口";
  $("home-hand").disabled = !allowed || !connected || controlBusy;
  $("home-hand").textContent = homing ? "停止找零" : "一键找零";
  $("homing-result-open").disabled = !hardware.homing_report || homing;
  $("homing-iq").disabled = homing || controlBusy;
  const blockers=armBlockers({allowed,connected,homing,busy:controlBusy,active,
    hardware:{...hardware,last_error:serverHardware.last_error,mode_confirmed:localControl.modeConfirmed},telemetryIssue:localControl.controlTelemetryIssue(),
    requireHandConfirmation:localControl.requireHandConfirmation,directMode:directMode(),
    handAgeMs:performance.now()-(localControl.handAt??-Infinity),side:$("side").value,mode:$("collision-mode").value});
  $("arm-hand").disabled = blockers.length>0;
  $("arm-blockers").textContent=hardware.armed?(hardware.collision_paused?"碰撞保护暂停，手势安全后自动恢复":"正在实时下发"):blockers.length?`暂不能开启：${blockers.join("；")}`:hardware.rebase_required?`故障停发：${serverHardware.last_error}；重新开启将从反馈姿态起步`:"条件已满足，可开启实时下发";
  if (isA30()) $("arm-blockers").textContent += ` · ${collisionProtectionLabel($("collision-mode").value)}`;
  $("arm-hand").title=blockers.join("；");
  diagnostics.change("arm_readiness",{blocked:!hardware.armed&&blockers.length>0,armed:!!hardware.armed,reasons:blockers});
  $("serial-diagnostics").textContent=connected?`串口接收 ${hardware.rx_bytes} B · 有效手部包 ${hardware.parsed_packets} · 已更新 ${hardware.accepted_packets} 包`+
    (hardware.telemetry_payload_bytes?` · 载荷 ${hardware.telemetry_payload_bytes} B · 最近更新 ${hardware.telemetry_age_ms} ms 前`:""):"";
  if(connected&&hardware.board_telemetry){
    const board=hardware.board_telemetry,metric=(v,unit)=>v===null?"—":`${v.toFixed(2)} ${unit}`;
    $("serial-diagnostics").textContent+=` · 板载 ${board.stale?"超时":"在线"} · NTC-C ${metric(board.ntc_c,"℃")} · NTC-B ${metric(board.ntc_b,"℃")} · VIN ${metric(board.vin_v,"V")}`;
  }
  if(connected&&isA30()&&hardware.telemetry_issue&&!localControl.controlTelemetryIssue())
    $("serial-diagnostics").textContent+=` · ${hardware.telemetry_issue}（提示，不阻断下发）`;
  if(connected&&hardware.telemetry_decode_issues?.length)
    $("serial-diagnostics").textContent+=` · 原始遥测含 ${hardware.telemetry_decode_issues.length} 项异常字段（已记录，请核对固件格式）`;
  if (hardware.armed) $("arm-hand").disabled = false;
  $("arm-hand").textContent = hardware.armed ? "关闭实时下发" : "开启实时下发";
  $("emergency-hand").disabled = !allowed || !connected;
  $("close-control").disabled = stopping || !(session || connected || recorder.active);
  const phase = {waiting: "等待识别", confirming: "确认手势", recovering: "平滑恢复", tracking: "跟随中", braking: "减速停稳", holding: "保持姿态", collision_paused: "碰撞暂停，等待安全手势", collision_limited: `碰撞手指保持：${(hardware.held_fingers||[]).map(f=>({thumb:'拇指',index:'食指',mid:'中指',ring:'无名指',pinky:'小指'}[f]||f)).join('、')}；其余继续跟随`, blocked: "下发已阻断"}[hardware.tracking_transition?.phase] || "等待识别";
  const homeResult = hardware.homing_result && !homing ? ` · ${hardware.homing_status}` : "";
  const timing = hardware.control_timing;
  const rate = timing?.actual_sample_hz ? ` · 控制 ${timing.actual_sample_hz.toFixed(0)} Hz` : "";
  const sync = hardware.motion_synchronization?.mode === 'finger_groups' ? ' · 按手指同步' : '';
  const outputPhase=hardware.trajectory_recovering?'轨迹连接恢复中':!hardware.armed&&!homing?'预览（未下发）':phase;
  $("control-stats").textContent = `${directMode()?"Ruckig 已关闭 · 直接映射":"Ruckig 已开启"+sync}${rate} · 本机写入 ${hardware.send_ticks || 0} 帧 · ${hardware.local_send_hz.toFixed(0)} Hz · ${outputPhase}${homeResult}`;
  if (hardware.recording_error) notice(`记录失败：${hardware.recording_error}`);
  refreshJointSelection();
  refreshControls();
}

function showResponsePose(pose) {
  // A3 streaming display is refreshed on the next screen frame. HTTP replies
  // must never overwrite a successfully written command with a future pose.
  if(isA30()&&(localControl.armed||localControl.armPending||localControl.enabling||a30Preview.latest))return;
  viewer.setPose(pose);
}

function beginHardwarePolling() {
  clearTimeout(hardwarePoll);
  const poll = async () => {
    const current = session;
    if (!current || !["TsenseI_v2", ...A30_MODEL_IDS].includes(current.model_id)) return;
    try {
      const requestedAt = performance.now();
      const status = await api(controlUrl(), {session: current.session, action: "status"});
      if (session !== current) return;
      renderHardware(status);
      localControl.startStream?.(current.session,status,requestedAt);
      if(isA30Model(current.model_id)) {
        if(localControl.armed||localControl.armPending||localControl.enabling)a30Preview.stop();
        else a30Preview.start(current.session,status,requestedAt);
      }
      recorder.record("planner_sample", {frame_id: status.sample_frame_id??status.frame_id,
        target_frame_id:status.frame_id,hand_side: $("side").value,
        direct_mode:status.direct_mode,ruckig_enabled:status.ruckig_enabled,
        direct_filter:status.direct_filter,
        motion_limits:status.motion_limits,held_fingers:status.held_fingers,
        motion_synchronization:status.motion_synchronization,
        limit_recoveries:status.limit_recoveries,last_limit_event:status.last_limit_event,
        boundary_roundoffs:status.boundary_roundoffs,last_boundary_roundoff:status.last_boundary_roundoff,
        sample_ticks:status.sample_ticks,target_ready:status.target_ready,
        tracking_transition:status.tracking_transition,last_error:status.last_error,
      joints: status.command_joint_angles_rad || status.ruckig_joint_angles_rad, target: status.target_joint_angles_rad,
        sample_collision: status.sample_collision, control_timing: status.control_timing,
        source: "received_server_sample"});
      if(status.direct_mode&&status.direct_filter)recorder.record('direct_filter_detail',{
        ...status.direct_filter,frame_id:status.sample_frame_id??status.frame_id,sample_ticks:status.sample_ticks,
        hand_side:$("side").value,sent:false,source:'received_server_status'});
      if(current.model_id==='TsenseI_v2'&&status.direct_mode&&!localControl.armed){
        showResponsePose(status.ruckig_joint_angles_rad);
        recorder.record('direct_filter',{joints:status.ruckig_joint_angles_rad,frame_id:status.sample_frame_id??status.frame_id,
          hand_side:$("side").value,sample_ticks:status.sample_ticks,sent:false,stage:'checked',source:'browser_preview_sample'});
      }
      if (!directMode() && !status.direct_mode && !localControl.armed && Object.keys(status.ruckig_joint_angles_rad || {}).length)
        recorder.record("h7", {frame_id: status.sample_frame_id??status.frame_id, hand_side: $("side").value,
          joints_ui: status.ruckig_joint_angles_rad, joints_ruckig_ui: status.ruckig_joint_angles_rad,
          joints_hw: status.hardware_joint_angles_rad, sent: false, usb_connected: localControl.connected,
          ruckig_faults: [], guard_limited: null, source: "browser/preview_sample"});
      await localControl.sample(status, requestedAt);
      if(current.model_id==='TsenseI_v2'&&directMode()&&status.direct_mode&&localControl.armed
          &&localControl.lastSample===status.sample_ticks)showResponsePose(localControl.held);
      if (!directMode() && !status.direct_mode && !active && status.ruckig_joint_angles_rad) showResponsePose(status.ruckig_joint_angles_rad);
    } catch (error) {
      if (session !== current) return;
      await stop({message: "控制连接已中断"});
      notice(`控制连接中断：${error.message}`);
      return;
    }
    if (session === current) hardwarePoll = setTimeout(poll, isA30Model(current.model_id) ? 100 : localControl.armed ? 20 : 150);
  };
  hardwarePoll = setTimeout(poll, 0);
}

async function ensureControlSession() {
  if (session) return session;
  if (!sessionPromise) {
    const token = generation;
    sessionPromise = api("/api/session", {model_id: $("hand-model").value, side: $("side").value,
      input_source: inputSource(),
      recording: false, collision_mode: $("collision-mode").value, direct_mode:directMode()}).then(async created => {
      if (token !== generation) {
        await api("/api/stop", {session: created.session});
        throw new Error("启动已取消");
      }
      session = created;
      if(isA30Model(created.model_id)){
        localControl.fallbackTargets={linear:created.hardware.linear_strokes_mm,rotary:created.hardware.rotary_positions_deg};
        localControl.feedbackPositionLimits=created.hardware.feedback_position_limits;
      }
      $("download-control").hidden = true;
      renderHardware(created.hardware);
      if(isA32Model(created.model_id) && created.hardware) { a32Panel.update(created.hardware); a32Panel.begin(); }
      beginHardwarePolling();
      return created;
    }).finally(() => { sessionPromise = null; });
  }
  return sessionPromise;
}

async function controlAction(action, values = {}) {
  const began=performance.now();
  diagnostics.record("control_action_requested",{action,values,state:diagnosticState(),latest_telemetry:localControl.latestPayload});
  if (controlBusy && !["emergency_stop", "cancel_home"].includes(action)){
    diagnostics.record("control_action_ignored",{action,reason:"已有控制操作正在执行"});return;
  }
  const actionGeneration = generation;
  const checkAction = () => { if (actionGeneration !== generation || stopping) throw new Error("控制操作已取消"); };
  controlBusy = !["home", "cancel_home"].includes(action);
  renderHardware();
  try {
    if (action === "ports") {
      ports = await navigator.serial.getPorts();
      $("serial-port").replaceChildren(new Option("选择新的 USB 串口…", "new"));
      ports.forEach((port, i) => {
        const info = port.getInfo();
        $("serial-port").add(new Option(`已授权 USB ${info.usbVendorId?.toString(16) || "串口"}:${info.usbProductId?.toString(16) || ""} #${i+1}`, String(i)));
      });
    } else if (action === "connect") {
      // requestPort must be called while this click still has user activation.
      const port = ports[Number(values.port)] && values.port !== "new"
        ? ports[Number(values.port)] : await navigator.serial.requestPort();
      checkAction();
      await ensureLocalRecording();
      if (actionGeneration !== generation || stopping) {
        showDownload(await recorder.stop("connection_cancelled"));
        checkAction();
      }
      await ensureControlSession();
      checkAction();
      await localControl.connect(port, $("side").value, Number($("serial-baud").value));
      if (actionGeneration !== generation || stopping) { await localControl.disconnect(); checkAction(); }
    } else if (action === "disconnect") await localControl.disconnect();
    else if (action === "emergency_stop" || action === "cancel_home") await localControl.emergency();
    else if (action === "home") {
      await ensureLocalRecording();
      const current = await ensureControlSession();
      checkAction();
      const report = await localControl.home(values.iq_seek, current.init_pose, current.joint_limits);
      if (actionGeneration === generation && !stopping) showHomingReport(report);
    } else if (action === "arm") {
      if (!values.enabled) localControl.disarm();
      else {
        if(!localControl.modeConfirmed)throw new Error("控制模式尚未确认，请重新切换模式");
        if (!active) throw new Error("请先开启识别");
        if (isV2() && ["off", "monitor"].includes($("collision-mode").value)) throw new Error("请开启碰撞保护");
        const current = session;
        localControl.limits = current.joint_limits;
        await localControl.arm((pose, telemetry, options={}) => api(controlUrl(),
          {session: current.session, action: "browser_rebase", pose, telemetry, ...options}),
          {session: current.session});
      }
    }
    diagnostics.record("control_action_completed",{action,duration_ms:performance.now()-began,state:diagnosticState()});
    notice();
  } catch (error) { diagnostics.record("control_action_failed",{action,duration_ms:performance.now()-began,error:errorDetails(error),state:diagnosticState(),latest_telemetry:localControl.latestPayload});notice(error.message); }
  finally { controlBusy = false; renderHardware(); }
}

async function ensureLocalRecording() {
  if (!$("recording").checked || recorder.active) return;
  await recorder.start({model_id: $("hand-model").value, hand_side: $("side").value,
    hand_name:$("recording-hand-name").value.trim(),
    direct_mode:directMode(),
    joint_enabled: {...localControl.jointEnabled}, joint_enable_mode:"motor_power",
    ...(isA30() ? {...a30TelemetryMetadata($("side").value), model_id:$("hand-model").value} : {}),
    ...(isA32() ? {transport:"a32_native_sdk",hardware_location:"local Python service / native A3 SDK",
      sampling:"browser-received source/retarget results and SDK successful-write samples; SDK feedback cache polled about every 200 ms; full host-control log is recorded separately by the local service"} : {}),
    source: `web/${inputSource()}`, self_collision_mode: $("collision-mode").value,
    direct_filter:serverHardware.direct_filter||null,
    collision_protection_enabled: !directMode() || isV2() ? !["off", "monitor"].includes($("collision-mode").value) : false,
    control_order: (directMode() && !isV2() ? `MediaPipe -> retarget -> joint limits -> linkage/slot mapping -> ${isA32() ? "native A3.2 SDK" : "browser Web Serial (when armed)"}; no Ruckig, collision correction or extra joint smoothing` : isA32() ? "MediaPipe -> retarget -> model collision -> Ruckig -> sample collision -> A3.2 linkage/slot mapping -> native A3.2 SDK" : directMode() ? "MediaPipe -> retarget -> collision -> adaptive exponential filter -> sample collision -> linkage -> browser Web Serial (when armed)" : isA30()
      ? "MediaPipe -> retarget -> collision -> finger-group Ruckig -> sample collision -> linkage -> browser Web Serial -> UI from successful write; unarmed UI from planner preview stream"
      : "MediaPipe -> retarget -> collision -> Ruckig -> sample collision -> UI/HW -> linkage -> browser Web Serial").replace("MediaPipe", isLeapMotion() ? "Leap Motion 2 -> 21" : isWuji() ? "Wuji Glove 21" : isWebXR() ? "Quest WebXR 25 -> 21" : "MediaPipe")});
  $("record-status").textContent = "完整记录保存在本机浏览器 · 每段 512 MiB · 结束会话后写入已授权目录";
}

function showHomingReport(report = localControl.homeReport) {
  if (!report) return;
  $("homing-result-summary").textContent = report.failures.length
    ? `${report.total - report.failures.length} / ${report.total} 个关节已确认成功，以下 ${report.failures.length} 个未成功。`
    : `${report.total} 个参与找零的关节全部成功。`;
  $("homing-result-note").textContent = report.failures.length
    ? "未成功的关节仅作提示，不会阻止开启实时下发。下一次开启下发将根据当前位置反馈接续轨迹。"
    : "可开始识别并手动开启实时下发。";
  const body = $("homing-result-rows"); body.replaceChildren();
  for (const item of report.failures) {
    const row = document.createElement("tr");
    for (const text of [item.label, item.channel, item.reason]) {
      const cell = document.createElement("td"); cell.textContent = text; row.append(cell);
    }
    body.append(row);
  }
  $("homing-result-table").hidden = !report.failures.length;
  if (!$("homing-result-dialog").open) $("homing-result-dialog").showModal();
}
function showDownload(file, automatic = false) {
  if (!file) return;
  if (lastDownloadUrl) URL.revokeObjectURL(lastDownloadUrl);
  lastDownloadUrl = file.url;
  for (const id of ["download-control", "download-local"]) {
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
    return "相机权限未开启。请在地址栏允许访问摄像头，再点击开始识别。";
  if (error.name === "NotFoundError")
    return "没有找到摄像头。请连接或启用电脑内置摄像头后重试。";
  if (error.name === "NotReadableError")
    return "摄像头被其他程序占用或无法读取，请关闭其他相机程序后重试。";
  if (error.name === "OverconstrainedError")
    return "所选摄像头已不可用，请重新选择视频来源。";
  return error.message || String(error);
}

async function enumerateCameras() {
  if (!navigator.mediaDevices) return;
  const selected = $("camera").value;
  const devices = (await navigator.mediaDevices.enumerateDevices()).filter(
    (d) => d.kind === "videoinput",
  );
  $("camera").replaceChildren(new Option("默认摄像头", ""));
  devices.forEach((device, i) =>
    $("camera").add(
      new Option(device.label || `摄像头 ${i + 1}`, device.deviceId),
    ),
  );
  if ([...$("camera").options].some((option) => option.value === selected))
    $("camera").value = selected;
}

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
    loadJointSelection(description);
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
  localControl.disarm("识别已停止");
  ++generation;
  questReceiver?.close();
  questReceiver = null;
  $("quest-pairing").hidden = true;
  $("quest-link").value = "";
  $("quest-token").textContent = "";
  $("quest-state").textContent = "已停止";
  $("wuji-state").textContent = "已停止";
  $("leapmotion-state").textContent = "已停止";
  framePipeline?.close();
  framePipeline = null;
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
  catch (error) { if (isA32()) notice(`SDK 停止响应：${error.message}`); }
  session = null;
  a32Panel.reset();
  a30Preview.stop();displayedSample=null;
  await localControl.disconnect();
  clearTimeout(hardwarePoll);
  hardwarePoll = null;
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
  serverHardware = {};
  renderHardware();
}

function schedule(token) {
  if (!active || token !== generation) return;
  const callback = () => {
    frameCallback = null;
    processFrame(token);
  };
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
  if (video.readyState < 2 || video.currentTime === lastVideoTime) {
    schedule(token);
    return;
  }
  lastVideoTime = video.currentTime;
  const timestamp_ms = performance.now(),
    capture_unix_ms = Date.now();
  try {
    const bitmap = await createImageBitmap(video);
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
    if (isControlModel()) localControl.observeHand(hand?.label === expected, timestamp_ms);
    const packet = {
      session: session.session,
      sequence: frameId,
      timestamp_ms,
      capture_unix_ms,
      camera_size: [video.videoWidth, video.videoHeight],
      hand,
      inference_ms: detection.inference_ms || 0,
      ...(isV2() || isA32() ? {raw_detection: {
        world: allWorld,
        normalized: allNormalized,
        labels: handedness.map(c => ({label: c[0]?.categoryName, score: c[0]?.score})),
      }} : {}),
    };
    framePipeline.offer({packet,frameId,hand,expected,timestamp_ms,inferredAt:performance.now()});
    // Start transport before the canvas draw and log snapshot. Recording
    // remains ordered: fetch resolves asynchronously, after this raw record.
    recorder.record("mediapipe_raw", {frame_id: frameId, tracking_session_id:session.session, timestamp_ms, infer_ms: detection.inference_ms || 0,
      capture_timing:{bitmap_ms:bitmapReadyAt-timestamp_ms,
        worker_roundtrip_ms:detectionReadyAt-bitmapReadyAt,capture_to_detection_ms:detectionReadyAt-timestamp_ms},
      normalized_landmarks: allNormalized,
      world_landmarks: allWorld,
      handedness: handedness.map(c => c.map(v => ({label: v.categoryName, score: v.score})))});
    overlay.setHands(
      result.landmarks || [],
      handedness[index]?.[0]?.categoryName === expected ? index : -1,
    );
  } catch (error) {
    if (token !== generation) return;
    await stop({ message: "识别已中断" });
    notice(cameraError(error));
  }
}

function applyFrameUpdate(frame, update) {
    const {frameId,hand,expected,timestamp_ms} = frame;
    const fromWebXR = frame.packet?.source === "webxr";
    const source = frame.packet?.source || "mediapipe";
    if (update.points) recorder.record(source, {landmarks: update.points, frame_id: frameId,
      hand_side: $("side").value, source,
      depth_source: update.depth_source || (source === "mediapipe" ? "mediapipe_world" : `${source}_metric`), timestamp_ms});
    const rawPose = update.retarget_joint_angles_rad;
    if (isA32Model(update.actuation?.model_id)) {
      recorder.record("a32_conversion", {frame_id: frameId, ...update.actuation});
    }
    if (isA30() && update.actuation) {
      recorder.record("a30_conversion", {frame_id: frameId, ...update.actuation});
    }
    // A completed solve can still be rejected by collision/control gates.
    // Preserve the raw result independently of permission to move the hand.
    const hasRetarget=rawPose&&typeof rawPose==='object'&&Object.keys(rawPose).length>0&&Object.values(rawPose).every(Number.isFinite);
    if (hasRetarget) {
      recorder.record("retarget", {joints: rawPose, frame_id: frameId, tracking_session_id:frame.packet?.session, hand_side: $("side").value,
        solve_ms: update.solve_ms, landmark_repairs: [],tracking:!!update.tracking,control_accepted:!!update.tracking,message:update.message||""});
    } else {
      recorder.record("retarget_skipped", {frame_id: frameId, tracking_session_id:frame.packet?.session, hand_side: $("side").value,
        timestamp_ms, detected_label: hand?.label || null, expected_label: expected,
        reason: !hand ? "no_hand" : hand.label !== expected ? "wrong_hand"
          : rawPose ? "invalid_retarget" : update.tracking ? "missing_retarget" : "solve_rejected",
        message: update.message || "", tracking: !!update.tracking});
    }
    if(hasRetarget&&update.tracking){
      recorder.record("control_target", {joints: update.collision_safe_target_joint_angles_rad || update.pose,
        frame_id: frameId, hand_side: $("side").value, self_collision: update.collision,
        pipeline_timing:{capture_to_response_ms:source==='mediapipe'?performance.now()-timestamp_ms:null,
          source_age_at_receive_ms:frame.sourceAgeMs??null,
          source_age_at_solve_ms:update.source_age_at_solve_ms??null,
          source_to_response_upper_ms:Number.isFinite(frame.sourceAgeMs)
            ?frame.sourceAgeMs+performance.now()-(frame.receivedAt??timestamp_ms):null,
          queue_wait_ms:frame.requestStartedAt-frame.inferredAt,
          request_response_ms:performance.now()-frame.requestStartedAt}});
    }else if(hasRetarget){
      recorder.record("control_target_skipped",{frame_id:frameId,tracking_session_id:frame.packet?.session,
        hand_side:$("side").value,reason:'control_rejected',message:update.message||"",self_collision:update.collision});
    }
    // Mode switches may race an older /api/frame response. Never display the
    // wrong source or let a sampled trajectory overwrite direct targets.
    if(!update.background_discarded&&!update.display_discarded&&(!(isControlModel()||isA32()||isA30())||!!update.direct_mode===directMode())) {
      if(!directMode() || !isV2())showResponsePose(update.hardware?.armed
        ? update.command_joint_angles_rad || update.pose : update.pose);
    }
    if (update.hardware&&!update.background_discarded&&!update.display_discarded) {
      if (isA32()) a32Panel.update(update.hardware);
      else renderHardware(update.hardware);
    }
    const collision = update.collision;
    $("collision-status").textContent = directMode() && !isV2() ? "直控 · 碰撞处理关闭 · 关节限位后直接映射" : isV2()
      ? `V2 · ${collision?.limited ? "避碰介入" : ({avoid: "避碰规划", stop: "碰撞停止", off: "碰撞关闭"}[$("collision-mode").value])} · ${directMode()?"直接映射":"Ruckig"}`
      : collision?.mode === "monitor" ? `仅检测 · ${collision.contact_count} 处接触 · 不修正姿态`
      : collision?.mode === "stop" ? `碰撞停止 · ${collision.limited ? "已限制姿态" : "路径畅通"}`
      : collision?.mode === "mink" && update.tracking
      ? `Mink · 间距 ${collision.min_distance_mm.toFixed(2)} mm · 修正 ${collision.correction_deg.toFixed(1)}°`
      : $("collision-mode").value === "mink" ? "Mink 避碰 · 等待手势" : "避碰关闭 · 原始映射";
    $("collision-status").title = collision?.mode === "mink" && update.tracking
      ? `原始间距 ${collision.raw_min_distance_mm.toFixed(2)} mm；避碰耗时 ${collision.solve_ms.toFixed(1)} ms；${collision.closest_pair.join(" ↔ ")}；${collision.status}`
      : "距离按碰撞代理计算；停止识别后可以切换模式对比。";
    $("solve-ms").textContent = update.tracking
      ? update.solve_ms.toFixed(1)
      : "—";
    $("tracking-dot").classList.toggle("online", update.tracking);
    $("tracking-text").textContent = update.tracking
      ? `${expected === "Left" ? "左手" : "右手"}已识别 · 正在跟随`
      : update.message;
    $("tracking-text").title = update.message;
    $("skeleton-status").textContent = update.tracking
      ? fromWebXR ? "WebXR 25 → 21 点 · 实时" : source === "wuji_glove" ? "Wuji 21 点 · 实时" : source === "leapmotion" ? "Leap Motion 2 · 21 点 · 实时" : "21 点 · 实时"
      : "未检测";
    latestWorld = update.tracking ? hand?.world : null;
    skeleton?.setPoints(latestWorld, hand?.label || expected);
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
  if (isControlModel()) localControl.observeHand(hand?.label === expected,
    hand ? timestamp_ms - snapshot.age_ms : timestamp_ms);
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
  const params = new URLSearchParams({model: $("hand-model").value, side: $("side").value, source: inputSource()});
  $("calibration-link").href = '/calibration?' + params;
}

function configureInputSource() {
  updateCalibrationLink();
  const quest = isWebXR();
  const wuji = isWuji(), leap = isLeapMotion(), metric = isMetricInput();
  $("quest-input-panel").hidden = !quest;
  $("wuji-input-panel").hidden = !wuji;
  $("leapmotion-input-panel").hidden = !leap;
  $("camera-control").hidden = metric;
  $("camera-title").textContent = "原始关键点输入";
  $("video").parentElement.querySelector(".mirror-label").hidden = metric;
  const placeholder = $("camera-placeholder");
  placeholder.querySelector("h3").textContent = leap ? "用 Leap Motion 2 采集手势" : wuji ? "用 Wuji 手套采集手势" : quest ? "用 Quest 采集手势" : "让手势进入画面";
  placeholder.querySelector("p").textContent = leap ? "启动 Linux Ultraleap Tracking Service 后点击「开始识别」，将所选手放入设备视野" : wuji ? "连接手套后点击「开始识别」，在下方查看骨架与模型" : quest ? "开始识别后，按上方连接信息在 Quest 进入 XR" : "点击「开始识别」，允许浏览器使用相机";
  placeholder.querySelector(":scope > span").textContent = metric ? "在下方骨架视图查看 21 点与模型映射" : "建议保持手掌完整入镜";
  $("quest-origin-note").textContent = location.protocol !== "https:" || ["localhost","127.0.0.1","[::1]"].includes(location.hostname)
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
    if (!window.isSecureContext || !navigator.mediaDevices?.getUserMedia)
      throw new Error(
        "访问公网摄像头需要 HTTPS。请使用本站的 https:// 地址；本机开发可使用 localhost。",
      );
    const selected = $("camera").value;
    const acquired = await navigator.mediaDevices.getUserMedia({
      audio: false,
      video: {
        width: { ideal: 640 },
        height: { ideal: 480 },
        frameRate: { ideal: 30, max: 30 },
        ...(selected
          ? { deviceId: { exact: selected } }
          : { facingMode: "user" }),
      },
    });
    if (token !== generation) {
      acquired.getTracks().forEach((track) => track.stop());
      return;
    }
    stream = acquired;
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
    client = new DetectorClient();
    const ready = await client.ask({ type: "init" });
    if (token !== generation) return;
    gpuBadge(ready.delegate);
    if (ready.fallbackReason)
      notice(
        "当前浏览器的 GPU 识别初始化失败，已切换为 CPU。可在浏览器设置中检查图形加速是否开启。",
      );
    }
    const created = (isControlModel() || isA32()) ? await ensureControlSession() : await api("/api/session", {
      input_source: inputSource(),
      model_id: $("hand-model").value,
      side: $("side").value,
      recording: false,
      collision_mode: $("collision-mode").value,
      direct_mode: directMode(),
    });
    if (token !== generation) {
      await api("/api/stop", { session: created.session });
      return;
    }
    session = created;
    if (isControlModel()) renderHardware(await api(controlUrl(), {session: created.session, action: "tracking_start"}));
    if (isA32() && created.hardware) a32Panel.update(await api('/api/a32/control', {session: created.session, action: 'tracking_start'}));
    showResponsePose(created.pose);
    $("record-status").textContent = recorder.active
      ? "完整记录保存在本机浏览器 · 每段 512 MiB · 结束会话后写入已授权目录"
      : "角度记录已关闭";
    if (!isMetricInput()) await enumerateCameras();
    if (token !== generation) return;
    starting = false;
    framePipeline = new LatestFrameQueue({
      process: frame => api("/api/frame",frame.packet),
      onDrop:(frame,reason,error)=>recorder.record("retarget_skipped",{
        frame_id:frame.frameId,tracking_session_id:frame.packet.session,hand_side:$("side").value,
        reason,tracking:false,stage:'browser_queue',age_ms:performance.now()-frame.timestamp_ms,
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
      const pairing = await api("/api/webxr/create", {session: created.session});
      if (token !== generation) return;
      $("quest-link").value = pairing.entry_url ?? new URL(pairing.entry_path, location.href).href;
      $("quest-link").placeholder = pairing.connection_error ? "局域网采集地址暂不可用" : "";
      $("quest-copy").disabled = !$("quest-link").value;
      $("quest-connection-note").textContent = pairing.connection_error || "电脑与 Quest 请连接同一局域网；在头显打开上方 HTTPS 地址。";
      $("quest-token").textContent = pairing.pairing_code;
      $("quest-pairing").hidden = false;
      $("camera-state").textContent = "等待 Quest";
      $("camera-caption").textContent = "QUEST / WEBXR 25 → 21";
      $("gpu-badge").className = "chip ready";
      $("gpu-badge").querySelector("span").textContent = "Quest WebXR";
      questReceiver = new WebXRReceiver({session: created.session,
        staleAfterMs: pairing.stale_after_ms,
        combined:pairing.combined_frame===true, background:pairing.background_processing===true,
        onDiagnostic: (event, detail) => diagnostics.record(event, detail),
        onFrame: snapshot => offerQuestFrame(token, snapshot),
        onStatus: snapshot => {
          if (token !== generation) return;
          $("quest-state").textContent = questStatus(snapshot);
          $("camera-state").textContent = snapshot.hand ? "实时" : "等待 Quest";
          $("camera-placeholder").querySelector("h3").textContent = snapshot.hand ? "Quest 手势已连接" : "等待 Quest 手势";
          $("camera-placeholder").querySelector("p").textContent = snapshot.hand
            ? "原生 25 关节已接入，在下方查看三维骨架" : "请在 Quest 进入 XR，并将所选手放入头显视野";
        },
        onError: async error => {if (token === generation) {await stop({message:"Quest 输入已中断"});notice(error.message);}},
      });
      refreshControls();
      void questReceiver.poll();
      return;
    }
    $("camera-state").textContent = "实时";
    $("camera-caption").textContent =
      `CAMERA / ${$("video").videoWidth} × ${$("video").videoHeight}`;
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
$("quest-copy").addEventListener("click", async () => {
  try {await navigator.clipboard.writeText($("quest-link").value);notice("Quest 连接地址已复制");}
  catch (_) {$("quest-link").select();notice("请手动复制已选中的连接地址");}
});
$("direct-preview").addEventListener("change", async () => {
  if(!supportsDirectMode())return;
  if (controlBusy || a32Panel.busy || starting || stopping) {
    $("direct-preview").checked = directMode(); return;
  }
  const enabled=$("direct-preview").checked;
  controlBusy=true;
  // Interlock synchronously, before the network request. Pending serial writes
  // and automatic recovery carry the old job/epoch and are cancelled.
  localControl.setDirectMode(enabled,false);
  a30Preview.stop();displayedSample=null;
  renderHardware();
  try {
    if(!isV2()) {
      // Recreate the A3 session on the next explicit start so both the target
      // pipeline and the hardware sampler use the same collision/motion mode.
      // stop() closes source workers and disconnects hardware; never re-arm.
      if(session || active || starting || recorder.active) {
        a32Panel.busy = true; a32Panel.render();
        await stop({message:"切换控制模式，当前会话已结束",announce:false});
      }
      if(enabled)$("collision-mode").value="off";
    } else if(session){
      const state=await api(controlUrl(),{session:session.session,action:"set_direct_mode",enabled});
      if(state.direct_mode!==enabled)throw new Error("服务器未确认控制模式，请更新服务");
      renderHardware(state);
    }
    localControl.setDirectMode(enabled);
    recorder.record("control_mode",{direct_mode:enabled,ruckig_enabled:!enabled,
      collision_protection_enabled:(!enabled||isV2())&&!["off","monitor"].includes($("collision-mode").value)});
    notice(isV2() ? enabled?"Ruckig 限速已关闭，可手动开启实时下发":"Ruckig 限速已恢复；实时下发需手动开启"
      : enabled ? "直控已开启：Ruckig、碰撞处理和额外关节平滑已关闭；请手动开始识别并开启下发"
      : "Ruckig 已恢复，可选择碰撞处理；请手动开始识别并开启下发");
  } catch(error) {
    notice(`切换失败，实时下发保持禁用：${error.message}`);
  } finally { controlBusy=false;renderHardware(); }
});
async function switchModel() {
  await stop({message: "切换模型，原会话已结束"});
  configureModelControls(); await loadModel();
  const url = new URL(location.href); url.searchParams.set("model", $("hand-model").value);
  history.replaceState(null, "", url);
}
$("side").addEventListener("change", switchModel);
$("hand-model").addEventListener("change", switchModel);
$("refresh-ports").addEventListener("click", () => controlAction("ports"));
$("serial-port").addEventListener("change", () => renderHardware());
$("connect-serial").addEventListener("click", () => controlAction(hardware.connected ? "disconnect" : "connect", {port: $("serial-port").value}));
$("home-hand").addEventListener("click", () => controlAction(hardware.homing_active ? "cancel_home" : "home", {iq_seek: Number($("homing-iq").value)}));
$("homing-result-open").addEventListener("click", () => showHomingReport());
$("arm-hand").addEventListener("click", () => controlAction("arm", {enabled: !hardware.armed}));
$("emergency-hand").addEventListener("click", () => controlAction("emergency_stop"));
$("close-control").addEventListener("click", () => stop());
$("reset-view").addEventListener("click", () => viewer?.resetView());
$("reset-feedback-view").addEventListener("click", () => feedbackViewer?.resetView());
navigator.mediaDevices?.addEventListener("devicechange", () =>
  enumerateCameras().catch(() => {}),
);
document.addEventListener("visibilitychange", () => {
  diagnostics.record("visibility_change",{visibility:document.visibilityState});diagnostics.flush();
  if (document.hidden && (active || starting || ["TsenseI_v2", ...A30_MODEL_IDS, ...A32_MODEL_IDS].includes(session?.model_id)))
    stop({ message: "页面已隐藏，识别已暂停" });
});
window.addEventListener("pagehide", () => {
  diagnostics.record("page_hide",{state:diagnosticState()});diagnostics.flush();
  localControl.disarm("页面已关闭");
  localControl.disconnect().catch(() => {});
  recorder.flush();
  client?.close();
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
  if (localControl.connected || a32Panel.state.connected || recorder.active) { event.preventDefault(); event.returnValue = ""; }
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
    for(const model of ['A30','A32','V2'])await directory.getDirectoryHandle(model,{create:true});
    await recorder.setDirectory(directory);
    $("recording-directory-status").textContent=`已授权 ${directory.name} · 结束会话自动按 A30 / A32 / V2 保存 · 每段 512 MiB`;
  }catch(error){if(error.name!=='AbortError')notice(error.message);}
});
recorder.getDirectory().then(directory=>{
  if(directory)$("recording-directory-status").textContent=directory.permission==='granted'
    ?`已授权 ${directory.name} · 结束会话按型号保存 · 每段 512 MiB`
    :`上次目录 ${directory.name} · 请点击“选择日志根目录”重新授权`;
}).catch(error=>{$("recording-directory-status").textContent=`目录设置不可用：${error.message} · 可使用下载`;});
$("recover-recording").addEventListener("click", async () => {
  try {
    if(localControl.armed||localControl.armPending||localControl.homing){notice("请先结束实时下发或找零，再导出历史记录");return;}
    const id = $("local-recordings").value;
    if (id) { await recorder.flush(); showDownload(await recorder.export(id), true); }
  } catch (error) { notice(error.message); }
});
$("download-recording-part").addEventListener("click",async()=>{
  try{
    if(localControl.armed||localControl.armPending||localControl.homing){notice("请先结束实时下发或找零，再导出历史记录");return;}
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
  if (["webxr", "wuji_glove", "leapmotion"].includes(requestedSource)) $("input-source").value = requestedSource;
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
  viewer.beforeRender=()=>{
    if(!isA30()||!modelReady)return;
    if(localControl.armed||localControl.armPending||localControl.enabling)a30Preview.stop();
    const sample=localControl.armed ? localControl.lastWrittenSample : a30Preview.latest;
    if(!sample||sample===displayedSample)return;
    if(!localControl.armed&&performance.now()-sample.received_at>150)return;
    viewer.setPose(sample.pose);displayedSample=sample;
    recorder.record('model_sample',{sample_ticks:sample.tick,frame_id:sample.frame_id,
      browser_epoch:sample.epoch,source:localControl.armed?'successful_usb_write':'planner_preview_stream',
      write_to_render_ms:localControl.armed?performance.now()-sample.written_at:null});
  };
  await loadModel();
  await enumerateCameras();
  await refreshRecordings();
} catch (error) {
  $("connection-text").textContent = "初始化失败";
  notice(`无法启动工作台：${error.message}`);
}
