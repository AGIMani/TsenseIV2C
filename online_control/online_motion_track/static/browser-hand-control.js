import {serialOptions,portDescription} from '../../shared/serial-options.js';
import {SerialTrace} from './serial-trace.js';
import {makePacer} from '../../shared/serial-timing.js';
import {collisionProtectionRequired} from './control-readiness.js';
import {frame, handType, linearHome, linearPower, rotaryDrive, rotaryZero, positions, decodePositions, UplinkParser, telemetry, hex, slotMaps, positionSlots, powerChannels, holdTelemetry} from "./serial-protocol.js";

const sleep = ms => new Promise(resolve => setTimeout(resolve, ms));
const LINEAR = [6, 6, 2].flatMap((count, bus) => Array.from({length: count}, (_, i) => [bus, i+1]));
const ROTARY = [[1,1], [1,0], [0,1], [0,0], [3,0], [3,1]];
const BASE = [0, 2, 6, 4];
const LINEAR_JOINTS = ["ring_MCP", "ring_PIP", "ring_DIP", "pinky_MCP", "pinky_PIP", "pinky_DIP", "index_MCP", "index_PIP", "index_DIP", "mid_MCP", "mid_PIP", "mid_DIP", "thumb_MCP", "thumb_PIP"];
const ROTARY_JOINTS = ["index_swing", "mid_swing", "ring_swing", "pinky_swing", "thumb_rota", "thumb_swing", "pinky_rota"];
const FINGERS = {ring: "无名指", pinky: "小拇指", index: "食指", mid: "中指", thumb: "拇指"};
function jointResult(side, kind, bus, motor, slot, reason = null) {
  const joint = (kind === "linear" ? LINEAR_JOINTS : ROTARY_JOINTS)[slot];
  const [finger, part] = joint.split("_");
  return {joint: `${side}_hand_${joint}_joint`, label: `${FINGERS[finger]} ${part === "rota" ? "旋转" : part === "swing" ? "swing 摆动" : part}`,
    kind, bus, motor, slot, channel: kind === "linear" ? `CAN${bus+1} M${motor}` : `SPI${bus+1} M${motor}`,
    success: reason === null, reason};
}
export class BrowserHandControl {
  constructor({recorder, changed = () => {}, now = () => performance.now(), wait = sleep, protocol = null} = {}) {
    this.protocol = protocol || {UplinkParser, telemetry, handType, positions, decodePositions, slotMaps, positionSlots, powerChannels, holdTelemetry};
    this.jointEnabled = {};
    this.powerSent = {}; this.powerBusy = false;
    this.rxBytes = 0; this.parsedPackets = 0; this.acceptedPackets = 0;
    this.recorder = recorder; this.changed = changed; this.now = now;
    this.wait = wait === sleep ? ms => (this.pacer ??= makePacer()).wait(ms) : wait;
    this.homeReport = null; this.resumeRotary = []; this.requiresTelemetrySeed = false;
    this.job = 0; this.connected = false; this.armed = false; this.homing = false;
    this.requireHandConfirmation = true;
    this.directMode = false;
    this.modeConfirmed = true;
    this.homed = false; this.seq = 0; this.sent = 0; this.drops = 0; this.lastError = "";
    this.tail = Promise.resolve(); this.writing = false; this.latest = null;
    this.drive = [[0,0], [0,0], [0,0], [0,0]];
  }
  snapshot() {
    return {connected: this.connected, device: this.deviceLabel || "本机 USB 串口", armed: this.armed,
      homing_active: this.homing, homing_status: this.homeStatus || "未找零", homing_result: this.homeResult,
      homing_report: this.homeReport, joint_enabled: {...this.jointEnabled}, joint_power_sent: {...this.powerSent}, power_busy: this.powerBusy,
      send_ticks: this.sent, send_drops: this.drops, last_error: this.lastError,
      serial_first_error: this.serialTrace?.firstError || null,
      telemetry_issue:this.telemetryIssue(), rx_bytes:this.rxBytes, parsed_packets:this.parsedPackets,
      accepted_packets:this.acceptedPackets, telemetry_age_ms:this.latest?Math.max(0,Math.round(this.now()-this.telemetryAt)):null,
      telemetry_payload_bytes:this.latestPayload?this.latestPayload.length/2:null,
      local_homed: this.homed, local_send_hz: this.sendStarted ? this.sent*1000/Math.max(1,this.now()-this.sendStarted) : 0};
  }
  event(type, data = {}) { return this.recorder?.record(type, {hand_side: this.side,
    direct_mode:this.directMode,ruckig_enabled:!this.directMode,...data}); }
  setDirectMode(enabled, confirmed=true) {
    this.directMode = !!enabled;
    this.modeConfirmed = confirmed;
    this.disarm(enabled ? "Ruckig 已关闭，可手动开启直接下发" : "Ruckig 已恢复，请手动开启实时下发");
  }
  powerSupported(joint) { return !!this.protocol.powerChannels(this.side)?.[joint]; }
  async sendJointPower(joint, enabled, token) {
    const channel = this.protocol.powerChannels(this.side)[joint];
    if (!channel) throw new Error("当前协议不支持该关节独立断电/使能");
    const {kind, bus, motor} = channel;
    if (kind === "self") await this.write(this.protocol.power(bus, motor, enabled ? 1 : 2), {token});
    else if (kind === "linear") await this.write(linearPower(bus, motor, enabled), {token});
    else {
      const drive = [...this.drive[bus]]; drive[motor] = enabled ? 1 : 0;
      await this.write(rotaryDrive(bus, ...drive), {token}); this.drive[bus] = drive;
    }
    this.powerSent[joint] = enabled;
    this.event("joint_power", {joint, enabled, ...channel, sent:true, confirmation:"usb_write_only"});
  }
  async hold(token, source) {
    if (!this.freshTelemetry()) throw new Error("请等待有效的当前位置反馈");
    const bytes = this.protocol.holdTelemetry(this.seq++, this.latest, this.side);
    await this.write(bytes, {token});
    this.event("downlink", {...this.protocol.decodePositions(bytes), sent:true, source, wire_hex:hex(bytes)});
  }
  async setJointEnabled(joint, enabled) { return this.setJointsEnabled({[joint]:enabled}); }
  async setJointsEnabled(selection) {
    if (this.powerBusy || this.homing || this.armPending) throw new Error("请等待当前电机操作结束");
    for (const [joint, enabled] of Object.entries(selection))
      if (typeof enabled !== "boolean" || !this.powerSupported(joint)) throw new Error("当前协议不支持该关节独立断电/使能");
    this.powerBusy = true;
    if (this.connected) this.disarm("关节电源已切换，重新开启实时下发将从反馈姿态起步");
    const token = this.job;
    try {
      // Power switching is independent of telemetry and position commands.
      // Motion stays disarmed; the explicit arm action performs the pose rebase.
      for (const [joint, enabled] of Object.entries(selection)) {
        if (this.connected) await this.sendJointPower(joint, enabled, token);
        this.jointEnabled[joint] = enabled;
        this.event("joint_enable", {joint, enabled, joint_enabled:{...this.jointEnabled}, mode:"motor_power", sent:this.connected});
      }
    } catch (error) { this.lastError = error.message; throw error; }
    finally { this.powerBusy = false; this.changed(); }
  }
  async connect(port, side, baudRate = 115200) {
    if (this.connected) throw new Error("请先断开当前串口");
    const token = ++this.job;
    this.side = side; this.port = port; this.deviceLabel=portDescription(port); this.baudRate=baudRate; this.lastError = ""; this.homed = false;
    this.powerSent = {}; this.drive = [[0,0],[0,0],[0,0],[0,0]];
    this.homeResult = null; this.homeStatus = "未找零"; this.sent = 0; this.drops = 0; this.sendStarted = null;
    this.homeReport = null; this.resumeRotary = []; this.requiresTelemetrySeed = false;
    this.held = null; this.latest = null; this.lastTelemetrySeq = -1; this.lastTelemetryTick = null;
    this.latestPayload = null; this.rxBytes = 0; this.parsedPackets = 0; this.acceptedPackets = 0;
    this.boardTelemetry = null; this.boardTelemetryAt = null;
    this.parser = new this.protocol.UplinkParser(); this.tail = Promise.resolve();
    this.serialTrace = new SerialTrace({now:this.now,emit:(type,data)=>this.event(type,data),
      context:()=>({connected:this.connected,armed:this.armed,port_info:port.getInfo?.()||{},
        parsed_packets:this.parsedPackets,accepted_packets:this.acceptedPackets,
        telemetry_seq:this.lastTelemetrySeq,telemetry_tick_ms:this.lastTelemetryTick,
        telemetry_age_ms:this.latest?this.now()-this.telemetryAt:null})});
    this.serialTrace.event('serial_open_requested',{options:serialOptions(baudRate)});
    try { await port.open(serialOptions(baudRate)); }
    catch(error){this.serialTrace.error('open',error);throw error;}
    if (token !== this.job) { await port.close(); throw new Error("连接已取消"); }
    try {
      this.portDisconnectListener=()=>this.serialTrace.event('serial_device_disconnect',{
        io:this.serialTrace.summary(),recent_io:this.serialTrace.recent()});
      port.addEventListener?.('disconnect',this.portDisconnectListener);
      this.writer = port.writable.getWriter(); this.reader = port.readable.getReader();
      this.connected = true; this.reading = this.readLoop();
      const initial = this.protocol.handType?.(side);
      if (initial) await this.write(initial, {token});
      for (const joint of Object.keys(this.protocol.powerChannels(side)))
        if (this.jointEnabled[joint] === false) await this.sendJointPower(joint, false, token);
      if (token !== this.job) throw new Error("连接已取消");
      this.timer = setInterval(() => this.watchdog(), 25);
      this.event("serial_connect", {baud_rate: baudRate, port_info: port.getInfo(),serial_session_id:this.serialTrace.id}); this.changed();
    } catch (error) { this.serialTrace.error('connect',error);await this.disconnect(); throw error; }
  }
  async readLoop() {
    try {
      while (this.connected) {
        const {done, value} = await this.reader.read();
        if (done) {this.serialTrace?.event('serial_read_end',{expected:!this.connected});break;}
        this.serialTrace?.received(value);
        this.rxBytes += value.length;
        for (const payload of this.parser.push(value)) {
          const t = this.protocol.telemetry(payload); if (!t) continue;
          this.parsedPackets++;
          const restarted = this.lastTelemetryTick !== null &&
            ((t.tick_ms-this.lastTelemetryTick) >>> 0) > 0x80000000;
          const delta = (t.telemetry_seq - this.lastTelemetrySeq) & 65535;
          const accepted = this.lastTelemetrySeq < 0 || restarted ||
            delta > 0 && delta < 0x8000 && t.tick_ms !== this.lastTelemetryTick;
          this.event("telemetry", {...t, hand_side: t.hand_side ?? this.side, accepted,
            payload_hex: hex(payload), ...(payload.wire_info?{wire_info:payload.wire_info}:{}),
            correlation: "latest_host_write", latest_host_command: this.lastCommand || null});
          if (!accepted) continue;
          this.acceptedPackets++;
          this.lastTelemetrySeq = t.telemetry_seq; this.lastTelemetryTick = t.tick_ms;
          this.latest = t; this.telemetryAt = this.now();
          this.latestPayload = hex(payload);
          if (restarted || t.hand_side && t.hand_side !== this.side) {
            this.serialTrace?.event('serial_feedback_reset',{clock_restarted:restarted,
              reported_hand:t.hand_side,telemetry_seq:t.telemetry_seq,telemetry_tick_ms:t.tick_ms});
            const reason = restarted ? "下位机时钟重置，请确认反馈后重新开启下发" : "下位机返回的左右手配置不符";
            this.powerSent = {};
            this.homed = false;
            if (this.homing) this.emergency(reason).catch(() => {});
            else if (this.armed) this.disarm(reason);
          }
        }
        // Board packets have their own clock: they never refresh motor feedback or its rebase payload.
        for (const payload of this.parser.drainBoard?.() || []) {
          const board = this.protocol.boardTelemetry(payload); if (!board) continue;
          this.boardTelemetry = board; this.boardTelemetryAt = this.now();
          this.event("board_telemetry", {...board,payload_hex:hex(payload)});
        }
      }
      if (this.connected) throw new Error("USB 串口已断开");
    } catch (error) {
      if (this.connected) { this.serialTrace?.error('read',error);this.lastError = error.message; this.homed = false; this.disarm(error.message); }
    } finally {
      this.reader?.releaseLock(); this.reader = null;
      // Do not await disconnect here: disconnect itself joins this read loop.
      if (this.connected) setTimeout(() => this.disconnect().catch(() => {}), 0);
    }
  }
  write(bytes, {token = null, stream = false} = {}) {
    const task = this.tail.then(async () => {
      if (!this.connected || !this.writer || token !== null && token !== this.job || stream && (!this.armed || !this.modeConfirmed))
        throw new Error("串口动作已取消");
      let sent = false, timer, writeError;
      const started=this.now();
      try {
        if (bytes[5] === 0x20) {
          this.lastPositionWriteStartedAt = started;
          bytes.jointEnabled = {...this.jointEnabled};
        }
        await Promise.race([this.writer.write(bytes), new Promise((_, reject) => {
          timer = setTimeout(() => reject(new Error("串口写入超过 250 ms，设备状态需重新确认")), 250);
        })]);
        sent = true;
      } catch (error) {
        writeError=error;
        this.homed = false;
        // Abort a stalled stream so no later position commands can queue behind it.
        this.writer?.abort?.(error).catch(() => {});
        throw error;
      } finally { clearTimeout(timer);
        this.serialTrace?.written(bytes,started,sent);
        if(writeError)this.serialTrace?.error('write',writeError,{command:bytes[5],wire_seq:bytes[6],write_timeout_ms:250});
        this.event("usb_tx", {wire_hex: this.recorder?.active ? hex(bytes) : undefined, sent,
        ...(bytes.jointEnabled ? {joint_enabled: bytes.jointEnabled} : {})}); }
    });
    this.tail = task.catch(() => {});
    return task;
  }
  freshTelemetry() {
    return this.latest && this.now()-this.telemetryAt < 500 && (!this.latest.hand_side || this.latest.hand_side === this.side);
  }
  telemetryIssue() {
    if (!this.latest) return this.rxBytes ? "已收到串口数据，但尚未解析到有效手部遥测（检查型号、固件格式和 CRC）" : "尚未收到手部遥测数据";
    if (this.latest.hand_side && this.latest.hand_side !== this.side) return "遥测左右手与所选模型不一致";
    if (this.now()-this.telemetryAt >= 500) return "有效遥测已超过 500 ms 未更新";
    return "";
  }
  controlTelemetryIssue() { return this.telemetryIssue(); }
  observeHand(valid, capturedAt) {
    if (valid) this.handAt = capturedAt;
    else if (this.armed && this.requireHandConfirmation) this.disarm("手势丢失或左右手不符，请重新开启下发");
  }
  disarm(reason = "实时下发已关闭") {
    this.armed = false; this.armPending = false; ++this.job;
    this.lastError = reason; this.event("control_disarm", {reason}); this.changed();
  }
  async arm(rebase) {
    if(!this.modeConfirmed)throw new Error("控制模式尚未确认，请重新切换模式");
    if (!this.connected || this.homing || this.powerBusy || this.armPending)
      throw new Error("请先连接本机串口并等待当前电机操作结束");
    const telemetryIssue = this.controlTelemetryIssue();
    if (telemetryIssue) throw new Error(telemetryIssue);
    if (this.requireHandConfirmation && this.now()-(this.handAt ?? -Infinity) > 400)
      throw new Error("请等待有效手势");
    if (!this.latestPayload || !this.limits) throw new Error("请等待当前位置遥测和模型关节范围");
    const token = ++this.job; this.armPending = true; this.lastError = "";
    try {
      const state = await rebase(this.held, this.latestPayload);
      if (token !== this.job || !this.connected || this.controlTelemetryIssue()) throw new Error("开启下发已取消");
      if (!!state.direct_mode !== this.directMode ||
          (collisionProtectionRequired(this.modelId, state.collision_mode) &&
            (state.collision_mode === "off" || !state.sample_collision_ready)) ||
          (!state.ruckig_enabled && !state.direct_mode))
        throw new Error("控制模式不匹配或碰撞保护不可用");
      this.held = {...state.ruckig_joint_angles_rad};
      // Homing is optional. Always seed the planner and all motor targets from
      // measured feedback before enabling only the selected drives.
      await this.hold(token, "arming/feedback_hold");
      for (const joint of Object.keys(this.protocol.powerChannels(this.side)))
        await this.sendJointPower(joint, this.jointEnabled[joint] !== false, token);
      if (token !== this.job || !this.connected || this.controlTelemetryIssue() ||
          this.requireHandConfirmation && this.now()-(this.handAt ?? -Infinity)>400)
        throw new Error("开启下发已取消或反馈/手势超时");
      this.resumeRotary = []; this.requiresTelemetrySeed = false;
      this.epoch = state.browser_epoch; this.lastSample = -1; this.lastSentAt = this.now();
      this.accepted = false; this.armAt = this.now(); this.lastPollAt = this.now();
      this.armed = true; this.armPending = false; this.lastError = "";
      this.homeResult = null;
      this.event("control_arm", {browser_epoch: this.epoch, seed_pose: this.held}); this.changed();
    } catch (error) { this.disarm(error.message); throw error; }
  }
  writeWatchdogExpired(now) { return this.accepted && now-this.lastSentAt > 150; }
  watchdog() {
    this.serialTrace?.flush();
    if (!this.armed) return;
    const now = this.now();
    const telemetryIssue = this.controlTelemetryIssue();
    if (telemetryIssue) this.disarm(telemetryIssue);
    else if (this.requireHandConfirmation && now-(this.handAt ?? -Infinity) > 400) this.disarm("相机识别超时，实时下发已关闭");
    else if (now-this.lastPollAt > 150 || this.writeWatchdogExpired(now))
      this.disarm("计算或串口链路超时，实时下发已关闭");
    else if (this.requireHandConfirmation && !this.accepted && now-this.armAt > 1500) this.disarm("手势确认超时，请重新开启下发");
  }
  async sample(state, requestedAt) {
    state={...state,frame_id:state.sample_frame_id??state.frame_id};
    const now = this.now();
    if (state.browser_epoch !== this.epoch || state.hand_side !== this.side) return;
    if (!this.armed) return;
    if (now-requestedAt > 100) { this.disarm("计算响应超过 100 ms，已丢弃延迟轨迹"); return; }
    this.lastPollAt = now; this.watchdog(); if (!this.armed) return;
    const phase = state.tracking_transition?.phase;
    if (!state.target_ready || !["recovering", "tracking", ...(!this.requireHandConfirmation ? ["holding"] : [])].includes(phase)) {
      if (this.accepted || phase === "blocked") this.disarm("跟踪保护已阻断下发，请重新开启");
      return;
    }
    if (!!state.direct_mode !== this.directMode || (!state.ruckig_enabled && !state.direct_mode) ||
        (collisionProtectionRequired(this.modelId, state.collision_mode) &&
          (!state.sample_collision_ready || state.collision_mode === "off"))) {
      this.disarm("控制模式不匹配或碰撞保护不可用"); return;
    }
    if (this.writing || state.sample_ticks <= this.lastSample || now-this.lastSentAt < 20) { this.drops++; return; }
    const pose = state.command_joint_angles_rad || state.ruckig_joint_angles_rad;
    const dt = Math.min(.1, Math.max(.002, (now-this.lastSentAt)/1000));
    if (!pose || Object.keys(pose).length !== Object.keys(this.held).length ||
      Object.entries(this.held).some(([key,q]) => {
        const value = pose[key], lim = this.limits?.[key];
        const velocity = state.motion_limits?.[key]?.velocity ?? 10;
        return !Number.isFinite(value) || !lim || value < lim[0]-1e-5 || value > lim[1]+1e-5 ||
          (!this.directMode && Math.abs(value-q) > velocity*3*dt+.002);
      })) { this.disarm("轨迹越界或跳变过大，已阻断下发"); return; }
    const linear = state.linear_strokes_mm, rotary = state.rotary_positions_deg;
    const token = this.job; let bytes;
    try { bytes = this.protocol.positions(this.seq++, linear, rotary, this.side); }
    catch (error) { this.disarm(error.message); return; }
    this.writing = true;
    let sent = false;
    try {
      await this.write(bytes, {token, stream: true}); sent = true;
      this.lastSentAt = this.now(); this.sendStarted ||= this.lastSentAt;
      this.lastSample = state.sample_ticks; this.held = {...pose}; this.accepted = true; this.sent++;
    } catch (error) { if (token === this.job) { this.homed = false; this.disarm(error.message); } }
    finally {
      this.writing = false;
      const enabled = bytes.jointEnabled || {...this.jointEnabled};
      const command = this.event("conversion", {joints_hw: state.hardware_joint_angles_rad,
        linear_mm: linear, rotary_deg: rotary, joint_enabled: enabled,
        frame_id: state.frame_id, direct_mode:this.directMode,
        source: this.directMode ? "browser/direct_motor_power" : "browser/ruckig_motor_power"});
      if(this.directMode)this.event(this.modelId?.startsWith('TsenseI_v3_')?'direct_command':'direct_filter',{joints:pose,frame_id:state.frame_id,
        sample_ticks:state.sample_ticks,sent,stage:'checked',source:'browser_sample'});
      this.event("h7", {joints_ui: pose, joints_hw: state.hardware_joint_angles_rad,
        joint_enabled: enabled,
        frame_id: state.frame_id, sent, usb_connected: this.connected, joints_ruckig_ui: this.directMode?{}:pose,
        ruckig_faults: state.ruckig_faults || [], guard_limited: null, control_dt_s: dt, source: "browser/received_sample"});
      const seq = this.event("downlink", {...this.protocol.decodePositions(bytes), wire_hex: hex(bytes), sent,
        joint_enabled: enabled,
        command_id: command, frame_id: state.frame_id, source: "browser/WebSerial"});
      if (sent) this.lastCommand = {downlink_seq: seq, command_id: command, wire_seq: bytes[6], frame_id: state.frame_id};
    }
  }
  async home(iq, initPose, limits) {
    if (!Number.isFinite(iq) || iq < 0 || iq > 5000) throw new Error("找零 Iq 必须为 0..5000");
    if (!this.connected || this.homing || this.powerBusy || this.armPending || !this.freshTelemetry()) throw new Error("请先连接并等待下位机遥测及当前操作结束");
    this.disarm(""); const token = this.job; this.homing = true; this.homed = false; this.homeResult = null;
    this.homeReport = null; this.resumeRotary = []; this.requiresTelemetrySeed = false;
    const results = [];
    this.drive = [[0,0], [0,0], [0,0], [0,0]];
    const check = () => { if (token !== this.job || !this.connected) throw new Error("找零已取消"); };
    const wait = async ms => { await this.wait(ms); check(); if (!this.freshTelemetry()) throw new Error("找零遥测超时"); };
    const send = async bytes => { check(); await this.write(bytes, {token}); };
    try {
      this.homeStatus = "14 路直线电机找零"; this.changed(); this.event("homing_start", {iq_seek: iq});
      await send(handType(this.side));
      const homeStarted = this.now();
      for (const [slot, [bus, motor]] of LINEAR.entries()) {
        const joint=`${this.side}_hand_${LINEAR_JOINTS[slot]}_joint`;
        if(this.jointEnabled[joint]===false) await this.sendJointPower(joint,false,token);
        else await send(linearHome(bus, motor, 1, iq));
      }
      await wait(1000);
      for (const [bus, motor] of ROTARY) {
        const joint=`${this.side}_hand_${ROTARY_JOINTS[BASE[bus]+motor]}_joint`;
        if(this.jointEnabled[joint]===false){
          await this.sendJointPower(joint,false,token);
          results.push({...jointResult(this.side,"rotary",bus,motor,BASE[bus]+motor,"已选择断电，跳过找零"),skipped:true});
          continue;
        }
        this.homeStatus = `旋转找零 SPI${bus+1} M${motor}`; this.changed();
        this.drive[bus][motor] = 1; await send(rotaryDrive(bus, ...this.drive[bus])); await wait(200);
        const slot = () => this.latest.rotary[BASE[bus]+motor];
        const initial = slot().pos_actual_mm;
        let clear = false, seek = false, motion = false, done = false;
        await send(rotaryZero(bus, motor)); const started = this.now(); let seq = this.lastTelemetrySeq;
        while (this.now()-started < 15000) {
          await wait(50);
          if (seq === this.lastTelemetrySeq) continue; seq = this.lastTelemetrySeq;
          const s = slot(), elapsed = this.now()-started;
          if (![s.iq_ref, s.pos_actual_mm].every(Number.isFinite)) throw new Error("旋转遥测无效");
          clear ||= s.home_label === 0; seek ||= Math.abs(s.iq_ref) >= 200;
          motion ||= Math.abs(s.pos_actual_mm-initial) >= .5;
          if (s.home_label >= 1 && (clear || elapsed >= 1200 && (seek && Math.abs(s.iq_ref) < 50 || motion && Math.abs(s.pos_actual_mm) < 2))) { done = true; break; }
        }
        const result = jointResult(this.side, "rotary", bus, motor, BASE[bus]+motor, done ? null : "15 秒内未确认找零完成");
        results.push(result);
        if (!done) {
          this.drive[bus][motor] = 0;
          await send(rotaryDrive(bus, ...this.drive[bus]));
          this.resumeRotary.push([bus, motor]);
        }
        this.event(done ? "homing_axis_complete" : "homing_axis_failed", result);
      }
      for (const [slot, [bus, motor]] of LINEAR.entries()) {
        if(this.jointEnabled[`${this.side}_hand_${LINEAR_JOINTS[slot]}_joint`]===false){
          results.push({...jointResult(this.side,"linear",bus,motor,slot,"已选择断电，跳过找零"),skipped:true});
          continue;
        }
        const state = this.latest.linear[slot]?.mech_home_state;
        const reason = this.telemetryAt <= homeStarted ? "未收到本次找零后的反馈" :
          state === 2 ? null : ({0: "未开始或未完成", 1: "仍在寻零", 3: "下位机报告找零错误"}[state] || "未确认完成状态");
        const result = jointResult(this.side, "linear", bus, motor, slot, reason);
        results.push(result);
        // Stop an unfinished autonomous LM seek before normal streaming resumes.
        if (reason) await send(linearHome(bus, motor, 2, iq));
        this.event(reason ? "homing_axis_failed" : "homing_axis_complete", result);
      }
      check(); this.held = {...initPose}; this.limits = limits; this.homed = true;
      const failures = results.filter(result => !result.success);
      this.requiresTelemetrySeed = failures.length > 0;
      this.homeReport = {hand_side: this.side, completed: true, total: results.length, results, failures};
      this.homeResult = failures.length ? "partial" : "complete";
      this.homeStatus = failures.length ? `找零结束 · ${failures.length} 个关节未成功，可继续实时下发` : "找零完成 · 实时下发保持关闭";
      this.event("homing_complete", {...this.homeReport, pose: this.requiresTelemetrySeed ? null : this.held,
        seed_source: this.requiresTelemetrySeed ? "telemetry_on_arm" : "init_pose"});
      return this.homeReport;
    } catch (error) {
      if (token === this.job) { this.homeResult = "failed"; this.homeStatus = error.message; await this.emergency(error.message); }
      throw error;
    } finally { this.homing = false; this.changed(); }
  }
  async emergency(reason = "已停止下发并请求电机断电") {
    this.disarm(reason); this.homed = false; this.homing = false; this.homeResult = "cancelled";
    if (!this.connected) return;
    // Stop autonomous homing and disable drives. Do not send a sudden zero pose.
    const results = [];
    for (const [bus, motor] of LINEAR) {
      results.push(await this.write(linearHome(bus, motor, 2, 0)).then(() => true, () => false));
      results.push(await this.write(frame(0x11, new Uint8Array([bus,motor,2,0]))).then(() => true, () => false));
    }
    for (let bus = 0; bus < 4; bus++) results.push(await this.write(rotaryDrive(bus,0,0)).then(() => true, () => false));
    if (results.some(ok => !ok)) this.lastError = "停止指令未全部写入串口，请检查设备";
    this.drive = [[0,0],[0,0],[0,0],[0,0]]; this.powerSent = {};
    this.event("emergency_stop", {sent: results.every(Boolean), reason}); this.changed();
  }
  async disconnect() {
    if (this.disconnecting) return this.disconnecting;
    this.disconnecting = (async () => {
      this.serialTrace?.event('serial_close_requested',{first_error:this.serialTrace.firstError,io:this.serialTrace.summary()});
      this.disarm(this.serialTrace?.firstError?.message || "串口已断开");
      if (this.homing) await this.emergency("断开串口，停止找零");
      clearInterval(this.timer); this.connected = false; this.homed = false;
      this.pacer?.close(); this.pacer = null;
      try { await this.reader?.cancel(); await this.reading; } catch(error) {this.serialTrace?.error('reader_cleanup',error);}
      try { await this.tail; this.writer?.releaseLock(); } catch(error) {this.serialTrace?.error('writer_cleanup',error);}
      this.writer = null;
      try { await this.port?.close(); } catch(error) {this.serialTrace?.error('close',error);}
      this.port?.removeEventListener?.('disconnect',this.portDisconnectListener);this.portDisconnectListener=null;
      this.serialTrace?.flush(true);
      this.serialTrace?.event('serial_close_completed',{first_error:this.serialTrace.firstError,recent_io:this.serialTrace.recent()});
      this.port = null; this.event("serial_disconnect",{first_error:this.serialTrace?.firstError||null}); this.changed();
    })();
    try { await this.disconnecting; } finally { this.disconnecting = null; }
  }
}
