// Native SDK calls are executed in the local Python service, not Web Serial.
import {collisionProtectionLabel} from './control-readiness.js';
export class A32SdkPanel {
  constructor({api, ensureSession, currentSession, isRunning, notice, record, display}) {
    Object.assign(this, {api, ensureSession, currentSession, isRunning, notice, record, display});
    this.state = {}; this.busy = false; this.timer = null; this.serial = 0; this.lastTick = 0;
    this.$ = id => document.getElementById(id);
    for (const [id, action] of [['a32-connect', 'connect'], ['a32-home', 'home'], ['a32-reset-home', 'reset_home'],
      ['a32-arm', 'arm'], ['a32-stop', 'emergency_stop']]) {
      this.$(id).addEventListener('click', () => this.command(action));
    }
    this.$('a32-transport').addEventListener('change', () => this.render());
  }
  configure(model, side, available) {
    this.reset(); this.enabled = model === 'TsenseI_v3_2' || model === 'TsenseI_v3_2_withid';
    this.available = available; this.side = side;
    this.$('a32-sdk-panel').hidden = !this.enabled;
    this.render();
  }
  reset() {
    this.serial++; clearTimeout(this.timer); this.timer = null; this.state = {}; this.updatedAt = null; this.busy = false; this.lastTick = 0;
  }
  update(state, {display = false} = {}) {
    if (!this.enabled || !state) return;
    this.state = state;
    this.updatedAt = performance.now();
    if (state.dropped_command_samples) this.record('a32_sdk_recording_gap', {count: state.dropped_command_samples});
    for (const sample of state.command_samples || []) {
      if (sample.tick <= this.lastTick) continue;
      const direct = sample.direct_mode ?? state.direct_mode;
      this.record('h7', {...sample, sample_ticks: sample.tick, joints_ui: sample.joints,
        joints_hw: sample.joints, joints_ruckig_ui: direct ? {} : sample.ruckig_output_joint_angles_rad,
        direct_mode: !!direct, ruckig_enabled: !direct,
        sent: true, sdk_connected: true, source: 'a32_sdk_successful_write'});
      if (direct) this.record('direct_command', {...sample, sample_ticks: sample.tick,
        direct_mode: true, ruckig_enabled: false, collision_protection_enabled: false,
        sent: true, source: 'a32_sdk_successful_write'});
      this.lastTick = sample.tick;
    }
    const pose = state.command_joint_angles_rad || state.ruckig_joint_angles_rad;
    if (display && state.armed && pose) this.display(pose);
    this.render();
  }
  render() {
    if (!this.enabled) return;
    const s = this.state, connected = !!s.connected, busy = this.busy;
    const allowed = this.available && (this.side === 'left' || this.side === 'right');
    const homing = !!s.homing_active;
    const armIssue = !allowed ? '本机 SDK 或手侧不可用' : !connected ? '请先连接 SDK'
      : !this.isRunning() ? '请先开始识别' : homing ? '自研电机找零中'
      : s.arm_issue || s.control_telemetry_issue || (!s.telemetry_seen ? '首次开启请先接收一包有效的 A3 手部遥测' : s.feedback_valid === false ? '开启下发需要本次连接的新鲜完整反馈' : '');
    this.$('a32-sdk-state').textContent = !this.available ? '预览可用 · 实物控制需本机原生 SDK 服务'
      : s.last_error || (homing ? '自研电机找零中' : connected ? s.armed ? s.sample_collision?.limited ? `碰撞相关手指保持：${(s.held_fingers || []).join('、')}` : 'SDK 实时下发中' : armIssue || '已连接 · 实时下发关闭' : '未连接');
    this.$('a32-connect').textContent = connected ? '断开 SDK' : '连接 SDK';
    this.$('a32-connect').disabled = !allowed || busy;
    this.$('a32-transport').disabled = connected || busy;
    this.$('a32-address').disabled = connected || busy;
    this.$('a32-address').placeholder = this.$('a32-transport').value === 'eth' ? '192.168.1.100' : 'auto 或 /dev/ttyACM0';
    this.$('a32-home').disabled = !connected || busy || homing;
    this.$('a32-reset-home').disabled = !connected || busy;
    this.$('a32-iq').disabled = busy || homing;
    this.$('a32-arm').disabled = busy || (!s.armed && !!armIssue);
    this.$('a32-arm').title = s.armed ? '' : armIssue;
    this.$('a32-arm').textContent = s.armed ? '关闭实时下发' : '开启实时下发';
    this.$('a32-stop').disabled = !connected;
    this.$('a32-sdk-detail').textContent = connected
      ? `18 路 self + 2 路 MIT · 已写入 ${s.sample_ticks || 0} 帧 · 遥测 ${s.telemetry_age_ms ?? '—'} ms · ${s.control_telemetry_issue || (!s.telemetry_seen ? '等待首包遥测' : s.feedback_valid ? '反馈就绪' : '反馈过期或不完整，重新开启前需更新反馈')}${s.seed_missing?.length ? ` · ${s.seed_missing.length} 路反馈无法用于起步` : ''}`
      : '由本机服务连接机械手，USB 自动选择 H7 CDC，ETH 使用 TCP 5001。';
    if (s.collision_mode) this.$('a32-sdk-detail').textContent += ` · ${collisionProtectionLabel(s.collision_mode)}`;
    if (s.telemetry_units?.self_current === 'A') this.$('a32-sdk-detail').textContent += ' · 电流反馈 A · 找零 Iq 仍为码值';
    if (typeof s.direct_mode === 'boolean') this.$('a32-sdk-detail').textContent += s.direct_mode
      ? ' · 直控：Ruckig、碰撞处理及额外关节平滑关闭' : ' · Ruckig 限速';
    const recording = s.recording;
    if (connected && s.packet_timing) {
      const t = s.packet_timing;
      this.$('a32-sdk-detail').textContent += ` · 目标 ${t.target_period_ms} ms/包 · 超过 4 ms：${t.above_4ms} · 小于 1 ms：${t.below_1ms}`;
    }
    if (recording?.path) this.$('a32-sdk-detail').textContent += ` · ${recording.active ? '记录中' : '日志已保存'}：${recording.path}`;
    if (recording?.error) this.$('a32-sdk-detail').textContent += ` · 日志错误：${recording.error}`;
  }
  begin() {
    clearTimeout(this.timer);
    const serial = this.serial;
    const poll = async () => {
      const current = this.currentSession();
      if (!this.enabled || !current || serial !== this.serial) return;
      try {
        const state = await this.api('/api/a32/control', {session: current.session, action: 'status', after_tick: this.lastTick});
        if (serial !== this.serial || current !== this.currentSession()) return;
        this.update(state, {display: true});
        this.record('a32_sdk_sample', {hardware:state, frame_id:state.frame_id,
          telemetry_format:state.telemetry_format, telemetry_units:state.telemetry_units,
          direct_mode:!!state.direct_mode, ruckig_enabled:!state.direct_mode,
          sent:!!state.armed && state.sample_ticks>0});
      } catch (error) {
        if (serial !== this.serial) return;
        this.record("a32_sdk_error", {action:"status",error:error.message,hand_side:this.side});
        this.notice(`A3.2 控制连接中断：${error.message}`);
        // The device worker also stops on a missing browser heartbeat.
        this.state = {...this.state, armed: false, last_error: error.message}; this.render();
        return;
      }
      if (serial === this.serial) this.timer = setTimeout(poll, 200);
    };
    this.timer = setTimeout(poll, 0);
  }
  async command(action) {
    if (this.busy && action !== 'emergency_stop') return;
    const serial = this.serial;
    this.busy = true; this.render();
    try {
      const current = await this.ensureSession();
      if (serial !== this.serial) return;
      const data = {session: current.session, action, after_tick: this.lastTick};
      if (action === 'connect') {
        if (this.state.connected) data.action = 'disconnect';
        else {
          data.transport = this.$('a32-transport').value;
          const address = this.$('a32-address').value.trim();
          if (data.transport === 'eth') { data.host = address; data.port = 5001; }
          else data.port = address || 'auto';
        }
      }
      if (action === 'home') data.iq_counts = Number(this.$('a32-iq').value);
      if (action === 'arm') data.enabled = !this.state.armed;
      const state = await this.api('/api/a32/control', data);
      if (serial !== this.serial) return;
      this.update(state); this.begin(); this.notice();
      this.record('a32_sdk_action', {action: data.action, hardware: state});
    } catch (error) {
      if (serial === this.serial) {
        this.record('a32_sdk_error', {action,error:error.message,hand_side:this.side});
        this.notice(error.message);
      }
    }
    finally { if (serial === this.serial) { this.busy = false; this.render(); } }
  }
  async finish(current) {
    clearTimeout(this.timer);
    if (!this.enabled || !current?.hardware) return;
    const action=this.state.connected ? 'disconnect' : 'status';
    try {
      const state = await this.api('/api/a32/control', {session: current.session,action,after_tick: this.lastTick});
      this.update(state);
      this.record('a32_sdk_action', {action,hardware:state,hand_side:this.side});
    } catch (error) {
      this.record('a32_sdk_error', {action,error:error.message,hand_side:this.side});
      throw error;
    }
  }
}
