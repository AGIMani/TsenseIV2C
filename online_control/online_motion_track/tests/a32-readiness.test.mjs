import assert from 'node:assert/strict';
import test from 'node:test';
import {A32SdkPanel} from '../static/a32-sdk-control.js';

test('A32 uses A30 startup gates: first telemetry, active tracking, no fresh-hand requirement', async () => {
  const previous = globalThis.document;
  const elements = new Map();
  globalThis.document = {getElementById(id) {
    if (!elements.has(id)) elements.set(id, {value: '', addEventListener() {}});
    return elements.get(id);
  }};
  try {
    const calls = [];
    const session = {session: 'simulated'};
    let running = true;
    const panel = new A32SdkPanel({
      api: async (_url, body) => { calls.push(body); return {...panel.state, armed: body.enabled}; },
      ensureSession: async () => session, currentSession: () => session,
      isRunning: () => running, notice() {}, record() {}, display() {},
    });
    panel.begin = () => {}; // No polling or device access in this UI test.
    for (const model of ['TsenseI_v3_2', 'TsenseI_v3_2_withid']) {
      panel.configure(model, 'left', true);
      panel.update({connected: true, ready: false, feedback_valid: false,
        telemetry_age_ms: -1, telemetry_seen: false, target_ready: true, seed_missing: Array(20).fill('joint')});
      assert.equal(elements.get('a32-arm').disabled, true);
      assert.match(elements.get('a32-arm').title, /一包有效/);
      panel.update({...panel.state, telemetry_seen: true, target_ready: false, hand_recent: false});
      assert.equal(elements.get('a32-arm').disabled, true);
      assert.match(elements.get('a32-arm').title, /新鲜完整反馈/);
      assert.match(elements.get('a32-sdk-detail').textContent, /重新开启前需更新反馈/);
      panel.update({...panel.state, feedback_valid: true, telemetry_age_ms: 0, seed_missing: []});
      assert.equal(elements.get('a32-arm').disabled, false);
      panel.update({...panel.state, telemetry_units: {self_current: 'A', external_current: 'A'}});
      assert.match(elements.get('a32-sdk-detail').textContent, /电流反馈 A · 找零 Iq 仍为码值/);
      for (const collision_mode of ['off', 'monitor']) {
        panel.update({...panel.state, collision_mode, sample_collision_ready: false});
        assert.equal(elements.get('a32-arm').disabled, false);
        assert.match(elements.get('a32-sdk-detail').textContent, collision_mode === 'off' ? /碰撞保护关闭/ : /仅检测 · 不修正/);
      }
      await panel.command('arm');
      assert.equal(calls.at(-1).enabled, true);
      assert.equal(elements.get('a32-arm').textContent, '关闭实时下发');
      panel.update({...panel.state, armed: false, target_ready: false});
      assert.equal(elements.get('a32-arm').disabled, false);
      running = false; panel.render();
      assert.equal(elements.get('a32-arm').disabled, true);
      running = true;
      panel.update({...panel.state, arm_issue: '外接拇指电机 M1 报告故障'});
      assert.equal(elements.get('a32-arm').disabled, true);
      assert.match(elements.get('a32-arm').title, /故障/);
      panel.update({...panel.state, arm_issue: ''});
      panel.update({...panel.state, target_ready: true, homing_active: true});
      assert.equal(elements.get('a32-arm').disabled, true);
      panel.update({...panel.state, homing_active: false, connected: false});
      assert.equal(elements.get('a32-arm').disabled, true);
    }
  } finally { globalThis.document = previous; }
});
