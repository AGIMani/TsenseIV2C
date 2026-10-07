import test from 'node:test';
import assert from 'node:assert/strict';
import {A30BrowserHandControl} from '../static/a30-browser-hand-control.js';
import {BrowserHandControl} from '../static/browser-hand-control.js';
import * as protocol from '../static/a30-serial-protocol.js';

function fixture(modelId, mode, directMode = false) {
  let now = 1000;
  const writes = [];
  const c = new A30BrowserHandControl({modelId, now: () => now, wait: async ms => {now += ms;}});
  const linear = {}, rotary = {};
  protocol.SELF.forEach((name, i) => (protocol.ROTARY.has(i) ? rotary : linear)[`left_hand_${name}_joint`] = 0);
  protocol.EXTERNAL.forEach(name => rotary[`left_hand_${name}`] = 0);
  const pose = Object.fromEntries([...Object.keys(linear), ...Object.keys(rotary)].map(name => [name, 0]));
  c.connected = true; c.side = 'left'; c.directMode = directMode;
  c.latest = protocol.telemetry(new Uint8Array(484));
  c.latestPayload = '00'.repeat(484); c.telemetryAt = now;
  c.limits = Object.fromEntries(Object.keys(pose).map(name => [name, [-2, 2]]));
  c.feedbackPositionLimits = Object.fromEntries(Object.entries({linear, rotary}).map(([kind, joints]) =>
    [kind, Object.fromEntries(Object.keys(joints).map(name => [name, [-100, 100]]))]));
  c.writer = {async write(bytes) {writes.push(bytes.slice());}};
  const state = {model_id: modelId, collision_mode: mode, direct_mode: directMode,
    sample_collision_ready: false, ruckig_enabled: !directMode, browser_epoch: 1, hand_side: 'left',
    ruckig_joint_angles_rad: pose, linear_strokes_mm: linear, rotary_positions_deg: rotary,
    target_ready: true, tracking_transition: {phase: 'holding'}, sample_ticks: 1};
  return {c, writes, state, advance: ms => now += ms};
}

for (const modelId of ['TsenseI_v3_0', 'TsenseI_v3_0_withid']) {
  for (const mode of ['off', 'monitor']) {
    for (const directMode of [false, true]) {
      test(`${modelId}/${mode}/direct=${directMode}: manual arm writes USB and timeout still disarms`, async () => {
        const {c, writes, state, advance} = fixture(modelId, mode, directMode);
        await c.sample(state, c.now());
        assert.equal(writes.length, 0, 'preview must never enable or send hardware commands');
        await c.arm(async () => state);
        assert(c.armed);
        assert(writes.some(bytes => bytes[5] === 0x20), 'manual arm sends the feedback seed');
        const armedWrites = writes.length;
        advance(25);
        await c.sample(state, c.now());
        assert(c.armed);
        assert.equal(writes.length, armedWrites + 1);
        assert.equal(c.sent, 1);
        assert.equal(writes.at(-1)[5], 0x20);
        advance(151);
        c.watchdog();
        assert.equal(c.armed, false);
        assert.match(c.lastError, /超时/);
        await c.sample({...state, sample_ticks: 2}, c.now());
        assert.equal(writes.length, armedWrites + 1, 'fresh samples cannot re-arm after a timeout');
      });
    }
  }
}

test('optional collision does not waive a missing guard for stop mode or V2 collision-off restriction', async () => {
  const {c, state, writes} = fixture('TsenseI_v3_0', 'off');
  // Test the shared base arm method to avoid A3 emergency-stop packets on a rejected arm.
  await assert.rejects(BrowserHandControl.prototype.arm.call(c, async () => ({...state, collision_mode: 'stop'})), /保护不可用/);
  assert.equal(writes.length, 0);
  c.modelId = 'TsenseI_v2';
  await assert.rejects(BrowserHandControl.prototype.arm.call(c, async () => state), /保护不可用/);
  assert.equal(writes.length, 0);
});

test('collision-off keeps direct-mode confirmation and motor faults mandatory', async () => {
  const {c, state, writes} = fixture('TsenseI_v3_0', 'off');
  c.modeConfirmed = false;
  await assert.rejects(c.arm(async () => state), /尚未确认/);
  assert.equal(writes.length, 0);
  c.modeConfirmed = true;
  c.latest.external[0].online = true;
  c.latest.external[0].fault = 1;
  await assert.rejects(BrowserHandControl.prototype.arm.call(c, async () => state), /故障/);
  assert.equal(writes.length, 0);
});

test('manual A3 arm immediately opens its validated epoch stream without waiting for status polling', async () => {
  const {c, state, writes} = fixture('TsenseI_v3_0', 'off');
  const names = Object.keys(state.ruckig_joint_angles_rad);
  const seeded = {...state, control_stream: true, server_time: 1, control_joint_names: names,
    motion_limits: Object.fromEntries(names.map(name => [name, {velocity: 10, acceleration: 80, jerk: 200}]))};
  const packet = new Uint8Array(220), view = new DataView(packet.buffer);
  view.setUint32(0, 216, true);
  packet.set([71, 84, 82, 49], 4);
  view.setUint32(8, seeded.browser_epoch, true);
  view.setUint32(12, 1, true);
  view.setFloat64(20, 1, true);
  view.setUint32(28, 1, true);
  packet.set(protocol.positions(1, seeded.linear_strokes_mm, seeded.rotary_positions_deg, 'left'), 112);
  const previousFetch = globalThis.fetch, requests = [];
  globalThis.fetch = async (url, options) => {
    requests.push({url, ...JSON.parse(options.body)});
    return new Response(new ReadableStream({start(controller) {controller.enqueue(packet);}}));
  };
  try {
    await c.arm(async () => seeded, {session: 'verified-session'});
    assert.deepEqual(requests, [{url: '/api/a30/stream', session: 'verified-session', epoch: 1}]);
    await new Promise(resolve => setImmediate(resolve));
    assert.equal(c.sent, 1);
    assert(c.armed && c.lastWrittenSample);
    assert.equal(writes.filter(bytes => bytes[5] === 0x20).length, 2, 'one seed and one streamed sample');
    c.disarm('manual');
    await c.streamTask;
    assert.equal(c.armed, false);
  } finally {
    c.disarm('test finished');
    globalThis.fetch = previousFetch;
  }
});

test('failure opening the immediate A3 stream disarms without retrying or re-enabling', async () => {
  const {c, state, writes} = fixture('TsenseI_v3_0', 'off');
  const previousFetch = globalThis.fetch;
  let requests = 0;
  globalThis.fetch = async () => {requests++; throw Error('injected stream failure');};
  try {
    await c.arm(async () => ({...state, control_stream: true, server_time: 1}), {session: 'verified-session'});
    await c.streamTask;
    assert.equal(c.armed, false);
    assert.match(c.lastError, /injected stream failure/);
    assert.equal(requests, 1);
    assert.equal(writes.filter(bytes => bytes[5] === 0x20).length, 1, 'only the explicit arm seed was sent');
  } finally {globalThis.fetch = previousFetch;}
});
