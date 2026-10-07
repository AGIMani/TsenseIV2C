import test from 'node:test';
import assert from 'node:assert/strict';
import { FeedbackPreview } from '../static/feedback-preview.js';

function deferred() {
  let resolve, reject;
  const promise = new Promise((accept, fail) => { resolve = accept; reject = fail; });
  return { promise, resolve, reject };
}

function harness(request = async () => ({ pose: { a: 0.1, b: 0.2 }, missing_joints: [], total_joints: 2 })) {
  let time = 0;
  const requests = [], poses = [], states = [];
  const preview = new FeedbackPreview({
    request: args => { requests.push(args); return request(args); },
    display: pose => poses.push(pose),
    changed: state => states.push(state),
    now: () => time,
  });
  preview.configure('model-a', 'left', ['a', 'b']);
  return {
    preview, requests, poses, states,
    setTime: value => { time = value; },
    update: (telemetry = {}, extra = {}) => preview.update({ connected: true, telemetry, receivedAt: time, ...extra }),
    state: () => states.at(-1),
  };
}

test('fresh complete feedback is displayed once per measurement, including valid zero angles', async () => {
  const h = harness(async () => ({ pose: { a: 0, b: 0.2, extra: 42 }, missing_joints: [], total_joints: 2 }));
  const telemetry = { motors: [1, 2] };
  h.update(telemetry);
  await h.preview.tick();
  assert.deepEqual(h.requests, [{ model_id: 'model-a', side: 'left', telemetry }]);
  assert.deepEqual(h.poses, [{ a: 0, b: 0.2 }]);
  assert.deepEqual(h.state(), { status: 'live', ageMs: 0, missingCount: 0, hasPose: true, error: null });
  h.setTime(100);
  h.update(telemetry);
  await h.preview.tick();
  assert.equal(h.requests.length, 1);
  assert.equal(h.state().ageMs, 100);
});

test('polls with the same device identity cannot refresh old telemetry', async () => {
  const h = harness();
  h.update({ frame: 1 }, { identity: 1 });
  await h.preview.tick();
  h.setTime(499);
  h.update({ frame: 1 }, { identity: 1 });
  assert.equal(h.state().ageMs, 499);
  h.setTime(500);
  h.update({ frame: 1 }, { identity: 1 });
  await h.preview.tick();
  assert.equal(h.state().status, 'stale');
  assert.equal(h.state().hasPose, true);
  assert.equal(h.requests.length, 1);
  h.update({ frame: 2 }, { identity: 2 });
  await h.preview.tick();
  assert.equal(h.requests.length, 2);
  assert.equal(h.state().status, 'live');
});

test('an expired measurement does not issue a request or show a default pose', async () => {
  const h = harness();
  h.setTime(600);
  h.update({}, { receivedAt: 100 });
  await h.preview.tick();
  assert.equal(h.requests.length, 0);
  assert.equal(h.poses.length, 0);
  assert.equal(h.state().status, 'stale');
  assert.equal(h.state().hasPose, false);
});

for (const [label, pose, missing_joints] of [
  ['absent joint', { a: 0.5 }, []],
  ['NaN angle', { a: 0.5, b: NaN }, []],
  ['infinite angle', { a: 0.5, b: Infinity }, []],
  ['null angle', { a: 0.5, b: null }, []],
  ['string angle', { a: 0.5, b: '0.2' }, []],
  ['server-reported missing joint', { a: 0.5, b: 0.2 }, ['b']],
]) {
  test(`incomplete first sample (${label}) displays only the measured joints`, async () => {
    const h = harness(async () => ({ pose, missing_joints }));
    h.update();
    await h.preview.tick();
    assert.deepEqual(h.poses, [{ a: 0.5 }]);
    assert.equal(h.state().status, 'incomplete');
    assert.equal(h.state().missingCount, 1);
    assert.equal(h.state().hasPose, true);
  });
}

test('each partial sample updates only the angles that arrived in that measurement', async () => {
  const results = [{ pose: { a: 0.1 } }, { pose: { b: 0.2 } }, { pose: { a: 0.3, b: 0.4 } }, { pose: { a: 0.8 } }];
  const h = harness(async () => results.shift());
  for (let i = 0; i < 4; i++) {
    h.setTime(i * 40);
    h.update({ sequence: i });
    await h.preview.tick();
    assert.equal(h.poses.length, i + 1);
  }
  assert.deepEqual(h.poses, [{ a: 0.1 }, { b: 0.2 }, { a: 0.3, b: 0.4 }, { a: 0.8 }]);
  const modelPose = {};
  for (const update of h.poses) Object.assign(modelPose, update);
  assert.deepEqual(modelPose, { a: 0.8, b: 0.4 });
  assert.equal(h.state().status, 'incomplete');
  assert.equal(h.state().hasPose, true);
});

test('conversion failures expose the error, avoid a request storm, and recover on fresh data', async () => {
  let fail = true;
  const h = harness(() => {
    if (fail) throw new Error('conversion offline');
    return { pose: { a: 0.2, b: 0.3 } };
  });
  h.update({}, { identity: 'sample-1' });
  await h.preview.tick();
  assert.equal(h.state().status, 'error');
  assert.equal(h.state().error, 'conversion offline');
  h.setTime(200);
  await h.preview.tick();
  assert.equal(h.requests.length, 1);
  h.setTime(500);
  await h.preview.tick();
  assert.equal(h.state().status, 'stale');
  assert.equal(h.requests.length, 1);
  fail = false;
  h.update({}, { identity: 'sample-2' });
  await h.preview.tick();
  assert.equal(h.requests.length, 2);
  assert.equal(h.state().status, 'live');
  assert.equal(h.state().error, null);
});

test('a malformed conversion response is an error and preserves a prior complete pose', async () => {
  const results = [{ pose: { a: 0.1, b: 0.2 } }, { pose: null }];
  const h = harness(async () => results.shift());
  h.update();
  await h.preview.tick();
  h.update();
  await h.preview.tick();
  assert.equal(h.poses.length, 1);
  assert.equal(h.state().status, 'error');
  assert.equal(h.state().hasPose, true);
});

test('one request is in flight and pending measurements collapse to the latest', async () => {
  const first = deferred();
  const h = harness(args => args.telemetry.sequence === 1 ? first.promise : { pose: { a: 0.8, b: 0.9 } });
  h.update({ sequence: 1 });
  const running = h.preview.tick();
  await Promise.resolve();
  h.setTime(10);
  h.update({ sequence: 2 });
  h.preview.tick();
  h.setTime(20);
  h.update({ sequence: 3 });
  h.preview.tick();
  assert.equal(h.requests.length, 1);
  first.resolve({ pose: { a: 0.1, b: 0.2 } });
  await running;
  await h.preview.tick();
  assert.deepEqual(h.requests.map(args => args.telemetry.sequence), [1, 3]);
  assert.deepEqual(h.poses.at(-1), { a: 0.8, b: 0.9 });
});

test('a response arriving after its measurement expires cannot move the model', async () => {
  const pending = deferred();
  const h = harness(() => pending.promise);
  h.update();
  const running = h.preview.tick();
  await Promise.resolve();
  h.setTime(500);
  pending.resolve({ pose: { a: 0.1, b: 0.2 } });
  await running;
  assert.equal(h.poses.length, 0);
  assert.equal(h.state().status, 'stale');
});

for (const change of ['model', 'side', 'disconnect', 'unsupported', 'dispose']) {
  test(`${change} invalidates an outstanding response`, async () => {
    const pending = deferred();
    const h = harness(() => pending.promise);
    h.update();
    const running = h.preview.tick();
    await Promise.resolve();
    if (change === 'model') h.preview.configure('model-b', 'left', ['a', 'b']);
    if (change === 'side') h.preview.configure('model-a', 'right', ['a', 'b']);
    if (change === 'disconnect') h.preview.update({ connected: false, telemetry: null });
    if (change === 'unsupported') h.update({}, { supported: false });
    if (change === 'dispose') h.preview.dispose();
    const count = h.states.length;
    pending.resolve({ pose: { a: 0.1, b: 0.2 } });
    await running;
    assert.equal(h.poses.length, 0);
    assert.equal(h.states.length, count);
    if (change === 'disconnect') assert.equal(h.state().status, 'disconnected');
    if (change === 'unsupported') assert.equal(h.state().status, 'unsupported');
    if (change === 'dispose') {
      h.update();
      await h.preview.tick();
      assert.equal(h.states.length, count);
      assert.equal(h.requests.length, 1);
    }
  });
}

test('configuration changes cannot create overlapping requests and use the new model and side', async () => {
  const pending = deferred();
  const h = harness(args => args.model_id === 'model-a' ? pending.promise : { pose: { c: 0.4 } });
  h.update();
  const running = h.preview.tick();
  await Promise.resolve();
  h.preview.configure('model-b', 'right', ['c']);
  h.update({ sequence: 2 });
  h.preview.tick();
  assert.equal(h.requests.length, 1);
  pending.resolve({ pose: { a: 0.1, b: 0.2 } });
  await running;
  await h.preview.tick();
  assert.deepEqual(h.poses, [{ c: 0.4 }]);
  assert.equal(h.requests[1].model_id, 'model-b');
  assert.equal(h.requests[1].side, 'right');
});

test('disconnect keeps the displayed pose indefinitely and reconnect updates only received joints', async () => {
  const results = [{ pose: { a: 0.1, b: 0.2 } }, { pose: { a: 0.5 } }];
  const h = harness(async () => results.shift());
  h.update();
  await h.preview.tick();
  h.preview.update({ connected: false, telemetry: null });
  assert.equal(h.state().hasPose, true);
  h.setTime(60_000);
  await h.preview.tick();
  assert.equal(h.state().status, 'disconnected');
  assert.equal(h.state().hasPose, true);
  assert.deepEqual(h.poses, [{ a: 0.1, b: 0.2 }]);
  h.update();
  await h.preview.tick();
  assert.deepEqual(h.poses, [{ a: 0.1, b: 0.2 }, { a: 0.5 }]);
  assert.equal(h.state().hasPose, true);
  assert.equal(h.state().status, 'incomplete');
});

test('connected without telemetry waits and disabled sources issue no conversion requests', async () => {
  const h = harness();
  h.preview.update({ connected: true, telemetry: null });
  await h.preview.tick();
  assert.equal(h.state().status, 'waiting');
  assert.equal(h.state().ageMs, null);
  h.update({}, { supported: false });
  await h.preview.tick();
  assert.equal(h.state().status, 'unsupported');
  assert.equal(h.requests.length, 0);
});

for (const receivedAt of [null, undefined, NaN, Infinity, -Infinity]) {
  test(`a telemetry object without a finite acquisition time (${String(receivedAt)}) waits for the first measurement`, async () => {
    const h = harness();
    h.update({ connected: true, telemetry_age_ms: -1 }, { receivedAt });
    await h.preview.tick();
    assert.equal(h.requests.length, 0);
    assert.equal(h.poses.length, 0);
    assert.equal(h.state().status, 'waiting');
    assert.equal(h.state().ageMs, null);
  });
}

test('new raw packets cannot keep an old displayed pose live while conversion is pending', async () => {
  const pending = deferred();
  const h = harness(args => args.telemetry.sequence === 0
    ? { pose: { a: 0.1, b: 0.2 } } : pending.promise);
  h.update({ sequence: 0 });
  await h.preview.tick();
  h.setTime(20);
  h.update({ sequence: 1 });
  const running = h.preview.tick();
  await Promise.resolve();
  for (let time = 100; time <= 600; time += 100) {
    h.setTime(time);
    h.update({ sequence: time });
    h.preview.tick();
    assert.equal(h.state().ageMs, time);
    if (time >= 500) assert.equal(h.state().status, 'stale');
  }
  assert.equal(h.requests.length, 2);
  pending.resolve({ pose: { a: 0.9, b: 0.8 } });
  await running;
  assert.deepEqual(h.poses, [{ a: 0.1, b: 0.2 }]);
  assert.equal(h.state().status, 'stale');
});

for (const failure of ['incomplete', 'error']) {
  test(`slow ${failure} conversions remain visible while newer raw measurements arrive`, async () => {
    const pending = [];
    const h = harness(args => {
      if (args.telemetry.sequence === 0) return { pose: { a: 0.1, b: 0.2 } };
      const request = deferred();
      pending.push(request);
      return request.promise;
    });
    h.update({ sequence: 0 });
    await h.preview.tick();
    for (let cycle = 0; cycle < 4; cycle++) {
      const time = cycle * 200 + 10;
      h.setTime(time);
      h.update({ sequence: time });
      if (cycle) assert.equal(h.state().status, failure);
      const running = h.preview.tick();
      await Promise.resolve();
      h.setTime(time + 50);
      h.update({ sequence: time + 50 });
      h.setTime(time + 100);
      h.update({ sequence: time + 100 });
      if (failure === 'error') pending.at(-1).reject(new Error('conversion unavailable'));
      else pending.at(-1).resolve({ pose: { a: 0.8 }, missing_joints: ['b'] });
      await running;
      assert.equal(h.state().status, failure);
      assert.equal(h.state().ageMs, failure === 'incomplete' ? 100 : time + 100);
      assert.equal(h.state().hasPose, true);
      if (failure === 'error') assert.equal(h.state().error, 'conversion unavailable');
      else assert.equal(h.state().missingCount, 1);
    }
    assert.deepEqual(h.poses, failure === 'incomplete'
      ? [{ a: 0.1, b: 0.2 }, { a: 0.8 }, { a: 0.8 }, { a: 0.8 }, { a: 0.8 }]
      : [{ a: 0.1, b: 0.2 }]);
    h.setTime(850);
    h.update({ sequence: 850 });
    assert.equal(h.state().status, failure);
    const recovered = h.preview.tick();
    await Promise.resolve();
    pending.at(-1).resolve({ pose: { a: 0.3, b: 0.4 } });
    await recovered;
    assert.equal(h.state().status, 'live');
    assert.equal(h.state().ageMs, 0);
    assert.equal(h.state().error, null);
    assert.equal(h.state().missingCount, 0);
  });
}

test('an invalid acquisition time cancels an earlier outstanding conversion', async () => {
  const pending = deferred();
  const h = harness(() => pending.promise);
  h.update();
  const running = h.preview.tick();
  await Promise.resolve();
  h.update({}, { receivedAt: null });
  pending.resolve({ pose: { a: 0.1, b: 0.2 } });
  await running;
  assert.equal(h.poses.length, 0);
  assert.equal(h.state().status, 'waiting');
});

test('a measurement with no usable joints never changes the pose or invents a first measurement', async () => {
  const h = harness(async () => ({ pose: { a: NaN, b: 0.4 }, missing_joints: ['b'] }));
  h.update();
  await h.preview.tick();
  assert.deepEqual(h.poses, []);
  assert.equal(h.state().hasPose, false);
  assert.equal(h.state().status, 'incomplete');
  assert.equal(h.state().missingCount, 2);
});

test('disconnect preserves partial pose and rejects a pending update after reconnect', async () => {
  const pending = deferred();
  const h = harness(args => args.telemetry.sequence === 0 ? { pose: { a: 0.3 } } : pending.promise);
  h.update({ sequence: 0 });
  await h.preview.tick();
  h.update({ sequence: 1 });
  const running = h.preview.tick();
  await Promise.resolve();
  h.preview.update({ connected: false, telemetry: null });
  assert.equal(h.state().hasPose, true);
  h.update({ sequence: 2 });
  pending.resolve({ pose: { a: 0.9, b: 0.8 } });
  await running;
  assert.deepEqual(h.poses, [{ a: 0.3 }]);
  assert.equal(h.state().hasPose, true);
});

for (const [model, side] of [['model-b', 'left'], ['model-a', 'right']]) {
  test(`changing to ${model}/${side} clears the prior measured pose even while offline`, async () => {
    const h = harness(async () => ({ pose: { a: 0.3 } }));
    h.update();
    await h.preview.tick();
    h.preview.update({ connected: false, telemetry: null });
    assert.equal(h.state().hasPose, true);
    h.preview.configure(model, side, ['a', 'b']);
    assert.equal(h.state().hasPose, false);
    assert.equal(h.state().ageMs, null);
  });
}
