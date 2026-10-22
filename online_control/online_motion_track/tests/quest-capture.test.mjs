import test from 'node:test';
import assert from 'node:assert/strict';
import {SCHEMA, ADB_SCHEMA, ADB_JOINT_SET, JOINT_NAMES, collectHand, collectHands, toAdbHand, tokenFromFragment,
  QuestPublisher} from '../static/quest-capture-core.js';

const flush = () => new Promise(resolve => setImmediate(resolve));
const message = (sequence, type = 'hand_skeleton') => ({schema: SCHEMA, stream_id: 'stream-a', sequence, type});
const ok = result => ({ok: true, status: 200, json: async () => result || {accepted: true}});

function harness(options = {}) {
  const calls = [], releases = [], errors = [];
  let clock = 0, active = 0, peak = 0;
  const publisher = new QuestPublisher({token: 'test-secret', now: () => clock, retryMs: 0,
    fetchImpl: (url, request) => {
      calls.push({url, body: JSON.parse(request.body), signal: request.signal});
      peak = Math.max(peak, ++active);
      return new Promise((resolve, reject) => releases.push({
        resolve: result => { active--; resolve(result || ok()); },
        reject: error => { active--; reject(error); },
      }));
    }, onFatal: error => errors.push(error), ...options});
  return {publisher, calls, releases, errors, now: value => { clock = value; }, peak: () => peak};
}

test('capture preserves native WebXR joint order, metres, orientation, and partial tracking', () => {
  const visited = [];
  const frame = {getJointPose(name) {
    visited.push(name);
    if (name === 'index-finger-tip') return null;
    return {transform: {position: {x: 0.123456789, y: 1, z: -0.5}, orientation: {x: 0, y: 0, z: 0, w: 1}}, radius: 0.008};
  }};
  const hand = collectHand(frame, {handedness: 'left', hand: {get: name => name}}, {});
  assert.equal(JOINT_NAMES.length, 25);
  assert.deepEqual(visited, JOINT_NAMES);
  assert.equal(JOINT_NAMES[5], 'index-finger-metacarpal');
  assert.equal(JOINT_NAMES[24], 'pinky-finger-tip');
  assert.equal(hand.tracked, false);
  assert.equal(hand.valid_joint_count, 24);
  assert.deepEqual(hand.joints[0].position, [0.123457, 1, -0.5]);
  assert.deepEqual(hand.joints[0].orientation, [0, 0, 0, 1]);
  assert.deepEqual(hand.joints[9], {name: 'index-finger-tip', valid: false, position: null, orientation: null, radius: null});
});

test('missing sources, exceptions, and nonfinite poses produce null joints instead of stale positions', () => {
  const input = {handedness: 'right', hand: {get: name => name}};
  const invalid = {getJointPose() { return {transform: {position: {x: NaN, y: 0, z: 0}}}; }};
  const throwing = {getJointPose() { throw new Error('source removed'); }};
  for (const frame of [invalid, throwing]) {
    const hands = collectHands(frame, [input, {handedness: 'none', hand: input.hand}], {});
    assert.equal(hands.length, 2);
    assert.deepEqual(hands.map(hand => hand.handedness), ['left', 'right']);
    assert.ok(hands.every(hand => !hand.tracked && hand.valid_joint_count === 0));
    assert.ok(hands.every(hand => hand.joints.every(joint => joint.position === null)));
  }
});

test('pairing token is read from a URL fragment without treating it as a path', () => {
  assert.equal(tokenFromFragment('#token=abc_def-12'), 'abc_def-12');
  assert.equal(tokenFromFragment('#other=1&token=a%2Bb'), 'a+b');
  assert.equal(tokenFromFragment(''), '');
});

test('USB ADB maps native joints into metric MediaPipe order and ignores only non-thumb metacarpals', () => {
  const hand = collectHand({getJointPose(name) {
    const index = JOINT_NAMES.indexOf(name);
    if ([5, 10, 15, 20].includes(index)) return null;
    return {transform: {position: {x: index / 100, y: 1, z: -0.5}, orientation: {x: 0, y: 0, z: 0, w: 1}}};
  }}, {handedness: 'left', hand: {get: name => name}}, {});
  assert.equal(hand.tracked, false, 'native25 has unavailable metacarpals');
  const wire = toAdbHand(hand);
  assert.equal(wire.tracked, true, 'all 21 transmitted joints remain valid');
  assert.equal(wire.handedness, 'left');
  assert.deepEqual(wire.landmarks, [0, 1, 2, 3, 4, 6, 7, 8, 9, 11, 12, 13, 14, 16, 17, 18, 19, 21, 22, 23, 24]
    .map(index => [index / 100, 1, -0.5]));
  wire.landmarks[0][0] = 100;
  assert.equal(hand.joints[0].position[0], 0, 'wire data cannot mutate native rendering data');
});

test('USB ADB missing or nonfinite mapped joint invalidates the entire hand without old points', () => {
  const original = collectHand({getJointPose() {
    return {transform: {position: {x: 0.1, y: 1, z: -0.5}, orientation: {x: 0, y: 0, z: 0, w: 1}}};
  }}, {handedness: 'right', hand: {get: name => name}}, {});
  for (const index of [0, 4, 6, 9, 24]) {
    for (const invalid of [null, {valid: false, position: [1, 2, 3]}, {valid: true, position: [NaN, 2, 3]}]) {
      const hand = {...original, joints: [...original.joints]};
      hand.joints[index] = invalid;
      assert.deepEqual(toAdbHand(hand), {handedness: 'right', tracked: false, landmarks: []});
    }
  }
  assert.deepEqual(collectHands({}, [], {}).map(toAdbHand), [
    {handedness: 'left', tracked: false, landmarks: []},
    {handedness: 'right', tracked: false, landmarks: []},
  ]);
});

test('USB ADB publisher uses its dedicated endpoint and preserves stream envelope and order', async () => {
  const h = harness({endpoint: '/api/quest-adb/publish'});
  const fields = {schema: ADB_SCHEMA, joint_set: ADB_JOINT_SET, units: 'm', transport: 'adb'};
  h.publisher.offer({...message(0, 'client_hello'), ...fields});
  h.publisher.offer({...message(1), ...fields, client_perf_ms: 12, hands: collectHands({}, [], {}).map(toAdbHand)});
  h.releases[0].resolve(); await flush();
  assert.ok(h.calls.every(call => call.url === '/api/quest-adb/publish'));
  assert.deepEqual(h.calls.map(call => call.body.message.sequence), [0, 1]);
  assert.equal(h.calls[1].body.token, 'test-secret');
  assert.equal(h.calls[1].body.message.stream_id, 'stream-a');
  assert.equal(h.calls[1].body.message.client_perf_ms, 12);
  assert.equal(h.calls[1].body.message.schema, 'teleop.quest.adb.hand_skeleton.v1');
  assert.equal(h.calls[1].body.message.joint_set, 'mediapipe-21');
  assert.equal(h.calls[1].body.message.units, 'm');
  h.releases[1].resolve(); await flush();
  h.publisher.close();
});

test('single flight keeps only the latest pose and sends the pairing envelope', async () => {
  const h = harness();
  h.publisher.offer(message(0, 'client_hello'));
  h.publisher.offer(message(1)); h.publisher.offer(message(2));
  assert.equal(h.calls.length, 1);
  h.releases[0].resolve(); await flush();
  assert.deepEqual(h.calls.map(call => call.body.message.sequence), [0, 2]);
  assert.equal(h.calls[1].url, '/api/webxr/publish');
  assert.equal(h.calls[1].body.token, 'test-secret');
  h.releases[1].resolve(); await flush();
  assert.equal(h.peak(), 1);
  assert.equal(h.publisher.stats.replaced, 1);
  h.publisher.close();
});

test('events remain ordered around the replaceable pose', async () => {
  const h = harness();
  h.publisher.offer(message(0, 'client_hello'));
  h.publisher.offer(message(1, 'session_started'));
  h.publisher.offer(message(2));
  h.publisher.offer(message(3, 'tracking_restored'));
  h.publisher.offer(message(4));
  for (let i = 0; i < 4; i++) { h.releases[i].resolve(); await flush(); }
  assert.deepEqual(h.calls.map(call => call.body.message.sequence), [0, 1, 3, 4]);
  h.publisher.close();
});

test('tracking loss clears the queued old pose and missing-hand frames replace it', async () => {
  const h = harness();
  h.publisher.offer(message(0));
  h.publisher.offer(message(1));
  h.publisher.offer(message(2, 'tracking_lost'));
  h.publisher.offer({...message(3), hands: collectHands({}, [], {})});
  for (let i = 0; i < 3; i++) { h.releases[i].resolve(); await flush(); }
  assert.deepEqual(h.calls.map(call => call.body.message.sequence), [0, 2, 3]);
  assert.ok(h.calls[2].body.message.hands.every(hand => !hand.tracked));
  h.publisher.close();
});

test('expired pending skeletons are discarded after a slow request', async () => {
  const h = harness();
  h.publisher.offer(message(0)); h.publisher.offer(message(1));
  h.now(251);
  h.releases[0].resolve(); await flush();
  assert.equal(h.calls.length, 1);
  assert.equal(h.publisher.stats.expired, 1);
  h.publisher.close();
});

test('end clears buffered poses and rejects future poses while retaining stream handshake', async () => {
  const h = harness();
  h.publisher.offer(message(0, 'client_hello'));
  h.publisher.offer(message(1, 'session_started'));
  h.publisher.offer(message(2));
  const done = h.publisher.finish(message(3, 'session_ended'));
  h.publisher.offer(message(4));
  for (let i = 0; i < 3; i++) { h.releases[i].resolve(); await flush(); }
  await done;
  assert.deepEqual(h.calls.map(call => call.body.message.type), ['client_hello', 'session_started', 'session_ended']);
  assert.equal(h.publisher.closed, true);
});

test('stream handshake retries before new poses and duplicate ACK is safe', async () => {
  let calls = 0;
  const messages = [];
  const publisher = new QuestPublisher({token: 't', retryMs: 0,
    fetchImpl: async (url, request) => {
      messages.push(JSON.parse(request.body).message);
      if (++calls === 1) throw new Error('connection reset after server accepted hello');
      return ok(calls === 2 ? {accepted: false, reason: 'stale_sequence'} : undefined);
    }});
  publisher.offer(message(0, 'client_hello'));
  publisher.offer(message(1));
  await new Promise(resolve => setTimeout(resolve, 15));
  assert.deepEqual(messages.map(item => item.sequence), [0, 0, 1]);
  publisher.close();
});

test('failed poses are never retried, and invalid pairing stops the publisher', async () => {
  let calls = 0;
  const sent = [], fatal = [];
  const publisher = new QuestPublisher({token: 't', retryMs: 0,
    onFatal: error => fatal.push(error),
    fetchImpl: async (url, request) => {
      sent.push(JSON.parse(request.body).message.sequence);
      if (++calls === 1) throw new Error('offline');
      return {ok: false, status: 400, json: async () => ({detail: 'pairing expired'})};
    }});
  publisher.offer(message(1)); publisher.offer(message(2));
  await new Promise(resolve => setTimeout(resolve, 15));
  assert.deepEqual(sent, [1, 2]);
  assert.equal(publisher.closed, true);
  assert.equal(fatal[0].message, 'pairing expired');
});

test('close aborts the current request and prevents stale queued delivery', async () => {
  const h = harness();
  h.publisher.offer(message(0)); h.publisher.offer(message(1));
  h.publisher.close();
  assert.equal(h.calls[0].signal.aborted, true);
  h.releases[0].resolve(); await flush();
  assert.equal(h.calls.length, 1);
});

test('slow pose aborts within its age budget and yields to newest pose without handshake backoff', async () => {
  const sent=[];
  let publisher;
  publisher=new QuestPublisher({token:'t',maxFrameAgeMs:30,timeoutMs:2000,retryMs:1000,
    fetchImpl:(_url,request)=>{
      const seq=JSON.parse(request.body).message.sequence;sent.push(seq);
      if(seq===0)return new Promise((_resolve,reject)=>{
        request.signal.addEventListener('abort',()=>{
          publisher.offer(message(2));
          reject(new DOMException('expired','AbortError'));
        },{once:true});
      });
      return Promise.resolve(ok());
    }});
  try{
    publisher.offer(message(0));publisher.offer(message(1));
    await new Promise(resolve=>setTimeout(resolve,100));
    assert.deepEqual(sent,[0,2]);
    assert.equal(publisher.stats.timedOut,1);
    assert.equal(publisher.stats.sent,1);
    assert.equal(publisher.closed,false);
  }finally{publisher.close();}
});

test('retryable HTTP status keeps pairing; server error detail survives fatal rejection', async () => {
  for(const status of [408,429,503]){
    const h=harness();h.publisher.offer(message(1));h.publisher.offer(message(2));
    h.releases[0].resolve({ok:false,status,json:async()=>({error:'temporary'})});
    await new Promise(resolve=>setTimeout(resolve,15));
    assert.equal(h.publisher.closed,false);assert.equal(h.calls.length,2);
    h.releases[1].resolve({ok:false,status:400,json:async()=>({error:'new pairing required'})});
    await flush();assert.equal(h.errors[0].message,'new pairing required');
  }
});
