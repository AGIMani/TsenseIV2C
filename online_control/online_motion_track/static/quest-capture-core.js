// The joint order and wire format are shared with src/tracking/webxr_capture.html.
export const SCHEMA = 'teleop.webxr.hand_skeleton.v1';
export const JOINT_SET = 'webxr-hand-25';
export const ADB_SCHEMA = 'teleop.quest.adb.hand_skeleton.v1';
export const ADB_JOINT_SET = 'mediapipe-21';
// Omit the four non-thumb metacarpals to match the desktop's 21-point input.
export const WEBXR_25_TO_MEDIAPIPE_21 = Object.freeze([
  0, 1, 2, 3, 4, 6, 7, 8, 9, 11, 12, 13, 14, 16, 17, 18, 19, 21, 22, 23, 24,
]);
export const JOINT_NAMES = Object.freeze([
  'wrist', 'thumb-metacarpal', 'thumb-phalanx-proximal', 'thumb-phalanx-distal', 'thumb-tip',
  ...['index', 'middle', 'ring', 'pinky'].flatMap(finger => [
    `${finger}-finger-metacarpal`, `${finger}-finger-phalanx-proximal`,
    `${finger}-finger-phalanx-intermediate`, `${finger}-finger-phalanx-distal`, `${finger}-finger-tip`,
  ]),
]);
export const FINGER_CHAINS = Object.freeze([
  [0, 1, 2, 3, 4], [0, 5, 6, 7, 8, 9], [0, 10, 11, 12, 13, 14],
  [0, 15, 16, 17, 18, 19], [0, 20, 21, 22, 23, 24],
]);
const round6 = value => Math.round(value * 1e6) / 1e6;
const emptyJoint = name => ({name, valid: false, position: null, orientation: null, radius: null});

export function tokenFromFragment(fragment) {
  return (new URLSearchParams(String(fragment).replace(/^#/, '')).get('token') || '').trim();
}

export function collectHand(frame, inputSource, referenceSpace) {
  if (!['left', 'right'].includes(inputSource.handedness)) return null;
  const joints = JOINT_NAMES.map(name => {
    const jointSpace = inputSource.hand?.get(name);
    let pose = null;
    try { pose = jointSpace && referenceSpace ? frame.getJointPose(jointSpace, referenceSpace) : null; }
    catch { /* A disappearing input source is an unavailable joint, never the previous pose. */ }
    if (!pose) return emptyJoint(name);
    const p = pose.transform?.position, q = pose.transform?.orientation;
    const values = [p?.x, p?.y, p?.z, q?.x, q?.y, q?.z, q?.w];
    if (!values.every(Number.isFinite)) return emptyJoint(name);
    return {
      name, valid: true, position: values.slice(0, 3).map(round6),
      orientation: values.slice(3).map(round6),
      radius: Number.isFinite(pose.radius) && pose.radius >= 0 ? round6(pose.radius) : null,
    };
  });
  const valid_joint_count = joints.filter(joint => joint.valid).length;
  return {handedness: inputSource.handedness, tracked: valid_joint_count === 25, valid_joint_count, joints};
}

export function missingHand(handedness) {
  return {handedness, tracked: false, valid_joint_count: 0, joints: JOINT_NAMES.map(emptyJoint)};
}

export function collectHands(frame, inputSources, referenceSpace) {
  const found = new Map();
  for (const input of inputSources) {
    if (!input.hand) continue;
    const hand = collectHand(frame, input, referenceSpace);
    if (hand) found.set(hand.handedness, hand);
  }
  return ['left', 'right'].map(side => found.get(side) || missingHand(side));
}

export function toAdbHand(hand) {
  const joints = WEBXR_25_TO_MEDIAPIPE_21.map(index => hand.joints[index]);
  const tracked = joints.every(joint => joint?.valid && Array.isArray(joint.position)
    && joint.position.length === 3 && joint.position.every(Number.isFinite));
  return {handedness: hand.handedness, tracked,
    landmarks: tracked ? joints.map(joint => [...joint.position]) : []};
}

// One HTTP request in flight, one replaceable skeleton, and a bounded event queue.
// The hello is retried until acknowledged so a frame cannot precede stream creation.
export class QuestPublisher {
  constructor({token, endpoint = '/api/webxr/publish', fetchImpl = (...args) => globalThis.fetch(...args), now = () => performance.now(),
    onStatus = () => {}, onFatal = () => {}, maxFrameAgeMs = 250, timeoutMs = 2000, retryMs = 500}) {
    Object.assign(this, {token, endpoint, fetchImpl, now, onStatus, onFatal, maxFrameAgeMs, timeoutMs, retryMs});
    this.pending = null;
    this.events = [];
    this.running = false;
    this.ending = false;
    this.closed = false;
    this.controller = null;
    this.stats = {sent: 0, replaced: 0, expired: 0, failed: 0, timedOut: 0, requestMs: 0};
    this.done = new Promise(resolve => { this.resolveDone = resolve; });
  }

  offer(message) {
    if (this.closed || this.ending) return;
    const item = {message, offeredAt: this.now()};
    if (message.type === 'hand_skeleton') {
      if (this.pending) this.stats.replaced++;
      this.pending = item;
    } else {
      // A loss notification invalidates any skeleton waiting behind a slow request.
      if (message.type === 'tracking_lost') this.pending = null;
      this.events.push(item);
      if (this.events.length > 16) {
        const index = this.events.findIndex(event => !this.isHandshake(event));
        if (index >= 0) this.events.splice(index, 1);
      }
    }
    void this.drain();
  }

  isHandshake(item) { return ['client_hello', 'session_started'].includes(item.message.type); }

  finish(message) {
    if (this.closed || this.ending) return this.done;
    this.ending = true;
    this.pending = null;
    this.events = this.events.filter(item => this.isHandshake(item));
    this.events.push({message, offeredAt: this.now()});
    void this.drain();
    return this.done;
  }

  close() {
    this.closed = true;
    this.pending = null;
    this.events = [];
    this.controller?.abort();
    this.resolveDone();
  }

  next() {
    if (this.pending && this.now() - this.pending.offeredAt > this.maxFrameAgeMs) {
      this.pending = null;
      this.stats.expired++;
    }
    if (this.events.length && (!this.pending || this.events[0].message.sequence < this.pending.message.sequence)) {
      return this.events.shift();
    }
    const item = this.pending;
    this.pending = null;
    return item;
  }

  async drain() {
    if (this.running || this.closed) return;
    this.running = true;
    try {
      let item;
      while (!this.closed && (item = this.next())) {
        const terminal = item.message.type === 'session_ended';
        const handshake = this.isHandshake(item);
        const controller = new AbortController();
        this.controller = controller;
        // A pose that can no longer be used must not hold newer poses behind a
        // two-second request plus handshake backoff. Keep handshake/end reliable.
        const budget = item.message.type === 'hand_skeleton'
          ? Math.max(1, this.maxFrameAgeMs - (this.now() - item.offeredAt))
          : this.maxFrameAgeMs;
        const began = this.now();
        const timer = setTimeout(() => controller.abort(),
          handshake || terminal ? this.timeoutMs : Math.min(this.timeoutMs, budget));
        try {
          const response = await this.fetchImpl(this.endpoint, {
            method: 'POST', headers: {'Content-Type': 'application/json'},
            body: JSON.stringify({token: this.token, message: item.message}),
            signal: controller.signal, cache: 'no-store',
          });
          const result = await response.json().catch(() => ({}));
          if (!response.ok) {
            const error = new Error(result.error || result.detail || `HTTP ${response.status}`);
            error.fatal = response.status >= 400 && response.status < 500
              && ![408, 429].includes(response.status);
            throw error;
          }
          if (result.accepted === false && result.reason !== 'stale_sequence') {
            const error = new Error(result.reason || '接收端拒绝此采集会话');
            error.fatal = true;
            throw error;
          }
          if (!this.closed) {
            this.stats.requestMs = this.now() - began;
            this.stats.sent++;
            this.onStatus('connected', this.stats);
          }
        } catch (error) {
          if (this.closed) break;
          this.stats.failed++;
          if (controller.signal.aborted) this.stats.timedOut++;
          this.onStatus('error', this.stats, error);
          if (error.fatal) {
            this.close();
            this.onFatal(error);
            break;
          }
          // Never replay a failed pose. Initial stream events alone require an ACK.
          if (handshake && !this.ending) this.events.unshift(item);
          clearTimeout(timer);
          if (!terminal) await new Promise(resolve => setTimeout(resolve, handshake ? this.retryMs : 0));
        } finally {
          clearTimeout(timer);
          if (this.controller === controller) this.controller = null;
        }
        if (terminal) { this.close(); break; }
      }
    } finally { this.running = false; }
  }
}
