import test from "node:test";
import assert from "node:assert/strict";
import { CameraOverlay, landmarkPixel } from "../static/overlay.js";
import { OverlaySmoothing } from "../static/overlay-smoothing.js";

const hand = (x = .5, y = .5) => Array.from({length: 21}, (_, id) =>
  ({x: x + (id % 4) * .01, y: y + Math.floor(id / 4) * .01, z: -.01 * id}));
function random(seed) {
  return () => {
    seed = (1664525 * seed + 1013904223) >>> 0;
    return seed / 2 ** 32;
  };
}

for (const rate of [15, 30, 60, 90, 120]) {
  test(`${rate} FPS: rest jitter, moving lag and step response have bounded budgets`, () => {
    const smoother = new OverlaySmoothing(), rng = random(491 + rate), truth = hand();
    let rawError = 0, filteredError = 0;
    for (let i = 0; i < rate * 8; i++) {
      const raw = truth.map(p => ({...p, x: p.x + (rng() - .5) * .012, y: p.y + (rng() - .5) * .012}));
      const actual = smoother.update([raw], i * 1000 / rate, ['Left'])[0];
      if (i > rate) for (let j = 0; j < 21; j++) {
        rawError += (raw[j].x - truth[j].x) ** 2;
        filteredError += (actual[j].x - truth[j].x) ** 2;
      }
    }
    assert.ok(Math.sqrt(filteredError / rawError) < .62, 'stationary RMS drops at least 38%');
    smoother.reset();
    let residual = 0, count = 0;
    for (let i = 0; i < rate * 3; i++) {
      const x = .2 + .3 * i / rate;
      const actual = smoother.update([hand(x)], i * 1000 / rate, ['Left'])[0][0].x;
      if (i > rate) { residual += x - actual; count++; }
    }
    assert.ok(residual / count / .3 < .015, 'steady motion adds less than 15 ms lag');
    smoother.reset();
    smoother.update([hand()], 0, ['Left']);
    const first = smoother.update([hand(.65)], 1000 / rate, ['Left'])[0][0].x;
    assert.ok((first - .5) / .15 > .75, 'large motion follows at least 75% on its first sample');
    assert.ok(first <= .65, 'no prediction or overshoot');
  });
}

test('small deliberate motion still moves; irregular timing and reversals never overshoot', () => {
  const smoother = new OverlaySmoothing();
  smoother.update([hand()], 0, ['Left']);
  let last;
  for (let i = 1; i <= 120; i++) last = smoother.update([hand(.502)], i * 1000 / 60, ['Left']);
  assert.ok(last[0][0].x > .50199, 'no dead zone that freezes small movements');
  const rng = random(41);
  let at = 2000, previous = last[0];
  for (let i = 0; i < 100; i++) {
    at += 5 + 70 * rng();
    const raw = hand(.4 + rng() * .2, .4 + rng() * .2);
    const actual = smoother.update([raw], at, ['Left'])[0];
    for (let j = 0; j < 21; j++) for (const axis of ['x', 'y']) {
      assert.ok(actual[j][axis] >= Math.min(raw[j][axis], previous[j][axis]) - 1e-14);
      assert.ok(actual[j][axis] <= Math.max(raw[j][axis], previous[j][axis]) + 1e-14);
    }
    previous = actual;
  }
});

test('two-hand result reordering preserves identity; missing or ambiguous hands discard history', () => {
  const smoother = new OverlaySmoothing(), reference = new OverlaySmoothing();
  smoother.update([hand(.2), hand(.7)], 0, ['Left', 'Right']);
  reference.update([hand(.2), hand(.7)], 0, ['Left', 'Right']);
  const ordered = reference.update([hand(.22), hand(.68)], 16, ['Left', 'Right']);
  assert.deepEqual(smoother.update([hand(.68), hand(.22)], 16, ['Right', 'Left']), [...ordered].reverse());
  smoother.update([hand(.68)], 32, ['Right']);
  assert.deepEqual(smoother.update([hand(.23), hand(.68)], 48, ['Left', 'Right'])[0], hand(.23));
  assert.deepEqual(smoother.update([hand(.25), hand(.69)], 64, ['Left', 'Left']), [hand(.25), hand(.69)]);
  assert.deepEqual(smoother.update([hand(.3)], 80, ['Left']), [hand(.3)]);
  assert.deepEqual(smoother.update([], 96), []);
  assert.deepEqual(smoother.update([hand(.4)], 112, ['Left']), [hand(.4)]);
});

test('tracking gaps, label changes, large relocations and video aspect changes start fresh', () => {
  for (const [time, label, x, aspect] of [[400, 'Left', .51, 1], [16, 'Right', .51, 1],
    [16, 'Left', .95, 1], [16, 'Left', .51, 16 / 9]]) {
    const smoother = new OverlaySmoothing();
    smoother.update([hand()], 0, ['Left']);
    assert.deepEqual(smoother.update([hand(x)], time, [label], aspect), [hand(x)]);
  }
});

test('raw detections, returned arrays and irrelevant noisy depth cannot corrupt XY state', () => {
  const smoother = new OverlaySmoothing(), reference = new OverlaySmoothing();
  const raw = hand();
  const before = structuredClone(raw);
  smoother.update([raw], 0, ['Left'])[0][0].x = 999;
  reference.update([raw], 0, ['Left']);
  assert.deepEqual(raw, before);
  const actual = smoother.update([hand(.51).map(p => ({...p, z: 999}))], 16, ['Left']);
  const expected = reference.update([hand(.51)], 16, ['Left']);
  assert.deepEqual(actual[0].map(p => [p.x, p.y]), expected[0].map(p => [p.x, p.y]));
  actual[0][0].x = 999;
  assert.deepEqual(smoother.update([hand(.52)], 32, ['Left']), reference.update([hand(.52)], 32, ['Left']));
});

test('invalid hands clear immediately; late frames do not change the displayed pose or history', () => {
  const smoother = new OverlaySmoothing();
  const first = smoother.update([hand()], 10, ['Left']);
  assert.deepEqual(smoother.update([hand(.6)], 10, ['Left']), first);
  assert.deepEqual(smoother.update([hand(.6)], 9, ['Left']), first);
  for (const invalid of [[], hand().slice(1), hand().map(p => ({...p, x: NaN})), null]) {
    smoother.reset();
    smoother.update([hand()], 0, ['Left']);
    assert.deepEqual(smoother.update([invalid], 16, ['Left']), [[]]);
    assert.deepEqual(smoother.update([hand(.51)], 32, ['Left']), [hand(.51)]);
  }
  assert.deepEqual(smoother.update([hand()], Infinity, ['Left']), []);
  assert.deepEqual(smoother.update([hand(.6)], 0, ['Left']), [hand(.6)]);
});

test('actual overlay filters once per observation; redraw and clear do not retain stale state', () => {
  const previousObserver = globalThis.ResizeObserver;
  globalThis.ResizeObserver = class { observe() {} disconnect() {} };
  class TestOverlay extends CameraOverlay { draw() { this.draws = (this.draws || 0) + 1; } }
  try {
    const overlay = new TestOverlay({parentElement: {}}, {videoWidth: 640, videoHeight: 480});
    const raw = hand(.51);
    overlay.setHands([hand()], 0, {timestampMs: 0, labels: ['Left']});
    overlay.setHands([raw], 0, {timestampMs: 16, labels: ['Left']});
    assert.ok(overlay.hands[0][0].x > .5 && overlay.hands[0][0].x < .51);
    assert.equal(raw[0].x, .51);
    const displayed = structuredClone(overlay.hands);
    overlay.draw();
    assert.deepEqual(overlay.hands, displayed);
    overlay.clear();
    assert.deepEqual(overlay.hands, []);
    overlay.setHands([raw], 0, {timestampMs: 0, labels: ['Left']});
    assert.deepEqual(overlay.hands, [raw]);
    overlay.dispose();
    assert.deepEqual(overlay.hands, []);
  } finally { globalThis.ResizeObserver = previousObserver; }
});

test("landmarks follow mirrored preview and vertical/horizontal letterboxing", () => {
  assert.deepEqual(landmarkPixel({ x: 0, y: 0 }, 640, 480, 640, 480), [640, 0]);
  assert.deepEqual(landmarkPixel({ x: 1, y: 1 }, 640, 480, 640, 480), [0, 480]);
  assert.deepEqual(
    landmarkPixel({ x: 0.25, y: 0.5 }, 640, 480, 640, 480),
    [480, 240],
  );
  assert.deepEqual(
    landmarkPixel({ x: 0, y: 0 }, 640, 640, 640, 480),
    [640, 80],
  );
  assert.deepEqual(
    landmarkPixel({ x: 1, y: 1 }, 800, 480, 640, 480),
    [80, 480],
  );
});

test('overlay smoothing can be toggled live without modifying raw detections or retaining old history', () => {
  const previousObserver = globalThis.ResizeObserver;
  globalThis.ResizeObserver = class { observe() {} disconnect() {} };
  class TestOverlay extends CameraOverlay { draw() {} }
  try {
    const overlay = new TestOverlay({parentElement: {}}, {videoWidth: 640, videoHeight: 480});
    overlay.setHands([hand()], 0, {timestampMs: 0, labels: ['Left']});
    const raw = hand(.51);
    overlay.setHands([raw], 0, {timestampMs: 16, labels: ['Left']});
    assert.notDeepEqual(overlay.hands, [raw]);
    overlay.setSmoothingEnabled(false);
    assert.deepEqual(overlay.hands, [raw], 'switch redraws the latest raw detection immediately');
    overlay.hands[0][0].x = 999;
    assert.equal(raw[0].x, .51, 'display cannot mutate raw input');
    overlay.setHands([hand(.52)], 0, {timestampMs: 32, labels: ['Left']});
    assert.deepEqual(overlay.hands, [hand(.52)]);
    overlay.setSmoothingEnabled(true);
    overlay.setHands([hand(.6)], 0, {timestampMs: 48, labels: ['Left']});
    assert.deepEqual(overlay.hands, [hand(.6)], 're-enable starts at the next sample');
    overlay.setHands([hand(.61)], 0, {timestampMs: 64, labels: ['Left']});
    assert.notDeepEqual(overlay.hands, [hand(.61)]);
    overlay.clear();
    overlay.setSmoothingEnabled(false);
    assert.deepEqual(overlay.hands, []);
    overlay.dispose();
  } finally { globalThis.ResizeObserver = previousObserver; }
});
