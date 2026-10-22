// Deterministic regressions for the inline viewer, without a GPU or hardware.
import test from 'node:test';
import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
import vm from 'node:vm';

const html = await readFile(new URL('./index.html', import.meta.url), 'utf8');
const viewer = html.slice(html.indexOf('let THREE, renderer,'), html.indexOf("$('resetView').onclick"));
function harness() {
  const timers = new Map(), frames = new Map(), note = {textContent:''};
  let clock = 0, nextID = 0, draws = 0, moving = false, renderCost = 0;
  const renderer = {domElement:{dataset:{}}, render() { draws++; clock += renderCost; }};
  const context = vm.createContext({
    document:{hidden:false}, performance:{now:() => clock},
    setTimeout(callback, delay) { const id = ++nextID; timers.set(id,{callback, delay}); return id; },
    clearTimeout(id) { timers.delete(id); },
    requestAnimationFrame(callback) { const id = ++nextID; frames.set(id,callback); return id; },
    cancelAnimationFrame(id) { frames.delete(id); },
    AbortController, fetch:() => Promise.reject(Error('Unexpected fetch')), $:() => note,
    fakeRenderer:renderer, fakeOrbit:{update:() => moving},
    primitives:{
      Group:class { traverse() {} },
      Quaternion:class { setFromAxisAngle() { return this; } },
    },
  });
  vm.runInContext(viewer + `
    THREE = primitives; renderer = fakeRenderer; orbit = fakeOrbit; viewerReady = true;
    globalThis.api = {invalidateViewer, cancelViewerFrame, applyPose, loadModel,
      setJoint(joint) { jointNodes.set('joint', joint); },
      display(url) { modelSide = url; },
      state() { return {modelSide, loadingSide, failedModelSide, modelGeneration}; }};
  `, context);
  return {context, api:context.api, note, renderer, timers, frames,
    get draws() { return draws; }, set moving(value) { moving = value; },
    set renderCost(value) { renderCost = value; },
    runTimer() { const [id,{callback,delay}] = timers.entries().next().value; timers.delete(id); clock += delay; callback(); return delay; },
    runFrame() { const [id,callback] = frames.entries().next().value; frames.delete(id); callback(); },
  };
}

test('unchanged polled poses do not continuously render a static model', () => {
  const h = harness();
  const quaternion = {copy() { return this; }, multiply() { return this; }};
  h.api.setJoint({node:{quaternion},origin:{},axis:{}});
  h.api.applyPose({joint:.1}); h.api.applyPose({joint:.1});
  assert.equal(h.timers.size, 1);
  h.runTimer(); h.runFrame();
  for (let i = 0; i < 100; i++) h.api.applyPose({joint:.1});
  assert.equal(h.draws, 1); assert.equal(h.timers.size, 0);
  h.api.applyPose({joint:.2}); h.api.applyPose({joint:.3});
  assert.equal(h.timers.size, 1);
  h.runTimer(); h.runFrame(); assert.equal(h.draws, 2);
});

test('orbit damping continues, yields between costly frames, then goes idle', () => {
  const h = harness(); h.moving = true; h.renderCost = 100;
  h.api.display('/left.urdf'); h.api.invalidateViewer(); h.api.invalidateViewer();
  assert.equal(h.timers.size, 1);
  h.runTimer(); h.runFrame();
  assert.equal(h.renderer.domElement.dataset.modelUrl, '/left.urdf');
  assert.equal(h.timers.size, 1);
  assert.ok(h.runTimer() >= 100, 'slow frames must leave a turn for controls and polling');
  h.moving = false; h.runFrame();
  assert.equal(h.draws, 2); assert.equal(h.timers.size, 0);
});

test('a hidden page cancels pending frames and can resume rendering', () => {
  const h = harness(); h.api.invalidateViewer(); h.runTimer();
  h.context.document.hidden = true; h.api.cancelViewerFrame(); h.api.invalidateViewer();
  assert.equal(h.timers.size, 0); assert.equal(h.frames.size, 0);
  h.context.document.hidden = false; h.api.invalidateViewer();
  h.runTimer(); h.runFrame(); assert.equal(h.draws, 1);
});

test('returning to the displayed hand aborts its pending replacement', async () => {
  const h = harness(); h.api.display('/left.urdf');
  let signal;
  h.context.fetch = (_url, options) => new Promise((_resolve, reject) => {
    signal = options.signal; signal.addEventListener('abort', () => reject(signal.reason));
  });
  const pending = h.api.loadModel('/right.urdf');
  assert.equal(h.api.state().loadingSide, '/right.urdf');
  await h.api.loadModel('/left.urdf'); await pending;
  assert.equal(signal.aborted, true);
  assert.equal(h.api.state().modelSide, '/left.urdf');
  assert.equal(h.api.state().loadingSide, ''); assert.equal(h.note.textContent, '');
});

test('a stale request failure cannot replace the latest load status', async () => {
  const h = harness(); const rejectors = [];
  h.context.fetch = () => new Promise((_resolve, reject) => rejectors.push(reject));
  const oldLoad = h.api.loadModel('/old.urdf'), newLoad = h.api.loadModel('/new.urdf');
  rejectors[0](Error('old failure')); await oldLoad;
  assert.equal(h.api.state().loadingSide, '/new.urdf');
  assert.equal(h.note.textContent, '正在载入手部模型…');
  rejectors[1](Error('new failure')); await newLoad;
  assert.equal(h.api.state().loadingSide, '');
  assert.equal(h.api.state().failedModelSide, '/new.urdf');
  assert.equal(h.note.textContent, '模型加载失败：new failure');
});
