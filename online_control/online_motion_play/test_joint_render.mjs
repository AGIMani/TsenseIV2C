// Exercise the actual inline rendering path: syntax checks cannot catch
// block-scoped variables used during state polling and model replacement.
import test from 'node:test';
import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
import vm from 'node:vm';

const html = await readFile(new URL('./index.html', import.meta.url), 'utf8');
const source = html.slice(html.indexOf('function control(spec,'), html.indexOf('async function poll()'));

function harness(script = source) {
  const elements = new Map(), loaded = [];
  class Element {
    constructor() { this.children = []; this.dataset = {}; this.style = {}; this.attributes = {}; this.classList = {remove() {}, toggle() {}}; }
    set id(id) { elements.set(id, this); }
    append(...children) { this.children.push(...children); }
    replaceChildren(...children) { this.children = children; }
    setAttribute(name, value) { this.attributes[name] = value; }
  }
  const $ = id => {
    if (!elements.has(id) && !id.startsWith('finger-')) elements.set(id, new Element());
    return elements.get(id);
  };
  const context = vm.createContext({
    $, document: {getElementById:$, createElement:() => new Element(), body:new Element()},
    localStorage:{getItem:() => null, setItem() {}},
    setInterval() {}, bindings:new Map(), pendingJoints:new Map(),
    state:null, ownsControl:false, viewerReady:true,
    loadingSide:'', failedModelSide:'', modelSide:'',
    loadModel:url => loaded.push(url), applyPose() {}, sendWidget() {},
    localSerial:{sampleMetrics:() => ({})}, serialSupport:() => '',
    isA30Model:model => model.startsWith('TsenseI_v3_0'),
    isA32Model:model => model.startsWith('TsenseI_v3_2'),
  });
  vm.runInContext(script + '\nglobalThis.ui = {render, setJointDisplayMode, jointRows};', context);
  return {ui:context.ui, $, loaded};
}

function snapshot(model, side) {
  const count = model === 'TsenseI_v2' ? 21 : 20;
  const joints = Array.from({length:count}, (_, i) => ({
    id:`${model}:${side}:${i}`, name:`${side}_hand_index_joint${i}`, short:`index_joint${i}`,
    kind:'number', enabled:true, min:0, max:1, step:.001, value:.5,
    downlink:{min:0, max:10, unit:'mm', slot:`self ${i}`, at_ui_min:0, at_ui_max:10, curve:[[0,0],[.5,3],[1,10]]},
  }));
  return {model_id:model, side, joints, urdf:`/${model}/${side}.urdf`, pose:{},
    controls:{}, control:{owned:true, available:false}, connected:false,
    device:{report:[]}, dialogs:[], count:0, file:'test.json'};
}

test('model/side changes and repeated polls render every joint in all three units', () => {
  const h = harness();
  for (const model of ['TsenseI_v2', 'TsenseI_v3_0', 'TsenseI_v3_0_withid', 'TsenseI_v3_2', 'TsenseI_v3_2_withid']) {
    for (const side of ['left', 'right']) {
      const state = snapshot(model, side);
      h.ui.render(state);
      assert.equal(h.$('modelSelect').disabled, false);
      assert.equal(h.$('modelSelect').value, model);
      assert.equal(h.loaded.at(-1), state.urdf);
      assert.equal(h.ui.jointRows.size, state.joints.length);
      for (const [mode, expected] of [['rad', .5], ['deg', 28.6479], ['stroke', 3]]) {
        h.ui.setJointDisplayMode(mode);
        h.ui.render(state); // Existing rows must work on every subsequent poll.
        for (const row of h.ui.jointRows.values()) {
          assert.equal(Number(row.binding.el.value), expected);
          assert.ok(row.binding.el.attributes['aria-label'].includes('食指'));
        }
      }
    }
  }
});

test('model selector keeps the connected/read-only restrictions', () => {
  const h = harness(), state = snapshot('TsenseI_v3_2_withid', 'right');
  h.ui.render({...state, connected:true});
  assert.equal(h.$('modelSelect').disabled, true);
  h.ui.render({...state, control:{owned:false, available:true}});
  assert.equal(h.$('modelSelect').disabled, true);
  h.ui.render(state);
  assert.equal(h.$('modelSelect').disabled, false);
});

test('regression fixture reproduces the previous rendering failure', () => {
  const broken = source.replace("    const [finger, ...parts] = spec.short.split('_');\n    const part = parts.join('_');\n", '')
    .replace("      let group = $('finger-' + finger);", "      const [finger, ...parts] = spec.short.split('_'); const part = parts.join('_'); let group = $('finger-' + finger);");
  assert.throws(() => harness(broken).ui.render(snapshot('TsenseI_v2', 'left')), /finger is not defined/);
});
