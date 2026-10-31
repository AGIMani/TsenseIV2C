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
    constructor() { this.children = []; this.dataset = {}; this.style = {}; this.attributes = {}; this.classes = new Set(); this.classList = {remove: name => this.classes.delete(name), toggle: (name, enabled) => enabled ? this.classes.add(name) : this.classes.delete(name)}; }
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
  vm.runInContext(script + '\nglobalThis.ui = {render, setJointDisplayMode, jointRows, markJointTelemetryMissing};', context);
  return {ui:context.ui, $, loaded, document:context.document};
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

test('A3.2 compensation checkbox renders server state and respects control ownership', () => {
  const actionSection = html.slice(html.indexOf('<h2>动作与序列</h2>'), html.indexOf('id="a32Motors"'));
  assert.ok(actionSection.includes('id="a32EnableMotorCompensation"'));
  const h = harness(), state = snapshot('TsenseI_v3_2', 'left');
  state.controls.a32EnableMotorCompensation = {
    id:'left:a32EnableMotorCompensation', kind:'check', enabled:true,
    text:'增加系数补偿', value:true,
  };
  h.ui.render(state);
  const label = h.$('a32EnableMotorCompensation').children[0];
  const input = label.children[0];
  assert.equal(input.type, 'checkbox');
  assert.equal(input.checked, true);
  assert.equal(label.children[1].textContent, '增加系数补偿');
  state.controls.a32EnableMotorCompensation.value = false;
  h.ui.render(state);
  assert.equal(input.checked, false);
  h.ui.render({...state, control:{owned:false, available:true}});
  assert.equal(input.disabled, true);
  assert.equal(h.$('a32EnableMotorCompensation').hidden, false);
  h.ui.render(snapshot('TsenseI_v2', 'left'));
  assert.equal(h.$('a32EnableMotorCompensation').hidden, true);
});


test('live numbers follow measured feedback in every unit and flag missing/stale channels', () => {
  const h = harness(), state = snapshot('TsenseI_v3_2', 'left');
  state.controls.realtimeJointControl = {id:'live', kind:'check', enabled:true, value:true};
  const joint = state.joints[0];
  joint.feedback = {rad:.2, stroke:-4.2};
  h.ui.render(state);
  const row = h.ui.jointRows.get(joint.name), input = row.binding.el;
  assert.equal(Number(input.value), .2);
  assert.equal(Number(row.slider.value), .5, 'feedback never becomes the command target');
  assert.equal(input.classes.has('telemetry-missing'), false);
  assert.equal(h.ui.jointRows.get(state.joints[1].name).binding.el.classes.has('telemetry-missing'), true);
  h.document.activeElement = input;
  joint.feedback = {rad:.3, stroke:-5};
  h.ui.render(state);
  assert.equal(Number(input.value), .3, 'untouched focus must not freeze feedback');
  for (const [mode, expected] of [['deg',17.1887], ['stroke',-5], ['rad',.3]]) {
    h.ui.setJointDisplayMode(mode);
    assert.equal(Number(input.value), expected);
  }
  joint.feedback = null;
  h.ui.render(state);
  assert.equal(Number(input.value), .3, 'keep the last measured number when telemetry is lost');
  assert.equal(input.classes.has('telemetry-missing'), true);
  joint.feedback = {rad:.4, stroke:-6};
  h.ui.render(state);
  assert.equal(Number(input.value), .4);
  assert.equal(input.classes.has('telemetry-missing'), false);
  h.ui.markJointTelemetryMissing();
  assert.equal(input.classes.has('telemetry-missing'), true);
  input.value = '.7'; input.oninput();
  h.ui.render(state);
  assert.equal(input.value, '.7', 'active manual input survives a telemetry poll');
  row.binding.dirty = false;
  h.document.activeElement = null;
  state.controls.realtimeJointControl.value = false;
  h.ui.render(state);
  assert.equal(Number(input.value), .5);
  assert.equal(input.classes.has('telemetry-missing'), false);
});
