// Regression checks for delayed HTTP replies and native select repainting.
import test from 'node:test';
import assert from 'node:assert/strict';
import vm from 'node:vm';
import {readFile} from 'node:fs/promises';
const html = await readFile(new URL('./index.html', import.meta.url), 'utf8');
const controlSource = html.slice(html.indexOf('function control('), html.indexOf('const fingerNames'));
const commandSource = html.slice(html.indexOf('let commandRevision'), html.indexOf('function control('));
const pollSource = html.slice(html.indexOf('async function poll()'), html.indexOf('async function refreshPorts()'));

test('unchanged polls leave the native select untouched, changed options still refresh', () => {
  const writes = [];
  const el = new Proxy({disabled:false,title:'',value:'',replaceChildren(...options) { writes.push('options'); this.options = options; }}, {
    set(target, key, value) { writes.push(key); target[key] = value; return true; },
  });
  const context = vm.createContext({bindings:new Map(), ownsControl:true, pendingJoints:new Map(),
    document:{createElement:() => el, activeElement:null}, Option:class { constructor(text,value) {this.text=text;this.value=value;} },
  });
  vm.runInContext(controlSource, context);
  const host = {append() {}};
  const spec = {kind:'select',enabled:true,value:0,options:['动作1','动作2']};
  context.control(spec,host,'choosemotion'); writes.length = 0;
  for (let i=0;i<30;i++) context.control({...spec,options:[...spec.options]},host,'choosemotion');
  assert.deepEqual(writes, []);
  context.control({...spec,value:1,options:['动作1','动作2','动作3']},host,'choosemotion');
  assert.equal(el.value,'1'); assert.equal(el.options.length,3);
});

test('a state response racing a sync/selection command cannot revert the UI', async () => {
  let resolveState, resolveCommand;
  const rendered = [], sent = [];
  const context = vm.createContext({commandTail:Promise.resolve(), authenticated:true, connected:true,
    api(action,body) { sent.push([action,body]); return new Promise(resolve => { if(action === 'state') resolveState=resolve; else resolveCommand=resolve; }); },
    render:s => rendered.push(s), notify:() => {}, setTimeout:() => {},
  });
  vm.runInContext(commandSource + pollSource, context);
  const poll = context.poll();
  const command = context.command('widget',{id:'sync',value:true});
  await Promise.resolve(); resolveCommand({}); await command;
  resolveState({sync:false}); await poll;
  assert.deepEqual(rendered,[]);
  const fresh = context.poll(); resolveState({sync:true}); await fresh;
  assert.deepEqual(rendered,[{sync:true}]);
  assert.equal(sent[1][1].value,true);
});
