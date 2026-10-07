import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import vm from 'node:vm';
const app=readFileSync(new URL('../static/app.js',import.meta.url),'utf8');
const source=app.slice(app.indexOf('function showDownload('),app.indexOf('function gpuBadge('));
test('download UI exposes every part and the selected part filename',()=>{
  const nodes=new Map(),revoked=[];
  const get=id=>{if(!nodes.has(id))nodes.set(id,{hidden:true,clicks:0,click(){this.clicks++;},replaceChildren(...children){this.children=children;}});return nodes.get(id);};
  const context=vm.createContext({$:get,lastDownloadUrl:null,downloadRecordingId:null,
    URL:{revokeObjectURL:url=>revoked.push(url)},Option:class{constructor(text,value){Object.assign(this,{text,value});}}});
  vm.runInContext(source,context);
  context.file={id:'capture',partIndex:2,partCount:5,name:'capture_part003.jsonl',url:'blob:third',incomplete:false};
  vm.runInContext('showDownload(file,true)',context);
  assert.equal(context.downloadRecordingId,'capture');
  assert.equal(get('recording-part').children.length,5);assert.equal(get('recording-part').value,'2');
  assert.equal(get('download-local').download,'capture_part003.jsonl');assert.equal(get('download-local').clicks,1);
  assert.equal(get('download-recording-part').hidden,false);assert.match(get('record-status').textContent,/共 5 段/);
  context.file={id:'small',name:'small.jsonl',url:'blob:small',partCount:1};vm.runInContext('showDownload(file)',context);
  assert.equal(get('recording-part').hidden,true);assert.equal(get('download-recording-part').hidden,true);
  assert.deepEqual(revoked,['blob:third']);
});
