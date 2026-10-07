// Production response recording -> Worker/IndexedDB -> JSONL -> offline viewer.
// All fixtures are synthetic; no camera, glove or robot connection is opened.
import {chromium} from '@playwright/test';
import {createServer} from 'node:http';
import {readFile} from 'node:fs/promises';
import assert from 'node:assert/strict';
import * as a30 from '../static/a30-serial-protocol.js';
const root=new URL('../../../',import.meta.url);
const channels=JSON.parse(await readFile(new URL('src/a32_driver/a32_hardware.json',root),'utf8')).channels;
const app=await readFile(new URL('../static/app.js',import.meta.url),'utf8');
const block=app.slice(app.indexOf('function applyFrameUpdate'),app.indexOf('    // Mode switches may race'))+'}';
const server=createServer(async(req,res)=>{
  if(req.url==='/'){res.setHeader('Content-Type','text/html');res.end('<!doctype html><title>Recording matrix</title>');return;}
  const name=req.url.slice(1),viewer=name==='viewer';
  if(!viewer&&!['worker-recorder.js','recording.worker.js','local-recorder.js'].includes(name)){res.writeHead(404).end();return;}
  res.setHeader('Content-Type',viewer?'text/html':'text/javascript');
  res.end(await readFile(viewer?new URL('tools/hand_control_log_viewer.html',root):new URL('../static/'+name,import.meta.url)));
});
await new Promise(resolve=>server.listen(0,'127.0.0.1',resolve));
const browser=await chromium.launch({headless:true,executablePath:process.env.CHROME_PATH});
const capture=await browser.newPage(),viewer=await browser.newPage(),errors=[];
for(const page of [capture,viewer])page.on('pageerror',e=>errors.push(e.message));
const base=`http://127.0.0.1:${server.address().port}`;
try{
  await capture.goto(base);await viewer.goto(base+'/viewer');
  // Viewer is offline-capable; never request the live log catalogue in this test.
  await viewer.route('**/api/**',route=>route.fulfill({status:404,body:'test origin'}));
  let cases=0;
  for(const model of ['TsenseI_v3_0','TsenseI_v3_0_withid','TsenseI_v3_2','TsenseI_v3_2_withid'])
  for(const side of ['left','right'])for(const source of ['mediapipe','webxr','wuji_glove']){
    const is30=model.includes('v3_0');
    const maps=is30?a30.slotMaps(side):Object.fromEntries(['linear','rotary'].map(kind=>[kind,Object.fromEntries(channels.filter(c=>(c.unit==='mm')===(kind==='linear')).map((c,i)=>[c.joint.replace('left_hand_',side+'_hand_'),i]))]));
    const payload=new Uint8Array(484),v=new DataView(payload.buffer);v.setUint32(8,0x3ffff,true);v.setUint16(2,3,true);
    for(let i=0;i<18;i++){const o=12+24*i;payload[o+3]=2;v.setFloat32(o+4,-1,true);v.setFloat32(o+8,2,true);v.setFloat32(o+12,3,true);v.setFloat32(o+16,-1.5,true);v.setInt16(o+22,3500,true);}
    for(let i=0;i<2;i++){const o=444+20*i;payload[o]=5+4*i;payload[o+1]=5;v.setUint32(o+2,0x80000000,true);v.setUint16(o+6,0x8000,true);v.setUint16(o+8,0x8000,true);}
    const sdk={connected:true,hand_side:side,telemetry_age_ms:1,self:[],mit:[]};
    for(const c of channels)sdk[c.channel][c.index]={joint:c.joint.replace('left_hand_',side+'_hand_'),online:true,valid:true,position:1,target:1.5,iq:2,iq_ref:3,temperature_centi:3500};
    const text=await capture.evaluate(async({model,side,source,maps,block,is30,sdk,hex,decoded})=>{
      const {WorkerRecorder}=await import('/worker-recorder.js');
      const r=new WorkerRecorder(e=>{throw e;});
      await r.start({model_id:model,hand_side:side,slot_maps:maps,source:'web/'+source,
        hardware_location:is30?'visitor browser Web Serial':'local Python service / native A3 SDK',sampling:'synthetic matrix capture'});
      const points=Array.from({length:21},(_,i)=>[i/100,i/200,0]);
      const joints=Object.fromEntries(Object.values(maps).flatMap(m=>Object.keys(m).map(n=>[n,.2])));
      const actuation={model_id:model,hand_side:side,valid:true,joint_angles_rad:joints,
        linear_strokes_mm:Object.fromEntries(Object.keys(maps.linear).map(n=>[n,1.5])),
        rotary_positions_deg:Object.fromEntries(Object.keys(maps.rotary).map(n=>[n,2]))};
      const raw=source==='wuji_glove'?{tracked:true,raw_wuji:{positions_xyz:points}}:source==='webxr'
        ?{world_landmarks:[Array.from({length:25},(_,i)=>[i/100,0,0])]}:{world_landmarks:[points],normalized_landmarks:[points]};
      r.record(source+'_raw',{frame_id:7,hand_side:side,...raw});
      const apply=new Function('recorder','$','isA30','isA32Model','performance',block+';return applyFrameUpdate;')
        (r,()=>({value:side}),()=>is30,m=>m?.includes('v3_2'),performance);
      apply({frameId:7,packet:{source,session:r.id},hand:{label:side==='left'?'Left':'Right'},expected:side==='left'?'Left':'Right',timestamp_ms:0,inferredAt:0,requestStartedAt:0},
        {points,tracking:true,pose:joints,retarget_joint_angles_rad:joints,actuation,solve_ms:1});
      if(is30){
        const command=r.record('conversion',{frame_id:7,hand_side:side,joints_hw:joints,linear_mm:Array(9).fill(1.5),rotary_deg:Array(11).fill(2)});
        r.record('downlink',{frame_id:7,command_id:command,hand_side:side,sent:true,linear_mm:Array(9).fill(1.5),rotary_deg:Array(11).fill(2)});
        r.record('h7',{frame_id:7,hand_side:side,sent:true,joints_ui:joints});
        r.record('telemetry',{...decoded,hand_side:side,payload_hex:hex,accepted:true});
      }else{
        r.record('a32_sdk_sample',{hand_side:side,hardware:sdk,sent:true});
        r.record('h7',{frame_id:7,hand_side:side,source:'a32_sdk_successful_write',sent:true,joints_ui:joints,self:Array(18).fill(1.5),mit_pos:[2,2]});
      }
      r.record('control_disarm',{hand_side:side,reason:'matrix_end'});
      await r.stop('matrix_complete',{exportFile:false});
      const file=await r.export(r.id),text=await(await fetch(file.url)).text();
      URL.revokeObjectURL(file.url);await r.remove(r.id);r.worker.terminate();return text;
    },{model,side,source,maps,block,is30,sdk,hex:Buffer.from(payload).toString('hex'),decoded:a30.telemetry(payload)});
    const rows=text.trim().split('\n').map(JSON.parse),end=rows.at(-1);
    assert.equal(end.dropped_events,0);assert(rows.every((r,i)=>r.seq===i+1));
    assert.equal(rows[0].metadata.sampling,'synthetic matrix capture');
    if(!is30)assert.equal(rows[0].metadata.hardware_location,'local Python service / native A3 SDK');
    await viewer.locator('#file').setInputFiles({name:model+'-'+side+'-'+source+'.jsonl',mimeType:'application/x-ndjson',buffer:Buffer.from(text)});
    await viewer.waitForFunction(()=>!document.querySelector('#file').disabled);
    const loaded=await viewer.evaluate(()=>({status:document.querySelector('#status').textContent,side:document.querySelector('#handSide').value,
      rows:Object.fromEntries(['raw','mp','retarget','control','conversion','downlink','telemetry','events'].map(k=>[k,state[k].length])),
      charts:Object.fromEntries(['position','current','temperature'].map(k=>[k,document.querySelector('#'+k+'Range').textContent]))}));
    assert(!loaded.status.includes('读取失败'));assert.equal(loaded.side,side);
    for(const k of ['raw','mp','retarget','control','downlink','telemetry','events'])assert.equal(loaded.rows[k],1,`${model}/${side}/${source}/${k}`);
    assert.equal(loaded.rows.conversion,is30?2:1);
    for(const value of Object.values(loaded.charts))assert.notEqual(value,'无数据');
    cases++;
  }
  for(const filename of process.argv.slice(2)){
    const rows=(await readFile(filename,'utf8')).trim().split(/\r?\n/).map(JSON.parse);
    await viewer.locator('#file').setInputFiles(filename);
    await viewer.waitForFunction(()=>!document.querySelector('#file').disabled);
    const loaded=await viewer.evaluate(()=>({status:document.querySelector('#status').textContent,
      counts:state.counts,total:Object.values(state.counts).reduce((sum,n)=>sum+n,0),
      rows:Object.fromEntries(['retarget','conversion','downlink','telemetry','sdk','sdkEvents'].map(k=>[k,state[k].length]))}));
    assert(!loaded.status.includes('读取失败'),filename+': '+loaded.status);assert.equal(loaded.total,rows.length,filename);
    for(const k of ['retarget','conversion','downlink','telemetry'])assert.equal(loaded.rows[k],rows.filter(r=>r.type===k).length,filename+': '+k);
  }
  console.log(`PASS ${process.argv.length-2} native recorder files loaded in Chromium with matching event counts.`);
  assert.deepEqual(errors,[]);console.log(`PASS ${cases} recording/export/viewer cases: 4 models × 2 sides × 3 input sources; synthetic hardware.`);
}finally{await browser.close();await new Promise(resolve=>server.close(resolve));}
