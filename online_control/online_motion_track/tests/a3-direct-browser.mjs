// Production UI/session lifecycle with synthetic API responses and a lightweight
// renderer. No camera, glove, serial device or native SDK is opened.
import {chromium} from '@playwright/test';
import {createServer} from 'node:http';
import {readFile} from 'node:fs/promises';
import assert from 'node:assert/strict';

const models=['TsenseI_v3_0','TsenseI_v3_0_withid','TsenseI_v3_2','TsenseI_v3_2_withid'];
const root=new URL('../../../',import.meta.url), sessions=new Map(), calls=[];
const joint='left_hand_index_PIP_joint';
let available=true, sequence=0;
const pose=q=>({[joint]:q});
function hardware(s){return {model_id:s.model_id,hand_side:'left',direct_mode:s.direct_mode,
  ruckig_enabled:!s.direct_mode,collision_protection_enabled:!s.direct_mode&&s.collision_mode!=='off',
  collision_mode:s.collision_mode,armed:!!s.armed,connected:!!s.connected,
  command_joint_angles_rad:pose(0),ruckig_joint_angles_rad:pose(0),
  linear_strokes_mm:{},rotary_positions_deg:{},target_ready:true,sample_ticks:0,
  local_send_hz:0,server_time:performance.now()/1000,browser_epoch:1,tracking_transition:{phase:'tracking'}};}
const viewer=`export class HandViewer {
  constructor(){this.pose={};this.poses=[];this.robot={root:{visible:true}};}
  async load(){this.pose={};this.poses=[];return {initial_pose:{${joint}:0}};}
  setPose(p){this.pose=p;this.poses.push(p);} dispose(){} resetView(){}
} export class SkeletonViewer {setPoints(){} dispose(){} resetView(){}}`;
const server=createServer(async(req,res)=>{
  try {
    const pathname=new URL(req.url,'http://fixture').pathname;
    if(pathname.startsWith('/api/')){
      let text='';for await(const chunk of req)text+=chunk;
      const body=text?JSON.parse(text):{};calls.push({path:pathname,...body});
      const s=sessions.get(body.session);let result;
      if(pathname==='/api/health')result={models,a30_control_available:available,a32_control_available:available,
        a30_collision_modes:['off','monitor','stop','mink'],a32_collision_modes:Object.fromEntries(models.map(m=>[m,['off','stop','mink']]))};
      else if(pathname==='/api/session'){
        const id=`s${sessions.size+1}`;const entry={...body,session:id};sessions.set(id,entry);
        result={...entry,pose:pose(0),init_pose:pose(0),joint_limits:{[joint]:[0,2]},hardware:available?hardware(entry):null};
      }else if(pathname==='/api/webxr/create')result={entry_path:'/quest',pairing_code:'SYNTHETIC',combined_frame:true};
      else if(pathname==='/api/webxr/latest'){
        const q=(++sequence%5)/5+.1;
        result={connected:true,revision:sequence,age_ms:0,source:'webxr',hand:{label:'Left',world:Array.from({length:21},()=>[0,0,0])},
          update:{sequence,tracking:true,solve_ms:1,direct_mode:s.direct_mode,pose:pose(q),retarget_joint_angles_rad:pose(q),
            command_joint_angles_rad:pose(0),hardware:available?hardware(s):null}};
      }else if(pathname.endsWith('/control')){
        if(body.action==='disconnect'){s.connected=false;s.armed=false;}
        if(body.action==='arm')s.armed=!!body.enabled;
        result=hardware(s);
      }else if(pathname==='/api/stop'){s.stopped=true;s.armed=false;result={ok:true};}
      else if(pathname==='/api/feedback-pose')result={valid:false,missing:[joint]};
      else result={};
      res.setHeader('Content-Type','application/json');res.end(JSON.stringify(result));return;
    }
    if(pathname==='/viewer.js'){res.setHeader('Content-Type','text/javascript');res.end(viewer);return;}
    const path=pathname==='/'?'online_control/online_motion_track/static/index.html'
      :pathname.startsWith('/shared/')?'online_control'+pathname
      :'online_control/online_motion_track/static'+pathname;
    let content=await readFile(new URL(path,root));
    if(pathname==='/app.js')content=Buffer.from(content.toString()+`
      window.directTest={get viewer(){return viewer;},get session(){return session;},
        get active(){return active;},get control(){return localControl;},sdk:a32Panel};`);
    res.setHeader('Content-Type',pathname.endsWith('.js')?'text/javascript':pathname.endsWith('.css')?'text/css':pathname.endsWith('.png')?'image/png':'text/html');
    res.end(content);
  }catch(error){res.writeHead(500);res.end(error.message);}
});
await new Promise(resolve=>server.listen(0,'127.0.0.1',resolve));
const browser=await chromium.launch({headless:true,...(process.env.CHROME_PATH?{executablePath:process.env.CHROME_PATH}:{})});
const errors=[];let cases=0,latestPage;
try{
  for(const hasControl of [true,false])for(const model of models){
    available=hasControl;
    const page=await browser.newPage();latestPage=page;page.setDefaultTimeout(10000);page.on('pageerror',error=>errors.push(error.message));
    await page.goto(`http://127.0.0.1:${server.address().port}/?model=${model}&source=webxr`);
    await page.waitForFunction(()=>window.directTest&&!document.querySelector('#direct-preview').disabled);
    assert.match(await page.locator('#direct-preview-note').textContent(),hasControl?/Ruckig 限速/:/预览模式/);
    await page.locator('label:has(#recording)').click();
    await page.locator('label:has(#direct-preview)').click();
    await page.waitForFunction(()=>!document.querySelector('#direct-preview').disabled);
    assert.equal(await page.locator('#collision-mode').inputValue(),'off');
    assert.equal(await page.locator('#collision-mode').isDisabled(),true);
    assert.match(await page.locator('#direct-preview-note').textContent(),/额外关节平滑已关闭/);
    await page.locator('#start').click();
    await page.waitForFunction(()=>directTest.active&&directTest.viewer.poses.length>=4);
    let id=await page.evaluate(()=>directTest.session.session);
    assert.equal(sessions.get(id).direct_mode,true);assert.equal(sessions.get(id).collision_mode,'off');
    assert(await page.evaluate(()=>new Set(directTest.viewer.poses.map(p=>p.left_hand_index_PIP_joint)).size>=3),
      `${model}: unarmed direct preview must move despite a stationary command pose`);
    // Simulate an armed native session only in fixture state. No transport is opened.
    if(hasControl&&model.includes('v3_2')){
      sessions.get(id).connected=true;sessions.get(id).armed=true;
      await page.waitForFunction(()=>directTest.sdk.state.armed===true);
    }
    const before=calls.length;
    await page.locator('label:has(#direct-preview)').click();
    await page.waitForFunction(()=>!document.querySelector('#direct-preview').disabled&&directTest.session===null);
    assert.equal(await page.evaluate(()=>directTest.active),false);
    assert.equal(sessions.get(id).stopped,true);
    if(hasControl&&model.includes('v3_2'))assert(calls.slice(before).some(c=>c.action==='disconnect'));
    assert(!calls.slice(before).some(c=>c.action==='arm'&&c.enabled),'mode switch must never arm');
    assert.equal(await page.locator('#collision-mode').isDisabled(),false);
    assert.equal(await page.locator('#collision-mode').inputValue(),'off');
    await page.locator('#collision-mode').selectOption('stop');
    await page.locator('#start').click();
    await page.waitForFunction(()=>directTest.active&&directTest.session!==null);
    id=await page.evaluate(()=>directTest.session.session);
    assert.equal(sessions.get(id).direct_mode,false);assert.equal(sessions.get(id).collision_mode,'stop');
    await page.locator('label:has(#direct-preview)').click();
    await page.waitForFunction(()=>!document.querySelector('#direct-preview').disabled&&directTest.session===null);
    assert.equal(sessions.get(id).stopped,true);assert.equal(await page.locator('#collision-mode').inputValue(),'off');
    await page.close();cases++;
  }
  assert.deepEqual(errors,[]);
  console.log(`PASS ${cases} A3 direct-mode UI cases (four models, control/preview-only): start, moving unarmed preview, mode teardown and collision selection; synthetic APIs only.`);
}catch(error){console.error({errors,notice:await latestPage?.locator('#notice').textContent(),calls:calls.slice(-5)});throw error;}
finally{await browser.close();await new Promise(resolve=>server.close(resolve));}
