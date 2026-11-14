// Production page/queue with a fake webcam, delayed detector and fake API.
// No real camera, native SDK or H7 is opened. Python tests cover the real gate.
import {chromium} from '@playwright/test';
import {createServer} from 'node:http';
import {readFile} from 'node:fs/promises';
import assert from 'node:assert/strict';

const root=new URL('../../../',import.meta.url), frames=[], errors=[];
const joint='left_hand_index_PIP_joint', initial={[joint]:0};
let inferenceMs=134;
const viewer=`export class HandViewer {
  constructor(){this.robot={root:{visible:true}};this.poses=[];}
  async load(){return {initial_pose:${JSON.stringify(initial)}};}
  setPose(p){this.poses.push(p);} dispose(){} resetView(){}
} export class SkeletonViewer {
  constructor(){this.frames=[];} setPoints(p){if(p)this.frames.push(p);} dispose(){} resetView(){}
}`;
const server=createServer(async(req,res)=>{
  try {
    const path=new URL(req.url,'http://fixture').pathname;
    if(path.startsWith('/api/')){
      let text='';for await(const chunk of req)text+=chunk;
      const body=text?JSON.parse(text):{};let result={};
      if(path==='/api/health')result={models:['TsenseI_v3_2_withid'],a32_control_available:false,
        a32_collision_modes:{TsenseI_v3_2_withid:['off']}};
      if(path==='/api/session')result={session:'fixture',pose:initial};
      if(path==='/api/frame'){
        frames.push(body);
        const pose={[joint]:(body.sequence%20)*.01}, accepted=body.source_age_ms<400;
        result={sequence:body.sequence,tracking:true,pose,retarget_joint_angles_rad:pose,
          optimized_points:body.hand.world,solve_ms:10,control_accepted:accepted,
          ...(accepted?{}:{control_skip_reason:'stale_camera_frame',
            control_message:'预览已更新 · 观测过期，未更新 H7 目标'})};
      }
      res.setHeader('Content-Type','application/json');res.end(JSON.stringify(result));return;
    }
    if(path==='/viewer.js'||path==='/detector.worker.js'){
      res.setHeader('Content-Type','text/javascript');
      res.end(path==='/viewer.js'?viewer:`let n=0;self.onmessage=({data})=>{
        if(data.type==='init'){self.postMessage({type:'ready',delegate:'GPU'});return;}
        data.bitmap.close();const x=.4+(++n%10)*.002;
        const hand=Array.from({length:21},(_,i)=>({x:x+i*.005,y:.4+i*.005,z:0}));
        setTimeout(()=>self.postMessage({type:'result',inference_ms:${inferenceMs},result:{
          worldLandmarks:[hand],landmarks:[hand],handedness:[[{categoryName:'Left',score:.99}]]}}),${inferenceMs});
      };`);return;
    }
    const file=path==='/'?'online_control/online_motion_track/static/index.html'
      :path.startsWith('/shared/')?'online_control'+path:'online_control/online_motion_track/static'+path;
    let content=await readFile(new URL(file,root));
    if(path==='/app.js')content=Buffer.from(content.toString()+`
      window.latencyTest={get viewer(){return viewer;},get skeleton(){return skeleton;},
        get queue(){return framePipeline;},get active(){return active;},get session(){return session;}};`);
    res.setHeader('Content-Type',path.endsWith('.js')?'text/javascript':path.endsWith('.css')?'text/css':path.endsWith('.png')?'image/png':'text/html');
    res.end(content);
  }catch(error){res.writeHead(500);res.end(error.message);}
});
await new Promise(resolve=>server.listen(0,'127.0.0.1',resolve));
const browser=await chromium.launch({headless:true,
  ...(process.env.CHROME_PATH?{executablePath:process.env.CHROME_PATH}:{}),
  args:['--use-fake-ui-for-media-stream','--use-fake-device-for-media-stream']});
try {
  const page=await browser.newPage();page.on('pageerror',e=>errors.push(e.message));
  await page.goto(`http://127.0.0.1:${server.address().port}`);
  await page.waitForFunction(()=>window.latencyTest&&!document.querySelector('#start').disabled);
  await page.locator('label:has(#recording)').click();
  const summary=[];
  for(const delay of [134,550]){
    inferenceMs=delay;
    const before=frames.length;
    const offsets=await page.evaluate(()=>({poses:latencyTest.viewer.poses.length,points:latencyTest.skeleton.frames.length}));
    await page.locator('#start').click();
    await page.waitForFunction(()=>latencyTest.queue?.stats.processed>=12,null,{timeout:12000});
    const state=await page.evaluate(offsets=>({queue:latencyTest.queue.stats,
      poses:latencyTest.viewer.poses.slice(offsets.poses),points:latencyTest.skeleton.frames.slice(offsets.points),
      message:document.querySelector('#tracking-text').textContent}),offsets);
    assert.equal(state.queue.expired,0);
    assert.ok(new Set(state.poses.map(p=>p[joint])).size>=10);
    assert.ok(state.points.length>=12);
    const received=frames.slice(before);
    assert.ok(received.length>=12);
    assert.ok(received.every(f=>f.source_age_ms>=delay-5));
    if(delay>400)assert.match(state.message,/未更新 H7/);
    summary.push({inference_ms:delay,queue:state.queue});
    await page.locator('#start').click();
    await page.waitForFunction(()=>!latencyTest.active&&latencyTest.session===null);
  }
  assert.deepEqual(errors,[]);
  console.log('PASS delayed inference keeps skeleton/URDF updating; original observation age retained.',JSON.stringify(summary));
}finally{
  await browser.close();await new Promise(resolve=>server.close(resolve));
}
