// Real page and camera overlay with a fake webcam, detector and retarget API.
// No physical camera, SDK, serial port or robotic hand is opened.
import {chromium} from '@playwright/test';
import {createServer} from 'node:http';
import {readFile} from 'node:fs/promises';
import assert from 'node:assert/strict';

const root = new URL('../../../', import.meta.url), frames = [], errors = [];
const joint = 'left_hand_index_PIP_joint';
const pose = {[joint]: 0};
const viewer = `export class HandViewer {
  constructor(){this.robot={root:{visible:true}};}
  async load(){return {initial_pose:${JSON.stringify(pose)}};}
  setPose(){} dispose(){} resetView(){}
} export class SkeletonViewer {setPoints(){} dispose(){} resetView(){}}`;
const worker = `let n=0; self.onmessage=({data})=>{
  if(data.type==='init'){self.postMessage({type:'ready',delegate:'CPU'});return;}
  data.bitmap.close();
  const x=.4+(++n%10)*.001;
  const hand=Array.from({length:21},(_,i)=>({x:x+i*.005,y:.4+i*.005,z:0}));
  self.postMessage({type:'result',result:{worldLandmarks:[hand],landmarks:[hand],
    handedness:[[{categoryName:'Left',score:.99}]]},inference_ms:1});
};`;
const server = createServer(async(req,res)=>{
  try {
    const pathname = new URL(req.url,'http://fixture').pathname;
    if (pathname.startsWith('/api/')) {
      let text=''; for await(const chunk of req) text+=chunk;
      const body=text?JSON.parse(text):{};
      let result={};
      if(pathname==='/api/health')result={models:['TsenseI_v3_2'],a32_control_available:false,
        a32_collision_modes:{TsenseI_v3_2:['off']}};
      if(pathname==='/api/session')result={...body,session:'fixture',pose,actuation:{}};
      if(pathname==='/api/frame') {
        frames.push(body);
        result={sequence:body.sequence,tracking:!!body.hand,pose,
          mediapipe_filter_enabled:body.mediapipe_filter_enabled,solve_ms:1};
      }
      res.setHeader('Content-Type','application/json');res.end(JSON.stringify(result));return;
    }
    if(pathname==='/viewer.js'||pathname==='/detector.worker.js') {
      res.setHeader('Content-Type','text/javascript');res.end(pathname==='/viewer.js'?viewer:worker);return;
    }
    const path=pathname==='/'?'online_control/online_motion_track/static/index.html'
      :pathname.startsWith('/shared/')?'online_control'+pathname
      :'online_control/online_motion_track/static'+pathname;
    let content=await readFile(new URL(path,root));
    if(pathname==='/app.js')content=Buffer.from(content.toString()+`
      window.smoothingTest={overlay,get active(){return active;},get session(){return session;}};`);
    res.setHeader('Content-Type',pathname.endsWith('.js')?'text/javascript':pathname.endsWith('.css')?'text/css':pathname.endsWith('.png')?'image/png':'text/html');
    res.end(content);
  } catch(error) {res.writeHead(500);res.end(error.message);}
});
await new Promise(resolve=>server.listen(0,'127.0.0.1',resolve));
const browser=await chromium.launch({headless:true,
  ...(process.env.CHROME_PATH?{executablePath:process.env.CHROME_PATH}:{}),
  args:['--use-fake-ui-for-media-stream','--use-fake-device-for-media-stream']});
try {
  const page=await browser.newPage({viewport:{width:1440,height:1060}});
  page.on('pageerror',error=>errors.push(error.message));
  await page.goto(`http://127.0.0.1:${server.address().port}`);
  await page.waitForFunction(()=>window.smoothingTest&&!document.querySelector('#start').disabled);
  for(const id of ['overlay-smoothing','mediapipe-filter']) {
    assert.equal(await page.locator('#'+id).isChecked(),true);
    assert.equal(await page.locator('#'+id+'-control').isVisible(),true);
  }
  // Show only relevant controls for each source, preserving their choices.
  await page.locator('#input-source').selectOption('webxr');
  assert.equal(await page.locator('#overlay-smoothing-control').isVisible(),false);
  assert.equal(await page.locator('#mediapipe-filter-control').isVisible(),false);
  await page.locator('#input-source').selectOption('realsense_rgbd');
  assert.equal(await page.locator('#overlay-smoothing-control').isVisible(),true);
  assert.equal(await page.locator('#mediapipe-filter-control').isVisible(),false);
  await page.locator('#input-source').selectOption('leapmotion_camera');
  assert.equal(await page.locator('#mediapipe-filter-control').isVisible(),true);
  await page.locator('#input-source').selectOption('mediapipe');
  await page.locator('label:has(#recording)').click();
  await page.locator('#start').click();
  await page.waitForFunction(()=>smoothingTest.active&&smoothingTest.overlay.hands.length===1);
  const session=await page.evaluate(()=>smoothingTest.session.session);
  for(const [overlay,input] of [[true,true],[false,true],[false,false],[true,false],[true,true]]) {
    for(const [id,enabled] of [['overlay-smoothing',overlay],['mediapipe-filter',input]]) {
      if(await page.locator('#'+id).isChecked()!==enabled)await page.locator('label:has(#'+id+')').click();
    }
    const after=frames.length;
    await page.waitForFunction(({overlay})=>smoothingTest.overlay.smoothingEnabled===overlay,{overlay});
    await page.waitForTimeout(120);
    assert(frames.slice(after).some(frame=>frame.mediapipe_filter_enabled===input));
    assert.equal(await page.evaluate(()=>smoothingTest.active),true);
    assert.equal(await page.evaluate(()=>smoothingTest.session.session),session);
    if(!overlay)assert(await page.evaluate(()=>JSON.stringify(smoothingTest.overlay.hands)===JSON.stringify(smoothingTest.overlay.rawHands)));
  }
  await page.setViewportSize({width:390,height:844});
  assert(await page.evaluate(()=>document.documentElement.scrollWidth<=window.innerWidth),'mobile controls fit');
  for(const width of [390,320]) {
    await page.setViewportSize({width,height:844});
    assert(await page.evaluate(()=>['overlay-smoothing-control','mediapipe-filter-control'].every(id=>{
      const label=document.getElementById(id), button=label.querySelector('.switch');
      return button.getBoundingClientRect().width===27 && label.scrollWidth<=label.clientWidth;
    })),'mobile toggle labels and switches fit without shrinking');
  }
  await page.setViewportSize({width:390,height:844});
  if(process.env.SMOOTHING_SCREENSHOT)await page.screenshot({path:process.env.SMOOTHING_SCREENSHOT,fullPage:true});
  await page.locator('#start').click();
  await page.waitForFunction(()=>!smoothingTest.active&&smoothingTest.session===null);
  assert.deepEqual(errors,[]);
  console.log('PASS independent overlay/input toggles: defaults, source visibility, all four live combinations, uninterrupted session and mobile layout.');
} finally {
  await browser.close();await new Promise(resolve=>server.close(resolve));
}
