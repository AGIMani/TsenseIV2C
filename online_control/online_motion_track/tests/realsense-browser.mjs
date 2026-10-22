// Synthetic RGB/depth, test-only detector, actual HTTP fusion/retarget and UI.
// This smoke validates integration; it does not assess native capture accuracy.
import {chromium} from '@playwright/test';
import {spawn} from 'node:child_process';
import {mkdir,access,readFile} from 'node:fs/promises';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
const root=fileURLToPath(new URL('../../../',import.meta.url));
const python=process.env.HAND_PYTHON||root+'.venvs/track/bin/python';
const fixture=spawn(python,['-u','tests/rgbd_browser_server.py'],{cwd:root,stdio:['pipe','pipe','pipe']});
let output='',fixtureData,url;
const states=[];
const started=await new Promise((resolve,reject)=>{
  const deadline=setTimeout(()=>reject(Error('RGB-D fixture startup timed out')),30000);
  fixture.stdout.on('data',chunk=>{
    output+=chunk;
    const rows=output.split('\n');output=rows.pop();
    for(const row of rows) {
      if(row.startsWith('RGBD_FIXTURE_DATA='))fixtureData=JSON.parse(row.slice('RGBD_FIXTURE_DATA='.length));
      if(row.startsWith('RGBD_TEST_URL='))url=row.slice('RGBD_TEST_URL='.length);
      if(row.startsWith('RGBD_STATE='))states.push(JSON.parse(row.slice('RGBD_STATE='.length)));
    }
    if(url&&fixtureData){clearTimeout(deadline);resolve(true);}
  });
  fixture.stderr.on('data',chunk=>process.stderr.write(chunk));
  fixture.once('exit',code=>{clearTimeout(deadline);reject(Error(`RGB-D fixture exited ${code}`));});
});
assert(started);
const executablePath=process.env.CHROME_PATH||chromium.executablePath();
await access(executablePath);
const browser=await chromium.launch({headless:true,executablePath,args:[
  '--use-angle=swiftshader','--enable-unsafe-swiftshader','--use-fake-device-for-media-stream','--use-fake-ui-for-media-stream']});
const page=await browser.newPage({viewport:{width:1440,height:1050}});
page.setDefaultTimeout(20000);
const errors=[],packets=[],responses=[],controlActions=[];
let calibrating=false;
page.on('pageerror',error=>errors.push(error.message));
page.on('request',request=>{
  if(request.url().endsWith('/api/frame'))packets.push({calibrating,packet:request.postDataJSON()});
  if(/\/api\/(a32|a30|v2)\/control$/.test(request.url()))controlActions.push(request.postDataJSON()?.action);
});
page.on('response',async response=>{
  if(response.url().endsWith('/api/frame')&&response.ok()) {
    const body=await response.json().catch(()=>null);if(body)responses.push({calibrating,body});
  }
});
await page.addInitScript(()=>{
  window.smokeRenderOnce=()=>{for(const paint of window.smokeRenderers||[])paint();};
  window.cameraRequests=0;
  const original=navigator.mediaDevices.getUserMedia.bind(navigator.mediaDevices);
  navigator.mediaDevices.getUserMedia=(...args)=>{window.cameraRequests++;return original(...args);};
});
// Suppress only continuous CAD rendering in this headless smoke. Keep actual
// scene construction, updates and rendering for explicit screenshot paints;
// browser requestAnimationFrame and capture freshness remain untouched.
const viewer=await readFile(root+'online_control/online_motion_track/static/viewer.js','utf8');
assert(viewer.includes('this.renderer.setAnimationLoop(() => {'));
await page.route('**/viewer.js',route=>route.fulfill({contentType:'text/javascript',body:viewer.replace(
  'this.renderer.setAnimationLoop(() => {',
  '((paint) => { (window.smokeRenderers ??= []).push(paint); })(() => {')}));
await page.route('**/detector.worker.js',route=>route.fulfill({contentType:'text/javascript',body:`
  const points=${JSON.stringify(fixtureData)};
  self.onmessage=async ({data})=>{
    if(data.type==='init'){self.postMessage({type:'ready',delegate:'CPU'});return;}
    const canvas=new OffscreenCanvas(data.bitmap.width,data.bitmap.height);
    const ctx=canvas.getContext('2d');ctx.drawImage(data.bitmap,0,0);data.bitmap.close();
    const pixel=ctx.getImageData(0,0,1,1).data;
    const id=pixel[0]+pixel[1]*256+pixel[2]*65536;
    const world=points.world.map(p=>({x:p[0],y:p[1],z:p[2]}));
    world[0].x+=id*1e-8;
    await new Promise(resolve=>setTimeout(resolve,65));
    self.postMessage({type:'result',inference_ms:65,result:{worldLandmarks:[world],
      landmarks:[points.normalized.map(p=>({x:p[0],y:p[1],z:p[2]}))],
      handedness:[[{categoryName:'Left',score:.99}]]}});
  };`}));
const ready=()=>page.waitForFunction(()=>!document.querySelector('#start').disabled);
const live=()=>page.waitForFunction(()=>document.querySelector('#tracking-dot').classList.contains('online'));
const state=async()=>{
  const count=states.length;fixture.stdin.write('state\n');
  await new Promise((resolve,reject)=>{const start=Date.now();const poll=()=>{
    if(states.length>count)resolve();else if(Date.now()-start>3000)reject(Error('fixture state timed out'));else setTimeout(poll,20);
  };poll();});return states.at(-1);
};
const stop=async()=>{await page.locator('#start').click();await page.waitForFunction(()=>!document.querySelector('#input-source').disabled);};
try {
  await page.goto(url+'/?source=realsense_rgbd&model=TsenseI_v3_2');
  await page.addStyleTag({content:'#model-stage,#feedback-stage,#skeleton-stage{height:200px!important;min-height:200px!important;}'});await ready();
  console.log('RGBD_UI page ready');
  for(const source of ['mediapipe','realsense_rgbd','leapmotion_camera','leapmotion','webxr','wuji_glove'])
    assert.equal(await page.locator(`#input-source option[value="${source}"]`).count(),1);
  assert.equal(await page.locator('#input-source').inputValue(),'realsense_rgbd');
  assert(await page.locator('#realsense-camera-panel').isVisible());
  assert(!(await page.locator('#camera-control').isVisible()));
  if(await page.locator('#recording').isChecked())await page.locator('label').filter({has:page.locator('#recording')}).click();
  await page.locator('#start').click();console.log('RGBD_UI started');await live();
  await page.waitForFunction(()=>document.querySelector('#realsense-camera-state').textContent.includes('识别'));
  assert.match(await page.locator('#skeleton-status').textContent(),/D455f 深度融合 · 21 点/);
  assert.match(await page.locator('#realsense-fusion-state').textContent(),/21 点有效深度/);
  assert.equal(await page.evaluate(()=>window.cameraRequests),0);
  assert(await page.locator('#leap-camera-canvas').isVisible());
  await mkdir(root+'.work/realsense',{recursive:true});
  await page.evaluate(()=>window.smokeRenderOnce());
  await page.screenshot({path:root+'.work/realsense/tracking.png',fullPage:true});
  fixture.stdin.write('pause\n');
  await page.waitForFunction(()=>!document.querySelector('#tracking-dot').classList.contains('online'));
  assert.match(await page.locator('#realsense-fusion-state').textContent(),/保持姿态/);
  fixture.stdin.write('resume\n');await live();
  await stop();assert.equal((await state()).sessions,0);
  assert.match(await page.locator('#realsense-camera-state').textContent(),/已停止/);
  console.log('RGBD_UI synthetic paired depth tracking / unavailable hold / resume / stop passed');

  await page.locator('#input-source').selectOption('mediapipe');
  assert(await page.locator('#camera-control').isVisible());assert(!(await page.locator('#realsense-camera-panel').isVisible()));
  await page.locator('#start').click();await live();
  assert.equal(await page.evaluate(()=>window.cameraRequests),1);
  assert.equal((await state()).sessions,0,'ordinary webcam must not lease depth capture');
  await stop();
  await page.locator('#input-source').selectOption('realsense_rgbd');
  assert.match(await page.locator('#calibration-link').getAttribute('href'),/source=realsense_rgbd/);
  console.log('RGBD_UI ordinary browser camera preserved');

  calibrating=true;
  await page.goto(url+'/calibration?source=realsense_rgbd');
  await page.addStyleTag({content:'.cal-stage{height:200px!important;min-height:200px!important;}'});await ready();
  assert.equal(await page.locator('#input-source').inputValue(),'realsense_rgbd');
  assert(!(await page.locator('#camera-control').isVisible()));
  await page.locator('#start').click();
  await page.waitForFunction(()=>document.querySelector('#input-status').textContent.includes('21 点有效深度'));
  assert.equal(await page.evaluate(()=>window.cameraRequests),0);
  assert(await page.locator('#leap-camera-canvas').isVisible());
  await page.evaluate(()=>window.smokeRenderOnce());
  await page.screenshot({path:root+'.work/realsense/calibration.png',fullPage:true});
  await page.locator('#start').click();
  await page.waitForFunction(()=>document.querySelector('#input-status').textContent.includes('输入已冻结'));
  assert.equal(await page.locator('#input-x').isDisabled(),false);assert.equal((await state()).sessions,0);
  console.log('RGBD_UI calibration live / freeze passed');

  const paired=packets.filter(({packet})=>packet.source==='realsense_rgbd'&&packet.hand);
  assert(paired.length>=3);assert(paired.some(p=>p.calibrating));
  for(const {packet} of paired) {
    assert.equal(packet.realsense_stream_id,'capture-stream');
    assert(Math.abs(packet.hand.world[0][0]-packet.realsense_frame_id*1e-8)<1e-12,
      'pixels independently encode the exact paired depth frame identity');
  }
  const fused=responses.filter(({body})=>body.depth_fusion?.status==='fused'&&body.tracking);
  assert(fused.length>1);assert(fused.some(s=>s.calibrating));
  assert(fused.some(({body})=>body.fused_world[8][2]<-.01));
  assert(!controlActions.some(action=>['arm','connect','enable'].includes(action)));
  assert.deepEqual(errors,[]);
  console.log(JSON.stringify({passed:true,pairedFrames:paired.length,fusedFrames:fused.length,
    scope:'synthetic RGB pixels and depth, real browser/client/API/fusion/retarget; no physical camera or robot validation'}));
} catch(error) {
  console.error('RGBD_UI_FAILURE',JSON.stringify({message:error.message,errors,packets:packets.length,responses:responses.slice(-2).map(({body})=>({tracking:body.tracking,message:body.message,depth_fusion:body.depth_fusion?.status})),
    notice:await page.locator('#notice').textContent().catch(()=>null),
    input:await page.locator(calibrating?'#input-status':'#realsense-camera-state').textContent().catch(()=>null)}));
  throw error;
} finally {
  await browser.close();fixture.stdin.write('quit\n');
  const kill=setTimeout(()=>fixture.kill(),5000);fixture.once('exit',()=>clearTimeout(kill));
}
