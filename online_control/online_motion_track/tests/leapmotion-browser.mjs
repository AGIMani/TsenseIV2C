// Synthetic trusted Leap observations -> actual server/solver -> browser UI.
// This does not validate LeapC, its service, USB hardware or physical accuracy.
import {chromium} from '@playwright/test';
import {spawn} from 'node:child_process';
import {mkdir} from 'node:fs/promises';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';

const root=fileURLToPath(new URL('../../../',import.meta.url));
const fixture=spawn(process.env.HAND_PYTHON||'python', ['-u','tests/run_leapmotion_browser_fixture.py'],
  {cwd:root,stdio:['pipe','pipe','pipe']});
const url=await new Promise((resolve,reject)=>{
  let text='';const deadline=setTimeout(()=>{fixture.kill();reject(Error('Leap fixture timed out'));},30000);
  fixture.stdout.on('data',chunk=>{text+=chunk;const match=text.match(/LEAPMOTION_TEST_URL=(http:\/\/\S+)/);
    if(match){clearTimeout(deadline);resolve(match[1]);}});
  fixture.stderr.on('data',chunk=>process.stderr.write(chunk));
  fixture.once('exit',code=>{clearTimeout(deadline);reject(Error(`Leap fixture exited ${code}: ${text}`));});
});
const browser=await chromium.launch({headless:true,...(process.env.CHROME_PATH?{executablePath:process.env.CHROME_PATH}:{}),
  args:['--use-angle=swiftshader','--enable-unsafe-swiftshader']});
const page=await browser.newPage({viewport:{width:1100,height:850}});
page.setDefaultTimeout(30000);
const errors=[],results=[],calibrationRequests=[],controlActions=[];
let legacyTrackingFrames=0,calibrating=false;
page.on('pageerror',error=>errors.push(error.message));
page.on('request',request=>{
  if(request.url().endsWith('/api/frame')) {
    if(calibrating)calibrationRequests.push(request.postDataJSON());else legacyTrackingFrames++;
  }
  if(/\/api\/(a32|a30|v2)\/control$/.test(request.url()))controlActions.push(request.postDataJSON()?.action);
});
page.on('response',async response=>{
  if(response.url().endsWith('/api/leapmotion/frame')&&response.ok()) {
    const snapshot=await response.json().catch(()=>null);if(snapshot?.update)results.push(snapshot);
  }
});
await page.addInitScript(()=>{
  window.cameraRequests=0;
  navigator.mediaDevices.getUserMedia=()=>{window.cameraRequests++;throw Error('Leap must not request the camera');};
});
const running=()=>page.waitForFunction(()=>document.querySelector('#tracking-dot').classList.contains('online'));
const ready=()=>page.waitForFunction(()=>!document.querySelector('#start').disabled);
async function stop() {
  await page.locator('#start').click();await page.waitForFunction(()=>!document.querySelector('#input-source').disabled);
}
try {
  await page.goto(url+'/?source=leapmotion');
  await page.addStyleTag({content:'#model-stage,#feedback-stage,#skeleton-stage{height:220px!important;min-height:220px!important;}'});
  await ready();
  for (const source of ['mediapipe','webxr','wuji_glove','leapmotion','leapmotion_camera']) {
    assert.equal(await page.locator(`#input-source option[value="${source}"]`).count(),1);
  }
  assert.equal(await page.locator('#input-source').inputValue(),'leapmotion');
  assert(await page.locator('#leapmotion-input-panel').isVisible());
  assert(!(await page.locator('#camera-control').isVisible()));
  assert.match(await page.locator('#leapmotion-input-panel').textContent(),/Ultraleap Tracking Service/);
  if(await page.locator('#recording').isChecked())await page.locator('label').filter({has:page.locator('#recording')}).click();
  await page.locator('#start').click();await running();
  assert.match(await page.locator('#skeleton-status').textContent(),/Leap Motion 2.*21 点/);
  assert(results.some(s=>s.update.source==='leapmotion'&&s.update.tracking));
  assert.equal(await page.evaluate(()=>window.cameraRequests),0);
  console.log('LEAP_UI left tracking');

  fixture.stdin.write('pause\n');
  await page.waitForFunction(()=>document.querySelector('#leapmotion-state').textContent.includes('过期'));
  await page.waitForFunction(()=>!document.querySelector('#tracking-dot').classList.contains('online'));
  fixture.stdin.write('resume\n');await running();
  await page.route('**/api/leapmotion/frame',route=>route.abort());
  await page.waitForFunction(()=>document.querySelector('#leapmotion-state').textContent.includes('正在重试'));
  await page.waitForFunction(()=>!document.querySelector('#tracking-dot').classList.contains('online'));
  await page.unroute('**/api/leapmotion/frame');await running();
  console.log('LEAP_UI stale/network recovery');

  await stop();assert.equal(await page.locator('#leapmotion-state').textContent(),'已停止');
  await page.locator('#side').selectOption('right');await ready();
  await page.locator('#start').click();await running();
  assert.match(await page.locator('#tracking-text').textContent(),/右手已识别/);
  await stop();await page.locator('#input-source').selectOption('mediapipe');
  assert(await page.locator('#camera-control').isVisible());assert(!(await page.locator('#leapmotion-input-panel').isVisible()));
  await page.locator('#input-source').selectOption('leapmotion');
  assert.match(await page.locator('#calibration-link').getAttribute('href'),/source=leapmotion/);
  assert.equal(legacyTrackingFrames,0,'tracking must consume server solves without reuploading skeletons');
  console.log('LEAP_UI right/stop/source switch');

  calibrating=true;
  await page.goto(url+'/calibration?source=leapmotion');await ready();
  await page.addStyleTag({content:'.cal-stage{height:220px!important;min-height:220px!important;}'});
  assert.equal(await page.locator('#input-source').inputValue(),'leapmotion');
  assert(!(await page.locator('#camera-control').isVisible()));
  assert.match(await page.locator('#source-help').textContent(),/LeapC SDK/);
  await page.locator('#start').click();
  await page.waitForFunction(()=>document.querySelector('#solve-status').textContent.includes('实时求解'));
  assert(calibrationRequests.some(p=>p.source==='leapmotion'&&p.leapmotion_revision>=0&&Number.isFinite(p.source_epoch)));
  assert.equal(await page.evaluate(()=>window.cameraRequests),0);
  await page.locator('#start').click();
  await page.waitForFunction(()=>document.querySelector('#start').textContent==='接入实时输入');
  assert.equal(await page.locator('#input-x').isDisabled(),false);
  console.log('LEAP_UI calibration latest/revision/freeze');

  await mkdir(root+'.work/leapmotion',{recursive:true});
  await page.screenshot({path:root+'.work/leapmotion/calibration.png',fullPage:true});
  assert.deepEqual(errors,[]);
  assert(!controlActions.some(action=>['arm','connect','enable'].includes(action)),'input must not activate robot hardware');
  console.log(JSON.stringify({passed:true,solvedFrames:results.length,calibrationFrames:calibrationRequests.length,
    scope:'synthetic trusted Leap observations with real solver and UI; no native SDK/device validation',errors}));
} catch(error) {
  console.error('LEAP_UI_FAILURE',JSON.stringify({message:error.message,errors,
    notice:await page.locator('#notice').textContent().catch(()=>null),
    input:await page.locator(calibrating?'#input-status':'#leapmotion-state').textContent().catch(()=>null)}));
  throw error;
} finally {
  await browser.close();fixture.stdin.end('quit\n');
  await new Promise(resolve=>{const timer=setTimeout(()=>{fixture.kill();resolve();},6000);
    fixture.once('exit',()=>{clearTimeout(timer);resolve();});});
}
