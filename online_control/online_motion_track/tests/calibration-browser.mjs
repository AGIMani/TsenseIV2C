// End-to-end real FK, HTTP, three.js dragging, camera worker and metric relays.
import {chromium} from '@playwright/test';
import {spawn} from 'node:child_process';
import {readFile,mkdir} from 'node:fs/promises';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';

const root=fileURLToPath(new URL('../../../',import.meta.url));
const fixture=spawn(process.env.HAND_PYTHON||'python', ['-u','tests/run_calibration_browser_fixture.py'],{cwd:root,stdio:['pipe','pipe','pipe']});
const url=await new Promise((resolve,reject)=>{
  let text='';const deadline=setTimeout(()=>reject(Error('Fixture timed out')),30000);
  fixture.stdout.on('data',chunk=>{text+=chunk;const match=text.match(/WUJI_TEST_URL=(http:\/\/\S+)/);if(match){clearTimeout(deadline);resolve(match[1]);}});
  fixture.stderr.on('data',chunk=>process.stderr.write(chunk));fixture.on('exit',code=>{clearTimeout(deadline);reject(Error(`Fixture exited ${code}: ${text}`));});
});
const browser=await chromium.launch({headless:true,...(process.env.CHROME_PATH?{executablePath:process.env.CHROME_PATH}:{}),
  args:['--use-angle=swiftshader','--enable-unsafe-swiftshader','--use-fake-ui-for-media-stream','--use-fake-device-for-media-stream']});
const page=await browser.newPage({viewport:{width:1440,height:1050}}), errors=[], outputs=[];
page.setDefaultTimeout(20000);
page.on('pageerror',error=>errors.push(error.message));
page.on('response',async response=>{if(['/api/frame','/api/calibration/update'].some(path=>response.url().endsWith(path))&&response.ok())outputs.push(await response.json());});
// Inspect world/screen transforms from the actual rendered scene, without a
// production debug export. This also allows mouse drags along real gizmo axes.
const source=await readFile(new URL('../static/calibration-viewer.js',import.meta.url),'utf8');
await page.route('**/calibration-viewer.js',route=>route.fulfill({contentType:'text/javascript',body:source.replace('super(element);','super(element); (window.calibrationViews ||= []).push(this);')}));
// Headless SwiftShader shader compilation can exceed the detector timeout.
// Exercise the actual worker/model with its supported CPU delegate in this test.
const workerSource=await readFile(new URL('../static/detector.worker.js',import.meta.url),'utf8');
await page.route('**/detector.worker.js',route=>route.fulfill({contentType:'text/javascript',body:workerSource.replaceAll('"GPU"','"CPU"')}));
await mkdir(root+'.work/web-calibration',{recursive:true});
async function waitSolved(after=0) {
  await page.waitForFunction(()=>document.querySelector('#solve-status').textContent.includes('求解更新'));
  for(let i=0;outputs.length<=after&&i<100;i++)await page.waitForTimeout(50);
  assert(outputs.length>after);assert(!(await page.locator('#notice').isVisible()),await page.locator('#notice').textContent());
}
async function screenPoint(view,index) {
  return page.evaluate(([view,index])=>{
    const v=window.calibrationViews[view], p=v.nodes[index].position.clone().project(v.camera), r=v.renderer.domElement.getBoundingClientRect();
    return [r.left+(p.x+1)*r.width/2,r.top+(1-p.y)*r.height/2];
  },[view,index]);
}
async function drag(view,index) {
  await page.locator(view===0?'#input-stage':'#calibration-stage').scrollIntoViewIfNeeded();
  const point=await screenPoint(view,index);await page.mouse.move(...point);await page.mouse.down();
  await page.mouse.move(point[0]+35,point[1]-12,{steps:8});await page.mouse.up();
}
try {
  const createdPromise=page.waitForResponse(response=>response.url().endsWith('/api/session')&&response.ok());
  await page.goto(url+'/calibration?model=TsenseI_v3_2_withid');
  const created=await (await createdPromise).json();
  await page.waitForFunction(()=>!document.querySelector('#start').disabled);await waitSolved();
  console.log('PHASE loaded');
  assert.equal(await page.locator('.cal-stage canvas').count(),3);
  const shown=await page.evaluate(()=>window.calibrationViews[1].nodes.map(node=>node.position.toArray()));
  assert(shown.every((point,i)=>point.every((value,j)=>Math.abs(value-created.calibration_points[i][j])<1e-10)));
  assert.equal(await page.locator('[data-preset]').count(),18);
  const originalRows=created.calibration_rows;
  let before=outputs.length;
  await page.locator('[data-preset="pinch_middle"]').click();await waitSolved(before);
  assert.match(await page.locator('#preset-detail').textContent(),/拇–中/);
  const firstPreset=outputs.findLast(result=>result.preset);
  assert(firstPreset.preset.gap_mm<.1);
  await page.locator('#input-point').selectOption('8');
  const beforeDrag=Number(await page.locator('#input-x').inputValue());
  before=outputs.length;await drag(0,8);await waitSolved(before);
  assert.notEqual(Number(await page.locator('#input-x').inputValue()),beforeDrag);
  assert.equal(await page.locator('#draft-state').textContent(),'标定已载入');
  console.log('PHASE input drag');
  await page.locator('#calibration-point').selectOption('8');
  before=outputs.length;await drag(1,8);await waitSolved(before);
  assert.equal(await page.locator('#draft-state').textContent(),'有未保存标定');
  const snapshot=await page.evaluate(()=>window.calibrationViews[0].nodes.map(node=>node.position.toArray()));
  await page.locator('#calibration-z').fill('27.5');before=outputs.length;await page.locator('#calibration-z').press('Tab');await waitSolved(before);
  const afterCalibration=await page.evaluate(()=>window.calibrationViews[0].nodes.map(node=>node.position.toArray()));
  assert.deepEqual(afterCalibration,snapshot,'Editing static calibration must not move the posed source');
  console.log('PHASE calibration drag');
  for(const id of ['pinch_index','pinch_ring','pinch_little','kapandji_1','kapandji_10']) {
    before=outputs.length;await page.locator(`[data-preset="${id}"]`).click();await waitSolved(before);
  }
  await page.locator('#save').click();await page.waitForFunction(()=>document.querySelector('#save-status').textContent.startsWith('已保存'));
  assert.equal(await page.locator('#draft-state').textContent(),'标定已载入');
  const saved=await (await fetch(url+'/api/calibration/export',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({session:created.session})})).json();
  assert(!saved.yaml.includes('input_points'));assert(saved.yaml.includes('27.5')||saved.yaml.includes('0.0275'));
  const downloadPromise=page.waitForEvent('download');await page.locator('#export').click();const download=await downloadPromise;
  assert.equal(download.suggestedFilename(),'robot_keypoints_21.yaml');
  await page.locator('#reload').click();await page.waitForFunction(()=>document.querySelector('#save-status').textContent.startsWith('已重新加载'));
  console.log('PHASE files');
  // A real worker on a fake camera verifies initialization and clean freezing.
  if(!process.env.CALIBRATION_SKIP_REAL_CAMERA) {
  await page.locator('#start').click();
  await page.waitForFunction(()=>document.querySelector('#start').textContent==='停止 / 冻结',null,{timeout:60000});
  console.log('PHASE camera active');
  await page.waitForFunction(()=>document.querySelector('#input-status').textContent.includes('未检测'),null,{timeout:60000});
  assert(await page.locator('#input-x').isDisabled());assert(!(await page.locator('#calibration-x').isDisabled()));
  await page.locator('#start').click();await page.waitForFunction(()=>document.querySelector('#start').textContent==='接入实时输入');
  assert(await page.evaluate(()=>document.querySelector('#video').srcObject===null));
  console.log('PHASE real camera worker');
  }
  // Deterministic camera landmarks then exercise the same transferred-frame UI
  // and HTTP solve path. Only the worker output is substituted in the test.
  await page.evaluate(points=>{
    window.syntheticCameraPoints=points;window.syntheticHand=true;window.originalWorker=window.Worker;
    window.Worker=class {
      constructor(url,options){if(!String(url).includes('detector.worker'))return new window.originalWorker(url,options);}
      postMessage(data){data.bitmap?.close();setTimeout(()=>this.onmessage?.({data:data.type==='init'?{type:'ready',delegate:'TEST'}:{type:'result',result:{
        handedness:window.syntheticHand?[[{categoryName:'Left',score:1}]]:[],
        worldLandmarks:window.syntheticHand?[window.syntheticCameraPoints.map(([x,y,z])=>({x,y,z}))]:[],
        landmarks:window.syntheticHand?[window.syntheticCameraPoints.map(()=>({x:.5,y:.5,z:0}))]:[],
      }}}),10);}
      terminate(){this.onmessage=null;}
    };
  },firstPreset.input_points);
  await page.locator('#start').click();await page.waitForFunction(()=>document.querySelector('#input-status').textContent==='21 点实时输入');
  before=outputs.length;await page.locator('#calibration-x').fill('5');await page.locator('#calibration-x').press('Tab');
  await page.waitForFunction(()=>document.querySelector('#solve-status').textContent.includes('实时求解'));
  await page.waitForTimeout(350);assert(outputs.length>before);
  await page.evaluate(()=>window.syntheticHand=false);
  await page.waitForFunction(()=>document.querySelector('#input-status').textContent.includes('未检测'));
  await page.locator('#start').click();await page.waitForFunction(()=>document.querySelector('#start').textContent==='接入实时输入');
  // Reuse the real bridge transport and receiver; switching never opens a camera.
  console.log('PHASE synthetic camera');
  await page.evaluate(()=>{navigator.mediaDevices.getUserMedia=()=>{throw Error('Metric source must not open camera');};});
  await page.locator('#input-source').selectOption('wuji_glove');
  const gloveFrame=page.waitForResponse(async response=>response.url().endsWith('/api/frame')&&response.ok()&&
    (await response.json()).source==='wuji_glove'&&(await response.json()).tracking);
  await page.locator('#start').click();await page.waitForFunction(()=>document.querySelector('#input-status').textContent.includes('21 点实时输入')||document.querySelector('#input-status').textContent.includes('手套已连接'),null,{timeout:30000});
  assert((await (await gloveFrame).json()).tracking);
  fixture.stdin.write('pause\n');await page.waitForFunction(()=>document.querySelector('#input-status').textContent.includes('未检测')||document.querySelector('#input-status').textContent.includes('过期'));
  await page.locator('#start').click();await page.waitForFunction(()=>document.querySelector('#start').textContent==='接入实时输入');
  fixture.stdin.write('resume\n');
  console.log('PHASE wuji');
  await page.locator('#input-source').selectOption('webxr');await page.locator('#start').click();await page.locator('#quest-code').waitFor({state:'visible'});
  const code=await page.locator('#quest-code').textContent();assert(code.length>=4);
  const paired=await (await fetch(url+'/api/webxr/pair',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({code})})).json();
  assert(paired.token);
  const names=['wrist','thumb-metacarpal','thumb-phalanx-proximal','thumb-phalanx-distal','thumb-tip',
    ...['index','middle','ring','pinky'].flatMap(f=>['metacarpal','phalanx-proximal','phalanx-intermediate','phalanx-distal','tip'].map(j=>`${f}-finger-${j}`))];
  const mapping=[0,1,2,3,4,6,7,8,9,11,12,13,14,16,17,18,19,21,22,23,24];
  const joints=names.map((name,i)=>({name,valid:true,position:firstPreset.input_points[Math.max(0,mapping.indexOf(i))],orientation:[0,0,0,1],radius:.004}));
  const post=message=>fetch(url+'/api/webxr/publish',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({token:paired.token,message})});
  const base={schema:'teleop.webxr.hand_skeleton.v1',stream_id:'calibration-browser',timestamp_ms:Date.now()};
  await post({...base,type:'client_hello',sequence:0});
  for(let i=1;i<16;i++) {
    const response=await post({...base,type:'hand_skeleton',sequence:i,client_perf_ms:performance.now(),joint_set:'webxr-hand-25',coordinate_space:'webxr_local_floor',hands:[{handedness:'left',tracked:true,joints}]});
    assert(response.ok,await response.text());await page.waitForTimeout(40);
  }
  assert(outputs.some(result=>result.source==='webxr'&&result.tracking));
  console.log('PHASE quest');
  await page.locator('#start').click();await page.waitForFunction(()=>document.querySelector('#start').textContent==='接入实时输入');
  await page.screenshot({path:root+'.work/web-calibration/desktop.png',fullPage:true});
  await page.setViewportSize({width:390,height:844});await page.screenshot({path:root+'.work/web-calibration/mobile.png',fullPage:true});
  assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth));
  await page.setViewportSize({width:1440,height:1050});
  page.on('dialog',dialog=>dialog.accept());
  await page.locator('#side').selectOption('right');await page.waitForFunction(()=>!document.querySelector('#start').disabled);
  await waitSolved(outputs.length-1);assert.match(await page.locator('#point-link').inputValue(),/^right_hand_index_DIP_link$/);
  assert.deepEqual(errors,[]);
  assert.equal(originalRows.length,21);
  console.log(JSON.stringify({passed:true,solves:outputs.length,sources:[...new Set(outputs.map(item=>item.source).filter(Boolean))],errors}));
} catch(error) {
  console.log('FAILED_UI',error.message,await page.locator('#notice').textContent(),await page.locator('#input-status').textContent(),await page.locator('#solve-status').textContent());
  await page.screenshot({path:root+'.work/web-calibration/failure.png',fullPage:true});
  throw error;
} finally {
  await browser.close();fixture.stdin.write('quit\n');
  await new Promise(resolve=>{const timer=setTimeout(()=>{fixture.kill();resolve();},6000);fixture.once('exit',()=>{clearTimeout(timer);resolve();});});
}
