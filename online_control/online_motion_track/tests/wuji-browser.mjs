import {chromium} from '@playwright/test';
import {spawn} from 'node:child_process';
import {mkdir} from 'node:fs/promises';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';

const root=fileURLToPath(new URL('../../../',import.meta.url));
const sdkFixture=process.env.WUJI_TEST_SOURCE==='fake_sdk';
const fixture=spawn(process.env.HAND_PYTHON||'E:/conda/envs/hand_dex/python.exe',
  ['-u','-X','utf8',sdkFixture?'tests/run_wuji_sdk_browser_fixture.py':'tests/run_wuji_browser_fixture.py'],{cwd:root,stdio:['pipe','pipe','pipe']});
const url=await new Promise((resolve,reject)=>{
  let output='';const timer=setTimeout(()=>reject(Error('fixture startup timeout')),30000);
  fixture.stdout.on('data',chunk=>{output+=chunk;const match=output.match(/WUJI_TEST_URL=(http:\/\/[^\s]+)/);
    if(match){clearTimeout(timer);resolve(match[1]);}});
  fixture.once('exit',code=>{clearTimeout(timer);reject(Error(`fixture exited ${code}: ${output}`));});
  fixture.stderr.on('data',chunk=>process.stderr.write(chunk));
});
const inspections=[];let inspectionBuffer='';
fixture.stdout.on('data',chunk=>{
  inspectionBuffer+=chunk;
  const lines=inspectionBuffer.split('\n');inspectionBuffer=lines.pop();
  for(const line of lines)if(line.startsWith('WUJI_TEST_STATE='))inspections.push(JSON.parse(line.slice(16)));
});
async function inspectFixture(){
  const count=inspections.length;fixture.stdin.write('inspect\n');
  const until=Date.now()+3000;
  while(inspections.length===count&&Date.now()<until)await new Promise(r=>setTimeout(r,10));
  assert(inspections.length>count,'fixture inspection timed out');return inspections.at(-1);
}
const browser=await chromium.launch({headless:true,
  args:process.env.HAND_BROWSER_SWIFTSHADER==='1'?['--use-angle=swiftshader','--enable-unsafe-swiftshader']:[],
  executablePath:process.env.CHROME_PATH||'C:/Program Files/Google/Chrome/Application/chrome.exe'});
const page=await browser.newPage({viewport:process.env.HAND_BROWSER_COMPACT==='1'?{width:960,height:800}:{width:1440,height:1100}});
const errors=[],results=[],snapshots=[];
page.on('pageerror',e=>errors.push(e.message));
let legacyRequests=0,trackingSession=null;
page.on('request',r=>{
  if(r.url().endsWith('/api/frame'))legacyRequests++;
  if(r.url().endsWith('/api/wuji/frame'))trackingSession=r.postDataJSON()?.session;
});
page.on('response',async r=>{if(r.url().endsWith('/api/wuji/frame')&&r.ok()){
  const snapshot=await r.json().catch(()=>null);if(snapshot){snapshots.push(snapshot);if(snapshot.update)results.push(snapshot.update);}
}});
await page.addInitScript(()=>{
  window.cameraRequests=0;
  navigator.mediaDevices.getUserMedia=()=>{window.cameraRequests++;throw Error('Unexpected camera request');};
});
try{
  await page.goto(url+'/?source=wuji_glove');
  if(process.env.HAND_BROWSER_SMALL_SCENES==='1')await page.addStyleTag({content:
    '#model-stage,#skeleton-stage{width:360px!important;max-width:100%!important;height:240px!important;min-height:240px!important;}'});
  await page.waitForFunction(()=>!document.querySelector('#start').disabled);
  assert.equal(await page.locator('#input-source').inputValue(),'wuji_glove');
  assert(await page.locator('#wuji-input-panel').isVisible());
  assert(!(await page.locator('#camera-control').isVisible()));
  if(await page.locator('#recording').isChecked())await page.locator('label').filter({has:page.locator('#recording')}).click();
  await page.locator('#start').click();
  await page.waitForFunction(()=>document.querySelector('#tracking-text').textContent.includes('正在跟随'));
  console.log('UI_STAGE left_tracking');
  assert(results.some(r=>r.source==='wuji_glove'&&r.depth_source==='wuji_glove_metric'&&r.tracking));
  if(sdkFixture){
    assert(snapshots.some(s=>s.background_processing&&s.update_revision>0&&s.update?.input_path==='server_metric_worker'));
    assert(snapshots.some(s=>s.source_metadata?.transport?.mode==='sdk'));
    // The fake SDK keeps producing independently of the page. Losing PC
    // reads must stop commits when its lease expires, even with fresh input.
    // Hardware status is also a legitimate PC heartbeat, so interrupt both.
    await page.route('**/api/a32/control',route=>route.abort());
    await page.route('**/api/wuji/frame',route=>route.abort());
    // Establish the lease boundary outside the software-rendered page's
    // event loop. This one read shares its cookies; no further reads follow.
    const heartbeat=await page.request.post(url+'/api/wuji/frame',{data:{session:trackingSession,after_revision:-1}});
    assert.equal(heartbeat.status(),200);await heartbeat.body();
    const disconnected=await inspectFixture();
    await new Promise(r=>setTimeout(r,250));
    const independent=await inspectFixture();
    console.log('SDK_READS_BLOCKED',JSON.stringify({disconnected,independent}));
    assert(independent.workers[0].update_revision>disconnected.workers[0].update_revision,
      'fresh SDK callbacks continue solving without desktop reads while the client lease is valid');
    await page.waitForFunction(()=>document.querySelector('#wuji-state').textContent.includes('正在重试'));
    await page.waitForFunction(()=>!document.querySelector('#tracking-dot').classList.contains('online'));
    await new Promise(r=>setTimeout(r,2000));
    const before=await inspectFixture();await new Promise(r=>setTimeout(r,200));const after=await inspectFixture();
    console.log('SDK_LEASE_EXPIRED',JSON.stringify({before,after}));
    assert.equal(before.workers.length,1);assert(before.workers[0].client_age>=2);
    assert.equal(before.workers[0].update_revision,after.workers[0].update_revision,'PC lease expiry stops background solves');
    assert(after.emitted>before.emitted,'synthetic SDK remains live while PC reads are blocked');
    assert.equal(after.workers[0].armed,false);assert.equal(after.workers[0].connected,false);
    assert.deepEqual(after.forbidden_attempts,[]);
    await page.unroute('**/api/a32/control');
    await page.unroute('**/api/wuji/frame');
    await page.waitForFunction(()=>document.querySelector('#tracking-dot').classList.contains('online'));
  }
  assert.equal(await page.evaluate(()=>window.cameraRequests),0);
  fixture.stdin.write('pause\n');
  await page.waitForFunction(()=>document.querySelector('#wuji-state').textContent.includes('过期'));
  await page.waitForFunction(()=>!document.querySelector('#tracking-dot').classList.contains('online'));
  fixture.stdin.write('resume\n');
  await page.waitForFunction(()=>document.querySelector('#tracking-dot').classList.contains('online'));
  console.log('UI_STAGE stale_recovered');
  await mkdir(new URL('../../../logs/wuji_validation/',import.meta.url),{recursive:true});
  if(process.env.HAND_BROWSER_SCREENSHOTS!=='0')
    await page.screenshot({path:root+'logs/wuji_validation/wuji-simulated.png',fullPage:process.env.HAND_BROWSER_COMPACT!=='1'});
  await page.locator('#start').click();
  await page.waitForFunction(()=>!document.querySelector('#input-source').disabled);
  await page.locator('#side').selectOption('right');
  console.log('UI_STAGE right_loading');
  await page.waitForFunction(()=>!document.querySelector('#start').disabled);
  await page.locator('#start').click();
  await page.waitForFunction(()=>document.querySelector('#tracking-text').textContent.includes('右手已识别'));
  console.log('UI_STAGE right_tracking');
  await page.locator('#start').click();
  await page.waitForFunction(()=>!document.querySelector('#input-source').disabled);
  await page.locator('#input-source').selectOption('mediapipe');
  assert(await page.locator('#camera-control').isVisible());
  assert(!(await page.locator('#wuji-input-panel').isVisible()));
  assert.deepEqual(errors,[]);
  assert.equal(legacyRequests,0,'Wuji must not send skeletons back through /api/frame');
  if(sdkFixture){
    const state=await inspectFixture();assert.equal(state.workers.length,0);assert.deepEqual(state.forbidden_attempts,[]);
  }
  console.log(`PASS: simulated Wuji ${sdkFixture?'SDK background worker':'HTTP bridge'} -> real solver -> browser; ${results.length} frames; left/right, stale/recovery, stop/source switch; no camera or robot; screenshots ${process.env.HAND_BROWSER_SCREENSHOTS==='0'?'skipped':'captured'}`);
  const summarize=key=>{const values=results.map(r=>r[key]).filter(Number.isFinite).sort((a,b)=>a-b);
    return {count:values.length,p50:values[Math.floor(values.length*.5)]??null,p95:values[Math.floor(values.length*.95)]??null};};
  console.log(JSON.stringify({scenario:sdkFixture?'fake_sdk_background_ui':'synthetic_bridge_ui',
    observed_results:results.length,legacy_frame_requests:legacyRequests,solve_ms:summarize('solve_ms'),
    source_age_at_solve_ms:summarize('source_age_at_solve_ms'),
    measurement_scope:'UI-observed synthetic results, not camera/device end-to-end latency'}));
} catch(error) {
  console.error(JSON.stringify({notice:await page.locator('#notice').innerText().catch(()=>null),
    input:await page.locator('#wuji-state').innerText().catch(()=>null),
    tracking:await page.locator('#tracking-text').innerText().catch(()=>null), errors,
    lastResult:results.at(-1)}));
  throw error;
} finally {
  await browser.close();fixture.stdin.end('quit\n');
}
