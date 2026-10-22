// Synthetic Quest transport -> real server retarget -> desktop UI. No hardware.
import {chromium} from '@playwright/test';
import {execFileSync} from 'node:child_process';
import {mkdir} from 'node:fs/promises';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';

const root=fileURLToPath(new URL('../../../',import.meta.url));
const adb=process.env.QUEST_TRANSPORT==='adb';
const apiPrefix=adb?'/api/quest-adb':'/api/webxr';
const fixture=JSON.parse(execFileSync(process.env.HAND_PYTHON||'python3',[
  '-c',`import sys,json
sys.path[:0]=['.','src','src/tracking']
from hand_profiles import load_hand_profile
from webxr import WEBXR_HAND_JOINT_NAMES, WEBXR_25_TO_MEDIAPIPE_21
p=load_hand_profile('TsenseI_v3_2');s=p.create_retargeter('left')
points=s.fk.compute(p.init_pose()).tolist()
native=[[0,0,0] for _ in range(25)]
for i,j in enumerate(WEBXR_25_TO_MEDIAPIPE_21):native[j]=points[i]
print(json.dumps({'names':list(WEBXR_HAND_JOINT_NAMES),'positions':native,'landmarks':points}))`
],{cwd:root,encoding:'utf8'}));
const browser=await chromium.launch({headless:true,
  args:process.env.HAND_BROWSER_SWIFTSHADER==='1'?['--use-angle=swiftshader','--enable-unsafe-swiftshader']:[],
  executablePath:process.env.CHROME_PATH||undefined});
const viewport=process.env.HAND_BROWSER_COMPACT==='1'?{width:960,height:800}:{width:1440,height:1100};
const page=await browser.newPage({viewport});
const errors=[],responses=[];let legacyFrameRequests=0;
page.on('pageerror',e=>errors.push(e.message));
page.on('request',r=>{if(r.url().endsWith('/api/frame'))legacyFrameRequests++;});
page.on('response',async r=>{
  if(!r.ok())return;
  if(r.url().endsWith('/api/frame'))responses.push(await r.json());
  else if(r.url().endsWith('/api/webxr/latest')){
    const snapshot=await r.json().catch(()=>null);if(snapshot?.update)responses.push(snapshot.update);
  }
});
await page.addInitScript(()=>{
  window.cameraRequests=0;
  if(navigator.mediaDevices)navigator.mediaDevices.getUserMedia=()=>{window.cameraRequests++;throw Error('WebXR must not request camera');};
});
try {
  await page.goto((process.env.WEB_URL||'http://127.0.0.1:18866')+`/?source=${adb?'quest_adb':'webxr'}`);
  await page.waitForFunction(()=>!document.querySelector('#start').disabled);
  assert.equal(await page.locator('#input-source').inputValue(),adb?'quest_adb':'webxr');
  await page.locator('label').filter({has:page.locator('#recording')}).click();
  await page.locator('#start').click();
  await page.locator('#quest-pairing').waitFor({state:'visible'});
  const code=await page.locator('#quest-token').innerText();
  assert.equal(code,'123456');
  assert((await page.locator('#quest-link').inputValue()).endsWith(adb?'/q/adb':'/q'));
  if(adb) assert.match(await page.locator('#camera-caption').innerText(),/USB ADB/);
  const {token}=await page.evaluate(async({code,apiPrefix})=>(await fetch(`${apiPrefix}/pair`,{
    method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({code})})).json(),{code,apiPrefix});
  assert(token.length>=32);
  await page.evaluate(async({fixture,token,adb,apiPrefix})=>{
    let sequence=0;
    const send=async(type,hands)=>fetch(`${apiPrefix}/publish`,{
      method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({token,message:{
        schema:adb?'teleop.quest.adb.hand_skeleton.v1':'teleop.webxr.hand_skeleton.v1',type,stream_id:'browser-integration-test',sequence:sequence++,
        timestamp_ms:Date.now(),client_perf_ms:performance.now(),coordinate_space:'webxr_local_floor',
        joint_set:adb?'mediapipe-21':'webxr-hand-25',units:'m',
        ...(adb?{transport:'adb'}:{joint_names:fixture.names}),hands,
      }})});
    window.questTestSend=send;
    await send('session_started');
    const hand=adb?{handedness:'left',tracked:true,landmarks:fixture.landmarks.map(point=>point.map((v,k)=>v+[1,1.5,-.5][k]))}
      :{handedness:'left',tracked:true,joints:fixture.names.map((name,i)=>({
      name,valid:true,position:fixture.positions[i].map((v,k)=>v+[1,1.5,-.5][k]),orientation:[0,0,0,1],radius:.005,
    }))};
    const pump=async()=>{await send('hand_skeleton',[hand]);if(window.questTestRunning)window.questTestTimer=setTimeout(pump,20);};
    window.questTestRunning=true;void pump();
  },{fixture,token,adb,apiPrefix});
  await page.waitForFunction(()=>document.querySelector('#tracking-text').textContent.includes('正在跟随'),{timeout:30000});
  await page.waitForFunction(()=>Number(document.querySelector('#fps').textContent)>=1);
  assert.equal(await page.evaluate(()=>window.cameraRequests),0);
  assert(responses.some(r=>r.tracking&&r.source==='webxr'&&r.depth_source==='webxr_metric'));
  if(adb){
    assert(responses.some(r=>r.tracking&&r.source_metadata?.transport==='adb'&&r.source_metadata?.source_joint_count===21));
    assert.match(await page.locator('#skeleton-status').innerText(),/USB ADB · 21 点/);
  }
  assert.equal(legacyFrameRequests,0,'A3.2 consumes background solves without replaying the skeleton');
  // A single delayed desktop poll must signal loss without destroying the
  // pairing or requiring another click to restart tracking.
  await page.evaluate(()=>{
    window.questSawInterrupted=false;
    window.questInterruptionObserver=new MutationObserver(()=>{
      if(document.querySelector('#quest-state').textContent.includes('正在重试'))window.questSawInterrupted=true;
    });
    window.questInterruptionObserver.observe(document.querySelector('#quest-state'),{childList:true,subtree:true,characterData:true});
  });
  let delayedPoll=false;
  await page.route('**/api/webxr/latest',async route=>{
    if(delayedPoll){await route.continue();return;}
    delayedPoll=true;
    await new Promise(resolve=>setTimeout(resolve,450));
    await route.abort().catch(()=>{});
  });
  await page.waitForFunction(()=>window.questSawInterrupted);
  await page.waitForFunction(()=>document.querySelector('#quest-state').textContent.includes('数据龄期'));
  assert.equal(await page.locator('#quest-token').innerText(),code,'transient timeout preserves pairing');
  await page.unroute('**/api/webxr/latest');
  await page.evaluate(()=>window.questInterruptionObserver.disconnect());
  assert.equal(await page.locator('#model-stage canvas').count(),1);
  await page.locator('#skeleton-stage canvas').waitFor();
  await mkdir(new URL('../../../logs/webxr_validation/',import.meta.url),{recursive:true});
  if(process.env.HAND_BROWSER_SCREENSHOTS!=='0')
    await page.screenshot({path:fileURLToPath(new URL('../../../logs/webxr_validation/desktop-webxr.png',import.meta.url)),fullPage:process.env.HAND_BROWSER_COMPACT!=='1'});
  await page.setViewportSize({width:390,height:844});
  assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth),'mobile layout overflow');
  await page.setViewportSize(viewport);
  // A disconnected producer cannot keep the old hand alive through PC polling.
  await page.evaluate(()=>{window.questTestRunning=false;clearTimeout(window.questTestTimer);});
  await page.waitForFunction(()=>!document.querySelector('#tracking-dot').classList.contains('online'));
  assert(responses.some(r=>!r.tracking&&r.source==='webxr'));
  await page.locator('#start').click();
  await page.waitForFunction(()=>document.querySelector('#camera-state').textContent==='已停止');
  assert.equal(await page.locator('#input-source').isEnabled(),true);
  assert.equal(await page.locator('#quest-pairing').isVisible(),false);
  const expired=await page.evaluate(async()=>{const r=await window.questTestSend('session_ended');return r.status;});
  assert.equal(expired,400);
  await page.locator('#input-source').selectOption('mediapipe');
  assert.equal(await page.locator('#camera-control').isVisible(),true);
  assert.equal(await page.locator('#quest-input-panel').isVisible(),false);
  assert.deepEqual(errors,[]);
  assert.equal(legacyFrameRequests,0,'source loss and recovery must not resume the legacy replay path');
  console.log(`PASS: Quest ${adb?'USB ADB 21':'WebXR 25'} synthetic -> real retarget -> UI, ${responses.length} frames, lost input, stop, source switch; zero camera requests; screenshots ${process.env.HAND_BROWSER_SCREENSHOTS==='0'?'skipped':'captured'}`);
  const summarize=key=>{const values=responses.map(r=>r[key]).filter(Number.isFinite).sort((a,b)=>a-b);
    return {count:values.length,p50:values[Math.floor(values.length*.5)]??null,p95:values[Math.floor(values.length*.95)]??null};};
  console.log(JSON.stringify({scenario:'synthetic_quest_background_ui',observed_results:responses.length,
    legacy_frame_requests:legacyFrameRequests,solve_ms:summarize('solve_ms'),
    source_age_at_solve_ms:summarize('source_age_at_solve_ms'),
    measurement_scope:'UI-observed synthetic results, not headset/device end-to-end latency'}));
} catch(error) {
  console.error('UI STATE',await page.locator('#notice').innerText(),await page.locator('#tracking-text').innerText(),errors);
  throw error;
} finally {await browser.close();}
