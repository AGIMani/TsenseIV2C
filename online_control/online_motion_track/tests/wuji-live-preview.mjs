// Manual smoke check against the real glove. Starts preview only; never connects/arms a robot.
import {chromium} from '@playwright/test';
import {mkdir,writeFile} from 'node:fs/promises';
import {fileURLToPath} from 'node:url';
const root=fileURLToPath(new URL('../../../',import.meta.url));
const browser=await chromium.launch({headless:true,
  executablePath:process.env.CHROME_PATH||'C:/Program Files/Google/Chrome/Application/chrome.exe'});
// The launcher verifies TLS against its local CA; this preview check uses a fresh browser profile.
const page=await browser.newPage({viewport:{width:1440,height:1100},ignoreHTTPSErrors:true});
const errors=[],frames=[];
const snapshots=[],latestTimes=[],requestStarts=new WeakMap();
page.on('request',r=>{if(r.url().endsWith('/api/wuji/frame'))requestStarts.set(r,performance.now());});
page.on('pageerror',e=>errors.push(e.message));
page.on('response',async r=>{
  if(r.url().endsWith('/api/frame')&&r.ok()){
    const frame=await r.json().catch(()=>null); // A final response may be cancelled during browser cleanup.
    if(frame)frames.push(frame);
  }
  if(r.url().endsWith('/api/wuji/frame')&&r.ok()){
    const snapshot=await r.json().catch(()=>null);
    if(snapshot){snapshots.push(snapshot);latestTimes.push(performance.now()-requestStarts.get(r.request()));
      if(snapshot.update)frames.push(snapshot.update);}
  }
});
try{
  await page.goto((process.env.WEB_URL||'https://127.0.0.1:8766')+'/?model='+
    (process.env.HAND_MODEL||'TsenseI_v3_0')+'&source=wuji_glove');
  await page.waitForFunction(()=>!document.querySelector('#start').disabled);
  if(await page.locator('#recording').isChecked())await page.locator('label').filter({has:page.locator('#recording')}).click();
  await page.locator('#start').click();
  await page.waitForFunction(()=>document.querySelector('#wuji-state').textContent.includes('已连接'),{timeout:30000});
  await page.waitForFunction(()=>document.querySelector('#tracking-text').textContent.includes('正在跟随'),{timeout:30000});
  await page.waitForFunction(()=>Number(document.querySelector('#fps').textContent)>0);
  const duration=Number(process.env.DURATION_SECONDS||2);
  const started=performance.now();
  const frameStart=frames.length,snapshotStart=snapshots.length;
  await new Promise(resolve=>setTimeout(resolve,duration*1000));
  const measuredFrames=frames.slice(frameStart),measuredSnapshots=snapshots.slice(snapshotStart);
  const stats=values=>{const a=values.filter(Number.isFinite).sort((a,b)=>a-b);
    return a.length?Object.fromEntries([['p50',.5],['p95',.95],['max',1]].map(([k,p])=>[k,a[Math.floor((a.length-1)*p)]])):null;};
  const summary={time:new Date().toISOString(),url:page.url(),source:'REAL_WUJI_GLOVE',
    input:await page.locator('#wuji-state').innerText(),tracking:await page.locator('#tracking-text').innerText(),
    fps:await page.locator('#fps').innerText(),received:frames.length,trackingFrames:frames.filter(f=>f.tracking).length,
    measurement:{duration_s:(performance.now()-started)/1000,frames:measuredFrames.length,
      tracking:measuredFrames.filter(f=>f.tracking).length,
      polls:measuredSnapshots.length,validPolls:measuredSnapshots.filter(s=>s.connected).length,
      latest_request_ms:stats(latestTimes.slice(snapshotStart)),
      source_age_ms:stats(measuredSnapshots.map(s=>s.age_ms)),
      solve_ms:stats(measuredFrames.filter(f=>f.tracking).map(f=>f.solve_ms)),
      network:measuredSnapshots.at(-1)?.source_metadata?.transport},
    errors,deviceControl:'not connected or armed by this check'};
  await mkdir(root+'logs/wuji_validation',{recursive:true});
  await page.screenshot({path:root+'logs/wuji_validation/wuji-live.png',fullPage:true});
  await writeFile(root+'logs/wuji_validation/wuji-live.json',JSON.stringify(summary,null,2));
  console.log(JSON.stringify(summary));
}catch(error){
  const text=selector=>page.locator(selector).innerText({timeout:1000}).catch(()=>null);
  console.error(JSON.stringify({error:String(error),url:page.url(),notice:await text('#notice'),
    tracking:await text('#tracking-text'),input:await text('#wuji-state'),errors}));
  throw error;
}finally{
  if(await page.locator('#start').innerText().then(s=>s.includes('停止')).catch(()=>false))await page.locator('#start').click();
  await browser.close();
}
