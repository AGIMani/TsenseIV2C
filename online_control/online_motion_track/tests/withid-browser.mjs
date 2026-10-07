// Starts isolated localhost test servers; never connects USB or deploys node0.
import {chromium} from '@playwright/test';
import {spawn} from 'node:child_process';
import {mkdir,writeFile} from 'node:fs/promises';
import {fileURLToPath} from 'node:url';
import net from 'node:net';
import assert from 'node:assert/strict';
const root=fileURLToPath(new URL('../../../',import.meta.url));
const out=`${root}/.work/withid-browser`;
await mkdir(out,{recursive:true});
const python=process.env.HAND_PYTHON||'python';
const port=()=>new Promise(resolve=>{const s=net.createServer();s.listen(0,'127.0.0.1',()=>{const p=s.address().port;s.close(()=>resolve(p));});});
const trackPort=await port(),playPort=await port();
await writeFile(`${out}/config.json`,JSON.stringify({data_dir:`${out}/data`}));
const children=[];
function start(args){const child=spawn(python,['-B','-X','utf8',...args],{cwd:root,windowsHide:true,env:{...process.env,NO_PROXY:'127.0.0.1,localhost'}});let log='';child.stdout.on('data',b=>log+=b);child.stderr.on('data',b=>log+=b);children.push({child,log:()=>log});return child;}
let browser, page;
const errors=[];
try{
  start(['online_control/online_motion_track/main.py','--port',String(trackPort)]);
  start(['online_control/online_motion_play/server.py','--port',String(playPort),'--no-browser','--config',`${out}/config.json`]);
  for(const p of [trackPort,playPort]){
    let ready=false;
    for(let i=0;i<120;i++){try{ready=(await fetch(`http://127.0.0.1:${p}/`)).ok;}catch{}if(ready)break;await new Promise(r=>setTimeout(r,250));}
    assert.ok(ready,children.map(c=>c.log()).join('\n'));
  }
  browser=await chromium.launch({executablePath:process.env.CHROME_PATH||'C:/Program Files/Google/Chrome/Application/chrome.exe',headless:true});
  page=await browser.newPage({viewport:{width:1440,height:1080}});
  page.on('pageerror',e=>errors.push(e.message));
  page.on('response',r=>{if(r.status()>=400)errors.push(`${r.status()} ${r.url()}`);});
  await page.goto(`http://127.0.0.1:${trackPort}/?model=TsenseI_v3_0_withid`);
  await page.waitForFunction(()=>!document.querySelector('#start').disabled);
  assert.equal(await page.locator('#hand-model').inputValue(),'TsenseI_v3_0_withid');
  assert.match(await page.locator('#model-caption').innerText(),/with ID.*21 点待人工确认/);
  await page.screenshot({path:`${out}/track-left.png`,fullPage:true});
  await page.locator('#side').selectOption('right');
  await page.waitForFunction(()=>document.querySelector('#model-caption').textContent.includes('RIGHT'));
  await page.waitForFunction(()=>!document.querySelector('#start').disabled);
  await page.screenshot({path:`${out}/track-right.png`,fullPage:true});
  await page.goto(`http://127.0.0.1:${playPort}/`);
  await page.waitForFunction(()=>document.querySelector('#dof').textContent.includes('21'));
  await page.waitForFunction(()=>document.querySelector('#control').textContent.includes('释放控制'));
  await page.waitForFunction(()=>!document.querySelector('#modelSelect').disabled);
  await page.locator('#modelSelect').selectOption('TsenseI_v3_0_withid');
  await page.waitForFunction(()=>document.querySelector('#dof').textContent.includes('20'));
  await page.waitForFunction(()=>document.querySelector('#modelNote').textContent==='');
  await page.screenshot({path:`${out}/play-left.png`,fullPage:true});
  assert.ok(await page.locator('#a30Motors').isVisible());
  assert.deepEqual(errors,[]);
  console.log(JSON.stringify({result:'PASS',pages:['track left','track right','play with ID'],errors,output:out}));
}catch(error){
  console.error('BROWSER ERRORS',errors);
  if(page){console.error((await page.locator('body').innerText()).slice(-5000));await page.screenshot({path:`${out}/failure.png`,fullPage:true});}
  throw error;
}finally{
  if(browser)await browser.close();
  for(const {child} of children)if(child.exitCode===null)child.kill();
  await writeFile(`${out}/servers.log`,children.map(c=>c.log()).join('\n'));
}
