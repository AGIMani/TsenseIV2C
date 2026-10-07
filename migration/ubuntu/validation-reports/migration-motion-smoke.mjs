// Run only against isolated test servers with empty motion data; never connects hardware.
const {chromium}=await import(process.env.PLAYWRIGHT_MODULE || '@playwright/test');
import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
const browser=await chromium.launch({executablePath:process.env.CHROME_PATH,headless:true,args:['--use-angle=swiftshader','--enable-unsafe-swiftshader']});
const page=await browser.newPage({viewport:{width:1450,height:1050}});
const errors=[],failed=[];
page.on('pageerror',e=>errors.push(e.message));
page.on('response',r=>{if(!r.ok()&&(r.url().includes('/assets/')||r.url().includes('/vendor/')||r.url().includes('/api/')))failed.push(r.url());});
try{
 await page.goto(process.env.MOTION_URL);
 await page.waitForFunction(()=>document.querySelector('#control').textContent==='释放控制');
 const models=['TsenseI_v2','TsenseI_v3_0','TsenseI_v3_0_withid','TsenseI_v3_2','TsenseI_v3_2_withid'];
 for(const model of models){
  console.log('MODEL',model);
  await page.locator('#modelSelect').selectOption(model);
  const expectedFile={TsenseI_v2:'motions.json',TsenseI_v3_0:'motions_a30_left.json',TsenseI_v3_0_withid:'motions_a30_withid_left.json',TsenseI_v3_2:'motions_a32_left.json',TsenseI_v3_2_withid:'motions_a32_withid_left.json'}[model];
  await page.waitForFunction(file=>document.querySelector('#fileNote').textContent.includes(file),expectedFile);
  await page.waitForFunction(()=>document.querySelector('#modelNote').textContent==='');
  await page.waitForFunction(n=>document.querySelectorAll('#joints input[type=number]').length===n,model==='TsenseI_v2'?21:20);
  const joint=page.locator('#joints input[type=number]').first();
  await joint.fill('0.1');await joint.press('Enter');
  await page.locator('#savemotion button').click();
  await page.locator('#dialog-items input[type=text]').fill('Ubuntu迁移验证-'+model);
  await page.getByRole('button',{name:'确认',exact:true}).click();
  await page.waitForFunction(()=>document.querySelector('#motionCount').textContent==='1 个动作');
  if(model==='TsenseI_v2') await page.getByRole('button',{name:'确认',exact:true}).click();
  const downloadPromise=page.waitForEvent('download');await page.locator('#export').click();
  const download=await downloadPromise;const text=await readFile(await download.path(),'utf8');
  const payload=JSON.parse(text);assert.equal(payload._hand_side,'left');
  assert(Object.keys(payload.motions||payload).includes('Ubuntu迁移验证-'+model));
  await page.locator('#deletemotion button').click();
  await page.getByRole('button',{name:'确认',exact:true}).click();
  await page.waitForFunction(()=>document.querySelector('#motionCount').textContent==='0 个动作');
  await page.locator('#file').setInputFiles({name:download.suggestedFilename(),mimeType:'application/json',buffer:Buffer.from(text)});
  await page.waitForFunction(()=>document.querySelector('#motionCount').textContent==='1 个动作');
  await page.locator('#motionstreamplay button').click();
  await page.waitForFunction(()=>document.querySelector('#status').textContent.includes('正在播放'));
  await page.waitForFunction(()=>document.querySelector('#status').textContent.includes('就绪'),null,{timeout:10000});
  assert.equal(Number(await joint.inputValue()),.1);
  const sideResponse=page.waitForResponse(async r=>r.url().endsWith('/api/state')&&(await r.json()).side==='right');
  await page.locator('#handSide select').selectOption('1');
  await sideResponse;
  if(model!=='TsenseI_v2') await page.waitForFunction(()=>document.querySelector('#fileNote').textContent.includes('_right.json'));
  await page.waitForFunction(()=>document.querySelector('#modelNote').textContent==='');
 }
 await page.screenshot({path:process.env.MOTION_QA_OUTPUT+'/five-models.png',fullPage:true});
 assert.deepEqual(errors,[]);assert.deepEqual(failed,[]);
 console.log(JSON.stringify({ok:true,models:5,left_right:10,save_delete_import_export_playback:true,errors,failed}));
} finally {await browser.close();}
