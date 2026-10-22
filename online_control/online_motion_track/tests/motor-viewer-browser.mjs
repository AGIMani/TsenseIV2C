// Isolated viewer and storage; no tracking service, camera or motor is used.
import {chromium} from '@playwright/test';
import {createServer} from 'node:http';
import {readFile,mkdir} from 'node:fs/promises';
import assert from 'node:assert/strict';

const server=createServer(async(req,res)=>{
  if(req.url!=='/viewer'){res.writeHead(404).end();return;}
  res.setHeader('Content-Type','text/html');
  res.end(await readFile(new URL('../../../tools/hand_control_log_viewer.html',import.meta.url)));
});
await new Promise(resolve=>server.listen(0,'127.0.0.1',resolve));
const browser=await chromium.launch({executablePath:process.env.CHROME_PATH,headless:true});
const page=await browser.newPage({viewport:{width:1350,height:1000}}),errors=[];
page.on('pageerror',error=>errors.push(error.message));
const base=`http://127.0.0.1:${server.address().port}/viewer`;
try{
  await page.goto(base);
  const origin=1791546487119;
  const rows=[{type:'session',t:.013,started_at:new Date(origin+13).toISOString(),
    metadata:{model_id:'TsenseI_v3_2_withid',hand_side:'left'}}];
  for(let i=0;i<100;i++){
    const t=1+i*.002,sent_at_unix_ms=origin+t*1000;
    rows.push({type:'h7',t:1.25,source:'a32_sdk_successful_write',sent:true,tick:i+1,
      sent_at_unix_ms,self:[i*.1],mit_pos:[0,0],control_dt_s:.002,
      control_timing:{host_write_interval_ms:2}});
    rows.push({type:'joint_angles',stage:'sdk',t,source:'a32_sdk_successful_write',sent:true,tick:i+1,
      sent_at_unix_ms,joints:{left_hand_pinky_DIP_joint:i*.01}});
  }
  const fixture={name:'batched-sdk.jsonl',mimeType:'application/x-ndjson',buffer:Buffer.from(rows.map(JSON.stringify).join('\n')+'\n')};
  await page.locator('#file').setInputFiles(fixture);
  await page.waitForFunction(()=>!document.querySelector('#file').disabled);
  assert.equal(await page.locator('#positionLegend input').count(),5);
  assert.equal(await page.locator('#currentLegend input').count(),3);
  assert.equal(await page.locator('#temperatureLegend input').count(),1);
  assert.equal(await page.locator('#controlPeriodLegend input').count(),2);
  assert((await page.locator('#dataNotice').textContent()).includes('还原 100 条 SDK 下发记录'));
  assert(Math.abs(await page.evaluate(()=>state.downlink[1].t-state.downlink[0].t)-.002)<1e-9);
  await page.locator('#viewStart').fill('1.02');await page.locator('#viewEnd').fill('1.1');await page.locator('#applyTimeRange').click();
  const view=await page.evaluate(()=>{const m=chartModels.get($('positionCanvas'));return [m.xmin,m.xmax];});
  await page.locator('#positionSeries0').uncheck();
  assert.equal(await page.evaluate(()=>chartModels.get($('positionCanvas')).series.some(s=>s.name==='转换结果')),false);
  assert.deepEqual(await page.evaluate(()=>{const m=chartModels.get($('positionCanvas'));return [m.xmin,m.xmax];}),view);
  await page.locator('#currentSeries1').uncheck();
  await page.locator('#controlPeriodSeries1').uncheck();
  await page.locator('#motor').selectOption('rotary:0');
  assert.equal(await page.locator('#positionSeries0').isChecked(),false);
  await page.reload();
  assert.equal(await page.locator('#positionSeries0').isChecked(),false);
  assert.equal(await page.locator('#currentSeries1').isChecked(),false);
  assert.equal(await page.locator('#controlPeriodSeries1').isChecked(),false);
  await page.locator('#file').setInputFiles(fixture);
  await page.waitForFunction(()=>!document.querySelector('#file').disabled);
  for(const input of await page.locator('#positionLegend input').all())await input.uncheck();
  assert.equal(await page.evaluate(()=>chartModels.has($('positionCanvas'))),false);
  for(const input of await page.locator('#positionLegend input').all())await input.check();
  assert.equal(await page.evaluate(()=>chartModels.get($('positionCanvas')).series.length),5);

  if(process.env.VIEWER_TEST_LOG){
    console.log('Loading supplied recording');
    await page.locator('#file').setInputFiles(process.env.VIEWER_TEST_LOG);
    await page.waitForFunction(()=>!document.querySelector('#file').disabled,null,{timeout:120000});
    const stats=await page.evaluate(()=>{
      const rows=state.downlink,median=values=>values.sort((a,b)=>a-b)[Math.floor(values.length/2)];
      return {writes:rows.length,restored:rows.filter(r=>r.time_source==='sdk_write').length,
        repeatedTimes:rows.slice(1).filter((r,i)=>r.t===rows[i].t).length,
        medianIntervalMs:median(rows.slice(1).map((r,i)=>(r.t-rows[i].t)*1000)),
        matchedAngles:rows.every((r,i)=>Math.abs(r.t-angleStages.sdk[i].t)<1e-9)};
    });
    assert(stats.writes>0);assert.equal(stats.repeatedTimes,0);assert(stats.matchedAngles);
    console.log(JSON.stringify(stats));
    await page.locator('#viewStart').fill('11');await page.locator('#viewEnd').fill('14');await page.locator('#applyTimeRange').click();
    const output=new URL('../../../.work/',import.meta.url);await mkdir(output,{recursive:true});
    await page.locator('details[data-fold="position"]').screenshot({path:new URL('sdk-viewer-after.png',output).pathname});
  }
  assert.deepEqual(errors,[]);
  console.log('Motor series selection, persistence and SDK write time browser checks passed');
}finally{await browser.close();await new Promise(resolve=>server.close(resolve));}
