// Auto-import UX with an in-memory local API fixture; no camera or motor.
import {chromium} from '@playwright/test';
import {createServer} from 'node:http';
import {readFile} from 'node:fs/promises';
import assert from 'node:assert/strict';

const fixtures=Object.fromEntries([
  ['A30','daxian_v3_0','left',120],['A32','daxian_v3_2','right',180],['V2','daxian_v2','left',240],
].map(([model,modelId,side,duration],index)=>{
  const started_at=new Date(Date.parse('2026-09-24T00:00:00Z')+index*86400000).toISOString();
  const name=`hand_control_${model}.jsonl`,header={type:'session',schema:'hand-control-session/v2',started_at,
    metadata:{model_id:modelId,hand_side:side,hand_name:`${model} 样机`}};
  const text=[header,{type:'retarget',t:0,hand_side:side,joints:{}},
    {type:'retarget',t:duration,hand_side:side,joints:{}},{type:'session_end',t:duration,duration_s:duration}].map(JSON.stringify).join('\n')+'\n';
  return [model,{name,text,record:{id:`log:${started_at}:${model}:${side}`,model,side,name:`${model} 样机`,start:Date.parse(started_at),
    parts:[{index:1,count:null,start:Date.parse(started_at),end:Date.parse(started_at)+duration*1000,closed:true,filename:name}]}}];
}));
let parent='E:\\TsenseI_hand_logs';
const server=createServer(async(req,res)=>{
  const url=new URL(req.url,'http://localhost');
  if(url.pathname==='/data-viewer'){
    res.setHeader('Content-Type','text/html');res.end(await readFile(new URL('../../../tools/hand_control_log_viewer.html',import.meta.url)));
  }else if(url.pathname==='/api/viewer/logs'){
    const fixture=fixtures[url.searchParams.get('model')];
    if(!fixture){res.writeHead(400).end();return;}
    res.setHeader('Content-Type','application/json');res.end(JSON.stringify({parent,files:[{name:fixture.name,bytes:Buffer.byteLength(fixture.text)}],records:[fixture.record]}));
  }else if(url.pathname==='/api/viewer/log-file'){
    const fixture=fixtures[url.searchParams.get('model')];
    if(!fixture||url.searchParams.get('name')!==fixture.name){res.writeHead(404).end();return;}
    res.setHeader('Content-Type','application/x-ndjson');res.end(fixture.text);
  }else if(url.pathname==='/api/viewer/log-parent'&&req.method==='POST'){
    let body='';for await(const chunk of req)body+=chunk;
    parent=JSON.parse(body).parent;
    res.setHeader('Content-Type','application/json');res.end(JSON.stringify({parent,folders:['A30','A32','V2'].map(model=>`${parent}/${model}`)}));
  }else res.writeHead(404).end();
});
await new Promise(resolve=>server.listen(0,'127.0.0.1',resolve));
const browser=await chromium.launch({executablePath:process.env.CHROME_PATH||'C:/Program Files/Google/Chrome/Application/chrome.exe',headless:true});
const page=await browser.newPage(),errors=[];page.on('pageerror',e=>errors.push(e.message));
try{
  await page.goto(`http://127.0.0.1:${server.address().port}/data-viewer`);
  await page.waitForFunction(()=>document.querySelector('#usageStatus').textContent.includes('A30 文件夹：1 份'));
  assert.equal(await page.locator('#usageRows tr').count(),1);
  assert((await page.locator('#usageTotals').textContent()).includes('0 时 2 分 0 秒'));
  assert((await page.locator('#usageLogParent').textContent()).includes('TsenseI_hand_logs'));
  await page.locator('#usageRefreshFolder').click();
  assert.equal(await page.locator('#usageRows tr').count(),1);
  for(const model of ['A32','V2']){
    await page.locator('#usageModel').selectOption(model);
    await page.waitForFunction(value=>document.querySelector('#usageStatus').textContent.includes(`${value} 文件夹：1 份`),model);
    assert.equal(await page.locator('#usageRows tr').count(),1);
    assert((await page.locator('#usageTotals').textContent()).includes(model));
    assert((await page.locator('#usageLogFile').inputValue())==='');
  }
  await page.locator('#usageParentPath').fill('C:\\CustomHandLogs');
  await page.locator('#usageSaveParent').click();
  await page.waitForFunction(()=>document.querySelector('#usageStatus').textContent.includes('已确认 A30、A32、V2'));
  assert.equal(await page.locator('#usageParentPath').inputValue(),'C:\\CustomHandLogs');
  await page.locator('#usageModel').selectOption('A30');
  await page.waitForFunction(()=>document.querySelector('#usageStatus').textContent.includes('A30 文件夹：1 份'));
  await page.locator('#usageLogFile').selectOption(fixtures.A30.name);
  await page.locator('#usageOpenLog').click();
  await page.waitForFunction(()=>document.querySelector('#status').textContent.includes('4 条'));
  assert.equal(await page.locator('#usageRows tr').count(),1);
  assert.deepEqual(errors,[]);
  console.log('A30/A32/V2 auto-import, model switch and parent editor passed');
}finally{await browser.close();await new Promise(resolve=>server.close(resolve));}
