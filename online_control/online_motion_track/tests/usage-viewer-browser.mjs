// Isolated browser storage / fixtures only; no camera, network service or motor.
import {chromium} from '@playwright/test';
import {createServer} from 'node:http';
import {readFile,mkdir} from 'node:fs/promises';
import assert from 'node:assert/strict';
const server=createServer(async(req,res)=>{
  const name=req.url.slice(1);
  try{
    if(name==='viewer'){res.setHeader('Content-Type','text/html');res.end(await readFile(new URL('../../../tools/hand_control_log_viewer.html',import.meta.url)));}
    else if(['worker-recorder.js','recording.worker.js','local-recorder.js'].includes(name)){
      res.setHeader('Content-Type','text/javascript');res.end(await readFile(new URL(`../static/${name}`,import.meta.url)));
    }else res.end('<!doctype html><title>Storage fixture</title>');
  }catch(error){res.writeHead(500).end(error.message);}
});
await new Promise(resolve=>server.listen(0,'127.0.0.1',resolve));
const browser=await chromium.launch({executablePath:process.env.CHROME_PATH,headless:true});
const page=await browser.newPage({viewport:{width:1500,height:1100}}),errors=[];
page.on('pageerror',e=>errors.push(e.message));
page.on('console',message=>console.log(message.text()));
const base=`http://127.0.0.1:${server.address().port}`;
try{
  await page.goto(base);
  const storage=await page.evaluate(async()=>{
    const {WorkerRecorder}=await import('/worker-recorder.js');
    const root=await(await navigator.storage.getDirectory()).getDirectoryHandle('test-hand-logs',{create:true}),results=[];
    const r=new WorkerRecorder();await r.setDirectory(root);
    for(const [model,folder] of [['TsenseI_v3_0','A30'],['TsenseI_v3_2','A32'],['TsenseI_v2','V2']]){
      await r.start({model_id:model,hand_side:'left',hand_name:'样机 01'});
      r.record('retarget',{frame_id:1,joints:{left_hand_index_MCP_joint:.2}});
      const file=await r.stop('test');
      if(file.saveError)throw Error(file.saveError);
      if(file.savedPaths?.length!==1)throw Error('missing directory output');
      const directory=await root.getDirectoryHandle(folder),saved=await(await directory.getFileHandle(file.name)).getFile();
      const rows=(await saved.text()).trim().split('\n').map(JSON.parse);
      if(rows[0].metadata.model_id!==model||rows.at(-1).type!=='session_end')throw Error('incomplete directory file');
      results.push({folder,bytes:saved.size});URL.revokeObjectURL(file.url);
    }
    const restored=new WorkerRecorder(),directory=await restored.getDirectory();
    if(!directory||directory.name!=='test-hand-logs'||directory.permission!=='prompt')throw Error('directory preference not restored');
    // Permission failure preserves export and IndexedDB instead of pretending to save.
    const {LocalRecorder}=await import('/local-recorder.js');
    const failed=new LocalRecorder(()=>{}, {dbName:'save-failure'});
    await failed.start({model_id:'TsenseI_v3_0'});failed.directory={queryPermission:async()=>'denied'};
    const recovery=await failed.stop('test');
    if(!recovery.saveError||!recovery.url||(await failed.list()).length!==1)throw Error('permission failure lost recovery');
    URL.revokeObjectURL(recovery.url);failed.db.close();
    r.worker.terminate();restored.worker.terminate();
    return results;
  });
  await page.goto(base+'/viewer');
  const header={type:'session',schema:'hand-control-session/v2',session_id:'browser-usage',started_at:'2026-09-24T01:00:00Z',
    metadata:{model_id:'TsenseI_v3_0',hand_side:'left',hand_name:'样机 01'}};
  const fixture=(index)=>({name:`fixture_part00${index}.jsonl`,mimeType:'application/x-ndjson',buffer:Buffer.from([
    {...header,recording_part:{index,count:2,session_id:header.session_id}},
    {type:'joint_angles',stage:'retarget',t:index===1?0:60,hand_side:'left',joints:{left_hand_index_MCP_joint:.1}},
    {type:'joint_angles',stage:'retarget',t:index===1?59:119,hand_side:'left',joints:{left_hand_index_MCP_joint:.2}},
    ...(index===2?[{type:'session_end',t:120,duration_s:120}]:[])].map(JSON.stringify).join('\n')+'\n')});
  await page.locator('#file').setInputFiles([fixture(2),fixture(1)]);
  await page.waitForFunction(()=>document.querySelector('#usageTotals').textContent.includes('0 时 2 分 0 秒'));
  assert.equal(await page.locator('#usageRows tr').count(),1);
  await page.locator('#usageRows input').fill('左手样机 A');await page.locator('#usageRows input').press('Tab');
  await page.locator('#file').setInputFiles(fixture(2));
  await page.waitForFunction(()=>!document.querySelector('#file').disabled);
  assert.equal(await page.locator('#usageRows tr').count(),1);
  assert.equal(await page.locator('#usageRows input').inputValue(),'左手样机 A');
  // Every primary panel is a native keyboard-operable details element.
  assert.equal(await page.locator('section.panel,section.toolbar,div.panel').count(),0);
  assert.equal(await page.locator('details[data-fold]').count(),13);
  await page.locator('details[data-fold="section-2"] summary').click();
  await page.waitForFunction(()=>!document.querySelector('[data-fold="section-2"]').open
    &&JSON.parse(localStorage.getItem('tsensei-viewer-folds')||'{}')['section-2']===false);
  await page.reload();
  assert.equal(await page.locator('#usageRows tr').count(),1);
  assert.equal(await page.locator('details[data-fold="section-2"]').getAttribute('open'),null);
  await page.locator('#usageModel').selectOption('A32');await page.locator('#usageSide').selectOption('right');
  await page.locator('#usageName').fill('右手样机 B');
  await page.locator('#usageStart').fill('2026-09-24T10:00');await page.locator('#usageEnd').fill('2026-09-24T10:30');
  await page.locator('#usageAdd').click();assert.equal(await page.locator('#usageRows tr').count(),1);
  assert((await page.locator('#usageTotals').textContent()).includes('0 时 30 分 0 秒'));
  await page.locator('#file').setInputFiles(fixture(2));await page.waitForFunction(()=>!document.querySelector('#file').disabled);
  const out=new URL('../../../logs/usage-viewer-20260924/',import.meta.url);await mkdir(out,{recursive:true});
  await page.screenshot({path:new URL('viewer.png',out).pathname.replace(/^\/([A-Za-z]:)/,'$1'),fullPage:true});
  assert.deepEqual(errors,[]);console.log(JSON.stringify({directoryWrites:storage,usageRows:1,folding:true,persistence:true,errors},null,2));
}finally{await browser.close();await new Promise(resolve=>server.close(resolve));}
