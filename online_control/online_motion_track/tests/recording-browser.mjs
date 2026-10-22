// Real Worker + IndexedDB coverage; isolated HTTP origin, no camera or hardware.
import {chromium} from '@playwright/test';
import {createServer} from 'node:http';
import {readFile} from 'node:fs/promises';
import assert from 'node:assert/strict';

const server=createServer(async(req,res)=>{
  if(req.url==='/'){res.setHeader('Content-Type','text/html');res.end('<!doctype html><title>Recorder test</title>');return;}
  const name=req.url.slice(1);
  if(!['worker-recorder.js','recording.worker.js','local-recorder.js'].includes(name)){res.writeHead(404).end();return;}
  try{res.setHeader('Content-Type','text/javascript');res.end(await readFile(new URL(`../static/${name}`,import.meta.url)));}
  catch{res.writeHead(500).end();}
});
await new Promise(resolve=>server.listen(0,'127.0.0.1',resolve));
const browser=await chromium.launch({executablePath:process.env.CHROME_PATH,headless:true});
const page=await browser.newPage(),errors=[];
page.on('pageerror',e=>errors.push(e.message));
try{
  await page.goto(`http://127.0.0.1:${server.address().port}/`);
  const result=await page.evaluate(async()=>{
    const {WorkerRecorder}=await import('/worker-recorder.js');
    const check=(value,message)=>{if(!value)throw Error(message);};
    const r=new WorkerRecorder(e=>{throw e;});
    await Promise.all([r.start({model_id:'capture-test'}),r.start({model_id:'capture-test'})]);
    const id=r.id;
    r.record('retarget',{frame_id:0,joints:{left_hand_index_swing_joint:.1}});
    // Export itself must drain the unflushed tail.
    const snapshot=await r.export(id);
    const snapshotText=await(await fetch(snapshot.url)).text();URL.revokeObjectURL(snapshot.url);
    check(snapshot.incomplete&&snapshotText.includes('"frame_id":0'),'active export omitted tail');
    // 530 MiB exercises the actual 512 MiB boundary in a real worker/IndexedDB.
    const padding='x'.repeat(256*1024),total=2120;
    for(let i=1;i<=total;i++){
      r.record('retarget',{frame_id:i,joints:{left_hand_index_swing_joint:i/10000}});
      r.record('telemetry',{frame_id:i,padding});
      if(i%16===0)await r.flush();
    }
    check(r.active&&!r.error,'recording stopped at old capacity limit');
    r.record('retarget',{frame_id:total+1,joints:{left_hand_index_swing_joint:.3}});
    await r.stop('test_complete',{exportFile:false});
    const first=await r.export(id),ids=[],seqs=[],sizes=[];let end;
    check(first.partCount>1,'large recording was not split');
    for(let i=0;i<first.partCount;i++){
      const file=i===0?first:await r.export(id,i);
      const blob=await(await fetch(file.url)).blob();sizes.push(blob.size);
      const reader=blob.stream().pipeThrough(new TextDecoderStream()).getReader();let carry='',firstRow=true;
      for(;;){
        const {value,done}=await reader.read();carry+=value||'';
        const lines=carry.split('\n');carry=lines.pop();
        for(const line of lines){
          const row=JSON.parse(line);
          if(firstRow){check(row.type==='session'&&row.recording_part.index===i+1,'missing standalone part metadata');firstRow=false;continue;}
          seqs.push(row.seq);
          if(row.type==='joint_angles'&&row.stage==='retarget')ids.push(row.frame_id);
          if(row.type==='session_end'){check(!end,'duplicate end');end=row;}
        }
        if(done)break;
      }
      URL.revokeObjectURL(file.url);
    }
    check(ids.length===total+2&&ids.every((id,i)=>id===i),'lost or duplicated retarget across parts');
    check(seqs.every((n,i)=>n===i+2),'non-contiguous stored event sequences');
    check(end.dropped_events===0&&end.event_counts.joint_angles===total+2,'incorrect capture summary');
    check(sizes.every(n=>n<=512*1024*1024),'export exceeds 512 MiB');
    // A fresh worker can discover and export every persisted part.
    const recovery=new WorkerRecorder(),history=await recovery.list();
    check(history.find(s=>s.id===id)?.parts.length===first.partCount,'missing persisted parts');
    const tail=await recovery.export(id,first.partCount-1);
    check((await(await fetch(tail.url)).text()).includes('test_complete'),'recovered tail missing');
    URL.revokeObjectURL(tail.url);await recovery.remove(id);recovery.worker.terminate();r.worker.terminate();

    let seal,error;
    const failed=new WorkerRecorder(e=>{error=e;seal=failed.stop('recording_error',{exportFile:false});});
    await failed.start({});failed.record('retarget',{frame_id:0,joints:{a:1}});
    failed.record('uncloneable',{callback(){}});await seal;
    const bad=await failed.export(failed.id),badRows=(await(await fetch(bad.url)).text()).trim().split('\n').map(JSON.parse);
    check(error&&badRows.at(-1).dropped_events===1,'clone failure reported zero loss');
    check(badRows.some(r=>r.type==='joint_angles'&&r.stage==='retarget'),'clone failure lost earlier data');
    URL.revokeObjectURL(bad.url);await failed.remove(failed.id);failed.worker.terminate();

    const {LocalRecorder}=await import('/local-recorder.js');
    let quotaSeal;
    const quota=new LocalRecorder(()=>{quotaSeal=quota.stop('recording_error',{exportFile:false});},{dbName:'quota-test',maxBytes:Infinity,partBytes:8192});
    await quota.start({});await quota.flush();
    const transaction=quota.db.transaction.bind(quota.db);let fail=true;
    quota.db.transaction=(stores,...args)=>{
      if(fail&&Array.isArray(stores)&&stores.includes('chunks')){fail=false;throw new DOMException('TEST quota','QuotaExceededError');}
      return transaction(stores,...args);
    };
    quota.record('retarget',{frame_id:19,joints:{a:1}});await quota.flush();await quotaSeal;
    const qids=[];let qend;
    for(let i=0;i<quota.info.parts.length;i++){
      const f=await quota.export(quota.id,i),rows=(await(await fetch(f.url)).text()).trim().split('\n').map(JSON.parse);
      for(const row of rows){if(row.type==='retarget')qids.push(row.frame_id);if(row.type==='session_end')qend=row;}
      URL.revokeObjectURL(f.url);
    }
    check(qids.length===1&&qids[0]===19&&qend.recording_error==='TEST quota','failed chunk recovery lost/duplicated data');
    await quota.remove(quota.id);quota.db.close();
    return {retarget:ids.length,parts:first.partCount,totalBytes:sizes.reduce((a,b)=>a+b,0),maxPartBytes:Math.max(...sizes),cloneFailureDropped:badRows.at(-1).dropped_events,quotaRecovery:true};
  });
  assert.deepEqual(errors,[]);console.log(JSON.stringify(result,null,2));
}finally{await browser.close();await new Promise(resolve=>server.close(resolve));}
