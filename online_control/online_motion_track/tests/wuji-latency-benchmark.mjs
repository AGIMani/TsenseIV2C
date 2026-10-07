// A/B benchmark: synthetic glove + actual HTTP app/solver; never connect or arm hardware.
import {chromium} from '@playwright/test';
import {spawn} from 'node:child_process';
import {writeFile,mkdir} from 'node:fs/promises';
import {fileURLToPath} from 'node:url';
const root=fileURLToPath(new URL('../../../',import.meta.url));
const fixture=spawn('E:/conda/envs/hand_dex/python.exe',['-u','-X','utf8','tests/run_wuji_browser_fixture.py'],
  {cwd:root,env:{...process.env,WUJI_TEST_HZ:'100'},stdio:['pipe','pipe','pipe']});
const url=await new Promise((resolve,reject)=>{
  let out='';const timer=setTimeout(()=>reject(Error('fixture timeout')),30000);
  fixture.stdout.on('data',c=>{out+=c;const match=out.match(/WUJI_TEST_URL=(http:\/\/\S+)/);if(match){clearTimeout(timer);resolve(match[1]);}});
  fixture.once('exit',c=>{clearTimeout(timer);reject(Error(`fixture exit ${c}`));});
});
fixture.stderr.on('data',()=>{});
const browser=await chromium.launch({headless:true,executablePath:'C:/Program Files/Google/Chrome/Application/chrome.exe'});
try{
  const page=await browser.newPage();await page.goto(url);
  const result=await page.evaluate(async()=>{
    const sleep=ms=>new Promise(r=>setTimeout(r,ms));
    const api=async(path,data)=>{const r=await fetch(path,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data)});const v=await r.json();if(!r.ok)throw Error(v.error);return v;};
    const stats=a=>{a.sort((a,b)=>a-b);return Object.fromEntries([['p50',.5],['p95',.95],['max',1]].map(([k,p])=>[k,a[Math.floor((a.length-1)*p)]]));};
    const results=[];
    for(const mode of ['legacy','combined']){
      const created=await api('/api/session',{model_id:'TsenseI_v3_2_withid',side:'left',input_source:'wuji_glove',recording:false,collision_mode:'mink'});
      const session=created.session;await api('/api/wuji/create',{session});
      let seq=0,last=-1,pending=null,worker=null,error=null,requests=0,frames=0;
      const ages=[],solves=[],rtts=[];
      const start=performance.now(),end=start+7000;
      const record=(update,age,began)=>{if(update?.tracking&&performance.now()-start>1000){frames++;ages.push(age+performance.now()-began);solves.push(update.solve_ms);rtts.push(performance.now()-began);}};
      const pump=()=>{
        if(worker)return;
        worker=(async()=>{while(pending){const {packet,snapshot,began}=pending;pending=null;requests++;
          const update=await api('/api/frame',packet);record(update,snapshot.age_ms,began);}})()
          .catch(e=>{error=e;}).finally(()=>{worker=null;});
      };
      try{
        while(performance.now()<end){
          if(error)throw error;
          const began=performance.now();const packet={session,sequence:seq++,timestamp_ms:began,capture_unix_ms:Date.now(),source:'wuji_glove'};
          requests++;
          if(mode==='legacy'){
            const snapshot=await api('/api/wuji/latest',{session});
            if(snapshot.hand&&snapshot.revision!==last){last=snapshot.revision;
              pending={packet:{...packet,wuji_revision:snapshot.revision},snapshot,began};pump();}
            await sleep(16);
          }else{
            const snapshot=await api('/api/wuji/frame',packet);
            record(snapshot.update,snapshot.update?.source_age_at_solve_ms??0,began);
            await sleep(Math.max(0,8-(performance.now()-began)));
          }
        }
        if(worker)await worker;
        results.push({mode,requests,frames,measured_seconds:6,frames_per_second:frames/6,
          source_to_response_upper_ms:stats(ages),solve_ms:stats(solves),request_ms:stats(rtts)});
      }finally{await api('/api/stop',{session});}
    }
    return results;
  });
  const report={source:'SYNTHETIC_100HZ_LOCALHOST',hardware:'never connected or armed',
    note:'Both modes use the same current solver and 120 Hz upstream worker. Legacy emulates independent 16 ms browser polling and latest-only frame queue.',result};
  await mkdir(root+'logs/wuji_validation',{recursive:true});
  await writeFile(root+'logs/wuji_validation/latency-ab-20260929.json',JSON.stringify(report,null,2));
  console.log(JSON.stringify(report));
}finally{await browser.close();fixture.stdin.end('quit\n');}
