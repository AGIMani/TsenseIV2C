// Standalone replay report: no server, camera or physical serial access.
import {chromium} from '@playwright/test';
import {pathToFileURL} from 'node:url';
import {mkdir} from 'node:fs/promises';
import assert from 'node:assert/strict';
const report=process.argv[2],out=process.argv[3];
if(!report||!out)throw Error('Usage: node tests/direct-filter-viewer.mjs report.html output-dir');
await mkdir(out,{recursive:true});
const browser=await chromium.launch({executablePath:'C:/Program Files/Google/Chrome/Application/chrome.exe',headless:true});
try{
  const page=await browser.newPage({viewport:{width:1440,height:1050}}),errors=[];
  page.on('pageerror',e=>errors.push(e.message));
  await page.goto(pathToFileURL(report).href);
  await page.waitForFunction(()=>document.querySelector('#status').textContent.includes('已加载'));
  assert.match(await page.locator('#dataNotice').innerText(),/离线滤波回放/);
  for(const box of await page.locator('#jointChecks input').all())await box.uncheck();
  await page.locator('#jointChecks input[value="mid_PIP"]').check();
  const metrics=await page.evaluate(()=>{
    const i=state.control.reduce((best,r,i,a)=>i>0&&Math.abs(r.joints.left_hand_mid_PIP_joint-a[i-1].joints.left_hand_mid_PIP_joint)>best.delta?{i,delta:Math.abs(r.joints.left_hand_mid_PIP_joint-a[i-1].joints.left_hand_mid_PIP_joint)}:best,{i:1,delta:0}).i;
    const t=state.control[i].t;
    applyChartView($('jointCanvas'),{xmin:t-.15,xmax:t+.25});
    inspectFrame(t+.01);
    return {filtered:state.filtered.length,detail:state.filterDetail.length,raw:state.control.length,
      writes:state.downlink.filter(r=>r.sent).length,series:makeJointSeries().filter(s=>s.points.length).map(s=>s.name),
      frame:JSON.parse($('frameDetail').textContent).direct_filter};
  });
  assert(metrics.filtered>5000&&metrics.detail>100&&metrics.raw===332);
  assert.equal(metrics.writes,0);assert.equal(metrics.frame.sent,false);
  assert(metrics.series.some(n=>n.includes('直接滤波'))&&metrics.series.some(n=>n.includes('控制目标')));
  await page.screenshot({path:`${out}/replay-viewer.png`,fullPage:false});
  assert.deepEqual(errors,[]);
  console.log(JSON.stringify({...metrics,frame:undefined}));
}finally{await browser.close();}
